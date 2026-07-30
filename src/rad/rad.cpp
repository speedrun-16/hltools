#include "rad.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "../common/error.h"
#include "../common/filesystem.h"
#include "../common/log.h"
#include "../common/progress.h"
#include "../common/string_util.h"
#include "../common/threads.h"
#include "compress.h"
#include "internal.h"

// the stage driver: reads the compiled bsp, builds patches and direct lights,
// runs the direct pass, bounces light between patches through the transfer
// lists, blends it back into the samples and lays out the lighting lump in the
// reference radworld order

namespace rad
{
    namespace
    {
        // the reference parseentity applies these conversions while reading
        // the entity lump; the csg write path
        // reverses them, so the lump round trips byte identically
        void apply_parse_conversions(rad_state &state)
        {
            for (size_t i = 0; i < state.entities.size(); i++)
            {
                format::entity *mapent = &state.entities[i];

                if (!strncmp(mapent->value("classname"), "light", 5) && *mapent->value("_tex"))
                {
                    mapent->set("convertto", mapent->value("classname"));
                    mapent->set("classname", "light_surface");
                }
                if (!strcmp(mapent->value("convertfrom"), "light_shadow")
                    || !strcmp(mapent->value("convertfrom"), "light_bounce"))
                {
                    mapent->set("convertto", mapent->value("classname"));
                    mapent->set("classname", mapent->value("convertfrom"));
                    mapent->set("convertfrom", "");
                }
                if (!strcmp(mapent->value("classname"), "light_environment") &&
                    !strcmp(mapent->value("convertfrom"), "info_sunlight"))
                {
                    state.entities.erase(state.entities.begin() + (long long)i);
                    i--;
                    continue;
                }
                if (!strcmp(mapent->value("classname"), "light_environment") &&
                    int_for_key(*mapent, "_fake"))
                {
                    mapent->set("classname", "info_sunlight");
                }
            }
        }

        // ===== bounce light =====

        // per patch light staging used only during the bounce passes
        struct bounce_arrays
        {
            vec3v (*emitlight)[maxlightmaps] = nullptr;
            vec3v (*addlight)[maxlightmaps] = nullptr;
            unsigned char (*newstyles)[maxlightmaps] = nullptr;
        };

        inline bool is_point_finite(const vec3v &v)
        {
            return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
        }

        void collect_light(rad_state &state, bounce_arrays &ba)
        {
            unsigned j;
            unsigned i;
            patch *pt;

            for (i = 0, pt = state.patches.data(); i < state.num_patches; i++, pt++)
            {
                vec3v newtotallight[maxlightmaps];
                for (j = 0; j < maxlightmaps && ba.newstyles[i][j] != 255; j++)
                {
                    math::clear(newtotallight[j]);
                    int k;
                    for (k = 0; k < maxlightmaps && pt->totalstyle[k] != 255; k++)
                    {
                        if (pt->totalstyle[k] == ba.newstyles[i][j])
                        {
                            math::copy(pt->totallight[k], newtotallight[j]);
                            break;
                        }
                    }
                }
                for (j = 0; j < maxlightmaps; j++)
                {
                    if (ba.newstyles[i][j] != 255)
                    {
                        pt->totalstyle[j] = ba.newstyles[i][j];
                        math::copy(newtotallight[j], pt->totallight[j]);
                        math::copy(ba.addlight[i][j], ba.emitlight[i][j]);
                    }
                    else
                    {
                        pt->totalstyle[j] = 255;
                    }
                }
            }
        }

        // gathers light from the transfer list of one patch runs multithreaded
        void gather_light_item(rad_state &state, bounce_arrays &ba, unsigned int &fastfind_index, int j)
        {
            patch *pt;
            unsigned k, m;
            unsigned i_index;
            transfer_data *t_data;
            transfer_index *t_index;
            float f;
            vec3v adds[allstyles];
            int style;

            memset(adds, 0, allstyles * sizeof(vec3v));

            pt = &state.patches[(size_t)j];

            t_data = pt->t_data;
            t_index = pt->t_index;
            i_index = pt->i_index;

            for (m = 0; m < maxlightmaps && pt->totalstyle[m] != 255; m++)
            {
                math::add(adds[pt->totalstyle[m]], pt->totallight[m], adds[pt->totalstyle[m]]);
            }

            for (k = 0; k < i_index; k++, t_index++)
            {
                unsigned l;
                unsigned size = (t_index->size + 1);
                unsigned patchnum = t_index->index;

                for (l = 0; l < size; l++, t_data += float_format_size[(int)state.options.transfer_compress_type], patchnum++)
                {
                    vec3v v;
                    patch *emitpatch = &state.patches[patchnum];
                    unsigned emitstyle;
                    int opaquestyle = -1;
                    get_style(state, (unsigned)j, patchnum, opaquestyle, fastfind_index);
                    float_decompress(state.options.transfer_compress_type, t_data, &f);

                    // for each style on the emitting patch
                    for (emitstyle = 0; emitstyle < maxlightmaps && emitpatch->directstyle[emitstyle] != 255; emitstyle++)
                    {
                        math::scale(emitpatch->directlight[emitstyle], f, v);
                        math::multiply(v, emitpatch->bouncereflectivity, v);
                        if (is_point_finite(v))
                        {
                            int addstyle = emitpatch->directstyle[emitstyle];
                            if (emitpatch->bouncestyle != -1)
                            {
                                if (addstyle == 0 || addstyle == emitpatch->bouncestyle)
                                    addstyle = emitpatch->bouncestyle;
                                else
                                    continue;
                            }
                            if (opaquestyle != -1)
                            {
                                if (addstyle == 0 || addstyle == opaquestyle)
                                    addstyle = opaquestyle;
                                else
                                    continue;
                            }
                            math::add(adds[addstyle], v, adds[addstyle]);
                        }
                    }
                    for (emitstyle = 0; emitstyle < maxlightmaps && emitpatch->totalstyle[emitstyle] != 255; emitstyle++)
                    {
                        math::scale(ba.emitlight[patchnum][emitstyle], f, v);
                        math::multiply(v, emitpatch->bouncereflectivity, v);
                        if (is_point_finite(v))
                        {
                            int addstyle = emitpatch->totalstyle[emitstyle];
                            if (emitpatch->bouncestyle != -1)
                            {
                                if (addstyle == 0 || addstyle == emitpatch->bouncestyle)
                                    addstyle = emitpatch->bouncestyle;
                                else
                                    continue;
                            }
                            if (opaquestyle != -1)
                            {
                                if (addstyle == 0 || addstyle == opaquestyle)
                                    addstyle = opaquestyle;
                                else
                                    continue;
                            }
                            math::add(adds[addstyle], v, adds[addstyle]);
                        }
                    }
                }
            }

            vec_t maxlights[allstyles];
            for (style = 0; style < allstyles; style++)
            {
                maxlights[style] = vector_maximum(adds[style]);
            }
            for (m = 0; m < maxlightmaps; m++)
            {
                unsigned char beststyle = 255;
                if (m == 0)
                {
                    beststyle = 0;
                }
                else
                {
                    vec_t bestmaxlight = 0;
                    for (style = 1; style < allstyles; style++)
                    {
                        if (maxlights[style] > bestmaxlight + math::normal_epsilon)
                        {
                            bestmaxlight = maxlights[style];
                            beststyle = (unsigned char)style;
                        }
                    }
                }
                if (beststyle != 255)
                {
                    maxlights[beststyle] = 0;
                    ba.newstyles[j][m] = beststyle;
                    math::copy(adds[beststyle], ba.addlight[j][m]);
                }
                else
                {
                    ba.newstyles[j][m] = 255;
                }
            }
            for (style = 1; style < allstyles; style++)
            {
                if (maxlights[style] > state.maxdiscardedlight + math::normal_epsilon)
                {
                    std::lock_guard<std::mutex> guard(state.lock);
                    if (maxlights[style] > state.maxdiscardedlight + math::normal_epsilon)
                    {
                        state.maxdiscardedlight = maxlights[style];
                        math::copy(pt->origin, state.maxdiscardedpos);
                    }
                }
            }
        }

        // rgb transfer version
        void gather_rgb_light_item(rad_state &state, bounce_arrays &ba, unsigned int &fastfind_index, int j)
        {
            patch *pt;
            unsigned k, m;
            unsigned i_index;
            rgb_transfer_data *t_rgb_data;
            transfer_index *t_index;
            float f[3];
            vec3v adds[allstyles];
            int style;

            memset(adds, 0, allstyles * sizeof(vec3v));

            pt = &state.patches[(size_t)j];

            t_rgb_data = pt->t_rgb_data;
            t_index = pt->t_index;
            i_index = pt->i_index;

            for (m = 0; m < maxlightmaps && pt->totalstyle[m] != 255; m++)
            {
                math::add(adds[pt->totalstyle[m]], pt->totallight[m], adds[pt->totalstyle[m]]);
            }

            for (k = 0; k < i_index; k++, t_index++)
            {
                unsigned l;
                unsigned size = (t_index->size + 1);
                unsigned patchnum = t_index->index;
                for (l = 0; l < size; l++, t_rgb_data += vector_format_size[(int)state.options.rgbtransfer_compress_type], patchnum++)
                {
                    vec3v v;
                    patch *emitpatch = &state.patches[patchnum];
                    unsigned emitstyle;
                    int opaquestyle = -1;
                    get_style(state, (unsigned)j, patchnum, opaquestyle, fastfind_index);
                    vector_decompress(state.options.rgbtransfer_compress_type, t_rgb_data, &f[0], &f[1], &f[2]);

                    // for each style on the emitting patch
                    for (emitstyle = 0; emitstyle < maxlightmaps && emitpatch->directstyle[emitstyle] != 255; emitstyle++)
                    {
                        vec3v fv{f[0], f[1], f[2]};
                        math::multiply(emitpatch->directlight[emitstyle], fv, v);
                        math::multiply(v, emitpatch->bouncereflectivity, v);
                        if (is_point_finite(v))
                        {
                            int addstyle = emitpatch->directstyle[emitstyle];
                            if (emitpatch->bouncestyle != -1)
                            {
                                if (addstyle == 0 || addstyle == emitpatch->bouncestyle)
                                    addstyle = emitpatch->bouncestyle;
                                else
                                    continue;
                            }
                            if (opaquestyle != -1)
                            {
                                if (addstyle == 0 || addstyle == opaquestyle)
                                    addstyle = opaquestyle;
                                else
                                    continue;
                            }
                            math::add(adds[addstyle], v, adds[addstyle]);
                        }
                    }
                    for (emitstyle = 0; emitstyle < maxlightmaps && emitpatch->totalstyle[emitstyle] != 255; emitstyle++)
                    {
                        vec3v fv{f[0], f[1], f[2]};
                        math::multiply(ba.emitlight[patchnum][emitstyle], fv, v);
                        math::multiply(v, emitpatch->bouncereflectivity, v);
                        if (is_point_finite(v))
                        {
                            int addstyle = emitpatch->totalstyle[emitstyle];
                            if (emitpatch->bouncestyle != -1)
                            {
                                if (addstyle == 0 || addstyle == emitpatch->bouncestyle)
                                    addstyle = emitpatch->bouncestyle;
                                else
                                    continue;
                            }
                            if (opaquestyle != -1)
                            {
                                if (addstyle == 0 || addstyle == opaquestyle)
                                    addstyle = opaquestyle;
                                else
                                    continue;
                            }
                            math::add(adds[addstyle], v, adds[addstyle]);
                        }
                    }
                }
            }

            vec_t maxlights[allstyles];
            for (style = 0; style < allstyles; style++)
            {
                maxlights[style] = vector_maximum(adds[style]);
            }
            for (m = 0; m < maxlightmaps; m++)
            {
                unsigned char beststyle = 255;
                if (m == 0)
                {
                    beststyle = 0;
                }
                else
                {
                    vec_t bestmaxlight = 0;
                    for (style = 1; style < allstyles; style++)
                    {
                        if (maxlights[style] > bestmaxlight + math::normal_epsilon)
                        {
                            bestmaxlight = maxlights[style];
                            beststyle = (unsigned char)style;
                        }
                    }
                }
                if (beststyle != 255)
                {
                    maxlights[beststyle] = 0;
                    ba.newstyles[j][m] = beststyle;
                    math::copy(adds[beststyle], ba.addlight[j][m]);
                }
                else
                {
                    ba.newstyles[j][m] = 255;
                }
            }
            for (style = 1; style < allstyles; style++)
            {
                if (maxlights[style] > state.maxdiscardedlight + math::normal_epsilon)
                {
                    std::lock_guard<std::mutex> guard(state.lock);
                    if (maxlights[style] > state.maxdiscardedlight + math::normal_epsilon)
                    {
                        state.maxdiscardedlight = maxlights[style];
                        math::copy(pt->origin, state.maxdiscardedpos);
                    }
                }
            }
        }

        // -dump: writes every patch with its accumulated light after a bounce
        void write_world(rad_state &state, const char *name)
        {
            unsigned i;
            unsigned j;
            FILE *out;
            patch *pt;
            const math::winding *w;

            out = fopen(name, "w");

            if (!out)
                err::fatal("Couldn't open %s", name);

            for (j = 0, pt = state.patches.data(); j < state.num_patches; j++, pt++)
            {
                w = pt->winding;
                logging::info("%i\n", w->size());
                for (i = 0; i < (unsigned)w->size(); i++)
                {
                    logging::info("%5.2f %5.2f %5.2f %5.3f %5.3f %5.3f\n",
                                  (*w)[(int)i][0], (*w)[(int)i][1], (*w)[(int)i][2],
                                  pt->totallight[0][0] / 256, pt->totallight[0][1] / 256, pt->totallight[0][2] / 256);
                }
                logging::info("\n");
            }

            fclose(out);
        }

        void bounce_light(rad_state &state, bounce_arrays &ba)
        {
            unsigned i;
            char name[64];
            unsigned j;

            for (i = 0; i < state.num_patches; i++)
            {
                patch *pt = &state.patches[i];
                for (j = 0; j < maxlightmaps && pt->totalstyle[j] != 255; j++)
                {
                    math::copy(pt->totallight[j], ba.emitlight[i][j]);
                }
            }

            auto bounce_start = std::chrono::steady_clock::now();
            for (i = 0; i < state.options.numbounce; i++)
            {
                if (state.options.rgb_transfers)
                {
                    threads::run((int)state.num_patches, [&](int idx) {
                        thread_local unsigned int fastfind_index = 0;
                        gather_rgb_light_item(state, ba, fastfind_index, idx);
                    });
                }
                else
                {
                    threads::run((int)state.num_patches, [&](int idx) {
                        thread_local unsigned int fastfind_index = 0;
                        gather_light_item(state, ba, fastfind_index, idx);
                    });
                }
                collect_light(state, ba);

                if (state.options.dumppatches)
                {
                    sprintf(name, "bounce%u.txt", i);
                    write_world(state, name);
                }
                logging::console("\r  %-25s (bounce %u/%u)          ", "bouncing light", i + 1, state.options.numbounce);
            }
            if (state.options.numbounce > 0)
            {
                double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - bounce_start).count();
                char detail[32];
                std::snprintf(detail, sizeof(detail), "(%u passes)", state.options.numbounce);
                progress::summary("bouncing light", detail, secs);
            }
            for (i = 0; i < state.num_patches; i++)
            {
                patch *pt = &state.patches[i];
                for (j = 0; j < maxlightmaps && pt->totalstyle[j] != 255; j++)
                {
                    math::copy(ba.emitlight[i][j], pt->totallight[j]);
                }
            }
        }

        void check_max_patches(rad_state &state)
        {
            switch (state.options.method)
            {
            case vis_method::vismatrix:
                err::require((int)state.num_patches < max_vismatrix_patches, "exceeded max patches for the vismatrix method");
                break;
            case vis_method::sparse_vismatrix:
                err::require((int)state.num_patches < max_sparse_vismatrix_patches, "exceeded max patches for the sparse vismatrix method");
                break;
            case vis_method::no_vismatrix:
                err::require((int)state.num_patches < max_patches, "exceeded max patches");
                break;
            }
        }

        void make_scales_stub(rad_state &state)
        {
            switch (state.options.method)
            {
            case vis_method::vismatrix:
                make_scales_vismatrix(state);
                break;
            case vis_method::sparse_vismatrix:
                make_scales_sparse_vismatrix(state);
                break;
            case vis_method::no_vismatrix:
                make_scales_no_vismatrix(state);
                break;
            }
        }

        void free_transfers(rad_state &state)
        {
            unsigned x;
            patch *pt = state.patches.data();

            for (x = 0; x < state.num_patches; x++, pt++)
            {
                if (pt->t_data)
                {
                    free(pt->t_data);
                    pt->t_data = nullptr;
                }
                if (pt->t_rgb_data)
                {
                    free(pt->t_rgb_data);
                    pt->t_rgb_data = nullptr;
                }
                if (pt->t_index)
                {
                    free(pt->t_index);
                    pt->t_index = nullptr;
                }
            }
        }

        // expands the lightdata array a little so the engine always reads
        // within its valid range
        void extend_lightmap_buffer(rad_state &state)
        {
            format::map_data &map = *state.map;
            int maxsize;
            int i;
            int j;
            int ofs;
            const format::dface_t *f;

            maxsize = 0;
            for (i = 0; i < (int)map.faces.size(); i++)
            {
                f = &map.faces[(size_t)i];
                if (f->lightofs >= 0)
                {
                    ofs = f->lightofs;
                    for (j = 0; j < maxlightmaps && f->styles[j] != 255; j++)
                    {
                        ofs += (limits::max_surface_extent + 1) * (limits::max_surface_extent + 1) * 3;
                    }
                    if (ofs > maxsize)
                    {
                        maxsize = ofs;
                    }
                }
            }
            if (maxsize >= (int)map.lighting.size())
            {
                err::require(maxsize <= state.max_map_lightdata, "extend_lightmap_buffer: exceeded MAX_MAP_LIGHTING");
                map.lighting.resize((size_t)maxsize, 0);
            }
        }

        // debug pts writers for -drawpatch and -drawedge
        void write_patch_points(rad_state &state)
        {
            std::string name = state.base_path + "_patch.pts";
            logging::info("Writing '%s' ...\n", name.c_str());
            FILE *f = fopen(name.c_str(), "w");
            if (f)
            {
                const int pos_count = 15;
                const vec3v pos[pos_count] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {-1, 0, 0}, {0, -1, 0},
                                              {1, 0, 0}, {0, 0, 1}, {-1, 0, 0}, {0, 0, -1}, {0, -1, 0},
                                              {0, 0, 1}, {0, 1, 0}, {0, 0, -1}, {1, 0, 0}, {0, 0, 0}};
                unsigned j;
                int k;
                patch *pt;
                vec3v v;
                for (j = 0, pt = state.patches.data(); j < state.num_patches; j++, pt++)
                {
                    if (pt->flags == patch_flag_outside)
                        continue;
                    math::copy(pt->origin, v);
                    for (k = 0; k < pos_count; ++k)
                        fprintf(f, "%g %g %g\n", v[0] + pos[k][0], v[1] + pos[k][1], v[2] + pos[k][2]);
                }
                fclose(f);
                logging::info("OK.\n");
            }
            else
                logging::info("Error.\n");
        }

        void write_edge_points(rad_state &state)
        {
            format::map_data &map = *state.map;
            std::string name = state.base_path + "_edge.pts";
            logging::info("Writing '%s' ...\n", name.c_str());
            FILE *f = fopen(name.c_str(), "w");
            if (f)
            {
                const int pos_count = 15;
                const vec3v pos[pos_count] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {-1, 0, 0}, {0, -1, 0},
                                              {1, 0, 0}, {0, 0, 1}, {-1, 0, 0}, {0, 0, -1}, {0, -1, 0},
                                              {0, 0, 1}, {0, 1, 0}, {0, 0, -1}, {1, 0, 0}, {0, 0, 0}};
                int k;
                vec3v v;
                for (size_t j = 0; j < state.edgeshares.size(); j++)
                {
                    const edgeshare *es = &state.edgeshares[j];
                    if (es->smooth)
                    {
                        int v0 = map.edges[j].v[0], v1 = map.edges[j].v[1];
                        for (int c = 0; c < 3; c++)
                            v[c] = map.vertexes[(size_t)v0].point[c] + map.vertexes[(size_t)v1].point[c];
                        math::scale(v, 0.5, v);
                        math::add(v, es->interface_normal, v);
                        math::add(v, state.face_offset[(size_t)(es->faces[0] - map.faces.data())], v);
                        for (k = 0; k < pos_count; ++k)
                            fprintf(f, "%g %g %g\n", v[0] + pos[k][0], v[1] + pos[k][1], v[2] + pos[k][2]);
                    }
                }
                fclose(f);
                logging::info("OK.\n");
            }
            else
                logging::info("Error.\n");
        }

        void rad_world(rad_state &state)
        {
            format::map_data &map = *state.map;

            load_planes(state);
            make_tnodes(state);
            create_opaque_nodes(state);
            load_opaque_entities(state);

            // turn each face into a single patch
            make_patches(state);
            if (state.options.drawpatch)
            {
                write_patch_points(state);
            }
            // check for exceeding max patches here, to prevent a lot of work
            // from occurring before an error
            check_max_patches(state);
            sort_patches(state); // makes the runs in the transfer compression really good
            pair_edges(state);
            if (state.options.drawedge)
            {
                write_edge_points(state);
            }

            build_diffuse_normals(state);
            // create directlights out of patches and lights
            create_direct_lights(state);
            load_studio_models(state);
            logging::file("\n");

            if (state.options.direct_state_hook)
            {
                // hand the prepared direct lighting state to the comparison test
                // state and skip the lighting passes entirely
                state.options.direct_state_hook(state, state.options.direct_state_hook_ctx);
                return;
            }

            // generate a position map for each face
            threads::run_phase("lighting", "placing light samples", (int)map.faces.size(),
                               [&](int i) { find_face_positions(state, i); });

            // build initial facelights; with -gpu the gather itself runs on
            // the gpu (collect pass + kernel + near pairs) and the phase
            // below consumes the stored results
            bool gpu_lighting = false;
            if (state.options.gpu)
                gpu_lighting = gpu_gather_run(state);
            if (state.options.dumpgather)
                state.gather_dump.resize(map.faces.size());
            threads::run_phase("lighting", "building direct lighting", (int)map.faces.size(),
                               [&](int i)
                               {
                                   gpu_gather_begin_face(i);
                                   build_facelights(state, i);
                               });
            if (gpu_lighting)
                gpu_gather_finish(state);

            if (state.options.dumpgather)
            {
                std::vector<byte> out;
                const char magic[8] = {'G', 'A', 'T', 'H', 'E', 'R', '1', 0};
                out.insert(out.end(), magic, magic + 8);
                unsigned int numfaces = (unsigned int)state.gather_dump.size();
                out.insert(out.end(), (const byte *)&numfaces, (const byte *)&numfaces + 4);
                for (const std::vector<byte> &blob : state.gather_dump)
                {
                    unsigned int len = (unsigned int)blob.size();
                    out.insert(out.end(), (const byte *)&len, (const byte *)&len + 4);
                    out.insert(out.end(), blob.begin(), blob.end());
                }
                std::string path = state.base_path + ".gather";
                if (fs::write_all(path, out.data(), out.size()))
                    logging::info("direct gather dump: %s (%zu bytes)\n", path.c_str(), out.size());
                else
                    logging::warn("could not write '%s'", path.c_str());
                state.gather_dump.clear();
                state.gather_dump.shrink_to_fit();
            }

            free_position_maps(state);

            // free up the direct lights now that we have facelights
            delete_direct_lights(state);

            if (state.options.numbounce > 0)
            {
                // build transfer lists; the "building patch visibility" and
                // "computing light transfers" bars are drawn from inside
                make_scales_stub(state);

                // these arrays are only used in collect_light, gather_light and bounce_light
                bounce_arrays ba;
                ba.emitlight = (vec3v (*)[maxlightmaps])calloc(state.num_patches + 1, sizeof(vec3v[maxlightmaps]));
                ba.addlight = (vec3v (*)[maxlightmaps])calloc(state.num_patches + 1, sizeof(vec3v[maxlightmaps]));
                ba.newstyles = (unsigned char (*)[maxlightmaps])calloc(state.num_patches + 1, sizeof(unsigned char[maxlightmaps]));
                err::require(ba.emitlight != nullptr && ba.addlight != nullptr && ba.newstyles != nullptr,
                             "rad_world: out of memory");
                // spread light around
                bounce_light(state, ba);

                free(ba.emitlight);
                free(ba.addlight);
                free(ba.newstyles);
            }

            free_transfers(state);
            free_style_arrays(state);

            threads::run_phase("lighting", "triangulating patches", (int)map.faces.size(),
                               [&](int i) { create_triangulations(state, i); });

            // blend bounced light into direct light and save
            precomp_lightmap_offsets(state);

            scale_direct_lights(state);

            {
                create_facelight_dependency_list(state);

                threads::run_phase("lighting", "applying bounced light", (int)map.faces.size(),
                                   [&](int i) { add_patch_lights(state, i); });

                free_facelight_dependency_list(state);
            }

            free_triangulations(state);

            threads::run_phase("lighting", "finalizing lightmaps", (int)map.faces.size(),
                               [&](int i) { final_light_face(state, i); });
            mdl_light_hack(state);
            reduce_lightmap(state);
            if (map.lighting.empty())
            {
                map.lighting.assign(1, 0);
            }
            extend_lightmap_buffer(state); // ensure the engine reads within the valid range
        }
    }

    bool run_rad(format::map_data &map, const std::string &base_path, const rad_options &options,
                 int *alloc_block_pages,
                 std::vector<format::texlight> *used_texlights)
    {
        if (alloc_block_pages)
        {
            *alloc_block_pages = -1;
        }
        rad_state state;
        state.map = &map;
        state.options = options;
        state.base_path = base_path;
        state.max_map_lightdata = options.max_map_lightdata;

        compress_compatability_test();

        state.entities = format::parse_entities(map.entities);
        apply_parse_conversions(state);

        if (state.options.fastmode)
        {
            state.options.numbounce = 0;
            state.options.softsky = false;
        }

        delete_embedded_lightmaps(state);
        load_textures(state);

        for (size_t i = 0; i < state.options.rad_files.size(); i++)
        {
            read_light_file(state, state.options.rad_files[i].c_str());
        }
        read_info_tex_and_minlights(state);

        if (used_texlights)
        {
            used_texlights->clear();
            for (const format::dface_t &face : map.faces)
            {
                const char *texture = texture_by_number(state, face.texinfo);
                if (!texture[0])
                    continue;
                bool already_added = false;
                for (const format::texlight &existing : *used_texlights)
                {
                    if (str::iequals(existing.name.c_str(), texture))
                    {
                        already_added = true;
                        break;
                    }
                }
                if (already_added)
                    continue;
                for (const rad_texlight &candidate : state.texlights)
                {
                    if (!str::iequals(candidate.name.c_str(), texture))
                        continue;
                    format::texlight light;
                    light.name = texture;
                    light.value[0] = (float)candidate.value[0];
                    light.value[1] = (float)candidate.value[1];
                    light.value[2] = (float)candidate.value[2];
                    light.source = candidate.source;
                    used_texlights->push_back(std::move(light));
                    break;
                }
            }
        }

        unsigned generic_texlights = 0;
        for (const rad_texlight &texlight : state.texlights)
            if (texlight.source == state.options.generic_rad_file)
                generic_texlights++;
        if (generic_texlights)
        {
            std::string preferred = base_path + ".rad";
            char warning[512];
            std::snprintf(warning, sizeof(warning),
                          "using %u texlight definitions from generic '%s'. lights.rad remains supported, "
                          "but support may be removed in a future release. Prefer '%s' next to the .map "
                          "or an info_texlights entity inside the map.",
                          generic_texlights, state.options.generic_rad_file.c_str(),
                          preferred.c_str());
            logging::add_warning_summary(warning);
        }

        // cosine of the smoothing angles (in radians)
        state.smoothing_threshold = (float)cos(state.options.smoothing_value * (math::pi / 180.0));

        read_custom_chop_value(state);
        read_custom_smooth_value(state);
        read_translucent_textures(state);
        read_lighting_cone(state);
        state.smoothing_threshold_2 = state.options.smoothing_value_2 < 0
            ? state.smoothing_threshold
            : (float)cos(state.options.smoothing_value_2 * (math::pi / 180.0));
        {
            int style;
            for (style = 0; style < allstyles; ++style)
            {
                state.corings[style] = style ? state.options.coring : 0;
            }
        }
        if (state.options.direct_scale != 1.0)
        {
            logging::warn("dscale value should be 1.0 for final compile.\nIf you need to adjust the bounced light, use the '-texreflectscale' and '-texreflectgamma' options instead.");
        }
        if (state.options.colour_lightscale[0] != 2.0 || state.options.colour_lightscale[1] != 2.0 || state.options.colour_lightscale[2] != 2.0)
        {
            logging::warn("light scale value should be 2.0 for final compile.\nValues other than 2.0 will result in incorrect interpretation of light_environment's brightness when the engine loads the map.");
        }
        if (state.options.drawlerp)
        {
            state.options.direct_scale = 0.0;
        }

        if (map.visibility.empty())
        {
            logging::warn("No vis information.");
        }
        if (state.options.blur < 1.0)
        {
            state.options.blur = 1.0;
        }

        // size the per face tables
        state.face_patches.assign(map.faces.size(), nullptr);
        state.face_entity.assign(map.faces.size(), nullptr);
        state.face_texlights.assign(map.faces.size(), nullptr);
        state.face_offset.assign(map.faces.size(), vec3v{});
        state.face_centroids.assign(map.faces.size(), vec3v{});
        state.face_lightmode.assign(map.faces.size(), 0);
        state.facelights.assign(map.faces.size(), facelight{});
        state.face_positions.resize(map.faces.size());
        state.facetriangulations.assign(map.faces.size(), nullptr);

        // fail fast on lightmap atlas overflow: it only needs face extents, so
        // there is no point running the heavy lighting passes on a map the
        // engine cannot load
        if (check_alloc_block_budget(state, alloc_block_pages))
        {
            return false;
        }

        rad_world(state);
        if (state.options.direct_state_hook)
        {
            return true; // test hook ran inside rad_world; skip the output stages
        }
        free_studio_models(state);
        free_opaque_face_list(state);
        free_patches(state);
        delete_opaque_nodes(state);

        embed_lightmap_in_textures(state);

        return true;
    }
}
