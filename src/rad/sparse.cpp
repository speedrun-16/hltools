#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "../common/error.h"
#include "../common/limits.h"
#include "../common/log.h"
#include "../common/progress.h"
#include "../common/threads.h"
#include "internal.h"

#ifdef HLTOOLS_GPU_ENABLED
#include <vector>
#include "gpu/gpu.h"
#endif

// the sparse visibility matrix method (the default): per patch columns of
// runs of 8 bit values, so empty regions of the matrix cost nothing

namespace rad
{
    namespace
    {
        // binary search the column for the row containing bit y
        unsigned is_visbit_in_array(const rad_state &state, const unsigned x, const unsigned y)
        {
            int first, last, current;
            int y_byte = (int)(y / 8);
            const sparse_row *row;
            const sparse_column *column = state.sparse_columns.data() + x;

            if (!column->count)
            {
                return (unsigned)-1;
            }

            first = 0;
            last = column->count - 1;

            while (1)
            {
                current = (first + last) / 2;
                row = column->row + current;
                if ((int)(row->offset) < y_byte)
                {
                    first = current + 1;
                }
                else if ((int)(row->offset) > y_byte)
                {
                    last = current - 1;
                }
                else
                {
                    return (unsigned)current;
                }
                if (first > last)
                {
                    return (unsigned)-1;
                }
            }
        }

        void set_vis_column(rad_state &state, int patchnum, const char *uncompressedcolumn)
        {
            sparse_column *column;
            int mbegin;
            int m;
            int i;
            unsigned int bits;

            column = &state.sparse_columns[(size_t)patchnum];
            if (column->count || column->row)
            {
                err::fatal("set_vis_column: column has been set");
            }

            for (mbegin = 0; mbegin < (int)state.num_patches; mbegin += 8)
            {
                bits = 0;
                for (m = mbegin; m < mbegin + 8; m++)
                {
                    if (m >= (int)state.num_patches)
                    {
                        break;
                    }
                    if (uncompressedcolumn[m]) // visible
                    {
                        if (m < patchnum)
                        {
                            err::fatal("set_vis_column: invalid parameter: m < patchnum");
                        }
                        bits |= (1 << (m - mbegin));
                    }
                }
                if (bits)
                {
                    column->count++;
                }
            }

            if (!column->count)
            {
                return;
            }
            column->row = (sparse_row *)std::malloc((size_t)column->count * sizeof(sparse_row));
            err::require(column->row != nullptr, "set_vis_column: out of memory");

            i = 0;
            for (mbegin = 0; mbegin < (int)state.num_patches; mbegin += 8)
            {
                bits = 0;
                for (m = mbegin; m < mbegin + 8; m++)
                {
                    if (m >= (int)state.num_patches)
                    {
                        break;
                    }
                    if (uncompressedcolumn[m]) // visible
                    {
                        bits |= (1 << (m - mbegin));
                    }
                }
                if (bits)
                {
                    column->row[i].offset = (unsigned)(mbegin / 8);
                    column->row[i].values = bits;
                    i++;
                }
            }
            if (i != column->count)
            {
                err::fatal("set_vis_column: internal error");
            }
        }

        bool check_vis_bit_sparse(rad_state &state, unsigned x, unsigned y,
                                  vec3v &transparency_out, unsigned int &next_index)
        {
            unsigned offset;

            transparency_out = vec3_one;

            if (x == y)
            {
                return true;
            }

            const unsigned a = x;
            const unsigned b = y;

            if (x > y)
            {
                x = b;
                y = a;
            }

            if (x > state.num_patches)
            {
                logging::warn("in CheckVisBit(), x > num_patches");
            }
            if (y > state.num_patches)
            {
                logging::warn("in CheckVisBit(), y > num_patches");
            }

            if ((offset = is_visbit_in_array(state, x, y)) != (unsigned)-1)
            {
                if (state.options.customshadow_with_bouncelight)
                {
                    get_transparency(state, a, b, transparency_out, next_index);
                }
                return (state.sparse_columns[x].row[offset].values & (1 << (y & 7))) != 0;
            }

            return false;
        }

        // sets vis bits for all patches in the face
        void test_patch_to_face(rad_state &state, const unsigned patchnum, const int facenum,
                                byte *pvs, char *uncompressedcolumn)
        {
            patch *pt = &state.patches[patchnum];
            patch *patch2 = state.face_patches[(size_t)facenum];

            // if emitter is behind that face plane, skip all patches

            if (patch2)
            {
                const plane *plane2 = plane_from_face_number(state, (unsigned)facenum);

                if (math::dot(pt->origin, plane2->normal) > patch_plane_dist(state, patch2) + math::on_epsilon - pt->emitter_range)
                {
                    // we need to do a real test
                    const plane *pl = plane_from_face_number(state, (unsigned)pt->facenumber);

                    for (; patch2; patch2 = patch2->next)
                    {
                        unsigned m = (unsigned)(patch2 - state.patches.data());

                        vec3v transparency = {1.0, 1.0, 1.0};
                        int opaquestyle = -1;

                        // check vis between patch and patch2
                        // if bit has not already been set
                        //  && v2 is not behind light plane
                        //  && v2 is visible from v1
                        if (m > patchnum)
                        {
                            if (patch2->leafnum == 0 || !(pvs[(patch2->leafnum - 1) >> 3] & (1 << ((patch2->leafnum - 1) & 7))))
                            {
                                continue;
                            }
                            vec3v origin1, origin2;
                            vec3v delta;
                            vec_t dist;
                            math::subtract(pt->origin, patch2->origin, delta);
                            dist = (vec_t)math::length(delta);
                            if (dist < patch2->emitter_range - math::on_epsilon)
                            {
                                get_alternate_origin(state, pt->origin, pl->normal, patch2, origin2);
                            }
                            else
                            {
                                math::copy(patch2->origin, origin2);
                            }
                            if (math::dot(origin2, pl->normal) <= patch_plane_dist(state, pt) + minimum_patch_distance)
                            {
                                continue;
                            }
                            if (dist < pt->emitter_range - math::on_epsilon)
                            {
                                get_alternate_origin(state, patch2->origin, plane2->normal, pt, origin1);
                            }
                            else
                            {
                                math::copy(pt->origin, origin1);
                            }
                            if (math::dot(origin1, plane2->normal) <= patch_plane_dist(state, patch2) + minimum_patch_distance)
                            {
                                continue;
                            }
                            if (test_line(state, origin1, origin2) != contents_empty)
                            {
                                continue;
                            }
                            if (test_segment_against_opaque_list(state, origin1, origin2, transparency, opaquestyle))
                            {
                                continue;
                            }

                            if (opaquestyle != -1)
                            {
                                add_style_to_style_array(state, m, patchnum, opaquestyle);
                                add_style_to_style_array(state, patchnum, m, opaquestyle);
                            }

                            if (state.options.customshadow_with_bouncelight && !math::equal(transparency, vec3_one))
                            {
                                add_transparency_to_raw_array(state, patchnum, m, transparency);
                            }
                            uncompressedcolumn[m] = 1;
                        }
                    }
                }
            }
        }

        thread_local std::vector<char> tls_uncompressedcolumn;

        // one work item per vis leaf; runs on multiple threads
        void build_vis_leafs_item(rad_state &state, int i)
        {
            byte pvs[(limits::max_map_leafs + 7) / 8];
            format::dleaf_t *srcleaf;
            patch *pt;
            unsigned patchnum;

            tls_uncompressedcolumn.resize(state.num_patches);
            char *uncompressedcolumn = tls_uncompressedcolumn.data();

            i++; // skip leaf 0
            srcleaf = &state.map->leafs[(size_t)i];
            if (state.map->visibility.empty())
            {
                std::memset(pvs, 255, (size_t)((state.map->models[0].visleafs + 7) / 8));
            }
            else
            {
                if (srcleaf->visofs == -1)
                {
                    return;
                }
                decompress_vis(state, &state.map->visibility[(size_t)srcleaf->visofs], pvs, sizeof(pvs));
            }

            // go through all the faces inside the leaf, and process the
            // patches that actually have origins inside
            for (int facenum = 0; facenum < (int)state.map->faces.size(); facenum++)
            {
                for (pt = state.face_patches[(size_t)facenum]; pt; pt = pt->next)
                {
                    if (pt->leafnum != i)
                        continue;
                    patchnum = (unsigned)(pt - state.patches.data());
                    for (int m = 0; m < (int)state.num_patches; m++)
                    {
                        uncompressedcolumn[m] = 0;
                    }
                    for (int facenum2 = facenum + 1; facenum2 < (int)state.map->faces.size(); facenum2++)
                        test_patch_to_face(state, patchnum, facenum2, pvs, uncompressedcolumn);
                    set_vis_column(state, (int)patchnum, uncompressedcolumn);
                }
            }
        }

#ifdef HLTOOLS_GPU_ENABLED
        // gpu variant of the vis matrix build: the pre trace logic mirrors
        // test_patch_to_face exactly (pvs cull, plane rejects, alternate
        // origins on the cpu), and only the test_line runs on the gpu trace
        // kernel which matches the cpu tracer so
        // the columns come out identical to the cpu build only used when the
        // map has no opaque entities or studio models (their segment tests and
        // the style/transparency side arrays are cpu only paths)

        thread_local std::vector<byte> tls_gpu_pvs;
        thread_local int tls_gpu_pvs_visofs = -3; // -3 = nothing cached

        // decompressed pvs row for a patch's leaf, cached per thread
        const byte *gpu_patch_pvs(rad_state &state, const patch *pt)
        {
            const size_t pvs_bytes = (size_t)((limits::max_map_leafs + 7) / 8);
            tls_gpu_pvs.resize(pvs_bytes);
            if (state.map->visibility.empty())
            {
                if (tls_gpu_pvs_visofs != -2)
                {
                    std::memset(tls_gpu_pvs.data(), 255,
                                (size_t)((state.map->models[0].visleafs + 7) / 8));
                    tls_gpu_pvs_visofs = -2;
                }
                return tls_gpu_pvs.data();
            }
            const int visofs = state.map->leafs[(size_t)pt->leafnum].visofs;
            if (visofs == -1)
            {
                return nullptr; // the cpu build skips these leaves entirely
            }
            if (tls_gpu_pvs_visofs != visofs)
            {
                decompress_vis(state, &state.map->visibility[(size_t)visofs],
                               tls_gpu_pvs.data(), (unsigned)pvs_bytes);
                tls_gpu_pvs_visofs = visofs;
            }
            return tls_gpu_pvs.data();
        }

        // mirror of test_patch_to_face up to (but not including) the
        // test_line: emits one segment + target patch per surviving pair
        void gpu_enumerate_patch(rad_state &state, const unsigned patchnum,
                                 std::vector<gpu::trace_segment> &segments,
                                 std::vector<unsigned> &targets)
        {
            patch *pt = &state.patches[patchnum];
            if (pt->leafnum < 1 || pt->leafnum > (int)state.map->models[0].visleafs)
            {
                return; // the cpu build only walks leaves 1visleafs
            }
            const byte *pvs = gpu_patch_pvs(state, pt);
            if (!pvs)
            {
                return;
            }
            const plane *pl = plane_from_face_number(state, (unsigned)pt->facenumber);

            for (int facenum2 = pt->facenumber + 1; facenum2 < (int)state.map->faces.size(); facenum2++)
            {
                patch *patch2 = state.face_patches[(size_t)facenum2];
                if (!patch2)
                {
                    continue;
                }
                const plane *plane2 = plane_from_face_number(state, (unsigned)facenum2);
                if (!(math::dot(pt->origin, plane2->normal)
                      > patch_plane_dist(state, patch2) + math::on_epsilon - pt->emitter_range))
                {
                    continue;
                }
                for (; patch2; patch2 = patch2->next)
                {
                    unsigned m = (unsigned)(patch2 - state.patches.data());
                    if (m <= patchnum)
                    {
                        continue;
                    }
                    if (patch2->leafnum == 0
                        || !(pvs[(patch2->leafnum - 1) >> 3] & (1 << ((patch2->leafnum - 1) & 7))))
                    {
                        continue;
                    }
                    vec3v origin1, origin2;
                    vec3v delta;
                    vec_t dist;
                    math::subtract(pt->origin, patch2->origin, delta);
                    dist = (vec_t)math::length(delta);
                    if (dist < patch2->emitter_range - math::on_epsilon)
                    {
                        get_alternate_origin(state, pt->origin, pl->normal, patch2, origin2);
                    }
                    else
                    {
                        math::copy(patch2->origin, origin2);
                    }
                    if (math::dot(origin2, pl->normal) <= patch_plane_dist(state, pt) + minimum_patch_distance)
                    {
                        continue;
                    }
                    if (dist < pt->emitter_range - math::on_epsilon)
                    {
                        get_alternate_origin(state, patch2->origin, plane2->normal, pt, origin1);
                    }
                    else
                    {
                        math::copy(pt->origin, origin1);
                    }
                    if (math::dot(origin1, plane2->normal) <= patch_plane_dist(state, patch2) + minimum_patch_distance)
                    {
                        continue;
                    }
                    gpu::trace_segment seg;
                    seg.start[0] = origin1[0];
                    seg.start[1] = origin1[1];
                    seg.start[2] = origin1[2];
                    seg.stop[0] = origin2[0];
                    seg.stop[1] = origin2[1];
                    seg.stop[2] = origin2[2];
                    segments.push_back(seg);
                    targets.push_back(m);
                }
            }
        }

        bool gpu_build_vis_matrix(rad_state &state)
        {
            if (!state.opaque_list.empty() || state.num_studio_models != 0)
            {
                return false; // opaque/studio segment tests are cpu only
            }
            if (!gpu::available())
            {
                logging::warn("-gpu: %s; building patch visibility on the cpu",
                              gpu::last_error().c_str());
                return false;
            }

            std::vector<gpu::tnode_gpu> tnodes(state.tnodes.size());
            for (size_t i = 0; i < state.tnodes.size(); i++)
            {
                const tnode &tn = state.tnodes[i];
                tnodes[i].normal[0] = tn.normal[0];
                tnodes[i].normal[1] = tn.normal[1];
                tnodes[i].normal[2] = tn.normal[2];
                tnodes[i].dist = tn.dist;
                tnodes[i].type = tn.type;
                tnodes[i].children[0] = tn.children[0];
                tnodes[i].children[1] = tn.children[1];
            }

            progress::section("lighting");
            progress::begin("building patch visibility", (int)state.num_patches);

            constexpr unsigned chunk_patches = 2048;
            std::vector<std::vector<gpu::trace_segment>> segs(chunk_patches);
            std::vector<std::vector<unsigned>> targs(chunk_patches);

            for (unsigned base = 0; base < state.num_patches; base += chunk_patches)
            {
                const unsigned count = state.num_patches - base < chunk_patches
                    ? state.num_patches - base : chunk_patches;

                // pass a (cpu, threaded): enumerate candidate pairs
                threads::run((int)count, [&](int k)
                             {
                                 segs[(size_t)k].clear();
                                 targs[(size_t)k].clear();
                                 gpu_enumerate_patch(state, base + (unsigned)k,
                                                     segs[(size_t)k], targs[(size_t)k]);
                             });

                // pass b (gpu): trace every candidate segment
                std::vector<gpu::trace_segment> flat;
                std::vector<size_t> first((size_t)count + 1, 0);
                for (unsigned k = 0; k < count; k++)
                {
                    first[k] = flat.size();
                    flat.insert(flat.end(), segs[k].begin(), segs[k].end());
                }
                first[count] = flat.size();
                std::vector<gpu::trace_result> results;
                if (!flat.empty()
                    && !gpu::trace_batch(tnodes, flat, results))
                {
                    progress::end();
                    logging::warn("-gpu: %s; building patch visibility on the cpu",
                                  gpu::last_error().c_str());
                    return false;
                }

                // pass c (cpu, threaded): fill and compress the columns
                threads::run((int)count, [&](int k)
                             {
                                 const unsigned patchnum = base + (unsigned)k;
                                 patch *pt = &state.patches[patchnum];
                                 // untouched leaves keep their default empty
                                 // column, like the cpu build
                                 bool active = pt->leafnum >= 1
                                     && pt->leafnum <= (int)state.map->models[0].visleafs
                                     && (state.map->visibility.empty()
                                         || state.map->leafs[(size_t)pt->leafnum].visofs != -1);
                                 if (active)
                                 {
                                     tls_uncompressedcolumn.resize(state.num_patches);
                                     char *column = tls_uncompressedcolumn.data();
                                     std::memset(column, 0, state.num_patches);
                                     for (size_t s = first[(size_t)k]; s < first[(size_t)k + 1]; s++)
                                     {
                                         if (results[s].contents == contents_empty)
                                         {
                                             column[targs[(size_t)k][s - first[(size_t)k]]] = 1;
                                         }
                                     }
                                     set_vis_column(state, (int)patchnum, column);
                                 }
                                 progress::add(1);
                             });
            }

            progress::end();
            return true;
        }
#endif

        void build_vis_matrix(rad_state &state)
        {
            state.sparse_columns.assign(state.num_patches, sparse_column{});

#ifdef HLTOOLS_GPU_ENABLED
            // measured on celerior: bit identical columns but 22x slower
            // than the cpu build (58s -> 130s) - the phase is bound by the
            // pvs cull and alternate origin hunts, not the traces, and the
            // segment upload/readback outweighs the ray savings kept behind
            // an env switch as the base for a future overlap pipelined try
            if (state.options.gpu && std::getenv("HLTOOLS_GPU_VISMATRIX")
                && gpu_build_vis_matrix(state))
            {
                return;
            }
#endif
            threads::run_phase("lighting", "building patch visibility",
                               state.map->models[0].visleafs,
                               [&](int i) { build_vis_leafs_item(state, i); });
        }

        void free_vis_matrix(rad_state &state)
        {
            for (size_t x = 0; x < state.sparse_columns.size(); x++)
            {
                if (state.sparse_columns[x].row)
                {
                    std::free(state.sparse_columns[x].row);
                }
            }
            state.sparse_columns.clear();
            state.sparse_columns.shrink_to_fit();
        }

        void dump_vismatrix_info(const rad_state &state)
        {
            size_t total_vismatrix_memory = sizeof(sparse_column) * state.num_patches;

            for (size_t x = 0; x < state.sparse_columns.size(); x++)
            {
                total_vismatrix_memory += (size_t)state.sparse_columns[x].count * sizeof(sparse_row);
            }

            logging::file("      %-18s %.1f MB\n", "vismatrix", total_vismatrix_memory / (1024 * 1024.0));
        }
    }

    void make_scales_sparse_vismatrix(rad_state &state)
    {
        err::require((int)state.num_patches < max_sparse_vismatrix_patches,
                     "make_scales_sparse_vismatrix: exceeded MAX_SPARSE_VISMATRIX_PATCHES");

        std::string transferfile = state.base_path + ".inc";

        if (!state.options.incremental || !read_transfers(state, transferfile.c_str(), (long)state.num_patches))
        {
            // determine visibility between patches
            build_vis_matrix(state);
            dump_vismatrix_info(state);
            state.check_vis_bit = check_vis_bit_sparse;

            create_final_transparency_arrays(state, "custom shadow array");

            run_transfer_scales(state);
            free_vis_matrix(state);
            free_transparency_arrays(state);

            if (state.options.incremental)
            {
                write_transfers(state, transferfile.c_str(), (long)state.num_patches);
            }
            else
            {
                std::remove(transferfile.c_str());
            }
            dump_transfers_memory_usage(state);
            create_final_style_arrays(state, "dynamic shadow array");
        }
    }
}
