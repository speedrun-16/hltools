#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "../common/error.h"
#include "../common/log.h"
#include "../common/string_util.h"
#include "format/bsp/face_extents.h"
#include "internal.h"

// the output passes: scale the direct light, add the interpolated bounced
// light on top, convert to the final byte lightmap (gamma, clipping, minlight)
// and lay out the lighting lump also handles zhlt_copylight for brush models

namespace rad
{
    // ===== scale direct lights =====

    void scale_direct_lights(rad_state &state)
    {
        format::map_data &map = *state.map;
        int facenum;
        format::dface_t *f;
        facelight *fl;
        int i;
        int k;
        sample *samp;

        for (facenum = 0; facenum < (int)map.faces.size(); facenum++)
        {
            f = &map.faces[(size_t)facenum];

            if (map.texinfo[(size_t)f->texinfo].flags & tex_special)
            {
                continue;
            }

            fl = &state.facelights[(size_t)facenum];

            for (k = 0; k < maxlightmaps && f->styles[k] != 255; k++)
            {
                for (i = 0; i < fl->numsamples; i++)
                {
                    samp = &fl->samples[k][i];
                    math::scale(samp->light, state.options.direct_scale, samp->light);
                }
            }
        }
    }

    // ===== facelight dependencies =====

    void create_facelight_dependency_list(rad_state &state)
    {
        format::map_data &map = *state.map;
        int facenum;
        format::dface_t *f;
        facelight *fl;
        int i;
        int k;
        int surface;

        state.dependentfacelights.assign(map.faces.size(), {});

        // for each face
        for (facenum = 0; facenum < (int)map.faces.size(); facenum++)
        {
            f = &map.faces[(size_t)facenum];
            fl = &state.facelights[(size_t)facenum];
            if (map.texinfo[(size_t)f->texinfo].flags & tex_special)
            {
                continue;
            }

            for (k = 0; k < maxlightmaps && f->styles[k] != 255; k++)
            {
                for (i = 0; i < fl->numsamples; i++)
                {
                    surface = fl->samples[k][i].surface; // that surface contains at least one sample from this face
                    if (0 <= surface && surface < (int)map.faces.size())
                    {
                        // insert this face into the dependency list of that surface
                        std::vector<int> &list = state.dependentfacelights[(size_t)surface];
                        if (std::find(list.begin(), list.end(), facenum) != list.end())
                        {
                            continue;
                        }
                        list.push_back(facenum);
                    }
                }
            }
        }
    }

    void free_facelight_dependency_list(rad_state &state)
    {
        state.dependentfacelights.clear();
        state.dependentfacelights.shrink_to_fit();
    }

    // ===== add patch lights =====

    // adds the interpolated patch (bounced) light on top of each sample that
    // grew into this face runs multithreaded per face
    void add_patch_lights(rad_state &state, int facenum)
    {
        format::map_data &map = *state.map;
        format::dface_t *f;
        format::dface_t *f_other;
        facelight *fl_other;
        int k;
        int i;
        sample *samp;

        f = &map.faces[(size_t)facenum];

        if (map.texinfo[(size_t)f->texinfo].flags & tex_special)
        {
            return;
        }

        // the reference built this list by prepending, so iterate in reverse
        const std::vector<int> &items = state.dependentfacelights[(size_t)facenum];
        for (int it = (int)items.size() - 1; it >= 0; it--)
        {
            f_other = &map.faces[(size_t)items[(size_t)it]];
            fl_other = &state.facelights[(size_t)items[(size_t)it]];
            for (k = 0; k < maxlightmaps && f_other->styles[k] != 255; k++)
            {
                for (i = 0; i < fl_other->numsamples; i++)
                {
                    samp = &fl_other->samples[k][i];
                    if (samp->surface != facenum)
                    {
                        // the sample is not in this surface
                        continue;
                    }

                    {
                        vec3v v;

                        int style = f_other->styles[k];
                        interpolate_sample_light(state, samp->pos, samp->surface, 1, &style, &v);

                        math::add(samp->light, v, v);
                        if (vector_maximum(v) >= state.corings[f_other->styles[k]])
                        {
                            math::copy(v, samp->light);
                        }
                        else
                        {
                            if (vector_maximum(v) > state.maxdiscardedlight + math::normal_epsilon)
                            {
                                std::lock_guard<std::mutex> guard(state.lock);
                                if (vector_maximum(v) > state.maxdiscardedlight + math::normal_epsilon)
                                {
                                    state.maxdiscardedlight = vector_maximum(v);
                                    math::copy(samp->pos, state.maxdiscardedpos);
                                }
                            }
                        }
                    }
                } // loop samples
            }
        }
    }

    // ===== final light face =====

    // adds the indirect lighting on top of the direct lighting and saves it
    // into the final map format
    void final_light_face(rad_state &state, const int facenum)
    {
        format::map_data &map = *state.map;

        if (facenum == 0 && state.options.drawsample)
        {
            std::string name = state.base_path + "_sample.pts";
            logging::info("Writing '%s' ...\n", name.c_str());
            FILE *f = std::fopen(name.c_str(), "w");
            if (f)
            {
                const int pos_count = 15;
                const vec3v pos[pos_count] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {-1, 0, 0}, {0, -1, 0},
                                              {1, 0, 0}, {0, 0, 1}, {-1, 0, 0}, {0, 0, -1}, {0, -1, 0},
                                              {0, 0, 1}, {0, 1, 0}, {0, 0, -1}, {1, 0, 0}, {0, 0, 0}};
                int i, j, k;
                vec3v v, dist;
                vec3v origin{state.options.drawsample_origin[0], state.options.drawsample_origin[1],
                             state.options.drawsample_origin[2]};
                for (i = 0; i < (int)map.faces.size(); ++i)
                {
                    const facelight *fl = &state.facelights[(size_t)i];
                    for (j = 0; j < fl->numsamples; ++j)
                    {
                        math::copy(fl->samples[0][j].pos, v);
                        math::subtract(v, origin, dist);
                        if (math::dot(dist, dist) < state.options.drawsample_radius * state.options.drawsample_radius)
                        {
                            for (k = 0; k < pos_count; ++k)
                                std::fprintf(f, "%g %g %g\n", v[0] + pos[k][0], v[1] + pos[k][1], v[2] + pos[k][2]);
                        }
                    }
                }
                std::fclose(f);
                logging::info("OK.\n");
            }
            else
                logging::info("Error.\n");
        }
        int i, j, k;
        vec3v lb;
        facelight *fl;
        sample *samp;
        float minlight;
        int lightstyles;
        format::dface_t *f;
        vec3v *original_basiclight;
        int (*final_basiclight)[3];
        int lbi[3];

        float temp_rand;

        f = &map.faces[(size_t)facenum];
        fl = &state.facelights[(size_t)facenum];

        if (map.texinfo[(size_t)f->texinfo].flags & tex_special)
        {
            return; // non lit texture
        }

        for (lightstyles = 0; lightstyles < maxlightmaps; lightstyles++)
        {
            if (f->styles[lightstyles] == 255)
            {
                break;
            }
        }

        if (!lightstyles)
        {
            return;
        }

        minlight = float_for_key(*state.face_entity[(size_t)facenum], "_minlight") * 255;
        minlight = (minlight > 255) ? 255 : minlight;

        const char *texname = texture_by_number(state, f->texinfo);

        if (texname[0] == '%') // texture name with the % minlight flag
        {
            size_t texname_length = strlen(texname);

            if (texname_length > 1)
            {
                if (texname[1] >= '0' && texname[1] <= '9')
                {
                    std::string value;
                    size_t x = 1;
                    while (texname[x] != '\0' && texname[x] >= '0' && texname[x] <= '9' && value.size() < texname_length)
                    {
                        value.push_back(texname[x++]);
                    }
                    minlight = (float)atoi(value.c_str());
                    minlight = (minlight > 255) ? 255 : minlight;
                }
            }
            else
            {
                minlight = 255;
            }
        }

        for (size_t it = 0; it < state.minlights.size(); it++)
        {
            if (str::iequals(texname, state.minlights[it].name.c_str()))
            {
                float minlight_value = state.minlights[it].value * 255.0f;
                minlight = (float)static_cast<int>(minlight_value);
                minlight = (minlight > 255) ? 255 : minlight;
            }
        }
        original_basiclight = (vec3v *)calloc((size_t)fl->numsamples, sizeof(vec3v));
        final_basiclight = (int (*)[3])calloc((size_t)fl->numsamples, sizeof(int[3]));
        err::require(original_basiclight != nullptr, "final_light_face: out of memory");
        err::require(final_basiclight != nullptr, "final_light_face: out of memory");
        for (k = 0; k < lightstyles; k++)
        {
            samp = fl->samples[k];
            for (j = 0; j < fl->numsamples; j++, samp++)
            {
                math::copy(samp->light, lb);
                if (f->styles[0] != 0)
                {
                    logging::warn("wrong f->styles[0]");
                }
                for (i = 0; i < 3; i++)
                    lb[i] = lb[i] > 0 ? lb[i] : 0;
                if (k == 0)
                {
                    math::copy(lb, original_basiclight[j]);
                }
                else
                {
                    math::add(lb, original_basiclight[j], lb);
                }
                // colour lightscale
                lb[0] *= state.options.colour_lightscale[0];
                lb[1] *= state.options.colour_lightscale[1];
                lb[2] *= state.options.colour_lightscale[2];

                // clip from the bottom first
                for (i = 0; i < 3; i++)
                {
                    if (lb[i] < minlight)
                    {
                        lb[i] = minlight;
                    }
                }

                if (state.options.colour_qgamma[0] != 1.0)
                    lb[0] = (float)std::pow((double)(lb[0] / 256.0f), state.options.colour_qgamma[0]) * 256.0f;

                if (state.options.colour_qgamma[1] != 1.0)
                    lb[1] = (float)std::pow((double)(lb[1] / 256.0f), state.options.colour_qgamma[1]) * 256.0f;

                if (state.options.colour_qgamma[2] != 1.0)
                    lb[2] = (float)std::pow((double)(lb[2] / 256.0f), state.options.colour_qgamma[2]) * 256.0f;

                // clip from the top
                {
                    vec_t max = vector_maximum(lb);
                    if (state.options.limitthreshold >= 0 && max > state.options.limitthreshold)
                    {
                        if (!state.options.drawoverload)
                        {
                            math::scale(lb, state.options.limitthreshold / max, lb);
                        }
                    }
                    else
                    {
                        if (state.options.drawoverload)
                        {
                            math::scale(lb, 0.1, lb); // darken good points
                        }
                    }
                }
                for (i = 0; i < 3; ++i)
                    if (lb[i] < state.options.minlight)
                        lb[i] = state.options.minlight;
                for (i = 0; i < 3; ++i)
                {
                    lbi[i] = (int)std::floor(lb[i] + 0.5);
                    if (lbi[i] < 0)
                        lbi[i] = 0;
                }
                if (k == 0)
                {
                    for (i = 0; i < 3; i++)
                        final_basiclight[j][i] = lbi[i];
                }
                else
                {
                    for (i = 0; i < 3; i++)
                        lbi[i] = lbi[i] - final_basiclight[j][i];
                }
                if (k == 0)
                {
                    if (state.options.colour_jitter_hack[0] || state.options.colour_jitter_hack[1] || state.options.colour_jitter_hack[2])
                        for (i = 0; i < 3; i++)
                            lbi[i] = (int)(lbi[i] + state.options.colour_jitter_hack[i] * ((float)rand() / RAND_MAX - 0.5));
                    if (state.options.jitter_hack[0] || state.options.jitter_hack[1] || state.options.jitter_hack[2])
                    {
                        temp_rand = (float)rand() / RAND_MAX - 0.5f;
                        for (i = 0; i < 3; i++)
                            lbi[i] = (int)(lbi[i] + state.options.jitter_hack[i] * temp_rand);
                    }
                }
                for (i = 0; i < 3; ++i)
                {
                    if (lbi[i] < 0)
                        lbi[i] = 0;
                    if (lbi[i] > 255)
                        lbi[i] = 255;
                }
                {
                    unsigned char *colors = &map.lighting[(size_t)(f->lightofs + k * fl->numsamples * 3 + j * 3)];

                    colors[0] = (unsigned char)lbi[0];
                    colors[1] = (unsigned char)lbi[1];
                    colors[2] = (unsigned char)lbi[2];
                }
            }
        }
        free(original_basiclight);
        free(final_basiclight);
    }

    // ===== lightmap offsets =====

    void precomp_lightmap_offsets(rad_state &state)
    {
        format::map_data &map = *state.map;
        int facenum;
        format::dface_t *f;
        facelight *fl;
        int lightstyles;

        patch *pt;

        int lightdatasize = 0;

        for (facenum = 0; facenum < (int)map.faces.size(); facenum++)
        {
            f = &map.faces[(size_t)facenum];
            fl = &state.facelights[(size_t)facenum];

            if (map.texinfo[(size_t)f->texinfo].flags & tex_special)
            {
                continue; // non lit texture
            }

            {
                int i, j, k;
                vec_t maxlights[allstyles];
                {
                    vec3v maxlights1[allstyles];
                    vec3v maxlights2[allstyles];
                    for (j = 0; j < allstyles; j++)
                    {
                        math::clear(maxlights1[j]);
                        math::clear(maxlights2[j]);
                    }
                    for (k = 0; k < maxlightmaps && f->styles[k] != 255; k++)
                    {
                        for (i = 0; i < fl->numsamples; i++)
                        {
                            for (int x = 0; x < 3; x++)
                            {
                                vec_t v = fl->samples[k][i].light[x];
                                if (v > maxlights1[f->styles[k]][x])
                                    maxlights1[f->styles[k]][x] = v;
                            }
                        }
                    }
                    int numpatches;
                    const int *patches;
                    get_triangulation_patches(state, facenum, &numpatches, &patches); // collect patches and their neighbors

                    for (i = 0; i < numpatches; i++)
                    {
                        pt = &state.patches[(size_t)patches[i]];
                        for (k = 0; k < maxlightmaps && pt->totalstyle[k] != 255; k++)
                        {
                            for (int x = 0; x < 3; x++)
                            {
                                vec_t v = pt->totallight[k][x];
                                if (v > maxlights2[pt->totalstyle[k]][x])
                                    maxlights2[pt->totalstyle[k]][x] = v;
                            }
                        }
                    }
                    for (j = 0; j < allstyles; j++)
                    {
                        vec3v v;
                        math::add(maxlights1[j], maxlights2[j], v);
                        maxlights[j] = vector_maximum(v);
                        if (maxlights[j] <= state.corings[j] * 0.01)
                        {
                            if (maxlights[j] > state.maxdiscardedlight + math::normal_epsilon)
                            {
                                state.maxdiscardedlight = maxlights[j];
                                math::copy(state.face_centroids[(size_t)facenum], state.maxdiscardedpos);
                            }
                            maxlights[j] = 0;
                        }
                    }
                }
                unsigned char oldstyles[maxlightmaps];
                sample *oldsamples[maxlightmaps];
                for (k = 0; k < maxlightmaps; k++)
                {
                    oldstyles[k] = f->styles[k];
                    oldsamples[k] = fl->samples[k];
                }
                for (k = 0; k < maxlightmaps; k++)
                {
                    unsigned char beststyle = 255;
                    if (k == 0)
                    {
                        beststyle = 0;
                    }
                    else
                    {
                        vec_t bestmaxlight = 0;
                        for (j = 1; j < allstyles; j++)
                        {
                            if (maxlights[j] > bestmaxlight + math::normal_epsilon)
                            {
                                bestmaxlight = maxlights[j];
                                beststyle = (unsigned char)j;
                            }
                        }
                    }
                    if (beststyle != 255)
                    {
                        maxlights[beststyle] = 0;
                        f->styles[k] = beststyle;
                        fl->samples[k] = (sample *)malloc((size_t)fl->numsamples * sizeof(sample));
                        err::require(fl->samples[k] != nullptr, "precomp_lightmap_offsets: out of memory");
                        for (i = 0; i < maxlightmaps && oldstyles[i] != 255; i++)
                        {
                            if (oldstyles[i] == f->styles[k])
                            {
                                break;
                            }
                        }
                        if (i < maxlightmaps && oldstyles[i] != 255)
                        {
                            memcpy(fl->samples[k], oldsamples[i], (size_t)fl->numsamples * sizeof(sample));
                        }
                        else
                        {
                            // copy samplepos from style 0 to the new style,
                            // because samplepos is the same for all styles
                            memcpy(fl->samples[k], oldsamples[0], (size_t)fl->numsamples * sizeof(sample));
                            for (j = 0; j < fl->numsamples; j++)
                            {
                                math::clear(fl->samples[k][j].light);
                            }
                        }
                    }
                    else
                    {
                        f->styles[k] = 255;
                        fl->samples[k] = nullptr;
                    }
                }
                for (j = 1; j < allstyles; j++)
                {
                    if (maxlights[j] > state.maxdiscardedlight + math::normal_epsilon)
                    {
                        state.maxdiscardedlight = maxlights[j];
                        math::copy(state.face_centroids[(size_t)facenum], state.maxdiscardedpos);
                    }
                }
                for (k = 0; k < maxlightmaps && oldstyles[k] != 255; k++)
                {
                    free(oldsamples[k]);
                }
            }

            for (lightstyles = 0; lightstyles < maxlightmaps; lightstyles++)
            {
                if (f->styles[lightstyles] == 255)
                {
                    break;
                }
            }

            if (!lightstyles)
            {
                continue;
            }

            f->lightofs = lightdatasize;
            lightdatasize += fl->numsamples * 3 * lightstyles;
            err::require(lightdatasize <= state.max_map_lightdata,
                         "precomp_lightmap_offsets: exceeded MAX_MAP_LIGHTING; use -lightdata to raise the limit");
        }

        map.lighting.assign((size_t)lightdatasize, 0);
    }

    void reduce_lightmap(rad_state &state)
    {
        format::map_data &map = *state.map;
        std::vector<byte> oldlightdata = map.lighting;
        int lightdatasize = 0;

        int facenum;
        for (facenum = 0; facenum < (int)map.faces.size(); facenum++)
        {
            format::dface_t *f = &map.faces[(size_t)facenum];
            facelight *fl = &state.facelights[(size_t)facenum];
            if (map.texinfo[(size_t)f->texinfo].flags & tex_special)
            {
                continue; // non lit texture
            }
            if (is_unlit_texture(state, texture_by_number(state, f->texinfo)))
            {
                // An ordinary GoldSrc face with lightofs -1 is not fullbright:
                // the renderer still takes its regular lightmapped path and the
                // missing samples appear black. Give unlit materials one
                // constant-white style-0 lightmap instead. Modulating the base
                // texture by white reproduces Source's whole-surface unlit
                // shader while retaining the normal surface render path.
                f->lightofs = lightdatasize;
                f->styles[0] = 0;
                for (int k = 1; k < maxlightmaps; k++)
                    f->styles[k] = 255;
                int bytes = fl->numsamples * 3;
                err::require(lightdatasize + bytes <= state.max_map_lightdata,
                             "reduce_lightmap: exceeded MAX_MAP_LIGHTING");
                std::fill_n(map.lighting.begin() + lightdatasize,
                            (std::size_t)bytes, (byte)255);
                lightdatasize += bytes;
                continue;
            }
            // just need to zero the lightmap so that it does not contribute to
            // the lightdata size
            if (int_for_key(*state.face_entity[(size_t)facenum], "zhlt_striprad"))
            {
                f->lightofs = lightdatasize;
                for (int k = 0; k < maxlightmaps; k++)
                {
                    f->styles[k] = 255;
                }
                continue;
            }
            if (f->lightofs == -1)
            {
                continue;
            }

            int i, k;
            int oldofs;
            unsigned char oldstyles[maxlightmaps];
            oldofs = f->lightofs;
            f->lightofs = lightdatasize;
            for (k = 0; k < maxlightmaps; k++)
            {
                oldstyles[k] = f->styles[k];
                f->styles[k] = 255;
            }
            int numstyles = 0;
            for (k = 0; k < maxlightmaps && oldstyles[k] != 255; k++)
            {
                unsigned char maxb = 0;
                for (i = 0; i < fl->numsamples; i++)
                {
                    unsigned char *v = &oldlightdata[(size_t)(oldofs + fl->numsamples * 3 * k + i * 3)];
                    unsigned char m = v[1] > v[2] ? v[1] : v[2];
                    m = v[0] > m ? v[0] : m;
                    maxb = maxb > m ? maxb : m;
                }
                if (maxb <= 0) // black
                {
                    continue;
                }
                f->styles[numstyles] = oldstyles[k];
                err::require(lightdatasize + fl->numsamples * 3 * (numstyles + 1) <= state.max_map_lightdata,
                             "reduce_lightmap: exceeded MAX_MAP_LIGHTING");
                memcpy(&map.lighting[(size_t)(f->lightofs + fl->numsamples * 3 * numstyles)],
                       &oldlightdata[(size_t)(oldofs + fl->numsamples * 3 * k)], (size_t)(fl->numsamples * 3));
                numstyles++;
            }
            lightdatasize += fl->numsamples * 3 * numstyles;
        }
        map.lighting.resize((size_t)lightdatasize);
    }

    // ===== mdl light hack =====

    // changes the sample light right under a mdl entity's origin; used when a
    // model in shadow has incorrect brightness

    namespace
    {
        const int mlh_maxfacecount = 16;
        const int mlh_maxsamplecount = 4;

        struct mdllight
        {
            vec3v origin;
            vec3v floor;
            struct
            {
                int num;
                struct
                {
                    bool exist;
                    int seq;
                } style[allstyles];
                struct
                {
                    int num;
                    vec3v pos;
                    unsigned char *style[allstyles];
                } sample[mlh_maxsamplecount];
                int samplecount;
            } face[mlh_maxfacecount];
            int facecount;
        };

        int mlh_add_face(const rad_state &state, mdllight *ml, int facenum)
        {
            const format::dface_t *f = &state.map->faces[(size_t)facenum];
            int i, j;
            for (i = 0; i < ml->facecount; i++)
            {
                if (ml->face[i].num == facenum)
                {
                    return -1;
                }
            }
            if (ml->facecount >= mlh_maxfacecount)
            {
                return -1;
            }
            i = ml->facecount;
            ml->facecount++;
            ml->face[i].num = facenum;
            ml->face[i].samplecount = 0;
            for (j = 0; j < allstyles; j++)
            {
                ml->face[i].style[j].exist = false;
            }
            for (j = 0; j < maxlightmaps && f->styles[j] != 255; j++)
            {
                ml->face[i].style[f->styles[j]].exist = true;
                ml->face[i].style[f->styles[j]].seq = j;
            }
            return i;
        }

        void mlh_add_sample(rad_state &state, mdllight *ml, int facenum, int w, int h, int s, int t, const vec3v &pos)
        {
            const format::dface_t *f = &state.map->faces[(size_t)facenum];
            int i, j;
            int r = mlh_add_face(state, ml, facenum);
            if (r == -1)
            {
                return;
            }
            int size = w * h;
            int num = s + w * t;
            for (i = 0; i < ml->face[r].samplecount; i++)
            {
                if (ml->face[r].sample[i].num == num)
                {
                    return;
                }
            }
            if (ml->face[r].samplecount >= mlh_maxsamplecount)
            {
                return;
            }
            i = ml->face[r].samplecount;
            ml->face[r].samplecount++;
            ml->face[r].sample[i].num = num;
            math::copy(pos, ml->face[r].sample[i].pos);
            for (j = 0; j < allstyles; j++)
            {
                if (ml->face[r].style[j].exist)
                {
                    ml->face[r].sample[i].style[j] = &state.map->lighting[(size_t)(f->lightofs + (num + size * ml->face[r].style[j].seq) * 3)];
                }
            }
        }

        void mlh_calc_extents(const rad_state &state, const format::dface_t *f, int *texturemins, int *extents)
        {
            int bmins[2];
            int bmaxs[2];
            int i;

            format::get_face_extents(*state.map, (int)(f - state.map->faces.data()), bmins, bmaxs);
            for (i = 0; i < 2; i++)
            {
                texturemins[i] = bmins[i] * texture_step;
                extents[i] = (bmaxs[i] - bmins[i]) * texture_step;
            }
        }

        void mlh_get_samples_r(rad_state &state, mdllight *ml, int nodenum, const float *start, const float *end)
        {
            if (nodenum < 0)
                return;
            const format::dnode_t *node = &state.map->nodes[(size_t)nodenum];
            const plane *pl;
            float front, back, frac;
            float mid[3];
            int side;
            pl = &state.planes[(size_t)node->planenum];
            front = (start[0] * pl->normal[0] + start[1] * pl->normal[1] + start[2] * pl->normal[2]) - pl->dist;
            back = (end[0] * pl->normal[0] + end[1] * pl->normal[1] + end[2] * pl->normal[2]) - pl->dist;
            side = front < 0;
            if ((back < 0) == side)
            {
                mlh_get_samples_r(state, ml, node->children[side], start, end);
                return;
            }
            frac = front / (front - back);
            mid[0] = start[0] + (end[0] - start[0]) * frac;
            mid[1] = start[1] + (end[1] - start[1]) * frac;
            mid[2] = start[2] + (end[2] - start[2]) * frac;
            mlh_get_samples_r(state, ml, node->children[side], start, mid);
            if (ml->facecount > 0)
            {
                return;
            }
            {
                int i;
                for (i = 0; i < node->numfaces; i++)
                {
                    const format::dface_t *f = &state.map->faces[(size_t)(node->firstface + i)];
                    const format::texinfo_t *tex = &state.map->texinfo[(size_t)f->texinfo];
                    const char *texname = texture_by_number(state, f->texinfo);
                    if (!strncmp(texname, "sky", 3))
                    {
                        continue;
                    }
                    if (f->lightofs == -1)
                    {
                        continue;
                    }
                    int s = (int)((mid[0] * tex->vecs[0][0] + mid[1] * tex->vecs[0][1] + mid[2] * tex->vecs[0][2]) + tex->vecs[0][3]);
                    int t = (int)((mid[0] * tex->vecs[1][0] + mid[1] * tex->vecs[1][1] + mid[2] * tex->vecs[1][2]) + tex->vecs[1][3]);
                    int texturemins[2], extents[2];
                    mlh_calc_extents(state, f, texturemins, extents);
                    if (s < texturemins[0] || t < texturemins[1])
                    {
                        continue;
                    }
                    int ds = s - texturemins[0];
                    int dt = t - texturemins[1];
                    if (ds > extents[0] || dt > extents[1])
                    {
                        continue;
                    }
                    ds >>= 4;
                    dt >>= 4;
                    vec3v midv{mid[0], mid[1], mid[2]};
                    mlh_add_sample(state, ml, node->firstface + i, extents[0] / texture_step + 1, extents[1] / texture_step + 1, ds, dt, midv);
                    break;
                }
            }
            if (ml->facecount > 0)
            {
                math::copy(vec3v{mid[0], mid[1], mid[2]}, ml->floor);
                return;
            }
            mlh_get_samples_r(state, ml, node->children[!side], mid, end);
        }

        void mlh_mdllight_create(rad_state &state, mdllight *ml)
        {
            // from quake
            float p[3];
            float end[3];
            ml->facecount = 0;
            math::copy(ml->origin, ml->floor);
            p[0] = ml->origin[0];
            p[1] = ml->origin[1];
            p[2] = ml->origin[2];
            end[0] = ml->origin[0];
            end[1] = ml->origin[1];
            end[2] = ml->origin[2];
            end[2] -= 2048;
            mlh_get_samples_r(state, ml, 0, p, end);
        }

        int mlh_copy_light(rad_state &state, const vec3v &from, const vec3v &to)
        {
            int i, j, k, count = 0;
            mdllight mlfrom, mlto;
            math::copy(from, mlfrom.origin);
            math::copy(to, mlto.origin);
            mlh_mdllight_create(state, &mlfrom);
            mlh_mdllight_create(state, &mlto);
            if (mlfrom.facecount == 0 || mlfrom.face[0].samplecount == 0)
                return -1;
            for (i = 0; i < mlto.facecount; ++i)
                for (j = 0; j < mlto.face[i].samplecount; ++j, ++count)
                    for (k = 0; k < allstyles; ++k)
                        if (mlto.face[i].style[k].exist && mlfrom.face[0].style[k].exist)
                        {
                            unsigned char *src = mlfrom.face[0].sample[0].style[k];
                            unsigned char *dst = mlto.face[i].sample[j].style[k];
                            dst[0] = src[0];
                            dst[1] = src[1];
                            dst[2] = src[2];
                        }
            return count;
        }
    }

    void mdl_light_hack(rad_state &state)
    {
        format::entity *ent1, *ent2;
        vec3v origin1, origin2;
        const char *target;
        int used = 0, countent = 0, countsample = 0, r;
        for (size_t ient = 0; ient < state.entities.size(); ++ient)
        {
            ent1 = &state.entities[ient];
            target = ent1->value("zhlt_copylight");
            if (!strcmp(target, ""))
                continue;
            used = 1;
            ent2 = find_target_entity(state, target);
            if (ent2 == nullptr)
            {
                logging::warn("target entity '%s' not found", target);
                continue;
            }
            vector_for_key(*ent1, "origin", origin1);
            vector_for_key(*ent2, "origin", origin2);
            r = mlh_copy_light(state, origin2, origin1);
            if (r < 0)
                logging::warn("can not copy light from (%f,%f,%f)", origin2[0], origin2[1], origin2[2]);
            else
            {
                countent += 1;
                countsample += r;
            }
        }
        if (used)
            logging::info("Adjust mdl light: modified %d samples for %d entities\n", countsample, countent);
    }
}
