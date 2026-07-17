#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "../common/error.h"
#include "../common/log.h"
#include "../common/progress.h"
#include "../common/threads.h"
#include "compress.h"
#include "internal.h"

#ifdef HLTOOLS_GPU_ENABLED
#include <vector>
#include "gpu/gpu.h"
#endif

// builds each patch's compressed transfer list: which patches it collects
// light from and the form factor of each this is the primary time sink of the
// radiosity setup it includes scalar and rgb scale generation plus an
// incremental transfer cache

namespace rad
{
    int find_transfer_offset_patchnum(transfer_index *t_index, const patch *pt, const unsigned patchnum)
    {
        // binary search for match
        int low = 0;
        int high = (int)pt->i_index - 1;
        int offset;

        while (1)
        {
            offset = (low + high) / 2;

            if ((t_index[offset].index + t_index[offset].size) < patchnum)
            {
                low = offset + 1;
            }
            else if (t_index[offset].index > patchnum)
            {
                high = offset - 1;
            }
            else
            {
                unsigned x;
                unsigned int rval = 0;
                transfer_index *p_index = t_index;

                for (x = 0; x < (unsigned)offset; x++, p_index++)
                {
                    rval += p_index->size + 1;
                }
                rval += patchnum - t_index[offset].index;
                return (int)rval;
            }
            if (low > high)
            {
                return -1;
            }
        }
    }

    namespace
    {
        unsigned get_length_of_run(const transfer_raw_index *raw, const transfer_raw_index *const end)
        {
            unsigned run_size = 0;

            while (raw < end)
            {
                if (((*raw) + 1) == (*(raw + 1)))
                {
                    raw++;
                    run_size++;

                    if (run_size >= max_compressed_transfer_index_size)
                    {
                        return run_size;
                    }
                }
                else
                {
                    return run_size;
                }
            }
            return run_size;
        }

        transfer_index *compress_transfer_indicies(rad_state &state, transfer_raw_index *t_raw,
                                                   const unsigned raw_size, unsigned *i_size)
        {
            unsigned x;
            unsigned size = raw_size;
            unsigned compressed_count = 0;

            transfer_raw_index *raw = t_raw;
            // -1 since we compare current with next and would bump into the end
            transfer_raw_index *end = t_raw + raw_size - 1;

            unsigned compressed_count_1 = 0;

            for (x = 0; x < raw_size; x++)
            {
                x += get_length_of_run(t_raw + x, t_raw + raw_size - 1);
                compressed_count_1++;
            }

            if (!compressed_count_1)
            {
                return nullptr;
            }

            transfer_index *compressed_array = (transfer_index *)std::calloc(compressed_count_1, sizeof(transfer_index));
            err::require(compressed_array != nullptr, "compress_transfer_indicies: out of memory");
            transfer_index *compressed = compressed_array;

            for (x = 0; x < size; x++, raw++, compressed++)
            {
                compressed->index = (*raw);
                // zero based: a size of 0 still means 1 item in the list
                compressed->size = get_length_of_run(raw, end);
                raw += compressed->size;
                x += compressed->size;
                compressed_count++; // number of entries in compressed table
            }

            *i_size = compressed_count;

            if (compressed_count != compressed_count_1)
            {
                err::fatal("compress_transfer_indicies: internal error");
            }

            {
                std::lock_guard<std::mutex> guard(state.lock);
                state.transfer_index_bytes += sizeof(transfer_index) * compressed_count;
            }

            return compressed_array;
        }

        // per thread scratch reused across work items, standing in for the
        // buffers the reference allocated once per worker thread make_scales
        // runs once per compile, so the thread_local lifetime is safe
        struct scales_scratch
        {
            std::vector<transfer_raw_index> tindex_all;
            std::vector<float> tdata_all;
            unsigned int fastfind_index = 0;
        };

        thread_local scales_scratch tls_scratch;

        // builds the transfer list of patch i: which patch2's it collects
        // light from
        void make_scales_item(rad_state &state, const int i)
        {
            unsigned j;
            vec3v delta;
            vec_t dist;
            int count;
            float trans;
            patch *pt;
            patch *patch2;
            vec3v origin;

            scales_scratch &scratch = tls_scratch;
            scratch.tindex_all.resize(state.num_patches + 1);
            scratch.tdata_all.resize(state.num_patches + 1);

            count = 0;

            pt = &state.patches[(size_t)i];
            pt->i_index = 0;
            pt->i_data = 0;

            transfer_raw_index *t_index = scratch.tindex_all.data();
            float *t_data = scratch.tdata_all.data();

            math::copy(pt->origin, origin);
            const vec3v &normal1 = plane_from_face_number(state, (unsigned)pt->facenumber)->normal;

            vec3v backorigin;
            vec3v backnormal;
            if (pt->translucent_b)
            {
                math::multiply_add(pt->origin, -(state.options.translucentdepth + 2 * patch_hunt_offset), normal1, backorigin);
                for (int k = 0; k < 3; k++)
                    backnormal[k] = 0.0f - normal1[k];
            }
            bool lighting_diversify;
            vec_t lighting_power;
            vec_t lighting_scale;
            int miptex = state.map->texinfo[(size_t)state.map->faces[(size_t)pt->facenumber].texinfo].miptex;
            lighting_power = state.lightingconeinfo[(size_t)miptex][0];
            lighting_scale = state.lightingconeinfo[(size_t)miptex][1];
            lighting_diversify = (lighting_power != 1.0 || lighting_scale != 1.0);

            // find out which patch2's the patch collects light from
            for (j = 0, patch2 = state.patches.data(); j < state.num_patches; j++, patch2++)
            {
                vec_t dot1;
                vec_t dot2;

                vec3v transparency = {1.0, 1.0, 1.0};
                bool useback;
                useback = false;

                if (!state.check_vis_bit(state, (unsigned)i, j, transparency, scratch.fastfind_index) || ((unsigned)i == j))
                {
                    if (pt->translucent_b)
                    {
                        if (((unsigned)i == j) ||
                            !check_vis_bit_backwards(state, (unsigned)i, j, backorigin, backnormal, transparency))
                        {
                            continue;
                        }
                        useback = true;
                    }
                    else
                    {
                        continue;
                    }
                }

                const vec3v &normal2 = plane_from_face_number(state, (unsigned)patch2->facenumber)->normal;

                // calculate transference
                math::subtract(patch2->origin, origin, delta);
                if (useback)
                {
                    math::subtract(patch2->origin, backorigin, delta);
                }
                // move emitter back to its plane
                math::multiply_add(delta, -patch_hunt_offset, normal2, delta);

                dist = math::normalize(delta);
                dot1 = math::dot(delta, normal1);
                if (useback)
                {
                    dot1 = math::dot(delta, backnormal);
                }
                dot2 = -math::dot(delta, normal2);
                bool light_behind_surface = false;
                if (dot1 <= math::normal_epsilon)
                {
                    light_behind_surface = true;
                }
                if (dot2 * dist <= minimum_patch_distance)
                {
                    continue;
                }

                if (lighting_diversify && !light_behind_surface)
                {
                    dot1 = (vec_t)(lighting_scale * std::pow((double)dot1, lighting_power));
                }
                // inverse square falloff factoring angle between patch normals
                trans = (dot1 * dot2) / (dist * dist);
                if (trans * patch2->area > 0.8f)
                    trans = 0.8f / patch2->area;
                if (dist < patch2->emitter_range - math::on_epsilon)
                {
                    if (light_behind_surface)
                    {
                        trans = 0.0;
                    }
                    vec_t sightarea;
                    const vec3v *receiver_origin = &origin;
                    const vec3v *receiver_normal = &normal1;
                    const math::winding *emitter_winding;
                    if (useback)
                    {
                        receiver_origin = &backorigin;
                        receiver_normal = &backnormal;
                    }
                    emitter_winding = patch2->winding;
                    sightarea = calc_sight_area(state, *receiver_origin, *receiver_normal, emitter_winding,
                                                patch2->emitter_skylevel, lighting_power, lighting_scale);

                    vec_t frac;
                    frac = dist / patch2->emitter_range;
                    frac = (frac - 0.5f) * 2.0f; // smooth transition between the two methods
                    frac = frac < 1 ? frac : 1;
                    frac = 0 > frac ? 0 : frac;
                    trans = frac * trans + (1 - frac) * (sightarea / patch2->area); // because later we will multiply this back
                }
                else
                {
                    if (light_behind_surface)
                    {
                        continue;
                    }
                }

                trans *= patch2->exposure;
                trans = trans * ((transparency[0] + transparency[1] + transparency[2]) / 3); // add transparency effect
                if (pt->translucent_b)
                {
                    if (useback)
                    {
                        trans *= (pt->translucent_v[0] + pt->translucent_v[1] + pt->translucent_v[2]) / 3;
                    }
                    else
                    {
                        trans *= 1 - (pt->translucent_v[0] + pt->translucent_v[1] + pt->translucent_v[2]) / 3;
                    }
                }

                {
                    trans = trans * patch2->area;
                }
                if (trans <= 0.0)
                {
                    continue;
                }

                *t_data = trans;
                *t_index = j;
                t_data++;
                t_index++;
                pt->i_data++;
                count++;
            }

            // copy the transfers out
            if (pt->i_data)
            {
                unsigned data_size = pt->i_data * (unsigned)float_format_size[(int)state.options.transfer_compress_type] + (unsigned)transfer_unused_size;

                pt->t_data = (transfer_data *)std::calloc(data_size, 1);
                pt->t_index = compress_transfer_indicies(state, scratch.tindex_all.data(), pt->i_data, &pt->i_index);

                err::require(pt->t_data != nullptr, "make_scales: out of memory");
                err::require(pt->t_index != nullptr, "make_scales: out of memory");

                {
                    std::lock_guard<std::mutex> guard(state.lock);
                    state.transfer_data_bytes += data_size;
                }

                vec_t total = (vec_t)(1 / math::pi);
                {
                    unsigned x;
                    transfer_data *t1 = pt->t_data;
                    float *t2 = scratch.tdata_all.data();

                    float f;
                    for (x = 0; x < pt->i_data; x++, t1 += float_format_size[(int)state.options.transfer_compress_type], t2++)
                    {
                        f = (*t2) * total;
                        float_compress(state.options.transfer_compress_type, t1, &f);
                    }
                }
            }

            {
                std::lock_guard<std::mutex> guard(state.lock);
                state.total_transfer += (size_t)count;
            }
        }

        thread_local std::vector<float> tls_rgbdata_all;

        void make_rgb_scales_item(rad_state &state, const int i)
        {
            unsigned j;
            vec3v delta;
            vec_t dist;
            int count;
            float trans[3];
            float trans_one;
            patch *pt;
            patch *patch2;
            vec3v origin;

            scales_scratch &scratch = tls_scratch;
            scratch.tindex_all.resize(state.num_patches + 1);
            tls_rgbdata_all.resize(3 * (state.num_patches + 1));

            count = 0;

            pt = &state.patches[(size_t)i];
            pt->i_index = 0;
            pt->i_data = 0;

            transfer_raw_index *t_index = scratch.tindex_all.data();
            float *t_rgbdata = tls_rgbdata_all.data();

            math::copy(pt->origin, origin);
            const vec3v &normal1 = plane_from_face_number(state, (unsigned)pt->facenumber)->normal;

            vec3v backorigin;
            vec3v backnormal;
            if (pt->translucent_b)
            {
                math::multiply_add(pt->origin, -(state.options.translucentdepth + 2 * patch_hunt_offset), normal1, backorigin);
                for (int k = 0; k < 3; k++)
                    backnormal[k] = 0.0f - normal1[k];
            }
            bool lighting_diversify;
            vec_t lighting_power;
            vec_t lighting_scale;
            int miptex = state.map->texinfo[(size_t)state.map->faces[(size_t)pt->facenumber].texinfo].miptex;
            lighting_power = state.lightingconeinfo[(size_t)miptex][0];
            lighting_scale = state.lightingconeinfo[(size_t)miptex][1];
            lighting_diversify = (lighting_power != 1.0 || lighting_scale != 1.0);

            // find out which patch2's the patch collects light from
            for (j = 0, patch2 = state.patches.data(); j < state.num_patches; j++, patch2++)
            {
                vec_t dot1;
                vec_t dot2;
                vec3v transparency = {1.0, 1.0, 1.0};
                bool useback;
                useback = false;

                if (!state.check_vis_bit(state, (unsigned)i, j, transparency, scratch.fastfind_index) || ((unsigned)i == j))
                {
                    if (pt->translucent_b)
                    {
                        if (!check_vis_bit_backwards(state, (unsigned)i, j, backorigin, backnormal, transparency) || ((unsigned)i == j))
                        {
                            continue;
                        }
                        useback = true;
                    }
                    else
                    {
                        continue;
                    }
                }

                const vec3v &normal2 = plane_from_face_number(state, (unsigned)patch2->facenumber)->normal;

                // calculate transference
                math::subtract(patch2->origin, origin, delta);
                if (useback)
                {
                    math::subtract(patch2->origin, backorigin, delta);
                }
                // move emitter back to its plane
                math::multiply_add(delta, -patch_hunt_offset, normal2, delta);

                dist = math::normalize(delta);
                dot1 = math::dot(delta, normal1);
                if (useback)
                {
                    dot1 = math::dot(delta, backnormal);
                }
                dot2 = -math::dot(delta, normal2);
                bool light_behind_surface = false;
                if (dot1 <= math::normal_epsilon)
                {
                    light_behind_surface = true;
                }
                if (dot2 * dist <= minimum_patch_distance)
                {
                    continue;
                }

                if (lighting_diversify && !light_behind_surface)
                {
                    dot1 = (vec_t)(lighting_scale * std::pow((double)dot1, lighting_power));
                }
                // inverse square falloff factoring angle between patch normals
                trans_one = (dot1 * dot2) / (dist * dist);

                if (trans_one * patch2->area > 0.8f)
                {
                    trans_one = 0.8f / patch2->area;
                }
                if (dist < patch2->emitter_range - math::on_epsilon)
                {
                    if (light_behind_surface)
                    {
                        trans_one = 0.0;
                    }
                    vec_t sightarea;
                    const vec3v *receiver_origin = &origin;
                    const vec3v *receiver_normal = &normal1;
                    const math::winding *emitter_winding;
                    if (useback)
                    {
                        receiver_origin = &backorigin;
                        receiver_normal = &backnormal;
                    }
                    emitter_winding = patch2->winding;
                    sightarea = calc_sight_area(state, *receiver_origin, *receiver_normal, emitter_winding,
                                                patch2->emitter_skylevel, lighting_power, lighting_scale);

                    vec_t frac;
                    frac = dist / patch2->emitter_range;
                    frac = (frac - 0.5f) * 2.0f; // smooth transition between the two methods
                    frac = frac < 1 ? frac : 1;
                    frac = 0 > frac ? 0 : frac;
                    trans_one = frac * trans_one + (1 - frac) * (sightarea / patch2->area); // because later we will multiply this back
                }
                else
                {
                    if (light_behind_surface)
                    {
                        continue;
                    }
                }
                trans_one *= patch2->exposure;
                trans[0] = trans_one;
                trans[1] = trans_one;
                trans[2] = trans_one;
                for (int x = 0; x < 3; x++)
                {
                    trans[x] = trans[x] * transparency[x]; // add transparency effect
                }
                if (pt->translucent_b)
                {
                    if (useback)
                    {
                        for (int x = 0; x < 3; x++)
                        {
                            trans[x] = pt->translucent_v[x] * trans[x];
                        }
                    }
                    else
                    {
                        for (int x = 0; x < 3; x++)
                        {
                            trans[x] = (1 - pt->translucent_v[x]) * trans[x];
                        }
                    }
                }

                if (trans_one <= 0.0)
                {
                    continue;
                }
                {
                    for (int x = 0; x < 3; x++)
                    {
                        trans[x] = (float)(trans[x] * patch2->area);
                    }
                }

                t_rgbdata[0] = trans[0];
                t_rgbdata[1] = trans[1];
                t_rgbdata[2] = trans[2];
                *t_index = j;
                t_rgbdata += 3;
                t_index++;
                pt->i_data++;
                count++;
            }

            // copy the transfers out
            if (pt->i_data)
            {
                unsigned data_size = pt->i_data * (unsigned)vector_format_size[(int)state.options.rgbtransfer_compress_type] + (unsigned)transfer_unused_size;

                pt->t_rgb_data = (rgb_transfer_data *)std::calloc(data_size, 1);
                pt->t_index = compress_transfer_indicies(state, scratch.tindex_all.data(), pt->i_data, &pt->i_index);

                err::require(pt->t_rgb_data != nullptr, "make_rgb_scales: out of memory");
                err::require(pt->t_index != nullptr, "make_rgb_scales: out of memory");

                {
                    std::lock_guard<std::mutex> guard(state.lock);
                    state.transfer_data_bytes += data_size;
                }

                vec_t total = (vec_t)(1 / math::pi);
                {
                    unsigned x;
                    rgb_transfer_data *t1 = pt->t_rgb_data;
                    float *t2 = tls_rgbdata_all.data();

                    float f[3];
                    for (x = 0; x < pt->i_data; x++, t1 += vector_format_size[(int)state.options.rgbtransfer_compress_type], t2 += 3)
                    {
                        f[0] = (float)(t2[0] * total);
                        f[1] = (float)(t2[1] * total);
                        f[2] = (float)(t2[2] * total);
                        vector_compress(state.options.rgbtransfer_compress_type, t1, &f[0], &f[1], &f[2]);
                    }
                }
            }

            {
                std::lock_guard<std::mutex> guard(state.lock);
                state.total_transfer += (size_t)count;
            }
        }
    }

#ifdef HLTOOLS_GPU_ENABLED
    namespace
    {
        // cpu recompute for pairs the kernel bounced back (nan: emitter
        // winding larger than its fixed size edge array); mirrors the same
        // make_scales_item math the kernel runs
        float gpu_transfer_fallback(rad_state &state, const patch *pt, const patch *patch2,
                                    const vec3v &normal1, vec_t lighting_power,
                                    vec_t lighting_scale, bool lighting_diversify)
        {
            vec3v delta;
            const vec3v &normal2 = plane_from_face_number(state, (unsigned)patch2->facenumber)->normal;
            math::subtract(patch2->origin, pt->origin, delta);
            math::multiply_add(delta, -patch_hunt_offset, normal2, delta);
            vec_t dist = math::normalize(delta);
            vec_t dot1 = math::dot(delta, normal1);
            vec_t dot2 = -math::dot(delta, normal2);
            bool light_behind_surface = (dot1 <= math::normal_epsilon);
            if (dot2 * dist <= minimum_patch_distance)
                return 0.0f;
            if (lighting_diversify && !light_behind_surface)
                dot1 = (vec_t)(lighting_scale * std::pow((double)dot1, lighting_power));
            float trans = (dot1 * dot2) / (dist * dist);
            if (trans * patch2->area > 0.8f)
                trans = 0.8f / patch2->area;
            if (dist < patch2->emitter_range - math::on_epsilon)
            {
                if (light_behind_surface)
                    trans = 0.0;
                vec_t sightarea = calc_sight_area(state, pt->origin, normal1, patch2->winding,
                                                  patch2->emitter_skylevel, lighting_power,
                                                  lighting_scale);
                vec_t frac = dist / patch2->emitter_range;
                frac = (vec_t)((frac - 0.5) * 2);
                frac = frac < 1 ? frac : 1;
                frac = 0 > frac ? 0 : frac;
                trans = frac * trans + (1 - frac) * (sightarea / patch2->area);
            }
            else if (light_behind_surface)
            {
                return 0.0f;
            }
            trans *= patch2->exposure;
            return trans * patch2->area;
        }

        bool gpu_make_scales(rad_state &state)
        {
            if (state.options.method != vis_method::sparse_vismatrix)
                return false; // pair enumeration below reads the sparse columns
            if (state.options.customshadow_with_bouncelight)
                return false; // per pair transparency is a cpu path
            for (unsigned x = 0; x < state.num_patches; x++)
            {
                if (state.patches[x].translucent_b)
                    return false; // backside transfers are a cpu path
            }
            if (!gpu::available())
            {
                logging::warn("-gpu: %s; computing light transfers on the cpu",
                              gpu::last_error().c_str());
                return false;
            }

            // per map scene: patch soa (+ per patch receiver cone info),
            // flattened windings, and every sky normal level
            gpu::formfactor_scene scene;
            scene.patches.resize(state.num_patches);
            for (unsigned x = 0; x < state.num_patches; x++)
            {
                const patch &pt = state.patches[x];
                gpu::patch_gpu &out = scene.patches[x];
                const vec3v &normal = plane_from_face_number(state, (unsigned)pt.facenumber)->normal;
                int miptex = state.map->texinfo[(size_t)state.map->faces[(size_t)pt.facenumber].texinfo].miptex;
                for (int k = 0; k < 3; k++)
                {
                    out.origin[k] = pt.origin[k];
                    out.normal[k] = normal[k];
                }
                out.area = pt.area;
                out.emitter_range = pt.emitter_range;
                out.exposure = pt.exposure;
                out.cone_power = state.lightingconeinfo[(size_t)miptex][0];
                out.cone_scale = state.lightingconeinfo[(size_t)miptex][1];
                out.skylevel = pt.emitter_skylevel;
                out.wind_ofs = (int)(scene.windings.size() / 3);
                out.wind_count = pt.winding ? (int)pt.winding->size() : 0;
                if (pt.winding)
                {
                    for (int v = 0; v < (int)pt.winding->size(); v++)
                    {
                        scene.windings.push_back((*pt.winding)[v][0]);
                        scene.windings.push_back((*pt.winding)[v][1]);
                        scene.windings.push_back((*pt.winding)[v][2]);
                    }
                }
            }
            for (int level = 0; level <= skylevel_max; level++)
            {
                scene.sky_levels.push_back((int32_t)(scene.sky_normals.size() / 4));
                scene.sky_levels.push_back(state.numskynormals[level]);
                for (int j = 0; j < state.numskynormals[level]; j++)
                {
                    scene.sky_normals.push_back(state.skynormals[level][(size_t)j][0]);
                    scene.sky_normals.push_back(state.skynormals[level][(size_t)j][1]);
                    scene.sky_normals.push_back(state.skynormals[level][(size_t)j][2]);
                    scene.sky_normals.push_back(state.skynormalsizes[level][(size_t)j]);
                }
            }

            progress::section("lighting");
            progress::begin("computing light transfers", (int)state.num_patches);

            // enumerate visible pairs straight from the sparse columns
            // (o(visible), not the o(patches²) binary searches the cpu loop
            // does): column x holds partners y > x, so partners below i come
            // from a one shot transposed pass outer x ascends, keeping every
            // list in the ascending order the cpu loop walks
            std::vector<std::vector<unsigned>> below(state.num_patches);
            for (unsigned x = 0; x < state.num_patches; x++)
            {
                const sparse_column &column = state.sparse_columns[x];
                for (int r = 0; r < column.count; r++)
                {
                    const unsigned bits = column.row[r].values;
                    const unsigned ybase = column.row[r].offset * 8;
                    for (unsigned b = 0; b < 8; b++)
                    {
                        if (bits & (1u << b))
                            below[ybase + b].push_back(x);
                    }
                }
            }

            constexpr unsigned chunk_patches = 4096;
            std::vector<std::vector<unsigned>> vis(chunk_patches);

            for (unsigned base = 0; base < state.num_patches; base += chunk_patches)
            {
                const unsigned count = state.num_patches - base < chunk_patches
                    ? state.num_patches - base : chunk_patches;

                // pass a (cpu, threaded): the visible pair lists, in the same
                // ascending order the cpu loop walks
                threads::run((int)count, [&](int k)
                             {
                                 vis[(size_t)k].clear();
                                 const unsigned i = base + (unsigned)k;
                                 vis[(size_t)k] = below[i];
                                 const sparse_column &column = state.sparse_columns[i];
                                 for (int r = 0; r < column.count; r++)
                                 {
                                     const unsigned bits = column.row[r].values;
                                     const unsigned ybase = column.row[r].offset * 8;
                                     for (unsigned b = 0; b < 8; b++)
                                     {
                                         if (bits & (1u << b))
                                             vis[(size_t)k].push_back(ybase + b);
                                     }
                                 }
                             });

                // pass b (gpu): one transfer factor per pair
                std::vector<gpu::transfer_pair> pairs;
                std::vector<size_t> first((size_t)count + 1, 0);
                for (unsigned k = 0; k < count; k++)
                {
                    first[k] = pairs.size();
                    for (unsigned j : vis[k])
                        pairs.push_back({(int32_t)(base + k), (int32_t)j});
                }
                first[count] = pairs.size();
                std::vector<float> trans;
                if (!pairs.empty() && !gpu::formfactor_batch(scene, pairs, trans))
                {
                    progress::end();
                    logging::warn("-gpu: %s; computing light transfers on the cpu",
                                  gpu::last_error().c_str());
                    return false;
                }

                // pass c (cpu, threaded): filter and compress, mirroring the
                // tail of make_scales_item
                threads::run((int)count, [&](int k)
                             {
                                 const unsigned i = base + (unsigned)k;
                                 patch *pt = &state.patches[(size_t)i];
                                 pt->i_index = 0;
                                 pt->i_data = 0;

                                 scales_scratch &scratch = tls_scratch;
                                 scratch.tindex_all.resize(state.num_patches + 1);
                                 scratch.tdata_all.resize(state.num_patches + 1);
                                 transfer_raw_index *t_index = scratch.tindex_all.data();
                                 float *t_data = scratch.tdata_all.data();
                                 int local_count = 0;

                                 const vec3v &normal1 =
                                     plane_from_face_number(state, (unsigned)pt->facenumber)->normal;
                                 int miptex = state.map->texinfo[(size_t)state.map->faces[(size_t)pt->facenumber].texinfo].miptex;
                                 vec_t power = state.lightingconeinfo[(size_t)miptex][0];
                                 vec_t scale = state.lightingconeinfo[(size_t)miptex][1];
                                 bool diversify = (power != 1.0 || scale != 1.0);

                                 for (size_t s = first[(size_t)k]; s < first[(size_t)k + 1]; s++)
                                 {
                                     float t = trans[s];
                                     unsigned j = vis[(size_t)k][s - first[(size_t)k]];
                                     if (std::isnan(t))
                                     {
                                         t = gpu_transfer_fallback(state, pt, &state.patches[(size_t)j],
                                                                   normal1, power, scale, diversify);
                                     }
                                     if (t <= 0.0f)
                                         continue;
                                     *t_data = t;
                                     *t_index = j;
                                     t_data++;
                                     t_index++;
                                     pt->i_data++;
                                     local_count++;
                                 }

                                 if (pt->i_data)
                                 {
                                     unsigned data_size = pt->i_data
                                         * (unsigned)float_format_size[(int)state.options.transfer_compress_type]
                                         + (unsigned)transfer_unused_size;
                                     pt->t_data = (transfer_data *)std::calloc(data_size, 1);
                                     pt->t_index = compress_transfer_indicies(
                                         state, scratch.tindex_all.data(), pt->i_data, &pt->i_index);
                                     err::require(pt->t_data != nullptr, "gpu_make_scales: out of memory");
                                     err::require(pt->t_index != nullptr, "gpu_make_scales: out of memory");
                                     {
                                         std::lock_guard<std::mutex> guard(state.lock);
                                         state.transfer_data_bytes += data_size;
                                     }
                                     vec_t total = (vec_t)(1 / math::pi);
                                     transfer_data *t1 = pt->t_data;
                                     float *t2 = scratch.tdata_all.data();
                                     for (unsigned x = 0; x < pt->i_data;
                                          x++, t1 += float_format_size[(int)state.options.transfer_compress_type], t2++)
                                     {
                                         float f = (*t2) * total;
                                         float_compress(state.options.transfer_compress_type, t1, &f);
                                     }
                                 }
                                 {
                                     std::lock_guard<std::mutex> guard(state.lock);
                                     state.total_transfer += (size_t)local_count;
                                 }
                                 progress::add(1);
                             });
            }

            progress::end();
            return true;
        }
    }
#endif

    void run_transfer_scales(rad_state &state)
    {
        if (state.options.rgb_transfers)
        {
            threads::run_phase("lighting", "computing light transfers", (int)state.num_patches,
                               [&](int i) { make_rgb_scales_item(state, i); });
        }
        else
        {
#ifdef HLTOOLS_GPU_ENABLED
            if (state.options.gpu && gpu_make_scales(state))
            {
                return;
            }
#endif
            threads::run_phase("lighting", "computing light transfers", (int)state.num_patches,
                               [&](int i) { make_scales_item(state, i); });
        }
    }

    // more human readable numbers
    void dump_transfers_memory_usage(const rad_state &state)
    {
        logging::file("      %-18s %.2fM  (indices %.2f MB, data %.2f MB)\n", "transfers",
                      (double)state.total_transfer / (1000.0 * 1000.0),
                      (double)state.transfer_index_bytes / (1024.0 * 1024.0),
                      (double)state.transfer_data_bytes / (1024.0 * 1024.0));
    }

    void write_transfers(rad_state &state, const char *transferfile, const long total_patches)
    {
        FILE *file;

        file = std::fopen(transferfile, "w+b");
        if (file != nullptr)
        {
            unsigned amtwritten;
            patch *pt;

            logging::info("Writing transfers file [%s]\n", transferfile);

            amtwritten = (unsigned)std::fwrite(&total_patches, sizeof(total_patches), 1, file);
            if (amtwritten != 1)
            {
                goto failed_write;
            }

            {
                long patchcount = total_patches;
                for (pt = state.patches.data(); patchcount-- > 0; pt++)
                {
                    amtwritten = (unsigned)std::fwrite(&pt->i_index, sizeof(pt->i_index), 1, file);
                    if (amtwritten != 1)
                    {
                        goto failed_write;
                    }

                    if (pt->i_index)
                    {
                        amtwritten = (unsigned)std::fwrite(pt->t_index, sizeof(transfer_index), pt->i_index, file);
                        if (amtwritten != pt->i_index)
                        {
                            goto failed_write;
                        }
                    }

                    amtwritten = (unsigned)std::fwrite(&pt->i_data, sizeof(pt->i_data), 1, file);
                    if (amtwritten != 1)
                    {
                        goto failed_write;
                    }
                    if (pt->i_data)
                    {
                        if (state.options.rgb_transfers)
                        {
                            amtwritten = (unsigned)std::fwrite(pt->t_rgb_data, vector_format_size[(int)state.options.rgbtransfer_compress_type], pt->i_data, file);
                        }
                        else
                        {
                            amtwritten = (unsigned)std::fwrite(pt->t_data, float_format_size[(int)state.options.transfer_compress_type], pt->i_data, file);
                        }
                        if (amtwritten != pt->i_data)
                        {
                            goto failed_write;
                        }
                    }
                }
            }

            std::fclose(file);
        }
        else
        {
            err::fatal("Failed to open incremenetal file [%s] for writing\n", transferfile);
        }
        return;

      failed_write:
        std::fclose(file);
        std::remove(transferfile);
        logging::warn("Failed to generate incremental file [%s] (probably ran out of disk space)\n", transferfile);
    }

    bool read_transfers(rad_state &state, const char *transferfile, const long numpatches)
    {
        FILE *file;
        long total_patches;

        file = std::fopen(transferfile, "rb");
        if (file != nullptr)
        {
            unsigned amtread;
            patch *pt;

            logging::info("Reading transfers file [%s]\n", transferfile);

            amtread = (unsigned)std::fread(&total_patches, sizeof(total_patches), 1, file);
            if (amtread != 1)
            {
                goto failed_read;
            }
            if (total_patches != numpatches)
            {
                goto failed_read;
            }

            {
                long patchcount = total_patches;
                for (pt = state.patches.data(); patchcount-- > 0; pt++)
                {
                    amtread = (unsigned)std::fread(&pt->i_index, sizeof(pt->i_index), 1, file);
                    if (amtread != 1)
                    {
                        goto failed_read;
                    }
                    if (pt->i_index)
                    {
                        pt->t_index = (transfer_index *)std::calloc(pt->i_index, sizeof(transfer_index));
                        err::require(pt->t_index != nullptr, "read_transfers: out of memory");
                        amtread = (unsigned)std::fread(pt->t_index, sizeof(transfer_index), pt->i_index, file);
                        if (amtread != pt->i_index)
                        {
                            goto failed_read;
                        }
                    }

                    amtread = (unsigned)std::fread(&pt->i_data, sizeof(pt->i_data), 1, file);
                    if (amtread != 1)
                    {
                        goto failed_read;
                    }
                    if (pt->i_data)
                    {
                        if (state.options.rgb_transfers)
                        {
                            pt->t_rgb_data = (rgb_transfer_data *)std::calloc(pt->i_data * vector_format_size[(int)state.options.rgbtransfer_compress_type] + transfer_unused_size, 1);
                            err::require(pt->t_rgb_data != nullptr, "read_transfers: out of memory");
                            amtread = (unsigned)std::fread(pt->t_rgb_data, vector_format_size[(int)state.options.rgbtransfer_compress_type], pt->i_data, file);
                        }
                        else
                        {
                            pt->t_data = (transfer_data *)std::calloc(pt->i_data * float_format_size[(int)state.options.transfer_compress_type] + transfer_unused_size, 1);
                            err::require(pt->t_data != nullptr, "read_transfers: out of memory");
                            amtread = (unsigned)std::fread(pt->t_data, float_format_size[(int)state.options.transfer_compress_type], pt->i_data, file);
                        }
                        if (amtread != pt->i_data)
                        {
                            goto failed_read;
                        }
                    }
                }
            }

            std::fclose(file);
            logging::warn("Finished reading transfers file [%s]\n", transferfile);
            return true;
        }
        logging::warn("Failed to open transfers file [%s]\n", transferfile);
        return false;

      failed_read:
        {
            unsigned x;
            patch *pt = state.patches.data();

            for (x = 0; x < state.num_patches; x++, pt++)
            {
                std::free(pt->t_data);
                std::free(pt->t_index);
                pt->i_data = 0;
                pt->i_index = 0;
                pt->t_data = nullptr;
                pt->t_index = nullptr;
            }
        }
        std::fclose(file);
        std::remove(transferfile);
        return false;
    }
}
