// compares the cpu and gpu gather implementations through the real rad setup
// it sends the same work items through both paths and compares each light style
//
// maps with opaque entities or studio models are skipped because production
// lighting routes those maps through the cpu path
//
// usage: rad_gpu_gather_check <mapbsp> [num_positions]

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <random>
#include <string>
#include <vector>

#include "format/bsp/file.h"
#include "../src/rad/internal.h"
#include "../src/rad/gpu/gpu.h"

using rad::vec3v;

namespace
{
    struct check_ctx
    {
        size_t num_positions = 4000;
        int exit_code = 2;
    };

    // mirrors texlightgap_setup in gather_sample_light (lightmapcpp:2454)
    void face_gap_vectors(const rad::rad_state &state, int facenum, float *out6)
    {
        const format::dface_t *f = &state.map->faces[(size_t)facenum];
        const rad::plane *dp = rad::plane_from_face_number(state, (unsigned)facenum);
        const format::texinfo_t *tex = &state.map->texinfo[(size_t)f->texinfo];
        for (int x = 0; x < 2; x++)
        {
            vec3v tv{tex->vecs[1 - x][0], tex->vecs[1 - x][1], tex->vecs[1 - x][2]};
            vec3v world;
            math::cross(tv, dp->normal, world);
            vec3v sv{tex->vecs[x][0], tex->vecs[x][1], tex->vecs[x][2]};
            vec_t len = math::dot(world, sv);
            if (std::fabs(len) < math::normal_epsilon)
                math::clear(world);
            else
                math::scale(world, 1 / len, world);
            out6[x * 3 + 0] = world[0];
            out6[x * 3 + 1] = world[1];
            out6[x * 3 + 2] = world[2];
        }
    }

    // the cpu resolver for near pairs: replicates the emit_type::surface case
    // of gather_sample_light (lightmapcpp lines 2684 to 2862) for one light with the
    // near branch taken only valid when there are no opaque entities or
    // studio models (guarded at startup)
    bool resolve_near_pair(rad::rad_state &state, const vec3v &pos, const vec3v &normal,
                           const rad::directlight *l, float cone_power, float cone_scale,
                           const float *gap6, vec3v &add_out)
    {
        bool diversify = (cone_power != 1.0f || cone_scale != 1.0f);

        vec3v testline_origin;
        math::copy(l->origin, testline_origin);
        vec3v delta;
        math::subtract(l->origin, pos, delta);
        math::multiply_add(delta, -rad::patch_hunt_offset, l->normal, delta);
        float dist = math::normalize(delta);
        float dot = math::dot(delta, normal);
        if (dist < 1.0f)
            dist = 1.0f;

        bool light_behind_surface = false;
        if (dot <= math::normal_epsilon)
            light_behind_surface = true;
        if (diversify && !light_behind_surface)
            dot = (vec_t)(cone_scale * std::pow((double)dot, cone_power));
        float dot2 = -math::dot(delta, l->normal);
        if (l->texlightgap > 0)
        {
            vec_t test = dot2 * dist;
            vec3v g0{gap6[0], gap6[1], gap6[2]};
            vec3v g1{gap6[3], gap6[4], gap6[5]};
            test -= l->texlightgap * std::fabs(math::dot(l->normal, g0));
            test -= l->texlightgap * std::fabs(math::dot(l->normal, g1));
            if (test < -math::on_epsilon)
                return false;
        }
        if (dot2 * dist <= rad::minimum_patch_distance)
            return false;

        vec_t range = l->patch_emitter_range;
        float ratio;
        if (l->stopdot > 0.0)
        {
            vec_t range_scale = 1 - l->stopdot2 * l->stopdot2;
            range_scale = (vec_t)(1 / std::sqrt(math::normal_epsilon > range_scale
                                                    ? math::normal_epsilon
                                                    : (double)range_scale));
            range_scale = range_scale < 2 ? range_scale : 2;
            range *= range_scale;
            if (dot2 <= l->stopdot2 + math::normal_epsilon)
            {
                if (dist >= range)
                    return false;
                ratio = 0.0;
            }
            else if (dot2 <= l->stopdot)
            {
                ratio = dot * dot2 * (dot2 - l->stopdot2) / (dist * dist * (l->stopdot - l->stopdot2));
            }
            else
            {
                ratio = dot * dot2 / (dist * dist);
            }
        }
        else
        {
            ratio = dot * dot2 / (dist * dist);
        }
        if (ratio * l->patch_area > 0.4f)
            ratio = 0.4f / l->patch_area;

        if (!(dist < range - math::on_epsilon))
            return false; // not actually near; the gpu should not have sent it

        if (light_behind_surface)
        {
            dot = 0.0;
            ratio = 0.0;
        }
        rad::get_alternate_origin(state, pos, normal, l->source_patch, testline_origin);
        vec_t sightarea;
        int skylevel = l->source_patch->emitter_skylevel;
        if (l->stopdot > 0.0)
        {
            const vec3v &emitnormal =
                rad::plane_from_face_number(state, (unsigned)l->source_patch->facenumber)->normal;
            if (l->stopdot2 >= 0.8)
                skylevel += 1;
            sightarea = rad::calc_sight_area_spotlight(state, pos, normal, l->source_patch->winding,
                                                       emitnormal, l->stopdot, l->stopdot2, skylevel,
                                                       cone_power, cone_scale);
        }
        else
        {
            sightarea = rad::calc_sight_area(state, pos, normal, l->source_patch->winding, skylevel,
                                             cone_power, cone_scale);
        }
        vec_t frac = dist / range;
        frac = (vec_t)((frac - 0.5) * 2);
        frac = frac < 1 ? frac : 1;
        frac = 0 > frac ? 0 : frac;
        vec_t ratio2 = (sightarea / l->patch_area);
        ratio = frac * ratio + (1 - frac) * ratio2;

        if (rad::test_line(state, pos, testline_origin) != rad::contents_empty)
            return false;
        math::scale(l->intensity, ratio, add_out);
        return true;
    }

    void hook(rad::rad_state &state, void *ctx_raw)
    {
        check_ctx &ctx = *(check_ctx *)ctx_raw;
        format::map_data &map = *state.map;

        if (!state.opaque_list.empty() || state.num_studio_models != 0)
        {
            std::printf("SKIP: map has opaque entities or studio models\n");
            ctx.exit_code = 2;
            return;
        }
        if (!rad::gpu::available())
        {
            std::printf("FAIL: no gpu device: %s\n", rad::gpu::last_error().c_str());
            ctx.exit_code = 2;
            return;
        }
        std::printf("gpu: %s\n", rad::gpu::device_name().c_str());
        std::printf("lights %zu in %zu lit leaves\n", state.lightarray.size(), state.lightleafs.size());

        // ---- scene ---------------------------------------------------------
        rad::gpu::gather_scene scene;
        scene.tnodes.resize(state.tnodes.size());
        for (size_t i = 0; i < state.tnodes.size(); i++)
        {
            const rad::tnode &tn = state.tnodes[i];
            rad::gpu::tnode_gpu &out = scene.tnodes[i];
            out.normal[0] = tn.normal[0];
            out.normal[1] = tn.normal[1];
            out.normal[2] = tn.normal[2];
            out.dist = tn.dist;
            out.type = tn.type;
            out.children[0] = tn.children[0];
            out.children[1] = tn.children[1];
        }
        scene.lightleafs.resize(state.lightleafs.size());
        for (size_t i = 0; i < state.lightleafs.size(); i++)
        {
            scene.lightleafs[i].leafnum = state.lightleafs[i].leafnum;
            scene.lightleafs[i].firstlight = state.lightleafs[i].firstlight;
            scene.lightleafs[i].numlights = state.lightleafs[i].numlights;
        }

        // compact style mapping (first appearance order over the light array)
        int style_to_compact[rad::allstyles];
        std::vector<int> compact_to_style;
        for (int s = 0; s < rad::allstyles; s++)
            style_to_compact[s] = -1;
        scene.lights.resize(state.lightarray.size());
        for (size_t i = 0; i < state.lightarray.size(); i++)
        {
            const rad::directlight &l = state.lightarray[i];
            rad::gpu::light_gpu &out = scene.lights[i];
            if (l.style < 0 || l.style >= rad::allstyles)
            {
                std::printf("FAIL: light style %d out of range\n", l.style);
                ctx.exit_code = 2;
                return;
            }
            if (style_to_compact[l.style] < 0)
            {
                if ((int)compact_to_style.size() >= rad::gpu::max_compact_styles)
                {
                    std::printf("FAIL: more than %d distinct light styles\n",
                                rad::gpu::max_compact_styles);
                    ctx.exit_code = 2;
                    return;
                }
                style_to_compact[l.style] = (int)compact_to_style.size();
                compact_to_style.push_back(l.style);
            }
            out.type = (int)l.type;
            out.compact_style = style_to_compact[l.style];
            out.topatch = l.topatch ? 1 : 0;
            out.sun_ofs = (int)(scene.sun_normals.size() / 4);
            out.sun_count = 0;
            if (l.type == rad::emit_type::skylight && l.sunnormals && l.sunnormalweights)
            {
                out.sun_count = l.numsunnormals;
                for (int j = 0; j < l.numsunnormals; j++)
                {
                    scene.sun_normals.push_back(l.sunnormals[j][0]);
                    scene.sun_normals.push_back(l.sunnormals[j][1]);
                    scene.sun_normals.push_back(l.sunnormals[j][2]);
                    scene.sun_normals.push_back(l.sunnormalweights[j]);
                }
            }
            for (int x = 0; x < 3; x++)
            {
                out.origin[x] = l.origin[x];
                out.intensity[x] = l.intensity[x];
                out.normal[x] = l.normal[x];
                out.diffuse[x] = l.diffuse_intensity[x];
                out.diffuse2[x] = l.diffuse_intensity2[x];
            }
            out.stopdot = l.stopdot;
            out.stopdot2 = l.stopdot2;
            out.fade = l.fade;
            out.texlightgap = l.texlightgap;
            out.patch_area = l.patch_area;
            out.patch_emitter_range = l.patch_emitter_range;
        }

        const int skylevel = state.options.softsky ? rad::skylevel_softsky_on
                                                   : rad::skylevel_softsky_off;
        for (int j = 0; j < state.numskynormals[skylevel]; j++)
        {
            scene.sky_normals.push_back(state.skynormals[skylevel][(size_t)j][0]);
            scene.sky_normals.push_back(state.skynormals[skylevel][(size_t)j][1]);
            scene.sky_normals.push_back(state.skynormals[skylevel][(size_t)j][2]);
            scene.sky_normals.push_back(state.skynormalsizes[skylevel][(size_t)j]);
        }
        scene.sky_lighting_fix = state.options.sky_lighting_fix ? 1 : 0;
        scene.sky_step_match = (state.options.softsky || state.options.fastmode) ? 1 : 0;
        scene.indirect_sun = state.options.indirect_sun;

        scene.face_gap.assign(map.faces.size() * 6, 0.0f);
        for (size_t f = 0; f < map.faces.size(); f++)
            face_gap_vectors(state, (int)f, &scene.face_gap[f * 6]);

        // ---- work items ----------------------------------------------------
        const size_t rowbytes = (size_t)((map.models[0].visleafs + 7) / 8);
        const uint32_t stride_words = (uint32_t)((rowbytes + 3) / 4 + 1);
        std::vector<uint32_t> pvs_words;
        std::vector<std::vector<byte>> pvs_rows_bytes;
        std::map<int, int> visofs_to_row;

        auto pvs_row_for = [&](const vec3v &pos) -> int
        {
            int visofs = -2; // -2 = "no vis data" whole map row
            if (!map.visibility.empty())
                visofs = rad::point_in_leaf(state, pos)->visofs;
            auto it = visofs_to_row.find(visofs);
            if (it != visofs_to_row.end())
                return it->second;
            std::vector<byte> row(rowbytes + 8, 0); // slack like the cpu buffer tail
            if (visofs == -2)
                std::memset(row.data(), 255, rowbytes);
            else if (visofs == -1)
                std::memset(row.data(), 0, rowbytes);
            else
                rad::decompress_vis(state, &map.visibility[(size_t)visofs], row.data(),
                                    (unsigned)rowbytes);
            int index = (int)pvs_rows_bytes.size();
            pvs_rows_bytes.push_back(row);
            visofs_to_row[visofs] = index;
            pvs_words.resize((size_t)(index + 1) * stride_words, 0);
            std::memcpy(&pvs_words[(size_t)index * stride_words], row.data(), rowbytes);
            return index;
        };

        std::mt19937 rng(20260712);
        std::uniform_real_distribution<float> u01(0.0f, 1.0f);
        std::vector<rad::gpu::work_item_gpu> items;
        std::vector<vec3v> item_pos, item_normal;
        std::vector<int> item_face;

        size_t attempts = 0;
        while (item_pos.size() < ctx.num_positions && attempts < ctx.num_positions * 20)
        {
            attempts++;
            int facenum = (int)(u01(rng) * (float)map.faces.size()) % (int)map.faces.size();
            const format::dface_t &f = map.faces[(size_t)facenum];
            if (map.texinfo[(size_t)f.texinfo].flags & rad::tex_special)
                continue;
            // face centroid from the winding, plus the model offset
            vec3v centroid{};
            for (int e = 0; e < f.numedges; e++)
            {
                int se = map.surfedges[(size_t)(f.firstedge + e)];
                int v = se >= 0 ? map.edges[(size_t)se].v[0] : map.edges[(size_t)-se].v[1];
                centroid[0] += map.vertexes[(size_t)v].point[0];
                centroid[1] += map.vertexes[(size_t)v].point[1];
                centroid[2] += map.vertexes[(size_t)v].point[2];
            }
            math::scale(centroid, (vec_t)1.0 / f.numedges, centroid);
            math::add(centroid, state.face_offset[(size_t)facenum], centroid);
            const rad::plane *pl = rad::plane_from_face_number(state, (unsigned)facenum);
            vec3v pos;
            math::multiply_add(centroid, (vec_t)1.0, pl->normal, pos);

            format::dleaf_t *leaf = rad::point_in_leaf(state, pos);
            if (leaf->contents == rad::contents_solid)
                continue; // inside a wall; gather would be all zero anyway

            item_pos.push_back(pos);
            item_normal.push_back(pl->normal);
            item_face.push_back(facenum);
        }

        int miptex_count = (int)state.lightingconeinfo.size();
        for (size_t p = 0; p < item_pos.size(); p++)
        {
            int facenum = item_face[p];
            const format::dface_t &f = map.faces[(size_t)facenum];
            int miptex = map.texinfo[(size_t)f.texinfo].miptex;
            float power = 1.0f, scale = 1.0f;
            if (miptex >= 0 && miptex < miptex_count)
            {
                power = state.lightingconeinfo[(size_t)miptex][0];
                scale = state.lightingconeinfo[(size_t)miptex][1];
            }
            int row = pvs_row_for(item_pos[p]);
            for (int step = 0; step <= 1; step++)
            {
                rad::gpu::work_item_gpu item = {};
                for (int x = 0; x < 3; x++)
                {
                    item.pos[x] = item_pos[p][x];
                    item.normal[x] = item_normal[p][x];
                }
                item.cone_power = power;
                item.cone_scale = scale;
                item.pvs_row = row;
                item.step = step;
                item.face = facenum;
                items.push_back(item);
            }
        }
        std::printf("work items      %zu (%zu positions, steps 0+1, %zu pvs rows)\n",
                    items.size(), item_pos.size(), pvs_rows_bytes.size());

        // ---- cpu pass ------------------------------------------------------
        auto cpu_start = std::chrono::steady_clock::now();
        std::vector<std::vector<vec3v>> cpu_adds(items.size()); // indexed by real style
        for (size_t i = 0; i < items.size(); i++)
        {
            vec3v sampled[rad::allstyles];
            byte styles[rad::allstyles];
            std::memset(sampled, 0, sizeof(sampled));
            std::memset(styles, 255, sizeof(styles));
            styles[0] = 0;
            vec3v pos{items[i].pos[0], items[i].pos[1], items[i].pos[2]};
            vec3v normal{items[i].normal[0], items[i].normal[1], items[i].normal[2]};
            const byte *pvs = pvs_rows_bytes[(size_t)items[i].pvs_row].data();
            int miptex = map.texinfo[(size_t)map.faces[(size_t)items[i].face].texinfo].miptex;
            rad::gather_sample_light_for_check(state, pos, pvs, normal, sampled, styles,
                                               items[i].step, miptex, items[i].face);
            cpu_adds[i].assign(rad::allstyles, vec3v{});
            for (int j = 0; j < rad::allstyles && styles[j] != 255; j++)
                cpu_adds[i][(size_t)styles[j]] = sampled[j];
        }
        std::chrono::duration<double> cpu_elapsed = std::chrono::steady_clock::now() - cpu_start;

        // ---- gpu pass ------------------------------------------------------
        auto gpu_start = std::chrono::steady_clock::now();
        std::vector<rad::gpu::gather_result_gpu> results;
        std::vector<rad::gpu::near_pair> near_pairs;
        if (!rad::gpu::gather_batch(scene, items, pvs_words, stride_words, results, near_pairs))
        {
            std::printf("FAIL: gather_batch: %s\n", rad::gpu::last_error().c_str());
            ctx.exit_code = 2;
            return;
        }
        std::chrono::duration<double> gpu_elapsed = std::chrono::steady_clock::now() - gpu_start;

        // near pairs resolved with the reference cpu code
        auto near_start = std::chrono::steady_clock::now();
        std::vector<std::vector<vec3v>> gpu_adds(items.size());
        for (size_t i = 0; i < items.size(); i++)
        {
            gpu_adds[i].assign(rad::allstyles, vec3v{});
            for (int c = 0; c < (int)compact_to_style.size(); c++)
            {
                gpu_adds[i][(size_t)compact_to_style[(size_t)c]] =
                    vec3v{results[i].adds[c * 3 + 0], results[i].adds[c * 3 + 1],
                          results[i].adds[c * 3 + 2]};
            }
        }
        for (const rad::gpu::near_pair &pair : near_pairs)
        {
            const rad::gpu::work_item_gpu &item = items[pair.item];
            const rad::directlight *l = &state.lightarray[pair.light];
            vec3v pos{item.pos[0], item.pos[1], item.pos[2]};
            vec3v normal{item.normal[0], item.normal[1], item.normal[2]};
            vec3v add;
            if (resolve_near_pair(state, pos, normal, l, item.cone_power, item.cone_scale,
                                  &scene.face_gap[(size_t)item.face * 6], add))
            {
                math::add(gpu_adds[pair.item][(size_t)l->style], add,
                          gpu_adds[pair.item][(size_t)l->style]);
            }
        }
        std::chrono::duration<double> near_elapsed = std::chrono::steady_clock::now() - near_start;

        // ---- compare -------------------------------------------------------
        size_t compared = 0, exceed = 0, shown = 0;
        double max_delta = 0, max_rel = 0;
        for (size_t i = 0; i < items.size(); i++)
        {
            for (int s = 0; s < rad::allstyles; s++)
            {
                const vec3v &a = cpu_adds[i][(size_t)s];
                const vec3v &b = gpu_adds[i][(size_t)s];
                for (int x = 0; x < 3; x++)
                {
                    double d = std::fabs((double)a[x] - (double)b[x]);
                    double mag = std::fabs((double)a[x]) > std::fabs((double)b[x])
                        ? std::fabs((double)a[x]) : std::fabs((double)b[x]);
                    if (mag > 0 || d > 0)
                        compared++;
                    if (d > max_delta)
                        max_delta = d;
                    if (mag > 1.0 && d / mag > max_rel)
                        max_rel = d / mag;
                    // tolerance: light values feed luxels from 0 to 255 after scaling;
                    // 005 raw units is far below one lightmap step
                    if (d > 0.05 && (mag <= 1.0 || d / mag > 1e-3))
                    {
                        exceed++;
                        if (shown < 10)
                        {
                            shown++;
                            std::printf("  delta item %zu style %d ch %d: cpu %g gpu %g\n",
                                        i, s, x, (double)a[x], (double)b[x]);
                        }
                    }
                }
            }
        }

        std::printf("cpu             %.3fs\n", cpu_elapsed.count());
        std::printf("gpu             %.3fs (+%.3fs cpu near-pair resolve, %zu pairs)\n",
                    gpu_elapsed.count(), near_elapsed.count(), near_pairs.size());
        std::printf("styles          %zu distinct\n", compact_to_style.size());
        std::printf("channels        %zu nonzero-compared\n", compared);
        std::printf("max abs delta   %g\n", max_delta);
        std::printf("max rel delta   %g (on magnitudes > 1)\n", max_rel);
        std::printf("exceeders       %zu\n", exceed);

        if (items.empty() || exceed > 0)
        {
            std::printf("FAIL\n");
            ctx.exit_code = 1;
            return;
        }
        std::printf("OK\n");
        ctx.exit_code = 0;
    }
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::printf("usage: rad_gpu_gather_check <map.bsp> [num_positions]\n");
        return 2;
    }
    const char *bsp_path = argv[1];

    format::map_data map;
    if (!format::bsp_file::load(bsp_path, map))
    {
        std::printf("FAIL: could not load '%s'\n", bsp_path);
        return 2;
    }
    if (map.nodes.empty() || map.models.empty())
    {
        std::printf("FAIL: '%s' has no bsp tree\n", bsp_path);
        return 2;
    }

    check_ctx ctx;
    if (argc > 2)
        ctx.num_positions = (size_t)std::atoll(argv[2]);

    rad::rad_options options;
    // lightsrad next to the map (the cli's primary cascade rule), and the
    // map folder as a wad directory so textures resolve without a wa_
    {
        std::string base = bsp_path;
        size_t slash = base.find_last_of("/\\");
        std::string dir = slash == std::string::npos ? "" : base.substr(0, slash + 1);
        std::string rad_file = dir + "lights.rad";
        FILE *f = std::fopen(rad_file.c_str(), "rb");
        if (f)
        {
            std::fclose(f);
            options.rad_files.push_back(rad_file);
        }
        options.wad_folders.push_back(dir.empty() ? "." : dir);
    }
    options.direct_state_hook = &hook;
    options.direct_state_hook_ctx = &ctx;

    std::string base = bsp_path;
    size_t dot = base.find_last_of('.');
    if (dot != std::string::npos)
        base = base.substr(0, dot);
    rad::run_rad(map, base, options);
    return ctx.exit_code;
}
