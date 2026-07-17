// compares the cpu and gpu bsp trace implementations using a compiled bsp
// it sends a repeatable batch of random segments through both implementations
// and fails when the mismatch rate exceeds one in a million
//
// usage: rad_gpu_trace_check <mapbsp> [segment_count]

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <random>
#include <vector>

#include "format/bsp/file.h"
#include "../src/rad/internal.h"
#include "../src/rad/gpu/gpu.h"

namespace
{
    const char *contents_name(int contents)
    {
        switch (contents)
        {
        case rad::contents_empty: return "empty";
        case rad::contents_solid: return "solid";
        case rad::contents_sky: return "sky";
        default: return "other";
        }
    }
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::printf("usage: rad_gpu_trace_check <map.bsp> [segment_count]\n");
        return 2;
    }
    const char *bsp_path = argv[1];
    const size_t num_segments = argc > 2 ? (size_t)std::atoll(argv[2]) : 1000000;

    format::map_data map;
    if (!format::bsp_file::load(bsp_path, map))
    {
        std::printf("FAIL: could not load '%s'\n", bsp_path);
        return 2;
    }
    if (map.nodes.empty() || map.models.empty())
    {
        std::printf("FAIL: '%s' has no bsp tree (run hlbsp first)\n", bsp_path);
        return 2;
    }

    auto state = std::make_unique<rad::rad_state>();
    state->map = &map;
    rad::load_planes(*state);
    rad::make_tnodes(*state);
    std::printf("%s: %zu tnodes\n", bsp_path, state->tnodes.size());

    if (!rad::gpu::available())
    {
        std::printf("FAIL: no gpu device: %s\n", rad::gpu::last_error().c_str());
        return 2;
    }
    std::printf("gpu: %s\n", rad::gpu::device_name().c_str());

    // world bounds, padded so some segments start or end outside
    const format::dmodel_t &world = map.models[0];
    float mins[3], maxs[3];
    for (int i = 0; i < 3; i++)
    {
        mins[i] = world.mins[i] - 128.0f;
        maxs[i] = world.maxs[i] + 128.0f;
    }

    // deterministic segment mix: short local segments (sample->light style),
    // full range segments, and long sun style rays leaving the world
    std::mt19937 rng(20260712);
    std::uniform_real_distribution<float> u01(0.0f, 1.0f);
    auto random_point = [&](float *out)
    {
        for (int i = 0; i < 3; i++)
            out[i] = mins[i] + (maxs[i] - mins[i]) * u01(rng);
    };

    std::vector<rad::gpu::trace_segment> segments(num_segments);
    for (size_t s = 0; s < num_segments; s++)
    {
        rad::gpu::trace_segment &seg = segments[s];
        random_point(seg.start);
        const int kind = (int)(u01(rng) * 3.0f);
        if (kind == 0)
        {
            // short segment near the start point
            for (int i = 0; i < 3; i++)
                seg.stop[i] = seg.start[i] + (u01(rng) - 0.5f) * 256.0f;
        }
        else if (kind == 1)
        {
            random_point(seg.stop);
        }
        else
        {
            // sun style: long ray in a random direction, like the skylight
            // gather's pos + sunnormal * -bogus_range probes
            float dir[3] = {u01(rng) - 0.5f, u01(rng) - 0.5f, u01(rng) - 0.5f};
            float len = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
            if (len < 1e-6f)
            {
                dir[0] = 1.0f;
                len = 1.0f;
            }
            for (int i = 0; i < 3; i++)
                seg.stop[i] = seg.start[i] + dir[i] / len * 65536.0f;
        }
    }

    // cpu pass
    auto cpu_start = std::chrono::steady_clock::now();
    std::vector<int> cpu_contents(num_segments);
    std::vector<math::vec3v> cpu_skyhit(num_segments);
    for (size_t s = 0; s < num_segments; s++)
    {
        math::vec3v start = {segments[s].start[0], segments[s].start[1], segments[s].start[2]};
        math::vec3v stop = {segments[s].stop[0], segments[s].stop[1], segments[s].stop[2]};
        cpu_contents[s] = rad::test_line(*state, start, stop, &cpu_skyhit[s][0]);
    }
    std::chrono::duration<double> cpu_elapsed = std::chrono::steady_clock::now() - cpu_start;

    // gpu pass
    std::vector<rad::gpu::tnode_gpu> tnodes(state->tnodes.size());
    for (size_t i = 0; i < tnodes.size(); i++)
    {
        const rad::tnode &tn = state->tnodes[i];
        rad::gpu::tnode_gpu &out = tnodes[i];
        out.normal[0] = tn.normal[0];
        out.normal[1] = tn.normal[1];
        out.normal[2] = tn.normal[2];
        out.dist = tn.dist;
        out.type = tn.type;
        out.children[0] = tn.children[0];
        out.children[1] = tn.children[1];
    }

    auto gpu_start = std::chrono::steady_clock::now();
    std::vector<rad::gpu::trace_result> gpu_results;
    if (!rad::gpu::trace_batch(tnodes, segments, gpu_results))
    {
        std::printf("FAIL: trace_batch: %s\n", rad::gpu::last_error().c_str());
        return 2;
    }
    std::chrono::duration<double> gpu_elapsed = std::chrono::steady_clock::now() - gpu_start;

    // compare
    size_t mismatches = 0;
    size_t coplanar_mismatches = 0;
    size_t coplanar_total = 0;
    size_t overflow_total = 0;
    size_t skyhit_drift = 0;
    size_t shown = 0;
    for (size_t s = 0; s < num_segments; s++)
    {
        const rad::gpu::trace_result &gr = gpu_results[s];
        if (gr.flags & 1)
            coplanar_total++;
        if (gr.flags & 2)
            overflow_total++;
        if (gr.contents != cpu_contents[s])
        {
            if (gr.flags & 1)
            {
                coplanar_mismatches++;
            }
            else
            {
                mismatches++;
                if (shown < 10)
                {
                    shown++;
                    std::printf("  mismatch seg %zu: cpu %s gpu %s flags %d  (%.1f %.1f %.1f)->(%.1f %.1f %.1f)\n",
                                s, contents_name(cpu_contents[s]), contents_name(gr.contents), gr.flags,
                                segments[s].start[0], segments[s].start[1], segments[s].start[2],
                                segments[s].stop[0], segments[s].stop[1], segments[s].stop[2]);
                }
            }
        }
        else if (gr.contents == rad::contents_sky)
        {
            float dx = gr.skyhit[0] - (float)cpu_skyhit[s][0];
            float dy = gr.skyhit[1] - (float)cpu_skyhit[s][1];
            float dz = gr.skyhit[2] - (float)cpu_skyhit[s][2];
            if (dx * dx + dy * dy + dz * dz > 0.5f * 0.5f)
                skyhit_drift++;
        }
    }

    std::printf("segments        %zu\n", num_segments);
    std::printf("cpu             %.3fs (%.1fM rays/s, single thread)\n",
                cpu_elapsed.count(), num_segments / cpu_elapsed.count() / 1e6);
    std::printf("gpu             %.3fs (%.1fM rays/s, includes upload+readback)\n",
                gpu_elapsed.count(), num_segments / gpu_elapsed.count() / 1e6);
    std::printf("mismatches      %zu (%.2e)\n", mismatches, (double)mismatches / num_segments);
    std::printf("coplanar        %zu flagged, %zu mismatched\n", coplanar_total, coplanar_mismatches);
    std::printf("stack overflow  %zu\n", overflow_total);
    std::printf("skyhit drift    %zu (> 0.5 units)\n", skyhit_drift);

    const size_t budget = num_segments / 1000000 + 1;
    if (mismatches > budget)
    {
        std::printf("FAIL: mismatch rate above 1e-6\n");
        return 1;
    }
    std::printf("OK\n");
    return 0;
}
