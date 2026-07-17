// exercises the bsp input readers against the features fixture's csg sidecars:
// planes and hull sizes load, every hull's surface chain and detail brush list
// parse to the end, and the surfaces carry sane plane numbers and bounds
// pass the base path of a compiled features fixture as the first argument

#include <cstdio>
#include <string>

#include "bsp/internal.h"
#include "common/filesystem.h"
#include "format/bsp/file.h"

namespace
{
    bool expect(bool ok, const char *label)
    {
        if (!ok)
            std::printf("%s failed\n", label);
        return ok;
    }
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::printf("usage: bsp_inputs_check <compiled-map-base-path>\n");
        return 2;
    }
    const std::string base = argv[1];
    if (!fs::exists(base + ".p0"))
    {
        std::printf("bsp_inputs_check skipped (fixture sidecars not present)\n");
        return 0;
    }

    bool ok = true;

    format::map_data map;
    ok &= expect(format::bsp_file::load(base + ".bsp", map), "load csg bsp");

    bsp::bsp_state state;
    state.map = &map;
    state.base_path = base;

    bsp::open_input_files(state);
    bsp::load_hull_sizes(state);
    bsp::load_plane_file(state);

    ok &= expect(state.planes.size() == map.planes.size(), "plane count matches bsp");
    ok &= expect(state.hull_size[1][0] == math::vec3v(-16, -16, -36), "hull 1 mins");
    ok &= expect(state.hull_size[2][1] == math::vec3v(32, 32, 32), "hull 2 maxs");

    // the fixture has one worldspawn model and several brush entities; every
    // hull's file must parse cleanly through all models to the terminator
    int models = 0;
    while (true)
    {
        bsp::surfchain *chain = bsp::read_surfs(state, 0);
        if (!chain)
            break;
        bsp::brush *detail = bsp::read_brushes(state, 0);
        int surfaces = 0;
        for (bsp::surface *s = chain->surfaces; s; s = s->next)
        {
            surfaces++;
            ok &= expect(s->planenum >= 0 && s->planenum < (int)state.planes.size(),
                         "surface planenum in range");
            ok &= expect(s->faces != nullptr, "surface has faces");
        }
        if (models == 0)
        {
            ok &= expect(surfaces > 4, "world has surfaces");
            ok &= expect(chain->mins.x < chain->maxs.x, "world bounds sane");
        }
        for (int hull = 1; hull < bsp::num_hulls; hull++)
        {
            bsp::surfchain *hull_chain = bsp::read_surfs(state, hull);
            ok &= expect(hull_chain != nullptr, "hull chain present");
            bsp::read_brushes(state, hull);
        }
        (void)detail;
        models++;
    }
    ok &= expect(models >= 4, "all models parsed");

    // brush splitting round trip on a box: split down the middle, both halves
    // keep volume and the original bounds survive as the union
    bsp::brush *box = bsp::brush_from_box({0, 0, 0}, {64, 64, 64});
    ok &= expect(box != nullptr, "box brush built");
    bsp::plane split;
    split.normal = {1, 0, 0};
    split.dist = 32;
    bsp::brush *front = nullptr;
    bsp::brush *back = nullptr;
    bsp::split_brush(box, &split, &front, &back);
    ok &= expect(front != nullptr && back != nullptr, "box split in two");
    if (front && back)
    {
        math::vec3v mins, maxs;
        bsp::calc_brush_bounds(front, mins, maxs);
        ok &= expect(mins == math::vec3v(32, 0, 0) && maxs == math::vec3v(64, 64, 64),
                     "front half bounds");
        bsp::calc_brush_bounds(back, mins, maxs);
        ok &= expect(mins == math::vec3v(0, 0, 0) && maxs == math::vec3v(32, 64, 64),
                     "back half bounds");
        bsp::free_brush(front);
        bsp::free_brush(back);
    }

    if (!ok)
        return 1;
    std::printf("bsp_inputs_check passed (%d models)\n", models);
    return 0;
}
