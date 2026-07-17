#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

#include "common/string_util.h"
#include "csg/csg.h"
#include "support/scratch.h"
#include "test_wad.h"

namespace
{
    bool expect(bool ok, const char *label)
    {
        if (!ok)
            std::printf("%s failed\n", label);
        return ok;
    }

    bool expect_vec(const math::vec3v &actual, const math::vec3v &expected, const char *label)
    {
        if (math::equal(actual, expected))
            return true;
        std::printf("%s expected (%.3f, %.3f, %.3f), got (%.3f, %.3f, %.3f)\n",
                    label,
                    expected.x, expected.y, expected.z,
                    actual.x, actual.y, actual.z);
        return false;
    }

    void add_side(csg::map_source &map,
                  const char *texture,
                  const math::vec3v &p0,
                  const math::vec3v &p1,
                  const math::vec3v &p2)
    {
        csg::brush_side side;
        str::copy(side.texture.name, sizeof(side.texture.name), texture);
        side.planepts[0] = p0;
        side.planepts[1] = p1;
        side.planepts[2] = p2;
        map.sides.push_back(side);
    }

    void add_cube(csg::map_source &map, const math::vec3v &mins, const math::vec3v &maxs)
    {
        add_side(map, "STONE", {mins.x, mins.y, mins.z}, {mins.x, maxs.y, mins.z}, {mins.x, mins.y, maxs.z});
        add_side(map, "STONE", {maxs.x, mins.y, mins.z}, {maxs.x, mins.y, maxs.z}, {maxs.x, maxs.y, mins.z});
        add_side(map, "STONE", {mins.x, mins.y, mins.z}, {mins.x, mins.y, maxs.z}, {maxs.x, mins.y, mins.z});
        add_side(map, "STONE", {mins.x, maxs.y, mins.z}, {maxs.x, maxs.y, mins.z}, {mins.x, maxs.y, maxs.z});
        add_side(map, "STONE", {mins.x, mins.y, mins.z}, {maxs.x, mins.y, mins.z}, {mins.x, maxs.y, mins.z});
        add_side(map, "STONE", {mins.x, mins.y, maxs.z}, {mins.x, maxs.y, maxs.z}, {maxs.x, mins.y, maxs.z});
    }

    csg::map_source cube_map(const std::string &wad_path)
    {
        csg::map_source map;
        csg::map_entity world;
        world.pairs.emplace_back("wad", wad_path);
        world.pairs.emplace_back("classname", "worldspawn");
        world.num_brushes = 1;
        map.entities.push_back(world);

        add_cube(map, {0, 0, 0}, {64, 64, 64});
        csg::map_brush brush;
        brush.num_sides = 6;
        map.brushes.push_back(brush);
        return map;
    }

    void write_hull_file(const char *path)
    {
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        out << "20 22 24\n";
        out << "40 42 44\n";
        out << "60 62 64\n";
    }
}

int main()
{
    bool ok = true;
    const std::filesystem::path scratch =
        test_support::scratch_directory("csg_stage_check");

    csg::clip_type clip = csg::clip_type::legacy;
    ok &= expect(csg::parse_clip_type("precise", clip), "parse precise");
    ok &= expect(clip == csg::clip_type::precise, "precise enum");
    ok &= expect(!csg::parse_clip_type("bad", clip), "reject bad cliptype");
    ok &= expect(std::string(csg::clip_type_name(csg::clip_type::normalized)) == "normalized", "cliptype name");

    const std::string hull_file = (scratch / "hulls.hull").string();
    const std::string wad_path = (scratch / "textures.wad").string();
    write_hull_file(hull_file.c_str());
    test_wad::write(wad_path, {"STONE"});

    csg::csg_options options;
    options.hull_file_path = hull_file;
    csg::csg_result result = csg::run_csg(cube_map(wad_path), options);
    ok &= expect(result.entities.size() == 1, "entity count");
    ok &= expect(!result.entities.empty() && result.entities[0].brushes.size() == 1, "brush count");
    ok &= expect(result.texinfos.entries().size() == 3, "texinfo count");
    ok &= expect_vec(result.entities[0].brushes[0].hulls[1].bounds.mins, {-10, -11, -12}, "run_csg hull1 mins");
    ok &= expect_vec(result.entities[0].brushes[0].hulls[1].bounds.maxs, {74, 75, 76}, "run_csg hull1 maxs");

    if (ok)
        std::printf("csg_stage_check passed\n");
    return ok ? 0 : 1;
}
