#include <cstdio>
#include <vector>

#include "common/string_util.h"
#include "csg/brush_union.h"
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

    csg::map_source two_cube_map(const std::string &wad_path,
                                 const math::vec3v &second_mins,
                                 const math::vec3v &second_maxs)
    {
        csg::map_source map;
        csg::map_entity world;
        world.pairs.emplace_back("wad", wad_path);
        world.pairs.emplace_back("classname", "worldspawn");
        world.num_brushes = 2;
        map.entities.push_back(world);

        add_cube(map, {0, 0, 0}, {64, 64, 64});
        csg::map_brush first;
        first.num_sides = 6;
        first.original_brush_num = 0;
        map.brushes.push_back(first);

        add_cube(map, second_mins, second_maxs);
        csg::map_brush second;
        second.first_side = 6;
        second.num_sides = 6;
        second.brush_num = 1;
        second.original_brush_num = 1;
        map.brushes.push_back(second);
        return map;
    }
}

int main()
{
    bool ok = true;
    const std::filesystem::path scratch =
        test_support::scratch_directory("csg_brush_union_check");
    const std::string wad_path = (scratch / "textures.wad").string();

    test_wad::write(wad_path, {"STONE"});

    csg::csg_options options;
    csg::csg_result overlap = csg::run_csg(
        two_cube_map(wad_path, {32, 0, 0}, {96, 64, 64}), options);
    std::vector<csg::brush_union_warning> warnings = csg::calculate_brush_union_warnings(
        overlap, (vec_t)0.01, options.brush.world_extent);
    ok &= expect(warnings.size() == 2, "overlap warning count");
    if (warnings.size() == 2)
    {
        ok &= expect(warnings[0].brush_num == 0 && warnings[0].other_brush_num == 1, "first warning brushes");
        ok &= expect(warnings[1].brush_num == 1 && warnings[1].other_brush_num == 0, "second warning brushes");
        ok &= expect(warnings[0].percent > 49.9 && warnings[0].percent < 50.1, "first warning percent");
        ok &= expect(warnings[1].percent > 49.9 && warnings[1].percent < 50.1, "second warning percent");
    }

    std::vector<csg::brush_union_warning> disabled = csg::calculate_brush_union_warnings(
        overlap, (vec_t)0.0, options.brush.world_extent);
    ok &= expect(disabled.empty(), "disabled threshold");

    csg::csg_result disjoint = csg::run_csg(
        two_cube_map(wad_path, {96, 0, 0}, {160, 64, 64}), options);
    std::vector<csg::brush_union_warning> disjoint_warnings = csg::calculate_brush_union_warnings(
        disjoint, (vec_t)0.01, options.brush.world_extent);
    ok &= expect(disjoint_warnings.empty(), "disjoint warnings");

    if (ok)
        std::printf("csg_brush_union_check passed\n");
    return ok ? 0 : 1;
}
