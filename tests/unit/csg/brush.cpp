#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "common/string_util.h"
#include "csg/brush.h"
#include "csg/hull_file.h"
#include "csg/map_post.h"
#include "csg/textures.h"
#include "support/scratch.h"

namespace
{
    bool expect_content(const char *name, csg::content expected)
    {
        csg::content actual = csg::texture_content(name);
        if (actual != expected)
        {
            std::printf("texture_content(%s) expected %s, got %s\n",
                        name, csg::content_to_string(expected), csg::content_to_string(actual));
            return false;
        }
        return true;
    }

    bool near(vec_t a, vec_t b)
    {
        return std::fabs(a - b) <= (vec_t)0.001;
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

    void write_text_file(const char *path, const char *contents)
    {
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        out << contents;
    }

    csg::brush_side side(const char *texture)
    {
        csg::brush_side s;
        str::copy(s.texture.name, sizeof(s.texture.name), texture);
        return s;
    }

    void add_side(csg::map_source &map,
                  const char *texture,
                  const math::vec3v &p0,
                  const math::vec3v &p1,
                  const math::vec3v &p2)
    {
        csg::brush_side s = side(texture);
        s.planepts[0] = p0;
        s.planepts[1] = p1;
        s.planepts[2] = p2;
        map.sides.push_back(s);
    }

    void add_cube(csg::map_source &map,
                  const char *texture,
                  const math::vec3v &mins,
                  const math::vec3v &maxs)
    {
        add_side(map, texture, {mins.x, mins.y, mins.z}, {mins.x, maxs.y, mins.z}, {mins.x, mins.y, maxs.z});
        add_side(map, texture, {maxs.x, mins.y, mins.z}, {maxs.x, mins.y, maxs.z}, {maxs.x, maxs.y, mins.z});
        add_side(map, texture, {mins.x, mins.y, mins.z}, {mins.x, mins.y, maxs.z}, {maxs.x, mins.y, mins.z});
        add_side(map, texture, {mins.x, maxs.y, mins.z}, {maxs.x, maxs.y, mins.z}, {mins.x, maxs.y, maxs.z});
        add_side(map, texture, {mins.x, mins.y, mins.z}, {maxs.x, mins.y, mins.z}, {mins.x, maxs.y, mins.z});
        add_side(map, texture, {mins.x, mins.y, maxs.z}, {mins.x, maxs.y, maxs.z}, {maxs.x, mins.y, maxs.z});
    }

    csg::map_source cube_map(const char *texture = "STONE")
    {
        csg::map_source map;
        csg::map_entity world;
        world.pairs.emplace_back("classname", "worldspawn");
        map.entities.push_back(world);
        add_cube(map, texture, {0, 0, 0}, {64, 64, 64});
        return map;
    }

    csg::map_source sloped_map()
    {
        csg::map_source map;
        csg::map_entity world;
        world.pairs.emplace_back("classname", "worldspawn");
        map.entities.push_back(world);

        add_side(map, "STONE", {0, 0, 0}, {0, 64, 0}, {0, 0, 16});
        add_side(map, "STONE", {64, 0, 0}, {64, 0, 80}, {64, 64, 0});
        add_side(map, "STONE", {0, 0, 0}, {0, 0, 16}, {64, 0, 0});
        add_side(map, "STONE", {0, 64, 0}, {64, 64, 0}, {0, 64, 16});
        add_side(map, "STONE", {0, 0, 0}, {64, 0, 0}, {0, 64, 0});
        add_side(map, "STONE", {0, 0, 16}, {0, 64, 16}, {64, 0, 80});
        return map;
    }

    csg::map_source hullshape_map(csg::map_brush &solid_brush)
    {
        csg::map_source map;
        csg::map_entity world;
        world.pairs.emplace_back("classname", "worldspawn");
        map.entities.push_back(world);

        csg::map_entity shape_entity;
        shape_entity.pairs.emplace_back("classname", "info_hullshape");
        shape_entity.pairs.emplace_back("targetname", "small");
        shape_entity.pairs.emplace_back("origin", "0 0 0");
        shape_entity.first_brush = 0;
        shape_entity.num_brushes = 1;
        map.entities.push_back(shape_entity);

        add_cube(map, "STONE", {-8, -8, -8}, {8, 8, 8});
        csg::map_brush shape_brush;
        shape_brush.entity_num = 1;
        shape_brush.num_sides = 6;
        map.brushes.push_back(shape_brush);

        add_cube(map, "STONE", {0, 0, 0}, {64, 64, 64});
        solid_brush.num_sides = 6;
        solid_brush.first_side = 6;
        solid_brush.hull_shapes[1] = "small";
        return map;
    }

    csg::content classify(const char *classname, const char *a, const char *b = nullptr)
    {
        csg::map_source map;
        csg::map_entity entity;
        entity.pairs.emplace_back("classname", classname);
        map.entities.push_back(entity);
        map.sides.push_back(side(a));
        if (b)
            map.sides.push_back(side(b));

        csg::map_brush brush;
        brush.entity_num = 0;
        brush.num_sides = b ? 2 : 1;
        brush.first_side = 0;
        return csg::check_brush_contents(map, brush);
    }
}

int main()
{
    bool ok = true;
    const std::filesystem::path scratch =
        test_support::scratch_directory("csg_brush_check");

    ok &= expect_content("contentsolid", csg::content::solid);
    ok &= expect_content("contentwater", csg::content::water);
    ok &= expect_content("contentempty", csg::content::to_empty);
    ok &= expect_content("sky_day01", csg::content::sky);
    ok &= expect_content("env_sky01", csg::content::sky);
    ok &= expect_content("{!lava1", csg::content::lava);
    ok &= expect_content("!slime2", csg::content::slime);
    ok &= expect_content("!cur_270", csg::content::current_270);
    ok &= expect_content("origin", csg::content::origin);
    ok &= expect_content("boundingbox", csg::content::bounding_box);
    ok &= expect_content("solidhint", csg::content::null);
    ok &= expect_content("splitface", csg::content::hint);
    ok &= expect_content("@glass", csg::content::translucent);
    ok &= expect_content("bevel", csg::content::null);

    ok &= classify("worldspawn", "NULL") == csg::content::solid;
    ok &= classify("worldspawn", "SKIP", "BRICK") == csg::content::to_empty;
    ok &= classify("worldspawn", "CONTENTWATER", "BRICK") == csg::content::water;

    math::plane floor;
    floor.normal = {0, 0, 1};
    math::vec3v xv, yv;
    csg::texture_axis_from_plane(floor, xv, yv);
    ok &= xv == math::vec3v{1, 0, 0};
    ok &= yv == math::vec3v{0, -1, 0};

    math::plane east_wall;
    east_wall.normal = {-1, 0, 0};
    csg::texture_axis_from_plane(east_wall, xv, yv);
    ok &= xv == math::vec3v{0, 1, 0};
    ok &= yv == math::vec3v{0, 0, -1};

    csg::plane_store planes;
    int floor_plane = planes.plane_from_points({0, 0, 0}, {0, 128, 0}, {128, 0, 0});
    ok &= floor_plane == 0;
    ok &= planes.planes().size() == 2;
    ok &= planes.planes()[0].normal == math::vec3v{0, 0, 1};
    ok &= planes.planes()[1].normal == math::vec3v{0, 0, -1};
    ok &= planes.plane_from_points({0, 0, 0}, {0, 128, 0}, {128, 0, 0}) == 0;
    ok &= planes.find_int_plane({0, 0, -1}, {0, 0, 0}) == 1;

    csg::plane_store negative_first;
    int negative_plane = negative_first.plane_from_points({0, 0, 0}, {128, 0, 0}, {0, 128, 0});
    ok &= negative_plane == 1;
    ok &= negative_first.planes()[0].normal == math::vec3v{0, 0, 1};
    ok &= negative_first.planes()[1].normal == math::vec3v{0, 0, -1};

    ok &= planes.plane_from_points({0, 0, 0}, {128, 0, 0}, {256, 0, 0}) == -1;

    csg::map_source brush_map;
    brush_map.sides.push_back(side("STONE"));
    brush_map.sides[0].planepts[0] = {0, 0, 0};
    brush_map.sides[0].planepts[1] = {0, 128, 0};
    brush_map.sides[0].planepts[2] = {128, 0, 0};
    brush_map.sides.push_back(side("BEVEL"));
    brush_map.sides[1].bevel = true;
    brush_map.sides[1].planepts[0] = {10, 0, 0};
    brush_map.sides[1].planepts[1] = {10, 0, 128};
    brush_map.sides[1].planepts[2] = {10, 128, 0};

    csg::map_brush parsed_brush;
    parsed_brush.num_sides = 2;
    parsed_brush.first_side = 0;
    csg::plane_store brush_planes;
    std::vector<csg::brush_face> faces = csg::make_brush_planes(
        brush_map, parsed_brush, brush_planes, {1, 2, 3});
    ok &= faces.size() == 2;
    ok &= faces[0].planenum == 2;
    ok &= faces[0].bevel;
    ok &= faces[1].planenum == 0;
    ok &= !faces[1].bevel;
    ok &= brush_map.sides[0].planepts[0] == math::vec3v{-1, -2, -3};
    ok &= brush_planes.planes()[0].dist == -3;
    ok &= brush_planes.planes()[2].dist == 9;

    csg::map_source tex_map;
    tex_map.map_file_version = 220;
    tex_map.sides.push_back(side("STONE"));
    tex_map.sides[0].planepts[0] = {0, 0, 0};
    tex_map.sides[0].planepts[1] = {0, 128, 0};
    tex_map.sides[0].planepts[2] = {128, 0, 0};
    tex_map.sides[0].texture.valve.u_axis = {1, 0, 0};
    tex_map.sides[0].texture.valve.v_axis = {0, -1, 0};
    tex_map.sides[0].texture.valve.scale[0] = 1;
    tex_map.sides[0].texture.valve.scale[1] = 1;

    csg::map_brush tex_brush;
    tex_brush.num_sides = 1;
    csg::plane_store tex_planes;
    csg::texinfo_store texinfos;
    std::vector<csg::brush_face> tex_faces = csg::make_brush_planes(
        tex_map, tex_brush, tex_planes, {}, &texinfos);
    ok &= tex_faces.size() == 1;
    ok &= tex_faces[0].texinfo == 0;
    ok &= texinfos.entries().size() == 1;

    csg::map_source only_map = tex_map;
    csg::plane_store only_planes;
    std::vector<csg::brush_face> only_faces = csg::make_brush_planes(
        only_map, tex_brush, only_planes, {}, nullptr, true);
    ok &= only_faces.size() == 1;
    ok &= only_faces[0].texinfo == 0;

    csg::plane_store hull_planes;
    std::vector<csg::brush_face> hull_faces;
    auto add_hull_face = [&hull_planes, &hull_faces](const math::vec3v &normal, const math::vec3v &origin)
    {
        csg::brush_face face;
        face.planenum = hull_planes.find_int_plane(normal, origin);
        hull_faces.push_back(face);
    };

    add_hull_face({-1, 0, 0}, {0, 0, 0});
    add_hull_face({1, 0, 0}, {64, 0, 0});
    add_hull_face({0, -1, 0}, {0, 0, 0});
    add_hull_face({0, 1, 0}, {0, 64, 0});
    add_hull_face({0, 0, -1}, {0, 0, 0});
    add_hull_face({0, 0, 1}, {0, 0, 64});

    csg::map_brush hull_brush;
    hull_brush.original_entity_num = 7;
    hull_brush.original_brush_num = 11;
    csg::brush_hull hull = csg::make_hull_faces(hull_brush, hull_planes, hull_faces);
    ok &= hull.faces.size() == 6;
    for (const csg::brush_face &face : hull.faces)
    {
        ok &= face.winding.size() == 4;
        ok &= near(face.winding.area(), (vec_t)4096);
    }
    ok &= expect_vec(hull.bounds.mins, {0, 0, 0}, "hull mins");
    ok &= expect_vec(hull.bounds.maxs, {64, 64, 64}, "hull maxs");

    csg::map_source solid_map = cube_map();
    csg::map_brush solid_brush;
    solid_brush.num_sides = 6;
    solid_brush.contents = csg::check_brush_contents(solid_map, solid_brush);
    csg::plane_store solid_planes;
    csg::built_brush solid = csg::create_brush(solid_map, solid_brush, solid_planes, {});
    ok &= solid.contents == csg::content::solid;
    ok &= !solid.skipped;
    ok &= solid.hulls[0].faces.size() == 6;
    ok &= expect_vec(solid.hulls[0].bounds.mins, {0, 0, 0}, "solid brush mins");
    ok &= expect_vec(solid.hulls[0].bounds.maxs, {64, 64, 64}, "solid brush maxs");
    ok &= solid.hulls[1].faces.size() == 6;
    ok &= expect_vec(solid.hulls[1].bounds.mins, {-16, -16, -36}, "solid hull1 mins");
    ok &= expect_vec(solid.hulls[1].bounds.maxs, {80, 80, 100}, "solid hull1 maxs");
    ok &= expect_vec(solid.hulls[2].bounds.mins, {-32, -32, -32}, "solid hull2 mins");
    ok &= expect_vec(solid.hulls[2].bounds.maxs, {96, 96, 96}, "solid hull2 maxs");
    ok &= expect_vec(solid.hulls[3].bounds.mins, {-16, -16, -18}, "solid hull3 mins");
    ok &= expect_vec(solid.hulls[3].bounds.maxs, {80, 80, 82}, "solid hull3 maxs");

    const std::string old_hull_path = (scratch / "old_hulls.hull").string();
    write_text_file(old_hull_path.c_str(),
                    "( 0 0 0 ) ( 0 0 0 )\n"
                    "( -10 -11 -12 ) ( 10 11 12 )\n"
                    "( -20 -21 -22 ) ( 20 21 22 )\n"
                    "( -30 -31 -32 ) ( 30 31 32 )\n");
    csg::brush_build_options old_hull_options;
    csg::load_hull_file(old_hull_path, old_hull_options);
    ok &= expect_vec(old_hull_options.hull_size[0][0], {0, 0, 0}, "old hull0 mins");
    ok &= expect_vec(old_hull_options.hull_size[1][0], {-10, -11, -12}, "old hull1 mins");
    ok &= expect_vec(old_hull_options.hull_size[1][1], {10, 11, 12}, "old hull1 maxs");
    ok &= expect_vec(old_hull_options.hull_size[3][0], {-30, -31, -32}, "old hull3 mins");
    ok &= expect_vec(old_hull_options.hull_size[3][1], {30, 31, 32}, "old hull3 maxs");

    const std::string new_hull_path = (scratch / "new_hulls.hull").string();
    write_text_file(new_hull_path.c_str(),
                    "20 22 24\n"
                    "40 42 44\n"
                    "60 62 64\n");
    csg::brush_build_options new_hull_options;
    csg::load_hull_file(new_hull_path, new_hull_options);
    ok &= expect_vec(new_hull_options.hull_size[0][0], {0, 0, 0}, "new hull0 mins");
    ok &= expect_vec(new_hull_options.hull_size[1][0], {-10, -11, -12}, "new hull1 mins");
    ok &= expect_vec(new_hull_options.hull_size[1][1], {10, 11, 12}, "new hull1 maxs");
    ok &= expect_vec(new_hull_options.hull_size[3][0], {-30, -31, -32}, "new hull3 mins");
    ok &= expect_vec(new_hull_options.hull_size[3][1], {30, 31, 32}, "new hull3 maxs");

    csg::map_source slope_map = sloped_map();
    csg::map_brush slope_brush;
    slope_brush.num_sides = 6;
    slope_brush.contents = csg::check_brush_contents(slope_map, slope_brush);
    csg::plane_store slope_planes;
    csg::built_brush slope = csg::create_brush(slope_map, slope_brush, slope_planes, {});
    ok &= slope.contents == csg::content::solid;
    ok &= slope.hulls[0].faces.size() == 6;
    ok &= slope.hulls[1].faces.size() > 6;
    ok &= expect_vec(slope.hulls[0].bounds.mins, {0, 0, 0}, "slope hull0 mins");
    ok &= expect_vec(slope.hulls[0].bounds.maxs, {64, 64, 80}, "slope hull0 maxs");
    ok &= expect_vec(slope.hulls[1].bounds.mins, {-16, -16, -36}, "slope hull1 mins");
    ok &= expect_vec(slope.hulls[1].bounds.maxs, {80, 80, 116}, "slope hull1 maxs");

    csg::map_brush custom_hull_brush;
    csg::map_source custom_hull_map = hullshape_map(custom_hull_brush);
    custom_hull_map.brushes[0].contents =
        csg::check_brush_contents(custom_hull_map, custom_hull_map.brushes[0]);
    csg::hull_shape_library hull_shapes;
    csg::add_hull_shape(custom_hull_map, 1, hull_shapes);
    custom_hull_brush.contents = csg::check_brush_contents(custom_hull_map, custom_hull_brush);
    csg::brush_build_options custom_hull_options;
    custom_hull_options.hull_shapes = &hull_shapes;
    csg::plane_store custom_hull_planes;
    csg::built_brush custom_hull = csg::create_brush(
        custom_hull_map, custom_hull_brush, custom_hull_planes, {}, nullptr, custom_hull_options);
    ok &= custom_hull.hulls[1].faces.size() == 6;
    ok &= expect_vec(custom_hull.hulls[1].bounds.mins, {-8, -8, -8}, "custom hull1 mins");
    ok &= expect_vec(custom_hull.hulls[1].bounds.maxs, {72, 72, 72}, "custom hull1 maxs");

    csg::map_source clip_map = cube_map();
    csg::map_brush clip_brush = solid_brush;
    clip_brush.cliphull = 1 << 1;
    csg::plane_store clip_planes;
    csg::built_brush clip = csg::create_brush(clip_map, clip_brush, clip_planes, {});
    ok &= clip.contents == csg::content::solid;
    ok &= clip.hulls[0].faces.empty();
    ok &= clip.hulls[1].faces.size() == 6;
    ok &= clip.hulls[2].faces.empty();
    ok &= clip.hulls[3].faces.empty();
    ok &= expect_vec(clip.hulls[1].bounds.mins, {-16, -16, -36}, "clip hull1 mins");
    ok &= expect_vec(clip.hulls[1].bounds.maxs, {80, 80, 100}, "clip hull1 maxs");

    csg::map_source noclip_map = cube_map();
    csg::plane_store noclip_planes;
    csg::brush_build_options noclip_options;
    noclip_options.noclip = true;
    csg::built_brush noclip = csg::create_brush(noclip_map, clip_brush, noclip_planes, {}, nullptr, noclip_options);
    ok &= noclip.hulls[0].faces.empty();

    // origin brushes: the post parse walk builds them fully (registering their
    // planes first), sets the entity origin key, and leaves the brush inert
    csg::map_source origin_map = cube_map("ORIGIN");
    csg::map_entity func_door;
    func_door.pairs.emplace_back("classname", "func_door");
    func_door.first_brush = 0;
    func_door.num_brushes = 1;
    origin_map.entities.push_back(func_door);
    csg::map_brush origin_brush;
    origin_brush.entity_num = 1;
    origin_brush.num_sides = 6;
    origin_map.brushes.push_back(origin_brush);

    csg::plane_store origin_planes;
    csg::hull_shape_library origin_shapes;
    csg::map_post_options origin_post;
    csg::post_process_map(origin_map, origin_planes, nullptr, origin_shapes, origin_post);
    ok &= origin_map.brushes[0].contents == csg::content::origin;
    ok &= origin_map.entities[1].value("origin") == std::string("32 32 32");
    ok &= !origin_planes.planes().empty();
    csg::built_brush origin = csg::create_brush(origin_map, origin_map.brushes[0], origin_planes, {});
    ok &= origin.contents == csg::content::origin;
    ok &= origin.skipped;
    ok &= origin.hulls[0].faces.empty();

    csg::map_source bounds_map = cube_map("BOUNDINGBOX");
    csg::map_entity func_train;
    func_train.pairs.emplace_back("classname", "func_train");
    func_train.first_brush = 0;
    func_train.num_brushes = 1;
    bounds_map.entities.push_back(func_train);
    csg::map_brush bounds_brush;
    bounds_brush.entity_num = 1;
    bounds_brush.num_sides = 6;
    bounds_map.brushes.push_back(bounds_brush);

    csg::plane_store bounds_planes;
    csg::hull_shape_library bounds_shapes;
    csg::map_post_options bounds_post;
    csg::post_process_map(bounds_map, bounds_planes, nullptr, bounds_shapes, bounds_post);
    ok &= bounds_map.brushes[0].contents == csg::content::bounding_box;
    // the negative zero is real: the x=0 face plane has normal (-1,0,0), and
    // normal * dist produces -00 corner coordinates, which %0f prints as
    // "-0" the reference performs the identical arithmetic
    ok &= bounds_map.entities[1].value("zhlt_minsmaxs") == std::string("-0 0 0 64 64 64");

    csg::map_source entity_map;
    csg::map_entity entity_world;
    entity_world.pairs.emplace_back("classname", "worldspawn");
    entity_map.entities.push_back(entity_world);
    csg::map_entity entity_door;
    entity_door.pairs.emplace_back("classname", "func_door");
    entity_door.first_brush = 0;
    entity_door.num_brushes = 2;
    entity_map.entities.push_back(entity_door);

    add_cube(entity_map, "STONE", {32, 32, 32}, {96, 96, 96});
    add_cube(entity_map, "ORIGIN", {0, 0, 0}, {64, 64, 64});
    csg::map_brush entity_solid;
    entity_solid.entity_num = 1;
    entity_solid.num_sides = 6;
    entity_map.brushes.push_back(entity_solid);
    csg::map_brush entity_origin = entity_solid;
    entity_origin.first_side = 6;
    entity_origin.original_brush_num = 1;
    entity_map.brushes.push_back(entity_origin);

    csg::plane_store entity_planes;
    csg::hull_shape_library entity_shapes;
    csg::map_post_options entity_post;
    csg::post_process_map(entity_map, entity_planes, nullptr, entity_shapes, entity_post);
    csg::built_entity built_entity = csg::build_entity_brushes(entity_map, 1, entity_planes);
    ok &= built_entity.brushes.size() == 2;
    ok &= entity_map.entities[1].value("origin") == std::string("32 32 32");
    ok &= expect_vec(entity_map.entities[1].origin, {32, 32, 32}, "entity origin");
    ok &= built_entity.brushes[0].contents == csg::content::solid;
    ok &= built_entity.brushes[0].hulls[0].faces.size() == 6;
    ok &= expect_vec(built_entity.brushes[0].hulls[0].bounds.mins, {0, 0, 0}, "entity solid mins");
    ok &= expect_vec(built_entity.brushes[0].hulls[0].bounds.maxs, {64, 64, 64}, "entity solid maxs");
    ok &= built_entity.brushes[1].contents == csg::content::origin;
    ok &= built_entity.brushes[1].skipped;

    csg::map_source moved_map;
    moved_map.map_file_version = 220;
    csg::map_entity moved_world;
    moved_world.pairs.emplace_back("classname", "worldspawn");
    moved_map.entities.push_back(moved_world);
    csg::map_entity moved_door;
    moved_door.pairs.emplace_back("zhlt_transform", "16 0 0");
    moved_door.pairs.emplace_back("classname", "func_door");
    moved_door.first_brush = 0;
    moved_door.num_brushes = 2;
    moved_map.entities.push_back(moved_door);

    add_cube(moved_map, "STONE", {32, 32, 32}, {96, 96, 96});
    add_cube(moved_map, "ORIGIN", {0, 0, 0}, {64, 64, 64});
    moved_map.sides[0].texture.valve.u_axis = {1, 0, 0};
    moved_map.sides[0].texture.valve.v_axis = {0, 1, 0};
    moved_map.sides[0].texture.valve.scale[0] = 1;
    moved_map.sides[0].texture.valve.scale[1] = 1;
    moved_map.brushes.push_back(entity_solid);
    moved_map.brushes.push_back(entity_origin);

    csg::plane_store moved_planes;
    csg::hull_shape_library moved_shapes;
    csg::map_post_options moved_post;
    csg::post_process_map(moved_map, moved_planes, nullptr, moved_shapes, moved_post);
    csg::built_entity moved_entity = csg::build_entity_brushes(moved_map, 1, moved_planes);
    ok &= moved_map.entities[1].value("zhlt_transform") == std::string("");
    ok &= moved_map.entities[1].value("origin") == std::string("32 32 32");
    ok &= moved_map.sides[0].texture.valve.shift[0] == (vec_t)-16;
    ok &= moved_map.sides[0].texture.valve.shift[1] == (vec_t)0;
    ok &= expect_vec(moved_entity.brushes[0].hulls[0].bounds.mins, {16, 0, 0}, "moved solid mins");
    ok &= expect_vec(moved_entity.brushes[0].hulls[0].bounds.maxs, {80, 64, 64}, "moved solid maxs");

    if (!ok)
        return 1;
    std::printf("csg_brush_check passed\n");
    return 0;
}
