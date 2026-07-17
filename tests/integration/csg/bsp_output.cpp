#include <cstdio>
#include <string>
#include <vector>

#include "common/filesystem.h"
#include "common/string_util.h"
#include "csg/bsp_output.h"
#include "csg/csg.h"
#include "format/bsp/file.h"
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

    void add_cube(csg::map_source &map, const math::vec3v &mins, const math::vec3v &maxs, const char *texture = "STONE")
    {
        add_side(map, texture, {mins.x, mins.y, mins.z}, {mins.x, maxs.y, mins.z}, {mins.x, mins.y, maxs.z});
        add_side(map, texture, {maxs.x, mins.y, mins.z}, {maxs.x, mins.y, maxs.z}, {maxs.x, maxs.y, mins.z});
        add_side(map, texture, {mins.x, mins.y, mins.z}, {mins.x, mins.y, maxs.z}, {maxs.x, mins.y, mins.z});
        add_side(map, texture, {mins.x, maxs.y, mins.z}, {maxs.x, maxs.y, mins.z}, {mins.x, maxs.y, maxs.z});
        add_side(map, texture, {mins.x, mins.y, mins.z}, {maxs.x, mins.y, mins.z}, {mins.x, maxs.y, mins.z});
        add_side(map, texture, {mins.x, mins.y, maxs.z}, {mins.x, maxs.y, maxs.z}, {maxs.x, mins.y, maxs.z});
    }

    csg::map_source cube_map(const std::string &wad_path, const char *texture = "STONE")
    {
        csg::map_source map;
        csg::map_entity world;
        world.pairs.emplace_back("wad", wad_path);
        world.pairs.emplace_back("classname", "worldspawn");
        world.num_brushes = 1;
        map.entities.push_back(world);

        add_cube(map, {0, 0, 0}, {64, 64, 64}, texture);
        csg::map_brush brush;
        brush.num_sides = 6;
        map.brushes.push_back(brush);
        return map;
    }

    csg::map_source overlapping_cube_map(const std::string &wad_path)
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
        map.brushes.push_back(first);

        add_cube(map, {32, 0, 0}, {96, 64, 64});
        csg::map_brush second;
        second.first_side = 6;
        second.num_sides = 6;
        second.brush_num = 1;
        map.brushes.push_back(second);
        return map;
    }

    int count_surface_headers(const std::string &text)
    {
        int count = 0;
        size_t start = 0;
        while (start < text.size())
        {
            size_t end = text.find('\n', start);
            if (end == std::string::npos)
                end = text.size();
            int detail = 0;
            int plane = 0;
            int texinfo = 0;
            int contents = 0;
            unsigned points = 0;
            std::string line = text.substr(start, end - start);
            if (std::sscanf(line.c_str(), "%d %d %d %d %u", &detail, &plane, &texinfo, &contents, &points) == 5
                && plane != -1)
            {
                count++;
            }
            start = end + 1;
        }
        return count;
    }
}

int main()
{
    bool ok = true;
    const std::filesystem::path scratch =
        test_support::scratch_directory("csg_bsp_output_check");
    const std::string wad_path = (scratch / "textures.wad").string();
    const std::string base = (scratch / "output").string();

    test_wad::write(wad_path, {"STONE", "AAATRIGGER"});

    csg::csg_options options;
    csg::csg_result result = csg::run_csg(cube_map(wad_path), options);
    format::map_data data = csg::build_bsp_data(result);

    ok &= expect(data.planes.size() == result.planes.size(), "plane count");
    ok &= expect(data.texinfo.size() == result.texinfos.entries().size(), "texinfo count");
    ok &= expect(!data.textures.empty(), "textures lump");
    ok &= expect(data.entities.find("\"classname\" \"worldspawn\"") != std::string::npos, "entities lump");
    ok &= expect(data.models.empty() && data.faces.empty() && data.nodes.empty(), "csg bsp has no final geometry");

    ok &= expect(csg::write_intermediate_files(base, result, options.brush), "write intermediates");

    std::vector<unsigned char> p0;
    ok &= expect(fs::read_all(base + ".p0", p0), "read p0");
    std::string p0_text(reinterpret_cast<const char *>(p0.data()), p0.size());
    ok &= expect(p0_text.find("-1 -1 -1 -1 -1\r\n") != std::string::npos, "p0 model marker");
    ok &= expect(p0_text.find("0 ") == 0, "p0 starts with face");

    std::vector<unsigned char> b0;
    ok &= expect(fs::read_all(base + ".b0", b0), "read b0");
    std::string b0_text(reinterpret_cast<const char *>(b0.data()), b0.size());
    ok &= expect(b0_text == "-1\r\n", "b0 marker only");

    ok &= expect(format::bsp_file::write(base + ".bsp", data), "write bsp");
    format::map_data roundtrip;
    ok &= expect(format::bsp_file::load(base + ".bsp", roundtrip), "load bsp");
    ok &= expect(roundtrip.planes.size() == data.planes.size(), "roundtrip planes");
    ok &= expect(roundtrip.texinfo.size() == data.texinfo.size(), "roundtrip texinfo");

    csg::csg_result trigger = csg::run_csg(cube_map(wad_path, "AAATRIGGER"), options);
    for (const csg::brush_face &face : trigger.entities[0].brushes[0].hulls[0].faces)
        ok &= expect(face.texinfo >= 0, "aaatrigger preserved texinfo");

    csg::csg_options nullify_options;
    nullify_options.brush.nullify_trigger = true;
    csg::csg_result nullified_trigger = csg::run_csg(cube_map(wad_path, "AAATRIGGER"), nullify_options);
    for (const csg::brush_face &face : nullified_trigger.entities[0].brushes[0].hulls[0].faces)
        ok &= expect(face.texinfo == -1, "aaatrigger opt-in null texinfo");

    csg::csg_result overlap = csg::run_csg(overlapping_cube_map(wad_path), options);
    ok &= expect(csg::write_intermediate_files(base + "_overlap", overlap, options.brush), "write overlap");
    std::vector<unsigned char> overlap_p0;
    ok &= expect(fs::read_all(base + "_overlap.p0", overlap_p0), "read overlap p0");
    std::string overlap_text(reinterpret_cast<const char *>(overlap_p0.data()), overlap_p0.size());
    int overlap_faces = count_surface_headers(overlap_text);
    ok &= expect(overlap_faces > 0, "overlap has faces");
    ok &= expect(overlap_faces < 24, "overlap carved internal faces");

    if (ok)
        std::printf("csg_bsp_output_check passed\n");
    return ok ? 0 : 1;
}
