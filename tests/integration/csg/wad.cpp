#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "common/binary.h"
#include "common/filesystem.h"
#include "common/string_util.h"
#include "csg/csg.h"
#include "format/bsp/types.h"
#include "format/wad/archive.h"
#include "support/scratch.h"

namespace
{
    bool expect(bool ok, const char *label)
    {
        if (!ok)
            std::printf("%s failed\n", label);
        return ok;
    }

    std::vector<unsigned char> miptex_bytes(const char *name)
    {
        std::vector<unsigned char> out(40, 0);
        binary::writer output(out);
        std::memcpy(out.data(), name, std::strlen(name));
        output.patch_i32(16, 16);
        output.patch_i32(20, 16);
        output.patch_i32(24, 40);
        output.patch_i32(28, 296);
        output.patch_i32(32, 360);
        output.patch_i32(36, 376);
        out.resize(380, 7);
        return out;
    }

    void write_test_wad(const std::string &path, const char *texture)
    {
        std::vector<unsigned char> mip = miptex_bytes(texture);
        std::vector<unsigned char> out;
        out.push_back('W');
        out.push_back('A');
        out.push_back('D');
        out.push_back('3');
        binary::writer output(out);
        output.i32(1);
        output.i32(12 + (int)mip.size());
        out.insert(out.end(), mip.begin(), mip.end());
        output.i32(12);
        output.i32((int)mip.size());
        output.i32((int)mip.size());
        out.push_back(0x43);
        out.push_back(0);
        out.push_back(0);
        out.push_back(0);
        char name[16] = {};
        std::memcpy(name, texture, std::strlen(texture));
        out.insert(out.end(), name, name + 16);
        fs::write_all(path, out.data(), out.size());
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

    csg::map_source cube_map(const char *wad_path)
    {
        csg::map_source map;
        csg::map_entity world;
        world.pairs.emplace_back("classname", "worldspawn");
        world.pairs.emplace_back("wad", wad_path);
        world.num_brushes = 1;
        map.entities.push_back(world);

        add_side(map, "STONE", {0, 0, 0}, {0, 64, 0}, {0, 0, 64});
        add_side(map, "STONE", {64, 0, 0}, {64, 0, 64}, {64, 64, 0});
        add_side(map, "STONE", {0, 0, 0}, {0, 0, 64}, {64, 0, 0});
        add_side(map, "STONE", {0, 64, 0}, {64, 64, 0}, {0, 64, 64});
        add_side(map, "STONE", {0, 0, 0}, {64, 0, 0}, {0, 64, 0});
        add_side(map, "STONE", {0, 0, 64}, {0, 64, 64}, {64, 0, 64});

        csg::map_brush brush;
        brush.num_sides = 6;
        map.brushes.push_back(brush);
        return map;
    }
}

int main()
{
    bool ok = true;
    const std::filesystem::path scratch =
        test_support::scratch_directory("csg_wad_check");
    const std::string wad_path = (scratch / "textures.wad").string();
    const std::string unused_wad_path = (scratch / "unused.wad").string();
    write_test_wad(wad_path, "STONE");
    write_test_wad(unused_wad_path, "OTHER");

    format::wad_archive wad;
    ok &= expect(wad.load(wad_path), "load wad");
    ok &= expect(wad.find_texture("stone") != nullptr, "find texture case insensitive");

    csg::csg_options runtime_options;
    runtime_options.wad.wad_textures = true; // not the default: reference the wads
    csg::csg_result runtime = csg::run_csg(cube_map(wad_path.c_str()), runtime_options);
    binary::reader runtime_input(runtime.textures);
    std::int32_t runtime_count = 0;
    std::int32_t runtime_ofs = 0;
    std::int32_t runtime_mip_ofs = -1;
    ok &= expect(runtime_input.i32_at(0, runtime_count) && runtime_count == 1,
                 "runtime texture count");
    ok &= expect(runtime_input.i32_at(4, runtime_ofs), "runtime texture offset readable");
    ok &= expect((int)runtime.textures.size() == runtime_ofs + (int)sizeof(format::miptex_t), "runtime header size");
    ok &= expect(runtime_input.i32_at((size_t)runtime_ofs + 24, runtime_mip_ofs)
                     && runtime_mip_ofs == 0,
                 "runtime mip offset zeroed");
    ok &= expect(std::string(runtime.map.entities[0].value("wad")) == "textures.wad;", "runtime wad value");
    ok &= expect(runtime.texinfos.entries()[0].info.miptex == 0, "runtime texinfo remap");

    csg::csg_options embedded_options; // embedding is the default
    csg::csg_result embedded = csg::run_csg(cube_map(wad_path.c_str()), embedded_options);
    binary::reader embedded_input(embedded.textures);
    std::int32_t embedded_ofs = 0;
    std::int32_t embedded_mip_ofs = 0;
    ok &= expect(embedded_input.i32_at(4, embedded_ofs), "embedded texture offset readable");
    ok &= expect((int)embedded.textures.size() == embedded_ofs + 380, "embedded texture size");
    ok &= expect(embedded_input.i32_at((size_t)embedded_ofs + 24, embedded_mip_ofs)
                     && embedded_mip_ofs == 40,
                 "embedded mip offset kept");
    ok &= expect(std::string(embedded.map.entities[0].value("wad")).empty(), "embedded wad value removed");

    const std::string cfg_path = (scratch / "textures.cfg").string();
    std::string cfg = "include \"" + wad_path + "\"\n";
    fs::write_all(cfg_path, cfg.data(), cfg.size());
    csg::csg_options cfg_options;
    cfg_options.wad.wad_cfg_file = cfg_path;
    csg::csg_result cfg_result = csg::run_csg(cube_map(""), cfg_options);
    binary::reader cfg_input(cfg_result.textures);
    std::int32_t cfg_ofs = 0;
    std::int32_t cfg_mip_ofs = 0;
    ok &= expect(cfg_input.i32_at(4, cfg_ofs)
                     && cfg_input.i32_at((size_t)cfg_ofs + 24, cfg_mip_ofs)
                     && cfg_mip_ofs == 40,
                 "wadcfg include embeds texture");
    ok &= expect(std::string(cfg_result.map.entities[0].value("wad")).empty(), "wadcfg include removes wad value");

    csg::csg_options no_auto_options;
    // the wad list only survives in the bsp when textures are referenced rather
    // than embedded, so pruning is observable only in that mode
    no_auto_options.wad.wad_textures = true;
    no_auto_options.wad.wad_auto_detect = false;
    std::string two_wads = wad_path + ";" + unused_wad_path;
    csg::csg_result no_auto = csg::run_csg(cube_map(two_wads.c_str()), no_auto_options);
    ok &= expect(std::string(no_auto.map.entities[0].value("wad")) == "textures.wad;unused.wad;",
                 "no autodetect keeps unused wad");

    if (ok)
        std::printf("csg_wad_check passed\n");
    return ok ? 0 : 1;
}
