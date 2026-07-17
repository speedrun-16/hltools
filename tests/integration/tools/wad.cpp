#include <cstdio>
#include <cstring>
#include <filesystem>

#include "common/binary.h"
#include "format/bmp/file.h"
#include "format/bsp/data.h"
#include "format/bsp/file.h"
#include "format/bsp/texture_lump.h"
#include "format/bsp/types.h"
#include "format/miptex/texture.h"
#include "format/wad/archive.h"

namespace stdfs = std::filesystem;

static int failures = 0;

static void expect(bool ok, const char *what)
{
    std::printf("  [%s] %s\n", ok ? "ok" : "FAIL", what);
    if (!ok)
        failures++;
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::printf("usage: wad_tools_check <scratch-directory>\n");
        return 2;
    }
    stdfs::path scratch(argv[1]);
    std::error_code ec;
    stdfs::create_directories(scratch, ec);

    format::indexed_image image;
    image.width = 16;
    image.height = 16;
    image.pixels.resize(256);
    for (size_t i = 0; i < 256; i++)
    {
        image.pixels[i] = (byte)i;
        image.palette[i][0] = (byte)i;
        image.palette[i][1] = (byte)(255 - i);
        image.palette[i][2] = (byte)(i / 2);
    }

    std::string error;
    format::mip_texture built;
    expect(format::build_mip_texture("TEST", image, built, &error),
           "build four-level miptex from indexed pixels");
    expect(built.width == 16 && built.height == 16, "miptex dimensions preserved");

    stdfs::path wad_path = scratch / "roundtrip.wad";
    expect(format::write_wad3(wad_path.string(), {built}, &error), "write WAD3 archive");
    format::wad_archive wad;
    expect(wad.load(wad_path.string(), &error), "load written WAD3 archive");
    expect(wad.lumps().size() == 1, "WAD3 contains one directory entry");
    format::mip_texture loaded;
    expect(!wad.lumps().empty() && format::mip_texture_from_lump(wad.lumps()[0], loaded, &error),
           "decode texture from WAD lump");
    expect(loaded.data == built.data, "WAD preserves the complete miptex payload");

    stdfs::path bmp_path = scratch / "TEST.bmp";
    expect(format::write_indexed_bmp(bmp_path.string(), built, &error), "extract indexed BMP");
    format::indexed_image bmp;
    expect(format::load_indexed_bmp(bmp_path.string(), bmp, &error), "load extracted indexed BMP");
    expect(bmp.width == image.width && bmp.height == image.height && bmp.pixels == image.pixels,
           "BMP preserves dimensions and palette indices");
    expect(bmp.palette == image.palette, "BMP preserves the 256-colour palette");

    format::map_data map;
    binary::writer texture_output(map.textures);
    texture_output.i32(2);
    texture_output.i32(12);
    texture_output.i32(12 + (int)built.data.size());
    map.textures.insert(map.textures.end(), built.data.begin(), built.data.end());
    format::miptex_t external{};
    std::memcpy(external.name, "EXTERNAL", 8);
    external.width = external.height = 16;
    const byte *external_bytes = reinterpret_cast<const byte *>(&external);
    map.textures.insert(map.textures.end(), external_bytes,
                        external_bytes + sizeof(external));

    std::vector<format::mip_texture> embedded;
    std::vector<std::string> external_names;
    expect(format::collect_bsp_textures(map, embedded, &external_names, &error),
           "read BSP miptex directory");
    expect(embedded.size() == 1 && embedded[0].data == built.data,
           "BSP extraction preserves embedded miptex bytes");
    expect(external_names.size() == 1 && external_names[0] == "EXTERNAL",
           "BSP extraction reports header-only external texture references");

    stdfs::path bsp_path = scratch / "embedded.bsp";
    expect(format::bsp_file::write(bsp_path.string(), map),
           "write BSP fixture for command-level extraction checks");

    std::printf("%s (%d failures)\n", failures ? "FAILED" : "PASSED", failures);
    return failures ? 1 : 0;
}
