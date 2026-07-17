#include <cstdio>
#include <cstring>
#include <filesystem>

#include "common/binary.h"
#include "common/filesystem.h"
#include "format/bmp/file.h"
#include "format/bsp/data.h"
#include "format/bsp/file.h"
#include "format/bsp/lightmap_atlas.h"

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
        std::printf("usage: lightmap_export_check <scratch-directory>\n");
        return 2;
    }
    stdfs::path scratch(argv[1]);
    std::error_code ec;
    stdfs::create_directories(scratch, ec);

    format::map_data map;
    map.vertexes.resize(4);
    float points[4][3] = {{0, 0, 0}, {16, 0, 0}, {16, 16, 0}, {0, 16, 0}};
    for (int i = 0; i < 4; i++)
        std::memcpy(map.vertexes[(size_t)i].point, points[i], sizeof(points[i]));
    map.edges.resize(4);
    map.edges[0].v[0] = 0; map.edges[0].v[1] = 1;
    map.edges[1].v[0] = 1; map.edges[1].v[1] = 2;
    map.edges[2].v[0] = 2; map.edges[2].v[1] = 3;
    map.edges[3].v[0] = 3; map.edges[3].v[1] = 0;
    map.surfedges = {0, 1, 2, 3};
    map.texinfo.resize(1);
    map.texinfo[0].vecs[0][0] = 1;
    map.texinfo[0].vecs[1][1] = 1;
    map.texinfo[0].miptex = -1;
    map.faces.resize(1);
    map.faces[0].firstedge = 0;
    map.faces[0].numedges = 4;
    map.faces[0].texinfo = 0;
    map.faces[0].styles[0] = 0;
    map.faces[0].styles[1] = 32;
    map.faces[0].styles[2] = 255;
    map.faces[0].styles[3] = 255;
    map.faces[0].lightofs = 0;
    map.lighting.resize(24);
    for (size_t i = 0; i < map.lighting.size(); i++)
        map.lighting[i] = (byte)(i + 1);

    format::lightmap_atlas atlas;
    std::string error;
    expect(format::build_lightmap_atlas(map, atlas, &error),
           "build deterministic lightmap atlas");
    expect(atlas.width == 1024 && atlas.height == 4,
           "atlas has fixed comparison width and cropped height");
    expect(atlas.faces == 1 && atlas.tiles == 2,
           "all face style slots become atlas tiles");
    size_t first = ((size_t)1 * atlas.width + 1) * 3;
    size_t second = ((size_t)1 * atlas.width + 4) * 3;
    expect(atlas.pixels[first] == 1 && atlas.pixels[first + 1] == 2
               && atlas.pixels[first + 2] == 3,
           "style 0 RGB bytes copied without conversion");
    expect(atlas.pixels[second] == 13 && atlas.pixels[second + 1] == 14
               && atlas.pixels[second + 2] == 15,
           "additional style RGB bytes copied without conversion");

    stdfs::path bmp_path = scratch / "lightmaps.bmp";
    expect(format::write_rgb_bmp(bmp_path.string(), atlas.width, atlas.height,
                                 atlas.pixels, &error),
           "write 24-bit BMP atlas");
    std::vector<byte> bmp;
    expect(fs::read_all(bmp_path.string(), bmp), "read exported BMP");
    binary::reader bmp_input(bmp);
    std::uint16_t bits_per_pixel = 0;
    std::uint32_t stored_width = 0;
    std::uint32_t stored_height = 0;
    expect(bmp.size() >= 54 && bmp[0] == 'B' && bmp[1] == 'M'
               && bmp_input.u16_at(28, bits_per_pixel) && bits_per_pixel == 24,
           "BMP header declares 24-bit RGB data");
    expect(bmp_input.u32_at(18, stored_width) && bmp_input.u32_at(22, stored_height)
               && stored_width == atlas.width && stored_height == atlas.height,
           "BMP dimensions match the atlas");
    size_t stride = ((size_t)atlas.width * 3 + 3) & ~(size_t)3;
    size_t stored = 54 + (size_t)(atlas.height - 1 - 1) * stride + 3;
    expect(bmp[stored] == 3 && bmp[stored + 1] == 2 && bmp[stored + 2] == 1,
           "BMP stores exact RGB sample using standard BGR byte order");

    stdfs::path bsp_path = scratch / "lightmaps.bsp";
    expect(format::bsp_file::write(bsp_path.string(), map),
           "write BSP fixture for command-level export check");

    std::printf("%s (%d failures)\n", failures ? "FAILED" : "PASSED", failures);
    return failures ? 1 : 0;
}
