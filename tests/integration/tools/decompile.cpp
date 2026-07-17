#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>

#include "common/binary.h"
#include "csg/map_parser.h"
#include "decompile/map_decompiler.h"
#include "format/bsp/data.h"

namespace stdfs = std::filesystem;

namespace
{
    void expect(bool condition, const char *message)
    {
        if (!condition)
        {
            std::fprintf(stderr, "FAIL: %s\n", message);
            std::exit(1);
        }
    }

    format::map_data cube_halfspace()
    {
        format::map_data map;
        format::dmodel_t model{};
        model.mins[0] = model.mins[1] = model.mins[2] = -64;
        model.maxs[0] = model.maxs[1] = model.maxs[2] = 64;
        model.headnode[0] = 0;
        model.firstface = 0;
        model.numfaces = 1;
        map.models.push_back(model);

        format::dplane_t plane{};
        plane.normal[0] = 1;
        map.planes.push_back(plane);

        format::dnode_t node{};
        node.planenum = 0;
        node.children[0] = -2; // empty leaf 1
        node.children[1] = -1; // solid leaf 0
        map.nodes.push_back(node);

        format::dleaf_t solid{};
        solid.contents = -2;
        format::dleaf_t empty{};
        empty.contents = -1;
        map.leafs.push_back(solid);
        map.leafs.push_back(empty);

        const float points[4][3] = {
            {0, -64, -64}, {0, -64, 64}, {0, 64, 64}, {0, 64, -64},
        };
        for (const auto &point : points)
        {
            format::dvertex_t vertex{};
            std::memcpy(vertex.point, point, sizeof(point));
            map.vertexes.push_back(vertex);
        }
        for (unsigned short i = 0; i < 4; i++)
        {
            format::dedge_t edge{};
            edge.v[0] = i;
            edge.v[1] = (unsigned short)((i + 1) % 4);
            map.edges.push_back(edge);
            map.surfedges.push_back(i);
        }

        format::texinfo_t texinfo{};
        texinfo.vecs[0][1] = 1;
        texinfo.vecs[0][3] = 8;
        texinfo.vecs[1][2] = -1;
        texinfo.vecs[1][3] = 16;
        map.texinfo.push_back(texinfo);

        format::dface_t face{};
        face.planenum = 0;
        face.firstedge = 0;
        face.numedges = 4;
        face.texinfo = 0;
        map.faces.push_back(face);

        int offset = 8;
        map.textures.resize(8 + sizeof(format::miptex_t));
        binary::writer texture_output(map.textures);
        expect(texture_output.patch_i32(0, 1), "write fixture texture count");
        expect(texture_output.patch_i32(4, offset), "write fixture texture offset");
        format::miptex_t miptex{};
        std::memcpy(miptex.name, "STONE", 5);
        miptex.width = miptex.height = 64;
        std::memcpy(map.textures.data() + offset, &miptex, sizeof(miptex));
        map.entities = "{\n\"classname\" \"worldspawn\"\n}\n\0";
        return map;
    }

    format::map_data split_texture_halfspace()
    {
        format::map_data map = cube_halfspace();
        map.models[0].numfaces = 2;
        map.vertexes.clear();
        map.edges.clear();
        map.surfedges.clear();
        map.faces.clear();

        auto add_face = [&map](float y0, float y1, short texinfo)
        {
            const float points[4][3] = {
                {0, y0, -64}, {0, y0, 64}, {0, y1, 64}, {0, y1, -64},
            };
            unsigned short first_vertex = (unsigned short)map.vertexes.size();
            int first_edge = (int)map.edges.size();
            for (const auto &point : points)
            {
                format::dvertex_t vertex{};
                std::memcpy(vertex.point, point, sizeof(point));
                map.vertexes.push_back(vertex);
            }
            for (unsigned short i = 0; i < 4; i++)
            {
                format::dedge_t edge{};
                edge.v[0] = (unsigned short)(first_vertex + i);
                edge.v[1] = (unsigned short)(first_vertex + (i + 1) % 4);
                map.edges.push_back(edge);
                map.surfedges.push_back(first_edge + i);
            }
            format::dface_t face{};
            face.planenum = 0;
            face.firstedge = (int)map.surfedges.size() - 4;
            face.numedges = 4;
            face.texinfo = texinfo;
            map.faces.push_back(face);
        };
        add_face(-64, 0, 0);
        add_face(0, 64, 1);

        format::texinfo_t second = map.texinfo[0];
        second.miptex = 1;
        map.texinfo.push_back(second);

        int offsets[2] = {12, 12 + (int)sizeof(format::miptex_t)};
        map.textures.assign(12 + 2 * sizeof(format::miptex_t), 0);
        binary::writer texture_output(map.textures);
        expect(texture_output.patch_i32(0, 2), "write split fixture texture count");
        expect(texture_output.patch_i32(4, offsets[0]), "write first split texture offset");
        expect(texture_output.patch_i32(8, offsets[1]), "write second split texture offset");
        format::miptex_t stone{};
        std::memcpy(stone.name, "STONE", 5);
        stone.width = stone.height = 64;
        format::miptex_t brick{};
        std::memcpy(brick.name, "BRICK", 5);
        brick.width = brick.height = 64;
        std::memcpy(map.textures.data() + offsets[0], &stone, sizeof(stone));
        std::memcpy(map.textures.data() + offsets[1], &brick, sizeof(brick));
        return map;
    }
}

int main(int argc, char **argv)
{
    expect(argc == 2, "scratch directory argument");
    format::map_data map = cube_halfspace();
    decompile::map_result result;
    std::string error;
    expect(decompile::reconstruct_map(map, {}, result, &error), error.c_str());
    expect(result.brushes == 1, "one solid leaf becomes one brush");
    expect(result.textured_sides == 1, "visible BSP face supplies one textured side");
    expect(result.generated_sides == 5, "bounds supply five NULL sides");
    expect(result.text.find("STONE [ 0 1 0 8 ] [ 0 0 -1 16 ] 0 1 1") != std::string::npos,
           "Valve 220 axes preserve texinfo exactly");

    stdfs::path scratch = stdfs::path(argv[1]);
    stdfs::create_directories(scratch);
    stdfs::path output = scratch / "decompiled.map";
    std::FILE *file = std::fopen(output.string().c_str(), "wb");
    expect(file != nullptr, "open reconstructed map");
    expect(std::fwrite(result.text.data(), 1, result.text.size(), file) == result.text.size(),
           "write reconstructed map");
    std::fclose(file);

    csg::map_source parsed = csg::load_map_file(output.string());
    expect(parsed.map_file_version == 220, "output declares Valve 220 format");
    expect(parsed.entities.size() == 1, "output entity parses");
    expect(parsed.brushes.size() == 1, "output brush parses");
    expect(parsed.brushes[0].num_sides == 6, "output brush is a closed convex cell");

    format::map_data split_map = split_texture_halfspace();
    decompile::map_result split_result;
    expect(decompile::reconstruct_map(split_map, {}, split_result, &error), error.c_str());
    expect(split_result.brushes == 2, "a texture boundary splits the solid cell");
    expect(split_result.texture_splits == 1, "texture boundary split is reported");
    expect(split_result.textured_sides == 2, "both texture regions remain visible");
    expect(split_result.text.find("STONE") != std::string::npos
        && split_result.text.find("BRICK") != std::string::npos,
        "both texture names survive cell splitting");

    std::puts("decompile check passed");
    return 0;
}
