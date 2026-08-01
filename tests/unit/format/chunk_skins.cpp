#include <vector>
#include <utility>

#include "format/mdl/chunk_skins.h"
#include "support/test.h"

namespace
{
    format::studio_vertex vertex(float x, float y, float u, float v)
    {
        format::studio_vertex out;
        out.position[0] = x;
        out.position[1] = y;
        out.normal[2] = 1;
        out.u = u;
        out.v = v;
        return out;
    }

    format::studio_model model_with_triangles(
        const std::vector<format::studio_vertex> &vertices)
    {
        format::studio_model model;
        model.materials.push_back("material");
        model.vertices = vertices;
        format::studio_mesh mesh;
        mesh.material = 0;
        for (std::size_t i = 0; i < vertices.size(); i++)
            mesh.indices.push_back((int)i);
        model.meshes.push_back(std::move(mesh));
        return model;
    }
}

suite("unit.format.chunk_skins")
{
    test("chunk_skins.splits rectangular layouts on the long axis only")
    {
        format::studio_model model = model_with_triangles({
            vertex(0, 0, 0, 0), vertex(2, 0, 1, 0), vertex(0, 1, 0, 1),
        });
        format::tile_layout layout;
        layout.count_u = 2;
        layout.core_u = 0.5f;
        layout.inset_u = 1.0f / 512.0f;

        std::vector<format::skin_chunk> chunks;
        format::chunk_model_skins(model, {layout}, 1.0f, chunks);

        require(chunks.size() == 2);
        expect(chunks[0].count_u == 2);
        expect(chunks[0].count_v == 1);
        expect(chunks[1].count_u == 2);
        expect(chunks[1].count_v == 1);
        expect(model.meshes.size() == 2);
        expect(model.triangle_count() >= 2);
    }

    test("chunk_skins.keeps tile zero separate from the coarse fallback")
    {
        // the large triangle ranks tile 0 first. the tiny triangle in tile 1
        // falls beyond the 50% area budget and must use a distinct whole-image
        // skin instead of colliding with tiled material (0,0).
        format::studio_model model = model_with_triangles({
            vertex(0, 0, 0.05f, 0.1f), vertex(20, 0, 0.45f, 0.1f),
            vertex(0, 20, 0.05f, 0.9f),
            vertex(30, 0, 0.6f, 0.1f), vertex(31, 0, 0.9f, 0.1f),
            vertex(30, 1, 0.6f, 0.9f),
        });
        format::tile_layout layout;
        layout.count_u = 2;
        layout.core_u = 0.5f;

        std::vector<format::skin_chunk> chunks;
        format::chunk_model_skins(model, {layout}, 0.5f, chunks);

        require(chunks.size() == 2);
        bool tiled_zero = false;
        bool fallback = false;
        for (const format::skin_chunk &chunk : chunks)
        {
            tiled_zero |= chunk.count_u == 2 && chunk.tile_u == 0;
            fallback |= chunk.count_u == 1 && chunk.count_v == 1;
        }
        expect(tiled_zero);
        expect(fallback);
    }

    test("chunk_skins.preserves triangles with wrapped uvs on the fallback")
    {
        format::studio_model model = model_with_triangles({
            vertex(0, 0, -0.2f, 0.1f), vertex(1, 0, 0.2f, 0.1f),
            vertex(0, 1, -0.2f, 0.9f),
        });
        format::tile_layout layout;
        layout.count_u = 2;
        layout.core_u = 0.5f;

        std::vector<format::skin_chunk> chunks;
        format::chunk_model_skins(model, {layout}, 1.0f, chunks);

        require(chunks.size() == 1);
        expect(chunks[0].count_u == 1);
        expect(model.triangle_count() == 1);
        require(model.vertices.size() == 3);
        expect(model.vertices[0].u == -0.2f);
    }
}
