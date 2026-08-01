#pragma once

#include <vector>

#include "format/mdl/studio_model.h"

namespace format
{
    // how one material's uv space is divided.
    struct tile_layout
    {
        int count_u = 1, count_v = 1;
        float core_u = 1.0f, core_v = 1.0f;
        float inset_u = 0.0f, inset_v = 0.0f;
    };

    // one output skin: a rectangular tile of an original material's uv space.
    struct skin_chunk
    {
        int source_material = 0; // index into the model's ORIGINAL material list
        int count_u = 1, count_v = 1;
        int tile_u = 0, tile_v = 0;
        // uv origin and size of this tile's content region.
        float origin_u = 0, origin_v = 0;
        float core_u = 1.0f, core_v = 1.0f;
    };

    // splits triangles at tile boundaries and rewrites materials and meshes.
    // only used tiles are emitted; wrapped UVs and tiles outside area_keep use
    // one untiled fallback per material. out_chunks follows material order.
    void chunk_model_skins(studio_model &model,
                           const std::vector<tile_layout> &layout_for_material,
                           float area_keep, std::vector<skin_chunk> &out_chunks);
}
