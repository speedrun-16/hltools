#pragma once

#include <string>
#include <vector>

#include "common/types.h"

namespace format
{
    struct map_data;

    struct lightmap_atlas
    {
        unsigned width = 0;
        unsigned height = 0;
        int faces = 0;
        int tiles = 0;
        std::vector<byte> pixels; // top down rgb8
    };

    // packs every lit face/style into a deterministic, one pixel separated
    // atlas in face then style slot order
    bool build_lightmap_atlas(const map_data &map, lightmap_atlas &atlas,
                              std::string *error = nullptr);
}
