#pragma once

#include <array>
#include <string>
#include <vector>

#include "common/types.h"
#include "math/vector.h"

namespace format
{
    struct source_phy_convex
    {
        std::vector<math::vec3<double>> vertices;
        std::vector<std::array<unsigned, 3>> triangles;
    };

    struct source_phy_model
    {
        std::vector<source_phy_convex> convexes;
    };

    // reads source's compact IVP collision tree. vertices are converted into
    // source/goldsrc model-local coordinates using IVP metres and axis order.
    bool load_source_phy(const std::vector<byte> &data, source_phy_model &out,
                         std::string *error = nullptr);
}
