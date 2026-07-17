#pragma once

#include <array>
#include <vector>

#include "common/types.h"

namespace format
{
    struct indexed_image
    {
        unsigned width = 0;
        unsigned height = 0;
        std::vector<byte> pixels; // top down, one palette index per pixel
        std::array<std::array<byte, 3>, 256> palette{}; // rgb
    };
}
