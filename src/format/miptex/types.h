#pragma once

namespace format
{
    constexpr int mip_levels = 4;

    struct miptex_t
    {
        char name[16];
        unsigned width, height;
        unsigned offsets[mip_levels];
    };

    static_assert(sizeof(miptex_t) == 40, "miptex_t layout");
}
