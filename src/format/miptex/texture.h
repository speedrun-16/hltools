#pragma once

#include <string>
#include <vector>

#include "common/types.h"
#include "types.h"

namespace format
{
    constexpr int max_miptex_name = 16;

    struct indexed_image;

    // a complete goldsrc miptex payload: header, four indexed mip levels and
    // palette the raw bytes are retained for lossless bsp/wad extraction
    struct mip_texture
    {
        std::string name;
        unsigned width = 0;
        unsigned height = 0;
        std::vector<byte> data;
    };

    bool decode_mip_texture(const std::vector<byte> &data, mip_texture &texture,
                            std::string *error = nullptr);
    bool build_mip_texture(const std::string &name, const indexed_image &image,
                           mip_texture &texture, std::string *error = nullptr);
}
