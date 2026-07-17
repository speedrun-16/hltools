#pragma once

#include <string>
#include <vector>

#include "data.h"
#include "format/miptex/texture.h"

namespace format
{
    // read embedded miptex payloads and external texture names from a bsp
    // texture lump
    bool collect_bsp_textures(const map_data &map, std::vector<mip_texture> &textures,
                              std::vector<std::string> *external = nullptr,
                              std::string *error = nullptr);
}
