#pragma once

#include <string>
#include <vector>

#include "../common/types.h"
#include "map_parser.h"
#include "textures.h"

namespace csg
{
    struct wad_build_options
    {
        bool wad_textures = true;
        bool wad_auto_detect = true;
        std::vector<std::string> wad_include = {"hltools.wad"};
        std::string wad_cfg_file;
        std::string wad_config_name;
        std::string map_path;
    };

    struct miptex_build_result
    {
        std::vector<byte> textures;
        std::vector<int> texinfo_miptex;
        std::string wad_value;

        // the <map>wa_ temp wad: full lump data of every runtime loaded
        // texture, which hlrad reads to light textured surfaces
        std::vector<byte> temp_wad;
    };

    miptex_build_result build_miptex_lump(const map_source &map,
                                          const texinfo_store &texinfos,
                                          const wad_build_options &options);
}
