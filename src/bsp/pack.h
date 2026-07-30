#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace bsp
{
    struct pack_options
    {
        std::string game_dir;
        std::vector<std::string> base_dirs;
        bool force = false;
        bool strict = false;
    };

    struct pack_result
    {
        std::vector<std::string> resources;
        std::vector<std::string> missing;
        std::vector<std::string> provided_by_base;
        std::size_t copied = 0;
        std::size_t unchanged = 0;
        std::string res_path;
    };

    // copies a compiled bsp and every discoverable external dependency using
    // game relative paths; a .zip output creates an archive, otherwise a folder
    // maps/<name>.res is generated beside the packed bsp
    bool pack_map(const std::string &bsp_path, const std::string &output,
                  const pack_options &options, pack_result &out,
                  std::string *error = nullptr);
}
