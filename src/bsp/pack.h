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

    // Copies a compiled BSP and every discoverable external dependency into
    // output_root using the same paths they have below the game directory.
    // maps/<name>.res is generated beside the packed BSP.
    bool pack_map(const std::string &bsp_path, const std::string &output_root,
                  const pack_options &options, pack_result &out,
                  std::string *error = nullptr);
}
