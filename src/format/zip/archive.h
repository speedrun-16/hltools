#pragma once

#include <string>
#include <utility>
#include <vector>

#include "common/types.h"

namespace format
{
    using zip_entry = std::pair<std::string, std::vector<byte>>;
    inline constexpr const char *embedded_map_name = "source.map";

    // builds a regular deflate compressed zip in memory; the returned bytes can
    // be appended to a bsp and are intentionally independent of bsp offsets
    bool create_zip(const std::vector<zip_entry> &entries, std::vector<byte> &out,
                    std::string *error = nullptr);

    // reads one named file from an in memory zip
    bool read_zip_entry(const std::vector<byte> &archive, const std::string &name,
                        std::vector<byte> &out, std::string *error = nullptr);

    // reads source.map, falling back to the first top level .map entry so
    // decompilation still works after a zip editor renames it
    bool read_zip_map(const std::vector<byte> &archive, std::vector<byte> &out,
                      std::string *entry_name = nullptr,
                      std::string *error = nullptr);
}
