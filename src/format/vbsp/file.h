#pragma once

#include <string>
#include <vector>

#include "common/types.h"
#include "data.h"

namespace format
{
    // reads a source engine (vbsp) map. returns false and fills error on a bad
    // ident, unsupported version, malformed lump, or an lzma compressed lump
    // (compression support is deferred; the message names the offending lump).
    struct source_bsp_file
    {
        static bool load(const std::string &path, source_map_data &out,
                         std::string *error = nullptr);

        // same, over an already loaded file image. keeps the parser testable
        // without touching the filesystem.
        static bool parse(const std::vector<byte> &file, source_map_data &out,
                          std::string *error = nullptr);
    };
}
