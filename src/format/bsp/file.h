#pragma once

#include <string>

namespace format
{
    struct map_data;

    // bsp v30 load and write load fills a map_data from a file; write emits one
    // back the writer reproduces the reference layout exactly (the historical
    // lump order and zero padding to 4 byte boundaries) so a load then write round trip is
    // byte identical, which compile_diffpy verifies
    namespace bsp_file
    {
        constexpr const char *embed_locator_lump = "HLTOOLS_EMBED_LOCATOR";

        bool load(const std::string &path, map_data &out);
        bool write(const std::string &path, const map_data &data);
    }
}
