#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "common/types.h"

namespace format
{
    // read only view over a source map's embedded pakfile (a zip archive in the
    // PAKFILE lump). entries are indexed by lowercased name for case insensitive
    // lookup, matching how source resolves material paths. only stored (method 0)
    // entries are supported; a deflate entry fails extraction with a clear error.
    class pakfile
    {
    public:
        // parses the central directory of an in-memory zip. returns false with an
        // error only on a malformed archive; an empty archive parses to empty.
        bool open(std::vector<byte> archive, std::string *error = nullptr);

        bool contains(const std::string &name) const;

        // extracts a stored entry by name (case insensitive). returns false if the
        // name is absent or the entry is compressed.
        bool extract(const std::string &name, std::vector<byte> &out,
                     std::string *error = nullptr) const;

        std::size_t size() const { return entries_.size(); }

    private:
        struct entry
        {
            unsigned local_header_offset = 0;
            unsigned compressed_size = 0;
            unsigned uncompressed_size = 0;
            unsigned method = 0;
        };

        std::vector<byte> archive_;
        std::unordered_map<std::string, entry> entries_;
    };
}
