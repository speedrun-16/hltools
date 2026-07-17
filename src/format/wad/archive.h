#pragma once

#include <string>
#include <vector>

#include "common/types.h"
#include "format/miptex/texture.h"

namespace format
{
    constexpr int max_wad_name = 16;
    constexpr char wad_miptex_type = 0x43;

    struct wad_lump
    {
        int filepos = 0;
        int disksize = 0;
        int size = 0;
        char type = 0;
        char compression = 0;
        char pad1 = 0;
        char pad2 = 0;
        char name[max_wad_name] = {};
        std::vector<byte> data;
    };

    class wad_archive
    {
    public:
        bool load(const std::string &path, std::string *error = nullptr);

        const std::string &path() const {
            return path_;
        }
        const std::vector<wad_lump> &lumps() const {
            return lumps_;
        }
        const wad_lump *find_texture(const char *name) const;

    private:
        std::string path_;
        std::vector<wad_lump> lumps_;
    };

    bool mip_texture_from_lump(const wad_lump &lump, mip_texture &out,
                               std::string *error = nullptr);
    bool write_wad3(const std::string &path, const std::vector<mip_texture> &textures,
                    std::string *error = nullptr);
}
