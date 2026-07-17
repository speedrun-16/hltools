#pragma once

// writes a minimal wad3 fixture so run_csg's texture resolution succeeds
// (a used texture missing from every wad is a fatal error, like the reference)

#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "common/binary.h"
#include "common/filesystem.h"

namespace test_wad
{
    inline void require_patch(bool ok)
    {
        if (!ok)
            std::abort();
    }

    // a 16x16 miptex with all four mip levels present
    inline std::vector<unsigned char> miptex_bytes(const char *name)
    {
        std::vector<unsigned char> out(40, 0);
        binary::writer output(out);
        std::memcpy(out.data(), name, std::strlen(name));
        require_patch(output.patch_i32(16, 16));
        require_patch(output.patch_i32(20, 16));
        require_patch(output.patch_i32(24, 40));
        require_patch(output.patch_i32(28, 296));
        require_patch(output.patch_i32(32, 360));
        require_patch(output.patch_i32(36, 376));
        out.resize(380, 7);
        return out;
    }

    inline void write(const std::string &path, const std::vector<const char *> &textures)
    {
        std::vector<unsigned char> out;
        out.push_back('W');
        out.push_back('A');
        out.push_back('D');
        out.push_back('3');
        binary::writer output(out);
        output.i32((int)textures.size());
        output.i32(0); // info table offset, patched below

        std::vector<unsigned char> infos;
        binary::writer info_output(infos);
        for (const char *texture : textures)
        {
            std::vector<unsigned char> mip = miptex_bytes(texture);
            info_output.i32((int)out.size());
            info_output.i32((int)mip.size());
            info_output.i32((int)mip.size());
            infos.push_back(0x43);
            infos.push_back(0);
            infos.push_back(0);
            infos.push_back(0);
            char name[16] = {};
            std::memcpy(name, texture, std::strlen(texture));
            infos.insert(infos.end(), name, name + 16);
            out.insert(out.end(), mip.begin(), mip.end());
        }
        require_patch(output.patch_i32(8, (int)out.size()));
        out.insert(out.end(), infos.begin(), infos.end());
        fs::write_all(path, out.data(), out.size());
    }
}
