#include "texture_lump.h"

#include <cstring>

#include "common/binary.h"
#include "types.h"

namespace format
{
    namespace
    {
        void set_error(std::string *error, const char *message)
        {
            if (error)
                *error = message;
        }
    }

    bool collect_bsp_textures(const map_data &map, std::vector<mip_texture> &textures,
                              std::vector<std::string> *external, std::string *error)
    {
        textures.clear();
        if (external)
            external->clear();
        const std::vector<byte> &data = map.textures;
        if (data.size() < sizeof(int))
        {
            set_error(error, "BSP texture lump is too small");
            return false;
        }
        binary::reader input(data);
        std::int32_t count;
        if (!input.i32(count))
        {
            set_error(error, "BSP texture lump is too small");
            return false;
        }
        if (count < 0 || (size_t)count > (data.size() - sizeof(int)) / sizeof(int))
        {
            set_error(error, "BSP texture directory is invalid");
            return false;
        }
        size_t header_size = sizeof(int) + (size_t)count * sizeof(int);
        std::vector<int> offsets((size_t)count);
        for (int i = 0; i < count; i++)
        {
            if (!input.i32(offsets[(size_t)i]))
            {
                set_error(error, "BSP texture directory is truncated");
                return false;
            }
        }

        for (int i = 0; i < count; i++)
        {
            int raw_offset = offsets[(size_t)i];
            if (raw_offset < 0)
                continue;
            size_t offset = (size_t)raw_offset;
            if (offset < header_size || offset > data.size()
                || data.size() - offset < sizeof(miptex_t))
            {
                set_error(error, "BSP texture entry is outside the texture lump");
                return false;
            }

            miptex_t header;
            std::memcpy(&header, data.data() + offset, sizeof(header));
            char name[max_miptex_name + 1] = {};
            std::memcpy(name, header.name, max_miptex_name);
            bool embedded = false;
            for (int level = 0; level < mip_levels; level++)
                embedded = embedded || header.offsets[level] != 0;
            if (!embedded)
            {
                if (external)
                    external->emplace_back(name);
                continue;
            }

            size_t end = data.size();
            for (int candidate : offsets)
            {
                if (candidate > raw_offset && (size_t)candidate < end)
                    end = (size_t)candidate;
            }
            std::vector<byte> lump(data.begin() + offset, data.begin() + end);
            mip_texture texture;
            std::string detail;
            if (!decode_mip_texture(lump, texture, &detail))
            {
                if (error)
                    *error = std::string("BSP texture '") + name + "': " + detail;
                return false;
            }
            textures.push_back(std::move(texture));
        }
        return true;
    }
}
