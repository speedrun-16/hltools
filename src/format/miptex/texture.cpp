#include "texture.h"

#include <climits>
#include <cstring>

#include "format/bmp/image.h"

namespace format
{
    namespace
    {
        void set_error(std::string *error, const char *message)
        {
            if (error)
                *error = message;
        }

        byte nearest_colour(const indexed_image &image, int r, int g, int b,
                            bool transparent)
        {
            int limit = transparent ? 255 : 256;
            int best = 0;
            int best_distance = INT_MAX;
            for (int i = 0; i < limit; i++)
            {
                int dr = r - image.palette[(size_t)i][0];
                int dg = g - image.palette[(size_t)i][1];
                int db = b - image.palette[(size_t)i][2];
                int distance = dr * dr + dg * dg + db * db;
                if (distance < best_distance)
                {
                    best = i;
                    best_distance = distance;
                    if (!distance)
                        break;
                }
            }
            return (byte)best;
        }

        std::vector<byte> next_mip(const std::vector<byte> &source, unsigned width,
                                   unsigned height, const indexed_image &image,
                                   bool transparent)
        {
            unsigned next_width = width / 2;
            unsigned next_height = height / 2;
            std::vector<byte> out((size_t)next_width * next_height);
            for (unsigned y = 0; y < next_height; y++)
            {
                for (unsigned x = 0; x < next_width; x++)
                {
                    int r = 0, g = 0, b = 0, opaque = 0, transparent_count = 0;
                    for (unsigned dy = 0; dy < 2; dy++)
                    {
                        for (unsigned dx = 0; dx < 2; dx++)
                        {
                            byte index = source[(size_t)(y * 2 + dy) * width + x * 2 + dx];
                            if (transparent && index == 255)
                            {
                                transparent_count++;
                                continue;
                            }
                            r += image.palette[index][0];
                            g += image.palette[index][1];
                            b += image.palette[index][2];
                            opaque++;
                        }
                    }
                    if (transparent && transparent_count >= 2)
                        out[(size_t)y * next_width + x] = 255;
                    else if (opaque)
                        out[(size_t)y * next_width + x] =
                            nearest_colour(image, r / opaque, g / opaque, b / opaque, transparent);
                    else
                        out[(size_t)y * next_width + x] = 255;
                }
            }
            return out;
        }
    }

    bool decode_mip_texture(const std::vector<byte> &data, mip_texture &texture,
                            std::string *error)
    {
        if (data.size() < sizeof(miptex_t))
        {
            set_error(error, "texture lump is smaller than its miptex header");
            return false;
        }

        miptex_t header;
        std::memcpy(&header, data.data(), sizeof(header));
        char name[max_miptex_name + 1] = {};
        std::memcpy(name, header.name, max_miptex_name);
        if (!name[0])
        {
            set_error(error, "texture has an empty name");
            return false;
        }
        if (!header.width || !header.height)
        {
            set_error(error, "texture has zero dimensions");
            return false;
        }

        size_t last_end = sizeof(miptex_t);
        for (int level = 0; level < mip_levels; level++)
        {
            size_t width = (size_t)header.width >> level;
            size_t height = (size_t)header.height >> level;
            if (!width || !height || width > SIZE_MAX / height)
            {
                set_error(error, "texture has invalid mip dimensions");
                return false;
            }
            size_t offset = header.offsets[level];
            size_t count = width * height;
            if (offset < sizeof(miptex_t) || offset > data.size()
                || count > data.size() - offset)
            {
                set_error(error, "texture mip level is outside its lump");
                return false;
            }
            last_end = offset + count;
        }
        if (last_end > data.size() || data.size() - last_end < 2)
        {
            set_error(error, "texture palette header is missing");
            return false;
        }
        unsigned palette_count = (unsigned)data[last_end]
            | ((unsigned)data[last_end + 1] << 8);
        if (!palette_count || palette_count > 256
            || (size_t)palette_count * 3 > data.size() - last_end - 2)
        {
            set_error(error, "texture palette is invalid");
            return false;
        }

        texture.name = name;
        texture.width = header.width;
        texture.height = header.height;
        texture.data = data;
        return true;
    }

    bool build_mip_texture(const std::string &name, const indexed_image &image,
                           mip_texture &texture, std::string *error)
    {
        if (name.empty() || name.size() >= max_miptex_name)
        {
            set_error(error, "texture name must contain 1 to 15 characters");
            return false;
        }
        if (image.width < 16 || image.height < 16 || image.width > 512 || image.height > 512
            || image.width % 16 || image.height % 16
            || image.pixels.size() != (size_t)image.width * image.height)
        {
            set_error(error, "texture dimensions must be multiples of 16 from 16 to 512");
            return false;
        }

        std::vector<std::vector<byte>> mips;
        mips.push_back(image.pixels);
        bool transparent = name[0] == '{';
        unsigned width = image.width;
        unsigned height = image.height;
        for (int level = 1; level < mip_levels; level++)
        {
            mips.push_back(next_mip(mips.back(), width, height, image, transparent));
            width /= 2;
            height /= 2;
        }

        size_t total = sizeof(miptex_t) + 2 + 256 * 3 + 2;
        for (const auto &mip : mips)
            total += mip.size();
        texture = mip_texture{};
        texture.name = name;
        texture.width = image.width;
        texture.height = image.height;
        texture.data.resize(total, 0);

        miptex_t header{};
        std::memcpy(header.name, name.data(), name.size());
        header.width = image.width;
        header.height = image.height;
        size_t offset = sizeof(miptex_t);
        for (int level = 0; level < mip_levels; level++)
        {
            header.offsets[level] = (unsigned)offset;
            std::memcpy(texture.data.data() + offset, mips[(size_t)level].data(),
                        mips[(size_t)level].size());
            offset += mips[(size_t)level].size();
        }
        std::memcpy(texture.data.data(), &header, sizeof(header));
        texture.data[offset++] = 0;
        texture.data[offset++] = 1;
        for (const auto &colour : image.palette)
        {
            texture.data[offset++] = colour[0];
            texture.data[offset++] = colour[1];
            texture.data[offset++] = colour[2];
        }
        return true;
    }
}
