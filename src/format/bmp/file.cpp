#include "file.h"

#include <algorithm>
#include <climits>
#include <cstdint>
#include <cstring>

#include "common/binary.h"
#include "common/filesystem.h"

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

    bool load_indexed_bmp(const std::string &path, indexed_image &image, std::string *error)
    {
        image = indexed_image{};
        std::vector<byte> data;
        if (!fs::read_all(path, data))
        {
            set_error(error, "could not open BMP file");
            return false;
        }
        if (data.size() < 54 || data[0] != 'B' || data[1] != 'M')
        {
            set_error(error, "file is not a Windows BMP");
            return false;
        }
        binary::reader input(data);
        std::uint32_t pixel_offset;
        std::uint32_t dib_size;
        std::int32_t width;
        std::int32_t signed_height;
        std::uint16_t planes;
        std::uint16_t bits_per_pixel;
        std::uint32_t compression;
        if (!input.seek(10) || !input.u32(pixel_offset) || !input.u32(dib_size)
            || !input.i32(width) || !input.i32(signed_height)
            || !input.u16(planes) || !input.u16(bits_per_pixel)
            || !input.u32(compression))
        {
            set_error(error, "BMP header is truncated");
            return false;
        }
        if (dib_size < 40 || (size_t)14 + dib_size > data.size())
        {
            set_error(error, "BMP DIB header is unsupported");
            return false;
        }
        if (width <= 0 || signed_height == 0 || signed_height == INT_MIN
            || planes != 1 || bits_per_pixel != 8 || compression != 0)
        {
            set_error(error, "BMP must be uncompressed, indexed 8-bit colour");
            return false;
        }
        unsigned height = (unsigned)(signed_height < 0 ? -signed_height : signed_height);
        std::uint32_t colors;
        if (!input.seek(46) || !input.u32(colors))
        {
            set_error(error, "BMP DIB header is truncated");
            return false;
        }
        if (!colors)
            colors = 256;
        if (colors > 256)
        {
            set_error(error, "BMP palette contains more than 256 colours");
            return false;
        }
        size_t palette_offset = (size_t)14 + dib_size;
        if ((size_t)colors * 4 > data.size() - palette_offset)
        {
            set_error(error, "BMP palette is truncated");
            return false;
        }
        if ((size_t)pixel_offset < palette_offset + (size_t)colors * 4)
        {
            set_error(error, "BMP pixel data overlaps its palette");
            return false;
        }
        size_t stride = ((size_t)width + 3) & ~(size_t)3;
        if ((size_t)height > SIZE_MAX / stride || pixel_offset > data.size()
            || stride * height > data.size() - pixel_offset)
        {
            set_error(error, "BMP pixel data is truncated");
            return false;
        }

        image.width = (unsigned)width;
        image.height = height;
        image.pixels.resize((size_t)image.width * image.height);
        for (unsigned i = 0; i < colors; i++)
        {
            image.palette[i][0] = data[palette_offset + (size_t)i * 4 + 2];
            image.palette[i][1] = data[palette_offset + (size_t)i * 4 + 1];
            image.palette[i][2] = data[palette_offset + (size_t)i * 4 + 0];
        }
        for (unsigned y = 0; y < image.height; y++)
        {
            unsigned source_y = signed_height < 0 ? y : image.height - 1 - y;
            std::memcpy(image.pixels.data() + (size_t)y * image.width,
                        data.data() + pixel_offset + (size_t)source_y * stride,
                        image.width);
        }
        return true;
    }

    bool write_indexed_bmp(const std::string &path, const mip_texture &texture,
                           std::string *error)
    {
        if (texture.data.size() < sizeof(miptex_t))
        {
            set_error(error, "texture miptex header is missing");
            return false;
        }
        miptex_t header;
        std::memcpy(&header, texture.data.data(), sizeof(header));
        size_t pixels = (size_t)header.width * header.height;
        size_t pixel_offset_in_lump = header.offsets[0];
        size_t palette_offset = header.offsets[3]
            + ((size_t)header.width >> 3) * ((size_t)header.height >> 3);
        if (!header.width || !header.height || pixel_offset_in_lump > texture.data.size()
            || pixels > texture.data.size() - pixel_offset_in_lump
            || palette_offset > texture.data.size() || texture.data.size() - palette_offset < 2)
        {
            set_error(error, "texture miptex data is invalid");
            return false;
        }
        binary::reader texture_input(texture.data);
        std::uint16_t colors;
        if (!texture_input.seek(palette_offset) || !texture_input.u16(colors))
        {
            set_error(error, "texture palette is invalid");
            return false;
        }
        palette_offset += 2;
        if (!colors || colors > 256 || (size_t)colors * 3 > texture.data.size() - palette_offset)
        {
            set_error(error, "texture palette is invalid");
            return false;
        }

        size_t stride = ((size_t)header.width + 3) & ~(size_t)3;
        size_t bmp_pixels = stride * header.height;
        constexpr size_t bmp_header = 14 + 40 + 256 * 4;
        if (bmp_pixels > UINT_MAX - bmp_header)
        {
            set_error(error, "texture is too large for BMP output");
            return false;
        }
        std::vector<byte> out(bmp_header + bmp_pixels, 0);
        binary::writer output(out);
        out[0] = 'B';
        out[1] = 'M';
        if (!output.patch_u32(2, (unsigned)out.size())
            || !output.patch_u32(10, (unsigned)bmp_header)
            || !output.patch_u32(14, 40)
            || !output.patch_u32(18, header.width)
            || !output.patch_u32(22, header.height)
            || !output.patch_u16(26, 1)
            || !output.patch_u16(28, 8)
            || !output.patch_u32(34, (unsigned)bmp_pixels)
            || !output.patch_u32(46, 256))
        {
            set_error(error, "could not build BMP header");
            return false;
        }
        for (unsigned i = 0; i < colors; i++)
        {
            out[54 + (size_t)i * 4 + 0] = texture.data[palette_offset + (size_t)i * 3 + 2];
            out[54 + (size_t)i * 4 + 1] = texture.data[palette_offset + (size_t)i * 3 + 1];
            out[54 + (size_t)i * 4 + 2] = texture.data[palette_offset + (size_t)i * 3 + 0];
        }
        const byte *source = texture.data.data() + pixel_offset_in_lump;
        for (unsigned y = 0; y < header.height; y++)
        {
            std::memcpy(out.data() + bmp_header + (size_t)(header.height - 1 - y) * stride,
                        source + (size_t)y * header.width, header.width);
        }
        if (!fs::write_all(path, out.data(), out.size()))
        {
            set_error(error, "could not write BMP file");
            return false;
        }
        return true;
    }

    bool write_rgb_bmp(const std::string &path, unsigned width, unsigned height,
                       const std::vector<byte> &pixels, std::string *error)
    {
        if (!width || !height || width > INT_MAX || height > INT_MAX)
        {
            set_error(error, "RGB image has invalid dimensions");
            return false;
        }
        size_t row_bytes = (size_t)width * 3;
        if (row_bytes / 3 != width || (size_t)height > SIZE_MAX / row_bytes
            || pixels.size() != row_bytes * height)
        {
            set_error(error, "RGB image pixel data has the wrong size");
            return false;
        }
        size_t stride = (row_bytes + 3) & ~(size_t)3;
        constexpr size_t header_size = 14 + 40;
        if ((size_t)height > SIZE_MAX / stride || stride * height > UINT_MAX - header_size)
        {
            set_error(error, "RGB image is too large for BMP output");
            return false;
        }

        size_t image_bytes = stride * height;
        std::vector<byte> out(header_size + image_bytes, 0);
        binary::writer output(out);
        out[0] = 'B';
        out[1] = 'M';
        if (!output.patch_u32(2, (unsigned)out.size())
            || !output.patch_u32(10, (unsigned)header_size)
            || !output.patch_u32(14, 40)
            || !output.patch_u32(18, width)
            || !output.patch_u32(22, height)
            || !output.patch_u16(26, 1)
            || !output.patch_u16(28, 24)
            || !output.patch_u32(34, (unsigned)image_bytes))
        {
            set_error(error, "could not build BMP header");
            return false;
        }

        for (unsigned y = 0; y < height; y++)
        {
            const byte *source = pixels.data() + (size_t)y * row_bytes;
            byte *destination = out.data() + header_size + (size_t)(height - 1 - y) * stride;
            for (unsigned x = 0; x < width; x++)
            {
                destination[(size_t)x * 3 + 0] = source[(size_t)x * 3 + 2];
                destination[(size_t)x * 3 + 1] = source[(size_t)x * 3 + 1];
                destination[(size_t)x * 3 + 2] = source[(size_t)x * 3 + 0];
            }
        }
        if (!fs::write_all(path, out.data(), out.size()))
        {
            set_error(error, "could not write BMP file");
            return false;
        }
        return true;
    }
}
