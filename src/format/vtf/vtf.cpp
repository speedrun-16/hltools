#include "vtf.h"

#include <algorithm>
#include <cstring>

#include "common/binary.h"

namespace format
{
    namespace
    {
        enum vtf_format
        {
            fmt_rgba8888 = 0,
            fmt_abgr8888 = 1,
            fmt_rgb888 = 2,
            fmt_bgr888 = 3,
            fmt_rgb565 = 4,
            fmt_i8 = 5,
            fmt_argb8888 = 11,
            fmt_bgra8888 = 12,
            fmt_dxt1 = 13,
            fmt_dxt3 = 14,
            fmt_dxt5 = 15,
            fmt_bgrx8888 = 16,
            fmt_bgr565 = 17,
            fmt_dxt1_onebit = 20,
        };

        void set_error(std::string *error, const char *message)
        {
            if (error)
                *error = message;
        }

        bool is_dxt(int format)
        {
            return format == fmt_dxt1 || format == fmt_dxt3 || format == fmt_dxt5
                || format == fmt_dxt1_onebit;
        }

        // bytes occupied by one mip level in the given pixel format.
        std::size_t image_bytes(int format, unsigned w, unsigned h, bool &supported)
        {
            supported = true;
            switch (format)
            {
            case fmt_dxt1:
            case fmt_dxt1_onebit:
                return (std::size_t)((w + 3) / 4) * ((h + 3) / 4) * 8;
            case fmt_dxt3:
            case fmt_dxt5:
                return (std::size_t)((w + 3) / 4) * ((h + 3) / 4) * 16;
            case fmt_rgb888:
            case fmt_bgr888:
                return (std::size_t)w * h * 3;
            case fmt_rgba8888:
            case fmt_abgr8888:
            case fmt_argb8888:
            case fmt_bgra8888:
            case fmt_bgrx8888:
                return (std::size_t)w * h * 4;
            case fmt_rgb565:
            case fmt_bgr565:
                return (std::size_t)w * h * 2;
            case fmt_i8:
                return (std::size_t)w * h;
            default:
                supported = false;
                return 0;
            }
        }

        void rgb_from_565(unsigned short v, int out[3])
        {
            int r = (v >> 11) & 0x1f;
            int g = (v >> 5) & 0x3f;
            int b = v & 0x1f;
            out[0] = (r << 3) | (r >> 2);
            out[1] = (g << 2) | (g >> 4);
            out[2] = (b << 3) | (b >> 2);
        }

        // decodes one 8-byte dxt1 color block into 16 rgb texels. when opaque is
        // true (dxt3/dxt5 color blocks) the four-color interpolation is always
        // used; otherwise c0<=c1 selects the three-color-plus-black mode.
        void decode_color_block(const byte *src, bool opaque, int texel[16][3])
        {
            unsigned short c0 = (unsigned short)(src[0] | (src[1] << 8));
            unsigned short c1 = (unsigned short)(src[2] | (src[3] << 8));
            int color[4][3];
            rgb_from_565(c0, color[0]);
            rgb_from_565(c1, color[1]);
            if (opaque || c0 > c1)
            {
                for (int k = 0; k < 3; k++)
                {
                    color[2][k] = (2 * color[0][k] + color[1][k]) / 3;
                    color[3][k] = (color[0][k] + 2 * color[1][k]) / 3;
                }
            }
            else
            {
                for (int k = 0; k < 3; k++)
                {
                    color[2][k] = (color[0][k] + color[1][k]) / 2;
                    color[3][k] = 0;
                }
            }
            unsigned bits = (unsigned)src[4] | ((unsigned)src[5] << 8)
                | ((unsigned)src[6] << 16) | ((unsigned)src[7] << 24);
            for (int i = 0; i < 16; i++)
            {
                int idx = (bits >> (2 * i)) & 3;
                texel[i][0] = color[idx][0];
                texel[i][1] = color[idx][1];
                texel[i][2] = color[idx][2];
            }
        }

        void put_texel(std::vector<byte> &rgb, unsigned w, unsigned h, unsigned x,
                       unsigned y, int r, int g, int b)
        {
            if (x >= w || y >= h)
                return;
            std::size_t o = ((std::size_t)y * w + x) * 3;
            rgb[o] = (byte)r;
            rgb[o + 1] = (byte)g;
            rgb[o + 2] = (byte)b;
        }

        // expands one dxt5 alpha block into its 16 texel opacities
        void decode_alpha_block(const byte *src, byte out[16])
        {
            int table[8];
            table[0] = src[0];
            table[1] = src[1];
            if (table[0] > table[1])
            {
                for (int i = 1; i < 7; i++)
                    table[i + 1] = ((7 - i) * table[0] + i * table[1]) / 7;
            }
            else
            {
                for (int i = 1; i < 5; i++)
                    table[i + 1] = ((5 - i) * table[0] + i * table[1]) / 5;
                table[6] = 0;
                table[7] = 255;
            }
            unsigned long long bits = 0;
            for (int i = 0; i < 6; i++)
                bits |= (unsigned long long)src[2 + i] << (8 * i);
            for (int i = 0; i < 16; i++)
                out[i] = (byte)table[(bits >> (3 * i)) & 7];
        }

        void put_alpha(std::vector<byte> &alpha, unsigned w, unsigned h, unsigned x,
                       unsigned y, byte value)
        {
            if (x >= w || y >= h)
                return;
            alpha[(std::size_t)y * w + x] = value;
        }

        bool decode_mip(const byte *src, std::size_t src_len, int format, unsigned w,
                        unsigned h, std::vector<byte> &rgb, std::vector<byte> &alpha,
                        unsigned &alpha_mean, std::string *error)
        {
            rgb.assign((std::size_t)w * h * 3, 0);
            alpha.clear();
            alpha_mean = 255;
            if (is_dxt(format))
            {
                // dxt3/dxt5 blocks carry 8 bytes of alpha before the color block
                std::size_t color_offset = (format == fmt_dxt3 || format == fmt_dxt5) ? 8 : 0;
                std::size_t block = (format == fmt_dxt1 || format == fmt_dxt1_onebit) ? 8 : 16;
                bool opaque = format != fmt_dxt1 && format != fmt_dxt1_onebit;
                std::size_t bx = (w + 3) / 4, by = (h + 3) / 4;
                std::size_t need = bx * by * block;
                if (src_len < need)
                {
                    set_error(error, "vtf dxt data is truncated");
                    return false;
                }
                bool has_alpha = format == fmt_dxt3 || format == fmt_dxt5;
                if (has_alpha)
                    alpha.assign((std::size_t)w * h, 255);
                for (std::size_t cy = 0; cy < by; cy++)
                {
                    for (std::size_t cx = 0; cx < bx; cx++)
                    {
                        const byte *b = src + (cy * bx + cx) * block + color_offset;
                        byte texel_alpha[16];
                        if (format == fmt_dxt5)
                        {
                            decode_alpha_block(b - 8, texel_alpha);
                        }
                        else if (format == fmt_dxt3)
                        {
                            const byte *a = b - 8;
                            for (int i = 0; i < 16; i++)
                                texel_alpha[i] = (byte)(((i & 1) ? (a[i / 2] >> 4)
                                                                : (a[i / 2] & 0x0f)) * 17);
                        }
                        int texel[16][3];
                        decode_color_block(b, opaque, texel);
                        for (int py = 0; py < 4; py++)
                            for (int px = 0; px < 4; px++)
                            {
                                const int *t = texel[py * 4 + px];
                                put_texel(rgb, w, h, (unsigned)(cx * 4 + px),
                                          (unsigned)(cy * 4 + py), t[0], t[1], t[2]);
                                if (has_alpha)
                                    put_alpha(alpha, w, h, (unsigned)(cx * 4 + px),
                                              (unsigned)(cy * 4 + py),
                                              texel_alpha[py * 4 + px]);
                            }
                    }
                }
                if (!alpha.empty())
                {
                    unsigned long long total = 0;
                    for (byte a : alpha)
                        total += a;
                    alpha_mean = (unsigned)(total / alpha.size());
                }
                return true;
            }

            bool supported = true;
            std::size_t need = image_bytes(format, w, h, supported);
            if (src_len < need)
            {
                set_error(error, "vtf image data is truncated");
                return false;
            }
            std::size_t count = (std::size_t)w * h;
            unsigned long long alpha_total = 0;
            std::size_t alpha_samples = 0;
            bool has_alpha = format == fmt_rgba8888 || format == fmt_abgr8888
                || format == fmt_argb8888 || format == fmt_bgra8888;
            if (has_alpha)
                alpha.assign(count, 255);
            for (std::size_t i = 0; i < count; i++)
            {
                int r = 0, g = 0, b = 0;
                switch (format)
                {
                case fmt_rgb888: r = src[i*3]; g = src[i*3+1]; b = src[i*3+2]; break;
                case fmt_bgr888: b = src[i*3]; g = src[i*3+1]; r = src[i*3+2]; break;
                case fmt_rgba8888: r = src[i*4]; g = src[i*4+1]; b = src[i*4+2];
                    alpha[i] = src[i*4+3];
                    alpha_total += src[i*4+3]; alpha_samples++; break;
                case fmt_abgr8888: b = src[i*4+1]; g = src[i*4+2]; r = src[i*4+3];
                    alpha[i] = src[i*4];
                    alpha_total += src[i*4]; alpha_samples++; break;
                case fmt_argb8888: r = src[i*4+1]; g = src[i*4+2]; b = src[i*4+3];
                    alpha[i] = src[i*4];
                    alpha_total += src[i*4]; alpha_samples++; break;
                case fmt_bgra8888: b = src[i*4]; g = src[i*4+1]; r = src[i*4+2];
                    alpha[i] = src[i*4+3];
                    alpha_total += src[i*4+3]; alpha_samples++; break;
                case fmt_bgrx8888: b = src[i*4]; g = src[i*4+1]; r = src[i*4+2]; break;
                case fmt_i8: r = g = b = src[i]; break;
                case fmt_rgb565:
                {
                    int c[3];
                    rgb_from_565((unsigned short)(src[i*2] | (src[i*2+1] << 8)), c);
                    r = c[0]; g = c[1]; b = c[2];
                    break;
                }
                case fmt_bgr565:
                {
                    int c[3];
                    rgb_from_565((unsigned short)(src[i*2] | (src[i*2+1] << 8)), c);
                    b = c[0]; g = c[1]; r = c[2];
                    break;
                }
                default:
                    set_error(error, "unsupported vtf pixel format");
                    return false;
                }
                rgb[i*3] = (byte)r; rgb[i*3+1] = (byte)g; rgb[i*3+2] = (byte)b;
            }
            if (alpha_samples)
                alpha_mean = (unsigned)(alpha_total / alpha_samples);
            return true;
        }

        // locates the high-res mip data start. for v7.3+ the offset comes from the
        // resource table; earlier versions place it right after the low-res image.
        bool find_highres_offset(const std::vector<byte> &d, int version_major,
                                 int version_minor, unsigned header_size,
                                 int lowres_format, unsigned lowres_w, unsigned lowres_h,
                                 std::size_t &data_offset, std::string *error)
        {
            if (version_major == 7 && version_minor >= 3)
            {
                binary::reader r(d);
                std::uint32_t num_resources = 0;
                if (!r.u32_at(68, num_resources) || num_resources > 32)
                {
                    set_error(error, "vtf resource table is invalid");
                    return false;
                }
                for (unsigned i = 0; i < num_resources; i++)
                {
                    std::size_t pos = 80 + (std::size_t)i * 8;
                    if (pos + 8 > d.size())
                        break;
                    if (d[pos] == 0x30 && d[pos + 1] == 0 && d[pos + 2] == 0)
                    {
                        std::uint32_t off = 0;
                        r.u32_at(pos + 4, off);
                        data_offset = off;
                        return true;
                    }
                }
                set_error(error, "vtf has no high-res image resource");
                return false;
            }

            std::size_t offset = header_size;
            if (lowres_format != -1 && lowres_w > 0 && lowres_h > 0)
            {
                bool supported = true;
                offset += image_bytes(fmt_dxt1, lowres_w, lowres_h, supported);
            }
            data_offset = offset;
            return true;
        }
    }

    bool decode_vtf(const std::vector<byte> &data, unsigned max_dim, vtf_image &out,
                    std::string *error)
    {
        out = vtf_image{};
        if (data.size() < 80 || std::memcmp(data.data(), "VTF\0", 4) != 0)
        {
            set_error(error, "not a vtf file");
            return false;
        }

        binary::reader r(data);
        std::uint32_t version_major = 0, version_minor = 0, header_size = 0;
        std::uint16_t width = 0, height = 0;
        r.u32_at(4, version_major);
        r.u32_at(8, version_minor);
        r.u32_at(12, header_size);
        r.u16_at(16, width);
        r.u16_at(18, height);

        std::int32_t high_format = 0, low_format = 0;
        std::uint8_t mip_count = 0, low_w = 0, low_h = 0;
        r.i32_at(52, high_format);
        byte mc = 0; r.seek(56); r.u8(mc); mip_count = mc;
        r.i32_at(57, low_format);
        byte lw = 0, lh = 0; r.seek(61); r.u8(lw); r.u8(lh);
        low_w = lw; low_h = lh;

        if (width == 0 || height == 0 || mip_count == 0)
        {
            set_error(error, "vtf has no image data");
            return false;
        }

        std::size_t data_offset = 0;
        if (!find_highres_offset(data, (int)version_major, (int)version_minor,
                                 header_size, low_format, low_w, low_h, data_offset, error))
            return false;

        // pick the smallest mip level (largest image) that fits within max_dim.
        int level = 0;
        while (level < mip_count - 1
               && ((unsigned)(width >> level) > max_dim
                   || (unsigned)(height >> level) > max_dim))
            level++;
        unsigned target_w = std::max(1u, (unsigned)(width >> level));
        unsigned target_h = std::max(1u, (unsigned)(height >> level));

        // walk the stored mips (smallest first) to find the chosen level's bytes.
        std::size_t cursor = data_offset;
        std::vector<byte> chosen;
        unsigned chosen_w = 0, chosen_h = 0;
        for (int stored = mip_count - 1; stored >= 0; stored--)
        {
            unsigned mw = std::max(1u, (unsigned)(width >> stored));
            unsigned mh = std::max(1u, (unsigned)(height >> stored));
            bool supported = true;
            std::size_t bytes = image_bytes(high_format, mw, mh, supported);
            if (!supported)
            {
                set_error(error, "unsupported vtf pixel format");
                return false;
            }
            if (cursor + bytes > data.size())
            {
                set_error(error, "vtf mip data extends past the file");
                return false;
            }
            if (stored == level)
            {
                chosen.assign(data.begin() + cursor, data.begin() + cursor + bytes);
                chosen_w = mw;
                chosen_h = mh;
            }
            cursor += bytes;
        }
        if (chosen.empty() && !(target_w == 0))
        {
            set_error(error, "vtf chosen mip level was not found");
            return false;
        }

        std::vector<byte> rgb;
        std::vector<byte> alpha;
        unsigned alpha_mean = 255;
        if (!decode_mip(chosen.data(), chosen.size(), high_format, chosen_w, chosen_h,
                        rgb, alpha, alpha_mean, error))
            return false;

        out.width = chosen_w;
        out.height = chosen_h;
        out.full_width = width;
        out.full_height = height;
        out.alpha_mean = alpha_mean;
        out.rgb = std::move(rgb);
        out.alpha = std::move(alpha);
        return true;
    }
}
