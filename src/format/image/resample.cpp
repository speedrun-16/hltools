#include "format/image/resample.h"

namespace format
{
    namespace
    {
        // the half open source span [begin, end) that destination index i covers.
        // computed in integer arithmetic so the spans tile the source exactly:
        // every source pixel belongs to exactly one destination pixel, with no
        // gaps and no double counting.
        struct span
        {
            unsigned begin;
            unsigned end;
        };

        span cover(unsigned i, unsigned src, unsigned dst)
        {
            span s;
            s.begin = (unsigned)(((unsigned long long)i * src) / dst);
            s.end = (unsigned)(((unsigned long long)(i + 1) * src) / dst);
            if (s.end <= s.begin)
                s.end = s.begin + 1; // magnifying: one source pixel
            if (s.end > src)
                s.end = src;
            return s;
        }
    }

    std::vector<byte> resample_rgb(const std::vector<byte> &src, unsigned src_w,
                                   unsigned src_h, unsigned dst_w, unsigned dst_h,
                                   const std::vector<byte> *alpha)
    {
        std::vector<byte> out((std::size_t)dst_w * dst_h * 3);
        if (!src_w || !src_h || !dst_w || !dst_h)
            return out;

        bool weighted = alpha && alpha->size() >= (std::size_t)src_w * src_h;
        for (unsigned y = 0; y < dst_h; y++)
        {
            span rows = cover(y, src_h, dst_h);
            for (unsigned x = 0; x < dst_w; x++)
            {
                span cols = cover(x, src_w, dst_w);
                unsigned long long r = 0, g = 0, b = 0, weight = 0;
                unsigned long long flat_r = 0, flat_g = 0, flat_b = 0, count = 0;
                for (unsigned sy = rows.begin; sy < rows.end; sy++)
                    for (unsigned sx = cols.begin; sx < cols.end; sx++)
                    {
                        std::size_t pixel = (std::size_t)sy * src_w + sx;
                        std::size_t s = pixel * 3;
                        if (s + 2 >= src.size())
                            continue;
                        unsigned long long a = weighted ? (*alpha)[pixel] : 255;
                        r += (unsigned long long)src[s] * a;
                        g += (unsigned long long)src[s + 1] * a;
                        b += (unsigned long long)src[s + 2] * a;
                        weight += a;
                        flat_r += src[s];
                        flat_g += src[s + 1];
                        flat_b += src[s + 2];
                        count++;
                    }
                std::size_t o = ((std::size_t)y * dst_w + x) * 3;
                if (weight != 0)
                {
                    out[o] = (byte)(r / weight);
                    out[o + 1] = (byte)(g / weight);
                    out[o + 2] = (byte)(b / weight);
                }
                else if (count != 0)
                {
                    // the whole footprint is transparent: no visible colour to
                    // preserve, so fall back to the plain average
                    out[o] = (byte)(flat_r / count);
                    out[o + 1] = (byte)(flat_g / count);
                    out[o + 2] = (byte)(flat_b / count);
                }
            }
        }
        return out;
    }

    std::vector<byte> resample_alpha(const std::vector<byte> &src, unsigned src_w,
                                     unsigned src_h, unsigned dst_w, unsigned dst_h)
    {
        std::vector<byte> out((std::size_t)dst_w * dst_h);
        if (!src_w || !src_h || !dst_w || !dst_h)
            return out;

        for (unsigned y = 0; y < dst_h; y++)
        {
            span rows = cover(y, src_h, dst_h);
            for (unsigned x = 0; x < dst_w; x++)
            {
                span cols = cover(x, src_w, dst_w);
                unsigned long long sum = 0, count = 0;
                for (unsigned sy = rows.begin; sy < rows.end; sy++)
                    for (unsigned sx = cols.begin; sx < cols.end; sx++)
                    {
                        std::size_t s = (std::size_t)sy * src_w + sx;
                        if (s >= src.size())
                            continue;
                        sum += src[s];
                        count++;
                    }
                if (count)
                    out[(std::size_t)y * dst_w + x] = (byte)(sum / count);
            }
        }
        return out;
    }
}
