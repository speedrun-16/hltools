#include "quantize.h"

#include <algorithm>
#include <array>
#include <climits>
#include <unordered_map>
#include <vector>

namespace format
{
    namespace
    {
        struct color_entry
        {
            byte rgb[3];
            unsigned count;
        };

        struct color_box
        {
            int start;
            int end; // exclusive
            int longest_axis;
            int extent;
        };

        void measure_box(const std::vector<color_entry> &colors, color_box &box)
        {
            int lo[3] = {255, 255, 255};
            int hi[3] = {0, 0, 0};
            for (int i = box.start; i < box.end; i++)
                for (int k = 0; k < 3; k++)
                {
                    lo[k] = std::min(lo[k], (int)colors[(std::size_t)i].rgb[k]);
                    hi[k] = std::max(hi[k], (int)colors[(std::size_t)i].rgb[k]);
                }
            box.longest_axis = 0;
            box.extent = -1;
            for (int k = 0; k < 3; k++)
            {
                int range = hi[k] - lo[k];
                if (range > box.extent)
                {
                    box.extent = range;
                    box.longest_axis = k;
                }
            }
        }
    }

    namespace
    {
        constexpr byte masked_index = 255;

        // median-cut over the pixels the caller counts as opaque. slots is how
        // many palette entries the boxes may occupy, so a masked image can keep
        // the last one for its transparent index.
        bool quantize(const byte *rgb, const byte *alpha, byte threshold,
                      unsigned width, unsigned height, std::size_t slots,
                      indexed_image &out)
        {
            out = indexed_image{};
            out.width = width;
            out.height = height;
            std::size_t pixels = (std::size_t)width * height;
            if (pixels == 0)
                return false;

            auto transparent = [&](std::size_t i) {
                return alpha != nullptr && alpha[i] < threshold;
            };

            // histogram of unique colors so median-cut works on distinct entries
            // weighted by population.
            std::unordered_map<unsigned, unsigned> histogram;
            histogram.reserve(pixels);
            for (std::size_t i = 0; i < pixels; i++)
            {
                if (transparent(i))
                    continue;
                unsigned key = (unsigned)rgb[i * 3] | ((unsigned)rgb[i * 3 + 1] << 8)
                    | ((unsigned)rgb[i * 3 + 2] << 16);
                histogram[key]++;
            }

            // nothing opaque to quantize: the image is entirely the mask colour
            if (histogram.empty())
            {
                out.palette[masked_index][2] = 255;
                out.pixels.assign(pixels, masked_index);
                return true;
            }

            std::vector<color_entry> colors;
            colors.reserve(histogram.size());
            for (const auto &kv : histogram)
            {
                color_entry e;
                e.rgb[0] = (byte)(kv.first & 0xff);
                e.rgb[1] = (byte)((kv.first >> 8) & 0xff);
                e.rgb[2] = (byte)((kv.first >> 16) & 0xff);
                e.count = kv.second;
                colors.push_back(e);
            }

            std::vector<color_box> boxes;
            color_box initial{0, (int)colors.size(), 0, 0};
            measure_box(colors, initial);
            boxes.push_back(initial);

            // repeatedly split the box with the widest channel until we fill the
            // available palette slots or nothing can be split further.
            while (boxes.size() < slots)
            {
                int target = -1;
                int best_extent = 0;
                for (std::size_t i = 0; i < boxes.size(); i++)
                    if (boxes[i].end - boxes[i].start > 1 && boxes[i].extent > best_extent)
                    {
                        best_extent = boxes[i].extent;
                        target = (int)i;
                    }
                if (target < 0)
                    break;

                color_box &box = boxes[(std::size_t)target];
                int axis = box.longest_axis;
                std::sort(colors.begin() + box.start, colors.begin() + box.end,
                          [axis](const color_entry &a, const color_entry &b)
                          { return a.rgb[axis] < b.rgb[axis]; });

                // split at the population median so dense regions get finer boxes
                unsigned total = 0;
                for (int i = box.start; i < box.end; i++)
                    total += colors[(std::size_t)i].count;
                unsigned half = total / 2;
                unsigned acc = 0;
                int split = box.start + 1;
                for (int i = box.start; i < box.end - 1; i++)
                {
                    acc += colors[(std::size_t)i].count;
                    if (acc >= half)
                    {
                        split = i + 1;
                        break;
                    }
                }

                color_box lo{box.start, split, 0, 0};
                color_box hi{split, box.end, 0, 0};
                measure_box(colors, lo);
                measure_box(colors, hi);
                boxes[(std::size_t)target] = lo;
                boxes.push_back(hi);
            }

            // palette entry = population-weighted average of each box; unused slots
            // stay black.
            std::unordered_map<unsigned, byte> color_index;
            color_index.reserve(colors.size());
            for (std::size_t b = 0; b < boxes.size(); b++)
            {
                const color_box &box = boxes[b];
                unsigned long long sum[3] = {0, 0, 0};
                unsigned long long weight = 0;
                for (int i = box.start; i < box.end; i++)
                {
                    const color_entry &e = colors[(std::size_t)i];
                    for (int k = 0; k < 3; k++)
                        sum[k] += (unsigned long long)e.rgb[k] * e.count;
                    weight += e.count;
                }
                for (int k = 0; k < 3; k++)
                    out.palette[b][(std::size_t)k] =
                        (byte)(weight ? sum[k] / weight : 0);
                for (int i = box.start; i < box.end; i++)
                {
                    const color_entry &e = colors[(std::size_t)i];
                    unsigned key = (unsigned)e.rgb[0] | ((unsigned)e.rgb[1] << 8)
                        | ((unsigned)e.rgb[2] << 16);
                    color_index[key] = (byte)b;
                }
            }

            // goldsrc reads the last palette entry of a '{' texture as the colour to
            // punch out; the classic value is pure blue.
            if (alpha != nullptr)
            {
                out.palette[masked_index][0] = 0;
                out.palette[masked_index][1] = 0;
                out.palette[masked_index][2] = 255;
            }

            out.pixels.resize(pixels);
            for (std::size_t i = 0; i < pixels; i++)
            {
                if (transparent(i))
                {
                    out.pixels[i] = masked_index;
                    continue;
                }
                unsigned key = (unsigned)rgb[i * 3] | ((unsigned)rgb[i * 3 + 1] << 8)
                    | ((unsigned)rgb[i * 3 + 2] << 16);
                out.pixels[i] = color_index[key];
            }
            return true;
        }
    }

    bool quantize_rgb(const byte *rgb, unsigned width, unsigned height, indexed_image &out)
    {
        return quantize(rgb, nullptr, 0, width, height, 256, out);
    }

    bool quantize_rgb_fixed(const byte *rgb, const byte *alpha, byte threshold,
                            unsigned width, unsigned height,
                            const std::array<std::array<byte, 3>, 256> &palette,
                            indexed_image &out)
    {
        if (rgb == nullptr || width == 0 || height == 0)
            return false;

        out.width = width;
        out.height = height;
        out.palette = palette;
        std::size_t pixels = (std::size_t)width * height;
        out.pixels.resize(pixels);

        // the punch-out slot is not a colour to match against
        int usable = alpha != nullptr ? (int)masked_index : 256;

        std::unordered_map<unsigned, byte> cache;
        cache.reserve(4096);
        for (std::size_t i = 0; i < pixels; i++)
        {
            if (alpha != nullptr && alpha[i] < threshold)
            {
                out.pixels[i] = masked_index;
                continue;
            }
            unsigned key = (unsigned)rgb[i * 3] | ((unsigned)rgb[i * 3 + 1] << 8)
                | ((unsigned)rgb[i * 3 + 2] << 16);
            auto found = cache.find(key);
            if (found != cache.end())
            {
                out.pixels[i] = found->second;
                continue;
            }
            int best = 0;
            long best_distance = -1;
            for (int p = 0; p < usable; p++)
            {
                long dr = (long)rgb[i * 3] - palette[(std::size_t)p][0];
                long dg = (long)rgb[i * 3 + 1] - palette[(std::size_t)p][1];
                long db = (long)rgb[i * 3 + 2] - palette[(std::size_t)p][2];
                long distance = dr * dr + dg * dg + db * db;
                if (best_distance < 0 || distance < best_distance)
                {
                    best_distance = distance;
                    best = p;
                    if (distance == 0)
                        break;
                }
            }
            cache.emplace(key, (byte)best);
            out.pixels[i] = (byte)best;
        }
        return true;
    }

    bool quantize_rgb_masked(const byte *rgb, const byte *alpha, byte threshold,
                             unsigned width, unsigned height, indexed_image &out)
    {
        if (alpha == nullptr)
            return quantize_rgb(rgb, width, height, out);
        return quantize(rgb, alpha, threshold, width, height, masked_index, out);
    }
}
