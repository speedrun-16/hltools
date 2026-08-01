#include <vector>

#include "common/types.h"
#include "format/image/quantize.h"
#include "support/test.h"

namespace
{
    // four pixels: red, green, blue, white
    std::vector<byte> four_colors()
    {
        return {255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 255};
    }
}

suite("unit.format.quantize")
{
    test("quantize.keeps every palette slot available when unmasked")
    {
        std::vector<byte> rgb = four_colors();
        format::indexed_image out;
        require(format::quantize_rgb(rgb.data(), 2, 2, out));

        expect(out.width == 2);
        expect(out.height == 2);
        require(out.pixels.size() == 4);
        // four distinct colours fit comfortably, so none of them collide
        expect(out.pixels[0] != out.pixels[1]);
        expect(out.pixels[1] != out.pixels[2]);
        // an unmasked image must not have blue forced into the last slot
        for (byte index : out.pixels)
            expect(index != 255);
    }

    test("quantize.masks pixels below the alpha threshold to index 255")
    {
        std::vector<byte> rgb = four_colors();
        std::vector<byte> alpha = {255, 0, 255, 0}; // green and white cut out
        format::indexed_image out;
        require(format::quantize_rgb_masked(rgb.data(), alpha.data(), 128, 2, 2, out));

        require(out.pixels.size() == 4);
        expect(out.pixels[0] != 255); // red survives
        expect(out.pixels[1] == 255); // green is transparent
        expect(out.pixels[2] != 255); // blue survives
        expect(out.pixels[3] == 255); // white is transparent

        // goldsrc reads the last entry as the colour to punch out
        expect(out.palette[255][0] == 0);
        expect(out.palette[255][1] == 0);
        expect(out.palette[255][2] == 255);
    }

    test("quantize.masks an entirely transparent image to the mask colour")
    {
        std::vector<byte> rgb = four_colors();
        std::vector<byte> alpha = {0, 0, 0, 0};
        format::indexed_image out;
        require(format::quantize_rgb_masked(rgb.data(), alpha.data(), 128, 2, 2, out));

        require(out.pixels.size() == 4);
        for (byte index : out.pixels)
            expect(index == 255);
        expect(out.palette[255][2] == 255);
    }

    test("quantize.masked without an alpha channel matches the plain path")
    {
        std::vector<byte> rgb = four_colors();
        format::indexed_image masked;
        format::indexed_image plain;
        require(format::quantize_rgb_masked(rgb.data(), nullptr, 128, 2, 2, masked));
        require(format::quantize_rgb(rgb.data(), 2, 2, plain));
        expect(masked.pixels == plain.pixels);
        expect(masked.palette == plain.palette);
    }

    test("quantize.fixed maps onto the supplied palette")
    {
        std::vector<byte> rgb = {250, 4, 3, 2, 245, 5};
        std::array<std::array<byte, 3>, 256> palette{};
        palette[10] = {255, 0, 0};
        palette[20] = {0, 255, 0};
        format::indexed_image out;
        require(format::quantize_rgb_fixed(rgb.data(), nullptr, 128, 2, 1,
                                           palette, out));
        require(out.pixels.size() == 2);
        expect(out.palette == palette);
        expect(out.pixels[0] == 10);
        expect(out.pixels[1] == 20);
    }

    test("quantize.fixed reserves palette index 255 for masked pixels")
    {
        std::vector<byte> rgb = {0, 0, 255, 255, 0, 0};
        std::vector<byte> alpha = {255, 0};
        std::array<std::array<byte, 3>, 256> palette{};
        palette[1] = {0, 0, 250};
        palette[255] = {0, 0, 255};
        format::indexed_image out;
        require(format::quantize_rgb_fixed(rgb.data(), alpha.data(), 128, 2, 1,
                                           palette, out));
        require(out.pixels.size() == 2);
        expect(out.pixels[0] == 1);
        expect(out.pixels[1] == 255);
    }
}
