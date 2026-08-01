#include <vector>

#include "format/image/resample.h"
#include "support/test.h"

suite("unit.format.resample")
{
    test("resample.averages every source pixel when reducing")
    {
        std::vector<byte> rgb = {
            0, 0, 0,       100, 0, 0,
            0, 100, 0,     0, 0, 100,
        };
        std::vector<byte> out = format::resample_rgb(rgb, 2, 2, 1, 1);
        require(out.size() == 3);
        expect(out[0] == 25);
        expect(out[1] == 25);
        expect(out[2] == 25);
    }

    test("resample.weights rgb by alpha without changing alpha averaging")
    {
        std::vector<byte> rgb = {255, 0, 0, 0, 0, 255};
        std::vector<byte> alpha = {255, 0};
        std::vector<byte> color = format::resample_rgb(rgb, 2, 1, 1, 1, &alpha);
        std::vector<byte> opacity = format::resample_alpha(alpha, 2, 1, 1, 1);
        require(color.size() == 3);
        require(opacity.size() == 1);
        expect(color[0] == 255);
        expect(color[1] == 0);
        expect(color[2] == 0);
        expect(opacity[0] == 127);
    }
}
