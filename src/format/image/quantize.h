#pragma once

#include "common/types.h"
#include "format/bmp/image.h"

namespace format
{
    // reduces a 24-bit rgb image (row major, three bytes per pixel) to a 256
    // color indexed_image using median-cut. always succeeds for a non empty
    // image; width/height are copied through unchanged.
    bool quantize_rgb(const byte *rgb, unsigned width, unsigned height,
                      indexed_image &out);

    // as quantize_rgb, but reserves the last palette slot for goldsrc's masked
    // transparency: pixels whose alpha is below the threshold become index 255
    // and palette[255] is set to pure blue, while the opaque pixels share the
    // remaining 255 slots. alpha is one byte per pixel, same layout as rgb.
    // a '{' prefixed texture name is what makes the engine honour the mask.
    bool quantize_rgb_masked(const byte *rgb, const byte *alpha, byte threshold,
                             unsigned width, unsigned height, indexed_image &out);
}
