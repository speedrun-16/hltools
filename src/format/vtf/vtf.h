#pragma once

#include <string>
#include <vector>

#include "common/types.h"

namespace format
{
    // a decoded, opaque rgb image extracted from a vtf. one byte per channel,
    // row major top to bottom, three bytes per pixel.
    struct vtf_image
    {
        unsigned width = 0;   // dimensions of the decoded (possibly reduced) mip
        unsigned height = 0;
        unsigned full_width = 0;  // dimensions of mip 0, for uv axis scaling
        unsigned full_height = 0;
        // average opacity of the decoded mip, 0..255; 255 for formats without
        // alpha. lets the caller reproduce translucent materials.
        unsigned alpha_mean = 255;
        std::vector<byte> rgb;
        // one byte of opacity per pixel, or empty when the source format has no
        // alpha channel. carries the cutout shape of $alphatest materials, which
        // goldsrc reproduces as a '{' masked texture.
        std::vector<byte> alpha;
    };

    // decodes a vtf's high resolution image to rgb, choosing the largest mip
    // level whose dimensions do not exceed max_dim. handles DXT1/DXT3/DXT5 and
    // the common uncompressed layouts; an unsupported pixel format fails with a
    // descriptive error so the caller can fall back.
    bool decode_vtf(const std::vector<byte> &data, unsigned max_dim, vtf_image &out,
                    std::string *error = nullptr);
}
