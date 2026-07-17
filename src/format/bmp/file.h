#pragma once

#include <string>
#include <vector>

#include "common/types.h"
#include "image.h"
#include "format/miptex/texture.h"

namespace format
{
    bool load_indexed_bmp(const std::string &path, indexed_image &image,
                          std::string *error = nullptr);
    bool write_indexed_bmp(const std::string &path, const mip_texture &texture,
                           std::string *error = nullptr);

    // write top down rgb8 pixels as an uncompressed 24 bit windows bmp
    bool write_rgb_bmp(const std::string &path, unsigned width, unsigned height,
                       const std::vector<byte> &pixels, std::string *error = nullptr);
}
