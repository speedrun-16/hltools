#pragma once

#include <vector>

#include "common/types.h"

namespace format
{
    // area-averaging RGB resize with optional alpha weights for visible colors.
    std::vector<byte> resample_rgb(const std::vector<byte> &src, unsigned src_w,
                                   unsigned src_h, unsigned dst_w, unsigned dst_h,
                                   const std::vector<byte> *alpha = nullptr);

    // area-averaging single-channel resize.
    std::vector<byte> resample_alpha(const std::vector<byte> &src, unsigned src_w,
                                     unsigned src_h, unsigned dst_w, unsigned dst_h);
}
