#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "common/types.h"

namespace model
{
    // RGB pixels are stored top row first. The loader accepts the uncompressed
    // 24/32-bit TGA files used by GoldSrc and by hltools' Source sky exporter.
    struct rgb_image
    {
        unsigned width = 0;
        unsigned height = 0;
        std::vector<byte> rgb;
    };

    struct skybox_model_options
    {
        // The model is centred on its entity origin and extends half this far
        // along every axis. 131072 is the established sky-model convention.
        float world_size = 131072.0f;
        // GoldSrc studio skins cannot exceed 512 pixels. A larger face is
        // divided into square tiles of this size without resampling.
        unsigned tile_size = 512;
        // Stored in the MDL header; each emitted part appends its numeric index.
        std::string model_name = "skybox";
    };

    struct skybox_model_part
    {
        std::string name;
        std::vector<byte> data;
        std::size_t textures = 0;
        std::size_t triangles = 0;
    };

    bool load_tga(const std::string &path, rgb_image &out,
                  std::string *error = nullptr);

    // Face order is up, left, front, right, back, down. This matches gchimp's
    // proven SkyMod orientation and the conventional {up,lf,ft,rt,bk,dn}
    // GoldSrc suffixes.
    bool build_skybox_models(const std::array<rgb_image, 6> &faces,
                             const skybox_model_options &options,
                             std::vector<skybox_model_part> &out,
                             std::string *error = nullptr);
}
