#pragma once

#include <string>
#include <vector>

#include "format/bsp/data.h"
#include "format/rad/texlights.h"
#include "../math/vector.h"

// the rad stage: radiosity lighting reads a compiled bsp (after csg, bsp and
// vis), computes the lighting lump and writes it back into the map float
// precision throughout, like the reference build

namespace rad
{
    // which visibility method builds the patch to patch transfer lists
    enum class vis_method
    {
        vismatrix,        // dense triangular bit matrix
        sparse_vismatrix, // sparse rows, the default
        no_vismatrix,     // test visibility on demand
    };

    // storage formats for compressed transfer data, from compressh
    enum class float_format
    {
        float32 = 0,
        float16,
        float8,
    };

    enum class vector_format
    {
        vector96 = 0,
        vector48,
        vector32,
        vector24,
    };

    struct rad_options
    {
        bool fastmode = false;
        bool extra = false;
        vis_method method = vis_method::sparse_vismatrix;
        bool lerp_enabled = true;
        bool studioshadow = true;
        float fade = 1.0f;
        unsigned numbounce = 8;
        bool dumppatches = false;
        math::vec3f ambient{0.0f, 0.0f, 0.0f};
        float limitthreshold = 255.0f;
        bool drawoverload = false;
        float chop = 64.0f;    // normal texture chop
        float texchop = 32.0f; // texture light chop
        float lightscale = 2.0f;
        float dlight_threshold = 10.0f; // direct light threshold
        float direct_scale = 1.0f;      // direct light scale ("-dlight" scale)
        float smoothing_value = 50.0f;  // phong smoothing angle in degrees
        float smoothing_value_2 = 0.0f; // 0 means fall back to smoothing_value
        bool incremental = false;
        float indirect_sun = 1.0f;
        bool sky_lighting_fix = true;
        bool circus = false;
        float coring = 0.01f;
        bool subdivide = true; // chop patches
        bool allow_opaques = true;
        bool allow_spread = true;
        math::vec3f colour_qgamma{0.55f, 0.55f, 0.55f};
        math::vec3f colour_lightscale{2.0f, 2.0f, 2.0f};
        math::vec3f colour_jitter_hack{0.0f, 0.0f, 0.0f};
        math::vec3f jitter_hack{0.0f, 0.0f, 0.0f};
        bool customshadow_with_bouncelight = false;
        bool rgb_transfers = false;
        float transtotal_hack = 0.2f;
        unsigned char minlight = 0;
        float_format transfer_compress_type = float_format::float16;
        vector_format rgbtransfer_compress_type = vector_format::vector32;
        bool softsky = true;
        int blockopaque = 1;
        float translucentdepth = 2.0f;
        bool notextures = false;
        float texreflectgamma = 1.76f;
        float texreflectscale = 0.7f;
        float blur = 1.5f;
        bool noemitterrange = false;
        bool bleedfix = true;
        float texlightgap = 0.0f;
        bool pre25update = false; // clamp to the fullbright threshold from before the 25th anniversary

        bool texscale = true; // scale patches with texture scale

        // lighting data memory budget in bytes (-lightdata)
        int max_map_lightdata = 0x3000000;

        // texture memory budget in bytes (-texdata)
        int max_map_miptex = 0x2000000;

        // lightsrad style files to read, in order (resolved by the cli)
        std::vector<std::string> rad_files;

        // automatically selected lightsrad, tracked so rad can report when
        // any of its definitions survive the complete override cascade
        std::string generic_rad_file;

        // extra folders to search for wad files
        std::vector<std::string> wad_folders;

        // -dumpgather: write <base>gather with every face's per style direct
        // gather results (the seam the gpu backend replaces; compare two dumps
        // with gather_diffpy to bisect a lighting divergence to a sample)
        bool dumpgather = false;

        // -gpu: run the direct lighting gather on the gpu (approximate mode;
        // the cpu path is the reference)
        // falls back to the cpu with a warning when no device is usable or
        // the map is out of the kernel's scope
        bool gpu = false;

        // test seam for cpu and gpu comparisons when set run_rad stops
        // right after create_direct_lights/load_studio_models and hands the
        // fully prepared state to the hook instead of lighting the map
        void (*direct_state_hook)(struct rad_state &state, void *ctx) = nullptr;
        void *direct_state_hook_ctx = nullptr;

        // debug point file outputs
        bool drawpatch = false;
        bool drawsample = false;
        math::vec3f drawsample_origin{0.0f, 0.0f, 0.0f};
        float drawsample_radius = 0.0f;
        bool drawedge = false;
        bool drawlerp = false;
        bool drawnudge = false;
    };

    // the gpu the -gpu path will use, for the settings table: the device name,
    // or why the cpu fallback will run instead initializes the device on
    // first call (a few ms), so a failed init surfaces at the top of the log
    // rather than mid compile
    std::string gpu_device_description();

    // calculates how many engine lightmap atlas pages this bsp requires
    // this is read only and can be used by reporting tools without running rad
    int count_alloc_blocks(const format::map_data &map);

    // lights the map in place returns false when the lightmap atlas budget
    // overflowed and lighting was skipped (the map would not load in game)
    bool run_rad(format::map_data &map, const std::string &base_path, const rad_options &options,
                 int *alloc_block_pages = nullptr,
                 std::vector<format::texlight> *used_texlights = nullptr);
}
