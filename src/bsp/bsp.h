#pragma once

#include <string>
#include <vector>

#include "../common/types.h"
#include "format/bsp/data.h"

namespace bsp
{
    constexpr int num_hulls = 4;

    // (max_surface_extent - 1) * texture_step, like the reference default
    constexpr int default_subdivide_size = (16 - 1) * 16;
    constexpr int default_maxnode_size = 1024;

    struct bsp_options
    {
        bool nofill = false;
        bool noinsidefill = false;
        bool notjunc = false;
        bool nobrink = false;
        bool noclip = false;
        bool noopt = false;
        bool noclipnodemerge = false;
        bool leakonly = false;
        // survey every hole instead of reporting only the first one found, and
        // put all of their paths in one pointfile
        bool allleaks = false;
        bool nulltex = true;
        bool nohull2 = false;
        int maxnode_size = default_maxnode_size;
        int subdivide_size = default_subdivide_size;
    };

    // runs the bsp stage over the loaded csg output base_path names the csg
    // sidecar files (p0 through p3, b0 through b3, hsz, pln) next to the bsp
    void run_bsp(format::map_data &map, const std::string &base_path, const bsp_options &options);

    // the csg intermediates handed over in memory for a single process compile:
    // the same bytes the sidecar files would hold (text buffers may be \n where
    // the files are \r\n; the tokenizer treats both as whitespace)
    struct bsp_input
    {
        std::string surfaces[num_hulls]; // p0 through p3
        std::string brushes[num_hulls];  // b0 through b3
        std::string hull_sizes;          // hsz
        std::vector<byte> planes;        // pln
    };

    // same stage, but consumes in memory csg output instead of the sidecar
    // files base_path still names the outputs (prt, pts leak files)
    void run_bsp(format::map_data &map, const std::string &base_path, const bsp_options &options,
                 bsp_input input);
}
