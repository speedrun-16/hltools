#pragma once

#include <string>

#include "format/bsp/data.h"

namespace vis
{
    struct vis_options
    {
        std::string portal_path;
        bool fast = false;
        bool full = true; // exact full visibility is the default (see vis_tool)
        bool nofixprt = false;
        bool chart = true;
        unsigned maxdistance = 0;
    };

    void run_vis(format::map_data &map, const vis_options &options);
}
