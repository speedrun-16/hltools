#pragma once

#include <string>

#include "../../common/cmdline.h"
#include "../../vis/vis.h"

namespace tools
{
    vis::vis_options parse_vis_options(const cli::args &args, const std::string &portal_path);

    // the hlvis main body
    int run_vis_tool(int argc, char **argv);
}
