#pragma once

#include "../../common/cmdline.h"
#include "../../bsp/bsp.h"

namespace tools
{
    bsp::bsp_options parse_bsp_options(const cli::args &args);

    // the hltools bsp command body
    int run_bsp_tool(int argc, char **argv);
}
