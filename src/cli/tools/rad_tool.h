#pragma once

#include <string>

#include "../../common/cmdline.h"
#include "../../rad/rad.h"

namespace tools
{
    // args -> rad options, including the repeatable/multi argument flags that
    // need a raw argv scan and the lightsrad file cascade (which resolves
    // relative to the map folder, the tool folder in argv0, and the cwd)
    rad::rad_options parse_rad_options(const cli::args &args, int argc, char **argv,
                                       const std::string &base);

    // the hlrad main body
    int run_rad_tool(int argc, char **argv);
}
