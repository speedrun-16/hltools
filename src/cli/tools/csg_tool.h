#pragma once

#include <string>

#include "../../common/cmdline.h"
#include "../../csg/csg.h"

namespace tools
{
    // args -> stage options, shared by hltools compile and hltools csg
    // buffers the stage's "settings" rows; the caller flushes the table
    csg::csg_options parse_csg_options(const cli::args &args, const std::string &map_path);

    // the hltools csg command body
    int run_csg_tool(int argc, char **argv);
}
