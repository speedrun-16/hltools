#pragma once

namespace tools
{
    // "hltools bspinfo <mapbsp>": report a compiled bsp's contents and how
    // full each lump is against its engine limit read only
    int run_bspinfo_tool(int argc, char **argv);
}
