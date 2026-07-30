#pragma once

namespace tools
{
    // "hltools bsp info <mapbsp>": report a compiled bsp's contents and how
    // full each lump is against its engine limit read only
    int run_bsp_info_tool(int argc, char **argv);
}
