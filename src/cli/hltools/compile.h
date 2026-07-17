#pragma once

namespace tools
{
    // hltools compile: csg -> bsp -> vis -> rad in one process on one map_data,
    // with the csg -> bsp geometry handed over in memory (phase 1; portals and
    // textures still round trip through prt / wa_ files)
    int run_compile(int argc, char **argv);
}
