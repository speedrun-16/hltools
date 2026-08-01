#include <cstring>
#include <string>
#include <vector>

#include "../../common/log.h"
#include "../../common/string_util.h"
#include "../tools/csg_tool.h"
#include "../tools/bsp_tool.h"
#include "../tools/vis_tool.h"
#include "../tools/rad_tool.h"
#include "../tools/lightmap_tool.h"
#include "../tools/wad_tool.h"
#include "../tools/decompile_tool.h"
#include "../tools/model_tool.h"
#include "compile.h"

// hltools: single-binary goldsrc map and asset toolchain

namespace
{
    void print_help()
    {
        logging::banner("goldsrc map and asset toolchain");
        logging::console(
            "  tools for working with GoldSrc maps, models, and assets.\n"
            "\n"
            "usage\n"
            "  hltools <command> [options] <target>\n"
            "\n"
            "map compilation\n"
            "  compile      csg -> bsp -> vis -> rad in one pass\n"
            "  csg          carve brushes into geometry and embed textures\n"
            "  bsp          build the bsp tree, clip hulls, and portals\n"
            "  vis          compute the potentially visible set (pvs)\n"
            "  rad          radiosity lighting\n"
            "\n"
            "map / bsp tools\n"
            "  bsp info     report a compiled bsp's contents and limits\n"
            "  bsp pack     collect a BSP's resources and generate its .res file\n"
            "  bsp port     port a Source BSP to GoldSrc MAP and assets\n"
            "  lightmap     export compiled lightmaps as a 24-bit bmp atlas\n"
            "  ripent       import / export the entity lump                  (planned)\n"
            "  decompile    restore or reconstruct a GoldSrc Valve 220 MAP\n"
            "\n"
            "assets\n"
            "  wad          list / extract / build wad texture archives\n"
            "  model        convert Source models and build skybox models;\n"
            "               GoldSrc model compile/decompile is planned\n"
            "\n"
            "common options\n"
            "  -threads <n>   worker threads                       (default: all cores)\n"
            "  -h, -help      show help - per command: hltools <command> -h\n"
            "\n"
            "run 'hltools <command> -h' for a command's options.\n");
    }

    bool is_help_token(const char *token)
    {
        return str::iequals(token, "-h") || str::iequals(token, "-help")
            || str::iequals(token, "--help") || str::iequals(token, "help");
    }
}

int main(int argc, char **argv)
{
    logging::init_console();

    if (argc < 2 || is_help_token(argv[1]))
    {
        print_help();
        return argc < 2 ? 1 : 0;
    }

    const char *command = argv[1];

    // hand the subcommand the remaining arguments, keeping argv[0] (the exe
    // path) for tool folder relative lookups like the lightsrad cascade
    std::vector<char *> sub_argv;
    sub_argv.push_back(argv[0]);
    for (int i = 2; i < argc; i++)
        sub_argv.push_back(argv[i]);
    sub_argv.push_back(nullptr);
    int sub_argc = (int)sub_argv.size() - 1;

    if (str::iequals(command, "compile"))
        return tools::run_compile(sub_argc, sub_argv.data());
    if (str::iequals(command, "csg"))
        return tools::run_csg_tool(sub_argc, sub_argv.data());
    if (str::iequals(command, "bsp"))
        return tools::run_bsp_tool(sub_argc, sub_argv.data());
    if (str::iequals(command, "vis"))
        return tools::run_vis_tool(sub_argc, sub_argv.data());
    if (str::iequals(command, "rad"))
        return tools::run_rad_tool(sub_argc, sub_argv.data());
    if (str::iequals(command, "lightmap"))
        return tools::run_lightmap_tool(sub_argc, sub_argv.data());
    if (str::iequals(command, "wad"))
        return tools::run_wad_tool(sub_argc, sub_argv.data());
    if (str::iequals(command, "decompile"))
        return tools::run_decompile_tool(sub_argc, sub_argv.data());
    if (str::iequals(command, "model"))
        return tools::run_model_tool(sub_argc, sub_argv.data());

    if (str::iequals(command, "ripent"))
    {
        logging::console("hltools %s: planned, not available yet\n", command);
        return 1;
    }

    logging::console("hltools: unknown command '%s'\nrun 'hltools' for the command list\n",
                     command);
    return 1;
}
