#include "lightmap_tool.h"

#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include "../../common/filesystem.h"
#include "../../common/log.h"
#include "../../common/string_util.h"
#include "format/bmp/file.h"
#include "format/bsp/data.h"
#include "format/bsp/file.h"
#include "format/bsp/lightmap_atlas.h"

namespace stdfs = std::filesystem;

namespace tools
{
    namespace
    {
        void print_help()
        {
            logging::console(
                "usage\n"
                "  hltools lightmap <map.bsp> <output.bmp> [-force]\n"
                "\n"
                "  exports every lit face and light style to one deterministic 24-bit\n"
                "  rgb atlas for fast visual or pixel-by-pixel comparisons.\n");
        }

        bool is_help(const char *s)
        {
            return str::iequals(s, "-h") || str::iequals(s, "-help")
                || str::iequals(s, "--help") || str::iequals(s, "help");
        }

        bool extension_is(const std::string &path, const char *extension)
        {
            return str::iequals(stdfs::path(path).extension().string().c_str(), extension);
        }
    }

    int run_lightmap_tool(int argc, char **argv)
    {
        if (argc < 2 || is_help(argv[1]))
        {
            print_help();
            return argc >= 2 ? 0 : 1;
        }
        bool force = false;
        std::vector<std::string> positional;
        for (int i = 1; i < argc; i++)
        {
            if (str::iequals(argv[i], "-force"))
                force = true;
            else if (argv[i][0] != '-')
                positional.emplace_back(argv[i]);
        }
        if (positional.size() != 2)
        {
            print_help();
            return 1;
        }
        const std::string &source = positional[0];
        const std::string &destination = positional[1];
        if (!extension_is(source, ".bsp") || !extension_is(destination, ".bmp"))
        {
            logging::console("lightmap: expected a .bsp source and .bmp destination\n");
            return 1;
        }
        if (fs::exists(destination) && !force)
        {
            logging::console("lightmap: '%s' already exists (use -force to overwrite)\n",
                             destination.c_str());
            return 1;
        }

        format::map_data map;
        if (!format::bsp_file::load(source, map))
        {
            logging::console("lightmap: could not load '%s'\n", source.c_str());
            return 1;
        }
        format::lightmap_atlas atlas;
        std::string error;
        if (!format::build_lightmap_atlas(map, atlas, &error))
        {
            logging::console("lightmap: %s\n", error.c_str());
            return 1;
        }
        if (!format::write_rgb_bmp(destination, atlas.width, atlas.height,
                                   atlas.pixels, &error))
        {
            logging::console("lightmap: %s\n", error.c_str());
            return 1;
        }
        logging::console("wrote %s (%ux%u, %d face%s, %d style tile%s)\n",
                         destination.c_str(), atlas.width, atlas.height,
                         atlas.faces, atlas.faces == 1 ? "" : "s",
                         atlas.tiles, atlas.tiles == 1 ? "" : "s");
        return 0;
    }
}
