#include "decompile_tool.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include "../../common/filesystem.h"
#include "../../common/log.h"
#include "../../common/string_util.h"
#include "../../decompile/map_decompiler.h"
#include "format/bsp/data.h"
#include "format/bsp/file.h"
#include "format/bsp/texture_lump.h"
#include "format/wad/archive.h"

namespace stdfs = std::filesystem;

namespace tools
{
    namespace
    {
        void print_help()
        {
            logging::console(
                "usage\n"
                "  hltools decompile <map.bsp> <output.map> [-wad <output.wad>] [-force]\n"
                "\n"
                "  reconstructs editable Valve 220 brushes from the BSP's hull-0 tree.\n"
                "  original brush grouping was lost during compilation and cannot be restored.\n"
                "  embedded source textures are written to <output>.wad automatically.\n"
                "\n"
                "options\n"
                "  -wad <file>  override the automatic companion WAD path\n"
                "  -force       overwrite existing output files\n");
        }

        bool is_help(const char *value)
        {
            return str::iequals(value, "-h") || str::iequals(value, "-help")
                || str::iequals(value, "--help") || str::iequals(value, "help");
        }

        bool extension_is(const std::string &path, const char *extension)
        {
            return str::iequals(stdfs::path(path).extension().string().c_str(), extension);
        }

        bool is_internal_rad_texture(const std::string &name)
        {
            return name.size() >= 5 && (name[0] == '_' || name[0] == '{')
                && str::istarts_with(name.c_str() + 1, "_rad");
        }
    }

    int run_decompile_tool(int argc, char **argv)
    {
        if (argc < 2 || is_help(argv[1]))
        {
            print_help();
            return argc < 2 ? 1 : 0;
        }

        bool force = false;
        bool explicit_wad = false;
        std::string wad_path;
        std::vector<std::string> positional;
        for (int i = 1; i < argc; i++)
        {
            if (str::iequals(argv[i], "-force"))
            {
                force = true;
                continue;
            }
            if (str::iequals(argv[i], "-wad"))
            {
                if (++i >= argc)
                {
                    logging::console("decompile: -wad requires an output path\n");
                    return 1;
                }
                wad_path = argv[i];
                explicit_wad = true;
                continue;
            }
            if (argv[i][0] == '-')
            {
                logging::console("decompile: unknown option '%s'\n", argv[i]);
                return 1;
            }
            positional.emplace_back(argv[i]);
        }

        if (positional.size() != 2 || !extension_is(positional[0], ".bsp")
            || !extension_is(positional[1], ".map"))
        {
            logging::console("decompile: expected a .bsp source and .map destination\n");
            return 1;
        }
        if (!wad_path.empty() && !extension_is(wad_path, ".wad"))
        {
            logging::console("decompile: companion texture output must end in .wad\n");
            return 1;
        }
        if (fs::exists(positional[1]) && !force)
        {
            logging::console("decompile: '%s' already exists (use -force to overwrite)\n",
                             positional[1].c_str());
            return 1;
        }
        format::map_data map;
        if (!format::bsp_file::load(positional[0], map))
        {
            logging::console("decompile: could not load '%s'\n", positional[0].c_str());
            return 1;
        }

        std::vector<format::mip_texture> textures;
        std::vector<std::string> external;
        std::string error;
        decompile::map_options options;
        if (!format::collect_bsp_textures(map, textures, &external, &error))
        {
            logging::console("decompile: %s\n", error.c_str());
            return 1;
        }
        textures.erase(std::remove_if(textures.begin(), textures.end(),
                                       [](const format::mip_texture &texture)
                                       { return is_internal_rad_texture(texture.name); }),
                       textures.end());
        if (explicit_wad && textures.empty())
        {
            logging::console("decompile: BSP has no embedded source textures for a companion WAD\n");
            return 1;
        }
        if (!textures.empty())
        {
            if (wad_path.empty())
                wad_path = fs::with_extension(positional[1], ".wad");
            if (fs::exists(wad_path) && !force)
            {
                logging::console("decompile: '%s' already exists (use -force to overwrite)\n",
                                 wad_path.c_str());
                return 1;
            }
            // csg resolves worldspawn wad entries from its working directory,
            // not relative to the map, so store an absolute path that works
            // immediately regardless of where the command is launched
            options.additional_wad = stdfs::absolute(stdfs::path(wad_path)).lexically_normal().string();
            options.replace_wad_list = external.empty();
        }

        decompile::map_result result;
        if (!decompile::reconstruct_map(map, options, result, &error))
        {
            logging::console("decompile: %s\n", error.c_str());
            return 1;
        }
        if (!wad_path.empty() && !format::write_wad3(wad_path, textures, &error))
        {
            logging::console("decompile: %s\n", error.c_str());
            return 1;
        }
        if (!fs::write_all(positional[1], result.text.data(), result.text.size()))
        {
            logging::console("decompile: could not write '%s'\n", positional[1].c_str());
            return 1;
        }

        logging::console("wrote %s (%zu reconstructed brushes, %zu textured sides, %zu generated sides)\n",
                         positional[1].c_str(), result.brushes,
                         result.textured_sides, result.generated_sides);
        if (result.texture_splits != 0)
            logging::console("preserved texture boundaries with %zu convex cell splits\n",
                             result.texture_splits);
        if (!wad_path.empty())
            logging::console("wrote %s (%zu embedded source textures)\n",
                             wad_path.c_str(), textures.size());
        if (result.discarded_cells != 0)
            logging::console("warning: discarded %zu degenerate BSP cells\n", result.discarded_cells);
        if (result.unresolved_texture_boundaries != 0)
            logging::console("warning: %zu texture boundaries exceeded the split-depth limit\n",
                             result.unresolved_texture_boundaries);
        return 0;
    }
}
