#include "decompile_tool.h"

#include <algorithm>
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
#include "format/zip/archive.h"

namespace stdfs = std::filesystem;

namespace tools
{
    namespace
    {
        void print_help()
        {
            logging::console(
                "usage\n"
                "  hltools decompile <map.bsp> <output.map> [options]\n"
                "\n"
                "  restores an embedded MAP or reconstructs GoldSrc BSP geometry.\n"
                "\n"
                "options\n"
                "  -wad <file>   companion WAD path\n"
                "  -reconstruct  ignore an embedded MAP\n"
                "  -force        overwrite output files\n");
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
        bool ignore_embedded_source = false;
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
            if (str::iequals(argv[i], "-reconstruct"))
            {
                ignore_embedded_source = true;
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
            logging::console("decompile: companion output must end in .wad\n");
            return 1;
        }
        if (fs::exists(positional[1]) && !force)
        {
            logging::console("decompile: '%s' already exists (use -force)\n",
                             positional[1].c_str());
            return 1;
        }

        format::map_data map;
        if (!format::bsp_file::load(positional[0], map))
        {
            logging::console("decompile: could not load '%s'\n", positional[0].c_str());
            return 1;
        }

        if (!ignore_embedded_source && !map.embedded_zip.empty())
        {
            std::vector<unsigned char> source;
            std::string entry;
            std::string zip_error;
            if (!format::read_zip_map(map.embedded_zip, source, &entry, &zip_error))
            {
                logging::console("decompile: embedded source ZIP is invalid (%s); "
                                 "reconstructing the BSP\n", zip_error.c_str());
            }
            else
            {
                if (explicit_wad)
                    logging::console("decompile: embedded MAP found; ignoring -wad\n");
                if (!fs::write_all(positional[1], source.data(), source.size()))
                {
                    logging::console("decompile: could not write '%s'\n",
                                     positional[1].c_str());
                    return 1;
                }
                logging::console("wrote %s from embedded source '%s'\n",
                                 positional[1].c_str(), entry.c_str());
                return 0;
            }
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
            logging::console("decompile: BSP has no embedded textures\n");
            return 1;
        }
        if (!textures.empty())
        {
            if (wad_path.empty())
                wad_path = fs::with_extension(positional[1], ".wad");
            if (fs::exists(wad_path) && !force)
            {
                logging::console("decompile: '%s' already exists (use -force)\n",
                                 wad_path.c_str());
                return 1;
            }
            options.additional_wad =
                stdfs::absolute(stdfs::path(wad_path)).lexically_normal().string();
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

        logging::console("wrote %s (%zu brushes, %zu textured sides, %zu generated)\n",
                         positional[1].c_str(), result.brushes,
                         result.textured_sides, result.generated_sides);
        if (result.texture_splits != 0)
            logging::console("preserved texture boundaries with %zu splits\n",
                             result.texture_splits);
        if (!wad_path.empty())
            logging::console("wrote %s (%zu textures)\n", wad_path.c_str(),
                             textures.size());
        if (result.discarded_cells != 0)
            logging::console("warning: discarded %zu degenerate BSP cells\n",
                             result.discarded_cells);
        if (result.unresolved_texture_boundaries != 0)
            logging::console("warning: %zu texture boundaries exceeded the split limit\n",
                             result.unresolved_texture_boundaries);
        return 0;
    }
}
