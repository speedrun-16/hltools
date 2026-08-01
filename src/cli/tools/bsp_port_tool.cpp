#include "bsp_port_tool.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#include "../../common/filesystem.h"
#include "../../common/limits.h"
#include "../../common/log.h"
#include "../../common/string_util.h"
#include "../../decompile/source/map.h"
#include "format/vbsp/file.h"
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
                "  hltools bsp port <source.bsp> <output.map> [options]\n"
                "\n"
                "  ports a Source BSP to GoldSrc MAP, WAD, models, and sky assets.\n"
                "\n"
                "options\n"
                "  -wad <file>         companion WAD path\n"
                "  -game <dir>         content root; repeatable\n"
                "  -toolwad <file>     compiler-texture WAD\n"
                "  -skysize <n>        sky face size, 64..4096; 0 skips (default 256)\n"
                "  -skyexposure <n>    HDR sky brightness, 0.01..16 (default 1)\n"
                "  -maxtexsize <n>     WAD texture cap, 16..512 in steps of 16\n"
                "  -fulltex <material> bypass the texture cap; repeatable\n"
                "  -force              overwrite output files\n");
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

        bool write_skybox(const std::string &map_path,
                          const decompile::skybox_result &sky,
                          bool force, std::string &error)
        {
            if (sky.faces.empty())
                return true;
            std::string dir = fs::directory(map_path);
            std::string env = dir.empty() ? "gfx/env" : dir + "/gfx/env";
            if (!fs::make_directory(env))
            {
                error = "could not create '" + env + "'";
                return false;
            }
            for (const decompile::skybox_face &face : sky.faces)
            {
                std::string path = env + "/" + face.filename;
                if (fs::exists(path) && !force)
                {
                    error = "'" + path + "' already exists (use -force)";
                    return false;
                }
                if (!fs::write_all(path, face.tga.data(), face.tga.size()))
                {
                    error = "could not write '" + path + "'";
                    return false;
                }
            }
            return true;
        }

        bool write_models(const std::string &map_path,
                          const decompile::model_result &models,
                          bool force, std::string &error)
        {
            std::string root = fs::directory(map_path);
            for (const decompile::converted_model &model : models.models)
            {
                std::string path = root.empty() ? model.path : root + "/" + model.path;
                std::string dir = fs::directory(path);
                if (!dir.empty() && !fs::make_directory(dir))
                {
                    error = "could not create '" + dir + "'";
                    return false;
                }
                if (fs::exists(path) && !force)
                {
                    error = "'" + path + "' already exists (use -force)";
                    return false;
                }
                if (!fs::write_all(path, model.data.data(), model.data.size()))
                {
                    error = "could not write '" + path + "'";
                    return false;
                }
            }
            return true;
        }

        int run_port(const std::string &bsp_path, const std::string &map_path,
                     std::string wad_path, const std::vector<std::string> &game_dirs,
                     const std::string &tool_wad, unsigned sky_size,
                     double sky_exposure, unsigned max_texture_size,
                     const std::vector<std::string> &full_size_textures, bool force)
        {
            format::source_map_data source;
            std::string error;
            if (!format::source_bsp_file::load(bsp_path, source, &error))
            {
                logging::console("bsp port: %s\n", error.c_str());
                return 1;
            }
            if (fs::exists(map_path) && !force)
            {
                logging::console("bsp port: '%s' already exists (use -force)\n",
                                 map_path.c_str());
                return 1;
            }
            if (wad_path.empty())
                wad_path = fs::with_extension(map_path, ".wad");

            decompile::source_options options;
            options.game_dirs = game_dirs;
            options.sky_size = sky_size;
            options.sky_exposure = sky_exposure;
            options.max_texture_size = max_texture_size;
            options.full_size_textures = full_size_textures;
            options.additional_wad =
                stdfs::absolute(stdfs::path(wad_path)).lexically_normal().string();
            if (!tool_wad.empty())
                options.tool_wad =
                    stdfs::absolute(stdfs::path(tool_wad)).lexically_normal().string();

            decompile::source_result result;
            if (!decompile::port_source_map(source, options, result, &error))
            {
                logging::console("bsp port: %s\n", error.c_str());
                return 1;
            }
            if (!fs::write_all(map_path, result.text.data(), result.text.size()))
            {
                logging::console("bsp port: could not write '%s'\n", map_path.c_str());
                return 1;
            }
            if (!result.wad_textures.empty())
            {
                if (fs::exists(wad_path) && !force)
                {
                    logging::console("bsp port: '%s' already exists (use -force)\n",
                                     wad_path.c_str());
                    return 1;
                }
                if (!format::write_wad3(wad_path, result.wad_textures, &error))
                {
                    logging::console("bsp port: %s\n", error.c_str());
                    return 1;
                }
            }
            if (!write_skybox(map_path, result.skybox, force, error)
                || !write_models(map_path, result.models, force, error))
            {
                logging::console("bsp port: %s\n", error.c_str());
                return 1;
            }

            logging::console("wrote %s (vbsp v%d: %zu entities, %zu world brushes, "
                             "%zu detail brushes, %zu entity brushes, %zu sides)\n",
                             map_path.c_str(), source.version, result.entities,
                             result.world_brushes, result.detail_brushes,
                             result.entity_brushes, result.sides);
            if (!result.wad_textures.empty())
                logging::console("wrote %s (%zu textures, %zu unresolved)\n",
                                 wad_path.c_str(), result.converted_materials,
                                 result.placeholder_materials);
            if (!result.engine_textures.empty() && tool_wad.empty())
                logging::console("warning: %zu compiler textures need -toolwad\n",
                                 result.engine_textures.size());
            if (result.unlit_materials != 0)
                logging::console("preserved %zu unlit materials\n",
                                 result.unlit_materials);
            if (!result.skybox.faces.empty())
                logging::console("wrote gfx/env/%s{bk,dn,ft,lf,rt,up}.tga (%zu/6)\n",
                                 result.skybox.sky_name.c_str(),
                                 result.skybox.faces.size());
            if (result.skybox.missing != 0)
                logging::console("warning: %zu sky faces unresolved\n",
                                 result.skybox.missing);
            if (result.player_start)
                logging::console("selected one player start; removed %zu extras\n",
                                 result.dropped_spawns);
            if (!result.models.models.empty() || result.models.failed != 0)
                logging::console("converted %zu models (%zu placed, %zu failed)\n",
                                 result.models.converted, result.prop_entities,
                                 result.models.failed);
            if (result.models.missing_skins != 0)
                logging::console("warning: %zu model skins unresolved\n",
                                 result.models.missing_skins);
            logging::console("%zu brush models of %d\n", result.brush_models + 1,
                             limits::max_map_models);
            if ((int)result.brush_models + 1 > limits::max_map_models)
                logging::console("warning: map exceeds the model limit\n");
            if (result.skipped_brushes != 0)
                logging::console("warning: skipped %zu degenerate brushes\n",
                                 result.skipped_brushes);
            if (result.skipped_displacement_faces != 0
                || result.skipped_displacement_triangles != 0)
                logging::console("warning: skipped %zu displacement faces and %zu "
                                 "triangles\n", result.skipped_displacement_faces,
                                 result.skipped_displacement_triangles);
            return 0;
        }
    }

    int run_bsp_port_tool(int argc, char **argv)
    {
        if (argc < 2 || is_help(argv[1]))
        {
            print_help();
            return argc < 2 ? 1 : 0;
        }

        bool force = false;
        std::string wad_path;
        std::vector<std::string> game_dirs;
        std::string tool_wad;
        unsigned sky_size = 256;
        double sky_exposure = 1.0;
        unsigned max_texture_size = 512;
        std::vector<std::string> full_size_textures;
        std::vector<std::string> positional;
        for (int i = 1; i < argc; i++)
        {
            if (str::iequals(argv[i], "-force"))
            {
                force = true;
                continue;
            }
            if (str::iequals(argv[i], "-wad") || str::iequals(argv[i], "-toolwad")
                || str::iequals(argv[i], "-game") || str::iequals(argv[i], "-skysize")
                || str::iequals(argv[i], "-skyexposure")
                || str::iequals(argv[i], "-maxtexsize")
                || str::iequals(argv[i], "-fulltex"))
            {
                const std::string option = argv[i];
                if (++i >= argc || argv[i][0] == '\0')
                {
                    logging::console("bsp port: %s requires a value\n", option.c_str());
                    return 1;
                }
                if (str::iequals(option.c_str(), "-wad"))
                    wad_path = argv[i];
                else if (str::iequals(option.c_str(), "-toolwad"))
                    tool_wad = argv[i];
                else if (str::iequals(option.c_str(), "-game"))
                    game_dirs.emplace_back(argv[i]);
                else if (str::iequals(option.c_str(), "-fulltex"))
                    full_size_textures.emplace_back(argv[i]);
                else if (str::iequals(option.c_str(), "-skysize"))
                {
                    char *end = nullptr;
                    long value = std::strtol(argv[i], &end, 10);
                    if (!end || *end != '\0' || value < 0 || value > 4096
                        || (value != 0 && (value < 64 || (value & (value - 1)) != 0)))
                    {
                        logging::console("bsp port: -skysize expects 0 or a power of "
                                         "two from 64..4096\n");
                        return 1;
                    }
                    sky_size = (unsigned)value;
                }
                else if (str::iequals(option.c_str(), "-maxtexsize"))
                {
                    char *end = nullptr;
                    long value = std::strtol(argv[i], &end, 10);
                    if (!end || *end != '\0' || value < 16 || value > 512
                        || value % 16 != 0)
                    {
                        logging::console("bsp port: -maxtexsize expects a multiple "
                                         "of 16 from 16..512\n");
                        return 1;
                    }
                    max_texture_size = (unsigned)value;
                }
                else
                {
                    char *end = nullptr;
                    double value = std::strtod(argv[i], &end);
                    if (!end || *end != '\0' || !std::isfinite(value)
                        || value < 0.01 || value > 16.0)
                    {
                        logging::console("bsp port: -skyexposure expects 0.01..16\n");
                        return 1;
                    }
                    sky_exposure = value;
                }
                continue;
            }
            if (argv[i][0] == '-')
            {
                logging::console("bsp port: unknown option '%s'\n", argv[i]);
                return 1;
            }
            positional.emplace_back(argv[i]);
        }

        if (positional.size() != 2 || !extension_is(positional[0], ".bsp")
            || !extension_is(positional[1], ".map"))
        {
            logging::console("bsp port: expected a Source .bsp and .map destination\n");
            return 1;
        }
        if (!wad_path.empty() && !extension_is(wad_path, ".wad"))
        {
            logging::console("bsp port: companion output must end in .wad\n");
            return 1;
        }
        return run_port(positional[0], positional[1], wad_path, game_dirs,
                        tool_wad, sky_size, sky_exposure, max_texture_size,
                        full_size_textures, force);
    }
}
