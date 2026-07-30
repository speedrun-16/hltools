#include "decompile_tool.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "../../common/filesystem.h"
#include "../../common/limits.h"
#include "../../common/log.h"
#include "../../common/string_util.h"
#include "../../decompile/map_decompiler.h"
#include "../../decompile/source_map_decompiler.h"
#include "format/zip/archive.h"
#include "format/bsp/data.h"
#include "format/bsp/file.h"
#include "format/bsp/texture_lump.h"
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
                "  hltools decompile <map.bsp> <output.map> [-wad <output.wad>] [-force]\n"
                "                    [-game <dir>] [-skysize <n>] [-skyexposure <n>]\n"
                "                    [-maxtexsize <n>] [-fulltex <material> ...]\n"
                "                    [-reconstruct]\n"
                "\n"
                "  goldsrc maps: reconstructs editable Valve 220 brushes from the BSP's\n"
                "  hull-0 tree; embedded textures are written to <output>.wad.\n"
                "\n"
                "  source (VBSP) maps: ports brushes, entities and materials to a goldsrc\n"
                "  map plus a companion WAD converted from the map's VMT/VTF materials.\n"
                "  the 2D skybox is written as gfx/env/<sky>{bk,dn,ft,lf,rt,up}.tga next\n"
                "  to the map.\n"
                "\n"
                "options\n"
                "  -wad <file>   override the automatic companion WAD path\n"
                "  -force        overwrite existing output files\n"
                "  -reconstruct  ignore source.map and reconstruct from the BSP\n"
                "  -game <dir>   source content root for materials not in the pakfile;\n"
                "                repeat for each content tree, searched in order. a\n"
                "                source game splits content across trees, so a mod\n"
                "                normally needs its own plus the shared hl2 one:\n"
                "                -game .../cstrike -game .../hl2\n"
                "  -toolwad <wad> wad providing the engine textures (NULL, SKY,\n"
                "                AAATRIGGER ...), normally sdhlt.wad. these are not\n"
                "                synthesized, so without this they resolve nowhere;\n"
                "                the wad is added to the ported map's wad list\n"
                "  -skysize <n>  sky face edge length, 64..4096 (default 256, which every\n"
                "                stock GoldSrc sky uses); 1024+ can feed 'model skybox';\n"
                "                0 skips the skybox\n"
                "  -skyexposure <n> linear brightness multiplier for exported sky faces\n"
                "                in the range 0.01..16 (default 1 preserves Source RGB)\n"
                "  -maxtexsize <n> maximum WAD texture dimension, any multiple of 16\n"
                "                from 16..512 (default 512); lower values preserve UV\n"
                "                coverage while reducing BSP faces and lightmap density.\n"
                "                sizes between the powers of two (80, 96, 144 ...) let\n"
                "                you trade the least quality that still fits the limits\n"
                "  -fulltex <material> preserve one Source material at native size (up to\n"
                "                512), bypassing -maxtexsize; repeat for more textures\n");
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

        // peeks the first four bytes for the 'VBSP' ident so the tool can route
        // a source engine map to the porter instead of the goldsrc reconstructor
        bool bsp_is_source(const std::string &path)
        {
            std::ifstream input(path, std::ios::binary);
            char magic[4] = {};
            if (!input.read(magic, sizeof(magic)))
                return false;
            unsigned ident;
            std::memcpy(&ident, magic, sizeof(ident));
            return ident == format::vbsp_ident;
        }

        // ports a source engine (vbsp) map to valve 220 and builds a companion
        // wad from its embedded/game materials.
        // sky faces land in gfx/env beside the map, the layout a goldsrc mod
        // directory expects, so the folder can be copied straight into the game.
        bool write_skybox(const std::string &map_path, const decompile::skybox_result &sky,
                          bool force, std::string &error)
        {
            if (sky.faces.empty())
                return true;
            std::string dir = fs::directory(map_path);
            std::string env = (dir.empty() ? std::string("gfx/env") : dir + "/gfx/env");
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
                    error = "'" + path + "' already exists (use -force to overwrite)";
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

        // converted studio models keep their source relative path, so the output
        // folder mirrors a goldsrc mod directory and can be copied straight in
        bool write_models(const std::string &map_path, const decompile::model_result &models,
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
                    error = "'" + path + "' already exists (use -force to overwrite)";
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

        int run_source_decompile(const std::string &bsp_path, const std::string &map_path,
                                 std::string wad_path, const std::vector<std::string> &game_dirs,
                                 const std::string &tool_wad,
                                 unsigned sky_size, double sky_exposure,
                                 unsigned max_texture_size,
                                 const std::vector<std::string> &full_size_textures,
                                 bool force)
        {
            format::source_map_data source;
            std::string error;
            if (!format::source_bsp_file::load(bsp_path, source, &error))
            {
                logging::console("decompile: %s\n", error.c_str());
                return 1;
            }
            if (fs::exists(map_path) && !force)
            {
                logging::console("decompile: '%s' already exists (use -force to overwrite)\n",
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
            // csg resolves wad entries from its working directory, so store an
            // absolute path that works regardless of where the command is launched
            options.additional_wad =
                stdfs::absolute(stdfs::path(wad_path)).lexically_normal().string();
            if (!tool_wad.empty())
                options.tool_wad =
                    stdfs::absolute(stdfs::path(tool_wad)).lexically_normal().string();

            decompile::source_result result;
            if (!decompile::port_source_map(source, options, result, &error))
            {
                logging::console("decompile: %s\n", error.c_str());
                return 1;
            }
            if (!fs::write_all(map_path, result.text.data(), result.text.size()))
            {
                logging::console("decompile: could not write '%s'\n", map_path.c_str());
                return 1;
            }
            if (!result.wad_textures.empty())
            {
                if (fs::exists(wad_path) && !force)
                {
                    logging::console("decompile: '%s' already exists (use -force to overwrite)\n",
                                     wad_path.c_str());
                    return 1;
                }
                if (!format::write_wad3(wad_path, result.wad_textures, &error))
                {
                    logging::console("decompile: %s\n", error.c_str());
                    return 1;
                }
            }
            if (!write_skybox(map_path, result.skybox, force, error))
            {
                logging::console("decompile: %s\n", error.c_str());
                return 1;
            }
            if (!write_models(map_path, result.models, force, error))
            {
                logging::console("decompile: %s\n", error.c_str());
                return 1;
            }

            logging::console("wrote %s (vbsp v%d: %zu entities, %zu world brushes, "
                             "%zu func_detail brushes, %zu entity brushes, %zu sides)\n",
                             map_path.c_str(), source.version, result.entities,
                             result.world_brushes, result.detail_brushes,
                             result.entity_brushes, result.sides);
            if (!result.wad_textures.empty())
                logging::console("wrote %s (%zu textures converted, %zu unresolved)\n",
                                 wad_path.c_str(), result.converted_materials,
                                 result.placeholder_materials);
            if (!result.engine_textures.empty())
            {
                std::string names;
                for (const std::string &name : result.engine_textures)
                {
                    if (!names.empty())
                        names += ", ";
                    names += name;
                }
                if (tool_wad.empty())
                    logging::console("warning: the map uses %zu engine textures (%s) "
                                     "that no wad provides; pass -toolwad <sdhlt.wad> "
                                     "or csg will not find them\n",
                                     result.engine_textures.size(), names.c_str());
                else
                    logging::console("%zu engine textures (%s) resolve from %s\n",
                                     result.engine_textures.size(), names.c_str(),
                                     tool_wad.c_str());
            }
            else if (result.placeholder_materials != 0)
                logging::console("warning: no materials could be converted "
                                 "(%zu unresolved); try -game <content dir>\n",
                                 result.placeholder_materials);
            if (result.unlit_materials != 0)
                logging::console("preserved %zu UnlitGeneric materials as unlit "
                                 "textures\n", result.unlit_materials);
            if (!result.skybox.faces.empty())
                logging::console("wrote gfx/env/%s{bk,dn,ft,lf,rt,up}.tga "
                                 "(%zu of 6 sky faces)\n",
                                 result.skybox.sky_name.c_str(), result.skybox.faces.size());
            if (result.skybox.missing != 0)
                logging::console("warning: %zu sky faces could not be resolved\n",
                                 result.skybox.missing);
            if (result.player_start)
                logging::console("placed one info_player_start (dropped %zu team spawns)\n",
                                 result.dropped_spawns);
            if (result.realigned_sides != 0)
                logging::console("realigned %zu sides whose texture axes were edge-on "
                                 "to their plane\n", result.realigned_sides);
            if (result.masked_brushes != 0 || result.masked_entities != 0)
                logging::console("converted %zu masked ('{') textures: %zu world brushes "
                                 "to func_wall, %zu brush entities retagged rendermode "
                                 "solid\n",
                                 result.masked_materials, result.masked_brushes,
                                 result.masked_entities);
            if (!result.models.models.empty() || result.models.failed != 0)
                logging::console("converted %zu studio models (%zu placed as "
                                 "cycler_sprite, %zu failed)\n",
                                 result.models.converted, result.prop_entities,
                                 result.models.failed);
            if (result.models.missing_skins != 0)
                logging::console("warning: %zu model skins could not be resolved\n",
                                 result.models.missing_skins);
            if (result.merged_brush_entities != 0)
                logging::console("folded %zu inert brush entities into %zu\n",
                                 result.merged_brush_entities,
                                 result.brush_entity_groups);
            logging::console("%zu brush models of %d\n", result.brush_models + 1,
                             limits::max_map_models);
            if ((int)result.brush_models + 1 > limits::max_map_models)
                logging::console("warning: over the engine's model limit; the map "
                                 "will not compile as is\n");
            if (result.skybox_brushes != 0)
                logging::console("dropped %zu 3D skybox brushes "
                                 "(no goldsrc equivalent)\n",
                                 result.skybox_brushes);
            if (result.clip_brushes != 0)
                logging::console("gave %zu player clip volumes the CLIP texture\n",
                                 result.clip_brushes);
            if (result.translucent_entities != 0)
                logging::console("gave %zu brush entities their material's "
                                 "translucency (rendermode texture)\n",
                                 result.translucent_entities);
            if (result.window_brushes != 0)
                logging::console("converted %zu translucent brushes to func_wall "
                                 "(rendermode texture)\n", result.window_brushes);
            if (result.water_brushes != 0)
                logging::console("converted %zu water brushes to func_water\n",
                                 result.water_brushes);
            if (result.skipped_brushes != 0)
                logging::console("warning: skipped %zu degenerate brushes\n",
                                 result.skipped_brushes);
            if (result.displacement_faces != 0)
                logging::console("converted %zu displacement faces to %zu "
                                 "func_detail triangle brushes plus %zu "
                                 "simplified collision brushes\n",
                                 result.displacement_faces,
                                 result.displacement_brushes,
                                 result.displacement_collision_brushes);
            if (result.skipped_displacement_faces != 0
                || result.skipped_displacement_triangles != 0)
                logging::console("warning: skipped %zu displacement faces and "
                                 "%zu degenerate displacement triangles\n",
                                 result.skipped_displacement_faces,
                                 result.skipped_displacement_triangles);
            return 0;
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
            if (str::iequals(argv[i], "-toolwad"))
            {
                // wad supplying NULL/SKY/AAATRIGGER and friends, normally
                // sdhlt.wad; added to the ported map's worldspawn wad list
                if (++i >= argc)
                {
                    logging::console("decompile: -toolwad requires a wad path\n");
                    return 1;
                }
                tool_wad = argv[i];
                continue;
            }
            if (str::iequals(argv[i], "-game"))
            {
                // source content root (e.g. .../cstrike) for resolving materials
                // not embedded in the map's pakfile; used by the vbsp porter
                if (++i >= argc)
                {
                    logging::console("decompile: -game requires a content directory\n");
                    return 1;
                }
                game_dirs.emplace_back(argv[i]);
                continue;
            }
            if (str::iequals(argv[i], "-skysize"))
            {
                if (++i >= argc)
                {
                    logging::console("decompile: -skysize requires a face size\n");
                    return 1;
                }
                char *end = nullptr;
                long value = std::strtol(argv[i], &end, 10);
                if (!end || *end != '\0' || value < 0 || value > 4096
                    || (value != 0 && (value < 64 || (value & (value - 1)) != 0)))
                {
                    logging::console("decompile: -skysize expects 0 or a power of two "
                                     "between 64 and 4096\n");
                    return 1;
                }
                sky_size = (unsigned)value;
                continue;
            }
            if (str::iequals(argv[i], "-maxtexsize"))
            {
                if (++i >= argc)
                {
                    logging::console("decompile: -maxtexsize requires a dimension\n");
                    return 1;
                }
                char *end = nullptr;
                long value = std::strtol(argv[i], &end, 10);
                // goldsrc miptex only needs dimensions divisible by 16 (four
                // mip levels), not powers of two, so sizes like 80 or 96 are
                // legal and give finer control over the face/vertex budget
                if (!end || *end != '\0' || value < 16 || value > 512
                    || (value % 16) != 0)
                {
                    logging::console("decompile: -maxtexsize expects a multiple of 16 "
                                     "between 16 and 512\n");
                    return 1;
                }
                max_texture_size = (unsigned)value;
                continue;
            }
            if (str::iequals(argv[i], "-fulltex"))
            {
                if (++i >= argc || argv[i][0] == '\0')
                {
                    logging::console("decompile: -fulltex requires a material name\n");
                    return 1;
                }
                full_size_textures.emplace_back(argv[i]);
                continue;
            }
            if (str::iequals(argv[i], "-skyexposure"))
            {
                if (++i >= argc)
                {
                    logging::console("decompile: -skyexposure requires a multiplier\n");
                    return 1;
                }
                char *end = nullptr;
                double value = std::strtod(argv[i], &end);
                if (!end || *end != '\0' || !std::isfinite(value)
                    || value < 0.01 || value > 16.0)
                {
                    logging::console("decompile: -skyexposure expects a number "
                                     "between 0.01 and 16\n");
                    return 1;
                }
                sky_exposure = value;
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
        if (bsp_is_source(positional[0]))
            return run_source_decompile(positional[0], positional[1], wad_path,
                                        game_dirs, tool_wad, sky_size, sky_exposure,
                                        max_texture_size, full_size_textures, force);

        if (!game_dirs.empty())
            logging::console("decompile: -game only applies to source (vbsp) maps; ignoring\n");

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
            if (!format::read_zip_map(map.embedded_zip, source, &entry,
                                     &zip_error))
            {
                logging::console(
                    "decompile: embedded source ZIP is invalid (%s); "
                    "falling back to BSP reconstruction\n",
                    zip_error.c_str());
            }
            else
            {
                if (explicit_wad)
                {
                    logging::console(
                        "decompile: embedded MAP found; -wad is not needed "
                        "and will be ignored\n");
                }
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
