#include "csg_tool.h"

#include <chrono>
#include <cstdlib>
#include <utility>

#include "../../common/error.h"
#include "../../common/filesystem.h"
#include "../../common/log.h"
#include "../../common/threads.h"
#include "format/bsp/data.h"
#include "format/bsp/file.h"
#include "../../csg/bsp_output.h"
#include "../../csg/map_parser.h"

namespace tools
{
    namespace
    {
        csg::clip_type clip_type_arg(const cli::args &args)
        {
            csg::clip_type clip = csg::clip_type::simple;
            const char *value = args.value("-cliptype", nullptr);
            if (value && !csg::parse_clip_type(value, clip))
                err::fatal("-cliptype must be smallest, normalized, simple, precise, or legacy");
            return clip;
        }
    }

    csg::csg_options parse_csg_options(const cli::args &args, const std::string &map_path)
    {
        csg::csg_options options;
        options.brush.noclip = args.has("-noclip");
        options.brush.only_entities = args.has("-onlyents");
        options.brush.nullify_trigger = args.has("-nullifytrigger");
        options.brush.world_extent = (vec_t)args.float_value("-worldextent", options.brush.world_extent);
        options.brush.global_scale = (vec_t)args.float_value("-scale", options.brush.global_scale);
        options.brush.clip = clip_type_arg(args);
        options.brush_union_threshold = (vec_t)args.float_value("-brushunion", options.brush_union_threshold);
        options.wad.map_path = map_path;
        options.wad.wad_textures = args.has("-wadtextures");
        options.wad.wad_auto_detect = !args.has("-nowadautodetect");
        for (const auto &include : args.values("-wadinclude"))
            options.wad.wad_include.push_back(include);
        if (const char *wad_cfg_file = args.value("-wadcfgfile", nullptr))
            options.wad.wad_cfg_file = wad_cfg_file;
        if (const char *wad_config = args.value("-wadconfig", nullptr))
            options.wad.wad_config_name = wad_config;
        if (const char *hull_file = args.value("-hullfile", nullptr))
            options.hull_file_path = hull_file;
        options.sky_clip = !args.has("-noskyclip");
        if (const char *null_file = args.value("-nullfile", nullptr))
            options.invisible_items = csg::load_invisible_items(null_file);

        logging::setting("wadtextures", options.wad.wad_textures ? "on" : "off", "off",
                         options.wad.wad_textures);

        // embedding drops every wad from the runtime list, so the options that
        // only shape that list have nothing left to act on. -wadcfgfile and
        // -wadconfig still matter: they say which wads the textures are read
        // from in the first place.
        if (!options.wad.wad_textures)
        {
            if (args.has("-nowadautodetect"))
                logging::warn("-nowadautodetect has no effect without -wadtextures: "
                              "embedded textures leave no wad list to prune");
            if (!args.values("-wadinclude").empty())
                logging::warn("-wadinclude has no effect without -wadtextures: "
                              "every used texture is already embedded");
        }
        logging::setting("clip type", csg::clip_type_name(options.brush.clip), "simple",
                         args.has("-cliptype"));
        return options;
    }

    namespace
    {
        void print_csg_help()
        {
            logging::console(
                "usage\n"
                "  hltools csg [options] <map>        (standalone: hlcsg)\n"
                "\n"
                "  carves brushes into renderable geometry, builds the clipping hull\n"
                "  brushes, and resolves the map's textures from its wads.\n"
                "\n"
                "textures\n"
                "  -wadtextures          keep textures in the wads and reference them at\n"
                "                        runtime (default: embed them into the bsp)\n"
                "  -wadinclude <name>    embed textures from wads matching <name> (repeatable)\n"
                "  -nowadautodetect      disable wad auto-detection\n"
                "  -wadcfgfile <file>    wad configuration file\n"
                "  -wadconfig <name>     named configuration from the wad config file\n"
                "\n"
                "geometry\n"
                "  -cliptype <type>      smallest | normalized | simple | precise | legacy\n"
                "                                                       (default: simple)\n"
                "  -noclip               don't create the clipping hulls\n"
                "  -noskyclip            don't clip the world with sky brushes\n"
                "  -scale <n>            scale the whole map\n"
                "  -worldextent <n>      world bounds per axis              (default: 65536)\n"
                "  -brushunion <n>       warn above overlap ratio <n> (0.1 = 10%%)\n"
                "  -hullfile <file>      custom clipping hull sizes\n"
                "  -nullfile <file>      list of textures/entities to nullify\n"
                "  -nullifytrigger       convert AAATRIGGER faces to null\n"
                "\n"
                "entities\n"
                "  -onlyents             update the entity lump of an existing bsp only\n"
                "  -nolightopt           don't strip redundant named lights\n"
                "\n"
                "misc\n"
                "  -threads <n>          worker threads             (default: all cores)\n");
        }
    }

    int run_csg_tool(int argc, char **argv)
    {
        cli::args args(argc, argv);
        const bool want_help = args.has("-h") || args.has("-help") || args.has("--help");
        if (args.empty() || args.map_name().empty() || want_help)
        {
            print_csg_help();
            return want_help ? 0 : 1;
        }

        threads::set_count(args.int_value("-threads", 0));

        std::string map_path = fs::with_extension(args.map_name(), ".map");
        std::string base = fs::strip_extension(map_path);
        logging::open_stage_log(base, "csg");
        logging::banner("csg");

        logging::setting("threads", std::to_string(threads::count()).c_str(), "varies",
                         args.has("-threads"));
        csg::csg_options options = parse_csg_options(args, map_path);
        bool optimize_lights = !args.has("-nolightopt");
        logging::flush_settings();

        csg::map_source map = csg::load_map_file(map_path);

        auto start = std::chrono::steady_clock::now();
        csg::csg_result result = csg::run_csg(std::move(map), options);
        std::string bsp_path = base + ".bsp";

        if (options.brush.only_entities)
        {
            // rewrite only the entity lump of the existing bsp
            format::map_data existing;
            if (!format::bsp_file::load(bsp_path, existing))
                err::fatal("-onlyents requires an existing bsp '%s'", bsp_path.c_str());
            csg::replace_entities(result, existing, optimize_lights);
            if (!format::bsp_file::write(bsp_path, existing))
                err::fatal("could not write bsp '%s'", bsp_path.c_str());

            logging::console("updated entity lump: %zu entities\n", result.map.entities.size());
            auto onlyents_end = std::chrono::steady_clock::now();
            std::chrono::duration<double> onlyents_elapsed = onlyents_end - start;
            logging::done(onlyents_elapsed.count());
            logging::close();
            return 0;
        }

        if (!csg::write_intermediate_files(base, result, options.brush))
            err::fatal("could not write csg intermediate files");
        // build_bsp_data mutates the entities (model numbers, reordering), so the
        // intermediate files above must be written first
        format::map_data bsp = csg::build_bsp_data(result, optimize_lights);
        if (!format::bsp_file::write(bsp_path, bsp))
            err::fatal("could not write bsp '%s'", bsp_path.c_str());

        logging::console("\nbuilt csg data: %zu entities, %zu texinfos, %zu texture bytes, cliptype %s\n",
                         result.entities.size(), result.texinfos.entries().size(), result.textures.size(),
                         csg::clip_type_name(options.brush.clip));

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        logging::done(elapsed.count());
        logging::close();
        return 0;
    }
}
