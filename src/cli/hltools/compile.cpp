#include "compile.h"

#include <chrono>
#include <cstdio>
#include <string>
#include <utility>

#include "../../common/cmdline.h"
#include "../../common/error.h"
#include "../../common/filesystem.h"
#include "../../common/log.h"
#include "../../common/threads.h"
#include "format/bsp/file.h"
#include "format/bsp/usage_chart.h"
#include "../../csg/bsp_output.h"
#include "../../csg/map_parser.h"
#include "../tools/csg_tool.h"
#include "../tools/bsp_tool.h"
#include "../tools/vis_tool.h"
#include "../tools/rad_tool.h"

namespace tools
{
    namespace
    {
        void print_compile_help()
        {
            logging::console(
                "usage\n"
                "  hltools compile [options] <map>\n"
                "\n"
                "  runs csg, bsp, vis and rad in one pass with in-memory handoff and a\n"
                "  single combined log. accepts every stage option - see\n"
                "  'hltools csg -h', 'bsp -h', 'vis -h', 'rad -h' - plus:\n"
                "\n"
                "  -dumpintermediates    write the csg intermediate files to disk\n"
                "  -nochart              skip the bsp usage chart\n"
                "  -threads <n>          worker threads             (default: all cores)\n"
                "\n"
                "common picks: -fast for iteration builds, -extra -gpu for\n"
                "quality compiles, -nowadtextures (csg) to embed all textures.\n");
        }
    }

    int run_compile(int argc, char **argv)
    {
        cli::args args(argc, argv);
        const bool want_help = args.has("-h") || args.has("-help") || args.has("--help");
        if (args.empty() || args.map_name().empty() || want_help)
        {
            print_compile_help();
            return want_help ? 0 : 1;
        }

        threads::set_count(args.int_value("-threads", 0));

        std::string map_path = fs::with_extension(args.map_name(), ".map");
        std::string base = fs::strip_extension(map_path);
        std::string bsp_path = base + ".bsp";

        // one log for the whole compile: logs/<map>log
        {
            std::string dir = fs::directory(base);
            std::string logdir = dir.empty() ? "logs" : dir + "/logs";
            fs::make_directory(logdir);
            std::string log_path = logdir + "/" + fs::filename(base) + ".log";
            logging::open(log_path.c_str());
        }
        logging::banner("compile");

        if (args.has("-onlyents"))
            err::fatal("-onlyents is a csg-only mode; run 'hltools csg -onlyents <map>'");

        bool dump_intermediates = args.has("-dumpintermediates")
            || args.has("--dump-intermediates");

        // one combined settings table for all four stages
        logging::setting("threads", std::to_string(threads::count()).c_str(), "varies",
                         args.has("-threads"));
        csg::csg_options csg_options = parse_csg_options(args, map_path);
        bool optimize_lights = !args.has("-nolightopt");
        bsp::bsp_options bsp_options = parse_bsp_options(args);
        vis::vis_options vis_options = parse_vis_options(args, base + ".prt");
        rad::rad_options rad_options = parse_rad_options(args, argc, argv, base);
        logging::flush_settings();

        auto start = std::chrono::steady_clock::now();

        // ---- csg ----------------------------------------------------------
        csg::map_source source = csg::load_map_file(map_path);
        csg::csg_result result = csg::run_csg(std::move(source), csg_options);

        // build the intermediates before build_bsp_data mutates the entities,
        // exactly like the file based flow
        csg::intermediate_data inter = csg::build_intermediate_data(result, csg_options.brush);
        if (dump_intermediates && !csg::write_intermediate_files(base, inter, result))
            err::fatal("could not write csg intermediate files");

        // rad still reads the textures from the temp wad on disk (phase 2
        // hands them over in memory)
        if (!fs::write_all(base + ".wa_", result.temp_wad.data(), result.temp_wad.size()))
            err::fatal("could not write '%s'", (base + ".wa_").c_str());

        format::map_data map = csg::build_bsp_data(result, optimize_lights);

        // ---- bsp (csg handed over in memory) ------------------------------
        bsp::bsp_input input;
        for (int hull = 0; hull < bsp::num_hulls; hull++)
        {
            input.surfaces[hull] = std::move(inter.surfaces[hull]);
            input.brushes[hull] = std::move(inter.brushes[hull]);
        }
        input.hull_sizes = std::move(inter.hull_sizes);
        input.planes = std::move(inter.planes);
        bsp::run_bsp(map, base, bsp_options, std::move(input));

        if (logging::had_leak() || logging::had_error())
        {
            // keep the tree and the pts pointfile on disk for leak debugging,
            // and stop before vis and rad like the file based pipeline
            format::bsp_file::write(bsp_path, map);
            auto leak_end = std::chrono::steady_clock::now();
            std::chrono::duration<double> leak_elapsed = leak_end - start;
            logging::done(leak_elapsed.count());
            logging::close();
            return 1;
        }

        // ---- vis (portals via prt for now) -------------------------------
        vis::run_vis(map, vis_options);

        // ---- rad -----------------------------------------------------------
        // if the lightmap atlas overflows, rad skips lighting; keep the post vis
        // state for the on disk bsp then, like the file based pipeline does
        format::map_data unlit = map;
        int alloc_block_pages = -1;
        bool lit = rad::run_rad(map, base, rad_options, &alloc_block_pages);
        std::remove((base + ".wa_").c_str());

        if (lit)
        {
            if (!args.has("-nochart"))
                format::print_usage_chart(map, false, alloc_block_pages);
            if (!format::bsp_file::write(bsp_path, map))
                err::fatal("could not write bsp '%s'", bsp_path.c_str());
        }
        else if (!format::bsp_file::write(bsp_path, unlit))
        {
            err::fatal("could not write bsp '%s'", bsp_path.c_str());
        }

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        logging::done(elapsed.count());
        logging::close();
        return (lit && !logging::had_error()) ? 0 : 1;
    }
}
