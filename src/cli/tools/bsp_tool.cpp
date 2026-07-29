#include "bsp_tool.h"

#include <chrono>
#include <string>

#include "../../common/error.h"
#include "../../common/filesystem.h"
#include "../../common/log.h"
#include "../../common/threads.h"
#include "format/bsp/file.h"
#include "format/bsp/usage_chart.h"

namespace tools
{
    bsp::bsp_options parse_bsp_options(const cli::args &args)
    {
        bsp::bsp_options options;
        options.nofill = args.has("-nofill");
        options.noinsidefill = args.has("-noinsidefill");
        options.notjunc = args.has("-notjunc");
        options.nobrink = args.has("-nobrink");
        options.noclip = args.has("-noclip");
        options.noopt = args.has("-noopt");
        options.noclipnodemerge = args.has("-noclipnodemerge");
        options.leakonly = args.has("-leakonly");
        options.allleaks = args.has("-allleaks");
        options.nulltex = !args.has("-nonulltex");
        options.nohull2 = args.has("-nohull2");
        options.maxnode_size = args.int_value("-maxnodesize", options.maxnode_size);
        options.subdivide_size = args.int_value("-subdivide", options.subdivide_size);
        return options;
    }

    namespace
    {
        void print_bsp_help()
        {
            logging::console(
                "usage\n"
                "  hltools bsp [options] <map>        (standalone: hlbsp)\n"
                "\n"
                "  builds the bsp trees for all four hulls from csg's plane lists,\n"
                "  fills the outside, fixes t-junctions, and writes the portal file.\n"
                "\n"
                "tree\n"
                "  -maxnodesize <n>      largest node size before an axial split\n"
                "                                                       (default: 1024)\n"
                "  -subdivide <n>        face subdivision size            (default: 240)\n"
                "  -noopt                don't strip unused planes on write\n"
                "  -noclipnodemerge      don't merge duplicate clipnodes\n"
                "  -notjunc              don't fix t-junctions\n"
                "  -nobrink              don't fix brinks (stuck-on-edge clipnodes)\n"
                "\n"
                "hulls / fill\n"
                "  -noclip               don't build the clipping hulls\n"
                "  -nohull2              skip hull 2 (large monsters)\n"
                "  -nofill               don't fill the outside (leak debugging)\n"
                "  -noinsidefill         don't fill enclosed pockets\n"
                "  -leakonly             stop after the leak check\n"
                "  -allleaks             report every hole, not just the first, and
"
                "                        put all their paths in one pointfile
"
                "\n"
                "misc\n"
                "  -nonulltex            don't strip null faces\n"
                "  -threads <n>          worker threads             (default: all cores)\n");
        }
    }

    int run_bsp_tool(int argc, char **argv)
    {
        cli::args args(argc, argv);
        const bool want_help = args.has("-h") || args.has("-help") || args.has("--help");
        if (args.empty() || args.map_name().empty() || want_help)
        {
            print_bsp_help();
            return want_help ? 0 : 1;
        }

        threads::set_count(args.int_value("-threads", 0));

        std::string base = fs::strip_extension(fs::with_extension(args.map_name(), ".bsp"));
        logging::open_stage_log(base, "bsp");
        logging::banner("bsp");

        bsp::bsp_options options = parse_bsp_options(args);

        logging::setting("threads", std::to_string(threads::count()).c_str(), "varies",
                         args.has("-threads"));
        logging::flush_settings();

        std::string bsp_path = base + ".bsp";
        format::map_data map;
        if (!format::bsp_file::load(bsp_path, map))
            err::fatal("could not load bsp '%s'", bsp_path.c_str());

        auto start = std::chrono::steady_clock::now();
        bsp::run_bsp(map, base, options);

        if (!format::bsp_file::write(bsp_path, map))
            err::fatal("could not write bsp '%s'", bsp_path.c_str());

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        logging::done(elapsed.count());
        logging::close();
        // a leak (or hard error) fails the build so the editor's compile pipeline
        // stops here instead of running vis and rad on a leaking map the bsp and
        // pts pointfile are still written above, for debugging the leak
        return (logging::had_leak() || logging::had_error()) ? 1 : 0;
    }
}
