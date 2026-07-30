#include "vis_tool.h"

#include <chrono>
#include <cstdlib>

#include "../../common/error.h"
#include "../../common/filesystem.h"
#include "../../common/log.h"
#include "../../common/threads.h"
#include "format/bsp/data.h"
#include "format/bsp/file.h"
#include "format/bsp/usage_chart.h"
#include "compile_parameters.h"

namespace tools
{
    vis::vis_options parse_vis_options(const cli::args &args, const std::string &portal_path)
    {
        vis::vis_options options;
        options.portal_path = portal_path;
        options.fast = args.has("-fast");
        // full vis is the default: it culls best, and on portal-dense maps
        // the tighter source clipping prunes the flow recursion so hard that
        // it is also faster than normal vis; -fast stays the iteration mode
        options.full = !args.has("-nofull") && !args.has("-fast");
        options.nofixprt = args.has("-nofixprt");
        options.chart = true;
        options.maxdistance = (unsigned)std::abs(args.int_value("-maxdistance", 0));

        logging::setting("full vis", options.full ? "on" : "off", "on", !options.full);
        logging::setting("fast vis", options.fast ? "on" : "off", "off", options.fast);
        return options;
    }

    namespace
    {
        void print_vis_help()
        {
            logging::console(
                "usage\n"
                "  hltools vis [options] <map>\n"
                "\n"
                "  computes the potentially visible set: which leaves can see which,\n"
                "  so the engine only renders (and rad only bounces) what matters.\n"
                "\n"
                "  -nofull               normal instead of exact full visibility\n"
                "                        (full is the default: best culling, and usually\n"
                "                        no slower; -full is still accepted)\n"
                "  -fast                 quick approximate visibility (iteration builds)\n"
                "  -maxdistance <n>      recognized but unavailable (explicit error)\n"
                "  -nofixprt             skip the portal-file fixup pass\n"
                "  -threads <n>          worker threads             (default: all cores)\n");
        }
    }

    int run_vis_tool(int argc, char **argv)
    {
        cli::args command_line(argc, argv);
        const bool want_help = command_line.has("-h")
            || command_line.has("-help") || command_line.has("--help");
        if (command_line.empty() || command_line.map_name().empty() || want_help)
        {
            print_vis_help();
            return want_help ? 0 : 1;
        }

        std::string base = fs::strip_extension(command_line.map_name());
        std::string bsp_path = base + ".bsp";
        std::string portal_path = base + ".prt";

        logging::open_stage_log(base, "vis");
        logging::banner("vis");

        format::map_data map;
        if (!format::bsp_file::load(bsp_path, map))
            err::fatal("failed to load bsp '%s'", bsp_path.c_str());

        const tools::compile_parameters parameters =
            compile_parameters_from_bsp(map);
        staged_arguments arguments(
            argc, argv, parameters, cli::compiler_stage::vis);
        cli::args args(arguments.argc(), arguments.argv.data());

        threads::set_count(args.int_value("-threads", 0));
        logging::setting("threads", std::to_string(threads::count()).c_str(), "varies",
                         args.has("-threads"));
        vis::vis_options options = parse_vis_options(args, portal_path);
        logging::flush_settings();

        auto start = std::chrono::steady_clock::now();
        vis::run_vis(map, options);

        if (options.chart)
            format::print_usage_chart(map);

        if (!format::bsp_file::write(bsp_path, map))
            err::fatal("failed to write bsp '%s'", bsp_path.c_str());

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        logging::done(elapsed.count());
        logging::close();
        return 0;
    }
}
