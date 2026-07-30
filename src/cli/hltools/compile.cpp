#include "compile.h"

#include <chrono>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include "../../common/cmdline.h"
#include "../../common/error.h"
#include "../../common/filesystem.h"
#include "../../common/log.h"
#include "../../common/threads.h"
#include "format/bsp/file.h"
#include "format/bsp/usage_chart.h"
#include "format/map/document.h"
#include "format/rad/texlights.h"
#include "format/zip/archive.h"
#include "../../csg/bsp_output.h"
#include "../../csg/map_parser.h"
#include "../tools/csg_tool.h"
#include "../tools/bsp_tool.h"
#include "../tools/compile_parameters.h"
#include "../tools/vis_tool.h"
#include "../tools/rad_tool.h"

namespace tools
{
    namespace
    {
        std::string source_number(float value)
        {
            char buffer[64];
            std::snprintf(buffer, sizeof(buffer), "%.9g", value);
            return buffer;
        }

        std::string enriched_map_source(
            const std::string &source,
            const std::vector<format::entity::pair> &compile_parameters,
            const std::vector<format::texlight> &used_texlights)
        {
            std::string enriched = source;
            format::erase_map_entities(
                enriched, {"info_compile_parameters", "info_texlights"});

            if (!compile_parameters.empty())
            {
                format::entity parameters;
                parameters.append("classname", "info_compile_parameters");
                for (const format::entity::pair &pair : compile_parameters)
                    parameters.append(pair.first, pair.second);
                format::append_map_entity(enriched, parameters);
            }

            if (!used_texlights.empty())
            {
                format::entity texlights;
                texlights.append("classname", "info_texlights");
                for (const format::texlight &light : used_texlights)
                {
                    texlights.append(
                        light.name,
                        source_number(light.value[0]) + " "
                            + source_number(light.value[1]) + " "
                            + source_number(light.value[2]));
                }
                format::append_map_entity(enriched, texlights);
            }
            return enriched;
        }

        void attach_embedded_source(
            format::map_data &map, const std::vector<unsigned char> &source,
            const std::vector<format::entity::pair> &compile_parameters,
            const std::vector<format::texlight> &used_texlights)
        {
            const std::string source_text(source.begin(), source.end());
            const std::string enriched = enriched_map_source(
                source_text, compile_parameters, used_texlights);
            std::vector<unsigned char> enriched_bytes(enriched.begin(),
                                                      enriched.end());
            std::string zip_error;
            if (!format::create_zip({
                    {format::embedded_map_name, enriched_bytes}
                }, map.embedded_zip, &zip_error))
            {
                err::fatal("could not create embedded source ZIP: %s",
                           zip_error.c_str());
            }
        }

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
                "  -noembedsource        do not embed the map source in the BSP\n"
                "  -nochart              skip the bsp usage chart\n"
                "  -threads <n>          worker threads             (default: all cores)\n"
                "\n"
                "common picks: -fast for iteration builds, -extra -gpu for\n"
                "quality compiles, -wadtextures (csg) to keep textures in the wads.\n");
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
        bool embed_source = !args.has("-noembedsource")
            && !args.has("--no-embed-source");

        std::vector<unsigned char> source_bytes;
        if (!fs::read_all(map_path, source_bytes))
            err::fatal("could not read map source '%s'",
                       map_path.c_str());
        const std::string source_text(
            source_bytes.begin(), source_bytes.end());
        const tools::compile_parameters source_parameters =
            compile_parameters_from_map(source_text);

        // the map entity supplies defaults and is prepended to each stage's
        // argument stream, so the real command line remains the final
        // authority for duplicate value options
        csg::map_source source = csg::load_map_file(map_path);
        staged_arguments csg_arguments(
            argc, argv, source_parameters, cli::compiler_stage::csg);
        staged_arguments bsp_arguments(
            argc, argv, source_parameters, cli::compiler_stage::bsp);
        staged_arguments vis_arguments(
            argc, argv, source_parameters, cli::compiler_stage::vis);
        staged_arguments rad_arguments(
            argc, argv, source_parameters, cli::compiler_stage::rad);
        cli::args csg_args(csg_arguments.argc(), csg_arguments.argv.data());
        cli::args bsp_args(bsp_arguments.argc(), bsp_arguments.argv.data());
        cli::args vis_args(vis_arguments.argc(), vis_arguments.argv.data());
        cli::args rad_args(rad_arguments.argc(), rad_arguments.argv.data());
        threads::set_count(csg_args.int_value("-threads", 0));
        std::vector<format::entity::pair> compile_parameters =
            canonical_compile_parameters(argc, argv, source_parameters);

        // one combined settings table for all four stages
        logging::setting("threads", std::to_string(threads::count()).c_str(), "varies",
                         args.has("-threads"));
        csg::csg_options csg_options = parse_csg_options(csg_args, map_path);
        csg_options.compile_parameters_consumed = true;
        if (csg_options.brush.only_entities)
            err::fatal("-onlyents cannot be enabled by info_compile_parameters "
                       "in a unified compile");
        bool optimize_lights = !csg_args.has("-nolightopt");
        bsp::bsp_options bsp_options = parse_bsp_options(bsp_args);
        vis::vis_options vis_options = parse_vis_options(vis_args, base + ".prt");
        rad::rad_options rad_options = parse_rad_options(
            rad_args, rad_arguments.argc(), rad_arguments.argv.data(), base);
        logging::flush_settings();

        auto start = std::chrono::steady_clock::now();

        // ---- csg ----------------------------------------------------------
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
            if (embed_source)
                attach_embedded_source(map, source_bytes,
                                       compile_parameters, {});
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
        std::vector<format::texlight> used_texlights;
        bool lit = rad::run_rad(map, base, rad_options, &alloc_block_pages,
                                &used_texlights);
        std::remove((base + ".wa_").c_str());

        if (embed_source)
        {
            attach_embedded_source(map, source_bytes,
                                   compile_parameters, used_texlights);
            unlit.embedded_zip = map.embedded_zip;
        }

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
