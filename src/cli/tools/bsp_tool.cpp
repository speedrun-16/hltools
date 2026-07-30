#include "bsp_tool.h"

#include <cctype>
#include <chrono>
#include <filesystem>
#include <string>
#include <vector>

#include "../../common/error.h"
#include "../../common/filesystem.h"
#include "../../common/log.h"
#include "../../common/string_util.h"
#include "../../common/threads.h"
#include "bsp/pack.h"
#include "format/bsp/file.h"
#include "format/bsp/usage_chart.h"
#include "bsp_info_tool.h"
#include "compile_parameters.h"

namespace stdfs = std::filesystem;

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
                "  hltools bsp [options] <map>\n"
                "  hltools bsp info <map[.bsp]>\n"
                "  hltools bsp pack <map.bsp> <output-dir> [options]\n"
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
                "  -allleaks             report every hole, not just the first, and\n"
                "                        put all their paths in one pointfile\n"
                "\n"
                "misc\n"
                "  -nonulltex            don't strip null faces\n"
                "  -threads <n>          worker threads             (default: all cores)\n");
        }

        void print_pack_help()
        {
            logging::console(
                "usage\n"
                "  hltools bsp pack <map.bsp> <output-dir> [options]\n"
                "\n"
                "  copies the BSP and its external resources below <output-dir>,\n"
                "  preserving game-relative paths such as maps/, models/, sound/,\n"
                "  sprites/, and gfx/env/. Also writes maps/<map>.res.\n"
                "\n"
                "options\n"
                "  -game <dir>  source game directory; inferred when the BSP is\n"
                "               inside a maps directory\n"
                "  -base <dir>  installed base content to exclude; repeatable\n"
                "  -force       overwrite files already present in the output\n"
                "  -strict      fail when any referenced resource cannot be found\n");
        }

        bool is_help(const char *value)
        {
            return str::iequals(value, "-h") || str::iequals(value, "-help")
                || str::iequals(value, "--help") || str::iequals(value, "help");
        }

        std::string infer_game_dir(const std::string &bsp_path)
        {
            stdfs::path parent = stdfs::absolute(bsp_path).parent_path();
            if (str::iequals(parent.filename().string().c_str(), "maps"))
                return parent.parent_path().string();
            return {};
        }

        void add_inferred_base_dirs(bsp::pack_options &options)
        {
            stdfs::path game = stdfs::absolute(options.game_dir);
            std::string name = game.filename().string();
            std::string lowered = name;
            for (char &c : lowered)
                c = (char)std::tolower((unsigned char)c);
            const std::string suffix = "_downloads";
            std::vector<stdfs::path> candidates;
            if (lowered.size() > suffix.size()
                && lowered.substr(lowered.size() - suffix.size()) == suffix)
            {
                name.resize(name.size() - suffix.size());
                candidates.push_back(game.parent_path() / name);
            }
            candidates.push_back(game.parent_path() / "valve");

            for (const stdfs::path &base : candidates)
            {
                std::error_code ec;
                if (!stdfs::is_directory(base, ec))
                    continue;
                bool duplicate = false;
                for (const std::string &existing : options.base_dirs)
                    if (str::iequals(stdfs::absolute(existing).string().c_str(),
                                     stdfs::absolute(base).string().c_str()))
                    {
                        duplicate = true;
                        break;
                    }
                if (!duplicate
                    && !str::iequals(game.string().c_str(),
                                     stdfs::absolute(base).string().c_str()))
                    options.base_dirs.push_back(base.string());
            }
        }

        int run_pack(int argc, char **argv)
        {
            if (argc >= 3 && is_help(argv[2]))
            {
                print_pack_help();
                return 0;
            }

            bsp::pack_options options;
            std::vector<std::string> positional;
            for (int i = 2; i < argc; i++)
            {
                if (str::iequals(argv[i], "-force"))
                {
                    options.force = true;
                    continue;
                }
                if (str::iequals(argv[i], "-strict"))
                {
                    options.strict = true;
                    continue;
                }
                if (str::iequals(argv[i], "-game"))
                {
                    if (++i >= argc)
                    {
                        logging::console(
                            "bsp pack: -game requires a source directory\n");
                        return 1;
                    }
                    options.game_dir = argv[i];
                    continue;
                }
                if (str::iequals(argv[i], "-base"))
                {
                    if (++i >= argc)
                    {
                        logging::console(
                            "bsp pack: -base requires a content directory\n");
                        return 1;
                    }
                    options.base_dirs.emplace_back(argv[i]);
                    continue;
                }
                if (argv[i][0] == '-')
                {
                    logging::console("bsp pack: unknown option '%s'\n", argv[i]);
                    return 1;
                }
                positional.emplace_back(argv[i]);
            }
            if (positional.size() != 2)
            {
                print_pack_help();
                return 1;
            }

            std::string source = fs::with_extension(positional[0], ".bsp");
            if (options.game_dir.empty())
                options.game_dir = infer_game_dir(source);
            if (options.game_dir.empty())
            {
                logging::console(
                    "bsp pack: could not infer the game directory; use -game\n");
                return 1;
            }
            add_inferred_base_dirs(options);

            bsp::pack_result result;
            std::string error;
            bool ok = bsp::pack_map(
                source, positional[1], options, result, &error);
            for (const std::string &missing : result.missing)
                logging::console("warning: missing %s\n", missing.c_str());
            if (!ok)
            {
                logging::console("bsp pack: %s\n", error.c_str());
                return 1;
            }

            logging::console(
                "packed %zu resource(s) to %s (%zu copied, %zu already in place)\n"
                "wrote %s\n",
                result.resources.size(), positional[1].c_str(), result.copied,
                result.unchanged, result.res_path.c_str());
            if (!result.provided_by_base.empty())
                logging::console(
                    "%zu referenced resource(s) are provided by base content\n",
                    result.provided_by_base.size());
            if (!result.missing.empty())
                logging::console(
                    "%zu referenced resource(s) were not found; use -strict to "
                    "treat this as an error\n",
                    result.missing.size());
            return 0;
        }
    }

    int run_bsp_tool(int argc, char **argv)
    {
        if (argc >= 2 && str::iequals(argv[1], "info"))
            return run_bsp_info_tool(argc - 1, argv + 1);
        if (argc >= 2 && str::iequals(argv[1], "pack"))
            return run_pack(argc, argv);

        cli::args command_line(argc, argv);
        const bool want_help = command_line.has("-h")
            || command_line.has("-help") || command_line.has("--help");
        if (command_line.empty() || command_line.map_name().empty() || want_help)
        {
            print_bsp_help();
            return want_help ? 0 : 1;
        }

        std::string base = fs::strip_extension(
            fs::with_extension(command_line.map_name(), ".bsp"));
        logging::open_stage_log(base, "bsp");
        logging::banner("bsp");

        std::string bsp_path = base + ".bsp";
        format::map_data map;
        if (!format::bsp_file::load(bsp_path, map))
            err::fatal("could not load bsp '%s'", bsp_path.c_str());

        const tools::compile_parameters parameters =
            compile_parameters_from_bsp(map);
        staged_arguments arguments(
            argc, argv, parameters, cli::compiler_stage::bsp);
        cli::args args(arguments.argc(), arguments.argv.data());

        threads::set_count(args.int_value("-threads", 0));
        bsp::bsp_options options = parse_bsp_options(args);
        logging::setting("threads", std::to_string(threads::count()).c_str(), "varies",
                         args.has("-threads"));
        logging::flush_settings();

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
