#include "model_tool.h"

#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#include "common/filesystem.h"
#include "common/log.h"
#include "common/string_util.h"
#include "decompile/source/models.h"
#include "model/skybox_model.h"

namespace stdfs = std::filesystem;

namespace tools
{
    namespace
    {
        void print_help()
        {
            logging::console(
                "usage\n"
                "  hltools model convert <source.mdl> <output.mdl> [options]\n"
                "  hltools model skybox <face-prefix> <output-prefix> [options]\n"
                "\n"
                "convert\n"
                "  converts a Source studio model and its sibling .vvd/.vtx files\n"
                "  into one self-contained GoldSrc v10 MDL. VMT/VTF skins are\n"
                "  resolved from loose game content and VPK archives.\n"
                "\n"
                "  -game <dir>    Source game content directory; inferred from a\n"
                "                 source path below a models directory when omitted\n"
                "  -force         overwrite an existing output model\n"
                "\n"
                "skybox\n"
                "  builds one or more GoldSrc studio models from the six TGA files\n"
                "  <face-prefix>{up,lf,ft,rt,bk,dn}.tga. Large faces are divided into\n"
                "  model-safe tiles without reducing their resolution. Output files are\n"
                "  named <output-prefix>0.mdl, <output-prefix>1.mdl, and so on.\n"
                "\n"
                "  -size <units>  cube edge length, 1024..262144 (default 131072)\n"
                "  -tile <pixels> model skin edge, power of two 16..512 (default 512)\n"
                "  -force         overwrite existing output models\n"
                "\n"
                "planned\n"
                "  GoldSrc QC/SMD compilation and decompilation are not yet "
                "implemented.\n");
        }

        bool is_help(const char *value)
        {
            return str::iequals(value, "-h") || str::iequals(value, "-help")
                || str::iequals(value, "--help") || str::iequals(value, "help");
        }

        std::string strip_face_suffix(std::string prefix)
        {
            stdfs::path path(prefix);
            if (str::iequals(path.extension().string().c_str(), ".tga"))
            {
                prefix = (path.parent_path() / path.stem()).string();
                if (prefix.size() >= 2)
                {
                    std::string suffix = prefix.substr(prefix.size() - 2);
                    static const char *const suffixes[6] =
                        {"up", "lf", "ft", "rt", "bk", "dn"};
                    for (const char *candidate : suffixes)
                        if (str::iequals(suffix.c_str(), candidate))
                        {
                            prefix.resize(prefix.size() - 2);
                            break;
                        }
                }
            }
            return prefix;
        }

        std::string infer_game_dir(const std::string &source)
        {
            stdfs::path path = stdfs::absolute(source);
            stdfs::path current;
            for (const stdfs::path &part : path)
            {
                if (str::iequals(part.string().c_str(), "models"))
                    return current.string();
                current /= part;
            }
            return {};
        }

        int run_convert(int argc, char **argv)
        {
            bool force = false;
            std::vector<std::string> game_dirs;
            std::vector<std::string> positional;
            for (int i = 2; i < argc; i++)
            {
                if (str::iequals(argv[i], "-force"))
                {
                    force = true;
                    continue;
                }
                if (str::iequals(argv[i], "-game"))
                {
                    if (++i >= argc)
                    {
                        logging::console(
                            "model convert: -game requires a content directory\n");
                        return 1;
                    }
                    game_dirs.emplace_back(argv[i]);
                    continue;
                }
                if (argv[i][0] == '-')
                {
                    logging::console("model convert: unknown option '%s'\n", argv[i]);
                    return 1;
                }
                positional.emplace_back(argv[i]);
            }
            if (positional.size() != 2)
            {
                logging::console(
                    "model convert: expected a Source MDL and an output MDL\n");
                return 1;
            }

            const std::string &source = positional[0];
            const std::string &output = positional[1];
            if (!str::iequals(stdfs::path(source).extension().string().c_str(), ".mdl")
                || !str::iequals(stdfs::path(output).extension().string().c_str(), ".mdl"))
            {
                logging::console(
                    "model convert: input and output paths must end in .mdl\n");
                return 1;
            }
            std::error_code equivalent_error;
            if (stdfs::equivalent(source, output, equivalent_error)
                && !equivalent_error)
            {
                logging::console(
                    "model convert: input and output must be different files\n");
                return 1;
            }
            if (fs::exists(output) && !force)
            {
                logging::console(
                    "model convert: '%s' already exists (use -force to overwrite)\n",
                    output.c_str());
                return 1;
            }
            if (game_dirs.empty())
                game_dirs.emplace_back(infer_game_dir(source));

            decompile::source_model_conversion conversion;
            std::string error;
            if (!decompile::convert_source_model(source, game_dirs, conversion, &error))
            {
                logging::console("model convert: %s\n", error.c_str());
                return 1;
            }

            std::string output_dir = fs::directory(output);
            if (!output_dir.empty() && !fs::make_directory(output_dir))
            {
                logging::console("model convert: could not create '%s'\n",
                                 output_dir.c_str());
                return 1;
            }
            if (!fs::write_all(output, conversion.data.data(), conversion.data.size()))
            {
                logging::console("model convert: could not write '%s'\n", output.c_str());
                return 1;
            }

            logging::console(
                "wrote %s (%zu vertices, %zu triangles, %zu skins)\n",
                output.c_str(), conversion.vertices, conversion.triangles,
                conversion.textures);
            if (conversion.missing_skins)
                logging::console(
                    "warning: %zu skin(s) could not be resolved and use a purple "
                    "placeholder; check -game\n",
                    conversion.missing_skins);
            return 0;
        }

        int run_skybox(int argc, char **argv)
        {
            model::skybox_model_options options;
            bool force = false;
            std::vector<std::string> positional;
            for (int i = 2; i < argc; i++)
            {
                if (str::iequals(argv[i], "-force"))
                {
                    force = true;
                    continue;
                }
                if (str::iequals(argv[i], "-size"))
                {
                    if (++i >= argc)
                    {
                        logging::console("model skybox: -size requires a world size\n");
                        return 1;
                    }
                    char *end = nullptr;
                    double value = std::strtod(argv[i], &end);
                    if (!end || *end != '\0' || !std::isfinite(value)
                        || value < 1024 || value > 262144)
                    {
                        logging::console("model skybox: -size expects 1024..262144\n");
                        return 1;
                    }
                    options.world_size = (float)value;
                    continue;
                }
                if (str::iequals(argv[i], "-tile"))
                {
                    if (++i >= argc)
                    {
                        logging::console(
                            "model skybox: -tile requires a pixel size\n");
                        return 1;
                    }
                    char *end = nullptr;
                    long value = std::strtol(argv[i], &end, 10);
                    if (!end || *end != '\0' || value < 16 || value > 512
                        || (value & (value - 1)) != 0)
                    {
                        logging::console(
                            "model skybox: -tile expects a power of two from "
                            "16..512\n");
                        return 1;
                    }
                    options.tile_size = (unsigned)value;
                    continue;
                }
                if (argv[i][0] == '-')
                {
                    logging::console("model skybox: unknown option '%s'\n", argv[i]);
                    return 1;
                }
                positional.emplace_back(argv[i]);
            }
            if (positional.size() != 2)
            {
                logging::console(
                    "model skybox: expected a face prefix and an output prefix\n");
                return 1;
            }

            std::string input_prefix = strip_face_suffix(positional[0]);
            std::string output_prefix = fs::strip_extension(positional[1]);
            options.model_name = stdfs::path(output_prefix).filename().string();

            const char *const suffixes[6] = {"up", "lf", "ft", "rt", "bk", "dn"};
            std::array<model::rgb_image, 6> faces;
            std::string error;
            for (std::size_t i = 0; i < faces.size(); i++)
            {
                std::string path = input_prefix + suffixes[i] + ".tga";
                if (!model::load_tga(path, faces[i], &error))
                {
                    logging::console("model skybox: %s\n", error.c_str());
                    return 1;
                }
            }

            std::vector<model::skybox_model_part> parts;
            if (!model::build_skybox_models(faces, options, parts, &error))
            {
                logging::console("model skybox: %s\n", error.c_str());
                return 1;
            }
            for (std::size_t i = 0; i < parts.size(); i++)
            {
                std::string path = output_prefix + std::to_string(i) + ".mdl";
                if (fs::exists(path) && !force)
                {
                    logging::console(
                        "model skybox: '%s' already exists (use -force to overwrite)\n",
                        path.c_str());
                    return 1;
                }
            }

            std::string output_dir = fs::directory(output_prefix);
            if (!output_dir.empty() && !fs::make_directory(output_dir))
            {
                logging::console("model skybox: could not create '%s'\n",
                                 output_dir.c_str());
                return 1;
            }
            std::size_t textures = 0, triangles = 0;
            for (std::size_t i = 0; i < parts.size(); i++)
            {
                std::string path = output_prefix + std::to_string(i) + ".mdl";
                if (!fs::write_all(path, parts[i].data.data(), parts[i].data.size()))
                {
                    logging::console(
                        "model skybox: could not write '%s'\n", path.c_str());
                    return 1;
                }
                textures += parts[i].textures;
                triangles += parts[i].triangles;
                logging::console("wrote %s (%zu skins, %zu triangles)\n",
                                 path.c_str(), parts[i].textures, parts[i].triangles);
            }

            logging::console(
                "skybox model: %ux%u faces -> %zu model(s), %zu skins, "
                "%zu triangles\n"
                "place one cycler_sprite for each model at the same origin\n",
                faces[0].width, faces[0].height, parts.size(), textures, triangles);
            return 0;
        }
    }

    int run_model_tool(int argc, char **argv)
    {
        if (argc < 2 || is_help(argv[1]))
        {
            print_help();
            return argc < 2 ? 1 : 0;
        }
        if (str::iequals(argv[1], "convert"))
            return run_convert(argc, argv);
        if (str::iequals(argv[1], "skybox"))
            return run_skybox(argc, argv);
        logging::console("model: expected the 'convert' or 'skybox' operation\n");
        return 1;
    }
}
