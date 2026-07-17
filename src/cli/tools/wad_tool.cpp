#include "wad_tool.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <filesystem>
#include <string>
#include <unordered_set>
#include <vector>

#include "../../common/filesystem.h"
#include "../../common/log.h"
#include "../../common/string_util.h"
#include "format/bmp/file.h"
#include "format/bsp/data.h"
#include "format/bsp/file.h"
#include "format/bsp/texture_lump.h"
#include "format/miptex/texture.h"
#include "format/wad/archive.h"

namespace stdfs = std::filesystem;

namespace tools
{
    namespace
    {
        void print_wad_help()
        {
            logging::console(
                "usage\n"
                "  hltools wad list <archive.wad|map.bsp>\n"
                "  hltools wad extract <archive.wad|map.bsp> <directory|output.wad> [-force] [-all]\n"
                "  hltools wad build <input-directory> <output.wad> [-force]\n"
                "\n"
                "  list       show archive or bsp texture contents\n"
                "  extract    write indexed bmp files, or a wad when the destination ends in .wad\n"
                "  build      create a wad3 archive from indexed 8-bit bmp files\n"
                "\n"
                "options\n"
                "  -force     overwrite existing output\n"
                "  -all       include rad-generated embedded lightmap textures\n");
        }

        bool is_help(const char *s)
        {
            return str::iequals(s, "-h") || str::iequals(s, "-help")
                || str::iequals(s, "--help") || str::iequals(s, "help");
        }

        bool has_flag(int argc, char **argv, const char *flag)
        {
            for (int i = 1; i < argc; i++)
                if (str::iequals(argv[i], flag))
                    return true;
            return false;
        }

        std::vector<std::string> positional(int argc, char **argv)
        {
            std::vector<std::string> out;
            for (int i = 1; i < argc; i++)
            {
                if (argv[i][0] == '-')
                    continue;
                out.emplace_back(argv[i]);
            }
            return out;
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

        std::string uppercase(std::string value)
        {
            for (char &c : value)
                c = (char)std::toupper((unsigned char)c);
            return value;
        }

        std::string bmp_filename(const std::string &name)
        {
            std::string safe = name;
            for (char &c : safe)
            {
                unsigned char u = (unsigned char)c;
                if (u < 32 || c == '<' || c == '>' || c == ':' || c == '"'
                    || c == '/' || c == '\\' || c == '|' || c == '?' || c == '*')
                    c = '_';
            }
            return safe + ".bmp";
        }

        bool load_source(const std::string &path, std::vector<format::mip_texture> &textures,
                         std::vector<std::string> &external, std::string &error)
        {
            textures.clear();
            external.clear();
            if (extension_is(path, ".bsp"))
            {
                format::map_data map;
                if (!format::bsp_file::load(path, map))
                {
                    error = "could not load BSP file";
                    return false;
                }
                return format::collect_bsp_textures(map, textures, &external, &error);
            }

            format::wad_archive wad;
            if (!wad.load(path, &error))
                return false;
            for (const format::wad_lump &lump : wad.lumps())
            {
                if ((unsigned char)lump.type != (unsigned char)format::wad_miptex_type)
                    continue;
                format::mip_texture texture;
                std::string detail;
                if (!format::mip_texture_from_lump(lump, texture, &detail))
                {
                    error = std::string("texture '") + lump.name + "': " + detail;
                    return false;
                }
                textures.push_back(std::move(texture));
            }
            return true;
        }

        void print_external_summary(const std::vector<std::string> &external)
        {
            if (external.empty())
                return;
            logging::console("\n  %zu external texture reference%s (pixel data not embedded)\n",
                             external.size(), external.size() == 1 ? "" : "s");
        }

        int run_list(const std::string &source)
        {
            std::vector<format::mip_texture> textures;
            std::vector<std::string> external;
            std::string error;
            if (!load_source(source, textures, external, error))
            {
                logging::console("wad list: %s: %s\n", source.c_str(), error.c_str());
                return 1;
            }
            logging::console("\ntextures: %s\n\n", source.c_str());
            logging::console("  %-16s %9s %10s\n", "name", "dimensions", "size");
            logging::console("  ---------------------------------------\n");
            char size[32];
            for (const format::mip_texture &texture : textures)
            {
                char dimensions[32];
                std::snprintf(dimensions, sizeof(dimensions), "%ux%u", texture.width, texture.height);
                logging::console("  %-16s %9s %10s%s\n", texture.name.c_str(), dimensions,
                                 str::human_bytes((long long)texture.data.size(), size, sizeof(size)),
                                 is_internal_rad_texture(texture.name) ? "  (rad internal)" : "");
            }
            for (const std::string &name : external)
                logging::console("  %-16s %9s %10s\n", name.c_str(), "external", "-");
            logging::console("\n  %zu embedded texture%s\n", textures.size(),
                             textures.size() == 1 ? "" : "s");
            print_external_summary(external);
            return 0;
        }

        int run_extract(const std::string &source, const std::string &destination,
                        bool force, bool include_all)
        {
            std::vector<format::mip_texture> textures;
            std::vector<std::string> external;
            std::string error;
            if (!load_source(source, textures, external, error))
            {
                logging::console("wad extract: %s: %s\n", source.c_str(), error.c_str());
                return 1;
            }
            if (!include_all)
            {
                textures.erase(std::remove_if(textures.begin(), textures.end(),
                                               [](const format::mip_texture &texture)
                                               { return is_internal_rad_texture(texture.name); }),
                               textures.end());
            }
            if (textures.empty())
            {
                logging::console("wad extract: no embedded textures to extract\n");
                print_external_summary(external);
                return 1;
            }

            if (extension_is(destination, ".wad"))
            {
                if (fs::exists(destination) && !force)
                {
                    logging::console("wad extract: '%s' already exists (use -force to overwrite)\n",
                                     destination.c_str());
                    return 1;
                }
                if (!format::write_wad3(destination, textures, &error))
                {
                    logging::console("wad extract: %s\n", error.c_str());
                    return 1;
                }
                logging::console("wrote %s (%zu texture%s)\n", destination.c_str(), textures.size(),
                                 textures.size() == 1 ? "" : "s");
                print_external_summary(external);
                return 0;
            }

            if (fs::exists(destination) && !stdfs::is_directory(stdfs::path(destination)))
            {
                logging::console("wad extract: destination is not a directory: %s\n",
                                 destination.c_str());
                return 1;
            }
            if (!fs::make_directory(destination))
            {
                logging::console("wad extract: could not create directory: %s\n",
                                 destination.c_str());
                return 1;
            }

            std::vector<std::string> paths;
            std::unordered_set<std::string> unique;
            for (const format::mip_texture &texture : textures)
            {
                std::string path = (stdfs::path(destination) / bmp_filename(texture.name)).string();
                std::string key = uppercase(path);
                if (!unique.insert(key).second)
                {
                    logging::console("wad extract: texture names collide as files: %s\n",
                                     texture.name.c_str());
                    return 1;
                }
                if (fs::exists(path) && !force)
                {
                    logging::console("wad extract: '%s' already exists (use -force to overwrite)\n",
                                     path.c_str());
                    return 1;
                }
                paths.push_back(std::move(path));
            }
            for (size_t i = 0; i < textures.size(); i++)
            {
                if (!format::write_indexed_bmp(paths[i], textures[i], &error))
                {
                    logging::console("wad extract: %s: %s\n", textures[i].name.c_str(), error.c_str());
                    return 1;
                }
            }
            logging::console("wrote %zu indexed bmp texture%s to %s\n", textures.size(),
                             textures.size() == 1 ? "" : "s", destination.c_str());
            print_external_summary(external);
            return 0;
        }

        int run_build(const std::string &directory, const std::string &destination, bool force)
        {
            std::error_code ec;
            if (!stdfs::is_directory(stdfs::path(directory), ec))
            {
                logging::console("wad build: input is not a directory: %s\n", directory.c_str());
                return 1;
            }
            if (!extension_is(destination, ".wad"))
            {
                logging::console("wad build: output must end in .wad\n");
                return 1;
            }
            if (fs::exists(destination) && !force)
            {
                logging::console("wad build: '%s' already exists (use -force to overwrite)\n",
                                 destination.c_str());
                return 1;
            }

            std::vector<stdfs::path> files;
            for (const auto &entry : stdfs::directory_iterator(stdfs::path(directory), ec))
            {
                if (ec)
                    break;
                if (entry.is_regular_file() && extension_is(entry.path().string(), ".bmp"))
                    files.push_back(entry.path());
            }
            if (ec)
            {
                logging::console("wad build: could not read directory: %s\n", directory.c_str());
                return 1;
            }
            std::sort(files.begin(), files.end(),
                      [](const stdfs::path &a, const stdfs::path &b)
                      { return uppercase(a.filename().string()) < uppercase(b.filename().string()); });
            if (files.empty())
            {
                logging::console("wad build: no .bmp files found in %s\n", directory.c_str());
                return 1;
            }

            std::vector<format::mip_texture> textures;
            std::unordered_set<std::string> names;
            std::string error;
            for (const stdfs::path &file : files)
            {
                std::string name = file.stem().string();
                if (!names.insert(uppercase(name)).second)
                {
                    logging::console("wad build: duplicate texture name: %s\n", name.c_str());
                    return 1;
                }
                format::indexed_image image;
                if (!format::load_indexed_bmp(file.string(), image, &error))
                {
                    logging::console("wad build: %s: %s\n", file.string().c_str(), error.c_str());
                    return 1;
                }
                format::mip_texture texture;
                if (!format::build_mip_texture(name, image, texture, &error))
                {
                    logging::console("wad build: %s: %s\n", file.string().c_str(), error.c_str());
                    return 1;
                }
                textures.push_back(std::move(texture));
            }
            if (!format::write_wad3(destination, textures, &error))
            {
                logging::console("wad build: %s\n", error.c_str());
                return 1;
            }
            logging::console("wrote %s (%zu texture%s)\n", destination.c_str(), textures.size(),
                             textures.size() == 1 ? "" : "s");
            return 0;
        }
    }

    int run_wad_tool(int argc, char **argv)
    {
        if (argc < 2 || is_help(argv[1]))
        {
            print_wad_help();
            return argc >= 2 ? 0 : 1;
        }
        std::vector<std::string> args = positional(argc, argv);
        if (args.empty())
        {
            print_wad_help();
            return 1;
        }
        if (str::iequals(args[0].c_str(), "list") && args.size() == 2)
            return run_list(args[1]);
        if (str::iequals(args[0].c_str(), "extract") && args.size() == 3)
            return run_extract(args[1], args[2], has_flag(argc, argv, "-force"),
                               has_flag(argc, argv, "-all"));
        if (str::iequals(args[0].c_str(), "build") && args.size() == 3)
            return run_build(args[1], args[2], has_flag(argc, argv, "-force"));

        print_wad_help();
        return 1;
    }
}
