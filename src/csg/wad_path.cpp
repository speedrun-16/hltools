#include "wad_path.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>

#include "../common/binary.h"
#include "../common/error.h"
#include "../common/filesystem.h"
#include "../common/limits.h"
#include "../common/log.h"
#include "../common/string_util.h"
#include "format/bsp/types.h"
#include "format/wad/archive.h"

namespace stdfs = std::filesystem;

namespace csg
{
    namespace
    {
        struct wad_path
        {
            std::string path;
            bool runtime = true;
            int used_textures = 0;
            int total_textures = 0;
        };

        struct loaded_wad
        {
            wad_path path;
            format::wad_archive file;
        };

        struct miptex_entry
        {
            char name[format::max_wad_name] = {};
            int wad_index = 0;
            const format::wad_lump *lump = nullptr;
        };

        class token_reader
        {
        public:
            explicit token_reader(std::string text)
                : text_(std::move(text)) {}

            bool next()
            {
                while (pos_ < text_.size() && is_space(text_[pos_]))
                    pos_++;
                if (pos_ >= text_.size())
                    return false;
                token_.clear();
                if (text_[pos_] == '"')
                {
                    pos_++;
                    while (pos_ < text_.size() && text_[pos_] != '"')
                        token_.push_back(text_[pos_++]);
                    if (pos_ < text_.size())
                        pos_++;
                    return true;
                }
                char c = text_[pos_];
                if (c == '{' || c == '}')
                {
                    token_.assign(1, c);
                    pos_++;
                    return true;
                }
                while (pos_ < text_.size() && !is_space(text_[pos_]) && text_[pos_] != '{' && text_[pos_] != '}')
                    token_.push_back(text_[pos_++]);
                return true;
            }

            const std::string &token() const {
                return token_;
            }

        private:
            static bool is_space(char c)
            {
                return c == ' ' || c == '\t' || c == '\r' || c == '\n';
            }

            std::string text_;
            size_t pos_ = 0;
            std::string token_;
        };

        void cleanup_name(const char *in, char out[format::max_wad_name])
        {
            int i = 0;
            for (; i < format::max_wad_name; i++)
            {
                if (!in[i])
                    break;
                unsigned char c = (unsigned char)in[i];
                out[i] = (char)std::toupper(c);
            }
            for (; i < format::max_wad_name; i++)
                out[i] = 0;
        }

        bool contains_i(const std::string &haystack, const std::string &needle)
        {
            if (needle.empty())
                return true;
            auto it = std::search(haystack.begin(), haystack.end(),
                                  needle.begin(), needle.end(),
                                  [](char a, char b)
                                  {
                                      return std::tolower((unsigned char)a) == std::tolower((unsigned char)b);
                                  });
            return it != haystack.end();
        }

        void add_wad_path(std::vector<wad_path> &paths, std::string path, bool runtime)
        {
            if (path.empty())
                return;
            for (auto &existing : paths)
            {
                if (existing.path == path)
                {
                    existing.runtime = existing.runtime && runtime;
                    return;
                }
            }
            wad_path wad;
            wad.path = std::move(path);
            wad.runtime = runtime;
            paths.push_back(std::move(wad));
        }

        void parse_wad_key(std::vector<wad_path> &paths, const char *value)
        {
            const char *start = value;
            for (const char *p = value;; p++)
            {
                if (*p != ';' && *p != '\0')
                    continue;
                if (p > start)
                    add_wad_path(paths, std::string(start, (size_t)(p - start)), true);
                if (*p == '\0')
                    break;
                start = p + 1;
            }
        }

        void load_wad_cfg_file(std::vector<wad_path> &paths, const std::string &path)
        {
            std::vector<byte> bytes;
            if (!fs::read_all(path, bytes))
                err::fatal("could not open wad configuration file '%s'", path.c_str());
            token_reader reader(std::string(reinterpret_cast<const char *>(bytes.data()), bytes.size()));
            while (reader.next())
            {
                bool runtime = true;
                std::string token = reader.token();
                if (str::iequals(token.c_str(), "include"))
                {
                    runtime = false;
                    if (!reader.next())
                        err::fatal("unexpected end of wad configuration file '%s'", path.c_str());
                    token = reader.token();
                }
                add_wad_path(paths, token, runtime);
            }
        }

        void load_named_wad_config(std::vector<wad_path> &paths,
                                   const std::string &path,
                                   const std::string &config_name)
        {
            std::vector<byte> bytes;
            if (!fs::read_all(path, bytes))
                err::fatal("could not open wad configuration file '%s'", path.c_str());
            token_reader reader(std::string(reinterpret_cast<const char *>(bytes.data()), bytes.size()));
            int matches = 0;
            while (reader.next())
            {
                std::string name = reader.token();
                if (!reader.next() || reader.token() != "{")
                    err::fatal("parsing wad configuration '%s': missing opening brace", path.c_str());

                bool selected = str::iequals(name.c_str(), config_name.c_str());
                if (selected)
                    matches++;
                while (reader.next())
                {
                    if (reader.token() == "}")
                        break;
                    bool runtime = true;
                    std::string token = reader.token();
                    if (str::iequals(token.c_str(), "include"))
                    {
                        runtime = false;
                        if (!reader.next())
                            err::fatal("unexpected end of wad configuration '%s'", path.c_str());
                        token = reader.token();
                    }
                    if (selected)
                        add_wad_path(paths, token, runtime);
                }
            }
            if (matches < 1)
                err::fatal("could not find wad configuration '%s'", config_name.c_str());
            if (matches > 1)
                err::fatal("found more than one wad configuration '%s'", config_name.c_str());
        }

        // the reference open fallback chain: the path as is, the path with a
        // hardcoded drive letter stripped, wadroot/<subdir>/<file>, and a
        // rooted but driveless path tried against every drive letter
        std::string resolve_wad_path(const std::string &path)
        {
            if (fs::exists(path))
                return path;

            if (path.size() > 2 && path[1] == ':')
            {
                std::string stripped = path.substr(2);
                if (fs::exists(stripped))
                    return stripped;
            }

            const char *wad_root = std::getenv("WADROOT");
            if (wad_root && wad_root[0])
            {
                stdfs::path p(path);
                stdfs::path parent = p.parent_path();
                stdfs::path candidate = stdfs::path(wad_root) / parent.filename() / p.filename();
                if (fs::exists(candidate.string()))
                    return candidate.string();
            }

            if (!path.empty() && path[0] == '\\')
            {
                for (char drive = 'C'; drive <= 'Z'; drive++)
                {
                    std::string candidate = std::string(1, drive) + ":" + path;
                    if (fs::exists(candidate))
                        return candidate;
                }
            }
            return path;
        }

        std::vector<wad_path> collect_wad_paths(const map_source &map, const wad_build_options &options)
        {
            std::vector<wad_path> paths;
            const map_entity *world = map.entities.empty() ? nullptr : &map.entities[0];
            std::string cfg_file = options.wad_cfg_file;
            std::string cfg_name = options.wad_config_name;
            if (world)
            {
                if (cfg_file.empty())
                    cfg_file = world->value("wadcfgfile");
                if (cfg_name.empty())
                    cfg_name = world->value("wadconfig");
            }

            if (!cfg_name.empty())
            {
                if (cfg_file.empty())
                    cfg_file = "wad.cfg";
                load_named_wad_config(paths, cfg_file, cfg_name);
            }
            else if (!cfg_file.empty())
            {
                load_wad_cfg_file(paths, cfg_file);
            }
            else if (world)
            {
                parse_wad_key(paths, world->value("wad"));
            }

            for (auto &path : paths)
            {
                if (!options.wad_textures)
                    path.runtime = false;
                for (const auto &include : options.wad_include)
                {
                    if (contains_i(path.path, include))
                        path.runtime = false;
                }
            }
            return paths;
        }

        std::vector<loaded_wad> load_wads(std::vector<wad_path> paths)
        {
            std::vector<loaded_wad> out;
            out.reserve(paths.size());
            for (auto &path : paths)
            {
                path.path = resolve_wad_path(path.path);
                loaded_wad wad;
                wad.path = std::move(path);
                std::string error;
                if (!wad.file.load(wad.path.path, &error))
                    err::fatal("%s: %s", error.c_str(), wad.path.path.c_str());
                wad.path.total_textures = (int)wad.file.lumps().size();
                out.push_back(std::move(wad));
            }
            return out;
        }

        int find_miptex(std::vector<miptex_entry> &entries, const char *name)
        {
            // a full 16 character name has no room for the terminator in the
            // miptex header; the reference errors rather than truncating
            if (std::strlen(name) >= format::max_wad_name)
                err::fatal("texture name is too long (%s)", name);

            char clean[format::max_wad_name];
            cleanup_name(name, clean);
            for (int i = 0; i < (int)entries.size(); i++)
            {
                if (std::strncmp(entries[(size_t)i].name, clean, format::max_wad_name) == 0)
                    return i;
            }
            if (entries.size() >= limits::max_map_textures)
                err::fatal("exceeded max_map_textures");
            miptex_entry entry;
            std::memcpy(entry.name, clean, sizeof(entry.name));
            entries.push_back(entry);
            return (int)entries.size() - 1;
        }

        bool choose_texture(miptex_entry &entry, const std::vector<loaded_wad> &wads)
        {
            const format::wad_lump *best_lump = nullptr;
            int best_wad = 0;
            for (int wad_index = 0; wad_index < (int)wads.size(); wad_index++)
            {
                const auto &lumps = wads[(size_t)wad_index].file.lumps();
                for (int lump_index = 0; lump_index < (int)lumps.size(); lump_index++)
                {
                    const format::wad_lump &lump = lumps[(size_t)lump_index];
                    if (std::strncmp(lump.name, entry.name, format::max_wad_name) != 0)
                        continue;
                    bool better = false;
                    if (!best_lump)
                    {
                        better = true;
                    }
                    else if (wads[(size_t)wad_index].path.runtime != wads[(size_t)best_wad].path.runtime)
                    {
                        better = !wads[(size_t)wad_index].path.runtime;
                    }
                    else if (wad_index != best_wad)
                    {
                        better = wad_index < best_wad;
                    }
                    else
                    {
                        better = lump.filepos < best_lump->filepos;
                    }
                    if (better)
                    {
                        best_lump = &lump;
                        best_wad = wad_index;
                    }
                }
            }

            entry.lump = best_lump;
            entry.wad_index = best_wad;
            return best_lump != nullptr;
        }

        void add_animating_textures(std::vector<miptex_entry> &entries, const std::vector<loaded_wad> &wads)
        {
            size_t base = entries.size();
            for (size_t i = 0; i < base; i++)
            {
                if (entries[i].name[0] != '+' && entries[i].name[0] != '-')
                    continue;
                char name[format::max_wad_name];
                std::memcpy(name, entries[i].name, sizeof(name));
                for (int frame = 0; frame < 20; frame++)
                {
                    name[1] = frame < 10 ? (char)('0' + frame) : (char)('A' + frame - 10);
                    for (const auto &wad : wads)
                    {
                        if (wad.file.find_texture(name))
                        {
                            find_miptex(entries, name);
                            break;
                        }
                    }
                }
            }
        }

        void append_runtime_header(std::vector<byte> &out, const format::wad_lump &lump)
        {
            if (lump.data.size() < sizeof(format::miptex_t))
                err::fatal("wad texture '%s' is too small", lump.name);
            size_t start = out.size();
            out.insert(out.end(), lump.data.begin(), lump.data.begin() + sizeof(format::miptex_t));
            binary::writer output(out);
            for (int i = 0; i < format::mip_levels; i++)
                err::require(output.patch_i32(start + 24 + (size_t)i * 4, 0),
                             "could not patch runtime miptex header");
        }

        void append_embedded_texture(std::vector<byte> &out, const format::wad_lump &lump)
        {
            out.insert(out.end(), lump.data.begin(), lump.data.end());
        }

        std::string runtime_wad_value(const std::vector<loaded_wad> &wads, bool wad_auto_detect)
        {
            std::string out;
            for (const auto &wad : wads)
            {
                if (!wad.path.runtime)
                    continue;
                if (wad.path.used_textures <= 0 && wad_auto_detect)
                    continue;
                out += stdfs::path(wad.path.path).filename().string();
                out += ';';
            }
            return out;
        }
    }

    miptex_build_result build_miptex_lump(const map_source &map,
                                          const texinfo_store &texinfos,
                                          const wad_build_options &options)
    {
        std::vector<loaded_wad> wads = load_wads(collect_wad_paths(map, options));

        std::vector<miptex_entry> entries;
        std::vector<int> texinfo_miptex(texinfos.entries().size(), -1);
        for (int i = 0; i < (int)texinfos.entries().size(); i++)
        {
            int miptex = find_miptex(entries, texinfos.entries()[(size_t)i].texture_name.c_str());
            texinfo_miptex[(size_t)i] = miptex;
        }

        add_animating_textures(entries, wads);

        for (auto &entry : entries)
        {
            if (!choose_texture(entry, wads))
            {
                // the reference fatals in loadlump when a used texture is in
                // none of the wads; a warning here would ship a broken bsp
                err::fatal("texture %s not found in wad files", entry.name);
            }
            wads[(size_t)entry.wad_index].path.used_textures++;
        }

        std::sort(entries.begin(), entries.end(),
                  [](const miptex_entry &a, const miptex_entry &b)
                  {
                      if (a.wad_index != b.wad_index)
                          return a.wad_index < b.wad_index;
                      return std::strncmp(a.name, b.name, format::max_wad_name) < 0;
                  });

        for (int i = 0; i < (int)texinfos.entries().size(); i++)
        {
            char clean[format::max_wad_name];
            cleanup_name(texinfos.entries()[(size_t)i].texture_name.c_str(), clean);
            for (int j = 0; j < (int)entries.size(); j++)
            {
                if (std::strncmp(entries[(size_t)j].name, clean, format::max_wad_name) == 0)
                {
                    texinfo_miptex[(size_t)i] = j;
                    break;
                }
            }
        }

        miptex_build_result result;
        result.wad_value = runtime_wad_value(wads, options.wad_auto_detect);
        binary::writer textures(result.textures);
        textures.i32((int)entries.size());
        for (size_t i = 0; i < entries.size(); i++)
            textures.i32(-1);

        // runtime loaded textures keep only a zeroed header in the bsp; their
        // full lump data goes into the <map>wa_ temp wad for rad the wad3
        // header is 12 bytes ("wad3", lump count, info table offset) followed
        // by the lump data and a 32 byte info record per lump
        std::vector<byte> wad_infos;
        binary::writer wad_info_output(wad_infos);
        int wad_lumps = 0;
        result.temp_wad.resize(12);
        binary::writer temp_wad_output(result.temp_wad);

        for (size_t i = 0; i < entries.size(); i++)
        {
            const miptex_entry &entry = entries[i];
            if (!entry.lump)
                continue;
            if (result.textures.size() >= (size_t)limits::max_map_miptex)
                err::fatal("exceeded max_map_miptex");
            err::require(textures.patch_i32(4 + i * 4, (int)result.textures.size()),
                         "could not patch BSP texture directory");
            if (wads[(size_t)entry.wad_index].path.runtime)
            {
                append_runtime_header(result.textures, *entry.lump);

                wad_info_output.i32((int)result.temp_wad.size());
                wad_info_output.i32(entry.lump->disksize);
                wad_info_output.i32(entry.lump->size);
                wad_infos.push_back((byte)entry.lump->type);
                wad_infos.push_back((byte)entry.lump->compression);
                wad_infos.push_back((byte)entry.lump->pad1);
                wad_infos.push_back((byte)entry.lump->pad2);
                wad_infos.insert(wad_infos.end(), entry.lump->name,
                                 entry.lump->name + format::max_wad_name);
                result.temp_wad.insert(result.temp_wad.end(),
                                       entry.lump->data.begin(), entry.lump->data.end());
                wad_lumps++;
            }
            else
            {
                append_embedded_texture(result.textures, *entry.lump);
            }
        }

        int info_table_offset = (int)result.temp_wad.size();
        result.temp_wad.insert(result.temp_wad.end(), wad_infos.begin(), wad_infos.end());
        result.temp_wad[0] = 'W';
        result.temp_wad[1] = 'A';
        result.temp_wad[2] = 'D';
        result.temp_wad[3] = '3';
        err::require(temp_wad_output.patch_i32(4, wad_lumps)
                     && temp_wad_output.patch_i32(8, info_table_offset),
                     "could not finalize temporary WAD header");

        if (result.textures.size() >= (size_t)limits::max_map_miptex)
            err::fatal("exceeded max_map_miptex");
        result.texinfo_miptex = std::move(texinfo_miptex);
        return result;
    }
}
