#include "pack.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <map>
#include <system_error>
#include <utility>

#include "common/binary.h"
#include "common/filesystem.h"
#include "format/bsp/data.h"
#include "format/bsp/entity_lump.h"
#include "format/bsp/file.h"
#include "format/miptex/types.h"

namespace stdfs = std::filesystem;

namespace bsp
{
    namespace
    {
        struct resource
        {
            std::string relative;
            std::string explicit_source;
        };

        std::string lower(std::string value)
        {
            for (char &c : value)
                c = (char)std::tolower((unsigned char)c);
            return value;
        }

        std::string trim(std::string value)
        {
            std::size_t first = 0;
            while (first < value.size()
                   && std::isspace((unsigned char)value[first]))
                first++;
            std::size_t last = value.size();
            while (last > first && std::isspace((unsigned char)value[last - 1]))
                last--;
            value = value.substr(first, last - first);
            if (value.size() >= 2 && value.front() == '"' && value.back() == '"')
                value = value.substr(1, value.size() - 2);
            return value;
        }

        std::string normalize_relative(std::string value)
        {
            value = trim(std::move(value));
            std::replace(value.begin(), value.end(), '\\', '/');
            while (value.rfind("./", 0) == 0)
                value.erase(0, 2);
            while (!value.empty() && value.front() == '/')
                value.erase(value.begin());
            if (value.empty() || value.find(':') != std::string::npos)
                return {};

            std::string clean;
            std::size_t at = 0;
            while (at <= value.size())
            {
                std::size_t end = value.find('/', at);
                if (end == std::string::npos)
                    end = value.size();
                std::string part = value.substr(at, end - at);
                if (part == "..")
                    return {};
                if (!part.empty() && part != ".")
                {
                    if (!clean.empty())
                        clean += '/';
                    clean += part;
                }
                if (end == value.size())
                    break;
                at = end + 1;
            }
            return clean;
        }

        bool starts_with_path(const std::string &path, const char *prefix)
        {
            std::string a = lower(path);
            std::string b = lower(prefix);
            return a.rfind(b, 0) == 0;
        }

        std::string extension(const std::string &path)
        {
            return lower(stdfs::path(path).extension().string());
        }

        class resource_catalog
        {
        public:
            void add(const std::string &relative,
                     const std::string &explicit_source = {})
            {
                std::string clean = normalize_relative(relative);
                if (clean.empty())
                    return;
                std::string key = lower(clean);
                auto found = entries_.find(key);
                if (found == entries_.end())
                    entries_.emplace(std::move(key),
                                     resource{std::move(clean), explicit_source});
                else if (found->second.explicit_source.empty()
                         && !explicit_source.empty())
                    found->second.explicit_source = explicit_source;
            }

            const std::map<std::string, resource> &entries() const
            {
                return entries_;
            }

        private:
            std::map<std::string, resource> entries_;
        };

        bool has_external_textures(const std::vector<byte> &lump)
        {
            if (lump.size() < 4)
                return false;
            binary::reader reader(lump);
            std::int32_t count = 0;
            if (!reader.i32(count) || count <= 0
                || lump.size() < 4 + (std::size_t)count * 4)
                return false;
            for (int i = 0; i < count; i++)
            {
                std::int32_t offset = 0;
                if (!reader.i32(offset) || offset < 0
                    || (std::size_t)offset + sizeof(format::miptex_t) > lump.size())
                    continue;
                format::miptex_t texture{};
                std::memcpy(&texture, lump.data() + offset, sizeof(texture));
                if (texture.offsets[0] == 0)
                    return true;
            }
            return false;
        }

        void add_file_value(resource_catalog &resources, std::string value)
        {
            value = normalize_relative(std::move(value));
            if (value.empty() || value.front() == '*' || value.front() == '!')
                return;

            std::string ext = extension(value);
            if (ext == ".mdl" && !starts_with_path(value, "models/"))
                value = "models/" + value;
            else if (ext == ".spr" && !starts_with_path(value, "sprites/"))
                value = "sprites/" + value;
            else if ((ext == ".wav" || ext == ".mp3")
                     && !starts_with_path(value, "sound/"))
                value = "sound/" + value;
            else if (ext != ".mdl" && ext != ".spr" && ext != ".wav"
                     && ext != ".mp3" && ext != ".tga" && ext != ".bmp")
                return;
            resources.add(value);
        }

        void add_wads(resource_catalog &resources, const std::string &value)
        {
            std::size_t start = 0;
            while (start <= value.size())
            {
                std::size_t end = value.find(';', start);
                if (end == std::string::npos)
                    end = value.size();
                std::string source = trim(value.substr(start, end - start));
                if (!source.empty())
                {
                    std::string name = stdfs::path(source).filename().string();
                    if (!name.empty())
                    {
                        std::error_code ec;
                        if (stdfs::is_regular_file(stdfs::path(source), ec))
                            resources.add(name, source);
                        else
                            resources.add(name);
                    }
                }
                if (end == value.size())
                    break;
                start = end + 1;
            }
        }

        void collect_entities(const format::map_data &map,
                              resource_catalog &resources)
        {
            bool external_wads = has_external_textures(map.textures);
            for (const format::entity &entity : format::parse_entities(map.entities))
            {
                for (const format::entity::pair &pair : entity.pairs())
                {
                    std::string key = lower(pair.first);
                    if (key == "wad")
                    {
                        if (external_wads)
                            add_wads(resources, pair.second);
                        continue;
                    }
                    if (key == "skyname")
                    {
                        std::string sky = normalize_relative(pair.second);
                        if (sky.empty())
                            continue;
                        if (starts_with_path(sky, "gfx/env/"))
                            sky.erase(0, 8);
                        static const char *const suffixes[6] =
                            {"bk", "dn", "ft", "lf", "rt", "up"};
                        for (const char *suffix : suffixes)
                            resources.add("gfx/env/" + sky + suffix + ".tga");
                        continue;
                    }
                    add_file_value(resources, pair.second);
                }
            }
        }

        void collect_res_file(const std::string &path,
                              const std::string &generated_relative,
                              resource_catalog &resources)
        {
            std::vector<byte> data;
            if (!fs::read_all(path, data))
                return;
            std::string text(data.begin(), data.end());
            std::size_t start = 0;
            while (start <= text.size())
            {
                std::size_t end = text.find('\n', start);
                if (end == std::string::npos)
                    end = text.size();
                std::string line = text.substr(start, end - start);
                std::size_t comment = line.find("//");
                if (comment != std::string::npos)
                    line.resize(comment);
                line = normalize_relative(std::move(line));
                if (!line.empty() && line.find('{') == std::string::npos
                    && line.find('}') == std::string::npos
                    && lower(line) != lower(generated_relative)
                    && extension(line) != ".res")
                    resources.add(line);
                if (end == text.size())
                    break;
                start = end + 1;
            }
        }

        bool path_below(const std::string &path, const std::string &directory)
        {
            std::error_code ec;
            std::string child =
                lower(stdfs::weakly_canonical(stdfs::path(path), ec).string());
            if (ec)
                child = lower(stdfs::absolute(stdfs::path(path), ec).string());
            ec.clear();
            std::string parent =
                lower(stdfs::weakly_canonical(stdfs::path(directory), ec).string());
            if (ec)
                parent = lower(stdfs::absolute(stdfs::path(directory), ec).string());
            if (child.size() <= parent.size()
                || child.compare(0, parent.size(), parent) != 0)
                return false;
            char separator = child[parent.size()];
            return separator == '/' || separator == '\\';
        }

        enum class source_location
        {
            missing,
            package,
            base,
        };

        source_location resolve_source(const pack_options &options,
                                       const resource &item,
                                       std::string &source)
        {
            if (!item.explicit_source.empty() && fs::exists(item.explicit_source))
            {
                source = item.explicit_source;
                for (const std::string &base : options.base_dirs)
                    if (path_below(source, base))
                        return source_location::base;
                return source_location::package;
            }
            source = (stdfs::path(options.game_dir)
                      / stdfs::path(item.relative)).string();
            if (fs::exists(source))
                return source_location::package;
            for (const std::string &base : options.base_dirs)
            {
                source = (stdfs::path(base)
                          / stdfs::path(item.relative)).string();
                if (fs::exists(source))
                    return source_location::base;
            }
            return source_location::missing;
        }

        void add_optional(resource_catalog &resources, const pack_options &options,
                          const std::string &relative)
        {
            resource item{relative, {}};
            std::string source;
            if (resolve_source(options, item, source) == source_location::package)
                resources.add(relative);
        }

        void add_model_companions(resource_catalog &resources,
                                  const pack_options &options)
        {
            std::vector<std::string> models;
            for (const auto &entry : resources.entries())
                if (extension(entry.second.relative) == ".mdl")
                    models.push_back(entry.second.relative);

            for (const std::string &path : models)
            {
                std::string base = fs::strip_extension(path);
                add_optional(resources, options, base + "T.mdl");
                for (int group = 1; group <= 99; group++)
                {
                    char suffix[8];
                    std::snprintf(suffix, sizeof(suffix), "%02d.mdl", group);
                    add_optional(resources, options, base + suffix);
                }
            }
        }

        bool same_file(const std::string &a, const std::string &b)
        {
            std::error_code ec;
            bool same = stdfs::equivalent(stdfs::path(a), stdfs::path(b), ec);
            return !ec && same;
        }

        void set_error(std::string *error, const std::string &message)
        {
            if (error)
                *error = message;
        }
    }

    bool pack_map(const std::string &bsp_path, const std::string &output_root,
                  const pack_options &options, pack_result &out,
                  std::string *error)
    {
        out = pack_result{};
        if (options.game_dir.empty())
        {
            set_error(error, "a game directory is required");
            return false;
        }

        format::map_data map;
        if (!format::bsp_file::load(bsp_path, map))
        {
            set_error(error, "could not load BSP '" + bsp_path + "'");
            return false;
        }

        std::string map_name = stdfs::path(bsp_path).filename().string();
        std::string stem = stdfs::path(map_name).stem().string();
        resource_catalog resources;
        resources.add("maps/" + map_name, bsp_path);
        collect_entities(map, resources);
        std::string generated_relative = "maps/" + stem + ".res";
        collect_res_file(
            (stdfs::path(bsp_path).parent_path() / (stem + ".res")).string(),
            generated_relative, resources);
        collect_res_file(
            (stdfs::path(options.game_dir) / generated_relative).string(),
            generated_relative, resources);

        add_optional(resources, options, "maps/" + stem + ".txt");
        add_optional(resources, options, "maps/" + stem + "_detail.txt");
        add_optional(resources, options, "maps/" + stem + ".nav");
        add_optional(resources, options, "maps/" + stem + ".cfg");
        add_optional(resources, options, "overviews/" + stem + ".txt");
        add_optional(resources, options, "overviews/" + stem + ".bmp");
        add_optional(resources, options, "overviews/" + stem + ".tga");
        add_model_companions(resources, options);

        struct copy_job
        {
            std::string relative;
            std::string source;
            std::string destination;
        };
        std::vector<copy_job> jobs;
        for (const auto &entry : resources.entries())
        {
            std::string source;
            source_location location =
                resolve_source(options, entry.second, source);
            if (location == source_location::missing)
            {
                out.missing.push_back(entry.second.relative);
                continue;
            }
            if (location == source_location::base)
            {
                out.provided_by_base.push_back(entry.second.relative);
                continue;
            }
            out.resources.push_back(entry.second.relative);
            jobs.push_back({
                entry.second.relative,
                std::move(source),
                (stdfs::path(output_root)
                 / stdfs::path(entry.second.relative)).string(),
            });
        }

        out.res_path =
            (stdfs::path(output_root) / generated_relative).string();
        if (options.strict && !out.missing.empty())
        {
            set_error(error, std::to_string(out.missing.size())
                      + " referenced resource(s) are missing");
            return false;
        }
        if (fs::exists(out.res_path) && !options.force)
        {
            set_error(error, "RES file already exists: '" + out.res_path
                      + "' (use -force to overwrite)");
            return false;
        }
        for (const copy_job &job : jobs)
        {
            if (fs::exists(job.destination) && !same_file(job.source, job.destination)
                && !options.force)
            {
                set_error(error, "packed file already exists: '" + job.destination
                          + "' (use -force to overwrite)");
                return false;
            }
        }

        for (const copy_job &job : jobs)
        {
            if (same_file(job.source, job.destination))
            {
                out.unchanged++;
                continue;
            }
            std::vector<byte> data;
            if (!fs::read_all(job.source, data))
            {
                set_error(error, "could not read resource '" + job.source + "'");
                return false;
            }
            std::string parent = fs::directory(job.destination);
            if (!parent.empty() && !fs::make_directory(parent))
            {
                set_error(error, "could not create packed directory '" + parent + "'");
                return false;
            }
            if (!fs::write_all(job.destination, data.data(), data.size()))
            {
                set_error(error, "could not write packed resource '"
                          + job.destination + "'");
                return false;
            }
            out.copied++;
        }

        std::string res_text = "// generated by hltools bsp pack\r\n";
        std::string bsp_relative = normalize_relative("maps/" + map_name);
        res_text += bsp_relative + "\r\n";
        for (const std::string &relative : out.resources)
            if (lower(relative) != lower(bsp_relative))
                res_text += relative + "\r\n";

        std::string res_dir = fs::directory(out.res_path);
        if (!res_dir.empty() && !fs::make_directory(res_dir))
        {
            set_error(error, "could not create packed maps directory");
            return false;
        }
        if (!fs::write_all(out.res_path, res_text.data(), res_text.size()))
        {
            set_error(error, "could not write RES file '" + out.res_path + "'");
            return false;
        }
        return true;
    }
}
