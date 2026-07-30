#include "compile_parameters.h"

#include <algorithm>
#include <cctype>
#include <map>
#include <sstream>
#include <utility>

#include "common/error.h"
#include "common/string_util.h"
#include "format/map/document.h"
#include "format/zip/archive.h"

namespace tools
{
    namespace
    {
        const char *stage_name(cli::compiler_stage stage)
        {
            switch (stage)
            {
            case cli::compiler_stage::csg: return "csg";
            case cli::compiler_stage::bsp: return "bsp";
            case cli::compiler_stage::vis: return "vis";
            case cli::compiler_stage::rad: return "rad";
            }
            return "";
        }

        std::string lower(std::string value)
        {
            for (char &c : value)
                c = (char)std::tolower((unsigned char)c);
            return value;
        }

        bool enabled_value(const std::string &value)
        {
            return value == "1" || str::iequals(value.c_str(), "true")
                || str::iequals(value.c_str(), "yes")
                || str::iequals(value.c_str(), "on");
        }

        void append_option(
            std::vector<std::string> &tokens, const std::string &flag,
            const std::string &value)
        {
            const int arity = cli::flag_arity(flag);
            if (arity == 0)
            {
                if (enabled_value(value))
                    tokens.push_back(flag);
                return;
            }

            tokens.push_back(flag);
            if (arity == 1)
            {
                tokens.push_back(value);
                return;
            }

            std::istringstream values(value);
            for (int i = 0; i < arity; i++)
            {
                std::string token;
                if (!(values >> token))
                {
                    err::fatal(
                        "%s requires %d values in info_compile_parameters",
                        flag.c_str(), arity);
                }
                tokens.push_back(std::move(token));
            }
        }

        std::vector<std::string> stage_tokens(
            const compile_parameters &parameters, cli::compiler_stage stage)
        {
            std::vector<std::string> tokens;
            const std::string prefix = std::string(stage_name(stage)) + "_";
            for (const format::entity::pair &pair : parameters)
            {
                const std::string key = lower(pair.first);
                if (key == "threads")
                {
                    append_option(tokens, "-threads", pair.second);
                    continue;
                }
                if (key.compare(0, prefix.size(), prefix) != 0)
                    continue;

                const std::string flag = "-" + key.substr(prefix.size());
                if (!cli::option_applies_to(stage, flag, true))
                {
                    err::fatal(
                        "unknown info_compile_parameters key '%s'",
                        pair.first.c_str());
                }
                append_option(tokens, flag, pair.second);
            }
            return tokens;
        }

        void append_entity_parameters(
            const format::entity &entity, compile_parameters &out)
        {
            if (!str::iequals(
                    entity.value("classname"), "info_compile_parameters"))
                return;
            for (const format::entity::pair &pair : entity.pairs())
            {
                if (str::iequals(pair.first.c_str(), "classname")
                    || str::iequals(pair.first.c_str(), "origin"))
                    continue;
                out.push_back(pair);
            }
        }

        void set_value(
            std::map<std::string, std::vector<std::string>> &values,
            const std::string &key, const std::string &value, bool repeatable)
        {
            std::vector<std::string> &stored = values[key];
            if (!repeatable)
            {
                stored.assign(1, value);
                return;
            }
            if (std::find(stored.begin(), stored.end(), value) == stored.end())
                stored.push_back(value);
        }
    }

    compile_parameters compile_parameters_from_map(const std::string &source)
    {
        compile_parameters out;
        for (const format::map_source_entity &entity :
             format::parse_map_source_entities(source))
        {
            append_entity_parameters(entity.keyvalues, out);
        }
        return out;
    }

    compile_parameters compile_parameters_from_bsp(const format::map_data &map)
    {
        if (!map.embedded_zip.empty())
        {
            std::vector<byte> source;
            if (format::read_zip_map(map.embedded_zip, source))
            {
                compile_parameters embedded = compile_parameters_from_map(
                    std::string(source.begin(), source.end()));
                if (!embedded.empty())
                    return embedded;
            }
        }

        compile_parameters out;
        for (const format::entity &entity : format::parse_entities(map.entities))
            append_entity_parameters(entity, out);
        return out;
    }

    staged_arguments::staged_arguments(
        int argc, char **actual, const compile_parameters &parameters,
        cli::compiler_stage stage)
    {
        storage.emplace_back(actual[0]);
        std::vector<std::string> defaults = stage_tokens(parameters, stage);
        storage.insert(storage.end(), defaults.begin(), defaults.end());
        for (int i = 1; i < argc; i++)
            storage.emplace_back(actual[i]);
        for (std::string &value : storage)
            argv.push_back(&value[0]);
        argv.push_back(nullptr);
    }

    compile_parameters canonical_compile_parameters(
        int argc, char **argv, const compile_parameters &existing)
    {
        std::map<std::string, std::vector<std::string>> values;
        for (const format::entity::pair &pair : existing)
        {
            const std::string key = lower(pair.first);
            if (key == "threads")
            {
                set_value(values, key, pair.second, false);
                continue;
            }
            for (cli::compiler_stage stage : {
                     cli::compiler_stage::csg, cli::compiler_stage::bsp,
                     cli::compiler_stage::vis, cli::compiler_stage::rad})
            {
                const std::string prefix =
                    std::string(stage_name(stage)) + "_";
                if (key.compare(0, prefix.size(), prefix) != 0)
                    continue;
                const std::string flag = "-" + key.substr(prefix.size());
                const cli::option_spec *option = cli::find_option_spec(flag);
                if (option && cli::option_applies_to(stage, flag, true))
                    set_value(values, key, pair.second, option->repeatable);
            }
        }

        for (int i = 1; i < argc; i++)
        {
            const std::string flag = argv[i];
            const cli::option_spec *option = cli::find_option_spec(flag);
            if (!option || !option->persist)
                continue;

            std::string value = "1";
            if (option->arity > 0)
            {
                if (i + option->arity >= argc)
                    break;
                value.clear();
                for (int j = 0; j < option->arity; j++)
                {
                    if (j)
                        value += ' ';
                    value += argv[i + 1 + j];
                }
            }

            if (str::iequals(flag.c_str(), "-threads"))
                set_value(values, "threads", value, false);
            else
            {
                std::string suffix = lower(flag);
                while (!suffix.empty() && suffix[0] == '-')
                    suffix.erase(suffix.begin());
                for (cli::compiler_stage stage : {
                         cli::compiler_stage::csg, cli::compiler_stage::bsp,
                         cli::compiler_stage::vis, cli::compiler_stage::rad})
                {
                    if (!cli::option_applies_to(stage, flag, true))
                        continue;
                    set_value(
                        values,
                        std::string(stage_name(stage)) + "_" + suffix,
                        value, option->repeatable);
                }
            }
            i += option->arity;
        }

        compile_parameters out;
        for (const auto &entry : values)
            for (const std::string &value : entry.second)
                out.emplace_back(entry.first, value);
        return out;
    }

    void erase_runtime_compile_parameters(format::map_data &map)
    {
        format::erase_map_entities(
            map.entities, {"info_compile_parameters"});
    }
}
