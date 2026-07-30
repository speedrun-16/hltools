#pragma once

#include <string>
#include <vector>

// command line accessor for a compile tool a tool is invoked as
// "hltools rad [options] mapname" (or "... mapname [options]"), so flags are queried
// by name and the map path is the first positional token that is not a flag or a
// flag's value cold path, run once at startup

namespace cli
{
    enum class compiler_stage : unsigned
    {
        csg = 1,
        bsp = 2,
        vis = 4,
        rad = 8,
    };

    // one registry entry for a compiler option; stage membership drives
    // info_compile_parameters and arity drives positional command line parsing
    // persist=false covers operational switches that should not be written
    // back into an embedded compile recipe
    struct option_spec
    {
        const char *flag;
        unsigned stages;
        int arity;
        bool persist;
        bool repeatable;
    };

    const option_spec *find_option_spec(const std::string &flag);
    bool option_applies_to(compiler_stage stage, const std::string &flag,
                           bool persistent_only = false);

    // number of value tokens consumed by a known option; zero covers boolean
    // switches and unknown names, while callers validate names separately
    int flag_arity(const std::string &flag);

    class args
    {
    public:
        args(int argc, char **argv);

        // "-extra" style presence flag
        bool has(const char *flag) const;

        // value following a flag ("-threads 12" -> "12"), or def if absent
        const char *value(const char *flag, const char *def = nullptr) const;
        std::vector<std::string> values(const char *flag) const;
        int int_value(const char *flag, int def) const;
        double float_value(const char *flag, double def) const;

        // the final argument, conventionally the map name
        const std::string &map_name() const {
            return map_name_;
        }
        bool empty() const {
            return tokens_.empty();
        }

    private:
        int index_of(const char *flag) const;

        std::vector<std::string> tokens_;
        std::string map_name_;
    };
}
