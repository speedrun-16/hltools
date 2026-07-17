#pragma once

#include <string>
#include <vector>

// command line accessor for a compile tool a tool is invoked as
// "hlrad [options] mapname" (or "hlrad mapname [options]"), so flags are queried
// by name and the map path is the first positional token that is not a flag or a
// flag's value cold path, run once at startup

namespace cli
{
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
