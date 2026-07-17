#include "cmdline.h"
#include "string_util.h"

#include <cstdlib>
#include <cstring>

namespace cli
{
    namespace
    {
        // how many argument tokens a value consuming flag eats after itself
        // this is the union of every value flag across hlcsg/hlbsp/hlvis/hlrad
        // (and the ripent/common diagnostics), so that a value like the "12" in
        // "-threads 12" is never mistaken for the map name regardless of whether
        // the map name comes before or after the options boolean flags are not
        // listed (arity 0) matches the reference tools, which consume each
        // value inline with argv[++i] names are matched case insensitively
        int value_flag_arity(const std::string &flag)
        {
            static const char *arity3[] = {
                "-ambient", "-colourgamma", "-colourscale", "-colourjitter", "-jitter"};
            static const char *arity4[] = {"-drawsample"};
            static const char *arity1[] = {
                // common / diagnostic
                "-threads", "-texdata", "-lightdata", "-dev", "-lang",
                // hlcsg
                "-worldextent", "-scale", "-cliptype", "-brushunion", "-wadinclude",
                "-wadcfgfile", "-wadconfig", "-hullfile", "-nullfile",
                // hlbsp
                "-maxnodesize", "-subdivide",
                // hlvis
                "-maxdistance", "-connect", "-port", "-rate",
                // hlrad
                "-bounce", "-chop", "-texchop", "-fade", "-limiter", "-gamma",
                "-dlight", "-sky", "-smooth", "-smooth2", "-coring", "-dscale",
                "-minlight", "-softsky", "-compress", "-rgbcompress", "-depth",
                "-blockopaque", "-waddir", "-texreflectgamma", "-texreflectscale",
                "-blur", "-texlightgap", "-lights", "-vismatrix", "-bscale"};
            for (const char *f : arity4)
                if (str::iequals(flag.c_str(), f))
                    return 4;
            for (const char *f : arity3)
                if (str::iequals(flag.c_str(), f))
                    return 3;
            for (const char *f : arity1)
                if (str::iequals(flag.c_str(), f))
                    return 1;
            return 0;
        }
    }

    args::args(int argc, char **argv)
    {
        for (int i = 1; i < argc; i++)
            tokens_.emplace_back(argv[i]);
        // the map name is the first positional token: scan left to right,
        // skipping flags and the value tokens they consume this handles both
        // "hlrad mapname -opts" and "hlrad -opts mapname" invocations
        for (size_t i = 0; i < tokens_.size();)
        {
            const std::string &t = tokens_[i];
            if (!t.empty() && t[0] == '-')
            {
                i += 1 + (size_t)value_flag_arity(t);
                continue;
            }
            map_name_ = t;
            break;
        }
    }

    int args::index_of(const char *flag) const
    {
        for (size_t i = 0; i < tokens_.size(); i++)
        {
            if (tokens_[i] == flag)
                return (int)i;
        }
        return -1;
    }

    bool args::has(const char *flag) const {
        return index_of(flag) >= 0;
    }

    const char *args::value(const char *flag, const char *def) const
    {
        int i = index_of(flag);
        if (i < 0 || (size_t)(i + 1) >= tokens_.size())
            return def;
        return tokens_[i + 1].c_str();
    }

    std::vector<std::string> args::values(const char *flag) const
    {
        std::vector<std::string> out;
        for (size_t i = 0; i + 1 < tokens_.size(); i++)
        {
            if (tokens_[i] == flag)
                out.push_back(tokens_[i + 1]);
        }
        return out;
    }

    int args::int_value(const char *flag, int def) const
    {
        const char *v = value(flag, nullptr);
        return v ? std::atoi(v) : def;
    }

    double args::float_value(const char *flag, double def) const
    {
        const char *v = value(flag, nullptr);
        return v ? std::atof(v) : def;
    }
}
