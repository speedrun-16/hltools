#include "cmdline.h"
#include "string_util.h"

#include <cstdlib>
#include <cstring>

namespace cli
{
    namespace
    {
        constexpr unsigned csg = (unsigned)compiler_stage::csg;
        constexpr unsigned bsp = (unsigned)compiler_stage::bsp;
        constexpr unsigned vis = (unsigned)compiler_stage::vis;
        constexpr unsigned rad = (unsigned)compiler_stage::rad;
        constexpr unsigned all = csg | bsp | vis | rad;

        // flag, stages, following values, persisted in compile entity,
        // repeatable; the authoritative compiler option registry
        const option_spec options[] = {
            {"-threads", all, 1, true, false},

            {"-cliptype", csg, 1, true, false},
            {"-noclip", csg | bsp, 0, true, false},
            {"-nullifytrigger", csg, 0, true, false},
            {"-worldextent", csg, 1, true, false},
            {"-scale", csg | rad, 1, true, false},
            {"-brushunion", csg, 1, true, false},
            {"-wadtextures", csg, 0, true, false},
            {"-nowadautodetect", csg, 0, true, false},
            {"-wadinclude", csg, 1, true, true},
            {"-wadcfgfile", csg, 1, true, false},
            {"-wadconfig", csg, 1, true, false},
            {"-hullfile", csg, 1, true, false},
            {"-noskyclip", csg, 0, true, false},
            {"-nullfile", csg, 1, true, false},
            {"-nolightopt", csg, 0, true, false},
            {"-onlyents", csg, 0, false, false},

            {"-nofill", bsp, 0, true, false},
            {"-noinsidefill", bsp, 0, true, false},
            {"-notjunc", bsp, 0, true, false},
            {"-nobrink", bsp, 0, true, false},
            {"-noopt", bsp, 0, true, false},
            {"-noclipnodemerge", bsp, 0, true, false},
            {"-leakonly", bsp, 0, true, false},
            {"-allleaks", bsp, 0, true, false},
            {"-nonulltex", bsp, 0, true, false},
            {"-nohull2", bsp, 0, true, false},
            {"-maxnodesize", bsp, 1, true, false},
            {"-subdivide", bsp, 1, true, false},

            {"-fast", vis | rad, 0, true, false},
            {"-nofull", vis, 0, true, false},
            {"-nofixprt", vis, 0, true, false},
            {"-maxdistance", vis, 1, true, false},
            {"-full", vis, 0, false, false},

            {"-dump", rad, 0, true, false},
            {"-dumpgather", rad, 0, true, false},
            {"-gpu", rad, 0, true, false},
            {"-extra", rad, 0, true, false},
            {"-bounce", rad, 1, true, false},
            {"-nolerp", rad, 0, true, false},
            {"-chop", rad, 1, true, false},
            {"-texchop", rad, 1, true, false},
            {"-notexscale", rad, 0, true, false},
            {"-nosubdivide", rad, 0, true, false},
            {"-fade", rad, 1, true, false},
            {"-limiter", rad, 1, true, false},
            {"-drawoverload", rad, 0, true, false},
            {"-circus", rad, 0, true, false},
            {"-noskyfix", rad, 0, true, false},
            {"-incremental", rad, 0, true, false},
            {"-gamma", rad, 1, true, false},
            {"-dlight", rad, 1, true, false},
            {"-sky", rad, 1, true, false},
            {"-smooth", rad, 1, true, false},
            {"-smooth2", rad, 1, true, false},
            {"-coring", rad, 1, true, false},
            {"-lightdata", rad, 1, true, false},
            {"-texdata", rad, 1, true, false},
            {"-vismatrix", rad, 1, true, false},
            {"-nospread", rad, 0, true, false},
            {"-nopaque", rad, 0, true, false},
            {"-noopaque", rad, 0, true, false},
            {"-dscale", rad, 1, true, false},
            {"-customshadowwithbounce", rad, 0, true, false},
            {"-rgbtransfers", rad, 0, true, false},
            {"-minlight", rad, 1, true, false},
            {"-softsky", rad, 1, true, false},
            {"-nostudioshadow", rad, 0, true, false},
            {"-drawpatch", rad, 0, true, false},
            {"-drawedge", rad, 0, true, false},
            {"-drawlerp", rad, 0, true, false},
            {"-drawnudge", rad, 0, true, false},
            {"-compress", rad, 1, true, false},
            {"-rgbcompress", rad, 1, true, false},
            {"-depth", rad, 1, true, false},
            {"-blockopaque", rad, 1, true, false},
            {"-notextures", rad, 0, true, false},
            {"-texreflectgamma", rad, 1, true, false},
            {"-texreflectscale", rad, 1, true, false},
            {"-blur", rad, 1, true, false},
            {"-noemitterrange", rad, 0, true, false},
            {"-nobleedfix", rad, 0, true, false},
            {"-texlightgap", rad, 1, true, false},
            {"-pre25", rad, 0, true, false},
            {"-waddir", rad, 1, true, true},
            {"-lights", rad, 1, true, false},
            {"-ambient", rad, 3, true, false},
            {"-colourgamma", rad, 3, true, false},
            {"-colourscale", rad, 3, true, false},
            {"-colourjitter", rad, 3, true, false},
            {"-jitter", rad, 3, true, false},
            {"-drawsample", rad, 4, true, false},
            {"-nochart", rad, 0, false, false},

            // driver and diagnostic options still need arity metadata so their
            // values are not mistaken for map names
            {"-dumpintermediates", 0, 0, false, false},
            {"--dump-intermediates", 0, 0, false, false},
            {"-noembedsource", 0, 0, false, false},
            {"--no-embed-source", 0, 0, false, false},
            {"-dev", 0, 1, false, false},
            {"-lang", 0, 1, false, false},
            {"-connect", 0, 1, false, false},
            {"-port", 0, 1, false, false},
            {"-rate", 0, 1, false, false},
            {"-bscale", 0, 1, false, false},
        };
    }

    const option_spec *find_option_spec(const std::string &flag)
    {
        for (const option_spec &option : options)
            if (str::iequals(flag.c_str(), option.flag))
                return &option;
        return nullptr;
    }

    bool option_applies_to(compiler_stage stage, const std::string &flag,
                           bool persistent_only)
    {
        const option_spec *option = find_option_spec(flag);
        return option
            && (option->stages & (unsigned)stage) != 0
            && (!persistent_only || option->persist);
    }

    int flag_arity(const std::string &flag)
    {
        const option_spec *option = find_option_spec(flag);
        return option ? option->arity : 0;
    }

    args::args(int argc, char **argv)
    {
        for (int i = 1; i < argc; i++)
            tokens_.emplace_back(argv[i]);
        // the map name is the first positional token: scan left to right,
        // skipping flags and the value tokens they consume this handles both
        // "hltools rad mapname -opts" and "... -opts mapname" invocations
        for (size_t i = 0; i < tokens_.size();)
        {
            const std::string &t = tokens_[i];
            if (!t.empty() && t[0] == '-')
            {
                i += 1 + (size_t)flag_arity(t);
                continue;
            }
            map_name_ = t;
            break;
        }
    }

    int args::index_of(const char *flag) const
    {
        for (size_t i = tokens_.size(); i > 0; i--)
        {
            if (tokens_[i - 1] == flag)
                return (int)i - 1;
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
