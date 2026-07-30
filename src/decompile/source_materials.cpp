#include "source_materials.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <cstdlib>

#include <filesystem>

#include "common/filesystem.h"
#include "format/bmp/image.h"
#include "format/image/quantize.h"
#include "format/vbsp/data.h"
#include "format/vbsp/pakfile.h"
#include "format/vmt/vmt.h"
#include "format/vpk/vpk.h"
#include "format/vtf/vtf.h"

namespace decompile
{
    namespace
    {
        constexpr unsigned goldsrc_max_dim = 512;
        constexpr unsigned goldsrc_min_dim = 16;
        constexpr int max_goldsrc_texname = 15;

        std::string to_lower(std::string v)
        {
            std::transform(v.begin(), v.end(), v.begin(),
                           [](unsigned char c) { return (char)std::tolower(c); });
            return v;
        }

        bool iequals(const std::string &a, const char *b)
        {
            std::size_t i = 0;
            for (; i < a.size(); i++)
            {
                if (b[i] == '\0'
                    || std::tolower((unsigned char)a[i]) != std::tolower((unsigned char)b[i]))
                    return false;
            }
            return b[i] == '\0';
        }

        const char *special_material(const std::string &m)
        {
            if (iequals(m, "TOOLS/TOOLSNODRAW") || iequals(m, "TOOLS/TOOLSINVISIBLE")
                || iequals(m, "TOOLS/TOOLSINVISIBLENONSOLID"))
                return "NULL";
            if (iequals(m, "TOOLS/TOOLSCLIP") || iequals(m, "TOOLS/TOOLSPLAYERCLIP")
                || iequals(m, "TOOLS/TOOLSNPCCLIP") || iequals(m, "TOOLS/TOOLSGRENADECLIP"))
                return "CLIP";
            if (iequals(m, "TOOLS/TOOLSTRIGGER"))
                return "AAATRIGGER";
            if (iequals(m, "TOOLS/TOOLSHINT"))
                return "HINT";
            if (iequals(m, "TOOLS/TOOLSSKIP"))
                return "SKIP";
            if (iequals(m, "TOOLS/TOOLSSKYBOX") || iequals(m, "TOOLS/TOOLSSKYBOX2D"))
                return "SKY";
            return nullptr;
        }

        // csg and the map parser derive brush contents from a texture name's
        // leading characters, so an ordinary material whose name happens to
        // begin with one of these would silently become sky, clip, an origin
        // brush and so on. source material names collide with this more often
        // than you would think: a decorative "sky" prefixed stripe is not sky.
        bool has_reserved_prefix(const std::string &name)
        {
            static const char *const reserved[] = {
                "sky", "env_sky", "content", "origin", "boundingbox", "hint",
                "solidhint", "bevelhint", "splitface", "skip", "translucent",
                "null", "bevel", "clip",
            };
            for (const char *word : reserved)
            {
                std::size_t length = std::strlen(word);
                if (name.size() < length)
                    continue;
                bool match = true;
                for (std::size_t i = 0; i < length; i++)
                    if (std::tolower((unsigned char)name[i]) != word[i])
                    {
                        match = false;
                        break;
                    }
                if (match)
                    return true;
            }
            return false;
        }

        std::string basename_of(const std::string &path)
        {
            std::size_t slash = path.find_last_of("/\\");
            return slash == std::string::npos ? path : path.substr(slash + 1);
        }

        // selectors may use either a Source material path or its generated
        // GoldSrc texture name. Match on the basename because GoldSrc's
        // 15-character name limit cannot reliably retain source directories.
        std::string texture_selector(std::string value)
        {
            std::replace(value.begin(), value.end(), '\\', '/');
            while (!value.empty() && value.front() == '/')
                value.erase(value.begin());
            value = basename_of(value);
            while (!value.empty()
                   && (value.front() == '{' || value.front() == '!'
                       || value.front() == '+' || value.front() == '-'))
                value.erase(value.begin());
            return to_lower(value);
        }

        bool preserve_full_size(const std::string &material,
                                const std::vector<std::string> &selectors)
        {
            std::string candidate = texture_selector(material);
            for (const std::string &selector : selectors)
                if (candidate == texture_selector(selector))
                    return true;
            return false;
        }

        // VMT texture references are game-relative even when authored with a
        // leading slash. Pak/VPK entries never carry that slash, so normalize it
        // before building the "materials/<name>.vtf" lookup path.
        std::string normalize_material_reference(std::string path)
        {
            std::replace(path.begin(), path.end(), '\\', '/');
            while (!path.empty() && path.front() == '/')
                path.erase(path.begin());
            return path;
        }

        // reads an asset from the pakfile first, then the -game content dirs
        // (loose files override the game's stock vpk archives, matching the
        // engine's search order). dirs are searched in the order given, so the
        // mod directory should come before shared content like hl2.
        bool load_asset(const format::pakfile &pak,
                        const std::vector<std::string> &game_dirs,
                        const std::vector<format::vpk_archive> &vpks,
                        const std::string &relative, std::vector<byte> &out)
        {
            if (pak.extract(relative, out))
                return true;
            for (const std::string &dir : game_dirs)
                if (!dir.empty() && fs::read_all(dir + "/" + relative, out))
                    return true;
            for (const format::vpk_archive &vpk : vpks)
                if (vpk.extract(relative, out))
                    return true;
            return false;
        }

        // opens every *_dir.vpk archive in each -game content dir. a source game
        // splits its content across trees (cstrike plus the shared hl2 one), so
        // several dirs are the norm rather than the exception.
        std::vector<format::vpk_archive>
        open_game_vpks(const std::vector<std::string> &game_dirs)
        {
            std::vector<format::vpk_archive> vpks;
            for (const std::string &game_dir : game_dirs)
            {
                if (game_dir.empty())
                    continue;
                std::error_code ec;
                for (const auto &item : std::filesystem::directory_iterator(game_dir, ec))
                {
                    std::string path = item.path().string();
                    if (path.size() < 8
                        || !iequals(path.substr(path.size() - 8), "_dir.vpk"))
                        continue;
                    format::vpk_archive vpk;
                    if (vpk.open(path))
                        vpks.push_back(std::move(vpk));
                }
            }
            return vpks;
        }

        // rounds a dimension into the requested GoldSrc-legal range. Reducing
        // the converted image and its UV vectors together preserves apparent
        // world scale while reducing BSP subdivision and lightmap density.
        // decode_vtf hands back the largest MIP no bigger than the cap, and mips
        // only come in powers of two. Asking it for 96 would therefore return
        // the 64 mip and fit_dim could never reach 96, so round the decode cap
        // up to the enclosing power of two and let fit_dim/resample land on the
        // requested size from above.
        unsigned mip_cap(unsigned max_dim)
        {
            unsigned cap = goldsrc_min_dim;
            while (cap < max_dim && cap < goldsrc_max_dim)
                cap *= 2;
            return cap;
        }

        unsigned fit_dim(unsigned d, unsigned max_dim)
        {
            if (d < goldsrc_min_dim)
                return goldsrc_min_dim;
            if (d > max_dim)
                return max_dim;
            return d - (d % goldsrc_min_dim);
        }

        std::vector<byte> resample_nearest(const std::vector<byte> &src, unsigned sw,
                                           unsigned sh, unsigned dw, unsigned dh)
        {
            std::vector<byte> out((std::size_t)dw * dh * 3);
            for (unsigned y = 0; y < dh; y++)
            {
                unsigned sy = sh ? (y * sh) / dh : 0;
                for (unsigned x = 0; x < dw; x++)
                {
                    unsigned sx = sw ? (x * sw) / dw : 0;
                    std::size_t s = ((std::size_t)sy * sw + sx) * 3;
                    std::size_t o = ((std::size_t)y * dw + x) * 3;
                    out[o] = src[s];
                    out[o + 1] = src[s + 1];
                    out[o + 2] = src[s + 2];
                }
            }
            return out;
        }

        std::vector<byte> resample_alpha_nearest(const std::vector<byte> &src, unsigned sw,
                                                 unsigned sh, unsigned dw, unsigned dh)
        {
            std::vector<byte> out((std::size_t)dw * dh);
            for (unsigned y = 0; y < dh; y++)
            {
                unsigned sy = sh ? (y * sh) / dh : 0;
                for (unsigned x = 0; x < dw; x++)
                {
                    unsigned sx = sw ? (x * sw) / dw : 0;
                    out[(std::size_t)y * dw + x] = src[(std::size_t)sy * sw + sx];
                }
            }
            return out;
        }

        // opacity below which a masked texel is punched out
        constexpr byte mask_threshold = 128;

        // true when the alpha channel holds a cutout rather than one flat opacity.
        // a uniformly translucent material (glass) is better served by goldsrc's
        // per entity renderamt, so only a channel with both a real transparent and
        // a real opaque population becomes a '{' mask.
        bool alpha_carries_shape(const std::vector<byte> &alpha)
        {
            if (alpha.empty())
                return false;
            std::size_t below = 0;
            for (byte a : alpha)
                if (a < mask_threshold)
                    below++;
            double fraction = (double)below / (double)alpha.size();
            return fraction > 0.02 && fraction < 0.98;
        }
    }

    namespace
    {
        // 16 is the smallest goldsrc-legal dimension and a multiple of 16, which
        // keeps four valid mip levels
        constexpr unsigned flat_texture_dim = 16;

        bool build_flat_texture(const std::string &name, const byte rgb[3],
                                format::mip_texture &out)
        {
            format::indexed_image image;
            image.width = flat_texture_dim;
            image.height = flat_texture_dim;
            image.pixels.assign((std::size_t)flat_texture_dim * flat_texture_dim, 0);
            for (int k = 0; k < 3; k++)
                image.palette[0][(std::size_t)k] = rgb[k];
            return format::build_mip_texture(name, image, out);
        }
    }

    std::string material_catalog::unique_name(const std::string &material,
                                              const char *prefix)
    {
        std::string base = prefix + basename_of(material);
        for (char &c : base)
            c = (char)std::toupper((unsigned char)c);
        for (char &c : base)
        {
            if (!(std::isalnum((unsigned char)c) || c == '_' || c == '{' || c == '!'
                  || c == '+' || c == '-'))
                c = '_';
        }
        if (base.empty())
            base = "TEX";
        // a leading underscore keeps the readable name while stepping out of the
        // compiler's reserved namespace
        if (has_reserved_prefix(base))
            base.insert(base.begin(), '_');
        if (base.size() > max_goldsrc_texname)
            base.resize(max_goldsrc_texname);

        std::string candidate = base;
        for (int suffix = 1; used_names_.count(to_lower(candidate)); suffix++)
        {
            std::string tag = std::to_string(suffix);
            candidate = base.substr(0, max_goldsrc_texname - tag.size()) + tag;
        }
        used_names_.insert(to_lower(candidate));
        return candidate;
    }

    void material_catalog::build(const format::source_map_data &map,
                                 const std::vector<std::string> &game_dirs,
                                 unsigned max_texture_size,
                                 const std::vector<std::string> &full_size_textures)
    {
        max_texture_size =
            std::max(goldsrc_min_dim, std::min(goldsrc_max_dim, max_texture_size));
        format::pakfile pak;
        pak.open(map.pakfile);
        std::vector<format::vpk_archive> vpks = open_game_vpks(game_dirs);

        for (const std::string &material : map.material_names)
        {
            if (material.empty())
                continue;
            std::string key = to_lower(material);
            if (by_material_.count(key))
                continue;

            resolved_material rm;
            if (const char *special = special_material(material))
            {
                rm.name = special;
                rm.special = true;
                by_material_[key] = rm;
                continue;
            }

            // resolve the vmt, following one level of patch indirection. patch
            // replacements are already flattened by the parser, so a property is
            // looked up in the patch first, then the included base material.
            std::vector<byte> vmt_bytes;
            std::string basetexture, translucent_value, alpha_value, alphatest_value;
            bool water = false;
            std::string material_reference =
                normalize_material_reference(material);
            if (load_asset(pak, game_dirs, vpks,
                           "materials/" + material_reference + ".vmt",
                           vmt_bytes))
            {
                format::vmt_material vmt;
                if (format::parse_vmt(vmt_bytes, vmt))
                {
                    format::vmt_material base;
                    if (vmt.is_patch())
                    {
                        std::string include = vmt.patch_include();
                        std::vector<byte> base_bytes;
                        if (include.empty()
                            || !load_asset(pak, game_dirs, vpks, include, base_bytes)
                            || !format::parse_vmt(base_bytes, base))
                        {
                            base = format::vmt_material{};
                        }
                    }
                    auto lookup = [&](const char *property)
                    {
                        std::string value = vmt.get(property);
                        return value.empty() ? base.get(property) : value;
                    };
                    basetexture =
                        normalize_material_reference(lookup("$basetexture"));
                    translucent_value = lookup("$translucent");
                    alpha_value = lookup("$alpha");
                    alphatest_value = lookup("$alphatest");

                    // the Water shader has no $basetexture (it renders from a
                    // runtime reflection/refraction pass); its editor preview
                    // image is the closest thing to a paintable water surface
                    const std::string &shader = vmt.is_patch() ? base.shader : vmt.shader;
                    water = iequals(shader, "water") || lookup("%compilewater") == "1";
                    rm.unlit = iequals(shader, "UnlitGeneric");
                    if (water)
                        basetexture =
                            normalize_material_reference(lookup("%tooltexture"));
                }
            }

            format::vtf_image image;
            bool decoded = false;
            unsigned material_max_size =
                preserve_full_size(material, full_size_textures)
                ? goldsrc_max_dim : max_texture_size;
            if (!basetexture.empty())
            {
                std::vector<byte> vtf_bytes;
                if (load_asset(pak, game_dirs, vpks,
                               "materials/" + basetexture + ".vtf", vtf_bytes))
                    decoded = format::decode_vtf(vtf_bytes, mip_cap(material_max_size),
                                                 image);
            }

            if (decoded && image.full_width && image.full_height)
            {
                unsigned gw = fit_dim(image.width, material_max_size);
                unsigned gh = fit_dim(image.height, material_max_size);
                bool rescaled = gw != image.width || gh != image.height;
                std::vector<byte> rgb = rescaled
                    ? resample_nearest(image.rgb, image.width, image.height, gw, gh)
                    : image.rgb;
                std::vector<byte> alpha = (rescaled && !image.alpha.empty())
                    ? resample_alpha_nearest(image.alpha, image.width, image.height, gw, gh)
                    : image.alpha;

                // $alpha scales the whole material uniformly, so a value of zero
                // means invisible whatever the alpha channel holds. those brushes
                // (invisible slide clips here) must stay on the renderamt path.
                bool alpha_scaled = !alpha_value.empty();
                double alpha_scale =
                    alpha_scaled ? std::atof(alpha_value.c_str()) : 1.0;
                bool invisible = alpha_scaled && alpha_scale <= 0.01;

                // $alphatest cuts the shape out of the texture; a $translucent
                // material whose alpha holds a cutout (a fence, a decal) needs the
                // same treatment, because goldsrc's renderamt is per entity and
                // would otherwise flatten the artwork to one uniform colour.
                // a plain $selfillum material is excluded: there the alpha channel
                // is a glow mask over solid geometry, and masking it would punch
                // holes through the map's main textures.
                bool masked = !water && !invisible && !alpha.empty()
                    && (alphatest_value == "1"
                        || (translucent_value == "1" && alpha_carries_shape(alpha)));

                format::indexed_image indexed;
                format::mip_texture texture;
                std::string name =
                    unique_name(material, water ? "!" : (masked ? "{" : ""));
                bool built = masked
                    ? format::quantize_rgb_masked(rgb.data(), alpha.data(),
                                                  mask_threshold, gw, gh, indexed)
                    : format::quantize_rgb(rgb.data(), gw, gh, indexed);
                if (built && format::build_mip_texture(name, indexed, texture))
                {
                    rm.name = name;
                    rm.has_texture = true;
                    rm.u_scale = (double)gw / (double)image.full_width;
                    rm.v_scale = (double)gh / (double)image.full_height;
                    rm.water = water;
                    if (water)
                    {
                        // goldsrc water opacity comes from the func_water entity,
                        // not the texture; half-transparent is the usual look
                        rm.render_amount = 128;
                    }
                    else if (masked)
                    {
                        // the cutout lives in the palette, so the entity renders
                        // solid: rendermode 4 with a full renderamt
                        rm.masked = true;
                        rm.render_amount = 255;
                        masked_++;
                    }
                    else
                    {
                        rm.translucent = translucent_value == "1";
                        rm.render_amount = (int)image.alpha_mean;
                        // $alpha 0 is a legitimate "draw nothing", so the scale
                        // is applied from zero upwards, not from just above it
                        if (alpha_scaled && alpha_scale >= 0 && alpha_scale < 1)
                        {
                            rm.translucent = true;
                            rm.render_amount = (int)(rm.render_amount * alpha_scale);
                        }
                    }
                    textures_.push_back(std::move(texture));
                    by_material_[key] = rm;
                    converted_++;
                    continue;
                }
            }

            // unresolved: give the sides a stable unique name and a flat magenta
            // tile, so the map still compiles against the companion wad alone and
            // the missing material is obvious in game rather than a load failure.
            rm.name = unique_name(material, water ? "!" : "");
            rm.water = water;
            static const byte missing_rgb[3] = {255, 0, 220};
            format::mip_texture placeholder;
            if (build_flat_texture(rm.name, missing_rgb, placeholder))
            {
                rm.has_texture = true;
                textures_.push_back(std::move(placeholder));
            }
            by_material_[key] = rm;
            placeholders_++;
        }

        fallback_.name = "NULL";
    }

    const resolved_material &material_catalog::resolve(const std::string &material) const
    {
        auto it = by_material_.find(to_lower(material));
        return it == by_material_.end() ? fallback_ : it->second;
    }
}
