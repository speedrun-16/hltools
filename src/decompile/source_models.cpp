#include "source_models.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <set>
#include <system_error>

#include "common/binary.h"
#include "common/filesystem.h"
#include "format/bsp/entity_lump.h"
#include "format/image/quantize.h"
#include "format/mdl/goldsrc_mdl.h"
#include "format/mdl/source_mdl.h"
#include "format/vbsp/data.h"
#include "format/vbsp/pakfile.h"
#include "format/vmt/vmt.h"
#include "format/vpk/vpk.h"
#include "format/vtf/vtf.h"

namespace decompile
{
    namespace
    {
        // goldsrc skins must be at most 512 on a side and a multiple of 16
        constexpr unsigned max_skin_dim = 512;

        std::string to_lower(std::string s)
        {
            for (char &c : s)
                c = (char)std::tolower((unsigned char)c);
            return s;
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

        std::string normalize(std::string path)
        {
            std::replace(path.begin(), path.end(), '\\', '/');
            while (!path.empty() && path.front() == '/')
                path.erase(path.begin());
            return path;
        }

        unsigned fit_dim(unsigned d)
        {
            if (d < 16)
                return 16;
            if (d > max_skin_dim)
                return max_skin_dim;
            return d - (d % 16);
        }

        std::vector<byte> resample(const std::vector<byte> &src, unsigned sw, unsigned sh,
                                   unsigned dw, unsigned dh)
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

        // pakfile, then loose content, then the game's vpk archives
        struct content_source
        {
            format::pakfile pak;
            std::vector<std::string> game_dirs;
            std::vector<format::vpk_archive> vpks;

            bool load(const std::string &relative, std::vector<byte> &out) const
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
        };

        void open_vpks(content_source &content)
        {
            for (const std::string &game_dir : content.game_dirs)
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
                        content.vpks.push_back(std::move(vpk));
                }
            }
        }

        // the model's material names are relative to its cdmaterials list, so
        // every directory is tried until one resolves
        bool resolve_skin(const content_source &content, const format::studio_model &model,
                          const std::string &material, format::studio_texture &out)
        {
            out.name = to_lower(material) + ".bmp";

            std::string basetexture;
            for (const std::string &dir : model.material_dirs)
            {
                std::vector<byte> vmt_bytes;
                std::string path = "materials/" + normalize(dir) + material + ".vmt";
                if (!content.load(path, vmt_bytes))
                    continue;
                format::vmt_material vmt;
                if (!format::parse_vmt(vmt_bytes, vmt))
                    continue;
                basetexture = normalize(vmt.get("$basetexture"));
                // an alpha cut material becomes a masked skin, the model
                // equivalent of a '{' world texture
                if (vmt.get("$alphatest") == "1")
                    out.flags |= format::studio_nf_masked;
                break;
            }
            if (basetexture.empty())
                basetexture = normalize(material);

            std::vector<byte> vtf_bytes;
            if (!content.load("materials/" + basetexture + ".vtf", vtf_bytes))
            {
                for (const std::string &dir : model.material_dirs)
                {
                    std::string path =
                        "materials/" + normalize(dir) + material + ".vtf";
                    if (content.load(path, vtf_bytes))
                        break;
                }
            }
            if (vtf_bytes.empty())
                return false;

            format::vtf_image image;
            if (!format::decode_vtf(vtf_bytes, max_skin_dim, image) || image.width == 0)
                return false;

            unsigned w = fit_dim(image.width), h = fit_dim(image.height);
            std::vector<byte> rgb = (w == image.width && h == image.height)
                ? image.rgb
                : resample(image.rgb, image.width, image.height, w, h);
            std::vector<byte> alpha;
            if (!image.alpha.empty())
            {
                alpha.assign((std::size_t)w * h, 255);
                for (unsigned y = 0; y < h; y++)
                    for (unsigned x = 0; x < w; x++)
                    {
                        unsigned sy = image.height ? (y * image.height) / h : 0;
                        unsigned sx = image.width ? (x * image.width) / w : 0;
                        alpha[(std::size_t)y * w + x] =
                            image.alpha[(std::size_t)sy * image.width + sx];
                    }
            }

            if ((out.flags & format::studio_nf_masked) && !alpha.empty())
                format::quantize_rgb_masked(rgb.data(), alpha.data(), 128, w, h, out.image);
            else
            {
                out.flags &= ~format::studio_nf_masked;
                format::quantize_rgb(rgb.data(), w, h, out.image);
            }
            return true;
        }

        bool convert_loaded(const content_source &content,
                            const std::vector<byte> &mdl,
                            const std::vector<byte> &vvd,
                            const std::vector<byte> &vtx,
                            source_model_conversion &out,
                            std::string *error)
        {
            out = source_model_conversion{};

            format::studio_model model;
            if (!format::load_source_model(mdl, vvd, vtx, model, error))
                return false;

            out.vertices = model.vertices.size();
            out.triangles = model.triangle_count();
            for (const std::string &material : model.materials)
            {
                format::studio_texture texture;
                if (!resolve_skin(content, model, material, texture))
                {
                    // a flat placeholder keeps the model loadable and makes the
                    // missing material obvious rather than silently black
                    texture.name = to_lower(material) + ".bmp";
                    texture.image.width = 16;
                    texture.image.height = 16;
                    texture.image.pixels.assign(16 * 16, 0);
                    texture.image.palette[0] = {255, 0, 220};
                    out.missing_skins++;
                }
                model.textures.push_back(std::move(texture));
            }
            out.textures = model.textures.size();
            return format::write_goldsrc_model(model, out.data, error);
        }

        // "models/props/tree.mdl" -> the three files that make up the model
        bool convert_one(const content_source &content, const std::string &reference,
                         source_model_conversion &out)
        {
            std::string base = normalize(reference);
            if (base.size() > 4 && iequals(base.substr(base.size() - 4), ".mdl"))
                base.resize(base.size() - 4);

            std::vector<byte> mdl, vvd, vtx;
            if (!content.load(base + ".mdl", mdl) || !content.load(base + ".vvd", vvd))
                return false;
            // the dx90 index buffer is the one shipped with modern content; the
            // unsuffixed name is the fallback older tools wrote
            if (!content.load(base + ".dx90.vtx", vtx)
                && !content.load(base + ".dx80.vtx", vtx)
                && !content.load(base + ".vtx", vtx))
                return false;

            return convert_loaded(content, mdl, vvd, vtx, out, nullptr);
        }

        // every "model" key that names a studio model, from the entity block
        void collect_entity_models(const format::source_map_data &map,
                                   std::set<std::string> &out)
        {
            for (const format::entity &entity : format::parse_entities(map.entities))
            {
                std::string value = entity.value("model");
                if (value.size() > 4
                    && iequals(value.substr(value.size() - 4), ".mdl"))
                    out.insert(to_lower(normalize(value)));
            }
        }

        // the sprp payload: a model name dictionary, a leaf table, then the
        // placements. the per prop record grew across versions, but origin,
        // angles, model index and skin never moved, so the stride is derived
        // from the payload size instead of hard coding every revision.
        void read_static_props(const format::source_map_data &map,
                               std::vector<std::string> &names,
                               std::vector<prop_placement> &props)
        {
            const std::vector<byte> &data = map.static_props;
            if (data.size() < 12)
                return;
            binary::reader reader(data);

            std::int32_t dict = 0;
            if (!reader.i32_at(0, dict) || dict < 0 || dict > 65536)
                return;
            std::size_t at = 4 + (std::size_t)dict * 128;
            if (at + 4 > data.size())
                return;
            for (int i = 0; i < dict; i++)
            {
                std::size_t name_at = 4 + (std::size_t)i * 128;
                std::size_t end = name_at;
                while (end < name_at + 128 && end < data.size() && data[end] != 0)
                    end++;
                names.push_back(to_lower(normalize(
                    std::string((const char *)data.data() + name_at, end - name_at))));
            }

            std::int32_t leaves = 0;
            if (!reader.i32_at(at, leaves) || leaves < 0)
                return;
            at += 4 + (std::size_t)leaves * 2;
            std::int32_t count = 0;
            if (at + 4 > data.size() || !reader.i32_at(at, count) || count <= 0)
                return;
            at += 4;

            std::size_t stride = (data.size() - at) / (std::size_t)count;
            if (stride < 26)
                return;
            for (int i = 0; i < count; i++)
            {
                std::size_t record = at + (std::size_t)i * stride;
                prop_placement prop;
                float value = 0;
                for (int k = 0; k < 3; k++)
                {
                    reader.f32_at(record + (std::size_t)k * 4, value);
                    prop.origin[k] = value;
                    reader.f32_at(record + 12 + (std::size_t)k * 4, value);
                    prop.angles[k] = value;
                }
                std::uint16_t type = 0;
                reader.u16_at(record + 24, type);
                if (stride >= 36)
                {
                    std::int32_t skin = 0;
                    reader.i32_at(record + 32, skin);
                    prop.skin = skin;
                }
                if (type < names.size())
                    prop.model = names[type];
                props.push_back(prop);
            }
        }
    }

    bool convert_source_model(const std::string &source_mdl,
                              const std::vector<std::string> &game_dirs,
                              source_model_conversion &out,
                              std::string *error)
    {
        out = source_model_conversion{};
        std::string base = fs::strip_extension(source_mdl);

        std::vector<byte> mdl, vvd, vtx;
        if (!fs::read_all(source_mdl, mdl))
        {
            if (error)
                *error = "could not read Source MDL '" + source_mdl + "'";
            return false;
        }
        std::string vvd_path = base + ".vvd";
        if (!fs::read_all(vvd_path, vvd))
        {
            if (error)
                *error = "could not read sibling VVD '" + vvd_path + "'";
            return false;
        }

        std::string vtx_path;
        const char *const vtx_suffixes[] = {".dx90.vtx", ".dx80.vtx", ".vtx"};
        for (const char *suffix : vtx_suffixes)
        {
            std::string candidate = base + suffix;
            if (fs::read_all(candidate, vtx))
            {
                vtx_path = std::move(candidate);
                break;
            }
        }
        if (vtx_path.empty())
        {
            if (error)
                *error = "could not read a sibling .dx90.vtx, .dx80.vtx, or .vtx "
                         "for '" + source_mdl + "'";
            return false;
        }

        content_source content;
        content.game_dirs = game_dirs;
        open_vpks(content);
        return convert_loaded(content, mdl, vvd, vtx, out, error);
    }

    void convert_source_models(const format::source_map_data &map,
                               const std::vector<std::string> &game_dirs, model_result &out)
    {
        out = model_result{};

        content_source content;
        content.pak.open(map.pakfile);
        content.game_dirs = game_dirs;
        open_vpks(content);

        std::vector<std::string> prop_names;
        read_static_props(map, prop_names, out.props);

        std::set<std::string> wanted(prop_names.begin(), prop_names.end());
        collect_entity_models(map, wanted);

        for (const std::string &reference : wanted)
        {
            if (reference.empty())
                continue;
            source_model_conversion conversion;
            if (!convert_one(content, reference, conversion))
            {
                out.failed++;
                continue;
            }
            converted_model model;
            model.path = reference;
            model.data = std::move(conversion.data);
            out.models.push_back(std::move(model));
            out.converted++;
            out.missing_skins += conversion.missing_skins;
        }
    }
}
