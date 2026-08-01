#include "models.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <filesystem>
#include <set>
#include <system_error>

#include "common/binary.h"
#include "common/filesystem.h"
#include "common/progress.h"
#include "format/bsp/entity_lump.h"
#include "format/image/quantize.h"
#include "format/mdl/chunk_skins.h"
#include "format/mdl/goldsrc/model.h"
#include "format/mdl/source/model.h"
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
        constexpr unsigned goldsrc_max_skin_dim = 512;
        // one texel gives bilinear filtering a neighbor across tile boundaries.
        constexpr unsigned tile_border = 1;

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

        unsigned clamp_skin_dim(unsigned cap)
        {
            if (cap < 16)
                return 16;
            if (cap > goldsrc_max_skin_dim)
                return goldsrc_max_skin_dim;
            return cap - (cap % 16);
        }

        unsigned fit_dim(unsigned d, unsigned cap)
        {
            if (d < 16)
                return 16;
            if (d > cap)
                return cap;
            return d - (d % 16);
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

        // decoded once before tiling.
        struct material_source
        {
            bool ok = false;
            int flags = 0;
            unsigned width = 0, height = 0;
            std::vector<byte> rgb, alpha;
        };

        // the model's material names are relative to its cdmaterials list, so
        // every directory is tried until one resolves
        bool load_material_source(const content_source &content,
                                  const format::studio_model &model,
                                  const std::string &material, unsigned decode_cap,
                                  material_source &out)
        {
            out = material_source{};

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

                // a patch material overrides individual properties of a base
                // material, so a property missing from the patch is looked up in
                // the base, and the shader is always the base's
                format::vmt_material base;
                if (vmt.is_patch())
                {
                    std::vector<byte> base_bytes;
                    std::string include = vmt.patch_include();
                    if (!include.empty() && content.load(include, base_bytes))
                        format::parse_vmt(base_bytes, base);
                }
                auto lookup = [&](const char *property)
                {
                    std::string value = vmt.get(property);
                    return value.empty() ? base.get(property) : value;
                };
                const std::string &shader = vmt.is_patch() ? base.shader : vmt.shader;

                basetexture = normalize(lookup("$basetexture"));
                // an alpha cut material becomes a masked skin, the model
                // equivalent of a '{' world texture
                if (lookup("$alphatest") == "1")
                    out.flags |= format::studio_nf_masked;
                // match unlit rendering by bypassing both light sampling and
                // per-vertex shading.
                if (iequals(shader, "UnlitGeneric"))
                    out.flags |= format::studio_nf_fullbright
                               | format::studio_nf_flatshade;
                // source draws blending from the material, goldsrc from the skin
                if (lookup("$additive") == "1")
                    out.flags |= format::studio_nf_additive;
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
            if (!format::decode_vtf(vtf_bytes, decode_cap, image) || image.width == 0)
                return false;

            out.width = image.width;
            out.height = image.height;
            out.rgb = std::move(image.rgb);
            out.alpha = std::move(image.alpha);
            out.ok = true;
            return true;
        }

        // area averages an arbitrary source rectangle into a dim x dim image.
        // the rectangle may reach outside the image, where the edge pixel is
        // repeated: that is what gives a tile its border, so bilinear filtering
        // at a tile edge has real texels to read instead of the next tile's.
        void extract_region(const material_source &src, double u0, double u1,
                            double v0, double v1, unsigned dst_w, unsigned dst_h,
                            std::vector<byte> &out_rgb, std::vector<byte> &out_alpha)
        {
            out_rgb.assign((std::size_t)dst_w * dst_h * 3, 0);
            bool has_alpha = !src.alpha.empty();
            if (has_alpha)
                out_alpha.assign((std::size_t)dst_w * dst_h, 255);
            if (!src.width || !src.height || !dst_w || !dst_h)
                return;

            for (unsigned y = 0; y < dst_h; y++)
            {
                double sy0 = v0 + (v1 - v0) * y / dst_h;
                double sy1 = v0 + (v1 - v0) * (y + 1) / dst_h;
                int iy0 = (int)std::floor(sy0), iy1 = (int)std::ceil(sy1);
                if (iy1 <= iy0) iy1 = iy0 + 1;
                for (unsigned x = 0; x < dst_w; x++)
                {
                    double sx0 = u0 + (u1 - u0) * x / dst_w;
                    double sx1 = u0 + (u1 - u0) * (x + 1) / dst_w;
                    int ix0 = (int)std::floor(sx0), ix1 = (int)std::ceil(sx1);
                    if (ix1 <= ix0) ix1 = ix0 + 1;

                    unsigned long long r = 0, g = 0, b = 0, a = 0, n = 0;
                    for (int sy = iy0; sy < iy1; sy++)
                    {
                        int cy = sy < 0 ? 0
                            : (sy >= (int)src.height ? (int)src.height - 1 : sy);
                        for (int sx = ix0; sx < ix1; sx++)
                        {
                            int cx = sx < 0 ? 0
                                : (sx >= (int)src.width ? (int)src.width - 1 : sx);
                            std::size_t p = (std::size_t)cy * src.width + cx;
                            r += src.rgb[p * 3];
                            g += src.rgb[p * 3 + 1];
                            b += src.rgb[p * 3 + 2];
                            if (has_alpha)
                                a += src.alpha[p];
                            n++;
                        }
                    }
                    std::size_t o = (std::size_t)y * dst_w + x;
                    out_rgb[o * 3] = (byte)(r / n);
                    out_rgb[o * 3 + 1] = (byte)(g / n);
                    out_rgb[o * 3 + 2] = (byte)(b / n);
                    if (has_alpha)
                        out_alpha[o] = (byte)(a / n);
                }
            }
        }

        // divide each axis independently and reserve a bilinear border.
        format::tile_layout layout_for(const material_source &source,
                                       unsigned max_skin, int chunk_level)
        {
            format::tile_layout layout;
            if (!source.ok || chunk_level < 2 || max_skin <= 2 * tile_border)
                return layout;

            // never cut finer than the caller asked for
            unsigned budget = max_skin;
            for (int level = 1; level < chunk_level; level *= 2)
                budget *= 2;

            // keep a full max_skin content span; the border is resampled into it.
            auto divide_axis = [&](unsigned native, int &count, float &core,
                                   float &inset)
            {
                native = std::min(native, budget);
                count = (int)((native + max_skin - 1) / max_skin);
                if (count < 2)
                    return;
                // each ordinary tile covers max_skin source texels. the final
                // tile is pulled back to the image edge and overlaps the prior
                // one instead of shrinking or resampling a partial crop.
                core = std::min(1.0f, (float)max_skin / (float)native);
                inset = (float)tile_border / (float)max_skin;
            };
            divide_axis(source.width, layout.count_u, layout.core_u, layout.inset_u);
            divide_axis(source.height, layout.count_v, layout.core_v, layout.inset_v);
            return layout;
        }

        // builds one goldsrc skin from a tile of a decoded material
        void build_chunk_texture(const material_source &source,
                                 const format::skin_chunk &chunk,
                                 const std::string &name, unsigned max_skin,
                                 unsigned border,
                                 const std::array<std::array<byte, 3>, 256> *shared,
                                 format::studio_texture &out)
        {
            out.name = to_lower(name) + ".bmp";
            out.flags = source.flags;
            bool tiled = chunk.count_u > 1 || chunk.count_v > 1;
            if (tiled)
            {
                // tile mipmaps bleed across unrelated edges; keep mip 0 only.
                out.flags |= format::studio_nf_nomips;
            }

            std::vector<byte> rgb, alpha;
            unsigned w, h;
            if (!tiled)
            {
                // untiled: the whole image, each axis fitted on its own
                w = fit_dim(source.width, max_skin);
                h = fit_dim(source.height, max_skin);
                extract_region(source, 0, source.width, 0, source.height, w, h,
                               rgb, alpha);
            }
            else
            {
                // the content region in source texels plus one neighbouring
                // texel on each tiled side. that narrow crop is filtered into
                // the legal skin dimensions.
                w = chunk.count_u > 1 ? max_skin : fit_dim(source.width, max_skin);
                h = chunk.count_v > 1 ? max_skin : fit_dim(source.height, max_skin);
                double su0 = chunk.count_u > 1
                    ? (double)chunk.origin_u * source.width : 0.0;
                double sv0 = chunk.count_v > 1
                    ? (double)chunk.origin_v * source.height : 0.0;
                double su1 = chunk.count_u > 1
                    ? su0 + (double)chunk.core_u * source.width : source.width;
                double sv1 = chunk.count_v > 1
                    ? sv0 + (double)chunk.core_v * source.height : source.height;
                double border_u = chunk.count_u > 1 ? border : 0;
                double border_v = chunk.count_v > 1 ? border : 0;
                extract_region(source, su0 - border_u, su1 + border_u,
                               sv0 - border_v, sv1 + border_v, w, h, rgb, alpha);
            }

            bool masked = (out.flags & format::studio_nf_masked) && !alpha.empty();
            if (!masked)
                out.flags &= ~format::studio_nf_masked;

            if (shared != nullptr)
                format::quantize_rgb_fixed(rgb.data(), masked ? alpha.data() : nullptr,
                                           128, w, h, *shared, out.image);
            else if (masked)
                format::quantize_rgb_masked(rgb.data(), alpha.data(), 128, w, h,
                                            out.image);
            else
                format::quantize_rgb(rgb.data(), w, h, out.image);
        }

        // build shared palettes from emitted tiles only.
        bool build_tile_palette(const material_source &source,
                                const std::vector<format::skin_chunk> &chunks,
                                int material, unsigned max_skin, unsigned border,
                                std::array<std::array<byte, 3>, 256> &out)
        {
            std::vector<byte> pooled, pooled_alpha;
            std::vector<byte> rgb, alpha;
            for (const format::skin_chunk &chunk : chunks)
            {
                if (chunk.source_material != material
                    || (chunk.count_u <= 1 && chunk.count_v <= 1))
                    continue;
                unsigned width = chunk.count_u > 1
                    ? max_skin : fit_dim(source.width, max_skin);
                unsigned height = chunk.count_v > 1
                    ? max_skin : fit_dim(source.height, max_skin);
                double su0 = chunk.count_u > 1
                    ? (double)chunk.origin_u * source.width : 0.0;
                double sv0 = chunk.count_v > 1
                    ? (double)chunk.origin_v * source.height : 0.0;
                double su1 = chunk.count_u > 1
                    ? su0 + (double)chunk.core_u * source.width : source.width;
                double sv1 = chunk.count_v > 1
                    ? sv0 + (double)chunk.core_v * source.height : source.height;
                double border_u = chunk.count_u > 1 ? border : 0;
                double border_v = chunk.count_v > 1 ? border : 0;
                extract_region(source, su0 - border_u, su1 + border_u,
                               sv0 - border_v, sv1 + border_v, width, height,
                               rgb, alpha);
                // every 4th pixel is plenty to characterise the colour range and
                // keeps the histogram cheap on a model with dozens of tiles
                for (std::size_t i = 0; i < (std::size_t)width * height; i += 4)
                {
                    pooled.push_back(rgb[i * 3]);
                    pooled.push_back(rgb[i * 3 + 1]);
                    pooled.push_back(rgb[i * 3 + 2]);
                    if (!alpha.empty())
                        pooled_alpha.push_back(alpha[i]);
                }
            }
            if (pooled.empty())
                return false;

            unsigned count = (unsigned)(pooled.size() / 3);
            format::indexed_image sample;
            bool masked = (source.flags & format::studio_nf_masked)
                && pooled_alpha.size() == count;
            bool built = masked
                ? format::quantize_rgb_masked(pooled.data(), pooled_alpha.data(), 128,
                                              count, 1, sample)
                : format::quantize_rgb(pooled.data(), count, 1, sample);
            if (!built)
                return false;
            out = sample.palette;
            return true;
        }

        bool convert_loaded(const content_source &content,
                            const std::vector<byte> &mdl,
                            const std::vector<byte> &vvd,
                            const std::vector<byte> &vtx,
                            unsigned max_skin, int chunk_level, bool shared_palette,
                            float area_keep, source_model_conversion &out,
                            std::string *error)
        {
            out = source_model_conversion{};

            format::studio_model model;
            if (!format::load_source_model(mdl, vvd, vtx, model, error))
                return false;

            // decode every material once, at the best resolution the requested
            // chunk level can actually use
            unsigned decode_cap = max_skin;
            for (int level = 1; level < chunk_level; level *= 2)
                decode_cap *= 2;
            if (decode_cap > 4096)
                decode_cap = 4096;

            std::vector<material_source> sources(model.materials.size());
            for (std::size_t i = 0; i < model.materials.size(); i++)
                load_material_source(content, model, model.materials[i], decode_cap,
                                     sources[i]);

            std::vector<format::tile_layout> layouts(model.materials.size());
            for (std::size_t i = 0; i < sources.size(); i++)
                layouts[i] = layout_for(sources[i], max_skin, chunk_level);

            // cut the geometry along the tile grid before any texture is built,
            // since that is what decides which tiles exist at all
            std::vector<format::skin_chunk> chunks;
            format::chunk_model_skins(model, layouts, area_keep, chunks);
            out.vertices = model.vertices.size();
            out.triangles = model.triangle_count();

            // one palette per tiled material, built from the tiles this model
            // actually uses, so neighbouring tiles agree on colour
            std::vector<std::array<std::array<byte, 3>, 256>> palettes(sources.size());
            std::vector<bool> has_palette(sources.size(), false);
            for (std::size_t i = 0; i < sources.size(); i++)
                if (shared_palette && sources[i].ok
                    && (layouts[i].count_u > 1 || layouts[i].count_v > 1))
                    has_palette[i] = build_tile_palette(sources[i], chunks, (int)i,
                                                        max_skin, tile_border,
                                                        palettes[i]);

            for (std::size_t i = 0; i < chunks.size(); i++)
            {
                const format::skin_chunk &chunk = chunks[i];
                std::size_t material = (std::size_t)chunk.source_material;
                const material_source &source = sources[material];
                std::string name = i < model.materials.size()
                    ? model.materials[i] : std::string("skin");
                format::studio_texture texture;
                if (source.ok)
                {
                    const std::array<std::array<byte, 3>, 256> *shared =
                        ((chunk.count_u > 1 || chunk.count_v > 1)
                         && has_palette[material])
                            ? &palettes[material] : nullptr;
                    build_chunk_texture(source, chunk, name, max_skin, tile_border,
                                        shared, texture);
                    if (chunk.count_u > 1 || chunk.count_v > 1)
                        out.chunked_skins++;
                }
                else
                {
                    // a flat placeholder keeps the model loadable and makes the
                    // missing material obvious rather than silently black
                    texture.name = to_lower(name) + ".bmp";
                    texture.image.width = 16;
                    texture.image.height = 16;
                    texture.image.pixels.assign(16 * 16, 0);
                    texture.image.palette[0] = {255, 0, 220};
                    out.missing_skins++;
                }
                if (texture.flags & format::studio_nf_fullbright)
                    out.fullbright_skins++;
                if (texture.flags & format::studio_nf_additive)
                    out.additive_skins++;
                model.textures.push_back(std::move(texture));
            }
            out.textures = model.textures.size();
            return format::write_goldsrc_model(model, out.data, error);
        }

        // "models/props/tree.mdl" -> the three files that make up the model
        bool convert_one(const content_source &content, const std::string &reference,
                         unsigned max_skin, int chunk_level, bool shared_palette,
                         float area_keep,
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

            return convert_loaded(content, mdl, vvd, vtx, max_skin, chunk_level,
                                  shared_palette, area_keep, out,
                                  nullptr);
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
                if (stride >= 31)
                    prop.solid = data[record + 30];
                if (type < names.size())
                    prop.model = names[type];
                props.push_back(prop);
            }
        }
    }

    bool convert_source_model(const std::string &source_mdl,
                              const std::vector<std::string> &game_dirs,
                              unsigned max_skin_size, int chunk_level,
                              bool shared_tile_palette, float tile_area_keep,
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
        return convert_loaded(content, mdl, vvd, vtx, clamp_skin_dim(max_skin_size),
                              chunk_level, shared_tile_palette, tile_area_keep,
                              out, error);
    }

    void convert_source_models(const format::source_map_data &map,
                               const std::vector<std::string> &game_dirs,
                               unsigned max_skin_size, int chunk_level,
                               bool shared_tile_palette, float tile_area_keep,
                               bool load_physics,
                               model_result &out)
    {
        out = model_result{};
        unsigned max_skin = clamp_skin_dim(max_skin_size);
        if (chunk_level < 1)
            chunk_level = 1;

        content_source content;
        content.pak.open(map.pakfile);
        content.game_dirs = game_dirs;
        open_vpks(content);

        std::vector<std::string> prop_names;
        read_static_props(map, prop_names, out.props);

        std::set<std::string> wanted;
        for (const prop_placement &prop : out.props)
            if (!prop.model.empty())
                wanted.insert(prop.model);
        collect_entity_models(map, wanted);

        // rebuilding a model decodes and requantises every texture it uses and
        // can take seconds per texture on a map with dozens of them, so this phase
        // needs a bar like every other long one
        progress::section("models");
        progress::begin("converting models", (int)wanted.size());
        for (const std::string &reference : wanted)
        {
            progress::add(1);
            if (reference.empty())
                continue;

            if (load_physics)
            {
                std::vector<byte> phy_data;
                std::string phy_path = fs::strip_extension(reference) + ".phy";
                if (content.load(phy_path, phy_data))
                {
                    converted_collision collision;
                    collision.model = reference;
                    if (format::load_source_phy(phy_data, collision.physics))
                        out.collisions.push_back(std::move(collision));
                }
            }
            source_model_conversion conversion;
            if (!convert_one(content, reference, max_skin, chunk_level,
                             shared_tile_palette, tile_area_keep, conversion))
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
            out.fullbright_skins += conversion.fullbright_skins;
            out.additive_skins += conversion.additive_skins;
            out.chunked_skins += conversion.chunked_skins;
            out.triangles += conversion.triangles;
        }
        progress::end();
    }
}
