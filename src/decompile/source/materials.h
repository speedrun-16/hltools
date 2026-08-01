#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "format/miptex/texture.h"

namespace format
{
    struct source_map_data;
}

namespace decompile
{
    // the goldsrc mapping of one source material: the texture name used on brush
    // sides, whether a companion wad texture was produced, and the axis scale
    // factors that keep uvs correct after resizing to goldsrc dimensions.
    struct resolved_material
    {
        std::string name = "NULL";
        bool has_texture = false;
        double u_scale = 1.0; // multiply source texture_vecs[0] by this
        double v_scale = 1.0; // multiply source texture_vecs[1] by this
        // $translucent/$alpha material; render_amount is the goldsrc renderamt
        // equivalent derived from the texture's average opacity
        bool translucent = false;
        int render_amount = 255;
        // the material's alpha channel carried a cutout shape, so the goldsrc
        // texture is '{' masked. the owning brush must become an entity with
        // rendermode 4 (solid) and renderamt 255 for the engine to honour it
        bool masked = false;
        // engine-special mapping (NULL/CLIP/AAATRIGGER/...): the name is consumed
        // by the compiler, so uv axes are meaningless and sides should use clean
        // world-aligned axes instead of the source material's
        bool special = false;
        // source Water shader material: the goldsrc name carries the '!' water
        // prefix and the owning brush should become a func_water entity
        bool water = false;
        // Source UnlitGeneric shades the whole material without a lightmap.
        // The map exporter records this separately so RAD can omit lightmaps
        // from every face using the converted GoldSrc texture.
        bool unlit = false;
    };

    // resolves every material referenced by a source map into goldsrc textures.
    // materials come from the embedded pakfile first, then the -game content dir.
    // tool materials map to engine names without a texture; unresolved materials
    // become a named placeholder so the map still loads.
    class material_catalog
    {
    public:
        // builds the catalog. never fails hard: individual materials that cannot
        // be resolved fall back to placeholders and are counted.
        void build(const format::source_map_data &map, const std::vector<std::string> &game_dirs,
                   unsigned max_texture_size = 512,
                   const std::vector<std::string> &full_size_textures = {});

        const resolved_material &resolve(const std::string &material) const;

        const std::vector<format::mip_texture> &textures() const { return textures_; }
        std::size_t converted() const { return converted_; }
        std::size_t placeholders() const { return placeholders_; }
        // converted textures that carry a '{' transparency mask
        std::size_t masked() const { return masked_; }

    private:
        std::string unique_name(const std::string &material, const char *prefix = "");

        std::unordered_map<std::string, resolved_material> by_material_;
        std::unordered_set<std::string> used_names_;
        std::vector<format::mip_texture> textures_;
        resolved_material fallback_;
        std::size_t converted_ = 0;
        std::size_t placeholders_ = 0;
        std::size_t masked_ = 0;
    };
}
