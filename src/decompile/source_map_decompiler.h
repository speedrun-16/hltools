#pragma once

#include <cstddef>
#include <set>
#include <string>
#include <vector>

#include "format/miptex/texture.h"
#include "source_models.h"
#include "source_skybox.h"

namespace format
{
    struct source_map_data;
}

namespace decompile
{
    struct source_options
    {
        // source content root for materials not embedded in the pakfile.
        // reserved for the material pipeline; unused by geometry porting.
        std::vector<std::string> game_dirs;
        // companion wad appended to worldspawn's wad list, as for the goldsrc
        // decompiler. reserved for the material pipeline.
        std::string additional_wad;
        // wad supplying the engine textures (NULL/SKY/AAATRIGGER/...), normally
        // sdhlt.wad. appended to worldspawn's wad list after the companion wad.
        std::string tool_wad;
        // edge length of the exported gfx/env sky faces. every stock goldsrc sky
        // is 256; values through 4096 support the skybox-model workflow; 0
        // disables skybox export.
        unsigned sky_size = 256;
        // linear RGB multiplier applied only to exported gfx/env sky faces.
        double sky_exposure = 1.0;
        // maximum WAD texture dimension. Lower values preserve UV coverage while
        // trading image resolution for fewer BSP subdivisions and lightmap luxels.
        unsigned max_texture_size = 512;
        // Source material names (or generated GoldSrc basenames) that bypass the
        // global cap and retain their native dimensions, up to GoldSrc's 512 limit.
        std::vector<std::string> full_size_textures;
        // rebuild the map's studio models as goldsrc models and place them as
        // entities. off leaves the map brush only.
        bool convert_models = true;
    };

    struct source_result
    {
        std::string text;
        std::size_t entities = 0;
        std::size_t world_brushes = 0;
        std::size_t entity_brushes = 0;
        std::size_t detail_brushes = 0; // world detail routed to func_detail
        std::size_t window_brushes = 0; // translucent brushes routed to func_wall
        std::size_t masked_brushes = 0;  // world brushes carrying a '{' texture
        std::size_t masked_entities = 0; // brush entities retagged rendermode 4
        // brush entities that inherited a translucent material's opacity
        std::size_t translucent_entities = 0;
        std::size_t water_brushes = 0;  // liquid brushes routed to func_water
        std::size_t clip_brushes = 0;   // pure playerclip volumes given CLIP
        // 3d skybox brushes dropped: outside the world, no goldsrc equivalent
        std::size_t skybox_brushes = 0;
        std::size_t sides = 0;
        std::size_t skipped_brushes = 0;
        std::size_t skipped_sides = 0;
        std::size_t realigned_sides = 0; // edge-on texture axes replaced
        std::size_t displacement_faces = 0;
        std::size_t displacement_brushes = 0;
        std::size_t displacement_collision_brushes = 0;
        std::size_t skipped_displacement_faces = 0;
        std::size_t skipped_displacement_triangles = 0;
        std::size_t converted_materials = 0;
        std::size_t placeholder_materials = 0;
        std::size_t masked_materials = 0; // converted with a '{' transparency mask
        // UnlitGeneric materials exported through info_unlittextures so RAD
        // gives their faces constant-white fullbright lightmaps.
        std::size_t unlit_materials = 0;
        // engine textures the map names (NULL/CLIP/SKY/...) that no source
        // material provides. they carry meaning to hlcsg rather than pixels, but
        // still need a real wad entry, so a tool wad like sdhlt.wad must be on
        // the map's wad list for these to resolve.
        std::set<std::string> engine_textures;
        std::size_t dropped_spawns = 0; // extra team spawns removed
        bool player_start = false;      // a single info_player_start was placed

        // companion wad textures built from the map's materials, to be written
        // by the caller alongside the map.
        std::vector<format::mip_texture> wad_textures;

        // gfx/env sky faces converted from the source 2d skybox, to be written
        // by the caller alongside the map. skybox.sky_name is already stored in
        // worldspawn's "skyname".
        skybox_result skybox;

        // studio models rebuilt for goldsrc, to be written by the caller. their
        // placements are already emitted into the map as entities.
        model_result models;
        std::size_t prop_entities = 0; // static props turned back into entities
        // inert brush entities folded together to stay under the model limit
        std::size_t merged_brush_entities = 0;
        std::size_t brush_entity_groups = 0;
        std::size_t brush_models = 0; // brush entities left in the ported map
    };

    // ports a source engine map to an editable valve 220 map. unlike the goldsrc
    // decompiler this reads the original brushes and brushsides directly, so no
    // tree reconstruction is needed. displacement surfaces are tessellated into
    // thin func_detail triangle brushes. texture names are mapped to goldsrc;
    // the companion wad is produced separately by the material pipeline.
    bool port_source_map(const format::source_map_data &map, const source_options &options,
                         source_result &result, std::string *error = nullptr);
}
