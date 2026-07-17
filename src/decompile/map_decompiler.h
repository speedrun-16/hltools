#pragma once

#include <cstddef>
#include <string>

namespace format
{
    struct map_data;
}

namespace decompile
{
    struct map_options
    {
        // a companion wad name to append to worldspawn's existing wad list
        // empty leaves the entity lump's original list untouched
        std::string additional_wad;
        // safe when the bsp has no external texture references: the companion
        // archive is then self contained and stale editor paths are unnecessary
        bool replace_wad_list = false;
    };

    struct map_result
    {
        std::string text;
        std::size_t brushes = 0;
        std::size_t textured_sides = 0;
        std::size_t generated_sides = 0;
        std::size_t discarded_cells = 0;
        std::size_t texture_splits = 0;
        std::size_t unresolved_texture_boundaries = 0;
    };

    // reconstructs editable convex brushes from each model's hull 0 bsp tree
    // and writes a valve 220 map the bsp has already lost the original brush
    // grouping, so the result represents solid leaf cells instead
    bool reconstruct_map(const format::map_data &map, const map_options &options,
                         map_result &result, std::string *error = nullptr);
}
