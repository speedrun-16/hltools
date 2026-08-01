#pragma once

#include <cstddef>
#include <vector>

namespace format
{
    class entity;
}

namespace decompile
{
    struct entity_remap_stats
    {
        std::size_t dropped_spawns = 0; // extra team spawns removed
        std::size_t studio_props = 0;   // prop_* retagged as cycler_sprite
        // func_brush split into func_wall / func_illusionary by its solidity
        std::size_t brush_entities = 0;
        bool player_start = false;      // a single info_player_start was placed
    };

    // rewrites source-only entities into goldsrc equivalents in place. counter
    // strike team spawns (info_player_terrorist / info_player_counterterrorist)
    // have no goldsrc analogue, so the first is retagged as a single
    // info_player_start and the rest are dropped. entities that need no change
    // pass through with Source editor bookkeeping such as hammerid removed;
    // order is preserved so brush-model indices stay valid.
    void remap_source_entities(std::vector<format::entity> &entities,
                               entity_remap_stats &stats);
}
