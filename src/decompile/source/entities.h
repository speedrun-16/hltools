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
        std::size_t dropped_spawns = 0; // team-specific or duplicate starts removed
        std::size_t studio_props = 0;   // prop_* retagged as cycler_sprite
        // func_brush split into func_wall / func_illusionary by its solidity
        std::size_t brush_entities = 0;
        std::size_t fog_controllers = 0; // env_fog_controller -> env_fog
        std::size_t cameras = 0;         // point_viewcontrol -> trigger_camera
        std::size_t lights = 0;          // light* with source keys renamed
        bool player_start = false;       // at least one info_player_start exists
    };

    // rewrites supported entities in place, removes unsupported metadata, and
    // keeps one player start. entity order remains stable for brush models.
    void remap_source_entities(std::vector<format::entity> &entities,
                               entity_remap_stats &stats);
}
