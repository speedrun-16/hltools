#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "common/types.h"

namespace format
{
    struct source_map_data;
}

namespace decompile
{
    // a converted goldsrc model, to be written by the caller
    struct converted_model
    {
        std::string path; // output relative, e.g. "models/props/tree.mdl"
        std::vector<byte> data;
    };

    // one placement of a model in the map. static props live in a game lump
    // rather than the entity block, so they have to be turned back into
    // entities for goldsrc, which has no such lump.
    struct prop_placement
    {
        std::string model; // output relative path
        double origin[3] = {};
        double angles[3] = {};
        int skin = 0;
    };

    struct model_result
    {
        std::vector<converted_model> models;
        std::vector<prop_placement> props;
        std::size_t converted = 0;      // models successfully rebuilt
        std::size_t failed = 0;         // models that could not be read or written
        std::size_t missing_skins = 0;  // materials with no resolvable image
        std::size_t split_models = 0;   // models that needed extra bodyparts
    };

    // result of converting one loose Source studio model. Source models are a
    // three-file asset: source_mdl must have a sibling .vvd and either a
    // .dx90.vtx, .dx80.vtx, or .vtx file. game_dir supplies loose materials
    // and the game's VPK archives.
    struct source_model_conversion
    {
        std::vector<byte> data;
        std::size_t vertices = 0;
        std::size_t triangles = 0;
        std::size_t textures = 0;
        std::size_t missing_skins = 0;
    };

    bool convert_source_model(const std::string &source_mdl,
                              const std::vector<std::string> &game_dirs,
                              source_model_conversion &out,
                              std::string *error = nullptr);

    // rebuilds every studio model the map references as a goldsrc model.
    //
    // sources are the map's own pakfile first, then the -game content dir and
    // its vpk archives, matching the engine's search order. models come from
    // two places: the static prop game lump, and any entity with a "model" key
    // ending in .mdl. failures are counted rather than fatal, so one broken
    // model never costs the rest of the port.
    void convert_source_models(const format::source_map_data &map,
                               const std::vector<std::string> &game_dirs, model_result &out);
}
