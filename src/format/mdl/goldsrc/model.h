#pragma once

#include <string>
#include <vector>

#include "common/types.h"
#include "format/mdl/studio_model.h"

namespace format
{
    // serializes a studio_model as a goldsrc studio model (IDST version 10).
    //
    // everything lives in the one file: bones, sequences with their run length
    // encoded animation, the mesh triangle command stream, and the palettized
    // skins. that is the whole reason the porter can skip qc and smd entirely,
    // which are only an authoring detour between two binary formats.
    //
    // model.textures must already be filled in; a model with none still writes,
    // but the engine has nothing to draw it with.
    bool write_goldsrc_model(const studio_model &model, std::vector<byte> &out,
                             std::string *error = nullptr);

    // the per submodel vertex ceiling the engine's studio renderer allocates
    // for. geometry above it is split across bodyparts, which are all drawn
    // together (the models *within* one bodypart are alternatives chosen by the
    // entity's body value, so splitting there would hide the other pieces).
    constexpr int goldsrc_max_studio_verts = 2048;
    constexpr int goldsrc_max_studio_bodyparts = 32;
    constexpr int goldsrc_studio_version = 10;
}
