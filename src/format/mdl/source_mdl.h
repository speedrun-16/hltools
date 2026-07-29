#pragma once

#include <string>
#include <vector>

#include "common/types.h"
#include "studio_model.h"

namespace format
{
    // reads a source engine studio model into the neutral studio_model.
    //
    // source splits one model across three files and all three are required:
    //   .mdl  header, bones, bodyparts/models/meshes, material names
    //   .vvd  the vertex pool (position, normal, uv, bone weights)
    //   .vtx  the index buffers, per level of detail
    // only lod 0 is read, since goldsrc has no lod concept.
    //
    // returns false with a description when a file is not a studio model, is a
    // version this reader does not cover, or is internally inconsistent.
    bool load_source_model(const std::vector<byte> &mdl, const std::vector<byte> &vvd,
                           const std::vector<byte> &vtx, studio_model &out,
                           std::string *error = nullptr);

    // lowest and highest .mdl versions the reader accepts. 44 is the original
    // source 2006 layout and 49 is source 2013 / csgo; the fields this reader
    // touches sit at the same offsets across that range.
    constexpr int source_mdl_min_version = 44;
    constexpr int source_mdl_max_version = 49;
}
