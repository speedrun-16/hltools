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
    // one converted skybox face, ready to be written under gfx/env.
    struct skybox_face
    {
        std::string filename; // "<skyname><suffix>.tga"
        std::vector<byte> tga;
    };

    struct skybox_result
    {
        // value worldspawn's "skyname" should carry in the ported map. source
        // a sky name may sit in a subdirectory, of any depth and name; goldsrc
        // looks only in gfx/env, so the directory part is dropped.
        std::string sky_name;
        std::vector<skybox_face> faces;
        std::size_t missing = 0; // faces whose vmt or vtf could not be resolved
    };

    // converts a source 2d skybox into the six goldsrc gfx/env tga files.
    //
    // both engines name the faces <skyname> + {bk,dn,ft,lf,rt,up}, so the
    // suffixes map straight across; source stores each face as a vmt pointing at
    // a vtf, goldsrc as a raw tga. face_size is the edge length every face is
    // produced at (all stock GoldSrc skies are 256). Larger exports can be fed
    // to `hltools model skybox`, which tiles them into 512-pixel studio skins.
    // exposure linearly scales the decoded RGB values before they are written;
    // 1 preserves the source.
    //
    // returns false only when the map declares no sky at all. individual faces
    // that cannot be resolved are counted in result.missing and omitted, so a
    // partial sky still exports.
    bool export_source_skybox(const format::source_map_data &map,
                              const std::vector<std::string> &game_dirs,
                              const std::string &source_sky_name,
                              unsigned face_size, double exposure,
                              skybox_result &out);
}
