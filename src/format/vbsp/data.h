#pragma once

#include <string>
#include <vector>

#include "types.h"

// the decompressed contents of a source engine map, in memory. mirrors the role
// of format::map_data for goldsrc, but holds the source lump set the porter
// needs. typed lumps are vectors of the wire structs; the entity block and the
// pakfile (a zip archive) are raw. material_names is a decoded convenience view
// of the texdata string table, indexed by texdata id.

namespace format
{
    // normalized leaf carrying only what brush->model assignment needs. the on
    // disk leaf differs between lump version 0 (56 bytes, trailing ambient light
    // cube) and version 1+ (32 bytes); the brush fields sit at the same offset in
    // both, so the reader extracts them with a version dependent stride.
    struct source_leaf_t
    {
        unsigned short firstleafbrush;
        unsigned short numleafbrushes;
    };

    struct source_map_data
    {
        int version = 0;
        int map_revision = 0;

        std::vector<source_dplane_t> planes;
        std::vector<source_dvertex_t> vertexes;
        std::vector<source_dedge_t> edges;
        std::vector<int> surfedges;
        std::vector<source_dface_t> faces;
        std::vector<source_texinfo_t> texinfo;
        std::vector<source_dtexdata_t> texdata;
        std::vector<source_dmodel_t> models;
        std::vector<source_dbrush_t> brushes;
        std::vector<source_dbrushside_t> brushsides;
        std::vector<source_ddispinfo_t> dispinfo;
        std::vector<source_ddispvert_t> disp_verts;
        std::vector<source_dnode_t> nodes;
        std::vector<source_leaf_t> leaves;
        std::vector<unsigned short> leafbrushes;

        std::vector<int> string_table;  // byte offsets into string_data
        std::string string_data;        // null terminated material paths
        std::vector<std::string> material_names; // decoded, per texdata id

        std::string entities;    // entity text block
        std::vector<byte> pakfile; // embedded zip archive (vmt/vtf/mdl)

        // payload of the 'sprp' sub lump of the game lump: the map's static
        // props, which are not entities at all. the file reader extracts it
        // because the game lump's offsets are absolute into the bsp file.
        std::vector<byte> static_props;
        int static_prop_version = 0;

        bool has_displacements = false; // any face/brushside references dispinfo

        // resolves the material path for a texinfo index, "" when out of range.
        const std::string &texinfo_material(int texinfo_index) const;
    };
}
