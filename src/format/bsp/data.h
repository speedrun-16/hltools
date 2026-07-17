#pragma once

#include <string>
#include <vector>

#include "types.h"

// the whole compiled map, in memory this replaces the fourteen fixed size
// global g_d* arrays the old tools shared: instead each stage receives a
// map_data by reference, so the tools become reentrant and testable and the
// shared code no longer branches on which tool is compiling it
//
// the typed lumps are vectors of the wire structs; the four variable length
// blobs (lighting, visibility, textures, entities) are raw bytes, since their
// internal structure is owned by the stages that produce them

namespace format
{
    struct map_data
    {
        std::vector<dmodel_t> models;
        std::vector<dplane_t> planes;
        std::vector<dvertex_t> vertexes;
        std::vector<dnode_t> nodes;
        std::vector<texinfo_t> texinfo;
        std::vector<dface_t> faces;
        std::vector<dclipnode_t> clipnodes;
        std::vector<dleaf_t> leafs;
        std::vector<unsigned short> marksurfaces;
        std::vector<dedge_t> edges;
        std::vector<int> surfedges;

        std::vector<byte> lighting;   // lightmap samples
        std::vector<byte> visibility; // compressed pvs
        std::vector<byte> textures;   // miptex lump (dmiptexlump_t + data)
        std::string entities;         // entity text block
    };
}
