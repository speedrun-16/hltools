#pragma once

#include "common/types.h"
#include "format/miptex/types.h"

// on disk bsp v30 wire structures this is the binary layout the engine reads,
// so the field order, the fixed c types, and the historical lowercase _t names
// are kept exactly our own code (map_data and up) is snake_case; these are not
// ours to rename all integers are stored little endian, which is native on the
// target, so no byte swapping is needed

namespace format
{
    constexpr int bsp_version = 30;

    enum lump_id
    {
        lump_entities = 0,
        lump_planes,
        lump_textures,
        lump_vertexes,
        lump_visibility,
        lump_nodes,
        lump_texinfo,
        lump_faces,
        lump_lighting,
        lump_clipnodes,
        lump_leafs,
        lump_marksurfaces,
        lump_edges,
        lump_surfedges,
        lump_models,
        header_lumps,
    };

    constexpr int max_map_hulls = 4;
    constexpr int max_lightmaps = 4;
    constexpr int num_ambients = 4;

    struct lump_t
    {
        int fileofs;
        int filelen;
    };

    struct dheader_t
    {
        int version;
        lump_t lumps[header_lumps];
    };

    struct dmodel_t
    {
        float mins[3], maxs[3];
        float origin[3];
        int headnode[max_map_hulls];
        int visleafs; // not counting the solid leaf 0
        int firstface, numfaces;
    };

    struct dvertex_t
    {
        float point[3];
    };

    struct dplane_t
    {
        float normal[3];
        float dist;
        int type; // plane axis classification, trivially regenerated
    };

    struct dnode_t
    {
        int planenum;
        short children[2]; // negative numbers are -(leafs + 1), not nodes
        short mins[3];
        short maxs[3];
        unsigned short firstface;
        unsigned short numfaces; // counting both sides
    };

    struct dclipnode_t
    {
        int planenum;
        short children[2]; // negative numbers are contents
    };

    struct texinfo_t
    {
        float vecs[2][4]; // [s/t][xyz offset]
        int miptex;
        int flags;
    };

    struct dedge_t
    {
        unsigned short v[2]; // vertex numbers
    };

    struct dface_t
    {
        unsigned short planenum;
        short side;
        int firstedge; // supports more than 64k edges
        short numedges;
        short texinfo;
        byte styles[max_lightmaps];
        int lightofs; // start of [numstyles * surfsize] samples, -1 if unlit
    };

    struct dleaf_t
    {
        int contents;
        int visofs; // -1 = no visibility info
        short mins[3];
        short maxs[3];
        unsigned short firstmarksurface;
        unsigned short nummarksurfaces;
        byte ambient_level[num_ambients];
    };

    struct dmiptexlump_t
    {
        int nummiptex;
        int dataofs[4]; // [nummiptex]
    };

    // guard the binary layout: the loader and writer copy whole arrays by size,
    // so any accidental padding change would corrupt every produced bsp
    static_assert(sizeof(lump_t) == 8, "lump_t layout");
    static_assert(sizeof(dheader_t) == 124, "dheader_t layout");
    static_assert(sizeof(dmodel_t) == 64, "dmodel_t layout");
    static_assert(sizeof(dvertex_t) == 12, "dvertex_t layout");
    static_assert(sizeof(dplane_t) == 20, "dplane_t layout");
    static_assert(sizeof(dnode_t) == 24, "dnode_t layout");
    static_assert(sizeof(dclipnode_t) == 8, "dclipnode_t layout");
    static_assert(sizeof(texinfo_t) == 40, "texinfo_t layout");
    static_assert(sizeof(dedge_t) == 4, "dedge_t layout");
    static_assert(sizeof(dface_t) == 20, "dface_t layout");
    static_assert(sizeof(dleaf_t) == 28, "dleaf_t layout");
}
