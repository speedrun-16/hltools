#pragma once

#include "common/types.h"

// on disk source engine (vbsp) wire structures. this is a different container
// from the goldsrc bsp v30 handled by format/bsp: a "VBSP" magic, 64 versioned
// lumps, optional per lump lzma compression, and crucially the *original*
// brushes and brushsides are still present, so geometry can be read directly
// instead of reconstructed from the tree.
//
// the historical valve field names and fixed c types are kept exactly, since
// this is the binary layout the engine reads. our own code above (source_map_data
// and the porter) is snake_case. all integers are little endian, native on the
// target, so no byte swapping is needed here.

namespace format
{
    constexpr unsigned vbsp_ident = 0x50534256; // 'VBSP' little endian
    constexpr int vbsp_min_version = 19;
    constexpr int vbsp_max_version = 21;

    // marks the start of an lzma compressed lump: 'LZMA' little endian. the
    // directory fourCC is unreliable across games, so compression is detected
    // from the lump payload itself.
    constexpr unsigned vbsp_lzma_ident = 0x414D5A4C;

    constexpr int vbsp_header_lumps = 64;

    enum vbsp_lump_id
    {
        vlump_entities = 0,
        vlump_planes = 1,
        vlump_texdata = 2,
        vlump_vertexes = 3,
        vlump_visibility = 4,
        vlump_nodes = 5,
        vlump_texinfo = 6,
        vlump_faces = 7,
        vlump_lighting = 8,
        vlump_leafs = 10,
        vlump_edges = 12,
        vlump_surfedges = 13,
        vlump_models = 14,
        vlump_leaffaces = 16,
        vlump_leafbrushes = 17,
        vlump_brushes = 18,
        vlump_brushsides = 19,
        vlump_dispinfo = 26,
        vlump_disp_verts = 33,
        vlump_game_lump = 35,
        vlump_pakfile = 40,
        vlump_texdata_string_data = 43,
        vlump_texdata_string_table = 44,
    };

    // texinfo/surface flags relevant to a goldsrc port (subset of surfflags.h)
    enum vbsp_surf_flags
    {
        vsurf_light = 0x0001,
        vsurf_sky2d = 0x0002,
        vsurf_sky = 0x0004,
        vsurf_nodraw = 0x0080,
        vsurf_hint = 0x0100,
        vsurf_skip = 0x0200,
        vsurf_trans = 0x0010,
        vsurf_hitbox = 0x8000,
    };

    struct vbsp_lump_t
    {
        int fileofs;
        int filelen;
        int version;
        int fourcc; // uncompressed size when compressed, else 0
    };

    struct vbsp_header_t
    {
        int ident;
        int version;
        vbsp_lump_t lumps[vbsp_header_lumps];
        int map_revision;
    };

    struct source_dplane_t
    {
        float normal[3];
        float dist;
        int type;
    };

    struct source_dvertex_t
    {
        float point[3];
    };

    struct source_dedge_t
    {
        unsigned short v[2];
    };

    // texture axes are stored directly (unlike goldsrc which folds them into a
    // shared texinfo). texture_vecs map world space to texel space; lightmap_vecs
    // are lighting only and unused by the goldsrc port.
    struct source_texinfo_t
    {
        float texture_vecs[2][4];
        float lightmap_vecs[2][4];
        int flags;
        int texdata;
    };

    struct source_dtexdata_t
    {
        float reflectivity[3];
        int name_string_table_id;
        int width, height;
        int view_width, view_height;
    };

    struct source_dmodel_t
    {
        float mins[3], maxs[3];
        float origin[3];
        int headnode;
        int firstface, numfaces;
    };

    struct source_dbrush_t
    {
        int firstside;
        int numsides;
        int contents;
    };

    struct source_dbrushside_t
    {
        unsigned short planenum;
        short texinfo;
        short dispinfo; // >= 0 marks a displacement surface
        byte bevel;     // non zero for a bevel plane, not a real face
        byte thin;      // v21 thin-brush flag; independent from bevel
    };

    struct source_dnode_t
    {
        int planenum;
        int children[2]; // >= 0 node index, < 0 is -(leaf + 1)
        short mins[3];
        short maxs[3];
        unsigned short firstface;
        unsigned short numfaces;
        short area;
        short padding;
    };

    struct source_dface_t
    {
        unsigned short planenum;
        byte side;
        byte on_node;
        int firstedge;
        short numedges;
        short texinfo;
        short dispinfo;
        short surface_fog_volume_id;
        byte styles[4];
        int lightofs;
        float area;
        int lightmap_mins[2];
        int lightmap_size[2];
        int orig_face;
        unsigned short num_prims;
        unsigned short first_prim_id;
        unsigned int smoothing_groups;
    };

    // Source stores each displacement as a regular grid projected over a quad.
    // The neighbor blocks are kept opaque here: geometry reconstruction only
    // needs the start corner, grid power, parent face and vertex range.
    struct source_ddispinfo_t
    {
        float start_position[3];
        int disp_vert_start;
        int disp_tri_start;
        int power;
        int min_tess;
        float smoothing_angle;
        int contents;
        unsigned short map_face;
        unsigned short padding;
        int lightmap_alpha_start;
        int lightmap_sample_position_start;
        byte edge_neighbors[48];
        byte corner_neighbors[40];
        unsigned int allowed_verts[10];
    };

    struct source_ddispvert_t
    {
        float vector[3];
        float dist;
        float alpha;
    };

    static_assert(sizeof(vbsp_lump_t) == 16, "vbsp_lump_t layout");
    static_assert(sizeof(vbsp_header_t) == 1036, "vbsp_header_t layout");
    static_assert(sizeof(source_dplane_t) == 20, "source_dplane_t layout");
    static_assert(sizeof(source_dvertex_t) == 12, "source_dvertex_t layout");
    static_assert(sizeof(source_dedge_t) == 4, "source_dedge_t layout");
    static_assert(sizeof(source_texinfo_t) == 72, "source_texinfo_t layout");
    static_assert(sizeof(source_dtexdata_t) == 32, "source_dtexdata_t layout");
    static_assert(sizeof(source_dmodel_t) == 48, "source_dmodel_t layout");
    static_assert(sizeof(source_dnode_t) == 32, "source_dnode_t layout");
    static_assert(sizeof(source_dbrush_t) == 12, "source_dbrush_t layout");
    static_assert(sizeof(source_dbrushside_t) == 8, "source_dbrushside_t layout");
    static_assert(sizeof(source_dface_t) == 56, "source_dface_t layout");
    static_assert(sizeof(source_ddispinfo_t) == 176, "source_ddispinfo_t layout");
    static_assert(sizeof(source_ddispvert_t) == 20, "source_ddispvert_t layout");
}
