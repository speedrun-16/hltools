#pragma once

// engine and format limits for goldsrc bsp v30 these are hard ceilings the
// game engine imposes; exceeding one produces a map the engine refuses to load

namespace limits
{
    constexpr int max_map_hulls = 4;
    constexpr int max_map_models = 512;
    constexpr int max_map_brushes = 32768;
    constexpr int max_map_entities = 16384;
    constexpr int max_map_entstring = 2048 * 1024;

    constexpr int max_map_planes = 32768;
    constexpr int max_internal_map_planes = 256 * 1024;
    constexpr int max_map_nodes = 32767;
    constexpr int max_map_clipnodes = 32767;
    constexpr int max_map_leafs = 32760;
    constexpr int max_map_leafs_engine = 8192;

    constexpr int max_map_verts = 65535;
    constexpr int max_map_faces = 65535;
    constexpr int max_map_worldfaces = 32768;
    constexpr int max_map_marksurfaces = 65535;

    constexpr int max_map_textures = 4096;
    constexpr int max_map_texinfo = 32767;
    constexpr int max_internal_map_texinfo = 262144;
    constexpr int max_map_edges = 256000;
    constexpr int max_map_surfedges = 512000;
    constexpr int max_map_visibility = 0x800000;

    // default ceilings for the variable byte lumps (the tools can raise these)
    constexpr int max_map_miptex = 0x2000000;   // 32 mb texture data
    constexpr int max_map_lightdata = 0x3000000; // 48 mb lightmap data

    // lightmap atlas: the engine packs every lit face into this many 128x128
    // pages overflow aborts the load with "allocblock: full"
    constexpr int max_alloc_block_pages = 64;
    constexpr int block_width = 128;
    constexpr int block_height = 128;

    // one face carries up to this many light styles, each a full lightmap
    constexpr int max_lightmaps = 4;

    // lightmap luxel grid: one sample every 16 world units extents past 16
    // luxels break the software renderer and dedicated server
    constexpr int texture_step = 16;
    constexpr int max_surface_extent = 16;
}
