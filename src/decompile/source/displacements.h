#pragma once

#include <cstddef>
#include <vector>

#include "math/vector.h"

namespace format
{
    struct source_map_data;
}

namespace decompile
{
    struct source_displacement_triangle
    {
        math::vec3<double> points[3];
        math::vec3<double> base_normal;
        double minimum_projection = 0;
        int texinfo = -1;
    };

    struct source_displacement_mesh
    {
        std::vector<source_displacement_triangle> triangles;
        // Collision is capped at a power-2 grid. Source's full power-3/4
        // tessellation is retained for drawing, but would overflow GoldSrc's
        // shared 16-bit clipnode lump when expanded into the player hulls.
        std::vector<source_displacement_triangle> collision_triangles;
        std::size_t faces = 0;
        std::size_t skipped_faces = 0;
        std::size_t skipped_triangles = 0;
    };

    // Reconstructs Source's regular displacement grids at full tessellation.
    // The returned triangles use outward winding and retain the parent face's
    // texinfo so the map porter can apply the original world-space UV axes.
    source_displacement_mesh build_source_displacement_mesh(
        const format::source_map_data &map);
}
