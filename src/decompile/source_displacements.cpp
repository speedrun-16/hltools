#include "source_displacements.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "format/vbsp/data.h"

namespace decompile
{
    namespace
    {
        using vec3 = math::vec3<double>;

        bool face_points(const format::source_map_data &map,
                         const format::source_dface_t &face,
                         std::vector<vec3> &points)
        {
            if (face.numedges != 4 || face.firstedge < 0)
                return false;
            std::size_t first = (std::size_t)face.firstedge;
            if (first > map.surfedges.size()
                || (std::size_t)face.numedges > map.surfedges.size() - first)
                return false;

            points.clear();
            points.reserve(4);
            for (int i = 0; i < face.numedges; i++)
            {
                int surfedge = map.surfedges[first + (std::size_t)i];
                if (surfedge == std::numeric_limits<int>::min())
                    return false;
                std::size_t edge_index =
                    (std::size_t)(surfedge < 0 ? -surfedge : surfedge);
                if (edge_index >= map.edges.size())
                    return false;
                const format::source_dedge_t &edge = map.edges[edge_index];
                unsigned short vertex = edge.v[surfedge < 0 ? 1 : 0];
                if ((std::size_t)vertex >= map.vertexes.size())
                    return false;
                const float *p = map.vertexes[(std::size_t)vertex].point;
                points.push_back({p[0], p[1], p[2]});
            }
            return true;
        }

        int closest_corner(const std::vector<vec3> &points, const float start[3])
        {
            vec3 wanted{start[0], start[1], start[2]};
            int best = 0;
            double best_distance = std::numeric_limits<double>::max();
            for (int i = 0; i < 4; i++)
            {
                vec3 delta = points[(std::size_t)i] - wanted;
                double distance = math::dot(delta, delta);
                if (distance < best_distance)
                {
                    best = i;
                    best_distance = distance;
                }
            }
            return best;
        }

        bool face_plane(const format::source_map_data &map,
                        const format::source_dface_t &face,
                        vec3 &normal, double &dist)
        {
            if ((std::size_t)face.planenum >= map.planes.size())
                return false;
            const format::source_dplane_t &plane =
                map.planes[(std::size_t)face.planenum];
            double sign = face.side ? -1.0 : 1.0;
            normal = {plane.normal[0] * sign, plane.normal[1] * sign,
                      plane.normal[2] * sign};
            dist = plane.dist * sign;
            return true;
        }

        int displacement_face(const format::source_map_data &map,
                              std::size_t displacement_index,
                              const format::source_ddispinfo_t &info)
        {
            if ((std::size_t)info.map_face < map.faces.size()
                && map.faces[(std::size_t)info.map_face].dispinfo
                    == (short)displacement_index)
                return (int)info.map_face;
            for (std::size_t i = 0; i < map.faces.size(); i++)
                if (map.faces[i].dispinfo == (short)displacement_index)
                    return (int)i;
            return -1;
        }

        void add_triangle(source_displacement_mesh &mesh,
                          std::vector<source_displacement_triangle> &output,
                          vec3 a, vec3 b, vec3 c,
                          const vec3 &base_normal, double minimum_projection,
                          int texinfo)
        {
            vec3 normal = math::cross(b - a, c - a);
            if (math::normalize(normal) == 0)
            {
                mesh.skipped_triangles++;
                return;
            }
            if (math::dot(normal, base_normal) < 0)
                std::swap(b, c);

            source_displacement_triangle triangle;
            triangle.points[0] = a;
            triangle.points[1] = b;
            triangle.points[2] = c;
            triangle.base_normal = base_normal;
            triangle.minimum_projection = minimum_projection;
            triangle.texinfo = texinfo;
            output.push_back(triangle);
        }
    }

    source_displacement_mesh build_source_displacement_mesh(
        const format::source_map_data &map)
    {
        source_displacement_mesh mesh;
        std::vector<vec3> corners;

        for (std::size_t displacement_index = 0;
             displacement_index < map.dispinfo.size(); displacement_index++)
        {
            const format::source_ddispinfo_t &info =
                map.dispinfo[displacement_index];
            int face_index = displacement_face(map, displacement_index, info);
            if (face_index < 0 || info.power < 1 || info.power > 4
                || info.disp_vert_start < 0)
            {
                mesh.skipped_faces++;
                continue;
            }

            const format::source_dface_t &face =
                map.faces[(std::size_t)face_index];
            if (!face_points(map, face, corners))
            {
                mesh.skipped_faces++;
                continue;
            }

            int start = closest_corner(corners, info.start_position);
            std::rotate(corners.begin(), corners.begin() + start, corners.end());

            int side = (1 << info.power) + 1;
            std::size_t vertex_count = (std::size_t)side * (std::size_t)side;
            std::size_t first = (std::size_t)info.disp_vert_start;
            if (first > map.disp_verts.size()
                || vertex_count > map.disp_verts.size() - first)
            {
                mesh.skipped_faces++;
                continue;
            }

            vec3 normal;
            double base_dist = 0;
            if (!face_plane(map, face, normal, base_dist)
                || math::normalize(normal) == 0)
            {
                mesh.skipped_faces++;
                continue;
            }

            std::vector<vec3> grid(vertex_count);
            double minimum_projection = std::numeric_limits<double>::max();
            double interval = 1.0 / (double)(side - 1);
            vec3 edge0 = (corners[1] - corners[0]) * interval;
            vec3 edge1 = (corners[2] - corners[3]) * interval;
            for (int row = 0; row < side; row++)
            {
                vec3 end0 = corners[0] + edge0 * (double)row;
                vec3 end1 = corners[3] + edge1 * (double)row;
                vec3 across = (end1 - end0) * interval;
                for (int column = 0; column < side; column++)
                {
                    std::size_t index =
                        (std::size_t)row * (std::size_t)side
                        + (std::size_t)column;
                    const format::source_ddispvert_t &disp =
                        map.disp_verts[first + index];
                    vec3 direction{
                        disp.vector[0], disp.vector[1], disp.vector[2]};
                    grid[index] = end0 + across * (double)column
                        + direction * (double)disp.dist;
                    minimum_projection = std::min(
                        minimum_projection, math::dot(normal, grid[index]));
                }
            }

            auto append_grid =
                [&](int step,
                    std::vector<source_displacement_triangle> &output)
            {
                for (int row = 0; row < side - 1; row += step)
                {
                    for (int column = 0; column < side - 1; column += step)
                    {
                        std::size_t a = (std::size_t)row * side + column;
                        std::size_t b = a + (std::size_t)step;
                        std::size_t d =
                            a + (std::size_t)step * (std::size_t)side;
                        std::size_t c = d + (std::size_t)step;
                        if ((((row / step) + (column / step)) & 1) == 0)
                        {
                            add_triangle(mesh, output, grid[a], grid[b], grid[c],
                                         normal, minimum_projection, face.texinfo);
                            add_triangle(mesh, output, grid[a], grid[c], grid[d],
                                         normal, minimum_projection, face.texinfo);
                        }
                        else
                        {
                            add_triangle(mesh, output, grid[a], grid[b], grid[d],
                                         normal, minimum_projection, face.texinfo);
                            add_triangle(mesh, output, grid[b], grid[c], grid[d],
                                         normal, minimum_projection, face.texinfo);
                        }
                    }
                }
            };

            append_grid(1, mesh.triangles);
            int collision_power = std::min(info.power, 2);
            int collision_step = 1 << (info.power - collision_power);
            append_grid(collision_step, mesh.collision_triangles);
            mesh.faces++;
        }
        return mesh;
    }
}
