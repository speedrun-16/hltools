#include "brush.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <utility>

#include "../common/error.h"
#include "../common/limits.h"
#include "../common/log.h"
#include "../common/string_util.h"
#include "textures.h"

namespace csg
{
    namespace
    {
        constexpr math::vec3v base_axis[18] = {
            {0, 0, 1}, {1, 0, 0}, {0, -1, 0},  // floor
            {0, 0, -1}, {1, 0, 0}, {0, -1, 0}, // ceiling
            {1, 0, 0}, {0, 1, 0}, {0, 0, -1},  // west wall
            {-1, 0, 0}, {0, 1, 0}, {0, 0, -1}, // east wall
            {0, 1, 0}, {1, 0, 0}, {0, 0, -1},  // south wall
            {0, -1, 0}, {1, 0, 0}, {0, 0, -1}, // north wall
        };

        constexpr vec_t dist_epsilon = 0.04;
        constexpr vec_t floor_z = (vec_t)0.7;

        struct axial_bevel_flags
        {
            bool skip_expansion[3][2] = {};
        };

        const brush_side &side_at(const map_source &map, const map_brush &brush, int index)
        {
            return map.sides[(size_t)brush.first_side + index];
        }

        brush_side &side_at(map_source &map, const map_brush &brush, int index)
        {
            return map.sides[(size_t)brush.first_side + index];
        }

        int plane_axis(math::plane_type type)
        {
            return (int)type % 3;
        }

        bool is_axial(math::plane_type type)
        {
            return type == math::plane_type::x
                || type == math::plane_type::y
                || type == math::plane_type::z;
        }

        int axial_count(const math::vec3v &normal)
        {
            return (std::fabs(normal.x) < math::normal_epsilon)
                + (std::fabs(normal.y) < math::normal_epsilon)
                + (std::fabs(normal.z) < math::normal_epsilon);
        }

        void sort_sides(std::vector<brush_face> &faces, const plane_store &planes)
        {
            std::vector<brush_face> sorted;
            std::vector<bool> used(faces.size(), false);
            sorted.reserve(faces.size());

            for (size_t i = 0; i < faces.size(); i++)
            {
                int best_side = -1;
                int best_axial = -1;
                for (size_t j = 0; j < faces.size(); j++)
                {
                    if (used[j])
                        continue;
                    int axial = axial_count(planes.planes()[(size_t)faces[j].planenum].normal);
                    if (axial > best_axial)
                    {
                        best_side = (int)j;
                        best_axial = axial;
                    }
                }
                sorted.push_back(faces[(size_t)best_side]);
                used[(size_t)best_side] = true;
            }

            faces = std::move(sorted);
        }

        void add_hull_plane(std::vector<brush_face> &faces,
                            plane_store &planes,
                            const math::vec3v &normal,
                            const math::vec3v &origin,
                            bool check_planenum)
        {
            int planenum = planes.find_int_plane(normal, origin);
            if (check_planenum)
            {
                for (const brush_face &face : faces)
                {
                    if (face.planenum == planenum)
                        return;
                }
            }

            brush_face face;
            face.planenum = planenum;
            face.texinfo = -1;
            face.contents = content::empty;
            faces.insert(faces.begin(), face);
        }

        axial_bevel_flags collect_axial_bevels(const brush_hull &hull0, const plane_store &planes)
        {
            axial_bevel_flags flags;
            for (const brush_face &face : hull0.faces)
            {
                const brush_plane &plane = planes.planes()[(size_t)face.planenum];
                if (!is_axial(plane.type) || !face.bevel)
                    continue;
                int axis = plane_axis(plane.type);
                flags.skip_expansion[axis][plane.normal[axis] > 0 ? 1 : 0] = true;
            }
            return flags;
        }

        math::vec3v offset_clip_origin(const math::vec3v &origin,
                                       const math::vec3v &normal,
                                       int hull_num,
                                       const brush_build_options &options,
                                       bool bevel_face)
        {
            // default simple moves by the full hull extents per axis
            // matching the reference fix for sticky seams on sloped clip hulls
            if (bevel_face)
                return origin;

            math::vec3v shifted = origin;
            if (options.clip == clip_type::precise && normal.z > floor_z)
            {
                shifted.z += options.hull_size[hull_num][1].z;
                return shifted;
            }
            if (options.clip == clip_type::legacy || options.clip == clip_type::normalized)
            {
                for (int i = 0; i < 3; i++)
                {
                    if (normal[i])
                        shifted[i] += normal[i] * (normal[i] > 0
                            ? options.hull_size[hull_num][1][i]
                            : -options.hull_size[hull_num][0][i]);
                }
                return shifted;
            }

            for (int i = 0; i < 3; i++)
                shifted[i] += options.hull_size[hull_num][normal[i] > 0 ? 1 : 0][i];
            return shifted;
        }

        const brush_face *find_edge_neighbor(const brush_hull &hull0,
                                             const brush_face &current,
                                             const math::vec3v &edge_start,
                                             const math::vec3v &edge_end)
        {
            for (const brush_face &other : hull0.faces)
            {
                if (&other == &current)
                    continue;

                bool start_found = false;
                bool end_found = false;
                for (const math::vec3v &point : other.winding.points())
                {
                    start_found = start_found || math::equal(point, edge_start);
                    end_found = end_found || math::equal(point, edge_end);
                    if (start_found && end_found)
                        return &other;
                }
            }
            return nullptr;
        }

        void add_bounding_clip_planes(std::vector<brush_face> &faces,
                                      const brush_hull &hull0,
                                      plane_store &planes,
                                      int hull_num,
                                      const brush_build_options &options,
                                      const axial_bevel_flags &axial_bevel)
        {
            math::vec3v origin;
            math::vec3v normal;

            math::add(hull0.bounds.mins, options.hull_size[hull_num][0], origin);
            normal = {-1, 0, 0};
            add_hull_plane(faces, planes, normal, axial_bevel.skip_expansion[0][0] ? hull0.bounds.mins : origin, false);
            normal = {0, -1, 0};
            add_hull_plane(faces, planes, normal, axial_bevel.skip_expansion[1][0] ? hull0.bounds.mins : origin, false);
            normal = {0, 0, -1};
            add_hull_plane(faces, planes, normal, axial_bevel.skip_expansion[2][0] ? hull0.bounds.mins : origin, false);

            math::add(hull0.bounds.maxs, options.hull_size[hull_num][1], origin);
            normal = {1, 0, 0};
            add_hull_plane(faces, planes, normal, axial_bevel.skip_expansion[0][1] ? hull0.bounds.maxs : origin, false);
            normal = {0, 1, 0};
            add_hull_plane(faces, planes, normal, axial_bevel.skip_expansion[1][1] ? hull0.bounds.maxs : origin, false);
            normal = {0, 0, 1};
            add_hull_plane(faces, planes, normal, axial_bevel.skip_expansion[2][1] ? hull0.bounds.maxs : origin, false);
        }

        std::vector<brush_face> make_expanded_clip_planes(const map_brush &brush,
                                                          const brush_hull &hull0,
                                                          const plane_store &source_planes,
                                                          plane_store &planes,
                                                          int hull_num,
                                                          const brush_build_options &options)
        {
            std::vector<brush_face> faces;
            axial_bevel_flags axial_bevel = collect_axial_bevels(hull0, source_planes);

            // face planes cover player vertex vs brush face collisions
            // non axial planes must be shifted by the hull extents
            // axial planes are supplied later by the expanded bounding box
            for (const brush_face &face : hull0.faces)
            {
                const brush_plane &plane = source_planes.planes()[(size_t)face.planenum];
                math::vec3v normal = plane.normal;
                math::vec3v plane_origin = plane.origin;
                if (is_axial(plane.type))
                    continue;

                math::vec3v origin = offset_clip_origin(plane_origin, normal, hull_num, options, face.bevel);
                add_hull_plane(faces, planes, normal, origin, false);
            }

            // edge bevels cover player edge vs brush edge collisions
            // they are only needed for the non legacy clip modes that avoid sticky outside corners
            // where neighboring face normals flip sign on an axis
            if (options.clip == clip_type::simple
                || options.clip == clip_type::precise
                || options.clip == clip_type::normalized)
            {
                bool warned = false;
                for (const brush_face &face : hull0.faces)
                {
                    const brush_plane &plane = source_planes.planes()[(size_t)face.planenum];
                    math::vec3v plane_normal = plane.normal;
                    const std::vector<math::vec3v> &points = face.winding.points();
                    for (int i = 0; i < (int)points.size(); i++)
                    {
                        const math::vec3v &edge_start = points[(size_t)i];
                        const math::vec3v &edge_end = points[(size_t)((i + 1) % (int)points.size())];
                        math::vec3v edge;
                        math::subtract(edge_end, edge_start, edge);

                        const brush_face *other = find_edge_neighbor(hull0, face, edge_start, edge_end);
                        if (!other)
                        {
                            if (hull_num == 1 && !warned)
                            {
                                logging::warn("illegal brush (edge without opposite face): entity %i, brush %i",
                                              brush.original_entity_num, brush.original_brush_num);
                                warned = true;
                            }
                            continue;
                        }

                        const brush_plane &other_plane = source_planes.planes()[(size_t)other->planenum];
                        math::vec3v other_normal = other_plane.normal;
                        for (int dir = 0; dir < 3; dir++)
                        {
                            if (plane_normal[dir] * other_normal[dir] >= -math::normal_epsilon)
                                continue;

                            math::vec3v bevel_edge;
                            bevel_edge[dir] = plane_normal[dir] > 0 ? (vec_t)-1 : (vec_t)1;
                            math::vec3v normal;
                            math::cross(edge, bevel_edge, normal);
                            math::normalize(normal);
                            if (std::fabs(normal[(dir + 1) % 3]) <= math::normal_epsilon
                                || std::fabs(normal[(dir + 2) % 3]) <= math::normal_epsilon)
                            {
                                continue;
                            }

                            math::vec3v origin = offset_clip_origin(edge_start, normal, hull_num, options, false);
                            add_hull_plane(faces, planes, normal, origin, true);
                        }
                    }
                }
            }

            add_bounding_clip_planes(faces, hull0, planes, hull_num, options, axial_bevel);
            return faces;
        }

        struct hull_edge
        {
            math::vec3v normals[2];
            math::vec3v point;
            math::vec3v vertexes[2];
            math::vec3v delta;
        };

        std::vector<brush_face> make_custom_clip_planes(const map_brush &brush,
                                                        const brush_hull &hull0,
                                                        const plane_store &source_planes,
                                                        plane_store &planes,
                                                        const hull_brush &shape,
                                                        int hull_num)
        {
            std::vector<brush_face> faces;
            std::vector<bool> axial_bevel(shape.faces.size(), false);
            bool warned = false;

            // face vertex collisions compare each brush face against the hull shape vertices
            for (const brush_face &face : hull0.faces)
            {
                const brush_plane &source_plane = source_planes.planes()[(size_t)face.planenum];
                math::vec3v brush_normal = source_plane.normal;
                math::vec3v brush_point = source_plane.origin;

                int coplanar_shape_face = -1;
                for (int i = 0; i < (int)shape.faces.size(); i++)
                {
                    const hull_brush_face &shape_face = shape.faces[(size_t)i];
                    if (-math::dot(shape_face.normal, brush_normal) < 1 - math::on_epsilon)
                        continue;

                    vec_t dot_min = (vec_t)80000;
                    vec_t dot_max = (vec_t)-80000;
                    for (const math::vec3v &vertex : shape_face.vertexes)
                    {
                        vec_t d = math::dot(vertex, brush_normal);
                        if (d < dot_min)
                            dot_min = d;
                        if (d > dot_max)
                            dot_max = d;
                    }
                    if (dot_max - dot_min <= math::equal_epsilon)
                    {
                        coplanar_shape_face = i;
                        break;
                    }
                }

                if (coplanar_shape_face >= 0)
                {
                    if (face.bevel)
                        axial_bevel[(size_t)coplanar_shape_face] = true;
                    continue;
                }

                math::vec3v best_vertex;
                vec_t best_dist = (vec_t)80000;
                for (const math::vec3v &vertex : shape.vertexes)
                {
                    vec_t d = math::dot(vertex, brush_normal);
                    if (&vertex == shape.vertexes.data() || d < best_dist - math::normal_epsilon)
                    {
                        best_dist = d;
                        best_vertex = vertex;
                    }
                }

                math::vec3v origin;
                if (face.bevel)
                    origin = brush_point;
                else
                    math::subtract(brush_point, best_vertex, origin);
                add_hull_plane(faces, planes, brush_normal, origin, true);
            }

            // edge edge collisions compare each brush edge against each hull shape edge
            for (const brush_face &face : hull0.faces)
            {
                const brush_plane &source_plane = source_planes.planes()[(size_t)face.planenum];
                const std::vector<math::vec3v> &points = face.winding.points();
                for (int i = 0; i < (int)points.size(); i++)
                {
                    hull_edge brush_edge;
                    brush_edge.normals[0] = source_plane.normal;
                    brush_edge.vertexes[0] = points[(size_t)((i + 1) % (int)points.size())];
                    brush_edge.vertexes[1] = points[(size_t)i];
                    brush_edge.point = brush_edge.vertexes[0];
                    math::subtract(brush_edge.vertexes[1], brush_edge.vertexes[0], brush_edge.delta);

                    int found = 0;
                    for (const brush_face &other : hull0.faces)
                    {
                        const std::vector<math::vec3v> &other_points = other.winding.points();
                        for (int j = 0; j < (int)other_points.size(); j++)
                        {
                            if (math::equal(other_points[(size_t)((j + 1) % (int)other_points.size())], brush_edge.vertexes[1])
                                && math::equal(other_points[(size_t)j], brush_edge.vertexes[0]))
                            {
                                brush_edge.normals[1] = source_planes.planes()[(size_t)other.planenum].normal;
                                found++;
                            }
                        }
                    }
                    if (found != 1)
                    {
                        if (hull_num == 1 && !warned)
                        {
                            logging::warn("illegal brush (edge without opposite face): entity %i, brush %i",
                                          brush.original_entity_num, brush.original_brush_num);
                            warned = true;
                        }
                        continue;
                    }

                    vec_t len = (vec_t)math::length(brush_edge.delta);
                    brush_edge.delta = math::cross(brush_edge.normals[0], brush_edge.normals[1]);
                    if (!math::normalize(brush_edge.delta))
                        continue;
                    math::scale(brush_edge.delta, len, brush_edge.delta);

                    for (const hull_brush_edge &shape_edge : shape.edges)
                    {
                        vec_t dot[4];
                        dot[0] = math::dot(shape_edge.delta, brush_edge.normals[0]);
                        dot[1] = math::dot(shape_edge.delta, brush_edge.normals[1]);
                        dot[2] = math::dot(brush_edge.delta, shape_edge.normals[0]);
                        dot[3] = math::dot(brush_edge.delta, shape_edge.normals[1]);
                        if (dot[0] <= math::on_epsilon || dot[1] >= -math::on_epsilon
                            || dot[2] <= math::on_epsilon || dot[3] >= -math::on_epsilon)
                        {
                            continue;
                        }

                        math::vec3v e1 = brush_edge.delta;
                        math::vec3v e2 = shape_edge.delta;
                        math::normalize(e1);
                        math::normalize(e2);
                        math::vec3v normal = math::cross(e1, e2);
                        if (!math::normalize(normal))
                            continue;
                        math::vec3v origin;
                        math::subtract(brush_edge.point, shape_edge.point, origin);
                        add_hull_plane(faces, planes, normal, origin, true);
                    }
                }
            }

            // vertex face collisions compare each hull shape face against brush vertices
            for (int i = 0; i < (int)shape.faces.size(); i++)
            {
                const hull_brush_face &shape_face = shape.faces[(size_t)i];
                if (hull0.faces.empty())
                    continue;

                math::vec3v best_vertex;
                vec_t best_dist = (vec_t)80000;
                bool have_vertex = false;
                for (const brush_face &face : hull0.faces)
                {
                    for (const math::vec3v &vertex : face.winding.points())
                    {
                        vec_t d = math::dot(vertex, shape_face.normal);
                        if (!have_vertex || d < best_dist - math::normal_epsilon)
                        {
                            best_dist = d;
                            best_vertex = vertex;
                            have_vertex = true;
                        }
                    }
                }

                math::vec3v normal = -shape_face.normal;
                math::vec3v origin;
                if (axial_bevel[(size_t)i])
                    origin = best_vertex;
                else
                    math::subtract(best_vertex, shape_face.point, origin);
                add_hull_plane(faces, planes, normal, origin, true);
            }

            return faces;
        }

        void build_clip_hulls(const map_brush &brush,
                              const plane_store &planes,
                              plane_store &mutable_planes,
                              built_brush &out,
                              const brush_build_options &options)
        {
            for (int h = 1; h < num_hulls; h++)
            {
                if (brush.cliphull && (brush.cliphull & (1 << h)) == 0)
                    continue;

                const hull_shape *shape = nullptr;
                if (options.hull_shapes)
                    shape = &options.hull_shapes->default_hulls[h];
                if (!brush.hull_shapes[h].empty())
                {
                    if (!options.hull_shapes)
                    {
                        err::fatal("entity %i, brush %i: could not find info_hullshape entity '%s'",
                                   brush.original_entity_num, brush.original_brush_num,
                                   brush.hull_shapes[h].c_str());
                    }

                    bool found = false;
                    for (const hull_shape &candidate : options.hull_shapes->shapes)
                    {
                        if (brush.hull_shapes[h] != candidate.id)
                            continue;
                        if (found)
                        {
                            logging::warn("entity %i, brush %i: found several info_hullshape entities with the same name '%s'",
                                          brush.original_entity_num, brush.original_brush_num,
                                          brush.hull_shapes[h].c_str());
                        }
                        shape = &candidate;
                        found = true;
                    }
                    if (!found)
                    {
                        err::fatal("entity %i, brush %i: could not find info_hullshape entity '%s'",
                                   brush.original_entity_num, brush.original_brush_num,
                                   brush.hull_shapes[h].c_str());
                    }
                }

                if (shape && !shape->disabled)
                {
                    if (shape->brushes.empty())
                        continue;
                    std::vector<brush_face> hull_faces = make_custom_clip_planes(
                        brush, out.hulls[0], planes, mutable_planes, shape->brushes[0], h);
                    out.hulls[h] = make_hull_faces(brush, mutable_planes, std::move(hull_faces), options.world_extent);
                    continue;
                }

                std::vector<brush_face> hull_faces = make_expanded_clip_planes(
                    brush, out.hulls[0], planes, mutable_planes, h, options);
                out.hulls[h] = make_hull_faces(brush, mutable_planes, std::move(hull_faces), options.world_extent);
            }
        }

        void set_rounded_origin_key(map_entity &entity, const math::vec3v &origin)
        {
            int rounded[3];
            for (int i = 0; i < 3; i++)
                rounded[i] = (int)(origin[i] >= 0 ? origin[i] + (vec_t)0.5 : origin[i] - (vec_t)0.5);
            char value[64];
            str::format(value, sizeof(value), "%d %d %d", rounded[0], rounded[1], rounded[2]);
            entity.set_value("origin", value);
            entity.origin = {(vec_t)rounded[0], (vec_t)rounded[1], (vec_t)rounded[2]};
        }

        void transform_point(math::vec3v &point,
                             bool scale_point,
                             vec_t scale,
                             const math::vec3v &scale_origin,
                             bool move_point,
                             const math::vec3v &move,
                             bool global_scale_point,
                             vec_t global_scale)
        {
            if (scale_point)
            {
                math::subtract(point, scale_origin, point);
                math::scale(point, scale, point);
                math::add(point, scale_origin, point);
            }
            if (move_point)
                math::add(point, move, point);
            if (global_scale_point)
                math::scale(point, global_scale, point);
        }

        bool parse_vec3(const char *value, math::vec3v &out)
        {
            double x = 0, y = 0, z = 0;
            if (std::sscanf(value, "%lf %lf %lf", &x, &y, &z) != 3)
                return false;
            out = {(vec_t)x, (vec_t)y, (vec_t)z};
            return true;
        }

        struct hull_shape_plane
        {
            math::vec3v normal;
            vec_t dist = 0;
        };

        hull_brush create_hull_brush(const map_source &map,
                                     const map_brush &brush,
                                     const math::vec3v &origin)
        {
            constexpr int max_size = 256;

            std::vector<hull_shape_plane> shape_planes;
            std::vector<math::winding> windings;
            std::vector<hull_brush_edge> edges;
            std::vector<math::vec3v> vertexes;
            bool failed = false;

            for (int i = 0; i < brush.num_sides; i++)
            {
                const brush_side &side = side_at(map, brush, i);
                math::vec3v p[3];
                for (int j = 0; j < 3; j++)
                {
                    math::subtract(side.planepts[j], origin, p[j]);
                    for (int k = 0; k < 3; k++)
                    {
                        vec_t rounded = (vec_t)std::floor(p[j][k] + (vec_t)0.5);
                        if (std::fabs(p[j][k] - rounded) <= math::on_epsilon && p[j][k] != rounded)
                        {
                            logging::warn("entity %i, brush %i: vertex (%4.8f %4.8f %4.8f) of an info_hullshape entity is slightly off grid",
                                          brush.original_entity_num, brush.original_brush_num,
                                          p[j].x, p[j].y, p[j].z);
                        }
                    }
                }

                math::vec3v v1;
                math::vec3v v2;
                math::subtract(p[0], p[1], v1);
                math::subtract(p[2], p[1], v2);
                math::vec3v normal;
                math::cross(v1, v2, normal);
                if (!math::normalize(normal))
                {
                    failed = true;
                    continue;
                }

                for (int k = 0; k < 3; k++)
                {
                    if (std::fabs(normal[k]) < math::normal_epsilon)
                    {
                        normal[k] = 0;
                        math::normalize(normal);
                    }
                }

                math::plane_type type = math::type_for_normal(normal);
                if (is_axial(type))
                {
                    int axis = plane_axis(type);
                    int sign = normal[axis] > 0 ? 1 : -1;
                    normal = {};
                    normal[axis] = (vec_t)sign;
                }

                if ((int)shape_planes.size() >= max_size)
                {
                    failed = true;
                    continue;
                }

                hull_shape_plane plane;
                plane.normal = normal;
                plane.dist = math::dot(p[1], normal);
                shape_planes.push_back(plane);
            }

            windings.reserve(shape_planes.size());
            for (int i = 0; i < (int)shape_planes.size(); i++)
            {
                const hull_shape_plane &plane = shape_planes[(size_t)i];
                math::winding winding = math::winding::from_plane(plane.normal, plane.dist, (vec_t)80000);
                for (int j = 0; j < (int)shape_planes.size(); j++)
                {
                    if (j == i)
                        continue;
                    math::vec3v normal = -shape_planes[(size_t)j].normal;
                    vec_t dist = -shape_planes[(size_t)j].dist;
                    if (!winding.chop(normal, dist))
                    {
                        failed = true;
                        break;
                    }
                }
                windings.push_back(std::move(winding));
            }

            for (int i = 0; i < (int)shape_planes.size(); i++)
            {
                for (int e = 0; e < windings[(size_t)i].size(); e++)
                {
                    if ((int)edges.size() >= max_size)
                    {
                        failed = true;
                        continue;
                    }

                    hull_brush_edge edge;
                    edge.vertexes[0] = windings[(size_t)i][(e + 1) % windings[(size_t)i].size()];
                    edge.vertexes[1] = windings[(size_t)i][e];
                    edge.point = edge.vertexes[0];
                    math::subtract(edge.vertexes[1], edge.vertexes[0], edge.delta);
                    if (math::length(edge.delta) < 1 - math::on_epsilon)
                    {
                        failed = true;
                        continue;
                    }

                    edge.normals[0] = shape_planes[(size_t)i].normal;
                    int found = 0;
                    int neighbor = -1;
                    for (int k = 0; k < (int)shape_planes.size(); k++)
                    {
                        for (int e2 = 0; e2 < windings[(size_t)k].size(); e2++)
                        {
                            if (math::equal(windings[(size_t)k][(e2 + 1) % windings[(size_t)k].size()], edge.vertexes[1])
                                && math::equal(windings[(size_t)k][e2], edge.vertexes[0]))
                            {
                                found++;
                                edge.normals[1] = shape_planes[(size_t)k].normal;
                                neighbor = k;
                            }
                        }
                    }
                    if (found != 1)
                    {
                        failed = true;
                        continue;
                    }
                    if (std::fabs(math::dot(edge.vertexes[0], edge.normals[0]) - shape_planes[(size_t)i].dist) > math::normal_epsilon
                        || std::fabs(math::dot(edge.vertexes[1], edge.normals[0]) - shape_planes[(size_t)i].dist) > math::normal_epsilon
                        || std::fabs(math::dot(edge.vertexes[0], edge.normals[1]) - shape_planes[(size_t)neighbor].dist) > math::normal_epsilon
                        || std::fabs(math::dot(edge.vertexes[1], edge.normals[1]) - shape_planes[(size_t)neighbor].dist) > math::normal_epsilon)
                    {
                        failed = true;
                        continue;
                    }

                    if (neighbor > i)
                        edges.push_back(edge);
                }
            }

            for (int i = 0; i < (int)shape_planes.size(); i++)
            {
                for (int e = 0; e < windings[(size_t)i].size(); e++)
                {
                    math::vec3v vertex = windings[(size_t)i][e];
                    bool exists = false;
                    for (const math::vec3v &known : vertexes)
                    {
                        if (math::equal(known, vertex))
                        {
                            exists = true;
                            break;
                        }
                    }
                    if (exists)
                        continue;
                    if ((int)vertexes.size() > max_size)
                    {
                        failed = true;
                        continue;
                    }

                    vertexes.push_back(vertex);
                    for (const hull_shape_plane &plane : shape_planes)
                    {
                        vec_t d = std::fabs(math::dot(vertex, plane.normal) - plane.dist);
                        if (d < math::on_epsilon && d > math::normal_epsilon)
                            failed = true;
                    }
                }
            }

            if (failed)
            {
                err::fatal("entity %i, brush %i: invalid brush cannot be used for info_hullshape",
                           brush.original_entity_num, brush.original_brush_num);
            }

            hull_brush hull;
            hull.edges = std::move(edges);
            hull.vertexes = std::move(vertexes);
            hull.faces.reserve(shape_planes.size());
            for (int i = 0; i < (int)shape_planes.size(); i++)
            {
                hull_brush_face face;
                face.normal = shape_planes[(size_t)i].normal;
                if (!windings[(size_t)i].empty())
                    face.point = windings[(size_t)i][0];
                face.vertexes = windings[(size_t)i].points();
                hull.faces.push_back(std::move(face));
            }
            return hull;
        }

    }

    void set_origin_key(map_entity &entity, const math::vec3v &origin)
    {
        char value[64];
        str::format(value, sizeof(value), "%i %i %i", (int)origin.x, (int)origin.y, (int)origin.z);
        entity.set_value("origin", value);
        entity.origin = origin;
    }

    void set_bounds_key(map_entity &entity, const math::vec3v &mins, const math::vec3v &maxs)
    {
        char value[128];
        str::format(value, sizeof(value), "%.0f %.0f %.0f %.0f %.0f %.0f",
                    mins.x, mins.y, mins.z, maxs.x, maxs.y, maxs.z);
        entity.set_value("zhlt_minsmaxs", value);
    }

    void apply_entity_transform(map_source &map,
                                int entity_num,
                                const brush_build_options &options)
    {
        map_entity &entity = map.entities[(size_t)entity_num];
        if (std::strcmp(entity.value("classname"), "info_hullshape") == 0)
            return;

        bool move = false;
        bool scale = false;
        bool global_scale = false;
        math::vec3v move_value;
        vec_t scale_value = 1;
        vec_t global_scale_value = 1;

        if (options.global_scale > 0)
        {
            global_scale = true;
            global_scale_value = options.global_scale;
        }

        double values[4] = {};
        const char *transform = entity.value("zhlt_transform");
        if (transform[0])
        {
            enum transform_value_count
            {
                transform_scale_only = 1,
                transform_move_only = 3,
                transform_scale_and_move = 4,
            };

            switch (std::sscanf(transform, "%lf %lf %lf %lf", values, values + 1, values + 2, values + 3))
            {
            case transform_scale_only:
                scale = true;
                scale_value = (vec_t)values[0];
                break;
            case transform_move_only:
                move = true;
                move_value = {(vec_t)values[0], (vec_t)values[1], (vec_t)values[2]};
                break;
            case transform_scale_and_move:
                scale = true;
                scale_value = (vec_t)values[0];
                move = true;
                move_value = {(vec_t)values[1], (vec_t)values[2], (vec_t)values[3]};
                break;
            default:
                logging::warn("bad value '%s' for key 'zhlt_transform'", transform);
                break;
            }
            entity.set_value("zhlt_transform", "");
        }

        if (!move && !scale && !global_scale)
            return;

        if (map.map_file_version < 220 || (!map.sides.empty() && map.sides[0].texture.txcommand != 0))
        {
            logging::warn("hlcsg scaling hack is not supported in worldcraft 2.1- or quark mode");
            return;
        }

        math::vec3v scale_origin;
        parse_vec3(entity.value("origin"), scale_origin);

        for (int i = 0; i < entity.num_brushes; i++)
        {
            const map_brush &brush = map.brushes[(size_t)entity.first_brush + i];
            for (int j = 0; j < brush.num_sides; j++)
            {
                brush_side &side = map.sides[(size_t)brush.first_side + j];
                for (int k = 0; k < 3; k++)
                {
                    transform_point(side.planepts[k], scale, scale_value, scale_origin,
                                    move, move_value, global_scale, global_scale_value);
                }

                bool zero_scale = false;
                if (!side.texture.valve.scale[0])
                    side.texture.valve.scale[0] = 1;
                if (!side.texture.valve.scale[1])
                    side.texture.valve.scale[1] = 1;

                if (scale)
                {
                    vec_t coord[2];
                    if (std::fabs(side.texture.valve.scale[0]) > math::normal_epsilon)
                    {
                        coord[0] = math::dot(scale_origin, side.texture.valve.u_axis)
                            / side.texture.valve.scale[0] + side.texture.valve.shift[0];
                        side.texture.valve.scale[0] *= scale_value;
                        if (std::fabs(side.texture.valve.scale[0]) > math::normal_epsilon)
                        {
                            side.texture.valve.shift[0] = coord[0]
                                - math::dot(scale_origin, side.texture.valve.u_axis)
                                    / side.texture.valve.scale[0];
                        }
                        else
                            zero_scale = true;
                    }
                    else
                        zero_scale = true;

                    if (std::fabs(side.texture.valve.scale[1]) > math::normal_epsilon)
                    {
                        coord[1] = math::dot(scale_origin, side.texture.valve.v_axis)
                            / side.texture.valve.scale[1] + side.texture.valve.shift[1];
                        side.texture.valve.scale[1] *= scale_value;
                        if (std::fabs(side.texture.valve.scale[1]) > math::normal_epsilon)
                        {
                            side.texture.valve.shift[1] = coord[1]
                                - math::dot(scale_origin, side.texture.valve.v_axis)
                                    / side.texture.valve.scale[1];
                        }
                        else
                            zero_scale = true;
                    }
                    else
                        zero_scale = true;
                }

                if (move)
                {
                    if (std::fabs(side.texture.valve.scale[0]) > math::normal_epsilon)
                    {
                        side.texture.valve.shift[0] -= math::dot(move_value, side.texture.valve.u_axis)
                            / side.texture.valve.scale[0];
                    }
                    else
                        zero_scale = true;

                    if (std::fabs(side.texture.valve.scale[1]) > math::normal_epsilon)
                    {
                        side.texture.valve.shift[1] -= math::dot(move_value, side.texture.valve.v_axis)
                            / side.texture.valve.scale[1];
                    }
                    else
                        zero_scale = true;
                }

                if (global_scale)
                {
                    side.texture.valve.scale[0] *= global_scale_value;
                    side.texture.valve.scale[1] *= global_scale_value;
                }

                if (zero_scale)
                {
                    err::fatal("entity %i, brush %i: invalid texture scale",
                               brush.original_entity_num, brush.original_brush_num);
                }
            }
        }

        if (global_scale && entity.value("origin")[0])
        {
            math::vec3v origin;
            parse_vec3(entity.value("origin"), origin);
            math::scale(origin, global_scale_value, origin);
            set_rounded_origin_key(entity, origin);
        }

        // the reference writes the two transformed corner points back
        // verbatim, without sorting them into mins/maxs, so a negative
        // scale keeps the corners swapped in the key
        double raw_bounds[2][3];
        if (std::sscanf(entity.value("zhlt_minsmaxs"), "%lf %lf %lf %lf %lf %lf",
                        &raw_bounds[0][0], &raw_bounds[0][1], &raw_bounds[0][2],
                        &raw_bounds[1][0], &raw_bounds[1][1], &raw_bounds[1][2]) == 6)
        {
            math::vec3v corners[2];
            for (int i = 0; i < 2; i++)
            {
                corners[i] = {(vec_t)raw_bounds[i][0], (vec_t)raw_bounds[i][1], (vec_t)raw_bounds[i][2]};
                transform_point(corners[i], scale, scale_value, scale_origin,
                                move, move_value, global_scale, global_scale_value);
            }
            set_bounds_key(entity, corners[0], corners[1]);
        }
    }

    void texture_axis_from_plane(const math::plane &plane, math::vec3v &xv, math::vec3v &yv)
    {
        int best_axis = 0;
        vec_t best = 0;

        for (int i = 0; i < 6; i++)
        {
            vec_t dot = math::dot(plane.normal, base_axis[i * 3]);
            if (dot > best)
            {
                best = dot;
                best_axis = i;
            }
        }

        math::copy(base_axis[best_axis * 3 + 1], xv);
        math::copy(base_axis[best_axis * 3 + 2], yv);
    }

    content texture_content(const char *name)
    {
        if (str::istarts_with(name, "contentsolid"))
            return content::solid;
        if (str::istarts_with(name, "contentwater"))
            return content::water;
        if (str::istarts_with(name, "contentempty"))
            return content::to_empty;
        if (str::istarts_with(name, "contentsky"))
            return content::sky;
        if (str::istarts_with(name, "sky"))
            return content::sky;
        if (str::istarts_with(name, "env_sky"))
            return content::sky;

        if (str::istarts_with(name + 1, "!lava"))
            return content::lava;
        if (str::istarts_with(name + 1, "!slime"))
            return content::slime;
        if (str::istarts_with(name, "!lava"))
            return content::lava;
        if (str::istarts_with(name, "!slime"))
            return content::slime;

        if (name[0] == '!')
        {
            if (str::istarts_with(name, "!cur_90"))
                return content::current_90;
            if (str::istarts_with(name, "!cur_0"))
                return content::current_0;
            if (str::istarts_with(name, "!cur_270"))
                return content::current_270;
            if (str::istarts_with(name, "!cur_180"))
                return content::current_180;
            if (str::istarts_with(name, "!cur_up"))
                return content::current_up;
            if (str::istarts_with(name, "!cur_dwn"))
                return content::current_down;
            return content::water;
        }

        if (str::istarts_with(name, "origin"))
            return content::origin;
        if (str::istarts_with(name, "boundingbox"))
            return content::bounding_box;
        if (str::istarts_with(name, "solidhint"))
            return content::null;
        if (str::istarts_with(name, "bevelhint"))
            return content::null;
        if (str::istarts_with(name, "splitface"))
            return content::hint;
        if (str::istarts_with(name, "hint"))
            return content::to_empty;
        if (str::istarts_with(name, "skip"))
            return content::to_empty;
        if (str::istarts_with(name, "translucent"))
            return content::translucent;
        if (name[0] == '@')
            return content::translucent;
        if (str::istarts_with(name, "null"))
            return content::null;
        if (str::istarts_with(name, "bevel"))
            return content::null;

        return content::solid;
    }

    const char *content_to_string(content type)
    {
        switch (type)
        {
        case content::empty:
            return "empty";
        case content::solid:
            return "solid";
        case content::water:
            return "water";
        case content::slime:
            return "slime";
        case content::lava:
            return "lava";
        case content::sky:
            return "sky";
        case content::origin:
            return "origin";
        case content::bounding_box:
            return "boundingbox";
        case content::current_0:
            return "current_0";
        case content::current_90:
            return "current_90";
        case content::current_180:
            return "current_180";
        case content::current_270:
            return "current_270";
        case content::current_up:
            return "current_up";
        case content::current_down:
            return "current_down";
        case content::translucent:
            return "translucent";
        case content::hint:
            return "hint";
        case content::null:
            return "null";
        case content::to_empty:
            return "empty";
        default:
            return "unknown";
        }
    }

    content check_brush_contents(const map_source &map, const map_brush &brush)
    {
        if (brush.num_sides == 0)
        {
            err::fatal("entity %i, brush %i: brush with no sides",
                       brush.original_entity_num, brush.original_brush_num);
        }

        int best_i = 0;
        const brush_side *side = &side_at(map, brush, 0);
        content best_content = texture_content(side->texture.name);
        bool assigned = str::istarts_with(side->texture.name, "content")
            || str::istarts_with(side->texture.name, "skip");

        for (int i = 1; i < brush.num_sides; i++)
        {
            side = &side_at(map, brush, i);
            content consider = texture_content(side->texture.name);
            if (assigned)
                continue;
            if (str::istarts_with(side->texture.name, "content") || str::istarts_with(side->texture.name, "skip"))
            {
                best_i = i;
                best_content = consider;
                assigned = true;
            }
            if ((int)consider > (int)best_content)
            {
                best_i = i;
                best_content = consider;
            }
        }

        content contents = best_content;

        for (int i = 0; i < brush.num_sides; i++)
        {
            side = &side_at(map, brush, i);
            content contents2 = texture_content(side->texture.name);
            if (assigned
                && !str::istarts_with(side->texture.name, "content")
                && !str::istarts_with(side->texture.name, "skip")
                && contents2 != content::origin
                && contents2 != content::hint
                && contents2 != content::bounding_box)
            {
                continue;
            }

            if (contents2 == content::sky)
                continue;
            if (contents2 == content::null)
                continue;

            if (contents2 != best_content)
            {
                err::fatal("entity %i, brush %i: mixed face contents\n    texture %s and %s",
                           brush.original_entity_num, brush.original_brush_num,
                           side_at(map, brush, best_i).texture.name, side->texture.name);
            }
        }

        if (contents == content::null)
            contents = content::solid;

        const char *classname = "";
        if (brush.entity_num >= 0 && brush.entity_num < (int)map.entities.size())
            classname = map.entities[(size_t)brush.entity_num].value("classname");

        if (brush.entity_num == 0 || std::strcmp("func_group", classname) == 0)
        {
            if ((contents == content::origin && brush.entity_num == 0)
                || contents == content::bounding_box)
            {
                err::fatal("entity %i, brush %i: %s brushes not allowed in world",
                           brush.original_entity_num, brush.original_brush_num,
                           content_to_string(contents));
            }
        }
        else
        {
            switch (contents)
            {
            case content::solid:
            case content::water:
            case content::slime:
            case content::lava:
            case content::origin:
            case content::bounding_box:
            case content::hint:
            case content::to_empty:
                break;
            default:
                err::fatal("entity %i, brush %i: %s brushes not allowed in entity",
                           brush.original_entity_num, brush.original_brush_num,
                           content_to_string(contents));
                break;
            }
        }

        return contents;
    }

    int plane_store::find_int_plane(const math::vec3v &normal, const math::vec3v &origin)
    {
        for (int i = 0; i < (int)planes_.size(); i++)
        {
            const brush_plane &plane = planes_[(size_t)i];
            vec_t t;
            if (-math::dir_epsilon < (t = normal.x - plane.normal.x) && t < math::dir_epsilon
                && -math::dir_epsilon < (t = normal.y - plane.normal.y) && t < math::dir_epsilon
                && -math::dir_epsilon < (t = normal.z - plane.normal.z) && t < math::dir_epsilon)
            {
                t = math::dot(origin, plane.normal) - plane.dist;
                if (-dist_epsilon < t && t < dist_epsilon)
                    return i;
            }
        }

        if (planes_.size() + 1 >= limits::max_internal_map_planes)
            err::fatal("exceeded max_internal_map_planes");

        brush_plane plane;
        math::copy(origin, plane.origin);
        math::copy(normal, plane.normal);
        math::normalize(plane.normal);
        plane.type = math::type_for_normal(plane.normal);
        if (is_axial(plane.type))
        {
            int axis = plane_axis(plane.type);
            for (int i = 0; i < 3; i++)
            {
                if (i == axis)
                    plane.normal[i] = plane.normal[i] > 0 ? 1 : -1;
                else
                    plane.normal[i] = 0;
            }
        }
        plane.dist = math::dot(origin, plane.normal);

        brush_plane opposite;
        math::copy(origin, opposite.origin);
        math::subtract(math::vec3v{}, plane.normal, opposite.normal);
        opposite.type = plane.type;
        opposite.dist = -plane.dist;

        int return_value = (int)planes_.size();
        if (normal[plane_axis(plane.type)] < 0)
        {
            std::swap(plane, opposite);
            return_value++;
        }

        planes_.push_back(plane);
        planes_.push_back(opposite);
        return return_value;
    }

    int plane_store::plane_from_points(const math::vec3v &p0, const math::vec3v &p1, const math::vec3v &p2)
    {
        math::vec3v v1, v2;
        math::subtract(p0, p1, v1);
        math::subtract(p2, p1, v2);
        math::vec3v normal;
        math::cross(v1, v2, normal);
        if (math::normalize(normal))
            return find_int_plane(normal, p0);
        return -1;
    }

    std::vector<brush_face> make_brush_planes(map_source &map,
                                              const map_brush &brush,
                                              plane_store &planes,
                                              const math::vec3v &origin,
                                              texinfo_store *texinfos,
                                              bool only_entities)
    {
        std::vector<brush_face> faces;

        for (int i = 0; i < brush.num_sides; i++)
        {
            brush_side &side = map.sides[(size_t)brush.first_side + i];
            for (int j = 0; j < 3; j++)
                math::subtract(side.planepts[j], origin, side.planepts[j]);

            int planenum = planes.plane_from_points(side.planepts[0], side.planepts[1], side.planepts[2]);
            if (planenum == -1)
            {
                err::fatal("entity %i, brush %i, side %i: plane with no normal",
                           brush.original_entity_num, brush.original_brush_num, i);
            }

            for (const brush_face &face : faces)
            {
                if (face.planenum == planenum || face.planenum == (planenum ^ 1))
                {
                    err::fatal("entity %i, brush %i, side %i: has a coplanar plane at (%.0f, %.0f, %.0f), texture %s",
                               brush.original_entity_num, brush.original_brush_num,
                               i,
                               side.planepts[0].x + origin.x,
                               side.planepts[0].y + origin.y,
                               side.planepts[0].z + origin.z,
                               side.texture.name);
                }
            }

            brush_face face;
            face.planenum = planenum;
            if (only_entities)
                face.texinfo = 0;
            else if (texinfos)
                face.texinfo = texinfos->texinfo_for_brush_texture(
                    planes.planes()[(size_t)planenum], side.texture, origin, map.map_file_version);
            face.bevel = brush.bevel || side.bevel;
            faces.insert(faces.begin(), face);
        }

        return faces;
    }

    brush_hull make_hull_faces(const map_brush &brush,
                               const plane_store &planes,
                               std::vector<brush_face> faces,
                               vec_t world_extent)
    {
        sort_sides(faces, planes);

    restart:
        math::bounding_box bounds;

        for (size_t i = 0; i < faces.size(); i++)
        {
            brush_face &face = faces[i];
            const brush_plane &plane = planes.planes()[(size_t)face.planenum];
            math::winding winding = math::winding::from_plane(plane.normal, plane.dist, (vec_t)80000);

            for (size_t j = 0; j < faces.size(); j++)
            {
                if (i == j)
                    continue;
                const brush_plane &clip = planes.planes()[(size_t)(faces[j].planenum ^ 1)];
                if (!winding.chop(clip.normal, clip.dist))
                    break;
            }

            winding.remove_colinear_points();
            if (winding.area() < (vec_t)0.1)
            {
                faces.erase(faces.begin() + i);
                goto restart;
            }

            face.contents = content::empty;
            face.winding = std::move(winding);
            for (const math::vec3v &point : face.winding.points())
                bounds.add(point);
        }

        for (int i = 0; i < 3; i++)
        {
            if (bounds.mins[i] < -world_extent / 2 || bounds.maxs[i] > world_extent / 2)
            {
                err::fatal("entity %i, brush %i: outside world(+/-%d): (%.0f,%.0f,%.0f)-(%.0f,%.0f,%.0f)",
                           brush.original_entity_num, brush.original_brush_num,
                           (int)(world_extent / 2),
                           bounds.mins.x, bounds.mins.y, bounds.mins.z,
                           bounds.maxs.x, bounds.maxs.y, bounds.maxs.z);
            }
        }

        brush_hull hull;
        hull.faces = std::move(faces);
        hull.bounds = bounds;
        return hull;
    }

    std::vector<brush_face> make_bounding_clip_planes(const brush_hull &hull0,
                                                      plane_store &planes,
                                                      int hull_num,
                                                      const brush_build_options &options)
    {
        std::vector<brush_face> faces;
        axial_bevel_flags axial_bevel = collect_axial_bevels(hull0, planes);
        add_bounding_clip_planes(faces, hull0, planes, hull_num, options, axial_bevel);
        return faces;
    }

    // registers one info_hullshape entity during the post parse walk a
    // zhlt_hull key can only reference shapes defined earlier in the map
    void add_hull_shape(map_source &map, int entity_num, hull_shape_library &library)
    {
        map_entity &entity = map.entities[(size_t)entity_num];

        math::vec3v origin;
        if (!parse_vec3(entity.value("origin"), origin))
            logging::warn("info_hullshape with no origin brush");

        hull_shape shape;
        shape.id = entity.value("targetname");
        shape.disabled = entity.int_value("disabled") != 0;

        for (int i = 0; i < entity.num_brushes; i++)
        {
            const map_brush &brush = map.brushes[(size_t)entity.first_brush + i];
            if (brush.contents == content::origin)
                continue;
            shape.brushes.push_back(create_hull_brush(map, brush, origin));
        }

        if (shape.brushes.size() >= 2)
        {
            const map_brush &brush = map.brushes[(size_t)entity.first_brush];
            err::fatal("entity %i, brush %i: too many brushes in info_hullshape",
                       brush.original_entity_num, brush.original_brush_num);
        }

        int defaulthulls = entity.int_value("defaulthulls");
        for (int h = 0; h < num_hulls; h++)
        {
            if (defaulthulls & (1 << h))
                library.default_hulls[h] = shape;
        }
        library.shapes.push_back(std::move(shape));
    }

    // builds the geometry of one brush from its stored contents, mirroring the
    // reference createbrush origin and boundingbox brushes were fully built
    // during the post parse walk (their planes and texinfos already sit at the
    // front of the stores), so here they stay inert like the reference's
    // early return
    built_brush create_brush(map_source &map,
                             const map_brush &brush,
                             plane_store &planes,
                             const math::vec3v &origin,
                             texinfo_store *texinfos,
                             const brush_build_options &options)
    {
        built_brush out;
        out.contents = brush.contents;
        out.original_entity_num = brush.original_entity_num;
        out.original_brush_num = brush.original_brush_num;
        out.detail_level = brush.detail_level;
        out.chop_down = brush.chop_down;
        out.chop_up = brush.chop_up;
        out.clipnode_detail_level = brush.clipnode_detail_level;
        out.coplanar_priority = brush.coplanar_priority;

        if (out.contents == content::origin || out.contents == content::bounding_box)
        {
            out.skipped = true;
            return out;
        }

        std::vector<brush_face> faces = make_brush_planes(
            map, brush, planes, origin, texinfos, options.only_entities);
        out.hulls[0] = make_hull_faces(brush, planes, std::move(faces), options.world_extent);

        if (out.contents == content::hint || out.contents == content::to_empty)
            return out;

        if (options.noclip)
        {
            if (brush.cliphull)
                out.hulls[0].faces.clear();
            return out;
        }

        if (brush.cliphull)
        {
            out.contents = content::solid;
            build_clip_hulls(brush, planes, planes, out, options);
            out.hulls[0].faces.clear();
            return out;
        }

        if (brush.noclip)
            return out;

        build_clip_hulls(brush, planes, planes, out, options);
        return out;
    }

    built_entity build_entity_brushes(map_source &map,
                                      int entity_num,
                                      plane_store &planes,
                                      texinfo_store *texinfos,
                                      const brush_build_options &options)
    {
        if (entity_num < 0 || entity_num >= (int)map.entities.size())
            err::fatal("build_entity_brushes: bad entity index %i", entity_num);

        map_entity &entity = map.entities[(size_t)entity_num];
        built_entity out;
        out.entity_num = entity_num;
        out.brushes.reserve((size_t)entity.num_brushes);

        for (int i = 0; i < entity.num_brushes; i++)
        {
            const map_brush &brush = map.brushes[(size_t)entity.first_brush + i];
            out.brushes.push_back(create_brush(map, brush, planes, entity.origin, texinfos, options));
        }

        return out;
    }
}
