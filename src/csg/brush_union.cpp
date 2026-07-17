#include "brush_union.h"

#include <cmath>
#include <utility>

namespace csg
{
    namespace
    {
        struct union_face
        {
            int planenum = -1;
            math::winding winding;
        };

        struct union_hull
        {
            std::vector<union_face> faces;
        };

        union_hull copy_hull(const brush_hull &hull)
        {
            union_hull out;
            out.faces.reserve(hull.faces.size());
            for (const brush_face &face : hull.faces)
            {
                union_face copy;
                copy.planenum = face.planenum;
                copy.winding = face.winding;
                out.faces.push_back(std::move(copy));
            }
            return out;
        }

        int number_of_hull_faces(const union_hull &hull)
        {
            return (int)hull.faces.size();
        }

        math::winding new_winding_from_plane(const csg_result &result,
                                             const union_hull &hull,
                                             int planenum)
        {
            const brush_plane &base = result.planes[(size_t)planenum];
            math::winding winding = math::winding::from_plane(base.normal, base.dist, (vec_t)80000);
            for (const union_face &face : hull.faces)
            {
                const brush_plane &plane = result.planes[(size_t)face.planenum];
                math::winding front;
                math::winding back;
                winding.clip(plane.normal, plane.dist, front, back);
                if (!back.empty())
                {
                    winding = std::move(back);
                }
                else
                {
                    return {};
                }
            }
            return winding;
        }

        void add_plane_to_union(const csg_result &result, union_hull &hull, int planenum)
        {
            if (hull.faces.empty())
                return;

            bool need_new_face = false;
            std::vector<union_face> new_faces;

            for (union_face face : hull.faces)
            {
                if (face.planenum == planenum)
                {
                    new_faces.push_back(std::move(face));
                    continue;
                }

                const brush_plane &split = result.planes[(size_t)planenum];
                math::winding front;
                math::winding back;
                face.winding.clip(split.normal, split.dist, front, back);

                if (!front.empty())
                {
                    need_new_face = true;
                    if (!back.empty())
                    {
                        face.winding = std::move(back);
                        new_faces.push_back(std::move(face));
                    }
                }
                else if (!back.empty())
                {
                    new_faces.push_back(std::move(face));
                }
            }

            hull.faces = std::move(new_faces);

            if (need_new_face && number_of_hull_faces(hull) > 2)
            {
                math::winding new_winding = new_winding_from_plane(result, hull, planenum);
                if (!new_winding.empty())
                {
                    union_face new_face;
                    new_face.planenum = planenum;
                    new_face.winding = std::move(new_winding);
                    hull.faces.insert(hull.faces.begin(), std::move(new_face));
                }
            }
        }

        math::vec3v winding_center(const math::winding &winding)
        {
            math::vec3v center;
            if (winding.empty())
                return center;

            for (const math::vec3v &point : winding.points())
                math::add(point, center, center);
            math::scale(center, (vec_t)(1.0 / winding.size()), center);
            return center;
        }

        vec_t calculate_solid_volume(const csg_result &result, const union_hull &hull)
        {
            int count = 0;
            vec_t volume = 0.0;
            math::vec3v midpoint;

            for (const union_face &face : hull.faces)
            {
                math::vec3v face_mid = winding_center(face.winding);
                math::add(midpoint, face_mid, midpoint);
                count++;
            }
            if (!count)
                return 0;

            math::scale(midpoint, (vec_t)(1.0 / count), midpoint);

            for (const union_face &face : hull.faces)
            {
                const brush_plane &plane = result.planes[(size_t)face.planenum];
                vec_t area = face.winding.area();
                vec_t dist = math::dot(plane.normal, midpoint);
                dist -= plane.dist;
                dist = (vec_t)std::fabs(dist);
                volume += area * dist / 3.0;
            }

            return volume;
        }

        vec_t calculate_solid_volume(const csg_result &result, const brush_hull &hull)
        {
            return calculate_solid_volume(result, copy_hull(hull));
        }

        bool is_invalid_hull(const union_hull &hull, vec_t world_extent)
        {
            math::bounding_box bounds;
            for (const union_face &face : hull.faces)
            {
                for (const math::vec3v &point : face.winding.points())
                    bounds.add(point);
            }

            for (int i = 0; i < 3; i++)
            {
                if (bounds.mins[i] < -world_extent / 2 || bounds.maxs[i] > world_extent / 2)
                    return true;
            }
            return false;
        }

        void add_pair_warnings(const csg_result &result,
                               const built_brush &first,
                               const built_brush &second,
                               const union_hull &hull,
                               vec_t threshold,
                               std::vector<brush_union_warning> &warnings)
        {
            vec_t union_volume = calculate_solid_volume(result, hull);
            vec_t first_volume = calculate_solid_volume(result, first.hulls[0]);
            vec_t second_volume = calculate_solid_volume(result, second.hulls[0]);
            if (first_volume == 0 || second_volume == 0)
                return;

            vec_t first_ratio = union_volume / first_volume;
            vec_t second_ratio = union_volume / second_volume;
            if (first_ratio > threshold)
            {
                brush_union_warning warning;
                warning.entity_num = first.original_entity_num;
                warning.brush_num = first.original_brush_num;
                warning.other_brush_num = second.original_brush_num;
                warning.percent = first_ratio * 100.0;
                warnings.push_back(warning);
            }
            if (second_ratio > threshold)
            {
                brush_union_warning warning;
                warning.entity_num = first.original_entity_num;
                warning.brush_num = second.original_brush_num;
                warning.other_brush_num = first.original_brush_num;
                warning.percent = second_ratio * 100.0;
                warnings.push_back(warning);
            }
        }
    }

    std::vector<brush_union_warning> calculate_brush_union_warnings(const csg_result &result,
                                                                    vec_t threshold,
                                                                    vec_t world_extent)
    {
        std::vector<brush_union_warning> warnings;
        if (threshold <= 0.0 || threshold > 100.0)
            return warnings;

        // the reference pair loop starts at the global brush index + 1 but
        // bounds it with the entity relative brush count, so in practice only
        // worldspawn (where the two numberings coincide) produces warnings
        // kept verbatim for output parity
        int entity_start = 0;
        for (const built_entity &entity : result.entities)
        {
            for (int first_index = 0; first_index < (int)entity.brushes.size(); first_index++)
            {
                const built_brush &first = entity.brushes[(size_t)first_index];
                const brush_hull &first_hull = first.hulls[0];
                if (first_hull.faces.empty())
                    continue;

                int global_first = entity_start + first_index;
                for (int second_index = global_first + 1; second_index < (int)entity.brushes.size(); second_index++)
                {
                    const built_brush &second = entity.brushes[(size_t)second_index];
                    const brush_hull &second_hull = second.hulls[0];
                    if (second_hull.faces.empty())
                        continue;
                    if (first.contents != second.contents)
                        continue;

                    union_hull hull = copy_hull(first_hull);
                    for (const brush_face &face : second_hull.faces)
                        add_plane_to_union(result, hull, face.planenum);
                    if (hull.faces.empty())
                        continue;
                    if (is_invalid_hull(hull, world_extent))
                        continue;

                    add_pair_warnings(result, first, second, hull, threshold, warnings);
                }
            }
            entity_start += (int)entity.brushes.size();
        }

        return warnings;
    }
}
