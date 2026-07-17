#include "map_decompiler.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "common/binary.h"
#include "format/bsp/data.h"
#include "format/bsp/entity_lump.h"
#include "format/bsp/face_extents.h"
#include "../math/vector.h"
#include "../math/winding.h"

namespace decompile
{
    namespace
    {
        using vec3 = math::vec3<double>;
        using winding = math::basic_winding<double>;

        constexpr double plane_epsilon = 0.04;
        constexpr double normal_epsilon = 0.0001;
        constexpr double minimum_face_area = 0.1;
        constexpr double bounds_padding = 8.0;
        constexpr double winding_range = 131072.0;
        constexpr int maximum_tree_depth = 4096;
        constexpr int maximum_texture_split_depth = 64;

        struct plane_side
        {
            vec3 normal;
            double dist = 0;
        };

        struct source_face
        {
            plane_side plane;
            winding polygon;
            int texinfo = -1;
        };

        struct output_side
        {
            plane_side plane;
            winding polygon;
            int texinfo = -1;
        };

        struct output_brush
        {
            std::vector<output_side> sides;
            int contents = -2;
        };

        void set_error(std::string *error, const std::string &message)
        {
            if (error)
                *error = message;
        }

        vec3 negate(const vec3 &value)
        {
            return {-value.x, -value.y, -value.z};
        }

        plane_side bsp_plane(const format::dplane_t &plane, bool opposite = false)
        {
            plane_side out;
            double sign = opposite ? -1.0 : 1.0;
            out.normal = {sign * plane.normal[0], sign * plane.normal[1], sign * plane.normal[2]};
            out.dist = sign * plane.dist;
            return out;
        }

        bool same_plane(const plane_side &a, const plane_side &b)
        {
            return std::fabs(a.normal.x - b.normal.x) < normal_epsilon
                && std::fabs(a.normal.y - b.normal.y) < normal_epsilon
                && std::fabs(a.normal.z - b.normal.z) < normal_epsilon
                && std::fabs(a.dist - b.dist) < plane_epsilon;
        }

        void add_unique_plane(std::vector<plane_side> &planes, const plane_side &plane)
        {
            for (const plane_side &existing : planes)
            {
                if (same_plane(existing, plane))
                    return;
            }
            planes.push_back(plane);
        }

        winding face_winding(const format::map_data &map, const format::dface_t &face)
        {
            winding out;
            if (face.firstedge < 0 || face.numedges < 3
                || (std::size_t)face.firstedge + (std::size_t)face.numedges > map.surfedges.size())
            {
                return out;
            }

            for (int i = 0; i < face.numedges; i++)
            {
                int surfedge = map.surfedges[(std::size_t)face.firstedge + (std::size_t)i];
                std::size_t edge_index = (std::size_t)(surfedge < 0 ? -surfedge : surfedge);
                if (edge_index >= map.edges.size())
                    return winding{};
                const format::dedge_t &edge = map.edges[edge_index];
                unsigned vertex_index = edge.v[surfedge < 0 ? 1 : 0];
                if (vertex_index >= map.vertexes.size())
                    return winding{};
                const float *point = map.vertexes[vertex_index].point;
                out.add_point({point[0], point[1], point[2]});
            }
            out.remove_colinear_points();
            return out;
        }

        bool collect_model_faces(const format::map_data &map, const format::dmodel_t &model,
                                 std::vector<source_face> &out, std::string *error)
        {
            if (model.firstface < 0 || model.numfaces < 0
                || (std::size_t)model.firstface + (std::size_t)model.numfaces > map.faces.size())
            {
                set_error(error, "model face range is outside the face lump");
                return false;
            }

            out.clear();
            out.reserve((std::size_t)model.numfaces);
            for (int i = 0; i < model.numfaces; i++)
            {
                const format::dface_t &face = map.faces[(std::size_t)model.firstface + (std::size_t)i];
                if (face.planenum >= map.planes.size())
                {
                    set_error(error, "face references an invalid plane");
                    return false;
                }
                if (face.texinfo < 0 || (std::size_t)face.texinfo >= map.texinfo.size())
                {
                    set_error(error, "face references an invalid texinfo");
                    return false;
                }
                winding polygon = face_winding(map, face);
                if (polygon.size() < 3 || polygon.area() < minimum_face_area)
                    continue;

                source_face source;
                source.plane = bsp_plane(map.planes[face.planenum], face.side != 0);
                source.polygon = std::move(polygon);
                source.texinfo = format::parse_texinfo_for_face(map, &face);
                out.push_back(std::move(source));
            }
            return true;
        }

        double overlap_area(const winding &cell, const source_face &face)
        {
            winding clipped = cell;
            vec3 center = face.polygon.center();
            for (int i = 0; i < face.polygon.size(); i++)
            {
                const vec3 &a = face.polygon[i];
                const vec3 &b = face.polygon[(i + 1) % face.polygon.size()];
                vec3 inward = math::cross(b - a, face.plane.normal);
                if (math::normalize(inward) == 0)
                    continue;
                double dist = math::dot(inward, a);
                if (math::dot(inward, center) < dist)
                {
                    inward = negate(inward);
                    dist = -dist;
                }
                if (!clipped.chop(inward, dist))
                    return 0;
            }
            return clipped.area();
        }

        int best_texinfo(const output_side &side, const std::vector<source_face> &faces)
        {
            double best_area = minimum_face_area;
            int best = -1;
            for (const source_face &face : faces)
            {
                if (!same_plane(side.plane, face.plane))
                    continue;
                double area = overlap_area(side.polygon, face);
                if (area > best_area)
                {
                    best_area = area;
                    best = face.texinfo;
                }
            }
            return best;
        }

        bool build_cell(const std::vector<plane_side> &planes,
                        const std::vector<source_face> &faces, int contents,
                        output_brush &out)
        {
            out = output_brush{};
            out.contents = contents;
            out.sides.reserve(planes.size());

            for (std::size_t i = 0; i < planes.size(); i++)
            {
                const plane_side &plane = planes[i];
                winding polygon = winding::from_plane(plane.normal, plane.dist, winding_range);
                for (std::size_t j = 0; j < planes.size() && !polygon.empty(); j++)
                {
                    if (i == j)
                        continue;
                    polygon.chop(negate(planes[j].normal), -planes[j].dist);
                }
                polygon.remove_colinear_points();
                if (polygon.size() < 3 || polygon.area() < minimum_face_area)
                    continue; // a redundant path plane does not bound this cell

                output_side side;
                side.plane = plane;
                side.polygon = std::move(polygon);
                side.texinfo = best_texinfo(side, faces);
                out.sides.push_back(std::move(side));
            }
            return out.sides.size() >= 4;
        }

        bool plane_already_used(const std::vector<plane_side> &planes,
                                const plane_side &candidate)
        {
            plane_side opposite{negate(candidate.normal), -candidate.dist};
            for (const plane_side &plane : planes)
            {
                if (same_plane(plane, candidate) || same_plane(plane, opposite))
                    return true;
            }
            return false;
        }

        bool find_texture_split(const output_brush &brush,
                                const std::vector<plane_side> &planes,
                                const std::vector<source_face> &faces,
                                plane_side &split)
        {
            for (const output_side &side : brush.sides)
            {
                if (side.texinfo < 0)
                    continue;
                for (const source_face &face : faces)
                {
                    if (face.texinfo < 0 || face.texinfo == side.texinfo
                        || !same_plane(side.plane, face.plane)
                        || overlap_area(side.polygon, face) <= minimum_face_area)
                    {
                        continue;
                    }

                    // a face edge becomes a plane perpendicular to the visible
                    // surface splitting the whole convex cell on that plane
                    // gives each texture region its own output brush side
                    for (int i = 0; i < face.polygon.size(); i++)
                    {
                        const vec3 &a = face.polygon[i];
                        const vec3 &b = face.polygon[(i + 1) % face.polygon.size()];
                        vec3 normal = math::cross(b - a, face.plane.normal);
                        if (math::normalize(normal) == 0)
                            continue;
                        plane_side candidate{normal, math::dot(normal, a)};
                        if (plane_already_used(planes, candidate))
                            continue;
                        if (side.polygon.on_plane_side(candidate.normal, candidate.dist)
                            != winding::side_cross)
                        {
                            continue;
                        }
                        split = candidate;
                        return true;
                    }
                }
            }
            return false;
        }

        void build_textured_cells(const std::vector<plane_side> &planes,
                                  const std::vector<source_face> &faces,
                                  int contents, int split_depth,
                                  std::vector<output_brush> &brushes,
                                  std::size_t &discarded,
                                  std::size_t &texture_splits,
                                  std::size_t &unresolved)
        {
            output_brush brush;
            if (!build_cell(planes, faces, contents, brush))
            {
                discarded++;
                return;
            }

            plane_side split;
            if (!find_texture_split(brush, planes, faces, split))
            {
                brushes.push_back(std::move(brush));
                return;
            }
            if (split_depth >= maximum_texture_split_depth)
            {
                unresolved++;
                brushes.push_back(std::move(brush));
                return;
            }

            texture_splits++;
            std::vector<plane_side> child = planes;
            add_unique_plane(child, {negate(split.normal), -split.dist});
            build_textured_cells(child, faces, contents, split_depth + 1,
                                 brushes, discarded, texture_splits, unresolved);
            child = planes;
            add_unique_plane(child, split);
            build_textured_cells(child, faces, contents, split_depth + 1,
                                 brushes, discarded, texture_splits, unresolved);
        }

        bool keep_leaf_contents(int contents)
        {
            // contents_empty is -1 every other negative leaf content describes
            // occupied space (solid, liquid, current or translucent)
            return contents <= -2;
        }

        bool walk_tree(const format::map_data &map, int node_index, int depth,
                       std::vector<plane_side> &path,
                       const std::vector<source_face> &faces,
                       std::vector<output_brush> &brushes,
                       std::size_t &discarded, std::size_t &texture_splits,
                       std::size_t &unresolved, std::string *error)
        {
            if (depth > maximum_tree_depth)
            {
                set_error(error, "hull-0 BSP tree exceeds the safe recursion depth");
                return false;
            }
            if (node_index < 0)
            {
                int leaf_index = -node_index - 1;
                if (leaf_index < 0 || (std::size_t)leaf_index >= map.leafs.size())
                {
                    set_error(error, "BSP node references an invalid leaf");
                    return false;
                }
                int contents = map.leafs[(std::size_t)leaf_index].contents;
                if (!keep_leaf_contents(contents))
                    return true;

                build_textured_cells(path, faces, contents, 0, brushes, discarded,
                                     texture_splits, unresolved);
                return true;
            }
            if ((std::size_t)node_index >= map.nodes.size())
            {
                set_error(error, "model references an invalid hull-0 node");
                return false;
            }

            const format::dnode_t &node = map.nodes[(std::size_t)node_index];
            if (node.planenum < 0 || (std::size_t)node.planenum >= map.planes.size())
            {
                set_error(error, "BSP node references an invalid plane");
                return false;
            }
            plane_side split = bsp_plane(map.planes[(std::size_t)node.planenum]);

            // children[0] is the front half space brush side normals point out
            // of the solid, therefore the front child's added side is -split
            std::size_t old_size = path.size();
            add_unique_plane(path, {negate(split.normal), -split.dist});
            if (!walk_tree(map, node.children[0], depth + 1, path, faces, brushes,
                           discarded, texture_splits, unresolved, error))
                return false;
            path.resize(old_size);

            add_unique_plane(path, split);
            if (!walk_tree(map, node.children[1], depth + 1, path, faces, brushes,
                           discarded, texture_splits, unresolved, error))
                return false;
            path.resize(old_size);
            return true;
        }

        void add_bounds(const format::dmodel_t &model, std::vector<plane_side> &path)
        {
            for (int axis = 0; axis < 3; axis++)
            {
                vec3 positive, negative;
                positive[axis] = 1;
                negative[axis] = -1;
                add_unique_plane(path, {positive, (double)model.maxs[axis] + bounds_padding});
                add_unique_plane(path, {negative, -(double)model.mins[axis] + bounds_padding});
            }
        }

        std::vector<std::string> texture_names(const format::map_data &map)
        {
            std::vector<std::string> names;
            if (map.textures.size() < sizeof(int))
                return names;
            binary::reader input(map.textures);
            std::int32_t count = 0;
            if (!input.i32(count))
                return names;
            if (count < 0 || (std::size_t)count > (map.textures.size() - sizeof(int)) / sizeof(int))
                return names;
            names.resize((std::size_t)count);
            for (int i = 0; i < count; i++)
            {
                std::int32_t offset = -1;
                if (!input.i32(offset))
                    return {};
                if (offset < 0 || (std::size_t)offset + sizeof(format::miptex_t) > map.textures.size())
                    continue;
                char name[17] = {};
                std::memcpy(name, map.textures.data() + offset, 16);
                names[(std::size_t)i] = name;
            }
            return names;
        }

        vec3 entity_origin(const format::entity &entity)
        {
            vec3 origin;
            std::sscanf(entity.value("origin"), "%lf %lf %lf", &origin.x, &origin.y, &origin.z);
            return origin;
        }

        int entity_model(const format::entity &entity, std::size_t entity_index)
        {
            if (entity_index == 0)
                return 0;
            const char *model = entity.value("model");
            if (model[0] != '*')
                return -1;
            char *end = nullptr;
            long value = std::strtol(model + 1, &end, 10);
            if (!end || *end != '\0' || value < 1 || value > std::numeric_limits<int>::max())
                return -1;
            return (int)value;
        }

        std::string escape_value(const std::string &value)
        {
            std::string out = value;
            std::replace(out.begin(), out.end(), '"', '\'');
            std::replace(out.begin(), out.end(), '\r', ' ');
            std::replace(out.begin(), out.end(), '\n', ' ');
            return out;
        }

        std::string number(double value)
        {
            if (std::fabs(value) < 0.00000005)
                value = 0;
            double rounded = std::round(value);
            if (std::fabs(value - rounded) < 0.000001)
                value = rounded;
            char buffer[64];
            std::snprintf(buffer, sizeof(buffer), "%.12g", value);
            return buffer;
        }

        const char *fallback_liquid_texture(int contents)
        {
            switch (contents)
            {
            case -4: return "!SLIME";
            case -5: return "!LAVA";
            case -9: return "!CUR_0";
            case -10: return "!CUR_90";
            case -11: return "!CUR_180";
            case -12: return "!CUR_270";
            case -13: return "!CUR_UP";
            case -14: return "!CUR_DWN";
            default: return "!WATER";
            }
        }

        bool is_liquid_contents(int contents)
        {
            return contents == -3 || contents == -4 || contents == -5
                || (contents >= -14 && contents <= -9);
        }

        struct texture_mapping
        {
            std::string name = "NULL";
            float vecs[2][4] = {
                {1, 0, 0, 0},
                {0, 1, 0, 0},
            };
        };

        texture_mapping mapping_for_texinfo(const format::map_data &map,
                                            const std::vector<std::string> &names,
                                            int texinfo)
        {
            texture_mapping mapping;
            if (texinfo < 0 || (std::size_t)texinfo >= map.texinfo.size())
                return mapping;
            const format::texinfo_t &source = map.texinfo[(std::size_t)texinfo];
            if (source.miptex < 0 || (std::size_t)source.miptex >= names.size()
                || names[(std::size_t)source.miptex].empty())
            {
                return mapping;
            }
            mapping.name = names[(std::size_t)source.miptex];
            std::memcpy(mapping.vecs, source.vecs, sizeof(mapping.vecs));
            return mapping;
        }

        void append_point(std::string &text, const vec3 &point, const vec3 &origin)
        {
            text += "( ";
            text += number(point.x + origin.x); text += ' ';
            text += number(point.y + origin.y); text += ' ';
            text += number(point.z + origin.z); text += " )";
        }

        void append_side(std::string &text, const output_side &side,
                         const texture_mapping &mapping, const vec3 &origin)
        {
            append_point(text, side.polygon[0], origin); text += ' ';
            append_point(text, side.polygon[1], origin); text += ' ';
            append_point(text, side.polygon[2], origin); text += ' ';
            text += mapping.name;
            for (int axis = 0; axis < 2; axis++)
            {
                double shift = mapping.vecs[axis][3]
                    - origin.x * mapping.vecs[axis][0]
                    - origin.y * mapping.vecs[axis][1]
                    - origin.z * mapping.vecs[axis][2];
                text += " [ ";
                text += number(mapping.vecs[axis][0]); text += ' ';
                text += number(mapping.vecs[axis][1]); text += ' ';
                text += number(mapping.vecs[axis][2]); text += ' ';
                text += number(shift); text += " ]";
            }
            text += " 0 1 1\n";
        }

        void append_brush(std::string &text, const output_brush &brush,
                          const format::map_data &map,
                          const std::vector<std::string> &names,
                          const vec3 &origin, map_result &result)
        {
            std::vector<texture_mapping> mappings;
            mappings.reserve(brush.sides.size());
            int liquid_mapping = -1;
            for (const output_side &side : brush.sides)
            {
                mappings.push_back(mapping_for_texinfo(map, names, side.texinfo));
                if (side.texinfo >= 0 && mappings.back().name != "NULL")
                    liquid_mapping = (int)mappings.size() - 1;
            }
            if (is_liquid_contents(brush.contents) && liquid_mapping < 0
                && !mappings.empty())
            {
                mappings[0].name = fallback_liquid_texture(brush.contents);
                liquid_mapping = 0;
            }

            text += "{\n";
            for (std::size_t i = 0; i < brush.sides.size(); i++)
            {
                texture_mapping mapping = mappings[i];
                // generated liquid boundaries must carry the liquid content as
                // well; null remains valid for generated solid boundaries
                if (is_liquid_contents(brush.contents)
                    && mapping.name == "NULL" && liquid_mapping >= 0)
                {
                    mapping = mappings[(std::size_t)liquid_mapping];
                }
                append_side(text, brush.sides[i], mapping, origin);
                if (brush.sides[i].texinfo >= 0)
                    result.textured_sides++;
                else
                    result.generated_sides++;
            }
            text += "}\n";
            result.brushes++;
        }

    }

    bool reconstruct_map(const format::map_data &map, const map_options &options,
                         map_result &result, std::string *error)
    {
        result = map_result{};
        if (map.models.empty())
        {
            set_error(error, "BSP contains no models");
            return false;
        }
        if (map.leafs.empty())
        {
            set_error(error, "BSP contains no leaves");
            return false;
        }

        std::vector<std::vector<output_brush>> model_brushes(map.models.size());
        for (std::size_t model_index = 0; model_index < map.models.size(); model_index++)
        {
            const format::dmodel_t &model = map.models[model_index];
            std::vector<source_face> faces;
            if (!collect_model_faces(map, model, faces, error))
                return false;
            std::vector<plane_side> path;
            add_bounds(model, path);
            if (!walk_tree(map, model.headnode[0], 0, path, faces,
                           model_brushes[model_index], result.discarded_cells,
                           result.texture_splits, result.unresolved_texture_boundaries, error))
            {
                return false;
            }
        }

        std::vector<format::entity> entities = format::parse_entities(map.entities);
        if (entities.empty())
        {
            set_error(error, "BSP contains no worldspawn entity");
            return false;
        }
        entities[0].set("mapversion", "220");
        if (!options.additional_wad.empty())
        {
            std::string wad = options.replace_wad_list ? std::string{} : entities[0].value("wad");
            if (!wad.empty() && wad.back() != ';')
                wad.push_back(';');
            wad += options.additional_wad;
            entities[0].set("wad", wad.c_str());
        }

        std::vector<std::string> names = texture_names(map);
        for (std::size_t entity_index = 0; entity_index < entities.size(); entity_index++)
        {
            format::entity &entity = entities[entity_index];
            int model_index = entity_model(entity, entity_index);
            if (model_index >= (int)map.models.size())
            {
                set_error(error, "entity references a model outside the model lump");
                return false;
            }
            if (model_index >= 0)
                entity.remove("model");

            result.text += "{\n";
            for (const format::entity::pair &pair : entity.pairs())
            {
                result.text += '"'; result.text += escape_value(pair.first);
                result.text += "\" \""; result.text += escape_value(pair.second);
                result.text += "\"\n";
            }
            if (model_index >= 0)
            {
                vec3 origin = entity_origin(entity);
                for (const output_brush &brush : model_brushes[(std::size_t)model_index])
                    append_brush(result.text, brush, map, names, origin, result);
            }
            result.text += "}\n";
        }
        return true;
    }
}
