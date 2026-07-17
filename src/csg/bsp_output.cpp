#include "bsp_output.h"

#include <algorithm>
#include <cstdio>
#include <utility>
#include <vector>

#include "../common/error.h"
#include "../common/filesystem.h"
#include "../common/string_util.h"
#include "format/bsp/entity_lump.h"
#include "entity_output_fixups.h"

namespace csg
{
    namespace
    {
        int content_value(content value)
        {
            return (int)value;
        }

        struct face_fragment
        {
            int planenum = -1;
            int texinfo = -1;
            content contents = content::empty;
            content back_contents = content::empty;
            math::winding winding;
            math::bounding_box bounds;
        };

        struct plane_file_record
        {
            vec_t normal[3] = {};
            vec_t origin[3] = {};
            vec_t dist = 0;
            int type = 0;
            int pad = 0;
        };

        static_assert(sizeof(plane_file_record) == 64, "csg .pln plane record must match legacy plane_t");

        const char *texture_name(const csg_result &result, int texinfo)
        {
            if (texinfo < 0 || texinfo >= (int)result.texinfos.entries().size())
                return "";
            return result.texinfos.entries()[(size_t)texinfo].texture_name.c_str();
        }

        bool is_hint_texture(const char *name)
        {
            return str::istarts_with(name, "skip")
                || str::istarts_with(name, "hint")
                || str::istarts_with(name, "solidhint")
                || str::istarts_with(name, "bevelhint");
        }

        bool is_solid_hint_texture(const char *name)
        {
            return str::istarts_with(name, "solidhint")
                || str::istarts_with(name, "bevelhint");
        }

        void update_bounds(face_fragment &face)
        {
            face.winding.bounds(face.bounds);
        }

        format::entity to_format_entity(const map_entity &source)
        {
            format::entity out;
            for (auto it = source.pairs.rbegin(); it != source.pairs.rend(); ++it)
                out.prepend(it->first, it->second);
            return out;
        }

        std::vector<format::entity> output_entities(const map_source &map, bool optimize_lights)
        {
            std::vector<format::entity> out;
            out.reserve(map.entities.size());
            for (const map_entity &entity : map.entities)
                out.push_back(to_format_entity(entity));
            apply_entity_output_fixups(out, optimize_lights);
            return out;
        }

        void set_model_numbers(map_source &map)
        {
            int models = 1;
            for (size_t i = 1; i < map.entities.size(); i++)
            {
                if (map.entities[i].num_brushes)
                {
                    char value[10];
                    str::format(value, sizeof(value), "*%i", models);
                    models++;
                    map.entities[i].set_value("model", value);
                }
            }
        }

        // gives zhlt_usemodel entities the model of their target and moves
        // them behind it (the reference does this to keep precache order and
        // entityformodel lookups working for map models)
        void reuse_models(map_source &map)
        {
            for (int i = (int)map.entities.size() - 1; i >= 1; i--)
            {
                std::string name = map.entities[(size_t)i].value("zhlt_usemodel");
                if (name.empty())
                    continue;
                int j;
                for (j = 1; j < (int)map.entities.size(); j++)
                {
                    if (map.entities[(size_t)j].value("zhlt_usemodel")[0])
                        continue;
                    if (name == map.entities[(size_t)j].value("targetname"))
                        break;
                }
                if (j == (int)map.entities.size())
                {
                    if (str::iequals(name.c_str(), "null"))
                    {
                        map.entities[(size_t)i].set_value("model", "");
                        continue;
                    }
                    err::fatal("zhlt_usemodel: can not find target entity '%s', "
                               "or that entity is also using 'zhlt_usemodel'",
                               name.c_str());
                }
                map.entities[(size_t)i].set_value("model", map.entities[(size_t)j].value("model"));
                if (j > i)
                {
                    std::rotate(map.entities.begin() + i,
                                map.entities.begin() + i + 1,
                                map.entities.begin() + j + 1);
                }
            }
        }

        // "style" key sentinels a level designer can put on a non light entity
        // to make it behave as a texlight, values from the reference
        enum class texlight_style : int
        {
            none = 0,                  // not a light, no style
            switchable = -1,           // normal switchable texlight
            switchable_backwards = -2, // backwards switchable texlight
            piggyback = -3,            // switched by a real light with the same name
        };

        // engine styles below this are reserved (0 = normal, 1 through 31 = animated)
        constexpr int first_switched_style = 32;

        // any controlled light needs a unique style number also allocates
        // styles for switchable texlights and resets piggyback texlights,
        // like the reference setlightstyles
        void set_light_styles(map_source &map)
        {
            constexpr int max_switched_lights = 32;

            std::string light_targets[max_switched_lights];
            int style_count = 0;
            char value[10];

            for (size_t i = 1; i < map.entities.size(); i++)
            {
                map_entity &entity = map.entities[i];
                const char *target = entity.value("classname");
                if (!str::istarts_with(target, "light"))
                {
                    switch ((texlight_style)entity.int_value("style"))
                    {
                    case texlight_style::none:
                        continue;
                    case texlight_style::switchable:
                        str::format(value, sizeof(value), "%i", first_switched_style + style_count);
                        entity.set_value("style", value);
                        style_count++;
                        continue;
                    case texlight_style::switchable_backwards:
                        str::format(value, sizeof(value), "%i", -(first_switched_style + style_count));
                        entity.set_value("style", value);
                        style_count++;
                        continue;
                    case texlight_style::piggyback:
                        entity.set_value("style", "0");
                        break;
                    default:
                        break;
                    }
                }
                target = entity.value("targetname");
                if (entity.value("zhlt_usestyle")[0])
                {
                    target = entity.value("zhlt_usestyle");
                    if (str::iequals(target, "null"))
                        target = "";
                }
                if (!target[0])
                    continue;

                int j;
                for (j = 0; j < style_count; j++)
                {
                    if (light_targets[j] == target)
                        break;
                }
                if (j == style_count)
                {
                    if (style_count >= max_switched_lights)
                        err::fatal("exceeded max_switched_lights");
                    light_targets[j] = target;
                    style_count++;
                }
                str::format(value, sizeof(value), "%i", first_switched_style + j);
                entity.set_value("style", value);
            }
        }

        std::vector<int> sorted_brush_indexes(const built_entity &entity)
        {
            std::vector<int> indexes;
            indexes.reserve(entity.brushes.size());
            for (int i = 0; i < (int)entity.brushes.size(); i++)
            {
                if (!entity.brushes[(size_t)i].skipped)
                    indexes.push_back(i);
            }

            std::stable_sort(indexes.begin(), indexes.end(),
                             [&entity](int a, int b)
                             {
                                 return content_value(entity.brushes[(size_t)a].contents)
                                     < content_value(entity.brushes[(size_t)b].contents);
                             });
            return indexes;
        }

        bool write_text(const std::string &path, const std::string &text)
        {
            std::string out;
            out.reserve(text.size());
            for (char ch : text)
            {
                if (ch == '\n')
                    out += '\r';
                out += ch;
            }
            return fs::write_all(path, out.data(), out.size());
        }

        void append_face(std::string &out,
                         int detail_level,
                         int planenum,
                         int texinfo,
                         content contents,
                         const math::winding &winding)
        {
            char line[256];
            str::format(line, sizeof(line), "%i %i %i %i %u\n",
                        detail_level, planenum, texinfo, content_value(contents),
                        (unsigned)winding.size());
            out += line;
            for (const math::vec3v &point : winding.points())
            {
                str::format(line, sizeof(line), "%5.8f %5.8f %5.8f\n",
                            (double)point.x, (double)point.y, (double)point.z);
                out += line;
            }
            out += "\n";
        }

        void append_mirrored_face(std::string &out,
                                  int detail_level,
                                  face_fragment face,
                                  int texinfo,
                                  content contents)
        {
            math::winding reversed;
            for (int i = face.winding.size() - 1; i >= 0; i--)
                reversed.add_point(face.winding[i]);
            append_face(out, detail_level, face.planenum ^ 1, texinfo, contents, reversed);
        }

        void append_detail_brush(std::string &out, const brush_hull &hull)
        {
            out += "0\n";
            for (const brush_face &face : hull.faces)
            {
                char line[128];
                str::format(line, sizeof(line), "%i %u\n", face.planenum, (unsigned)face.winding.size());
                out += line;
                for (const math::vec3v &point : face.winding.points())
                {
                    str::format(line, sizeof(line), "%5.8f %5.8f %5.8f\n",
                                (double)point.x, (double)point.y, (double)point.z);
                    out += line;
                }
            }
            out += "-1 -1\n";
        }

        void validate_detail_brush(const built_brush &brush, int hull)
        {
            int detail_level = hull ? brush.clipnode_detail_level : brush.detail_level;
            if (!detail_level || brush.hulls[hull].faces.empty())
                return;

            switch (brush.contents)
            {
            case content::origin:
            case content::bounding_box:
            case content::hint:
            case content::to_empty:
            case content::solid:
                return;
            default:
                err::fatal("entity %i, brush %i: %s brushes not allowed in detail",
                           brush.original_entity_num, brush.original_brush_num,
                           content_to_string(brush.contents));
            }
        }

        void validate_detail_brushes(const csg_result &result, int hull)
        {
            for (const built_entity &entity : result.entities)
            {
                for (const built_brush &brush : entity.brushes)
                    validate_detail_brush(brush, hull);
            }
        }

        std::vector<face_fragment> copy_faces_to_outside(const brush_hull &hull)
        {
            std::vector<face_fragment> out;
            for (const brush_face &face : hull.faces)
            {
                face_fragment copy;
                copy.planenum = face.planenum;
                copy.texinfo = face.texinfo;
                copy.contents = face.contents;
                copy.back_contents = face.back_contents;
                copy.winding = face.winding;
                update_bounds(copy);
                out.insert(out.begin(), std::move(copy));
            }
            return out;
        }

        bool can_chop(const built_brush &brush, const built_brush &cutter, int hull)
        {
            if (hull)
                return cutter.clipnode_detail_level <= brush.clipnode_detail_level;
            return cutter.detail_level - cutter.chop_down <= brush.detail_level + brush.chop_up;
        }

        bool cutter_has_higher_detail(const built_brush &brush, const built_brush &cutter, int hull)
        {
            if (hull)
                return cutter.clipnode_detail_level > brush.clipnode_detail_level;
            return cutter.detail_level > brush.detail_level;
        }

        bool cutter_has_lower_detail(const built_brush &brush, const built_brush &cutter, int hull)
        {
            if (hull)
                return cutter.clipnode_detail_level < brush.clipnode_detail_level;
            return cutter.detail_level < brush.detail_level;
        }

        bool same_content_different_detail(const built_brush &brush, const built_brush &cutter, int hull)
        {
            if (brush.contents != cutter.contents)
                return false;
            if (hull)
                return cutter.clipnode_detail_level != brush.clipnode_detail_level;
            return cutter.detail_level != brush.detail_level;
        }

        bool overwrite_for_cutter(const built_brush &brush,
                                  const built_brush &cutter,
                                  int brush_index,
                                  int cutter_index,
                                  int hull)
        {
            bool overwrite = cutter_index > brush_index;
            if (same_content_different_detail(brush, cutter, hull))
                overwrite = cutter_has_lower_detail(brush, cutter, hull);
            if (brush.contents == cutter.contents
                && hull == 0
                && cutter.detail_level == brush.detail_level
                && cutter.coplanar_priority != brush.coplanar_priority)
            {
                overwrite = cutter.coplanar_priority > brush.coplanar_priority;
            }
            return overwrite;
        }

        bool face_inside_cutter(const csg_result &result,
                                face_fragment &face,
                                const built_brush &cutter,
                                int hull,
                                bool overwrite,
                                math::winding &inside)
        {
            inside = face.winding;
            for (const brush_face &cut : cutter.hulls[hull].faces)
            {
                if (face.planenum == cut.planenum)
                {
                    if (!overwrite)
                    {
                        inside = {};
                        break;
                    }
                    continue;
                }
                if (face.planenum == (cut.planenum ^ 1))
                    continue;

                math::winding front;
                math::winding back;
                const brush_plane &plane = result.planes[(size_t)cut.planenum];
                inside.clip(plane.normal, plane.dist, front, back);
                if (!back.empty())
                {
                    inside = std::move(back);
                }
                else
                {
                    inside = {};
                    break;
                }
            }
            return !inside.empty();
        }

        void carve_face_by_cutter(const csg_result &result,
                                  const built_brush &brush,
                                  const built_brush &cutter,
                                  face_fragment face,
                                  int hull,
                                  bool overwrite,
                                  std::vector<face_fragment> &outside)
        {
            if (cutter_has_higher_detail(brush, cutter, hull))
            {
                const char *name = texture_name(result, face.texinfo);
                if (face.texinfo == -1 || is_hint_texture(name))
                {
                    outside.insert(outside.begin(), std::move(face));
                    return;
                }
            }

            math::winding inside;
            if (!face_inside_cutter(result, face, cutter, hull, overwrite, inside))
            {
                outside.insert(outside.begin(), std::move(face));
                return;
            }

            for (const brush_face &cut : cutter.hulls[hull].faces)
            {
                if (face.planenum == cut.planenum || face.planenum == (cut.planenum ^ 1))
                    continue;
                const brush_plane &plane = result.planes[(size_t)cut.planenum];
                int valid = 0;
                for (const math::vec3v &point : inside.points())
                {
                    vec_t dist = math::dot(point, plane.normal) - plane.dist;
                    if (dist >= (vec_t)(-math::on_epsilon * 4))
                        valid++;
                }
                if (valid < 2)
                    continue;

                math::winding front;
                math::winding back;
                face.winding.clip(plane.normal, plane.dist, front, back);
                if (!front.empty())
                {
                    face_fragment front_face = face;
                    front_face.winding = std::move(front);
                    update_bounds(front_face);
                    outside.insert(outside.begin(), std::move(front_face));
                }
                if (!back.empty())
                {
                    face.winding = std::move(back);
                    update_bounds(face);
                }
                else
                {
                    return;
                }
            }

            if (face.winding.area() < 0)
                return;

            if (cutter_has_higher_detail(brush, cutter, hull))
            {
                face.texinfo = -1;
                outside.insert(outside.begin(), std::move(face));
                return;
            }
            if (cutter_has_lower_detail(brush, cutter, hull) && cutter.contents == content::solid)
                return;

            if (brush.contents == content::to_empty)
            {
                bool on_front = true;
                bool on_back = true;
                for (const brush_face &cut : cutter.hulls[hull].faces)
                {
                    if (face.planenum == (cut.planenum ^ 1))
                        on_back = false;
                    if (face.planenum == cut.planenum)
                        on_front = false;
                }
                if (on_front && content_value(face.contents) < content_value(cutter.contents))
                    face.contents = cutter.contents;
                if (on_back && content_value(face.back_contents) < content_value(cutter.contents))
                    face.back_contents = cutter.contents;
                if (face.contents == content::solid
                    && face.back_contents == content::solid
                    && !is_solid_hint_texture(texture_name(result, face.texinfo)))
                {
                    return;
                }
                outside.insert(outside.begin(), std::move(face));
                return;
            }

            if (content_value(brush.contents) > content_value(cutter.contents)
                || (brush.contents == cutter.contents
                    && is_solid_hint_texture(texture_name(result, face.texinfo))))
            {
                face.contents = cutter.contents;
                outside.insert(outside.begin(), std::move(face));
            }
        }

        // carves one brush against the others the reference physically re sorts
        // each entity's brushes ascending by contents before csg ("sort the
        // contents down so stone bites water"), so both the cutter iteration
        // order and the position comparison feeding overwrite run on sorted
        // positions, not parse order brush_order is that sorted sequence and
        // order_pos is this brush's position within it
        std::vector<face_fragment> carve_brush_faces(const csg_result &result,
                                                     const built_entity &entity,
                                                     const std::vector<int> &brush_order,
                                                     int order_pos,
                                                     int hull)
        {
            const built_brush &brush = entity.brushes[(size_t)brush_order[(size_t)order_pos]];
            std::vector<face_fragment> outside = copy_faces_to_outside(brush.hulls[hull]);
            if (brush.contents == content::to_empty)
            {
                for (face_fragment &face : outside)
                {
                    face.contents = content::to_empty;
                    face.back_contents = content::to_empty;
                }
            }

            for (int cutter_pos = 0; cutter_pos < (int)brush_order.size(); cutter_pos++)
            {
                if (cutter_pos == order_pos)
                    continue;
                const built_brush &cutter = entity.brushes[(size_t)brush_order[(size_t)cutter_pos]];
                if (cutter.contents == content::to_empty)
                    continue;
                if (!can_chop(brush, cutter, hull))
                    continue;
                if (cutter.hulls[hull].faces.empty())
                    continue;
                if (brush.hulls[hull].bounds.disjoint(cutter.hulls[hull].bounds))
                    continue;

                bool overwrite = overwrite_for_cutter(brush, cutter, order_pos, cutter_pos, hull);
                std::vector<face_fragment> current = std::move(outside);
                outside.clear();
                for (face_fragment &face : current)
                {
                    if (cutter.hulls[hull].bounds.disjoint(face.bounds))
                    {
                        outside.insert(outside.begin(), std::move(face));
                        continue;
                    }
                    carve_face_by_cutter(result, brush, cutter, std::move(face), hull, overwrite, outside);
                }
            }
            return outside;
        }

        void append_saved_faces(std::string &out,
                                const csg_result &result,
                                int entity_num,
                                const built_brush &brush,
                                std::vector<face_fragment> faces,
                                int hull)
        {
            int detail_level = hull ? brush.clipnode_detail_level : brush.detail_level;
            for (face_fragment &face : faces)
            {
                int texinfo = face.texinfo;
                const char *name = texture_name(result, texinfo);
                content front_contents = face.contents;
                content back_contents = brush.contents == content::to_empty
                    ? face.back_contents
                    : brush.contents;
                if (front_contents == content::to_empty)
                    front_contents = content::empty;
                if (back_contents == content::to_empty)
                    back_contents = content::empty;

                bool front_null = false;
                bool back_null = false;
                if (brush.contents == content::to_empty && !is_hint_texture(name))
                    back_null = true;
                if (is_solid_hint_texture(name) && front_contents != back_contents)
                {
                    front_null = true;
                    back_null = true;
                }
                if (entity_num != 0 && str::istarts_with(name, "!"))
                    back_null = true;

                face.contents = front_contents;
                face.texinfo = front_null ? -1 : texinfo;
                if (face.winding.area() < 0)
                    continue;

                append_face(out, detail_level, face.planenum, face.texinfo, face.contents, face.winding);
                append_mirrored_face(out, detail_level, face, back_null ? -1 : texinfo, back_contents);
            }
        }

        std::string hull_surface_file(const csg_result &result, int hull)
        {
            std::string out;
            for (const built_entity &entity : result.entities)
            {
                if (entity.entity_num < 0
                    || entity.entity_num >= (int)result.map.entities.size()
                    || result.map.entities[(size_t)entity.entity_num].num_brushes <= 0)
                    continue;

                std::vector<int> brush_order = sorted_brush_indexes(entity);
                for (int order_pos = 0; order_pos < (int)brush_order.size(); order_pos++)
                {
                    const built_brush &brush = entity.brushes[(size_t)brush_order[(size_t)order_pos]];
                    std::vector<face_fragment> faces =
                        carve_brush_faces(result, entity, brush_order, order_pos, hull);
                    append_saved_faces(out, result, entity.entity_num, brush, std::move(faces), hull);
                }
                out += "-1 -1 -1 -1 -1\n";
            }
            return out;
        }

        std::string hull_detail_file(const csg_result &result, int hull)
        {
            std::string out;
            for (const built_entity &entity : result.entities)
            {
                if (entity.entity_num < 0
                    || entity.entity_num >= (int)result.map.entities.size()
                    || result.map.entities[(size_t)entity.entity_num].num_brushes <= 0)
                    continue;

                for (int brush_index : sorted_brush_indexes(entity))
                {
                    const built_brush &brush = entity.brushes[(size_t)brush_index];
                    int detail_level = hull ? brush.clipnode_detail_level : brush.detail_level;
                    if (detail_level && brush.contents == content::solid && !brush.hulls[hull].faces.empty())
                        append_detail_brush(out, brush.hulls[hull]);
                }
                out += "-1\n";
            }
            return out;
        }

        std::vector<byte> plane_file_bytes(const std::vector<brush_plane> &planes)
        {
            if (planes.empty())
                return {};

            std::vector<plane_file_record> records;
            records.reserve(planes.size());
            for (const brush_plane &plane : planes)
            {
                plane_file_record record;
                record.normal[0] = plane.normal.x;
                record.normal[1] = plane.normal.y;
                record.normal[2] = plane.normal.z;
                record.origin[0] = plane.origin.x;
                record.origin[1] = plane.origin.y;
                record.origin[2] = plane.origin.z;
                record.dist = plane.dist;
                record.type = (int)plane.type;
                records.push_back(record);
            }

            const byte *begin = reinterpret_cast<const byte *>(records.data());
            return std::vector<byte>(begin, begin + records.size() * sizeof(plane_file_record));
        }
    }

    void replace_entities(csg_result &result, format::map_data &existing, bool optimize_lights)
    {
        set_model_numbers(result.map);
        reuse_models(result.map);
        set_light_styles(result.map);

        // keep the existing bsp's wad key, like the reference loadwadvalue
        std::vector<format::entity> old_entities = format::parse_entities(existing.entities);
        const char *wad_value = old_entities.empty() ? "" : old_entities[0].value("wad");
        if (!result.map.entities.empty())
            result.map.entities[0].set_value("wad", wad_value);

        existing.entities = format::write_entities(output_entities(result.map, optimize_lights));
    }

    format::map_data build_bsp_data(csg_result &result, bool optimize_lights)
    {
        // the reference writebsp entity passes, in order: model numbers, then
        // zhlt_usemodel resolution (may reorder entities), then light styles
        set_model_numbers(result.map);
        reuse_models(result.map);
        set_light_styles(result.map);

        format::map_data data;
        data.planes.reserve(result.planes.size());
        for (const brush_plane &plane : result.planes)
        {
            format::dplane_t out = {};
            out.normal[0] = (float)plane.normal.x;
            out.normal[1] = (float)plane.normal.y;
            out.normal[2] = (float)plane.normal.z;
            out.dist = (float)plane.dist;
            out.type = (int)plane.type;
            data.planes.push_back(out);
        }

        data.texinfo.reserve(result.texinfos.entries().size());
        for (const texinfo_entry &entry : result.texinfos.entries())
            data.texinfo.push_back(entry.info);

        data.textures = result.textures;
        std::vector<format::entity> entities = output_entities(result.map, optimize_lights);
        data.entities = format::write_entities(entities);
        return data;
    }

    intermediate_data build_intermediate_data(const csg_result &result,
                                              const brush_build_options &options)
    {
        intermediate_data data;
        for (int hull = 0; hull < num_hulls; hull++)
        {
            validate_detail_brushes(result, hull);
            data.surfaces[hull] = hull_surface_file(result, hull);
            data.brushes[hull] = hull_detail_file(result, hull);
        }

        for (int hull = 0; hull < num_hulls; hull++)
        {
            char line[192];
            str::format(line, sizeof(line), "%g %g %g %g %g %g\n",
                        (double)options.hull_size[hull][0].x,
                        (double)options.hull_size[hull][0].y,
                        (double)options.hull_size[hull][0].z,
                        (double)options.hull_size[hull][1].x,
                        (double)options.hull_size[hull][1].y,
                        (double)options.hull_size[hull][1].z);
            data.hull_sizes += line;
        }

        data.planes = plane_file_bytes(result.planes);
        return data;
    }

    bool write_intermediate_files(const std::string &base_path,
                                  const intermediate_data &data,
                                  const csg_result &result)
    {
        for (int hull = 0; hull < num_hulls; hull++)
        {
            if (!write_text(base_path + ".p" + std::to_string(hull), data.surfaces[hull]))
                return false;
            if (!write_text(base_path + ".b" + std::to_string(hull), data.brushes[hull]))
                return false;
        }
        if (!write_text(base_path + ".hsz", data.hull_sizes))
            return false;
        if (!fs::write_all(base_path + ".pln", data.planes.data(), data.planes.size()))
            return false;
        return fs::write_all(base_path + ".wa_", result.temp_wad.data(), result.temp_wad.size());
    }

    bool write_intermediate_files(const std::string &base_path,
                                  const csg_result &result,
                                  const brush_build_options &options)
    {
        return write_intermediate_files(base_path, build_intermediate_data(result, options), result);
    }
}
