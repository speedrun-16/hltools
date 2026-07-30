#include "map_post.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "../common/error.h"
#include "../common/filesystem.h"
#include "../common/log.h"
#include "../common/string_util.h"

namespace csg
{
    namespace
    {
        constexpr double engine_entity_range = 4096.0;
        constexpr size_t max_console_entity_origins = 10;

        struct entity_origin_outside_range
        {
            int number = 0;
            std::string classname;
            std::string targetname;
            math::vec3v origin;
        };

        brush_side &side_at(map_source &map, const map_brush &brush, int index)
        {
            return map.sides[(size_t)brush.first_side + index];
        }

        // whether every face of this entity should be nullified (kgp)
        bool check_for_invisible(const map_entity &entity, const map_post_options &options)
        {
            if (options.invisible_items.count(entity.value("classname")))
                return true;
            if (options.invisible_items.count(entity.value("targetname")))
                return true;
            const char *value = entity.value("zhlt_invisible");
            return value[0] && std::strcmp(value, "0") != 0;
        }

        // the reference's parsebrush tail renames, in its exact pass order
        void rename_special_sides(map_source &map,
                                  const map_brush &brush,
                                  bool nullify,
                                  const map_post_options &options)
        {
            if (nullify)
            {
                for (int i = 0; i < brush.num_sides; i++)
                {
                    brush_side &side = side_at(map, brush, i);
                    const char *name = side.texture.name;
                    if (!str::istarts_with(name, "BEVEL")
                        && !str::istarts_with(name, "ORIGIN")
                        && !str::istarts_with(name, "HINT")
                        && !str::istarts_with(name, "SKIP")
                        && !str::istarts_with(name, "SOLIDHINT")
                        && !str::istarts_with(name, "BEVELHINT")
                        && !str::istarts_with(name, "SPLITFACE")
                        && !str::istarts_with(name, "BOUNDINGBOX")
                        && !str::istarts_with(name, "CONTENT")
                        && !str::istarts_with(name, "SKY"))
                    {
                        str::copy(side.texture.name, sizeof(side.texture.name), "NULL");
                    }
                }
            }
            for (int i = 0; i < brush.num_sides; i++)
            {
                brush_side &side = side_at(map, brush, i);
                if (str::istarts_with(side.texture.name, "SPLITFACE"))
                    str::copy(side.texture.name, sizeof(side.texture.name), "SKIP");
            }
            for (int i = 0; i < brush.num_sides; i++)
            {
                brush_side &side = side_at(map, brush, i);
                if (str::istarts_with(side.texture.name, "CONTENT"))
                    str::copy(side.texture.name, sizeof(side.texture.name), "NULL");
            }
            if (options.brush.nullify_trigger)
            {
                for (int i = 0; i < brush.num_sides; i++)
                {
                    brush_side &side = side_at(map, brush, i);
                    if (str::istarts_with(side.texture.name, "AAATRIGGER"))
                        str::copy(side.texture.name, sizeof(side.texture.name), "NULL");
                }
            }
        }

        // fully builds an origin or boundingbox brush (contents forced solid,
        // zero origin offset) so its planes and texinfos register before any
        // normal brush, exactly like the reference's parse time createbrush
        // the geometry itself is thrown away; only hull 0 bounds are returned
        math::bounding_box build_special_brush_bounds(map_source &map,
                                                      const map_brush &brush,
                                                      plane_store &planes,
                                                      texinfo_store *texinfos,
                                                      const brush_build_options &brush_options)
        {
            map_brush solid_brush = brush;
            solid_brush.contents = content::solid;
            built_brush built = create_brush(map, solid_brush, planes, math::vec3v{},
                                             texinfos, brush_options);
            return built.hulls[0].bounds;
        }

        // duplicates a brush right behind its source, like the reference
        // copycurrentbrush appended it during parsing before the rest of the
        // entity's brushes existed later brushes of the entity shift one
        // brush number up, and every following entity's block moves right
        int copy_brush_after(map_source &map, int entity_num, int global_index)
        {
            map_entity &entity = map.entities[(size_t)entity_num];

            map_brush copy = map.brushes[(size_t)global_index];
            copy.first_side = (int)map.sides.size();
            for (int i = 0; i < copy.num_sides; i++)
                map.sides.push_back(map.sides[(size_t)map.brushes[(size_t)global_index].first_side + i]);
            copy.entity_num = entity_num;
            copy.brush_num = map.brushes[(size_t)global_index].brush_num + 1;

            int insert_at = global_index + 1;
            for (int i = insert_at; i < entity.first_brush + entity.num_brushes; i++)
                map.brushes[(size_t)i].brush_num++;
            map.brushes.insert(map.brushes.begin() + insert_at, std::move(copy));
            entity.num_brushes++;

            for (size_t e = 0; e < map.entities.size(); e++)
            {
                if ((int)e != entity_num && map.entities[e].first_brush >= insert_at)
                    map.entities[e].first_brush++;
            }
            return insert_at;
        }

        // removes one brush (zhlt_usemodel discards its geometry at parse time)
        void remove_brush(map_source &map, int entity_num, int global_index)
        {
            map.brushes.erase(map.brushes.begin() + global_index);
            map.entities[(size_t)entity_num].num_brushes--;
            for (size_t e = 0; e < map.entities.size(); e++)
            {
                if ((int)e != entity_num && map.entities[e].first_brush > global_index)
                    map.entities[e].first_brush--;
            }
        }

        // the reference parsebrush tail for one brush returns the number of
        // array slots consumed (2 when a clip copy was inserted, 0 when the
        // brush removed itself)
        int post_process_brush(map_source &map,
                               int entity_num,
                               int global_index,
                               bool nullify,
                               plane_store &planes,
                               texinfo_store *texinfos,
                               const map_post_options &options,
                               const brush_build_options &brush_options)
        {
            map_entity &entity = map.entities[(size_t)entity_num];
            map_brush &brush = map.brushes[(size_t)global_index];

            brush.contents = check_brush_contents(map, brush);
            rename_special_sides(map, brush, nullify, options);

            // origin brushes are removed later, but they set the rotation
            // origin for the rest of the brushes in the entity
            if (brush.contents == content::origin)
            {
                if (entity.value("origin")[0])
                {
                    err::fatal("entity %i, brush %i: only one origin brush allowed",
                               brush.original_entity_num, brush.original_brush_num);
                }
                math::bounding_box bounds = build_special_brush_bounds(
                    map, brush, planes, texinfos, brush_options);
                if (entity_num != 0)
                    set_origin_key(entity, (bounds.mins + bounds.maxs) * (vec_t)0.5);
            }

            if (entity.value("zhlt_usemodel")[0])
            {
                remove_brush(map, entity_num, global_index);
                return 0;
            }

            if (std::strcmp(entity.value("classname"), "info_hullshape") == 0)
                return 1;

            if (brush.contents == content::bounding_box)
            {
                if (entity.value("zhlt_minsmaxs")[0])
                {
                    err::fatal("entity %i, brush %i: only one boundingbox brush allowed",
                               brush.original_entity_num, brush.original_brush_num);
                }
                math::bounding_box bounds = build_special_brush_bounds(
                    map, brush, planes, texinfos, brush_options);
                if (entity_num != 0)
                    set_bounds_key(entity, bounds.mins, bounds.maxs);
            }

            if (options.sky_clip && brush.contents == content::sky && !map.brushes[(size_t)global_index].noclip)
            {
                int copy_index = copy_brush_after(map, entity_num, global_index);
                map_brush &copy = map.brushes[(size_t)copy_index];
                copy.contents = content::solid;
                copy.cliphull = ~0u;
                for (int i = 0; i < copy.num_sides; i++)
                    str::copy(side_at(map, copy, i).texture.name, texture_name_size, "NULL");
                return 2;
            }

            if (map.brushes[(size_t)global_index].cliphull != 0
                && map.brushes[(size_t)global_index].contents == content::to_empty)
            {
                map_brush &clip_brush = map.brushes[(size_t)global_index];

                // check for a mix of clip and normal textures invisible faces
                // become skip so hull 0 drops them
                bool mixed = false;
                for (int i = 0; i < clip_brush.num_sides; i++)
                {
                    brush_side &side = side_at(map, clip_brush, i);
                    if (str::istarts_with(side.texture.name, "NULL"))
                        str::copy(side.texture.name, sizeof(side.texture.name), "SKIP");
                    if (!str::istarts_with(side.texture.name, "SKIP"))
                        mixed = true;
                }

                int consumed = 1;
                if (mixed)
                {
                    int copy_index = copy_brush_after(map, entity_num, global_index);
                    map.brushes[(size_t)copy_index].cliphull = 0;
                    consumed = 2;
                }
                map_brush &original = map.brushes[(size_t)global_index];
                original.contents = content::solid;
                for (int i = 0; i < original.num_sides; i++)
                    str::copy(side_at(map, original, i).texture.name, texture_name_size, "NULL");
                return consumed;
            }

            return 1;
        }

        // moves a func_group or func_detail entity's brushes to the end of
        // worldspawn's block and deletes the entity, like the reference merge
        // (which ran while this entity was still the last one parsed)
        void merge_into_worldspawn(map_source &map, int entity_num)
        {
            map_entity &entity = map.entities[(size_t)entity_num];
            int new_brushes = entity.num_brushes;
            int world_brushes = map.entities[0].num_brushes;
            int first = entity.first_brush;

            std::vector<map_brush> moved(map.brushes.begin() + first,
                                         map.brushes.begin() + first + new_brushes);
            for (map_brush &brush : moved)
            {
                brush.entity_num = 0;
                brush.brush_num += world_brushes;
            }

            map.brushes.erase(map.brushes.begin() + first,
                              map.brushes.begin() + first + new_brushes);
            map.brushes.insert(map.brushes.begin() + world_brushes,
                               moved.begin(), moved.end());

            map.entities[0].num_brushes += new_brushes;
            for (int e = 1; e < entity_num; e++)
                map.entities[(size_t)e].first_brush += new_brushes;
            map.entities.erase(map.entities.begin() + entity_num);

            for (map_brush &brush : map.brushes)
            {
                if (brush.entity_num > entity_num)
                    brush.entity_num--;
            }
        }

        // registers the hull shape then deletes the entity and its brushes,
        // like the reference createhullshape + deletecurrententity
        void delete_hullshape_entity(map_source &map, int entity_num)
        {
            map_entity &entity = map.entities[(size_t)entity_num];
            int first = entity.first_brush;
            int count = entity.num_brushes;

            map.brushes.erase(map.brushes.begin() + first,
                              map.brushes.begin() + first + count);
            for (size_t e = 0; e < map.entities.size(); e++)
            {
                if (map.entities[e].first_brush > first)
                    map.entities[e].first_brush -= count;
            }
            map.entities.erase(map.entities.begin() + entity_num);

            for (map_brush &brush : map.brushes)
            {
                if (brush.entity_num > entity_num)
                    brush.entity_num--;
            }
        }

        // the reference parsemapentity tail returns true when the entity was
        // deleted (merged or hullshape) so the walk stays on the same index
        bool post_process_entity(map_source &map,
                                 int entity_num,
                                 int parsed_num,
                                 plane_store &planes,
                                 texinfo_store *texinfos,
                                 hull_shape_library &hull_shapes,
                                 const map_post_options &options,
                                 const brush_build_options &brush_options,
                                 std::vector<entity_origin_outside_range> &outside_origins)
        {
            {
                map_entity &entity = map.entities[(size_t)entity_num];
                bool nullify = check_for_invisible(entity, options);

                int index = entity.first_brush;
                while (index < entity.first_brush + entity.num_brushes)
                {
                    index += post_process_brush(map, entity_num, index, nullify,
                                                planes, texinfos, options, brush_options);
                }

                if (entity.value("zhlt_usemodel")[0])
                {
                    if (!entity.value("origin")[0])
                    {
                        logging::warn("entity %i: 'zhlt_usemodel' requires the entity to have an origin brush",
                                      parsed_num);
                    }
                    entity.num_brushes = 0;
                }
            }

            apply_entity_transform(map, entity_num, brush_options);

            map_entity &entity = map.entities[(size_t)entity_num];
            const char *classname = entity.value("classname");

            if (str::iequals(classname, "info_compile_parameters"))
            {
                if (options.compile_parameters_consumed)
                {
                    // unified compilation already supplied every stage with
                    // its defaults; entity only csg also writes a final bsp
                    delete_hullshape_entity(map, entity_num);
                    return true;
                }
                // individual csg applies csg_* now and carries the remaining
                // recipe in its intermediate bsp for bsp, vis and rad
                return false;
            }

            entity.origin = math::vec3v{};
            std::sscanf(entity.value("origin"), "%lf %lf %lf",
                        &entity.origin.x, &entity.origin.y, &entity.origin.z);

            if (std::strcmp(classname, "func_group") == 0
                || std::strcmp(classname, "func_detail") == 0)
            {
                merge_into_worldspawn(map, entity_num);
                return true;
            }

            if (std::strcmp(classname, "info_hullshape") == 0)
            {
                add_hull_shape(map, entity_num, hull_shapes);
                delete_hullshape_entity(map, entity_num);
                return true;
            }

            if (std::fabs(entity.origin.x) > engine_entity_range + math::on_epsilon
                || std::fabs(entity.origin.y) > engine_entity_range + math::on_epsilon
                || std::fabs(entity.origin.z) > engine_entity_range + math::on_epsilon)
            {
                if (std::strncmp(classname, "light", 5) != 0)
                {
                    entity_origin_outside_range warning;
                    warning.number = parsed_num;
                    warning.classname = classname;
                    warning.targetname = entity.value("targetname");
                    warning.origin = entity.origin;
                    outside_origins.push_back(std::move(warning));
                }
            }
            return false;
        }

        void report_entity_origins(const std::vector<entity_origin_outside_range> &warnings,
                                   size_t count, bool console_output)
        {
            for (size_t i = 0; i < count; i++)
            {
                const entity_origin_outside_range &warning = warnings[i];
                if (console_output)
                    logging::console("\n    entity %d  %s", warning.number, warning.classname.c_str());
                else
                    logging::file("\n    entity %d  %s", warning.number, warning.classname.c_str());
                if (!warning.targetname.empty())
                {
                    if (console_output)
                        logging::console("  targetname \"%s\"", warning.targetname.c_str());
                    else
                        logging::file("  targetname \"%s\"", warning.targetname.c_str());
                }
                if (console_output)
                {
                    logging::console("\n      origin  (%.0f, %.0f, %.0f)\n",
                                     (double)warning.origin.x, (double)warning.origin.y,
                                     (double)warning.origin.z);
                }
                else
                {
                    logging::file("\n      origin  (%.0f, %.0f, %.0f)\n",
                                  (double)warning.origin.x, (double)warning.origin.y,
                                  (double)warning.origin.z);
                }
            }
        }

        void print_entity_origin_warnings(const std::vector<entity_origin_outside_range> &warnings)
        {
            if (warnings.empty())
                return;

            logging::info("\n  !!! WARNING: ENTITY ORIGINS OUTSIDE ENGINE RANGE\n\n");
            logging::info("    found    %zu %s outside the +/-%.0f unit range\n",
                          warnings.size(), warnings.size() == 1 ? "entity" : "entities",
                          engine_entity_range);
            logging::info("    limit    standard GoldSrc coordinate messages cover about +/-%.0f units\n",
                          engine_entity_range);
            logging::info("    action   move the listed entities inside the engine range\n");

            size_t console_count = std::min(warnings.size(), max_console_entity_origins);
            report_entity_origins(warnings, console_count, true);
            if (console_count < warnings.size())
                logging::console("\n    (%zu more entities; see the logfile)\n", warnings.size() - console_count);
            logging::console("\n");

            report_entity_origins(warnings, warnings.size(), false);
            logging::file(
                "\n  Standard GoldSrc coordinate messages store positions as signed 16 bit values at\n"
                "  1/8 unit precision. Origins outside this range can wrap when sent to clients.\n");

            char summary[160];
            std::snprintf(summary, sizeof(summary),
                          "%zu entity %s outside the +/-%.0f unit engine range",
                          warnings.size(), warnings.size() == 1 ? "origin" : "origins",
                          engine_entity_range);
            logging::add_warning_summary(summary);
        }
    }

    std::set<std::string> load_invisible_items(const std::string &path)
    {
        std::vector<unsigned char> bytes;
        if (!fs::read_all(path, bytes))
            err::fatal("could not find null entity list file '%s'", path.c_str());
        logging::info("Loading null entity list from '%s'\n", path.c_str());

        std::set<std::string> items;
        std::string line;
        for (size_t i = 0; i <= bytes.size(); i++)
        {
            if (i < bytes.size() && bytes[i] != '\n' && bytes[i] != '\r')
            {
                line.push_back((char)bytes[i]);
                continue;
            }
            if (!line.empty())
                items.insert(line);
            line.clear();
        }
        return items;
    }

    void post_process_map(map_source &map,
                          plane_store &planes,
                          texinfo_store *texinfos,
                          hull_shape_library &hull_shapes,
                          const map_post_options &options)
    {
        for (int h = 0; h < num_hulls; h++)
            hull_shapes.default_hulls[h].disabled = true;

        brush_build_options brush_options = options.brush;
        brush_options.hull_shapes = &hull_shapes;

        int parsed_num = 0;
        int entity_num = 0;
        std::vector<entity_origin_outside_range> outside_origins;
        while (entity_num < (int)map.entities.size())
        {
            bool deleted = post_process_entity(map, entity_num, parsed_num, planes,
                                               texinfos, hull_shapes, options, brush_options,
                                               outside_origins);
            parsed_num++;
            if (!deleted)
                entity_num++;
        }
        print_entity_origin_warnings(outside_origins);
    }
}
