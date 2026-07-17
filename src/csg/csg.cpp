#include "csg.h"

#include <cstring>
#include <utility>

#include "../common/log.h"
#include "../common/progress.h"
#include "../common/string_util.h"
#include "brush_union.h"
#include "hull_file.h"

namespace csg
{
    namespace
    {
        // writes a model_center key for zhlt_flags light_origin entities from
        // the hull 0 bounds of their visible brushes, like the reference
        // setmodelcenters
        void set_model_center(csg_result &result, int entity_num)
        {
            map_entity &entity = result.map.entities[(size_t)entity_num];
            if (entity_num == 0 || entity.num_brushes == 0)
                return;
            if (!entity.value("light_origin")[0])
                return;

            math::bounding_box bounds;
            for (const built_brush &brush : result.entities[(size_t)entity_num].brushes)
            {
                if (brush.contents != content::origin && brush.contents != content::bounding_box)
                    bounds.add(brush.hulls[0].bounds);
            }

            math::vec3v center = (bounds.mins + bounds.maxs) * (vec_t)0.5;
            char value[64];
            str::format(value, sizeof(value), "%i %i %i",
                        (int)center.x, (int)center.y, (int)center.z);
            entity.set_value("model_center", value);
        }
    }

    bool parse_clip_type(const char *value, clip_type &out)
    {
        if (!value)
            return false;
        if (str::iequals(value, "smallest"))
            out = clip_type::smallest;
        else if (str::iequals(value, "normalized"))
            out = clip_type::normalized;
        else if (str::iequals(value, "simple"))
            out = clip_type::simple;
        else if (str::iequals(value, "precise"))
            out = clip_type::precise;
        else if (str::iequals(value, "legacy"))
            out = clip_type::legacy;
        else
            return false;
        return true;
    }

    const char *clip_type_name(clip_type type)
    {
        switch (type)
        {
        case clip_type::smallest:
            return "smallest";
        case clip_type::normalized:
            return "normalized";
        case clip_type::simple:
            return "simple";
        case clip_type::precise:
            return "precise";
        case clip_type::legacy:
            return "legacy";
        default:
            return "unknown";
        }
    }

    csg_result run_csg(map_source map, const csg_options &options)
    {
        csg_result result;
        result.map = std::move(map);

        brush_build_options brush_options = options.brush;
        if (!options.hull_file_path.empty())
            load_hull_file(options.hull_file_path, brush_options);

        // parse time fixups: brush contents, invisible surface renames, origin
        // and boundingbox builds (their planes and texinfos come first), sky
        // clip copies, func_group and func_detail merges, hull shape registry
        plane_store planes;
        map_post_options post_options;
        post_options.sky_clip = options.sky_clip;
        post_options.invisible_items = options.invisible_items;
        post_options.brush = brush_options;
        post_process_map(result.map, planes, &result.texinfos, result.hull_shapes, post_options);
        brush_options.hull_shapes = &result.hull_shapes;

        // -onlyents stops after parsing: the reference skips the build pass,
        // model centers, brush union and miptex, and only rewrites the
        // entity lump of the existing bsp
        if (brush_options.only_entities)
            return result;

        // sequential (the plane store is shared and mutated in order, so this
        // pass is not parallelized); one live bar over the entities
        progress::section("processing");
        progress::begin("building brushes", (int)result.map.entities.size());
        for (int i = 0; i < (int)result.map.entities.size(); i++)
        {
            result.entities.push_back(build_entity_brushes(
                result.map, i, planes, &result.texinfos, brush_options));
            progress::add(1);
        }
        progress::end();
        result.planes = planes.planes();

        for (int i = 0; i < (int)result.map.entities.size(); i++)
            set_model_center(result, i);

        for (const brush_union_warning &warning : calculate_brush_union_warnings(
            result, options.brush_union_threshold, brush_options.world_extent))
        {
            logging::warn("entity %d: brush %d intersects with brush %d by %2.3f percent",
                          warning.entity_num, warning.brush_num, warning.other_brush_num,
                          (double)warning.percent);
        }

        miptex_build_result miptex = build_miptex_lump(result.map, result.texinfos, options.wad);
        result.textures = std::move(miptex.textures);
        result.temp_wad = std::move(miptex.temp_wad);
        for (int i = 0; i < (int)miptex.texinfo_miptex.size(); i++)
            result.texinfos.set_miptex_index(i, miptex.texinfo_miptex[(size_t)i]);
        if (!result.map.entities.empty())
            result.map.entities[0].set_value("wad", miptex.wad_value.c_str());

        return result;
    }
}
