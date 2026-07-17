#pragma once

#include <initializer_list>
#include <string>
#include <utility>

#include "common/string_util.h"
#include "csg/map_parser.h"

namespace test_support
{
    class map_builder
    {
    public:
        using keyvalue = std::pair<const char *, const char *>;

        map_builder()
        {
            csg::map_entity world;
            world.pairs.emplace_back("classname", "worldspawn");
            map_.entities.push_back(std::move(world));
        }

        map_builder &brush_entity(const char *classname,
                                  const math::vec3v &mins,
                                  const math::vec3v &maxs,
                                  const char *texture,
                                  std::initializer_list<keyvalue> keys = {})
        {
            int entity_num = (int)map_.entities.size();
            csg::map_entity entity;
            entity.first_brush = (int)map_.brushes.size();
            entity.num_brushes = 1;
            entity.pairs.emplace_back("classname", classname);
            for (const keyvalue &key : keys)
                entity.pairs.emplace_back(key.first, key.second);
            map_.entities.push_back(std::move(entity));

            csg::map_brush brush;
            brush.original_entity_num = entity_num;
            brush.entity_num = entity_num;
            brush.first_side = (int)map_.sides.size();
            brush.num_sides = 6;
            map_.brushes.push_back(brush);

            add_side(texture, {mins.x, mins.y, mins.z},
                     {mins.x, maxs.y, mins.z}, {mins.x, mins.y, maxs.z});
            add_side(texture, {maxs.x, mins.y, mins.z},
                     {maxs.x, mins.y, maxs.z}, {maxs.x, maxs.y, mins.z});
            add_side(texture, {mins.x, mins.y, mins.z},
                     {mins.x, mins.y, maxs.z}, {maxs.x, mins.y, mins.z});
            add_side(texture, {mins.x, maxs.y, mins.z},
                     {maxs.x, maxs.y, mins.z}, {mins.x, maxs.y, maxs.z});
            add_side(texture, {mins.x, mins.y, mins.z},
                     {maxs.x, mins.y, mins.z}, {mins.x, maxs.y, mins.z});
            add_side(texture, {mins.x, mins.y, maxs.z},
                     {mins.x, maxs.y, maxs.z}, {maxs.x, mins.y, maxs.z});
            return *this;
        }

        csg::map_source build()
        {
            map_.parsed_entities = (int)map_.entities.size();
            map_.parsed_brushes = (int)map_.brushes.size();
            return std::move(map_);
        }

    private:
        void add_side(const char *texture,
                      const math::vec3v &p0,
                      const math::vec3v &p1,
                      const math::vec3v &p2)
        {
            csg::brush_side side;
            str::copy(side.texture.name, sizeof(side.texture.name), texture);
            side.planepts[0] = p0;
            side.planepts[1] = p1;
            side.planepts[2] = p2;
            map_.sides.push_back(side);
        }

        csg::map_source map_;
    };
}
