#include <cstring>

#include "csg/map_post.h"
#include "support/map_builder.h"
#include "support/test.h"

namespace
{
    csg::map_source process_trigger(bool nullify)
    {
        test_support::map_builder builder;
        builder.brush_entity("trigger_once", {-32, -32, 0}, {32, 32, 64},
                             "AAATRIGGER", {{"target", "door"}});
        csg::map_source map = builder.build();

        csg::plane_store planes;
        csg::hull_shape_library hull_shapes;
        csg::map_post_options options;
        options.sky_clip = false;
        options.brush.nullify_trigger = nullify;
        csg::post_process_map(map, planes, nullptr, hull_shapes, options);
        return map;
    }
}

suite("entities.csg.trigger_once")
{
    test("trigger_once.preserves its render faces by default")
    {
        csg::map_source map = process_trigger(false);

        require(map.entities.size() == 2);
        expect(std::strcmp(map.entities[1].value("classname"), "trigger_once") == 0);
        expect(std::strcmp(map.entities[1].value("target"), "door") == 0);
        require(map.brushes.size() == 1);
        require(map.sides.size() == 6);
        for (const csg::brush_side &side : map.sides)
            expect(std::strcmp(side.texture.name, "AAATRIGGER") == 0);
    }

    test("trigger_once.nullifies its render faces when requested")
    {
        csg::map_source map = process_trigger(true);

        require(map.sides.size() == 6);
        for (const csg::brush_side &side : map.sides)
            expect(std::strcmp(side.texture.name, "NULL") == 0);
    }
}
