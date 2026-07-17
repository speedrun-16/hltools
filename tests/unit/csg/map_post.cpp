#include <iostream>
#include <sstream>

#include "common/log.h"
#include "csg/map_post.h"
#include "support/test.h"

namespace
{
    csg::map_entity entity(const char *classname, const char *origin = "",
                           const char *targetname = "")
    {
        csg::map_entity result;
        result.set_value("classname", classname);
        if (origin[0])
            result.set_value("origin", origin);
        if (targetname[0])
            result.set_value("targetname", targetname);
        return result;
    }

    std::string post_process(csg::map_source &map)
    {
        csg::plane_store planes;
        csg::hull_shape_library hull_shapes;
        csg::map_post_options options;

        std::ostringstream output;
        std::streambuf *previous = std::cout.rdbuf(output.rdbuf());
        csg::post_process_map(map, planes, nullptr, hull_shapes, options);
        std::cout.rdbuf(previous);
        return output.str();
    }
}

test("reports entity origins outside the engine range as one warning")
{
    csg::map_source map;
    map.entities.push_back(entity("worldspawn"));
    map.entities.push_back(entity("cycler_sprite", "-3064 -4248 -336", "distant_sprite"));
    map.entities.push_back(entity("env_sprite", "5000 0 0"));
    map.entities.push_back(entity("info_target", "4096 0 0"));

    unsigned warnings_before = logging::warning_count();
    const std::string output = post_process(map);

    expect(logging::warning_count() == warnings_before + 1);
    expect(output.find("ENTITY ORIGINS OUTSIDE ENGINE RANGE") != std::string::npos);
    expect(output.find("2 entities outside the +/-4096 unit range") != std::string::npos);
    expect(output.find("entity 1  cycler_sprite  targetname \"distant_sprite\"") != std::string::npos);
    expect(output.find("origin  (-3064, -4248, -336)") != std::string::npos);
    expect(output.find("entity 2  env_sprite") != std::string::npos);
    expect(output.find("entity 3  info_target") == std::string::npos);
    expect(output.find("origin outside +/-4096") == std::string::npos);
}

test("does not report compile lights outside the engine range")
{
    csg::map_source map;
    map.entities.push_back(entity("worldspawn"));
    map.entities.push_back(entity("light", "0 -5000 0"));
    map.entities.push_back(entity("light_environment", "5000 0 0"));

    unsigned warnings_before = logging::warning_count();
    const std::string output = post_process(map);

    expect(logging::warning_count() == warnings_before);
    expect(output.empty());
}
