#include <cstring>
#include <vector>

#include "decompile/source_entities.h"
#include "format/bsp/entity_lump.h"
#include "support/test.h"

suite("unit.decompile.source_entities")
{
    test("remap removes Hammer editor ids from point and brush entities")
    {
        format::entity world;
        world.append("classname", "worldspawn");
        world.append("hammerid", "1");

        format::entity sun;
        sun.append("classname", "light_environment");
        sun.append("hammerid", "123");
        sun.append("_light", "255 255 255 24");
        sun.append("SunSpreadAngle", "0");

        format::entity brush;
        brush.append("classname", "func_brush");
        brush.append("hammerid", "16622");
        brush.append("Solidity", "2");

        std::vector<format::entity> entities = {world, sun, brush};
        decompile::entity_remap_stats stats;
        decompile::remap_source_entities(entities, stats);

        require(entities.size() == 3);
        for (const format::entity &entity : entities)
            expect_false(entity.has("hammerid"));
        expect(std::strcmp(entities[1].value("classname"),
                           "light_environment") == 0);
        expect(std::strcmp(entities[1].value("_light"),
                           "255 255 255 24") == 0);
        expect(std::strcmp(entities[1].value("SunSpreadAngle"), "0") == 0);
        expect(std::strcmp(entities[2].value("classname"), "func_wall") == 0);
        expect_false(entities[2].has("Solidity"));
    }
}
