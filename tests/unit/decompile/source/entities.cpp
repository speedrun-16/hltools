#include <cstring>
#include <initializer_list>
#include <utility>
#include <vector>

#include "decompile/source/entities.h"
#include "format/bsp/entity_lump.h"
#include "support/test.h"

namespace
{
    format::entity make_entity(
        std::initializer_list<std::pair<const char *, const char *>> pairs)
    {
        format::entity entity;
        for (const auto &pair : pairs)
            entity.append(pair.first, pair.second);
        return entity;
    }
}

suite("unit.decompile.source_entities")
{
    test("remap removes source-only keys from point and brush entities")
    {
        format::entity world = make_entity({{"classname", "worldspawn"},
                                            {"hammerid", "1"}});
        format::entity sun = make_entity({{"classname", "light_environment"},
                                          {"hammerid", "123"},
                                          {"_light", "255 255 255 24"},
                                          {"_lightHDR", "-1 -1 -1 1"},
                                          {"SunSpreadAngle", "8"},
                                          {"_ambient", "80 100 130 20"}});
        format::entity brush = make_entity({{"classname", "func_brush"},
                                            {"hammerid", "16622"},
                                            {"StartDisabled", "0"},
                                            {"vrad_brush_cast_shadows", "0"},
                                            {"Solidity", "2"}});

        std::vector<format::entity> entities = {world, sun, brush};
        decompile::entity_remap_stats stats;
        decompile::remap_source_entities(entities, stats);

        require(entities.size() == 3);
        for (const format::entity &entity : entities)
            expect_false(entity.has("hammerid"));

        // hlrad reads _diffuse_light and _spread; the source names for the same
        // two quantities are ignored, which loses the sky fill colour and leaves
        // the sun a hard edged mathematical point
        expect(std::strcmp(entities[1].value("classname"), "light_environment") == 0);
        expect(std::strcmp(entities[1].value("_light"), "255 255 255 24") == 0);
        expect(std::strcmp(entities[1].value("_spread"), "8") == 0);
        expect(std::strcmp(entities[1].value("_diffuse_light"), "80 100 130 20") == 0);
        expect_false(entities[1].has("SunSpreadAngle"));
        expect_false(entities[1].has("_ambient"));
        expect_false(entities[1].has("_lightHDR"));
        expect(stats.lights == 1);

        expect(std::strcmp(entities[2].value("classname"), "func_wall") == 0);
        expect_false(entities[2].has("Solidity"));
        expect_false(entities[2].has("StartDisabled"));
        expect_false(entities[2].has("vrad_brush_cast_shadows"));
    }

    test("team-specific spawns collapse to one player start")
    {
        std::vector<format::entity> entities = {
            make_entity({{"classname", "worldspawn"}}),
            make_entity({{"classname", "info_player_terrorist"},
                         {"origin", "0 0 16"}}),
            make_entity({{"classname", "info_player_counterterrorist"},
                         {"origin", "0 64 16"}}),
            make_entity({{"classname", "info_player_counterterrorist"},
                         {"origin", "64 64 16"}}),
        };
        decompile::entity_remap_stats stats;
        decompile::remap_source_entities(entities, stats);

        require(entities.size() == 2);
        expect(std::strcmp(entities[1].value("classname"), "info_player_start") == 0);
        expect(std::strcmp(entities[1].value("origin"), "0 64 16") == 0);
        expect(stats.dropped_spawns == 2);
        expect(stats.player_start);
    }

    test("an existing player start wins and duplicates are removed")
    {
        std::vector<format::entity> entities = {
            make_entity({{"classname", "worldspawn"}}),
            make_entity({{"classname", "info_player_counterterrorist"},
                         {"origin", "0 64 16"}}),
            make_entity({{"classname", "info_player_start"},
                         {"origin", "16 24 32"}}),
            make_entity({{"classname", "info_player_start"},
                         {"origin", "64 64 64"}}),
        };
        decompile::entity_remap_stats stats;
        decompile::remap_source_entities(entities, stats);

        require(entities.size() == 2);
        expect(std::strcmp(entities[1].value("classname"), "info_player_start") == 0);
        expect(std::strcmp(entities[1].value("origin"), "16 24 32") == 0);
        expect(stats.dropped_spawns == 2);
        expect(stats.player_start);
    }

    test("env_fog_controller becomes env_fog")
    {
        std::vector<format::entity> entities = {
            make_entity({{"classname", "worldspawn"}}),
            make_entity({{"classname", "env_fog_controller"},
                         {"fogcolor", "12 20 28"},
                         {"fogcolor2", "0 0 0"},
                         {"fogstart", "256"},
                         {"fogend", "8192"},
                         {"fogmaxdensity", "1"},
                         {"fogdir", "1 0 0"},
                         {"farz", "-1"}}),
        };
        decompile::entity_remap_stats stats;
        decompile::remap_source_entities(entities, stats);

        const format::entity &fog = entities[1];
        expect(std::strcmp(fog.value("classname"), "env_fog") == 0);
        expect(std::strcmp(fog.value("rendercolor"), "12 20 28") == 0);
        expect(std::strcmp(fog.value("startdist"), "256") == 0);
        expect(std::strcmp(fog.value("enddist"), "8192") == 0);
        // not a copy of fogmaxdensity: goldsrc's density is a per-unit
        // exponential coefficient, so it is derived from the distance at which
        // source said the fog was full (3/8192), not from the 0..1 fraction
        expect(std::strcmp(fog.value("density"), "0.00036621") == 0);
        expect_false(fog.has("fogmaxdensity"));
        // no goldsrc equivalent: dropped rather than approximated
        expect_false(fog.has("fogcolor2"));
        expect_false(fog.has("fogdir"));
        expect_false(fog.has("farz"));
        expect(stats.fog_controllers == 1);
    }

    test("point_viewcontrol becomes trigger_camera")
    {
        std::vector<format::entity> entities = {
            make_entity({{"classname", "worldspawn"}}),
            make_entity({{"classname", "point_viewcontrol"},
                         {"targetname", "spec"},
                         {"wait", "3"},
                         {"acceleration", "500"},
                         {"deceleration", "500"}}),
        };
        decompile::entity_remap_stats stats;
        decompile::remap_source_entities(entities, stats);

        expect(std::strcmp(entities[1].value("classname"), "trigger_camera") == 0);
        expect(std::strcmp(entities[1].value("wait"), "3") == 0);
        expect(std::strcmp(entities[1].value("targetname"), "spec") == 0);
        expect_false(entities[1].has("acceleration"));
        expect(stats.cameras == 1);
    }

    test("a key already written for goldsrc wins over the source name")
    {
        // a map hand edited for goldsrc may carry both spellings; the goldsrc one
        // was written deliberately and must not be overwritten by the rename
        std::vector<format::entity> entities = {
            make_entity({{"classname", "worldspawn"}}),
            make_entity({{"classname", "light_environment"},
                         {"_diffuse_light", "1 2 3 4"},
                         {"_ambient", "9 9 9 9"}}),
        };
        decompile::entity_remap_stats stats;
        decompile::remap_source_entities(entities, stats);

        expect(std::strcmp(entities[1].value("_diffuse_light"), "1 2 3 4") == 0);
        expect_false(entities[1].has("_ambient"));
    }
}
