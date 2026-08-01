#include "entities.h"

#include <cctype>
#include <cstring>
#include <utility>

#include "format/bsp/entity_lump.h"

namespace decompile
{
    namespace
    {
        bool iequals(const char *a, const char *b)
        {
            for (; *a && *b; a++, b++)
                if (std::tolower((unsigned char)*a) != std::tolower((unsigned char)*b))
                    return false;
            return *a == *b;
        }

        bool is_cs_team_spawn(const char *classname)
        {
            return iequals(classname, "info_player_terrorist")
                || iequals(classname, "info_player_counterterrorist");
        }

        // source's prop family all render a studio model at a point. goldsrc's
        // nearest equivalent is cycler_sprite, which draws a model without being
        // solid: every prop in the maps this targets is non solid anyway, and
        // goldsrc takes collision from brushes rather than from models.
        // source's func_brush is a general purpose brush entity whose solidity
        // is a keyvalue; goldsrc splits that across two classnames instead.
        // Solidity 1 is "never solid", 0 ("toggle", solid while enabled) and 2
        // ("always solid") both end up solid.
        bool is_source_brush_entity(const char *classname)
        {
            return iequals(classname, "func_brush")
                || iequals(classname, "func_wall_toggle");
        }

        bool is_studio_prop(const char *classname)
        {
            return iequals(classname, "prop_dynamic")
                || iequals(classname, "prop_dynamic_override")
                || iequals(classname, "prop_static")
                || iequals(classname, "prop_physics")
                || iequals(classname, "prop_physics_multiplayer")
                || iequals(classname, "prop_physics_override")
                || iequals(classname, "prop_ragdoll");
        }
    }

    void remap_source_entities(std::vector<format::entity> &entities,
                               entity_remap_stats &stats)
    {
        std::vector<format::entity> kept;
        kept.reserve(entities.size());

        // a source map may already carry an info_player_start of its own beside
        // the cs team spawns. retagging a team spawn as well would
        // leave the ported map with two spawn points, so treat the
        // existing one as the spawn and drop every team spawn.
        bool spawn_written = false;
        for (const format::entity &entity : entities)
            if (iequals(entity.value("classname"), "info_player_start"))
            {
                spawn_written = true;
                break;
            }
        for (format::entity &entity : entities)
        {
            // Hammer assigns this editor-only identity to nearly every Source
            // entity. It has no GoldSrc runtime meaning and makes otherwise
            // identical ported entities appear different.
            entity.remove("hammerid");

            if (is_cs_team_spawn(entity.value("classname")))
            {
                if (spawn_written)
                {
                    stats.dropped_spawns++;
                    continue; // a single spawn is enough; drop the extras
                }
                entity.set("classname", "info_player_start");
                spawn_written = true;
                stats.player_start = true;
            }
            else if (is_source_brush_entity(entity.value("classname")))
            {
                bool solid = std::strcmp(entity.value("Solidity"), "1") != 0;
                entity.set("classname", solid ? "func_wall" : "func_illusionary");
                // solidity is now carried by the classname itself
                entity.remove("Solidity");
                stats.brush_entities++;
            }
            else if (is_studio_prop(entity.value("classname")))
            {
                entity.set("classname", "cycler_sprite");
                if (entity.value("framerate")[0] == '\0')
                    entity.append("framerate", "10");
                stats.studio_props++;
            }
            kept.push_back(std::move(entity));
        }

        entities.swap(kept);
    }
}
