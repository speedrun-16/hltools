#include "entities.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
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

        bool is_source_team_spawn(const char *classname)
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
        // solidity 1 is "never solid", 0 ("toggle", solid while enabled) and 2
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

        // keys the goldsrc engine and this compiler both ignore. they come from
        // hammer bookkeeping, source-only rendering controls, and the source
        // light attenuation model, which goldsrc replaces with a fixed falloff.
        // stripping them keeps the ported entity block readable and small enough
        // to see the keys that do still matter.
        const char *const dead_keys[] = {
            // hammer / vbsp bookkeeping
            "hammerid", "StartDisabled", "InputFilter", "invert_exclusion",
            "solidbsp", "PerformanceMode",
            // source-only shadow and render controls. note hlrad's own "_fade"
            // is a different key from source's prop fade distances below.
            "vrad_brush_cast_shadows", "disableshadows", "disablereceiveshadows",
            "disableflashlight", "shadowcastdist", "screenspacefade",
            "fademindist", "fademaxdist", "fadescale",
            "mindxlevel", "maxdxlevel", "MinDXLevel", "MaxDXLevel",
            "mincpulevel", "maxcpulevel", "mingpulevel", "maxgpulevel",
            // source hdr lighting, inert outside vrad
            "_lightHDR", "_lightscaleHDR", "_ambientHDR", "_AmbientScaleHDR",
            "_diffuse_lightHDR",
            // source's per light attenuation model. hlrad uses a fixed inverse
            // square falloff and reads none of these.
            "_quadratic_attn", "_linear_attn", "_constant_attn", "_distance",
            "_fifty_percent_distance", "_zero_percent_distance",
            // physics properties with no goldsrc analogue
            "physdamagescale", "nodamageforces", "massScale", "inertiaScale",
        };

        void strip_dead_keys(format::entity &entity)
        {
            for (const char *key : dead_keys)
                entity.remove(key);
        }

        // moves a key to the name goldsrc knows it by, keeping the value. a key
        // already present under the target name wins, since it was written for
        // goldsrc deliberately.
        bool rename_key(format::entity &entity, const char *from, const char *to)
        {
            const char *value = entity.value(from);
            if (value[0] == '\0')
                return false;
            std::string carried = value;
            entity.remove(from);
            if (entity.value(to)[0] == '\0')
                entity.set(to, carried.c_str());
            return true;
        }

        // source's fog lives on env_fog_controller and is read by the client's
        // material system; goldsrc/cs1.6 has a working env_fog taking the same
        // three quantities under different names. the keys with no equivalent
        // (the second blend colour, the blend direction and lerp, and the source
        // specific far z) are dropped rather than approximated.
        void convert_fog(format::entity &entity)
        {
            entity.set("classname", "env_fog");
            rename_key(entity, "fogcolor", "rendercolor");
            rename_key(entity, "fogstart", "startdist");
            std::string fog_end = entity.value("fogend");
            rename_key(entity, "fogend", "enddist");

            // not a rename: source's "fogmaxdensity" is a 0..1 fraction capping
            // how opaque the fog may become, while goldsrc's "density" is the
            // per-unit coefficient of an exponential falloff. copying the number
            // across gives density 1, which produces a black screen within a
            // unit. exponential fog is ~95% opaque at density*d == 3, so
            // derive the coefficient from the distance at which source said the
            // fog was full instead.
            entity.remove("fogmaxdensity");
            if (entity.value("density")[0] == '\0')
            {
                double end = std::atof(fog_end.c_str());
                double density = end > 1.0 ? 3.0 / end : 0.001;
                // the range every working goldsrc map on record sits in
                if (density < 0.0001)
                    density = 0.0001;
                if (density > 0.01)
                    density = 0.01;
                char text[32];
                std::snprintf(text, sizeof(text), "%.5g", density);
                entity.append("density", text);
            }
            entity.remove("fogcolor2");
            entity.remove("fogdir");
            entity.remove("fogblend");
            entity.remove("foglerptime");
            entity.remove("fogenable");
            entity.remove("farz");
            entity.remove("use_angles");
            entity.remove("angles");
        }

        // hlrad reads _diffuse_light and _spread; source writes the same two
        // quantities as _ambient and SunSpreadAngle. without the rename the sky
        // fill silently falls back to the sun's own colour and brightness, and
        // the sun becomes a mathematical point with razor sharp shadow edges.
        // brightness itself is left alone: it is a per map tuning decision, and
        // rescaling it here would quietly change every already ported map.
        void convert_light_keys(format::entity &entity)
        {
            rename_key(entity, "_ambient", "_diffuse_light");
            rename_key(entity, "SunSpreadAngle", "_spread");
        }
    }

    void remap_source_entities(std::vector<format::entity> &entities,
                               entity_remap_stats &stats)
    {
        std::vector<format::entity> kept;
        kept.reserve(entities.size());

        // prefer an existing generic start. otherwise select the first suitable
        // team-specific start and discard the rest: the port only needs one
        // deterministic entry point.
        bool existing_start = false;
        for (const format::entity &entity : entities)
            if (iequals(entity.value("classname"), "info_player_start"))
            {
                existing_start = true;
                break;
            }
        bool start_written = false;

        for (format::entity &entity : entities)
        {
            strip_dead_keys(entity);

            const char *classname = entity.value("classname");
            if (iequals(classname, "info_player_start"))
            {
                if (start_written)
                {
                    stats.dropped_spawns++;
                    continue;
                }
                start_written = true;
            }
            else if (is_source_team_spawn(classname))
            {
                bool suitable = iequals(classname, "info_player_counterterrorist");
                if (existing_start || start_written || !suitable)
                {
                    stats.dropped_spawns++;
                    continue;
                }
                entity.set("classname", "info_player_start");
                start_written = true;
            }
            else if (is_source_brush_entity(classname))
            {
                bool solid = std::strcmp(entity.value("Solidity"), "1") != 0;
                entity.set("classname", solid ? "func_wall" : "func_illusionary");
                // solidity is now carried by the classname itself
                entity.remove("Solidity");
                stats.brush_entities++;
            }
            else if (is_studio_prop(classname))
            {
                entity.set("classname", "cycler_sprite");
                if (entity.value("framerate")[0] == '\0')
                    entity.append("framerate", "10");
                stats.studio_props++;
            }
            else if (iequals(classname, "env_fog_controller"))
            {
                convert_fog(entity);
                stats.fog_controllers++;
            }
            else if (iequals(classname, "point_viewcontrol"))
            {
                // source renamed goldsrc's trigger_camera; the keys goldsrc reads
                // (targetname, target, wait, speed, moveto) keep their names, the
                // source-only motion controls do not exist.
                entity.set("classname", "trigger_camera");
                entity.remove("acceleration");
                entity.remove("deceleration");
                entity.remove("interpolatepositiontoplayer");
                stats.cameras++;
            }
            else if (std::strncmp(classname, "light", 5) == 0)
            {
                convert_light_keys(entity);
                stats.lights++;
            }
            kept.push_back(std::move(entity));
        }

        entities.swap(kept);
        stats.player_start = start_written;
    }
}
