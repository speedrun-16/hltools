#include <cstring>

#include "../common/error.h"
#include "../common/log.h"
#include "../common/string_util.h"
#include "internal.h"

// opaque studio model (mdl) shadows: models named by env_static entities or
// entities with zhlt_studioshadow block light like opaque brush entities
// mesh loading and tracing are not implemented yet, so maps using this feature
// stop the compile instead of silently missing shadows

namespace rad
{
    void load_studio_models(rad_state &state)
    {
        state.num_studio_models = 0;

        if (!state.options.studioshadow)
            return;

        for (size_t i = 0; i < state.entities.size(); i++)
        {
            const char *name, *model;

            format::entity *e = &state.entities[i];
            name = e->value("classname");

            if (str::iequals(name, "env_static"))
            {
                int spawnflags = int_for_key(*e, "spawnflags");
                if (spawnflags & 4)
                    continue; // shadow disabled

                model = e->value("model");

                if (!model || !*model)
                {
                    continue;
                }
            }
            else if (int_for_key(*e, "zhlt_studioshadow"))
            {
                model = e->value("model");

                if (!model || !*model)
                    continue;
            }
            else
            {
                continue;
            }

            err::fatal("studio model shadows are not implemented yet: entity %d (classname \"%s\", model \"%s\") "
                       "requests an opaque studio model. Recompile with -nostudioshadow to ignore studio shadows.",
                       (int)i, name, model);
        }

        if (state.num_studio_models)
            logging::info("  %-14s %d opaque studio models\n", "studio", state.num_studio_models);
    }

    void free_studio_models(rad_state &state)
    {
        state.num_studio_models = 0;
    }

    bool test_segment_against_studio_list(const rad_state &state, const vec3v &p1, const vec3v &p2)
    {
        (void)p1;
        (void)p2;
        if (!state.num_studio_models)
            return false; // easy out

        return false;
    }
}
