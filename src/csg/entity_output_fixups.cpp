#include "entity_output_fixups.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>

#include "common/error.h"
#include "common/log.h"

namespace csg
{
    namespace
    {
        constexpr double q_pi = 3.14159265358979323846;
        constexpr double fixup_normal_epsilon = 0.00001;

        int angles_for_vector(float angles[3], const float vector[3])
        {
            float z = vector[2];
            float r = (float)std::sqrt(vector[0] * vector[0] + vector[1] * vector[1]);
            float tmp;
            if (std::sqrt(z * z + r * r) < fixup_normal_epsilon)
                return -1;

            tmp = (float)std::sqrt(z * z + r * r);
            z /= tmp, r /= tmp;
            if (r < fixup_normal_epsilon)
            {
                if (z < 0)
                    angles[0] = -90, angles[1] = 0;
                else
                    angles[0] = 90, angles[1] = 0;
            }
            else
            {
                angles[0] = (float)(std::atan(z / r) / q_pi * 180);
                float x = vector[0], y = vector[1];
                tmp = (float)std::sqrt(x * x + y * y);
                x /= tmp, y /= tmp;
                if (x < -1 + fixup_normal_epsilon)
                {
                    angles[1] = -180;
                }
                else if (y >= 0)
                {
                    angles[1] = (float)(2 * std::atan(y / (1 + x)) / q_pi * 180);
                }
                else
                {
                    angles[1] = (float)(2 * std::atan(y / (1 + x)) / q_pi * 180 + 360);
                }
            }
            angles[2] = 0;
            return 0;
        }

        format::entity *find_target_entity(std::vector<format::entity> &entities,
                                           const char *target)
        {
            for (format::entity &ent : entities)
            {
                if (std::strcmp(ent.value("targetname"), target) == 0)
                    return &ent;
            }
            return nullptr;
        }
    }

    void apply_entity_output_fixups(std::vector<format::entity> &entities,
                                    bool optimize_lights)
    {
        for (size_t i = 0; i < entities.size(); i++)
        {
            const char *classname = entities[i].value("classname");
            if (std::strcmp(classname, "info_sunlight") != 0
                && std::strcmp(classname, "light_environment") != 0)
            {
                continue;
            }

            float vec[3] = {0, 0, 0};
            std::sscanf(entities[i].value("angles"), "%f %f %f", &vec[0], &vec[1], &vec[2]);
            float pitch = (float)std::atof(entities[i].value("pitch"));
            if (pitch)
                vec[0] = pitch;

            const char *target = entities[i].value("target");
            if (target[0])
            {
                format::entity *target_ent = find_target_entity(entities, target);
                if (target_ent)
                {
                    float origin1[3] = {0, 0, 0};
                    float origin2[3] = {0, 0, 0};
                    float normal[3];
                    std::sscanf(entities[i].value("origin"), "%f %f %f",
                                &origin1[0], &origin1[1], &origin1[2]);
                    std::sscanf(target_ent->value("origin"), "%f %f %f",
                                &origin2[0], &origin2[1], &origin2[2]);
                    for (int k = 0; k < 3; k++)
                        normal[k] = origin2[k] - origin1[k];
                    angles_for_vector(vec, normal);
                }
            }

            char value[1024];
            std::snprintf(value, sizeof(value), "%g %g %g", vec[0], vec[1], vec[2]);
            bool sunlight = std::strcmp(entities[i].value("classname"), "info_sunlight") == 0;
            entities[i].set("angles", value);
            entities[i].remove("pitch");

            if (sunlight)
            {
                format::entity fake;
                std::swap(fake, entities[i]);
                fake.set("classname", "light_environment");
                fake.set("_fake", "1");
                entities.push_back(std::move(fake));
            }
        }

        for (format::entity &ent : entities)
        {
            const char *classname = ent.value("classname");
            if (std::strcmp(classname, "light_shadow") != 0
                && std::strcmp(classname, "light_bounce") != 0)
            {
                continue;
            }
            ent.set("convertfrom", ent.value("classname"));
            std::string convert_to = ent.value("convertto");
            ent.set("classname", convert_to.empty() ? "light" : convert_to.c_str());
            ent.set("convertto", "");
        }

        for (format::entity &ent : entities)
        {
            if (std::strcmp(ent.value("classname"), "light_surface") != 0)
                continue;
            if (!ent.value("_tex")[0])
                ent.set("_tex", "                ");
            std::string convert_to = ent.value("convertto");
            if (convert_to.empty())
                ent.set("classname", "light");
            else if (std::strncmp(convert_to.c_str(), "light", 5) != 0)
                err::fatal("new classname for 'light_surface' should begin with 'light' not '%s'",
                           convert_to.c_str());
            else
                ent.set("classname", convert_to.c_str());
            ent.set("convertto", "");
        }

        if (!optimize_lights)
            return;

        std::vector<bool> need_compare(entities.size(), false);
        int count = 0;
        for (int i = (int)entities.size() - 1; i > -1; i--)
        {
            format::entity &ent = entities[(size_t)i];
            const char *classname = ent.value("classname");
            const char *targetname = ent.value("targetname");
            int style = std::atoi(ent.value("style"));
            if (!targetname[0]
                || (std::strcmp(classname, "light") != 0
                    && std::strcmp(classname, "light_spot") != 0
                    && std::strcmp(classname, "light_environment") != 0))
            {
                continue;
            }
            size_t j;
            for (j = (size_t)i + 1; j < entities.size(); j++)
            {
                if (!need_compare[j])
                    continue;
                if (style == std::atoi(entities[j].value("style"))
                    && std::strcmp(targetname, entities[j].value("targetname")) == 0)
                {
                    break;
                }
            }
            if (j < entities.size())
            {
                ent.remove("targetname");
                count++;
            }
            else
            {
                need_compare[(size_t)i] = true;
            }
        }
        if (count > 0)
            logging::info("%d redundant named lights optimized.\n", count);
    }
}
