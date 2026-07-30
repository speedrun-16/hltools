#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "../common/error.h"
#include "../common/log.h"
#include "../common/string_util.h"
#include "internal.h"

// patch creation: every face becomes one patch which is then subdivided along
// its texel grid down to the chop size also the texture keyed value tables
// (texlights, chopscale, smoothvalue, translucency, angular fade) and the
// opaque entity list

namespace rad
{
    // ===== texlights =====

    void read_light_file(rad_state &state, const char *filename)
    {
        FILE *f;
        char scan[4096];
        short argcnt;
        unsigned int file_texlights = 0;

        f = fopen(filename, "r");
        if (!f)
        {
            logging::warn("Could not open texlight file %s", filename);
            return;
        }
        while (fgets(scan, sizeof(scan), f))
        {
            char *comment;
            char sztexlight[260];
            vec_t r, g, b, i = 1;

            comment = strstr(scan, "//");
            if (comment)
            {
                // terminate the string early on a c++ style comment
                comment[0] = '\n';
                comment[1] = 0;
            }

            argcnt = (short)sscanf(scan, "%s %f %f %f %f", sztexlight, &r, &g, &b, &i);

            if (argcnt == 2)
            {
                // with 1+1 args, the r,g,b values are all equal to the first value
                g = b = r;
            }
            else if (argcnt == 5)
            {
                // with 1+4 args, the r,g,b values are scaled by the fourth value
                r = (vec_t)(r * (i / 255.0));
                g = (vec_t)(g * (i / 255.0));
                b = (vec_t)(b * (i / 255.0));
            }
            else if (argcnt != 4)
            {
                if (strlen(scan) > 4)
                {
                    logging::warn("ignoring bad texlight '%s' in %s", scan, filename);
                }
                continue;
            }

            for (size_t it = 0; it < state.texlights.size(); it++)
            {
                if (strcmp(state.texlights[it].name.c_str(), sztexlight) == 0)
                {
                    if (state.texlights[it].source == filename)
                    {
                        logging::warn("Duplication of texlight '%s' in file '%s'!",
                                      state.texlights[it].name.c_str(), state.texlights[it].source.c_str());
                    }
                    else if (state.texlights[it].value[0] != r || state.texlights[it].value[1] != g || state.texlights[it].value[2] != b)
                    {
                        logging::warn("Overriding '%s' from '%s' with '%s'!",
                                      state.texlights[it].name.c_str(), state.texlights[it].source.c_str(), filename);
                    }
                    else
                    {
                        logging::warn("Redundant '%s' def in '%s' AND '%s'!",
                                      state.texlights[it].name.c_str(), state.texlights[it].source.c_str(), filename);
                    }
                    state.texlights.erase(state.texlights.begin() + (long long)it);
                    break;
                }
            }

            rad_texlight texlight;
            texlight.name = sztexlight;
            texlight.value[0] = r;
            texlight.value[1] = g;
            texlight.value[2] = b;
            texlight.source = filename;
            file_texlights++;
            state.texlights.push_back(texlight);
        }
        fclose(f);
        logging::info("  %-14s %u parsed (%s)\n", "texlights", file_texlights, filename);
    }

    // parse texture keyed lighting metadata from info_texlights,
    // info_minlights, and info_unlittextures entities
    void read_info_tex_and_minlights(rad_state &state)
    {
        int values;
        float r, g, b, i, min;
        rad_texlight texlight;
        minlight ml;

        for (size_t k = 0; k < state.entities.size(); k++)
        {
            format::entity *mapent = &state.entities[k];
            bool found_minlights = false;
            bool found_texlights = false;

            if (!strcmp(mapent->value("classname"), "info_minlights"))
            {
                logging::info("Reading per-tex minlights from info_minlights map entity\n");

                for (size_t ep = 0; ep < mapent->pairs().size(); ep++)
                {
                    const char *key = mapent->pairs()[ep].first.c_str();
                    const char *value = mapent->pairs()[ep].second.c_str();
                    if (!strcmp(key, "classname") || !strcmp(key, "origin"))
                        continue; // we don't care about these keyvalues
                    if (sscanf(value, "%f", &min) != 1)
                    {
                        logging::warn("Ignoring bad minlight '%s' in info_minlights entity", key);
                        continue;
                    }
                    ml.name = key;
                    ml.value = min;
                    state.minlights.push_back(ml);
                }
            }
            else if (!strcmp(mapent->value("classname"), "info_texlights"))
            {
                logging::info("Reading texlights from info_texlights map entity\n");

                for (size_t ep = 0; ep < mapent->pairs().size(); ep++)
                {
                    const char *key = mapent->pairs()[ep].first.c_str();
                    const char *value = mapent->pairs()[ep].second.c_str();
                    if (!strcmp(key, "classname") || !strcmp(key, "origin"))
                        continue; // we don't care about these keyvalues

                    values = sscanf(value, "%f %f %f %f", &r, &g, &b, &i);

                    if (values == 1)
                    {
                        g = b = r;
                    }
                    else if (values == 4) // use the brightness value
                    {
                        r = (vec_t)(r * (i / 255.0));
                        g = (vec_t)(g * (i / 255.0));
                        b = (vec_t)(b * (i / 255.0));
                    }
                    else if (values != 3)
                    {
                        logging::warn("Ignoring bad texlight '%s' in info_texlights entity", key);
                        continue;
                    }

                    texlight.name = key;
                    texlight.value[0] = r;
                    texlight.value[1] = g;
                    texlight.value[2] = b;
                    texlight.source = "info_texlights";
                    state.texlights.push_back(texlight);
                }
                found_texlights = true;
            }
            else if (!strcmp(mapent->value("classname"), "info_unlittextures"))
            {
                const std::size_t before = state.unlittextures.size();
                for (size_t ep = 0; ep < mapent->pairs().size(); ep++)
                {
                    const char *key = mapent->pairs()[ep].first.c_str();
                    const char *value = mapent->pairs()[ep].second.c_str();
                    if (!strcmp(key, "classname") || !strcmp(key, "origin"))
                        continue;
                    int enabled = 0;
                    if (sscanf(value, "%d", &enabled) != 1)
                    {
                        logging::warn("Ignoring bad unlit flag '%s' in info_unlittextures entity", key);
                        continue;
                    }
                    if (enabled != 0)
                        state.unlittextures.emplace_back(key);
                }
                logging::info("  %-14s %zu parsed (info_unlittextures)\n", "unlittextures",
                              state.unlittextures.size() - before);
            }
            if (found_minlights && found_texlights)
            {
                break;
            }
        }
    }

    bool is_unlit_texture(const rad_state &state, const char *name)
    {
        for (const std::string &texture : state.unlittextures)
            if (str::iequals(name, texture.c_str()))
                return true;
        return false;
    }

    namespace
    {
        void light_for_texture(const rad_state &state, const char *name, vec3v &result)
        {
            for (size_t it = 0; it < state.texlights.size(); it++)
            {
                if (str::iequals(name, state.texlights[it].name.c_str()))
                {
                    math::copy(state.texlights[it].value, result);
                    return;
                }
            }
            math::clear(result);
        }

        void base_light_for_face(rad_state &state, const format::dface_t *f, vec3v &light)
        {
            int fn = (int)(f - state.map->faces.data());
            if (state.face_texlights[(size_t)fn])
            {
                double r = 0, g = 0, b = 0, scaler = 0;
                switch (sscanf(state.face_texlights[(size_t)fn]->value("_light"), "%lf %lf %lf %lf", &r, &g, &b, &scaler))
                {
                case -1:
                case 0:
                    r = 0.0;
                    g = b = r;
                    break;
                case 1:
                    g = b = r;
                    break;
                case 3:
                    break;
                case 4:
                    r *= scaler / 255.0;
                    g *= scaler / 255.0;
                    b *= scaler / 255.0;
                    break;
                default:
                {
                    vec3v origin;
                    vector_for_key(*state.face_texlights[(size_t)fn], "origin", origin);
                    logging::info("light at (%f,%f,%f) has bad or missing '_light' value : '%s'\n",
                                  origin[0], origin[1], origin[2], state.face_texlights[(size_t)fn]->value("_light"));
                    r = g = b = 0;
                    break;
                }
                }
                light[0] = (vec_t)(r > 0 ? r : 0);
                light[1] = (vec_t)(g > 0 ? g : 0);
                light[2] = (vec_t)(b > 0 ? b : 0);
                return;
            }

            // check for light emitted by texture
            light_for_texture(state, texture_by_number(state, f->texinfo), light);
        }

        bool is_special(const rad_state &state, const format::dface_t *f)
        {
            return (state.map->texinfo[(size_t)f->texinfo].flags & tex_special) != 0;
        }

        // finds a valid world position for the patch origin, and how exposed
        // the patch is
        bool place_patch_inside(rad_state &state, patch *pt)
        {
            const plane *pl;
            const vec3v &face_offset = state.face_offset[(size_t)pt->facenumber];

            pl = plane_from_face_number(state, (unsigned)pt->facenumber);

            vec_t pointsfound;
            vec_t pointstested;
            pointsfound = pointstested = 0;
            bool found;
            vec3v bestpoint;
            vec_t bestdist = -1.0;
            vec3v point;
            vec_t dist;
            vec3v v;

            vec3v center = pt->winding->center();
            found = false;

            math::multiply_add(center, patch_hunt_offset, pl->normal, point);
            pointstested++;
            if (hunt_for_world(state, point, face_offset, *pl, 4, 0.2f, patch_hunt_offset) ||
                hunt_for_world(state, point, face_offset, *pl, 4, 0.8f, patch_hunt_offset))
            {
                pointsfound++;
                math::subtract(point, center, v);
                dist = (vec_t)math::length(v);
                if (!found || dist < bestdist)
                {
                    found = true;
                    math::copy(point, bestpoint);
                    bestdist = dist;
                }
            }
            {
                for (int i = 0; i < pt->winding->size(); i++)
                {
                    const vec3v &p1 = (*pt->winding)[i];
                    const vec3v &p2 = (*pt->winding)[(i + 1) % pt->winding->size()];
                    math::add(p1, p2, point);
                    math::add(point, center, point);
                    math::scale(point, 1.0 / 3.0, point);
                    math::multiply_add(point, patch_hunt_offset, pl->normal, point);
                    pointstested++;
                    if (hunt_for_world(state, point, face_offset, *pl, 4, 0.2f, patch_hunt_offset) ||
                        hunt_for_world(state, point, face_offset, *pl, 4, 0.8f, patch_hunt_offset))
                    {
                        pointsfound++;
                        math::subtract(point, center, v);
                        dist = (vec_t)math::length(v);
                        if (!found || dist < bestdist)
                        {
                            found = true;
                            math::copy(point, bestpoint);
                            bestdist = dist;
                        }
                    }
                }
            }
            pt->exposure = pointsfound / pointstested;

            if (found)
            {
                math::copy(bestpoint, pt->origin);
                pt->flags = patch_flag_null;
                return true;
            }
            else
            {
                math::multiply_add(center, patch_hunt_offset, pl->normal, pt->origin);
                pt->flags = patch_flag_outside;
                return false;
            }
        }

        void update_emitter_info(const rad_state &state, patch *pt)
        {
            static_assert(accuratebounce_default_skylevel + 3 <= skylevel_max, "please raise skylevel_max");
            const vec3v &origin = pt->origin;
            const math::winding *winding = pt->winding;
            vec_t radius = (vec_t)math::on_epsilon;
            for (int x = 0; x < winding->size(); x++)
            {
                vec3v delta;
                vec_t dist;
                math::subtract((*winding)[x], origin, delta);
                dist = (vec_t)math::length(delta);
                if (dist > radius)
                {
                    radius = dist;
                }
            }
            int skylevel = accuratebounce_default_skylevel;
            vec_t area = winding->area();
            vec_t size = 0.8f;
            if (area < size * radius * radius) // the shape is too thin
            {
                skylevel++;
                size *= 0.25f;
                if (area < size * radius * radius)
                {
                    skylevel++;
                    size *= 0.25f;
                    if (area < size * radius * radius)
                    {
                        // stop here: when the area is small the new method
                        // becomes randomized and unstable, so just decrease
                        // the range to limit its use
                        radius = (vec_t)std::sqrt(area / size);
                    }
                }
            }
            pt->emitter_range = accuratebounce_threshold * radius;
            if (state.options.noemitterrange)
            {
                pt->emitter_range = 0.0;
            }
            pt->emitter_skylevel = skylevel;
        }

        // ===== subdivision =====

        // cuts the patch winding along its texel grid; the caller frees the
        // returned windings
        void cut_winding_with_grid(rad_state &state, patch *pt, const plane *pl_a, const plane *pl_b,
                                   std::vector<math::winding *> &windingarray)
        {
            // pl_a->dist and pl_b->dist are not used
            math::winding *winding = nullptr;
            vec_t chop;
            vec_t epsilon;
            const int max_gridsize = 64;
            vec_t gridstart_a;
            vec_t gridstart_b;
            int gridsize_a;
            int gridsize_b;
            vec_t gridchop_a;
            vec_t gridchop_b;
            int numstrips;
            (void)state;

            winding = new math::winding(*pt->winding); // perform all the operations on the copy
            chop = pt->chop;
            chop = chop > 1.0 ? chop : (vec_t)1.0;
            epsilon = 0.6f;

            // optimize the grid
            {
                vec_t min_a;
                vec_t max_a;
                vec_t min_b;
                vec_t max_b;

                min_a = min_b = bogus_range;
                max_a = max_b = -bogus_range;
                for (int x = 0; x < winding->size(); x++)
                {
                    const vec3v &point = (*winding)[x];
                    vec_t dot_a;
                    vec_t dot_b;
                    dot_a = math::dot(point, pl_a->normal);
                    min_a = min_a < dot_a ? min_a : dot_a;
                    max_a = max_a > dot_a ? max_a : dot_a;
                    dot_b = math::dot(point, pl_b->normal);
                    min_b = min_b < dot_b ? min_b : dot_b;
                    max_b = max_b > dot_b ? max_b : dot_b;
                }

                gridchop_a = chop;
                gridsize_a = (int)std::ceil((max_a - min_a - 2 * epsilon) / gridchop_a);
                gridsize_a = gridsize_a > 1 ? gridsize_a : 1;
                if (gridsize_a > max_gridsize)
                {
                    gridsize_a = max_gridsize;
                    gridchop_a = (max_a - min_a) / (vec_t)gridsize_a;
                }
                gridstart_a = (vec_t)((min_a + max_a) / 2.0 - (gridsize_a / 2.0) * gridchop_a);

                gridchop_b = chop;
                gridsize_b = (int)std::ceil((max_b - min_b - 2 * epsilon) / gridchop_b);
                gridsize_b = gridsize_b > 1 ? gridsize_b : 1;
                if (gridsize_b > max_gridsize)
                {
                    gridsize_b = max_gridsize;
                    gridchop_b = (max_b - min_b) / (vec_t)gridsize_b;
                }
                gridstart_b = (vec_t)((min_b + max_b) / 2.0 - (gridsize_b / 2.0) * gridchop_b);
            }
            // cut the winding along the direction of plane a
            {
                for (int i = 1; i < gridsize_a; i++)
                {
                    vec_t dist;
                    math::winding front, back;

                    dist = gridstart_a + i * gridchop_a;
                    winding->clip(pl_a->normal, dist, front, back);

                    if (front.empty() || front.on_plane_side(pl_a->normal, dist, epsilon) == math::winding::side_on) // ended
                    {
                        break;
                    }
                    if (back.empty() || back.on_plane_side(pl_a->normal, dist, epsilon) == math::winding::side_on) // didn't begin
                    {
                        continue;
                    }

                    delete winding;

                    windingarray.push_back(new math::winding(std::move(back)));

                    winding = new math::winding(std::move(front));
                }

                windingarray.push_back(winding);
                winding = nullptr;
            }

            // cut along the direction of plane b
            {
                numstrips = (int)windingarray.size();
                for (int i = 0; i < numstrips; i++)
                {
                    math::winding *strip = windingarray[(size_t)i];
                    windingarray[(size_t)i] = nullptr;

                    for (int j = 1; j < gridsize_b; j++)
                    {
                        vec_t dist;
                        math::winding front, back;

                        dist = gridstart_b + j * gridchop_b;
                        strip->clip(pl_b->normal, dist, front, back);

                        if (front.empty() || front.on_plane_side(pl_b->normal, dist, epsilon) == math::winding::side_on) // ended
                        {
                            break;
                        }
                        if (back.empty() || back.on_plane_side(pl_b->normal, dist, epsilon) == math::winding::side_on) // didn't begin
                        {
                            continue;
                        }

                        delete strip;

                        windingarray.push_back(new math::winding(std::move(back)));

                        strip = new math::winding(std::move(front));
                    }

                    windingarray.push_back(strip);
                }
            }

            delete pt->winding;
            pt->winding = nullptr;
        }

        // determines the perpendicular grid planes to subdivide with, from the
        // patch's texture axes
        void get_grid_planes(const rad_state &state, const patch *pt, plane *pl)
        {
            const format::dface_t *f = &state.map->faces[(size_t)pt->facenumber];
            const format::texinfo_t *tx = &state.map->texinfo[(size_t)f->texinfo];
            plane *p = pl;
            const plane *faceplane = plane_from_face_number(state, (unsigned)pt->facenumber);

            for (int x = 0; x < 2; x++, p++)
            {
                // cut the patch along texel grid planes
                vec_t val;
                vec3v tv{tx->vecs[!x][0], tx->vecs[!x][1], tx->vecs[!x][2]};
                val = math::dot(faceplane->normal, tv);
                math::multiply_add(tv, -val, faceplane->normal, p->normal);
                math::normalize(p->normal);
                p->dist = math::dot(p->normal, pt->origin);
            }
        }

        void subdivide_patch(rad_state &state, patch *pt)
        {
            plane planes[2];
            plane *pl_a = &planes[0];
            plane *pl_b = &planes[1];
            unsigned x;
            patch *new_patch;

            std::vector<math::winding *> windingarray;

            get_grid_planes(state, pt, planes);
            cut_winding_with_grid(state, pt, pl_a, pl_b, windingarray);

            x = 0;
            pt->next = nullptr;
            size_t wi = 0;
            while (windingarray[wi] == nullptr)
            {
                wi++;
                x++;
            }
            pt->winding = windingarray[wi];
            wi++;
            x++;
            pt->area = pt->winding->area();
            pt->origin = pt->winding->center();
            place_patch_inside(state, pt);
            update_emitter_info(state, pt);

            new_patch = state.patches.data() + state.num_patches;
            for (; x < (unsigned)windingarray.size(); x++, wi++)
            {
                if (windingarray[wi])
                {
                    *new_patch = *pt;

                    new_patch->winding = windingarray[wi];
                    new_patch->area = new_patch->winding->area();
                    new_patch->origin = new_patch->winding->center();
                    place_patch_inside(state, new_patch);
                    update_emitter_info(state, new_patch);

                    new_patch++;
                    state.num_patches++;
                    err::require((int)state.num_patches < max_patches, "subdivide_patch: exceeded MAX_PATCHES");
                }
            }

            // sort_patches relinks all the next pointers
        }
    }

    // ===== texture keyed value tables =====

    void read_custom_chop_value(rad_state &state)
    {
        int num;
        int i;

        num = (int)state.textures.size();
        state.chopscales.assign((size_t)num, (vec_t)1.0);
        for (size_t k = 0; k < state.entities.size(); k++)
        {
            format::entity *mapent = &state.entities[k];
            if (strcmp(mapent->value("classname"), "info_chopscale"))
                continue;
            for (i = 0; i < num; i++)
            {
                const char *texname = state.textures[(size_t)i].name;
                for (size_t ep = 0; ep < mapent->pairs().size(); ep++)
                {
                    const char *key = mapent->pairs()[ep].first.c_str();
                    const char *value = mapent->pairs()[ep].second.c_str();
                    if (!str::iequals(key, texname))
                        continue;
                    if (str::iequals(key, "origin"))
                        continue;
                    if (atof(value) <= 0)
                        continue;
                    state.chopscales[(size_t)i] = (vec_t)atof(value);
                }
            }
        }
    }

    namespace
    {
        vec_t chop_scale_for_texture(const rad_state &state, int facenum)
        {
            return state.chopscales[(size_t)state.map->texinfo[(size_t)state.map->faces[(size_t)facenum].texinfo].miptex];
        }
    }

    void read_custom_smooth_value(rad_state &state)
    {
        int num;
        int i;

        num = (int)state.textures.size();
        state.smoothvalues.assign((size_t)num, state.smoothing_threshold);
        for (size_t k = 0; k < state.entities.size(); k++)
        {
            format::entity *mapent = &state.entities[k];
            if (strcmp(mapent->value("classname"), "info_smoothvalue"))
                continue;
            for (i = 0; i < num; i++)
            {
                const char *texname = state.textures[(size_t)i].name;
                for (size_t ep = 0; ep < mapent->pairs().size(); ep++)
                {
                    const char *key = mapent->pairs()[ep].first.c_str();
                    const char *value = mapent->pairs()[ep].second.c_str();
                    if (!str::iequals(key, texname))
                        continue;
                    if (str::iequals(key, "origin"))
                        continue;
                    state.smoothvalues[(size_t)i] = (vec_t)cos(atof(value) * (math::pi / 180.0));
                }
            }
        }
    }

    void read_translucent_textures(rad_state &state)
    {
        int num;
        int i;

        num = (int)state.textures.size();
        state.translucenttextures.assign((size_t)num, vec3v{});
        for (size_t k = 0; k < state.entities.size(); k++)
        {
            format::entity *mapent = &state.entities[k];
            if (strcmp(mapent->value("classname"), "info_translucent"))
                continue;
            for (i = 0; i < num; i++)
            {
                const char *texname = state.textures[(size_t)i].name;
                for (size_t ep = 0; ep < mapent->pairs().size(); ep++)
                {
                    const char *key = mapent->pairs()[ep].first.c_str();
                    const char *value = mapent->pairs()[ep].second.c_str();
                    if (!str::iequals(key, texname))
                        continue;
                    if (str::iequals(key, "origin"))
                        continue;
                    double r, g, b;
                    int count;
                    count = sscanf(value, "%lf %lf %lf", &r, &g, &b);
                    if (count == 1)
                    {
                        g = b = r;
                    }
                    else if (count != 3)
                    {
                        logging::warn("ignore bad translucent value '%s'", value);
                        continue;
                    }
                    if (r < 0.0 || r > 1.0 || g < 0.0 || g > 1.0 || b < 0.0 || b > 1.0)
                    {
                        logging::warn("translucent value should be 0.0-1.0");
                        continue;
                    }
                    state.translucenttextures[(size_t)i][0] = (vec_t)r;
                    state.translucenttextures[(size_t)i][1] = (vec_t)g;
                    state.translucenttextures[(size_t)i][2] = (vec_t)b;
                }
            }
        }
    }

    namespace
    {
        vec_t default_scale_for_power(vec_t power)
        {
            vec_t scale;
            // scale = pi / integrate [2 pi * sin(x) * cos(x) ^ power, {x, 0, pi / 2}]
            scale = (vec_t)((1 + power) / 2.0);
            return scale;
        }
    }

    void read_lighting_cone(rad_state &state)
    {
        int num;
        int i;

        num = (int)state.textures.size();
        state.lightingconeinfo.assign((size_t)num, vec3v{1.0, 1.0, 0.0}); // default power and scale
        for (size_t k = 0; k < state.entities.size(); k++)
        {
            format::entity *mapent = &state.entities[k];
            if (strcmp(mapent->value("classname"), "info_angularfade"))
                continue;
            for (i = 0; i < num; i++)
            {
                const char *texname = state.textures[(size_t)i].name;
                for (size_t ep = 0; ep < mapent->pairs().size(); ep++)
                {
                    const char *key = mapent->pairs()[ep].first.c_str();
                    const char *value = mapent->pairs()[ep].second.c_str();
                    if (!str::iequals(key, texname))
                        continue;
                    if (str::iequals(key, "origin"))
                        continue;
                    double power, scale;
                    int count;
                    count = sscanf(value, "%lf %lf", &power, &scale);
                    if (count == 1)
                    {
                        scale = 1.0;
                    }
                    else if (count != 2)
                    {
                        logging::warn("ignore bad angular fade value '%s'", value);
                        continue;
                    }
                    if (power < 0.0 || scale < 0.0)
                    {
                        logging::warn("ignore disallowed angular fade value '%s'", value);
                        continue;
                    }
                    scale *= default_scale_for_power((vec_t)power);
                    state.lightingconeinfo[(size_t)i][0] = (vec_t)power;
                    state.lightingconeinfo[(size_t)i][1] = (vec_t)scale;
                }
            }
        }
    }

    // ===== patch scale and chop =====

    namespace
    {
        vec_t get_scale(const rad_state &state, const patch *pt)
        {
            const format::dface_t *f = &state.map->faces[(size_t)pt->facenumber];
            const format::texinfo_t *tx = &state.map->texinfo[(size_t)f->texinfo];

            if (state.options.texscale)
            {
                const plane *faceplane = plane_from_face(state, f);
                vec3v vecs_perpendicular[2];
                vec_t scale[2];
                vec_t dot;

                // snap the texture "vecs" to the faceplane without affecting alignment
                for (int x = 0; x < 2; x++)
                {
                    vec3v tv{tx->vecs[x][0], tx->vecs[x][1], tx->vecs[x][2]};
                    dot = math::dot(faceplane->normal, tv);
                    math::multiply_add(tv, -dot, faceplane->normal, vecs_perpendicular[x]);
                }

                {
                    double len0 = math::length(vecs_perpendicular[0]);
                    double len1 = math::length(vecs_perpendicular[1]);
                    scale[0] = (vec_t)(1 / (math::normal_epsilon > len0 ? math::normal_epsilon : len0));
                    scale[1] = (vec_t)(1 / (math::normal_epsilon > len1 ? math::normal_epsilon : len1));
                }

                // the angle between vecs[0] and vecs[1] does not matter,
                // because the grid planes will have the same angle
                return (vec_t)std::sqrt(scale[0] * scale[1]);
            }
            else
            {
                return 1.0;
            }
        }

        bool get_emit_mode(const rad_state &state, const patch *pt)
        {
            bool emitmode = false;
            vec_t value = math::dot(pt->baselight, pt->texturereflectivity) / 3;
            if (state.face_texlights[(size_t)pt->facenumber])
            {
                if (*state.face_texlights[(size_t)pt->facenumber]->value("_scale"))
                {
                    value *= float_for_key(*state.face_texlights[(size_t)pt->facenumber], "_scale");
                }
            }
            if (value > 0.0)
            {
                emitmode = true;
            }
            if (value < state.options.dlight_threshold)
            {
                emitmode = false;
            }
            if (state.face_texlights[(size_t)pt->facenumber])
            {
                switch (int_for_key(*state.face_texlights[(size_t)pt->facenumber], "_fast"))
                {
                case 1:
                    emitmode = false;
                    break;
                case 2:
                    emitmode = true;
                    break;
                }
            }
            return emitmode;
        }

        vec_t get_chop(const rad_state &state, const patch *pt)
        {
            vec_t rval;

            if (state.face_texlights[(size_t)pt->facenumber])
            {
                if (*state.face_texlights[(size_t)pt->facenumber]->value("_chop"))
                {
                    rval = float_for_key(*state.face_texlights[(size_t)pt->facenumber], "_chop");
                    if (rval < 1.0)
                    {
                        rval = 1.0;
                    }
                    return rval;
                }
            }
            if (!pt->emitmode)
            {
                rval = state.options.chop * get_scale(state, pt);
            }
            else
            {
                rval = state.options.texchop * get_scale(state, pt);
            }

            rval *= chop_scale_for_texture(state, pt->facenumber);
            return rval;
        }

        // ===== make patch =====

        void make_patch_for_face(rad_state &state, const int fn, math::winding *w, int style, int bouncestyle)
        {
            const format::dface_t *f = &state.map->faces[(size_t)fn];
            format::map_data &map = *state.map;

            // no patches at all for the sky
            if (!is_special(state, f))
            {
                if (state.face_texlights[(size_t)fn])
                {
                    style = int_for_key(*state.face_texlights[(size_t)fn], "style");
                    if (style < 0)
                        style = -style;
                    style = (unsigned char)style;
                    if (style >= allstyles)
                    {
                        err::fatal("invalid light style: style (%d) >= ALLSTYLES (%d)", style, allstyles);
                    }
                }
                patch *pt;
                vec3v light;
                vec3v centroid{0, 0, 0};

                int numpoints = w->size();

                if (numpoints < 3) // actually happens in real world maps
                {
                    return;
                }
                if (numpoints > max_points_on_winding)
                {
                    err::fatal("numpoints %d > MAX_POINTS_ON_WINDING", numpoints);
                    return;
                }

                pt = &state.patches[state.num_patches];
                err::require((int)state.num_patches < max_patches, "make_patch_for_face: exceeded MAX_PATCHES");
                *pt = patch{};

                pt->winding = w;

                pt->area = pt->winding->area();
                pt->origin = pt->winding->center();
                pt->facenumber = fn;

                state.totalarea += pt->area;

                base_light_for_face(state, f, light);
                math::copy(light, pt->baselight);

                pt->emitstyle = (unsigned char)style;

                math::copy(state.textures[(size_t)map.texinfo[(size_t)f->texinfo].miptex].reflectivity, pt->texturereflectivity);
                if (state.face_texlights[(size_t)fn] && *state.face_texlights[(size_t)fn]->value("_texcolor"))
                {
                    vec3v texturecolor;
                    vec3v texturereflectivity;
                    vector_for_key(*state.face_texlights[(size_t)fn], "_texcolor", texturecolor);
                    for (int k = 0; k < 3; k++)
                    {
                        texturecolor[k] = (vec_t)std::floor(texturecolor[k] + 0.001);
                    }
                    if (vector_minimum(texturecolor) < -0.001 || vector_maximum(texturecolor) > 255.001)
                    {
                        vec3v origin;
                        vector_for_key(*state.face_texlights[(size_t)fn], "origin", origin);
                        err::fatal("light_surface entity at (%g,%g,%g): texture color (%g,%g,%g) must be numbers between 0 and 255.",
                                   origin[0], origin[1], origin[2], texturecolor[0], texturecolor[1], texturecolor[2]);
                    }
                    math::scale(texturecolor, 1.0 / 255.0, texturereflectivity);
                    for (int k = 0; k < 3; k++)
                    {
                        texturereflectivity[k] = (vec_t)std::pow((double)texturereflectivity[k], state.options.texreflectgamma);
                    }
                    math::scale(texturereflectivity, state.options.texreflectscale, texturereflectivity);
                    if (vector_maximum(texturereflectivity) > 1.0 + math::normal_epsilon)
                    {
                        logging::warn("Texture '%s': reflectivity (%f,%f,%f) greater than 1.0.",
                                      state.textures[(size_t)map.texinfo[(size_t)f->texinfo].miptex].name,
                                      texturereflectivity[0], texturereflectivity[1], texturereflectivity[2]);
                    }
                    math::copy(texturereflectivity, pt->texturereflectivity);
                }
                {
                    vec_t opacity = 0.0;
                    if (state.face_entity[(size_t)fn] == &state.entities[0])
                    {
                        opacity = 1.0;
                    }
                    else
                    {
                        size_t x;
                        for (x = 0; x < state.opaque_list.size(); x++)
                        {
                            opaque_entity *op = &state.opaque_list[x];
                            if (op->entitynum == (int)(state.face_entity[(size_t)fn] - state.entities.data()))
                            {
                                opacity = 1.0;
                                if (op->transparency)
                                {
                                    opacity = (vec_t)(1.0 - (op->transparency_scale[0] + op->transparency_scale[1] + op->transparency_scale[2]) / 3);
                                    opacity = opacity > 1.0 ? (vec_t)1.0 : opacity < 0.0 ? (vec_t)0.0 : opacity;
                                }
                                if (op->style != -1)
                                {
                                    // toggleable opaque entity
                                    if (bouncestyle == -1)
                                    {
                                        // by default it does not reflect light
                                        opacity = 0.0;
                                    }
                                }
                                break;
                            }
                        }
                        if (x == state.opaque_list.size())
                        {
                            // not opaque
                            if (bouncestyle != -1)
                            {
                                // with light_bounce it reflects light
                                opacity = 1.0;
                            }
                        }
                    }
                    math::scale(pt->texturereflectivity, opacity, pt->bouncereflectivity);
                }
                pt->bouncestyle = bouncestyle;
                if (bouncestyle == 0)
                {
                    // there is an unnamed light_bounce: reflect light normally
                    pt->bouncestyle = -1;
                }
                pt->emitmode = get_emit_mode(state, pt);
                pt->scale = get_scale(state, pt);
                pt->chop = get_chop(state, pt);
                math::copy(state.translucenttextures[(size_t)map.texinfo[(size_t)f->texinfo].miptex], pt->translucent_v);
                pt->translucent_b = !math::equal(pt->translucent_v, vec3v{});
                place_patch_inside(state, pt);
                update_emitter_info(state, pt);

                state.face_patches[(size_t)fn] = pt;
                state.num_patches++;

                // per face data
                {
                    int j;

                    // centroid of the face for nudging samples in the direct lighting pass
                    for (j = 0; j < f->numedges; j++)
                    {
                        int edge = map.surfedges[(size_t)(f->firstedge + j)];

                        if (edge > 0)
                        {
                            const float *pt0 = map.vertexes[map.edges[(size_t)edge].v[0]].point;
                            const float *pt1 = map.vertexes[map.edges[(size_t)edge].v[1]].point;
                            for (int c = 0; c < 3; c++)
                            {
                                centroid[c] = pt0[c] + centroid[c];
                                centroid[c] = pt1[c] + centroid[c];
                            }
                        }
                        else
                        {
                            const float *pt0 = map.vertexes[map.edges[(size_t)-edge].v[1]].point;
                            const float *pt1 = map.vertexes[map.edges[(size_t)-edge].v[0]].point;
                            for (int c = 0; c < 3; c++)
                            {
                                centroid[c] = pt0[c] + centroid[c];
                                centroid[c] = pt1[c] + centroid[c];
                            }
                        }
                    }

                    // fixup the centroid for anything with an altered origin
                    // (rotating models and turrets mostly)
                    math::scale(centroid, 1.0 / (f->numedges * 2), centroid);
                    math::add(centroid, state.face_offset[(size_t)fn], state.face_centroids[(size_t)fn]);
                }

                {
                    math::basic_bounding_box<vec_t> bounds;
                    pt->winding->bounds(bounds);

                    if (state.options.subdivide)
                    {
                        vec_t amt;
                        vec_t length;
                        vec3v delta;

                        math::subtract(bounds.maxs, bounds.mins, delta);
                        length = (vec_t)math::length(delta);
                        amt = pt->chop;

                        if (length > amt)
                        {
                            if (pt->area < 1.0)
                            {
                            }
                            else
                            {
                                subdivide_patch(state, pt);
                            }
                        }
                    }
                }
            }
        }
    }

    // ===== opaque entities =====

    namespace
    {
        void add_face_to_opaque_list(rad_state &state, int entitynum, int modelnum, const vec3v &origin,
                                     const vec3v &transparency_scale, const bool transparency,
                                     int style, bool block)
        {
            opaque_entity opaque;

            if (transparency && style != -1)
            {
                logging::warn("Dynamic shadow is not allowed in entity with custom shadow.\n");
                style = -1;
            }
            math::copy(transparency_scale, opaque.transparency_scale);
            opaque.transparency = transparency;
            opaque.entitynum = entitynum;
            opaque.modelnum = modelnum;
            math::copy(origin, opaque.origin);
            opaque.style = style;
            opaque.block = block;
            state.opaque_list.push_back(opaque);
        }
    }

    void free_opaque_face_list(rad_state &state)
    {
        state.opaque_list.clear();
        state.opaque_list.shrink_to_fit();
    }

    void load_opaque_entities(rad_state &state)
    {
        format::map_data &map = *state.map;
        int modelnum;

        for (modelnum = 0; modelnum < (int)map.models.size(); modelnum++) // loop through brush models
        {
            char stringmodel[16];
            sprintf(stringmodel, "*%i", modelnum); // model number to string

            for (size_t entnum = 0; entnum < state.entities.size(); entnum++) // loop through map ents
            {
                format::entity *ent = &state.entities[entnum]; // get the current ent

                if (strcmp(ent->value("model"), stringmodel)) // skip ents that don't match the current model
                    continue;
                vec3v origin;
                {
                    vector_for_key(*ent, "origin", origin); // get the origin vector of the ent

                    // if the entity has a light_origin and model_center, calculate a new origin
                    if (*ent->value("light_origin") && *ent->value("model_center"))
                    {
                        format::entity *ent2 = find_target_entity(state, ent->value("light_origin"));

                        if (ent2)
                        {
                            vec3v light_origin, model_center;
                            vector_for_key(*ent2, "origin", light_origin);
                            vector_for_key(*ent, "model_center", model_center);
                            math::subtract(light_origin, model_center, origin); // new origin
                        }
                    }
                }
                bool opaque = false;
                {
                    // -noopaque disables the opaque light flag
                    if (state.options.allow_opaques && (int_for_key(*ent, "zhlt_lightflags") & lightmode_opaque))
                        opaque = true;
                }
                vec3v d_transparency{0.0, 0.0, 0.0};
                bool b_transparency = false;
                {
                    const char *s;

                    if (*(s = ent->value("zhlt_customshadow"))) // custom shadow (transparency) value
                    {
                        double r1 = 1.0, g1 = 1.0, b1 = 1.0, tmp = 1.0;

                        if (sscanf(s, "%lf %lf %lf", &r1, &g1, &b1) == 3) // try to read rgb values
                        {
                            if (r1 < 0.0) r1 = 0.0; // clamp to min 0
                            if (g1 < 0.0) g1 = 0.0;
                            if (b1 < 0.0) b1 = 0.0;
                            d_transparency[0] = (vec_t)r1;
                            d_transparency[1] = (vec_t)g1;
                            d_transparency[2] = (vec_t)b1;
                        }
                        else if (sscanf(s, "%lf", &tmp) == 1) // greyscale version
                        {
                            if (tmp < 0.0) tmp = 0.0;
                            d_transparency = vec3v{(vec_t)tmp, (vec_t)tmp, (vec_t)tmp};
                        }
                    }
                    if (!math::equal(d_transparency, vec3v{})) // not the default: set the transparency flag
                        b_transparency = true;
                }
                int opaquestyle = -1;
                {
                    for (size_t j = 0; j < state.entities.size(); j++) // find a matching light_shadow entity
                    {
                        format::entity *lightent = &state.entities[j];

                        if (!strcmp(lightent->value("classname"), "light_shadow") // light_shadow targeting the current entity
                            && *lightent->value("target")
                            && !strcmp(lightent->value("target"), ent->value("targetname")))
                        {
                            opaquestyle = int_for_key(*lightent, "style"); // get the style number and validate it

                            if (opaquestyle < 0)
                                opaquestyle = -opaquestyle;
                            opaquestyle = (unsigned char)opaquestyle;

                            if (opaquestyle >= allstyles)
                            {
                                err::fatal("invalid light style: style (%d) >= ALLSTYLES (%d)", opaquestyle, allstyles);
                            }
                            break;
                        }
                    }
                }
                bool block = false;
                {
                    if (state.options.blockopaque) // opaque blocking enabled
                    {
                        block = true;

                        // non solid, transparent or styled entities cannot block
                        if (int_for_key(*ent, "zhlt_lightflags") & lightmode_nonsolid)
                            block = false;
                        if (b_transparency)
                            block = false;
                        if (opaquestyle != -1)
                            block = false;
                    }
                }
                if (opaque) // add it to the opaque list with its properties
                {
                    add_face_to_opaque_list(state, (int)entnum, modelnum, origin,
                                            d_transparency, b_transparency, opaquestyle, block);
                }
            }
        }
        {
            int facecount = 0;
            for (size_t i = 0; i < state.opaque_list.size(); i++)
                facecount += count_opaque_faces(state, state.opaque_list[i].modelnum);
            // only report when the map actually has shadow casters
            if (!state.opaque_list.empty())
                logging::info("  %-14s %d models, %d faces\n", "shadowcasters",
                              (int)state.opaque_list.size(), facecount);
        }
    }

    // ===== make patches =====

    format::entity *entity_for_model(rad_state &state, int modnum)
    {
        char name[16];
        sprintf(name, "*%i", modnum);
        for (size_t i = 0; i < state.entities.size(); i++)
        {
            if (!strcmp(state.entities[i].value("model"), name))
            {
                return &state.entities[i];
            }
        }
        return state.entities.empty() ? nullptr : &state.entities[0];
    }

    namespace
    {
        format::entity *find_texlight_entity(rad_state &state, int facenum)
        {
            format::map_data &map = *state.map;
            const format::dface_t *face = &map.faces[(size_t)facenum];
            const plane *dplane = plane_from_face(state, face);
            const char *texname = texture_by_number(state, face->texinfo);
            format::entity *faceent = state.face_entity[(size_t)facenum];
            math::winding w = winding_from_face(state, *face);
            vec3v centroid = w.center();
            math::add(centroid, state.face_offset[(size_t)facenum], centroid);

            format::entity *found = nullptr;
            vec_t bestdist = -1;
            for (size_t i = 0; i < state.entities.size(); i++)
            {
                format::entity *ent = &state.entities[i];
                if (strcmp(ent->value("classname"), "light_surface"))
                    continue;
                if (!str::iequals(ent->value("_tex"), texname))
                    continue;
                vec3v delta;
                vector_for_key(*ent, "origin", delta);
                math::subtract(delta, centroid, delta);
                vec_t dist = (vec_t)math::length(delta);
                if (*ent->value("_frange"))
                {
                    if (dist > float_for_key(*ent, "_frange"))
                        continue;
                }
                if (*ent->value("_fdist"))
                {
                    if (fabs(math::dot(delta, dplane->normal)) > float_for_key(*ent, "_fdist"))
                        continue;
                }
                if (*ent->value("_fclass"))
                {
                    if (strcmp(faceent->value("classname"), ent->value("_fclass")))
                        continue;
                }
                if (*ent->value("_fname"))
                {
                    if (strcmp(faceent->value("targetname"), ent->value("_fname")))
                        continue;
                }
                if (bestdist >= 0 && dist > bestdist)
                    continue;
                found = ent;
                bestdist = dist;
            }
            return found;
        }
    }

    void make_patches(rad_state &state)
    {
        format::map_data &map = *state.map;
        int i;
        int j;
        int fn;
        const format::dmodel_t *mod;
        vec3v origin;
        format::entity *ent;
        const char *s;
        vec3v light_origin{};
        vec3v model_center{};
        bool b_light_origin;
        bool b_model_center;
        unsigned char lightmode;

        int style;

        state.patches.assign((size_t)max_patches, patch{});

        for (i = 0; i < (int)map.models.size(); i++)
        {
            b_light_origin = false;
            b_model_center = false;
            lightmode = lightmode_null;

            mod = &map.models[(size_t)i];
            ent = entity_for_model(state, i);
            math::clear(origin);

            if (*(s = ent->value("zhlt_lightflags")))
            {
                lightmode = (unsigned char)atoi(s);
            }

            // models with origin brushes need to be offset into their in use position
            if (*(s = ent->value("origin")))
            {
                double v1, v2, v3;

                if (sscanf(s, "%lf %lf %lf", &v1, &v2, &v3) == 3)
                {
                    origin[0] = (vec_t)v1;
                    origin[1] = (vec_t)v2;
                    origin[2] = (vec_t)v3;
                }
            }

            // allow models to be lit in an alternate location (pt1)
            if (*(s = ent->value("light_origin")))
            {
                format::entity *e = find_target_entity(state, s);

                if (e)
                {
                    if (*(s = e->value("origin")))
                    {
                        double v1, v2, v3;

                        if (sscanf(s, "%lf %lf %lf", &v1, &v2, &v3) == 3)
                        {
                            light_origin[0] = (vec_t)v1;
                            light_origin[1] = (vec_t)v2;
                            light_origin[2] = (vec_t)v3;

                            b_light_origin = true;
                        }
                    }
                }
            }

            // allow models to be lit in an alternate location (pt2)
            if (*(s = ent->value("model_center")))
            {
                double v1, v2, v3;

                if (sscanf(s, "%lf %lf %lf", &v1, &v2, &v3) == 3)
                {
                    model_center[0] = (vec_t)v1;
                    model_center[1] = (vec_t)v2;
                    model_center[2] = (vec_t)v3;

                    b_model_center = true;
                }
            }

            // allow models to be lit in an alternate location (pt3)
            if (b_light_origin && b_model_center)
            {
                math::subtract(light_origin, model_center, origin);
            }

            if (*(s = ent->value("style")))
            {
                style = atoi(s);
                if (style < 0)
                    style = -style;
            }
            else
            {
                style = 0;
            }
            style = (unsigned char)style;
            if (style >= allstyles)
            {
                err::fatal("invalid light style: style (%d) >= ALLSTYLES (%d)", style, allstyles);
            }
            int bouncestyle = -1;
            {
                for (size_t k = 0; k < state.entities.size(); k++)
                {
                    format::entity *lightent = &state.entities[k];
                    if (!strcmp(lightent->value("classname"), "light_bounce")
                        && *lightent->value("target")
                        && !strcmp(lightent->value("target"), ent->value("targetname")))
                    {
                        bouncestyle = int_for_key(*lightent, "style");
                        if (bouncestyle < 0)
                            bouncestyle = -bouncestyle;
                        bouncestyle = (unsigned char)bouncestyle;
                        if (bouncestyle >= allstyles)
                        {
                            err::fatal("invalid light style: style (%d) >= ALLSTYLES (%d)", bouncestyle, allstyles);
                        }
                        break;
                    }
                }
            }

            for (j = 0; j < mod->numfaces; j++)
            {
                fn = mod->firstface + j;
                state.face_entity[(size_t)fn] = ent;
                math::copy(origin, state.face_offset[(size_t)fn]);
                state.face_texlights[(size_t)fn] = find_texlight_entity(state, fn);
                state.face_lightmode[(size_t)fn] = lightmode;
                math::winding *w = new math::winding(winding_from_face(state, map.faces[(size_t)fn]));
                for (int k = 0; k < w->size(); k++)
                {
                    math::add((*w)[k], origin, (*w)[k]);
                }
                make_patch_for_face(state, fn, w, style, bouncestyle);
            }
        }

        {
            char b1[32], b2[32];
            logging::info("  %-14s %s faces, %s patches\n", "world",
                          str::with_commas((long long)map.faces.size(), b1, sizeof(b1)),
                          str::with_commas((long long)state.num_patches, b2, sizeof(b2)));
        }
    }

    // ===== sort patches =====

    namespace
    {
        int patch_sorter(const void *p1, const void *p2)
        {
            const patch *patch1 = (const patch *)p1;
            const patch *patch2 = (const patch *)p2;

            if (patch1->facenumber < patch2->facenumber)
            {
                return -1;
            }
            else if (patch1->facenumber > patch2->facenumber)
            {
                return 1;
            }
            else
            {
                return 0;
            }
        }
    }

    // sorts the patches by facenumber, which makes their transfer runs
    // compress much better patch addresses change here, so this is where the
    // pool shrinks to its real size
    void sort_patches(rad_state &state)
    {
        std::vector<patch> old_patches = std::move(state.patches);
        state.patches.assign((size_t)state.num_patches + 1, patch{}); // one extra slot like the reference
        memcpy(state.patches.data(), old_patches.data(), state.num_patches * sizeof(patch));
        old_patches.clear();
        old_patches.shrink_to_fit();
        std::qsort(state.patches.data(), (size_t)state.num_patches, sizeof(patch), patch_sorter);

        // fixup face_patches and patch->next
        state.face_patches.assign(state.map->faces.size(), nullptr);
        {
            unsigned x;
            patch *pt = state.patches.data() + 1;
            patch *prev = state.patches.data();

            state.face_patches[(size_t)prev->facenumber] = prev;

            for (x = 1; x < state.num_patches; x++, pt++)
            {
                if (pt->facenumber != prev->facenumber)
                {
                    prev->next = nullptr;
                    state.face_patches[(size_t)pt->facenumber] = pt;
                }
                else
                {
                    prev->next = pt;
                }
                prev = pt;
            }
        }
        for (unsigned x = 0; x < state.num_patches; x++)
        {
            patch *pt = &state.patches[x];
            pt->leafnum = (int)(point_in_leaf(state, pt->origin) - state.map->leafs.data());
        }
    }

    void free_patches(rad_state &state)
    {
        unsigned x;
        patch *pt = state.patches.data();

        for (x = 0; x < state.num_patches; x++, pt++)
        {
            delete pt->winding;
        }
        state.patches.clear();
        state.patches.shrink_to_fit();
        state.num_patches = 0;
    }
}
