#include <algorithm>
#include <cmath>
#include <cstdio>

#include "../common/log.h"
#include "internal.h"

// per face grids of valid sample positions, built once and queried by the
// sample placement code when a lightmap sample's own spot is blocked

namespace rad
{
    namespace
    {
        // whether the (s, t) spot maps to a world position that lands in a
        // normal leaf and does not slip past a wall at the face boundary
        bool is_position_valid(const rad_state &state, position_map *map, const vec3v &pos_st, vec3v &pos_out,
                               bool usephongnormal = true, bool doedgetest = true,
                               int hunt_size = 2, vec_t hunt_scale = 0.2f)
        {
            vec3v pos;
            vec3v pos_normal;
            vec_t hunt_offset;

            apply_matrix(map->textoworld, pos_st, pos);
            math::add(pos, map->face_offset, pos);
            if (usephongnormal)
            {
                get_phong_normal(state, map->facenum, pos, pos_normal);
            }
            else
            {
                math::copy(map->faceplanewithoffset.normal, pos_normal);
            }
            math::multiply_add(pos, default_hunt_offset, pos_normal, pos);

            // might be smaller than default_hunt_offset
            hunt_offset = math::dot(pos, map->faceplanewithoffset.normal) - map->faceplanewithoffset.dist;

            // push the point 02 units around to avoid walls
            if (!hunt_for_world(state, pos, vec3v{}, map->faceplanewithoffset, hunt_size, hunt_scale, hunt_offset))
            {
                return false;
            }

            if (doedgetest && !point_in_winding_noedge(map->facewindingwithoffset, map->faceplanewithoffset, pos, default_edge_width))
            {
                // the sample has gone beyond the face boundaries; be careful
                // that it has not passed a wall
                vec3v test;
                vec3v transparency;
                int opaquestyle;

                math::copy(pos, test);
                snap_to_winding_noedge(map->facewindingwithoffset, map->faceplanewithoffset, test,
                                       default_edge_width, 4 * default_edge_width);

                if (!hunt_for_world(state, test, vec3v{}, map->faceplanewithoffset, hunt_size, hunt_scale, hunt_offset))
                {
                    return false;
                }

                if (test_line(state, pos, test) != contents_empty)
                {
                    return false;
                }

                if (test_segment_against_opaque_list(state, pos, test, transparency, opaquestyle) == true
                    || opaquestyle != -1)
                {
                    return false;
                }
            }

            math::copy(pos, pos_out);
            return true;
        }

        void calc_single_position(const rad_state &state, position_map *map, int is, int it)
        {
            sample_position *p;
            vec_t smin, smax, tmin, tmax;
            plane clipplanes[4];
            const vec3v v_s = {1, 0, 0};
            const vec3v v_t = {0, 1, 0};

            p = &map->grid[(size_t)(is + map->w * it)];
            smin = map->start[0] + is * map->step[0];
            smax = map->start[0] + (is + 1) * map->step[0];
            tmin = map->start[1] + it * map->step[1];
            tmax = map->start[1] + (it + 1) * map->step[1];

            math::scale(v_s, 1, clipplanes[0].normal);
            clipplanes[0].dist = smin;
            math::scale(v_s, -1, clipplanes[1].normal);
            clipplanes[1].dist = -smax;
            math::scale(v_t, 1, clipplanes[2].normal);
            clipplanes[2].dist = tmin;
            math::scale(v_t, -1, clipplanes[3].normal);
            clipplanes[3].dist = -tmax;

            p->nudged = true; // nudged unless the position comes directly from its s,t
            math::winding zone = map->texwinding;
            for (int x = 0; x < 4 && !zone.empty(); x++)
            {
                zone.clip_in_place(clipplanes[x].normal, clipplanes[x].dist, false);
            }
            if (zone.empty())
            {
                p->valid = false;
            }
            else
            {
                vec3v original_st;
                vec3v test_st;

                original_st[0] = (vec_t)(map->start[0] + (is + 0.5) * map->step[0]);
                original_st[1] = (vec_t)(map->start[1] + (it + 0.5) * map->step[1]);
                original_st[2] = 0.0;

                p->valid = false;

                if (!p->valid)
                {
                    math::copy(original_st, test_st);
                    snap_to_winding(zone, map->texplane, test_st);

                    if (is_position_valid(state, map, test_st, p->pos))
                    {
                        p->valid = true;
                        p->nudged = false;
                        p->best_s = test_st[0];
                        p->best_t = test_st[1];
                    }
                }

                if (!p->valid)
                {
                    test_st = zone.center();
                    if (is_position_valid(state, map, test_st, p->pos))
                    {
                        p->valid = true;
                        p->best_s = test_st[0];
                        p->best_t = test_st[1];
                    }
                }

                if (!p->valid && !state.options.fastmode)
                {
                    const int numnudges = 12;
                    vec3v nudgelist[numnudges] = {{0.1f, 0, 0}, {-0.1f, 0, 0}, {0, 0.1f, 0}, {0, -0.1f, 0},
                                                  {0.3f, 0, 0}, {-0.3f, 0, 0}, {0, 0.3f, 0}, {0, -0.3f, 0},
                                                  {0.3f, 0.3f, 0}, {-0.3f, 0.3f, 0}, {-0.3f, -0.3f, 0}, {0.3f, -0.3f, 0}};

                    for (int i = 0; i < numnudges; i++)
                    {
                        math::multiply(nudgelist[i], map->step, test_st);
                        math::add(test_st, original_st, test_st);
                        snap_to_winding(zone, map->texplane, test_st);

                        if (is_position_valid(state, map, test_st, p->pos))
                        {
                            p->valid = true;
                            p->best_s = test_st[0];
                            p->best_t = test_st[1];
                            break;
                        }
                    }
                }
            }
        }
    }

    // must run after face_offset, face_centroids and edgeshares are calculated
    void find_face_positions(rad_state &state, int facenum)
    {
        format::map_data &bspmap = *state.map;
        const format::dface_t *f = &bspmap.faces[(size_t)facenum];
        position_map *map = &state.face_positions[(size_t)facenum];
        const vec3v v_up = {0, 0, 1};
        vec_t density;
        vec_t texmins[2] = {}, texmaxs[2] = {};
        int imins[2], imaxs[2];
        int x;
        int k;

        map->valid = true;
        map->facenum = facenum;
        map->facewinding = math::winding{};
        map->facewindingwithoffset = math::winding{};
        map->texwinding = math::winding{};
        map->grid.clear();

        const format::texinfo_t *ti = &bspmap.texinfo[(size_t)f->texinfo];
        if (ti->flags & tex_special)
        {
            map->valid = false;
            return;
        }

        math::copy(state.face_offset[(size_t)facenum], map->face_offset);
        math::copy(state.face_centroids[(size_t)facenum], map->face_centroid);
        translate_world_to_tex(state, facenum, map->worldtotex);
        if (!invert_matrix(map->worldtotex, map->textoworld))
        {
            map->valid = false;
            return;
        }

        map->facewinding = winding_from_face(state, *f);
        map->faceplane = *plane_from_face(state, f);
        {
            std::vector<vec3v> pts((size_t)map->facewinding.size());
            for (x = 0; x < map->facewinding.size(); x++)
            {
                math::add(map->facewinding[x], map->face_offset, pts[(size_t)x]);
            }
            map->facewindingwithoffset = math::winding{std::move(pts)};
        }
        map->faceplanewithoffset = map->faceplane;
        map->faceplanewithoffset.dist = map->faceplane.dist + math::dot(map->face_offset, map->faceplane.normal);

        {
            std::vector<vec3v> pts((size_t)map->facewinding.size());
            for (x = 0; x < map->facewinding.size(); x++)
            {
                apply_matrix(map->worldtotex, map->facewinding[x], pts[(size_t)x]);
                pts[(size_t)x][2] = 0.0;
            }
            map->texwinding = math::winding{std::move(pts)};
        }
        map->texwinding.remove_colinear_points();
        math::copy(v_up, map->texplane.normal);
        if (calc_matrix_sign(map->worldtotex) < 0.0)
        {
            map->texplane.normal[2] *= -1;
        }
        map->texplane.dist = 0.0;
        if (map->texwinding.empty())
        {
            map->facewinding = math::winding{};
            map->facewindingwithoffset = math::winding{};
            map->texwinding = math::winding{};
            map->valid = false;
            return;
        }
        vec3v v;
        math::subtract(map->face_centroid, map->face_offset, v);
        apply_matrix(map->worldtotex, v, map->texcentroid);
        map->texcentroid[2] = 0.0;

        for (x = 0; x < map->texwinding.size(); x++)
        {
            for (k = 0; k < 2; k++)
            {
                if (x == 0 || map->texwinding[x][k] < texmins[k])
                    texmins[k] = map->texwinding[x][k];
                if (x == 0 || map->texwinding[x][k] > texmaxs[k])
                    texmaxs[k] = map->texwinding[x][k];
            }
        }
        density = 3.0;
        if (state.options.fastmode)
        {
            density = 1.0;
        }
        map->step[0] = (vec_t)texture_step / density;
        map->step[1] = (vec_t)texture_step / density;
        map->step[2] = 1.0;
        for (k = 0; k < 2; k++)
        {
            imins[k] = (int)std::floor(texmins[k] / map->step[k] + 0.5 - math::on_epsilon);
            imaxs[k] = (int)std::ceil(texmaxs[k] / map->step[k] - 0.5 + math::on_epsilon);
        }
        map->start[0] = (vec_t)((imins[0] - 0.5) * map->step[0]);
        map->start[1] = (vec_t)((imins[1] - 0.5) * map->step[1]);
        map->start[2] = 0.0;
        map->w = imaxs[0] - imins[0] + 1;
        map->h = imaxs[1] - imins[1] + 1;
        if (map->w <= 0 || map->h <= 0 || (double)map->w * (double)map->h > 99999999)
        {
            map->facewinding = math::winding{};
            map->facewindingwithoffset = math::winding{};
            map->texwinding = math::winding{};
            map->valid = false;
            return;
        }

        map->grid.resize((size_t)(map->w * map->h));

        for (int it = 0; it < map->h; it++)
        {
            for (int is = 0; is < map->w; is++)
            {
                calc_single_position(state, map, is, it);
            }
        }
    }

    void free_position_maps(rad_state &state)
    {
        if (state.options.drawsample)
        {
            std::string name = state.base_path + "_positions.pts";
            logging::info("Writing '%s' ...\n", name.c_str());
            FILE *f = std::fopen(name.c_str(), "w");
            if (f)
            {
                const int pos_count = 15;
                const vec3v pos[pos_count] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {-1, 0, 0}, {0, -1, 0},
                                              {1, 0, 0}, {0, 0, 1}, {-1, 0, 0}, {0, 0, -1}, {0, -1, 0},
                                              {0, 0, 1}, {0, 1, 0}, {0, 0, -1}, {1, 0, 0}, {0, 0, 0}};
                vec3v v, dist;
                vec3v origin{state.options.drawsample_origin[0], state.options.drawsample_origin[1],
                             state.options.drawsample_origin[2]};
                for (size_t i = 0; i < state.map->faces.size(); ++i)
                {
                    position_map *map = &state.face_positions[i];
                    if (!map->valid)
                    {
                        continue;
                    }
                    for (int j = 0; j < map->h * map->w; ++j)
                    {
                        if (!map->grid[(size_t)j].valid)
                        {
                            continue;
                        }
                        math::copy(map->grid[(size_t)j].pos, v);
                        math::subtract(v, origin, dist);
                        if (math::dot(dist, dist) < state.options.drawsample_radius * state.options.drawsample_radius)
                        {
                            for (int k = 0; k < pos_count; ++k)
                                std::fprintf(f, "%g %g %g\n", v[0] + pos[k][0], v[1] + pos[k][1], v[2] + pos[k][2]);
                        }
                    }
                }
                std::fclose(f);
                logging::info("OK.\n");
            }
            else
                logging::info("Error.\n");
        }
        for (size_t facenum = 0; facenum < state.map->faces.size(); facenum++)
        {
            position_map *map = &state.face_positions[facenum];
            if (map->valid)
            {
                map->facewinding = math::winding{};
                map->facewindingwithoffset = math::winding{};
                map->texwinding = math::winding{};
                map->grid.clear();
                map->grid.shrink_to_fit();
                map->valid = false;
            }
        }
    }

    bool find_nearest_position(const rad_state &state, int facenum, const math::winding *texwinding,
                               const plane &texplane, vec_t s, vec_t t, vec3v &pos,
                               vec_t *best_s, vec_t *best_t, vec_t *dist, bool *nudged)
    {
        const position_map *map;
        vec3v original_st;
        int x;
        int itmin, itmax, ismin, ismax;
        int is;
        int it;
        vec3v v;
        bool found;
        int best_is = 0;
        int best_it = 0;
        vec_t best_dist = 0;

        map = &state.face_positions[(size_t)facenum];
        if (!map->valid)
        {
            return false;
        }

        original_st[0] = s;
        original_st[1] = t;
        original_st[2] = 0.0;

        if (point_in_winding(map->texwinding, map->texplane, original_st, (vec_t)(4 * math::on_epsilon)))
        {
            itmin = (int)std::ceil((original_st[1] - map->start[1] - 2 * math::on_epsilon) / map->step[1]) - 1;
            itmax = (int)std::floor((original_st[1] - map->start[1] + 2 * math::on_epsilon) / map->step[1]);
            ismin = (int)std::ceil((original_st[0] - map->start[0] - 2 * math::on_epsilon) / map->step[0]) - 1;
            ismax = (int)std::floor((original_st[0] - map->start[0] + 2 * math::on_epsilon) / map->step[0]);
            itmin = std::max(0, itmin);
            itmax = std::min(itmax, map->h - 1);
            ismin = std::max(0, ismin);
            ismax = std::min(ismax, map->w - 1);

            found = false;
            bool best_nudged = true;
            for (it = itmin; it <= itmax; it++)
            {
                for (is = ismin; is <= ismax; is++)
                {
                    const sample_position *p;
                    vec3v current_st;
                    vec_t d;

                    p = &map->grid[(size_t)(is + map->w * it)];
                    if (!p->valid)
                    {
                        continue;
                    }
                    current_st[0] = p->best_s;
                    current_st[1] = p->best_t;
                    current_st[2] = 0.0;

                    math::subtract(current_st, original_st, v);
                    d = (vec_t)math::length(v);

                    if (!found
                        || (!p->nudged && best_nudged)
                        || (p->nudged == best_nudged && d < best_dist - 2 * math::on_epsilon))
                    {
                        found = true;
                        best_is = is;
                        best_it = it;
                        best_dist = d;
                        best_nudged = p->nudged;
                    }
                }
            }

            if (found)
            {
                const sample_position *p = &map->grid[(size_t)(best_is + map->w * best_it)];
                math::copy(p->pos, pos);
                *best_s = p->best_s;
                *best_t = p->best_t;
                *dist = 0.0;
                *nudged = p->nudged;
                return true;
            }
        }
        *nudged = true;

        itmin = map->h;
        itmax = -1;
        ismin = map->w;
        ismax = -1;
        for (x = 0; x < texwinding->size(); x++)
        {
            it = (int)std::floor(((*texwinding)[x][1] - map->start[1] + 0.5 * math::on_epsilon) / map->step[1]);
            itmin = std::min(itmin, it);
            it = (int)std::ceil(((*texwinding)[x][1] - map->start[1] - 0.5 * math::on_epsilon) / map->step[1]) - 1;
            itmax = std::max(it, itmax);
            is = (int)std::floor(((*texwinding)[x][0] - map->start[0] + 0.5 * math::on_epsilon) / map->step[0]);
            ismin = std::min(ismin, is);
            is = (int)std::ceil(((*texwinding)[x][0] - map->start[0] - 0.5 * math::on_epsilon) / map->step[0]) - 1;
            ismax = std::max(is, ismax);
        }
        itmin = std::max(0, itmin);
        itmax = std::min(itmax, map->h - 1);
        ismin = std::max(0, ismin);
        ismax = std::min(ismax, map->w - 1);

        found = false;
        for (it = itmin; it <= itmax; it++)
        {
            for (is = ismin; is <= ismax; is++)
            {
                const sample_position *p;
                vec3v current_st;
                vec_t d;

                p = &map->grid[(size_t)(is + map->w * it)];
                if (!p->valid)
                {
                    continue;
                }
                current_st[0] = p->best_s;
                current_st[1] = p->best_t;
                current_st[2] = 0.0;

                math::subtract(current_st, original_st, v);
                d = (vec_t)math::length(v);

                if (!found || d < best_dist - math::on_epsilon)
                {
                    found = true;
                    best_is = is;
                    best_it = it;
                    best_dist = d;
                }
            }
        }

        if (found)
        {
            const sample_position *p = &map->grid[(size_t)(best_is + map->w * best_it)];
            math::copy(p->pos, pos);
            *best_s = p->best_s;
            *best_t = p->best_t;
            *dist = best_dist;
            return true;
        }

        return false;
    }
}
