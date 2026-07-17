#include <cmath>
#include <cstdlib>

#include "../common/error.h"
#include "internal.h"

// geometry helpers shared by the whole stage: winding point tests, the sample
// position hunts, matrix transforms and the weighted sight area used by
// texlights arithmetic order stays stable for reproducible lighting output

namespace rad
{
    // ===== windings =====

    math::winding winding_from_face(const rad_state &state, const format::dface_t &face)
    {
        const format::map_data &map = *state.map;
        std::vector<vec3v> points((size_t)face.numedges);

        for (int i = 0; i < face.numedges; i++)
        {
            int se = map.surfedges[(size_t)face.firstedge + i];
            int v;
            if (se < 0)
                v = map.edges[(size_t)-se].v[1];
            else
                v = map.edges[(size_t)se].v[0];

            const format::dvertex_t &dv = map.vertexes[(size_t)v];
            points[(size_t)i] = vec3v{dv.point[0], dv.point[1], dv.point[2]};
        }

        math::winding w{std::move(points)};
        w.remove_colinear_points();
        return w;
    }

    // returns whether the point is in the winding, including its edges the
    // point and the winding vertexes can move freely along the plane's normal
    // without changing the result
    bool point_in_winding(const math::winding &w, const plane &pl, const vec3v &point, vec_t epsilon)
    {
        int numpoints = w.size();

        for (int x = 0; x < numpoints; x++)
        {
            vec3v delta;
            vec3v normal;
            math::subtract(w[(x + 1) % numpoints], w[x], delta);
            math::cross(delta, pl.normal, normal);
            vec_t dist = math::dot(point, normal) - math::dot(w[x], normal);

            if (dist < 0.0
                && (epsilon == 0.0 || dist * dist > epsilon * epsilon * math::dot(normal, normal)))
            {
                return false;
            }
        }

        return true;
    }

    // checks whether a ball of the given radius around the point lies entirely
    // inside the winding
    bool point_in_winding_noedge(const math::winding &w, const plane &pl, const vec3v &point, vec_t width)
    {
        int numpoints = w.size();

        for (int x = 0; x < numpoints; x++)
        {
            vec3v delta;
            vec3v normal;
            math::subtract(w[(x + 1) % numpoints], w[x], delta);
            math::cross(delta, pl.normal, normal);
            vec_t dist = math::dot(point, normal) - math::dot(w[x], normal);

            if (dist < 0.0 || dist * dist <= width * width * math::dot(normal, normal))
            {
                return false;
            }
        }

        return true;
    }

    // moves the point to the nearest point inside the winding, preserving the
    // point's distance from the plane
    void snap_to_winding(const math::winding &w, const plane &pl, vec3v &point)
    {
        int numpoints = w.size();
        vec3v delta;
        vec3v normal;
        vec_t dist;
        vec3v bestpoint;
        vec_t bestdist = 0;

        bool in = true;
        for (int x = 0; x < numpoints; x++)
        {
            const vec3v &p1 = w[x];
            const vec3v &p2 = w[(x + 1) % numpoints];
            math::subtract(p2, p1, delta);
            math::cross(delta, pl.normal, normal);
            dist = math::dot(point, normal) - math::dot(p1, normal);

            if (dist < 0.0)
            {
                in = false;

                math::cross(pl.normal, normal, delta);
                vec_t dot = math::dot(delta, point);
                vec_t dot1 = math::dot(delta, p1);
                vec_t dot2 = math::dot(delta, p2);
                if (dot1 < dot && dot < dot2)
                {
                    dist = dist / math::dot(normal, normal);
                    math::multiply_add(point, -dist, normal, point);
                    return;
                }
            }
        }
        if (in)
        {
            return;
        }

        for (int x = 0; x < numpoints; x++)
        {
            const vec3v &p1 = w[x];
            math::subtract(p1, point, delta);
            dist = math::dot(delta, pl.normal) / math::dot(pl.normal, pl.normal);
            math::multiply_add(delta, -dist, pl.normal, delta);
            vec_t dot = math::dot(delta, delta);

            if (x == 0 || dot < bestdist)
            {
                math::add(point, delta, bestpoint);
                bestdist = dot;
            }
        }
        if (numpoints > 0)
        {
            math::copy(bestpoint, point);
        }
    }

    // snaps the point into the winding, then moves it towards the inside until
    // it is not close to any edge or cannot move further returns the maximal
    // distance the point could be kept away from all edges (at most width)
    vec_t snap_to_winding_noedge(const math::winding &w, const plane &pl, vec3v &point, vec_t width, vec_t maxmove)
    {
        int numplanes;
        vec3v v;
        vec_t newwidth;
        vec_t bestwidth;
        vec3v bestpoint;

        snap_to_winding(w, pl, point);

        std::vector<plane> planes((size_t)w.size());
        numplanes = 0;
        for (int x = 0; x < w.size(); x++)
        {
            math::subtract(w[(x + 1) % w.size()], w[x], v);
            math::cross(v, pl.normal, planes[(size_t)numplanes].normal);
            if (!math::normalize(planes[(size_t)numplanes].normal))
            {
                continue;
            }
            planes[(size_t)numplanes].dist = math::dot(w[x], planes[(size_t)numplanes].normal);
            numplanes++;
        }

        bestwidth = 0;
        math::copy(point, bestpoint);
        newwidth = width;

        // binary search for the maximal distance the point can be kept away
        // from all the edges, five iterations like the reference
        for (int pass = 0; pass < 5; pass++)
        {
            bool failed = true;
            vec3v newpoint;

            math::winding newwinding = w;
            for (int x = 0; x < numplanes && !newwinding.empty(); x++)
            {
                plane clipplane = planes[(size_t)x];
                clipplane.dist += newwidth;
                newwinding.clip_in_place(clipplane.normal, clipplane.dist, false);
            }

            if (!newwinding.empty())
            {
                math::copy(point, newpoint);
                snap_to_winding(newwinding, pl, newpoint);

                math::subtract(newpoint, point, v);
                if (math::length(v) <= maxmove + math::on_epsilon)
                {
                    failed = false;
                }
            }

            if (!failed)
            {
                bestwidth = newwidth;
                math::copy(newpoint, bestpoint);
                if (pass == 0)
                {
                    break;
                }
                newwidth = (vec_t)(newwidth + width * std::pow(0.5, pass + 1));
            }
            else
            {
                newwidth = (vec_t)(newwidth - width * std::pow(0.5, pass + 1));
            }
        }

        math::copy(bestpoint, point);
        return bestwidth;
    }

    bool intersect_linesegment_plane(const plane &pl, const vec3v &p1, const vec3v &p2, vec3v &point)
    {
        vec_t part1 = math::dot(p1, pl.normal) - pl.dist;
        vec_t part2 = math::dot(p2, pl.normal) - pl.dist;
        if (part1 * part2 > 0 || part1 == part2)
            return false;
        for (int i = 0; i < 3; ++i)
            point[i] = (part1 * p2[i] - part2 * p1[i]) / (part1 - part2);
        return true;
    }

    void plane_from_points(const vec3v &p1, const vec3v &p2, const vec3v &p3, plane &pl)
    {
        vec3v delta1;
        vec3v delta2;
        vec3v normal;

        math::subtract(p3, p2, delta1);
        math::subtract(p1, p2, delta2);
        math::cross(delta1, delta2, normal);
        math::normalize(normal);
        pl.dist = math::dot(normal, p1);
        math::copy(normal, pl.normal);
    }

    // expands the run length compressed pvs row for a leaf the row length
    // comes from the world model's visleaf count, exactly like the reference
    // (a wrong length makes the source pointer run into invalid data)
    void decompress_vis(const rad_state &state, const byte *src, byte *dest, unsigned int dest_length)
    {
        unsigned int current_length = 0;
        int c;
        byte *out;
        int row;

        row = (state.map->models[0].visleafs + 7) >> 3;
        out = dest;

        const byte *visdata = state.map->visibility.data();
        int visdatasize = (int)state.map->visibility.size();

        do
        {
            err::require(src - visdata < visdatasize, "decompress_vis: overflow");

            if (*src)
            {
                current_length++;
                err::require(current_length <= dest_length, "decompress_vis: overflow");

                *out = *src;
                out++;
                src++;
                continue;
            }

            err::require(&src[1] - visdata < visdatasize, "decompress_vis: overflow");

            c = src[1];
            src += 2;
            while (c)
            {
                current_length++;
                err::require(current_length <= dest_length, "decompress_vis: overflow");

                *out = 0;
                out++;
                c--;

                if (out - dest >= row)
                {
                    return;
                }
            }
        } while (out - dest < row);
    }

    // ===== opaque list =====

    // returns true if the segment is fully blocked by an item in the opaque
    // list partial blockers accumulate into scaleout, style converters into
    // opaquestyleout (-1 = no conversion)
    bool test_segment_against_opaque_list(const rad_state &state, const vec3v &p1, const vec3v &p2,
                                          vec3v &scaleout, int &opaquestyleout)
    {
        scaleout = vec3v{1.0, 1.0, 1.0};
        opaquestyleout = -1;
        for (size_t x = 0; x < state.opaque_list.size(); x++)
        {
            const opaque_entity &oe = state.opaque_list[x];
            if (!test_line_opaque(state, oe.modelnum, oe.origin, p1, p2))
            {
                continue;
            }
            if (oe.transparency)
            {
                math::multiply(scaleout, oe.transparency_scale, scaleout);
                continue;
            }
            if (oe.style != -1 && (opaquestyleout == -1 || oe.style == opaquestyleout))
            {
                opaquestyleout = oe.style;
                continue;
            }
            scaleout = vec3v{0.0, 0.0, 0.0};
            opaquestyleout = -1;
            return true;
        }
        if (test_segment_against_studio_list(state, p1, p2))
        {
            scaleout = vec3v{0.0, 0.0, 0.0};
            opaquestyleout = -1;
            return true;
        }
        if (scaleout[0] < 0.01 && scaleout[1] < 0.01 && scaleout[2] < 0.01)
        {
            // so much shadowing that the result is the same as a normal opaque face
            return true;
        }
        return false;
    }

    // ===== planes =====

    void snap_to_plane(const plane &pl, vec3v &point, vec_t offset)
    {
        vec_t dist = math::dot(point, pl.normal) - pl.dist;
        dist -= offset;
        math::multiply_add(point, -dist, pl.normal, point);
    }

    // widen the plane lump into rad's plane type and build the reversed copies
    // used by faces on the back side of their plane
    void load_planes(rad_state &state)
    {
        const format::map_data &map = *state.map;
        state.planes.resize(map.planes.size());
        state.backplanes.resize(map.planes.size());
        for (size_t i = 0; i < map.planes.size(); i++)
        {
            const format::dplane_t &dp = map.planes[i];
            plane &p = state.planes[i];
            p.normal = vec3v{dp.normal[0], dp.normal[1], dp.normal[2]};
            p.dist = dp.dist;
            p.type = dp.type;

            plane &bp = state.backplanes[i];
            bp.dist = -p.dist;
            for (int j = 0; j < 3; j++)
                bp.normal[j] = 0.0f - p.normal[j];
        }
    }

    const plane *plane_from_face(const rad_state &state, const format::dface_t *face)
    {
        if (!face)
        {
            err::fatal("plane_from_face: face was null");
        }

        if (face->side)
        {
            return &state.backplanes[(size_t)face->planenum];
        }
        else
        {
            return &state.planes[(size_t)face->planenum];
        }
    }

    const plane *plane_from_face_number(const rad_state &state, unsigned facenum)
    {
        const format::dface_t *face = &state.map->faces[facenum];

        if (face->side)
        {
            return &state.backplanes[(size_t)face->planenum];
        }
        else
        {
            return &state.planes[(size_t)face->planenum];
        }
    }

    // plane adjusted for the face offset of origin brush models, used mainly by
    // the opaque code
    void adjusted_plane_from_face_number(const rad_state &state, unsigned facenum, plane &out)
    {
        const format::dface_t *face = &state.map->faces[facenum];
        const vec3v &face_offset = state.face_offset[facenum];

        out.type = 0;

        if (face->side)
        {
            math::copy(state.backplanes[(size_t)face->planenum].normal, out.normal);
            vec_t dist = math::dot(out.normal, face_offset);
            out.dist = state.backplanes[(size_t)face->planenum].dist + dist;
        }
        else
        {
            math::copy(state.planes[(size_t)face->planenum].normal, out.normal);
            vec_t dist = math::dot(out.normal, face_offset);
            out.dist = state.planes[(size_t)face->planenum].dist + dist;
        }
    }

    void translate_plane(plane &pl, const vec3v &delta)
    {
        pl.dist += math::dot(pl.normal, delta);
    }

    // fixes up patch planes for brush models with an origin brush
    vec_t patch_plane_dist(const rad_state &state, const patch *pt)
    {
        const plane *pl = plane_from_face_number(state, (unsigned)pt->facenumber);

        return pl->dist + math::dot(state.face_offset[(size_t)pt->facenumber], pl->normal);
    }

    // ===== leaf lookups =====

    namespace
    {
        format::dleaf_t *point_in_leaf_worst_r(const rad_state &state, int nodenum, const vec3v &point)
        {
            format::map_data &map = *state.map;

            while (nodenum >= 0)
            {
                const format::dnode_t *node = &map.nodes[(size_t)nodenum];
                const plane *pl = &state.planes[(size_t)node->planenum];
                vec_t dist = math::dot(point, pl->normal) - pl->dist;
                if (dist > hunt_wall_epsilon)
                {
                    nodenum = node->children[0];
                }
                else if (dist < -hunt_wall_epsilon)
                {
                    nodenum = node->children[1];
                }
                else
                {
                    format::dleaf_t *result[2];
                    result[0] = point_in_leaf_worst_r(state, node->children[0], point);
                    result[1] = point_in_leaf_worst_r(state, node->children[1], point);
                    format::dleaf_t *leafs = map.leafs.data();
                    if (result[0] == leafs || result[0]->contents == contents_solid)
                        return result[0];
                    if (result[1] == leafs || result[1]->contents == contents_solid)
                        return result[1];
                    if (result[0]->contents == contents_sky)
                        return result[0];
                    if (result[1]->contents == contents_sky)
                        return result[1];
                    if (result[0]->contents == result[1]->contents)
                        return result[0];
                    return leafs;
                }
            }

            return &map.leafs[(size_t)(-nodenum - 1)];
        }
    }

    format::dleaf_t *point_in_leaf_worst(const rad_state &state, const vec3v &point)
    {
        return point_in_leaf_worst_r(state, 0, point);
    }

    format::dleaf_t *point_in_leaf(const rad_state &state, const vec3v &point)
    {
        format::map_data &map = *state.map;
        int nodenum = 0;
        while (nodenum >= 0)
        {
            const format::dnode_t *node = &map.nodes[(size_t)nodenum];
            const plane *pl = &state.planes[(size_t)node->planenum];
            vec_t dist = math::dot(point, pl->normal) - pl->dist;
            if (dist >= 0.0)
            {
                nodenum = node->children[0];
            }
            else
            {
                nodenum = node->children[1];
            }
        }

        return &map.leafs[(size_t)(-nodenum - 1)];
    }

    // radial search for a point that lands in a normal (not sky, not solid)
    // leaf never returns a sky or solid leaf; null when the hunt fails
    format::dleaf_t *hunt_for_world(const rad_state &state, vec3v &point, const vec3v &plane_offset,
                                    const plane &pl, int hunt_size, vec_t hunt_scale, vec_t hunt_offset)
    {
        format::dleaf_t *leaf;
        int x, y, z;

        vec3v current_point;
        vec3v original_point;

        vec3v best_point;
        format::dleaf_t *best_leaf = nullptr;
        vec_t best_dist = (vec_t)99999999.0;

        vec3v scales;

        plane new_plane = pl;

        scales[0] = 0.0;
        scales[1] = -hunt_scale;
        scales[2] = hunt_scale;

        math::copy(point, best_point);
        math::copy(point, original_point);

        translate_plane(new_plane, plane_offset);

        for (int a = 0; a < hunt_size; a++)
        {
            for (x = 0; x < 3; x++)
            {
                current_point[0] = original_point[0] + (scales[x % 3] * a);
                for (y = 0; y < 3; y++)
                {
                    current_point[1] = original_point[1] + (scales[y % 3] * a);
                    for (z = 0; z < 3; z++)
                    {
                        if (a == 0)
                        {
                            if (x || y || z)
                                continue;
                        }
                        vec3v delta;
                        vec_t dist;

                        current_point[2] = original_point[2] + (scales[z % 3] * a);

                        snap_to_plane(new_plane, current_point, hunt_offset);
                        math::subtract(current_point, original_point, delta);
                        dist = math::dot(delta, delta);

                        {
                            size_t ox;
                            for (ox = 0; ox < state.opaque_list.size(); ox++)
                            {
                                const opaque_entity &oe = state.opaque_list[ox];
                                if (test_point_opaque(state, oe.modelnum, oe.origin, oe.block, current_point))
                                    break;
                            }
                            if (ox < state.opaque_list.size())
                                continue;
                        }
                        if (dist < best_dist)
                        {
                            if ((leaf = point_in_leaf_worst(state, current_point)) != state.map->leafs.data())
                            {
                                if ((leaf->contents != contents_sky) && (leaf->contents != contents_solid))
                                {
                                    if (x || y || z)
                                    {
                                        best_dist = dist;
                                        best_leaf = leaf;
                                        math::copy(current_point, best_point);
                                        continue;
                                    }
                                    else
                                    {
                                        math::copy(current_point, point);
                                        return leaf;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if (best_leaf)
            {
                break;
            }
        }

        math::copy(best_point, point);
        return best_leaf;
    }

    // ===== matrices =====

    // out = m * (inx iny inz 1)
    void apply_matrix(const matrix &m, const vec3v &in, vec3v &out)
    {
        err::require(&in != &out, "apply_matrix: in must not alias out");
        out = vec3v{m.v[3][0], m.v[3][1], m.v[3][2]};
        for (int i = 0; i < 3; i++)
        {
            vec3v column{m.v[i][0], m.v[i][1], m.v[i][2]};
            math::multiply_add(out, in[i], column, out);
        }
    }

    // (x y z -dist) * m_inverse; out_normal is not normalized
    void apply_matrix_on_plane(const matrix &m_inverse, const vec3v &in_normal, vec_t in_dist,
                               vec3v &out_normal, vec_t &out_dist)
    {
        err::require(&in_normal != &out_normal, "apply_matrix_on_plane: in must not alias out");
        for (int i = 0; i < 3; i++)
        {
            vec3v column{m_inverse.v[i][0], m_inverse.v[i][1], m_inverse.v[i][2]};
            out_normal[i] = math::dot(in_normal, column);
        }
        vec3v translation{m_inverse.v[3][0], m_inverse.v[3][1], m_inverse.v[3][2]};
        out_dist = -(math::dot(in_normal, translation) - in_dist);
    }

    // applying m_right then m_left equals applying multiply_matrix(m_left, m_right)
    void multiply_matrix(const matrix &m_left, const matrix &m_right, matrix &m)
    {
        const vec_t lastrow[4] = {0, 0, 0, 1};

        err::require(&m != &m_left && &m != &m_right, "multiply_matrix: output must not alias an input");
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                m.v[j][i] = m_left.v[0][i] * m_right.v[j][0]
                          + m_left.v[1][i] * m_right.v[j][1]
                          + m_left.v[2][i] * m_right.v[j][2]
                          + m_left.v[3][i] * lastrow[j];
            }
        }
    }

    matrix multiply_matrix(const matrix &m_left, const matrix &m_right)
    {
        matrix m;
        multiply_matrix(m_left, m_right, m);
        return m;
    }

    void matrix_for_scale(const vec3v &center, vec_t scale, matrix &m)
    {
        for (int i = 0; i < 3; i++)
        {
            m.v[i][0] = 0;
            m.v[i][1] = 0;
            m.v[i][2] = 0;
            m.v[i][i] = scale;
        }
        vec3v translation;
        math::scale(center, 1 - scale, translation);
        m.v[3][0] = translation[0];
        m.v[3][1] = translation[1];
        m.v[3][2] = translation[2];
    }

    matrix matrix_for_scale(const vec3v &center, vec_t scale)
    {
        matrix m;
        matrix_for_scale(center, scale, m);
        return m;
    }

    vec_t calc_matrix_sign(const matrix &m)
    {
        vec3v v0{m.v[0][0], m.v[0][1], m.v[0][2]};
        vec3v v1{m.v[1][0], m.v[1][1], m.v[1][2]};
        vec3v v2{m.v[2][0], m.v[2][1], m.v[2][2]};
        vec3v v = math::cross(v0, v1);
        return math::dot(v, v2);
    }

    // world position to (s, t, distance from plane), without g_face_offset
    void translate_world_to_tex(const rad_state &state, int facenum, matrix &m)
    {
        const format::map_data &map = *state.map;
        const format::dface_t *f = &map.faces[(size_t)facenum];
        const format::texinfo_t *ti = &map.texinfo[(size_t)f->texinfo];
        const plane *fp = plane_from_face(state, f);
        for (int i = 0; i < 3; i++)
        {
            m.v[i][0] = ti->vecs[0][i];
            m.v[i][1] = ti->vecs[1][i];
            m.v[i][2] = fp->normal[i];
        }
        m.v[3][0] = ti->vecs[0][3];
        // the reference reads vecs[1][i] with i left at 3 by the loop above,
        // which lands on the correct offset by accident; keep the same slot
        m.v[3][1] = ti->vecs[1][3];
        m.v[3][2] = -fp->dist;
    }

    namespace
    {
        // three component helpers over the double arrays invert_matrix works
        // in, matching the reference macros operating on double[4] rows
        double dot3(const double *a, const double *b)
        {
            return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        }

        void cross3(const double *a, const double *b, double *out)
        {
            out[0] = a[1] * b[2] - a[2] * b[1];
            out[1] = a[2] * b[0] - a[0] * b[2];
            out[2] = a[0] * b[1] - a[1] * b[0];
        }
    }

    bool invert_matrix(const matrix &m, matrix &m_inverse)
    {
        double texplanes[2][4];
        double faceplane[4];
        int i;
        double texaxis[2][3];
        double normalaxis[3];
        double det, sqrlen1, sqrlen2, sqrlen3;
        double texorg[3];

        for (i = 0; i < 4; i++)
        {
            texplanes[0][i] = m.v[i][0];
            texplanes[1][i] = m.v[i][1];
            faceplane[i] = m.v[i][2];
        }

        sqrlen1 = dot3(texplanes[0], texplanes[0]);
        sqrlen2 = dot3(texplanes[1], texplanes[1]);
        sqrlen3 = dot3(faceplane, faceplane);
        if (sqrlen1 <= math::normal_epsilon * math::normal_epsilon
            || sqrlen2 <= math::normal_epsilon * math::normal_epsilon
            || sqrlen3 <= math::normal_epsilon * math::normal_epsilon)
        {
            // s gradient, t gradient or face normal is too close to 0
            return false;
        }

        cross3(texplanes[0], texplanes[1], normalaxis);
        det = dot3(normalaxis, faceplane);
        if (det * det <= sqrlen1 * sqrlen2 * sqrlen3 * math::normal_epsilon * math::normal_epsilon)
        {
            // s gradient, t gradient and face normal are coplanar
            return false;
        }
        for (i = 0; i < 3; i++)
            normalaxis[i] = normalaxis[i] * (1 / det);

        cross3(texplanes[1], faceplane, texaxis[0]);
        for (i = 0; i < 3; i++)
            texaxis[0][i] = texaxis[0][i] * (1 / det);

        cross3(faceplane, texplanes[0], texaxis[1]);
        for (i = 0; i < 3; i++)
            texaxis[1][i] = texaxis[1][i] * (1 / det);

        for (i = 0; i < 3; i++)
            texorg[i] = normalaxis[i] * -faceplane[3];
        for (i = 0; i < 3; i++)
            texorg[i] = texorg[i] + -texplanes[0][3] * texaxis[0][i];
        for (i = 0; i < 3; i++)
            texorg[i] = texorg[i] + -texplanes[1][3] * texaxis[1][i];

        for (i = 0; i < 3; i++)
        {
            m_inverse.v[0][i] = (vec_t)texaxis[0][i];
            m_inverse.v[1][i] = (vec_t)texaxis[1][i];
            m_inverse.v[2][i] = (vec_t)normalaxis[i];
            m_inverse.v[3][i] = (vec_t)texorg[i];
        }
        return true;
    }

    // ===== sight area =====

    // the weighted solid angle of an emitter winding as seen from a receiver,
    // sampled over the precomputed sky normal set of the given level
    vec_t calc_sight_area(const rad_state &state, const vec3v &receiver_origin, const vec3v &receiver_normal,
                          const math::winding *emitter_winding, int skylevel,
                          vec_t lighting_power, vec_t lighting_scale)
    {
        vec_t area = 0.0;

        int numedges = emitter_winding->size();
        // this function runs once per (sample, nearby texlight patch); avoid heap traffic
        vec3v edges_stack[max_points_on_winding];
        std::vector<vec3v> edges_heap;
        vec3v *edges = edges_stack;
        if (numedges > max_points_on_winding)
        {
            edges_heap.resize((size_t)numedges);
            edges = edges_heap.data();
        }
        bool error = false;
        for (int x = 0; x < numedges; x++)
        {
            vec3v v1, v2, normal;
            math::subtract((*emitter_winding)[x], receiver_origin, v1);
            math::subtract((*emitter_winding)[(x + 1) % numedges], receiver_origin, v2);
            math::cross(v1, v2, normal); // pointing inward
            if (!math::normalize(normal))
            {
                error = true;
            }
            math::copy(normal, edges[x]);
        }
        if (!error)
        {
            const vec3v *pnormal = state.skynormals[skylevel].data();
            const vec_t *psize = state.skynormalsizes[skylevel].data();
            for (int i = 0; i < state.numskynormals[skylevel]; i++, pnormal++, psize++)
            {
                vec_t dot = math::dot(*pnormal, receiver_normal);
                if (dot <= 0)
                    continue;
                int j;
                const vec3v *pedge = edges;
                for (j = 0; j < numedges; j++, pedge++)
                {
                    if (math::dot(*pnormal, *pedge) <= 0)
                    {
                        break;
                    }
                }
                if (j < numedges)
                {
                    continue;
                }
                if (lighting_power != 1.0)
                {
                    dot = (vec_t)std::pow((double)dot, lighting_power);
                }
                area += dot * (*psize);
            }
            area = (vec_t)(area * 4 * math::pi); // convert to absolute sphere area
        }
        area *= lighting_scale;
        return area;
    }

    // as calc_sight_area, but attenuated by the spotlight cone:
    //   ratio = 10 when dot2 >= stopdot, 00 when stopdot2 >= dot2,
    //   linear in between
    vec_t calc_sight_area_spotlight(const rad_state &state, const vec3v &receiver_origin, const vec3v &receiver_normal,
                                    const math::winding *emitter_winding, const vec3v &emitter_normal,
                                    vec_t emitter_stopdot, vec_t emitter_stopdot2, int skylevel,
                                    vec_t lighting_power, vec_t lighting_scale)
    {
        vec_t area = 0.0;

        int numedges = emitter_winding->size();
        vec3v edges_stack[max_points_on_winding];
        std::vector<vec3v> edges_heap;
        vec3v *edges = edges_stack;
        if (numedges > max_points_on_winding)
        {
            edges_heap.resize((size_t)numedges);
            edges = edges_heap.data();
        }
        bool error = false;
        for (int x = 0; x < numedges; x++)
        {
            vec3v v1, v2, normal;
            math::subtract((*emitter_winding)[x], receiver_origin, v1);
            math::subtract((*emitter_winding)[(x + 1) % numedges], receiver_origin, v2);
            math::cross(v1, v2, normal); // pointing inward
            if (!math::normalize(normal))
            {
                error = true;
            }
            math::copy(normal, edges[x]);
        }
        if (!error)
        {
            const vec3v *pnormal = state.skynormals[skylevel].data();
            const vec_t *psize = state.skynormalsizes[skylevel].data();
            for (int i = 0; i < state.numskynormals[skylevel]; i++, pnormal++, psize++)
            {
                vec_t dot = math::dot(*pnormal, receiver_normal);
                if (dot <= 0)
                    continue;
                int j;
                const vec3v *pedge = edges;
                for (j = 0; j < numedges; j++, pedge++)
                {
                    if (math::dot(*pnormal, *pedge) <= 0)
                    {
                        break;
                    }
                }
                if (j < numedges)
                {
                    continue;
                }
                if (lighting_power != 1.0)
                {
                    dot = (vec_t)std::pow((double)dot, lighting_power);
                }
                vec_t dot2 = -math::dot(*pnormal, emitter_normal);
                if (dot2 <= emitter_stopdot2 + math::normal_epsilon)
                {
                    dot = 0;
                }
                else if (dot2 < emitter_stopdot)
                {
                    dot = dot * (dot2 - emitter_stopdot2) / (emitter_stopdot - emitter_stopdot2);
                }
                area += dot * (*psize);
            }
            area = (vec_t)(area * 4 * math::pi); // convert to absolute sphere area
        }
        area *= lighting_scale;
        return area;
    }

    // ===== alternate patch origin =====

    // picks an origin for the visible part of a patch that a clip plane cuts,
    // hunting for a spot that lands in the world
    void get_alternate_origin(const rad_state &state, const vec3v &pos, const vec3v &normal,
                              const patch *pt, vec3v &origin)
    {
        const plane *faceplane = plane_from_face_number(state, (unsigned)pt->facenumber);
        const vec3v &faceplaneoffset = state.face_offset[(size_t)pt->facenumber];
        const vec3v &facenormal = faceplane->normal;
        plane clipplane;
        math::copy(normal, clipplane.normal);
        clipplane.dist = math::dot(pos, clipplane.normal);

        math::winding w = *pt->winding;
        if (w.on_plane_side(clipplane.normal, clipplane.dist) != math::winding::side_cross)
        {
            math::copy(pt->origin, origin);
        }
        else
        {
            w.clip_in_place(clipplane.normal, clipplane.dist, false);
            if (w.empty())
            {
                math::copy(pt->origin, origin);
            }
            else
            {
                bool found;
                vec3v bestpoint;
                vec_t bestdist = -1.0;
                vec3v point;
                vec_t dist;
                vec3v v;

                vec3v center = w.center();
                found = false;

                math::multiply_add(center, patch_hunt_offset, facenormal, point);
                if (hunt_for_world(state, point, faceplaneoffset, *faceplane, 2, 1.0, patch_hunt_offset))
                {
                    math::subtract(point, center, v);
                    dist = (vec_t)math::length(v);
                    if (!found || dist < bestdist)
                    {
                        found = true;
                        math::copy(point, bestpoint);
                        bestdist = dist;
                    }
                }
                if (!found)
                {
                    for (int i = 0; i < w.size(); i++)
                    {
                        const vec3v &p1 = w[i];
                        const vec3v &p2 = w[(i + 1) % w.size()];
                        math::add(p1, p2, point);
                        math::add(point, center, point);
                        math::scale(point, 1.0 / 3.0, point);
                        math::multiply_add(point, patch_hunt_offset, facenormal, point);
                        if (hunt_for_world(state, point, faceplaneoffset, *faceplane, 1, 0.0, patch_hunt_offset))
                        {
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

                if (found)
                {
                    math::copy(bestpoint, origin);
                }
                else
                {
                    math::copy(pt->origin, origin);
                }
            }
        }
    }
}
