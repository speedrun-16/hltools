#include <cmath>
#include <cstdlib>
#include <cstring>

#include "../common/error.h"
#include "../common/log.h"
#include "format/bsp/face_extents.h"
#include "internal.h"

// the lightmap core: edge pairing for phong smoothing, sample placement with
// growth across smooth edges, direct light creation, the per sample light
// gather (the hottest loop of the stage) and build_facelights lazy
// accumulators and a contiguous light leaf list keep the hot path compact

namespace rad
{
    constexpr int max_singlemap = (limits::max_surface_extent + 1) * (limits::max_surface_extent + 1);

    // ===== pair edges =====

    namespace
    {
        struct intersect_test
        {
            int numclipplanes;
            std::vector<plane> clipplanes;
        };

        bool test_face_intersect(const rad_state &state, intersect_test *t, int facenum)
        {
            const format::dface_t *f2 = &state.map->faces[(size_t)facenum];
            math::winding w = winding_from_face(state, *f2);
            int k;
            for (k = 0; k < w.size(); k++)
            {
                math::add(w[k], state.face_offset[(size_t)facenum], w[k]);
            }
            for (k = 0; k < t->numclipplanes; k++)
            {
                if (!w.clip_in_place(t->clipplanes[(size_t)k].normal, t->clipplanes[(size_t)k].dist, false))
                {
                    break;
                }
            }
            bool intersect = w.size() > 0;
            return intersect;
        }

        intersect_test *create_intersect_test(const rad_state &state, const plane *p, int facenum)
        {
            const format::dface_t *f = &state.map->faces[(size_t)facenum];
            intersect_test *t = new intersect_test;
            t->clipplanes.resize((size_t)f->numedges);
            t->numclipplanes = 0;
            for (int j = 0; j < f->numedges; j++)
            {
                int edgenum = state.map->surfedges[(size_t)(f->firstedge + j)];
                {
                    vec3v v0, v1;
                    vec3v dir, normal;
                    if (edgenum < 0)
                    {
                        const float *pt0 = state.map->vertexes[state.map->edges[(size_t)-edgenum].v[1]].point;
                        const float *pt1 = state.map->vertexes[state.map->edges[(size_t)-edgenum].v[0]].point;
                        v0 = vec3v{pt0[0], pt0[1], pt0[2]};
                        v1 = vec3v{pt1[0], pt1[1], pt1[2]};
                    }
                    else
                    {
                        const float *pt0 = state.map->vertexes[state.map->edges[(size_t)edgenum].v[0]].point;
                        const float *pt1 = state.map->vertexes[state.map->edges[(size_t)edgenum].v[1]].point;
                        v0 = vec3v{pt0[0], pt0[1], pt0[2]};
                        v1 = vec3v{pt1[0], pt1[1], pt1[2]};
                    }
                    math::add(v0, state.face_offset[(size_t)facenum], v0);
                    math::add(v1, state.face_offset[(size_t)facenum], v1);
                    math::subtract(v1, v0, dir);
                    math::cross(dir, p->normal, normal); // facing inward
                    if (!math::normalize(normal))
                    {
                        continue;
                    }
                    math::copy(normal, t->clipplanes[(size_t)t->numclipplanes].normal);
                    t->clipplanes[(size_t)t->numclipplanes].dist = math::dot(v0, normal);
                    t->numclipplanes++;
                }
            }
            return t;
        }

        void free_intersect_test(intersect_test *t)
        {
            delete t;
        }

        // walks around a vertex from face to face across smooth edges,
        // returning the angle each face contributes at the vertex
        int add_face_for_vertex_normal(const rad_state &state, const int edgeabs, int &edgeabsnext,
                                       const int edgeend, int &edgeendnext,
                                       format::dface_t *const f, format::dface_t *&fnext,
                                       vec_t &angle, vec3v &normal)
        {
            const format::map_data &map = *state.map;
            math::copy(plane_from_face(state, f)->normal, normal);
            int vnum = map.edges[(size_t)edgeabs].v[edgeend];
            int iedge = 0, iedgenext = 0, edge = 0, edgenext = 0;
            int i, e, count1, count2;
            vec_t dot;
            for (count1 = count2 = 0, i = 0; i < f->numedges; i++)
            {
                e = map.surfedges[(size_t)(f->firstedge + i)];
                if (map.edges[(size_t)abs(e)].v[0] == map.edges[(size_t)abs(e)].v[1])
                    continue;
                if (abs(e) == edgeabs)
                {
                    iedge = i;
                    edge = e;
                    count1++;
                }
                else if (map.edges[(size_t)abs(e)].v[0] == vnum || map.edges[(size_t)abs(e)].v[1] == vnum)
                {
                    iedgenext = i;
                    edgenext = e;
                    count2++;
                }
            }
            (void)iedge;
            (void)iedgenext;
            if (count1 != 1 || count2 != 1)
            {
                return -1;
            }
            int vnum11, vnum12, vnum21, vnum22;
            vec3v vec1, vec2;
            vnum11 = map.edges[(size_t)abs(edge)].v[edge > 0 ? 0 : 1];
            vnum12 = map.edges[(size_t)abs(edge)].v[edge > 0 ? 1 : 0];
            vnum21 = map.edges[(size_t)abs(edgenext)].v[edgenext > 0 ? 0 : 1];
            vnum22 = map.edges[(size_t)abs(edgenext)].v[edgenext > 0 ? 1 : 0];
            if (vnum == vnum12 && vnum == vnum21 && vnum != vnum11 && vnum != vnum22)
            {
                for (int x = 0; x < 3; x++)
                {
                    vec1[x] = map.vertexes[(size_t)vnum11].point[x] - map.vertexes[(size_t)vnum].point[x];
                    vec2[x] = map.vertexes[(size_t)vnum22].point[x] - map.vertexes[(size_t)vnum].point[x];
                }
                edgeabsnext = abs(edgenext);
                edgeendnext = edgenext > 0 ? 0 : 1;
            }
            else if (vnum == vnum11 && vnum == vnum22 && vnum != vnum12 && vnum != vnum21)
            {
                for (int x = 0; x < 3; x++)
                {
                    vec1[x] = map.vertexes[(size_t)vnum12].point[x] - map.vertexes[(size_t)vnum].point[x];
                    vec2[x] = map.vertexes[(size_t)vnum21].point[x] - map.vertexes[(size_t)vnum].point[x];
                }
                edgeabsnext = abs(edgenext);
                edgeendnext = edgenext > 0 ? 1 : 0;
            }
            else
            {
                return -1;
            }
            math::normalize(vec1);
            math::normalize(vec2);
            dot = math::dot(vec1, vec2);
            dot = dot > 1 ? 1 : dot < -1 ? -1 : dot;
            angle = std::acos(dot);
            const edgeshare *es = &state.edgeshares[(size_t)edgeabsnext];
            if (!(es->faces[0] && es->faces[1]))
                return 1;
            if (es->faces[0] == f && es->faces[1] != f)
                fnext = es->faces[1];
            else if (es->faces[1] == f && es->faces[0] != f)
                fnext = es->faces[0];
            else
            {
                return -1;
            }
            return 0;
        }

        // creates a matrix translating texture coords in face1 into texture
        // coords in face2, keeping all points on the common edge invariant
        bool translate_tex_to_tex(const rad_state &state, int facenum, int edgenum, int facenum2,
                                  matrix &m, matrix &m_inverse)
        {
            matrix worldtotex;
            matrix worldtotex2;
            int i;
            vec3v face_vert[2];
            vec3v face2_vert[2];
            vec3v face_axis[2];
            vec3v face2_axis[2];
            const vec3v v_up = {0, 0, 1};
            vec_t len;
            vec_t len2;
            matrix edgetotex, edgetotex2;
            matrix inv, inv2;

            translate_world_to_tex(state, facenum, worldtotex);
            translate_world_to_tex(state, facenum2, worldtotex2);

            const format::dedge_t *e = &state.map->edges[(size_t)edgenum];
            for (i = 0; i < 2; i++)
            {
                const float *pt = state.map->vertexes[e->v[i]].point;
                vec3v point{pt[0], pt[1], pt[2]};
                apply_matrix(worldtotex, point, face_vert[i]);
                face_vert[i][2] = 0; // naturally close to 0 assuming the edge is on the face plane
                apply_matrix(worldtotex2, point, face2_vert[i]);
                face2_vert[i][2] = 0;
            }

            math::subtract(face_vert[1], face_vert[0], face_axis[0]);
            len = (vec_t)math::length(face_axis[0]);
            math::cross(v_up, face_axis[0], face_axis[1]);
            if (calc_matrix_sign(worldtotex) < 0.0)
            {
                // the three vectors s, t, facenormal are in reverse order
                face_axis[1] = -face_axis[1];
            }

            math::subtract(face2_vert[1], face2_vert[0], face2_axis[0]);
            len2 = (vec_t)math::length(face2_axis[0]);
            math::cross(v_up, face2_axis[0], face2_axis[1]);
            if (calc_matrix_sign(worldtotex2) < 0.0)
            {
                face2_axis[1] = -face2_axis[1];
            }

            for (i = 0; i < 3; i++)
            {
                edgetotex.v[0][i] = face_axis[0][i]; // a rotation (possibly with a reflection by the edge)
                edgetotex.v[1][i] = face_axis[1][i];
                edgetotex.v[2][i] = v_up[i] * len; // encode the length into the 3rd value of the matrix
                edgetotex.v[3][i] = face_vert[0][i]; // map (0,0) into the origin point

                edgetotex2.v[0][i] = face2_axis[0][i];
                edgetotex2.v[1][i] = face2_axis[1][i];
                edgetotex2.v[2][i] = v_up[i] * len2;
                edgetotex2.v[3][i] = face2_vert[0][i];
            }

            if (!invert_matrix(edgetotex, inv) || !invert_matrix(edgetotex2, inv2))
            {
                return false;
            }
            multiply_matrix(edgetotex2, inv, m);
            multiply_matrix(edgetotex, inv2, m_inverse);

            return true;
        }
    }

    void pair_edges(rad_state &state)
    {
        format::map_data &map = *state.map;
        int i, j, k;
        format::dface_t *f;
        edgeshare *e;

        state.edgeshares.assign(map.edges.size(), edgeshare{});

        f = map.faces.data();
        for (i = 0; i < (int)map.faces.size(); i++, f++)
        {
            if (map.texinfo[(size_t)f->texinfo].flags & tex_special)
            {
                // special textures don't have lightmaps
                continue;
            }
            for (j = 0; j < f->numedges; j++)
            {
                k = map.surfedges[(size_t)(f->firstedge + j)];
                if (k < 0)
                {
                    e = &state.edgeshares[(size_t)-k];
                    e->faces[1] = f;
                }
                else
                {
                    e = &state.edgeshares[(size_t)k];
                    e->faces[0] = f;
                }

                if (e->faces[0] && e->faces[1])
                {
                    // determine if coplanar
                    if (e->faces[0]->planenum == e->faces[1]->planenum
                        && e->faces[0]->side == e->faces[1]->side)
                    {
                        e->coplanar = true;
                        math::copy(plane_from_face(state, e->faces[0])->normal, e->interface_normal);
                        e->cos_normals_angle = 1.0;
                    }
                    else
                    {
                        // see if they fall into a smoothing group based on the angle of the normals
                        vec3v normals[2];

                        math::copy(plane_from_face(state, e->faces[0])->normal, normals[0]);
                        math::copy(plane_from_face(state, e->faces[1])->normal, normals[1]);

                        e->cos_normals_angle = math::dot(normals[0], normals[1]);

                        vec_t smoothvalue;
                        int m0 = map.texinfo[(size_t)e->faces[0]->texinfo].miptex;
                        int m1 = map.texinfo[(size_t)e->faces[1]->texinfo].miptex;
                        smoothvalue = state.smoothvalues[(size_t)m0] > state.smoothvalues[(size_t)m1]
                            ? state.smoothvalues[(size_t)m0] : state.smoothvalues[(size_t)m1];
                        if (m0 != m1)
                        {
                            smoothvalue = smoothvalue > state.smoothing_threshold_2 ? smoothvalue : state.smoothing_threshold_2;
                        }
                        if (smoothvalue >= 1.0 - math::normal_epsilon)
                        {
                            smoothvalue = 2.0;
                        }
                        if (e->cos_normals_angle > (1.0 - math::normal_epsilon))
                        {
                            e->coplanar = true;
                            math::copy(plane_from_face(state, e->faces[0])->normal, e->interface_normal);
                            e->cos_normals_angle = 1.0;
                        }
                        else
                        {
                            // the reference compares against qmax(smoothvalue -
                            // normal_epsilon, normal_epsilon) in double
                            double threshold = (smoothvalue - math::normal_epsilon) > math::normal_epsilon
                                ? (smoothvalue - math::normal_epsilon) : math::normal_epsilon;
                            if (e->cos_normals_angle >= threshold)
                            {
                                math::add(normals[0], normals[1], e->interface_normal);
                                math::normalize(e->interface_normal);
                            }
                        }
                    }
                    if (!math::equal(state.translucenttextures[(size_t)map.texinfo[(size_t)e->faces[0]->texinfo].miptex],
                                     state.translucenttextures[(size_t)map.texinfo[(size_t)e->faces[1]->texinfo].miptex]))
                    {
                        e->coplanar = false;
                        math::clear(e->interface_normal);
                    }
                    {
                        int miptex0, miptex1;
                        miptex0 = map.texinfo[(size_t)e->faces[0]->texinfo].miptex;
                        miptex1 = map.texinfo[(size_t)e->faces[1]->texinfo].miptex;
                        if (std::fabs(state.lightingconeinfo[(size_t)miptex0][0] - state.lightingconeinfo[(size_t)miptex1][0]) > math::normal_epsilon ||
                            std::fabs(state.lightingconeinfo[(size_t)miptex0][1] - state.lightingconeinfo[(size_t)miptex1][1]) > math::normal_epsilon)
                        {
                            e->coplanar = false;
                            math::clear(e->interface_normal);
                        }
                    }
                    if (!math::equal(e->interface_normal, vec3v{}))
                    {
                        e->smooth = true;
                    }
                    if (e->smooth)
                    {
                        // compute the matrix in advance
                        if (!translate_tex_to_tex(state, (int)(e->faces[0] - map.faces.data()), abs(k),
                                                  (int)(e->faces[1] - map.faces.data()), e->textotex[0], e->textotex[1]))
                        {
                            e->smooth = false;
                            e->coplanar = false;
                            math::clear(e->interface_normal);
                        }
                    }
                }
            }
        }
        {
            int edgeabs, edgeabsnext;
            int edgeend, edgeendnext;
            int d;
            format::dface_t *fl, *fcurrent, *fnext;
            vec_t angle = 0, angles;
            vec3v normal, normals;
            vec3v edgenormal;
            int r, count;
            for (edgeabs = 0; edgeabs < (int)state.edgeshares.size(); edgeabs++)
            {
                e = &state.edgeshares[(size_t)edgeabs];
                if (!e->smooth)
                    continue;
                math::copy(e->interface_normal, edgenormal);
                if (map.edges[(size_t)edgeabs].v[0] == map.edges[(size_t)edgeabs].v[1])
                {
                    math::copy(edgenormal, e->vertex_normal[0]);
                    math::copy(edgenormal, e->vertex_normal[1]);
                }
                else
                {
                    const plane *p0 = plane_from_face(state, e->faces[0]);
                    const plane *p1 = plane_from_face(state, e->faces[1]);
                    intersect_test *test0 = create_intersect_test(state, p0, (int)(e->faces[0] - map.faces.data()));
                    intersect_test *test1 = create_intersect_test(state, p1, (int)(e->faces[1] - map.faces.data()));
                    for (edgeend = 0; edgeend < 2; edgeend++)
                    {
                        angles = 0;
                        math::clear(normals);

                        for (d = 0; d < 2; d++)
                        {
                            fl = e->faces[d];
                            count = 0, fnext = fl, edgeabsnext = edgeabs, edgeendnext = edgeend;
                            while (1)
                            {
                                fcurrent = fnext;
                                r = add_face_for_vertex_normal(state, edgeabsnext, edgeabsnext, edgeendnext, edgeendnext,
                                                               fcurrent, fnext, angle, normal);
                                count++;
                                if (r == -1)
                                {
                                    break;
                                }
                                if (count >= 100)
                                {
                                    break;
                                }
                                if (math::dot(normal, p0->normal) <= math::normal_epsilon
                                    || math::dot(normal, p1->normal) <= math::normal_epsilon)
                                    break;
                                vec_t smoothvalue;
                                int m0 = map.texinfo[(size_t)fl->texinfo].miptex;
                                int m1 = map.texinfo[(size_t)fcurrent->texinfo].miptex;
                                smoothvalue = state.smoothvalues[(size_t)m0] > state.smoothvalues[(size_t)m1]
                                    ? state.smoothvalues[(size_t)m0] : state.smoothvalues[(size_t)m1];
                                if (m0 != m1)
                                {
                                    smoothvalue = smoothvalue > state.smoothing_threshold_2 ? smoothvalue : state.smoothing_threshold_2;
                                }
                                if (smoothvalue >= 1.0 - math::normal_epsilon)
                                {
                                    smoothvalue = 2.0;
                                }
                                {
                                    double threshold = (smoothvalue - math::normal_epsilon) > math::normal_epsilon
                                        ? (smoothvalue - math::normal_epsilon) : math::normal_epsilon;
                                    if (math::dot(edgenormal, normal) < threshold)
                                        break;
                                }
                                if (fcurrent != e->faces[0] && fcurrent != e->faces[1] &&
                                    (test_face_intersect(state, test0, (int)(fcurrent - map.faces.data()))
                                     || test_face_intersect(state, test1, (int)(fcurrent - map.faces.data()))))
                                {
                                    break;
                                }
                                angles += angle;
                                math::multiply_add(normals, angle, normal, normals);
                                {
                                    bool in = false;
                                    if (fcurrent == e->faces[0] || fcurrent == e->faces[1])
                                    {
                                        in = true;
                                    }
                                    for (facelist *l = e->vertex_facelist[edgeend]; l; l = l->next)
                                    {
                                        if (fcurrent == l->face)
                                        {
                                            in = true;
                                        }
                                    }
                                    if (!in)
                                    {
                                        facelist *l = (facelist *)malloc(sizeof(facelist));
                                        err::require(l != nullptr, "pair_edges: out of memory");
                                        l->face = fcurrent;
                                        l->next = e->vertex_facelist[edgeend];
                                        e->vertex_facelist[edgeend] = l;
                                    }
                                }
                                if (r != 0 || fnext == fl)
                                    break;
                            }
                        }

                        if (angles < math::normal_epsilon)
                        {
                            math::copy(edgenormal, e->vertex_normal[edgeend]);
                        }
                        else
                        {
                            math::normalize(normals);
                            math::copy(normals, e->vertex_normal[edgeend]);
                        }
                    }
                    free_intersect_test(test0);
                    free_intersect_test(test1);
                }
                if (e->coplanar)
                {
                    if (!math::equal(e->vertex_normal[0], e->interface_normal)
                        || !math::equal(e->vertex_normal[1], e->interface_normal))
                    {
                        e->coplanar = false;
                    }
                }
            }
        }
    }

    // ===== light info =====

    enum wallflag
    {
        wallflag_none = 0,
        wallflag_nudged = 0x1,
        // only happens when the entire face and its surroundings are covered
        // by solid or opaque entities
        wallflag_blocked = 0x2,
        wallflag_shadowed = 0x4,
    };

    struct lightinfo
    {
        vec_t facedist;
        vec3v facenormal;
        bool translucent_b;
        vec3v translucent_v;
        int miptex;

        int numsurfpt;
        vec3v surfpt[max_singlemap];
        // surfpt_position are valid positions for light tracing, while surfpt
        // are positions for phong normals and patch interpolation
        vec3v *surfpt_position;
        int *surfpt_surface; // the face that owns this position
        bool surfpt_lightoutside[max_singlemap];

        vec3v texorg;
        vec3v worldtotex[2]; // s = (world - texorg)  worldtotex[0]
        vec3v textoworld[2]; // world = texorg + s * textoworld[0]
        vec3v texnormal;

        vec_t exactmins[2], exactmaxs[2];

        int texmins[2], texsize[2];
        int surfnum;
        format::dface_t *face;
        int lmcache_density; // shared by both s and t direction
        int lmcache_offset;  // shared by both s and t direction
        int lmcache_side;
        vec3v (*lmcache)[allstyles]; // lm: short for lightmap
        vec3v *lmcache_normal;       // the phong normals
        int *lmcache_wallflags;      // wallflag values
        int lmcachewidth;
        int lmcacheheight;
    };

    // ===== face extents and vectors =====

    namespace
    {
        const char *texture_name_from_face(const rad_state &state, const format::dface_t *f)
        {
            return texture_by_number(state, f->texinfo);
        }

        // fills in texmins, texsize, exactmins, exactmaxs and the sample cache
        void calc_face_extents(rad_state &state, lightinfo *l)
        {
            const int facenum = l->surfnum;
            format::dface_t *s;
            float mins[2], maxs[2], val;
            int i, j, e;
            const format::dvertex_t *v;
            const format::texinfo_t *tex;
            const format::map_data &map = *state.map;

            s = l->face;

            mins[0] = mins[1] = 99999999.0f;
            maxs[0] = maxs[1] = -99999999.0f;

            tex = &map.texinfo[(size_t)s->texinfo];

            for (i = 0; i < s->numedges; i++)
            {
                e = map.surfedges[(size_t)(s->firstedge + i)];
                if (e >= 0)
                {
                    v = &map.vertexes[map.edges[(size_t)e].v[0]];
                }
                else
                {
                    v = &map.vertexes[map.edges[(size_t)-e].v[1]];
                }

                for (j = 0; j < 2; j++)
                {
                    val = v->point[0] * tex->vecs[j][0] +
                        v->point[1] * tex->vecs[j][1] + v->point[2] * tex->vecs[j][2] + tex->vecs[j][3];
                    if (val < mins[j])
                    {
                        mins[j] = val;
                    }
                    if (val > maxs[j])
                    {
                        maxs[j] = val;
                    }
                }
            }

            for (i = 0; i < 2; i++)
            {
                l->exactmins[i] = mins[i];
                l->exactmaxs[i] = maxs[i];
            }
            int bmins[2];
            int bmaxs[2];
            format::get_face_extents(*state.map, l->surfnum, bmins, bmaxs);
            for (i = 0; i < 2; i++)
            {
                mins[i] = (float)bmins[i];
                maxs[i] = (float)bmaxs[i];
                l->texmins[i] = bmins[i];
                l->texsize[i] = bmaxs[i] - bmins[i];
            }

            if (!(tex->flags & tex_special))
            {
                if ((l->texsize[0] > limits::max_surface_extent) || (l->texsize[1] > limits::max_surface_extent)
                    || l->texsize[0] < 0 || l->texsize[1] < 0)
                {
                    std::lock_guard<std::mutex> guard(state.lock);
                    logging::warn("\nfor Face %d (texture %s) at ", (int)(s - map.faces.data()), texture_name_from_face(state, s));

                    for (i = 0; i < s->numedges; i++)
                    {
                        e = map.surfedges[(size_t)(s->firstedge + i)];
                        if (e >= 0)
                        {
                            v = &map.vertexes[map.edges[(size_t)e].v[0]];
                        }
                        else
                        {
                            v = &map.vertexes[map.edges[(size_t)-e].v[1]];
                        }
                        vec3v pos;
                        vec3v point{v->point[0], v->point[1], v->point[2]};
                        math::add(point, state.face_offset[(size_t)facenum], pos);
                        logging::info("(%4.3f %4.3f %4.3f) ", pos[0], pos[1], pos[2]);
                    }
                    logging::info("\n");

                    err::fatal("Bad surface extents (%d x %d)\nCheck the file ZHLTProblems.html for a detailed explanation of this problem", l->texsize[0], l->texsize[1]);
                }
            }
            // allocate sample light cache
            {
                if (state.options.extra && !state.options.fastmode)
                {
                    l->lmcache_density = 3;
                }
                else
                {
                    l->lmcache_density = 1;
                }
                l->lmcache_side = (int)std::ceil((0.5 * state.options.blur * l->lmcache_density - 0.5) * (1 - math::normal_epsilon));
                l->lmcache_offset = l->lmcache_side;
                l->lmcachewidth = l->texsize[0] * l->lmcache_density + 1 + 2 * l->lmcache_side;
                l->lmcacheheight = l->texsize[1] * l->lmcache_density + 1 + 2 * l->lmcache_side;
                l->lmcache = (vec3v (*)[allstyles])malloc((size_t)(l->lmcachewidth * l->lmcacheheight) * sizeof(vec3v[allstyles]));
                err::require(l->lmcache != nullptr, "calc_face_extents: out of memory");
                l->lmcache_normal = (vec3v *)malloc((size_t)(l->lmcachewidth * l->lmcacheheight) * sizeof(vec3v));
                err::require(l->lmcache_normal != nullptr, "calc_face_extents: out of memory");
                l->lmcache_wallflags = (int *)malloc((size_t)(l->lmcachewidth * l->lmcacheheight) * sizeof(int));
                err::require(l->lmcache_wallflags != nullptr, "calc_face_extents: out of memory");
                l->surfpt_position = (vec3v *)malloc(max_singlemap * sizeof(vec3v));
                l->surfpt_surface = (int *)malloc(max_singlemap * sizeof(int));
                err::require(l->surfpt_position != nullptr && l->surfpt_surface != nullptr, "calc_face_extents: out of memory");
            }
        }

        // fills in texorg, worldtotex and textoworld
        void calc_face_vectors(rad_state &state, lightinfo *l)
        {
            const format::texinfo_t *tex;
            int i, j;
            vec3v texnormal;
            vec_t distscale;
            vec_t dist, len;

            tex = &state.map->texinfo[(size_t)l->face->texinfo];

            for (i = 0; i < 2; i++)
            {
                for (j = 0; j < 3; j++)
                {
                    l->worldtotex[i][j] = tex->vecs[i][j];
                }
            }

            // calculate a normal to the texture axis points can be moved
            // along this without changing their s/t
            {
                vec3v tv1{tex->vecs[1][0], tex->vecs[1][1], tex->vecs[1][2]};
                vec3v tv0{tex->vecs[0][0], tex->vecs[0][1], tex->vecs[0][2]};
                math::cross(tv1, tv0, texnormal);
            }
            math::normalize(texnormal);

            // flip it towards the plane normal
            distscale = math::dot(texnormal, l->facenormal);
            if (distscale == 0.0)
            {
                err::fatal("Malformed face (%d) normal", (int)(l->face - state.map->faces.data()));
            }

            if (distscale < 0)
            {
                distscale = -distscale;
                for (i = 0; i < 3; i++)
                    texnormal[i] = 0.0f - texnormal[i];
            }

            // distscale is the ratio of the distance along the texture normal
            // to the distance along the plane normal
            distscale = (vec_t)(1.0 / distscale);

            for (i = 0; i < 2; i++)
            {
                math::cross(l->worldtotex[!i], l->facenormal, l->textoworld[i]);
                len = math::dot(l->textoworld[i], l->worldtotex[i]);
                math::scale(l->textoworld[i], 1 / len, l->textoworld[i]);
            }

            // calculate texorg on the texture plane
            for (i = 0; i < 3; i++)
            {
                l->texorg[i] = -tex->vecs[0][3] * l->textoworld[0][i] - tex->vecs[1][3] * l->textoworld[1][i];
            }

            // project back to the face plane
            dist = math::dot(l->texorg, l->facenormal) - l->facedist;
            dist *= distscale;
            math::multiply_add(l->texorg, -dist, texnormal, l->texorg);
            math::copy(texnormal, l->texnormal);
        }

        void set_surf_from_st(const rad_state &state, const lightinfo *l, vec3v &surf, const vec_t s, const vec_t t)
        {
            const int facenum = l->surfnum;
            int j;

            for (j = 0; j < 3; j++)
            {
                surf[j] = l->texorg[j] + l->textoworld[0][j] * s + l->textoworld[1][j] * t;
            }

            // adjust for origin based models
            math::add(surf, state.face_offset[(size_t)facenum], surf);
        }

        void set_st_from_surf(const rad_state &state, const lightinfo *l, const vec3v &surf, vec_t &s, vec_t &t)
        {
            const int facenum = l->surfnum;
            int j;

            s = t = 0;
            for (j = 0; j < 3; j++)
            {
                s += (surf[j] - state.face_offset[(size_t)facenum][j] - l->texorg[j]) * l->worldtotex[0][j];
                t += (surf[j] - state.face_offset[(size_t)facenum][j] - l->texorg[j]) * l->worldtotex[1][j];
            }
        }
    }

    enum light_flag
    {
        light_outside,        // not lit
        light_shifted,        // used hunt_for_world on a fully dark face
        light_shifted_inside, // moved to a neighbor on the second cleanup pass
        light_normal,         // normally lit with no movement
        light_pulled_inside,  // pulled inside by bleed code adjustments
        light_simple_nudge,   // a simple nudge 1/3 or 2/3 towards center along an axis
    };

    // ===== sample fragments (growing samples across smooth edges) =====

    namespace
    {
        struct samplefragedge
        {
            int edgenum; // edge lump index
            int edgeside;
            int nextfacenum; // where to grow
            bool tried;

            vec3v point1;    // start point
            vec3v point2;    // end point
            vec3v direction; // normalized, from point1 to point2

            bool noseam;
            vec_t distance; // distance from origin
            vec_t distancereduction;
            vec_t flippedangle;

            vec_t ratio; // if ratio != 1, a seam is unavoidable
            matrix prevtonext;
            matrix nexttoprev;
        };

        struct samplefragrect
        {
            plane planes[4];
        };

        struct samplefrag
        {
            samplefrag *next;       // since this is a node in a list
            samplefrag *parentfrag; // where it grew from
            samplefragedge *parentedge;
            int facenum;

            vec_t flippedangle; // copied from the parent edge
            bool noseam;        // copied from the parent edge

            matrix coordtomycoord; // v[2][2] > 0, v[2][0] = v[2][1] = v[0][2] = v[1][2] = 00
            matrix mycoordtocoord;

            vec3v origin;         // original s,t
            vec3v myorigin;       // relative to the texture coordinate on that face
            samplefragrect rect;  // original rectangle that forms the boundary
            samplefragrect myrect; // relative to the texture coordinate on that face

            math::winding *winding;    // a fragment of the original rectangle in the texture plane
            plane windingplane;        // normal is (0,0,1) or (0,0,-1)
            math::winding *mywinding;  // relative to the texture coordinate on that face
            plane mywindingplane;

            int numedges;         // number of candidates for the next growth
            samplefragedge *edges; // candidates for the next growth
        };

        struct samplefraginfo
        {
            int maxsize;
            int size;
            samplefrag *head;
        };

        // fills winding, windingplane, mywinding, mywindingplane, numedges, edges
        void chop_frag(const rad_state &state, samplefrag *frag)
        {
            // get the shape of the fragment by clipping the face using the boundaries
            const format::dface_t *f;
            matrix worldtotex;
            const vec3v v_up = {0, 0, 1};
            const format::map_data &map = *state.map;

            f = &map.faces[(size_t)frag->facenum];
            math::winding facewinding = winding_from_face(state, *f);

            translate_world_to_tex(state, frag->facenum, worldtotex);
            {
                std::vector<vec3v> pts((size_t)facewinding.size());
                for (int x = 0; x < facewinding.size(); x++)
                {
                    apply_matrix(worldtotex, facewinding[x], pts[(size_t)x]);
                    pts[(size_t)x][2] = 0.0;
                }
                frag->mywinding = new math::winding{std::move(pts)};
            }
            frag->mywinding->remove_colinear_points();
            // the same as applying the worldtotex matrix to the faceplane
            math::copy(v_up, frag->mywindingplane.normal);
            if (calc_matrix_sign(worldtotex) < 0.0)
            {
                frag->mywindingplane.normal[2] *= -1;
            }
            frag->mywindingplane.dist = 0.0;

            for (int x = 0; x < 4 && frag->mywinding->size() > 0; x++)
            {
                frag->mywinding->clip_in_place(frag->myrect.planes[x].normal, frag->myrect.planes[x].dist, false);
            }

            {
                std::vector<vec3v> pts((size_t)frag->mywinding->size());
                for (int x = 0; x < frag->mywinding->size(); x++)
                {
                    apply_matrix(frag->mycoordtocoord, (*frag->mywinding)[x], pts[(size_t)x]);
                }
                frag->winding = new math::winding{std::move(pts)};
            }
            frag->winding->remove_colinear_points();
            math::copy(frag->mywindingplane.normal, frag->windingplane.normal);
            if (calc_matrix_sign(frag->mycoordtocoord) < 0.0)
            {
                frag->windingplane.normal[2] *= -1;
            }
            frag->windingplane.dist = 0.0;

            // find the edges where the fragment can grow in the future
            frag->numedges = 0;
            frag->edges = (samplefragedge *)malloc((size_t)f->numedges * sizeof(samplefragedge));
            err::require(frag->edges != nullptr, "chop_frag: out of memory");
            for (int i = 0; i < f->numedges; i++)
            {
                samplefragedge *e;
                const edgeshare *es;
                const format::dedge_t *de;
                vec_t frac1, frac2;
                vec_t edgelen;
                vec_t dot, dot1, dot2;
                vec3v tmp, v, normal;
                const matrix *m;
                const matrix *m_inverse;

                e = &frag->edges[frag->numedges];

                // some basic info
                e->edgenum = abs(map.surfedges[(size_t)(f->firstedge + i)]);
                e->edgeside = (map.surfedges[(size_t)(f->firstedge + i)] < 0 ? 1 : 0);
                es = &state.edgeshares[(size_t)e->edgenum];
                if (!es->smooth)
                {
                    continue;
                }
                if ((int)(es->faces[e->edgeside] - map.faces.data()) != frag->facenum)
                {
                    err::fatal("internal error 1 in GrowSingleSampleFrag");
                }
                m = &es->textotex[e->edgeside];
                m_inverse = &es->textotex[1 - e->edgeside];
                e->nextfacenum = (int)(es->faces[1 - e->edgeside] - map.faces.data());
                if (e->nextfacenum == frag->facenum)
                {
                    continue; // an invalid edge (usually very short)
                }
                e->tried = false; // because the frag hasn't been linked into the list yet

                // translate the edge points from world to the texture plane of
                // the original frag, so the distances are comparable among
                // edges from different frags
                de = &map.edges[(size_t)e->edgenum];
                const float *dv1 = map.vertexes[de->v[e->edgeside]].point;
                const float *dv2 = map.vertexes[de->v[1 - e->edgeside]].point;
                {
                    vec3v p1{dv1[0], dv1[1], dv1[2]};
                    apply_matrix(worldtotex, p1, tmp);
                    apply_matrix(frag->mycoordtocoord, tmp, e->point1);
                    e->point1[2] = 0.0;
                    vec3v p2{dv2[0], dv2[1], dv2[2]};
                    apply_matrix(worldtotex, p2, tmp);
                    apply_matrix(frag->mycoordtocoord, tmp, e->point2);
                    e->point2[2] = 0.0;
                }
                math::subtract(e->point2, e->point1, e->direction);
                edgelen = math::normalize(e->direction);
                if (edgelen <= math::on_epsilon)
                {
                    continue;
                }

                // clip the edge
                frac1 = 0;
                frac2 = 1;
                for (int x = 0; x < 4; x++)
                {
                    vec_t d1;
                    vec_t d2;

                    d1 = math::dot(e->point1, frag->rect.planes[x].normal) - frag->rect.planes[x].dist;
                    d2 = math::dot(e->point2, frag->rect.planes[x].normal) - frag->rect.planes[x].dist;
                    if (d1 <= math::on_epsilon && d2 <= math::on_epsilon)
                    {
                        frac1 = 1;
                        frac2 = 0;
                    }
                    else if (d1 < 0)
                    {
                        vec_t f1 = d1 / (d1 - d2);
                        frac1 = frac1 > f1 ? frac1 : f1;
                    }
                    else if (d2 < 0)
                    {
                        vec_t f2 = d1 / (d1 - d2);
                        frac2 = frac2 < f2 ? frac2 : f2;
                    }
                }
                if (edgelen * (frac2 - frac1) <= math::on_epsilon)
                {
                    continue;
                }
                math::multiply_add(e->point1, edgelen * frac2, e->direction, e->point2);
                math::multiply_add(e->point1, edgelen * frac1, e->direction, e->point1);

                // calculate the distance etc which determine its priority
                e->noseam = frag->noseam;
                dot = math::dot(frag->origin, e->direction);
                dot1 = math::dot(e->point1, e->direction);
                dot2 = math::dot(e->point2, e->direction);
                dot = dot < dot2 ? dot : dot2;
                dot = dot1 > dot ? dot1 : dot;
                math::multiply_add(e->point1, dot - dot1, e->direction, v);
                math::subtract(v, frag->origin, v);
                e->distance = (vec_t)math::length(v);
                math::cross(e->direction, frag->windingplane.normal, normal);
                math::normalize(normal); // points inward
                e->distancereduction = math::dot(v, normal);
                e->flippedangle = (vec_t)(frag->flippedangle + std::acos(es->cos_normals_angle < 1.0 ? (double)es->cos_normals_angle : 1.0));

                // calculate the matrix
                e->ratio = (*m_inverse).v[2][2];
                if (e->ratio <= math::normal_epsilon || (1 / e->ratio) <= math::normal_epsilon)
                {
                    continue;
                }

                if (std::fabs(e->ratio - 1) < 0.005)
                {
                    e->prevtonext = *m;
                    e->nexttoprev = *m_inverse;
                }
                else
                {
                    e->noseam = false;
                    e->prevtonext = *m;
                    e->nexttoprev = *m_inverse;
                }

                frag->numedges++;
            }
        }

        samplefrag *grow_single_frag(const rad_state &state, const samplefraginfo *info, samplefrag *parent, samplefragedge *edge)
        {
            samplefrag *frag;
            bool overlap;
            int numclipplanes;

            frag = (samplefrag *)malloc(sizeof(samplefrag));
            err::require(frag != nullptr, "grow_single_frag: out of memory");

            // some basic info
            frag->next = nullptr;
            frag->parentfrag = parent;
            frag->parentedge = edge;
            frag->facenum = edge->nextfacenum;

            frag->flippedangle = edge->flippedangle;
            frag->noseam = edge->noseam;

            // calculate the matrix
            multiply_matrix(edge->prevtonext, parent->coordtomycoord, frag->coordtomycoord);
            multiply_matrix(parent->mycoordtocoord, edge->nexttoprev, frag->mycoordtocoord);

            // fill in origin
            math::copy(parent->origin, frag->origin);
            apply_matrix(frag->coordtomycoord, frag->origin, frag->myorigin);

            // fill in boundaries
            frag->rect = parent->rect;
            for (int x = 0; x < 4; x++)
            {
                // a plane's parameters are in the dual coordinate space, so
                // the original absolute plane is translated into this relative
                // plane by multiplying the inverse matrix
                apply_matrix_on_plane(frag->mycoordtocoord, frag->rect.planes[x].normal, frag->rect.planes[x].dist,
                                      frag->myrect.planes[x].normal, frag->myrect.planes[x].dist);
                double len = math::length(frag->myrect.planes[x].normal);
                if (!len)
                {
                    free(frag);
                    return nullptr;
                }
                math::scale(frag->myrect.planes[x].normal, 1 / len, frag->myrect.planes[x].normal);
                frag->myrect.planes[x].dist = (vec_t)(frag->myrect.planes[x].dist / len);
            }

            // chop windings and edges
            chop_frag(state, frag);

            if (frag->winding->size() == 0 || frag->mywinding->size() == 0)
            {
                // empty
                delete frag->mywinding;
                delete frag->winding;
                free(frag->edges);
                free(frag);
                return nullptr;
            }

            // do the overlap test
            overlap = false;
            std::vector<plane> clipplanes((size_t)frag->winding->size());
            numclipplanes = 0;
            for (int x = 0; x < frag->winding->size(); x++)
            {
                vec3v v;
                math::subtract((*frag->winding)[(x + 1) % frag->winding->size()], (*frag->winding)[x], v);
                math::cross(v, frag->windingplane.normal, clipplanes[(size_t)numclipplanes].normal);
                if (!math::normalize(clipplanes[(size_t)numclipplanes].normal))
                {
                    continue;
                }
                clipplanes[(size_t)numclipplanes].dist = math::dot((*frag->winding)[x], clipplanes[(size_t)numclipplanes].normal);
                numclipplanes++;
            }
            for (samplefrag *f2 = info->head; f2 && !overlap; f2 = f2->next)
            {
                math::winding w = *f2->winding;
                for (int x = 0; x < numclipplanes && w.size() > 0; x++)
                {
                    w.clip_in_place(clipplanes[(size_t)x].normal, clipplanes[(size_t)x].dist, false);
                }
                if (w.size() > 0)
                {
                    overlap = true;
                }
            }
            if (overlap)
            {
                // in the original texture plane, this fragment overlaps some
                // existing fragments
                delete frag->mywinding;
                delete frag->winding;
                free(frag->edges);
                free(frag);
                return nullptr;
            }

            return frag;
        }

        bool find_best_edge(samplefraginfo *info, samplefrag *&bestfrag, samplefragedge *&bestedge)
        {
            samplefrag *f;
            samplefragedge *e;
            bool found;

            found = false;

            for (f = info->head; f; f = f->next)
            {
                for (e = f->edges; e < f->edges + f->numedges; e++)
                {
                    if (e->tried)
                    {
                        continue;
                    }

                    bool better;

                    if (!found)
                    {
                        better = true;
                    }
                    else if ((e->flippedangle < math::pi + math::normal_epsilon) != (bestedge->flippedangle < math::pi + math::normal_epsilon))
                    {
                        better = ((e->flippedangle < math::pi + math::normal_epsilon) && !(bestedge->flippedangle < math::pi + math::normal_epsilon));
                    }
                    else if (e->noseam != bestedge->noseam)
                    {
                        better = (e->noseam && !bestedge->noseam);
                    }
                    else if (std::fabs(e->distance - bestedge->distance) > math::on_epsilon)
                    {
                        better = (e->distance < bestedge->distance);
                    }
                    else if (std::fabs(e->distancereduction - bestedge->distancereduction) > math::on_epsilon)
                    {
                        better = (e->distancereduction > bestedge->distancereduction);
                    }
                    else
                    {
                        better = e->edgenum < bestedge->edgenum;
                    }

                    if (better)
                    {
                        found = true;
                        bestfrag = f;
                        bestedge = e;
                    }
                }
            }

            return found;
        }

        samplefraginfo *create_sample_frag(const rad_state &state, int facenum, vec_t s, vec_t t,
                                           const vec_t square[2][2], int maxsize)
        {
            samplefraginfo *info;
            const vec3v v_s = {1, 0, 0};
            const vec3v v_t = {0, 1, 0};

            info = (samplefraginfo *)malloc(sizeof(samplefraginfo));
            err::require(info != nullptr, "create_sample_frag: out of memory");
            info->maxsize = maxsize;
            info->size = 1;
            info->head = (samplefrag *)malloc(sizeof(samplefrag));
            err::require(info->head != nullptr, "create_sample_frag: out of memory");

            info->head->next = nullptr;
            info->head->parentfrag = nullptr;
            info->head->parentedge = nullptr;
            info->head->facenum = facenum;

            info->head->flippedangle = 0.0;
            info->head->noseam = true;

            matrix_for_scale(vec3v{}, 1.0, info->head->coordtomycoord);
            matrix_for_scale(vec3v{}, 1.0, info->head->mycoordtocoord);

            info->head->origin[0] = s;
            info->head->origin[1] = t;
            info->head->origin[2] = 0.0;
            math::copy(info->head->origin, info->head->myorigin);

            math::scale(v_s, 1, info->head->rect.planes[0].normal);
            info->head->rect.planes[0].dist = square[0][0]; // smin
            math::scale(v_s, -1, info->head->rect.planes[1].normal);
            info->head->rect.planes[1].dist = -square[1][0]; // smax
            math::scale(v_t, 1, info->head->rect.planes[2].normal);
            info->head->rect.planes[2].dist = square[0][1]; // tmin
            math::scale(v_t, -1, info->head->rect.planes[3].normal);
            info->head->rect.planes[3].dist = -square[1][1]; // tmax
            info->head->myrect = info->head->rect;

            chop_frag(state, info->head);

            if (info->head->winding->size() == 0 || info->head->mywinding->size() == 0)
            {
                // empty
                delete info->head->mywinding;
                delete info->head->winding;
                free(info->head->edges);
                free(info->head);
                info->head = nullptr;
                info->size = 0;
            }
            else
            {
                // prune edges
                for (samplefragedge *e = info->head->edges; e < info->head->edges + info->head->numedges; e++)
                {
                    if (e->nextfacenum == info->head->facenum)
                    {
                        e->tried = true;
                    }
                }
            }

            while (info->size < info->maxsize)
            {
                samplefrag *bestfrag;
                samplefragedge *bestedge;
                samplefrag *newfrag;

                if (!find_best_edge(info, bestfrag, bestedge))
                {
                    break;
                }

                newfrag = grow_single_frag(state, info, bestfrag, bestedge);
                bestedge->tried = true;

                if (newfrag)
                {
                    newfrag->next = info->head;
                    info->head = newfrag;
                    info->size++;

                    for (samplefrag *f = info->head; f; f = f->next)
                    {
                        for (samplefragedge *e = newfrag->edges; e < newfrag->edges + newfrag->numedges; e++)
                        {
                            if (e->nextfacenum == f->facenum)
                            {
                                e->tried = true;
                            }
                        }
                    }
                    for (samplefrag *f = info->head; f; f = f->next)
                    {
                        for (samplefragedge *e = f->edges; e < f->edges + f->numedges; e++)
                        {
                            if (e->nextfacenum == newfrag->facenum)
                            {
                                e->tried = true;
                            }
                        }
                    }
                }
            }

            return info;
        }

        void delete_sample_frag(samplefraginfo *fraginfo)
        {
            while (fraginfo->head)
            {
                samplefrag *f;

                f = fraginfo->head;
                fraginfo->head = f->next;
                delete f->mywinding;
                delete f->winding;
                free(f->edges);
                free(f);
            }
            free(fraginfo);
        }

        light_flag set_sample_from_st(rad_state &state, vec3v &point, vec3v &position, int *surface,
                                      bool *nudged, const lightinfo *l,
                                      const vec_t original_s, const vec_t original_t,
                                      const vec_t square[2][2])
        {
            light_flag luxel_flag;
            int facenum;
            const format::dface_t *face;
            const plane *faceplane;
            samplefraginfo *fraginfo;
            samplefrag *f;

            facenum = l->surfnum;
            face = l->face;
            faceplane = plane_from_face(state, face);

            fraginfo = create_sample_frag(state, facenum, original_s, original_t, square, 100);

            bool found;
            samplefrag *bestfrag = nullptr;
            vec3v bestpos;
            vec_t bests = 0, bestt = 0;
            vec_t best_dist = 0;
            bool best_nudged = true;

            found = false;
            for (f = fraginfo->head; f; f = f->next)
            {
                vec3v pos;
                vec_t s, t;
                vec_t dist;

                bool nudged_one;
                if (!find_nearest_position(state, f->facenum, f->mywinding, f->mywindingplane,
                                           f->myorigin[0], f->myorigin[1], pos, &s, &t, &dist, &nudged_one))
                {
                    continue;
                }

                bool better;

                if (!found)
                {
                    better = true;
                }
                else if (nudged_one != best_nudged)
                {
                    better = !nudged_one;
                }
                else if (std::fabs(dist - best_dist) > 2 * math::on_epsilon)
                {
                    better = (dist < best_dist);
                }
                else if (f->noseam != bestfrag->noseam)
                {
                    better = (f->noseam && !bestfrag->noseam);
                }
                else
                {
                    better = (f->facenum < bestfrag->facenum);
                }

                if (better)
                {
                    found = true;
                    bestfrag = f;
                    math::copy(pos, bestpos);
                    bests = s;
                    bestt = t;
                    best_dist = dist;
                    best_nudged = nudged_one;
                }
            }

            if (found)
            {
                matrix worldtotex, textoworld;
                vec3v tex;

                translate_world_to_tex(state, bestfrag->facenum, worldtotex);
                if (!invert_matrix(worldtotex, textoworld))
                {
                    err::fatal("Malformed face (%d) normal", bestfrag->facenum);
                }

                // point
                tex[0] = bests;
                tex[1] = bestt;
                tex[2] = 0.0;
                {
                    vec3v v;
                    apply_matrix(textoworld, tex, v);
                    math::copy(v, point);
                }
                math::add(point, state.face_offset[(size_t)bestfrag->facenum], point);
                // position
                math::copy(bestpos, position);
                // surface
                *surface = bestfrag->facenum;
                // whether nudged to fit
                *nudged = best_nudged;
                // returned value
                luxel_flag = light_normal;
            }
            else
            {
                set_surf_from_st(state, l, point, original_s, original_t);
                math::multiply_add(point, default_hunt_offset, faceplane->normal, position);
                *surface = facenum;
                *nudged = true;
                luxel_flag = light_outside;
            }

            delete_sample_frag(fraginfo);

            return luxel_flag;
        }

        // for each texture aligned grid point, back project onto the plane to
        // get the world xyz value of the sample point
        void calc_points(rad_state &state, lightinfo *l)
        {
            const int h = l->texsize[1] + 1;
            const int w = l->texsize[0] + 1;
            const vec_t starts = (vec_t)(l->texmins[0] * texture_step);
            const vec_t startt = (vec_t)(l->texmins[1] * texture_step);
            light_flag luxel_flags[max_singlemap] = {};
            light_flag *pluxelflags;
            vec_t us, ut;
            int s, t;
            l->numsurfpt = w * h;
            for (t = 0; t < h; t++)
            {
                for (s = 0; s < w; s++)
                {
                    vec3v &surf = l->surfpt[s + w * t];
                    pluxelflags = &luxel_flags[s + w * t];
                    us = starts + s * texture_step;
                    ut = startt + t * texture_step;
                    vec_t square[2][2];
                    square[0][0] = us - texture_step;
                    square[0][1] = ut - texture_step;
                    square[1][0] = us + texture_step;
                    square[1][1] = ut + texture_step;
                    bool nudged;
                    *pluxelflags = set_sample_from_st(state, surf,
                                                      l->surfpt_position[s + w * t], &l->surfpt_surface[s + w * t],
                                                      &nudged, l, us, ut, square);
                }
            }
            {
                int i, n;
                int s_other, t_other;
                light_flag *pluxelflags_other;
                bool adjusted;
                for (i = 0; i < h + w; i++)
                {
                    // propagate valid light samples
                    adjusted = false;
                    for (t = 0; t < h; t++)
                    {
                        for (s = 0; s < w; s++)
                        {
                            vec3v &surf = l->surfpt[s + w * t];
                            pluxelflags = &luxel_flags[s + w * t];
                            if (*pluxelflags != light_outside)
                                continue;
                            for (n = 0; n < 4; n++)
                            {
                                switch (n)
                                {
                                case 0: s_other = s + 1; t_other = t; break;
                                case 1: s_other = s - 1; t_other = t; break;
                                case 2: s_other = s; t_other = t + 1; break;
                                default: s_other = s; t_other = t - 1; break;
                                }
                                if (t_other < 0 || t_other >= h || s_other < 0 || s_other >= w)
                                    continue;
                                vec3v &surf_other = l->surfpt[s_other + w * t_other];
                                pluxelflags_other = &luxel_flags[s_other + w * t_other];
                                if (*pluxelflags_other != light_outside && *pluxelflags_other != light_shifted)
                                {
                                    *pluxelflags = light_shifted;
                                    math::copy(surf_other, surf);
                                    math::copy(l->surfpt_position[s_other + w * t_other], l->surfpt_position[s + w * t]);
                                    l->surfpt_surface[s + w * t] = l->surfpt_surface[s_other + w * t_other];
                                    adjusted = true;
                                    break;
                                }
                            }
                        }
                    }
                    for (t = 0; t < h; t++)
                    {
                        for (s = 0; s < w; s++)
                        {
                            pluxelflags = &luxel_flags[s + w * t];
                            if (*pluxelflags == light_shifted)
                            {
                                *pluxelflags = light_shifted_inside;
                            }
                        }
                    }
                    if (!adjusted)
                        break;
                }
            }
            for (int i = 0; i < max_singlemap; i++)
            {
                l->surfpt_lightoutside[i] = (luxel_flags[i] == light_outside);
            }
        }
    }

    // ===== direct lights =====

    namespace
    {
        // builds the compact per leaf light list the sample gather iterates
        void build_light_leaf_list(rad_state &state)
        {
            int i;
            int leafcount = 0;
            int lightcount = 0;
            directlight *dl;

            state.lightleafs.clear();
            state.lightarray.clear();
            for (i = 0; i < 1 + state.map->models[0].visleafs; i++)
            {
                if (state.directlights[(size_t)i])
                {
                    leafcount++;
                    for (dl = state.directlights[(size_t)i]; dl; dl = dl->next)
                    {
                        lightcount++;
                    }
                }
            }
            state.lightleafs.reserve((size_t)leafcount);
            state.lightarray.reserve((size_t)lightcount);
            lightcount = 0;
            for (i = 0; i < 1 + state.map->models[0].visleafs; i++)
            {
                if (state.directlights[(size_t)i])
                {
                    lightleaf ll;
                    ll.leafnum = i;
                    ll.firstlight = lightcount;
                    for (dl = state.directlights[(size_t)i]; dl; dl = dl->next)
                    {
                        state.lightarray.push_back(*dl);
                        lightcount++;
                    }
                    ll.numlights = lightcount - ll.firstlight;
                    state.lightleafs.push_back(ll);
                }
            }
        }

        // formats an integer with thousands separators like the reference commanum
        std::string comma_num(int num)
        {
            std::string digits = std::to_string(num);
            std::string out;
            int count = 0;
            for (int i = (int)digits.size() - 1; i >= 0; i--)
            {
                out.insert(out.begin(), digits[(size_t)i]);
                count++;
                if (count % 3 == 0 && i > 0)
                {
                    out.insert(out.begin(), ',');
                }
            }
            return out;
        }
    }

    void create_direct_lights(rad_state &state)
    {
        unsigned i;
        patch *p;
        directlight *dl;
        format::dleaf_t *leaf;
        int leafnum;
        format::entity *e;
        format::entity *e2;
        const char *name;
        const char *target;
        float angle;
        vec3v dest;
        format::map_data &map = *state.map;

        state.directlights.assign(map.leafs.size(), nullptr);
        int styleused[allstyles];
        memset(styleused, 0, allstyles * sizeof(styleused[0]));
        styleused[0] = true;
        int numstyles = 1;

        //
        // surfaces
        //
        for (i = 0, p = state.patches.data(); i < state.num_patches; i++, p++)
        {
            if (p->emitstyle < allstyles)
            {
                if (styleused[p->emitstyle] == false)
                {
                    styleused[p->emitstyle] = true;
                    numstyles++;
                }
            }
            if (math::dot(p->baselight, p->texturereflectivity) / 3 > 0.0
                && !(state.face_texlights[(size_t)p->facenumber]
                     && *state.face_texlights[(size_t)p->facenumber]->value("_scale")
                     && float_for_key(*state.face_texlights[(size_t)p->facenumber], "_scale") <= 0))
            {
                dl = (directlight *)calloc(1, sizeof(directlight));
                err::require(dl != nullptr, "create_direct_lights: out of memory");

                math::copy(p->origin, dl->origin);

                leaf = point_in_leaf(state, dl->origin);
                leafnum = (int)(leaf - map.leafs.data());

                dl->next = state.directlights[(size_t)leafnum];
                state.directlights[(size_t)leafnum] = dl;
                dl->style = p->emitstyle;
                dl->topatch = false;
                if (!p->emitmode)
                {
                    dl->topatch = true;
                }
                if (state.options.fastmode)
                {
                    dl->topatch = true;
                }
                dl->patch_area = p->area;
                dl->patch_emitter_range = p->emitter_range;
                dl->source_patch = p;
                dl->texlightgap = state.options.texlightgap;
                if (state.face_texlights[(size_t)p->facenumber]
                    && *state.face_texlights[(size_t)p->facenumber]->value("_texlightgap"))
                {
                    dl->texlightgap = float_for_key(*state.face_texlights[(size_t)p->facenumber], "_texlightgap");
                }
                dl->stopdot = 0.0;
                dl->stopdot2 = 0.0;
                if (state.face_texlights[(size_t)p->facenumber])
                {
                    format::entity *tl = state.face_texlights[(size_t)p->facenumber];
                    if (*tl->value("_cone"))
                    {
                        dl->stopdot = float_for_key(*tl, "_cone");
                        dl->stopdot = dl->stopdot >= 90 ? 0 : (float)cos(dl->stopdot / 180 * math::pi);
                    }
                    if (*tl->value("_cone2"))
                    {
                        dl->stopdot2 = float_for_key(*tl, "_cone2");
                        dl->stopdot2 = dl->stopdot2 >= 90 ? 0 : (float)cos(dl->stopdot2 / 180 * math::pi);
                    }
                    if (dl->stopdot2 > dl->stopdot)
                        dl->stopdot2 = dl->stopdot;
                }

                dl->type = emit_type::surface;
                math::copy(plane_from_face_number(state, (unsigned)p->facenumber)->normal, dl->normal);
                math::copy(p->baselight, dl->intensity);
                if (state.face_texlights[(size_t)p->facenumber])
                {
                    if (*state.face_texlights[(size_t)p->facenumber]->value("_scale"))
                    {
                        vec_t scale = float_for_key(*state.face_texlights[(size_t)p->facenumber], "_scale");
                        math::scale(dl->intensity, scale, dl->intensity);
                    }
                }
                math::scale(dl->intensity, p->area, dl->intensity);
                math::scale(dl->intensity, p->exposure, dl->intensity);
                math::scale(dl->intensity, 1.0 / math::pi, dl->intensity);
                math::multiply(dl->intensity, p->texturereflectivity, dl->intensity);

                const format::dface_t *f = &map.faces[(size_t)p->facenumber];
                if (state.face_entity[(size_t)p->facenumber] != &state.entities[0]
                    && texture_by_number(state, f->texinfo)[0] == '!')
                {
                    directlight *dl2;
                    dl2 = (directlight *)calloc(1, sizeof(directlight));
                    err::require(dl2 != nullptr, "create_direct_lights: out of memory");
                    *dl2 = *dl;
                    math::multiply_add(dl->origin, -2, dl->normal, dl2->origin);
                    for (int x = 0; x < 3; x++)
                        dl2->normal[x] = 0.0f - dl->normal[x];
                    leaf = point_in_leaf(state, dl2->origin);
                    leafnum = (int)(leaf - map.leafs.data());
                    dl2->next = state.directlights[(size_t)leafnum];
                    state.directlights[(size_t)leafnum] = dl2;
                }
            }
        }

        //
        // entities
        //
        for (i = 0; i < (unsigned)state.entities.size(); i++)
        {
            const char *plight;
            double r, g, b, scaler;
            float l1;
            int argcnt;

            e = &state.entities[i];
            name = e->value("classname");
            if (strncmp(name, "light", 5))
                continue;
            {
                int style = int_for_key(*e, "style");
                if (style < 0)
                {
                    style = -style;
                }
                style = (unsigned char)style;
                if (style > 0 && style < allstyles && *e->value("zhlt_stylecoring"))
                {
                    state.corings[style] = float_for_key(*e, "zhlt_stylecoring");
                }
            }
            if (!strcmp(name, "light_shadow") || !strcmp(name, "light_bounce"))
            {
                int style = int_for_key(*e, "style");
                if (style < 0)
                {
                    style = -style;
                }
                style = (unsigned char)style;
                if (style >= 0 && style < allstyles)
                {
                    if (styleused[style] == false)
                    {
                        styleused[style] = true;
                        numstyles++;
                    }
                }
                continue;
            }
            if (!strcmp(name, "light_surface"))
            {
                continue;
            }

            dl = (directlight *)calloc(1, sizeof(directlight));
            err::require(dl != nullptr, "create_direct_lights: out of memory");

            vector_for_key(*e, "origin", dl->origin);

            leaf = point_in_leaf(state, dl->origin);
            leafnum = (int)(leaf - map.leafs.data());

            dl->next = state.directlights[(size_t)leafnum];
            state.directlights[(size_t)leafnum] = dl;

            dl->style = int_for_key(*e, "style");
            if (dl->style < 0)
                dl->style = -dl->style;
            dl->style = (unsigned char)dl->style;
            if (dl->style >= allstyles)
            {
                err::fatal("invalid light style: style (%d) >= ALLSTYLES (%d)", dl->style, allstyles);
            }
            if (dl->style >= 0 && dl->style < allstyles)
            {
                if (styleused[dl->style] == false)
                {
                    styleused[dl->style] = true;
                    numstyles++;
                }
            }
            dl->topatch = false;
            if (int_for_key(*e, "_fast") == 1)
            {
                dl->topatch = true;
            }
            if (state.options.fastmode)
            {
                dl->topatch = true;
            }
            plight = e->value("_light");
            // scanf into doubles, then assign, so it is vec_t size independent
            r = g = b = scaler = 0;
            argcnt = sscanf(plight, "%lf %lf %lf %lf", &r, &g, &b, &scaler);
            dl->intensity[0] = (float)r;
            if (argcnt == 1)
            {
                // the r,g,b values are all equal
                dl->intensity[1] = dl->intensity[2] = (float)r;
            }
            else if (argcnt == 3 || argcnt == 4)
            {
                // save the other two g,b values
                dl->intensity[1] = (float)g;
                dl->intensity[2] = (float)b;

                // did we also get an intensity scaler value?
                if (argcnt == 4)
                {
                    // scale the normalized r,g,b values from 0 to 255 by the intensity scaler
                    dl->intensity[0] = dl->intensity[0] / 255 * (float)scaler;
                    dl->intensity[1] = dl->intensity[1] / 255 * (float)scaler;
                    dl->intensity[2] = dl->intensity[2] / 255 * (float)scaler;
                }
            }
            else
            {
                logging::info("light at (%f,%f,%f) has bad or missing '_light' value : '%s'\n",
                              dl->origin[0], dl->origin[1], dl->origin[2], plight);
                continue;
            }

            dl->fade = float_for_key(*e, "_fade");
            if (dl->fade == 0.0)
            {
                dl->fade = state.options.fade;
            }

            target = e->value("target");

            if (!strcmp(name, "light_spot") || !strcmp(name, "light_environment") || target[0])
            {
                dl->type = emit_type::spotlight;
                dl->stopdot = float_for_key(*e, "_cone");
                if (!dl->stopdot)
                {
                    dl->stopdot = 10;
                }
                dl->stopdot2 = float_for_key(*e, "_cone2");
                if (!dl->stopdot2)
                {
                    dl->stopdot2 = dl->stopdot;
                }
                if (dl->stopdot2 < dl->stopdot)
                {
                    dl->stopdot2 = dl->stopdot;
                }
                dl->stopdot2 = (float)cos(dl->stopdot2 / 180 * math::pi);
                dl->stopdot = (float)cos(dl->stopdot / 180 * math::pi);

                if (!find_target_entity(state, target))
                {
                    logging::warn("light at (%i %i %i) has missing target",
                                  (int)dl->origin[0], (int)dl->origin[1], (int)dl->origin[2]);
                    target = "";
                }
                if (target[0])
                {
                    // point towards target
                    e2 = find_target_entity(state, target);
                    if (!e2)
                    {
                        logging::warn("light at (%i %i %i) has missing target",
                                      (int)dl->origin[0], (int)dl->origin[1], (int)dl->origin[2]);
                    }
                    else
                    {
                        vector_for_key(*e2, "origin", dest);
                        math::subtract(dest, dl->origin, dl->normal);
                        math::normalize(dl->normal);
                    }
                }
                else
                {
                    // point down angle
                    vec3v vangles;

                    vector_for_key(*e, "angles", vangles);

                    angle = (float)float_for_key(*e, "angle");
                    if (angle == angle_up)
                    {
                        dl->normal[0] = dl->normal[1] = 0;
                        dl->normal[2] = 1;
                    }
                    else if (angle == angle_down)
                    {
                        dl->normal[0] = dl->normal[1] = 0;
                        dl->normal[2] = -1;
                    }
                    else
                    {
                        // if we don't have a specific angle use the angles yaw
                        if (!angle)
                        {
                            angle = vangles[1];
                        }

                        dl->normal[2] = 0;
                        dl->normal[0] = (float)cos(angle / 180 * math::pi);
                        dl->normal[1] = (float)sin(angle / 180 * math::pi);
                    }

                    angle = float_for_key(*e, "pitch");
                    if (!angle)
                    {
                        // if we don't have a specific pitch use the angles pitch
                        angle = vangles[0];
                    }

                    dl->normal[2] = (float)sin(angle / 180 * math::pi);
                    dl->normal[0] *= (float)cos(angle / 180 * math::pi);
                    dl->normal[1] *= (float)cos(angle / 180 * math::pi);
                }

                if (float_for_key(*e, "_sky") || !strcmp(name, "light_environment"))
                {
                    // diffuse light_environment intensity (adam foster's sky hack)
                    plight = e->value("_diffuse_light");
                    r = g = b = scaler = 0;
                    argcnt = sscanf(plight, "%lf %lf %lf %lf", &r, &g, &b, &scaler);
                    dl->diffuse_intensity[0] = (float)r;
                    if (argcnt == 1)
                    {
                        dl->diffuse_intensity[1] = dl->diffuse_intensity[2] = (float)r;
                    }
                    else if (argcnt == 3 || argcnt == 4)
                    {
                        dl->diffuse_intensity[1] = (float)g;
                        dl->diffuse_intensity[2] = (float)b;
                        if (argcnt == 4)
                        {
                            dl->diffuse_intensity[0] = dl->diffuse_intensity[0] / 255 * (float)scaler;
                            dl->diffuse_intensity[1] = dl->diffuse_intensity[1] / 255 * (float)scaler;
                            dl->diffuse_intensity[2] = dl->diffuse_intensity[2] / 255 * (float)scaler;
                        }
                    }
                    else
                    {
                        // backwards compatibility with maps without _diffuse_light
                        dl->diffuse_intensity[0] = dl->intensity[0];
                        dl->diffuse_intensity[1] = dl->intensity[1];
                        dl->diffuse_intensity[2] = dl->intensity[2];
                    }
                    plight = e->value("_diffuse_light2");
                    r = g = b = scaler = 0;
                    argcnt = sscanf(plight, "%lf %lf %lf %lf", &r, &g, &b, &scaler);
                    dl->diffuse_intensity2[0] = (float)r;
                    if (argcnt == 1)
                    {
                        dl->diffuse_intensity2[1] = dl->diffuse_intensity2[2] = (float)r;
                    }
                    else if (argcnt == 3 || argcnt == 4)
                    {
                        dl->diffuse_intensity2[1] = (float)g;
                        dl->diffuse_intensity2[2] = (float)b;
                        if (argcnt == 4)
                        {
                            dl->diffuse_intensity2[0] = dl->diffuse_intensity2[0] / 255 * (float)scaler;
                            dl->diffuse_intensity2[1] = dl->diffuse_intensity2[1] / 255 * (float)scaler;
                            dl->diffuse_intensity2[2] = dl->diffuse_intensity2[2] / 255 * (float)scaler;
                        }
                    }
                    else
                    {
                        dl->diffuse_intensity2[0] = dl->diffuse_intensity[0];
                        dl->diffuse_intensity2[1] = dl->diffuse_intensity[1];
                        dl->diffuse_intensity2[2] = dl->diffuse_intensity[2];
                    }

                    dl->type = emit_type::skylight;
                    dl->stopdot2 = float_for_key(*e, "_sky"); // hack stopdot2 to a sky key number
                    dl->sunspreadangle = float_for_key(*e, "_spread");
                    if (!state.options.allow_spread)
                    {
                        dl->sunspreadangle = 0;
                    }
                    if (dl->sunspreadangle < 0.0 || dl->sunspreadangle > 180)
                    {
                        err::fatal("Invalid spread angle '%s'. Please use a number between 0 and 180.\n", e->value("_spread"));
                    }
                    if (dl->sunspreadangle > 0.0)
                    {
                        int j;
                        vec_t testangle = dl->sunspreadangle;
                        if (dl->sunspreadangle < sunspread_threshold)
                        {
                            // we will later centralize all the normals we collected
                            testangle = sunspread_threshold;
                        }
                        {
                            vec_t totalweight = 0;
                            int count;
                            vec_t testdot = (vec_t)cos(testangle * (math::pi / 180.0));
                            for (count = 0, j = 0; j < state.numskynormals[sunspread_skylevel]; j++)
                            {
                                const vec3v &testnormal = state.skynormals[sunspread_skylevel][(size_t)j];
                                vec_t dot = math::dot(dl->normal, testnormal);
                                if (dot >= testdot - math::normal_epsilon)
                                {
                                    // this is not the right formula when sunspreadangle is
                                    // below the threshold, but it gives almost the same result
                                    totalweight = (vec_t)(totalweight
                                        + (0 > dot - testdot ? 0 : dot - testdot) * state.skynormalsizes[sunspread_skylevel][(size_t)j]);
                                    count++;
                                }
                            }
                            if (count <= 10 || totalweight <= math::normal_epsilon)
                            {
                                err::fatal("collect spread normals: internal error: can not collect enough normals.");
                            }
                            dl->numsunnormals = count;
                            dl->sunnormals = (vec3v *)malloc((size_t)count * sizeof(vec3v));
                            dl->sunnormalweights = (vec_t *)malloc((size_t)count * sizeof(vec_t));
                            err::require(dl->sunnormals != nullptr, "create_direct_lights: out of memory");
                            err::require(dl->sunnormalweights != nullptr, "create_direct_lights: out of memory");
                            for (count = 0, j = 0; j < state.numskynormals[sunspread_skylevel]; j++)
                            {
                                const vec3v &testnormal = state.skynormals[sunspread_skylevel][(size_t)j];
                                vec_t dot = math::dot(dl->normal, testnormal);
                                if (dot >= testdot - math::normal_epsilon)
                                {
                                    if (count >= dl->numsunnormals)
                                    {
                                        err::fatal("collect spread normals: internal error.");
                                    }
                                    math::copy(testnormal, dl->sunnormals[count]);
                                    dl->sunnormalweights[count] = (vec_t)((0 > dot - testdot ? 0 : dot - testdot) * state.skynormalsizes[sunspread_skylevel][(size_t)j] / totalweight);
                                    count++;
                                }
                            }
                            if (count != dl->numsunnormals)
                            {
                                err::fatal("collect spread normals: internal error.");
                            }
                        }
                        if (dl->sunspreadangle < sunspread_threshold)
                        {
                            for (j = 0; j < dl->numsunnormals; j++)
                            {
                                vec3v tmp;
                                math::scale(dl->sunnormals[j], 1 / math::dot(dl->sunnormals[j], dl->normal), tmp);
                                math::subtract(tmp, dl->normal, tmp);
                                math::multiply_add(dl->normal, dl->sunspreadangle / sunspread_threshold, tmp, dl->sunnormals[j]);
                                math::normalize(dl->sunnormals[j]);
                            }
                        }
                    }
                    else
                    {
                        dl->numsunnormals = 1;
                        dl->sunnormals = (vec3v *)malloc(sizeof(vec3v));
                        dl->sunnormalweights = (vec_t *)malloc(sizeof(vec_t));
                        err::require(dl->sunnormals != nullptr, "create_direct_lights: out of memory");
                        err::require(dl->sunnormalweights != nullptr, "create_direct_lights: out of memory");
                        math::copy(dl->normal, dl->sunnormals[0]);
                        dl->sunnormalweights[0] = 1.0;
                    }
                }
            }
            else
            {
                dl->type = emit_type::point;
            }

            if (dl->type != emit_type::skylight)
            {
                l1 = dl->intensity[1] > dl->intensity[2] ? dl->intensity[1] : dl->intensity[2];
                l1 = dl->intensity[0] > l1 ? dl->intensity[0] : l1;
                l1 = l1 * l1 / 10;

                dl->intensity[0] *= l1;
                dl->intensity[1] *= l1;
                dl->intensity[2] *= l1;
            }
        }

        int countnormallights = 0, countfastlights = 0;
        {
            int l;
            for (l = 0; l < 1 + map.models[0].visleafs; l++)
            {
                for (dl = state.directlights[(size_t)l]; dl; dl = dl->next)
                {
                    switch (dl->type)
                    {
                    case emit_type::surface:
                    case emit_type::point:
                    case emit_type::spotlight:
                        if (!math::equal(dl->intensity, vec3v{}))
                        {
                            if (dl->topatch)
                            {
                                countfastlights++;
                            }
                            else
                            {
                                countnormallights++;
                            }
                        }
                        break;
                    case emit_type::skylight:
                        if (!math::equal(dl->intensity, vec3v{}))
                        {
                            if (dl->topatch)
                            {
                                countfastlights++;
                                if (dl->sunspreadangle > 0.0)
                                {
                                    countfastlights--;
                                    countfastlights += dl->numsunnormals;
                                }
                            }
                            else
                            {
                                countnormallights++;
                                if (dl->sunspreadangle > 0.0)
                                {
                                    countnormallights--;
                                    countnormallights += dl->numsunnormals;
                                }
                            }
                        }
                        if (state.options.indirect_sun > 0 && !math::equal(dl->diffuse_intensity, vec3v{}))
                        {
                            if (state.options.softsky)
                            {
                                countfastlights += state.numskynormals[skylevel_softsky_on];
                            }
                            else
                            {
                                countfastlights += state.numskynormals[skylevel_softsky_off];
                            }
                        }
                        break;
                    default:
                        err::fatal("create_direct_lights: bad light type");
                        break;
                    }
                }
            }
        }
        {
            logging::info("  %-14s %s direct (%s fast), %d style%s\n", "lights",
                          comma_num(countnormallights).c_str(), comma_num(countfastlights).c_str(),
                          numstyles, numstyles == 1 ? "" : "s");
        }
        // move all emit_skylight to leaf 0 (the solid leaf)
        if (state.options.sky_lighting_fix)
        {
            directlight *skylights = nullptr;
            int l;
            for (l = 0; l < 1 + map.models[0].visleafs; l++)
            {
                directlight **pdl;
                for (dl = state.directlights[(size_t)l], pdl = &state.directlights[(size_t)l]; dl; dl = *pdl)
                {
                    if (dl->type == emit_type::skylight)
                    {
                        *pdl = dl->next;
                        dl->next = skylights;
                        skylights = dl;
                    }
                    else
                    {
                        pdl = &dl->next;
                    }
                }
            }
            while ((dl = state.directlights[0]) != nullptr)
            {
                // since they are in leaf 0, they won't emit a light anyway
                state.directlights[0] = dl->next;
                free(dl->sunnormals);
                free(dl->sunnormalweights);
                free(dl);
            }
            state.directlights[0] = skylights;
        }
        if (state.options.sky_lighting_fix)
        {
            int countlightenvironment = 0;
            int countinfosunlight = 0;
            for (size_t j = 0; j < state.entities.size(); j++)
            {
                format::entity *ent = &state.entities[j];
                const char *classname = ent->value("classname");
                if (!strcmp(classname, "light_environment"))
                {
                    countlightenvironment++;
                }
                if (!strcmp(classname, "info_sunlight"))
                {
                    countinfosunlight++;
                }
            }
            if (countlightenvironment > 1 && countinfosunlight == 0)
            {
                // the game can only recognize one light_environment when
                // setting sv_skycolor and sv_skyvec
                logging::warn("More than one light_environments are in use. Add entity info_sunlight to clarify the sunlight's brightness for in-game model(.mdl) rendering.");
            }
        }
        build_light_leaf_list(state);
    }

    void delete_direct_lights(rad_state &state)
    {
        int l;
        directlight *dl;

        for (l = 0; l < 1 + state.map->models[0].visleafs; l++)
        {
            dl = state.directlights[(size_t)l];
            while (dl)
            {
                state.directlights[(size_t)l] = dl->next;
                free(dl->sunnormals);
                free(dl->sunnormalweights);
                free(dl);
                dl = state.directlights[(size_t)l];
            }
        }
        state.lightleafs.clear();
        state.lightarray.clear();

        // note: rad must not modify entity data, because a bsp that is
        // relit later must produce the same file
    }

    // ===== sky sampling normals =====

    namespace
    {
        typedef double sky_point_t[3];
        struct sky_edge
        {
            int point[2];
            bool divided;
            int child[2];
        };
        struct sky_triangle
        {
            int edge[3];
            int dir[3];
        };

        void copy_to_skynormals(rad_state &state, int skylevel, int numpoints, sky_point_t *points,
                                int numedges, sky_edge *edges, int numtriangles, sky_triangle *triangles)
        {
            err::require(numpoints == (1 << (2 * skylevel)) + 2, "copy_to_skynormals: point count");
            err::require(numedges == (1 << (2 * skylevel)) * 4 - 4, "copy_to_skynormals: edge count");
            err::require(numtriangles == (1 << (2 * skylevel)) * 2, "copy_to_skynormals: triangle count");
            state.numskynormals[skylevel] = numpoints;
            state.skynormals[skylevel].resize((size_t)numpoints);
            state.skynormalsizes[skylevel].resize((size_t)numpoints);
            int j, k;
            for (j = 0; j < numpoints; j++)
            {
                for (k = 0; k < 3; k++)
                    state.skynormals[skylevel][(size_t)j][k] = (vec_t)points[j][k];
                state.skynormalsizes[skylevel][(size_t)j] = 0;
            }
            double totalsize = 0;
            for (j = 0; j < numtriangles; j++)
            {
                int pt[3];
                for (k = 0; k < 3; k++)
                {
                    pt[k] = edges[triangles[j].edge[k]].point[triangles[j].dir[k]];
                }
                double currentsize;
                double tmp[3];
                tmp[0] = points[pt[0]][1] * points[pt[1]][2] - points[pt[0]][2] * points[pt[1]][1];
                tmp[1] = points[pt[0]][2] * points[pt[1]][0] - points[pt[0]][0] * points[pt[1]][2];
                tmp[2] = points[pt[0]][0] * points[pt[1]][1] - points[pt[0]][1] * points[pt[1]][0];
                currentsize = tmp[0] * points[pt[2]][0] + tmp[1] * points[pt[2]][1] + tmp[2] * points[pt[2]][2];
                err::require(currentsize > 0, "copy_to_skynormals: degenerate triangle");
                state.skynormalsizes[skylevel][(size_t)pt[0]] = (vec_t)(state.skynormalsizes[skylevel][(size_t)pt[0]] + currentsize / 3.0);
                state.skynormalsizes[skylevel][(size_t)pt[1]] = (vec_t)(state.skynormalsizes[skylevel][(size_t)pt[1]] + currentsize / 3.0);
                state.skynormalsizes[skylevel][(size_t)pt[2]] = (vec_t)(state.skynormalsizes[skylevel][(size_t)pt[2]] + currentsize / 3.0);
                totalsize += currentsize;
            }
            for (j = 0; j < numpoints; j++)
            {
                state.skynormalsizes[skylevel][(size_t)j] = (vec_t)(state.skynormalsizes[skylevel][(size_t)j] / totalsize);
            }
        }
    }

    void build_diffuse_normals(rad_state &state)
    {
        int i, j, k;
        state.numskynormals[0] = 0;
        state.skynormals[0].clear(); // don't use this
        state.skynormalsizes[0].clear();
        int numpoints = 6;
        sky_point_t *points = (sky_point_t *)malloc(((size_t)(1 << (2 * skylevel_max)) + 2) * sizeof(sky_point_t));
        err::require(points != nullptr, "build_diffuse_normals: out of memory");
        points[0][0] = 1, points[0][1] = 0, points[0][2] = 0;
        points[1][0] = -1, points[1][1] = 0, points[1][2] = 0;
        points[2][0] = 0, points[2][1] = 1, points[2][2] = 0;
        points[3][0] = 0, points[3][1] = -1, points[3][2] = 0;
        points[4][0] = 0, points[4][1] = 0, points[4][2] = 1;
        points[5][0] = 0, points[5][1] = 0, points[5][2] = -1;
        int numedges = 12;
        sky_edge *edges = (sky_edge *)malloc(((size_t)(1 << (2 * skylevel_max)) * 4 - 4) * sizeof(sky_edge));
        err::require(edges != nullptr, "build_diffuse_normals: out of memory");
        edges[0].point[0] = 0, edges[0].point[1] = 2, edges[0].divided = false;
        edges[1].point[0] = 2, edges[1].point[1] = 1, edges[1].divided = false;
        edges[2].point[0] = 1, edges[2].point[1] = 3, edges[2].divided = false;
        edges[3].point[0] = 3, edges[3].point[1] = 0, edges[3].divided = false;
        edges[4].point[0] = 2, edges[4].point[1] = 4, edges[4].divided = false;
        edges[5].point[0] = 4, edges[5].point[1] = 3, edges[5].divided = false;
        edges[6].point[0] = 3, edges[6].point[1] = 5, edges[6].divided = false;
        edges[7].point[0] = 5, edges[7].point[1] = 2, edges[7].divided = false;
        edges[8].point[0] = 4, edges[8].point[1] = 0, edges[8].divided = false;
        edges[9].point[0] = 0, edges[9].point[1] = 5, edges[9].divided = false;
        edges[10].point[0] = 5, edges[10].point[1] = 1, edges[10].divided = false;
        edges[11].point[0] = 1, edges[11].point[1] = 4, edges[11].divided = false;
        int numtriangles = 8;
        sky_triangle *triangles = (sky_triangle *)malloc(((size_t)(1 << (2 * skylevel_max)) * 2) * sizeof(sky_triangle));
        err::require(triangles != nullptr, "build_diffuse_normals: out of memory");
        triangles[0].edge[0] = 0, triangles[0].dir[0] = 0, triangles[0].edge[1] = 4, triangles[0].dir[1] = 0, triangles[0].edge[2] = 8, triangles[0].dir[2] = 0;
        triangles[1].edge[0] = 1, triangles[1].dir[0] = 0, triangles[1].edge[1] = 11, triangles[1].dir[1] = 0, triangles[1].edge[2] = 4, triangles[1].dir[2] = 1;
        triangles[2].edge[0] = 2, triangles[2].dir[0] = 0, triangles[2].edge[1] = 5, triangles[2].dir[1] = 1, triangles[2].edge[2] = 11, triangles[2].dir[2] = 1;
        triangles[3].edge[0] = 3, triangles[3].dir[0] = 0, triangles[3].edge[1] = 8, triangles[3].dir[1] = 1, triangles[3].edge[2] = 5, triangles[3].dir[2] = 0;
        triangles[4].edge[0] = 0, triangles[4].dir[0] = 1, triangles[4].edge[1] = 9, triangles[4].dir[1] = 0, triangles[4].edge[2] = 7, triangles[4].dir[2] = 0;
        triangles[5].edge[0] = 1, triangles[5].dir[0] = 1, triangles[5].edge[1] = 7, triangles[5].dir[1] = 1, triangles[5].edge[2] = 10, triangles[5].dir[2] = 0;
        triangles[6].edge[0] = 2, triangles[6].dir[0] = 1, triangles[6].edge[1] = 10, triangles[6].dir[1] = 1, triangles[6].edge[2] = 6, triangles[6].dir[2] = 1;
        triangles[7].edge[0] = 3, triangles[7].dir[0] = 1, triangles[7].edge[1] = 6, triangles[7].dir[1] = 0, triangles[7].edge[2] = 9, triangles[7].dir[2] = 1;
        copy_to_skynormals(state, 1, numpoints, points, numedges, edges, numtriangles, triangles);
        for (i = 1; i < skylevel_max; i++)
        {
            int oldnumedges = numedges;
            for (j = 0; j < oldnumedges; j++)
            {
                if (!edges[j].divided)
                {
                    err::require(numpoints < (1 << (2 * skylevel_max)) + 2, "build_diffuse_normals: point overflow");
                    sky_point_t mid;
                    double len;
                    for (k = 0; k < 3; k++)
                        mid[k] = points[edges[j].point[0]][k] + points[edges[j].point[1]][k];
                    len = sqrt(mid[0] * mid[0] + mid[1] * mid[1] + mid[2] * mid[2]);
                    err::require(len > 0.2, "build_diffuse_normals: short edge");
                    for (k = 0; k < 3; k++)
                        mid[k] = mid[k] * (1 / len);
                    int p2 = numpoints;
                    for (k = 0; k < 3; k++)
                        points[numpoints][k] = mid[k];
                    numpoints++;
                    err::require(numedges < (1 << (2 * skylevel_max)) * 4 - 4, "build_diffuse_normals: edge overflow");
                    edges[j].child[0] = numedges;
                    edges[numedges].divided = false;
                    edges[numedges].point[0] = edges[j].point[0];
                    edges[numedges].point[1] = p2;
                    numedges++;
                    err::require(numedges < (1 << (2 * skylevel_max)) * 4 - 4, "build_diffuse_normals: edge overflow");
                    edges[j].child[1] = numedges;
                    edges[numedges].divided = false;
                    edges[numedges].point[0] = p2;
                    edges[numedges].point[1] = edges[j].point[1];
                    numedges++;
                    edges[j].divided = true;
                }
            }
            int oldnumtriangles = numtriangles;
            for (j = 0; j < oldnumtriangles; j++)
            {
                int mid[3];
                for (k = 0; k < 3; k++)
                {
                    err::require(numtriangles < (1 << (2 * skylevel_max)) * 2, "build_diffuse_normals: triangle overflow");
                    mid[k] = edges[edges[triangles[j].edge[k]].child[0]].point[1];
                    triangles[numtriangles].edge[0] = edges[triangles[j].edge[k]].child[1 - triangles[j].dir[k]];
                    triangles[numtriangles].dir[0] = triangles[j].dir[k];
                    triangles[numtriangles].edge[1] = edges[triangles[j].edge[(k + 1) % 3]].child[triangles[j].dir[(k + 1) % 3]];
                    triangles[numtriangles].dir[1] = triangles[j].dir[(k + 1) % 3];
                    triangles[numtriangles].edge[2] = numedges + k;
                    triangles[numtriangles].dir[2] = 1;
                    numtriangles++;
                }
                for (k = 0; k < 3; k++)
                {
                    err::require(numedges < (1 << (2 * skylevel_max)) * 4 - 4, "build_diffuse_normals: edge overflow");
                    triangles[j].edge[k] = numedges;
                    triangles[j].dir[k] = 0;
                    edges[numedges].divided = false;
                    edges[numedges].point[0] = mid[k];
                    edges[numedges].point[1] = mid[(k + 1) % 3];
                    numedges++;
                }
            }
            copy_to_skynormals(state, i + 1, numpoints, points, numedges, edges, numtriangles, triangles);
        }
        free(points);
        free(edges);
        free(triangles);
    }

    // ===== gather sample light =====

    namespace
    {
        // accumulates the direct light arriving at one point from every light
        // that can see it step selects normal (0) or topatch/fast (1) lights
        void gather_sample_light(rad_state &state, const vec3v &pos, const byte *const pvs,
                                 const vec3v &normal, vec3v *sample, byte *styles, int step,
                                 int miptex, int texlightgap_surfacenum)
        {
            if (state.gpu_gather_phase != 0)
            {
                // -gpu: collect records this call as a work item; consume
                // serves the gpu result through the same tail merge
                gpu_gather_intercept(state, pos, pvs, normal, sample, styles, step,
                                     miptex, texlightgap_surfacenum);
                return;
            }
            int i;
            directlight *l;
            vec3v delta;
            float dot, dot2;
            float dist;
            float ratio;
            int style_index;
            int step_match;
            bool sky_used = false;
            vec3v testline_origin;
            vec3v adds[allstyles];
            unsigned long long adds_touched = 0; // bit per style; adds[style] is zeroed lazily on first touch
            int style;
            bool lighting_diversify;
            vec_t lighting_power;
            vec_t lighting_scale;
            lighting_power = state.lightingconeinfo[(size_t)miptex][0];
            lighting_scale = state.lightingconeinfo[(size_t)miptex][1];
            lighting_diversify = (lighting_power != 1.0 || lighting_scale != 1.0);
            // when false, test_segment_against_opaque_list is a no op
            const bool shadowtests_active = (!state.opaque_list.empty() || state.num_studio_models != 0);
            vec3v texlightgap_textoworld[2];
            // only needed for texlights with a nonzero gap; computed on demand
            bool texlightgap_computed = false;

            auto adds_touch = [&](int s)
            {
                if (!(adds_touched & (1ull << s)))
                {
                    adds_touched |= 1ull << s;
                    math::clear(adds[s]);
                }
            };

            auto texlightgap_setup = [&]()
            {
                if (!texlightgap_computed)
                {
                    texlightgap_computed = true;
                    const format::dface_t *f = &state.map->faces[(size_t)texlightgap_surfacenum];
                    const plane *dp = plane_from_face(state, f);
                    const format::texinfo_t *tex = &state.map->texinfo[(size_t)f->texinfo];
                    int x;
                    vec_t len;
                    for (x = 0; x < 2; x++)
                    {
                        vec3v tv{tex->vecs[1 - x][0], tex->vecs[1 - x][1], tex->vecs[1 - x][2]};
                        math::cross(tv, dp->normal, texlightgap_textoworld[x]);
                        len = math::dot(texlightgap_textoworld[x], tex->vecs[x]);
                        if (std::fabs(len) < math::normal_epsilon)
                        {
                            math::clear(texlightgap_textoworld[x]);
                        }
                        else
                        {
                            math::scale(texlightgap_textoworld[x], 1 / len, texlightgap_textoworld[x]);
                        }
                    }
                }
            };

            for (int ileaf = 0; ileaf < (int)state.lightleafs.size(); ileaf++)
            {
                i = state.lightleafs[(size_t)ileaf].leafnum;
                {
                    if (i == 0 ? state.options.sky_lighting_fix : pvs[(i - 1) >> 3] & (1 << ((i - 1) & 7)))
                    {
                        directlight *lend = state.lightarray.data() + state.lightleafs[(size_t)ileaf].firstlight + state.lightleafs[(size_t)ileaf].numlights;
                        for (l = state.lightarray.data() + state.lightleafs[(size_t)ileaf].firstlight; l < lend; l++)
                        {
                            // skylights work fundamentally differently than normal lights
                            if (l->type == emit_type::skylight)
                            {
                                if (!state.options.sky_lighting_fix)
                                {
                                    if (sky_used)
                                    {
                                        continue;
                                    }
                                    sky_used = true;
                                }
                                do // add sun light
                                {
                                    // check step
                                    step_match = (int)l->topatch;
                                    if (step != step_match)
                                        continue;
                                    // check intensity
                                    if (!(l->intensity[0] || l->intensity[1] || l->intensity[2]))
                                        continue;
                                    // loop over the normals
                                    for (int j = 0; j < l->numsunnormals; j++)
                                    {
                                        // make sure the angle is okay
                                        dot = -math::dot(normal, l->sunnormals[j]);
                                        if (dot <= math::normal_epsilon)
                                        {
                                            continue;
                                        }

                                        // search back to see if we can hit a sky brush
                                        math::scale(l->sunnormals[j], -bogus_range, delta);
                                        math::add(pos, delta, delta);
                                        vec3v skyhit;
                                        math::copy(delta, skyhit);
                                        if (test_line(state, pos, delta, &skyhit[0]) != contents_sky)
                                        {
                                            continue; // occluded
                                        }

                                        vec3v transparency = {1.0, 1.0, 1.0};
                                        int opaquestyle = -1;
                                        if (shadowtests_active)
                                        {
                                            if (test_segment_against_opaque_list(state, pos, skyhit, transparency, opaquestyle))
                                            {
                                                continue;
                                            }
                                        }

                                        vec3v add_one;
                                        if (lighting_diversify)
                                        {
                                            dot = (vec_t)(lighting_scale * std::pow((double)dot, lighting_power));
                                        }
                                        math::scale(l->intensity, dot * l->sunnormalweights[j], add_one);
                                        math::multiply(add_one, transparency, add_one);
                                        // add to the total brightness of this sample
                                        style = l->style;
                                        if (opaquestyle != -1)
                                        {
                                            if (style == 0 || style == opaquestyle)
                                                style = opaquestyle;
                                            else
                                                continue; // dynamic light of other styles hitting a toggleable opaque entity vanishes
                                        }
                                        adds_touch(style);
                                        math::add(adds[style], add_one, adds[style]);
                                    } // (loop over the normals)
                                } while (0);
                                do // add sky light
                                {
                                    // check step
                                    step_match = 0;
                                    if (state.options.softsky)
                                        step_match = 1;
                                    if (state.options.fastmode)
                                        step_match = 1;
                                    if (step != step_match)
                                        continue;
                                    // check intensity
                                    if (state.options.indirect_sun <= 0.0 ||
                                        (math::equal(l->diffuse_intensity, vec3v{})
                                         && math::equal(l->diffuse_intensity2, vec3v{})))
                                        continue;

                                    vec3v sky_intensity;

                                    // loop over the normals
                                    int skylevel = state.options.softsky ? skylevel_softsky_on : skylevel_softsky_off;
                                    const vec3v *skynormals = state.skynormals[skylevel].data();
                                    const vec_t *skyweights = state.skynormalsizes[skylevel].data();
                                    for (int j = 0; j < state.numskynormals[skylevel]; j++)
                                    {
                                        // make sure the angle is okay
                                        dot = -math::dot(normal, skynormals[j]);
                                        if (dot <= math::normal_epsilon)
                                        {
                                            continue;
                                        }

                                        // search back to see if we can hit a sky brush
                                        math::scale(skynormals[j], -bogus_range, delta);
                                        math::add(pos, delta, delta);
                                        vec3v skyhit;
                                        math::copy(delta, skyhit);
                                        if (test_line(state, pos, delta, &skyhit[0]) != contents_sky)
                                        {
                                            continue; // occluded
                                        }

                                        vec3v transparency = {1.0, 1.0, 1.0};
                                        int opaquestyle = -1;
                                        if (shadowtests_active)
                                        {
                                            if (test_segment_against_opaque_list(state, pos, skyhit, transparency, opaquestyle))
                                            {
                                                continue;
                                            }
                                        }

                                        // how far this piece of sky has deviated from the sun
                                        vec_t factor;
                                        {
                                            float x = (1 - math::dot(l->normal, skynormals[j])) / 2;
                                            double y = 0.0 > x ? 0.0 : x;
                                            factor = (vec_t)(y < 1.0 ? y : 1.0);
                                        }
                                        math::scale(l->diffuse_intensity, 1 - factor, sky_intensity);
                                        math::multiply_add(sky_intensity, factor, l->diffuse_intensity2, sky_intensity);
                                        math::scale(sky_intensity, skyweights[j] * state.options.indirect_sun / 2, sky_intensity);
                                        vec3v add_one;
                                        if (lighting_diversify)
                                        {
                                            dot = (vec_t)(lighting_scale * std::pow((double)dot, lighting_power));
                                        }
                                        math::scale(sky_intensity, dot, add_one);
                                        math::multiply(add_one, transparency, add_one);
                                        // add to the total brightness of this sample
                                        style = l->style;
                                        if (opaquestyle != -1)
                                        {
                                            if (style == 0 || style == opaquestyle)
                                                style = opaquestyle;
                                            else
                                                continue;
                                        }
                                        adds_touch(style);
                                        math::add(adds[style], add_one, adds[style]);
                                    } // (loop over the normals)
                                } while (0);
                            }
                            else // not emit_skylight
                            {
                                step_match = (int)l->topatch;
                                if (step != step_match)
                                    continue;
                                if (!(l->intensity[0] || l->intensity[1] || l->intensity[2]))
                                    continue;
                                math::copy(l->origin, testline_origin);

                                math::subtract(l->origin, pos, delta);
                                if (l->type == emit_type::surface)
                                {
                                    // move emitter back to its plane
                                    math::multiply_add(delta, -patch_hunt_offset, l->normal, delta);
                                }
                                dist = math::normalize(delta);
                                dot = math::dot(delta, normal);

                                if (dist < 1.0)
                                {
                                    dist = 1.0;
                                }

                                vec3v add;
                                switch (l->type)
                                {
                                case emit_type::point:
                                {
                                    if (dot <= math::normal_epsilon)
                                    {
                                        continue;
                                    }
                                    vec_t denominator = dist * dist * l->fade;
                                    if (lighting_diversify)
                                    {
                                        dot = (vec_t)(lighting_scale * std::pow((double)dot, lighting_power));
                                    }
                                    ratio = dot / denominator;
                                    math::scale(l->intensity, ratio, add);
                                    break;
                                }

                                case emit_type::surface:
                                {
                                    bool light_behind_surface = false;
                                    if (dot <= math::normal_epsilon)
                                    {
                                        light_behind_surface = true;
                                    }
                                    if (lighting_diversify && !light_behind_surface)
                                    {
                                        dot = (vec_t)(lighting_scale * std::pow((double)dot, lighting_power));
                                    }
                                    dot2 = -math::dot(delta, l->normal);
                                    // discard the texlight if the spot is too close to the texlight plane
                                    if (l->texlightgap > 0)
                                    {
                                        vec_t test;

                                        texlightgap_setup();
                                        test = dot2 * dist; // distance from spot to texlight plane
                                        // maximum distance reduction if the spot is allowed to shift texlightgap pixels along the s and t axes
                                        test -= l->texlightgap * std::fabs(math::dot(l->normal, texlightgap_textoworld[0]));
                                        test -= l->texlightgap * std::fabs(math::dot(l->normal, texlightgap_textoworld[1]));
                                        if (test < -math::on_epsilon)
                                        {
                                            continue;
                                        }
                                    }
                                    if (dot2 * dist <= minimum_patch_distance)
                                    {
                                        continue;
                                    }
                                    vec_t range = l->patch_emitter_range;
                                    if (l->stopdot > 0.0) // stopdot2 > 00 or stopdot > 00
                                    {
                                        vec_t range_scale;
                                        range_scale = 1 - l->stopdot2 * l->stopdot2;
                                        range_scale = (vec_t)(1 / std::sqrt(math::normal_epsilon > range_scale ? math::normal_epsilon : (double)range_scale));
                                        // range_scale = 1 / sin(cone2)
                                        range_scale = range_scale < 2 ? range_scale : 2; // restrict to 2 because skylevel has a limit
                                        range *= range_scale; // because smaller cones are more likely to create the ugly grid effect

                                        if (dot2 <= l->stopdot2 + math::normal_epsilon)
                                        {
                                            if (dist >= range) // the old method, which merely gives 0 in this case
                                            {
                                                continue;
                                            }
                                            ratio = 0.0;
                                        }
                                        else if (dot2 <= l->stopdot)
                                        {
                                            ratio = dot * dot2 * (dot2 - l->stopdot2) / (dist * dist * (l->stopdot - l->stopdot2));
                                        }
                                        else
                                        {
                                            ratio = dot * dot2 / (dist * dist);
                                        }
                                    }
                                    else
                                    {
                                        ratio = dot * dot2 / (dist * dist);
                                    }

                                    // analogous to the one in make_scales
                                    // 04f is tested to fully eliminate bright spots
                                    if (ratio * l->patch_area > 0.4f)
                                    {
                                        ratio = 0.4f / l->patch_area;
                                    }
                                    if (dist < range - math::on_epsilon)
                                    {
                                        // do things slow
                                        if (light_behind_surface)
                                        {
                                            dot = 0.0;
                                            ratio = 0.0;
                                        }
                                        get_alternate_origin(state, pos, normal, l->source_patch, testline_origin);
                                        vec_t sightarea;
                                        int skylevel = l->source_patch->emitter_skylevel;
                                        if (l->stopdot > 0.0) // stopdot2 > 00 or stopdot > 00
                                        {
                                            const vec3v &emitnormal = plane_from_face_number(state, (unsigned)l->source_patch->facenumber)->normal;
                                            if (l->stopdot2 >= 0.8) // about 37 degrees
                                            {
                                                skylevel += 1; // because the range is larger
                                            }
                                            sightarea = calc_sight_area_spotlight(state, pos, normal, l->source_patch->winding,
                                                                                  emitnormal, l->stopdot, l->stopdot2, skylevel,
                                                                                  lighting_power, lighting_scale); // because we have doubled the range
                                        }
                                        else
                                        {
                                            sightarea = calc_sight_area(state, pos, normal, l->source_patch->winding, skylevel,
                                                                        lighting_power, lighting_scale);
                                        }

                                        vec_t frac = dist / range;
                                        frac = (vec_t)((frac - 0.5) * 2); // make a smooth transition between the two methods
                                        frac = frac < 1 ? frac : 1;
                                        frac = 0 > frac ? 0 : frac;

                                        // because l->patch->area has been multiplied into l->intensity
                                        vec_t ratio2 = (sightarea / l->patch_area);
                                        ratio = frac * ratio + (1 - frac) * ratio2;
                                    }
                                    else
                                    {
                                        if (light_behind_surface)
                                        {
                                            continue;
                                        }
                                    }
                                    math::scale(l->intensity, ratio, add);
                                    break;
                                }

                                case emit_type::spotlight:
                                {
                                    if (dot <= math::normal_epsilon)
                                    {
                                        continue;
                                    }
                                    dot2 = -math::dot(delta, l->normal);
                                    if (dot2 <= l->stopdot2)
                                    {
                                        continue; // outside light cone
                                    }

                                    // variable power falloff (1 = inverse linear, 2 = inverse square)
                                    vec_t denominator = dist * l->fade;
                                    {
                                        denominator *= dist;
                                    }
                                    if (lighting_diversify)
                                    {
                                        dot = (vec_t)(lighting_scale * std::pow((double)dot, lighting_power));
                                    }
                                    ratio = dot * dot2 / denominator;

                                    if (dot2 <= l->stopdot)
                                    {
                                        ratio *= (dot2 - l->stopdot2) / (l->stopdot - l->stopdot2);
                                    }
                                    math::scale(l->intensity, ratio, add);
                                    break;
                                }

                                default:
                                {
                                    err::fatal("gather_sample_light: bad light type");
                                    break;
                                }
                                }
                                if (test_line(state, pos, testline_origin) != contents_empty)
                                {
                                    continue;
                                }
                                // add to the total brightness of this sample
                                style = l->style;
                                if (shadowtests_active)
                                {
                                    vec3v transparency;
                                    int opaquestyle;
                                    if (test_segment_against_opaque_list(state, pos, testline_origin, transparency, opaquestyle))
                                    {
                                        continue;
                                    }
                                    math::multiply(add, transparency, add);
                                    if (opaquestyle != -1)
                                    {
                                        if (style == 0 || style == opaquestyle)
                                            style = opaquestyle;
                                        else
                                            continue;
                                    }
                                }
                                adds_touch(style);
                                math::add(adds[style], add, adds[style]);
                            } // end emit_skylight
                        }
                    }
                }
            }

            for (style = 0; style < allstyles; ++style)
            {
                if (!(adds_touched & (1ull << style)) && state.corings[style] >= 0)
                {
                    // adds[style] is exactly zero; neither branch below would have
                    // any effect (maxdiscardedlight only grows and starts at 0)
                    continue;
                }
                if (!(adds_touched & (1ull << style)))
                {
                    math::clear(adds[style]); // pathological negative coring: preserve original behavior
                }
                if (vector_maximum(adds[style]) > state.corings[style] * 0.1)
                {
                    for (style_index = 0; style_index < allstyles; style_index++)
                    {
                        if (styles[style_index] == style || styles[style_index] == 255)
                        {
                            break;
                        }
                    }

                    if (style_index == allstyles) // shouldn't happen
                    {
                        std::lock_guard<std::mutex> guard(state.lock);
                        if (++state.stylewarningcount >= state.stylewarningnext)
                        {
                            state.stylewarningnext = state.stylewarningcount * 2;
                            logging::warn("Too many direct light styles on a face(%f,%f,%f)", pos[0], pos[1], pos[2]);
                            logging::warn(" total %d warnings for too many styles", state.stylewarningcount);
                        }
                        return;
                    }

                    if (styles[style_index] == 255)
                    {
                        styles[style_index] = (byte)style;
                    }

                    math::add(sample[style_index], adds[style], sample[style_index]);
                }
                else
                {
                    if (vector_maximum(adds[style]) > state.maxdiscardedlight + math::normal_epsilon)
                    {
                        std::lock_guard<std::mutex> guard(state.lock);
                        if (vector_maximum(adds[style]) > state.maxdiscardedlight + math::normal_epsilon)
                        {
                            state.maxdiscardedlight = vector_maximum(adds[style]);
                            math::copy(pos, state.maxdiscardedpos);
                        }
                    }
                }
            }
        }

        // takes each sample's collected light and adds it back into the
        // appropriate patch for the radiosity pass
        void add_samples_to_patches(rad_state &state, const sample **samples, const unsigned char *styles,
                                    int facenum, const lightinfo *l)
        {
            patch *pt;
            int i, j, m, k;
            int numtexwindings;

            numtexwindings = 0;
            for (pt = state.face_patches[(size_t)facenum]; pt; pt = pt->next)
            {
                numtexwindings++;
            }
            std::vector<math::winding> texwindings((size_t)numtexwindings);

            // translate world windings into windings in the s,t plane
            for (j = 0, pt = state.face_patches[(size_t)facenum]; j < numtexwindings; j++, pt = pt->next)
            {
                std::vector<vec3v> pts((size_t)pt->winding->size());
                for (int x = 0; x < (int)pts.size(); x++)
                {
                    vec_t s, t;
                    set_st_from_surf(state, l, (*pt->winding)[x], s, t);
                    pts[(size_t)x][0] = s;
                    pts[(size_t)x][1] = t;
                    pts[(size_t)x][2] = 0.0;
                }
                texwindings[(size_t)j] = math::winding{std::move(pts)};
                texwindings[(size_t)j].remove_colinear_points();
            }

            for (i = 0; i < l->numsurfpt; i++)
            {
                // prepare clip planes
                vec_t s_vec, t_vec;
                s_vec = (vec_t)(l->texmins[0] * texture_step + (i % (l->texsize[0] + 1)) * texture_step);
                t_vec = (vec_t)(l->texmins[1] * texture_step + (i / (l->texsize[0] + 1)) * texture_step);

                plane clipplanes[4];
                math::clear(clipplanes[0].normal);
                clipplanes[0].normal[0] = 1;
                clipplanes[0].dist = (vec_t)(s_vec - 0.5 * texture_step);
                math::clear(clipplanes[1].normal);
                clipplanes[1].normal[0] = -1;
                clipplanes[1].dist = (vec_t)(-(s_vec + 0.5 * texture_step));
                math::clear(clipplanes[2].normal);
                clipplanes[2].normal[1] = 1;
                clipplanes[2].dist = (vec_t)(t_vec - 0.5 * texture_step);
                math::clear(clipplanes[3].normal);
                clipplanes[3].normal[1] = -1;
                clipplanes[3].dist = (vec_t)(-(t_vec + 0.5 * texture_step));

                // clip each patch
                for (j = 0, pt = state.face_patches[(size_t)facenum]; j < numtexwindings; j++, pt = pt->next)
                {
                    math::winding w = texwindings[(size_t)j];
                    for (k = 0; k < 4; k++)
                    {
                        if (w.size())
                        {
                            w.clip_in_place(clipplanes[k].normal, clipplanes[k].dist, false);
                        }
                    }
                    if (w.size())
                    {
                        // add the sample to the patch
                        vec_t area = w.area() / (texture_step * texture_step);
                        pt->samples += area;
                        for (m = 0; m < allstyles && styles[m] != 255; m++)
                        {
                            int style = styles[m];
                            const sample *s = &samples[m][i];
                            for (k = 0; k < allstyles && pt->totalstyle_all[k] != 255; k++)
                            {
                                if (pt->totalstyle_all[k] == style)
                                {
                                    break;
                                }
                            }
                            if (k == allstyles)
                            {
                                std::lock_guard<std::mutex> guard(state.lock);
                                if (++state.stylewarningcount >= state.stylewarningnext)
                                {
                                    state.stylewarningnext = state.stylewarningcount * 2;
                                    logging::warn("Too many direct light styles on a face(?,?,?)\n");
                                    logging::warn(" total %d warnings for too many styles", state.stylewarningcount);
                                }
                            }
                            else
                            {
                                if (pt->totalstyle_all[k] == 255)
                                {
                                    pt->totalstyle_all[k] = (unsigned char)style;
                                }
                                math::multiply_add(pt->samplelight_all[k], area, s->light, pt->samplelight_all[k]);
                            }
                        }
                    }
                }
            }
        }
    }

    // test seam for cpu and gpu comparisons which forwards to
    // the file local gather so gather_sample_light itself keeps its internal
    // linkage and codegen untouched
    void gather_sample_light_for_check(rad_state &state, const vec3v &pos, const byte *pvs,
                                       const vec3v &normal, vec3v *sample, byte *styles, int step,
                                       int miptex, int texlightgap_surfacenum)
    {
        gather_sample_light(state, pos, pvs, normal, sample, styles, step, miptex,
                            texlightgap_surfacenum);
    }

    // ===== phong normals =====

    void get_phong_normal(const rad_state &state, int facenum, const vec3v &spot, vec3v &phongnormal)
    {
        const format::map_data &map = *state.map;
        int j;
        int s; // split every edge into two parts
        const format::dface_t *f = &map.faces[(size_t)facenum];
        const plane *p = plane_from_face(state, f);
        vec3v facenormal;

        math::copy(p->normal, facenormal);
        math::copy(facenormal, phongnormal);

        {
            // find the edge normals that bound the point and interpolate
            for (j = 0; j < f->numedges; j++)
            {
                vec3v p1;
                vec3v p2;
                vec3v v1;
                vec3v v2;
                vec3v vspot;
                unsigned prev_edge;
                unsigned next_edge;
                int e;
                int e1;
                int e2;
                const edgeshare *es;
                const edgeshare *es1;
                const edgeshare *es2;
                float a1;
                float a2;
                float aa;
                float bb;
                float ab;

                if (j)
                {
                    prev_edge = (unsigned)(f->firstedge + ((j + f->numedges - 1) % f->numedges));
                }
                else
                {
                    prev_edge = (unsigned)(f->firstedge + f->numedges - 1);
                }

                if ((j + 1) != f->numedges)
                {
                    next_edge = (unsigned)(f->firstedge + ((j + 1) % f->numedges));
                }
                else
                {
                    next_edge = (unsigned)f->firstedge;
                }

                e = map.surfedges[(size_t)(f->firstedge + j)];
                e1 = map.surfedges[prev_edge];
                e2 = map.surfedges[next_edge];

                es = &state.edgeshares[(size_t)abs(e)];
                es1 = &state.edgeshares[(size_t)abs(e1)];
                es2 = &state.edgeshares[(size_t)abs(e2)];

                if ((!es->smooth || es->coplanar) && (!es1->smooth || es1->coplanar) && (!es2->smooth || es2->coplanar))
                {
                    continue;
                }

                if (e > 0)
                {
                    const float *pt1 = map.vertexes[map.edges[(size_t)e].v[0]].point;
                    const float *pt2 = map.vertexes[map.edges[(size_t)e].v[1]].point;
                    p1 = vec3v{pt1[0], pt1[1], pt1[2]};
                    p2 = vec3v{pt2[0], pt2[1], pt2[2]};
                }
                else
                {
                    const float *pt1 = map.vertexes[map.edges[(size_t)-e].v[1]].point;
                    const float *pt2 = map.vertexes[map.edges[(size_t)-e].v[0]].point;
                    p1 = vec3v{pt1[0], pt1[1], pt1[2]};
                    p2 = vec3v{pt2[0], pt2[1], pt2[2]};
                }

                // adjust for origin based models
                math::add(p1, state.face_offset[(size_t)facenum], p1);
                math::add(p2, state.face_offset[(size_t)facenum], p2);
                for (s = 0; s < 2; s++)
                {
                    vec3v s1, s2;
                    if (s == 0)
                    {
                        math::copy(p1, s1);
                    }
                    else
                    {
                        math::copy(p2, s1);
                    }

                    math::add(p1, p2, s2); // edge center
                    math::scale(s2, 0.5, s2);

                    math::subtract(s1, state.face_centroids[(size_t)facenum], v1);
                    math::subtract(s2, state.face_centroids[(size_t)facenum], v2);
                    math::subtract(spot, state.face_centroids[(size_t)facenum], vspot);

                    aa = math::dot(v1, v1);
                    bb = math::dot(v2, v2);
                    ab = math::dot(v1, v2);
                    a1 = (bb * math::dot(v1, vspot) - ab * math::dot(vspot, v2)) / (aa * bb - ab * ab);
                    a2 = (math::dot(vspot, v2) - a1 * ab) / bb;

                    // test the center to sample vector for inclusion between the center to vertex vectors
                    if (a1 >= -0.01 && a2 >= -0.01)
                    {
                        // calculate the distance from the edge to pos
                        vec3v n1, n2;
                        vec3v temp;

                        if (es->smooth)
                        {
                            if (s == 0)
                            {
                                math::copy(es->vertex_normal[e > 0 ? 0 : 1], n1);
                            }
                            else
                            {
                                math::copy(es->vertex_normal[e > 0 ? 1 : 0], n1);
                            }
                        }
                        else if (s == 0 && es1->smooth)
                        {
                            math::copy(es1->vertex_normal[e1 > 0 ? 1 : 0], n1);
                        }
                        else if (s == 1 && es2->smooth)
                        {
                            math::copy(es2->vertex_normal[e2 > 0 ? 0 : 1], n1);
                        }
                        else
                        {
                            math::copy(facenormal, n1);
                        }

                        if (es->smooth)
                        {
                            math::copy(es->interface_normal, n2);
                        }
                        else
                        {
                            math::copy(facenormal, n2);
                        }

                        // interpolate between the center and edge normals based on sample position
                        math::scale(facenormal, 1.0 - a1 - a2, phongnormal);
                        math::scale(n1, a1, temp);
                        math::add(phongnormal, temp, phongnormal);
                        math::scale(n2, a2, temp);
                        math::add(phongnormal, temp, phongnormal);
                        math::normalize(phongnormal);
                        break;
                    }
                } // s = 0, 1
            }
        }
    }

    const vec3v s_circuscolors[] = {
        {100000.0, 100000.0, 100000.0}, // white
        {100000.0, 0.0, 0.0},           // red
        {0.0, 100000.0, 0.0},           // green
        {0.0, 0.0, 100000.0},           // blue
        {0.0, 100000.0, 100000.0},      // cyan
        {100000.0, 0.0, 100000.0},      // magenta
        {100000.0, 100000.0, 0.0}       // yellow
    };

    // ===== build facelights =====

    namespace
    {
        void calc_lightmap(rad_state &state, lightinfo *l, byte *styles)
        {
            int facenum;
            int i, j;
            byte pvs[(limits::max_map_leafs + 7) / 8];
            int lastoffset = 0;
            byte pvs2[(limits::max_map_leafs + 7) / 8];
            int lastoffset2 = 0;

            facenum = l->surfnum;
            memset(l->lmcache, 0, (size_t)(l->lmcachewidth * l->lmcacheheight) * sizeof(vec3v[allstyles]));

            // for each sample whose light we need to calculate
            for (i = 0; i < l->lmcachewidth * l->lmcacheheight; i++)
            {
                vec_t s, t;
                vec_t s_vec, t_vec;
                int nearest_s, nearest_t;
                vec3v spot;
                // the max possible range in which this sample point affects the lighting on a face
                vec_t square[2][2];
                // the point on the surface (with no hunt offset applied), used
                // for getting the phong normal and doing patch interpolation
                vec3v surfpt;
                int surface;
                vec3v pointnormal;
                bool blocked;
                vec3v spot2;
                vec3v pointnormal2;
                vec3v *sampled;
                vec3v *normal_out;
                bool nudged;
                int *wallflags_out;

                // prepare input parameters and output destinations
                {
                    s = ((i % l->lmcachewidth) - l->lmcache_offset) / (vec_t)l->lmcache_density;
                    t = ((i / l->lmcachewidth) - l->lmcache_offset) / (vec_t)l->lmcache_density;
                    s_vec = (vec_t)(l->texmins[0] * texture_step + s * texture_step);
                    t_vec = (vec_t)(l->texmins[1] * texture_step + t * texture_step);
                    {
                        int ns = (int)std::floor(s + 0.5);
                        ns = ns < l->texsize[0] ? ns : l->texsize[0];
                        nearest_s = 0 > ns ? 0 : ns;
                        int nt = (int)std::floor(t + 0.5);
                        nt = nt < l->texsize[1] ? nt : l->texsize[1];
                        nearest_t = 0 > nt ? 0 : nt;
                    }
                    sampled = l->lmcache[i];
                    normal_out = &l->lmcache_normal[i];
                    wallflags_out = &l->lmcache_wallflags[i];

                    // the range in which a sample point affects lighting extends
                    // lmcache_side luxels around it (see the reference diagram)
                    square[0][0] = (vec_t)(l->texmins[0] * texture_step + std::ceil(s - (l->lmcache_side + 0.5) / (vec_t)l->lmcache_density) * texture_step - texture_step);
                    square[0][1] = (vec_t)(l->texmins[1] * texture_step + std::ceil(t - (l->lmcache_side + 0.5) / (vec_t)l->lmcache_density) * texture_step - texture_step);
                    square[1][0] = (vec_t)(l->texmins[0] * texture_step + std::floor(s + (l->lmcache_side + 0.5) / (vec_t)l->lmcache_density) * texture_step + texture_step);
                    square[1][1] = (vec_t)(l->texmins[1] * texture_step + std::floor(t + (l->lmcache_side + 0.5) / (vec_t)l->lmcache_density) * texture_step + texture_step);
                }
                // find the world's position for the sample
                {
                    {
                        blocked = false;
                        if (set_sample_from_st(state, surfpt, spot, &surface, &nudged, l, s_vec, t_vec, square) == light_outside)
                        {
                            j = nearest_s + (l->texsize[0] + 1) * nearest_t;
                            if (l->surfpt_lightoutside[j])
                            {
                                blocked = true;
                            }
                            else
                            {
                                // the area this light sample affects is
                                // completely covered by solid; take whatever
                                // valid position
                                math::copy(l->surfpt[j], surfpt);
                                math::copy(l->surfpt_position[j], spot);
                                surface = l->surfpt_surface[j];
                            }
                        }
                    }
                    if (l->translucent_b)
                    {
                        const plane *surfaceplane = plane_from_face_number(state, (unsigned)surface);
                        math::winding surfacewinding = winding_from_face(state, state.map->faces[(size_t)surface]);

                        math::copy(spot, spot2);
                        for (int x = 0; x < surfacewinding.size(); x++)
                        {
                            math::add(surfacewinding[x], state.face_offset[(size_t)surface], surfacewinding[x]);
                        }
                        if (!point_in_winding_noedge(surfacewinding, *surfaceplane, spot2, 0.2f))
                        {
                            snap_to_winding_noedge(surfacewinding, *surfaceplane, spot2, 0.2f, 4 * 0.2f);
                        }
                        math::multiply_add(spot2, -(state.options.translucentdepth + 2 * default_hunt_offset), surfaceplane->normal, spot2);
                    }
                    *wallflags_out = wallflag_none;
                    if (blocked)
                    {
                        *wallflags_out |= (wallflag_blocked | wallflag_nudged);
                    }
                    if (nudged)
                    {
                        *wallflags_out |= wallflag_nudged;
                    }
                }
                // calculate the normal for the sample
                {
                    get_phong_normal(state, surface, surfpt, pointnormal);
                    if (l->translucent_b)
                    {
                        for (int x = 0; x < 3; x++)
                            pointnormal2[x] = 0.0f - pointnormal[x];
                    }
                    math::copy(pointnormal, *normal_out);
                }
                // calculate visibility for the sample
                {
                    if (state.map->visibility.empty())
                    {
                        if (i == 0)
                        {
                            memset(pvs, 255, (size_t)((state.map->models[0].visleafs + 7) / 8));
                        }
                    }
                    else
                    {
                        format::dleaf_t *leaf = point_in_leaf(state, spot);
                        int thisoffset = leaf->visofs;
                        if (i == 0 || thisoffset != lastoffset)
                        {
                            if (thisoffset == -1)
                            {
                                memset(pvs, 0, (size_t)((state.map->models[0].visleafs + 7) / 8));
                            }
                            else
                            {
                                decompress_vis(state, &state.map->visibility[(size_t)leaf->visofs], pvs, sizeof(pvs));
                            }
                        }
                        lastoffset = thisoffset;
                    }
                    if (l->translucent_b)
                    {
                        if (state.map->visibility.empty())
                        {
                            if (i == 0)
                            {
                                memset(pvs2, 255, (size_t)((state.map->models[0].visleafs + 7) / 8));
                            }
                        }
                        else
                        {
                            format::dleaf_t *leaf2 = point_in_leaf(state, spot2);
                            int thisoffset2 = leaf2->visofs;
                            if (i == 0 || thisoffset2 != lastoffset2)
                            {
                                if (thisoffset2 == -1)
                                {
                                    memset(pvs2, 0, (size_t)((state.map->models[0].visleafs + 7) / 8));
                                }
                                else
                                {
                                    decompress_vis(state, &state.map->visibility[(size_t)leaf2->visofs], pvs2, sizeof(pvs2));
                                }
                            }
                            lastoffset2 = thisoffset2;
                        }
                    }
                }
                // gather light
                {
                    if (!blocked)
                    {
                        gather_sample_light(state, spot, pvs, pointnormal, sampled, styles, 0, l->miptex, surface);
                    }
                    if (l->translucent_b)
                    {
                        vec3v sampled2[allstyles];
                        memset(sampled2, 0, allstyles * sizeof(vec3v));
                        if (!blocked)
                        {
                            gather_sample_light(state, spot2, pvs2, pointnormal2, sampled2, styles, 0, l->miptex, surface);
                        }
                        for (j = 0; j < allstyles && styles[j] != 255; j++)
                        {
                            for (int x = 0; x < 3; x++)
                            {
                                sampled[j][x] = (vec_t)((1.0 - l->translucent_v[x]) * sampled[j][x] + l->translucent_v[x] * sampled2[j][x]);
                            }
                        }
                    }
                    if (state.options.drawnudge)
                    {
                        for (j = 0; j < allstyles && styles[j] != 255; j++)
                        {
                            if (blocked && styles[j] == 0)
                            {
                                sampled[j][0] = 200;
                                sampled[j][1] = 0;
                                sampled[j][2] = 0;
                            }
                            else if (nudged && styles[j] == 0) // we assume style 0 is always present
                            {
                                sampled[j] = vec3v{100, 100, 100};
                            }
                            else
                            {
                                math::clear(sampled[j]);
                            }
                        }
                    }
                }
            }
        }
    }

    void build_facelights(rad_state &state, const int facenum)
    {
        format::map_data &map = *state.map;
        format::dface_t *f;
        unsigned char f_styles[allstyles];
        sample *fl_samples[allstyles];
        lightinfo l;
        int i;
        int j;
        int k;
        sample *s;
        patch *pt;
        const plane *pl;
        byte pvs[(limits::max_map_leafs + 7) / 8];
        int thisoffset = -1, lastoffset = -1;
        int lightmapwidth;
        int lightmapheight;
        int size;
        vec3v spot2, normal2;
        byte pvs2[(limits::max_map_leafs + 7) / 8];
        int thisoffset2 = -1, lastoffset2 = -1;

        int *sample_wallflags;

        f = &map.faces[(size_t)facenum];

        //
        // some surfaces don't need lightmaps
        //
        f->lightofs = -1;
        for (j = 0; j < allstyles; j++)
        {
            f_styles[j] = 255;
        }

        if (map.texinfo[(size_t)f->texinfo].flags & tex_special)
        {
            for (j = 0; j < maxlightmaps; j++)
            {
                f->styles[j] = 255;
            }
            return; // non lit texture
        }

        f_styles[0] = 0;
        if (state.face_patches[(size_t)facenum] && state.face_patches[(size_t)facenum]->emitstyle)
        {
            f_styles[1] = state.face_patches[(size_t)facenum]->emitstyle;
        }

        memset(&l, 0, sizeof(l));

        l.surfnum = facenum;
        l.face = f;

        math::copy(state.translucenttextures[(size_t)map.texinfo[(size_t)f->texinfo].miptex], l.translucent_v);
        l.translucent_b = !math::equal(l.translucent_v, vec3v{});
        l.miptex = map.texinfo[(size_t)f->texinfo].miptex;

        //
        // rotate plane
        //
        pl = plane_from_face(state, f);
        math::copy(pl->normal, l.facenormal);
        l.facedist = pl->dist;

        calc_face_vectors(state, &l);
        calc_face_extents(state, &l);
        calc_points(state, &l);
        calc_lightmap(state, &l, f_styles);

        lightmapwidth = l.texsize[0] + 1;
        lightmapheight = l.texsize[1] + 1;

        size = lightmapwidth * lightmapheight;
        err::require(size <= max_singlemap, "build_facelights: exceeded MAX_SINGLEMAP");

        state.facelights[(size_t)facenum].numsamples = l.numsurfpt;

        for (k = 0; k < allstyles; k++)
        {
            fl_samples[k] = (sample *)calloc((size_t)l.numsurfpt, sizeof(sample));
            err::require(fl_samples[k] != nullptr, "build_facelights: out of memory");
        }
        for (pt = state.face_patches[(size_t)facenum]; pt; pt = pt->next)
        {
            pt->totalstyle_all = (unsigned char *)malloc(allstyles * sizeof(unsigned char));
            err::require(pt->totalstyle_all != nullptr, "build_facelights: out of memory");
            pt->samplelight_all = (vec3v *)malloc(allstyles * sizeof(vec3v));
            err::require(pt->samplelight_all != nullptr, "build_facelights: out of memory");
            pt->totallight_all = (vec3v *)malloc(allstyles * sizeof(vec3v));
            err::require(pt->totallight_all != nullptr, "build_facelights: out of memory");
            pt->directlight_all = (vec3v *)malloc(allstyles * sizeof(vec3v));
            err::require(pt->directlight_all != nullptr, "build_facelights: out of memory");
            for (j = 0; j < allstyles; j++)
            {
                pt->totalstyle_all[j] = 255;
                math::clear(pt->samplelight_all[j]);
                math::clear(pt->totallight_all[j]);
                math::clear(pt->directlight_all[j]);
            }
            pt->totalstyle_all[0] = 0;
        }

        sample_wallflags = (int *)malloc((size_t)((2 * l.lmcache_side + 1) * (2 * l.lmcache_side + 1)) * sizeof(int));
        for (i = 0; i < l.numsurfpt; i++)
        {
            const vec3v &spot = l.surfpt[i];

            for (k = 0; k < allstyles; k++)
            {
                math::copy(spot, fl_samples[k][i].pos);
                fl_samples[k][i].surface = l.surfpt_surface[i];
            }

            int s_i, t;
            int pos;
            int s_center, t_center;
            vec_t sizehalf;
            vec_t weighting, subsamples;
            vec3v centernormal;
            vec_t weighting_correction;
            int pass;
            s_center = (i % lightmapwidth) * l.lmcache_density + l.lmcache_offset;
            t_center = (i / lightmapwidth) * l.lmcache_density + l.lmcache_offset;
            sizehalf = (vec_t)(0.5 * state.options.blur * l.lmcache_density);
            subsamples = 0.0;
            math::copy(l.lmcache_normal[s_center + l.lmcachewidth * t_center], centernormal);
            if (state.options.bleedfix && !state.options.drawnudge)
            {
                int s_origin = s_center;
                int t_origin = t_center;
                for (s_i = s_center - l.lmcache_side; s_i <= s_center + l.lmcache_side; s_i++)
                {
                    for (t = t_center - l.lmcache_side; t <= t_center + l.lmcache_side; t++)
                    {
                        int *pwallflags = &sample_wallflags[(s_i - s_center + l.lmcache_side) + (2 * l.lmcache_side + 1) * (t - t_center + l.lmcache_side)];
                        *pwallflags = l.lmcache_wallflags[s_i + l.lmcachewidth * t];
                    }
                }
                // project the "shadow" from the origin point
                for (s_i = s_center - l.lmcache_side; s_i <= s_center + l.lmcache_side; s_i++)
                {
                    for (t = t_center - l.lmcache_side; t <= t_center + l.lmcache_side; t++)
                    {
                        int *pwallflags = &sample_wallflags[(s_i - s_center + l.lmcache_side) + (2 * l.lmcache_side + 1) * (t - t_center + l.lmcache_side)];
                        int coord[2] = {s_i - s_origin, t - t_origin};
                        int axis = abs(coord[0]) >= abs(coord[1]) ? 0 : 1;
                        int sign = coord[axis] >= 0 ? 1 : -1;
                        bool blocked1 = false;
                        bool blocked2 = false;
                        for (int dist = 1; dist < abs(coord[axis]); dist++)
                        {
                            int test1[2];
                            int test2[2];
                            test1[axis] = test2[axis] = sign * dist;
                            double intercept = (double)coord[1 - axis] * (double)test1[axis] / (double)coord[axis];
                            test1[1 - axis] = (int)std::floor(intercept + 0.01);
                            test2[1 - axis] = (int)std::ceil(intercept - 0.01);
                            if (abs(test1[0] + s_origin - s_center) > l.lmcache_side || abs(test1[1] + t_origin - t_center) > l.lmcache_side ||
                                abs(test2[0] + s_origin - s_center) > l.lmcache_side || abs(test2[1] + t_origin - t_center) > l.lmcache_side)
                            {
                                logging::warn("rad avoid wall bleed: internal error");
                                continue;
                            }
                            int wallflags1 = sample_wallflags[(test1[0] + s_origin - s_center + l.lmcache_side) + (2 * l.lmcache_side + 1) * (test1[1] + t_origin - t_center + l.lmcache_side)];
                            int wallflags2 = sample_wallflags[(test2[0] + s_origin - s_center + l.lmcache_side) + (2 * l.lmcache_side + 1) * (test2[1] + t_origin - t_center + l.lmcache_side)];
                            if (wallflags1 & wallflag_nudged)
                            {
                                blocked1 = true;
                            }
                            if (wallflags2 & wallflag_nudged)
                            {
                                blocked2 = true;
                            }
                        }
                        if (blocked1 && blocked2)
                        {
                            *pwallflags |= wallflag_shadowed;
                        }
                    }
                }
            }
            for (pass = 0; pass < 2; pass++)
            {
                for (s_i = s_center - l.lmcache_side; s_i <= s_center + l.lmcache_side; s_i++)
                {
                    for (t = t_center - l.lmcache_side; t <= t_center + l.lmcache_side; t++)
                    {
                        // the weight is the intersection length of the luxel's
                        // averaging square with the sample's cell
                        {
                            double wa = 0.5 < sizehalf - (s_i - s_center) ? 0.5 : sizehalf - (s_i - s_center);
                            double wb = -0.5 > -sizehalf - (s_i - s_center) ? -0.5 : -sizehalf - (s_i - s_center);
                            double wc = 0.5 < sizehalf - (t - t_center) ? 0.5 : sizehalf - (t - t_center);
                            double wd = -0.5 > -sizehalf - (t - t_center) ? -0.5 : -sizehalf - (t - t_center);
                            weighting = (vec_t)((wa - wb) * (wc - wd));
                        }
                        if (state.options.bleedfix && !state.options.drawnudge)
                        {
                            int wallflags = sample_wallflags[(s_i - s_center + l.lmcache_side) + (2 * l.lmcache_side + 1) * (t - t_center + l.lmcache_side)];
                            if (wallflags & (wallflag_blocked | wallflag_shadowed))
                            {
                                continue;
                            }
                            if (wallflags & wallflag_nudged)
                            {
                                if (pass == 0)
                                {
                                    continue;
                                }
                            }
                        }
                        pos = s_i + l.lmcachewidth * t;
                        // when the blur distance is large, the subsample can be
                        // very far from the original lightmap sample; this
                        // correction limits the effect of blur when the normal
                        // changes very fast, without breaking the smoothness
                        // that sample growing ensures
                        weighting_correction = math::dot(l.lmcache_normal[pos], centernormal);
                        weighting_correction = (weighting_correction > 0) ? weighting_correction * weighting_correction : 0;
                        weighting = weighting * weighting_correction;
                        for (j = 0; j < allstyles && f_styles[j] != 255; j++)
                        {
                            math::multiply_add(fl_samples[j][i].light, weighting, l.lmcache[pos][j], fl_samples[j][i].light);
                        }
                        subsamples += weighting;
                    }
                }
                if (subsamples > math::normal_epsilon)
                {
                    break;
                }
                else
                {
                    subsamples = 0.0;
                    for (j = 0; j < allstyles && f_styles[j] != 255; j++)
                    {
                        math::clear(fl_samples[j][i].light);
                    }
                }
            }
            if (subsamples > 0)
            {
                for (j = 0; j < allstyles && f_styles[j] != 255; j++)
                {
                    math::scale(fl_samples[j][i].light, 1.0 / subsamples, fl_samples[j][i].light);
                }
            }
        } // end of i loop
        free(sample_wallflags);

        // average up the direct light on each patch for radiosity
        add_samples_to_patches(state, (const sample **)fl_samples, f_styles, facenum, &l);
        {
            for (pt = state.face_patches[(size_t)facenum]; pt; pt = pt->next)
            {
                unsigned istyle;
                if (pt->samples <= math::on_epsilon * math::on_epsilon)
                    pt->samples = 0.0;
                if (pt->samples)
                {
                    for (istyle = 0; istyle < allstyles && pt->totalstyle_all[istyle] != 255; istyle++)
                    {
                        vec3v v;
                        math::scale(pt->samplelight_all[istyle], 1.0f / pt->samples, v);
                        math::add(pt->directlight_all[istyle], v, pt->directlight_all[istyle]);
                    }
                }
            }
        }
        for (pt = state.face_patches[(size_t)facenum]; pt; pt = pt->next)
        {
            // get the pvs for the pos to limit the number of checks
            if (state.map->visibility.empty())
            {
                memset(pvs, 255, (size_t)((map.models[0].visleafs + 7) / 8));
                lastoffset = -1;
            }
            else
            {
                format::dleaf_t *leaf = point_in_leaf(state, pt->origin);

                thisoffset = leaf->visofs;
                if (pt == state.face_patches[(size_t)facenum] || thisoffset != lastoffset)
                {
                    if (thisoffset == -1)
                    {
                        memset(pvs, 0, (size_t)((map.models[0].visleafs + 7) / 8));
                    }
                    else
                    {
                        decompress_vis(state, &state.map->visibility[(size_t)leaf->visofs], pvs, sizeof(pvs));
                    }
                }
                lastoffset = thisoffset;
            }
            if (l.translucent_b)
            {
                if (state.map->visibility.empty())
                {
                    memset(pvs2, 255, (size_t)((map.models[0].visleafs + 7) / 8));
                    lastoffset2 = -1;
                }
                else
                {
                    math::multiply_add(pt->origin, -(state.options.translucentdepth + 2 * patch_hunt_offset), l.facenormal, spot2);
                    format::dleaf_t *leaf2 = point_in_leaf(state, spot2);

                    thisoffset2 = leaf2->visofs;
                    if (l.numsurfpt == 0 || thisoffset2 != lastoffset2)
                    {
                        if (thisoffset2 == -1)
                        {
                            memset(pvs2, 0, (size_t)((map.models[0].visleafs + 7) / 8));
                        }
                        else
                        {
                            decompress_vis(state, &state.map->visibility[(size_t)leaf2->visofs], pvs2, sizeof(pvs2));
                        }
                    }
                    lastoffset2 = thisoffset2;
                }
                vec3v frontsampled[allstyles], backsampled[allstyles];
                for (j = 0; j < allstyles; j++)
                {
                    math::clear(frontsampled[j]);
                    math::clear(backsampled[j]);
                }
                for (int x = 0; x < 3; x++)
                    normal2[x] = 0.0f - l.facenormal[x];
                gather_sample_light(state, pt->origin, pvs, l.facenormal, frontsampled,
                                    pt->totalstyle_all, 1, l.miptex, facenum);
                gather_sample_light(state, spot2, pvs2, normal2, backsampled,
                                    pt->totalstyle_all, 1, l.miptex, facenum);
                for (j = 0; j < allstyles && pt->totalstyle_all[j] != 255; j++)
                {
                    for (int x = 0; x < 3; x++)
                    {
                        pt->totallight_all[j][x] = (vec_t)(pt->totallight_all[j][x]
                            + (1.0 - l.translucent_v[x]) * frontsampled[j][x] + l.translucent_v[x] * backsampled[j][x]);
                    }
                }
            }
            else
            {
                gather_sample_light(state, pt->origin, pvs, l.facenormal,
                                    pt->totallight_all, pt->totalstyle_all, 1, l.miptex, facenum);
            }
        }

        // add an ambient term if desired
        if (state.options.ambient[0] || state.options.ambient[1] || state.options.ambient[2])
        {
            for (j = 0; j < allstyles && f_styles[j] != 255; j++)
            {
                if (f_styles[j] == 0)
                {
                    s = fl_samples[j];
                    for (i = 0; i < l.numsurfpt; i++, s++)
                    {
                        vec3v amb{state.options.ambient[0], state.options.ambient[1], state.options.ambient[2]};
                        math::add(s->light, amb, s->light);
                    }
                    break;
                }
            }
        }

        // add circus lighting for finding black lightmaps
        if (state.options.circus)
        {
            for (j = 0; j < allstyles && f_styles[j] != 255; j++)
            {
                if (f_styles[j] == 0)
                {
                    int amt = 7;

                    s = fl_samples[j];

                    while ((l.numsurfpt % amt) == 0)
                    {
                        amt--;
                    }
                    if (amt < 2)
                    {
                        amt = 7;
                    }

                    for (i = 0; i < l.numsurfpt; i++, s++)
                    {
                        if ((s->light[0] == 0) && (s->light[1] == 0) && (s->light[2] == 0))
                        {
                            math::add(s->light, s_circuscolors[i % amt], s->light);
                        }
                    }
                    break;
                }
            }
        }

        // light from dlight_threshold and above is sent out, but the
        // texture itself should still be full bright
        {
            if (state.face_patches[(size_t)facenum])
            {
                for (j = 0; j < allstyles && f_styles[j] != 255; j++)
                {
                    if (f_styles[j] == state.face_patches[(size_t)facenum]->emitstyle)
                    {
                        break;
                    }
                }
                if (j == allstyles)
                {
                    std::lock_guard<std::mutex> guard(state.lock);
                    if (++state.stylewarningcount >= state.stylewarningnext)
                    {
                        state.stylewarningnext = state.stylewarningcount * 2;
                        logging::warn("Too many direct light styles on a face(?,?,?)");
                        logging::warn(" total %d warnings for too many styles", state.stylewarningcount);
                    }
                }
                else
                {
                    if (f_styles[j] == 255)
                    {
                        f_styles[j] = state.face_patches[(size_t)facenum]->emitstyle;
                    }

                    s = fl_samples[j];
                    for (i = 0; i < l.numsurfpt; i++, s++)
                    {
                        math::add(s->light, state.face_patches[(size_t)facenum]->baselight, s->light);
                    }
                }
            }
        }
        // samples
        {
            facelight *fl = &state.facelights[(size_t)facenum];
            vec_t maxlights[allstyles];
            for (j = 0; j < allstyles && f_styles[j] != 255; j++)
            {
                maxlights[j] = 0;
                for (i = 0; i < fl->numsamples; i++)
                {
                    vec_t b = vector_maximum(fl_samples[j][i].light);
                    maxlights[j] = maxlights[j] > b ? maxlights[j] : b;
                }
                if (maxlights[j] <= state.corings[f_styles[j]] * 0.1) // light is too dim; discard this style to reduce ram usage
                {
                    if (maxlights[j] > state.maxdiscardedlight + math::normal_epsilon)
                    {
                        std::lock_guard<std::mutex> guard(state.lock);
                        if (maxlights[j] > state.maxdiscardedlight + math::normal_epsilon)
                        {
                            state.maxdiscardedlight = maxlights[j];
                            math::copy(state.face_centroids[(size_t)facenum], state.maxdiscardedpos);
                        }
                    }
                    maxlights[j] = 0;
                }
            }
            for (k = 0; k < maxlightmaps; k++)
            {
                int bestindex = -1;
                if (k == 0)
                {
                    bestindex = 0;
                }
                else
                {
                    vec_t bestmaxlight = 0;
                    for (j = 1; j < allstyles && f_styles[j] != 255; j++)
                    {
                        if (maxlights[j] > bestmaxlight + math::normal_epsilon)
                        {
                            bestmaxlight = maxlights[j];
                            bestindex = j;
                        }
                    }
                }
                if (bestindex != -1)
                {
                    maxlights[bestindex] = 0;
                    f->styles[k] = f_styles[bestindex];
                    fl->samples[k] = (sample *)malloc((size_t)fl->numsamples * sizeof(sample));
                    err::require(fl->samples[k] != nullptr, "build_facelights: out of memory");
                    memcpy(fl->samples[k], fl_samples[bestindex], (size_t)fl->numsamples * sizeof(sample));
                }
                else
                {
                    f->styles[k] = 255;
                    fl->samples[k] = nullptr;
                }
            }
            for (j = 1; j < allstyles && f_styles[j] != 255; j++)
            {
                if (maxlights[j] > state.maxdiscardedlight + math::normal_epsilon)
                {
                    std::lock_guard<std::mutex> guard(state.lock);
                    if (maxlights[j] > state.maxdiscardedlight + math::normal_epsilon)
                    {
                        state.maxdiscardedlight = maxlights[j];
                        math::copy(state.face_centroids[(size_t)facenum], state.maxdiscardedpos);
                    }
                }
            }
            // -dumpgather: serialize the complete per style gather results
            // before the style slots are chosen; this is the seam the gpu
            // backend replaces, so two dumps bisect a divergence to a sample
            if (!state.gather_dump.empty())
            {
                std::vector<byte> &blob = state.gather_dump[(size_t)facenum];
                blob.clear(); // the face may run twice (-gpu collect + consume)
                unsigned int numstyles = 0;
                for (j = 0; j < allstyles && f_styles[j] != 255; j++)
                    numstyles++;
                unsigned int numsamples = (unsigned int)fl->numsamples;
                blob.reserve(8 + numstyles * (4 + (size_t)numsamples * 12));
                auto put32 = [&blob](unsigned int v)
                {
                    blob.push_back((byte)(v & 0xff));
                    blob.push_back((byte)((v >> 8) & 0xff));
                    blob.push_back((byte)((v >> 16) & 0xff));
                    blob.push_back((byte)((v >> 24) & 0xff));
                };
                put32(numsamples);
                put32(numstyles);
                for (j = 0; j < (int)numstyles; j++)
                {
                    put32(f_styles[j]);
                    for (i = 0; i < (int)numsamples; i++)
                    {
                        float v[3] = {(float)fl_samples[j][i].light[0],
                                      (float)fl_samples[j][i].light[1],
                                      (float)fl_samples[j][i].light[2]};
                        const byte *raw = (const byte *)v;
                        blob.insert(blob.end(), raw, raw + sizeof(v));
                    }
                }
            }
            for (j = 0; j < allstyles; j++)
            {
                free(fl_samples[j]);
            }
        }
        // patches
        for (pt = state.face_patches[(size_t)facenum]; pt; pt = pt->next)
        {
            vec_t maxlights[allstyles];
            for (j = 0; j < allstyles && pt->totalstyle_all[j] != 255; j++)
            {
                maxlights[j] = vector_maximum(pt->totallight_all[j]);
            }
            for (k = 0; k < maxlightmaps; k++)
            {
                int bestindex = -1;
                if (k == 0)
                {
                    bestindex = 0;
                }
                else
                {
                    vec_t bestmaxlight = 0;
                    for (j = 1; j < allstyles && pt->totalstyle_all[j] != 255; j++)
                    {
                        if (maxlights[j] > bestmaxlight + math::normal_epsilon)
                        {
                            bestmaxlight = maxlights[j];
                            bestindex = j;
                        }
                    }
                }
                if (bestindex != -1)
                {
                    maxlights[bestindex] = 0;
                    pt->totalstyle[k] = pt->totalstyle_all[bestindex];
                    math::copy(pt->totallight_all[bestindex], pt->totallight[k]);
                }
                else
                {
                    pt->totalstyle[k] = 255;
                }
            }
            for (j = 1; j < allstyles && pt->totalstyle_all[j] != 255; j++)
            {
                if (maxlights[j] > state.maxdiscardedlight + math::normal_epsilon)
                {
                    std::lock_guard<std::mutex> guard(state.lock);
                    if (maxlights[j] > state.maxdiscardedlight + math::normal_epsilon)
                    {
                        state.maxdiscardedlight = maxlights[j];
                        math::copy(pt->origin, state.maxdiscardedpos);
                    }
                }
            }
            for (j = 0; j < allstyles && pt->totalstyle_all[j] != 255; j++)
            {
                maxlights[j] = vector_maximum(pt->directlight_all[j]);
            }
            for (k = 0; k < maxlightmaps; k++)
            {
                int bestindex = -1;
                if (k == 0)
                {
                    bestindex = 0;
                }
                else
                {
                    vec_t bestmaxlight = 0;
                    for (j = 1; j < allstyles && pt->totalstyle_all[j] != 255; j++)
                    {
                        if (maxlights[j] > bestmaxlight + math::normal_epsilon)
                        {
                            bestmaxlight = maxlights[j];
                            bestindex = j;
                        }
                    }
                }
                if (bestindex != -1)
                {
                    maxlights[bestindex] = 0;
                    pt->directstyle[k] = pt->totalstyle_all[bestindex];
                    math::copy(pt->directlight_all[bestindex], pt->directlight[k]);
                }
                else
                {
                    pt->directstyle[k] = 255;
                }
            }
            for (j = 1; j < allstyles && pt->totalstyle_all[j] != 255; j++)
            {
                if (maxlights[j] > state.maxdiscardedlight + math::normal_epsilon)
                {
                    std::lock_guard<std::mutex> guard(state.lock);
                    if (maxlights[j] > state.maxdiscardedlight + math::normal_epsilon)
                    {
                        state.maxdiscardedlight = maxlights[j];
                        math::copy(pt->origin, state.maxdiscardedpos);
                    }
                }
            }
            free(pt->totalstyle_all);
            pt->totalstyle_all = nullptr;
            free(pt->samplelight_all);
            pt->samplelight_all = nullptr;
            free(pt->totallight_all);
            pt->totallight_all = nullptr;
            free(pt->directlight_all);
            pt->directlight_all = nullptr;
        }
        free(l.lmcache);
        free(l.lmcache_normal);
        free(l.lmcache_wallflags);
        free(l.surfpt_position);
        free(l.surfpt_surface);
    }

    // ===== total light lookup =====

    // the totallight entry of the patch for the style, or a zero vector
    const vec3v *get_total_light(const patch *pt, int style)
    {
        static const vec3v totallight_default{0, 0, 0};
        int i;
        for (i = 0; i < maxlightmaps && pt->totalstyle[i] != 255; i++)
        {
            if (pt->totalstyle[i] == style)
            {
                return &(pt->totallight[i]);
            }
        }
        return &totallight_default;
    }
}
