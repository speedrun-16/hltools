#include <cmath>
#include <cstring>

#include "../common/log.h"
#include "internal.h"

// light occlusion tracing: the world bsp flattened into tnodes, walked by the
// iterative test_line (the single hottest function of the stage), plus the
// per model face trees that make opaque entities block light test_line uses
// an explicit segment stack to avoid recursion in the hottest tracing path

namespace rad
{
    // ===== world tnodes =====

    namespace
    {
        // converts the disk node structure into the efficient tracing structure
        void make_tnode(rad_state &state, int &tnode_next, const int nodenum)
        {
            tnode *t = &state.tnodes[(size_t)tnode_next++];

            const format::dnode_t *node = &state.map->nodes[(size_t)nodenum];
            const plane *pl = &state.planes[(size_t)node->planenum];

            t->type = pl->type;
            math::copy(pl->normal, t->normal);
            if (pl->normal[pl->type % 3] < 0)
            {
                if (pl->type < 3)
                    logging::warn("MakeTnode: negative plane");
            }
            t->dist = pl->dist;

            for (int i = 0; i < 2; i++)
            {
                if (node->children[i] < 0)
                    t->children[i] = state.map->leafs[(size_t)(-node->children[i] - 1)].contents;
                else
                {
                    t->children[i] = tnode_next;
                    make_tnode(state, tnode_next, node->children[i]);
                }
            }
        }
    }

    // loads the node structure out of the bsp to be used for light occlusion
    void make_tnodes(rad_state &state)
    {
        state.tnodes.resize(state.map->nodes.size() + 1);
        int tnode_next = 0;
        make_tnode(state, tnode_next, 0);
    }

    // ===== test line =====

    namespace
    {
        int test_line_r(const rad_state &state, const int node, const vec3v &start, const vec3v &stop,
                        int &linecontent, vec_t *skyhit)
        {
            const tnode *tn;
            float front, back;
            vec3v mid;
            float frac;
            int side;
            int r;

            if (node < 0)
            {
                if (node == linecontent)
                    return contents_empty;
                if (node == contents_solid)
                {
                    return contents_solid;
                }
                if (node == contents_sky)
                {
                    if (skyhit)
                    {
                        skyhit[0] = start[0];
                        skyhit[1] = start[1];
                        skyhit[2] = start[2];
                    }
                    return contents_sky;
                }
                if (linecontent)
                {
                    return contents_solid;
                }
                linecontent = node;
                return contents_empty;
            }

            tn = &state.tnodes[(size_t)node];
            switch (tn->type)
            {
            case plane_x:
                front = start[0] - tn->dist;
                back = stop[0] - tn->dist;
                break;
            case plane_y:
                front = start[1] - tn->dist;
                back = stop[1] - tn->dist;
                break;
            case plane_z:
                front = start[2] - tn->dist;
                back = stop[2] - tn->dist;
                break;
            default:
                front = (start[0] * tn->normal[0] + start[1] * tn->normal[1] + start[2] * tn->normal[2]) - tn->dist;
                back = (stop[0] * tn->normal[0] + stop[1] * tn->normal[1] + stop[2] * tn->normal[2]) - tn->dist;
                break;
            }

            if (front > math::on_epsilon / 2 && back > math::on_epsilon / 2)
            {
                return test_line_r(state, tn->children[0], start, stop, linecontent, skyhit);
            }
            if (front < -math::on_epsilon / 2 && back < -math::on_epsilon / 2)
            {
                return test_line_r(state, tn->children[1], start, stop, linecontent, skyhit);
            }
            if (std::fabs(front) <= math::on_epsilon && std::fabs(back) <= math::on_epsilon)
            {
                int r1 = test_line_r(state, tn->children[0], start, stop, linecontent, skyhit);
                if (r1 == contents_solid)
                    return contents_solid;
                int r2 = test_line_r(state, tn->children[1], start, stop, linecontent, skyhit);
                if (r2 == contents_solid)
                    return contents_solid;
                if (r1 == contents_sky || r2 == contents_sky)
                    return contents_sky;
                return contents_empty;
            }
            side = (front - back) < 0;
            frac = front / (front - back);
            if (frac < 0)
                frac = 0;
            if (frac > 1)
                frac = 1;
            mid[0] = start[0] + (stop[0] - start[0]) * frac;
            mid[1] = start[1] + (stop[1] - start[1]) * frac;
            mid[2] = start[2] + (stop[2] - start[2]) * frac;
            r = test_line_r(state, tn->children[side], start, mid, linecontent, skyhit);
            if (r != contents_empty)
                return r;
            return test_line_r(state, tn->children[!side], mid, stop, linecontent, skyhit);
        }

        constexpr int testline_stack_size = 128;

        struct testline_span
        {
            int node;
            vec3v start;
            vec3v stop;
        };
    }

    // iterative version of test_line_r this is the single hottest function of
    // hlrad (it runs once per sample+visible light pair, billions of times on a
    // normal map), so the recursion is flattened into an explicit segment
    // stack behavior (traversal order, epsilon logic, linecontent state) is
    // identical to test_line_r; the rare "segment lies in the node plane" case
    // and stack overflow fall back to the recursive original to keep the exact
    // same semantics
    int test_line(const rad_state &state, const vec3v &start, const vec3v &stop, vec_t *skyhit)
    {
        int linecontent = 0;
        testline_span stack[testline_stack_size];
        testline_span *top = stack; // first free slot
        int node = 0;
        vec3v curstart, curstop;
        math::copy(start, curstart);
        math::copy(stop, curstop);

        while (1)
        {
            while (node >= 0)
            {
                const tnode *tn = &state.tnodes[(size_t)node];
                float front, back;
                switch (tn->type)
                {
                case plane_x:
                    front = curstart[0] - tn->dist;
                    back = curstop[0] - tn->dist;
                    break;
                case plane_y:
                    front = curstart[1] - tn->dist;
                    back = curstop[1] - tn->dist;
                    break;
                case plane_z:
                    front = curstart[2] - tn->dist;
                    back = curstop[2] - tn->dist;
                    break;
                default:
                    front = (curstart[0] * tn->normal[0] + curstart[1] * tn->normal[1] + curstart[2] * tn->normal[2]) - tn->dist;
                    back = (curstop[0] * tn->normal[0] + curstop[1] * tn->normal[1] + curstop[2] * tn->normal[2]) - tn->dist;
                    break;
                }

                if (front > math::on_epsilon / 2 && back > math::on_epsilon / 2)
                {
                    node = tn->children[0];
                    continue;
                }
                if (front < -math::on_epsilon / 2 && back < -math::on_epsilon / 2)
                {
                    node = tn->children[1];
                    continue;
                }
                if (std::fabs(front) <= math::on_epsilon && std::fabs(back) <= math::on_epsilon)
                {
                    // rare: segment lies (almost) in the node plane; the
                    // recursive version visits both children with the full
                    // segment and solid beats sky here
                    int r1 = test_line_r(state, tn->children[0], curstart, curstop, linecontent, skyhit);
                    if (r1 == contents_solid)
                        return contents_solid;
                    int r2 = test_line_r(state, tn->children[1], curstart, curstop, linecontent, skyhit);
                    if (r2 == contents_solid)
                        return contents_solid;
                    if (r1 == contents_sky || r2 == contents_sky)
                        return contents_sky;
                    node = -0x7FFFFFFF; // this subtree is done; fetch the next span from the stack
                    break;
                }
                {
                    int side = (front - back) < 0;
                    float frac = front / (front - back);
                    if (frac < 0)
                        frac = 0;
                    if (frac > 1)
                        frac = 1;
                    vec3v mid;
                    mid[0] = curstart[0] + (curstop[0] - curstart[0]) * frac;
                    mid[1] = curstart[1] + (curstop[1] - curstart[1]) * frac;
                    mid[2] = curstart[2] + (curstop[2] - curstart[2]) * frac;
                    if (top == stack + testline_stack_size)
                    {
                        // out of stack space (pathologically deep tree): finish recursively
                        int r = test_line_r(state, tn->children[side], curstart, mid, linecontent, skyhit);
                        if (r != contents_empty)
                            return r;
                        node = tn->children[!side];
                        math::copy(mid, curstart);
                        continue;
                    }
                    top->node = tn->children[!side];
                    math::copy(mid, top->start);
                    math::copy(curstop, top->stop);
                    top++;
                    node = tn->children[side];
                    math::copy(mid, curstop);
                }
            }

            if (node != -0x7FFFFFFF) // leaf reached (the sentinel means the coplanar case already handled this subtree)
            {
                if (node != linecontent)
                {
                    if (node == contents_solid)
                    {
                        return contents_solid;
                    }
                    if (node == contents_sky)
                    {
                        if (skyhit)
                        {
                            skyhit[0] = curstart[0];
                            skyhit[1] = curstart[1];
                            skyhit[2] = curstart[2];
                        }
                        return contents_sky;
                    }
                    if (linecontent)
                    {
                        return contents_solid;
                    }
                    linecontent = node;
                }
            }

            if (top == stack)
            {
                return contents_empty;
            }
            top--;
            node = top->node;
            math::copy(top->start, curstart);
            math::copy(top->stop, curstop);
        }
    }

    // ===== opaque entity nodes =====

    namespace
    {
        // merges f2 into f when they share an edge on the same plane, keeping
        // the merged winding convex
        bool try_merge(opaque_face *f, const opaque_face *f2)
        {
            if (!f->winding || !f2->winding)
            {
                return false;
            }
            if (std::fabs(f2->pl.dist - f->pl.dist) > math::on_epsilon
                || std::fabs(f2->pl.normal[0] - f->pl.normal[0]) > math::normal_epsilon
                || std::fabs(f2->pl.normal[1] - f->pl.normal[1]) > math::normal_epsilon
                || std::fabs(f2->pl.normal[2] - f->pl.normal[2]) > math::normal_epsilon)
            {
                return false;
            }
            if ((f->tex_alphatest || f2->tex_alphatest) && f->texinfo != f2->texinfo)
            {
                return false;
            }

            math::winding *w = f->winding;
            const math::winding *w2 = f2->winding;
            const vec3v *pA = nullptr, *pB = nullptr, *pC = nullptr, *pD = nullptr;
            const vec3v *p2A = nullptr, *p2B = nullptr, *p2C = nullptr, *p2D = nullptr;
            int i, i2;

            for (i = 0; i < w->size(); i++)
            {
                for (i2 = 0; i2 < w2->size(); i2++)
                {
                    pA = &(*w)[(i + w->size() - 1) % w->size()];
                    pB = &(*w)[i];
                    pC = &(*w)[(i + 1) % w->size()];
                    pD = &(*w)[(i + 2) % w->size()];
                    p2A = &(*w2)[(i2 + w2->size() - 1) % w2->size()];
                    p2B = &(*w2)[i2];
                    p2C = &(*w2)[(i2 + 1) % w2->size()];
                    p2D = &(*w2)[(i2 + 2) % w2->size()];
                    if (!math::equal(*pB, *p2C) || !math::equal(*pC, *p2B))
                    {
                        continue;
                    }
                    break;
                }
                if (i2 == w2->size())
                {
                    continue;
                }
                break;
            }
            if (i == w->size())
            {
                return false;
            }

            const vec3v &normal = f->pl.normal;
            vec3v e1, e2;
            plane pl1, pl2;
            int side1, side2;

            math::subtract(*p2D, *pA, e1);
            math::cross(normal, e1, pl1.normal); // pointing outward
            if (math::normalize(pl1.normal) == 0.0)
            {
                return false;
            }
            pl1.dist = math::dot(*pA, pl1.normal);
            if (math::dot(*pB, pl1.normal) - pl1.dist < -math::on_epsilon)
            {
                return false;
            }
            side1 = (math::dot(*pB, pl1.normal) - pl1.dist > math::on_epsilon) ? 1 : 0;

            math::subtract(*pD, *p2A, e2);
            math::cross(normal, e2, pl2.normal); // pointing outward
            if (math::normalize(pl2.normal) == 0.0)
            {
                return false;
            }
            pl2.dist = math::dot(*p2A, pl2.normal);
            if (math::dot(*p2B, pl2.normal) - pl2.dist < -math::on_epsilon)
            {
                return false;
            }
            side2 = (math::dot(*p2B, pl2.normal) - pl2.dist > math::on_epsilon) ? 1 : 0;

            std::vector<vec3v> newpts((size_t)(w->size() + w2->size() - 4 + side1 + side2));
            int j, k;
            k = 0;
            for (j = (i + 2) % w->size(); j != i; j = (j + 1) % w->size())
            {
                math::copy((*w)[j], newpts[(size_t)k]);
                k++;
            }
            if (side1)
            {
                math::copy((*w)[j], newpts[(size_t)k]);
                k++;
            }
            for (j = (i2 + 2) % w2->size(); j != i2; j = (j + 1) % w2->size())
            {
                math::copy((*w2)[j], newpts[(size_t)k]);
                k++;
            }
            if (side2)
            {
                math::copy((*w2)[j], newpts[(size_t)k]);
                k++;
            }
            math::winding *neww = new math::winding{std::move(newpts)};
            neww->remove_colinear_points();
            if (neww->size() < 3)
            {
                delete neww;
                neww = nullptr;
            }
            delete f->winding;
            f->winding = neww;
            return true;
        }

        int merge_opaque_faces(rad_state &state, int firstface, int numfaces)
        {
            int i, j, newnum;
            opaque_face *faces = &state.opaquefaces[(size_t)firstface];
            for (i = 0; i < numfaces; i++)
            {
                for (j = 0; j < i; j++)
                {
                    if (try_merge(&faces[i], &faces[j]))
                    {
                        delete faces[j].winding;
                        faces[j].winding = nullptr;
                        j = -1;
                        continue;
                    }
                }
            }
            for (i = 0, j = 0; i < numfaces; i++)
            {
                if (faces[i].winding)
                {
                    if (j != i)
                        faces[j] = std::move(faces[i]);
                    j++;
                }
            }
            newnum = j;
            for (; j < numfaces; j++)
            {
                faces[j] = opaque_face{};
            }
            return newnum;
        }

        void build_face_edges(opaque_face *f)
        {
            if (!f->winding)
                return;
            f->numedges = f->winding->size();
            f->edges.assign((size_t)f->numedges, plane{});
            const vec3v &n = f->pl.normal;
            vec3v e;
            for (int x = 0; x < f->winding->size(); x++)
            {
                const vec3v &p1 = (*f->winding)[x];
                const vec3v &p2 = (*f->winding)[(x + 1) % f->winding->size()];
                plane *pl = &f->edges[(size_t)x];
                math::subtract(p2, p1, e);
                math::cross(n, e, pl->normal);
                if (math::normalize(pl->normal) == 0.0)
                {
                    math::clear(pl->normal);
                    pl->dist = -1;
                    continue;
                }
                pl->dist = math::dot(pl->normal, p1);
            }
        }
    }

    void create_opaque_nodes(rad_state &state)
    {
        const format::map_data &map = *state.map;
        state.opaquemodels.assign(map.models.size(), opaque_model{});
        state.opaquenodes.assign(map.nodes.size(), opaque_node{});
        state.opaquefaces.assign(map.faces.size(), opaque_face{});
        for (size_t i = 0; i < map.faces.size(); i++)
        {
            opaque_face *of = &state.opaquefaces[i];
            const format::dface_t *df = &map.faces[i];
            of->winding = new math::winding{winding_from_face(state, *df)};
            if (of->winding->size() < 3)
            {
                delete of->winding;
                of->winding = nullptr;
            }
            of->pl = state.planes[(size_t)df->planenum];
            if (df->side)
            {
                of->pl.normal = -of->pl.normal;
                of->pl.dist = -of->pl.dist;
            }
            of->texinfo = df->texinfo;
            const format::texinfo_t *info = &map.texinfo[(size_t)of->texinfo];
            for (int j = 0; j < 2; j++)
            {
                for (int k = 0; k < 4; k++)
                {
                    of->tex_vecs[j][k] = info->vecs[j][k];
                }
            }
            const rad_texture *tex = &state.textures[(size_t)info->miptex];
            of->tex_alphatest = tex->name[0] == '{';
            of->tex_width = tex->width;
            of->tex_height = tex->height;
            of->tex_canvas = tex->canvas.data();
        }
        for (size_t i = 0; i < map.nodes.size(); i++)
        {
            opaque_node *on = &state.opaquenodes[i];
            const format::dnode_t *dn = &map.nodes[i];
            on->type = state.planes[(size_t)dn->planenum].type;
            math::copy(state.planes[(size_t)dn->planenum].normal, on->normal);
            on->dist = state.planes[(size_t)dn->planenum].dist;
            on->children[0] = dn->children[0];
            on->children[1] = dn->children[1];
            on->firstface = dn->firstface;
            on->numfaces = dn->numfaces;
            on->numfaces = merge_opaque_faces(state, on->firstface, on->numfaces);
        }
        for (size_t i = 0; i < map.faces.size(); i++)
        {
            build_face_edges(&state.opaquefaces[i]);
        }
        for (size_t i = 0; i < map.models.size(); i++)
        {
            opaque_model *om = &state.opaquemodels[i];
            const format::dmodel_t *dm = &map.models[i];
            om->headnode = dm->headnode[0];
            for (int j = 0; j < 3; j++)
            {
                om->mins[j] = dm->mins[j] - 1;
                om->maxs[j] = dm->maxs[j] + 1;
            }
        }
    }

    void delete_opaque_nodes(rad_state &state)
    {
        for (size_t i = 0; i < state.opaquefaces.size(); i++)
        {
            opaque_face *of = &state.opaquefaces[i];
            if (of->winding)
                delete of->winding;
        }
        state.opaquefaces.clear();
        state.opaquenodes.clear();
        state.opaquemodels.clear();
    }

    namespace
    {
        int test_line_opaque_face(const rad_state &state, int facenum, const vec3v &hit)
        {
            const opaque_face *thisface = &state.opaquefaces[(size_t)facenum];
            int x;
            if (thisface->numedges == 0)
            {
                return 0;
            }
            for (x = 0; x < thisface->numedges; x++)
            {
                if (math::dot(hit, thisface->edges[(size_t)x].normal) - thisface->edges[(size_t)x].dist > math::on_epsilon)
                {
                    return 0;
                }
            }
            if (thisface->tex_alphatest)
            {
                double x2, y;
                x2 = math::dot(hit, thisface->tex_vecs[0]) + thisface->tex_vecs[0][3];
                y = math::dot(hit, thisface->tex_vecs[1]) + thisface->tex_vecs[1][3];
                x2 = std::floor(x2 - thisface->tex_width * std::floor(x2 / thisface->tex_width));
                y = std::floor(y - thisface->tex_height * std::floor(y / thisface->tex_height));
                x2 = x2 > thisface->tex_width - 1 ? thisface->tex_width - 1 : x2 < 0 ? 0 : x2;
                y = y > thisface->tex_height - 1 ? thisface->tex_height - 1 : y < 0 ? 0 : y;
                if (thisface->tex_canvas[(int)y * thisface->tex_width + (int)x2] == 0xFF)
                {
                    return 0;
                }
            }
            return 1;
        }

        int test_line_opaque_r(const rad_state &state, int nodenum, const vec3v &start, const vec3v &stop)
        {
            const opaque_node *thisnode;
            vec_t front, back;
            if (nodenum < 0)
            {
                return 0;
            }
            thisnode = &state.opaquenodes[(size_t)nodenum];
            switch (thisnode->type)
            {
            case plane_x:
                front = start[0] - thisnode->dist;
                back = stop[0] - thisnode->dist;
                break;
            case plane_y:
                front = start[1] - thisnode->dist;
                back = stop[1] - thisnode->dist;
                break;
            case plane_z:
                front = start[2] - thisnode->dist;
                back = stop[2] - thisnode->dist;
                break;
            default:
                front = math::dot(start, thisnode->normal) - thisnode->dist;
                back = math::dot(stop, thisnode->normal) - thisnode->dist;
            }
            if (front > math::on_epsilon / 2 && back > math::on_epsilon / 2)
            {
                return test_line_opaque_r(state, thisnode->children[0], start, stop);
            }
            if (front < -math::on_epsilon / 2 && back < -math::on_epsilon / 2)
            {
                return test_line_opaque_r(state, thisnode->children[1], start, stop);
            }
            if (std::fabs(front) <= math::on_epsilon && std::fabs(back) <= math::on_epsilon)
            {
                return test_line_opaque_r(state, thisnode->children[0], start, stop)
                    || test_line_opaque_r(state, thisnode->children[1], start, stop);
            }
            {
                int side;
                vec_t frac;
                vec3v mid;
                int facenum;
                side = (front - back) < 0;
                frac = front / (front - back);
                if (frac < 0)
                    frac = 0;
                if (frac > 1)
                    frac = 1;
                mid[0] = start[0] + (stop[0] - start[0]) * frac;
                mid[1] = start[1] + (stop[1] - start[1]) * frac;
                mid[2] = start[2] + (stop[2] - start[2]) * frac;
                for (facenum = thisnode->firstface; facenum < thisnode->firstface + thisnode->numfaces; facenum++)
                {
                    if (test_line_opaque_face(state, facenum, mid))
                    {
                        return 1;
                    }
                }
                return test_line_opaque_r(state, thisnode->children[side], start, mid)
                    || test_line_opaque_r(state, thisnode->children[!side], mid, stop);
            }
        }
    }

    int test_line_opaque(const rad_state &state, int modelnum, const vec3v &modelorigin,
                         const vec3v &start, const vec3v &stop)
    {
        const opaque_model *thismodel = &state.opaquemodels[(size_t)modelnum];
        vec_t front, back, frac;
        vec3v p1, p2;
        math::subtract(start, modelorigin, p1);
        math::subtract(stop, modelorigin, p2);
        int axial;
        for (axial = 0; axial < 3; axial++)
        {
            front = p1[axial] - thismodel->maxs[axial];
            back = p2[axial] - thismodel->maxs[axial];
            if (front >= -math::on_epsilon && back >= -math::on_epsilon)
            {
                return 0;
            }
            if (front > math::on_epsilon || back > math::on_epsilon)
            {
                frac = front / (front - back);
                if (front > back)
                {
                    p1[0] = p1[0] + (p2[0] - p1[0]) * frac;
                    p1[1] = p1[1] + (p2[1] - p1[1]) * frac;
                    p1[2] = p1[2] + (p2[2] - p1[2]) * frac;
                }
                else
                {
                    p2[0] = p1[0] + (p2[0] - p1[0]) * frac;
                    p2[1] = p1[1] + (p2[1] - p1[1]) * frac;
                    p2[2] = p1[2] + (p2[2] - p1[2]) * frac;
                }
            }
            front = thismodel->mins[axial] - p1[axial];
            back = thismodel->mins[axial] - p2[axial];
            if (front >= -math::on_epsilon && back >= -math::on_epsilon)
            {
                return 0;
            }
            if (front > math::on_epsilon || back > math::on_epsilon)
            {
                frac = front / (front - back);
                if (front > back)
                {
                    p1[0] = p1[0] + (p2[0] - p1[0]) * frac;
                    p1[1] = p1[1] + (p2[1] - p1[1]) * frac;
                    p1[2] = p1[2] + (p2[2] - p1[2]) * frac;
                }
                else
                {
                    p2[0] = p1[0] + (p2[0] - p1[0]) * frac;
                    p2[1] = p1[1] + (p2[1] - p1[1]) * frac;
                    p2[2] = p1[2] + (p2[2] - p1[2]) * frac;
                }
            }
        }
        return test_line_opaque_r(state, thismodel->headnode, p1, p2);
    }

    namespace
    {
        int count_opaque_faces_r(const rad_state &state, const opaque_node *node)
        {
            int count = node->numfaces;
            if (node->children[0] >= 0)
            {
                count += count_opaque_faces_r(state, &state.opaquenodes[(size_t)node->children[0]]);
            }
            if (node->children[1] >= 0)
            {
                count += count_opaque_faces_r(state, &state.opaquenodes[(size_t)node->children[1]]);
            }
            return count;
        }

        int test_point_opaque_r(const rad_state &state, int nodenum, bool solid, const vec3v &point)
        {
            const opaque_node *thisnode;
            vec_t dist;
            while (1)
            {
                if (nodenum < 0)
                {
                    if (solid && state.map->leafs[(size_t)(-nodenum - 1)].contents == contents_solid)
                        return 1;
                    else
                        return 0;
                }
                thisnode = &state.opaquenodes[(size_t)nodenum];
                switch (thisnode->type)
                {
                case plane_x:
                    dist = point[0] - thisnode->dist;
                    break;
                case plane_y:
                    dist = point[1] - thisnode->dist;
                    break;
                case plane_z:
                    dist = point[2] - thisnode->dist;
                    break;
                default:
                    dist = math::dot(point, thisnode->normal) - thisnode->dist;
                }
                if (dist > hunt_wall_epsilon)
                {
                    nodenum = thisnode->children[0];
                }
                else if (dist < -hunt_wall_epsilon)
                {
                    nodenum = thisnode->children[1];
                }
                else
                {
                    break;
                }
            }
            {
                int facenum;
                for (facenum = thisnode->firstface; facenum < thisnode->firstface + thisnode->numfaces; facenum++)
                {
                    if (test_line_opaque_face(state, facenum, point))
                    {
                        return 1;
                    }
                }
            }
            return test_point_opaque_r(state, thisnode->children[0], solid, point)
                || test_point_opaque_r(state, thisnode->children[1], solid, point);
        }
    }

    int count_opaque_faces(const rad_state &state, int modelnum)
    {
        return count_opaque_faces_r(state, &state.opaquenodes[(size_t)state.opaquemodels[(size_t)modelnum].headnode]);
    }

    int test_point_opaque(const rad_state &state, int modelnum, const vec3v &modelorigin,
                          bool solid, const vec3v &point)
    {
        const opaque_model *thismodel = &state.opaquemodels[(size_t)modelnum];
        vec3v newpoint;
        math::subtract(point, modelorigin, newpoint);
        int axial;
        for (axial = 0; axial < 3; axial++)
        {
            if (newpoint[axial] > thismodel->maxs[axial])
                return 0;
            if (newpoint[axial] < thismodel->mins[axial])
                return 0;
        }
        return test_point_opaque_r(state, thismodel->headnode, solid, newpoint);
    }
}
