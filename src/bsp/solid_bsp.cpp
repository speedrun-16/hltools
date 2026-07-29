#include "internal.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "../common/error.h"
#include "../common/log.h"
#include "../common/progress.h"
#include "../common/string_util.h"

// the tree builder: picks split planes by the reference's entropy heuristic,
// divides surfaces, brushes and portals down the tree, and turns the remains
// into leafs every accumulation and comparison keeps the reference order

namespace bsp
{
    namespace
    {
        void update_status(bsp_state &state)
        {
            if (!state.report_progress)
                return;
            // counts nodes for the per hull summary line; no incremental output
            ++state.num_processed;
        }

        // bsp heuristic: which side of the split a face leans toward, counting
        // near plane geometry into epsilonsplit
        int face_side(face *in, const plane *split, double *epsilonsplit = nullptr)
        {
            const vec_t epsilonmin = 0.002, epsilonmax = 0.2;
            vec_t d_front = 0, d_back = 0;
            vec_t dot;

            if (split->type <= last_axial)
            {
                // axial planes are fast
                for (int i = 0; i < in->numpoints; i++)
                {
                    dot = in->pts[i][split->type] - split->dist;
                    if (dot > d_front)
                        d_front = dot;
                    if (dot < d_back)
                        d_back = dot;
                }
            }
            else
            {
                // sloping planes take longer
                for (int i = 0; i < in->numpoints; i++)
                {
                    dot = math::dot(in->pts[i], split->normal);
                    dot -= split->dist;
                    if (dot > d_front)
                        d_front = dot;
                    if (dot < d_back)
                        d_back = dot;
                }
            }
            if (d_front <= math::on_epsilon)
            {
                if (d_front > epsilonmin || d_back > -epsilonmax)
                {
                    if (epsilonsplit)
                        (*epsilonsplit)++;
                }
                return math::winding::side_back;
            }
            if (d_back >= -math::on_epsilon)
            {
                if (d_back < -epsilonmin || d_front < epsilonmax)
                {
                    if (epsilonsplit)
                        (*epsilonsplit)++;
                }
                return math::winding::side_front;
            }
            if (d_front < epsilonmax || d_back > -epsilonmax)
            {
                if (epsilonsplit)
                    (*epsilonsplit)++;
            }
            return math::winding::side_on;
        }

        // organizes all surfaces into a bounds tree to accelerate the split
        // tests; can cut more than 90% of the compile time on complicated maps
        struct surface_tree_node
        {
            int size = 0; // zero invalidates mins and maxs
            int size_discardable = 0;
            math::vec3v mins;
            math::vec3v maxs;
            bool isleaf = false;
            // node
            surface_tree_node *children[2] = {};
            std::vector<face *> *nodefaces = nullptr;
            int nodefaces_discardablesize = 0;
            // leaf
            std::vector<face *> *leaffaces = nullptr;
        };

        struct surface_tree
        {
            bool dontbuild = false;
            vec_t epsilon = 0; // faces closer than this to the plane land in middle
            surface_tree_node *headnode = nullptr;
            struct
            {
                int frontsize = 0;
                int backsize = 0;
                std::vector<face *> *middle = nullptr; // coplanar and discardable faces too
            } result;
        };

        void build_surface_tree_r(surface_tree *tree, surface_tree_node *node)
        {
            node->size = (int)node->leaffaces->size();
            node->size_discardable = 0;
            if (node->size == 0)
            {
                node->isleaf = true;
                return;
            }

            node->mins.x = node->mins.y = node->mins.z = (vec_t)bogus_range;
            node->maxs.x = node->maxs.y = node->maxs.z = (vec_t)-bogus_range;
            for (face *f : *node->leaffaces)
            {
                for (int x = 0; x < f->numpoints; x++)
                {
                    for (int k = 0; k < 3; k++)
                    {
                        if (f->pts[x][k] < node->mins[k])
                            node->mins[k] = f->pts[x][k];
                        if (f->pts[x][k] > node->maxs[k])
                            node->maxs[k] = f->pts[x][k];
                    }
                }
                if (f->style == face_style::discardable)
                    node->size_discardable++;
            }

            int bestaxis = -1;
            {
                vec_t bestdelta = 0;
                for (int k = 0; k < 3; k++)
                {
                    if (node->maxs[k] - node->mins[k] > bestdelta + math::on_epsilon)
                    {
                        bestaxis = k;
                        bestdelta = node->maxs[k] - node->mins[k];
                    }
                }
            }
            if (node->size <= 5 || tree->dontbuild || bestaxis == -1)
            {
                node->isleaf = true;
                return;
            }

            node->isleaf = false;
            vec_t dist = (node->mins[bestaxis] + node->maxs[bestaxis]) / 2;
            vec_t dist1 = (3 * node->mins[bestaxis] + node->maxs[bestaxis]) / 4;
            vec_t dist2 = (node->mins[bestaxis] + 3 * node->maxs[bestaxis]) / 4;
            // each child is at most 3/4 the parent's size; faces left in the
            // parent are those comparable in size to the parent itself
            node->nodefaces = new std::vector<face *>;
            node->nodefaces_discardablesize = 0;
            node->children[0] = new surface_tree_node();
            node->children[0]->leaffaces = new std::vector<face *>;
            node->children[1] = new surface_tree_node();
            node->children[1]->leaffaces = new std::vector<face *>;
            for (face *f : *node->leaffaces)
            {
                vec_t low = (vec_t)bogus_range;
                vec_t high = (vec_t)-bogus_range;
                for (int x = 0; x < f->numpoints; x++)
                {
                    if (f->pts[x][bestaxis] < low)
                        low = f->pts[x][bestaxis];
                    if (f->pts[x][bestaxis] > high)
                        high = f->pts[x][bestaxis];
                }
                if (low < dist1 + math::on_epsilon && high > dist2 - math::on_epsilon)
                {
                    node->nodefaces->push_back(f);
                    if (f->style == face_style::discardable)
                        node->nodefaces_discardablesize++;
                }
                else if (low >= dist1 && high <= dist2)
                {
                    if ((low + high) / 2 > dist)
                        node->children[0]->leaffaces->push_back(f);
                    else
                        node->children[1]->leaffaces->push_back(f);
                }
                else if (low >= dist1)
                {
                    node->children[0]->leaffaces->push_back(f);
                }
                else if (high <= dist2)
                {
                    node->children[1]->leaffaces->push_back(f);
                }
            }
            if (node->children[0]->leaffaces->size() == node->leaffaces->size()
                || node->children[1]->leaffaces->size() == node->leaffaces->size())
            {
                logging::warn("build_surface_tree_r: didn't split node with bound (%f,%f,%f)-(%f,%f,%f)",
                              (double)node->mins[0], (double)node->mins[1], (double)node->mins[2],
                              (double)node->maxs[0], (double)node->maxs[1], (double)node->maxs[2]);
                delete node->children[0]->leaffaces;
                delete node->children[1]->leaffaces;
                delete node->children[0];
                delete node->children[1];
                delete node->nodefaces;
                node->isleaf = true;
                return;
            }
            delete node->leaffaces;
            node->leaffaces = nullptr;
            build_surface_tree_r(tree, node->children[0]);
            build_surface_tree_r(tree, node->children[1]);
        }

        surface_tree *build_surface_tree(surface *surfaces, vec_t epsilon)
        {
            surface_tree *tree = new surface_tree();
            tree->epsilon = epsilon;
            tree->result.middle = new std::vector<face *>;
            tree->headnode = new surface_tree_node();
            tree->headnode->leaffaces = new std::vector<face *>;
            for (surface *p2 = surfaces; p2; p2 = p2->next)
            {
                if (p2->onnode)
                    continue;
                for (face *f = p2->faces; f; f = f->next)
                    tree->headnode->leaffaces->push_back(f);
            }
            tree->dontbuild = tree->headnode->leaffaces->size() < 20;
            build_surface_tree_r(tree, tree->headnode);
            if (tree->dontbuild)
            {
                *tree->result.middle = *tree->headnode->leaffaces;
                tree->result.backsize = 0;
                tree->result.frontsize = 0;
            }
            return tree;
        }

        void test_surface_tree_r(surface_tree *tree, const surface_tree_node *node,
                                 const plane *split)
        {
            if (node->size == 0)
                return;
            vec_t low, high;
            low = high = -split->dist;
            for (int k = 0; k < 3; k++)
            {
                if (split->normal[k] >= 0)
                {
                    high += split->normal[k] * node->maxs[k];
                    low += split->normal[k] * node->mins[k];
                }
                else
                {
                    high += split->normal[k] * node->mins[k];
                    low += split->normal[k] * node->maxs[k];
                }
            }
            if (low > tree->epsilon)
            {
                tree->result.frontsize += node->size;
                tree->result.frontsize -= node->size_discardable;
                return;
            }
            if (high < -tree->epsilon)
            {
                tree->result.backsize += node->size;
                tree->result.backsize -= node->size_discardable;
                return;
            }
            if (node->isleaf)
            {
                for (face *f : *node->leaffaces)
                    tree->result.middle->push_back(f);
            }
            else
            {
                for (face *f : *node->nodefaces)
                    tree->result.middle->push_back(f);
                test_surface_tree_r(tree, node->children[0], split);
                test_surface_tree_r(tree, node->children[1], split);
            }
        }

        void test_surface_tree(surface_tree *tree, const plane *split)
        {
            if (tree->dontbuild)
                return;
            tree->result.middle->clear();
            tree->result.backsize = 0;
            tree->result.frontsize = 0;
            test_surface_tree_r(tree, tree->headnode, split);
        }

        void delete_surface_tree_r(surface_tree_node *node)
        {
            if (node->isleaf)
            {
                delete node->leaffaces;
            }
            else
            {
                delete_surface_tree_r(node->children[0]);
                delete node->children[0];
                delete_surface_tree_r(node->children[1]);
                delete node->children[1];
                delete node->nodefaces;
            }
        }

        void delete_surface_tree(surface_tree *tree)
        {
            delete_surface_tree_r(tree->headnode);
            delete tree->headnode;
            delete tree->result.middle;
            delete tree;
        }

        // when there are a huge number of planes, choose an axial one close to
        // the middle smaller split metric values are better
        surface *choose_mid_plane_from_list(bsp_state &state, surface *surfaces,
                                            const math::vec3v &mins, const math::vec3v &maxs,
                                            int detaillevel)
        {
            surface_tree *surfacetree = build_surface_tree(surfaces, (vec_t)math::on_epsilon);

            vec_t bestvalue = 9e30;
            surface *bestsurface = nullptr;

            for (surface *p = surfaces; p; p = p->next)
            {
                if (p->onnode)
                    continue;
                if (p->detail_level != detaillevel)
                    continue;

                plane *split = &state.planes[(size_t)p->planenum];

                // check for axis aligned surfaces
                int l = split->type;
                if (l > last_axial)
                    continue;

                // calculate the split metric along axis l
                vec_t dist = split->dist * split->normal[l];
                if (maxs[l] - dist < math::on_epsilon || dist - mins[l] < math::on_epsilon)
                    continue;
                if (maxs[l] - dist < state.options.maxnode_size / 2.0 - math::on_epsilon
                    || dist - mins[l] < state.options.maxnode_size / 2.0 - math::on_epsilon)
                {
                    continue;
                }
                double crosscount = 0;
                double frontcount = 0;
                double backcount = 0;
                double coplanarcount = 0;

                test_surface_tree(surfacetree, split);
                frontcount += surfacetree->result.frontsize;
                backcount += surfacetree->result.backsize;
                for (face *f : *surfacetree->result.middle)
                {
                    if (f->style == face_style::discardable)
                        continue;
                    if (f->planenum == p->planenum || f->planenum == (p->planenum ^ 1))
                    {
                        coplanarcount++;
                        continue;
                    }
                    switch (face_side(f, split))
                    {
                    case math::winding::side_front:
                        frontcount++;
                        break;
                    case math::winding::side_back:
                        backcount++;
                        break;
                    case math::winding::side_on:
                        crosscount++;
                        break;
                    }
                }

                double frontsize = frontcount + 0.5 * coplanarcount + 0.5 * crosscount;
                double frontfrac = (maxs[l] - dist) / (maxs[l] - mins[l]);
                double backsize = backcount + 0.5 * coplanarcount + 0.5 * crosscount;
                double backfrac = (dist - mins[l]) / (maxs[l] - mins[l]);
                // the first part is how the split will increase the number of
                // faces, the second how it will increase the average tree depth
                vec_t value = crosscount
                    + 0.1 * (frontsize * (std::log(frontfrac) / std::log(2.0))
                             + backsize * (std::log(backfrac) / std::log(2.0)));

                if (value > bestvalue)
                    continue;

                bestvalue = value;
                bestsurface = p;
            }

            delete_surface_tree(surfacetree);
            return bestsurface;
        }

        // choose the plane that splits the least faces
        surface *choose_plane_from_list(bsp_state &state, surface *surfaces,
                                        const math::vec3v &mins, const math::vec3v &maxs,
                                        int detaillevel)
        {
            // mins and maxs are invalid when detaillevel > 0
            (void)mins;
            (void)maxs;

            double planecount = 0;
            double totalsplit = 0;
            std::vector<double> tmpvalue(state.planes.size() * 2);
            surface_tree *surfacetree = build_surface_tree(surfaces, (vec_t)math::on_epsilon);

            vec_t bestvalue = 9e30;
            surface *bestsurface = nullptr;

            for (surface *p = surfaces; p; p = p->next)
            {
                if (p->onnode)
                    continue;
                if (p->detail_level != detaillevel)
                    continue;
                planecount++;

                double crosscount = 0;
                double frontcount = 0;
                double backcount = 0;
                double coplanarcount = 0;
                double epsilonsplit = 0;

                plane *split = &state.planes[(size_t)p->planenum];

                for (face *f = p->faces; f; f = f->next)
                {
                    if (f->style == face_style::discardable)
                        continue;
                    coplanarcount++;
                }
                test_surface_tree(surfacetree, split);
                frontcount += surfacetree->result.frontsize;
                backcount += surfacetree->result.backsize;
                for (face *f : *surfacetree->result.middle)
                {
                    if (f->planenum == p->planenum || f->planenum == (p->planenum ^ 1))
                        continue;
                    if (f->style == face_style::discardable)
                    {
                        face_side(f, split, &epsilonsplit);
                        continue;
                    }
                    switch (face_side(f, split, &epsilonsplit))
                    {
                    case math::winding::side_front:
                        frontcount++;
                        break;
                    case math::winding::side_back:
                        backcount++;
                        break;
                    case math::winding::side_on:
                        totalsplit++;
                        crosscount++;
                        break;
                    }
                }

                vec_t value = crosscount - std::sqrt(coplanarcount);
                if (coplanarcount == 0)
                    crosscount += 1;
                // bsp balancing: small files without adjusting factors per map
                double frac = (coplanarcount / 2 + crosscount / 2 + frontcount)
                    / (coplanarcount + frontcount + backcount + crosscount);
                double ent = (0.0001 < frac && frac < 0.9999)
                    ? (-frac * std::log(frac) / std::log(2.0)
                       - (1 - frac) * std::log(1 - frac) / std::log(2.0))
                    : 0.0; // the formula tends to 0 when frac is 0 or 1
                tmpvalue[(size_t)p->planenum * 2 + 1] = crosscount * (1 - ent);
                value += epsilonsplit * 10000;

                tmpvalue[(size_t)p->planenum * 2 + 0] = value;
            }
            double avesplit = totalsplit / planecount;
            for (surface *p = surfaces; p; p = p->next)
            {
                if (p->onnode)
                    continue;
                if (p->detail_level != detaillevel)
                    continue;
                vec_t value = tmpvalue[(size_t)p->planenum * 2 + 0]
                    + avesplit * tmpvalue[(size_t)p->planenum * 2 + 1];
                if (value < bestvalue)
                {
                    bestvalue = value;
                    bestsurface = p;
                }
            }

            if (!bestsurface)
                err::fatal("choose_plane_from_list: no valid planes");
            delete_surface_tree(surfacetree);
            return bestsurface;
        }

        int calc_split_detaillevel(const node *n)
        {
            int bestdetaillevel = -1;
            for (surface *s = n->surfaces; s; s = s->next)
            {
                if (s->onnode)
                    continue;
                for (face *f = s->faces; f; f = f->next)
                {
                    if (f->style == face_style::discardable)
                        continue;
                    if (bestdetaillevel == -1 || f->detail_level < bestdetaillevel)
                        bestdetaillevel = f->detail_level;
                }
            }
            return bestdetaillevel;
        }

        // selects a surface to split the group on; nullptr means leaf
        surface *select_partition(bsp_state &state, surface *surfaces, const node *n,
                                  bool usemidsplit, int splitdetaillevel,
                                  const math::vec3v &validmins, const math::vec3v &validmaxs)
        {
            if (splitdetaillevel == -1)
                return nullptr;
            // now we must choose a surface of this detail level

            if (usemidsplit)
            {
                surface *s = choose_mid_plane_from_list(state, surfaces,
                                                        validmins, validmaxs, splitdetaillevel);
                if (s != nullptr)
                    return s;
            }
            return choose_plane_from_list(state, surfaces, n->mins, n->maxs, splitdetaillevel);
        }

        void calc_surface_info(surface *surf)
        {
            if (surf->faces == nullptr)
                err::fatal("calc_surface_info: surface without a face");

            for (int i = 0; i < 3; i++)
            {
                surf->mins[i] = 99999;
                surf->maxs[i] = -99999;
            }

            surf->detail_level = -1;
            for (face *f = surf->faces; f; f = f->next)
            {
                if (f->contents >= 0)
                    err::fatal("bad contents");
                for (int i = 0; i < f->numpoints; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        if (f->pts[i][j] < surf->mins[j])
                            surf->mins[j] = f->pts[i][j];
                        if (f->pts[i][j] > surf->maxs[j])
                            surf->maxs[j] = f->pts[i][j];
                    }
                }
                if (surf->detail_level == -1 || f->detail_level < surf->detail_level)
                    surf->detail_level = f->detail_level;
            }
        }

        // when moving to the next detail level, discardable faces of previous
        // levels remain off node; remove them now
        void fix_detaillevel_for_discardable(node *n, int detaillevel)
        {
            surface *s;
            surface **psnext;
            face *f;
            face **pfnext;
            for (psnext = &n->surfaces; (s = *psnext) != nullptr;)
            {
                if (s->onnode)
                {
                    psnext = &s->next;
                    continue;
                }
                for (pfnext = &s->faces; (f = *pfnext) != nullptr;)
                {
                    if (detaillevel == -1 || f->detail_level < detaillevel)
                    {
                        *pfnext = f->next;
                        free_face(f);
                    }
                    else
                    {
                        pfnext = &f->next;
                    }
                }
                if (!s->faces)
                {
                    *psnext = s->next;
                    delete s;
                }
                else
                {
                    psnext = &s->next;
                    calc_surface_info(s);
                }
            }
        }

        void divide_surface(bsp_state &state, surface *in, const plane *split,
                            surface **front, surface **back)
        {
            face *frontlist = nullptr;
            face *backlist = nullptr;
            face *next;
            plane *inplane = &state.planes[(size_t)in->planenum];
            bool split_lists = false;

            // parallel case is easy
            if (inplane->normal[0] == split->normal[0]
                && inplane->normal[1] == split->normal[1]
                && inplane->normal[2] == split->normal[2])
            {
                if (inplane->dist > split->dist)
                {
                    *front = in;
                    *back = nullptr;
                    return;
                }
                if (inplane->dist < split->dist)
                {
                    *front = nullptr;
                    *back = in;
                    return;
                }
                // split the surface into front and back
                for (face *facet = in->faces; facet; facet = next)
                {
                    next = facet->next;
                    if (facet->planenum & 1)
                    {
                        facet->next = backlist;
                        backlist = facet;
                    }
                    else
                    {
                        facet->next = frontlist;
                        frontlist = facet;
                    }
                }
                split_lists = true;
            }

            if (!split_lists)
            {
                // do a real split; may still end up entirely on one side
                for (face *facet = in->faces; facet; facet = next)
                {
                    next = facet->next;
                    face *frontfrag;
                    face *backfrag;
                    split_face(state, facet, split, &frontfrag, &backfrag);
                    if (frontfrag)
                    {
                        frontfrag->next = frontlist;
                        frontlist = frontfrag;
                    }
                    if (backfrag)
                    {
                        backfrag->next = backlist;
                        backlist = backfrag;
                    }
                }
            }

            // if nothing actually got split, just move the in plane
            if (frontlist == nullptr && backlist == nullptr)
            {
                *front = nullptr;
                *back = nullptr;
                return;
            }
            if (frontlist == nullptr)
            {
                *front = nullptr;
                *back = in;
                in->faces = backlist;
                return;
            }
            if (backlist == nullptr)
            {
                *front = in;
                *back = nullptr;
                in->faces = frontlist;
                return;
            }

            // stuff got split, so allocate one new surface and reuse in
            surface *news = new surface();
            *news = *in;
            news->faces = backlist;
            *back = news;

            in->faces = frontlist;
            *front = in;

            calc_surface_info(news);
            calc_surface_info(in);
        }

        void split_node_surfaces(bsp_state &state, surface *surfaces, const node *n)
        {
            plane *splitplane = &state.planes[(size_t)n->planenum];

            surface *frontlist = nullptr;
            surface *backlist = nullptr;
            surface *next;
            for (surface *p = surfaces; p; p = next)
            {
                next = p->next;
                surface *frontfrag;
                surface *backfrag;
                divide_surface(state, p, splitplane, &frontfrag, &backfrag);

                if (frontfrag)
                {
                    if (!frontfrag->faces)
                        err::fatal("surface with no faces");
                    frontfrag->next = frontlist;
                    frontlist = frontfrag;
                }
                if (backfrag)
                {
                    if (!backfrag->faces)
                        err::fatal("surface with no faces");
                    backfrag->next = backlist;
                    backlist = backfrag;
                }
            }

            n->children[0]->surfaces = frontlist;
            n->children[1]->surfaces = backlist;
        }

        void split_node_brushes(bsp_state &state, brush *brushes, const node *n)
        {
            brush *frontlist = nullptr;
            brush *backlist = nullptr;
            brush *next;
            const plane *splitplane = &state.planes[(size_t)n->planenum];
            for (brush *b = brushes; b; b = next)
            {
                next = b->next;
                brush *frontfrag;
                brush *backfrag;
                split_brush(b, splitplane, &frontfrag, &backfrag);
                if (frontfrag)
                {
                    frontfrag->next = frontlist;
                    frontlist = frontfrag;
                }
                if (backfrag)
                {
                    backfrag->next = backlist;
                    backlist = backfrag;
                }
            }
            n->children[0]->detailbrushes = frontlist;
            n->children[1]->detailbrushes = backlist;
        }

        int rank_for_contents(int contents)
        {
            switch (contents)
            {
            case contents_empty:
                return 0;
            case contents_water:
                return 1;
            case contents_translucent:
                return 2;
            case contents_current_0:
                return 3;
            case contents_current_90:
                return 4;
            case contents_current_180:
                return 5;
            case contents_current_270:
                return 6;
            case contents_current_up:
                return 7;
            case contents_current_down:
                return 8;
            case contents_slime:
                return 9;
            case contents_lava:
                return 10;
            case contents_sky:
                return 11;
            case contents_solid:
                return 12;
            default:
                err::fatal("rank_for_contents: bad contents %i", contents);
            }
        }

        int contents_for_rank(int rank)
        {
            switch (rank)
            {
            case -1:
                return contents_empty;
            case 0:
                return contents_empty;
            case 1:
                return contents_water;
            case 2:
                return contents_translucent;
            case 3:
                return contents_current_0;
            case 4:
                return contents_current_90;
            case 5:
                return contents_current_180;
            case 6:
                return contents_current_270;
            case 7:
                return contents_current_up;
            case 8:
                return contents_current_down;
            case 9:
                return contents_slime;
            case 10:
                return contents_lava;
            case 11:
                return contents_sky;
            case 12:
                return contents_solid;
            default:
                err::fatal("contents_for_rank: bad rank %i", rank);
            }
        }

        const char *contents_to_string(int contents)
        {
            switch (contents)
            {
            case contents_empty:
                return "EMPTY";
            case contents_solid:
                return "SOLID";
            case contents_water:
                return "WATER";
            case contents_slime:
                return "SLIME";
            case contents_lava:
                return "LAVA";
            case contents_sky:
                return "SKY";
            case contents_current_0:
                return "CURRENT_0";
            case contents_current_90:
                return "CURRENT_90";
            case contents_current_180:
                return "CURRENT_180";
            case contents_current_270:
                return "CURRENT_270";
            case contents_current_up:
                return "CURRENT_UP";
            case contents_current_down:
                return "CURRENT_DOWN";
            case contents_translucent:
                return "TRANSLUCENT";
            default:
                return "UNKNOWN";
            }
        }

        void free_leaf_surfs(node *leaf)
        {
            surface *snext;
            for (surface *surf = leaf->surfaces; surf; surf = snext)
            {
                snext = surf->next;
                face *fnext;
                for (face *f = surf->faces; f; f = fnext)
                {
                    fnext = f->next;
                    free_face(f);
                }
                delete surf;
            }
            leaf->surfaces = nullptr;
        }

        void free_leaf_brushes(node *leaf)
        {
            brush *next;
            for (brush *b = leaf->detailbrushes; b; b = next)
            {
                next = b->next;
                free_brush(b);
            }
            leaf->detailbrushes = nullptr;
        }

        // the engine keeps a leaf's mark surface count in an unsigned short, so
        // that is the real ceiling. the working buffer used to be a fixed 128kb
        // stack frame, which capped leaves far below what the format allows and
        // rejected heavily detailed maps; it is now grown on demand and reused
        // across leaves, so the allocation cost is paid once.
        constexpr int max_leaf_faces = 65535;

        void make_leaf(node *leafnode)
        {
            leafnode->planenum = -1;

            leafnode->iscontentsdetail = leafnode->detailbrushes != nullptr;
            free_leaf_brushes(leafnode);
            leafnode->detailbrushes = nullptr;
            if (leafnode->boundsbrush)
                free_brush(leafnode->boundsbrush);
            leafnode->boundsbrush = nullptr;

            if (!(leafnode->isportalleaf && leafnode->contents == contents_solid))
            {
                // reused across leaves; make_leaf never recurses
                static thread_local std::vector<face *> markfaces;
                markfaces.clear();
                for (surface *surf = leafnode->surfaces; surf; surf = surf->next)
                {
                    if (!surf->onnode)
                        continue;
                    for (face *f = surf->faces; f; f = f->next)
                    {
                        if (f->original == nullptr)
                            continue; // not on node or content is solid
                        if ((int)markfaces.size() >= max_leaf_faces)
                            err::fatal("exceeded max_leaf_faces\n"
                                       "    leaf bounds (%.0f %.0f %.0f)-(%.0f %.0f %.0f)"
                                       " detail=%d portalleaf=%d contents=%d",
                                       (double)leafnode->loosemins.x,
                                       (double)leafnode->loosemins.y,
                                       (double)leafnode->loosemins.z,
                                       (double)leafnode->loosemaxs.x,
                                       (double)leafnode->loosemaxs.y,
                                       (double)leafnode->loosemaxs.z,
                                       (int)leafnode->isdetail,
                                       (int)leafnode->isportalleaf,
                                       leafnode->contents);
                        markfaces.push_back(f->original);
                    }
                }
                markfaces.push_back(nullptr); // end marker

                leafnode->markfaces = new face *[markfaces.size()];
                std::memcpy(leafnode->markfaces, markfaces.data(),
                            markfaces.size() * sizeof(*leafnode->markfaces));
            }

            free_leaf_surfs(leafnode);
            leafnode->surfaces = nullptr;
        }

        // create the portal for the node's own plane by clipping the full
        // plane winding by all the other portal planes bounding the node
        void make_node_portal(bsp_state &state, node *n)
        {
            plane *split = &state.planes[(size_t)n->planenum];
            math::winding *w = new math::winding(
                math::winding::from_plane(split->normal, split->dist, (vec_t)plane_winding_range));

            portal *new_portal = new portal();
            new_portal->plane_ = *split;
            new_portal->onnode = n;

            int side = 0;
            for (portal *p = n->portals; p; p = p->next[side])
            {
                plane clipplane = p->plane_;
                if (p->nodes[0] == n)
                {
                    side = 0;
                }
                else if (p->nodes[1] == n)
                {
                    clipplane.dist = -clipplane.dist;
                    clipplane.normal = -clipplane.normal;
                    side = 1;
                }
                else
                {
                    err::fatal("make_node_portal: mislinked portal");
                }

                w->clip_in_place(clipplane.normal, clipplane.dist, true);
                if (w->empty())
                {
                    delete w;
                    delete new_portal;
                    return;
                }
            }

            new_portal->winding_ = w;
            add_portal_to_nodes(new_portal, n->children[0], n->children[1]);
        }

        // move or split the portals bounding the node down to its children
        void split_node_portals(bsp_state &state, node *n)
        {
            plane *split = &state.planes[(size_t)n->planenum];
            node *f = n->children[0];
            node *b = n->children[1];

            int side = 0;
            portal *next_portal;
            for (portal *p = n->portals; p; p = next_portal)
            {
                if (p->nodes[0] == n)
                    side = 0;
                else if (p->nodes[1] == n)
                    side = 1;
                else
                    err::fatal("split_node_portals: mislinked portal");
                next_portal = p->next[side];

                node *other_node = p->nodes[!side];
                remove_portal_from_node(p, p->nodes[0]);
                remove_portal_from_node(p, p->nodes[1]);

                // cut the portal into two portals, one on each side
                math::winding frontwinding, backwinding;
                math::winding::divide_side where =
                    p->winding_->divide(split->normal, split->dist, frontwinding, backwinding);

                if (where == math::winding::divide_side::back)
                {
                    if (side == 0)
                        add_portal_to_nodes(p, b, other_node);
                    else
                        add_portal_to_nodes(p, other_node, b);
                    continue;
                }
                if (where == math::winding::divide_side::front)
                {
                    if (side == 0)
                        add_portal_to_nodes(p, f, other_node);
                    else
                        add_portal_to_nodes(p, other_node, f);
                    continue;
                }

                // the winding is split
                portal *new_portal = new portal();
                *new_portal = *p;
                new_portal->winding_ = new math::winding(std::move(backwinding));
                *p->winding_ = std::move(frontwinding);

                if (side == 0)
                {
                    add_portal_to_nodes(p, f, other_node);
                    add_portal_to_nodes(new_portal, b, other_node);
                }
                else
                {
                    add_portal_to_nodes(p, other_node, f);
                    add_portal_to_nodes(new_portal, other_node, b);
                }
            }

            n->portals = nullptr;
        }

        // bounds a node by minmaxing its portal points returns true when the
        // node is large enough to midsplit
        bool calc_node_bounds(bsp_state &state, node *n,
                              math::vec3v &validmins, math::vec3v &validmaxs)
        {
            if (n->isdetail)
                return false;
            n->mins[0] = n->mins[1] = n->mins[2] = (vec_t)bogus_range;
            n->maxs[0] = n->maxs[1] = n->maxs[2] = (vec_t)-bogus_range;

            int side = 0;
            portal *next_portal;
            for (portal *p = n->portals; p; p = next_portal)
            {
                if (p->nodes[0] == n)
                    side = 0;
                else if (p->nodes[1] == n)
                    side = 1;
                else
                    err::fatal("calc_node_bounds: mislinked portal");
                next_portal = p->next[side];

                for (int i = 0; i < p->winding_->size(); i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        vec_t v = (*p->winding_)[i][j];
                        if (v < n->mins[j])
                            n->mins[j] = v;
                        if (v > n->maxs[j])
                            n->maxs[j] = v;
                    }
                }
            }

            if (n->isportalleaf)
                return false;
            for (int i = 0; i < 3; i++)
            {
                vec_t low = (vec_t)-(tree_extent + state.options.maxnode_size);
                vec_t high = (vec_t)(tree_extent + state.options.maxnode_size);
                validmins[i] = n->mins[i] > low ? n->mins[i] : low;
                validmaxs[i] = n->maxs[i] < high ? n->maxs[i] : high;
            }
            for (int i = 0; i < 3; i++)
            {
                if (validmaxs[i] - validmins[i] <= math::on_epsilon)
                    return false;
            }
            for (int i = 0; i < 3; i++)
            {
                if (validmaxs[i] - validmins[i] > state.options.maxnode_size + math::on_epsilon)
                    return true;
            }
            return false;
        }

        // final merge attempt then subdivision; the copies chopped into the
        // leafs will reference these originals
        void copy_faces_to_node(bsp_state &state, node *n, surface *surf)
        {
            // merge as much as possible
            merge_plane_faces(state, surf);

            // subdivide large faces
            face **prevptr = &surf->faces;
            while (true)
            {
                face *f = *prevptr;
                if (!f)
                    break;
                subdivide_face(state, f, prevptr);
                f = *prevptr;
                prevptr = &f->next;
            }

            // copy the faces to the node and consider them the originals
            n->surfaces = nullptr;
            n->faces = nullptr;
            for (face *f = surf->faces; f; f = f->next)
            {
                if (f->style == face_style::discardable)
                    continue;
                if (f->contents != contents_solid)
                {
                    face *newf = alloc_face();
                    *newf = *f;
                    f->original = newf;
                    newf->next = n->faces;
                    n->faces = newf;
                }
            }
        }

        void build_bsp_tree_r(bsp_state &state, node *n)
        {
            math::vec3v validmins, validmaxs;
            bool midsplit = calc_node_bounds(state, n, validmins, validmaxs);
            if (n->boundsbrush)
            {
                calc_brush_bounds(n->boundsbrush, n->loosemins, n->loosemaxs);
            }
            else
            {
                n->loosemins[0] = n->loosemins[1] = n->loosemins[2] = (vec_t)bogus_range;
                n->loosemaxs[0] = n->loosemaxs[1] = n->loosemaxs[2] = (vec_t)-bogus_range;
            }

            int splitdetaillevel = calc_split_detaillevel(n);
            fix_detaillevel_for_discardable(n, splitdetaillevel);
            surface *split = select_partition(state, n->surfaces, n, midsplit,
                                              splitdetaillevel, validmins, validmaxs);
            if (!n->isdetail && (!split || split->detail_level > 0))
            {
                n->isportalleaf = true;
                classify_leaf_contents(state, n->surfaces, n); // sets contents
                if (n->contents == contents_solid)
                    split = nullptr;
            }
            else
            {
                n->isportalleaf = false;
            }
            if (!split)
            {
                // this is a leaf node
                make_leaf(n);
                return;
            }

            // these are final polygons
            split->onnode = n; // can't use again
            surface *allsurfs = n->surfaces;
            n->planenum = split->planenum;
            n->faces = nullptr;
            copy_faces_to_node(state, n, split);

            n->children[0] = alloc_node();
            n->children[1] = alloc_node();
            n->children[0]->isdetail = split->detail_level > 0;
            n->children[1]->isdetail = split->detail_level > 0;

            // split all the polysurfaces into front and back lists
            split_node_surfaces(state, allsurfs, n);
            split_node_brushes(state, n->detailbrushes, n);
            if (n->boundsbrush)
            {
                for (int k = 0; k < 2; k++)
                {
                    plane p;
                    if (k == 0)
                    {
                        // front child
                        p.normal = state.planes[(size_t)split->planenum].normal;
                        p.dist = state.planes[(size_t)split->planenum].dist - (vec_t)bounds_expansion;
                    }
                    else
                    {
                        // back child
                        p.normal = -state.planes[(size_t)split->planenum].normal;
                        p.dist = -state.planes[(size_t)split->planenum].dist - (vec_t)bounds_expansion;
                    }
                    brush *copy = new_brush_from_brush(n->boundsbrush);
                    brush *front;
                    brush *back;
                    split_brush(copy, &p, &front, &back);
                    if (back)
                        free_brush(back);
                    if (!front)
                    {
                        logging::warn("build_bsp_tree_r: bounds was clipped away at (%f,%f,%f)-(%f,%f,%f)",
                                      (double)n->loosemins[0], (double)n->loosemins[1], (double)n->loosemins[2],
                                      (double)n->loosemaxs[0], (double)n->loosemaxs[1], (double)n->loosemaxs[2]);
                    }
                    n->children[k]->boundsbrush = front;
                }
                free_brush(n->boundsbrush);
            }
            n->boundsbrush = nullptr;

            if (!split->detail_level)
            {
                make_node_portal(state, n);
                split_node_portals(state, n);
            }

            // recursively do the children
            build_bsp_tree_r(state, n->children[0]);
            build_bsp_tree_r(state, n->children[1]);
            update_status(state);
        }
    }

    // determines the leaf content from its structural boundary faces and
    // records every distinct classification when those faces disagree
    void classify_leaf_contents(bsp_state &state, surface *planelist, node *leafnode)
    {
        int selected_rank = -1;
        std::vector<int> contents;
        for (surface *surf = planelist; surf; surf = surf->next)
        {
            if (!surf->onnode)
                continue;
            for (face *f = surf->faces; f; f = f->next)
            {
                if (f->contents == contents_hint)
                    f->contents = contents_empty;
                if (f->detail_level)
                    continue;

                int rank = rank_for_contents(f->contents);
                if (rank > selected_rank)
                    selected_rank = rank;
                if (std::find(contents.begin(), contents.end(), f->contents) == contents.end())
                    contents.push_back(f->contents);
            }
        }

        leafnode->contents = contents_for_rank(selected_rank);
        if (contents.size() < 2)
            return;

        std::sort(contents.begin(), contents.end(), [](int left, int right) {
            return rank_for_contents(left) < rank_for_contents(right);
        });

        leaf_content_conflict conflict;
        conflict.model = state.nummodels - 1;
        conflict.hull = state.hullnum;
        conflict.mins = leafnode->mins;
        conflict.maxs = leafnode->maxs;
        conflict.contents = std::move(contents);
        conflict.selected_content = leafnode->contents;
        state.leaf_content_conflicts.push_back(std::move(conflict));
    }

    namespace
    {
        constexpr size_t max_console_leaf_conflicts = 10;

        std::string content_resolution(const leaf_content_conflict &conflict)
        {
            std::string result;
            for (size_t i = 0; i < conflict.contents.size(); i++)
            {
                if (i)
                    result += " / ";
                result += contents_to_string(conflict.contents[i]);
            }
            result += " -> ";
            result += contents_to_string(conflict.selected_content);
            return result;
        }

        void report_model(bool console_output, const bsp_state &state, int model)
        {
            const format::entity *ent = entity_for_model(state, model);
            if (model != 0 && !state.entities.empty() && ent == &state.entities[0])
                ent = nullptr;

            const char *classname = ent ? ent->value("classname") : "unknown";
            const char *origin = ent ? ent->value("origin") : "";
            const char *targetname = ent ? ent->value("targetname") : "";
            if (console_output)
                logging::console("\n    model %d  %s", model, classname);
            else
                logging::file("\n    model %d  %s", model, classname);
            if (origin[0])
            {
                if (console_output)
                    logging::console("  origin \"%s\"", origin);
                else
                    logging::file("  origin \"%s\"", origin);
            }
            if (targetname[0])
            {
                if (console_output)
                    logging::console("  targetname \"%s\"", targetname);
                else
                    logging::file("  targetname \"%s\"", targetname);
            }
            if (console_output)
                logging::console("\n");
            else
                logging::file("\n");
        }

        void report_conflicts(const bsp_state &state, size_t count, bool console_output)
        {
            int previous_model = -1;
            for (size_t i = 0; i < count; i++)
            {
                const leaf_content_conflict &conflict = state.leaf_content_conflicts[i];
                if (conflict.model != previous_model)
                {
                    report_model(console_output, state, conflict.model);
                    previous_model = conflict.model;
                }

                std::string resolution = content_resolution(conflict);
                if (console_output)
                {
                    logging::console("      hull %d  %s\n", conflict.hull, resolution.c_str());
                    logging::console("      bounds  (%.0f, %.0f, %.0f) -> (%.0f, %.0f, %.0f)\n",
                                     (double)conflict.mins[0], (double)conflict.mins[1], (double)conflict.mins[2],
                                     (double)conflict.maxs[0], (double)conflict.maxs[1], (double)conflict.maxs[2]);
                }
                else
                {
                    logging::file("      hull %d  %s\n", conflict.hull, resolution.c_str());
                    logging::file("      bounds  (%.0f, %.0f, %.0f) -> (%.0f, %.0f, %.0f)\n",
                                  (double)conflict.mins[0], (double)conflict.mins[1], (double)conflict.mins[2],
                                  (double)conflict.maxs[0], (double)conflict.maxs[1], (double)conflict.maxs[2]);
                }
            }
        }
    }

    void print_leaf_content_conflicts(const bsp_state &state)
    {
        const size_t conflict_count = state.leaf_content_conflicts.size();
        if (!conflict_count)
            return;

        std::vector<int> models;
        for (const leaf_content_conflict &conflict : state.leaf_content_conflicts)
        {
            if (std::find(models.begin(), models.end(), conflict.model) == models.end())
                models.push_back(conflict.model);
        }

        logging::info("\n  !!! WARNING: BSP LEAF CONTENT CONFLICTS\n\n");
        logging::info("    found    %zu affected BSP %s across %zu brush %s\n",
                      conflict_count, conflict_count == 1 ? "leaf" : "leaves",
                      models.size(), models.size() == 1 ? "model" : "models");
        logging::info("    issue    structural boundary faces disagree on the content of each listed leaf\n");
        logging::info("    result   the highest ranked boundary content was assigned to each leaf\n");
        logging::info("    action   inspect brushwork touching the listed bounds for tiny gaps or geometry near compiler tolerances\n");

        size_t console_count = std::min(conflict_count, max_console_leaf_conflicts);
        report_conflicts(state, console_count, true);
        if (console_count < conflict_count)
            logging::console("\n    (%zu more affected leaves; see the logfile)\n", conflict_count - console_count);

        report_conflicts(state, conflict_count, false);
        logging::file("\n  The bounds describe the affected BSP leaf and may not locate the exact brush defect.\n");

        char warning[160];
        std::snprintf(warning, sizeof(warning), "%zu BSP leaf content %s found across %zu brush %s",
                      conflict_count, conflict_count == 1 ? "conflict" : "conflicts",
                      models.size(), models.size() == 1 ? "model" : "models");
        logging::add_warning_summary(warning);
    }

    // takes a chain of surfaces and returns a bsp tree with faces off the
    // nodes the original surface chain is completely freed
    node *solid_bsp(bsp_state &state, const surfchain *surfhead,
                    brush *detailbrushes, bool report_progress)
    {
        state.report_progress = report_progress;
        state.num_processed = state.num_reported = 0;

        auto hull_start = std::chrono::steady_clock::now();
        if (report_progress)
            progress::section("building bsp tree");

        node *headnode = alloc_node();
        headnode->surfaces = surfhead->surfaces;
        headnode->detailbrushes = detailbrushes;
        headnode->isdetail = false;
        math::vec3v brushmins, brushmaxs;
        for (int i = 0; i < 3; i++)
        {
            brushmins[i] = surfhead->mins[i] - side_space;
            brushmaxs[i] = surfhead->maxs[i] + side_space;
        }
        headnode->boundsbrush = brush_from_box(brushmins, brushmaxs);

        // generate six portals that enclose the entire world
        make_headnode_portals(state, headnode, surfhead->mins, surfhead->maxs);

        // recursively partition everything
        build_bsp_tree_r(state, headnode);

        if (report_progress)
        {
            ++state.num_processed;
            static const char *hull_tag[num_hulls] = {"visual", "standing", "large", "crouched"};
            const char *tag = state.hullnum < num_hulls ? hull_tag[state.hullnum] : "";
            double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - hull_start).count();
            char label[32], buf[32];
            std::snprintf(label, sizeof(label), "hull %d (%s)", state.hullnum, tag);
            logging::info("  %-22s %10s nodes  %8.2fs\n", label,
                          str::with_commas(state.num_processed, buf, sizeof(buf)), secs);
        }

        return headnode;
    }
}
