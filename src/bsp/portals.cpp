#include "internal.h"

#include <cstdio>

#include "../common/error.h"
#include "../common/log.h"

namespace bsp
{
    void add_portal_to_nodes(portal *p, node *front, node *back)
    {
        if (p->nodes[0] || p->nodes[1])
            err::fatal("add_portal_to_nodes: already included");

        p->nodes[0] = front;
        p->next[0] = front->portals;
        front->portals = p;

        p->nodes[1] = back;
        p->next[1] = back->portals;
        back->portals = p;
    }

    void remove_portal_from_node(portal *p, node *l)
    {
        portal **pp = &l->portals;
        portal *t;
        while (true)
        {
            t = *pp;
            if (!t)
                err::fatal("remove_portal_from_node: portal not in leaf");
            if (t == p)
                break;
            if (t->nodes[0] == l)
                pp = &t->next[0];
            else if (t->nodes[1] == l)
                pp = &t->next[1];
            else
                err::fatal("remove_portal_from_node: portal not bounding leaf");
        }

        if (p->nodes[0] == l)
        {
            *pp = p->next[0];
            p->nodes[0] = nullptr;
        }
        else if (p->nodes[1] == l)
        {
            *pp = p->next[1];
            p->nodes[1] = nullptr;
        }
    }

    // the created portals face the state's outside node
    void make_headnode_portals(bsp_state &state, node *headnode,
                               const math::vec3v &mins, const math::vec3v &maxs)
    {
        math::vec3v bounds[2];
        portal *portals[6];
        plane bplanes[6];

        // pad with some space so there will never be null volume leafs
        for (int i = 0; i < 3; i++)
        {
            bounds[0][i] = mins[i] - side_space;
            bounds[1][i] = maxs[i] + side_space;
        }

        state.outside_node.contents = contents_solid;
        state.outside_node.portals = nullptr;

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 2; j++)
            {
                int n = j * 3 + i;

                portal *p = new portal();
                portals[n] = p;

                plane *pl = &bplanes[n];
                *pl = plane();
                if (j)
                {
                    pl->normal[i] = -1;
                    pl->dist = -bounds[j][i];
                }
                else
                {
                    pl->normal[i] = 1;
                    pl->dist = bounds[j][i];
                }
                p->plane_ = *pl;
                p->winding_ = new math::winding(
                    math::winding::from_plane(pl->normal, pl->dist, (vec_t)plane_winding_range));
                add_portal_to_nodes(p, headnode, &state.outside_node);
            }
        }

        // clip the basewindings by all the other planes
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                if (j == i)
                    continue;
                portals[i]->winding_->clip_in_place(bplanes[j].normal, bplanes[j].dist, true);
            }
        }
    }

    // ==========================================================================
    // portal file generation
    // ==========================================================================

    namespace
    {
        void write_portal_file_r(std::FILE *pf, const node *n)
        {
            if (!n->isportalleaf)
            {
                write_portal_file_r(pf, n->children[0]);
                write_portal_file_r(pf, n->children[1]);
                return;
            }

            if (n->contents == contents_solid)
                return;

            for (const portal *p = n->portals; p;)
            {
                math::winding *w = p->winding_;
                if (w && p->nodes[0] == n)
                {
                    if (p->nodes[0]->contents == p->nodes[1]->contents)
                    {
                        // sometimes planes get turned around when they are very
                        // near the changeover point between different axes
                        // interpret the plane the way vis will and flip the
                        // side order if needed
                        math::vec3v normal2;
                        vec_t dist2;
                        w->plane(normal2, dist2);
                        if (math::dot(p->plane_.normal, normal2) < 1.0 - math::on_epsilon)
                        {
                            // backwards
                            if (math::dot(p->plane_.normal, normal2) > -1.0 + math::on_epsilon)
                                logging::warn("colinear portal");
                            else
                                logging::warn("backward portal");
                            std::fprintf(pf, "%u %i %i ", (unsigned)w->size(),
                                         p->nodes[1]->visleafnum, p->nodes[0]->visleafnum);
                        }
                        else
                        {
                            std::fprintf(pf, "%u %i %i ", (unsigned)w->size(),
                                         p->nodes[0]->visleafnum, p->nodes[1]->visleafnum);
                        }

                        for (int i = 0; i < w->size(); i++)
                        {
                            std::fprintf(pf, "(%f %f %f) ",
                                         (double)(*w)[i][0], (double)(*w)[i][1], (double)(*w)[i][2]);
                        }
                        std::fprintf(pf, "\n");
                    }
                }

                if (p->nodes[0] == n)
                    p = p->next[0];
                else
                    p = p->next[1];
            }
        }

        void number_leafs_r(bsp_state &state, node *n)
        {
            if (!n->isportalleaf)
            {
                // decision node
                n->visleafnum = -99;
                number_leafs_r(state, n->children[0]);
                number_leafs_r(state, n->children[1]);
                return;
            }

            if (n->contents == contents_solid)
            {
                // solid block, viewpoint never inside
                n->visleafnum = -1;
                return;
            }

            n->visleafnum = state.num_visleafs++;

            for (const portal *p = n->portals; p;)
            {
                if (p->nodes[0] == n)
                {
                    // only write out from the first leaf
                    if (p->nodes[0]->contents == p->nodes[1]->contents)
                        state.num_visportals++;
                    p = p->next[0];
                }
                else
                {
                    p = p->next[1];
                }
            }
        }

        int count_child_leafs_r(const node *n)
        {
            if (n->planenum == -1)
            {
                // dleaf
                return n->iscontentsdetail ? 0 : 1;
            }
            int count = 0;
            count += count_child_leafs_r(n->children[0]);
            count += count_child_leafs_r(n->children[1]);
            return count;
        }

        void write_leaf_count_r(std::FILE *pf, const node *n)
        {
            if (!n->isportalleaf)
            {
                write_leaf_count_r(pf, n->children[0]);
                write_leaf_count_r(pf, n->children[1]);
                return;
            }
            if (n->contents == contents_solid)
                return;
            int count = count_child_leafs_r(n);
            std::fprintf(pf, "%i\n", count);
        }
    }

    void write_portal_file(bsp_state &state, node *headnode)
    {
        // set the visleafnum field in every leaf and count the portals
        state.num_visleafs = 0;
        state.num_visportals = 0;
        number_leafs_r(state, headnode);

        std::FILE *pf = std::fopen(state.portfilename.c_str(), "w");
        if (!pf)
            err::fatal("error writing portal file %s", state.portfilename.c_str());

        std::fprintf(pf, "%i\n", state.num_visleafs);
        std::fprintf(pf, "%i\n", state.num_visportals);

        write_leaf_count_r(pf, headnode);
        write_portal_file_r(pf, headnode);
        std::fclose(pf);
        logging::file("  wrote portal file '%s'\n", state.portfilename.c_str());
    }

    void free_portals(node *n)
    {
        if (!n->isportalleaf)
        {
            free_portals(n->children[0]);
            free_portals(n->children[1]);
            return;
        }

        portal *nextp;
        for (portal *p = n->portals; p; p = nextp)
        {
            if (p->nodes[0] == n)
                nextp = p->next[0];
            else
                nextp = p->next[1];
            remove_portal_from_node(p, p->nodes[0]);
            remove_portal_from_node(p, p->nodes[1]);
            delete p->winding_;
            delete p;
        }
    }
}
