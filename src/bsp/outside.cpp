#include "internal.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include "../common/error.h"
#include "../common/log.h"
#include "../common/string_util.h"

// outside filling and leak detection, including this fork's shortest path
// leak trail (dijkstra over the portal graph) and the consolidated summary

namespace bsp
{
    namespace
    {
        // one surveyed hole: the entity that could see out, and where the path
        // it takes pierces the shell
        struct leak_site
        {
            std::string classname;
            math::vec3v origin;
            math::vec3v hole;
            bool hashole = false;
        };

        struct fill_state
        {
            int outleafs = 0;
            int valid = 0;
            int c_falsenodes = 0;
            int c_free_faces = 0;
            int c_keep_faces = 0;
            int hit_occupied = 0;

            // leak trail
            math::vec3v leakexit;   // where the shortest leak path pierces the shell
            bool haveleaktrail = false;
            node *occupiednode = nullptr; // the leaf of the entity that leaked
            std::vector<math::vec3v> leaktrail;

            // consolidated leak info printed once by print_leak_summary
            bool leak_any = false;
            int leak_hulls = 0; // bitmask of hulls that leaked
            char leak_entclass[64] = {};
            math::vec3v leak_entorigin;
            math::vec3v leak_hole;
            bool leak_hashole = false;

            // -allleaks survey: one entry per distinct hole
            std::vector<leak_site> sites;
            // occupied leafs the outside flood reached directly, collected with
            // occupied leafs treated as barriers so the frontier stays at the
            // holes instead of running on through the whole map
            std::vector<node *> frontier;
        };

        fill_state g_fill;

        void get_origin_key(const format::entity &ent, math::vec3v &out)
        {
            double v[3] = {0, 0, 0};
            std::sscanf(ent.value("origin"), "%lf %lf %lf", &v[0], &v[1], &v[2]);
            out = {(vec_t)v[0], (vec_t)v[1], (vec_t)v[2]};
        }

        node *point_in_leaf(bsp_state &state, node *n, const math::vec3v &point)
        {
            if (n->isportalleaf)
                return n;

            const plane &p = state.planes[(size_t)n->planenum];
            vec_t d = math::dot(p.normal, point) - p.dist;

            if (d > 0)
                return point_in_leaf(state, n->children[0], point);
            return point_in_leaf(state, n->children[1], point);
        }

        bool place_occupant(bsp_state &state, int num, const math::vec3v &point, node *headnode)
        {
            node *n = point_in_leaf(state, headnode, point);
            if (n->contents == contents_solid)
                return false;
            n->occupied = num;
            return true;
        }

        math::vec3v winding_center(const math::winding &w)
        {
            math::vec3v center{};
            for (int i = 0; i < w.size(); i++)
                math::add(center, w[i], center);
            math::scale(center, (vec_t)1.0 / w.size(), center);
            return center;
        }

        // perpendicular distance from point p to the segment a b
        vec_t point_seg_distance(const math::vec3v &p, const math::vec3v &a, const math::vec3v &b)
        {
            math::vec3v ab, ap, proj;
            math::subtract(b, a, ab);
            math::subtract(p, a, ap);
            vec_t ab2 = math::dot(ab, ab);
            vec_t t = (ab2 > 0) ? math::dot(ap, ab) / ab2 : 0;
            if (t < 0)
                t = 0;
            if (t > 1)
                t = 1;
            math::multiply_add(a, t, ab, proj);
            math::subtract(p, proj, proj);
            return (vec_t)math::length(proj);
        }

        // douglas peucker: mark which trail vertices to keep so the simplified
        // polyline stays within tol of the original
        void simplify_trail_r(int i0, int i1, vec_t tol, std::vector<char> &keep)
        {
            vec_t dmax = 0;
            int idx = -1;
            for (int i = i0 + 1; i < i1; i++)
            {
                vec_t d = point_seg_distance(g_fill.leaktrail[(size_t)i],
                                             g_fill.leaktrail[(size_t)i0],
                                             g_fill.leaktrail[(size_t)i1]);
                if (d > dmax)
                {
                    dmax = d;
                    idx = i;
                }
            }
            if (dmax > tol && idx > 0)
            {
                keep[(size_t)idx] = 1;
                simplify_trail_r(i0, idx, tol, keep);
                simplify_trail_r(idx, i1, tol, keep);
            }
        }

        // a dense 3d star at pos so the hole is an unmistakable blob in the
        // pointfile
        void write_leak_marker(std::FILE *f, const math::vec3v &pos)
        {
            const vec_t radius = 48.0;
            static const int dirs[13][3] = {
                {1, 0, 0}, {0, 1, 0}, {0, 0, 1},
                {1, 1, 0}, {1, -1, 0}, {1, 0, 1}, {1, 0, -1}, {0, 1, 1}, {0, 1, -1},
                {1, 1, 1}, {1, 1, -1}, {1, -1, 1}, {1, -1, -1},
            };
            for (int k = 0; k < 13; k++)
            {
                for (vec_t d = -radius; d <= radius; d += 2.0)
                {
                    math::vec3v p;
                    p[0] = pos[0] + dirs[k][0] * d;
                    p[1] = pos[1] + dirs[k][1] * d;
                    p[2] = pos[2] + dirs[k][2] * d;
                    std::fprintf(f, "%f %f %f\n", (double)p[0], (double)p[1], (double)p[2]);
                }
            }
        }

        // builds the geometrically shortest leak path from the leaked entity's
        // leaf out to the void: dijkstra over the portal graph weighted by the
        // distance between consecutive portal centers the path must cross the
        // hole, and its far end is the shell portal where the leak escapes
        bool build_shortest_leak_trail(bsp_state &state, node *occupied, const math::vec3v &startpos)
        {
            g_fill.leaktrail.clear();
            if (!occupied)
                return false;
            std::unordered_map<node *, portal *> parent_portal;
            std::unordered_map<node *, node *> parent_node;
            std::unordered_map<node *, double> dist;
            std::unordered_map<node *, math::vec3v> arrival; // portal center used to reach a node
            using queue_entry = std::pair<double, node *>;
            std::priority_queue<queue_entry, std::vector<queue_entry>, std::greater<queue_entry>> pq;

            dist[occupied] = 0;
            arrival[occupied] = startpos;
            parent_portal[occupied] = nullptr;
            parent_node[occupied] = nullptr;
            pq.push(queue_entry(0.0, occupied));

            node *exitleaf = nullptr;
            portal *exitportal = nullptr;
            double exitcost = 0;

            while (!pq.empty())
            {
                double d = pq.top().first;
                node *l = pq.top().second;
                pq.pop();
                if (d > dist[l])
                    continue; // stale entry
                if (exitleaf && d >= exitcost)
                    break; // best exit already settled

                for (portal *p = l->portals; p;)
                {
                    int side = (p->nodes[0] == l);
                    node *nb = p->nodes[side];
                    math::vec3v cp = winding_center(*p->winding_);
                    math::vec3v seg;
                    math::subtract(cp, arrival[l], seg);
                    double edge = math::length(seg);
                    double nd = d + edge;

                    if (nb == &state.outside_node)
                    {
                        if (!exitleaf || nd < exitcost)
                        {
                            exitleaf = l;
                            exitportal = p;
                            exitcost = nd;
                        }
                    }
                    else if (nb->contents != contents_solid
                             && nb->contents != contents_sky)
                    {
                        auto it = dist.find(nb);
                        if (it == dist.end() || nd < it->second)
                        {
                            dist[nb] = nd;
                            arrival[nb] = cp;
                            parent_portal[nb] = p;
                            parent_node[nb] = l;
                            pq.push(queue_entry(nd, nb));
                        }
                    }
                    p = p->next[!side];
                }
            }
            if (!exitleaf)
                return false;

            // backtrace exit -> entity, then reverse
            std::vector<math::vec3v> path;
            for (node *cur = exitleaf; cur && parent_portal[cur]; cur = parent_node[cur])
                path.push_back(winding_center(*parent_portal[cur]->winding_));
            std::reverse(path.begin(), path.end());
            g_fill.leaktrail = path;
            // append the shell portal center: the hole itself
            g_fill.leaktrail.push_back(winding_center(*exitportal->winding_));
            return true;
        }

        // emits one already-built trail into open pts/lin files and reports the
        // hole it pierces. shared by the single leak path and the -allleaks
        // survey, which concatenates several trails into the same pair of files
        bool emit_leak_trail(std::FILE *pts, std::FILE *lin, math::vec3v &hole_out)
        {
            int n = (int)g_fill.leaktrail.size();
            if (n < 1)
                return false;

            // the last point is the shell portal, whose center can sit past the
            // gap; the second to last is the constriction right at the hole
            math::vec3v hole = g_fill.leaktrail[(size_t)(n >= 2 ? n - 2 : n - 1)];
            hole_out = hole;

            std::vector<char> keep((size_t)n, 0);
            keep[0] = keep[(size_t)n - 1] = 1;
            if (n >= 2)
                simplify_trail_r(0, n - 1, 16.0, keep);
            std::vector<int> idx;
            for (int i = 0; i < n; i++)
            {
                if (keep[(size_t)i])
                    idx.push_back(i);
            }

            for (size_t s = 0; s + 1 < idx.size(); s++)
            {
                const math::vec3v &a = g_fill.leaktrail[(size_t)idx[s]];
                const math::vec3v &b = g_fill.leaktrail[(size_t)idx[s + 1]];
                // lin: one clean segment per simplified edge
                std::fprintf(lin, "%f %f %f - %f %f %f\n",
                             (double)a[0], (double)a[1], (double)a[2],
                             (double)b[0], (double)b[1], (double)b[2]);
                // pts: densify the edge for editors that render dots
                math::vec3v p = a;
                math::vec3v dir;
                math::subtract(b, a, dir);
                vec_t len = math::normalize(dir);
                while (len > 8)
                {
                    std::fprintf(pts, "%f %f %f\n", (double)p[0], (double)p[1], (double)p[2]);
                    math::multiply_add(p, 8, dir, p);
                    len -= 8;
                }
            }
            const math::vec3v &last = g_fill.leaktrail[(size_t)idx.back()];
            std::fprintf(pts, "%f %f %f\n", (double)last[0], (double)last[1], (double)last[2]);
            write_leak_marker(pts, hole); // obvious blob at the hole
            return true;
        }

        // opens the pair of leak files, or dies with the same message the
        // single trail writer always used
        void open_leak_files(const std::string &ptsname, const std::string &linname,
                             std::FILE *&pts, std::FILE *&lin)
        {
            lin = std::fopen(linname.c_str(), "w");
            pts = std::fopen(ptsname.c_str(), "w");
            if (!lin || !pts)
            {
                if (lin)
                    std::fclose(lin);
                if (pts)
                    std::fclose(pts);
                err::fatal("couldn't open leak file %s / %s", ptsname.c_str(), linname.c_str());
            }
        }

        // writes the simplified leak line (lin) and pointfile (pts)
        bool write_leak_files(const std::string &ptsname, const std::string &linname)
        {
            if (g_fill.leaktrail.empty())
                return false;
            std::FILE *pts = nullptr;
            std::FILE *lin = nullptr;
            open_leak_files(ptsname, linname, pts, lin);
            math::vec3v hole;
            bool ok = emit_leak_trail(pts, lin, hole);
            std::fclose(lin);
            std::fclose(pts);
            if (ok)
            {
                g_fill.leakexit = hole;
                g_fill.haveleaktrail = true;
            }
            return ok;
        }

        void free_detail_node_r(node *n)
        {
            if (n->planenum == -1)
            {
                if (!(n->isportalleaf && n->contents == contents_solid))
                {
                    delete[] n->markfaces;
                    n->markfaces = nullptr;
                }
                return;
            }
            for (int i = 0; i < 2; i++)
            {
                free_detail_node_r(n->children[i]);
                delete n->children[i];
                n->children[i] = nullptr;
            }
            face *next;
            for (face *f = n->faces; f; f = next)
            {
                next = f->next;
                free_face(f);
            }
            n->faces = nullptr;
        }

        void fill_leaf(node *l)
        {
            if (!l->isportalleaf)
            {
                logging::warn("fill_leaf: not leaf");
                return;
            }
            if (l->contents == contents_solid)
            {
                logging::warn("fill_leaf: fill solid");
                return;
            }
            free_detail_node_r(l);
            l->contents = contents_solid;
            l->planenum = -1;
        }

        // returns true if an occupied leaf is reached if fill is false, just
        // check, don't fill
        bool recursive_fill_outside(bsp_state &state, node *l, bool fill)
        {
            if (l->contents == contents_solid || l->contents == contents_sky)
                return false;

            if (l->valid == g_fill.valid)
                return false;

            if (l->occupied)
            {
                g_fill.hit_occupied = l->occupied;
                g_fill.occupiednode = l; // the leaked leaf starts the trail
                return true;
            }

            l->valid = g_fill.valid;

            // fill it and its neighbors
            if (fill)
                fill_leaf(l);
            g_fill.outleafs++;

            for (portal *p = l->portals; p;)
            {
                int s = (p->nodes[0] == l);
                if (recursive_fill_outside(state, p->nodes[s], fill))
                    return true; // leaked, so stop filling
                p = p->next[!s];
            }

            return false;
        }

        // the survey flood: same walk as recursive_fill_outside, except an
        // occupied leaf is a wall rather than a stopping condition. the flood
        // therefore covers the region that is unambiguously outside and halts
        // at every entity that can see out, which puts one frontier leaf at
        // each hole instead of one for the entire map
        void survey_outside_r(bsp_state &state, node *l)
        {
            if (l->contents == contents_solid || l->contents == contents_sky)
                return;
            if (l->valid == g_fill.valid)
                return;
            l->valid = g_fill.valid;

            if (l->occupied)
            {
                g_fill.frontier.push_back(l);
                return; // treat as a barrier so the flood stays outside
            }

            for (portal *p = l->portals; p;)
            {
                int s = (p->nodes[0] == l);
                survey_outside_r(state, p->nodes[s]);
                p = p->next[!s];
            }
        }

        // walks every frontier entity, builds its shortest path out, and keeps
        // one entry per distinct hole. several entities usually share a hole,
        // so sites are merged by proximity of the exit point rather than of the
        // entity
        void survey_leaks(bsp_state &state, const std::string &ptsname,
                          const std::string &linname)
        {
            constexpr vec_t same_hole = 128.0; // holes closer than this are one
            constexpr size_t max_sites = 256;  // bound the per frontier dijkstra

            g_fill.frontier.clear();
            g_fill.valid++;
            int s = !(state.outside_node.portals->nodes[1] == &state.outside_node);
            survey_outside_r(state, state.outside_node.portals->nodes[s]);
            if (g_fill.frontier.empty())
                return;

            // one representative per entity: a single entity can own several
            // leafs along the same opening
            std::vector<int> seen_entities;
            std::vector<node *> reps;
            for (node *l : g_fill.frontier)
            {
                if (std::find(seen_entities.begin(), seen_entities.end(), l->occupied)
                    != seen_entities.end())
                    continue;
                seen_entities.push_back(l->occupied);
                reps.push_back(l);
            }

            std::FILE *pts = nullptr;
            std::FILE *lin = nullptr;
            open_leak_files(ptsname, linname, pts, lin);

            for (node *l : reps)
            {
                if (g_fill.sites.size() >= max_sites)
                    break;
                const format::entity &ent = state.entities[(size_t)l->occupied];
                math::vec3v startpos;
                get_origin_key(ent, startpos);
                if (!build_shortest_leak_trail(state, l, startpos))
                    continue;

                int n = (int)g_fill.leaktrail.size();
                math::vec3v hole = g_fill.leaktrail[(size_t)(n >= 2 ? n - 2 : n - 1)];
                bool duplicate = false;
                for (const leak_site &site : g_fill.sites)
                {
                    math::vec3v d;
                    math::subtract(site.hole, hole, d);
                    if (math::length(d) < same_hole)
                    {
                        duplicate = true;
                        break;
                    }
                }
                if (duplicate)
                    continue;

                math::vec3v written;
                if (!emit_leak_trail(pts, lin, written))
                    continue;
                leak_site site;
                site.classname = ent.value("classname");
                site.origin = startpos;
                site.hole = written;
                site.hashole = true;
                g_fill.sites.push_back(site);
            }

            std::fclose(lin);
            std::fclose(pts);
        }

        void mark_faces_inside_r(node *n)
        {
            if (n->planenum == -1)
            {
                for (face **fp = n->markfaces; *fp; fp++)
                    (*fp)->output_number = 0;
            }
            else
            {
                mark_faces_inside_r(n->children[0]);
                mark_faces_inside_r(n->children[1]);
            }
        }

        // removes unused nodes
        node *clear_out_faces_r(node *n)
        {
            // mark the node and all its faces, so they can be removed if no
            // children use them
            n->valid = 0; // will be set if any children touch it
            for (face *f = n->faces; f; f = f->next)
                f->output_number = -1;

            if (!n->isportalleaf)
            {
                // decision node
                n->children[0] = clear_out_faces_r(n->children[0]);
                n->children[1] = clear_out_faces_r(n->children[1]);

                // free any faces not in open child leafs
                face *f = n->faces;
                n->faces = nullptr;

                face *fnext;
                for (; f; f = fnext)
                {
                    fnext = f->next;
                    if (f->output_number == -1)
                    {
                        // never referenced, so free it
                        g_fill.c_free_faces++;
                        free_face(f);
                    }
                    else
                    {
                        g_fill.c_keep_faces++;
                        f->next = n->faces;
                        n->faces = f;
                    }
                }

                if (!n->valid)
                {
                    // this node does not touch any interior leafs

                    // if both children are solid, just make this node solid
                    if (n->children[0]->contents == contents_solid
                        && n->children[1]->contents == contents_solid)
                    {
                        n->contents = contents_solid;
                        n->planenum = -1;
                        n->isportalleaf = true;
                        return n;
                    }

                    // if one child is solid, shortcut down the other side
                    if (n->children[0]->contents == contents_solid)
                        return n->children[1];
                    if (n->children[1]->contents == contents_solid)
                        return n->children[0];

                    g_fill.c_falsenodes++;
                }
                return n;
            }

            // leaf node
            if (n->contents != contents_solid)
            {
                // this node is still inside; mark all the nodes used as portals
                for (portal *p = n->portals; p;)
                {
                    if (p->onnode)
                        p->onnode->valid = 1;
                    if (p->nodes[0] == n) // only write out from first leaf
                        p = p->next[0];
                    else
                        p = p->next[1];
                }

                mark_faces_inside_r(n);
                return n;
            }

            return n;
        }

        void reset_mark_r(node *n)
        {
            if (n->isportalleaf)
            {
                if (n->contents == contents_solid || n->contents == contents_sky)
                    n->empty = 0;
                else
                    n->empty = 1;
            }
            else
            {
                reset_mark_r(n->children[0]);
                reset_mark_r(n->children[1]);
            }
        }

        void mark_occupied_r(node *n)
        {
            if (n->empty == 1)
            {
                n->empty = 0;
                int s;
                for (portal *p = n->portals; p; p = p->next[!s])
                {
                    s = (p->nodes[0] == n);
                    mark_occupied_r(p->nodes[s]);
                }
            }
        }

        void remove_unused_r(node *n)
        {
            if (n->isportalleaf)
            {
                if (n->empty == 1)
                    fill_leaf(n);
            }
            else
            {
                remove_unused_r(n->children[0]);
                remove_unused_r(n->children[1]);
            }
        }
    }

    void fill_inside(bsp_state &state, node *n)
    {
        state.outside_node.empty = 0;
        reset_mark_r(n);
        for (size_t i = 1; i < state.entities.size(); i++)
        {
            if (state.entities[i].value("origin")[0])
            {
                math::vec3v origin;
                get_origin_key(state.entities[i], origin);
                origin[2] += 1;
                node *innode = point_in_leaf(state, n, origin);
                mark_occupied_r(innode);
                origin[2] -= 2;
                innode = point_in_leaf(state, n, origin);
                mark_occupied_r(innode);
            }
        }
        remove_unused_r(n);
    }

    node *fill_outside(bsp_state &state, node *n, bool leakfile, int hullnum)
    {
        if (state.options.nofill)
        {
            logging::info("skipped\n");
            return n;
        }
        if (hullnum == 2 && state.options.nohull2)
            return n;

        // place markers for all entities so we know if we leak inside
        bool inside = false;
        for (int i = 1; i < (int)state.entities.size(); i++)
        {
            const format::entity &ent = state.entities[(size_t)i];
            math::vec3v origin;
            get_origin_key(ent, origin);
            const char *cl = ent.value("classname");
            if (ent.value("origin")[0])
            {
                origin[2] += 1; // so objects on the floor are ok

                // nudge playerstart around if needed so clipping hulls always
                // have a valid point
                if (std::strcmp(cl, "info_player_start") == 0)
                {
                    for (int x = -16; x <= 16 && !inside; x += 16)
                    {
                        for (int y = -16; y <= 16; y += 16)
                        {
                            origin[0] += x;
                            origin[1] += y;
                            if (place_occupant(state, i, origin, n))
                            {
                                inside = true;
                                break;
                            }
                            origin[0] -= x;
                            origin[1] -= y;
                        }
                    }
                }
                else
                {
                    if (place_occupant(state, i, origin, n))
                        inside = true;
                }
            }
        }

        if (!inside)
        {
            logging::warn("No entities exist in hull %i, no filling performed for this hull", hullnum);
            return n;
        }

        if (!state.outside_node.portals)
        {
            logging::warn("No outside node portal found in hull %i, no filling performed for this hull", hullnum);
            return n;
        }

        int s = !(state.outside_node.portals->nodes[1] == &state.outside_node);

        // first check to see if an occupied leaf is hit
        g_fill.outleafs = 0;
        g_fill.valid++;

        g_fill.haveleaktrail = false;
        g_fill.occupiednode = nullptr;
        g_fill.leaktrail.clear();

        bool ret = recursive_fill_outside(state, state.outside_node.portals->nodes[s], false);

        if (leakfile && ret)
        {
            // build the shortest leak path (entity -> hole) and write the
            // clean line plus marker
            math::vec3v startpos;
            get_origin_key(state.entities[(size_t)g_fill.hit_occupied], startpos);
            build_shortest_leak_trail(state, g_fill.occupiednode, startpos);
            write_leak_files(state.base_path + ".pts", state.base_path + ".lin");
            // the survey rewrites the same files with every hole's path, so it
            // runs after the single trail and simply supersedes it
            if (state.options.allleaks)
                survey_leaks(state, state.base_path + ".pts", state.base_path + ".lin");
        }

        if (ret)
        {
            // collect the leak; print_leak_summary shows one block at the end
            // the first leak owns the pointfile
            if (!g_fill.leak_any)
            {
                str::copy(g_fill.leak_entclass, sizeof(g_fill.leak_entclass),
                          state.entities[(size_t)g_fill.hit_occupied].value("classname"));
                get_origin_key(state.entities[(size_t)g_fill.hit_occupied], g_fill.leak_entorigin);
                if (g_fill.haveleaktrail)
                {
                    g_fill.leak_hole = g_fill.leakexit;
                    g_fill.leak_hashole = true;
                }
            }
            g_fill.leak_any = true;
            if (hullnum >= 0 && hullnum < num_hulls)
                g_fill.leak_hulls |= (1 << hullnum);

            if (state.options.leakonly)
            {
                print_leaf_content_conflicts(state);
                print_leak_summary();
                err::fatal("stopped by leak");
            }

            state.leaked = true;

            return n;
        }
        if (leakfile && !ret)
        {
            std::remove((state.base_path + ".lin").c_str());
            std::remove((state.base_path + ".pts").c_str());
        }

        // now go back and fill things in
        g_fill.valid++;
        recursive_fill_outside(state, state.outside_node.portals->nodes[s], true);

        // remove faces and nodes from filled in leafs
        g_fill.c_falsenodes = 0;
        g_fill.c_free_faces = 0;
        g_fill.c_keep_faces = 0;
        n = clear_out_faces_r(n);

        // save portal file for vis tracing
        if (hullnum == 0 && leakfile)
            write_portal_file(state, n);

        return n;
    }

    // one consolidated leak block after all hulls, listing every hull that
    // leaked plus the entity and hole from the first leak
    void print_leak_summary()
    {
        if (!g_fill.leak_any)
            return;

        char hulls[64];
        hulls[0] = '\0';
        int count = 0;
        for (int h = 0; h < num_hulls; h++)
        {
            if (g_fill.leak_hulls & (1 << h))
            {
                char tmp[8];
                str::format(tmp, sizeof(tmp), "%s%d", count ? ", " : "", h);
                size_t len = std::strlen(hulls);
                str::copy(hulls + len, sizeof(hulls) - len, tmp);
                count++;
            }
        }

        logging::info("\n  !!! LEAK - map is not sealed (hull%s %s)\n", count == 1 ? "" : "s", hulls);

        if (g_fill.sites.size() > 1)
        {
            // -allleaks: every hole, so the whole set can be sealed in one pass
            // through the editor instead of one hole per compile
            logging::info("    found   %zu holes; all of their paths are in the pointfile\n",
                          g_fill.sites.size());
            size_t index = 0;
            for (const leak_site &site : g_fill.sites)
            {
                index++;
                logging::info("\n    hole %-3zu (%.0f, %.0f, %.0f)\n", index,
                              (double)site.hole[0], (double)site.hole[1],
                              (double)site.hole[2]);
                logging::info("      from  %s @ (%.0f, %.0f, %.0f)\n",
                              site.classname.c_str(), (double)site.origin[0],
                              (double)site.origin[1], (double)site.origin[2]);
            }
            logging::info("\n    action  load the pointfile and seal every marked hole\n");
        }
        else
        {
            logging::info("    entity  %s @ (%.0f, %.0f, %.0f)\n",
                          g_fill.leak_entclass, (double)g_fill.leak_entorigin[0],
                          (double)g_fill.leak_entorigin[1], (double)g_fill.leak_entorigin[2]);
            if (g_fill.leak_hashole)
            {
                logging::info("    hole    (%.0f, %.0f, %.0f)   marked in the pointfile\n",
                              (double)g_fill.leak_hole[0], (double)g_fill.leak_hole[1],
                              (double)g_fill.leak_hole[2]);
            }
            logging::info("    action  load the pointfile in your editor and seal the marked hole\n");
            if (!g_fill.sites.empty())
                logging::info("    note    -allleaks surveyed the map and found only this one\n");
        }

        logging::file(
            "\n  A LEAK is a hole in the map where the inside is exposed to the outside void.\n"
            "  The listed entity is where the leak trace starts; trace the pointfile from it to\n"
            "  the marked hole and seal the gap. Unless the entity is accidentally outside the map,\n"
            "  do not delete it. Some rotating-object entities need their origin outside the map;\n"
            "  enclose such an origin brush in a solid world brush.\n");

        logging::set_leaked();
    }
}
