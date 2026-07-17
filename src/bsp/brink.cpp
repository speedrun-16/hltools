#include "internal.h"

#include <cstring>
#include <list>
#include <map>
#include <utility>
#include <vector>

#include "../common/error.h"
#include "../common/limits.h"
#include "../common/log.h"

// the brink pass: rebuilds each model's clip hulls as an exact cell complex,
// finds convex "brink" edges a player can catch on, and inserts partition
// planes into the leafs so movement across the brink stays smooth traversal
// and merging order remain deterministic so the clipnode lump is reproducible

namespace bsp
{
    namespace
    {
        // brink fix levels, most important first (lowest value); the driver
        // retries at ever weaker levels when the clipnode lump runs out
        constexpr int brink_none = 0;
        constexpr int brink_floor_blocking = 1;
        constexpr int brink_floor = 2;
        constexpr int brink_wall_blocking = 3;
        constexpr int brink_wall = 4;
        constexpr int brink_any = 5;

        constexpr int side_front = math::winding::side_front;
        constexpr int side_back = math::winding::side_back;
        constexpr int side_on = math::winding::side_on;
        constexpr int side_cross = math::winding::side_cross;

        struct bpartition
        {
            int planenum = 0;
            bool planeside = false;
            int content = 0;
            int type = brink_none;

            bpartition *next = nullptr;
        };

        struct btreeleaf;

        struct bclipnode
        {
            bool isleaf = false;

            int planenum = 0;
            const plane *plane_ = nullptr;
            bclipnode *children[2] = {};

            int content = 0;
            bpartition *partitions = nullptr;

            btreeleaf *treeleaf = nullptr;
        };

        // the clipnodes that meet a brink, extracted into a local bsp shape
        struct bbrinknode
        {
            bool isleaf = false;

            int planenum = 0;
            const plane *plane_ = nullptr;
            int children[2] = {};

            int content = 0;
            bclipnode *clipnode = nullptr;
        };

        struct btreeedge;

        struct bbrink
        {
            math::vec3v start, stop;
            math::vec3v direction;

            int numnodes = 0; // including both nodes and leafs
            std::vector<bbrinknode> *nodes = nullptr;

            btreeedge *edge = nullptr; // only for deciding the brink type
        };

        bbrink *copy_brink(bbrink *other)
        {
            bbrink *b = new bbrink();
            b->direction = other->direction;
            b->start = other->start;
            b->stop = other->stop;
            b->numnodes = other->numnodes;
            b->nodes = new std::vector<bbrinknode>(*other->nodes);
            return b;
        }

        void delete_brink(bbrink *b)
        {
            delete b->nodes;
            delete b;
        }

        bbrink *create_brink(const math::vec3v &start, const math::vec3v &stop)
        {
            bbrink *b = new bbrink();

            b->start = start;
            b->stop = stop;
            math::subtract(stop, start, b->direction);

            b->numnodes = 1;
            b->nodes = new std::vector<bbrinknode>;
            bbrinknode newnode;
            newnode.isleaf = true;
            newnode.clipnode = nullptr;
            b->nodes->push_back(newnode);

            // create_brink must be followed by brink_split_clipnode
            return b;
        }

        void brink_split_clipnode(bbrink *b, const plane *split, int planenum,
                                  bclipnode *prev, bclipnode *n0, bclipnode *n1)
        {
            int found = -1;
            int numfound = 0;
            for (int i = 0; i < b->numnodes; i++)
            {
                bbrinknode *node = &(*b->nodes)[(size_t)i];
                if (node->isleaf && node->clipnode == prev)
                {
                    found = i;
                    numfound++;
                }
            }
            if (numfound == 0)
                err::fatal("brink_split_clipnode: internal error: couldn't find clipnode");
            if (numfound > 1)
                err::fatal("brink_split_clipnode: internal error: found more than one clipnode");
            if (n0 == n1)
                err::fatal("brink_split_clipnode: internal error: n0 == n1");
            b->nodes->resize((size_t)b->numnodes + 2);
            bbrinknode *node = &(*b->nodes)[(size_t)found];
            bbrinknode *front = &(*b->nodes)[(size_t)b->numnodes];
            bbrinknode *back = &(*b->nodes)[(size_t)b->numnodes + 1];

            node->clipnode = nullptr;
            node->isleaf = false;
            node->plane_ = split;
            node->planenum = planenum;
            node->children[0] = b->numnodes;
            node->children[1] = b->numnodes + 1;

            front->isleaf = true;
            front->content = n0->content;
            front->clipnode = n0;

            back->isleaf = true;
            back->content = n1->content;
            back->clipnode = n1;

            b->numnodes += 2;
        }

        void brink_replace_clipnode(bbrink *b, bclipnode *prev, bclipnode *n)
        {
            int found = -1;
            int numfound = 0;
            for (int i = 0; i < b->numnodes; i++)
            {
                bbrinknode *node = &(*b->nodes)[(size_t)i];
                if (node->isleaf && node->clipnode == prev)
                {
                    found = i;
                    numfound++;
                }
            }
            if (numfound == 0)
                err::fatal("brink_replace_clipnode: internal error: couldn't find clipnode");
            if (numfound > 1)
                err::fatal("brink_replace_clipnode: internal error: found more than one clipnode");
            bbrinknode *node = &(*b->nodes)[(size_t)found];
            node->clipnode = n;
            node->content = n->content;
        }

        // the exact cell complex of the whole clip hull bsp:
        // 0d points, 1d edges, 2d faces, 3d leafs, all doubly referenced

        struct btreepoint;
        struct btreeface;

        struct btreeedge_r
        {
            btreeedge *e = nullptr;
            bool side = false;
        };

        struct btreeface_r
        {
            btreeface *f = nullptr;
            bool side = false;
        };

        struct btreeleaf_r
        {
            btreeleaf *l = nullptr;
            bool side = false;
        };

        using btreeedge_l = std::list<btreeedge_r>;
        using btreeface_l = std::list<btreeface_r>;

        struct btreepoint
        {
            math::vec3v v;
            bool infinite = false;

            btreeedge_l *edges = nullptr; // reversed reference

            bool tmp_tested = false;
            vec_t tmp_dist = 0;
            int tmp_side = 0;
        };

        struct btreepoint_r
        {
            btreepoint *p = nullptr;
            bool side = false;
        };

        struct btreeedge
        {
            btreepoint_r points[2]; // pointing from points[1] to points[0]
            bool infinite = false;  // both points lie on the bounding box

            btreeface_l *faces = nullptr; // reversed reference

            bbrink *brink = nullptr; // not defined for infinite edges

            bool tmp_tested = false;
            int tmp_side = 0;
            bool tmp_onleaf[2] = {};
        };

        struct btreeface
        {
            // empty faces are allowed to preserve topological correctness
            btreeedge_l *edges = nullptr;
            bool infinite = false; // all edges must then be infinite too

            btreeleaf_r leafs[2]; // pointing from leafs[0] to leafs[1]

            const plane *plane_ = nullptr; // not defined for infinite faces
            int planenum = 0;
            bool planeside = false; // when true, faces -plane->normal

            bool tmp_tested = false;
            int tmp_side = 0;
        };

        struct btreeleaf
        {
            btreeface_l *faces = nullptr;
            bool infinite = false; // the infinite leaf is not convex

            bclipnode *clipnode = nullptr; // not defined for the infinite leaf
        };

        btreepoint *alloc_treepoint(int &numobjects, bool infinite)
        {
            numobjects++;
            btreepoint *tp = new btreepoint();
            tp->edges = new btreeedge_l();
            tp->infinite = infinite;
            return tp;
        }

        btreeedge *alloc_treeedge(int &numobjects, bool infinite)
        {
            numobjects++;
            btreeedge *te = new btreeedge();
            te->points[0].p = nullptr;
            te->points[0].side = false;
            te->points[1].p = nullptr;
            te->points[1].side = true;
            te->faces = new btreeface_l();
            te->infinite = infinite;
            // should be followed by set_edge_points
            return te;
        }

        void attach_point_to_edge(btreeedge *te, btreepoint *tp, bool side)
        {
            if (te->points[side].p)
                err::fatal("attach_point_to_edge: internal error: point occupied");
            if (te->infinite && !tp->infinite)
                err::fatal("attach_point_to_edge: internal error: attaching a finite object to an infinite object");
            te->points[side].p = tp;

            btreeedge_r er;
            er.e = te;
            er.side = side;
            tp->edges->push_back(er);
        }

        void set_edge_points(btreeedge *te, btreepoint *tp0, btreepoint *tp1)
        {
            attach_point_to_edge(te, tp0, false);
            attach_point_to_edge(te, tp1, true);
        }

        btreeface *alloc_treeface(int &numobjects, bool infinite)
        {
            numobjects++;
            btreeface *tf = new btreeface();
            tf->edges = new btreeedge_l();
            tf->leafs[0].l = nullptr;
            tf->leafs[0].side = false;
            tf->leafs[1].l = nullptr;
            tf->leafs[1].side = true;
            tf->infinite = infinite;
            return tf;
        }

        void attach_edge_to_face(btreeface *tf, btreeedge *te, bool side)
        {
            if (tf->infinite && !te->infinite)
                err::fatal("attach_edge_to_face: internal error: attaching a finite object to an infinite object");
            btreeedge_r er;
            er.e = te;
            er.side = side;
            tf->edges->push_back(er);

            btreeface_r fr;
            fr.f = tf;
            fr.side = side;
            te->faces->push_back(fr);
        }

        void attach_face_to_leaf(btreeleaf *tl, btreeface *tf, bool side)
        {
            if (tl->infinite && !tf->infinite)
                err::fatal("attach_face_to_leaf: internal error: attaching a finite object to an infinite object");
            btreeface_r fr;
            fr.f = tf;
            fr.side = side;
            tl->faces->push_back(fr);

            if (tf->leafs[side].l)
                err::fatal("attach_face_to_leaf: internal error: leaf occupied");
            tf->leafs[side].l = tl;
        }

        void set_face_leafs(btreeface *tf, btreeleaf *tl0, btreeleaf *tl1)
        {
            attach_face_to_leaf(tl0, tf, false);
            attach_face_to_leaf(tl1, tf, true);
        }

        btreeleaf *alloc_treeleaf(int &numobjects, bool infinite)
        {
            numobjects++;
            btreeleaf *tl = new btreeleaf();
            tl->faces = new btreeface_l();
            tl->infinite = infinite;
            return tl;
        }

        btreeleaf *build_outside(int &numobjects)
        {
            btreeleaf *leaf_outside = alloc_treeleaf(numobjects, true);
            leaf_outside->clipnode = nullptr;
            return leaf_outside;
        }

        btreeleaf *build_base_cell(int &numobjects, bclipnode *clipnode, vec_t range,
                                   btreeleaf *leaf_outside)
        {
            btreepoint *tp[8];
            for (int i = 0; i < 8; i++)
            {
                tp[i] = alloc_treepoint(numobjects, true);
                tp[i]->v[0] = (i & 1) ? range : -range;
                tp[i]->v[1] = (i & 2) ? range : -range;
                tp[i]->v[2] = (i & 4) ? range : -range;
            }
            btreeedge *te[12];
            for (int i = 0; i < 12; i++)
                te[i] = alloc_treeedge(numobjects, true);
            set_edge_points(te[0], tp[1], tp[0]);
            set_edge_points(te[1], tp[3], tp[2]);
            set_edge_points(te[2], tp[5], tp[4]);
            set_edge_points(te[3], tp[7], tp[6]);
            set_edge_points(te[4], tp[2], tp[0]);
            set_edge_points(te[5], tp[3], tp[1]);
            set_edge_points(te[6], tp[6], tp[4]);
            set_edge_points(te[7], tp[7], tp[5]);
            set_edge_points(te[8], tp[4], tp[0]);
            set_edge_points(te[9], tp[5], tp[1]);
            set_edge_points(te[10], tp[6], tp[2]);
            set_edge_points(te[11], tp[7], tp[3]);
            btreeface *tf[6];
            for (int i = 0; i < 6; i++)
                tf[i] = alloc_treeface(numobjects, true);
            attach_edge_to_face(tf[0], te[4], true);
            attach_edge_to_face(tf[0], te[6], false);
            attach_edge_to_face(tf[0], te[8], false);
            attach_edge_to_face(tf[0], te[10], true);
            attach_edge_to_face(tf[1], te[5], false);
            attach_edge_to_face(tf[1], te[7], true);
            attach_edge_to_face(tf[1], te[9], true);
            attach_edge_to_face(tf[1], te[11], false);
            attach_edge_to_face(tf[2], te[0], false);
            attach_edge_to_face(tf[2], te[2], true);
            attach_edge_to_face(tf[2], te[8], true);
            attach_edge_to_face(tf[2], te[9], false);
            attach_edge_to_face(tf[3], te[1], true);
            attach_edge_to_face(tf[3], te[3], false);
            attach_edge_to_face(tf[3], te[10], false);
            attach_edge_to_face(tf[3], te[11], true);
            attach_edge_to_face(tf[4], te[0], true);
            attach_edge_to_face(tf[4], te[1], false);
            attach_edge_to_face(tf[4], te[4], false);
            attach_edge_to_face(tf[4], te[5], true);
            attach_edge_to_face(tf[5], te[2], false);
            attach_edge_to_face(tf[5], te[3], true);
            attach_edge_to_face(tf[5], te[6], true);
            attach_edge_to_face(tf[5], te[7], false);
            btreeleaf *tl = alloc_treeleaf(numobjects, false);
            for (int i = 0; i < 6; i++)
                set_face_leafs(tf[i], tl, leaf_outside);
            tl->clipnode = clipnode;
            return tl;
        }

        btreepoint *get_point_from_edge(btreeedge *te, bool side)
        {
            if (!te->points[side].p)
                err::fatal("get_point_from_edge: internal error: point not set");
            return te->points[side].p;
        }

        void remove_edge_from_list(btreeedge_l *el, btreeedge *te, bool side)
        {
            for (auto ei = el->begin(); ei != el->end(); ++ei)
            {
                if (ei->e == te && ei->side == side)
                {
                    el->erase(ei);
                    // only remove one copy if there are many, to preserve
                    // topological correctness
                    return;
                }
            }
            err::fatal("remove_edge_from_list: internal error: edge not found");
        }

        // the point is not freed
        void remove_point_from_edge(btreeedge *te, btreepoint *tp, bool side)
        {
            if (te->points[side].p != tp)
                err::fatal("remove_point_from_edge: internal error: point not found");
            te->points[side].p = nullptr;

            remove_edge_from_list(tp->edges, te, side);
        }

        void delete_point(int &numobjects, btreepoint *tp)
        {
            if (!tp->edges->empty())
                err::fatal("delete_point: internal error: point used by edge");
            delete tp->edges;
            delete tp;
            numobjects--;
        }

        void remove_face_from_list(btreeface_l *fl, btreeface *tf, bool side)
        {
            for (auto fi = fl->begin(); fi != fl->end(); ++fi)
            {
                if (fi->f == tf && fi->side == side)
                {
                    fl->erase(fi);
                    return;
                }
            }
            err::fatal("remove_face_from_list: internal error: face not found");
        }

        void remove_edge_from_face(btreeface *tf, btreeedge *te, bool side)
        {
            remove_edge_from_list(tf->edges, te, side);
            remove_face_from_list(te->faces, tf, side);
        }

        // points in this edge are freed when no other edge references them
        void delete_edge(int &numobjects, btreeedge *te)
        {
            if (!te->faces->empty())
                err::fatal("delete_edge: internal error: edge used by face");
            if (!te->infinite)
                delete_brink(te->brink);
            for (int side = 0; side < 2; side++)
            {
                btreepoint *tp = get_point_from_edge(te, side != 0);
                remove_point_from_edge(te, tp, side != 0);
                if (tp->edges->empty())
                    delete_point(numobjects, tp);
            }
            delete te->faces;
            delete te;
            numobjects--;
        }

        btreeleaf *get_leaf_from_face(btreeface *tf, bool side)
        {
            if (!tf->leafs[side].l)
                err::fatal("get_leaf_from_face: internal error: leaf not set");
            return tf->leafs[side].l;
        }

        void remove_face_from_leaf(btreeleaf *tl, btreeface *tf, bool side)
        {
            if (tf->leafs[side].l != tl)
                err::fatal("remove_face_from_leaf: internal error: leaf not found");
            tf->leafs[side].l = nullptr;

            remove_face_from_list(tl->faces, tf, side);
        }

        // edges in this face are freed when no other face references them
        void delete_face(int &numobjects, btreeface *tf)
        {
            btreeedge_l::iterator ei;
            while ((ei = tf->edges->begin()) != tf->edges->end())
            {
                btreeedge *te = ei->e;
                remove_face_from_list(te->faces, tf, ei->side);
                tf->edges->erase(ei);
                if (te->faces->empty())
                    delete_edge(numobjects, te);
            }
            for (int side = 0; side < 2; side++)
            {
                if (tf->leafs[side].l)
                    err::fatal("delete_face: internal error: face used by leaf");
            }
            delete tf->edges;
            delete tf;
            numobjects--;
        }

        void delete_leaf(int &numobjects, btreeleaf *tl)
        {
            btreeface_l::iterator fi;
            while ((fi = tl->faces->begin()) != tl->faces->end())
            {
                btreeface *tf = fi->f;
                remove_face_from_leaf(tl, tf, fi->side);
                if (!tf->leafs[false].l && !tf->leafs[true].l)
                    delete_face(numobjects, tf);
            }
            delete tl->faces;
            delete tl;
            numobjects--;
        }

        void split_tree_leaf(int &numobjects, btreeleaf *tl, const plane *split,
                             int planenum, vec_t epsilon, btreeleaf *&front, btreeleaf *&back,
                             bclipnode *c0, bclipnode *c1)
        {
            btreeface_l::iterator fi;
            btreeedge_l::iterator ei;
            bool restart = false;

            // clear all the flags
            for (fi = tl->faces->begin(); fi != tl->faces->end(); ++fi)
            {
                btreeface *tf = fi->f;
                tf->tmp_tested = false;
                for (ei = tf->edges->begin(); ei != tf->edges->end(); ++ei)
                {
                    btreeedge *te = ei->e;
                    te->tmp_tested = false;
                    for (int side = 0; side < 2; side++)
                        get_point_from_edge(te, side != 0)->tmp_tested = false;
                }
            }

            // split each point
            for (fi = tl->faces->begin(); fi != tl->faces->end(); ++fi)
            {
                btreeface *tf = fi->f;
                for (ei = tf->edges->begin(); ei != tf->edges->end(); ++ei)
                {
                    btreeedge *te = ei->e;
                    for (int side = 0; side < 2; side++)
                    {
                        btreepoint *tp = get_point_from_edge(te, side != 0);
                        if (tp->tmp_tested)
                            continue;
                        tp->tmp_tested = true;
                        vec_t dist = math::dot(tp->v, split->normal) - split->dist;
                        tp->tmp_dist = dist;
                        if (dist > epsilon)
                            tp->tmp_side = side_front;
                        else if (dist < -epsilon)
                            tp->tmp_side = side_back;
                        else
                            tp->tmp_side = side_on;
                    }
                }
            }

            // split each edge
            for (fi = tl->faces->begin(); fi != tl->faces->end(); ++fi)
            {
                btreeface *tf = fi->f;
                for (ei = tf->edges->begin(); ei != tf->edges->end();
                     restart ? (restart = false, ei = tf->edges->begin()) : ++ei)
                {
                    btreeedge *te = ei->e;
                    if (te->tmp_tested) // already split
                        continue;
                    te->tmp_tested = true;
                    te->tmp_side = side_on;
                    for (int side = 0; side < 2; side++)
                    {
                        btreepoint *tp = get_point_from_edge(te, side != 0);
                        if (te->tmp_side == side_on)
                            te->tmp_side = tp->tmp_side;
                        else if (tp->tmp_side != side_on && tp->tmp_side != te->tmp_side)
                            te->tmp_side = side_cross;
                    }
                    // the plane does not necessarily split the leaf in two
                    // because of epsilon problems; splitting the brink leafs
                    // now would break the integrity of the geometry, so the
                    // four passes stay independent
                    if (te->tmp_side == side_cross)
                    {
                        btreepoint *tp0 = get_point_from_edge(te, false);
                        btreepoint *tp1 = get_point_from_edge(te, true);
                        btreepoint *tpmid = alloc_treepoint(numobjects, te->infinite);
                        tpmid->tmp_tested = true;
                        tpmid->tmp_dist = 0;
                        tpmid->tmp_side = side_on;
                        vec_t frac = tp0->tmp_dist / (tp0->tmp_dist - tp1->tmp_dist);
                        for (int k = 0; k < 3; k++)
                            tpmid->v[k] = tp0->v[k] + frac * (tp1->v[k] - tp0->v[k]);
                        btreeedge *te0 = alloc_treeedge(numobjects, te->infinite);
                        set_edge_points(te0, tp0, tpmid);
                        te0->tmp_tested = true;
                        te0->tmp_side = tp0->tmp_side;
                        if (!te0->infinite)
                        {
                            te0->brink = copy_brink(te->brink);
                            te0->brink->start = tpmid->v;
                            te0->brink->stop = tp0->v;
                        }
                        btreeedge *te1 = alloc_treeedge(numobjects, te->infinite);
                        set_edge_points(te1, tpmid, tp1);
                        te1->tmp_tested = true;
                        te1->tmp_side = tp1->tmp_side;
                        if (!te1->infinite)
                        {
                            te1->brink = copy_brink(te->brink);
                            te1->brink->start = tp1->v;
                            te1->brink->stop = tpmid->v;
                        }
                        btreeface_l::iterator fj;
                        while ((fj = te->faces->begin()) != te->faces->end())
                        {
                            attach_edge_to_face(fj->f, te0, fj->side);
                            attach_edge_to_face(fj->f, te1, fj->side);
                            remove_edge_from_face(fj->f, te, fj->side);
                        }
                        delete_edge(numobjects, te);
                        restart = true;
                    }
                }
            }

            // split each face
            for (fi = tl->faces->begin(); fi != tl->faces->end();
                 restart ? (restart = false, fi = tl->faces->begin()) : ++fi)
            {
                btreeface *tf = fi->f;
                if (tf->tmp_tested)
                    continue;
                tf->tmp_tested = true;
                tf->tmp_side = side_on;
                for (ei = tf->edges->begin(); ei != tf->edges->end(); ++ei)
                {
                    if (tf->tmp_side == side_on)
                        tf->tmp_side = ei->e->tmp_side;
                    else if (ei->e->tmp_side != side_on && ei->e->tmp_side != tf->tmp_side)
                        tf->tmp_side = side_cross;
                }
                if (tf->tmp_side == side_cross)
                {
                    btreeface *frontface = alloc_treeface(numobjects, tf->infinite);
                    if (!tf->infinite)
                    {
                        frontface->plane_ = tf->plane_;
                        frontface->planenum = tf->planenum;
                        frontface->planeside = tf->planeside;
                    }
                    set_face_leafs(frontface, get_leaf_from_face(tf, false),
                                   get_leaf_from_face(tf, true));
                    frontface->tmp_tested = true;
                    frontface->tmp_side = side_front;
                    btreeface *backface = alloc_treeface(numobjects, tf->infinite);
                    if (!tf->infinite)
                    {
                        backface->plane_ = tf->plane_;
                        backface->planenum = tf->planenum;
                        backface->planeside = tf->planeside;
                    }
                    set_face_leafs(backface, get_leaf_from_face(tf, false),
                                   get_leaf_from_face(tf, true));
                    backface->tmp_tested = true;
                    backface->tmp_side = side_back;

                    std::map<btreepoint *, int> vertexes;
                    std::map<btreepoint *, int>::iterator vertex, vertex2;
                    for (ei = tf->edges->begin(); ei != tf->edges->end(); ++ei)
                    {
                        if (ei->e->tmp_side != side_back)
                        {
                            attach_edge_to_face(frontface, ei->e, ei->side);
                        }
                        else
                        {
                            attach_edge_to_face(backface, ei->e, ei->side);

                            btreeedge *e = ei->e;
                            for (int side = 0; side < 2; side++)
                            {
                                btreepoint *p = get_point_from_edge(e, side != 0);
                                // the default value is 0 when the key is new
                                vertexes[p] += ((side != 0) == ei->side ? 1 : -1);
                                vertex = vertexes.find(p);
                                if (vertex->second == 0)
                                    vertexes.erase(vertex);
                            }
                        }
                    }

                    while (true)
                    {
                        for (vertex = vertexes.begin(); vertex != vertexes.end(); ++vertex)
                        {
                            if (vertex->second > 0)
                                break;
                        }
                        for (vertex2 = vertexes.begin(); vertex2 != vertexes.end(); ++vertex2)
                        {
                            if (vertex2->second < 0)
                                break;
                        }
                        if (vertex == vertexes.end() && vertex2 == vertexes.end())
                            break;
                        if (vertex == vertexes.end() || vertex2 == vertexes.end())
                            err::fatal("split_tree_leaf: internal error: couldn't link edges");
                        if (vertex->first->tmp_side != side_on
                            || vertex2->first->tmp_side != side_on)
                        {
                            err::fatal("split_tree_leaf: internal error: tmp_side != side_on");
                        }

                        btreeedge *te = alloc_treeedge(numobjects, tf->infinite);
                        set_edge_points(te, vertex->first, vertex2->first);
                        if (!te->infinite)
                        {
                            te->brink = create_brink(vertex2->first->v, vertex->first->v);
                            if (get_leaf_from_face(tf, tf->planeside)->infinite
                                || get_leaf_from_face(tf, !tf->planeside)->infinite)
                            {
                                err::fatal("split_tree_leaf: internal error: an infinite object contains a finite object");
                            }
                            brink_split_clipnode(te->brink, tf->plane_, tf->planenum, nullptr,
                                                 get_leaf_from_face(tf, tf->planeside)->clipnode,
                                                 get_leaf_from_face(tf, !tf->planeside)->clipnode);
                        }
                        te->tmp_tested = true;
                        te->tmp_side = side_on;
                        attach_edge_to_face(frontface, te, false);
                        attach_edge_to_face(backface, te, true);

                        vertex->second--;
                        vertex2->second++;
                    }

                    for (int side = 0; side < 2; side++)
                        remove_face_from_leaf(get_leaf_from_face(tf, side != 0), tf, side != 0);
                    delete_face(numobjects, tf);
                    restart = true;
                }
            }

            // split the leaf
            {
                if (tl->infinite)
                    err::fatal("split_tree_leaf: internal error: splitting the infinite leaf");
                front = alloc_treeleaf(numobjects, tl->infinite);
                back = alloc_treeleaf(numobjects, tl->infinite);
                front->clipnode = c0;
                back->clipnode = c1;

                int tmp_side = side_on;
                for (fi = tl->faces->begin(); fi != tl->faces->end(); ++fi)
                {
                    if (tmp_side == side_on)
                        tmp_side = fi->f->tmp_side;
                    else if (fi->f->tmp_side != side_on && fi->f->tmp_side != tmp_side)
                        tmp_side = side_cross;
                }

                std::map<btreeedge *, int> edges;
                std::map<btreeedge *, int>::iterator edge;

                while ((fi = tl->faces->begin()) != tl->faces->end())
                {
                    btreeface *tf = fi->f;
                    bool side = fi->side;
                    // a face can only store two leafs
                    remove_face_from_leaf(tl, tf, side);

                    // fi is unusable now
                    if (tf->tmp_side == side_front
                        || (tf->tmp_side == side_on && tmp_side != side_back))
                    {
                        attach_face_to_leaf(front, tf, side);
                    }
                    else if (tf->tmp_side == side_back
                             || (tf->tmp_side == side_on && tmp_side == side_back))
                    {
                        attach_face_to_leaf(back, tf, side);

                        if (tmp_side == side_cross)
                        {
                            for (ei = tf->edges->begin(); ei != tf->edges->end(); ++ei)
                            {
                                edges[ei->e] += (ei->side == side ? 1 : -1);
                                edge = edges.find(ei->e);
                                if (edge->second == 0)
                                    edges.erase(edge);
                            }
                        }
                    }
                }

                if (tmp_side == side_cross)
                {
                    btreeface *tf = alloc_treeface(numobjects, tl->infinite);
                    if (!tf->infinite)
                    {
                        tf->plane_ = split;
                        tf->planenum = planenum;
                        tf->planeside = false;
                    }
                    tf->tmp_tested = true;
                    tf->tmp_side = side_on;
                    set_face_leafs(tf, front, back);
                    for (edge = edges.begin(); edge != edges.end(); ++edge)
                    {
                        if (edge->first->tmp_side != side_on)
                            err::fatal("split_tree_leaf: internal error");
                        while (edge->second > 0)
                        {
                            attach_edge_to_face(tf, edge->first, false);
                            edge->second--;
                        }
                        while (edge->second < 0)
                        {
                            attach_edge_to_face(tf, edge->first, true);
                            edge->second++;
                        }
                    }
                }

                btreeleaf *frontback[2] = {front, back};
                for (int side = 0; side < 2; side++)
                {
                    for (fi = frontback[side]->faces->begin();
                         fi != frontback[side]->faces->end(); ++fi)
                    {
                        for (ei = fi->f->edges->begin(); ei != fi->f->edges->end(); ++ei)
                        {
                            ei->e->tmp_onleaf[0] = ei->e->tmp_onleaf[1] = false;
                            ei->e->tmp_tested = false;
                        }
                    }
                }
                for (int side = 0; side < 2; side++)
                {
                    for (fi = frontback[side]->faces->begin();
                         fi != frontback[side]->faces->end(); ++fi)
                    {
                        for (ei = fi->f->edges->begin(); ei != fi->f->edges->end(); ++ei)
                            ei->e->tmp_onleaf[side] = true;
                    }
                }
                for (int side = 0; side < 2; side++)
                {
                    for (fi = frontback[side]->faces->begin();
                         fi != frontback[side]->faces->end(); ++fi)
                    {
                        for (ei = fi->f->edges->begin(); ei != fi->f->edges->end(); ++ei)
                        {
                            if (ei->e->tmp_tested)
                                continue;
                            ei->e->tmp_tested = true;
                            if (ei->e->infinite)
                                continue;
                            if (ei->e->tmp_onleaf[0] && ei->e->tmp_onleaf[1])
                            {
                                if (ei->e->tmp_side != side_on)
                                    err::fatal("split_tree_leaf: internal error");
                                brink_split_clipnode(ei->e->brink, split, planenum,
                                                     tl->clipnode, c0, c1);
                            }
                            else if (ei->e->tmp_onleaf[0])
                            {
                                if (ei->e->tmp_side == side_back)
                                    err::fatal("split_tree_leaf: internal error");
                                brink_replace_clipnode(ei->e->brink, tl->clipnode, c0);
                            }
                            else if (ei->e->tmp_onleaf[1])
                            {
                                if (ei->e->tmp_side == side_front)
                                    err::fatal("split_tree_leaf: internal error");
                                brink_replace_clipnode(ei->e->brink, tl->clipnode, c1);
                            }
                        }
                    }
                }
                delete_leaf(numobjects, tl);
            }
        }

        void build_tree_cells_r(int &numobjects, bclipnode *c)
        {
            if (c->isleaf)
                return;
            btreeleaf *tl = c->treeleaf;
            btreeleaf *front;
            btreeleaf *back;
            split_tree_leaf(numobjects, tl, c->plane_, c->planenum, (vec_t)math::on_epsilon,
                            front, back, c->children[0], c->children[1]);
            c->treeleaf = nullptr;
            c->children[0]->treeleaf = front;
            c->children[1]->treeleaf = back;
            build_tree_cells_r(numobjects, c->children[0]);
            build_tree_cells_r(numobjects, c->children[1]);
        }

        struct bbrinkinfo
        {
            int numclipnodes = 0;
            bclipnode *clipnodes = nullptr;
            int numobjects = 0;
            btreeleaf *leaf_outside = nullptr;
            int numbrinks = 0;
            bbrink **brinks = nullptr;
        };

        constexpr int max_expanded_clipnodes = limits::max_map_clipnodes * 8;

        bclipnode *expand_clipnodes_r(const bsp_state &state, bclipnode *bclipnodes,
                                      int &numbclipnodes,
                                      const format::dclipnode_t *clipnodes, int headnode)
        {
            if (numbclipnodes >= max_expanded_clipnodes)
                err::fatal("expand_clipnodes_r: exceeded max_expanded_clipnodes");
            bclipnode *c = &bclipnodes[numbclipnodes];
            numbclipnodes++;
            if (headnode < 0)
            {
                c->isleaf = true;
                c->content = headnode;
                c->partitions = nullptr;
            }
            else
            {
                c->isleaf = false;
                c->planenum = clipnodes[headnode].planenum;
                c->plane_ = &state.planes[(size_t)c->planenum];
                for (int k = 0; k < 2; k++)
                {
                    c->children[k] = expand_clipnodes_r(state, bclipnodes, numbclipnodes,
                                                        clipnodes, clipnodes[headnode].children[k]);
                }
            }
            return c;
        }

        void expand_clipnodes(const bsp_state &state, bbrinkinfo *info,
                              const format::dclipnode_t *clipnodes, int headnode)
        {
            std::vector<bclipnode> temp((size_t)max_expanded_clipnodes);
            info->numclipnodes = 0;
            expand_clipnodes_r(state, temp.data(), info->numclipnodes, clipnodes, headnode);
            info->clipnodes = new bclipnode[(size_t)info->numclipnodes];
            for (int i = 0; i < info->numclipnodes; i++)
            {
                info->clipnodes[i] = temp[(size_t)i];
                for (int k = 0; k < 2; k++)
                {
                    if (!temp[(size_t)i].isleaf)
                    {
                        info->clipnodes[i].children[k] =
                            info->clipnodes + (temp[(size_t)i].children[k] - temp.data());
                    }
                }
            }
        }

        void build_tree_cells(bbrinkinfo *info)
        {
            info->numobjects = 0;
            info->leaf_outside = build_outside(info->numobjects);
            info->clipnodes[0].treeleaf = build_base_cell(info->numobjects, &info->clipnodes[0],
                                                          (vec_t)bogus_range, info->leaf_outside);
            build_tree_cells_r(info->numobjects, &info->clipnodes[0]);
        }

        void delete_tree_cells_r(int &numobjects, bclipnode *node)
        {
            if (node->treeleaf)
            {
                delete_leaf(numobjects, node->treeleaf);
                node->treeleaf = nullptr;
            }
            if (!node->isleaf)
            {
                delete_tree_cells_r(numobjects, node->children[0]);
                delete_tree_cells_r(numobjects, node->children[1]);
            }
        }

        void delete_tree_cells(bbrinkinfo *info)
        {
            delete_leaf(info->numobjects, info->leaf_outside);
            info->leaf_outside = nullptr;
            delete_tree_cells_r(info->numobjects, &info->clipnodes[0]);
            if (info->numobjects != 0)
                err::fatal("delete_tree_cells: internal error: numobjects != 0");
        }

        void clear_marks_r(bclipnode *node)
        {
            if (node->isleaf)
            {
                for (auto fi = node->treeleaf->faces->begin();
                     fi != node->treeleaf->faces->end(); ++fi)
                {
                    for (auto ei = fi->f->edges->begin(); ei != fi->f->edges->end(); ++ei)
                        ei->e->tmp_tested = false;
                }
            }
            else
            {
                clear_marks_r(node->children[0]);
                clear_marks_r(node->children[1]);
            }
        }

        void collect_brinks_r(bclipnode *node, int &numbrinks, bbrink **brinks)
        {
            if (node->isleaf)
            {
                for (auto fi = node->treeleaf->faces->begin();
                     fi != node->treeleaf->faces->end(); ++fi)
                {
                    for (auto ei = fi->f->edges->begin(); ei != fi->f->edges->end(); ++ei)
                    {
                        if (ei->e->tmp_tested)
                            continue;
                        ei->e->tmp_tested = true;
                        if (!ei->e->infinite)
                        {
                            if (brinks != nullptr)
                            {
                                brinks[numbrinks] = ei->e->brink;
                                brinks[numbrinks]->edge = ei->e;
                                for (int i = 0; i < brinks[numbrinks]->numnodes; i++)
                                {
                                    bbrinknode *bnode = &(*brinks[numbrinks]->nodes)[(size_t)i];
                                    if (bnode->isleaf && !bnode->clipnode->isleaf)
                                        err::fatal("collect_brinks_r: internal error: not leaf");
                                }
                            }
                            numbrinks++;
                        }
                    }
                }
            }
            else
            {
                collect_brinks_r(node->children[0], numbrinks, brinks);
                collect_brinks_r(node->children[1], numbrinks, brinks);
            }
        }

        void collect_brinks(bbrinkinfo *info)
        {
            info->numbrinks = 0;
            clear_marks_r(&info->clipnodes[0]);
            collect_brinks_r(&info->clipnodes[0], info->numbrinks, nullptr);
            info->brinks = new bbrink *[(size_t)info->numbrinks];
            info->numbrinks = 0;
            clear_marks_r(&info->clipnodes[0]);
            collect_brinks_r(&info->clipnodes[0], info->numbrinks, info->brinks);
        }

        void free_brinks(bbrinkinfo *info)
        {
            delete[] info->brinks;
            info->brinks = nullptr;
        }

        struct bsurface;

        struct bwedge
        {
            int content = 0;
            int nodenum = 0;
            bsurface *prev = nullptr;
            bsurface *next = nullptr;
        };

        struct bsurface
        {
            math::vec3v normal; // pointing clockwise
            int nodenum = 0;
            bool nodeside = false;
            bwedge *prev = nullptr;
            bwedge *next = nullptr;
        };

        constexpr int max_brink_wedges = 64;

        struct bcircle
        {
            math::vec3v axis;
            math::vec3v basenormal;
            int numwedges[2] = {}; // the front and back side of nodes[0]
            bwedge wedges[2][max_brink_wedges];     // in counterclockwise order
            bsurface surfaces[2][max_brink_wedges]; // between two adjacent wedges
        };

        bool calculate_circle(bbrink *b, bcircle *c)
        {
            c->axis = b->direction;
            if (!math::normalize(c->axis))
                return false;
            c->basenormal = (*b->nodes)[0].plane_->normal;

            int side, i;
            for (side = 0; side < 2; side++)
            {
                math::vec3v facing;
                math::cross(c->basenormal, c->axis, facing);
                math::scale(facing, side ? -1 : 1, facing);
                if (math::normalize(facing) < 1 - 0.01)
                    return false;

                // sort the wedges
                c->numwedges[side] = 1;
                c->wedges[side][0].nodenum = (*b->nodes)[0].children[side];
                c->surfaces[side][0].nodenum = 0;
                c->surfaces[side][0].nodeside = side == 0;
                while (true)
                {
                    for (i = 0; i < c->numwedges[side]; i++)
                    {
                        int nodenum = c->wedges[side][i].nodenum;
                        bbrinknode *node = &(*b->nodes)[(size_t)nodenum];
                        if (!node->isleaf)
                        {
                            std::memmove(&c->wedges[side][i + 1], &c->wedges[side][i],
                                         ((size_t)c->numwedges[side] - i) * sizeof(bwedge));
                            std::memmove(&c->surfaces[side][i + 2], &c->surfaces[side][i + 1],
                                         ((size_t)c->numwedges[side] - 1 - i) * sizeof(bsurface));
                            c->numwedges[side]++;
                            bool flipnode = (math::dot(node->plane_->normal, facing) < 0);
                            c->wedges[side][i].nodenum = node->children[flipnode];
                            c->wedges[side][i + 1].nodenum = node->children[!flipnode];
                            c->surfaces[side][i + 1].nodenum = nodenum;
                            c->surfaces[side][i + 1].nodeside = flipnode;
                            break;
                        }
                    }
                    if (i == c->numwedges[side])
                        break;
                }
            }
            if ((c->numwedges[0] + c->numwedges[1]) * 2 - 1 != b->numnodes)
                err::fatal("calculate_circle: internal error 1");

            // fill in other information
            for (side = 0; side < 2; side++)
            {
                for (i = 0; i < c->numwedges[side]; i++)
                {
                    bwedge *w = &c->wedges[side][i];
                    bbrinknode *node = &(*b->nodes)[(size_t)w->nodenum];
                    if (!node->clipnode->isleaf)
                        err::fatal("calculate_circle: internal error: not leaf");
                    w->content = node->content;
                    w->prev = &c->surfaces[side][i];
                    w->next = (i == c->numwedges[side] - 1) ? &c->surfaces[!side][0]
                                                            : &c->surfaces[side][i + 1];
                    w->prev->next = w;
                    w->next->prev = w;
                }
                for (i = 0; i < c->numwedges[side]; i++)
                {
                    bsurface *s = &c->surfaces[side][i];
                    bbrinknode *node = &(*b->nodes)[(size_t)s->nodenum];
                    math::scale(node->plane_->normal, s->nodeside ? -1 : 1, s->normal);
                }
            }

            // check the normals
            for (side = 0; side < 2; side++)
            {
                for (i = 0; i < c->numwedges[side]; i++)
                {
                    bwedge *w = &c->wedges[side][i];
                    if (i == 0 && i == c->numwedges[side] - 1) // 180 degrees
                        continue;
                    math::vec3v v;
                    math::cross(w->prev->normal, w->next->normal, v);
                    if (!math::normalize(v) || math::dot(v, c->axis) < 1 - 0.01)
                        return false;
                }
            }
            return true;
        }

        bool add_partition(const bsp_state &state, bclipnode *clipnode, int planenum,
                           bool planeside, int content, int brinktype)
        {
            // make sure we won't do any harm
            if (!clipnode->isleaf)
                return false;
            bool onback = false;
            for (auto fi = clipnode->treeleaf->faces->begin();
                 fi != clipnode->treeleaf->faces->end(); ++fi)
            {
                for (auto ei = fi->f->edges->begin(); ei != fi->f->edges->end(); ++ei)
                {
                    for (int side = 0; side < 2; side++)
                    {
                        btreepoint *tp = get_point_from_edge(ei->e, side != 0);
                        const plane *pl = &state.planes[(size_t)planenum];
                        vec_t dist = math::dot(tp->v, pl->normal) - pl->dist;
                        if (planeside ? dist < -math::on_epsilon : dist > math::on_epsilon)
                            return false;
                        if (planeside ? dist > math::on_epsilon : dist < -math::on_epsilon)
                            onback = true;
                    }
                }
            }
            if (!onback)
                return false; // the whole leaf is on the plane, or has no vertex
            bpartition *p = new bpartition();
            p->next = clipnode->partitions;
            p->planenum = planenum;
            p->planeside = planeside;
            p->content = content;
            p->type = brinktype;
            clipnode->partitions = p;
            return true;
        }

        void analyze_brinks(const bsp_state &state, bbrinkinfo *info)
        {
            for (int i = 0; i < info->numbrinks; i++)
            {
                bbrink *b = info->brinks[i];
                if (b->numnodes <= 5)
                {
                    // quickly reject the most trivial brinks a brink is not
                    // necessarily split twice after its creation
                    if (b->numnodes != 3 && b->numnodes != 5)
                        err::fatal("analyze_brinks: internal error 1");
                    continue;
                }

                if (b->numnodes > 2 * max_brink_wedges - 1)
                    continue; // too complicated

                bcircle c;
                // build the circle to find the planes a player may move along
                if (!calculate_circle(b, &c))
                    continue;

                int transitionfound[2];
                bsurface *transitionpos[2] = {};
                bool transitionside[2] = {};
                for (int side = 0; side < 2; side++)
                {
                    transitionfound[side] = 0;
                    // the surfaces on the first split are considered later
                    for (int j = 1; j < c.numwedges[side]; j++)
                    {
                        bsurface *s = &c.surfaces[side][j];
                        if ((s->prev->content == contents_solid)
                            != (s->next->content == contents_solid))
                        {
                            transitionfound[side]++;
                            transitionpos[side] = s;
                            transitionside[side] = (s->prev->content == contents_solid);
                        }
                    }
                }

                if (transitionfound[0] == 0 || transitionfound[1] == 0)
                {
                    // at least one side of the first split is completely solid
                    // or empty; no bugs in this case
                    continue;
                }

                if (transitionfound[0] > 1 || transitionfound[1] > 1
                    || (c.surfaces[0][0].prev->content == contents_solid)
                        != (c.surfaces[0][0].next->content == contents_solid)
                    || (c.surfaces[1][0].prev->content == contents_solid)
                        != (c.surfaces[1][0].next->content == contents_solid))
                {
                    // at least 3 transition surfaces: too complicated, leave it
                    continue;
                }

                if (transitionside[1] != !transitionside[0])
                    err::fatal("analyze_brinks: internal error 2");
                const math::vec3v vup{0, 0, 1};
                bool isfloor = false;
                bool onfloor = false;
                for (int side2 = 0; side2 < 2; side2++)
                {
                    math::vec3v normal;
                    // pointing from solid to empty
                    math::scale(transitionpos[side2]->normal,
                                transitionside[side2] ? -1 : 1, normal);
                    if (math::dot(normal, vup) > brink_floor_threshold)
                        isfloor = true;
                }
                for (int side2 = 0; side2 < 2; side2++)
                {
                    btreepoint *tp = get_point_from_edge(b->edge, side2 != 0);
                    if (tp->infinite)
                        continue;
                    for (auto ei = tp->edges->begin(); ei != tp->edges->end(); ++ei)
                    {
                        for (auto fi = ei->e->faces->begin(); fi != ei->e->faces->end(); ++fi)
                        {
                            if (fi->f->infinite
                                || get_leaf_from_face(fi->f, false)->infinite
                                || get_leaf_from_face(fi->f, true)->infinite)
                            {
                                err::fatal("analyze_brinks: internal error: an infinite object contains a finite object");
                            }
                            for (int side3 = 0; side3 < 2; side3++)
                            {
                                math::vec3v normal;
                                math::scale(fi->f->plane_->normal,
                                            (fi->f->planeside != (side3 != 0)) ? -1 : 1, normal);
                                if (math::dot(normal, vup) > brink_floor_threshold
                                    && get_leaf_from_face(fi->f, side3 != 0)->clipnode->content
                                        == contents_solid
                                    && get_leaf_from_face(fi->f, side3 == 0)->clipnode->content
                                        != contents_solid)
                                {
                                    onfloor = true;
                                }
                            }
                        }
                    }
                }
                // this does not fix all the bugs, only most of them
                for (int side = 0; side < 2; side++)
                {
                    bsurface *smovement = transitionpos[side];
                    bsurface *s;
                    for (s = transitionside[!side] ? &c.surfaces[!side][0] : &c.surfaces[side][0];;
                         s = transitionside[!side] ? s->next->next : s->prev->prev)
                    {
                        bwedge *w = transitionside[!side] ? s->next : s->prev;
                        bsurface *snext = transitionside[!side] ? w->next : w->prev;
                        math::vec3v tmp;
                        math::cross(smovement->normal, snext->normal, tmp);
                        vec_t dot = math::dot(tmp, c.axis);
                        if (transitionside[!side] ? dot < 0.01 : dot > -0.01)
                            break;
                        if (w->content != contents_solid)
                            break;
                        if (snext == (transitionside[!side] ? &c.surfaces[side][0]
                                                            : &c.surfaces[!side][0]))
                        {
                            break; // surface past 0
                        }
                        bool blocking = !(math::dot(smovement->normal, s->normal) > 0.01);
                        bclipnode *clipnode = (*b->nodes)[(size_t)w->nodenum].clipnode;
                        int planenum = (*b->nodes)[(size_t)smovement->nodenum].planenum;
                        bool planeside = transitionside[!side] ? smovement->nodeside
                                                               : !smovement->nodeside;
                        int brinktype = isfloor
                            ? (blocking ? brink_floor_blocking : brink_floor)
                            : onfloor ? (blocking ? brink_wall_blocking : brink_wall)
                                      : brink_any;
                        add_partition(state, clipnode, planenum, planeside,
                                      contents_empty, brinktype);
                    }
                }
            }
        }

        void delete_clipnodes(bbrinkinfo *info)
        {
            for (int i = 0; i < info->numclipnodes; i++)
            {
                if (!info->clipnodes[i].isleaf)
                    continue;
                bpartition *p;
                while ((p = info->clipnodes[i].partitions) != nullptr)
                {
                    info->clipnodes[i].partitions = p->next;
                    delete p;
                }
            }
            delete[] info->clipnodes;
        }

        // merge equal partition planes so the clipnodes compress better
        void sort_partitions(bbrinkinfo *info)
        {
            for (int i = 0; i < info->numclipnodes; i++)
            {
                bclipnode *clipnode = &info->clipnodes[i];
                if (!clipnode->isleaf)
                    continue;
                bpartition *current;
                bpartition **pp;
                bpartition *partitions = clipnode->partitions;
                clipnode->partitions = nullptr;
                while ((current = partitions) != nullptr)
                {
                    partitions = current->next;
                    if (current->content != contents_empty)
                        err::fatal("sort_partitions: content of partition was not empty");
                    for (pp = &clipnode->partitions; *pp; pp = &(*pp)->next)
                    {
                        // normally the planeside should be identical
                        if ((*pp)->planenum > current->planenum
                            || ((*pp)->planenum == current->planenum
                                && (*pp)->planeside >= current->planeside))
                        {
                            break;
                        }
                    }
                    if (*pp && (*pp)->planenum == current->planenum
                        && (*pp)->planeside == current->planeside)
                    {
                        // pick the most important level of the two
                        (*pp)->type = (*pp)->type < current->type ? (*pp)->type : current->type;
                        delete current;
                        continue;
                    }
                    current->next = *pp;
                    *pp = current;
                }
            }
        }

        bbrinkinfo *create_brinkinfo(const bsp_state &state,
                                     const format::dclipnode_t *clipnodes, int headnode)
        {
            bbrinkinfo *info = new bbrinkinfo();
            expand_clipnodes(state, info, clipnodes, headnode);
            build_tree_cells(info);
            collect_brinks(info);
            analyze_brinks(state, info);
            free_brinks(info);
            delete_tree_cells(info);
            sort_partitions(info);
            return info;
        }

        using clipnode_key = std::pair<int, std::pair<int, int>>;
        using clipnode_map = std::map<clipnode_key, int>;

        clipnode_key make_key(const format::dclipnode_t &c)
        {
            return std::make_pair((int)c.planenum,
                                  std::make_pair((int)c.children[0], (int)c.children[1]));
        }

        struct fix_context
        {
            bool noclipnodemerge = false;
            int count_merged = 0;
        };

        bool fix_brinks_r_r(fix_context &ctx, const bclipnode *clipnode, const bpartition *p,
                            int level, int &headnode_out, format::dclipnode_t *begin,
                            format::dclipnode_t *end, format::dclipnode_t *&current,
                            clipnode_map *outputmap)
        {
            while (p && p->type > level)
                p = p->next;
            if (p == nullptr)
            {
                headnode_out = clipnode->content;
                return true;
            }
            format::dclipnode_t cn;
            format::dclipnode_t *c = current;
            current++;
            cn.planenum = p->planenum;
            cn.children[p->planeside] = (short)p->content;
            int r;
            if (!fix_brinks_r_r(ctx, clipnode, p->next, level, r, begin, end, current, outputmap))
                return false;
            cn.children[!p->planeside] = (short)r;
            clipnode_map::iterator output = outputmap->find(make_key(cn));
            if (ctx.noclipnodemerge || output == outputmap->end())
            {
                if (c >= end)
                    return false;
                *c = cn;
                (*outputmap)[make_key(cn)] = (int)(c - begin);
                headnode_out = (int)(c - begin);
            }
            else
            {
                ctx.count_merged++;
                if (current != c + 1)
                    err::fatal("merge clipnodes: internal error");
                current = c;
                headnode_out = output->second; // use the existing clipnode
            }
            return true;
        }

        bool fix_brinks_r(fix_context &ctx, const bclipnode *clipnode, int level,
                          int &headnode_out, format::dclipnode_t *begin,
                          format::dclipnode_t *end, format::dclipnode_t *&current,
                          clipnode_map *outputmap)
        {
            if (clipnode->isleaf)
            {
                return fix_brinks_r_r(ctx, clipnode, clipnode->partitions, level, headnode_out,
                                      begin, end, current, outputmap);
            }
            format::dclipnode_t cn;
            format::dclipnode_t *c = current;
            current++;
            cn.planenum = clipnode->planenum;
            for (int k = 0; k < 2; k++)
            {
                int r;
                if (!fix_brinks_r(ctx, clipnode->children[k], level, r, begin, end, current,
                                  outputmap))
                {
                    return false;
                }
                cn.children[k] = (short)r;
            }
            clipnode_map::iterator output = outputmap->find(make_key(cn));
            if (ctx.noclipnodemerge || output == outputmap->end())
            {
                if (c >= end)
                    return false;
                *c = cn;
                (*outputmap)[make_key(cn)] = (int)(c - begin);
                headnode_out = (int)(c - begin);
            }
            else
            {
                ctx.count_merged++;
                if (current != c + 1)
                    err::fatal("merge clipnodes: internal error");
                current = c;
                headnode_out = output->second; // use the existing clipnode
            }
            return true;
        }

        bool fix_brinks(fix_context &ctx, const bbrinkinfo *info, int level, int &headnode_out,
                        format::dclipnode_t *clipnodes_out, int maxsize, int size, int &size_out)
        {
            format::dclipnode_t *begin = clipnodes_out;
            format::dclipnode_t *end = &clipnodes_out[maxsize];
            format::dclipnode_t *current = &clipnodes_out[size];
            clipnode_map outputmap;
            int r;
            if (!fix_brinks_r(ctx, &info->clipnodes[0], level, r, begin, end, current, &outputmap))
                return false;
            headnode_out = r;
            size_out = (int)(current - begin);
            return true;
        }

        void delete_brinkinfo(bbrinkinfo *info)
        {
            delete_clipnodes(info);
            delete info;
        }
    }

    // the reference finishbspfile brink block: build brinkinfo for every
    // model hull, then rewrite the whole clipnode lump at the strongest fix
    // level that still fits
    void fix_all_brinks(bsp_state &state)
    {
        format::map_data &map = *state.map;
        std::vector<format::dclipnode_t> clipnodes((size_t)limits::max_map_clipnodes);

        int nummodels = (int)map.models.size();
        std::vector<bbrinkinfo *> brinkinfo((size_t)nummodels * num_hulls, nullptr);
        std::vector<int> headnode((size_t)nummodels * num_hulls, 0);

        for (int i = 0; i < nummodels; i++)
        {
            format::dmodel_t *m = &map.models[(size_t)i];
            for (int j = 1; j < num_hulls; j++)
            {
                brinkinfo[(size_t)(i * num_hulls + j)] =
                    create_brinkinfo(state, map.clipnodes.data(), m->headnode[j]);
            }
        }

        fix_context ctx;
        ctx.noclipnodemerge = state.options.noclipnodemerge;
        int numclipnodes = 0;
        int level;
        int i = 0;
        for (level = brink_any; level > brink_none; level--)
        {
            numclipnodes = 0;
            ctx.count_merged = 0;
            int j = 1;
            for (i = 0; i < nummodels; i++)
            {
                for (j = 1; j < num_hulls; j++)
                {
                    if (!fix_brinks(ctx, brinkinfo[(size_t)(i * num_hulls + j)], level,
                                    headnode[(size_t)(i * num_hulls + j)], clipnodes.data(),
                                    limits::max_map_clipnodes, numclipnodes, numclipnodes))
                    {
                        break;
                    }
                }
                if (j < num_hulls)
                    break;
            }
            if (i == nummodels)
                break;
        }
        for (i = 0; i < nummodels; i++)
        {
            for (int j = 1; j < num_hulls; j++)
                delete_brinkinfo(brinkinfo[(size_t)(i * num_hulls + j)]);
        }
        if (level == brink_none)
        {
            logging::warn("No brinks have been fixed because clipnode data is almost full");
        }
        else
        {
            if (level != brink_any)
                logging::warn("Not all brinks have been fixed because clipnode data is almost full");
            logging::file("  increased %d clipnodes to %d\n",
                          (int)map.clipnodes.size(), numclipnodes);
            map.clipnodes.assign(clipnodes.begin(), clipnodes.begin() + numclipnodes);
            for (i = 0; i < nummodels; i++)
            {
                format::dmodel_t *m = &map.models[(size_t)i];
                for (int j = 1; j < num_hulls; j++)
                    m->headnode[j] = headnode[(size_t)(i * num_hulls + j)];
            }
        }
    }
}
