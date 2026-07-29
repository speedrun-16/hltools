#include "internal.h"

#include <cmath>
#include <cstring>
#include <deque>
#include <new>

#include "../common/error.h"

// t junction fixing: collects every edge of every node face into a spatially
// hashed edge table, then reinserts missed vertexes along shared edges,
// splitting faces that grow past the point cap

namespace bsp
{
    namespace
    {
        constexpr int num_hash = 4096;
        constexpr int max_hash_neighbors = 4;
        constexpr double t_epsilon = math::on_epsilon;
        // the reference reinterpreted a 16kb buffer as a face with an
        // oversized point array; this is the same capacity
        constexpr int max_superface_edges = 680;

        struct wvert
        {
            vec_t t = 0;
            wvert *prev = nullptr;
            wvert *next = nullptr;
        };

        struct wedge
        {
            wedge *next = nullptr;
            math::vec3v dir;
            math::vec3v origin;
            wvert head;
            int numverts = 0;
        };

        // a face with room for every tjunc inserted point
        struct super_face
        {
            int planenum = -1;
            int texturenum = 0;
            int contents = 0;
            int detail_level = 0;
            face *original = nullptr;
            face_style style = face_style::normal;
            int numpoints = 0;
            math::vec3v pts[max_superface_edges];
        };

        struct tjunc_state
        {
            // Intrusive edge lists store direct pointers into these pools.
            // deque keeps them stable while allowing the temporary workspace
            // to grow with the input instead of enforcing legacy array caps.
            std::deque<wvert> wverts;
            std::deque<wedge> wedges;
            bsp_state *state = nullptr;
            int numwedges = 0;
            int numwverts = 0;
            int input_faces = 0;
            int input_edges = 0;
            int tjuncs = 0;
            int tjuncfaces = 0;
            wedge *densest_wedge = nullptr;

            wedge *wedge_hash[num_hash] = {};
            math::vec3v hash_min;
            math::vec3v hash_scale;
            int hash_numslots[3] = {};

            super_face superface;
            face *newlist = nullptr;
        };

        tjunc_state g_tjunc;

        math::vec3v point_on_wedge(const wedge *w, vec_t t)
        {
            math::vec3v point;
            math::multiply_add(w->origin, t, w->dir, point);
            return point;
        }

        [[noreturn]] void fatal_wvert_allocation(const wedge *w, vec_t t)
        {
            int model = g_tjunc.state ? g_tjunc.state->nummodels - 1 : -1;
            const format::entity *ent =
                g_tjunc.state ? entity_for_model(*g_tjunc.state, model) : nullptr;
            math::vec3v point = point_on_wedge(w, t);

            math::vec3v dense_start = {};
            math::vec3v dense_end = {};
            int dense_verts = 0;
            if (g_tjunc.densest_wedge)
            {
                const wedge *dense = g_tjunc.densest_wedge;
                dense_verts = dense->numverts;
                dense_start = point_on_wedge(dense, dense->head.next->t);
                dense_end = point_on_wedge(dense, dense->head.prev->t);
            }

            err::fatal(
                "could not grow T-junction vertex workspace\n"
                "  model              %d (%s; origin \"%s\"; targetname \"%s\")\n"
                "  temporary vertices %d\n"
                "  edge lines         %d\n"
                "  scanned geometry   %d faces, %d edges\n"
                "  attempted vertex   (%.3f %.3f %.3f), line currently has %d vertices\n"
                "  densest edge line  %d vertices, (%.3f %.3f %.3f) to (%.3f %.3f %.3f)\n"
                "  note               system memory allocation failed; this is not a BSP limit\n"
                "  action             simplify highly intersecting or fragmented brushwork",
                model,
                ent ? ent->value("classname") : "unknown entity",
                ent ? ent->value("origin") : "",
                ent ? ent->value("targetname") : "",
                g_tjunc.numwverts,
                g_tjunc.numwedges,
                g_tjunc.input_faces, g_tjunc.input_edges,
                (double)point[0], (double)point[1], (double)point[2], w->numverts,
                dense_verts,
                (double)dense_start[0], (double)dense_start[1], (double)dense_start[2],
                (double)dense_end[0], (double)dense_end[1], (double)dense_end[2]);
        }

        void init_hash()
        {
            // ignore the node bounds and keep things predictable, so there
            // are no strange cases like division by zero or extreme scales
            math::vec3v size;
            for (int i = 0; i < 3; i++)
            {
                g_tjunc.hash_min[i] = -8000;
                size[i] = 16000;
            }
            std::memset(g_tjunc.wedge_hash, 0, sizeof(g_tjunc.wedge_hash));

            vec_t volume = size[0] * size[1];
            vec_t scale = std::sqrt(volume / num_hash);

            g_tjunc.hash_numslots[0] = (int)std::floor(size[0] / scale);
            g_tjunc.hash_numslots[1] = (int)std::floor(size[1] / scale);
            while (g_tjunc.hash_numslots[0] * g_tjunc.hash_numslots[1] > num_hash)
            {
                g_tjunc.hash_numslots[0]--;
                g_tjunc.hash_numslots[1]--;
            }

            g_tjunc.hash_scale[0] = g_tjunc.hash_numslots[0] / size[0];
            g_tjunc.hash_scale[1] = g_tjunc.hash_numslots[1] / size[1];
        }

        int hash_vec(const math::vec3v &vec, int *num_hashneighbors, int *hashneighbors)
        {
            int slot[2];
            vec_t slotdiff[2];

            for (int i = 0; i < 2; i++)
            {
                vec_t normalized = g_tjunc.hash_scale[i] * (vec[i] - g_tjunc.hash_min[i]);
                slot[i] = (int)std::floor(normalized);
                slotdiff[i] = normalized - (vec_t)slot[i];

                slot[i] = (slot[i] + g_tjunc.hash_numslots[i]) % g_tjunc.hash_numslots[i];
                slot[i] = (slot[i] + g_tjunc.hash_numslots[i]) % g_tjunc.hash_numslots[i];
            }

            int h = slot[0] * g_tjunc.hash_numslots[1] + slot[1];

            *num_hashneighbors = 0;
            for (int x = -1; x <= 1; x++)
            {
                if ((x == -1 && slotdiff[0] > g_tjunc.hash_scale[0] * (2 * math::on_epsilon))
                    || (x == 1 && slotdiff[0] < 1 - g_tjunc.hash_scale[0] * (2 * math::on_epsilon)))
                {
                    continue;
                }
                for (int y = -1; y <= 1; y++)
                {
                    if ((y == -1 && slotdiff[1] > g_tjunc.hash_scale[1] * (2 * math::on_epsilon))
                        || (y == 1 && slotdiff[1] < 1 - g_tjunc.hash_scale[1] * (2 * math::on_epsilon)))
                    {
                        continue;
                    }
                    if (*num_hashneighbors >= max_hash_neighbors)
                        err::fatal("hash_vec: internal error");
                    hashneighbors[*num_hashneighbors] =
                        ((slot[0] + x + g_tjunc.hash_numslots[0]) % g_tjunc.hash_numslots[0])
                            * g_tjunc.hash_numslots[1]
                        + (slot[1] + y + g_tjunc.hash_numslots[1]) % g_tjunc.hash_numslots[1];
                    (*num_hashneighbors)++;
                }
            }

            return h;
        }

        bool canonical_vector(math::vec3v &vec)
        {
            if (math::normalize(vec))
            {
                if (vec[0] > math::normal_epsilon)
                    return true;
                if (vec[0] < -math::normal_epsilon)
                {
                    vec = -vec;
                    return true;
                }
                vec[0] = 0;

                if (vec[1] > math::normal_epsilon)
                    return true;
                if (vec[1] < -math::normal_epsilon)
                {
                    vec = -vec;
                    return true;
                }
                vec[1] = 0;

                if (vec[2] > math::normal_epsilon)
                    return true;
                if (vec[2] < -math::normal_epsilon)
                {
                    vec = -vec;
                    return true;
                }
                vec[2] = 0;
                return false;
            }
            return false;
        }

        wedge *find_edge(const math::vec3v &p1, const math::vec3v &p2, vec_t *t1, vec_t *t2)
        {
            math::vec3v dir;
            math::subtract(p2, p1, dir);
            canonical_vector(dir);

            *t1 = math::dot(p1, dir);
            *t2 = math::dot(p2, dir);

            math::vec3v origin;
            math::multiply_add(p1, -*t1, dir, origin);

            if (*t1 > *t2)
            {
                vec_t temp = *t1;
                *t1 = *t2;
                *t2 = temp;
            }

            int num_hashneighbors;
            int hashneighbors[max_hash_neighbors];
            int h = hash_vec(origin, &num_hashneighbors, hashneighbors);

            for (int i = 0; i < num_hashneighbors; ++i)
            {
                for (wedge *w = g_tjunc.wedge_hash[hashneighbors[i]]; w; w = w->next)
                {
                    if (std::fabs(w->origin[0] - origin[0]) > math::equal_epsilon
                        || std::fabs(w->origin[1] - origin[1]) > math::equal_epsilon
                        || std::fabs(w->origin[2] - origin[2]) > math::equal_epsilon)
                    {
                        continue;
                    }
                    if (std::fabs(w->dir[0] - dir[0]) > math::normal_epsilon
                        || std::fabs(w->dir[1] - dir[1]) > math::normal_epsilon
                        || std::fabs(w->dir[2] - dir[2]) > math::normal_epsilon)
                    {
                        continue;
                    }
                    return w;
                }
            }

            try
            {
                g_tjunc.wedges.emplace_back();
            }
            catch (const std::bad_alloc &)
            {
                err::fatal("could not grow T-junction edge workspace after %d edge lines",
                           g_tjunc.numwedges);
            }
            wedge *w = &g_tjunc.wedges.back();
            g_tjunc.numwedges++;

            w->next = g_tjunc.wedge_hash[h];
            g_tjunc.wedge_hash[h] = w;

            w->origin = origin;
            w->dir = dir;
            w->head.next = w->head.prev = &w->head;
            w->head.t = 99999;
            w->numverts = 0;
            return w;
        }

        void add_vert(wedge *w, vec_t t)
        {
            wvert *v = w->head.next;
            while (true)
            {
                if (std::fabs(v->t - t) < t_epsilon)
                    return;
                if (v->t > t)
                    break;
                v = v->next;
            }

            // insert a new wvert before v
            try
            {
                g_tjunc.wverts.emplace_back();
            }
            catch (const std::bad_alloc &)
            {
                fatal_wvert_allocation(w, t);
            }
            wvert *newv = &g_tjunc.wverts.back();
            g_tjunc.numwverts++;

            newv->t = t;
            newv->next = v;
            newv->prev = v->prev;
            v->prev->next = newv;
            v->prev = newv;
            w->numverts++;
            if (!g_tjunc.densest_wedge
                || w->numverts > g_tjunc.densest_wedge->numverts)
            {
                g_tjunc.densest_wedge = w;
            }
        }

        void add_edge(const math::vec3v &p1, const math::vec3v &p2)
        {
            vec_t t1, t2;
            wedge *w = find_edge(p1, p2, &t1, &t2);
            add_vert(w, t1);
            add_vert(w, t2);
        }

        void add_face_edges(const face *f)
        {
            g_tjunc.input_faces++;
            g_tjunc.input_edges += f->numpoints;
            for (int i = 0; i < f->numpoints; i++)
            {
                int j = (i + 1) % f->numpoints;
                add_edge(f->pts[i], f->pts[j]);
            }
        }

        face *new_face_from_super(const super_face *in)
        {
            face *newf = alloc_face();
            newf->planenum = in->planenum;
            newf->texturenum = in->texturenum;
            newf->original = in->original;
            newf->contents = in->contents;
            newf->style = in->style;
            newf->detail_level = in->detail_level;
            return newf;
        }

        void copy_super_to_face(const super_face *in, face *out)
        {
            out->planenum = in->planenum;
            out->texturenum = in->texturenum;
            out->original = in->original;
            out->contents = in->contents;
            out->style = in->style;
            out->detail_level = in->detail_level;
            out->numpoints = in->numpoints;
            for (int i = 0; i < in->numpoints; i++)
                out->pts[i] = in->pts[i];
        }

        // carves pieces off an oversized face until every piece fits the
        // read size point cap, chaining the pieces through original
        void split_face_for_tjunc(super_face *f, face *original)
        {
            face *chain = nullptr;
            while (true)
            {
                if (f->numpoints <= max_read_points)
                {
                    // small enough now, copy it back to the original
                    copy_super_to_face(f, original);
                    original->original = chain;
                    original->next = g_tjunc.newlist;
                    g_tjunc.newlist = original;
                    return;
                }

                g_tjunc.tjuncfaces++;

                int firstcorner, lastcorner;
                while (true)
                {
                    // find the last corner
                    math::vec3v dir, test;
                    math::subtract(f->pts[f->numpoints - 1], f->pts[0], dir);
                    math::normalize(dir);
                    for (lastcorner = f->numpoints - 1; lastcorner > 0; lastcorner--)
                    {
                        math::subtract(f->pts[lastcorner - 1], f->pts[lastcorner], test);
                        math::normalize(test);
                        vec_t v = math::dot(test, dir);
                        if (v < 1.0 - math::on_epsilon || v > 1.0 + math::on_epsilon)
                            break;
                    }

                    // find the first corner
                    math::subtract(f->pts[1], f->pts[0], dir);
                    math::normalize(dir);
                    for (firstcorner = 1; firstcorner < f->numpoints - 1; firstcorner++)
                    {
                        math::subtract(f->pts[firstcorner + 1], f->pts[firstcorner], test);
                        math::normalize(test);
                        vec_t v = math::dot(test, dir);
                        if (v < 1.0 - math::on_epsilon || v > 1.0 + math::on_epsilon)
                            break;
                    }

                    if (firstcorner + 2 >= max_read_points)
                    {
                        // rotate the point winding
                        math::vec3v first = f->pts[0];
                        for (int i = 1; i < f->numpoints; i++)
                            f->pts[i - 1] = f->pts[i];
                        f->pts[f->numpoints - 1] = first;
                        continue;
                    }
                    break;
                }

                // cut off as big a piece as possible, less than the cap, and
                // not past lastcorner
                face *newface = new_face_from_super(f);
                newface->original = chain;
                chain = newface;
                newface->next = g_tjunc.newlist;
                g_tjunc.newlist = newface;
                if (f->numpoints - firstcorner <= max_read_points)
                    newface->numpoints = firstcorner + 2;
                else if (lastcorner + 2 < max_read_points
                         && f->numpoints - lastcorner <= max_read_points)
                    newface->numpoints = lastcorner + 2;
                else
                    newface->numpoints = max_read_points;

                for (int i = 0; i < newface->numpoints; i++)
                    newface->pts[i] = f->pts[i];

                for (int i = newface->numpoints - 1; i < f->numpoints; i++)
                    f->pts[i - (newface->numpoints - 2)] = f->pts[i];
                f->numpoints -= (newface->numpoints - 2);
            }
        }

        void fix_face_edges(face *f)
        {
            super_face *sf = &g_tjunc.superface;
            sf->planenum = f->planenum;
            sf->texturenum = f->texturenum;
            sf->original = f->original;
            sf->contents = f->contents;
            sf->style = f->style;
            sf->detail_level = f->detail_level;
            sf->numpoints = f->numpoints;
            for (int i = 0; i < f->numpoints; i++)
                sf->pts[i] = f->pts[i];

        restart:
            for (int i = 0; i < sf->numpoints; i++)
            {
                int j = (i + 1) % sf->numpoints;

                vec_t t1, t2;
                wedge *w = find_edge(sf->pts[i], sf->pts[j], &t1, &t2);

                wvert *v;
                for (v = w->head.next; v->t < t1 + t_epsilon; v = v->next)
                {
                }

                if (v->t < t2 - t_epsilon)
                {
                    g_tjunc.tjuncs++;
                    // insert a new vertex here
                    for (int k = sf->numpoints; k > j; k--)
                        sf->pts[k] = sf->pts[k - 1];
                    math::multiply_add(w->origin, v->t, w->dir, sf->pts[j]);
                    sf->numpoints++;
                    if (sf->numpoints >= max_superface_edges)
                        err::fatal("exceeded max_superface_edges");
                    goto restart;
                }
            }

            if (sf->numpoints <= max_read_points)
            {
                copy_super_to_face(sf, f);
                f->next = g_tjunc.newlist;
                g_tjunc.newlist = f;
                return;
            }

            // the face needs to be split because of too many edges
            split_face_for_tjunc(sf, f);
        }

        void tjunc_find_r(node *n)
        {
            if (n->planenum == planenum_leaf)
                return;

            for (face *f = n->faces; f; f = f->next)
                add_face_edges(f);

            tjunc_find_r(n->children[0]);
            tjunc_find_r(n->children[1]);
        }

        void tjunc_fix_r(node *n)
        {
            if (n->planenum == planenum_leaf)
                return;

            g_tjunc.newlist = nullptr;

            face *next;
            for (face *f = n->faces; f; f = next)
            {
                next = f->next;
                fix_face_edges(f);
            }

            n->faces = g_tjunc.newlist;

            tjunc_fix_r(n->children[0]);
            tjunc_fix_r(n->children[1]);
        }
    }

    void tjunc(bsp_state &state, node *headnode)
    {
        if (state.options.notjunc)
            return;

        // identify all points on common edges
        init_hash();

        g_tjunc.state = &state;
        g_tjunc.wedges.clear();
        g_tjunc.wverts.clear();
        g_tjunc.numwedges = g_tjunc.numwverts = 0;
        g_tjunc.input_faces = g_tjunc.input_edges = 0;
        g_tjunc.densest_wedge = nullptr;

        tjunc_find_r(headnode);

        // add extra vertexes on edges where needed
        g_tjunc.tjuncs = g_tjunc.tjuncfaces = 0;

        tjunc_fix_r(headnode);
    }
}
