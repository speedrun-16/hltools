#include "internal.h"

#include <cmath>
#include <cstring>
#include <deque>

#include "../common/error.h"
#include "../common/limits.h"
#include "format/bsp/types.h"

// vertex and edge emission with the reference's spatial hash, plus the
// surface cache face subdivision

namespace bsp
{
    namespace
    {
        constexpr double point_epsilon = math::on_epsilon / 2;
        constexpr int num_hash = 4096;
        constexpr int max_hash_neighbors = 4;

        struct hashvert
        {
            hashvert *next = nullptr;
            math::vec3v point;
            int num = 0;
            int numplanes = 0; // for corner determination
            int planenums[2] = {};
            int numedges = 0;
        };

        struct edge_hash_state
        {
            // A deque keeps hashvert addresses stable while diagnostic counting
            // continues beyond GoldSrc's 16-bit vertex limit.
            std::deque<hashvert> hvertex;
            hashvert *hashverts[num_hash] = {};
            math::vec3v hash_min;
            math::vec3v hash_scale;
            int hash_numslots[3] = {};
            // the reference's edgefaces[max_map_edges][2], flattened
            std::vector<face *> edgefaces = std::vector<face *>((size_t)limits::max_map_edges * 2);
            int firstmodeledge = 1;
        };

        // shared by all models of one run, reset per model by make_face_edges
        edge_hash_state g_hash;

        void init_hash()
        {
            std::memset(g_hash.hashverts, 0, sizeof(g_hash.hashverts));
            g_hash.hvertex.clear();

            math::vec3v size;
            for (int i = 0; i < 3; i++)
            {
                g_hash.hash_min[i] = -8000;
                size[i] = 16000;
            }

            vec_t volume = size[0] * size[1];
            vec_t scale = std::sqrt(volume / num_hash);

            g_hash.hash_numslots[0] = (int)std::floor(size[0] / scale);
            g_hash.hash_numslots[1] = (int)std::floor(size[1] / scale);
            while (g_hash.hash_numslots[0] * g_hash.hash_numslots[1] > num_hash)
            {
                g_hash.hash_numslots[0]--;
                g_hash.hash_numslots[1]--;
            }

            g_hash.hash_scale[0] = g_hash.hash_numslots[0] / size[0];
            g_hash.hash_scale[1] = g_hash.hash_numslots[1] / size[1];

        }

        // returns the bucket a new vertex writes into; hashneighbors are the
        // buckets to read when checking for an existing vertex
        int hash_vec(const math::vec3v &vec, int *num_hashneighbors, int *hashneighbors)
        {
            int slot[2];
            vec_t slotdiff[2];

            for (int i = 0; i < 2; i++)
            {
                vec_t normalized = g_hash.hash_scale[i] * (vec[i] - g_hash.hash_min[i]);
                slot[i] = (int)std::floor(normalized);
                slotdiff[i] = normalized - (vec_t)slot[i];

                slot[i] = (slot[i] + g_hash.hash_numslots[i]) % g_hash.hash_numslots[i];
                slot[i] = (slot[i] + g_hash.hash_numslots[i]) % g_hash.hash_numslots[i];
            }

            int h = slot[0] * g_hash.hash_numslots[1] + slot[1];

            *num_hashneighbors = 0;
            for (int x = -1; x <= 1; x++)
            {
                if ((x == -1 && slotdiff[0] > g_hash.hash_scale[0] * (2 * point_epsilon))
                    || (x == 1 && slotdiff[0] < 1 - g_hash.hash_scale[0] * (2 * point_epsilon)))
                {
                    continue;
                }
                for (int y = -1; y <= 1; y++)
                {
                    if ((y == -1 && slotdiff[1] > g_hash.hash_scale[1] * (2 * point_epsilon))
                        || (y == 1 && slotdiff[1] < 1 - g_hash.hash_scale[1] * (2 * point_epsilon)))
                    {
                        continue;
                    }
                    if (*num_hashneighbors >= max_hash_neighbors)
                        err::fatal("hash_vec: internal error");
                    hashneighbors[*num_hashneighbors] =
                        ((slot[0] + x + g_hash.hash_numslots[0]) % g_hash.hash_numslots[0])
                            * g_hash.hash_numslots[1]
                        + (slot[1] + y + g_hash.hash_numslots[1]) % g_hash.hash_numslots[1];
                    (*num_hashneighbors)++;
                }
            }

            return h;
        }

        int get_vertex(bsp_state &state, const math::vec3v &in, int planenum)
        {
            math::vec3v vert;
            for (int i = 0; i < 3; i++)
            {
                if (std::fabs(in[i] - std::floor(in[i] + 0.5)) < 0.001)
                    vert[i] = std::floor(in[i] + 0.5);
                else
                    vert[i] = in[i];
            }

            int num_hashneighbors;
            int hashneighbors[max_hash_neighbors];
            int h = hash_vec(vert, &num_hashneighbors, hashneighbors);

            for (int i = 0; i < num_hashneighbors; i++)
            {
                for (hashvert *hv = g_hash.hashverts[hashneighbors[i]]; hv; hv = hv->next)
                {
                    if (std::fabs(hv->point[0] - vert[0]) < point_epsilon
                        && std::fabs(hv->point[1] - vert[1]) < point_epsilon
                        && std::fabs(hv->point[2] - vert[2]) < point_epsilon)
                    {
                        hv->numedges++;
                        if (hv->numplanes == 3)
                            return hv->num; // already known to be a corner
                        for (int j = 0; j < hv->numplanes; j++)
                        {
                            if (hv->planenums[j] == planenum)
                                return hv->num; // already know this plane
                        }
                        if (hv->numplanes != 2)
                            hv->planenums[hv->numplanes] = planenum;
                        hv->numplanes++;
                        return hv->num;
                    }
                }
            }

            if ((int)state.map->vertexes.size() >= limits::max_map_verts)
            {
                if (!state.vertex_limit_exceeded)
                {
                    state.vertex_limit_exceeded = true;
                    state.first_excess_vertex = vert;
                }
            }

            g_hash.hvertex.emplace_back();
            hashvert *hv = &g_hash.hvertex.back();
            hv->numedges = 1;
            hv->numplanes = 1;
            hv->planenums[0] = planenum;
            hv->next = g_hash.hashverts[h];
            g_hash.hashverts[h] = hv;
            hv->point = vert;
            hv->num = (int)state.map->vertexes.size();

            // emit a vertex
            format::dvertex_t dv;
            dv.point[0] = (float)vert[0];
            dv.point[1] = (float)vert[1];
            dv.point[2] = (float)vert[2];
            state.map->vertexes.push_back(dv);

            return hv->num;
        }
    }

    // don't allow four way edges
    int get_edge(bsp_state &state, const math::vec3v &p1, const math::vec3v &p2, face *f)
    {
        int v1 = get_vertex(state, p1, f->planenum);
        int v2 = get_vertex(state, p2, f->planenum);

        // Once an index no longer fits in dedge_t, edge output is unusable.
        // Vertex hashing remains valid, so keep visiting every face to obtain
        // the exact projected vertex total and fail after the pass.
        if (state.vertex_limit_exceeded)
            return 0;

        int i;
        for (i = g_hash.firstmodeledge; i < (int)state.map->edges.size(); i++)
        {
            format::dedge_t *edge = &state.map->edges[(size_t)i];
            if (v1 == edge->v[1] && v2 == edge->v[0]
                && !g_hash.edgefaces[(size_t)i * 2 + 1]
                && g_hash.edgefaces[(size_t)i * 2]->contents == f->contents
                && g_hash.edgefaces[(size_t)i * 2]->planenum != (f->planenum ^ 1))
            {
                g_hash.edgefaces[(size_t)i * 2 + 1] = f;
                return -i;
            }
        }

        // emit an edge
        if ((int)state.map->edges.size() >= limits::max_map_edges)
            err::fatal("exceeded max_map_edges");
        format::dedge_t edge;
        edge.v[0] = (unsigned short)v1;
        edge.v[1] = (unsigned short)v2;
        state.map->edges.push_back(edge);
        g_hash.edgefaces[(size_t)i * 2] = f;

        return i;
    }

    void make_face_edges(bsp_state &state)
    {
        init_hash();
        state.vertex_limit_exceeded = false;
        state.vertex_model_start = state.map->vertexes.size();
        state.first_excess_vertex = {};
        g_hash.firstmodeledge = (int)state.map->edges.size();
    }

    void fail_if_vertex_limit_exceeded(const bsp_state &state)
    {
        if (!state.vertex_limit_exceeded)
            return;

        const std::size_t projected = state.map->vertexes.size();
        const std::size_t over =
            projected > (std::size_t)limits::max_map_verts
                ? projected - (std::size_t)limits::max_map_verts
                : 0;
        const double over_budget =
            (double)over * 100.0 / (double)limits::max_map_verts;
        const double reduction =
            projected != 0 ? (double)over * 100.0 / (double)projected : 0.0;
        const int model = state.nummodels - 1;
        const format::entity *ent = entity_for_model(state, model);

        err::fatal(
            "BSP vertex limit exceeded\n"
            "  model              %d (%s; origin \"%s\"; targetname \"%s\")\n"
            "  projected vertices %zu / %d\n"
            "  exceeds limit by   %zu vertices (%.2f%% over budget)\n"
            "  reduction needed   at least %zu unique vertices (%.2f%% of projected)\n"
            "  model contribution %zu unique vertices\n"
            "  first excess at    (%.3f %.3f %.3f)\n"
            "  note               this is a hard GoldSrc BSP limit; edge vertex "
            "indices are 16 bit\n"
            "  action             simplify or repartition fragmented brushwork, then "
            "compare the projected total",
            model,
            ent ? ent->value("classname") : "unknown entity",
            ent ? ent->value("origin") : "",
            ent ? ent->value("targetname") : "",
            projected, limits::max_map_verts,
            over, over_budget,
            over, reduction,
            projected - state.vertex_model_start,
            (double)state.first_excess_vertex[0],
            (double)state.first_excess_vertex[1],
            (double)state.first_excess_vertex[2]);
    }

    // if the face is larger than the surface cache in either texture
    // direction, carve a valid sized piece off and insert the remainder in
    // the next link
    void subdivide_face(bsp_state &state, face *f, face **prevptr)
    {
        // special (non surface cached) faces don't need subdivision
        if (f->texturenum == -1)
            return;
        const format::texinfo_t *tex = &state.map->texinfo[(size_t)f->texturenum];

        if (tex->flags & tex_special)
            return;
        if (f->style == face_style::hint)
            return;
        if (f->style == face_style::skip)
            return;
        if (f->style == face_style::null)
            return; // ideally these have tex_special set; here just in case
        if (f->style == face_style::discardable)
            return;

        for (int axis = 0; axis < 2; axis++)
        {
            while (true)
            {
                vec_t mins = 99999999;
                vec_t maxs = -99999999;

                for (int i = 0; i < f->numpoints; i++)
                {
                    vec_t v = math::dot(f->pts[i], tex->vecs[axis]);
                    if (v < mins)
                        mins = v;
                    if (v > maxs)
                        maxs = v;
                }

                if ((maxs - mins) <= state.options.subdivide_size)
                    break;

                // split it
                math::vec3v temp{(vec_t)tex->vecs[axis][0], (vec_t)tex->vecs[axis][1],
                                 (vec_t)tex->vecs[axis][2]};
                vec_t v = math::normalize(temp);

                plane split;
                split.normal = temp;
                split.dist = (mins + state.options.subdivide_size - limits::texture_step) / v;
                face *next = f->next;
                face *front;
                face *back;
                split_face(state, f, &split, &front, &back);
                // the plane did not divide the polygon: every point landed on
                // one side, or a half collapsed in split_face's colinear point
                // cleanup. in practice this only happens to long thin slivers,
                // whose fragments both degenerate away. split_face only frees
                // the original when it hands back both halves, so f is still
                // valid and still linked here.
                //
                // such a face cannot be made to fit a lightmap: control only
                // reaches here when the extent is already past subdivide_size,
                // and rad counts luxels over the grid (ceil(max/16) -
                // floor(min/16)), so even an extent equal to the limit can
                // straddle one cell too many. left in, it reaches rad with
                // extents past max_surface_extent and kills the compile with
                // "Bad surface extents". its area is negligible, so drop it;
                // copy_faces_to_node skips discardable faces.
                if (!front || !back)
                {
                    f->style = face_style::discardable;
                    state.unsplittable_faces++;
                    break;
                }
                front->next = next;
                back->next = front;
                *prevptr = back;
                f = back;
            }
        }
    }
}
