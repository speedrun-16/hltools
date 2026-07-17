#include "internal.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <malloc.h> // alloca
#include <memory>
#include <vector>

#include "../common/error.h"
#include "../common/log.h"

#if defined(_M_X64) || defined(_M_AMD64) || defined(__SSE2__) \
    || (defined(_M_IX86_FP) && _M_IX86_FP >= 2)
#define HLVIS_SSE2 1
#include <emmintrin.h>
#endif

namespace vis
{
    namespace
    {
        constexpr double on_epsilon = 0.04;
        constexpr double normal_epsilon = 0.00001;
        constexpr double equal_epsilon = 0.004;
        constexpr int side_front = 0;
        constexpr int side_back = 1;
        constexpr int side_on = 2;

        winding *alloc_stack_winding(pstack *stack)
        {
            for (int i = 0; i < 3; i++)
            {
                if (stack->freewindings[i])
                {
                    stack->freewindings[i] = 0;
                    return &stack->windings[i];
                }
            }
            err::fatal("alloc_stack_winding: failed");
        }

        void free_stack_winding(const winding *w, pstack *stack)
        {
            int i = (int)(w - stack->windings);
            if (i < 0 || i > 2)
                return;
            if (stack->freewindings[i])
                err::fatal("free_stack_winding: already free");
            stack->freewindings[i] = 1;
        }

#if HLVIS_SSE2
        // plane distances for points[0..3] (aos, 12 byte stride) evaluated
        // per lane in exactly the scalar order ((x*nx + y*ny) + z*nz) - dist,
        // so the results are bit identical to the scalar loop; the three
        // loads cover exactly the four points (48 bytes), which is always in
        // bounds because winding::points has max_points_on_winding slots
        inline __m128 plane_dists4(const math::vec3v *pts, const plane &pl)
        {
            const float *p = &pts[0].x;
            __m128 a = _mm_loadu_ps(p);                                 // x0 y0 z0 x1
            __m128 b = _mm_loadu_ps(p + 4);                             // y1 z1 x2 y2
            __m128 c = _mm_loadu_ps(p + 8);                             // z2 x3 y3 z3
            __m128 t1 = _mm_shuffle_ps(a, b, _MM_SHUFFLE(1, 0, 2, 1));  // y0 z0 y1 z1
            __m128 t2 = _mm_shuffle_ps(b, c, _MM_SHUFFLE(2, 1, 3, 2));  // x2 y2 x3 y3
            __m128 x = _mm_shuffle_ps(a, t2, _MM_SHUFFLE(2, 0, 3, 0));  // x0 x1 x2 x3
            __m128 y = _mm_shuffle_ps(t1, t2, _MM_SHUFFLE(3, 1, 2, 0)); // y0 y1 y2 y3
            __m128 z = _mm_shuffle_ps(t1, c, _MM_SHUFFLE(3, 0, 3, 1));  // z0 z1 z2 z3
            __m128 d = _mm_add_ps(_mm_mul_ps(x, _mm_set1_ps(pl.normal.x)),
                                  _mm_mul_ps(y, _mm_set1_ps(pl.normal.y)));
            d = _mm_add_ps(d, _mm_mul_ps(z, _mm_set1_ps(pl.normal.z)));
            return _mm_sub_ps(d, _mm_set1_ps(pl.dist));
        }
#endif

        // note on epsilon comparisons: the scalar code compares the float
        // distance against the double 0.04, and the simd path against the
        // float 0.04f; those classify every float identically, because no
        // float lies between 0.04f and the double 0.04 (0.04f < 0.04 <
        // nextafter(0.04f)), so f > 0.04 and f > 0.04f admit exactly the
        // same set of floats (and likewise for the negated test)

        winding *chop_winding(winding *in, pstack *stack, const plane *split)
        {
            // one extra slot: the wrap around copy below writes index
            // numpoints, which the reference guard (> 128) lets reach the
            // array size
            float dists[max_points_on_winding + 1];
            int sides[max_points_on_winding + 1];
            int counts[3] = {};
            float dot_value;
            math::vec3v mid;

            // local copy so writes through neww (also a winding*) can't force
            // the compiler to reload the bound every iteration
            const int in_numpoints = in->numpoints;
            if (in_numpoints > max_points_on_winding)
                err::fatal("winding with too many sides");

            int i = 0;
#if HLVIS_SSE2
            {
                const __m128 eps = _mm_set1_ps((float)on_epsilon);
                const __m128 negeps = _mm_set1_ps(-(float)on_epsilon);
                const __m128i back_side = _mm_set1_epi32(side_back);
                const __m128i on_side = _mm_set1_epi32(side_on);
                static constexpr int popcount4[16] = {0, 1, 1, 2, 1, 2, 2, 3,
                                                      1, 2, 2, 3, 2, 3, 3, 4};
                for (; i + 4 <= in_numpoints; i += 4)
                {
                    __m128 d = plane_dists4(&in->points[i], *split);
                    _mm_storeu_ps(&dists[i], d);
                    __m128i front = _mm_castps_si128(_mm_cmpgt_ps(d, eps));
                    __m128i back = _mm_castps_si128(_mm_cmplt_ps(d, negeps));
                    // side_front = 0, side_back = 1, side_on = 2
                    __m128i side = _mm_or_si128(
                        _mm_and_si128(back, back_side),
                        _mm_andnot_si128(_mm_or_si128(front, back), on_side));
                    _mm_storeu_si128((__m128i *)&sides[i], side);
                    const int frontmask = _mm_movemask_ps(_mm_castsi128_ps(front));
                    const int backmask = _mm_movemask_ps(_mm_castsi128_ps(back));
                    counts[side_front] += popcount4[frontmask];
                    counts[side_back] += popcount4[backmask];
                    counts[side_on] += 4 - popcount4[frontmask | backmask];
                }
            }
#endif
            for (; i < in_numpoints; i++)
            {
                dot_value = math::dot(in->points[i], split->normal);
                dot_value -= split->dist;
                dists[i] = dot_value;
                if (dot_value > on_epsilon)
                    sides[i] = side_front;
                else if (dot_value < -on_epsilon)
                    sides[i] = side_back;
                else
                    sides[i] = side_on;
                counts[sides[i]]++;
            }

            if (!counts[1])
                return in;
            if (!counts[0])
            {
                free_stack_winding(in, stack);
                return nullptr;
            }

            sides[i] = sides[0];
            dists[i] = dists[0];

            winding *neww = alloc_stack_winding(stack);
            neww->numpoints = 0;

            for (i = 0; i < in_numpoints; i++)
            {
                math::vec3v *p1 = &in->points[i];

                if (neww->numpoints == max_points_on_fixed_winding)
                {
                    logging::warn("chop_winding: rejected(1) due to too many points");
                    free_stack_winding(neww, stack);
                    return in; // can't chop, fall back to the original
                }

                if (sides[i] == side_on)
                {
                    math::copy(*p1, neww->points[neww->numpoints]);
                    neww->numpoints++;
                    continue;
                }
                else if (sides[i] == side_front)
                {
                    math::copy(*p1, neww->points[neww->numpoints]);
                    neww->numpoints++;
                }

                if ((sides[i + 1] == side_on) | (sides[i + 1] == sides[i]))
                    continue;

                if (neww->numpoints == max_points_on_fixed_winding)
                {
                    logging::warn("chop_winding: rejected(2) due to too many points");
                    free_stack_winding(neww, stack);
                    return in; // can't chop, fall back to the original
                }

                unsigned tmp = i + 1;
                if (tmp >= (unsigned)in_numpoints)
                    tmp = 0;
                const math::vec3v *p2 = &in->points[tmp];

                dot_value = dists[i] / (dists[i] - dists[i + 1]);

                const math::vec3v &normal = split->normal;
                const float dist = split->dist;
                for (unsigned j = 0; j < 3; j++)
                {
                    if (normal[j] < (1.0 - normal_epsilon))
                    {
                        if (normal[j] > (-1.0 + normal_epsilon))
                            mid[j] = (*p1)[j] + dot_value * ((*p2)[j] - (*p1)[j]);
                        else
                            mid[j] = -dist;
                    }
                    else
                    {
                        mid[j] = dist;
                    }
                }

                math::copy(mid, neww->points[neww->numpoints]);
                neww->numpoints++;
            }

            free_stack_winding(in, stack);
            return neww;
        }

        void add_plane(pstack *stack, const plane *split)
        {
            for (int j = 0; j < stack->clip_plane_count; j++)
            {
                if (std::fabs(stack->clip_plane[j].dist - split->dist) <= equal_epsilon
                    && math::equal(stack->clip_plane[j].normal, split->normal))
                {
                    return;
                }
            }
            stack->clip_plane[stack->clip_plane_count] = *split;
            stack->clip_plane_count++;
        }

        winding *clip_to_separators(const winding *source, const winding *pass, winding *target,
                                    bool flipclip, pstack *stack)
        {
            int i, j, k, l;
            plane sep_plane;
            math::vec3v v1, v2;
            float d;
            int counts[3];
            bool fliptest;
            const unsigned numpoints = source->numpoints;

            for (i = 0, l = 1; i < (int)numpoints; i++, l++)
            {
                if (l == (int)numpoints)
                    l = 0;

                math::subtract(source->points[l], source->points[i], v1);

                for (j = 0; j < pass->numpoints; j++)
                {
                    math::subtract(pass->points[j], source->points[i], v2);
                    math::cross(v1, v2, sep_plane.normal);
                    if (math::normalize(sep_plane.normal) < on_epsilon)
                        continue;
                    sep_plane.dist = math::dot(pass->points[j], sep_plane.normal);

                    // these two scans stay scalar on purpose: they almost
                    // always decide within the first point or two, so a
                    // 4-lane simd group costs more than it saves (measured)
                    fliptest = false;
                    for (k = 0; k < (int)numpoints; k++)
                    {
                        if ((k == i) | (k == l))
                            continue;
                        d = math::dot(source->points[k], sep_plane.normal) - sep_plane.dist;
                        if (d < -on_epsilon)
                        {
                            fliptest = false;
                            break;
                        }
                        else if (d > on_epsilon)
                        {
                            fliptest = true;
                            break;
                        }
                    }
                    if (k == (int)numpoints)
                        continue;

                    if (fliptest)
                    {
                        math::scale(sep_plane.normal, -1.0f, sep_plane.normal);
                        sep_plane.dist = -sep_plane.dist;
                    }

                    counts[0] = counts[1] = counts[2] = 0;
                    for (k = 0; k < pass->numpoints; k++)
                    {
                        if (k == j)
                            continue;
                        d = math::dot(pass->points[k], sep_plane.normal) - sep_plane.dist;
                        if (d < -on_epsilon)
                            break;
                        else if (d > on_epsilon)
                            counts[0]++;
                        else
                            counts[2]++;
                    }
                    if (k != pass->numpoints)
                        continue;
                    if (!counts[0])
                        continue;

                    if (flipclip)
                    {
                        math::scale(sep_plane.normal, -1.0f, sep_plane.normal);
                        sep_plane.dist = -sep_plane.dist;
                    }

                    if (target != nullptr)
                    {
                        target = chop_winding(target, stack, &sep_plane);
                        if (!target)
                            return nullptr;
                    }
                    else
                    {
                        add_plane(stack, &sep_plane);
                    }

                    break;
                }
            }

            return target;
        }

        // one reusable pstack per recursion depth, per thread: the reference
        // declared pstack as an uninitialized stack local, but our pstack
        // value-initializes ~9kb (mightsee + three 128 point windings), and
        // paying that on every recursion frame dominated the whole leafthread
        // phase; every field is written before it is read, so reuse is safe
        pstack &stack_for_depth(unsigned depth)
        {
            thread_local std::vector<std::unique_ptr<pstack>> pool;
            while (pool.size() <= depth)
                pool.emplace_back(new pstack);
            return *pool[depth];
        }

        void recursive_leaf_flow(vis_state &state, int leafnum, const thread_data *thread,
                                 const pstack *prevstack, unsigned depth)
        {
            pstack &stack = stack_for_depth(depth);
            leaf *leaf_ptr = &state.leafs[leafnum];

            const unsigned offset = leafnum >> 3;
            const unsigned bit = (1 << (leafnum & 7));
            if (!(thread->leafvis[offset] & bit))
            {
                thread->leafvis[offset] |= bit;
                thread->base->numcansee++;
            }

            stack.head = prevstack->head;
            stack.leaf_ = leaf_ptr;
            stack.portal_ = nullptr;
            stack.clip_plane_count = -1;
            stack.clip_plane = nullptr;

            portal **plist = leaf_ptr->portals;
            const unsigned numportals = leaf_ptr->numportals;
            for (unsigned i = 0; i < numportals; i++, plist++)
            {
                portal *p = *plist;

                {
                    const unsigned next_offset = p->leaf >> 3;
                    const unsigned next_bit = 1 << (p->leaf & 7);
                    if (!(stack.head->mightsee[next_offset] & next_bit))
                        continue;
                    if (!(prevstack->mightsee[next_offset] & next_bit))
                        continue;
                }

                {
                    // single fused pass over the bit vectors: compute what this
                    // portal might add and check it against what the base
                    // portal already sees the bound and the "anything new"
                    // reduction live in locals so the compiler can keep them in
                    // registers and vectorize (reading state.bitlongs inside
                    // the loop forced a reload every iteration, because the
                    // uint32 store to *might could alias it)
                    const std::uint64_t *test = (const std::uint64_t *)
                        (p->status == portal_status::done ? p->visbits : p->mightsee);
                    const std::uint64_t *prevmight = (const std::uint64_t *)prevstack->mightsee;
                    const std::uint64_t *vis = (const std::uint64_t *)thread->leafvis;
                    std::uint64_t *might = (std::uint64_t *)stack.mightsee;
                    const unsigned words = state.bitlongs / 2; // bitbytes is a multiple of 8
                    std::uint64_t more = 0;
                    for (unsigned j = 0; j < words; j++)
                    {
                        might[j] = prevmight[j] & test[j];
                        more |= might[j] & ~vis[j];
                    }
                    if (!more)
                        continue; // can't see anything new
                }

                stack.portalplane = &p->plane_;
                plane backplane;
                math::scale(p->plane_.normal, -1.0f, backplane.normal);
                backplane.dist = -p->plane_.dist;

                if (math::equal(prevstack->portalplane->normal, backplane.normal))
                    continue;

                stack.portal_ = p;
                stack.freewindings[0] = 1;
                stack.freewindings[1] = 1;
                stack.freewindings[2] = 1;

                stack.pass = chop_winding(p->winding_, &stack, thread->pstack_head.portalplane);
                if (!stack.pass)
                    continue;

                stack.source = chop_winding(prevstack->source, &stack, &backplane);
                if (!stack.source)
                    continue;

                if (!prevstack->pass)
                {
                    recursive_leaf_flow(state, p->leaf, thread, &stack, depth + 1);
                    continue;
                }

                stack.pass = chop_winding(stack.pass, &stack, prevstack->portalplane);
                if (!stack.pass)
                    continue;

                if (stack.clip_plane_count == -1)
                {
                    // stack allocation like the reference: this is the hottest
                    // recursion in vis and must not touch the heap runs at
                    // most once per recursion frame
                    stack.clip_plane_count = 0;
                    stack.clip_plane = (plane *)alloca(
                        sizeof(plane) * prevstack->source->numpoints * prevstack->pass->numpoints);

                    clip_to_separators(prevstack->source, prevstack->pass, nullptr, false, &stack);
                    clip_to_separators(prevstack->pass, prevstack->source, nullptr, true, &stack);
                }

                if (stack.clip_plane_count > 0)
                {
                    for (unsigned j = 0; j < (unsigned)stack.clip_plane_count && stack.pass != nullptr; j++)
                        stack.pass = chop_winding(stack.pass, &stack, &stack.clip_plane[j]);
                    if (stack.pass == nullptr)
                        continue;
                }

                if (state.options.full)
                {
                    stack.source = clip_to_separators(stack.pass, prevstack->pass, stack.source, false, &stack);
                    if (!stack.source)
                        continue;
                    stack.source = clip_to_separators(prevstack->pass, stack.pass, stack.source, true, &stack);
                    if (!stack.source)
                        continue;
                }

                recursive_leaf_flow(state, p->leaf, thread, &stack, depth + 1);
            }
        }

        void simple_flood(vis_state &state, byte *srcmightsee, int leafnum, byte *portalsee,
                          unsigned *c_leafsee)
        {
            const unsigned offset = leafnum >> 3;
            const unsigned bit = (1 << (leafnum & 7));
            if (srcmightsee[offset] & bit)
                return;
            srcmightsee[offset] |= bit;

            (*c_leafsee)++;
            leaf *leaf_ptr = &state.leafs[leafnum];

            for (unsigned i = 0; i < leaf_ptr->numportals; i++)
            {
                portal *p = leaf_ptr->portals[i];
                if (!portalsee[p - state.portals.data()])
                    continue;
                simple_flood(state, srcmightsee, p->leaf, portalsee, c_leafsee);
            }
        }
    }

    void portal_flow(vis_state &state, portal *p)
    {
        if (p->status != portal_status::working)
            err::fatal("portal_flow: reflowed");

        p->visbits = (byte *)std::calloc(1, state.bitbytes);

        thread_data data;
        std::memset(&data, 0, sizeof(data));
        data.leafvis = p->visbits;
        data.base = p;

        data.pstack_head.head = &data.pstack_head;
        data.pstack_head.portal_ = p;
        data.pstack_head.source = p->winding_;
        data.pstack_head.portalplane = &p->plane_;

        for (unsigned i = 0; i < state.bitlongs; i++)
            ((std::uint32_t *)data.pstack_head.mightsee)[i] = ((std::uint32_t *)p->mightsee)[i];

        recursive_leaf_flow(state, p->leaf, &data, &data.pstack_head, 0);
        p->status = portal_status::done;
    }

    void base_portal_vis(vis_state &state, int index)
    {
        portal *p = state.portals.data() + index;
        p->mightsee = (byte *)std::calloc(1, state.bitbytes);

        const int portalsize = state.numportals * 2;
        std::vector<byte> portalsee(portalsize);

        for (int j = 0; j < portalsize; j++)
        {
            if (j == index)
                continue;

            portal *tp = state.portals.data() + j;
            winding *w = tp->winding_;
            int k;
            float d;
            for (k = 0; k < w->numpoints; k++)
            {
                d = math::dot(w->points[k], p->plane_.normal) - p->plane_.dist;
                if (d > on_epsilon)
                    break;
            }
            if (k == w->numpoints)
                continue;

            w = p->winding_;
            for (k = 0; k < w->numpoints; k++)
            {
                d = math::dot(w->points[k], tp->plane_.normal) - tp->plane_.dist;
                if (d < -on_epsilon)
                    break;
            }
            if (k == w->numpoints)
                continue;

            portalsee[j] = 1;
        }

        simple_flood(state, p->mightsee, p->leaf, portalsee.data(), &p->nummightsee);
    }

    void max_dist_vis(vis_state &, int)
    {
        err::fatal("-maxdistance is not implemented in hlvis yet");
    }
}
