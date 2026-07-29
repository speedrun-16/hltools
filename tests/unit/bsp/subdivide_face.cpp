#include "bsp/internal.h"
#include "format/bsp/data.h"
#include "support/test.h"

// A face whose points are colinear has no area, so both halves of any split
// collapse to nothing once split_face removes their colinear points and it
// hands back neither a front nor a back. subdivide_face has to notice that the
// plane did not divide anything and stop; walking on to the next face instead
// dereferences a null pointer when the sliver is last in the chain.

namespace
{
    // one texel per unit along u, so the face's texel extent equals its length
    // in world units and the default 240 subdivide size is easy to exceed
    format::texinfo_t unit_texinfo()
    {
        format::texinfo_t tex = {};
        tex.vecs[0][0] = 1.0f;
        tex.vecs[1][1] = 1.0f;
        tex.miptex = 0;
        tex.flags = 0;
        return tex;
    }

    // faces have to come from the pool: split_face frees the original once it
    // hands back both halves, so a stack face would corrupt the heap
    bsp::face *new_face()
    {
        bsp::face *f = bsp::alloc_face();
        f->texturenum = 0;
        f->style = bsp::face_style::normal;
        f->next = nullptr;
        return f;
    }

    // colinear along x and 1000 units long: over the subdivide size, and
    // degenerate enough that neither fragment survives
    bsp::face *make_colinear_face()
    {
        bsp::face *f = new_face();
        f->numpoints = 3;
        f->pts[0] = {0, 0, 0};
        f->pts[1] = {500, 0, 0};
        f->pts[2] = {1000, 0, 0};
        return f;
    }
}

test("subdivide_face leaves a degenerate face alone instead of running off the chain")
{
    format::map_data map;
    map.texinfo.push_back(unit_texinfo());

    bsp::bsp_state state;
    state.map = &map;

    bsp::face *f = make_colinear_face();

    bsp::face *chain = f;
    bsp::subdivide_face(state, chain, &chain);

    // the face could not be split, so it stays where it was and the chain head
    // still points at it rather than at a null successor
    expect(chain == f);
    expect(f->next == nullptr);
    expect(f->numpoints == 3);
}

test("subdivide_face still splits a face that is genuinely too large")
{
    format::map_data map;
    map.texinfo.push_back(unit_texinfo());

    bsp::bsp_state state;
    state.map = &map;

    // a real quad 1000 units across: every fragment must come in under the
    // subdivide size once the face has been carved up
    bsp::face *f = new_face();
    f->numpoints = 4;
    f->pts[0] = {0, 0, 0};
    f->pts[1] = {1000, 0, 0};
    f->pts[2] = {1000, 300, 0};
    f->pts[3] = {0, 300, 0};

    // one call only carves off a single valid slice and leaves the remainder
    // next in the chain, so drive the whole list the way copy_faces_to_node
    // does
    bsp::face *chain = f;
    bsp::face **prevptr = &chain;
    while (*prevptr)
    {
        bsp::subdivide_face(state, *prevptr, prevptr);
        prevptr = &(*prevptr)->next;
    }

    int fragments = 0;
    for (bsp::face *piece = chain; piece; piece = piece->next)
    {
        require(piece->numpoints > 0);
        for (int axis = 0; axis < 2; axis++)
        {
            vec_t mins = 1e30, maxs = -1e30;
            for (int i = 0; i < piece->numpoints; i++)
            {
                vec_t v = math::dot(piece->pts[i], map.texinfo[0].vecs[axis]);
                mins = v < mins ? v : mins;
                maxs = v > maxs ? v : maxs;
            }
            expect(maxs - mins <= state.options.subdivide_size + 1);
        }
        fragments++;
    }
    expect(fragments > 1);
}
