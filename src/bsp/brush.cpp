#include "internal.h"

#include "../common/error.h"

namespace bsp
{
    side *alloc_side()
    {
        return new side();
    }

    void free_side(side *s)
    {
        delete s;
    }

    side *new_side_from_side(const side *s)
    {
        side *news = alloc_side();
        news->plane_ = s->plane_;
        news->winding_ = s->winding_;
        return news;
    }

    brush *alloc_brush()
    {
        return new brush();
    }

    void free_brush(brush *b)
    {
        side *next;
        for (side *s = b->sides; s; s = next)
        {
            next = s->next;
            free_side(s);
        }
        delete b;
    }

    brush *new_brush_from_brush(const brush *b)
    {
        brush *newb = alloc_brush();
        side **pnews = &newb->sides;
        for (side *s = b->sides; s; s = s->next, pnews = &(*pnews)->next)
            *pnews = new_side_from_side(s);
        return newb;
    }

    // clips every side winding to the split plane and adds the split plane
    // itself as a new side, exactly like the reference clipbrush
    void clip_brush(brush **b, const plane *split)
    {
        side *s;
        side **pnext;
        for (pnext = &(*b)->sides, s = *pnext; s; s = *pnext)
        {
            if (s->winding_.clip_in_place(split->normal, split->dist, false))
            {
                pnext = &s->next;
            }
            else
            {
                *pnext = s->next;
                free_side(s);
            }
        }
        if (!(*b)->sides)
        {
            // empty brush
            free_brush(*b);
            *b = nullptr;
            return;
        }
        math::winding w = math::winding::from_plane(split->normal, split->dist,
                                                    (vec_t)plane_winding_range);
        for (s = (*b)->sides; s; s = s->next)
        {
            if (!w.clip_in_place(s->plane_.normal, s->plane_.dist, false))
                break;
        }
        if (!w.empty())
        {
            s = alloc_side();
            s->plane_ = *split;
            s->winding_ = std::move(w);
            s->next = (*b)->sides;
            (*b)->sides = s;
        }
    }

    // 'in' is freed (or handed to one of the outputs)
    void split_brush(brush *in, const plane *split, brush **front, brush **back)
    {
        in->next = nullptr;
        bool onfront = false;
        bool onback = false;
        for (side *s = in->sides; s; s = s->next)
        {
            switch (s->winding_.on_plane_side(split->normal, split->dist, 2 * (vec_t)math::on_epsilon))
            {
            case math::winding::side_cross:
                onfront = true;
                onback = true;
                break;
            case math::winding::side_front:
                onfront = true;
                break;
            case math::winding::side_back:
                onback = true;
                break;
            case math::winding::side_on:
                break;
            }
            if (onfront && onback)
                break;
        }
        if (!onfront && !onback)
        {
            free_brush(in);
            *front = nullptr;
            *back = nullptr;
            return;
        }
        if (!onfront)
        {
            *front = nullptr;
            *back = in;
            return;
        }
        if (!onback)
        {
            *front = in;
            *back = nullptr;
            return;
        }
        *front = in;
        *back = new_brush_from_brush(in);
        plane frontclip = *split;
        plane backclip = *split;
        backclip.normal = -backclip.normal;
        backclip.dist = -backclip.dist;
        clip_brush(front, &frontclip);
        clip_brush(back, &backclip);
    }

    brush *brush_from_box(const math::vec3v &mins, const math::vec3v &maxs)
    {
        brush *b = alloc_brush();
        plane planes[6];
        for (int k = 0; k < 3; k++)
        {
            planes[k].normal = {};
            planes[k].normal[k] = 1.0;
            planes[k].dist = mins[k];
            planes[k + 3].normal = {};
            planes[k + 3].normal[k] = -1.0;
            planes[k + 3].dist = -maxs[k];
        }
        b->sides = alloc_side();
        b->sides->plane_ = planes[0];
        b->sides->winding_ = math::winding::from_plane(planes[0].normal, planes[0].dist,
                                                       (vec_t)plane_winding_range);
        for (int k = 1; k < 6; k++)
        {
            clip_brush(&b, &planes[k]);
            if (b == nullptr)
                break;
        }
        return b;
    }

    void calc_brush_bounds(const brush *b, math::vec3v &mins, math::vec3v &maxs)
    {
        mins.x = mins.y = mins.z = (vec_t)bogus_range;
        maxs.x = maxs.y = maxs.z = (vec_t)-bogus_range;
        for (side *s = b->sides; s; s = s->next)
        {
            math::bounding_box winding_bounds;
            s->winding_.bounds(winding_bounds);
            for (int i = 0; i < 3; i++)
            {
                if (winding_bounds.mins[i] < mins[i])
                    mins[i] = winding_bounds.mins[i];
                if (winding_bounds.maxs[i] > maxs[i])
                    maxs[i] = winding_bounds.maxs[i];
            }
        }
    }

    node *alloc_node()
    {
        return new node();
    }
}
