#include "internal.h"

#include <cmath>

namespace bsp
{
    namespace
    {
        constexpr double continuous_epsilon = math::on_epsilon;

        // if two polygons share a common edge and the edges that meet at the
        // common points are both inside the other polygons, merge them
        // returns nullptr if they couldn't be merged; the originals are not
        // freed
        face *try_merge(bsp_state &state, face *f1, face *f2)
        {
            if (f1->numpoints == -1 || f2->numpoints == -1)
                return nullptr;
            if (f1->texturenum != f2->texturenum)
                return nullptr;
            if (f1->contents != f2->contents)
                return nullptr;
            if (f1->planenum != f2->planenum)
                return nullptr;
            if (f1->style != f2->style)
                return nullptr;
            if (f1->detail_level != f2->detail_level)
                return nullptr;

            // find a common edge
            const math::vec3v *p1 = nullptr;
            const math::vec3v *p2 = nullptr;
            int i, j = 0, k;
            for (i = 0; i < f1->numpoints; i++)
            {
                p1 = &f1->pts[i];
                p2 = &f1->pts[(i + 1) % f1->numpoints];
                for (j = 0; j < f2->numpoints; j++)
                {
                    const math::vec3v &p3 = f2->pts[j];
                    const math::vec3v &p4 = f2->pts[(j + 1) % f2->numpoints];
                    for (k = 0; k < 3; k++)
                    {
                        if (std::fabs((*p1)[k] - p4[k]) > math::on_epsilon)
                            break;
                        if (std::fabs((*p2)[k] - p3[k]) > math::on_epsilon)
                            break;
                    }
                    if (k == 3)
                        break;
                }
                if (j < f2->numpoints)
                    break;
            }

            if (i == f1->numpoints)
                return nullptr; // no matching edges

            // check the slope of the connected lines: if the slopes are
            // colinear, the point can be removed
            const plane *fplane = &state.planes[(size_t)f1->planenum];
            math::vec3v planenormal = fplane->normal;

            const math::vec3v *back = &f1->pts[(i + f1->numpoints - 1) % f1->numpoints];
            math::vec3v delta;
            math::vec3v normal;
            math::subtract(*p1, *back, delta);
            math::cross(planenormal, delta, normal);
            math::normalize(normal);

            back = &f2->pts[(j + 2) % f2->numpoints];
            math::subtract(*back, *p1, delta);
            vec_t dot = math::dot(delta, normal);
            if (dot > continuous_epsilon)
                return nullptr; // not a convex polygon
            bool keep1 = dot < -continuous_epsilon;

            back = &f1->pts[(i + 2) % f1->numpoints];
            math::subtract(*back, *p2, delta);
            math::cross(planenormal, delta, normal);
            math::normalize(normal);

            back = &f2->pts[(j + f2->numpoints - 1) % f2->numpoints];
            math::subtract(*back, *p2, delta);
            dot = math::dot(delta, normal);
            if (dot > continuous_epsilon)
                return nullptr; // not a convex polygon
            bool keep2 = dot < -continuous_epsilon;

            // build the new polygon
            if (f1->numpoints + f2->numpoints > max_edges_per_face)
                return nullptr;

            face *newf = new_face_from_face(f1);

            // copy first polygon
            for (k = (i + 1) % f1->numpoints; k != i; k = (k + 1) % f1->numpoints)
            {
                if (k == (i + 1) % f1->numpoints && !keep2)
                    continue;
                newf->pts[newf->numpoints] = f1->pts[k];
                newf->numpoints++;
            }

            // copy second polygon
            for (int l = (j + 1) % f2->numpoints; l != j; l = (l + 1) % f2->numpoints)
            {
                if (l == (j + 1) % f2->numpoints && !keep1)
                    continue;
                newf->pts[newf->numpoints] = f2->pts[l];
                newf->numpoints++;
            }

            return newf;
        }

        face *merge_face_to_list(bsp_state &state, face *f, face *list)
        {
            for (face *other = list; other; other = other->next)
            {
                face *newf = try_merge(state, f, other);
                if (!newf)
                    continue;
                free_face(f);
                other->numpoints = -1; // merged out
                return merge_face_to_list(state, newf, list);
            }

            // didn't merge, so add at the start
            f->next = list;
            return f;
        }

        face *free_merge_list_scraps(face *merged)
        {
            face *head = nullptr;
            face *next;
            for (; merged; merged = next)
            {
                next = merged->next;
                if (merged->numpoints == -1)
                {
                    free_face(merged);
                }
                else
                {
                    merged->next = head;
                    head = merged;
                }
            }
            return head;
        }
    }

    void merge_plane_faces(bsp_state &state, surface *plane_surf)
    {
        face *merged = nullptr;
        face *next;
        for (face *f = plane_surf->faces; f; f = next)
        {
            next = f->next;
            merged = merge_face_to_list(state, f, merged);
        }

        // chain all of the non empty faces to the plane
        plane_surf->faces = free_merge_list_scraps(merged);
    }

    void merge_all(bsp_state &state, surface *surfhead)
    {
        for (surface *surf = surfhead; surf; surf = surf->next)
            merge_plane_faces(state, surf);
    }
}
