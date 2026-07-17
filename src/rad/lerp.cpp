#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "../common/error.h"
#include "internal.h"

// sample interpolation: around every patch a local triangulation of nearby
// patch spots is built (unfolded onto the face plane across smooth edges), and
// lightmap samples blend the patch light through it during the final pass

namespace rad
{
    struct interpolation
    {
        struct point
        {
            int patchnum;
            vec_t weight;
        };

        bool isbiased;
        vec_t totalweight;
        std::vector<point> points;
    };

    struct local_triangulation
    {
        struct wedge
        {
            enum shape_type
            {
                shape_triangular,
                shape_convex,
                shape_concave,
                shape_square_left,
                shape_square_right,
            };

            shape_type shape;
            int leftpatchnum;
            vec3v leftspot;
            vec3v leftdirection;
            // the right side equals the left side of the next wedge

            vec3v wedgenormal; // for custom usage
        };
        struct hull_point
        {
            vec3v spot;
            vec3v direction;
        };

        plane pl;
        math::winding winding;
        vec3v center; // center is on the plane

        vec3v normal;
        int patchnum;
        std::vector<int> neighborfaces; // including the face itself

        std::vector<wedge> sortedwedges;         // in clockwise order (same as winding)
        std::vector<hull_point> sortedhullpoints; // in clockwise order (same as winding)
    };

    struct face_triangulation
    {
        struct wall
        {
            vec3v points[2];
            vec3v direction;
            vec3v normal;
        };

        int facenum;
        std::vector<int> neighbors; // including the face itself
        std::vector<wall> walls;
        std::vector<local_triangulation *> localtriangulations;
        std::vector<int> usedpatches;
    };

    namespace
    {
        // if one of the angles in a triangle exceeds this threshold, the most
        // distant point is removed or the triangle breaks into a convex wedge
        const double triangle_shape_threshold = 115.0 * math::pi / 180;

        // if the surface formed by the face and its neighbors is not flat, it
        // is unfolded onto the face plane; this maps a position on the surface
        // to the unfolded spot relative to the center
        bool calc_adapted_spot(const rad_state &state, const local_triangulation *lt,
                               const vec3v &position, int surface, vec3v &spot)
        {
            int i;
            vec_t dot;
            vec3v surfacespot;
            vec_t dist;
            vec_t dist2;
            vec3v phongnormal;
            vec_t frac;
            vec3v middle;
            vec3v v;

            for (i = 0; i < (int)lt->neighborfaces.size(); i++)
            {
                if (lt->neighborfaces[i] == surface)
                {
                    break;
                }
            }
            if (i == (int)lt->neighborfaces.size())
            {
                math::clear(spot);
                return false;
            }

            math::subtract(position, lt->center, surfacespot);
            dot = math::dot(surfacespot, lt->normal);
            math::multiply_add(surfacespot, -dot, lt->normal, spot);

            // use the phong normal instead of the face normal, because the
            // phong normal is a continuous function
            get_phong_normal(state, surface, position, phongnormal);
            dot = math::dot(spot, phongnormal);
            if (std::fabs(dot) > math::on_epsilon)
            {
                frac = math::dot(surfacespot, phongnormal) / dot;
                // correct some extreme cases
                frac = frac < 1 ? frac : 1;
                frac = 0 > frac ? 0 : frac;
            }
            else
            {
                frac = 0;
            }
            math::scale(spot, frac, middle);

            dist = (vec_t)math::length(spot);
            math::subtract(surfacespot, middle, v);
            dist2 = (vec_t)(math::length(middle) + math::length(v));

            if (dist > math::on_epsilon && std::fabs(dist2 - dist) > math::on_epsilon)
            {
                math::scale(spot, dist2 / dist, spot);
            }
            return true;
        }

        vec_t get_angle(const vec3v &leftdirection, const vec3v &rightdirection, const vec3v &normal)
        {
            vec_t angle;
            vec3v v;

            math::cross(rightdirection, leftdirection, v);
            angle = std::atan2(math::dot(v, normal), math::dot(rightdirection, leftdirection));

            return angle;
        }

        vec_t get_angle_diff(vec_t angle, vec_t base)
        {
            vec_t diff;

            diff = angle - base;
            if (diff < 0)
            {
                diff = (vec_t)(diff + 2 * math::pi);
            }
            return diff;
        }

        vec_t get_frac(const vec3v &leftspot, const vec3v &rightspot, const vec3v &direction, const vec3v &normal)
        {
            vec3v v;
            vec_t dot1;
            vec_t dot2;
            vec_t frac;

            math::cross(direction, normal, v);
            dot1 = math::dot(leftspot, v);
            dot2 = math::dot(rightspot, v);

            // dot1 <= 0 < dot2
            if (dot1 >= -math::normal_epsilon)
            {
                frac = 0.0;
            }
            else if (dot2 <= math::normal_epsilon)
            {
                frac = 1.0;
            }
            else
            {
                frac = dot1 / (dot1 - dot2);
                frac = frac < 1 ? frac : 1;
                frac = 0 > frac ? 0 : frac;
            }

            return frac;
        }

        vec_t get_direction(const vec3v &spot, const vec3v &normal, vec3v &direction_out)
        {
            vec_t dot;

            dot = math::dot(spot, normal);
            math::multiply_add(spot, -dot, normal, direction_out);
            return math::normalize(direction_out);
        }

        // returns true when the point is inside the hull region (with
        // boundary), even if the weight is 0
        bool calc_weight(const local_triangulation *lt, const vec3v &spot, vec_t *weight_out)
        {
            vec3v direction;
            const local_triangulation::hull_point *hp1;
            const local_triangulation::hull_point *hp2;
            bool istoofar;
            vec_t ratio;

            int i;
            int j;
            vec_t angle;
            std::vector<vec_t> angles;
            vec_t frac;
            vec_t len;
            vec_t dist;

            if (get_direction(spot, lt->normal, direction) <= 2 * math::on_epsilon)
            {
                *weight_out = 1.0;
                return true;
            }

            if ((int)lt->sortedhullpoints.size() == 0)
            {
                *weight_out = 0.0;
                return false;
            }

            angles.resize((size_t)lt->sortedhullpoints.size());
            for (i = 0; i < (int)lt->sortedhullpoints.size(); i++)
            {
                angle = get_angle(lt->sortedhullpoints[(size_t)i].direction, direction, lt->normal);
                angles[(size_t)i] = get_angle_diff(angle, 0);
            }
            j = 0;
            for (i = 1; i < (int)lt->sortedhullpoints.size(); i++)
            {
                if (angles[(size_t)i] < angles[(size_t)j])
                {
                    j = i;
                }
            }
            hp1 = &lt->sortedhullpoints[(size_t)j];
            hp2 = &lt->sortedhullpoints[(size_t)((j + 1) % (int)lt->sortedhullpoints.size())];

            frac = get_frac(hp1->spot, hp2->spot, direction, lt->normal);

            len = (1 - frac) * math::dot(hp1->spot, direction) + frac * math::dot(hp2->spot, direction);
            dist = math::dot(spot, direction);
            if (len <= math::on_epsilon / 4 || dist > len + 2 * math::on_epsilon)
            {
                istoofar = true;
                ratio = 1.0;
            }
            else if (dist >= len - math::on_epsilon)
            {
                istoofar = false; // would show many places as green in -drawlerp mode if true
                ratio = 1.0;      // to prevent excessively small weight
            }
            else
            {
                istoofar = false;
                ratio = dist / len;
                ratio = ratio < 1 ? ratio : 1;
                ratio = 0 > ratio ? 0 : ratio;
            }

            *weight_out = 1 - ratio;
            return !istoofar;
        }

        void calc_interpolation_square(const local_triangulation *lt, int i, const vec3v &spot, interpolation *interp)
        {
            const local_triangulation::wedge *w1;
            const local_triangulation::wedge *w2;
            const local_triangulation::wedge *w3;
            vec_t weights[4];
            vec_t dot1;
            vec_t dot2;
            vec_t dot;
            vec3v normal1;
            vec3v normal2;
            vec3v normal;
            vec_t frac;
            vec_t frac_near;
            vec_t frac_far;
            vec_t ratio;
            vec3v mid_far;
            vec3v mid_near;

            w1 = &lt->sortedwedges[(size_t)i];
            w2 = &lt->sortedwedges[(size_t)((i + 1) % (int)lt->sortedwedges.size())];
            w3 = &lt->sortedwedges[(size_t)((i + 2) % (int)lt->sortedwedges.size())];
            if (w1->shape != local_triangulation::wedge::shape_square_left
                || w2->shape != local_triangulation::wedge::shape_square_right)
            {
                err::fatal("calc_interpolation_square: internal error: not square.");
            }

            weights[0] = 0.0;
            weights[1] = 0.0;
            weights[2] = 0.0;
            weights[3] = 0.0;

            // find mid_near on (o,p3), mid_far on (p1,p2), spot on (mid_near,mid_far)
            math::cross(w1->leftdirection, lt->normal, normal1);
            math::normalize(normal1);
            math::cross(w2->wedgenormal, lt->normal, normal2);
            math::normalize(normal2);
            dot1 = math::dot(spot, normal1) - 0;
            dot2 = math::dot(spot, normal2) - math::dot(w3->leftspot, normal2);
            if (dot1 <= math::normal_epsilon)
            {
                frac = 0.0;
            }
            else if (dot2 <= math::normal_epsilon)
            {
                frac = 1.0;
            }
            else
            {
                frac = dot1 / (dot1 + dot2);
                frac = frac < 1 ? frac : 1;
                frac = 0 > frac ? 0 : frac;
            }

            dot1 = math::dot(w3->leftspot, normal1) - 0;
            dot2 = 0 - math::dot(w3->leftspot, normal2);
            if (dot1 <= math::normal_epsilon)
            {
                frac_near = 1.0;
            }
            else if (dot2 <= math::normal_epsilon)
            {
                frac_near = 0.0;
            }
            else
            {
                frac_near = (frac * dot2) / ((1 - frac) * dot1 + frac * dot2);
            }
            math::scale(w3->leftspot, frac_near, mid_near);

            dot1 = math::dot(w2->leftspot, normal1) - 0;
            dot2 = math::dot(w1->leftspot, normal2) - math::dot(w3->leftspot, normal2);
            if (dot1 <= math::normal_epsilon)
            {
                frac_far = 1.0;
            }
            else if (dot2 <= math::normal_epsilon)
            {
                frac_far = 0.0;
            }
            else
            {
                frac_far = (frac * dot2) / ((1 - frac) * dot1 + frac * dot2);
            }
            math::scale(w1->leftspot, 1 - frac_far, mid_far);
            math::multiply_add(mid_far, frac_far, w2->leftspot, mid_far);

            math::cross(lt->normal, w3->leftdirection, normal);
            math::normalize(normal);
            dot = math::dot(spot, normal) - 0;
            dot1 = (1 - frac_far) * math::dot(w1->leftspot, normal) + frac_far * math::dot(w2->leftspot, normal) - 0;
            if (dot <= math::normal_epsilon)
            {
                ratio = 0.0;
            }
            else if (dot >= dot1)
            {
                ratio = 1.0;
            }
            else
            {
                ratio = dot / dot1;
                ratio = ratio < 1 ? ratio : 1;
                ratio = 0 > ratio ? 0 : ratio;
            }

            weights[0] = (vec_t)(weights[0] + 0.5 * (1 - ratio) * (1 - frac_near));
            weights[3] = (vec_t)(weights[3] + 0.5 * (1 - ratio) * frac_near);
            weights[1] = (vec_t)(weights[1] + 0.5 * ratio * (1 - frac_far));
            weights[2] = (vec_t)(weights[2] + 0.5 * ratio * frac_far);

            // find mid_near on (o,p1), mid_far on (p2,p3), spot on (mid_near,mid_far)
            math::cross(lt->normal, w3->leftdirection, normal1);
            math::normalize(normal1);
            math::cross(w1->wedgenormal, lt->normal, normal2);
            math::normalize(normal2);
            dot1 = math::dot(spot, normal1) - 0;
            dot2 = math::dot(spot, normal2) - math::dot(w1->leftspot, normal2);
            if (dot1 <= math::normal_epsilon)
            {
                frac = 0.0;
            }
            else if (dot2 <= math::normal_epsilon)
            {
                frac = 1.0;
            }
            else
            {
                frac = dot1 / (dot1 + dot2);
                frac = frac < 1 ? frac : 1;
                frac = 0 > frac ? 0 : frac;
            }

            dot1 = math::dot(w1->leftspot, normal1) - 0;
            dot2 = 0 - math::dot(w1->leftspot, normal2);
            if (dot1 <= math::normal_epsilon)
            {
                frac_near = 1.0;
            }
            else if (dot2 <= math::normal_epsilon)
            {
                frac_near = 0.0;
            }
            else
            {
                frac_near = (frac * dot2) / ((1 - frac) * dot1 + frac * dot2);
            }
            math::scale(w1->leftspot, frac_near, mid_near);

            dot1 = math::dot(w2->leftspot, normal1) - 0;
            dot2 = math::dot(w3->leftspot, normal2) - math::dot(w1->leftspot, normal2);
            if (dot1 <= math::normal_epsilon)
            {
                frac_far = 1.0;
            }
            else if (dot2 <= math::normal_epsilon)
            {
                frac_far = 0.0;
            }
            else
            {
                frac_far = (frac * dot2) / ((1 - frac) * dot1 + frac * dot2);
            }
            math::scale(w3->leftspot, 1 - frac_far, mid_far);
            math::multiply_add(mid_far, frac_far, w2->leftspot, mid_far);

            math::cross(w1->leftdirection, lt->normal, normal);
            math::normalize(normal);
            dot = math::dot(spot, normal) - 0;
            dot1 = (1 - frac_far) * math::dot(w3->leftspot, normal) + frac_far * math::dot(w2->leftspot, normal) - 0;
            if (dot <= math::normal_epsilon)
            {
                ratio = 0.0;
            }
            else if (dot >= dot1)
            {
                ratio = 1.0;
            }
            else
            {
                ratio = dot / dot1;
                ratio = ratio < 1 ? ratio : 1;
                ratio = 0 > ratio ? 0 : ratio;
            }

            weights[0] = (vec_t)(weights[0] + 0.5 * (1 - ratio) * (1 - frac_near));
            weights[1] = (vec_t)(weights[1] + 0.5 * (1 - ratio) * frac_near);
            weights[3] = (vec_t)(weights[3] + 0.5 * ratio * (1 - frac_far));
            weights[2] = (vec_t)(weights[2] + 0.5 * ratio * frac_far);

            interp->isbiased = false;
            interp->totalweight = 1.0;
            interp->points.resize(4);
            interp->points[0].patchnum = lt->patchnum;
            interp->points[0].weight = weights[0];
            interp->points[1].patchnum = w1->leftpatchnum;
            interp->points[1].weight = weights[1];
            interp->points[2].patchnum = w2->leftpatchnum;
            interp->points[2].weight = weights[2];
            interp->points[3].patchnum = w3->leftpatchnum;
            interp->points[3].weight = weights[3];
        }

        // the interpolation function is defined over the entire plane, so this
        // never fails
        void calc_interpolation(const local_triangulation *lt, const vec3v &spot, interpolation *interp)
        {
            vec3v direction;
            const local_triangulation::wedge *w;
            const local_triangulation::wedge *wnext;

            int i;
            int j;
            vec_t angle;
            std::vector<vec_t> angles;

            if (get_direction(spot, lt->normal, direction) <= 2 * math::on_epsilon)
            {
                // spot happens to be at the center
                interp->isbiased = false;
                interp->totalweight = 1.0;
                interp->points.resize(1);
                interp->points[0].patchnum = lt->patchnum;
                interp->points[0].weight = 1.0;
                return;
            }

            if ((int)lt->sortedwedges.size() == 0) // this local triangulation only has a center patch
            {
                interp->isbiased = true;
                interp->totalweight = 1.0;
                interp->points.resize(1);
                interp->points[0].patchnum = lt->patchnum;
                interp->points[0].weight = 1.0;
                return;
            }

            // find the wedge with minimum non negative angle (counterclockwise)
            // past the spot
            angles.resize((size_t)lt->sortedwedges.size());
            for (i = 0; i < (int)lt->sortedwedges.size(); i++)
            {
                angle = get_angle(lt->sortedwedges[(size_t)i].leftdirection, direction, lt->normal);
                angles[(size_t)i] = get_angle_diff(angle, 0);
            }
            j = 0;
            for (i = 1; i < (int)lt->sortedwedges.size(); i++)
            {
                if (angles[(size_t)i] < angles[(size_t)j])
                {
                    j = i;
                }
            }
            w = &lt->sortedwedges[(size_t)j];
            wnext = &lt->sortedwedges[(size_t)((j + 1) % (int)lt->sortedwedges.size())];

            // different wedge types use different interpolation methods
            switch (w->shape)
            {
            case local_triangulation::wedge::shape_square_left:
            case local_triangulation::wedge::shape_square_right:
            case local_triangulation::wedge::shape_triangular:
                // w->wedgenormal is undefined
                {
                    vec_t frac;
                    vec_t len;
                    vec_t dist;
                    bool istoofar;
                    vec_t ratio;

                    frac = get_frac(w->leftspot, wnext->leftspot, direction, lt->normal);

                    len = (1 - frac) * math::dot(w->leftspot, direction) + frac * math::dot(wnext->leftspot, direction);
                    dist = math::dot(spot, direction);
                    if (len <= math::on_epsilon / 4 || dist > len + 2 * math::on_epsilon)
                    {
                        istoofar = true;
                        ratio = 1.0;
                    }
                    else if (dist >= len - math::on_epsilon)
                    {
                        istoofar = false;
                        ratio = 1.0;
                    }
                    else
                    {
                        istoofar = false;
                        ratio = dist / len;
                        ratio = ratio < 1 ? ratio : 1;
                        ratio = 0 > ratio ? 0 : ratio;
                    }

                    if (istoofar)
                    {
                        interp->isbiased = true;
                        interp->totalweight = 1.0;
                        interp->points.resize(2);
                        interp->points[0].patchnum = w->leftpatchnum;
                        interp->points[0].weight = 1 - frac;
                        interp->points[1].patchnum = wnext->leftpatchnum;
                        interp->points[1].weight = frac;
                    }
                    else if (w->shape == local_triangulation::wedge::shape_square_left)
                    {
                        i = (int)(w - &lt->sortedwedges[0]);
                        calc_interpolation_square(lt, i, spot, interp);
                    }
                    else if (w->shape == local_triangulation::wedge::shape_square_right)
                    {
                        i = (int)(w - &lt->sortedwedges[0]);
                        i = (i - 1 + (int)lt->sortedwedges.size()) % (int)lt->sortedwedges.size();
                        calc_interpolation_square(lt, i, spot, interp);
                    }
                    else
                    {
                        interp->isbiased = false;
                        interp->totalweight = 1.0;
                        interp->points.resize(3);
                        interp->points[0].patchnum = lt->patchnum;
                        interp->points[0].weight = 1 - ratio;
                        interp->points[1].patchnum = w->leftpatchnum;
                        interp->points[1].weight = ratio * (1 - frac);
                        interp->points[2].patchnum = wnext->leftpatchnum;
                        interp->points[2].weight = ratio * frac;
                    }
                }
                break;
            case local_triangulation::wedge::shape_convex:
                // w->wedgenormal is the unit vector pointing from w->leftspot to wnext->leftspot
                {
                    vec_t dot;
                    vec_t dot1;
                    vec_t dot2;
                    vec_t frac;

                    dot1 = math::dot(w->leftspot, w->wedgenormal) - math::dot(spot, w->wedgenormal);
                    dot2 = math::dot(wnext->leftspot, w->wedgenormal) - math::dot(spot, w->wedgenormal);
                    dot = 0 - math::dot(spot, w->wedgenormal);
                    // for the convex type: dot1 < dot < dot2

                    if (dot1 >= -math::normal_epsilon) // 0 <= dot1 < dot < dot2
                    {
                        interp->isbiased = true;
                        interp->totalweight = 1.0;
                        interp->points.resize(1);
                        interp->points[0].patchnum = w->leftpatchnum;
                        interp->points[0].weight = 1.0;
                    }
                    else if (dot2 <= math::normal_epsilon) // dot1 < dot < dot2 <= 0
                    {
                        interp->isbiased = true;
                        interp->totalweight = 1.0;
                        interp->points.resize(1);
                        interp->points[0].patchnum = wnext->leftpatchnum;
                        interp->points[0].weight = 1.0;
                    }
                    else if (dot > 0) // dot1 < 0 < dot < dot2
                    {
                        frac = dot1 / (dot1 - dot);
                        frac = frac < 1 ? frac : 1;
                        frac = 0 > frac ? 0 : frac;

                        interp->isbiased = true;
                        interp->totalweight = 1.0;
                        interp->points.resize(2);
                        interp->points[0].patchnum = w->leftpatchnum;
                        interp->points[0].weight = 1 - frac;
                        interp->points[1].patchnum = lt->patchnum;
                        interp->points[1].weight = frac;
                    }
                    else // dot1 < dot <= 0 < dot2
                    {
                        frac = dot / (dot - dot2);
                        frac = frac < 1 ? frac : 1;
                        frac = 0 > frac ? 0 : frac;

                        interp->isbiased = true;
                        interp->totalweight = 1.0;
                        interp->points.resize(2);
                        interp->points[0].patchnum = lt->patchnum;
                        interp->points[0].weight = 1 - frac;
                        interp->points[1].patchnum = wnext->leftpatchnum;
                        interp->points[1].weight = frac;
                    }
                }
                break;
            case local_triangulation::wedge::shape_concave:
                {
                    vec_t len;
                    vec_t dist;
                    vec_t ratio;

                    if (math::dot(spot, w->wedgenormal) < 0) // the spot is closer to the left edge than the right edge
                    {
                        len = math::dot(w->leftspot, w->leftdirection);
                        dist = math::dot(spot, w->leftdirection);
                        if (dist <= math::normal_epsilon)
                        {
                            interp->isbiased = true;
                            interp->totalweight = 1.0;
                            interp->points.resize(1);
                            interp->points[0].patchnum = lt->patchnum;
                            interp->points[0].weight = 1.0;
                        }
                        else if (dist >= len)
                        {
                            interp->isbiased = true;
                            interp->totalweight = 1.0;
                            interp->points.resize(1);
                            interp->points[0].patchnum = w->leftpatchnum;
                            interp->points[0].weight = 1.0;
                        }
                        else
                        {
                            ratio = dist / len;
                            ratio = ratio < 1 ? ratio : 1;
                            ratio = 0 > ratio ? 0 : ratio;

                            interp->isbiased = true;
                            interp->totalweight = 1.0;
                            interp->points.resize(2);
                            interp->points[0].patchnum = lt->patchnum;
                            interp->points[0].weight = 1 - ratio;
                            interp->points[1].patchnum = w->leftpatchnum;
                            interp->points[1].weight = ratio;
                        }
                    }
                    else // the spot is closer to the right edge than the left edge
                    {
                        len = math::dot(wnext->leftspot, wnext->leftdirection);
                        dist = math::dot(spot, wnext->leftdirection);
                        if (dist <= math::normal_epsilon)
                        {
                            interp->isbiased = true;
                            interp->totalweight = 1.0;
                            interp->points.resize(1);
                            interp->points[0].patchnum = lt->patchnum;
                            interp->points[0].weight = 1.0;
                        }
                        else if (dist >= len)
                        {
                            interp->isbiased = true;
                            interp->totalweight = 1.0;
                            interp->points.resize(1);
                            interp->points[0].patchnum = wnext->leftpatchnum;
                            interp->points[0].weight = 1.0;
                        }
                        else
                        {
                            ratio = dist / len;
                            ratio = ratio < 1 ? ratio : 1;
                            ratio = 0 > ratio ? 0 : ratio;

                            interp->isbiased = true;
                            interp->totalweight = 1.0;
                            interp->points.resize(2);
                            interp->points[0].patchnum = lt->patchnum;
                            interp->points[0].weight = 1 - ratio;
                            interp->points[1].patchnum = wnext->leftpatchnum;
                            interp->points[1].weight = ratio;
                        }
                    }
                }
                break;
            default:
                err::fatal("calc_interpolation: internal error: invalid wedge type.");
                break;
            }
        }

        void apply_interpolation(const rad_state &state, const interpolation *interp,
                                 int numstyles, const int *styles, vec3v *outs)
        {
            int i;
            int j;

            for (j = 0; j < numstyles; j++)
            {
                math::clear(outs[j]);
            }
            if (interp->totalweight <= 0)
            {
                return;
            }
            for (i = 0; i < (int)interp->points.size(); i++)
            {
                for (j = 0; j < numstyles; j++)
                {
                    const vec3v *b = get_total_light(&state.patches[(size_t)interp->points[(size_t)i].patchnum], styles[j]);
                    math::multiply_add(outs[j], interp->points[(size_t)i].weight / interp->totalweight, *b, outs[j]);
                }
            }
        }

        bool test_line_segment_intersect_wall(const face_triangulation *facetrian, const vec3v &p1, const vec3v &p2)
        {
            int i;
            const face_triangulation::wall *wall;
            vec_t front;
            vec_t back;
            vec_t dot1;
            vec_t dot2;
            vec_t dot;
            vec_t bottom;
            vec_t top;
            vec_t frac;

            for (i = 0; i < (int)facetrian->walls.size(); i++)
            {
                wall = &facetrian->walls[(size_t)i];
                bottom = math::dot(wall->points[0], wall->direction);
                top = math::dot(wall->points[1], wall->direction);
                front = math::dot(p1, wall->normal) - math::dot(wall->points[0], wall->normal);
                back = math::dot(p2, wall->normal) - math::dot(wall->points[0], wall->normal);
                if ((front > math::on_epsilon && back > math::on_epsilon)
                    || (front < -math::on_epsilon && back < -math::on_epsilon))
                {
                    continue;
                }
                dot1 = math::dot(p1, wall->direction);
                dot2 = math::dot(p2, wall->direction);
                if (std::fabs(front) <= 2 * math::on_epsilon && std::fabs(back) <= 2 * math::on_epsilon)
                {
                    vec_t maxdot = dot1 > dot2 ? dot1 : dot2;
                    vec_t mindot = dot1 < dot2 ? dot1 : dot2;
                    top = top < maxdot ? top : maxdot;
                    bottom = bottom > mindot ? bottom : mindot;
                }
                else
                {
                    frac = front / (front - back);
                    frac = frac < 1 ? frac : 1;
                    frac = 0 > frac ? 0 : frac;
                    dot = dot1 + frac * (dot2 - dot1);
                    top = top < dot ? top : dot;
                    bottom = bottom > dot ? bottom : dot;
                }
                if (top - bottom >= -math::on_epsilon)
                {
                    return true;
                }
            }

            return false;
        }

        bool test_far_patch(const local_triangulation *lt, const vec3v &p2, const math::winding &p2winding)
        {
            int i;
            vec3v v;
            vec_t dist;
            vec_t size1;
            vec_t size2;

            size1 = 0;
            for (i = 0; i < lt->winding.size(); i++)
            {
                math::subtract(lt->winding[i], lt->center, v);
                dist = (vec_t)math::length(v);
                if (dist > size1)
                {
                    size1 = dist;
                }
            }

            size2 = 0;
            for (i = 0; i < p2winding.size(); i++)
            {
                math::subtract(p2winding[i], p2, v);
                dist = (vec_t)math::length(v);
                if (dist > size2)
                {
                    size2 = dist;
                }
            }

            math::subtract(p2, lt->center, v);
            dist = (vec_t)math::length(v);

            return dist > 1.4 * (size1 + size2);
        }

        void gather_patches(rad_state &state, local_triangulation *lt, const face_triangulation *facetrian)
        {
            int i;
            int facenum2;
            const plane *dp2;
            const patch *patch2;
            int patchnum2;
            vec3v v;
            local_triangulation::wedge point;
            std::vector<local_triangulation::wedge> points;
            std::vector<std::pair<vec_t, int>> angles;
            vec_t angle;

            if (!state.options.lerp_enabled)
            {
                lt->sortedwedges.resize(0);
                return;
            }

            points.resize(0);
            for (i = 0; i < (int)lt->neighborfaces.size(); i++)
            {
                facenum2 = lt->neighborfaces[(size_t)i];
                dp2 = plane_from_face_number(state, (unsigned)facenum2);
                for (patch2 = state.face_patches[(size_t)facenum2]; patch2; patch2 = patch2->next)
                {
                    patchnum2 = (int)(patch2 - state.patches.data());

                    point.leftpatchnum = patchnum2;
                    math::multiply_add(patch2->origin, -patch_hunt_offset, dp2->normal, v);

                    // do permission tests using the original position of the patch
                    if (patchnum2 == lt->patchnum || point_in_winding(lt->winding, lt->pl, v))
                    {
                        continue;
                    }
                    if (facenum2 != facetrian->facenum && test_line_segment_intersect_wall(facetrian, lt->center, v))
                    {
                        continue;
                    }
                    if (test_far_patch(lt, v, *patch2->winding))
                    {
                        continue;
                    }

                    // store the adapted position of the patch
                    if (!calc_adapted_spot(state, lt, v, facenum2, point.leftspot))
                    {
                        continue;
                    }
                    if (get_direction(point.leftspot, lt->normal, point.leftdirection) <= 2 * math::on_epsilon)
                    {
                        continue;
                    }
                    points.push_back(point);
                }
            }

            // sort the patches into clockwise order
            angles.resize((size_t)points.size());
            for (i = 0; i < (int)points.size(); i++)
            {
                angle = get_angle(points[0].leftdirection, points[(size_t)i].leftdirection, lt->normal);
                if (i == 0)
                {
                    angle = 0.0;
                }
                angles[(size_t)i].first = get_angle_diff(angle, 0);
                angles[(size_t)i].second = i;
            }
            std::sort(angles.begin(), angles.end());

            lt->sortedwedges.resize((size_t)points.size());
            for (i = 0; i < (int)points.size(); i++)
            {
                lt->sortedwedges[(size_t)i] = points[(size_t)angles[(size_t)i].second];
            }
        }

        void purge_patches(local_triangulation *lt)
        {
            std::vector<local_triangulation::wedge> points;
            int i;
            int cur;
            std::vector<int> next;
            std::vector<int> prev;
            std::vector<int> valid;
            std::vector<std::pair<vec_t, int>> dists;
            vec_t angle;
            vec3v normal;
            vec3v v;

            points.swap(lt->sortedwedges);
            lt->sortedwedges.resize(0);

            next.resize((size_t)points.size());
            prev.resize((size_t)points.size());
            valid.resize((size_t)points.size());
            dists.resize((size_t)points.size());
            for (i = 0; i < (int)points.size(); i++)
            {
                next[(size_t)i] = (i + 1) % (int)points.size();
                prev[(size_t)i] = (i - 1 + (int)points.size()) % (int)points.size();
                valid[(size_t)i] = 1;
                dists[(size_t)i].first = math::dot(points[(size_t)i].leftspot, points[(size_t)i].leftdirection);
                dists[(size_t)i].second = i;
            }
            std::sort(dists.begin(), dists.end());

            for (i = 0; i < (int)points.size(); i++)
            {
                cur = dists[(size_t)i].second;
                if (valid[(size_t)cur] == 0)
                {
                    continue;
                }
                valid[(size_t)cur] = 2; // mark the current patch as final

                math::cross(points[(size_t)cur].leftdirection, lt->normal, normal);
                math::normalize(normal);
                math::scale(normal, std::cos(triangle_shape_threshold), v);
                math::multiply_add(v, std::sin(triangle_shape_threshold), points[(size_t)cur].leftdirection, v);
                while (next[(size_t)cur] != cur && valid[(size_t)next[(size_t)cur]] != 2)
                {
                    angle = get_angle(points[(size_t)cur].leftdirection, points[(size_t)next[(size_t)cur]].leftdirection, lt->normal);
                    if (std::fabs(angle) <= (1.0 * math::pi / 180) ||
                        (get_angle_diff(angle, 0) <= math::pi + math::normal_epsilon
                         && math::dot(points[(size_t)next[(size_t)cur]].leftspot, v) >= math::dot(points[(size_t)cur].leftspot, v) - math::on_epsilon / 2))
                    {
                        // remove the next patch
                        valid[(size_t)next[(size_t)cur]] = 0;
                        next[(size_t)cur] = next[(size_t)next[(size_t)cur]];
                        prev[(size_t)next[(size_t)cur]] = cur;
                        continue;
                    }
                    // the triangle is good
                    break;
                }

                math::cross(lt->normal, points[(size_t)cur].leftdirection, normal);
                math::normalize(normal);
                math::scale(normal, std::cos(triangle_shape_threshold), v);
                math::multiply_add(v, std::sin(triangle_shape_threshold), points[(size_t)cur].leftdirection, v);
                while (prev[(size_t)cur] != cur && valid[(size_t)prev[(size_t)cur]] != 2)
                {
                    angle = get_angle(points[(size_t)prev[(size_t)cur]].leftdirection, points[(size_t)cur].leftdirection, lt->normal);
                    if (std::fabs(angle) <= (1.0 * math::pi / 180) ||
                        (get_angle_diff(angle, 0) <= math::pi + math::normal_epsilon
                         && math::dot(points[(size_t)prev[(size_t)cur]].leftspot, v) >= math::dot(points[(size_t)cur].leftspot, v) - math::on_epsilon / 2))
                    {
                        // remove the previous patch
                        valid[(size_t)prev[(size_t)cur]] = 0;
                        prev[(size_t)cur] = prev[(size_t)prev[(size_t)cur]];
                        next[(size_t)prev[(size_t)cur]] = cur;
                        continue;
                    }
                    // the triangle is good
                    break;
                }
            }

            for (i = 0; i < (int)points.size(); i++)
            {
                if (valid[(size_t)i] == 2)
                {
                    lt->sortedwedges.push_back(points[(size_t)i]);
                }
            }
        }

        void place_hull_points(local_triangulation *lt)
        {
            int i;
            int j;
            int n;
            vec3v v;
            vec_t dot;
            vec_t angle;
            local_triangulation::hull_point hp;
            std::vector<local_triangulation::hull_point> spots;
            std::vector<std::pair<vec_t, int>> angles;
            const local_triangulation::wedge *w;
            const local_triangulation::wedge *wnext;
            std::vector<local_triangulation::hull_point> arc_spots;
            std::vector<vec_t> arc_angles;
            std::vector<int> next;
            std::vector<int> prev;
            vec_t frac;
            vec_t len;
            vec_t dist;

            spots.reserve((size_t)lt->winding.size());
            spots.resize(0);
            for (i = 0; i < lt->winding.size(); i++)
            {
                math::subtract(lt->winding[i], lt->center, v);
                dot = math::dot(v, lt->normal);
                math::multiply_add(v, -dot, lt->normal, hp.spot);
                if (!get_direction(hp.spot, lt->normal, hp.direction))
                {
                    continue;
                }
                spots.push_back(hp);
            }

            if ((int)lt->sortedwedges.size() == 0)
            {
                angles.resize((size_t)spots.size());
                for (i = 0; i < (int)spots.size(); i++)
                {
                    angle = get_angle(spots[0].direction, spots[(size_t)i].direction, lt->normal);
                    if (i == 0)
                    {
                        angle = 0.0;
                    }
                    angles[(size_t)i].first = get_angle_diff(angle, 0);
                    angles[(size_t)i].second = i;
                }
                std::sort(angles.begin(), angles.end());
                lt->sortedhullpoints.resize(0);
                for (i = 0; i < (int)spots.size(); i++)
                {
                    lt->sortedhullpoints.push_back(spots[(size_t)angles[(size_t)i].second]);
                }
                return;
            }

            lt->sortedhullpoints.resize(0);
            for (i = 0; i < (int)lt->sortedwedges.size(); i++)
            {
                w = &lt->sortedwedges[(size_t)i];
                wnext = &lt->sortedwedges[(size_t)((i + 1) % (int)lt->sortedwedges.size())];

                angles.resize((size_t)spots.size());
                for (j = 0; j < (int)spots.size(); j++)
                {
                    angle = get_angle(w->leftdirection, spots[(size_t)j].direction, lt->normal);
                    angles[(size_t)j].first = get_angle_diff(angle, 0);
                    angles[(size_t)j].second = j;
                }
                std::sort(angles.begin(), angles.end());
                angle = get_angle(w->leftdirection, wnext->leftdirection, lt->normal);
                if ((int)lt->sortedwedges.size() == 1)
                {
                    angle = (vec_t)(2 * math::pi);
                }
                else
                {
                    angle = get_angle_diff(angle, 0);
                }

                arc_spots.resize((size_t)spots.size() + 2);
                arc_angles.resize((size_t)spots.size() + 2);
                next.resize((size_t)spots.size() + 2);
                prev.resize((size_t)spots.size() + 2);

                math::copy(w->leftspot, arc_spots[0].spot);
                math::copy(w->leftdirection, arc_spots[0].direction);
                arc_angles[0] = 0;
                next[0] = 1;
                prev[0] = -1;
                n = 1;
                for (j = 0; j < (int)spots.size(); j++)
                {
                    if (math::normal_epsilon <= angles[(size_t)j].first && angles[(size_t)j].first <= angle - math::normal_epsilon)
                    {
                        arc_spots[(size_t)n] = spots[(size_t)angles[(size_t)j].second];
                        arc_angles[(size_t)n] = angles[(size_t)j].first;
                        next[(size_t)n] = n + 1;
                        prev[(size_t)n] = n - 1;
                        n++;
                    }
                }
                math::copy(wnext->leftspot, arc_spots[(size_t)n].spot);
                math::copy(wnext->leftdirection, arc_spots[(size_t)n].direction);
                arc_angles[(size_t)n] = angle;
                next[(size_t)n] = -1;
                prev[(size_t)n] = n - 1;
                n++;

                for (j = 1; next[(size_t)j] != -1; j = next[(size_t)j])
                {
                    while (prev[(size_t)j] != -1)
                    {
                        if (arc_angles[(size_t)next[(size_t)j]] - arc_angles[(size_t)prev[(size_t)j]] <= math::pi + math::normal_epsilon)
                        {
                            frac = get_frac(arc_spots[(size_t)prev[(size_t)j]].spot, arc_spots[(size_t)next[(size_t)j]].spot, arc_spots[(size_t)j].direction, lt->normal);
                            len = (1 - frac) * math::dot(arc_spots[(size_t)prev[(size_t)j]].spot, arc_spots[(size_t)j].direction)
                                + frac * math::dot(arc_spots[(size_t)next[(size_t)j]].spot, arc_spots[(size_t)j].direction);
                            dist = math::dot(arc_spots[(size_t)j].spot, arc_spots[(size_t)j].direction);
                            if (dist <= len + math::normal_epsilon)
                            {
                                j = prev[(size_t)j];
                                next[(size_t)j] = next[(size_t)next[(size_t)j]];
                                prev[(size_t)next[(size_t)j]] = j;
                                continue;
                            }
                        }
                        break;
                    }
                }

                for (j = 0; next[(size_t)j] != -1; j = next[(size_t)j])
                {
                    lt->sortedhullpoints.push_back(arc_spots[(size_t)j]);
                }
            }
        }

        bool try_make_square(local_triangulation *lt, int i)
        {
            local_triangulation::wedge *w1;
            local_triangulation::wedge *w2;
            local_triangulation::wedge *w3;
            vec3v v;
            vec3v dir1;
            vec3v dir2;
            vec_t angle;

            w1 = &lt->sortedwedges[(size_t)i];
            w2 = &lt->sortedwedges[(size_t)((i + 1) % (int)lt->sortedwedges.size())];
            w3 = &lt->sortedwedges[(size_t)((i + 2) % (int)lt->sortedwedges.size())];

            // (o, p1, p2) and (o, p2, p3) must be triangles and not in a square
            if (w1->shape != local_triangulation::wedge::shape_triangular
                || w2->shape != local_triangulation::wedge::shape_triangular)
            {
                return false;
            }

            // (o, p1, p3) must be a triangle
            angle = get_angle(w1->leftdirection, w3->leftdirection, lt->normal);
            angle = get_angle_diff(angle, 0);
            if (angle >= triangle_shape_threshold)
            {
                return false;
            }

            // (p2, p1, p3) must be a triangle
            math::subtract(w1->leftspot, w2->leftspot, v);
            if (!get_direction(v, lt->normal, dir1))
            {
                return false;
            }
            math::subtract(w3->leftspot, w2->leftspot, v);
            if (!get_direction(v, lt->normal, dir2))
            {
                return false;
            }
            angle = get_angle(dir2, dir1, lt->normal);
            angle = get_angle_diff(angle, 0);
            if (angle >= triangle_shape_threshold)
            {
                return false;
            }

            w1->shape = local_triangulation::wedge::shape_square_left;
            for (int k = 0; k < 3; k++)
                w1->wedgenormal[k] = 0.0f - dir1[k];
            w2->shape = local_triangulation::wedge::shape_square_right;
            math::copy(dir2, w2->wedgenormal);
            return true;
        }

        void find_squares(local_triangulation *lt)
        {
            int i;
            local_triangulation::wedge *w;
            std::vector<std::pair<vec_t, int>> dists;

            if ((int)lt->sortedwedges.size() <= 2)
            {
                return;
            }

            dists.resize((size_t)lt->sortedwedges.size());
            for (i = 0; i < (int)lt->sortedwedges.size(); i++)
            {
                w = &lt->sortedwedges[(size_t)i];
                dists[(size_t)i].first = math::dot(w->leftspot, w->leftdirection);
                dists[(size_t)i].second = i;
            }
            std::sort(dists.begin(), dists.end());

            for (i = 0; i < (int)lt->sortedwedges.size(); i++)
            {
                try_make_square(lt, dists[(size_t)i].second);
                try_make_square(lt, (dists[(size_t)i].second - 2 + (int)lt->sortedwedges.size()) % (int)lt->sortedwedges.size());
            }
        }

        local_triangulation *create_local_triangulation(rad_state &state, const face_triangulation *facetrian, int patchnum)
        {
            local_triangulation *lt;
            int i;
            const patch *pt;
            vec_t dot;
            int facenum;
            local_triangulation::wedge *w;
            local_triangulation::wedge *wnext;
            vec_t angle;
            vec3v v;
            vec3v normal;

            facenum = facetrian->facenum;
            pt = &state.patches[(size_t)patchnum];
            lt = new local_triangulation;

            // fill basic information for this local triangulation
            lt->pl = *plane_from_face_number(state, (unsigned)facenum);
            lt->pl.dist += math::dot(state.face_offset[(size_t)facenum], lt->pl.normal);
            lt->winding = *pt->winding;
            math::multiply_add(pt->origin, -patch_hunt_offset, lt->pl.normal, lt->center);
            dot = math::dot(lt->center, lt->pl.normal) - lt->pl.dist;
            math::multiply_add(lt->center, -dot, lt->pl.normal, lt->center);
            if (!point_in_winding_noedge(lt->winding, lt->pl, lt->center, default_edge_width))
            {
                snap_to_winding_noedge(lt->winding, lt->pl, lt->center, default_edge_width, 4 * default_edge_width);
            }
            math::copy(lt->pl.normal, lt->normal);
            lt->patchnum = patchnum;
            lt->neighborfaces = facetrian->neighbors;

            // gather all patches from nearby faces
            gather_patches(state, lt, facetrian);

            // remove distant patches
            purge_patches(lt);

            // calculate wedge types
            for (i = 0; i < (int)lt->sortedwedges.size(); i++)
            {
                w = &lt->sortedwedges[(size_t)i];
                wnext = &lt->sortedwedges[(size_t)((i + 1) % (int)lt->sortedwedges.size())];

                angle = get_angle(w->leftdirection, wnext->leftdirection, lt->normal);
                angle = get_angle_diff(angle, 0);
                if ((int)lt->sortedwedges.size() == 1)
                {
                    angle = (vec_t)(2 * math::pi);
                }

                if (angle <= math::pi + math::normal_epsilon)
                {
                    if (angle < triangle_shape_threshold)
                    {
                        w->shape = local_triangulation::wedge::shape_triangular;
                        math::clear(w->wedgenormal);
                    }
                    else
                    {
                        w->shape = local_triangulation::wedge::shape_convex;
                        math::subtract(wnext->leftspot, w->leftspot, v);
                        get_direction(v, lt->normal, w->wedgenormal);
                    }
                }
                else
                {
                    w->shape = local_triangulation::wedge::shape_concave;
                    math::add(wnext->leftdirection, w->leftdirection, v);
                    math::cross(lt->normal, v, normal);
                    math::subtract(wnext->leftdirection, w->leftdirection, v);
                    math::add(normal, v, normal);
                    get_direction(normal, lt->normal, w->wedgenormal);
                }
            }
            find_squares(lt);

            // calculate hull points
            place_hull_points(lt);

            return lt;
        }

        void free_local_triangulation(local_triangulation *lt)
        {
            delete lt;
        }

        void find_neighbors(rad_state &state, face_triangulation *facetrian)
        {
            int i;
            int j;
            int e;
            const edgeshare *es;
            int side;
            const facelist *fl;
            int facenum;
            int facenum2;
            const format::dface_t *f;
            const format::dface_t *f2;
            const plane *dp;
            const plane *dp2;

            facenum = facetrian->facenum;
            f = &state.map->faces[(size_t)facenum];
            dp = plane_from_face(state, f);

            facetrian->neighbors.resize(0);

            facetrian->neighbors.push_back(facenum);

            for (i = 0; i < f->numedges; i++)
            {
                e = state.map->surfedges[(size_t)(f->firstedge + i)];
                es = &state.edgeshares[(size_t)abs(e)];
                if (!es->smooth)
                {
                    continue;
                }
                f2 = es->faces[e > 0 ? 1 : 0];
                facenum2 = (int)(f2 - state.map->faces.data());
                dp2 = plane_from_face(state, f2);
                if (math::dot(dp->normal, dp2->normal) < -math::normal_epsilon)
                {
                    continue;
                }
                for (j = 0; j < (int)facetrian->neighbors.size(); j++)
                {
                    if (facetrian->neighbors[(size_t)j] == facenum2)
                    {
                        break;
                    }
                }
                if (j == (int)facetrian->neighbors.size())
                {
                    facetrian->neighbors.push_back(facenum2);
                }
            }

            for (i = 0; i < f->numedges; i++)
            {
                e = state.map->surfedges[(size_t)(f->firstedge + i)];
                es = &state.edgeshares[(size_t)abs(e)];
                if (!es->smooth)
                {
                    continue;
                }
                for (side = 0; side < 2; side++)
                {
                    for (fl = es->vertex_facelist[side]; fl; fl = fl->next)
                    {
                        f2 = fl->face;
                        facenum2 = (int)(f2 - state.map->faces.data());
                        dp2 = plane_from_face(state, f2);
                        if (math::dot(dp->normal, dp2->normal) < -math::normal_epsilon)
                        {
                            continue;
                        }
                        for (j = 0; j < (int)facetrian->neighbors.size(); j++)
                        {
                            if (facetrian->neighbors[(size_t)j] == facenum2)
                            {
                                break;
                            }
                        }
                        if (j == (int)facetrian->neighbors.size())
                        {
                            facetrian->neighbors.push_back(facenum2);
                        }
                    }
                }
            }
        }

        void build_walls(rad_state &state, face_triangulation *facetrian)
        {
            int i;
            int j;
            int facenum;
            int facenum2;
            const format::dface_t *f;
            const format::dface_t *f2;
            const plane *dp;
            const plane *dp2;
            int e;
            const edgeshare *es;
            vec_t dot;

            facenum = facetrian->facenum;
            f = &state.map->faces[(size_t)facenum];
            dp = plane_from_face(state, f);

            facetrian->walls.resize(0);

            for (i = 0; i < (int)facetrian->neighbors.size(); i++)
            {
                facenum2 = facetrian->neighbors[(size_t)i];
                f2 = &state.map->faces[(size_t)facenum2];
                dp2 = plane_from_face(state, f2);
                if (math::dot(dp->normal, dp2->normal) <= 0.1)
                {
                    continue;
                }
                for (j = 0; j < f2->numedges; j++)
                {
                    e = state.map->surfedges[(size_t)(f2->firstedge + j)];
                    es = &state.edgeshares[(size_t)abs(e)];
                    if (!es->smooth)
                    {
                        face_triangulation::wall wall;

                        const format::dvertex_t &v0 = state.map->vertexes[state.map->edges[(size_t)abs(e)].v[0]];
                        const format::dvertex_t &v1 = state.map->vertexes[state.map->edges[(size_t)abs(e)].v[1]];
                        vec3v p0{v0.point[0], v0.point[1], v0.point[2]};
                        vec3v p1{v1.point[0], v1.point[1], v1.point[2]};
                        math::add(p0, state.face_offset[(size_t)facenum], wall.points[0]);
                        math::add(p1, state.face_offset[(size_t)facenum], wall.points[1]);
                        math::subtract(wall.points[1], wall.points[0], wall.direction);
                        dot = math::dot(wall.direction, dp->normal);
                        math::multiply_add(wall.direction, -dot, dp->normal, wall.direction);
                        if (math::normalize(wall.direction))
                        {
                            math::cross(wall.direction, dp->normal, wall.normal);
                            math::normalize(wall.normal);
                            facetrian->walls.push_back(wall);
                        }
                    }
                }
            }
        }

        void collect_used_patches(face_triangulation *facetrian)
        {
            int i;
            int j;
            int k;
            int patchnum;
            const local_triangulation *lt;
            const local_triangulation::wedge *w;

            facetrian->usedpatches.resize(0);
            for (i = 0; i < (int)facetrian->localtriangulations.size(); i++)
            {
                lt = facetrian->localtriangulations[(size_t)i];

                patchnum = lt->patchnum;
                for (k = 0; k < (int)facetrian->usedpatches.size(); k++)
                {
                    if (facetrian->usedpatches[(size_t)k] == patchnum)
                    {
                        break;
                    }
                }
                if (k == (int)facetrian->usedpatches.size())
                {
                    facetrian->usedpatches.push_back(patchnum);
                }

                for (j = 0; j < (int)lt->sortedwedges.size(); j++)
                {
                    w = &lt->sortedwedges[(size_t)j];

                    patchnum = w->leftpatchnum;
                    for (k = 0; k < (int)facetrian->usedpatches.size(); k++)
                    {
                        if (facetrian->usedpatches[(size_t)k] == patchnum)
                        {
                            break;
                        }
                    }
                    if (k == (int)facetrian->usedpatches.size())
                    {
                        facetrian->usedpatches.push_back(patchnum);
                    }
                }
            }
        }
    }

    void interpolate_sample_light(rad_state &state, const vec3v &position, int surface,
                                  int numstyles, const int *styles, vec3v *outs)
    {
        const face_triangulation *ft;
        interpolation *maininterp;
        std::vector<vec_t> localweights;
        std::vector<interpolation *> localinterps;

        int i;
        int j;
        int n;
        const face_triangulation *ft2;
        const local_triangulation *lt;
        vec3v spot;
        vec_t weight;
        interpolation *interp;
        const local_triangulation *best;
        vec3v v;
        vec_t dist;
        vec_t bestdist = 0;
        vec_t dot;

        if (surface < 0 || surface >= (int)state.map->faces.size())
        {
            err::fatal("interpolate_sample_light: internal error: surface number out of range.");
        }
        ft = state.facetriangulations[(size_t)surface];
        maininterp = new interpolation;
        maininterp->points.reserve(64);

        // calculate local interpolations and their weights
        localweights.resize(0);
        localinterps.resize(0);
        if (state.options.lerp_enabled)
        {
            for (i = 0; i < (int)ft->neighbors.size(); i++) // for this face and each of its neighbors
            {
                ft2 = state.facetriangulations[(size_t)ft->neighbors[(size_t)i]];
                for (j = 0; j < (int)ft2->localtriangulations.size(); j++) // for each patch on that face
                {
                    lt = ft2->localtriangulations[(size_t)j];
                    if (!calc_adapted_spot(state, lt, position, surface, spot))
                    {
                        continue;
                    }
                    if (!calc_weight(lt, spot, &weight))
                    {
                        continue;
                    }
                    interp = new interpolation;
                    interp->points.reserve(4);
                    calc_interpolation(lt, spot, interp);

                    localweights.push_back(weight);
                    localinterps.push_back(interp);
                }
            }
        }

        // combine into one interpolation
        maininterp->isbiased = false;
        maininterp->totalweight = 0;
        maininterp->points.resize(0);
        for (i = 0; i < (int)localinterps.size(); i++)
        {
            if (localinterps[(size_t)i]->isbiased)
            {
                maininterp->isbiased = true;
            }
            for (j = 0; j < (int)localinterps[(size_t)i]->points.size(); j++)
            {
                weight = localinterps[(size_t)i]->points[(size_t)j].weight * localweights[(size_t)i];
                if (state.patches[(size_t)localinterps[(size_t)i]->points[(size_t)j].patchnum].flags == patch_flag_outside)
                {
                    weight = (vec_t)(weight * 0.01);
                }
                n = (int)maininterp->points.size();
                maininterp->points.resize((size_t)n + 1);
                maininterp->points[(size_t)n].patchnum = localinterps[(size_t)i]->points[(size_t)j].patchnum;
                maininterp->points[(size_t)n].weight = weight;
                maininterp->totalweight += weight;
            }
        }
        if (maininterp->totalweight > 0)
        {
            apply_interpolation(state, maininterp, numstyles, styles, outs);
            if (state.options.drawlerp)
            {
                for (j = 0; j < numstyles; j++)
                {
                    // white or yellow
                    outs[j][0] = 100;
                    outs[j][1] = 100;
                    outs[j][2] = (vec_t)(maininterp->isbiased ? 0 : 100);
                }
            }
        }
        else
        {
            // try again, don't multiply localweights[i] (which equals 0)
            maininterp->isbiased = false;
            maininterp->totalweight = 0;
            maininterp->points.resize(0);
            for (i = 0; i < (int)localinterps.size(); i++)
            {
                if (localinterps[(size_t)i]->isbiased)
                {
                    maininterp->isbiased = true;
                }
                for (j = 0; j < (int)localinterps[(size_t)i]->points.size(); j++)
                {
                    weight = localinterps[(size_t)i]->points[(size_t)j].weight;
                    if (state.patches[(size_t)localinterps[(size_t)i]->points[(size_t)j].patchnum].flags == patch_flag_outside)
                    {
                        weight = (vec_t)(weight * 0.01);
                    }
                    n = (int)maininterp->points.size();
                    maininterp->points.resize((size_t)n + 1);
                    maininterp->points[(size_t)n].patchnum = localinterps[(size_t)i]->points[(size_t)j].patchnum;
                    maininterp->points[(size_t)n].weight = weight;
                    maininterp->totalweight += weight;
                }
            }
            if (maininterp->totalweight > 0)
            {
                apply_interpolation(state, maininterp, numstyles, styles, outs);
                if (state.options.drawlerp)
                {
                    for (j = 0; j < numstyles; j++)
                    {
                        // red
                        outs[j][0] = 100;
                        outs[j][1] = 0;
                        outs[j][2] = (vec_t)(maininterp->isbiased ? 0 : 100);
                    }
                }
            }
            else
            {
                // worst case, simply use the nearest patch

                best = nullptr;
                for (i = 0; i < (int)ft->localtriangulations.size(); i++)
                {
                    lt = ft->localtriangulations[(size_t)i];
                    math::copy(position, v);
                    snap_to_winding(lt->winding, lt->pl, v);
                    math::subtract(v, position, v);
                    dist = (vec_t)math::length(v);
                    if (best == nullptr || dist < bestdist - math::on_epsilon)
                    {
                        best = lt;
                        bestdist = dist;
                    }
                }

                if (best)
                {
                    lt = best;
                    math::subtract(position, lt->center, spot);
                    dot = math::dot(spot, lt->normal);
                    math::multiply_add(spot, -dot, lt->normal, spot);
                    calc_interpolation(lt, spot, maininterp);

                    maininterp->totalweight = 0;
                    for (j = 0; j < (int)maininterp->points.size(); j++)
                    {
                        if (state.patches[(size_t)maininterp->points[(size_t)j].patchnum].flags == patch_flag_outside)
                        {
                            maininterp->points[(size_t)j].weight = (vec_t)(weight * 0.01);
                        }
                        maininterp->totalweight += maininterp->points[(size_t)j].weight;
                    }
                    apply_interpolation(state, maininterp, numstyles, styles, outs);
                    if (state.options.drawlerp)
                    {
                        for (j = 0; j < numstyles; j++)
                        {
                            // green
                            outs[j][0] = 0;
                            outs[j][1] = 100;
                            outs[j][2] = (vec_t)(maininterp->isbiased ? 0 : 100);
                        }
                    }
                }
                else
                {
                    maininterp->isbiased = true;
                    maininterp->totalweight = 0;
                    maininterp->points.resize(0);
                    apply_interpolation(state, maininterp, numstyles, styles, outs);
                    if (state.options.drawlerp)
                    {
                        for (j = 0; j < numstyles; j++)
                        {
                            // black
                            outs[j][0] = 0;
                            outs[j][1] = 0;
                            outs[j][2] = 0;
                        }
                    }
                }
            }
        }
        delete maininterp;

        for (i = 0; i < (int)localinterps.size(); i++)
        {
            delete localinterps[(size_t)i];
        }
    }

    void create_triangulations(rad_state &state, int facenum)
    {
        face_triangulation *facetrian;
        int patchnum;
        const patch *pt;
        local_triangulation *lt;

        state.facetriangulations[(size_t)facenum] = new face_triangulation;
        facetrian = state.facetriangulations[(size_t)facenum];

        facetrian->facenum = facenum;

        // find neighbors
        find_neighbors(state, facetrian);

        // build walls
        build_walls(state, facetrian);

        // create a local triangulation around each patch
        facetrian->localtriangulations.resize(0);
        for (pt = state.face_patches[(size_t)facenum]; pt; pt = pt->next)
        {
            patchnum = (int)(pt - state.patches.data());
            lt = create_local_triangulation(state, facetrian, patchnum);
            facetrian->localtriangulations.push_back(lt);
        }

        // collect used patches
        collect_used_patches(facetrian);
    }

    void get_triangulation_patches(const rad_state &state, int facenum, int *numpatches, const int **patches)
    {
        const face_triangulation *facetrian;

        facetrian = state.facetriangulations[(size_t)facenum];
        *numpatches = (int)facetrian->usedpatches.size();
        *patches = facetrian->usedpatches.data();
    }

    void free_triangulations(rad_state &state)
    {
        int i;
        int j;
        face_triangulation *facetrian;

        for (i = 0; i < (int)state.map->faces.size(); i++)
        {
            facetrian = state.facetriangulations[(size_t)i];

            for (j = 0; j < (int)facetrian->localtriangulations.size(); j++)
            {
                free_local_triangulation(facetrian->localtriangulations[(size_t)j]);
            }

            delete facetrian;
            state.facetriangulations[(size_t)i] = nullptr;
        }
    }
}
