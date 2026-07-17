#pragma once

#include "vector.h"

// axis aligned bounding box over the stage precision mirrors the old
// boundingbox helper: an empty box starts inverted so the first added point
// initializes it, and the test semantics match the original results

namespace math
{
    template <typename T>
    struct basic_bounding_box
    {
        // the reference reset() fill values, kept exactly in case an empty
        // box's corners ever reach an output path
        vec3<T> mins{ (T)999999999.999, (T)999999999.999, (T)999999999.999 };
        vec3<T> maxs{ (T)-999999999.999, (T)-999999999.999, (T)-999999999.999 };

        void add(const vec3<T> &p) {
            for (int i = 0; i < 3; i++)
            {
                if (p[i] < mins[i]) mins[i] = p[i];
                if (p[i] > maxs[i]) maxs[i] = p[i];
            }
        }

        void add(const basic_bounding_box &o) {
            add(o.mins);
            add(o.maxs);
        }

        bool is_empty() const {
            return mins.x > maxs.x;
        }

        bool contains(const vec3<T> &p) const {
            return p.x >= mins.x && p.x <= maxs.x
                && p.y >= mins.y && p.y <= maxs.y
                && p.z >= mins.z && p.z <= maxs.z;
        }

        // boxes closer than on_epsilon still count as touching, matching the
        // reference testdisjoint
        bool disjoint(const basic_bounding_box &o) const {
            return mins.x > o.maxs.x + (T)on_epsilon
                || mins.y > o.maxs.y + (T)on_epsilon
                || mins.z > o.maxs.z + (T)on_epsilon
                || maxs.x < o.mins.x - (T)on_epsilon
                || maxs.y < o.mins.y - (T)on_epsilon
                || maxs.z < o.mins.z - (T)on_epsilon;
        }
    };

    using bounding_box = basic_bounding_box<vec_t>;
}
