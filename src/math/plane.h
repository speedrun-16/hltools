#pragma once

#include <cmath>

#include "vector.h"

// geometric plane and the axis classification the tree builders use to pick
// splitting planes carried over from the reference build: an axial normal
// classifies as plane_x/y/z, anything else as the any_* variant of its dominant
// axis, tested against the same direction epsilon

namespace math
{
    enum class plane_type
    {
        x,      // axial, facing dominantly along +/-x
        y,
        z,
        any_x,  // non axial, x is the largest normal component
        any_y,
        any_z,
    };

    constexpr double dir_epsilon = 0.0001;

    template <typename T>
    struct basic_plane
    {
        vec3<T> normal;
        T dist = 0;

        T distance_to(const vec3<T> &p) const {
            return dot(normal, p) - dist;
        }
    };

    template <typename T>
    inline plane_type type_for_normal(const vec3<T> &normal)
    {
        T ax = std::fabs(normal.x);
        T ay = std::fabs(normal.y);
        T az = std::fabs(normal.z);

        if (ax > 1 - dir_epsilon && ay < dir_epsilon && az < dir_epsilon)
            return plane_type::x;
        if (ay > 1 - dir_epsilon && az < dir_epsilon && ax < dir_epsilon)
            return plane_type::y;
        if (az > 1 - dir_epsilon && ax < dir_epsilon && ay < dir_epsilon)
            return plane_type::z;

        if (ax >= ay && ax >= az)
            return plane_type::any_x;
        if (ay >= ax && ay >= az)
            return plane_type::any_y;
        return plane_type::any_z;
    }

    using plane = basic_plane<vec_t>;
}
