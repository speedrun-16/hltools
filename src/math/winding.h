#pragma once

#include <cmath>
#include <vector>

#include "bounding_box.h"
#include "vector.h"

// a convex polygon in 3d, the workhorse of the geometry stages templated on the
// stage precision the point values and the order of every arithmetic operation
// match the reference winding exactly, because clipping feeds the compiled
// geometry and must stay byte identical
//
// storage is a std::vector instead of the old manual new/resize, which changes
// nothing about the math: the same points in the same order produce the same
// bytes

namespace math
{
    template <typename T>
    class basic_winding
    {
    public:
        // point classification against a split plane, values kept from the
        // reference so the counts[] indexing matches
        enum side
        {
            side_front = 0,
            side_back = 1,
            side_on = 2,
            side_cross = -2,
        };

        basic_winding() = default;
        explicit basic_winding(std::vector<vec3<T>> pts) : points_(std::move(pts)) {}

        int size() const {
            return (int)points_.size();
        }
        bool empty() const {
            return points_.empty();
        }
        const vec3<T> &operator[](int i) const {
            return points_[i];
        }
        vec3<T> &operator[](int i) {
            return points_[i];
        }
        const std::vector<vec3<T>> &points() const {
            return points_;
        }
        void add_point(const vec3<T> &p) {
            points_.push_back(p);
        }

        // the largest quad lying on the plane, clipped down later by the brush
        // faces bogus_range is the half size of that quad and differs by old
        // call path, so the caller passes the exact reference value
        static basic_winding from_plane(const vec3<T> &normal, T dist, T bogus_range)
        {
            // dominant axis of the normal picks an up vector not parallel to it
            T max = -bogus_range;
            int x = -1;
            for (int i = 0; i < 3; i++)
            {
                T v = std::fabs(normal[i]);
                if (v > max)
                {
                    max = v;
                    x = i;
                }
            }

            vec3<T> vup{ 0, 0, 0 };
            if (x == 2)
                vup.x = 1;
            else
                vup.z = 1;

            T v = dot(vup, normal);
            vup = vup - normal * v; // project vup onto the plane
            normalize(vup);

            vec3<T> org = normal * dist;
            vec3<T> vright = cross(vup, normal);
            vup = vup * bogus_range;
            vright = vright * bogus_range;

            basic_winding w;
            w.points_.resize(4);
            w.points_[0] = org - vright + vup;
            w.points_[1] = org + vright + vup;
            w.points_[2] = org + vright - vup;
            w.points_[3] = org - vright - vup;
            return w;
        }

        // twice the sum of triangle fan areas, summed in double as the reference did
        T area() const
        {
            T total = 0;
            for (int i = 2; i < size(); i++)
            {
                vec3<T> d1 = points_[i - 1] - points_[0];
                vec3<T> d2 = points_[i] - points_[0];
                vec3<T> cr = cross(d1, d2);
                total = (T)(total + 0.5 * length(cr));
            }
            return total;
        }

        // the plane this winding lies on, from its first three points
        void plane(vec3<T> &normal, T &dist) const
        {
            if (size() >= 3)
            {
                vec3<T> v1 = points_[1] - points_[0];
                vec3<T> v2 = points_[2] - points_[0];
                normal = cross(v2, v1);
                normalize(normal);
                dist = dot(points_[0], normal);
            }
            else
            {
                normal = vec3<T>{};
                dist = 0;
            }
        }

        // average of the points, matching the reference getcenter exactly: the
        // scale factor is 10 / n narrowed to t first (the reference declares
        // it vec_t), then each component is multiplied in t rounding the scale
        // to float before the multiply is load bearing: it changes patch
        // origins in the last bit, which propagates into the whole lighting solve
        vec3<T> center() const
        {
            vec3<T> c{};
            if (!points_.empty())
            {
                for (const vec3<T> &p : points_)
                    c = p + c;
                T scale = (T)(1.0 / (unsigned)points_.size());
                c = c * scale;
            }
            return c;
        }

        void bounds(basic_bounding_box<T> &box) const
        {
            box = basic_bounding_box<T>{};
            for (const auto &p : points_)
                box.add(p);
        }

        // split by a plane into front and back windings, either of which may come
        // back empty classification always uses on_epsilon: the reference clip
        // takes an epsilon parameter but hardcodes on_epsilon in its side loop,
        // and removecolinearpoints ignores its parameter, so every legacy call
        // (including chop with normal_epsilon) effectively ran with 004
        void clip(const vec3<T> &normal, T dist, basic_winding &front, basic_winding &back) const
        {
            const T epsilon = (T)on_epsilon;
            int n = size();
            std::vector<T> dists(n + 1);
            std::vector<int> sides(n + 1);
            int counts[3] = { 0, 0, 0 };

            for (int i = 0; i < n; i++)
            {
                T d = dot(points_[i], normal);
                d -= dist;
                dists[i] = d;
                if (d > epsilon)
                    sides[i] = side_front;
                else if (d < -epsilon)
                    sides[i] = side_back;
                else
                    sides[i] = side_on;
                counts[sides[i]]++;
            }
            sides[n] = sides[0];
            dists[n] = dists[0];

            front = basic_winding{};
            back = basic_winding{};
            if (counts[side_front] == 0)
            {
                back = *this;
                return;
            }
            if (counts[side_back] == 0)
            {
                front = *this;
                return;
            }

            for (int i = 0; i < n; i++)
            {
                const vec3<T> &p1 = points_[i];
                if (sides[i] == side_on)
                {
                    front.points_.push_back(p1);
                    back.points_.push_back(p1);
                    continue;
                }
                if (sides[i] == side_front)
                    front.points_.push_back(p1);
                else if (sides[i] == side_back)
                    back.points_.push_back(p1);

                if (sides[i + 1] == side_on || sides[i + 1] == sides[i])
                    continue;

                // the edge crosses the plane: interpolate the split point, keeping
                // exact coordinates when the plane is axial
                int tmp = i + 1 >= n ? 0 : i + 1;
                const vec3<T> &p2 = points_[tmp];
                T frac = dists[i] / (dists[i] - dists[i + 1]);
                vec3<T> mid;
                for (int j = 0; j < 3; j++)
                {
                    if (normal[j] == 1)
                        mid[j] = dist;
                    else if (normal[j] == -1)
                        mid[j] = -dist;
                    else
                        mid[j] = p1[j] + frac * (p2[j] - p1[j]);
                }
                front.points_.push_back(mid);
                back.points_.push_back(mid);
            }

            front.remove_colinear_points();
            back.remove_colinear_points();
        }

        // keep only the front side of the plane; returns false if nothing remains
        bool chop(const vec3<T> &normal, T dist)
        {
            basic_winding front, back;
            clip(normal, dist, front, back);
            *this = std::move(front);
            return !points_.empty();
        }

        // which side of the plane the winding lies on like the reference
        // windingonplaneside the epsilon parameter was dead: classification
        // always uses on_epsilon
        int on_plane_side(const vec3<T> &normal, T dist, T epsilon = (T)on_epsilon) const
        {
            bool front = false;
            bool back = false;
            for (const vec3<T> &point : points_)
            {
                T d = dot(point, normal) - dist;
                if (d < -epsilon)
                {
                    if (front)
                        return side_cross;
                    back = true;
                    continue;
                }
                if (d > epsilon)
                {
                    if (back)
                        return side_cross;
                    front = true;
                    continue;
                }
            }
            if (back)
                return side_back;
            if (front)
                return side_front;
            return side_on;
        }

        // clips to the front side in place, the bsp only reference
        // clip(split, keepon) variant: keepon keeps a winding lying entirely on
        // the plane returns false when the winding is clipped away
        bool clip_in_place(const vec3<T> &normal, T dist, bool keepon)
        {
            int n = size();
            std::vector<T> dists((size_t)n + 1);
            std::vector<int> sides((size_t)n + 1);
            int counts[3] = { 0, 0, 0 };

            for (int i = 0; i < n; i++)
            {
                T d = dot(points_[(size_t)i], normal);
                d -= dist;
                dists[(size_t)i] = d;
                if (d > (T)on_epsilon)
                    sides[(size_t)i] = side_front;
                else if (d < -(T)on_epsilon)
                    sides[(size_t)i] = side_back;
                else
                    sides[(size_t)i] = side_on;
                counts[sides[(size_t)i]]++;
            }
            sides[(size_t)n] = sides[0];
            dists[(size_t)n] = dists[0];

            if (keepon && counts[side_front] == 0 && counts[side_back] == 0)
                return true;
            if (counts[side_front] == 0)
            {
                points_.clear();
                return false;
            }
            if (counts[side_back] == 0)
                return true;

            std::vector<vec3<T>> new_points;
            new_points.reserve((size_t)n + 4);
            for (int i = 0; i < n; i++)
            {
                const vec3<T> &p1 = points_[(size_t)i];
                if (sides[(size_t)i] == side_on)
                {
                    new_points.push_back(p1);
                    continue;
                }
                if (sides[(size_t)i] == side_front)
                    new_points.push_back(p1);

                if (sides[(size_t)i + 1] == side_on || sides[(size_t)i + 1] == sides[(size_t)i])
                    continue;

                int tmp = i + 1 >= n ? 0 : i + 1;
                const vec3<T> &p2 = points_[(size_t)tmp];
                T frac = dists[(size_t)i] / (dists[(size_t)i] - dists[(size_t)i + 1]);
                vec3<T> mid;
                for (int j = 0; j < 3; j++)
                {
                    if (normal[j] == 1)
                        mid[j] = dist;
                    else if (normal[j] == -1)
                        mid[j] = -dist;
                    else
                        mid[j] = p1[j] + frac * (p2[j] - p1[j]);
                }
                new_points.push_back(mid);
            }

            points_ = std::move(new_points);
            remove_colinear_points();
            return !points_.empty();
        }

        // where the winding went in a divide: wholly in front, wholly behind,
        // or split into the two out parameters
        enum class divide_side
        {
            front,
            back,
            split,
        };

        // divides by a plane without changing this winding, the bsp only
        // reference divide a winding lying entirely on the plane goes to the
        // side its summed point distances lean toward, and a split whose front
        // or back half degenerates collapses back to a whole side result
        divide_side divide(const vec3<T> &normal, T dist,
                           basic_winding &front, basic_winding &back) const
        {
            int n = size();
            std::vector<T> dists((size_t)n + 1);
            std::vector<int> sides((size_t)n + 1);
            int counts[3] = { 0, 0, 0 };

            for (int i = 0; i < n; i++)
            {
                T d = dot(points_[(size_t)i], normal);
                d -= dist;
                dists[(size_t)i] = d;
                if (d > (T)on_epsilon)
                    sides[(size_t)i] = side_front;
                else if (d < -(T)on_epsilon)
                    sides[(size_t)i] = side_back;
                else
                    sides[(size_t)i] = side_on;
                counts[sides[(size_t)i]]++;
            }
            sides[(size_t)n] = sides[0];
            dists[(size_t)n] = dists[0];

            if (counts[side_front] == 0 && counts[side_back] == 0)
            {
                T sum = 0.0;
                for (int i = 0; i < n; i++)
                {
                    T d = dot(points_[(size_t)i], normal);
                    d -= dist;
                    sum += d;
                }
                return sum > (T)normal_epsilon ? divide_side::front : divide_side::back;
            }
            if (counts[side_front] == 0)
                return divide_side::back;
            if (counts[side_back] == 0)
                return divide_side::front;

            front = basic_winding{};
            back = basic_winding{};
            for (int i = 0; i < n; i++)
            {
                const vec3<T> &p1 = points_[(size_t)i];
                if (sides[(size_t)i] == side_on)
                {
                    front.points_.push_back(p1);
                    back.points_.push_back(p1);
                    continue;
                }
                if (sides[(size_t)i] == side_front)
                    front.points_.push_back(p1);
                else if (sides[(size_t)i] == side_back)
                    back.points_.push_back(p1);

                if (sides[(size_t)i + 1] == side_on || sides[(size_t)i + 1] == sides[(size_t)i])
                    continue;

                int tmp = i + 1 >= n ? 0 : i + 1;
                const vec3<T> &p2 = points_[(size_t)tmp];
                T frac = dists[(size_t)i] / (dists[(size_t)i] - dists[(size_t)i + 1]);
                vec3<T> mid;
                for (int j = 0; j < 3; j++)
                {
                    if (normal[j] == 1)
                        mid[j] = dist;
                    else if (normal[j] == -1)
                        mid[j] = -dist;
                    else
                        mid[j] = p1[j] + frac * (p2[j] - p1[j]);
                }
                front.points_.push_back(mid);
                back.points_.push_back(mid);
            }

            front.remove_colinear_points();
            back.remove_colinear_points();
            if (front.points_.empty())
                return divide_side::back;
            if (back.points_.empty())
                return divide_side::front;
            return divide_side::split;
        }

        // drop points that lie on the line between their neighbours always uses
        // on_epsilon, like the reference (whose epsilon parameter is unused)
        void remove_colinear_points()
        {
            const T epsilon = (T)on_epsilon;
            for (int i = 0; i < (int)points_.size(); i++)
            {
                int n = (int)points_.size();
                const vec3<T> &p1 = points_[(i + n - 1) % n];
                const vec3<T> &p2 = points_[i];
                const vec3<T> &p3 = points_[(i + 1) % n];
                vec3<T> v1 = p2 - p1;
                vec3<T> v2 = p3 - p2;
                T d12 = dot(v1, v2);
                T d11 = dot(v1, v1);
                T d22 = dot(v2, v2);
                if (d12 * d12 >= d11 * d22 - epsilon * epsilon * (d11 + d22 + epsilon * epsilon))
                {
                    points_.erase(points_.begin() + i);
                    i = -1; // restart the scan, matching the reference
                }
            }
        }

    private:
        std::vector<vec3<T>> points_;
    };

    using winding = basic_winding<vec_t>;
}
