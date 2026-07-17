#pragma once

#include <cmath>

#include "../common/types.h"

// templated vector with three components shared by every stage the geometry stages instantiate
// vec3<double>, the lighting stages vec3<float>, so one source serves both
// precisions operations preserve the exact evaluation order of the original
// macros because float addition is not associative and the compiled output must
// stay byte identical to the reference build

namespace math
{
    constexpr double pi = 3.14159265358979323846;

    // geometry tolerances, carried over unchanged from the reference build
    constexpr double normal_epsilon = 0.00001;
    constexpr double on_epsilon = 0.04;
    constexpr double equal_epsilon = 0.004;

    template <typename T>
    struct vec3
    {
        T x, y, z;

        constexpr vec3() : x(0), y(0), z(0) {}
        constexpr vec3(T vx, T vy, T vz) : x(vx), y(vy), z(vz) {}

        // index access keeps interop with the on disk float[3] wire arrays
        T &operator[](int i) {
            return (&x)[i];
        }
        const T &operator[](int i) const {
            return (&x)[i];
        }

        vec3 operator+(const vec3 &o) const {
            return {x + o.x, y + o.y, z + o.z};
        }
        vec3 operator-(const vec3 &o) const {
            return {x - o.x, y - o.y, z - o.z};
        }
        vec3 operator*(T s) const {
            return {x * s, y * s, z * s};
        }
        vec3 operator-() const {
            return {-x, -y, -z};
        }
        vec3 &operator+=(const vec3 &o) {
            x += o.x; y += o.y; z += o.z;
            return *this;
        }
        vec3 &operator-=(const vec3 &o) {
            x -= o.x; y -= o.y; z -= o.z;
            return *this;
        }
        bool operator==(const vec3 &o) const {
            return x == o.x && y == o.y && z == o.z;
        }
    };

    template <typename T>
    inline T dot(const vec3<T> &a, const vec3<T> &b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    template <typename T>
    inline T dot(const vec3<T> &a, const float b[3]) {
        return a.x * b[0] + a.y * b[1] + a.z * b[2];
    }

    template <typename T>
    inline T dot(const float a[3], const vec3<T> &b) {
        return a[0] * b.x + a[1] * b.y + a[2] * b.z;
    }

    template <typename T>
    inline void copy(const vec3<T> &in, vec3<T> &out) {
        out.x = in.x; out.y = in.y; out.z = in.z;
    }

    template <typename T>
    inline void copy_to_float(const vec3<T> &in, float out[3]) {
        out[0] = (float)in.x; out[1] = (float)in.y; out[2] = (float)in.z;
    }

    template <typename T>
    inline void clear(vec3<T> &out) {
        out.x = (T)0; out.y = (T)0; out.z = (T)0;
    }

    template <typename T>
    inline void subtract(const vec3<T> &a, const vec3<T> &b, vec3<T> &out) {
        out.x = a.x - b.x; out.y = a.y - b.y; out.z = a.z - b.z;
    }

    template <typename T>
    inline void add(const vec3<T> &a, const vec3<T> &b, vec3<T> &out) {
        out.x = a.x + b.x; out.y = a.y + b.y; out.z = a.z + b.z;
    }

    template <typename T, typename S>
    inline void scale(const vec3<T> &in, S s, vec3<T> &out) {
        out.x = (T)(in.x * s); out.y = (T)(in.y * s); out.z = (T)(in.z * s);
    }

    template <typename T>
    inline void multiply(const vec3<T> &a, const vec3<T> &b, vec3<T> &out) {
        out.x = a.x * b.x; out.y = a.y * b.y; out.z = a.z * b.z;
    }

    template <typename T, typename S>
    inline void multiply_add(const vec3<T> &a, S scale, const vec3<T> &b, vec3<T> &out) {
        out.x = (T)(a.x + scale * b.x); out.y = (T)(a.y + scale * b.y); out.z = (T)(a.z + scale * b.z);
    }

    template <typename T>
    inline vec3<T> cross(const vec3<T> &a, const vec3<T> &b) {
        return {a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x};
    }

    template <typename T>
    inline void cross(const vec3<T> &a, const vec3<T> &b, vec3<T> &out) {
        out.x = a.y * b.z - a.z * b.y;
        out.y = a.z * b.x - a.x * b.z;
        out.z = a.x * b.y - a.y * b.x;
    }

    // sqrt is always taken in double, matching vectorlength in the reference
    // build each square is computed in t first and only then widened, exactly
    // like the reference macro: in the float stages the products round to
    // float before the double sum, and that rounding is part of the output
    template <typename T>
    inline double length(const vec3<T> &v) {
        return std::sqrt((double)((double)(v.x * v.x) + (double)(v.y * v.y) + (double)(v.z * v.z)));
    }

    // normalizes in place and returns the previous length mirrors
    // vectornormalize exactly: length summed in t, sqrt in double, each
    // component divided by the double length, near zero collapses to the origin
    template <typename T>
    inline T normalize(vec3<T> &v) {
        double len = dot(v, v);
        len = std::sqrt(len);
        if (len < normal_epsilon)
        {
            v = vec3<T>{};
            return 0;
        }
        // divide in double then narrow back to t, exactly as the original
        // component wise "/= length" did (length was a double there too)
        v.x = (T)(v.x / len);
        v.y = (T)(v.y / len);
        v.z = (T)(v.z / len);
        return (T)len;
    }

    // true when the two vectors agree within the geometry tolerance
    template <typename T>
    inline bool equal(const vec3<T> &a, const vec3<T> &b) {
        return std::fabs(a.x - b.x) <= equal_epsilon
            && std::fabs(a.y - b.y) <= equal_epsilon
            && std::fabs(a.z - b.z) <= equal_epsilon;
    }

    using vec3f = vec3<float>;
    using vec3d = vec3<double>;

    // the stage's chosen precision, from common/typesh
    using vec3v = vec3<vec_t>;
}
