#pragma once

#include <cmath>

#include "../common/types.h"
#include "../math/vector.h"

// stratified sampling shared by the passes that integrate over a set of
// directions or over an emitter's area (ambient occlusion, soft shadows)
//
// every one of them needs the same three things: a well spread point set at an
// arbitrary sample count, a way to decorrelate neighbouring luxels without
// giving up that spread, and a frame to orient the set in. the sequence offsets
// are hashed rather than drawn from a running rng so a luxel always draws the
// same set no matter how faces are distributed over threads

namespace rad
{
    namespace sampling
    {
        using math::vec3v;

        // r2 and r3, the low discrepancy sequences built from the roots of
        // x^3 = x + 1 and x^4 = x + 1. they stay well stratified at any sample
        // count, which a fixed grid only manages when the count factorizes
        constexpr double r2_alpha1 = 0.75487766624669276;
        constexpr double r2_alpha2 = 0.56984029099805327;
        constexpr double r3_alpha1 = 0.81917251339616450;
        constexpr double r3_alpha2 = 0.67104360670378930;
        constexpr double r3_alpha3 = 0.54970047790197020;

        // 32 bit integer hash (murmur style finalizer)
        inline unsigned hash_u32(unsigned x)
        {
            x ^= x >> 16;
            x *= 0x7feb352du;
            x ^= x >> 15;
            x *= 0x846ca68bu;
            x ^= x >> 16;
            return x;
        }

        inline double hash_unit(unsigned x)
        {
            return (double)hash_u32(x) * (1.0 / 4294967296.0);
        }

        // a seed for a world position, for the passes that have no luxel index
        // to hand. quantizing first keeps the seed stable against the last bits
        // of float error while still differing between neighbouring samples
        inline unsigned hash_position(const vec3v &p)
        {
            unsigned h = 0x9e3779b9u;
            for (int i = 0; i < 3; i++)
            {
                h ^= hash_u32((unsigned)(int)std::floor(p[i] * 16.0)) + 0x9e3779b9u + (h << 6) + (h >> 2);
            }
            return h;
        }

        inline double fract(double x)
        {
            return x - std::floor(x);
        }

        // branchless orthonormal basis (duff et al). neighbouring normals get
        // frames that vary smoothly, so the stratification stays coherent
        // across a surface instead of tumbling from sample to sample
        inline void build_frame(const vec3v &n, vec3v &tangent, vec3v &bitangent)
        {
            const vec_t sign = n[2] >= 0 ? (vec_t)1 : (vec_t)-1;
            const vec_t a = (vec_t)-1 / (sign + n[2]);
            const vec_t b = n[0] * n[1] * a;
            tangent = vec3v{1 + sign * n[0] * n[0] * a, sign * b, -sign * n[0]};
            bitangent = vec3v{b, sign + n[1] * n[1] * a, -n[1]};
        }

        // shirley's concentric map: the unit square onto the unit disc, keeping
        // adjacency, so a stratified square stays stratified on the disc. the
        // polar mapping would bunch samples at the centre and shear the strata
        inline void concentric_disc(double u, double v, double &x, double &y)
        {
            const double a = 2.0 * u - 1.0;
            const double b = 2.0 * v - 1.0;
            if (a == 0.0 && b == 0.0)
            {
                x = y = 0.0;
                return;
            }
            double r, theta;
            if (std::fabs(a) > std::fabs(b))
            {
                r = a;
                theta = (math::pi / 4.0) * (b / a);
            }
            else
            {
                r = b;
                theta = (math::pi / 2.0) - (math::pi / 4.0) * (a / b);
            }
            x = r * std::cos(theta);
            y = r * std::sin(theta);
        }
    }
}
