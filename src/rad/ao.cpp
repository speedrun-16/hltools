#include <cmath>

#include "internal.h"

// ambient occlusion: the fraction of the cosine weighted hemisphere above a
// sample that is blocked by nearby geometry
//
// rad already shadows every direct light exactly, so ao here is not a shadow
// term. it stands in for the visibility the light terms that have no rays of
// their own are missing: the flat "-ambient" and "_minlight" constants, and
// the bounced light, which radiosity resolves only down to patch resolution
// (-chop) and so cannot darken a corner narrower than a patch. where each of
// those is applied, so is the occlusion measured here
//
// the estimator is a cosine distributed hemisphere: the cosine factor lives in
// the sample density instead of a per ray weight, so every ray carries weight
// 1/n and the result is the plain mean of the visibility samples. the geodesic
// sky normal tables are deliberately not reused for this - they are uniform
// over the whole sphere, so half of every set falls below the horizon and most
// of the rest lands where the cosine weight is near zero

namespace rad
{
    namespace
    {
        // r3, the 3 dimensional low discrepancy sequence built from the root of
        // x^4 = x + 1. it is well stratified at any ray count, which a fixed
        // grid is not unless the count factorizes, and the first k points of it
        // are spread well enough to serve as the early out probe below
        constexpr double r3_alpha1 = 0.81917251339616450;
        constexpr double r3_alpha2 = 0.67104360670378930;
        constexpr double r3_alpha3 = 0.54970047790197020;

        // deterministic 32 bit integer hash (murmur style finalizer). the per
        // sample sequence offsets come from this rather than a running rng so
        // the same luxel always draws the same rays no matter how faces are
        // distributed over threads
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

        inline double fract(double x)
        {
            return x - std::floor(x);
        }

        // branchless orthonormal basis (duff et al). neighbouring normals get
        // frames that vary smoothly, so the stratification stays coherent
        // across a face instead of tumbling from luxel to luxel
        void build_frame(const vec3v &n, vec3v &tangent, vec3v &bitangent)
        {
            const vec_t sign = n[2] >= 0 ? (vec_t)1 : (vec_t)-1;
            const vec_t a = (vec_t)-1 / (sign + n[2]);
            const vec_t b = n[0] * n[1] * a;
            tangent = vec3v{1 + sign * n[0] * n[0] * a, sign * b, -sign * n[0]};
            bitangent = vec3v{b, sign + n[1] * n[1] * a, -n[1]};
        }

        // the fraction of the maximum distance this ray is allowed to travel
        //
        // grading occlusion by hit distance normally needs a tracer that
        // reports one, which test_line does not. it is not needed: for a
        // decreasing grading curve g with g(0) = 1 and g(1) = 0,
        //   g(x) = integral over t in [0,1] of [x < g_inverse(t)]
        // so drawing the ray length from g_inverse of a uniform variate makes
        // the mean of the plain binary hit tests equal the distance graded
        // integral exactly. the length is just another stratified dimension
        vec_t ray_length_fraction(ao_falloff falloff, double t)
        {
            switch (falloff)
            {
            case ao_falloff::binary:
                return 1;
            case ao_falloff::linear:
                // g(x) = 1 - x
                return (vec_t)(1.0 - t);
            case ao_falloff::quadratic:
                // g(x) = (1 - x)^2, so g_inverse(t) = 1 - sqrt(t)
                return (vec_t)(1.0 - std::sqrt(t));
            case ao_falloff::smooth:
            default:
                // g(x) = 1 - smoothstep(x), inverted through the trigonometric
                // form of the depressed cubic
                {
                    double c = 2.0 * t - 1.0;
                    c = c < -1.0 ? -1.0 : (c > 1.0 ? 1.0 : c);
                    return (vec_t)(0.5 - std::sin(std::asin(c) / 3.0));
                }
            }
        }

        inline vec_t luminance(const vec3v &v)
        {
            return (vec_t)(0.299 * v[0] + 0.587 * v[1] + 0.114 * v[2]);
        }
    }

    // how many rays one sample actually fires
    //
    // -extra puts a 3x3 cache grid behind every luxel, and each of those nine
    // samples draws its own decorrelated ray set, so the luxel already averages
    // nine independent estimates. spending the full ray count on each of them
    // buys a quality nobody asked for; the budget is divided instead so -extra
    // costs about what a plain compile does and still comes out cleaner
    int ao_rays_for_density(const rad_options &options, int lmcache_density)
    {
        int rays = options.ao_rays;
        if (lmcache_density > 1 && !options.ao_rays_explicit)
        {
            rays = (rays + lmcache_density - 1) / lmcache_density;
        }
        return rays < 4 ? 4 : rays;
    }



    // measures the occlusion above one sample
    //
    // pos must already be off the surface (calc_lightmap hands in the hunt
    // offset position) and normal is the phong normal, so curved smoothing
    // groups sample the hemisphere they appear to face rather than the one the
    // flat brush face faces. seed identifies the luxel and only shifts the
    // sequence, so the result is reproducible across thread counts
    vec_t ao_sample(const rad_state &state, const vec3v &pos, const vec3v &normal,
                    unsigned seed, int rays)
    {
        const rad_options &options = state.options;

        vec3v tangent, bitangent;
        build_frame(normal, tangent, bitangent);

        // the ray origin is lifted along the phong normal as well. the sample
        // already sits default_hunt_offset off its own plane, but that is the
        // face plane; on a smoothed or a barely non coplanar join a grazing ray
        // still starts close enough to re-hit the surface it left, which reads
        // as dark speckle along every brush seam
        vec3v origin;
        math::multiply_add(pos, options.ao_bias, normal, origin);

        // cranley patterson: one toroidal shift of the whole sequence per
        // sample. a shifted stratified set is still stratified, so this costs
        // no uniformity, and it decorrelates neighbouring luxels - without it
        // every luxel draws the same directions and the error is spatially
        // coherent, which shows up as terracing rather than as noise the
        // existing blur can remove
        const double du = hash_unit(seed * 0x9e3779b9u + 0x0u);
        const double dv = hash_unit(seed * 0x9e3779b9u + 0x1u);
        const double dw = hash_unit(seed * 0x9e3779b9u + 0x2u);

        // unanimous probe: open sky and sealed cavities are both extremely
        // common and both are decided by the first few well spread rays. only
        // a unanimous verdict short circuits, so a sample with any occlusion
        // structure at all still gets the full budget
        const int probe = (rays >= 32) ? rays / 4 : rays;

        vec_t occluded = 0;
        int traced = 0;

        for (int i = 0; i < rays; i++)
        {
            const double u = fract(0.5 + r3_alpha1 * (i + 1) + du);
            const double v = fract(0.5 + r3_alpha2 * (i + 1) + dv);
            const double w = fract(0.5 + r3_alpha3 * (i + 1) + dw);

            // malley: a cosine density on the hemisphere is a uniform density
            // on the disc lifted straight up
            const double r = std::sqrt(u);
            const double phi = 2.0 * math::pi * v;
            const double lx = r * std::cos(phi);
            const double ly = r * std::sin(phi);
            const double lz = std::sqrt(u >= 1.0 ? 0.0 : 1.0 - u);

            vec3v dir;
            for (int c = 0; c < 3; c++)
            {
                dir[c] = (vec_t)(tangent[c] * lx + bitangent[c] * ly + normal[c] * lz);
            }

            vec3v dest;
            const vec_t reach = options.ao_dist * ray_length_fraction(options.ao_falloff_mode, w);
            math::multiply_add(origin, reach, dir, dest);

            traced++;

            if (test_line(state, origin, dest) == contents_solid)
            {
                occluded += 1;
            }
            else
            {
                // opaque entities block outright, but a transparent one only
                // dims what passes through it. scaleout carries that product
                // whenever the walk did not hit something solid, so a grate or
                // a fence casts occlusion in proportion to how much it hides
                vec3v scale;
                int opaquestyle;
                if (test_segment_against_opaque_list(state, origin, dest, scale, opaquestyle))
                {
                    occluded += 1;
                }
                else
                {
                    const vec_t passthrough = luminance(scale);
                    if (passthrough < 1)
                    {
                        occluded += 1 - (passthrough < 0 ? 0 : passthrough);
                    }
                }
            }

            if (traced == probe && probe < rays)
            {
                if (occluded <= 0)
                {
                            return 0; // nothing above this sample at all
                }
                if (occluded >= (vec_t)probe)
                {
                            return 1; // sealed in every probed direction
                }
            }
        }

        if (traced <= 0)
        {
            return 0;
        }
        return occluded / (vec_t)traced;
    }

    // turns an occlusion fraction into the per channel factor a light term is
    // multiplied by
    //
    // plain "multiply by 1 - occlusion" is what makes baked ao read as grime:
    // it assumes a blocked direction delivers nothing, when in a real cavity
    // that direction delivers light that bounced off the cavity itself. the
    // multibounce fit (jimenez et al) restores that as a function of the
    // surface's own albedo, so a white corner barely darkens and a dark one
    // still goes black - and because the albedo is per channel, the occlusion
    // it produces is coloured by the texture instead of being flat grey
    void ao_visibility(const rad_state &state, vec_t occlusion, const vec3v &albedo,
                       vec_t strength, vec3v &out)
    {
        vec_t o = occlusion * state.options.ao_strength * strength;
        o = o < 0 ? 0 : (o > 1 ? 1 : o);

        const vec_t v = 1 - o;

        if (!state.options.ao_multibounce)
        {
            out = vec3v{v, v, v};
            return;
        }

        for (int c = 0; c < 3; c++)
        {
            vec_t a = albedo[c];
            a = a < 0 ? 0 : (a > 1 ? 1 : a);

            const vec_t ca = (vec_t)(2.0404 * a - 0.3324);
            const vec_t cb = (vec_t)(-4.7951 * a + 0.6417);
            const vec_t cc = (vec_t)(2.7552 * a + 0.6903);

            const vec_t mb = ((v * ca + cb) * v + cc) * v;
            out[c] = mb > v ? mb : v;
        }
    }

    int ao_rays_effective(const rad_options &options)
    {
        return ao_rays_for_density(options, options.extra ? 3 : 1);
    }

    float ao_bounce_effective(const rad_options &options)
    {
        return (float)ao_bounce_strength(options);
    }

    // the share of the bounced light ao is allowed to darken
    //
    // radiosity already carries a real visibility term, but only between
    // patches, so it resolves occlusion no finer than -chop. below that scale
    // the darkening ao measures is information the bounce genuinely lacks;
    // above it the two would be describing the same occlusion twice. ramping
    // by the ratio applies ao where it is the only source of the answer and
    // backs off where the transfer lists already have one
    vec_t ao_bounce_strength(const rad_options &options)
    {
        if (options.ao_bounce >= 0)
        {
            return options.ao_bounce;
        }
        const vec_t chop = options.chop > 1 ? (vec_t)options.chop : (vec_t)1;
        const vec_t ratio = options.ao_dist / chop;
        return ratio > 1 ? (vec_t)1 : ratio;
    }
}
