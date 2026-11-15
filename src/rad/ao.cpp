#include <cmath>

#include "internal.h"
#include "sampling.h"

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
    // buys a quality nobody asked for, so the budget is divided by the grid's
    // side. that is not by its area: the nine samples together still fire
    // about three times the rays of a plain compile (9 x 22 against 64 at the
    // default), which is what keeps -extra visibly cleaner
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
        sampling::build_frame(normal, tangent, bitangent);

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
        const double du = sampling::hash_unit(seed * 0x9e3779b9u + 0x0u);
        const double dv = sampling::hash_unit(seed * 0x9e3779b9u + 0x1u);
        const double dw = sampling::hash_unit(seed * 0x9e3779b9u + 0x2u);

        // every ray is traced. a unanimous first quarter cannot be taken as
        // the answer: a crease or a small occluder that covers a few percent
        // of the hemisphere slips between the probe rays, and the sample then
        // reads as completely open
        vec_t occluded = 0;
        int traced = 0;

        for (int i = 0; i < rays; i++)
        {
            const double u = sampling::fract(0.5 + sampling::r3_alpha1 * (i + 1) + du);
            const double v = sampling::fract(0.5 + sampling::r3_alpha2 * (i + 1) + dv);
            const double w = sampling::fract(0.5 + sampling::r3_alpha3 * (i + 1) + dw);

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
