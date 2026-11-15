#include <cmath>

#include "internal.h"
#include "sampling.h"

// soft shadows for point and spot lights
//
// a light entity is a mathematical point, so its shadow terminator is a step:
// the sample either sees the origin or it does not. real sources have area, and
// the width of the penumbra they cast grows with the distance from the occluder
// to the receiver, which is most of what reads as "not a lightmap from 1998"
//
// the sun already works this way. light_environment spreads its direction over
// a cone of sun normals (_spread), each traced separately, and the fraction
// that survives is the penumbra. this does the same thing for the light types
// that had no equivalent, sampling the emitter's area instead of a cone of
// directions
//
// a sphere of radius r seen from any point has a circular silhouette of radius
// r facing the viewer, so sampling that disc is exact for a spherical emitter
// rather than an approximation of one

namespace rad
{
    namespace
    {
        // the hard shadow of a point source: one ray to the origin
        vec_t point_visibility(const rad_state &state, const vec3v &pos,
                               const vec3v &light_origin, bool shadowtests_active,
                               vec3v &transparency_out, int &opaquestyle_out)
        {
            if (test_line(state, pos, light_origin) != contents_empty)
            {
                return 0;
            }
            if (shadowtests_active
                && test_segment_against_opaque_list(state, pos, light_origin, transparency_out,
                                                    opaquestyle_out))
            {
                return 0;
            }
            return 1;
        }
    }

    vec_t soft_shadow_visibility(const rad_state &state, const vec3v &pos,
                                 const vec3v &light_origin, vec_t radius, int samples,
                                 bool shadowtests_active, vec3v &transparency_out,
                                 int &opaquestyle_out)
    {
        math::copy(vec3_one, transparency_out);
        opaquestyle_out = -1;

        vec3v dir;
        math::subtract(light_origin, pos, dir);
        const vec_t dist = math::normalize(dir);

        // standing inside the emitter, or asked for a point source: one ray is
        // the whole answer and must stay bit for bit what the hard path did
        if (radius <= 0 || samples <= 1 || dist <= radius)
        {
            return point_visibility(state, pos, light_origin, shadowtests_active,
                                    transparency_out, opaquestyle_out);
        }

        vec3v tangent, bitangent;
        sampling::build_frame(dir, tangent, bitangent);

        // one toroidal shift of the sequence per receiving point. a shifted
        // stratified set is still stratified, so the penumbra keeps its even
        // gradient, but neighbouring luxels no longer share a set - without
        // this the terminator picks up the sample pattern as visible contours
        const unsigned seed = sampling::hash_position(pos);
        const double du = sampling::hash_unit(seed + 0u);
        const double dv = sampling::hash_unit(seed + 1u);

        // every sample is traced. a unanimous first quarter cannot be taken as
        // the answer: an occluder that hides a few percent of the disc slips
        // between the probe rays, and the luxels of every narrow penumbra then
        // come out fully lit
        int visible = 0;
        int traced = 0;
        vec3v transparency_sum{};

        for (int i = 0; i < samples; i++)
        {
            const double u = sampling::fract(0.5 + sampling::r2_alpha1 * (i + 1) + du);
            const double v = sampling::fract(0.5 + sampling::r2_alpha2 * (i + 1) + dv);

            double dx, dy;
            sampling::concentric_disc(u, v, dx, dy);

            vec3v target;
            math::copy(light_origin, target);
            math::multiply_add(target, (vec_t)(radius * dx), tangent, target);
            math::multiply_add(target, (vec_t)(radius * dy), bitangent, target);

            // the disc ignores the world, so near a wall or a ceiling part of
            // it lies inside the brush or beyond it. those points are not part
            // of the emitter at all: counted as blocked rays they would darken
            // a light that hangs close to a surface even with nothing between
            // it and the receiver, and counted as visible they would let it
            // shine through the wall
            if (test_line(state, light_origin, target) != contents_empty)
            {
                continue;
            }

            traced++;

            if (test_line(state, pos, target) != contents_empty)
            {
                // blocked by the world
            }
            else
            {
                vec3v transparency;
                int opaquestyle = -1;
                if (shadowtests_active
                    && test_segment_against_opaque_list(state, pos, target, transparency,
                                                        opaquestyle))
                {
                    // blocked by an opaque entity
                }
                else
                {
                    if (!shadowtests_active)
                    {
                        math::copy(vec3_one, transparency);
                    }
                    math::add(transparency_sum, transparency, transparency_sum);
                    if (opaquestyle != -1 && opaquestyle_out == -1)
                    {
                        opaquestyle_out = opaquestyle;
                    }
                    visible++;
                }
            }
        }

        // the whole disc is out of reach, which is a light buried in a brush:
        // the origin is all that is left of it
        if (traced == 0)
        {
            return point_visibility(state, pos, light_origin, shadowtests_active,
                                    transparency_out, opaquestyle_out);
        }
        if (visible <= 0)
        {
            return 0;
        }
        // divided rather than scaled by a reciprocal, so a sum of exact ones
        // stays exactly one and an unoccluded receiver matches a point light
        for (int c = 0; c < 3; c++)
        {
            transparency_out[c] = transparency_sum[c] / (vec_t)visible;
        }
        return (vec_t)visible / (vec_t)traced;
    }
}
