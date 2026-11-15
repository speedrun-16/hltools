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

        vec3v tangent, bitangent;
        sampling::build_frame(dir, tangent, bitangent);

        // one toroidal shift of the sequence per receiving point. a shifted
        // stratified set is still stratified, so the penumbra keeps its even
        // gradient, but neighbouring luxels no longer share a set - without
        // this the terminator picks up the sample pattern as visible contours
        const unsigned seed = sampling::hash_position(pos);
        const double du = sampling::hash_unit(seed + 0u);
        const double dv = sampling::hash_unit(seed + 1u);

        // umbra and full light are both decided unanimously, and between them
        // they are almost the whole map: only the penumbra band needs the full
        // budget. the probe is the first quarter of the sequence, which is
        // already spread over the whole disc rather than clustered
        const int probe = samples >= 8 ? samples / 4 : samples;

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

            if (traced == probe && probe < samples)
            {
                if (visible == 0)
                {
                    return 0; // fully in shadow of the whole disc
                }
                if (visible == traced)
                {
                    // fully lit: the transparency measured so far is the
                    // answer, since every remaining ray would agree
                    math::scale(transparency_sum, (vec_t)1 / (vec_t)visible, transparency_out);
                    return 1;
                }
            }
        }

        if (visible <= 0)
        {
            return 0;
        }
        math::scale(transparency_sum, (vec_t)1 / (vec_t)visible, transparency_out);
        return (vec_t)visible / (vec_t)traced;
    }
}
