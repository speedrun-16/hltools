#include <cstdio>

#include "../common/error.h"
#include "../common/log.h"
#include "internal.h"

// the no matrix method: visibility between patches is tested on demand during
// the transfer build instead of being precomputed also home of the backwards
// test translucent patches use to gather light from behind

namespace rad
{
    namespace
    {
        // patchnum1 = receiver, patchnum2 = emitter
        bool check_vis_bit_no_vismatrix(rad_state &state, unsigned patchnum1, unsigned patchnum2,
                                        vec3v &transparency_out, unsigned int &)
        {
            if (patchnum1 > state.num_patches)
            {
                logging::warn("in CheckVisBit(), patchnum1 > num_patches");
            }
            if (patchnum2 > state.num_patches)
            {
                logging::warn("in CheckVisBit(), patchnum2 > num_patches");
            }

            patch *pt = &state.patches[patchnum1];
            patch *patch2 = &state.patches[patchnum2];

            transparency_out = vec3_one;

            // if emitter is behind that face plane, skip all patches

            if (patch2)
            {
                const plane *plane2 = plane_from_face_number(state, (unsigned)patch2->facenumber);

                if (math::dot(pt->origin, plane2->normal) > patch_plane_dist(state, patch2) + math::on_epsilon - pt->emitter_range)
                {
                    // we need to do a real test
                    const plane *pl = plane_from_face_number(state, (unsigned)pt->facenumber);

                    vec3v transparency = {1.0, 1.0, 1.0};
                    int opaquestyle = -1;

                    // check vis between patch and patch2
                    //  if v2 is not behind light plane
                    //  && v2 is visible from v1
                    vec3v origin1, origin2;
                    vec3v delta;
                    vec_t dist;
                    math::subtract(pt->origin, patch2->origin, delta);
                    dist = (vec_t)math::length(delta);
                    if (dist < patch2->emitter_range - math::on_epsilon)
                    {
                        get_alternate_origin(state, pt->origin, pl->normal, patch2, origin2);
                    }
                    else
                    {
                        math::copy(patch2->origin, origin2);
                    }
                    if (math::dot(origin2, pl->normal) <= patch_plane_dist(state, pt) + minimum_patch_distance)
                    {
                        return false;
                    }
                    if (dist < pt->emitter_range - math::on_epsilon)
                    {
                        get_alternate_origin(state, patch2->origin, plane2->normal, pt, origin1);
                    }
                    else
                    {
                        math::copy(pt->origin, origin1);
                    }
                    if (math::dot(origin1, plane2->normal) <= patch_plane_dist(state, patch2) + minimum_patch_distance)
                    {
                        return false;
                    }
                    if (test_line(state, origin1, origin2) != contents_empty)
                    {
                        return false;
                    }
                    if (test_segment_against_opaque_list(state, origin1, origin2, transparency, opaquestyle))
                    {
                        return false;
                    }

                    {
                        if (opaquestyle != -1)
                        {
                            add_style_to_style_array(state, patchnum1, patchnum2, opaquestyle);
                        }
                        if (state.options.customshadow_with_bouncelight)
                        {
                            math::copy(transparency, transparency_out);
                        }
                        return true;
                    }
                }
            }

            return false;
        }
    }

    bool check_vis_bit_backwards(rad_state &state, unsigned receiver, unsigned emitter,
                                 const vec3v &backorigin, const vec3v &backnormal, vec3v &transparency_out)
    {
        patch *emitpatch = &state.patches[emitter];

        transparency_out = vec3_one;

        if (emitpatch)
        {
            const plane *emitplane = plane_from_face_number(state, (unsigned)emitpatch->facenumber);

            if (math::dot(backorigin, emitplane->normal) > (patch_plane_dist(state, emitpatch) + minimum_patch_distance))
            {
                vec3v transparency = {1.0, 1.0, 1.0};
                int opaquestyle = -1;

                vec3v emitorigin;
                vec3v delta;
                vec_t dist;
                math::subtract(backorigin, emitpatch->origin, delta);
                dist = (vec_t)math::length(delta);
                if (dist < emitpatch->emitter_range - math::on_epsilon)
                {
                    get_alternate_origin(state, backorigin, backnormal, emitpatch, emitorigin);
                }
                else
                {
                    math::copy(emitpatch->origin, emitorigin);
                }
                if (math::dot(emitorigin, backnormal) <= math::dot(backorigin, backnormal) + minimum_patch_distance)
                {
                    return false;
                }
                if (test_line(state, backorigin, emitorigin) != contents_empty)
                {
                    return false;
                }
                if (test_segment_against_opaque_list(state, backorigin, emitorigin, transparency, opaquestyle))
                {
                    return false;
                }

                {
                    if (opaquestyle != -1)
                    {
                        add_style_to_style_array(state, receiver, emitter, opaquestyle);
                    }
                    if (state.options.customshadow_with_bouncelight)
                    {
                        math::copy(transparency, transparency_out);
                    }
                    return true;
                }
            }
        }

        return false;
    }

    void make_scales_no_vismatrix(rad_state &state)
    {
        err::require((int)state.num_patches < max_patches,
                     "make_scales_no_vismatrix: exceeded MAX_PATCHES");

        std::string transferfile = state.base_path + ".inc";

        if (!state.options.incremental || !read_transfers(state, transferfile.c_str(), (long)state.num_patches))
        {
            state.check_vis_bit = check_vis_bit_no_vismatrix;
            run_transfer_scales(state);

            if (state.options.incremental)
            {
                write_transfers(state, transferfile.c_str(), (long)state.num_patches);
            }
            else
            {
                std::remove(transferfile.c_str());
            }
            dump_transfers_memory_usage(state);
            create_final_style_arrays(state, "dynamic shadow array");
        }
    }
}
