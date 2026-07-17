#include <cstdio>
#include <cstring>

#include "../common/error.h"
#include "../common/limits.h"
#include "../common/log.h"
#include "../common/threads.h"
#include "internal.h"

// the dense visibility matrix method: one triangular bit matrix over all patch
// pairs, built from the pvs by testing lines between patch origins each pair
// occupies one bit in the triangular matrix

namespace rad
{
    namespace
    {
        // sets vis bits for all patches in the face
        void test_patch_to_face(rad_state &state, const unsigned patchnum, const int facenum,
                                const unsigned int bitpos, byte *pvs)
        {
            patch *pt = &state.patches[patchnum];
            patch *patch2 = state.face_patches[(size_t)facenum];

            // if emitter is behind that face plane, skip all patches

            if (patch2)
            {
                const plane *plane2 = plane_from_face_number(state, (unsigned)facenum);

                if (math::dot(pt->origin, plane2->normal) > patch_plane_dist(state, patch2) + math::on_epsilon - pt->emitter_range)
                {
                    // we need to do a real test
                    const plane *pl = plane_from_face_number(state, (unsigned)pt->facenumber);

                    for (; patch2; patch2 = patch2->next)
                    {
                        unsigned m = (unsigned)(patch2 - state.patches.data());

                        vec3v transparency = {1.0, 1.0, 1.0};
                        int opaquestyle = -1;

                        // check vis between patch and patch2
                        // if bit has not already been set
                        //  && v2 is not behind light plane
                        //  && v2 is visible from v1
                        if (m > patchnum)
                        {
                            if (patch2->leafnum == 0 || !(pvs[(patch2->leafnum - 1) >> 3] & (1 << ((patch2->leafnum - 1) & 7))))
                            {
                                continue;
                            }
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
                                continue;
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
                                continue;
                            }
                            if (test_line(state, origin1, origin2) != contents_empty)
                            {
                                continue;
                            }
                            if (test_segment_against_opaque_list(state, origin1, origin2, transparency, opaquestyle))
                            {
                                continue;
                            }

                            if (opaquestyle != -1)
                            {
                                add_style_to_style_array(state, m, patchnum, opaquestyle);
                                add_style_to_style_array(state, patchnum, m, opaquestyle);
                            }

                            // patchnum can see patch m
                            unsigned bitset = bitpos + m;

                            if (state.options.customshadow_with_bouncelight && !math::equal(transparency, vec3_one))
                            {
                                add_transparency_to_raw_array(state, patchnum, m, transparency);
                            }

                            {
                                std::lock_guard<std::mutex> guard(state.lock);
                                state.dense_vismatrix[bitset >> 3] |= 1 << (bitset & 7);
                            }
                        }
                    }
                }
            }
        }

        // one work item per vis leaf; runs on multiple threads
        void build_vis_leafs_item(rad_state &state, int i)
        {
            byte pvs[(limits::max_map_leafs + 7) / 8];
            format::dleaf_t *srcleaf;
            patch *pt;
            unsigned bitpos;
            unsigned patchnum;

            i++; // skip leaf 0
            srcleaf = &state.map->leafs[(size_t)i];
            if (state.map->visibility.empty())
            {
                std::memset(pvs, 255, (size_t)((state.map->models[0].visleafs + 7) / 8));
            }
            else
            {
                if (srcleaf->visofs == -1)
                {
                    return;
                }
                decompress_vis(state, &state.map->visibility[(size_t)srcleaf->visofs], pvs, sizeof(pvs));
            }

            // go through all the faces inside the leaf, and process the
            // patches that actually have origins inside
            for (int facenum = 0; facenum < (int)state.map->faces.size(); facenum++)
            {
                for (pt = state.face_patches[(size_t)facenum]; pt; pt = pt->next)
                {
                    if (pt->leafnum != i)
                        continue;
                    patchnum = (unsigned)(pt - state.patches.data());
                    // triangular (halfbit) matrix layout
                    bitpos = patchnum * state.num_patches - (patchnum * (patchnum + 1)) / 2;
                    for (int facenum2 = facenum + 1; facenum2 < (int)state.map->faces.size(); facenum2++)
                        test_patch_to_face(state, patchnum, facenum2, bitpos, pvs);
                }
            }
        }

        void build_vis_matrix(rad_state &state)
        {
            // triangular halfbit matrix size
            int c = (int)((((size_t)state.num_patches + 1) * ((size_t)state.num_patches + 1)) / 16);
            c += 1;

            logging::file("      %-18s %.1f MB\n", "vismatrix", c / (1024 * 1024.0));

            state.dense_vismatrix.assign((size_t)c, 0);

            threads::run_phase("lighting", "building patch visibility",
                               state.map->models[0].visleafs,
                               [&](int i) { build_vis_leafs_item(state, i); });
        }

        void free_vis_matrix(rad_state &state)
        {
            state.dense_vismatrix.clear();
            state.dense_vismatrix.shrink_to_fit();
        }

        bool check_vis_bit_vismatrix(rad_state &state, unsigned p1, unsigned p2,
                                     vec3v &transparency_out, unsigned int &next_index)
        {
            unsigned bitpos;

            const unsigned a = p1;
            const unsigned b = p2;

            transparency_out = vec3_one;

            if (p1 > p2)
            {
                p1 = b;
                p2 = a;
            }

            if (p1 > state.num_patches)
            {
                logging::warn("in CheckVisBit(), p1 > num_patches");
            }
            if (p2 > state.num_patches)
            {
                logging::warn("in CheckVisBit(), p2 > num_patches");
            }

            bitpos = p1 * state.num_patches - (p1 * (p1 + 1)) / 2 + p2;

            if (state.dense_vismatrix[bitpos >> 3] & (1 << (bitpos & 7)))
            {
                if (state.options.customshadow_with_bouncelight)
                {
                    get_transparency(state, a, b, transparency_out, next_index);
                }
                return true;
            }

            return false;
        }
    }

    void make_scales_vismatrix(rad_state &state)
    {
        err::require((int)state.num_patches < max_vismatrix_patches,
                     "make_scales_vismatrix: exceeded MAX_VISMATRIX_PATCHES");

        std::string transferfile = state.base_path + ".inc";

        if (!state.options.incremental || !read_transfers(state, transferfile.c_str(), (long)state.num_patches))
        {
            // determine visibility between patches
            build_vis_matrix(state);
            state.check_vis_bit = check_vis_bit_vismatrix;

            create_final_transparency_arrays(state, "custom shadow array");

            run_transfer_scales(state);
            free_vis_matrix(state);
            free_transparency_arrays(state);

            if (state.options.incremental)
                write_transfers(state, transferfile.c_str(), (long)state.num_patches);
            else
                std::remove(transferfile.c_str());
            dump_transfers_memory_usage(state);
            create_final_style_arrays(state, "dynamic shadow array");
        }
    }
}
