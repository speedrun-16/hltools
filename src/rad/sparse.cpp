#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "../common/error.h"
#include "../common/limits.h"
#include "../common/log.h"
#include "../common/progress.h"
#include "../common/threads.h"
#include "internal.h"

#ifdef HLTOOLS_GPU_ENABLED
#include "gpu/gpu.h"
#endif

// the sparse visibility matrix method (the default): per patch columns of
// runs of 8 bit values, so empty regions of the matrix cost nothing

namespace rad
{
    namespace
    {
        // binary search the column for the row containing bit y
        unsigned is_visbit_in_array(const rad_state &state, const unsigned x, const unsigned y)
        {
            int first, last, current;
            int y_byte = (int)(y / 8);
            const sparse_row *row;
            const sparse_column *column = state.sparse_columns.data() + x;

            if (!column->count)
            {
                return (unsigned)-1;
            }

            first = 0;
            last = column->count - 1;

            while (1)
            {
                current = (first + last) / 2;
                row = column->row + current;
                if ((int)(row->offset) < y_byte)
                {
                    first = current + 1;
                }
                else if ((int)(row->offset) > y_byte)
                {
                    last = current - 1;
                }
                else
                {
                    return (unsigned)current;
                }
                if (first > last)
                {
                    return (unsigned)-1;
                }
            }
        }

        void add_visible_patch(std::vector<byte> &bits, unsigned patchnum)
        {
            bits[patchnum / 8] |= (byte)(1u << (patchnum & 7));
        }

        void set_vis_column(rad_state &state, unsigned patchnum, const std::vector<byte> &bits)
        {
            sparse_column *column = &state.sparse_columns[patchnum];
            if (column->count || column->row)
            {
                err::fatal("set_vis_column: column has been set");
            }

            // The temporary column is already bit-packed. Only emit its
            // nonempty bytes, starting at the source patch.
            thread_local std::vector<sparse_row> rows;
            rows.clear();
            for (unsigned offset = patchnum / 8; offset < bits.size(); offset++)
            {
                if (const byte mask = bits[offset])
                    rows.push_back({offset, mask});
            }
            if (rows.empty())
                return;

            const size_t bytes = rows.size() * sizeof(sparse_row);
            column->row = (sparse_row *)std::malloc(bytes);
            err::require(column->row != nullptr, "set_vis_column: out of memory");
            std::memcpy(column->row, rows.data(), bytes);
            column->count = (int)rows.size();
        }

        bool check_vis_bit_sparse(rad_state &state, unsigned x, unsigned y,
                                  vec3v &transparency_out, unsigned int &next_index)
        {
            unsigned offset;

            transparency_out = vec3_one;

            if (x == y)
            {
                return true;
            }

            const unsigned a = x;
            const unsigned b = y;

            if (x > y)
            {
                x = b;
                y = a;
            }

            if (x > state.num_patches)
            {
                logging::warn("in CheckVisBit(), x > num_patches");
            }
            if (y > state.num_patches)
            {
                logging::warn("in CheckVisBit(), y > num_patches");
            }

            if ((offset = is_visbit_in_array(state, x, y)) != (unsigned)-1)
            {
                if (state.options.customshadow_with_bouncelight)
                {
                    get_transparency(state, a, b, transparency_out, next_index);
                }
                return (state.sparse_columns[x].row[offset].values & (1 << (y & 7))) != 0;
            }

            return false;
        }

        struct vis_face_info
        {
            vec3v normal;
            vec_t plane_dist = 0;
            vec3v bounds_min;
            vec3v bounds_max;
            unsigned max_patch = 0;
        };

        // sets vis bits for all patches in the face
        void test_patch_to_face(rad_state &state, const unsigned patchnum, const int facenum,
                                const byte *pvs, std::vector<byte> &bits,
                                const vis_face_info &src_face, const vis_face_info &dst_face,
                                bool has_opaque)
        {
            patch *pt = &state.patches[patchnum];
            patch *patch2 = state.face_patches[(size_t)facenum];
            if (!patch2 || dst_face.max_patch <= patchnum)
                return;

            const vec3v &n2 = dst_face.normal;
            const vec_t d2 = dst_face.plane_dist;
            const vec_t pt_dot_n2 = math::dot(pt->origin, n2);

            // if emitter is behind that face plane, skip all patches
            if (!(pt_dot_n2 > d2 + math::on_epsilon - pt->emitter_range))
                return;

            // if the entire target face (including alternate-origin margin)
            // lies behind the source patch's plane, no patch on it can pass
            const vec3v &n1 = src_face.normal;
            const vec_t pt_thresh = src_face.plane_dist + minimum_patch_distance;
            const vec_t max_dst_dot =
                (n1[0] >= 0 ? dst_face.bounds_max[0] : dst_face.bounds_min[0]) * n1[0]
                + (n1[1] >= 0 ? dst_face.bounds_max[1] : dst_face.bounds_min[1]) * n1[1]
                + (n1[2] >= 0 ? dst_face.bounds_max[2] : dst_face.bounds_min[2]) * n1[2];
            if (max_dst_dot <= pt_thresh - (vec_t)0.01)
                return;

            const vec_t f2_thresh = d2 + minimum_patch_distance;
            const bool pt_in_front_of_f2 = pt_dot_n2 > f2_thresh;
            const float r1 = pt->emitter_range - (float)math::on_epsilon;
            const patch *const patches_base = state.patches.data();

            for (; patch2; patch2 = patch2->next)
            {
                const unsigned m = (unsigned)(patch2 - patches_base);
                if (m <= patchnum)
                    continue;
                if (patch2->leafnum == 0
                    || !(pvs[(patch2->leafnum - 1) >> 3] & (1 << ((patch2->leafnum - 1) & 7))))
                {
                    continue;
                }

                vec3v delta;
                math::subtract(pt->origin, patch2->origin, delta);
                const float dist_sq = math::dot(delta, delta);
                const float r2 = patch2->emitter_range - (float)math::on_epsilon;
                const float rmax = r2 > r1 ? r2 : r1;

                vec3v origin1, origin2;
                const vec3v *p1;
                const vec3v *p2;
                if (rmax <= 0.0f || dist_sq >= (rmax + 0.01f) * (rmax + 0.01f))
                {
                    // Neither patch is within its emitter range: alternate
                    // origins cannot trigger, so avoid sqrt and redundant dot.
                    if (!pt_in_front_of_f2)
                        continue;
                    if (math::dot(patch2->origin, n1) <= pt_thresh)
                        continue;
                    p1 = &pt->origin;
                    p2 = &patch2->origin;
                }
                else
                {
                    const vec_t dist = (vec_t)std::sqrt((double)dist_sq);
                    if (dist < patch2->emitter_range - math::on_epsilon)
                    {
                        get_alternate_origin(state, pt->origin, n1, patch2, origin2);
                    }
                    else
                    {
                        math::copy(patch2->origin, origin2);
                    }
                    if (math::dot(origin2, n1) <= pt_thresh)
                    {
                        continue;
                    }
                    if (dist < pt->emitter_range - math::on_epsilon)
                    {
                        get_alternate_origin(state, patch2->origin, n2, pt, origin1);
                    }
                    else
                    {
                        math::copy(pt->origin, origin1);
                    }
                    if (math::dot(origin1, n2) <= f2_thresh)
                    {
                        continue;
                    }
                    p1 = &origin1;
                    p2 = &origin2;
                }

                if (test_line(state, *p1, *p2) != contents_empty)
                {
                    continue;
                }

                if (has_opaque)
                {
                    vec3v transparency = {1.0, 1.0, 1.0};
                    int opaquestyle = -1;
                    if (test_segment_against_opaque_list(state, *p1, *p2, transparency, opaquestyle))
                    {
                        continue;
                    }

                    if (opaquestyle != -1)
                    {
                        add_style_to_style_array(state, m, patchnum, opaquestyle);
                        add_style_to_style_array(state, patchnum, m, opaquestyle);
                    }

                    if (state.options.customshadow_with_bouncelight && !math::equal(transparency, vec3_one))
                    {
                        add_transparency_to_raw_array(state, patchnum, m, transparency);
                    }
                }
                add_visible_patch(bits, m);
            }
        }

        thread_local std::vector<byte> tls_vis_bits;

        // a slice of one source leaf's patches; big leaves are cut into
        // several slices so one dense leaf cannot leave the phase running on
        // a single thread (furame: 56 leaves, the last one ran alone ~7s)
        struct vis_leaf_slice
        {
            int leaf;
            unsigned begin;
            unsigned end;
        };

        // per thread cache of the source leaf's pvs and candidate faces;
        // slices of one leaf are handed out back to back, so a thread usually
        // reuses it. keyed by a per build generation because the calling
        // thread's thread_locals outlive one threads::run
        struct vis_leaf_cache
        {
            unsigned generation = 0;
            int leaf = -1;
            bool active = false;
            std::vector<byte> pvs;
            std::vector<byte> visible_faces;
            std::vector<int> candidates;
        };
        thread_local vis_leaf_cache tls_leaf_cache;
        std::atomic<unsigned> g_vis_generation{0};

        // returns false when the leaf contributes no columns
        bool prepare_vis_leaf(rad_state &state, int i, unsigned generation,
                              const std::vector<std::vector<int>> &faces_by_leaf,
                              const std::vector<int> &unindexed_faces)
        {
            vis_leaf_cache &c = tls_leaf_cache;
            if (c.generation == generation && c.leaf == i)
                return c.active;
            c.generation = generation;
            c.leaf = i;
            c.active = false;
            c.candidates.clear();
            c.pvs.assign((limits::max_map_leafs + 7) / 8, 0);
            byte *pvs = c.pvs.data();
            const format::dleaf_t &srcleaf = state.map->leafs[(size_t)i];
            if (state.map->visibility.empty())
            {
                std::memset(pvs, 255, (size_t)((state.map->models[0].visleafs + 7) / 8));
            }
            else
            {
                if (srcleaf.visofs == -1)
                {
                    return false;
                }
                decompress_vis(state, &state.map->visibility[(size_t)srcleaf.visofs], pvs, (unsigned)c.pvs.size());
            }

            // Every patch in this source leaf shares its PVS. Cull whole
            // faces once, then keep the original face and patch order.
            c.visible_faces.assign(state.map->faces.size(), 0);
            for (int facenum : unindexed_faces)
                c.visible_faces[(size_t)facenum] = 1;
            for (size_t leaf = 1; leaf < faces_by_leaf.size(); leaf++)
            {
                if (!(pvs[(leaf - 1) >> 3] & (1u << ((leaf - 1) & 7))))
                    continue;
                for (int facenum : faces_by_leaf[leaf])
                    c.visible_faces[(size_t)facenum] = 1;
            }
            for (size_t face = 0; face < c.visible_faces.size(); face++)
            {
                if (c.visible_faces[face])
                    c.candidates.push_back((int)face);
            }
            c.active = true;
            return true;
        }

        // one work item per leaf slice; runs on multiple threads
        void build_vis_slice_item(rad_state &state, const vis_leaf_slice &slice, unsigned generation,
                                  const std::vector<std::vector<patch *>> &patches_by_leaf,
                                  const std::vector<std::vector<int>> &faces_by_leaf,
                                  const std::vector<int> &unindexed_faces,
                                  const std::vector<vis_face_info> &face_info,
                                  bool has_opaque)
        {
            if (!prepare_vis_leaf(state, slice.leaf, generation, faces_by_leaf, unindexed_faces))
                return;
            const vis_leaf_cache &c = tls_leaf_cache;
            const std::vector<patch *> &sources = patches_by_leaf[(size_t)slice.leaf];
            for (unsigned s = slice.begin; s < slice.end; s++)
            {
                const patch *pt = sources[s];
                const unsigned patchnum = (unsigned)(pt - state.patches.data());
                tls_vis_bits.assign((state.num_patches + 7) / 8, 0);
                const vis_face_info &src_info = face_info[(size_t)pt->facenumber];
                auto target = std::upper_bound(c.candidates.begin(), c.candidates.end(), pt->facenumber);
                for (; target != c.candidates.end(); ++target)
                    test_patch_to_face(state, patchnum, *target, c.pvs.data(), tls_vis_bits,
                                       src_info, face_info[(size_t)*target], has_opaque);
                set_vis_column(state, patchnum, tls_vis_bits);
            }
        }

#ifdef HLTOOLS_GPU_ENABLED
        // gpu variant of the vis matrix build: the pre trace logic mirrors
        // test_patch_to_face exactly (pvs cull, plane rejects, alternate
        // origins on the cpu), and only the test_line runs on the gpu trace
        // kernel which matches the cpu tracer so
        // the columns come out identical to the cpu build only used when the
        // map has no opaque entities or studio models (their segment tests and
        // the style/transparency side arrays are cpu only paths)

        thread_local std::vector<byte> tls_gpu_pvs;
        thread_local int tls_gpu_pvs_visofs = -3; // -3 = nothing cached

        // decompressed pvs row for a patch's leaf, cached per thread
        const byte *gpu_patch_pvs(rad_state &state, const patch *pt)
        {
            const size_t pvs_bytes = (size_t)((limits::max_map_leafs + 7) / 8);
            tls_gpu_pvs.resize(pvs_bytes);
            if (state.map->visibility.empty())
            {
                if (tls_gpu_pvs_visofs != -2)
                {
                    std::memset(tls_gpu_pvs.data(), 255,
                                (size_t)((state.map->models[0].visleafs + 7) / 8));
                    tls_gpu_pvs_visofs = -2;
                }
                return tls_gpu_pvs.data();
            }
            const int visofs = state.map->leafs[(size_t)pt->leafnum].visofs;
            if (visofs == -1)
            {
                return nullptr; // the cpu build skips these leaves entirely
            }
            if (tls_gpu_pvs_visofs != visofs)
            {
                decompress_vis(state, &state.map->visibility[(size_t)visofs],
                               tls_gpu_pvs.data(), (unsigned)pvs_bytes);
                tls_gpu_pvs_visofs = visofs;
            }
            return tls_gpu_pvs.data();
        }

        // mirror of test_patch_to_face up to (but not including) the
        // test_line: emits one segment + target patch per surviving pair
        void gpu_enumerate_patch(rad_state &state, const unsigned patchnum,
                                 std::vector<gpu::trace_segment> &segments,
                                 std::vector<unsigned> &targets)
        {
            patch *pt = &state.patches[patchnum];
            if (pt->leafnum < 1 || pt->leafnum > (int)state.map->models[0].visleafs)
            {
                return; // the cpu build only walks leaves 1visleafs
            }
            const byte *pvs = gpu_patch_pvs(state, pt);
            if (!pvs)
            {
                return;
            }
            const plane *pl = plane_from_face_number(state, (unsigned)pt->facenumber);

            for (int facenum2 = pt->facenumber + 1; facenum2 < (int)state.map->faces.size(); facenum2++)
            {
                patch *patch2 = state.face_patches[(size_t)facenum2];
                if (!patch2)
                {
                    continue;
                }
                const plane *plane2 = plane_from_face_number(state, (unsigned)facenum2);
                if (!(math::dot(pt->origin, plane2->normal)
                      > patch_plane_dist(state, patch2) + math::on_epsilon - pt->emitter_range))
                {
                    continue;
                }
                for (; patch2; patch2 = patch2->next)
                {
                    unsigned m = (unsigned)(patch2 - state.patches.data());
                    if (m <= patchnum)
                    {
                        continue;
                    }
                    if (patch2->leafnum == 0
                        || !(pvs[(patch2->leafnum - 1) >> 3] & (1 << ((patch2->leafnum - 1) & 7))))
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
                    gpu::trace_segment seg;
                    seg.start[0] = origin1[0];
                    seg.start[1] = origin1[1];
                    seg.start[2] = origin1[2];
                    seg.stop[0] = origin2[0];
                    seg.stop[1] = origin2[1];
                    seg.stop[2] = origin2[2];
                    segments.push_back(seg);
                    targets.push_back(m);
                }
            }
        }

        bool gpu_build_vis_matrix(rad_state &state)
        {
            if (!state.opaque_list.empty() || state.num_studio_models != 0)
            {
                return false; // opaque/studio segment tests are cpu only
            }
            if (!gpu::available())
            {
                logging::warn("-gpu: %s; building patch visibility on the cpu",
                              gpu::last_error().c_str());
                return false;
            }

            std::vector<gpu::tnode_gpu> tnodes(state.tnodes.size());
            for (size_t i = 0; i < state.tnodes.size(); i++)
            {
                const tnode &tn = state.tnodes[i];
                tnodes[i].normal[0] = tn.normal[0];
                tnodes[i].normal[1] = tn.normal[1];
                tnodes[i].normal[2] = tn.normal[2];
                tnodes[i].dist = tn.dist;
                tnodes[i].type = tn.type;
                tnodes[i].children[0] = tn.children[0];
                tnodes[i].children[1] = tn.children[1];
            }

            progress::section("lighting");
            progress::begin("building patch visibility", (int)state.num_patches);

            constexpr unsigned chunk_patches = 2048;
            std::vector<std::vector<gpu::trace_segment>> segs(chunk_patches);
            std::vector<std::vector<unsigned>> targs(chunk_patches);

            for (unsigned base = 0; base < state.num_patches; base += chunk_patches)
            {
                const unsigned count = state.num_patches - base < chunk_patches
                    ? state.num_patches - base : chunk_patches;

                // pass a (cpu, threaded): enumerate candidate pairs
                threads::run((int)count, [&](int k)
                             {
                                 segs[(size_t)k].clear();
                                 targs[(size_t)k].clear();
                                 gpu_enumerate_patch(state, base + (unsigned)k,
                                                     segs[(size_t)k], targs[(size_t)k]);
                             });

                // pass b (gpu): trace every candidate segment
                std::vector<gpu::trace_segment> flat;
                std::vector<size_t> first((size_t)count + 1, 0);
                for (unsigned k = 0; k < count; k++)
                {
                    first[k] = flat.size();
                    flat.insert(flat.end(), segs[k].begin(), segs[k].end());
                }
                first[count] = flat.size();
                std::vector<gpu::trace_result> results;
                if (!flat.empty()
                    && !gpu::trace_batch(tnodes, flat, results))
                {
                    progress::end();
                    logging::warn("-gpu: %s; building patch visibility on the cpu",
                                  gpu::last_error().c_str());
                    return false;
                }

                // pass c (cpu, threaded): fill and compress the columns
                threads::run((int)count, [&](int k)
                             {
                                 const unsigned patchnum = base + (unsigned)k;
                                 patch *pt = &state.patches[patchnum];
                                 // untouched leaves keep their default empty
                                 // column, like the cpu build
                                 bool active = pt->leafnum >= 1
                                     && pt->leafnum <= (int)state.map->models[0].visleafs
                                     && (state.map->visibility.empty()
                                         || state.map->leafs[(size_t)pt->leafnum].visofs != -1);
                                 if (active)
                                 {
                                     tls_vis_bits.assign((state.num_patches + 7) / 8, 0);
                                     for (size_t s = first[(size_t)k]; s < first[(size_t)k + 1]; s++)
                                     {
                                         if (results[s].contents == contents_empty)
                                         {
                                             add_visible_patch(tls_vis_bits, targs[(size_t)k][s - first[(size_t)k]]);
                                         }
                                     }
                                     set_vis_column(state, patchnum, tls_vis_bits);
                                 }
                                 progress::add(1);
                             });
            }

            progress::end();
            return true;
        }
#endif

        void build_vis_matrix(rad_state &state, bool show_progress)
        {
            state.sparse_columns.assign(state.num_patches, sparse_column{});

#ifdef HLTOOLS_GPU_ENABLED
            // measured on celerior: bit identical columns but 22x slower
            // than the cpu build (58s -> 130s) - the phase is bound by the
            // pvs cull and alternate origin hunts, not the traces, and the
            // segment upload/readback outweighs the ray savings kept behind
            // an env switch as the base for a future overlap pipelined try
            if (show_progress && state.options.gpu && std::getenv("HLTOOLS_GPU_VISMATRIX")
                && gpu_build_vis_matrix(state))
            {
                return;
            }
#endif
            const int visleafs = state.map->models[0].visleafs;
            std::vector<std::vector<patch *>> patches_by_leaf((size_t)visleafs + 1);
            std::vector<std::vector<int>> faces_by_leaf((size_t)visleafs + 1);
            std::vector<int> unindexed_faces;
            std::vector<vis_face_info> face_info(state.map->faces.size());
            const patch *const patches_base = state.patches.data();
            for (size_t face = 0; face < state.map->faces.size(); face++)
            {
                patch *first = state.face_patches[face];
                if (!first)
                    continue;
                vis_face_info &fi = face_info[face];
                const plane *pl = plane_from_face_number(state, (unsigned)face);
                fi.normal = pl->normal;
                fi.plane_dist = pl->dist + math::dot(state.face_offset[face], pl->normal);
                fi.bounds_min = vec3v{bogus_range, bogus_range, bogus_range};
                fi.bounds_max = vec3v{-bogus_range, -bogus_range, -bogus_range};
                auto grow = [&](const vec3v &v)
                {
                    for (int c = 0; c < 3; c++)
                    {
                        if (v[c] < fi.bounds_min[c])
                            fi.bounds_min[c] = v[c];
                        if (v[c] > fi.bounds_max[c])
                            fi.bounds_max[c] = v[c];
                    }
                };
                for (patch *pt = first; pt; pt = pt->next)
                {
                    const unsigned m = (unsigned)(pt - patches_base);
                    if (m > fi.max_patch)
                        fi.max_patch = m;
                    grow(pt->origin);
                    if (pt->winding)
                    {
                        for (int v = 0; v < pt->winding->size(); v++)
                            grow((*pt->winding)[v]);
                    }
                    if (pt->leafnum > 0 && pt->leafnum <= visleafs)
                    {
                        patches_by_leaf[(size_t)pt->leafnum].push_back(pt);
                        auto &fl = faces_by_leaf[(size_t)pt->leafnum];
                        if (fl.empty() || fl.back() != (int)face)
                            fl.push_back((int)face);
                    }
                    else if (pt->leafnum > visleafs
                             && (unindexed_faces.empty() || unindexed_faces.back() != (int)face))
                        unindexed_faces.push_back((int)face);
                }
                // Margin for get_alternate_origin's hunt_offset (0.5) + hunt_for_world (2.0).
                for (int c = 0; c < 3; c++)
                {
                    fi.bounds_min[c] -= 4.0f;
                    fi.bounds_max[c] += 4.0f;
                }
            }
            const bool has_opaque = !state.opaque_list.empty() || state.num_studio_models != 0;
            // slice size: small leaves stay whole (one candidate build each),
            // dense leaves split so the tail is a fraction of a percent
            const unsigned slice_patches = std::max<unsigned>(
                32u, state.num_patches / (unsigned)(std::max(1, threads::count()) * 32));
            std::vector<vis_leaf_slice> slices;
            for (int leaf = 1; leaf <= visleafs; leaf++)
            {
                const unsigned n = (unsigned)patches_by_leaf[(size_t)leaf].size();
                for (unsigned b = 0; b < n; b += slice_patches)
                    slices.push_back({leaf, b, std::min(n, b + slice_patches)});
            }
            const unsigned generation = ++g_vis_generation;
            auto item = [&](int i) {
                build_vis_slice_item(state, slices[(size_t)i], generation, patches_by_leaf,
                                     faces_by_leaf, unindexed_faces, face_info, has_opaque);
            };
            if (show_progress)
            {
                threads::run_phase("lighting", "building patch visibility", (int)slices.size(), item);
            }
            else
            {
                // built in the background during the gpu gather: yield to the
                // near pair resolution, which the gather's end waits on
                threads::run((int)slices.size(), [&](int i) {
                    thread_local unsigned lowered = 0;
                    if (lowered != generation)
                    {
                        threads::lower_current_priority();
                        lowered = generation;
                    }
                    item(i);
                });
            }
        }

        void free_vis_matrix(rad_state &state)
        {
            for (size_t x = 0; x < state.sparse_columns.size(); x++)
            {
                if (state.sparse_columns[x].row)
                {
                    std::free(state.sparse_columns[x].row);
                }
            }
            state.sparse_columns.clear();
            state.sparse_columns.shrink_to_fit();
        }

        void dump_vismatrix_info(const rad_state &state)
        {
            size_t total_vismatrix_memory = sizeof(sparse_column) * state.num_patches;

            for (size_t x = 0; x < state.sparse_columns.size(); x++)
            {
                total_vismatrix_memory += (size_t)state.sparse_columns[x].count * sizeof(sparse_row);
            }

            logging::file("      %-18s %.1f MB\n", "vismatrix", total_vismatrix_memory / (1024 * 1024.0));
        }
    }

    bool can_prebuild_sparse_vis_matrix(const rad_state &state)
    {
        // -incremental may skip the build by reading the transfers back
        return state.options.numbounce > 0 && !state.options.incremental
            && state.options.method == vis_method::sparse_vismatrix
            && (int)state.num_patches < max_sparse_vismatrix_patches;
    }

    void prebuild_sparse_vis_matrix(rad_state &state)
    {
        build_vis_matrix(state, false);
        state.sparse_columns_prebuilt = true;
    }

    void make_scales_sparse_vismatrix(rad_state &state)
    {
        err::require((int)state.num_patches < max_sparse_vismatrix_patches,
                     "make_scales_sparse_vismatrix: exceeded MAX_SPARSE_VISMATRIX_PATCHES");

        std::string transferfile = state.base_path + ".inc";

        if (!state.options.incremental || !read_transfers(state, transferfile.c_str(), (long)state.num_patches))
        {
            // determine visibility between patches
            if (!state.sparse_columns_prebuilt)
                build_vis_matrix(state, true);
            state.sparse_columns_prebuilt = false;
            dump_vismatrix_info(state);
            state.check_vis_bit = check_vis_bit_sparse;

            create_final_transparency_arrays(state, "custom shadow array");

            run_transfer_scales(state);
            free_vis_matrix(state);
            free_transparency_arrays(state);

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
