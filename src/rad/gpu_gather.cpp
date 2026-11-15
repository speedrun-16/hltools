#include "internal.h"

#include "../common/error.h"
#include "../common/log.h"
#include "../common/progress.h"
#include "../common/threads.h"

// the -gpu direct lighting path the build_facelights phase runs twice: a
// collect pass where
// gather_sample_light records each call as a work item without building
// the lightmap or modifying patches, then the gpu runs
// the gather kernel over all items (surface near branch pairs resolved on
// the cpu with the reference functions), then the real pass consumes the
// stored per style sums through the same coring/style slot tail the cpu
// gather ends with everything outside the gather itself - sample
// placement, the lmcache blur, bounces, final blending - is the untouched
// cpu code

#ifdef HLTOOLS_GPU_ENABLED

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <thread>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "gpu/gpu.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

namespace rad
{
    struct gpu_gather_data
    {
        // collect
        std::vector<std::vector<gpu::work_item_gpu>> face_items;
        std::vector<std::vector<byte>> pvs_rows;
        std::unordered_map<std::string, int> row_lookup;
        std::mutex row_mutex;
        size_t rowbytes = 0;

        // consume
        std::vector<gpu::work_item_gpu> items;   // flat, face major
        std::vector<int> face_first;
        std::vector<int> face_cursor;
        gpu::gather_results results;
        int style_to_compact[allstyles];
        std::vector<int> compact_to_style;
    };

    namespace
    {
        // the upper clamp, and the size the resident chunk buffers are cut to;
        // the dispatch loop steers the live chunk somewhere below this
        constexpr size_t gather_chunk_items = 65536;

        // a work item's cost scales with the number of emitters the kernel
        // walks for it, so a fixed item count says nothing about how long a
        // dispatch runs: a map carrying 16k texlight emitters spent ~18s in
        // one, against the ~2s windows gives a submit before it resets the
        // device (tdr) and we lose the whole gather to the cpu fallback.
        // measure each dispatch and steer the next one at a target instead
        // the target is deliberately far under the ~2s reset window: a chunk's
        // cost swings with how many emitters its faces actually see, so the
        // spread around the average is wide. Leave enough margin for a
        // heavier dispatch so the gather can stay on the GPU.
        constexpr double gather_target_seconds = 0.20;
        // Reduce the ceiling immediately after a long dispatch.
        constexpr double gather_ceiling_seconds = 0.40;
        constexpr unsigned gather_recovery_dispatches = 4;
        constexpr size_t gather_chunk_min = 256;
        constexpr size_t gather_chunk_start = 2048;

        // mirrors texlightgap_setup in gather_sample_light
        void face_gap_vectors(const rad_state &state, int facenum, float *out6)
        {
            const format::dface_t *f = &state.map->faces[(size_t)facenum];
            const plane *dp = plane_from_face_number(state, (unsigned)facenum);
            const format::texinfo_t *tex = &state.map->texinfo[(size_t)f->texinfo];
            for (int x = 0; x < 2; x++)
            {
                vec3v tv{tex->vecs[1 - x][0], tex->vecs[1 - x][1], tex->vecs[1 - x][2]};
                vec3v world;
                math::cross(tv, dp->normal, world);
                vec3v sv{tex->vecs[x][0], tex->vecs[x][1], tex->vecs[x][2]};
                vec_t len = math::dot(world, sv);
                if (std::fabs(len) < math::normal_epsilon)
                    math::clear(world);
                else
                    math::scale(world, 1 / len, world);
                out6[x * 3 + 0] = world[0];
                out6[x * 3 + 1] = world[1];
                out6[x * 3 + 2] = world[2];
            }
        }

        bool marshal_scene(rad_state &state, gpu_gather_data &d, gpu::gather_scene &scene)
        {
            const format::map_data &map = *state.map;

            scene.tnodes.resize(state.tnodes.size());
            for (size_t i = 0; i < state.tnodes.size(); i++)
            {
                const tnode &tn = state.tnodes[i];
                gpu::tnode_gpu &out = scene.tnodes[i];
                out.normal[0] = tn.normal[0];
                out.normal[1] = tn.normal[1];
                out.normal[2] = tn.normal[2];
                out.dist = tn.dist;
                out.type = tn.type;
                out.children[0] = tn.children[0];
                out.children[1] = tn.children[1];
            }
            scene.lightleafs.resize(state.lightleafs.size());
            for (size_t i = 0; i < state.lightleafs.size(); i++)
            {
                scene.lightleafs[i].leafnum = state.lightleafs[i].leafnum;
                scene.lightleafs[i].firstlight = state.lightleafs[i].firstlight;
                scene.lightleafs[i].numlights = state.lightleafs[i].numlights;
            }

            for (int s = 0; s < allstyles; s++)
                d.style_to_compact[s] = -1;
            scene.lights.resize(state.lightarray.size());
            for (size_t i = 0; i < state.lightarray.size(); i++)
            {
                const directlight &l = state.lightarray[i];
                gpu::light_gpu &out = scene.lights[i];
                if (l.style < 0 || l.style >= allstyles)
                    return false;
                if (d.style_to_compact[l.style] < 0)
                {
                    if ((int)d.compact_to_style.size() >= gpu::max_compact_styles)
                        return false; // more distinct styles than the kernel handles
                    d.style_to_compact[l.style] = (int)d.compact_to_style.size();
                    d.compact_to_style.push_back(l.style);
                }
                out.type = (int)l.type;
                out.compact_style = d.style_to_compact[l.style];
                out.topatch = l.topatch ? 1 : 0;
                out.sun_ofs = (int)(scene.sun_normals.size() / 4);
                out.sun_count = 0;
                if (l.type == emit_type::skylight && l.sunnormals && l.sunnormalweights)
                {
                    out.sun_count = l.numsunnormals;
                    for (int j = 0; j < l.numsunnormals; j++)
                    {
                        scene.sun_normals.push_back(l.sunnormals[j][0]);
                        scene.sun_normals.push_back(l.sunnormals[j][1]);
                        scene.sun_normals.push_back(l.sunnormals[j][2]);
                        scene.sun_normals.push_back(l.sunnormalweights[j]);
                    }
                }
                for (int x = 0; x < 3; x++)
                {
                    out.origin[x] = l.origin[x];
                    out.intensity[x] = l.intensity[x];
                    out.normal[x] = l.normal[x];
                    out.diffuse[x] = l.diffuse_intensity[x];
                    out.diffuse2[x] = l.diffuse_intensity2[x];
                }
                out.stopdot = l.stopdot;
                out.stopdot2 = l.stopdot2;
                out.fade = l.fade;
                out.texlightgap = l.texlightgap;
                out.patch_area = l.patch_area;
                out.patch_emitter_range = l.patch_emitter_range;
                // the same test gather_sample_light makes before it calls
                // soft_shadow_visibility; every other light keeps radius 0 and
                // with it the single hard shadow ray
                const bool soft = state.options.soft_samples > 1 && l.radius > 0
                    && (l.type == emit_type::point || l.type == emit_type::spotlight);
                out.radius = soft ? (float)l.radius : 0.0f;
                if (soft)
                    scene.soft_samples = (uint32_t)state.options.soft_samples;
            }

            const int skylevel = state.options.softsky ? skylevel_softsky_on : skylevel_softsky_off;
            for (int j = 0; j < state.numskynormals[skylevel]; j++)
            {
                scene.sky_normals.push_back(state.skynormals[skylevel][(size_t)j][0]);
                scene.sky_normals.push_back(state.skynormals[skylevel][(size_t)j][1]);
                scene.sky_normals.push_back(state.skynormals[skylevel][(size_t)j][2]);
                scene.sky_normals.push_back(state.skynormalsizes[skylevel][(size_t)j]);
            }
            scene.style_count = (uint32_t)d.compact_to_style.size();
            scene.sky_lighting_fix = state.options.sky_lighting_fix ? 1 : 0;
            scene.sky_step_match = (state.options.softsky || state.options.fastmode) ? 1 : 0;
            scene.indirect_sun = state.options.indirect_sun;

            scene.face_gap.assign(map.faces.size() * 6, 0.0f);
            for (size_t f = 0; f < map.faces.size(); f++)
                face_gap_vectors(state, (int)f, &scene.face_gap[f * 6]);
            return true;
        }

        // the cpu resolver for near pairs: replicates the emit_type::surface
        // case of gather_sample_light for one light with the near branch
        // taken (no opaque entities or studio models by the run guard)
        bool resolve_near_pair(rad_state &state, const vec3v &pos, const vec3v &normal,
                               const directlight *l, float cone_power, float cone_scale,
                               const float *gap6, vec3v &add_out)
        {
            bool diversify = (cone_power != 1.0f || cone_scale != 1.0f);

            vec3v testline_origin;
            math::copy(l->origin, testline_origin);
            vec3v delta;
            math::subtract(l->origin, pos, delta);
            math::multiply_add(delta, -patch_hunt_offset, l->normal, delta);
            float dist = math::normalize(delta);
            float dot = math::dot(delta, normal);
            if (dist < 1.0f)
                dist = 1.0f;

            bool light_behind_surface = false;
            if (dot <= math::normal_epsilon)
                light_behind_surface = true;
            if (diversify && !light_behind_surface)
                dot = (vec_t)(cone_scale * std::pow((double)dot, cone_power));
            float dot2 = -math::dot(delta, l->normal);
            if (l->texlightgap > 0)
            {
                vec_t test = dot2 * dist;
                vec3v g0{gap6[0], gap6[1], gap6[2]};
                vec3v g1{gap6[3], gap6[4], gap6[5]};
                test -= l->texlightgap * std::fabs(math::dot(l->normal, g0));
                test -= l->texlightgap * std::fabs(math::dot(l->normal, g1));
                if (test < -math::on_epsilon)
                    return false;
            }
            if (dot2 * dist <= minimum_patch_distance)
                return false;

            vec_t range = l->patch_emitter_range;
            float ratio;
            if (l->stopdot > 0.0)
            {
                vec_t range_scale = 1 - l->stopdot2 * l->stopdot2;
                range_scale = (vec_t)(1 / std::sqrt(math::normal_epsilon > range_scale
                                                        ? math::normal_epsilon
                                                        : (double)range_scale));
                range_scale = range_scale < 2 ? range_scale : 2;
                range *= range_scale;
                if (dot2 <= l->stopdot2 + math::normal_epsilon)
                {
                    if (dist >= range)
                        return false;
                    ratio = 0.0;
                }
                else if (dot2 <= l->stopdot)
                {
                    ratio = dot * dot2 * (dot2 - l->stopdot2)
                        / (dist * dist * (l->stopdot - l->stopdot2));
                }
                else
                {
                    ratio = dot * dot2 / (dist * dist);
                }
            }
            else
            {
                ratio = dot * dot2 / (dist * dist);
            }
            if (ratio * l->patch_area > 0.4f)
                ratio = 0.4f / l->patch_area;

            if (!(dist < range - math::on_epsilon))
                return false;

            if (light_behind_surface)
            {
                dot = 0.0;
                ratio = 0.0;
            }
            get_alternate_origin(state, pos, normal, l->source_patch, testline_origin);
            if (test_line(state, pos, testline_origin) != contents_empty)
                return false;

            vec_t sightarea;
            int skylevel = l->source_patch->emitter_skylevel;
            if (l->stopdot > 0.0)
            {
                const vec3v &emitnormal =
                    plane_from_face_number(state, (unsigned)l->source_patch->facenumber)->normal;
                if (l->stopdot2 >= 0.8)
                    skylevel += 1;
                sightarea = calc_sight_area_spotlight(state, pos, normal, l->source_patch->winding,
                                                      emitnormal, l->stopdot, l->stopdot2, skylevel,
                                                      cone_power, cone_scale);
            }
            else
            {
                sightarea = calc_sight_area(state, pos, normal, l->source_patch->winding, skylevel,
                                            cone_power, cone_scale);
            }
            vec_t frac = dist / range;
            frac = (vec_t)((frac - 0.5) * 2);
            frac = frac < 1 ? frac : 1;
            frac = 0 > frac ? 0 : frac;
            vec_t ratio2 = (sightarea / l->patch_area);
            ratio = frac * ratio + (1 - frac) * ratio2;

            math::scale(l->intensity, ratio, add_out);
            return true;
        }

    }

    namespace
    {
        thread_local int t_current_face = -1;

        // the dispatch loop mostly sleeps in the fence wait while cpu work
        // (near pairs, the patch visibility) saturates every core; keep its
        // wake up prompt so the gpu is not left idle and the dispatch times
        // the chunk controller steers by are not inflated by scheduling
        class dispatch_thread_priority
        {
        public:
            dispatch_thread_priority()
            {
#ifdef _WIN32
                previous_ = GetThreadPriority(GetCurrentThread());
                SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
#endif
            }
            ~dispatch_thread_priority()
            {
#ifdef _WIN32
                SetThreadPriority(GetCurrentThread(), previous_);
#endif
            }

        private:
            [[maybe_unused]] int previous_ = 0;
        };

        // resolves the near pairs of finished gather chunks on the cpu while
        // the gpu runs the next chunk. a chunk's items are final once
        // gather_batch returns, and every pair of an item comes back with the
        // chunk that holds the item, so per item the pairs are still applied
        // in ascending light order onto the finished gpu sum, exactly as the
        // serial pass after the gather did
        class near_pair_pipeline
        {
        public:
            using resolve_fn = std::function<void(const gpu::near_pair &)>;

            explicit near_pair_pipeline(resolve_fn fn) : resolve_(std::move(fn))
            {
                worker_ = std::thread([this] { loop(); });
            }
            ~near_pair_pipeline() { stop(); }

            // pairs carry chunk relative item indices
            void push(size_t base, const std::vector<gpu::near_pair> &chunk_pairs)
            {
                if (chunk_pairs.empty())
                    return;
                job j;
                j.pairs.reserve(chunk_pairs.size());
                for (gpu::near_pair pair : chunk_pairs)
                {
                    pair.item += (uint32_t)base;
                    j.pairs.push_back(pair);
                }
                total_ += j.pairs.size();
                std::lock_guard<std::mutex> guard(mutex_);
                jobs_.push_back(std::move(j));
                cv_.notify_one();
            }

            // stop taking queued chunks; the chunk in flight completes
            void stop()
            {
                {
                    std::lock_guard<std::mutex> guard(mutex_);
                    stopping_ = true;
                    cv_.notify_one();
                }
                if (worker_.joinable())
                    worker_.join();
            }

            // resolve whatever the background worker had not reached, with
            // the progress bar the serial pass used to draw
            void finish()
            {
                stop();
                std::vector<gpu::near_pair> rest;
                for (job &j : jobs_)
                    rest.insert(rest.end(), j.pairs.begin(), j.pairs.end());
                jobs_.clear();
                std::vector<std::pair<size_t, size_t>> runs;
                prepare(rest, runs);
                threads::run_phase("lighting", "resolving near texlights", (int)runs.size(),
                                   [&](int run) { resolve_run(rest, runs[(size_t)run]); });
            }

            size_t total() const { return total_; }

        private:
            struct job
            {
                std::vector<gpu::near_pair> pairs;
            };

            static void prepare(std::vector<gpu::near_pair> &pairs,
                                std::vector<std::pair<size_t, size_t>> &runs)
            {
                std::sort(pairs.begin(), pairs.end(),
                          [](const gpu::near_pair &a, const gpu::near_pair &b)
                          {
                              return a.item != b.item ? a.item < b.item : a.light < b.light;
                          });
                for (size_t i = 0; i < pairs.size();)
                {
                    size_t j = i;
                    while (j < pairs.size() && pairs[j].item == pairs[i].item)
                        j++;
                    runs.emplace_back(i, j);
                    i = j;
                }
            }

            void resolve_run(const std::vector<gpu::near_pair> &pairs, std::pair<size_t, size_t> run) const
            {
                for (size_t p = run.first; p < run.second; p++)
                    resolve_(pairs[p]);
            }

            void loop()
            {
                for (;;)
                {
                    job j;
                    {
                        std::unique_lock<std::mutex> guard(mutex_);
                        cv_.wait(guard, [this] { return stopping_ || !jobs_.empty(); });
                        if (stopping_)
                            return;
                        j = std::move(jobs_.front());
                        jobs_.pop_front();
                    }
                    std::vector<std::pair<size_t, size_t>> runs;
                    prepare(j.pairs, runs);
                    threads::run((int)runs.size(), [&](int run) { resolve_run(j.pairs, runs[(size_t)run]); });
                }
            }

            resolve_fn resolve_;
            std::thread worker_;
            std::mutex mutex_;
            std::condition_variable cv_;
            std::deque<job> jobs_;
            bool stopping_ = false;
            size_t total_ = 0;
        };
    }

    void gpu_gather_begin_face(int facenum)
    {
        t_current_face = facenum;
    }

    void gpu_gather_intercept(rad_state &state, const vec3v &pos, const byte *pvs,
                              const vec3v &normal, vec3v *sample, byte *styles, int step,
                              int miptex, int texlightgap_surfacenum, int *cached_pvs_row)
    {
        gpu_gather_data &d = *state.gpu_gather;
        // group by the face whose build_facelights is running, not by the
        // sample's surface: near edges texlightgap_surfacenum names a
        // neighboring face owned by another thread
        const int facenum = t_current_face;
        if (facenum < 0)
            err::fatal("gpu gather: intercept without gpu_gather_begin_face");

        if (state.gpu_gather_phase == 1)
        {
            // collect: record the call; results stay zero for this pass
            gpu::work_item_gpu item = {};
            for (int x = 0; x < 3; x++)
            {
                item.pos[x] = pos[x];
                item.normal[x] = normal[x];
            }
            item.cone_power = 1.0f;
            item.cone_scale = 1.0f;
            if (miptex >= 0 && miptex < (int)state.lightingconeinfo.size())
            {
                item.cone_power = state.lightingconeinfo[(size_t)miptex][0];
                item.cone_scale = state.lightingconeinfo[(size_t)miptex][1];
            }
            item.step = step;
            item.face = texlightgap_surfacenum; // the kernel's texlightgap axes
            item.pvs_row = cached_pvs_row ? *cached_pvs_row : -1;
            if (item.pvs_row < 0)
            {
                std::lock_guard<std::mutex> guard(d.row_mutex);
                std::string key((const char *)pvs, d.rowbytes);
                auto it = d.row_lookup.find(key);
                if (it == d.row_lookup.end())
                {
                    it = d.row_lookup.emplace(std::move(key), (int)d.pvs_rows.size()).first;
                    d.pvs_rows.emplace_back(pvs, pvs + d.rowbytes);
                }
                item.pvs_row = it->second;
                if (cached_pvs_row)
                    *cached_pvs_row = item.pvs_row;
            }
            d.face_items[(size_t)facenum].push_back(item);
            return;
        }

        // consume: the calls repeat in collect order (same deterministic code
        // path on the same inputs); any drift is a hard bug
        const int cursor = d.face_cursor[(size_t)facenum]++;
        const int flat = d.face_first[(size_t)facenum] + cursor;
        const gpu::work_item_gpu &recorded = d.items[(size_t)flat];
        if (recorded.step != step || recorded.face != texlightgap_surfacenum
            || recorded.pos[0] != pos[0] || recorded.pos[1] != pos[1] || recorded.pos[2] != pos[2])
        {
            err::fatal("gpu gather: call order drift on face %d (item %d)", facenum, cursor);
        }
        const uint32_t touched_mask = d.results.touched((size_t)flat);

        // the same tail merge the cpu gather ends with: coring threshold,
        // style slot assignment, discarded light bookkeeping
        for (int style = 0; style < allstyles; ++style)
        {
            const int compact = d.style_to_compact[style];
            const bool touched = compact >= 0 && (touched_mask & (1u << compact));
            if (!touched && state.corings[style] >= 0)
            {
                continue;
            }
            vec3v adds{};
            if (touched)
            {
                float rgb[3];
                d.results.read_color((size_t)flat, (uint32_t)compact, rgb);
                adds = vec3v{rgb[0], rgb[1], rgb[2]};
            }
            if (vector_maximum(adds) > state.corings[style] * 0.1)
            {
                int style_index;
                for (style_index = 0; style_index < allstyles; style_index++)
                {
                    if (styles[style_index] == style || styles[style_index] == 255)
                    {
                        break;
                    }
                }
                if (style_index == allstyles) // shouldn't happen
                {
                    std::lock_guard<std::mutex> guard(state.lock);
                    if (++state.stylewarningcount >= state.stylewarningnext)
                    {
                        state.stylewarningnext = state.stylewarningcount * 2;
                        logging::warn("Too many direct light styles on a face(%f,%f,%f)",
                                      pos[0], pos[1], pos[2]);
                        logging::warn(" total %d warnings for too many styles",
                                      state.stylewarningcount);
                    }
                    return;
                }
                if (styles[style_index] == 255)
                {
                    styles[style_index] = (byte)style;
                }
                math::add(sample[style_index], adds, sample[style_index]);
            }
            else
            {
                if (vector_maximum(adds) > state.maxdiscardedlight + math::normal_epsilon)
                {
                    std::lock_guard<std::mutex> guard(state.lock);
                    if (vector_maximum(adds) > state.maxdiscardedlight + math::normal_epsilon)
                    {
                        state.maxdiscardedlight = vector_maximum(adds);
                        math::copy(pos, state.maxdiscardedpos);
                    }
                }
            }
        }
    }

    bool gpu_gather_run(rad_state &state, const std::function<void()> &after_collect)
    {
        const format::map_data &map = *state.map;

        if (!state.opaque_list.empty() || state.num_studio_models != 0)
        {
            logging::warn("-gpu: map has opaque entities or studio models; using the cpu path");
            return false;
        }
        if (state.lightarray.empty() || map.faces.empty())
        {
            logging::warn("-gpu: no direct lights; using the cpu path");
            return false;
        }
        if (!gpu::available())
        {
            logging::warn("-gpu: %s; using the cpu path", gpu::last_error().c_str());
            return false;
        }

        gpu_gather_data *d = new gpu_gather_data();
        state.gpu_gather = d;
        d->rowbytes = (size_t)((map.models[0].visleafs + 7) / 8);
        d->face_items.resize(map.faces.size());

        gpu::gather_scene scene;
        if (!marshal_scene(state, *d, scene))
        {
            logging::warn("-gpu: map exceeds the kernel's light/style limits; using the cpu path");
            gpu_gather_finish(state);
            return false;
        }

        // Place samples and record gather calls without building face lights.
        state.gpu_gather_phase = 1;
        threads::run_phase("lighting", "collecting light samples", (int)map.faces.size(),
                           [&](int i)
                           {
                               gpu_gather_begin_face(i);
                               build_facelights(state, i);
                           });

        // flatten face major
        size_t total = 0;
        d->face_first.resize(map.faces.size());
        for (size_t f = 0; f < map.faces.size(); f++)
        {
            d->face_first[f] = (int)total;
            total += d->face_items[f].size();
        }
        d->items.reserve(total);
        for (size_t f = 0; f < map.faces.size(); f++)
        {
            d->items.insert(d->items.end(), d->face_items[f].begin(), d->face_items[f].end());
            d->face_items[f].clear();
            d->face_items[f].shrink_to_fit();
        }
        if (d->items.empty())
        {
            logging::warn("-gpu: no light samples collected; using the cpu path");
            state.gpu_gather_phase = 0;
            gpu_gather_finish(state);
            return false;
        }

        if (after_collect)
            after_collect();

        // pvs rows to words
        const uint32_t stride_words = (uint32_t)((d->rowbytes + 3) / 4 + 1);
        std::vector<uint32_t> pvs_words(d->pvs_rows.size() * stride_words, 0);
        for (size_t r = 0; r < d->pvs_rows.size(); r++)
            std::memcpy(&pvs_words[r * stride_words], d->pvs_rows[r].data(), d->rowbytes);

        // dispatch in chunks over a resident session (the scene and the
        // visible leaf lists upload once); near pairs come back per chunk
        // with chunk relative item indices
        const size_t chunk_cap = d->items.size() < gather_chunk_items
            ? d->items.size() : gather_chunk_items;
        if (!d->results.resize(d->items.size(), scene.style_count))
        {
            logging::warn("-gpu: could not allocate gather results; using the cpu path");
            gpu_gather_finish(state);
            return false;
        }
        if (!gpu::gather_begin(scene, pvs_words, stride_words, (uint32_t)chunk_cap))
        {
            logging::warn("-gpu: %s; using the cpu path", gpu::last_error().c_str());
            state.gpu_gather_phase = 0;
            gpu_gather_finish(state);
            return false;
        }
        {
            const char *fp64 = std::getenv("HLTOOLS_GPU_FP64_NORMALIZE");
            if (fp64 && *fp64 && *fp64 != '0')
                logging::file("gpu lighting: fp64-parity kernel variant active\n");
        }
        logging::file("gpu lighting: %zu bytes per result (style count %u)\n",
                      gpu::gather_results::words_per_item(scene.style_count) * sizeof(uint32_t),
                      scene.style_count);
        std::vector<gpu::near_pair> chunk_near;
        near_pair_pipeline near_resolver(
            [&](const gpu::near_pair &pair)
            {
                const gpu::work_item_gpu &item = d->items[pair.item];
                const directlight *l = &state.lightarray[pair.light];
                vec3v pos{item.pos[0], item.pos[1], item.pos[2]};
                vec3v normal{item.normal[0], item.normal[1], item.normal[2]};
                vec3v add;
                if (resolve_near_pair(state, pos, normal, l,
                                      item.cone_power, item.cone_scale,
                                      &scene.face_gap[(size_t)item.face * 6], add))
                {
                    const int compact = d->style_to_compact[l->style];
                    const float rgb[3] = {add[0], add[1], add[2]};
                    d->results.add_color(pair.item, (uint32_t)compact, rgb);
                }
            });
        progress::section("lighting");
        progress::begin("gathering light on the gpu", (int)d->items.size());
        dispatch_thread_priority dispatch_priority;
        size_t chunk = d->items.size() < gather_chunk_start
            ? d->items.size() : gather_chunk_start;
        size_t chunk_lo = chunk;
        size_t chunk_hi = chunk;
        size_t chunk_ceiling = gather_chunk_items;
        unsigned fast_at_ceiling = 0;
        size_t dispatches = 0;
        double spent = 0.0;
        double slowest = 0.0;
        for (size_t base = 0; base < d->items.size();)
        {
            const size_t count = d->items.size() - base < chunk
                ? d->items.size() - base : chunk;
            const auto started = std::chrono::steady_clock::now();
            const bool ok = gpu::gather_batch(&d->items[base], count, d->results, base, chunk_near);
            const double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - started).count();
            if (!ok)
            {
                progress::end();
                // say what the dispatch that died actually looked like, so a
                // fallback can be told apart from a chunk that ran too long
                logging::file("gpu lighting: gather failed on dispatch %zu at item"
                              " %zu/%zu (chunk %zu items, %.3fs; %zu dispatches ok,"
                              " %.3fs total, %.3fs slowest)\n",
                              dispatches + 1, base, d->items.size(), count, elapsed,
                              dispatches, spent, slowest);
                near_resolver.stop(); // it writes into d->results
                gpu::gather_end();
                logging::warn("-gpu: %s; using the cpu path", gpu::last_error().c_str());
                state.gpu_gather_phase = 0;
                gpu_gather_finish(state);
                return false;
            }
            dispatches++;
            spent += elapsed;
            if (elapsed > slowest)
                slowest = elapsed;
            near_resolver.push(base, chunk_near);
            base += count;
            progress::add((int)count);

            // Reduce the ceiling on a slow dispatch. Recover in steps when
            // repeated full chunks show enough headroom to grow safely.
            if (elapsed > gather_ceiling_seconds)
            {
                const double scaled = (double)count * (gather_ceiling_seconds / elapsed);
                const size_t capped = scaled < (double)gather_chunk_min
                    ? gather_chunk_min : (size_t)scaled;
                chunk_ceiling = std::min(chunk_ceiling, capped);
                fast_at_ceiling = 0;
            }
            else if (chunk_ceiling < gather_chunk_items && count == chunk_ceiling
                     && elapsed < gather_target_seconds / 2)
            {
                if (++fast_at_ceiling == gather_recovery_dispatches)
                {
                    chunk_ceiling = std::min(gather_chunk_items, chunk_ceiling + chunk_ceiling / 2);
                    fast_at_ceiling = 0;
                }
            }
            else
            {
                fast_at_ceiling = 0;
            }

            // steer the next chunk at the target. growth is capped at 1.5x a
            // step so the size creeps up to the ceiling instead of jumping
            // there, while a slow dispatch shrinks it proportionally at once
            if (elapsed > 1e-6)
            {
                double factor = gather_target_seconds / elapsed;
                if (factor > 1.5)
                    factor = 1.5;
                else if (factor < 0.25)
                    factor = 0.25;
                const double next = (double)count * factor;
                chunk = next < (double)gather_chunk_min
                    ? gather_chunk_min
                    : (next > (double)chunk_ceiling ? chunk_ceiling : (size_t)next);
                if (chunk < chunk_lo)
                    chunk_lo = chunk;
                if (chunk > chunk_hi)
                    chunk_hi = chunk;
            }
        }
        progress::end();
        logging::file("gpu lighting: gather chunk steered over %zu..%zu items"
                      " (target %.2fs a dispatch, ceiling %zu); %zu dispatches,"
                      " %.3fs total, %.3fs slowest\n",
                      chunk_lo, chunk_hi, gather_target_seconds, chunk_ceiling,
                      dispatches, spent, slowest);
        gpu::gather_end();

        // resolve the near pairs the background worker had not reached
        near_resolver.finish();
        const size_t near_total = near_resolver.total();

        // diagnostic counts go to the logfile only; the device itself is
        // already on the console in the settings table
        logging::file("gpu lighting: %s, %zu samples, %zu near pairs on the cpu\n",
                      gpu::device_name().c_str(), d->items.size(), near_total);

        d->face_cursor.assign(map.faces.size(), 0);
        // Style zero and the face's emitter style can already occupy slots.
        // Negative coring also admits zero contributions from absent styles.
        int cache_styles = 2;
        for (int style = 0; style < allstyles; style++)
        {
            if (d->style_to_compact[style] >= 0 || state.corings[style] < 0)
                cache_styles++;
        }
        state.gpu_gather_cache_styles = std::min(cache_styles, allstyles);
        state.gpu_gather_phase = 2;
        return true;
    }

    void gpu_gather_finish(rad_state &state)
    {
        state.gpu_gather_phase = 0;
        state.gpu_gather_cache_styles = allstyles;
        delete state.gpu_gather;
        state.gpu_gather = nullptr;
    }

    std::string gpu_device_description()
    {
        if (gpu::available())
            return gpu::device_name();
        return "none found (will use the cpu)";
    }
}

#else // !hltools_gpu_enabled

namespace rad
{
    bool gpu_gather_run(rad_state &state, const std::function<void()> &)
    {
        logging::warn("-gpu: this build has no gpu backend; using the cpu path");
        return false;
    }

    void gpu_gather_finish(rad_state &)
    {
    }

    void gpu_gather_begin_face(int)
    {
    }

    void gpu_gather_intercept(rad_state &, const vec3v &, const byte *, const vec3v &,
                              vec3v *, byte *, int, int, int, int *)
    {
        err::fatal("gpu_gather_intercept called without the gpu backend");
    }

    std::string gpu_device_description()
    {
        return "none in this build (will use the cpu)";
    }
}

#endif
