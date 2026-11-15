#pragma once

#include <cstdint>
#include <cstring>
#include <new>
#include <string>
#include <vector>

// the vulkan compute lighting backend everything here is host side plumbing
// with no vulkan types exposed,
// so the rest of rad can compile without the backend (hltools_gpu=off)

namespace rad
{
    namespace gpu
    {
        // lazily creates the d3d12 device on first call prefers the
        // high performance adapter (discrete over integrated); index overrides
        // via adapter_override (-1 = automatic)
        bool available();
        void set_adapter_override(int index);
        std::string device_name();
        // last initialization or dispatch error, for the fallback warning
        std::string last_error();

        // ===== batched bsp tracing =====

        // mirrors rad::tnode in an explicitly padded 32 byte layout
        struct tnode_gpu
        {
            float normal[3];
            float dist;
            int32_t type;
            int32_t children[2];
            int32_t pad = 0;
        };
        static_assert(sizeof(tnode_gpu) == 32, "must match the hlsl Tnode layout");

        struct trace_segment
        {
            float start[3];
            float stop[3];
        };
        static_assert(sizeof(trace_segment) == 24, "must match the hlsl Segment layout");

        // contents result per segment plus the sky entry point when the result
        // is contents_sky; flags bit 0 marks segments that took the rare
        // in plane fallback path (its sky vs solid ordering is approximated,
        // see trace_bsphlsl), bit 1 a traversal stack overflow
        struct trace_result
        {
            int32_t contents;
            int32_t flags;
            float skyhit[3];
            int32_t pad = 0;
        };
        static_assert(sizeof(trace_result) == 24, "must match the hlsl Result layout");

        bool trace_batch(const std::vector<tnode_gpu> &tnodes,
                         const std::vector<trace_segment> &segments,
                         std::vector<trace_result> &results);

        // ===== direct lighting gather =====
        // mirrors of the std430 structs in shaders/gathercomp; all layouts
        // are scalar only on purpose (std430 vec3/ivec2 alignment)

        constexpr int max_compact_styles = 16;

        struct lightleaf_gpu
        {
            int32_t leafnum;
            int32_t firstlight;
            int32_t numlights;
            int32_t pad = 0;
        };
        static_assert(sizeof(lightleaf_gpu) == 16, "must match the glsl LightLeaf layout");

        struct light_gpu
        {
            int32_t type;         // emit_type
            int32_t compact_style; // host assigned compact style slot
            int32_t topatch;
            int32_t sun_ofs;      // into the sun normal array, 4 floats per entry
            int32_t sun_count;
            int32_t pad0 = 0;
            float origin[3];
            float intensity[3];
            float normal[3];
            float stopdot;
            float stopdot2;
            float fade;
            float texlightgap;
            float patch_area;
            float patch_emitter_range;
            float diffuse[3];
            float diffuse2[3];
            float pad1 = 0;
        };
        static_assert(sizeof(light_gpu) == 112, "must match the glsl Light layout");

        struct work_item_gpu
        {
            float pos[3];
            float normal[3];
            float cone_power; // lightingconeinfo[miptex]
            float cone_scale;
            int32_t pvs_row;
            int32_t step;
            int32_t face; // index into the per face texlightgap axis table
            int32_t pad = 0;
        };
        static_assert(sizeof(work_item_gpu) == 48, "must match the glsl WorkItem layout");

        // One mask followed by three float words per active style, as in
        // gather.comp. A single-style sample occupies 16 bytes.
        class gather_results
        {
        public:
            static constexpr size_t words_per_item(uint32_t styles)
            {
                return 1 + 3 * styles;
            }

            bool resize(size_t count, uint32_t styles)
            {
                if (styles == 0 || styles > max_compact_styles
                    || count > words.max_size() / words_per_item(styles))
                    return false;
                try
                {
                    words.resize(count * words_per_item(styles));
                }
                catch (const std::bad_alloc &)
                {
                    return false;
                }
                style_count = styles;
                return true;
            }

            uint32_t styles() const { return style_count; }
            size_t size() const { return words.size() / words_per_item(style_count); }
            uint32_t *data_at(size_t item) { return words.data() + item * words_per_item(style_count); }
            uint32_t touched(size_t item) const { return words[item * words_per_item(style_count)]; }

            void read_color(size_t item, uint32_t style, float *rgb) const
            {
                std::memcpy(rgb, words.data() + color_offset(item, style), 3 * sizeof(float));
            }

            void add_color(size_t item, uint32_t style, const float *rgb)
            {
                float color[3];
                read_color(item, style, color);
                for (int i = 0; i < 3; i++)
                    color[i] += rgb[i];
                std::memcpy(words.data() + color_offset(item, style), color, sizeof(color));
                *data_at(item) |= 1u << style;
            }

        private:
            static_assert(sizeof(float) == sizeof(uint32_t), "gather uses 32-bit float words");

            size_t color_offset(size_t item, uint32_t style) const
            {
                return item * words_per_item(style_count) + 1 + 3 * style;
            }

            std::vector<uint32_t> words;
            uint32_t style_count = 0;
        };

        // the per map immutable inputs, uploaded once per gather session
        struct gather_scene
        {
            std::vector<tnode_gpu> tnodes;
            std::vector<lightleaf_gpu> lightleafs;
            std::vector<light_gpu> lights;
            std::vector<float> sun_normals;  // 4 floats per entry (xyz + weight)
            std::vector<float> sky_normals;  // 4 floats per entry, active sky level only
            std::vector<float> face_gap;     // 6 floats per face (texlightgap axes)
            uint32_t style_count = 0;
            uint32_t sky_lighting_fix = 0;
            int32_t sky_step_match = 0;
            float indirect_sun = 0;
        };

        // a (work item, light) pair the kernel routed to the cpu (the surface
        // near branch: sight area blend + alternate origin)
        struct near_pair
        {
            uint32_t item;
            uint32_t light;
        };

        // gather_begin uploads the scene once and keeps
        // it resident across the chunked dispatches (it also builds the
        // per pvs row visible lit leaf lists host side, and refuses maps
        // whose bsp tree is deeper than the kernel's trace stack); each
        // gather_batch call then only uploads count <= max_chunk_items work
        // items and reads its results back gather_end releases everything
        // (idempotent, safe on failure paths) requires available() the
        // default kernel normalizes in plain float and matches the validation
        // results from the fp64 parity version while needing no shaderfloat64
        // hltools_gpu_fp64_normalize=1 selects the fp64 vectornormalize parity
        // variant for debugging (requires shaderfloat64)
        bool gather_begin(const gather_scene &scene,
                          const std::vector<uint32_t> &pvs_words,
                          uint32_t pvs_stride_words,
                          uint32_t max_chunk_items);
        // Writes into count preallocated results starting at first_result.
        bool gather_batch(const work_item_gpu *items, size_t count,
                          gather_results &results, size_t first_result,
                          std::vector<near_pair> &near_pairs);
        void gather_end();

        // one call convenience for cpu and gpu comparison tests
        bool gather_batch(const gather_scene &scene,
                          const std::vector<work_item_gpu> &items,
                          const std::vector<uint32_t> &pvs_words,
                          uint32_t pvs_stride_words,
                          gather_results &results,
                          std::vector<near_pair> &near_pairs);

        // ===== patch to patch transfer factors =====

        struct patch_gpu
        {
            float origin[3];
            float normal[3];
            float area;
            float emitter_range;
            float exposure;
            float cone_power;
            float cone_scale;
            float pad0 = 0;
            int32_t skylevel;
            int32_t wind_ofs;
            int32_t wind_count;
            int32_t pad1 = 0;
        };
        static_assert(sizeof(patch_gpu) == 64, "must match the glsl PatchData layout");

        struct transfer_pair
        {
            int32_t receiver;
            int32_t emitter;
        };
        static_assert(sizeof(transfer_pair) == 8, "must match the glsl Pair layout");

        struct formfactor_scene
        {
            std::vector<patch_gpu> patches;
            std::vector<float> windings;      // 3 floats per vertex
            std::vector<float> sky_normals;   // 4 floats per normal, all levels
            std::vector<int32_t> sky_levels;  // (offset, count) per level
        };

        // stages the scene on the device for the formfactor_batch calls that
        // pass this same scene object, until formfactor_end; without it every
        // batch uploads the scene again
        bool formfactor_begin(const formfactor_scene &scene);
        void formfactor_end();

        // computes one transfer factor per pair; nan marks pairs the host
        // must recompute (emitter winding too large for the kernel)
        bool formfactor_batch(const formfactor_scene &scene,
                              const std::vector<transfer_pair> &pairs,
                              std::vector<float> &trans);
    }
}
