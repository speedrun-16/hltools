#pragma once

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include "../common/limits.h"
#include "../common/types.h"
#include "format/bsp/data.h"
#include "format/bsp/entity_lump.h"
#include "format/wad/archive.h"
#include "../math/vector.h"
#include "../math/winding.h"
#include "rad.h"

// shared state and cross module declarations for the rad stage the stage keeps
// the reference's data shapes (patch pool with intrusive next pointers, per face
// arrays, transfer lists) because the lighting solve depends on the exact order
// they impose on every accumulation

namespace rad
{
    using math::vec3v;

    // ===== constants =====

    // rad's own winding quad half size (csg 80000, bsp 144000)
    constexpr vec_t bogus_range = 131072;

    constexpr int max_points_on_winding = 128;

    // sample placement search: how far in front of the plane samples sit, and
    // the radial hunt iterations/growth used to escape walls
    constexpr vec_t default_hunt_offset = 0.5;
    constexpr int default_hunt_size = 11;
    constexpr vec_t default_hunt_scale = 0.1f;
    constexpr vec_t default_edge_width = 0.8f;
    constexpr vec_t patch_hunt_offset = 0.5;
    constexpr vec_t hunt_wall_epsilon = (vec_t)(3 * math::on_epsilon);

    constexpr vec_t minimum_patch_distance = (vec_t)math::on_epsilon;
    constexpr vec_t accuratebounce_threshold = 4.0;
    constexpr int accuratebounce_default_skylevel = 5;

    // the engine's light style ceiling; every style indexed array uses it
    constexpr int allstyles = 64;

    constexpr int maxlightmaps = limits::max_lightmaps;
    constexpr int texture_step = limits::texture_step;

    // texinfo flag: no lightmap (sky and liquids)
    constexpr int tex_special = 1;

    // special "angle" key values
    constexpr double angle_up = -1.0;
    constexpr double angle_down = -2.0;

    // engine contents values used by the tracing code
    constexpr int contents_empty = -1;
    constexpr int contents_solid = -2;
    constexpr int contents_water = -3;
    constexpr int contents_slime = -4;
    constexpr int contents_lava = -5;
    constexpr int contents_sky = -6;

    // sky sampling normal sets: level n holds 6, 18, 66, , 65538 normals
    constexpr int skylevel_max = 8;
    constexpr int skylevel_softsky_on = 7;
    constexpr int skylevel_softsky_off = 4;
    constexpr int sunspread_skylevel = 7;
    constexpr vec_t sunspread_threshold = 15.0;

    // transfer storage limits, set by the 12/20 bit split in transfer_index
    constexpr unsigned max_compressed_transfer_index_size = (1 << 12) - 1;
    constexpr int max_patches = 65535 * 16;
    constexpr int max_vismatrix_patches = 65535;
    constexpr int max_sparse_vismatrix_patches = max_patches;

    // lighting adjustments an opaque entity can carry
    constexpr int lightmode_null = 0;
    constexpr int lightmode_opaque = 0x02;
    constexpr int lightmode_nonsolid = 0x08; // opaque entity with {texture

    // ===== basic types =====

    // rad's plane: the wire dplane_t widened to the stage vector type, plus the
    // axial type the tracer switches on
    struct plane
    {
        vec3v normal;
        vec_t dist = 0;
        int type = 0; // plane_x  plane_anyz as stored in the bsp
    };

    constexpr int plane_x = 0;
    constexpr int plane_y = 1;
    constexpr int plane_z = 2;
    constexpr int plane_anyx = 3;
    constexpr int plane_anyy = 4;
    constexpr int plane_anyz = 5;

    // a 4x4 affine transform stored as four column vectors:
    //   out = v[0] * inx + v[1] * iny + v[2] * inz + v[3]
    struct matrix
    {
        vec_t v[4][3];
    };

    // ===== lights =====

    enum class emit_type
    {
        surface,
        point,
        spotlight,
        skylight,
    };

    struct patch;

    struct directlight
    {
        directlight *next;
        emit_type type;
        int style;
        vec3v origin;
        vec3v intensity;
        vec3v normal;   // for surfaces and spotlights
        float stopdot;  // for spotlights
        float stopdot2; // for spotlights

        // falloff scaling: 10 = normal, 05 = farther, 20 = shorter
        vec_t fade;

        // diffuse light_environment colour (adam foster's sky hack)
        vec3v diffuse_intensity;
        vec3v diffuse_intensity2;
        vec_t sunspreadangle;
        int numsunnormals;
        vec3v *sunnormals;
        vec_t *sunnormalweights;

        vec_t patch_area;
        vec_t patch_emitter_range;
        patch *source_patch;
        vec_t texlightgap;
        bool topatch;
    };

    // ===== patches =====

    // compressed transfer list entry: a run of patch indices sharing one data run
    struct transfer_index
    {
        unsigned size : 12;
        unsigned index : 20;
    };

    using transfer_raw_index = unsigned;
    using transfer_data = unsigned char;
    using rgb_transfer_data = unsigned char;

    enum patch_flags
    {
        patch_flag_null = 0,
        patch_flag_outside = 1,
    };

    struct patch
    {
        patch *next;                // next in face
        vec3v origin;               // center centroid of winding
        vec_t area;                 // surface area of this patch
        vec_t exposure;
        vec_t emitter_range;        // range from patch origin
        int emitter_skylevel;       // skylevel for normal sampling in accurate bounce
        math::winding *winding;
        vec_t scale;                // texture scale for this face
        vec_t chop;                 // texture chop for this face

        unsigned i_index;
        unsigned i_data;

        transfer_index *t_index;
        transfer_data *t_data;
        rgb_transfer_data *t_rgb_data;

        int facenumber;
        patch_flags flags;
        bool translucent_b;         // gather light from behind
        vec3v translucent_v;
        vec3v texturereflectivity;
        vec3v bouncereflectivity;

        unsigned char totalstyle[maxlightmaps];
        unsigned char directstyle[maxlightmaps];
        // totallight: all light gathered by patch during radiosity, excluding
        // what direct lighting already accounted for
        vec3v totallight[maxlightmaps];
        // directlight: emissive light gathered by sample
        vec3v directlight[maxlightmaps];
        int bouncestyle;            // light reflected from this patch converts to this style, -1 = normal
        unsigned char emitstyle;
        vec3v baselight;            // emissivity only, uses emitstyle
        bool emitmode;              // texlight emit mode: true normal, false fast
        vec_t samples;
        vec3v *samplelight_all;     // null, or [allstyles] during build_facelights
        unsigned char *totalstyle_all;
        vec3v *totallight_all;
        vec3v *directlight_all;
        int leafnum;
    };

    // ===== face adjacency (phong smoothing) =====

    struct facelist
    {
        format::dface_t *face;
        facelist *next;
    };

    struct edgeshare
    {
        format::dface_t *faces[2];
        vec3v interface_normal; // set when smooth
        vec3v vertex_normal[2];
        vec_t cos_normals_angle; // set when smooth
        bool coplanar;
        bool smooth;
        facelist *vertex_facelist[2]; // other possible smooth faces
        matrix textotex[2];           // texture coordinate translation between the faces
    };

    // ===== opaque entities =====

    struct opaque_entity
    {
        int entitynum;
        int modelnum;
        vec3v origin;

        vec3v transparency_scale;
        bool transparency;
        // -1 = no style; transparency must be false if style >= 0 style 0 and
        // the same style change to this style, other styles are blocked
        int style;
        // the entity cannot be seen inside, so samples must move outside of it
        bool block;
    };

    // ===== textures =====

    struct rad_texture
    {
        char name[16]; // not always the same as the name in texdata
        int width, height;
        std::vector<byte> canvas; // [height][width] palette indices
        byte palette[256][3];
        vec3v reflectivity;
    };

    struct minlight
    {
        std::string name;
        float value;
    };

    // a texlight definition, from lightsrad files or the info_texlights entity
    struct rad_texlight
    {
        std::string name;
        vec3v value;
        std::string source; // filename or "info_texlights", for override reports
    };

    // ===== tracing (tracecpp) =====

    // the disk node structure converted into the efficient tracing structure
    // negative children hold the leaf contents directly
    struct tnode
    {
        int type; // planetypes
        vec3v normal;
        float dist;
        int children[2];
    };

    // world faces of opaque entity models, merged per node and edge fenced for
    // the point in face test
    struct opaque_face
    {
        math::winding *winding;
        plane pl;
        int numedges;
        std::vector<plane> edges;
        int texinfo;
        bool tex_alphatest;
        vec_t tex_vecs[2][4];
        int tex_width;
        int tex_height;
        const byte *tex_canvas;
    };

    struct opaque_node
    {
        int type; // planetypes
        vec3v normal;
        vec_t dist;
        int children[2];
        int firstface;
        int numfaces;
    };

    struct opaque_model
    {
        vec3v mins, maxs;
        int headnode;
    };

    // ===== transfer machinery =====

    // one compressed row of the sparse visibility matrix: 8 patch bits at a
    // byte offset
    struct sparse_row
    {
        unsigned offset : 24;
        unsigned values : 8;
    };

    struct sparse_column
    {
        sparse_row *row;
        int count;
    };

    // custom shadow transparency between a patch pair, stored as an index into
    // the deduplicated value list
    struct trans_pair
    {
        unsigned p1;
        unsigned p2;
        unsigned data_index;
    };

    struct style_pair
    {
        unsigned p1;
        unsigned p2;
        char style;
    };

    struct rad_state;

    // the visibility test the transfer builders call, chosen by the method
    using check_vis_bit_fn = bool (*)(rad_state &, unsigned, unsigned, vec3v &, unsigned int &);

    inline const vec3v vec3_one{1.0, 1.0, 1.0};

    // the reference vectormaximum macro: qmax(v0, qmax(v1, v2))
    inline vec_t vector_maximum(const vec3v &v)
    {
        vec_t m = v[1] > v[2] ? v[1] : v[2];
        return v[0] > m ? v[0] : m;
    }

    inline vec_t vector_minimum(const vec3v &v)
    {
        vec_t m = v[1] < v[2] ? v[1] : v[2];
        return v[0] < m ? v[0] : m;
    }

    // ===== face light storage (lightmapcpp) =====

    struct sample
    {
        vec3v pos;
        vec3v light;
        int surface; // the sample can grow into another face
    };

    struct facelight
    {
        int numsamples;
        sample *samples[maxlightmaps];
    };

    // compact ascending list of leafs that contain at least one direct light,
    // with all lights copied into one contiguous array (grouped by leaf,
    // preserving the per leaf list order) gather_sample_light runs once per
    // lightmap sample; scanning every leaf of the map and chasing scattered
    // heap nodes there dominated its loop overhead
    struct lightleaf
    {
        int leafnum;
        int firstlight; // index into the light array
        int numlights;
    };

    // ===== sample positions (positioncpp) =====

    struct sample_position
    {
        bool valid;
        bool nudged;
        vec_t best_s; // find_nearest_position returns these
        vec_t best_t;
        vec3v pos; // with default_hunt_offset applied
    };

    struct position_map
    {
        bool valid = false;
        int facenum = 0;
        vec3v face_offset;
        vec3v face_centroid;
        matrix worldtotex;
        matrix textoworld;
        math::winding facewinding;
        plane faceplane;
        math::winding facewindingwithoffset;
        plane faceplanewithoffset;
        math::winding texwinding;
        plane texplane; // (0, 0, 1, 0) or (0, 0, -1, 0)
        vec3v texcentroid;
        vec3v start; // s_start, t_start, 0
        vec3v step;  // s_step, t_step, 0
        int w = 0;   // number of s
        int h = 0;   // number of t
        std::vector<sample_position> grid; // [h][w]
    };

    // ===== stage state =====

    struct rad_state
    {
        format::map_data *map = nullptr;
        rad_options options;
        std::string base_path;
        std::vector<format::entity> entities;

        // planes widened to vec_t, and the reversed copies used by faces that
        // lie on the back side of their plane
        std::vector<plane> planes;
        std::vector<plane> backplanes;

        // per face lighting inputs
        std::vector<patch *> face_patches;
        std::vector<format::entity *> face_entity;
        std::vector<format::entity *> face_texlights;
        std::vector<vec3v> face_offset; // for models with origins
        std::vector<vec3v> face_centroids;
        std::vector<unsigned char> face_lightmode;

        // the patch pool the reference allocates the worst case up front and
        // shrinks it afterwards; pointers into this storage live in
        // face_patches and patch::next, so it never reallocates while in use
        std::vector<patch> patches;
        unsigned num_patches = 0;

        std::vector<opaque_entity> opaque_list;

        // sky sampling normal sets per level, built by build_diffuse_normals
        int numskynormals[skylevel_max + 1] = {};
        std::vector<vec3v> skynormals[skylevel_max + 1];
        std::vector<vec_t> skynormalsizes[skylevel_max + 1]; // weight per normal

        // per miptex tables read from the worldspawn texture keys
        std::vector<vec_t> smoothvalues;
        std::vector<vec3v> translucenttextures;
        std::vector<vec3v> lightingconeinfo; // x = power, y = scale

        // loaded wad textures, aligned with the miptex lump
        std::vector<rad_texture> textures;

        // wad files opened while resolving textures the bsp only references
        std::vector<format::wad_archive> wad_files;
        bool wad_files_opened = false;

        // world tracing structure for light occlusion
        std::vector<tnode> tnodes;

        // opaque entity tracing structures
        std::vector<opaque_face> opaquefaces;
        std::vector<opaque_node> opaquenodes;
        std::vector<opaque_model> opaquemodels;

        std::vector<minlight> minlights;
        std::vector<rad_texlight> texlights;
        // texture names whose faces receive a constant-white lightmap. Source
        // decompilation uses this to preserve whole-surface unlit shaders.
        std::vector<std::string> unlittextures;

        // per miptex chop scale from info_chopscale
        std::vector<vec_t> chopscales;

        // total base patch area, logged after make_patches
        float totalarea = 0;

        // face edge adjacency for phong smoothing, sized to the edge lump
        std::vector<edgeshare> edgeshares;

        // sample position grids per face
        std::vector<position_map> face_positions;

        // per face patch triangulations for sample interpolation (lerpcpp)
        std::vector<struct face_triangulation *> facetriangulations;

        // per leaf direct light lists plus the flattened iteration copies
        std::vector<directlight *> directlights;
        std::vector<lightleaf> lightleafs;
        std::vector<directlight> lightarray;

        // per face computed light samples
        std::vector<facelight> facelights;

        // -dumpgather: per face serialized direct gather blobs, filled by
        // build_facelights (per face slot, so no locking) and written to
        // <base>gather after the phase empty when the flag is off
        std::vector<std::vector<byte>> gather_dump;

        // -gpu: gather interception (gpu_gathercpp) 0 = off (the cpu path),
        // 1 = collect (record work items, produce nothing), 2 = consume
        // (serve the gpu results through the same tail merge)
        int gpu_gather_phase = 0;
        struct gpu_gather_data *gpu_gather = nullptr;

        // faces whose samples grew into a given face (add_patch_lights)
        std::vector<std::vector<int>> dependentfacelights;

        // lightmap data budget (-lightdata), enforced while offsets are laid out
        int max_map_lightdata = limits::max_map_lightdata;

        // per style coring thresholds derived from optionscoring
        vec_t corings[allstyles] = {};

        // phong smoothing cosine thresholds derived from the smoothing angles
        float smoothing_threshold = 0;
        float smoothing_threshold_2 = 0;

        // discarded light diagnostics printed at the end of the pass
        vec_t maxdiscardedlight = 0;
        vec3v maxdiscardedpos;

        // style overflow warnings are rate limited like the reference
        int stylewarningcount = 0;
        int stylewarningnext = 1;

        // opaque studio models loaded by studiocpp; the count gates the
        // opaque segment tests in the sample gather
        int num_studio_models = 0;

        // the single stage wide lock, standing in for the reference threadlock
        std::mutex lock;

        // visibility test chosen by the transfer method
        check_vis_bit_fn check_vis_bit = nullptr;

        // transfer statistics for the memory usage dump
        size_t total_transfer = 0;
        size_t transfer_index_bytes = 0;
        size_t transfer_data_bytes = 0;

        // custom shadow transparency arrays shared by the vismatrix methods
        std::vector<vec3v> trans_list;
        std::vector<trans_pair> trans_raw;
        std::vector<trans_pair> trans_sorted;
        std::vector<style_pair> style_list;

        // method storage: the dense triangular bit matrix or the sparse columns
        std::vector<byte> dense_vismatrix;
        std::vector<sparse_column> sparse_columns;
    };

    // ===== utilcpp =====

    math::winding winding_from_face(const rad_state &state, const format::dface_t &face);
    void decompress_vis(const rad_state &state, const byte *src, byte *dest, unsigned int dest_length);

    bool point_in_winding(const math::winding &w, const plane &pl, const vec3v &point, vec_t epsilon = 0.0);
    bool point_in_winding_noedge(const math::winding &w, const plane &pl, const vec3v &point, vec_t width);
    void snap_to_winding(const math::winding &w, const plane &pl, vec3v &point);
    vec_t snap_to_winding_noedge(const math::winding &w, const plane &pl, vec3v &point, vec_t width, vec_t maxmove);
    bool intersect_linesegment_plane(const plane &pl, const vec3v &p1, const vec3v &p2, vec3v &point);
    void plane_from_points(const vec3v &p1, const vec3v &p2, const vec3v &p3, plane &pl);
    bool test_segment_against_opaque_list(const rad_state &state, const vec3v &p1, const vec3v &p2,
                                          vec3v &scaleout, int &opaquestyleout);
    void snap_to_plane(const plane &pl, vec3v &point, vec_t offset);
    vec_t calc_sight_area(const rad_state &state, const vec3v &receiver_origin, const vec3v &receiver_normal,
                          const math::winding *emitter_winding, int skylevel,
                          vec_t lighting_power, vec_t lighting_scale);
    vec_t calc_sight_area_spotlight(const rad_state &state, const vec3v &receiver_origin, const vec3v &receiver_normal,
                                    const math::winding *emitter_winding, const vec3v &emitter_normal,
                                    vec_t emitter_stopdot, vec_t emitter_stopdot2, int skylevel,
                                    vec_t lighting_power, vec_t lighting_scale);
    void get_alternate_origin(const rad_state &state, const vec3v &pos, const vec3v &normal,
                              const patch *pt, vec3v &origin);

    format::dleaf_t *point_in_leaf(const rad_state &state, const vec3v &point);
    format::dleaf_t *point_in_leaf_worst(const rad_state &state, const vec3v &point);
    vec_t patch_plane_dist(const rad_state &state, const patch *pt);
    void load_planes(rad_state &state); // widens the plane lump and builds the backplanes
    const plane *plane_from_face(const rad_state &state, const format::dface_t *face);
    const plane *plane_from_face_number(const rad_state &state, unsigned facenum);
    void adjusted_plane_from_face_number(const rad_state &state, unsigned facenum, plane &out);
    void translate_plane(plane &pl, const vec3v &delta);
    format::dleaf_t *hunt_for_world(const rad_state &state, vec3v &point, const vec3v &plane_offset,
                                    const plane &pl, int hunt_size, vec_t hunt_scale, vec_t hunt_offset);

    void apply_matrix(const matrix &m, const vec3v &in, vec3v &out);
    void apply_matrix_on_plane(const matrix &m_inverse, const vec3v &in_normal, vec_t in_dist,
                               vec3v &out_normal, vec_t &out_dist);
    void multiply_matrix(const matrix &m_left, const matrix &m_right, matrix &m);
    matrix multiply_matrix(const matrix &m_left, const matrix &m_right);
    void matrix_for_scale(const vec3v &center, vec_t scale, matrix &m);
    matrix matrix_for_scale(const vec3v &center, vec_t scale);
    vec_t calc_matrix_sign(const matrix &m);
    void translate_world_to_tex(const rad_state &state, int facenum, matrix &m);
    bool invert_matrix(const matrix &m, matrix &m_inverse);

    // ===== positioncpp =====

    void find_face_positions(rad_state &state, int facenum);
    void free_position_maps(rad_state &state);
    bool find_nearest_position(const rad_state &state, int facenum, const math::winding *texwinding,
                               const plane &texplane, vec_t s, vec_t t, vec3v &pos,
                               vec_t *best_s, vec_t *best_t, vec_t *best_dist, bool *nudged);

    // ===== tracecpp =====

    void make_tnodes(rad_state &state);
    int test_line(const rad_state &state, const vec3v &start, const vec3v &stop, vec_t *skyhitout = nullptr);
    void create_opaque_nodes(rad_state &state);
    void delete_opaque_nodes(rad_state &state);
    int test_line_opaque(const rad_state &state, int modelnum, const vec3v &modelorigin,
                         const vec3v &start, const vec3v &stop);
    int count_opaque_faces(const rad_state &state, int modelnum);
    int test_point_opaque(const rad_state &state, int modelnum, const vec3v &modelorigin,
                          bool solid, const vec3v &point);

    // ===== patchescpp =====

    void read_light_file(rad_state &state, const char *filename);
    void read_info_tex_and_minlights(rad_state &state);
    bool is_unlit_texture(const rad_state &state, const char *name);
    void read_custom_chop_value(rad_state &state);
    void read_custom_smooth_value(rad_state &state);
    void read_translucent_textures(rad_state &state);
    void read_lighting_cone(rad_state &state);
    void load_opaque_entities(rad_state &state);
    void free_opaque_face_list(rad_state &state);
    void make_patches(rad_state &state);
    void sort_patches(rad_state &state);
    void free_patches(rad_state &state);
    format::entity *entity_for_model(rad_state &state, int modnum);

    // ===== light_budgetcpp =====

    // early lightmap atlas (allocblock) overflow check; returns true on
    // overflow after reporting the per texture breakdown
    bool check_alloc_block_budget(rad_state &state, int *alloc_block_pages);

    // ===== embed_lightmapcpp =====

    void delete_embedded_lightmaps(rad_state &state);
    void embed_lightmap_in_textures(rad_state &state);

    // ===== transparencycpp =====

    void get_transparency(const rad_state &state, const unsigned p1, const unsigned p2,
                          vec3v &trans, unsigned int &next_index);
    void add_transparency_to_raw_array(rad_state &state, const unsigned p1, const unsigned p2, const vec3v &trans);
    void create_final_transparency_arrays(rad_state &state, const char *print_name);
    void free_transparency_arrays(rad_state &state);
    void get_style(const rad_state &state, const unsigned p1, const unsigned p2,
                   int &style, unsigned int &next_index);
    void add_style_to_style_array(rad_state &state, const unsigned p1, const unsigned p2, const int style);
    void create_final_style_arrays(rad_state &state, const char *print_name);
    void free_style_arrays(rad_state &state);

    // ===== transferscpp =====

    int find_transfer_offset_patchnum(transfer_index *t_index, const patch *pt, const unsigned patchnum);
    void run_transfer_scales(rad_state &state); // make_scales / make_rgb_scales over all patches
    void dump_transfers_memory_usage(const rad_state &state);
    bool read_transfers(rad_state &state, const char *transferfile, long numpatches);
    void write_transfers(rad_state &state, const char *transferfile, long total_patches);

    // ===== vismatrixcpp / sparsecpp / nomatrixcpp =====

    void make_scales_vismatrix(rad_state &state);
    void make_scales_sparse_vismatrix(rad_state &state);
    void make_scales_no_vismatrix(rad_state &state);
    bool check_vis_bit_backwards(rad_state &state, unsigned receiver, unsigned emitter,
                                 const vec3v &backorigin, const vec3v &backnormal, vec3v &transparency_out);

    // ===== lerpcpp =====

    void create_triangulations(rad_state &state, int facenum);
    void get_triangulation_patches(const rad_state &state, int facenum, int *numpatches, const int **patches);
    void interpolate_sample_light(rad_state &state, const vec3v &position, int surface,
                                  int numstyles, const int *styles, vec3v *outs);
    void free_triangulations(rad_state &state);

    // ===== texturescpp =====

    void load_textures(rad_state &state);
    void try_open_wad_files(rad_state &state);
    void try_close_wad_files(rad_state &state);
    const char *texture_by_number(const rad_state &state, int texinfo);

    // ===== studiocpp =====

    void load_studio_models(rad_state &state);
    void free_studio_models(rad_state &state);
    bool test_segment_against_studio_list(const rad_state &state, const vec3v &p1, const vec3v &p2);

    // ===== lightmapcpp =====

    void get_phong_normal(const rad_state &state, int facenum, const vec3v &spot, vec3v &phongnormal);
    void pair_edges(rad_state &state);
    void create_direct_lights(rad_state &state);
    void delete_direct_lights(rad_state &state);
    void build_diffuse_normals(rad_state &state);
    void build_facelights(rad_state &state, int facenum);
    // test seam for cpu and gpu comparisons which forwards to the file local
    // gather_sample_light without touching its linkage
    void gather_sample_light_for_check(rad_state &state, const vec3v &pos, const byte *pvs,
                                       const vec3v &normal, vec3v *sample, byte *styles, int step,
                                       int miptex, int texlightgap_surfacenum);

    // gpu_gathercpp: the -gpu direct lighting path (stubs when the gpu
    // backend is not built) gpu_gather_run collects every gather call the
    // build_facelights phase will make, runs them on the gpu (surface
    // near branch pairs resolved on the cpu), and leaves the state in
    // consume mode; returns false (with a warning) to fall back to the cpu
    bool gpu_gather_run(rad_state &state);
    void gpu_gather_finish(rad_state &state);
    // records which face's build_facelights this thread is running; the
    // intercept groups work by the calling face (a sample's
    // texlightgap_surfacenum can be a neighboring face, so it cannot key the
    // per face item lists without racing)
    void gpu_gather_begin_face(int facenum);
    void gpu_gather_intercept(rad_state &state, const vec3v &pos, const byte *pvs,
                              const vec3v &normal, vec3v *sample, byte *styles, int step,
                              int miptex, int texlightgap_surfacenum);
    const vec3v *get_total_light(const patch *pt, int style);

    // ===== finalcpp =====

    void scale_direct_lights(rad_state &state);
    void create_facelight_dependency_list(rad_state &state);
    void add_patch_lights(rad_state &state, int facenum);
    void free_facelight_dependency_list(rad_state &state);
    void final_light_face(rad_state &state, int facenum);
    void precomp_lightmap_offsets(rad_state &state);
    void reduce_lightmap(rad_state &state);
    void mdl_light_hack(rad_state &state);

    // ===== entity key helpers =====

    inline float float_for_key(const format::entity &ent, const char *key)
    {
        return (float)atof(ent.value(key));
    }

    inline int int_for_key(const format::entity &ent, const char *key)
    {
        return atoi(ent.value(key));
    }

    // reads "x y z", missing components zero, doubles narrowed like the
    // reference getvectorforkey
    inline void vector_for_key(const format::entity &ent, const char *key, vec3v &vec)
    {
        double v1 = 0, v2 = 0, v3 = 0;
        sscanf(ent.value(key), "%lf %lf %lf", &v1, &v2, &v3);
        vec[0] = (vec_t)v1;
        vec[1] = (vec_t)v2;
        vec[2] = (vec_t)v3;
    }

    inline format::entity *find_target_entity(rad_state &state, const char *target)
    {
        for (size_t i = 0; i < state.entities.size(); i++)
        {
            if (!strcmp(state.entities[i].value("targetname"), target))
            {
                return &state.entities[i];
            }
        }
        return nullptr;
    }
}
