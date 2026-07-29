#pragma once

#include <cstdio>
#include <string>
#include <vector>

#include "../common/types.h"
#include "format/bsp/data.h"
#include "format/bsp/entity_lump.h"
#include "../math/bounding_box.h"
#include "../math/vector.h"
#include "../math/winding.h"
#include "bsp.h"

// the bsp stage keeps the reference's intrusive linked lists and ownership:
// faces, surfaces, sides, brushes, nodes and portals are heap nodes threaded
// through next pointers, split in place and freed by the consumer, because the
// tree build depends on the exact list orders those operations produce

namespace bsp
{
    constexpr double bogus_range = 144000; // hlbsp's own; plane windings use 80000
    constexpr double plane_winding_range = 80000;
    constexpr int side_space = 24;
    constexpr int max_edges_per_face = 48; // maxedges
    constexpr int max_read_points = 28;    // maxpoints: cap on faces read from csg
    constexpr int planenum_leaf = -1;
    constexpr double bounds_expansion = 1.0;
    constexpr double brink_floor_threshold = 0.7;

    // engine contents values as they arrive in the csg surface files
    constexpr int contents_empty = -1;
    constexpr int contents_solid = -2;
    constexpr int contents_water = -3;
    constexpr int contents_slime = -4;
    constexpr int contents_lava = -5;
    constexpr int contents_sky = -6;
    constexpr int contents_origin = -7;
    constexpr int contents_current_0 = -9;
    constexpr int contents_current_90 = -10;
    constexpr int contents_current_180 = -11;
    constexpr int contents_current_270 = -12;
    constexpr int contents_current_up = -13;
    constexpr int contents_current_down = -14;
    constexpr int contents_translucent = -15;
    constexpr int contents_hint = -16;
    constexpr int contents_null = -17;
    constexpr int contents_toempty = -32;

    constexpr double engine_entity_range = 4096.0;
    // half the geometric range csg accepts (its world_extent defaults to 65536).
    // this is what bounds tree subdivision: it is a different quantity from the
    // networked entity range above, which only limits where entity origins may
    // sit, and using that smaller number here left every node beyond +/-4096
    // unsubdivided on a large map.
    constexpr double tree_extent = 32768.0;
    constexpr int tex_special = 1; // texinfo flag: no lightmap or subdivision
    constexpr int last_axial = 2;  // plane types x/y/z are axial

    enum class face_style
    {
        normal = 0,
        hint,
        skip,
        null,
        discardable, // contents must not differ between front and back
    };

    // the csg pln record: double precision planes hlbsp works with throughout
    struct plane
    {
        math::vec3v normal;
        math::vec3v origin;
        vec_t dist = 0;
        int type = 0;
        int pad = 0;
    };

    static_assert(sizeof(plane) == 64, "must match the csg .pln record");

    struct face
    {
        face *next = nullptr;
        int planenum = -1;
        int texturenum = 0;
        int contents = 0;      // contents in front of the face
        int detail_level = 0;  // defined by hlcsg
        int *output_edges = nullptr; // used by write_draw_nodes

        face *original = nullptr; // face on node
        int output_number = 0;    // only valid for original faces after write surfaces
        int numpoints = 0;
        face_style style = face_style::normal;
        int referenced = 0; // only valid for original faces

        math::vec3v pts[max_edges_per_face];
    };

    struct node;

    struct surface
    {
        surface *next = nullptr;
        int planenum = -1;
        math::vec3v mins, maxs;
        node *onnode = nullptr; // already used as a splitting node
        face *faces = nullptr;  // links to all faces on either side of the surf
        int detail_level = 0;   // minimum detail level of its faces
    };

    struct surfchain
    {
        math::vec3v mins, maxs;
        surface *surfaces = nullptr;
    };

    struct side
    {
        side *next = nullptr;
        plane plane_; // facing inside (reversed when loading the brush file)
        math::winding winding_; // (also reversed)
    };

    struct brush
    {
        brush *next = nullptr;
        side *sides = nullptr;
    };

    struct portal;

    struct node
    {
        surface *surfaces = nullptr;
        brush *detailbrushes = nullptr;
        brush *boundsbrush = nullptr;
        // all leafs and nodes have loose bounds; mins/maxs are only valid for
        // nondetail leafs and nodes
        math::vec3v loosemins, loosemaxs;

        bool isdetail = false;        // is under a diskleaf
        bool isportalleaf = false;    // not detail and children are detail
        bool iscontentsdetail = false; // inside a detail brush
        math::vec3v mins, maxs;       // bounding volume of portals

        // decision nodes
        int planenum = planenum_leaf;
        node *children[2] = {};
        face *faces = nullptr; // decision nodes only, list for both sides

        // leafs
        int contents = 0;          // leaf nodes (0 for decision nodes)
        face **markfaces = nullptr; // leaf nodes only, points to node faces
        portal *portals = nullptr;
        int visleafnum = 0; // -1 = solid
        int valid = 0;      // for flood filling
        int occupied = 0;   // light number in leaf for outside filling
        int empty = 0;
    };

    struct portal
    {
        plane plane_;
        node *onnode = nullptr; // nullptr = outside box
        node *nodes[2] = {};    // [0] = front side of plane
        portal *next[2] = {};
        math::winding *winding_ = nullptr;
    };

    struct leaf_content_conflict
    {
        int model = 0;
        int hull = 0;
        math::vec3v mins, maxs;
        std::vector<int> contents;
        int selected_content = contents_empty;
    };

    // whitespace delimited number reader over a csg sidecar text file,
    // matching the reference's fscanf loops
    class token_file
    {
    public:
        bool open(const std::string &path);
        void set_text(std::string text)
        {
            text_ = std::move(text);
            pos_ = 0;
        }
        bool next_int(int &out);
        bool next_double(double &out);

    private:
        std::string text_;
        size_t pos_ = 0;
    };

    struct bsp_state
    {
        format::map_data *map = nullptr;
        bsp_options options;
        std::string base_path;

        std::vector<plane> planes; // double planes from the csg pln file
        math::vec3v hull_size[num_hulls][2] = {
            {{0, 0, 0}, {0, 0, 0}},
            {{-16, -16, -36}, {16, 16, 36}},
            {{-32, -32, -32}, {32, 32, 32}},
            {{-16, -16, -18}, {16, 16, 18}},
        };

        token_file polyfiles[num_hulls];
        token_file brushfiles[num_hulls];

        std::vector<face *> validfaces; // per planenum face lists during reading
        int hullnum = 0;
        int nummodels = 0; // models emitted so far, for warnings
        bool leaked = false;

        // entity lookup for warnings (parsed once by the driver)
        std::vector<format::entity> entities;

        // leaf content conflicts collected for one consolidated report
        std::vector<leaf_content_conflict> leaf_content_conflicts;

        node outside_node; // portals outside the world face this

        // portal file bookkeeping
        std::string portfilename;
        int num_visleafs = 0;
        int num_visportals = 0;

        // solidbsp progress
        bool report_progress = false;
        int num_processed = 0;
        int num_reported = 0;
    };

    const format::entity *entity_for_model(const bsp_state &state, int modnum);

    // facecpp
    face *alloc_face();
    void free_face(face *f);
    face *new_face_from_face(const face *in);
    void split_face(bsp_state &state, face *in, const plane *split, face **front, face **back);
    const char *texture_by_number(const bsp_state &state, int texturenum);
    bool check_face_for_hint(const bsp_state &state, const face *f);
    bool check_face_for_skip(const bsp_state &state, const face *f);
    bool check_face_for_null(const bsp_state &state, const face *f);
    bool check_face_for_discardable(const bsp_state &state, const face *f);
    bool check_face_for_env_sky(const bsp_state &state, const face *f);
    face_style set_face_type(bsp_state &state, face *f);
    void add_point_to_bounds(const math::vec3v &v, math::vec3v &mins, math::vec3v &maxs);
    void add_face_to_bounds(const face *f, math::vec3v &mins, math::vec3v &maxs);
    void clear_bounds(math::vec3v &mins, math::vec3v &maxs);

    // brushcpp
    side *alloc_side();
    void free_side(side *s);
    side *new_side_from_side(const side *s);
    brush *alloc_brush();
    void free_brush(brush *b);
    brush *new_brush_from_brush(const brush *b);
    void clip_brush(brush **b, const plane *split);
    void split_brush(brush *in, const plane *split, brush **front, brush **back);
    brush *brush_from_box(const math::vec3v &mins, const math::vec3v &maxs);
    void calc_brush_bounds(const brush *b, math::vec3v &mins, math::vec3v &maxs);
    node *alloc_node();

    // mergecpp
    void merge_plane_faces(bsp_state &state, surface *plane_surf);
    void merge_all(bsp_state &state, surface *surfhead);

    // solid_bspcpp
    void classify_leaf_contents(bsp_state &state, surface *planelist, node *leafnode);
    void print_leaf_content_conflicts(const bsp_state &state);
    node *solid_bsp(bsp_state &state, const surfchain *surfhead,
                    brush *detailbrushes, bool report_progress);

    // portalscpp
    void add_portal_to_nodes(portal *p, node *front, node *back);
    void remove_portal_from_node(portal *p, node *l);
    void make_headnode_portals(bsp_state &state, node *headnode,
                               const math::vec3v &mins, const math::vec3v &maxs);
    void write_portal_file(bsp_state &state, node *headnode);
    void free_portals(node *n);

    // surfacescpp
    void subdivide_face(bsp_state &state, face *f, face **prevptr);
    void make_face_edges(bsp_state &state);
    int get_edge(bsp_state &state, const math::vec3v &p1, const math::vec3v &p2, face *f);

    // write_bspcpp
    void begin_bsp_file(bsp_state &state);
    void write_clip_nodes(bsp_state &state, node *nodes);
    void write_draw_nodes(bsp_state &state, node *headnode);
    void optimize_face_order(bsp_state &state);
    void finish_bsp_file(bsp_state &state);

    // outsidecpp
    node *fill_outside(bsp_state &state, node *n, bool leakfile, int hullnum);
    void fill_inside(bsp_state &state, node *n);
    void print_leak_summary();

    // tjunccpp
    void tjunc(bsp_state &state, node *headnode);

    // brinkcpp
    void fix_all_brinks(bsp_state &state);

    // inputscpp
    void open_input_files(bsp_state &state);
    void set_input_data(bsp_state &state, bsp_input &input);
    void load_hull_sizes(bsp_state &state);
    void load_plane_file(bsp_state &state);
    void parse_hull_sizes(bsp_state &state, token_file &file);
    void set_planes(bsp_state &state, const byte *bytes, size_t size);
    surfchain *read_surfs(bsp_state &state, int hull);
    brush *read_brushes(bsp_state &state, int hull);
}
