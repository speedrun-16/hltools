#pragma once

#include <string>
#include <vector>

#include "../math/bounding_box.h"
#include "../math/plane.h"
#include "../math/winding.h"
#include "map_parser.h"

namespace csg
{
    class texinfo_store;
    struct hull_shape_library;

    void texture_axis_from_plane(const math::plane &plane, math::vec3v &xv, math::vec3v &yv);

    content texture_content(const char *name);
    const char *content_to_string(content type);
    content check_brush_contents(const map_source &map, const map_brush &brush);

    struct brush_plane
    {
        math::vec3v normal;
        math::vec3v origin;
        vec_t dist = 0;
        math::plane_type type = math::plane_type::x;
    };

    struct brush_face
    {
        int planenum = -1;
        int texinfo = -1;
        bool bevel = false;
        content contents = content::empty;
        content back_contents = content::empty;
        math::winding winding;
    };

    struct brush_hull
    {
        std::vector<brush_face> faces;
        math::bounding_box bounds;
    };

    struct hull_brush_face
    {
        math::vec3v normal;
        math::vec3v point;
        std::vector<math::vec3v> vertexes;
    };

    struct hull_brush_edge
    {
        math::vec3v normals[2];
        math::vec3v point;
        math::vec3v vertexes[2];
        math::vec3v delta;
    };

    struct hull_brush
    {
        std::vector<hull_brush_face> faces;
        std::vector<hull_brush_edge> edges;
        std::vector<math::vec3v> vertexes;
    };

    struct hull_shape
    {
        std::string id;
        bool disabled = true;
        std::vector<hull_brush> brushes;
    };

    struct hull_shape_library
    {
        hull_shape default_hulls[num_hulls];
        std::vector<hull_shape> shapes;
    };

    enum class clip_type
    {
        smallest,
        normalized,
        simple,
        precise,
        legacy,
    };

    struct brush_build_options
    {
        bool noclip = false;
        bool only_entities = false;
        bool nullify_trigger = false;
        vec_t world_extent = 65536;
        vec_t global_scale = -1;
        clip_type clip = clip_type::simple;
        const hull_shape_library *hull_shapes = nullptr;
        math::vec3v hull_size[num_hulls][2] = {
            {{0, 0, 0}, {0, 0, 0}},
            {{-16, -16, -36}, {16, 16, 36}},
            {{-32, -32, -32}, {32, 32, 32}},
            {{-16, -16, -18}, {16, 16, 18}},
        };
    };

    struct built_brush
    {
        content contents = content::empty;
        int original_entity_num = 0;
        int original_brush_num = 0;
        int detail_level = 0;
        int chop_down = 0;
        int chop_up = 0;
        int clipnode_detail_level = 0;
        int coplanar_priority = 0;
        brush_hull hulls[num_hulls];
        bool skipped = false;
    };

    struct built_entity
    {
        int entity_num = -1;
        std::vector<built_brush> brushes;
    };

    class plane_store
    {
    public:
        int find_int_plane(const math::vec3v &normal, const math::vec3v &origin);
        int plane_from_points(const math::vec3v &p0, const math::vec3v &p1, const math::vec3v &p2);

        const std::vector<brush_plane> &planes() const {
            return planes_;
        }

    private:
        std::vector<brush_plane> planes_;
    };

    std::vector<brush_face> make_brush_planes(map_source &map,
                                              const map_brush &brush,
                                              plane_store &planes,
                                              const math::vec3v &origin,
                                              texinfo_store *texinfos = nullptr,
                                              bool only_entities = false);

    brush_hull make_hull_faces(const map_brush &brush,
                               const plane_store &planes,
                               std::vector<brush_face> faces,
                               vec_t world_extent = 65536);

    std::vector<brush_face> make_bounding_clip_planes(const brush_hull &hull0,
                                                      plane_store &planes,
                                                      int hull_num,
                                                      const brush_build_options &options = {});

    void add_hull_shape(map_source &map, int entity_num, hull_shape_library &library);

    void set_origin_key(map_entity &entity, const math::vec3v &origin);
    void set_bounds_key(map_entity &entity, const math::vec3v &mins, const math::vec3v &maxs);

    void apply_entity_transform(map_source &map,
                                int entity_num,
                                const brush_build_options &options);

    built_brush create_brush(map_source &map,
                             const map_brush &brush,
                             plane_store &planes,
                             const math::vec3v &origin,
                             texinfo_store *texinfos = nullptr,
                             const brush_build_options &options = {});

    built_entity build_entity_brushes(map_source &map,
                                      int entity_num,
                                      plane_store &planes,
                                      texinfo_store *texinfos = nullptr,
                                      const brush_build_options &options = {});
}
