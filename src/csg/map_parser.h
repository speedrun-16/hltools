#pragma once

#include <string>
#include <utility>
#include <vector>

#include "../math/vector.h"

namespace csg
{
    constexpr int num_hulls = 4;
    constexpr int texture_name_size = 32;

    // engine content types, values from the reference contents_t stored on the
    // brush at parse time (like the reference b->contents) because the post parse
    // fixups rename sides afterwards and a re check would classify differently
    enum class content : int
    {
        empty = -1,
        solid = -2,
        water = -3,
        slime = -4,
        lava = -5,
        sky = -6,
        origin = -7,
        current_0 = -9,
        current_90 = -10,
        current_180 = -11,
        current_270 = -12,
        current_up = -13,
        current_down = -14,
        translucent = -15,
        hint = -16,
        null = -17,
        bounding_box = -19,
        to_empty = -32,
    };

    struct valve_texture_axes
    {
        math::vec3v u_axis;
        math::vec3v v_axis;
        vec_t shift[2] = {};
        vec_t rotate = 0;
        vec_t scale[2] = {};
    };

    struct quark_texture_axes
    {
        float vecs[2][4] = {};
    };

    struct brush_texture
    {
        char txcommand = 0;
        valve_texture_axes valve;
        quark_texture_axes quark;
        char name[texture_name_size] = {};
    };

    struct brush_side
    {
        brush_texture texture;
        bool bevel = false;
        math::vec3v planepts[3];
    };

    struct map_brush
    {
        int original_entity_num = 0;
        int original_brush_num = 0;
        int entity_num = 0;
        int brush_num = 0;
        int first_side = 0;
        int num_sides = 0;
        unsigned noclip = 0;
        unsigned cliphull = 0;
        bool bevel = false;
        int detail_level = 0;
        int chop_down = 0;
        int chop_up = 0;
        int clipnode_detail_level = 0;
        int coplanar_priority = 0;
        content contents = content::empty;
        std::string hull_shapes[num_hulls];
    };

    struct map_entity
    {
        using pair = std::pair<std::string, std::string>;

        int first_brush = 0;
        int num_brushes = 0;
        math::vec3v origin;
        std::vector<pair> pairs;

        const char *value(const char *key) const;
        int int_value(const char *key) const;
        void set_value(const char *key, const char *value);
    };

    struct map_source
    {
        int map_file_version = 0;
        int parsed_entities = 0;
        int parsed_brushes = 0;
        std::vector<map_entity> entities;
        std::vector<map_brush> brushes;
        std::vector<brush_side> sides;
    };

    map_source load_map_file(const std::string &path);
}
