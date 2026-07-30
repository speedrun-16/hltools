#pragma once

#include <cstddef>
#include <initializer_list>
#include <string>
#include <utility>
#include <vector>

#include "format/bsp/entity_lump.h"
#include "math/vector.h"

// an in-memory valve 220 map: the editable-source counterpart of map_data (the
// compiled bsp). producers build entities and brushes programmatically and
// serialize once with write(); nothing outside this module knows the .map text
// syntax. trenchbroom groups are supported through their func_group convention
// (_tb_type/_tb_name/_tb_id keyvalues), which csg merges into the world, so
// grouping never affects a compile.

namespace format
{
    using map_keyvalues = std::initializer_list<std::pair<const char *, std::string>>;

    // one brush side: a plane given by three points (facing outward) plus its
    // valve 220 texture mapping
    struct map_side
    {
        math::vec3<double> points[3] = {};
        std::string texture = "NULL";
        double axes[2][4] = {{1, 0, 0, 0}, {0, -1, 0, 0}}; // u/v: xyz + offset
        double rotation = 0;
        double u_scale = 1;
        double v_scale = 1;

        // canonical points derived only from the plane equation: every side on
        // the same plane serializes identically, so the compiler collapses them
        // into one splitting plane
        static map_side from_plane(const math::vec3<double> &normal, double dist);
        static map_side from_points(const math::vec3<double> &a,
                                    const math::vec3<double> &b,
                                    const math::vec3<double> &c);
    };

    struct map_brush
    {
        std::vector<map_side> sides;

        // a six sided axis aligned box, textured uniformly
        static map_brush aabb(const math::vec3<double> &lo,
                              const math::vec3<double> &hi, const std::string &texture);
    };

    struct map_entity
    {
        entity keyvalues;
        std::vector<map_brush> brushes;

        map_entity &set(const char *key, const char *value);
        map_entity &set(const char *key, const std::string &value);
        map_entity &set(const char *key, int value);
    };

    // handle to a trenchbroom group; id 0 means "no group"
    struct map_group
    {
        int id = 0;
        explicit operator bool() const { return id != 0; }
    };

    struct map_document
    {
        std::vector<map_entity> entities; // [0] is worldspawn

        // creates worldspawn on first use
        map_entity &worldspawn();
        map_entity &add_entity(const char *classname, map_keyvalues kv = {});

        // trenchbroom group backed by a func_group entity; nested groups
        // reference their parent through _tb_group
        map_group add_group(const std::string &name, map_group parent = {});

        // a plain world brush, or a member of a group's func_group entity
        void add_brush(map_brush brush, map_group group = {});

        // routes a brush into an entity of the given classname and keyvalues.
        // brushes with an identical (classname, keyvalues, group) signature
        // share one entity; the first call creates it.
        map_entity &add_brush(map_brush brush, const char *classname,
                              map_keyvalues kv = {}, map_group group = {});

        // tags an existing entity as a member of a group
        void assign_group(map_entity &entity, map_group group);

        std::size_t brush_count() const;
        std::size_t side_count() const;

        std::string write() const;

    private:
        struct merged_entity
        {
            std::string signature;
            std::size_t entity_index;
        };
        std::vector<merged_entity> merged_;
        std::vector<std::pair<int, std::size_t>> groups_; // id -> entity index
        int next_group_id_ = 0;
    };

    // a top level entity located in existing map source; only its depth one
    // key/value pairs are parsed and brushes remain untouched in the source text
    struct map_source_entity
    {
        std::size_t begin = 0;
        std::size_t end = 0;
        entity keyvalues;
    };

    // source preserving map editing helpers allow compiler metadata to change
    // without parsing and rewriting brushes, comments or layout
    std::vector<map_source_entity> parse_map_source_entities(
        const std::string &text);
    void erase_map_entities(
        std::string &text,
        std::initializer_list<const char *> classnames);
    void append_map_entity(std::string &text, const entity &keyvalues);
}
