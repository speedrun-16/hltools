#include "document.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "math/vector.h"

namespace format
{
    namespace
    {
        using vec3 = math::vec3<double>;

        std::string number(double value)
        {
            if (std::fabs(value) < 0.00000005)
                value = 0;
            double rounded = std::round(value);
            if (std::fabs(value - rounded) < 0.000001)
                value = rounded;
            char buffer[64];
            std::snprintf(buffer, sizeof(buffer), "%.12g", value);
            return buffer;
        }

        void append_point(std::string &text, const vec3 &p)
        {
            text += "( ";
            text += number(p.x); text += ' ';
            text += number(p.y); text += ' ';
            text += number(p.z); text += " )";
        }

        void append_axis(std::string &text, const double axis[4])
        {
            text += " [ ";
            text += number(axis[0]); text += ' ';
            text += number(axis[1]); text += ' ';
            text += number(axis[2]); text += ' ';
            text += number(axis[3]); text += " ]";
        }

        std::string escape_value(const std::string &value)
        {
            std::string out = value;
            std::replace(out.begin(), out.end(), '"', '\'');
            std::replace(out.begin(), out.end(), '\r', ' ');
            std::replace(out.begin(), out.end(), '\n', ' ');
            return out;
        }

        void append_entity_pairs(std::string &text, const entity &keyvalues)
        {
            for (const entity::pair &pair : keyvalues.pairs())
            {
                text += '"'; text += escape_value(pair.first);
                text += "\" \""; text += escape_value(pair.second);
                text += "\"\n";
            }
        }

        void append_brush(std::string &text, const map_brush &brush)
        {
            text += "{\n";
            for (const map_side &side : brush.sides)
            {
                append_point(text, side.points[0]); text += ' ';
                append_point(text, side.points[1]); text += ' ';
                append_point(text, side.points[2]); text += ' ';
                text += side.texture;
                append_axis(text, side.axes[0]);
                append_axis(text, side.axes[1]);
                text += ' '; text += number(side.rotation);
                text += ' '; text += number(side.u_scale);
                text += ' '; text += number(side.v_scale);
                text += '\n';
            }
            text += "}\n";
        }
    }

    map_side map_side::from_plane(const vec3 &normal, double dist)
    {
        // a fixed separation gives the compiler well separated points and an
        // exact normal; deriving everything from (normal, dist) alone makes
        // coplanar sides serialize identically
        constexpr double spread = 128.0;

        int axis = 0;
        double best = std::fabs(normal.x);
        if (std::fabs(normal.y) > best) { best = std::fabs(normal.y); axis = 1; }
        if (std::fabs(normal.z) > best) { axis = 2; }

        vec3 up{0, 0, 0};
        if (axis == 2)
            up.x = 1;
        else
            up.z = 1;
        double d = math::dot(up, normal);
        vec3 u{up.x - d * normal.x, up.y - d * normal.y, up.z - d * normal.z};
        math::normalize(u);
        vec3 v = math::cross(normal, u); // u x v == normal: points face outward

        vec3 origin{normal.x * dist, normal.y * dist, normal.z * dist};
        map_side side;
        side.points[0] = {origin.x + u.x * spread, origin.y + u.y * spread,
                          origin.z + u.z * spread};
        side.points[1] = origin;
        side.points[2] = {origin.x + v.x * spread, origin.y + v.y * spread,
                          origin.z + v.z * spread};
        return side;
    }

    map_side map_side::from_points(const vec3 &a, const vec3 &b, const vec3 &c)
    {
        map_side side;
        side.points[0] = a;
        side.points[1] = b;
        side.points[2] = c;
        return side;
    }

    map_brush map_brush::aabb(const vec3 &lo, const vec3 &hi, const std::string &texture)
    {
        const struct { vec3 normal; double dist; } faces[6] = {
            {{1, 0, 0}, hi.x},  {{-1, 0, 0}, -lo.x},
            {{0, 1, 0}, hi.y},  {{0, -1, 0}, -lo.y},
            {{0, 0, 1}, hi.z},  {{0, 0, -1}, -lo.z},
        };
        map_brush brush;
        for (const auto &face : faces)
        {
            map_side side = map_side::from_plane(face.normal, face.dist);
            side.texture = texture;
            brush.sides.push_back(std::move(side));
        }
        return brush;
    }

    map_entity &map_entity::set(const char *key, const char *value)
    {
        if (keyvalues.has(key))
            keyvalues.set(key, value);
        else
            keyvalues.append(key, value);
        return *this;
    }

    map_entity &map_entity::set(const char *key, const std::string &value)
    {
        return set(key, value.c_str());
    }

    map_entity &map_entity::set(const char *key, int value)
    {
        return set(key, std::to_string(value).c_str());
    }

    map_entity &map_document::worldspawn()
    {
        if (entities.empty())
        {
            entities.emplace_back();
            entities[0].set("classname", "worldspawn");
        }
        return entities[0];
    }

    map_entity &map_document::add_entity(const char *classname, map_keyvalues kv)
    {
        worldspawn(); // slot 0 stays reserved
        entities.emplace_back();
        map_entity &entity = entities.back();
        entity.set("classname", classname);
        for (const auto &pair : kv)
            entity.set(pair.first, pair.second);
        return entity;
    }

    map_group map_document::add_group(const std::string &name, map_group parent)
    {
        map_group group{++next_group_id_};
        map_entity &entity = add_entity("func_group");
        entity.set("_tb_type", "_tb_group");
        entity.set("_tb_name", name);
        entity.set("_tb_id", group.id);
        if (parent)
            entity.set("_tb_group", parent.id);
        groups_.emplace_back(group.id, entities.size() - 1);
        return group;
    }

    void map_document::add_brush(map_brush brush, map_group group)
    {
        if (group)
        {
            for (const auto &entry : groups_)
            {
                if (entry.first == group.id)
                {
                    entities[entry.second].brushes.push_back(std::move(brush));
                    return;
                }
            }
        }
        worldspawn().brushes.push_back(std::move(brush));
    }

    map_entity &map_document::add_brush(map_brush brush, const char *classname,
                                        map_keyvalues kv, map_group group)
    {
        std::string signature = classname;
        for (const auto &pair : kv)
        {
            signature += '\x1f';
            signature += pair.first;
            signature += '\x1f';
            signature += pair.second;
        }
        signature += '\x1f';
        signature += std::to_string(group.id);

        for (const auto &merged : merged_)
        {
            if (merged.signature == signature)
            {
                map_entity &entity = entities[merged.entity_index];
                entity.brushes.push_back(std::move(brush));
                return entity;
            }
        }

        map_entity &entity = add_entity(classname, kv);
        if (group)
            assign_group(entity, group);
        merged_.push_back({std::move(signature), entities.size() - 1});
        entity.brushes.push_back(std::move(brush));
        return entity;
    }

    void map_document::assign_group(map_entity &entity, map_group group)
    {
        if (group)
            entity.set("_tb_group", group.id);
    }

    std::size_t map_document::brush_count() const
    {
        std::size_t count = 0;
        for (const map_entity &entity : entities)
            count += entity.brushes.size();
        return count;
    }

    std::size_t map_document::side_count() const
    {
        std::size_t count = 0;
        for (const map_entity &entity : entities)
            for (const map_brush &brush : entity.brushes)
                count += brush.sides.size();
        return count;
    }

    std::string map_document::write() const
    {
        std::string text;
        for (const map_entity &entity : entities)
        {
            if (entity.keyvalues.empty() && entity.brushes.empty())
                continue;
            text += "{\n";
            append_entity_pairs(text, entity.keyvalues);
            for (const map_brush &brush : entity.brushes)
                append_brush(text, brush);
            text += "}\n";
        }
        return text;
    }
}
