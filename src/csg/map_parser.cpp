#include "map_parser.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <utility>

#include "../common/build_info.h"
#include "../common/error.h"
#include "../common/filesystem.h"
#include "../common/log.h"
#include "../common/string_util.h"

namespace csg
{
    namespace
    {
        constexpr double scale_correction = 1.0 / 128.0;
        constexpr size_t max_token = 4096;
        class token_reader
        {
        public:
            explicit token_reader(std::string text)
                : text_(std::move(text)) {}

            bool next(bool crossline)
            {
                if (pos_ >= text_.size())
                    return false;

                skip_space(crossline);
                if (pos_ >= text_.size())
                    return false;

                if (is_comment_start())
                {
                    skip_comment(crossline);
                    return next(crossline);
                }

                token_.clear();
                if (text_[pos_] == '"')
                {
                    pos_++;
                    while (pos_ < text_.size() && text_[pos_] != '"')
                    {
                        token_.push_back(text_[pos_++]);
                        if (token_.size() >= max_token)
                            err::fatal("token too large on line %d", line_);
                    }
                    if (pos_ < text_.size())
                        pos_++;
                    return true;
                }

                while (pos_ < text_.size())
                {
                    unsigned char c = (unsigned char)text_[pos_];
                    if ((c <= 32 && c < 128) || c == ';')
                        break;
                    token_.push_back(text_[pos_++]);
                    if (token_.size() >= max_token)
                        err::fatal("token too large on line %d", line_);
                }
                return true;
            }

            const std::string &token() const {
                return token_;
            }

            char tx_command() const {
                return tx_command_;
            }

            void clear_tx_command() {
                tx_command_ = 0;
            }

        private:
            void skip_space(bool crossline)
            {
                while (pos_ < text_.size())
                {
                    unsigned char c = (unsigned char)text_[pos_];
                    if (!(c <= 32 && c < 128))
                        return;
                    pos_++;
                    if (c == '\n')
                    {
                        if (!crossline)
                            err::fatal("line %d is incomplete", line_);
                        line_++;
                    }
                }
            }

            bool is_comment_start() const
            {
                if (pos_ >= text_.size())
                    return false;
                if (text_[pos_] == ';' || text_[pos_] == '#')
                    return true;
                return text_[pos_] == '/' && pos_ + 1 < text_.size() && text_[pos_ + 1] == '/';
            }

            void skip_comment(bool crossline)
            {
                if (!crossline)
                    err::fatal("line %d is incomplete", line_);

                if (text_[pos_] == '/')
                    pos_++;
                if (pos_ + 3 < text_.size() && text_[pos_ + 1] == 'T' && text_[pos_ + 2] == 'X')
                    tx_command_ = text_[pos_ + 3];

                while (pos_ < text_.size() && text_[pos_] != '\n')
                    pos_++;
                if (pos_ < text_.size() && text_[pos_] == '\n')
                {
                    pos_++;
                    line_++;
                }
            }

            std::string text_;
            std::string token_;
            size_t pos_ = 0;
            int line_ = 1;
            char tx_command_ = 0;
        };

        void require_token(token_reader &reader, bool crossline, const char *expected)
        {
            if (!reader.next(crossline))
                err::fatal("unexpected end of map while reading '%s'", expected);
            if (reader.token() != expected)
                err::fatal("expected '%s', got '%s'", expected, reader.token().c_str());
        }

        vec_t token_float(token_reader &reader)
        {
            if (!reader.next(false))
                err::fatal("unexpected end of map while reading number");
            return (vec_t)std::atof(reader.token().c_str());
        }

        std::string upper_token(std::string value)
        {
            for (char &c : value)
                c = (char)std::toupper((unsigned char)c);
            return value;
        }

        void set_pair(map_entity &entity, std::string key, std::string value)
        {
            for (auto it = entity.pairs.begin(); it != entity.pairs.end(); ++it)
            {
                if (it->first == key)
                {
                    if (value.empty())
                        entity.pairs.erase(it);
                    else
                        it->second = std::move(value);
                    return;
                }
            }
            if (value.empty())
                return;
            entity.pairs.emplace(entity.pairs.begin(), std::move(key), std::move(value));
        }

        void parse_legacy_texture(token_reader &reader, brush_side &side)
        {
            side.texture.valve.shift[0] = token_float(reader);
            side.texture.valve.shift[1] = token_float(reader);
            side.texture.valve.rotate = token_float(reader);
            side.texture.valve.scale[0] = token_float(reader);
            side.texture.valve.scale[1] = token_float(reader);
        }

        void parse_valve_texture(token_reader &reader, brush_side &side)
        {
            require_token(reader, false, "[");
            side.texture.valve.u_axis.x = token_float(reader);
            side.texture.valve.u_axis.y = token_float(reader);
            side.texture.valve.u_axis.z = token_float(reader);
            side.texture.valve.shift[0] = token_float(reader);
            require_token(reader, false, "]");

            require_token(reader, false, "[");
            side.texture.valve.v_axis.x = token_float(reader);
            side.texture.valve.v_axis.y = token_float(reader);
            side.texture.valve.v_axis.z = token_float(reader);
            side.texture.valve.shift[1] = token_float(reader);
            require_token(reader, false, "]");

            if (!reader.next(false))
                err::fatal("missing texture rotation");
            side.texture.valve.rotate = 0;
            side.texture.valve.scale[0] = token_float(reader);
            side.texture.valve.scale[1] = token_float(reader);
        }

        void translate_quark_axes(brush_side &side)
        {
            if (side.texture.txcommand != '1' && side.texture.txcommand != '2')
                return;

            math::vec3v texpt[2];
            int k = side.texture.txcommand - '0';
            for (int j = 0; j < 3; j++)
                texpt[1][j] = (side.planepts[k][j] - side.planepts[0][j]) * scale_correction;
            k = 3 - k;
            for (int j = 0; j < 3; j++)
                texpt[0][j] = (side.planepts[k][j] - side.planepts[0][j]) * scale_correction;

            float dot22 = (float)math::dot(texpt[0], texpt[0]);
            float dot23 = (float)math::dot(texpt[0], texpt[1]);
            float dot33 = (float)math::dot(texpt[1], texpt[1]);
            float mdet = dot22 * dot33 - dot23 * dot23;
            float aa, bb, dd;
            if (mdet < 1E-6f && mdet > -1E-6f)
            {
                aa = bb = dd = 0.0f;
            }
            else
            {
                mdet = 1.0f / mdet;
                aa = dot33 * mdet;
                bb = -dot23 * mdet;
                dd = dot22 * mdet;
            }

            for (int j = 0; j < 3; j++)
            {
                side.texture.quark.vecs[0][j] = (float)(aa * texpt[0][j] + bb * texpt[1][j]);
                side.texture.quark.vecs[1][j] = (float)-(bb * texpt[0][j] + dd * texpt[1][j]);
            }
            side.texture.quark.vecs[0][3] = (float)-math::dot(side.texture.quark.vecs[0], side.planepts[0]);
            side.texture.quark.vecs[1][3] = (float)-math::dot(side.texture.quark.vecs[1], side.planepts[0]);
        }

        void apply_tool_texture_flags(std::string &texture, map_brush &brush, brush_side &side)
        {
            const char *name = texture.c_str();
            if (str::istarts_with(name, "NOCLIP") || str::istarts_with(name, "NULLNOCLIP"))
            {
                texture = "NULL";
                brush.noclip = true;
                name = texture.c_str();
            }
            if (str::istarts_with(name, "BEVELBRUSH"))
            {
                texture = "NULL";
                brush.bevel = true;
                name = texture.c_str();
            }
            if (str::istarts_with(name, "BEVEL"))
            {
                texture = "NULL";
                side.bevel = true;
                name = texture.c_str();
            }
            if (str::istarts_with(name, "BEVELHINT"))
                side.bevel = true;
            if (str::istarts_with(name, "CLIP"))
            {
                brush.cliphull |= (1 << num_hulls);
                int h = texture.size() > 8 ? texture[8] - '0' : 0;
                if (str::istarts_with(name, "CLIPHULL") && 0 < h && h < num_hulls)
                    brush.cliphull |= (1 << h);
                if (str::istarts_with(name, "CLIPBEVEL"))
                    side.bevel = true;
                if (str::istarts_with(name, "CLIPBEVELBRUSH"))
                    brush.bevel = true;
                texture = "SKIP";
            }
        }

        void parse_brush(token_reader &reader, map_source &map, map_entity &entity)
        {
            map_brush brush;
            brush.original_entity_num = map.parsed_entities;
            brush.original_brush_num = map.parsed_brushes;
            brush.entity_num = (int)map.entities.size() - 1;
            brush.brush_num = entity.num_brushes;
            brush.first_side = (int)map.sides.size();
            brush.noclip = entity.int_value("zhlt_noclip") ? 1u : 0u;
            brush.detail_level = entity.int_value("zhlt_detaillevel");
            brush.chop_down = entity.int_value("zhlt_chopdown");
            brush.chop_up = entity.int_value("zhlt_chopup");
            brush.clipnode_detail_level = entity.int_value("zhlt_clipnodedetaillevel");
            brush.coplanar_priority = entity.int_value("zhlt_coplanarpriority");
            bool wrong_detail_setting = false;
            if (brush.detail_level < 0)
            {
                brush.detail_level = 0;
                wrong_detail_setting = true;
            }
            if (brush.chop_down < 0)
            {
                brush.chop_down = 0;
                wrong_detail_setting = true;
            }
            if (brush.chop_up < 0)
            {
                brush.chop_up = 0;
                wrong_detail_setting = true;
            }
            if (brush.clipnode_detail_level < 0)
            {
                brush.clipnode_detail_level = 0;
                wrong_detail_setting = true;
            }
            if (wrong_detail_setting)
            {
                logging::warn("entity %i, brush %i: incorrect settings for detail brush",
                              brush.original_entity_num, brush.original_brush_num);
            }
            for (int h = 0; h < num_hulls; h++)
            {
                char key[16];
                str::format(key, sizeof(key), "zhlt_hull%d", h);
                brush.hull_shapes[h] = entity.value(key);
            }

            bool ok = reader.next(true);
            while (ok)
            {
                reader.clear_tx_command();
                if (reader.token() == "}")
                    break;
                if (reader.token() != "(")
                    err::fatal("expected '(' while parsing brush side");

                brush_side side;
                for (int i = 0; i < 3; i++)
                {
                    if (i != 0)
                        require_token(reader, true, "(");
                    for (int j = 0; j < 3; j++)
                        side.planepts[i][j] = token_float(reader);
                    require_token(reader, false, ")");
                }

                if (!reader.next(false))
                    err::fatal("missing texture name");
                std::string texture = upper_token(reader.token());
                apply_tool_texture_flags(texture, brush, side);
                str::copy(side.texture.name, sizeof(side.texture.name), texture.c_str());

                if (map.map_file_version < 220)
                    parse_legacy_texture(reader, side);
                else
                    parse_valve_texture(reader, side);

                ok = reader.next(true);
                side.texture.txcommand = reader.tx_command();
                translate_quark_axes(side);
                map.sides.push_back(side);
                brush.num_sides++;
            }
            if (!ok)
                err::fatal("parse_brush: eof without closing brace");

            if (brush.cliphull != 0)
            {
                unsigned any_hull = 0;
                for (int h = 1; h < num_hulls; h++)
                    any_hull |= (1 << h);
                if ((brush.cliphull & any_hull) == 0)
                    brush.cliphull |= any_hull;
            }

            map.brushes.push_back(std::move(brush));
            entity.num_brushes++;
        }

        bool parse_entity(token_reader &reader, map_source &map)
        {
            if (!reader.next(true))
                return false;
            if (reader.token() != "{")
                err::fatal("expected '{' while parsing entity");

            map_entity entity;
            entity.first_brush = (int)map.brushes.size();
            map.entities.push_back(entity);
            map_entity &current = map.entities.back();

            map.parsed_brushes = 0;
            while (true)
            {
                if (!reader.next(true))
                    err::fatal("parse_entity: eof without closing brace");
                if (reader.token() == "}")
                    break;
                if (reader.token() == "{")
                {
                    parse_brush(reader, map, current);
                    map.parsed_brushes++;
                    continue;
                }

                std::string key = reader.token();
                if (!reader.next(false))
                    err::fatal("parse_entity: missing value for key '%s'", key.c_str());
                std::string value = reader.token();
                if (key == "mapversion")
                    map.map_file_version = std::atoi(value.c_str());
                set_pair(current, std::move(key), std::move(value));
            }

            const char *origin = current.value("origin");
            if (origin[0])
            {
                double x = 0, y = 0, z = 0;
                std::sscanf(origin, "%lf %lf %lf", &x, &y, &z);
                current.origin = math::vec3v{ (vec_t)x, (vec_t)y, (vec_t)z };
            }
            if (map.parsed_entities == 0)
                current.set_value("compiler", build_info::compiler().c_str());
            map.parsed_entities++;
            return true;
        }
    }

    const char *map_entity::value(const char *key) const
    {
        for (const auto &p : pairs)
        {
            if (p.first == key)
                return p.second.c_str();
        }
        return "";
    }

    int map_entity::int_value(const char *key) const
    {
        return std::atoi(value(key));
    }

    void map_entity::set_value(const char *key, const char *value)
    {
        if (value[0] == '\0')
        {
            for (auto it = pairs.begin(); it != pairs.end(); ++it)
            {
                if (it->first == key)
                {
                    pairs.erase(it);
                    return;
                }
            }
            return;
        }

        for (auto &p : pairs)
        {
            if (p.first == key)
            {
                p.second = value;
                return;
            }
        }
        pairs.emplace(pairs.begin(), key, value);
    }

    map_source load_map_file(const std::string &path)
    {
        std::vector<unsigned char> bytes;
        if (!fs::read_all(path, bytes))
            err::fatal("failed to load map '%s'", path.c_str());
        std::string text(bytes.begin(), bytes.end());
        token_reader reader(std::move(text));

        map_source map;
        while (parse_entity(reader, map))
        {
        }
        return map;
    }
}
