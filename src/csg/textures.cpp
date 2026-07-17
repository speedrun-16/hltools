#include "textures.h"

#include <cmath>
#include "../common/error.h"
#include "../common/limits.h"
#include "../common/string_util.h"

namespace csg
{
    namespace
    {
        void copy_quark_texinfo(const brush_texture &texture, format::texinfo_t &tx)
        {
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 4; j++)
                    tx.vecs[i][j] = texture.quark.vecs[i][j];
            }
        }

        bool same_texinfo(const format::texinfo_t &a, const format::texinfo_t &b)
        {
            if (a.flags != b.flags)
                return false;
            for (int j = 0; j < 2; j++)
            {
                for (int k = 0; k < 4; k++)
                {
                    if (a.vecs[j][k] != b.vecs[j][k])
                        return false;
                }
            }
            return true;
        }

        bool is_special_texture(const char *name)
        {
            return name[0] == '*'
                || str::istarts_with(name, "sky")
                || str::istarts_with(name, "env_s")
                || str::istarts_with(name, "origin")
                || str::istarts_with(name, "null")
                || str::istarts_with(name, "aaatrigger");
        }
    }

    int texinfo_store::texinfo_for_brush_texture(const brush_plane &plane,
                                                 brush_texture &texture,
                                                 const math::vec3v &origin,
                                                 int map_file_version)
    {
        if (str::istarts_with(texture.name, "NULL"))
            return -1;

        format::texinfo_t tx = {};
        if (is_special_texture(texture.name))
            tx.flags |= tex_special;

        if (texture.txcommand)
        {
            copy_quark_texinfo(texture, tx);
            if (origin.x || origin.y || origin.z)
            {
                tx.vecs[0][3] = (float)(tx.vecs[0][3] + math::dot(origin, tx.vecs[0]));
                tx.vecs[1][3] = (float)(tx.vecs[1][3] + math::dot(origin, tx.vecs[1]));
            }
        }
        else
        {
            if (texture.valve.scale[0] == 0)
                texture.valve.scale[0] = 1;
            if (texture.valve.scale[1] == 0)
                texture.valve.scale[1] = 1;

            if (map_file_version < 220)
            {
                math::plane math_plane;
                math_plane.normal = plane.normal;
                math_plane.dist = plane.dist;
                math::vec3v vecs[2];
                texture_axis_from_plane(math_plane, vecs[0], vecs[1]);

                vec_t sinv, cosv;
                if (texture.valve.rotate == 0)
                {
                    sinv = 0;
                    cosv = 1;
                }
                else if (texture.valve.rotate == 90)
                {
                    sinv = 1;
                    cosv = 0;
                }
                else if (texture.valve.rotate == 180)
                {
                    sinv = 0;
                    cosv = -1;
                }
                else if (texture.valve.rotate == 270)
                {
                    sinv = -1;
                    cosv = 0;
                }
                else
                {
                    vec_t ang = texture.valve.rotate / 180 * math::pi;
                    sinv = std::sin(ang);
                    cosv = std::cos(ang);
                }

                int sv = vecs[0][0] ? 0 : vecs[0][1] ? 1 : 2;
                int tv = vecs[1][0] ? 0 : vecs[1][1] ? 1 : 2;

                for (int i = 0; i < 2; i++)
                {
                    vec_t ns = cosv * vecs[i][sv] - sinv * vecs[i][tv];
                    vec_t nt = sinv * vecs[i][sv] + cosv * vecs[i][tv];
                    vecs[i][sv] = ns;
                    vecs[i][tv] = nt;
                }

                for (int i = 0; i < 2; i++)
                {
                    for (int j = 0; j < 3; j++)
                        tx.vecs[i][j] = (float)(vecs[i][j] / texture.valve.scale[i]);
                }
            }
            else
            {
                math::vec3v scaled;
                vec_t scale = 1 / texture.valve.scale[0];
                math::scale(texture.valve.u_axis, scale, scaled);
                math::copy_to_float(scaled, tx.vecs[0]);

                scale = 1 / texture.valve.scale[1];
                math::scale(texture.valve.v_axis, scale, scaled);
                math::copy_to_float(scaled, tx.vecs[1]);
            }

            tx.vecs[0][3] = (float)(texture.valve.shift[0] + math::dot(origin, tx.vecs[0]));
            tx.vecs[1][3] = (float)(texture.valve.shift[1] + math::dot(origin, tx.vecs[1]));
        }

        for (int i = 0; i < (int)entries_.size(); i++)
        {
            const texinfo_entry &entry = entries_[(size_t)i];
            if (entry.texture_name != texture.name)
                continue;
            if (same_texinfo(entry.info, tx))
                return i;
        }

        if (entries_.size() >= limits::max_internal_map_texinfo)
            err::fatal("exceeded max_internal_map_texinfo");

        texinfo_entry entry;
        entry.info = tx;
        entry.texture_name = texture.name;
        entries_.push_back(entry);
        return (int)entries_.size() - 1;
    }

    void texinfo_store::set_miptex_index(int texinfo, int miptex)
    {
        if (texinfo < 0 || texinfo >= (int)entries_.size())
            err::fatal("invalid texinfo index");
        entries_[(size_t)texinfo].info.miptex = miptex;
    }
}
