#include "face_extents.h"

#include <cmath>
#include <cstring>

#include "common/binary.h"
#include "common/limits.h"
#include "common/log.h"
#include "common/string_util.h"

namespace format
{
    int parse_texinfo_for_face(const map_data &map, const dface_t *f)
    {
        int texinfo = f->texinfo;
        int miptex = map.texinfo[(size_t)texinfo].miptex;
        if (miptex == -1)
            return texinfo;

        if (map.textures.size() < 4)
            return texinfo;
        binary::reader textures(map.textures);
        std::int32_t numtextures;
        if (!textures.i32(numtextures))
            return texinfo;
        if (miptex < 0 || miptex >= numtextures)
            return texinfo;
        std::int32_t offset;
        if (!textures.i32_at(4 + (size_t)miptex * 4, offset))
            return texinfo;
        if (offset < 0 || offset < 4 + numtextures * 4
            || (size_t)offset > map.textures.size()
            || map.textures.size() - (size_t)offset < sizeof(miptex_t))
            return texinfo;

        char name[16];
        str::copy(name, sizeof(name), (const char *)(map.textures.data() + offset));
        if (!(std::strlen(name) >= 6 && str::istarts_with(&name[1], "_rad")
              && '0' <= name[5] && name[5] <= '9'))
        {
            return texinfo;
        }

        int texinfo2 = std::atoi(&name[5]);
        if (texinfo2 < 0 || texinfo2 >= (int)map.texinfo.size())
        {
            logging::warn("invalid index of original texinfo: %d parsed from texture name '%s'",
                          texinfo2, name);
            return texinfo;
        }
        return texinfo2;
    }

    float calculate_point_vecs_product(const volatile float *point, const volatile float *vecs)
    {
        volatile double val;
        volatile double tmp;

        val = (double)point[0] * (double)vecs[0]; // one operation at a time
        tmp = (double)point[1] * (double)vecs[1];
        val = val + tmp;
        tmp = (double)point[2] * (double)vecs[2];
        val = val + tmp;
        val = val + (double)vecs[3];

        return (float)val;
    }

    void get_face_extents(const map_data &map, int facenum, int mins_out[2], int maxs_out[2])
    {
        const dface_t *f = &map.faces[(size_t)facenum];

        float mins[2], maxs[2];
        mins[0] = mins[1] = 999999;
        maxs[0] = maxs[1] = -99999;

        const texinfo_t *tex = &map.texinfo[(size_t)parse_texinfo_for_face(map, f)];

        for (int i = 0; i < f->numedges; i++)
        {
            int e = map.surfedges[(size_t)f->firstedge + i];
            const dvertex_t *v;
            if (e >= 0)
                v = &map.vertexes[map.edges[(size_t)e].v[0]];
            else
                v = &map.vertexes[map.edges[(size_t)-e].v[1]];
            for (int j = 0; j < 2; j++)
            {
                float val = calculate_point_vecs_product(v->point, tex->vecs[j]);
                if (val < mins[j])
                    mins[j] = val;
                if (val > maxs[j])
                    maxs[j] = val;
            }
        }

        for (int i = 0; i < 2; i++)
        {
            mins_out[i] = (int)std::floor(mins[i] / limits::texture_step);
            maxs_out[i] = (int)std::ceil(maxs[i] / limits::texture_step);
        }
    }
}
