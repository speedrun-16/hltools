#include "internal.h"

#include <cstring>

#include "../common/binary.h"
#include "../common/error.h"
#include "../common/string_util.h"
#include "format/bsp/types.h"

namespace bsp
{
    face *alloc_face()
    {
        return new face();
    }

    void free_face(face *f)
    {
        delete f;
    }

    face *new_face_from_face(const face *in)
    {
        face *newf = alloc_face();
        newf->planenum = in->planenum;
        newf->texturenum = in->texturenum;
        newf->original = in->original;
        newf->contents = in->contents;
        newf->style = in->style;
        newf->detail_level = in->detail_level;
        return newf;
    }

    namespace
    {
        // like the reference splitfacetmp: the returned front/back may alias
        // the input when the face lies entirely on one side a coplanar detail
        // face follows its own normal; a coplanar structural face follows the
        // summed point distances
        void split_face_tmp(bsp_state &state, face *in, const plane *split,
                            face **front, face **back)
        {
            vec_t dists[max_edges_per_face + 1];
            int sides[max_edges_per_face + 1];
            int counts[3] = { 0, 0, 0 };
            vec_t dot;

            if (in->numpoints < 0)
                err::fatal("split_face: freed face");

            int i;
            for (i = 0; i < in->numpoints; i++)
            {
                dot = math::dot(in->pts[i], split->normal);
                dot -= split->dist;
                dists[i] = dot;
                if (dot > math::on_epsilon)
                    sides[i] = math::winding::side_front;
                else if (dot < -math::on_epsilon)
                    sides[i] = math::winding::side_back;
                else
                    sides[i] = math::winding::side_on;
                counts[sides[i]]++;
            }
            sides[i] = sides[0];
            dists[i] = dists[0];

            if (!counts[math::winding::side_front] && !counts[math::winding::side_back])
            {
                if (in->detail_level)
                {
                    // put the front face in the front node and the back face
                    // in the back node
                    const plane *faceplane = &state.planes[(size_t)in->planenum];
                    if (math::dot(faceplane->normal, split->normal) > math::normal_epsilon)
                    {
                        *front = in;
                        *back = nullptr;
                    }
                    else
                    {
                        *front = nullptr;
                        *back = in;
                    }
                }
                else
                {
                    // not func_detail: front face and back face need to pair
                    vec_t sum = 0.0;
                    for (i = 0; i < in->numpoints; i++)
                    {
                        dot = math::dot(in->pts[i], split->normal);
                        dot -= split->dist;
                        sum += dot;
                    }
                    if (sum > math::normal_epsilon)
                    {
                        *front = in;
                        *back = nullptr;
                    }
                    else
                    {
                        *front = nullptr;
                        *back = in;
                    }
                }
                return;
            }
            if (!counts[math::winding::side_front])
            {
                *front = nullptr;
                *back = in;
                return;
            }
            if (!counts[math::winding::side_back])
            {
                *front = in;
                *back = nullptr;
                return;
            }

            face *newf = new_face_from_face(in);
            face *new2 = new_face_from_face(in);
            *back = newf;
            *front = new2;

            for (i = 0; i < in->numpoints; i++)
            {
                if (newf->numpoints > max_edges_per_face || new2->numpoints > max_edges_per_face)
                    err::fatal("split_face: numpoints > max_edges_per_face");

                const math::vec3v &p1 = in->pts[i];

                if (sides[i] == math::winding::side_on)
                {
                    newf->pts[newf->numpoints] = p1;
                    newf->numpoints++;
                    new2->pts[new2->numpoints] = p1;
                    new2->numpoints++;
                    continue;
                }

                if (sides[i] == math::winding::side_front)
                {
                    new2->pts[new2->numpoints] = p1;
                    new2->numpoints++;
                }
                else
                {
                    newf->pts[newf->numpoints] = p1;
                    newf->numpoints++;
                }

                if (sides[i + 1] == math::winding::side_on || sides[i + 1] == sides[i])
                    continue;

                // generate a split point
                const math::vec3v &p2 = in->pts[(i + 1) % in->numpoints];
                dot = dists[i] / (dists[i] - dists[i + 1]);
                math::vec3v mid;
                for (int j = 0; j < 3; j++)
                {
                    if (split->normal[j] == 1)
                        mid[j] = split->dist;
                    else if (split->normal[j] == -1)
                        mid[j] = -split->dist;
                    else
                        mid[j] = p1[j] + dot * (p2[j] - p1[j]);
                }

                newf->pts[newf->numpoints] = mid;
                newf->numpoints++;
                new2->pts[new2->numpoints] = mid;
                new2->numpoints++;
            }

            if (newf->numpoints > max_edges_per_face || new2->numpoints > max_edges_per_face)
                err::fatal("split_face: numpoints > max_edges_per_face");

            for (face *half : { newf, new2 })
            {
                math::winding wd;
                for (int x = 0; x < half->numpoints; x++)
                    wd.add_point(half->pts[x]);
                wd.remove_colinear_points();
                half->numpoints = wd.size();
                for (int x = 0; x < half->numpoints; x++)
                    half->pts[x] = wd[x];
                if (half->numpoints == 0)
                {
                    if (half == newf)
                        *back = nullptr;
                    else
                        *front = nullptr;
                }
            }
        }
    }

    void split_face(bsp_state &state, face *in, const plane *split, face **front, face **back)
    {
        split_face_tmp(state, in, split, front, back);

        // free the original face now that it is represented by the fragments
        if (*front && *back)
            free_face(in);
    }

    const char *texture_by_number(const bsp_state &state, int texturenum)
    {
        if (texturenum == -1)
            return "";
        const format::map_data &map = *state.map;
        if (texturenum >= (int)map.texinfo.size() || map.textures.size() < 4)
            return "";
        int miptex = map.texinfo[(size_t)texturenum].miptex;
        binary::reader textures(map.textures);
        std::int32_t count;
        if (!textures.i32(count))
            return "";
        if (miptex < 0 || miptex >= count)
            return "";
        std::int32_t ofs;
        if (!textures.i32_at(4 + (size_t)miptex * 4, ofs)
            || ofs < 0 || (size_t)ofs > map.textures.size()
            || map.textures.size() - (size_t)ofs < sizeof(format::miptex_t))
            return "";
        return (const char *)(map.textures.data() + ofs); // miptex name is the first field
    }

    bool check_face_for_hint(const bsp_state &state, const face *f)
    {
        return str::istarts_with(texture_by_number(state, f->texturenum), "hint");
    }

    bool check_face_for_skip(const bsp_state &state, const face *f)
    {
        return str::istarts_with(texture_by_number(state, f->texturenum), "skip");
    }

    bool check_face_for_null(const bsp_state &state, const face *f)
    {
        if (f->contents == contents_sky)
        {
            const char *name = texture_by_number(state, f->texturenum);
            if (!str::istarts_with(name, "sky")) // for env_rain
                return true;
        }
        // null faces only strip when null texture stripping is enabled
        if (state.options.nulltex)
            return str::istarts_with(texture_by_number(state, f->texturenum), "null");
        return false;
    }

    bool check_face_for_env_sky(const bsp_state &state, const face *f)
    {
        return str::istarts_with(texture_by_number(state, f->texturenum), "env_sky");
    }

    bool check_face_for_discardable(const bsp_state &state, const face *f)
    {
        const char *name = texture_by_number(state, f->texturenum);
        return str::istarts_with(name, "SOLIDHINT") || str::istarts_with(name, "BEVELHINT");
    }

    face_style set_face_type(bsp_state &state, face *f)
    {
        if (check_face_for_hint(state, f))
            f->style = face_style::hint;
        else if (check_face_for_skip(state, f))
            f->style = face_style::skip;
        else if (check_face_for_null(state, f))
            f->style = face_style::null;
        else if (check_face_for_discardable(state, f))
            f->style = face_style::discardable;
        else if (check_face_for_env_sky(state, f))
            f->style = face_style::null;
        else
            f->style = face_style::normal;
        return f->style;
    }

    void add_point_to_bounds(const math::vec3v &v, math::vec3v &mins, math::vec3v &maxs)
    {
        for (int i = 0; i < 3; i++)
        {
            vec_t val = v[i];
            if (val < mins[i])
                mins[i] = val;
            if (val > maxs[i])
                maxs[i] = val;
        }
    }

    void add_face_to_bounds(const face *f, math::vec3v &mins, math::vec3v &maxs)
    {
        for (int i = 0; i < f->numpoints; i++)
            add_point_to_bounds(f->pts[i], mins, maxs);
    }

    void clear_bounds(math::vec3v &mins, math::vec3v &maxs)
    {
        mins.x = mins.y = mins.z = 99999;
        maxs.x = maxs.y = maxs.z = -99999;
    }
}
