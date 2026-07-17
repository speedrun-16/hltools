#include "internal.h"

#include <cstdlib>
#include <cstring>

#include "../common/error.h"
#include "../common/filesystem.h"
#include "../common/log.h"
#include "../common/string_util.h"

namespace bsp
{
    bool token_file::open(const std::string &path)
    {
        std::vector<unsigned char> bytes;
        if (!fs::read_all(path, bytes))
            return false;
        text_.assign(bytes.begin(), bytes.end());
        pos_ = 0;
        return true;
    }

    namespace
    {
        bool is_space(char c)
        {
            return c == ' ' || c == '\t' || c == '\r' || c == '\n';
        }
    }

    bool token_file::next_int(int &out)
    {
        while (pos_ < text_.size() && is_space(text_[pos_]))
            pos_++;
        if (pos_ >= text_.size())
            return false;
        size_t start = pos_;
        while (pos_ < text_.size() && !is_space(text_[pos_]))
            pos_++;
        out = std::atoi(text_.c_str() + start);
        return true;
    }

    bool token_file::next_double(double &out)
    {
        while (pos_ < text_.size() && is_space(text_[pos_]))
            pos_++;
        if (pos_ >= text_.size())
            return false;
        size_t start = pos_;
        while (pos_ < text_.size() && !is_space(text_[pos_]))
            pos_++;
        out = std::atof(text_.c_str() + start);
        return true;
    }

    void open_input_files(bsp_state &state)
    {
        for (int i = 0; i < num_hulls; i++)
        {
            std::string name = state.base_path + ".p" + std::to_string(i);
            if (!state.polyfiles[i].open(name))
                err::fatal("can't open %s", name.c_str());
            name = state.base_path + ".b" + std::to_string(i);
            if (!state.brushfiles[i].open(name))
                err::fatal("can't open %s", name.c_str());
        }
    }

    void parse_hull_sizes(bsp_state &state, token_file &f)
    {
        for (int i = 0; i < num_hulls; i++)
        {
            double v[6];
            for (int j = 0; j < 6; j++)
            {
                if (!f.next_double(v[j]))
                    err::fatal("load hull size (line %i): scanf failure", i + 1);
            }
            // the reference reads these as floats before widening
            state.hull_size[i][0] = {(vec_t)(float)v[0], (vec_t)(float)v[1], (vec_t)(float)v[2]};
            state.hull_size[i][1] = {(vec_t)(float)v[3], (vec_t)(float)v[4], (vec_t)(float)v[5]};
        }
    }

    void load_hull_sizes(bsp_state &state)
    {
        std::string name = state.base_path + ".hsz";
        token_file f;
        if (!f.open(name))
        {
            logging::warn("couldn't open %s", name.c_str());
            return;
        }
        parse_hull_sizes(state, f);
    }

    void set_planes(bsp_state &state, const byte *bytes, size_t size)
    {
        size_t numplanes = state.map->planes.size();
        if (size != numplanes * sizeof(plane))
            err::fatal("invalid plane data");
        state.planes.resize(numplanes);
        std::memcpy(state.planes.data(), bytes, size);
    }

    void load_plane_file(bsp_state &state)
    {
        size_t numplanes = state.map->planes.size();
        std::string name = state.base_path + ".pln";
        std::vector<unsigned char> bytes;
        if (!fs::read_all(name, bytes))
        {
            // fall back to widening the float planes from the bsp
            logging::warn("couldn't open %s", name.c_str());
            state.planes.resize(numplanes);
            for (size_t i = 0; i < numplanes; i++)
            {
                const format::dplane_t &dp = state.map->planes[i];
                plane &mp = state.planes[i];
                mp.normal = {(vec_t)dp.normal[0], (vec_t)dp.normal[1], (vec_t)dp.normal[2]};
                mp.dist = (vec_t)dp.dist;
                mp.type = dp.type;
            }
            return;
        }
        set_planes(state, bytes.data(), bytes.size());
    }

    // the in memory csg -> bsp bridge: identical bytes to the sidecar files,
    // fed to the same tokenizer, so the %58f vertex round trip is preserved
    void set_input_data(bsp_state &state, bsp_input &input)
    {
        for (int i = 0; i < num_hulls; i++)
        {
            state.polyfiles[i].set_text(std::move(input.surfaces[i]));
            state.brushfiles[i].set_text(std::move(input.brushes[i]));
        }
        token_file sizes;
        sizes.set_text(std::move(input.hull_sizes));
        parse_hull_sizes(state, sizes);
        set_planes(state, input.planes.data(), input.planes.size());
    }

    namespace
    {
        surfchain *surflist_from_validfaces(bsp_state &state)
        {
            surfchain *sc = new surfchain();
            clear_bounds(sc->mins, sc->maxs);
            sc->surfaces = nullptr;

            // grab planes from both sides
            for (int i = 0; i < (int)state.map->planes.size(); i += 2)
            {
                if (!state.validfaces[(size_t)i] && !state.validfaces[(size_t)i + 1])
                    continue;
                surface *n = new surface();
                n->next = sc->surfaces;
                sc->surfaces = n;
                clear_bounds(n->mins, n->maxs);
                n->detail_level = -1;
                n->planenum = i;

                n->faces = nullptr;
                face *next;
                for (face *f = state.validfaces[(size_t)i]; f; f = next)
                {
                    next = f->next;
                    f->next = n->faces;
                    n->faces = f;
                    add_face_to_bounds(f, n->mins, n->maxs);
                    if (n->detail_level == -1 || f->detail_level < n->detail_level)
                        n->detail_level = f->detail_level;
                }
                for (face *f = state.validfaces[(size_t)i + 1]; f; f = next)
                {
                    next = f->next;
                    f->next = n->faces;
                    n->faces = f;
                    add_face_to_bounds(f, n->mins, n->maxs);
                    if (n->detail_level == -1 || f->detail_level < n->detail_level)
                        n->detail_level = f->detail_level;
                }

                add_point_to_bounds(n->mins, sc->mins, sc->maxs);
                add_point_to_bounds(n->maxs, sc->mins, sc->maxs);

                state.validfaces[(size_t)i] = nullptr;
                state.validfaces[(size_t)i + 1] = nullptr;
            }

            // merge all possible polygons
            merge_all(state, sc->surfaces);

            return sc;
        }
    }

    surfchain *read_surfs(bsp_state &state, int hull)
    {
        token_file &file = state.polyfiles[hull];
        if (state.validfaces.size() < state.map->planes.size() + 2)
            state.validfaces.assign(state.map->planes.size() + 2, nullptr);

        // read in the polygons
        while (true)
        {
            if (hull == 2 && state.options.nohull2)
                break;

            int detaillevel, planenum, texinfo, contents, numpoints;
            if (!file.next_int(detaillevel))
                return nullptr; // all models are done
            if (!file.next_int(planenum) || !file.next_int(texinfo)
                || !file.next_int(contents) || !file.next_int(numpoints))
            {
                err::fatal("read_surfs: scanf failure");
            }
            if (planenum == -1)
                break; // end of model (the marker line is -1 -1 -1 -1 -1)
            if (numpoints > max_read_points)
                err::fatal("read_surfs: %i > max_read_points", numpoints);
            if (planenum > (int)state.map->planes.size())
                err::fatal("read_surfs: %i > numplanes", planenum);
            if (texinfo > (int)state.map->texinfo.size())
                err::fatal("read_surfs: %i > numtexinfo", texinfo);
            if (detaillevel < 0)
                err::fatal("read_surfs: detaillevel %i < 0", detaillevel);

            if (str::iequals(texture_by_number(state, texinfo), "skip"))
            {
                for (int i = 0; i < numpoints; i++)
                {
                    double v[3];
                    if (!file.next_double(v[0]) || !file.next_double(v[1]) || !file.next_double(v[2]))
                        err::fatal("read_surfs (face_skip): read of points failed");
                }
                continue;
            }

            face *f = alloc_face();
            f->detail_level = detaillevel;
            f->planenum = planenum;
            f->texturenum = texinfo;
            f->contents = contents;
            f->numpoints = numpoints;
            f->next = state.validfaces[(size_t)planenum];
            state.validfaces[(size_t)planenum] = f;

            set_face_type(state, f);

            for (int i = 0; i < f->numpoints; i++)
            {
                double v[3];
                if (!file.next_double(v[0]) || !file.next_double(v[1]) || !file.next_double(v[2]))
                    err::fatal("read_surfs (face_normal): read of points failed");
                f->pts[i] = {(vec_t)v[0], (vec_t)v[1], (vec_t)v[2]};
            }
        }

        return surflist_from_validfaces(state);
    }

    brush *read_brushes(bsp_state &state, int hull)
    {
        token_file &file = state.brushfiles[hull];
        brush *brushes = nullptr;
        while (true)
        {
            if (hull == 2 && state.options.nohull2)
                break;
            int brushinfo;
            if (!file.next_int(brushinfo))
            {
                if (brushes == nullptr)
                    err::fatal("read_brushes: no more models");
                else
                    err::fatal("read_brushes: file end");
            }
            if (brushinfo == -1)
                break;
            brush *b = alloc_brush();
            b->next = brushes;
            brushes = b;
            side **psn = &b->sides;
            while (true)
            {
                int planenum, numpoints;
                if (!file.next_int(planenum) || !file.next_int(numpoints))
                    err::fatal("read_brushes: get side failed");
                if (planenum == -1)
                    break;
                side *s = alloc_side();
                s->plane_ = state.planes[(size_t)(planenum ^ 1)];
                std::vector<math::vec3v> points((size_t)numpoints);
                for (int x = 0; x < numpoints; x++)
                {
                    double v[3];
                    if (!file.next_double(v[0]) || !file.next_double(v[1]) || !file.next_double(v[2]))
                        err::fatal("read_brushes: get point failed");
                    points[(size_t)(numpoints - 1 - x)] = {(vec_t)v[0], (vec_t)v[1], (vec_t)v[2]};
                }
                s->winding_ = math::winding(std::move(points));
                s->next = nullptr;
                *psn = s;
                psn = &s->next;
            }
        }
        return brushes;
    }
}
