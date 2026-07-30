#include "vis.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>

#include "../common/error.h"
#include "../common/filesystem.h"
#include "../common/limits.h"
#include "../common/log.h"
#include "../common/string_util.h"
#include "../common/threads.h"
#include "format/bsp/entity_lump.h"
#include "internal.h"

namespace vis
{
    namespace
    {
        constexpr double on_epsilon = 0.04;

        winding *new_winding(vis_state &state, int points)
        {
            if (points > max_points_on_winding)
                err::fatal("new_winding: %i points > max_points_on_winding", points);
            state.windings.emplace_back();
            return &state.windings.back();
        }

        void plane_from_winding(winding *w, plane *out)
        {
            math::vec3v v1, v2;
            math::subtract(w->points[2], w->points[1], v1);
            math::subtract(w->points[0], w->points[1], v2);
            math::cross(v2, v1, out->normal);
            math::normalize(out->normal);
            out->dist = math::dot(w->points[0], out->normal);
        }

        struct token_reader
        {
            explicit token_reader(std::string text) : text_(std::move(text)) {}

            bool next(std::string &out)
            {
                while (pos_ < text_.size() && is_separator(text_[pos_]))
                    pos_++;
                if (pos_ >= text_.size())
                    return false;

                size_t start = pos_;
                while (pos_ < text_.size() && !is_separator(text_[pos_]))
                    pos_++;
                out.assign(text_, start, pos_ - start);
                return true;
            }

            static bool is_separator(char c)
            {
                return c == ' ' || c == '(' || c == ')' || c == '\r'
                    || c == '\n' || c == '\t';
            }

            std::string text_;
            size_t pos_ = 0;
        };

        int parse_int_token(token_reader &reader, const char *what)
        {
            std::string token;
            if (!reader.next(token))
                err::fatal("load_portals: damaged or invalid .prt file");
            int value = 0;
            if (std::sscanf(token.c_str(), "%i", &value) != 1)
                err::fatal("load_portals: failed to read %s", what);
            return value;
        }

        unsigned parse_uint_token(token_reader &reader, const char *what)
        {
            std::string token;
            if (!reader.next(token))
                err::fatal("load_portals: damaged or invalid .prt file");
            unsigned value = 0;
            if (std::sscanf(token.c_str(), "%u", &value) != 1)
                err::fatal("load_portals: failed to read %s", what);
            return value;
        }

        double parse_double_token(token_reader &reader)
        {
            std::string token;
            if (!reader.next(token))
                err::fatal("load_portals: damaged or invalid .prt file");
            double value = 0.0;
            if (std::sscanf(token.c_str(), "%lf", &value) != 1)
                err::fatal("load_portals: reading portal point");
            return value;
        }

        // walks the bsp nodes to the leaf containing a point, then converts
        // from bsp leaf numbering to vis leaf numbering (-nodenum - 2)
        int vis_leafnum_for_point(const format::map_data &map, const float point[3])
        {
            int nodenum = 0;
            while (nodenum >= 0)
            {
                const format::dnode_t &node = map.nodes[(size_t)nodenum];
                const format::dplane_t &node_plane = map.planes[(size_t)node.planenum];
                float dist = point[0] * node_plane.normal[0]
                    + point[1] * node_plane.normal[1]
                    + point[2] * node_plane.normal[2]
                    - node_plane.dist;
                nodenum = dist >= 0.0 ? node.children[0] : node.children[1];
            }
            return -nodenum - 2;
        }

        void parse_origin_key(const format::entity &entity, float point[3])
        {
            double v[3] = {};
            std::sscanf(entity.value("origin"), "%lf %lf %lf", &v[0], &v[1], &v[2]);
            for (int i = 0; i < 3; i++)
                point[i] = (float)v[i];
        }

        // collects info_overview_point and info_portal entities, resolving
        // their origins to vis leaf numbers against the loaded bsp
        void collect_leaf_entities(vis_state &state)
        {
            std::vector<format::entity> entities = format::parse_entities(state.map->entities);
            for (int i = 0; i < (int)entities.size(); i++)
            {
                const char *classname = entities[(size_t)i].value("classname");

                if (std::strcmp(classname, "info_overview_point") == 0)
                {
                    float point[3];
                    parse_origin_key(entities[(size_t)i], point);
                    overview_point overview;
                    overview.visleafnum = vis_leafnum_for_point(*state.map, point);
                    overview.reverse = std::atoi(entities[(size_t)i].value("reverse"));
                    state.overview_points.push_back(overview);
                }
                else if (std::strcmp(classname, "info_portal") == 0)
                {
                    float point[3];
                    parse_origin_key(entities[(size_t)i], point);
                    room_portal room;
                    room.visleafnum = vis_leafnum_for_point(*state.map, point);
                    int neighbor = std::atoi(entities[(size_t)i].value("neighbor"));
                    room.neighbor = neighbor < 0 ? 0 : (neighbor > max_room_neighbor ? max_room_neighbor : neighbor);

                    const char *target = entities[(size_t)i].value("target");
                    if (target[0] == '\0')
                        continue; // like the reference, a room without a target is dropped

                    // the last info_leaf whose targetname matches wins
                    bool has_target = false;
                    for (const format::entity &candidate : entities)
                    {
                        if (std::strcmp(candidate.value("classname"), "info_leaf") == 0
                            && std::strcmp(candidate.value("targetname"), target) == 0)
                        {
                            float target_point[3];
                            parse_origin_key(candidate, target_point);
                            room.target_visleafnum = vis_leafnum_for_point(*state.map, target_point);
                            has_target = true;
                        }
                    }
                    if (!has_target)
                        logging::warn("entity %d (info_portal) does not have a target leaf", i);
                    state.rooms.push_back(room);
                }
            }
        }

        void load_portals(vis_state &state, const std::string &path)
        {
            std::vector<unsigned char> bytes;
            if (!fs::read_all(path, bytes))
                err::fatal("portal file '%s' does not exist, cannot vis the map", path.c_str());
            std::string image(bytes.begin(), bytes.end());
            token_reader reader(std::move(image));

            state.portalleafs = parse_uint_token(reader, "number of leafs");
            state.numportals = parse_int_token(reader, "number of portals");
            logging::file("  %d portal leafs, %d portals\n", state.portalleafs, state.numportals);

            state.bitbytes = ((state.portalleafs + 63) & ~63) >> 3;
            state.bitlongs = state.bitbytes / sizeof(std::uint32_t);

            state.portals.assign(2 * state.numportals, portal{});
            state.leafs.assign(state.portalleafs, leaf{});
            state.leafinfos.assign(state.portalleafs, leaf_info{});
            state.leafcounts.assign(state.portalleafs, 0);
            state.leafstarts.assign(state.portalleafs, 0);
            state.windings.reserve(2 * state.numportals);

            state.originalvismapsize = state.portalleafs * ((state.portalleafs + 7) / 8);
            state.visibility.assign(limits::max_map_visibility, 0);
            state.vismap_p = state.visibility.data();

            if (state.portalleafs > limits::max_map_leafs)
            {
                err::fatal("too many portalleafs (%u > %d)", state.portalleafs,
                           limits::max_map_leafs);
            }

            state.leafcount_all = 0;
            for (unsigned i = 0; i < state.portalleafs; i++)
            {
                state.leafcounts[i] = parse_int_token(reader, "leaf mapping");
                state.leafstarts[i] = state.leafcount_all;
                state.leafcount_all += state.leafcounts[i];
            }

            if (state.map->models.empty() || state.leafcount_all != state.map->models[0].visleafs)
            {
                err::fatal("corrupted leaf mapping (%d != %d)", state.leafcount_all,
                           state.map->models.empty() ? -1 : state.map->models[0].visleafs);
            }

            // fold the overview and room entities into per leaf flags now that
            // the bsp leaf to vis leaf mapping is known
            for (unsigned i = 0; i < state.portalleafs; i++)
            {
                for (const overview_point &overview : state.overview_points)
                {
                    int d = overview.visleafnum - state.leafstarts[i];
                    if (0 <= d && d < state.leafcounts[i])
                    {
                        if (overview.reverse)
                            state.leafinfos[i].isskyboxpoint = true;
                        else
                            state.leafinfos[i].isoverviewpoint = true;
                    }
                }

                for (const room_portal &room : state.rooms)
                {
                    int d1 = room.visleafnum - state.leafstarts[i];
                    if (0 <= d1 && d1 < state.leafcounts[i])
                    {
                        for (unsigned k = 0; k < state.portalleafs; k++)
                        {
                            int d2 = room.target_visleafnum - state.leafstarts[k];
                            if (0 <= d2 && d2 < state.leafcounts[k])
                            {
                                state.leafinfos[i].additional_leaves.push_back((int)k);
                                state.leafinfos[i].neighbor = room.neighbor;
                            }
                        }
                    }
                }
            }

            portal *p = state.portals.data();
            for (int i = 0; i < state.numportals; i++)
            {
                int numpoints = parse_int_token(reader, "portal point count");
                int leafnums[2];
                leafnums[0] = parse_int_token(reader, "portal leaf");
                leafnums[1] = parse_int_token(reader, "portal leaf");

                if (numpoints > max_points_on_winding)
                    err::fatal("load_portals: portal %i has too many points", i);
                if ((unsigned)leafnums[0] > state.portalleafs || (unsigned)leafnums[1] > state.portalleafs)
                    err::fatal("load_portals: reading portal %i", i);

                winding *w = p->winding_ = new_winding(state, numpoints);
                w->original = true;
                w->numpoints = numpoints;

                for (int j = 0; j < numpoints; j++)
                {
                    double v[3];
                    v[0] = parse_double_token(reader);
                    v[1] = parse_double_token(reader);
                    v[2] = parse_double_token(reader);
                    for (int k = 0; k < 3; k++)
                        w->points[j][k] = (float)v[k];
                }

                plane portal_plane;
                plane_from_winding(w, &portal_plane);

                leaf *l = &state.leafs[leafnums[0]];
                if (l->numportals >= max_portals_on_leaf)
                    err::fatal("max_portals_on_leaf");
                l->portals[l->numportals] = p;
                l->numportals++;

                p->winding_ = w;
                math::scale(portal_plane.normal, -1.0f, p->plane_.normal);
                p->plane_.dist = -portal_plane.dist;
                p->leaf = leafnums[1];
                p++;

                l = &state.leafs[leafnums[1]];
                if (l->numportals >= max_portals_on_leaf)
                    err::fatal("max_portals_on_leaf");
                l->portals[l->numportals] = p;
                l->numportals++;

                p->winding_ = new_winding(state, w->numpoints);
                p->winding_->numpoints = w->numpoints;
                for (int j = 0; j < w->numpoints; j++)
                    math::copy(w->points[w->numpoints - 1 - j], p->winding_->points[j]);

                p->plane_ = portal_plane;
                p->leaf = leafnums[0];
                p++;
            }
        }

        portal *get_next_portal(vis_state &state)
        {
            std::lock_guard<std::mutex> lock(state.portal_mutex);

            int min = 99999;
            portal *best = nullptr;
            for (int j = 0; j < state.numportals * 2; j++)
            {
                portal *tp = state.portals.data() + j;
                if ((int)tp->nummightsee < min && tp->status == portal_status::none)
                {
                    min = tp->nummightsee;
                    best = tp;
                }
            }

            if (best)
                best->status = portal_status::working;
            return best;
        }


        void leaf_flow_neighbor_add_leaf(vis_state &state, int current, int add, int neighbor,
                                         std::unordered_map<int, bool> &exclude)
        {
            byte *outbuffer = state.uncompressed.data() + current * state.bitbytes;
            outbuffer[add >> 3] |= (1 << (add & 7));
            exclude[current] = true;

            if (neighbor == 0)
                return;

            leaf *leaf_ptr = &state.leafs[current];
            for (unsigned i = 0; i < leaf_ptr->numportals; i++)
            {
                portal *p = leaf_ptr->portals[i];
                if (exclude[p->leaf])
                    continue;
                leaf_flow_neighbor_add_leaf(state, p->leaf, add, neighbor - 1, exclude);
            }
        }

        void leaf_flow(vis_state &state, int leafnum)
        {
            byte compressed[limits::max_map_leafs / 8] = {};
            byte *outbuffer = state.uncompressed.data() + leafnum * state.bitbytes;
            leaf *leaf_ptr = &state.leafs[leafnum];
            int tmp = 0;

            const unsigned offset = leafnum >> 3;
            const unsigned bit = (1 << (leafnum & 7));

            for (unsigned i = 0; i < leaf_ptr->numportals; i++)
            {
                portal *p = leaf_ptr->portals[i];
                if (p->status != portal_status::done)
                    err::fatal("portal not done (leaf %d)", leafnum);

                byte *dst = outbuffer;
                byte *src = p->visbits;
                for (unsigned j = 0; j < state.bitbytes; j++, dst++, src++)
                    *dst |= *src;

                if ((tmp == 0) && (outbuffer[offset] & bit))
                {
                    tmp = 1;
                    logging::warn("leaf portals saw into leaf");
                }
            }

            outbuffer[offset] |= bit;

            if (state.leafinfos[leafnum].isoverviewpoint)
            {
                for (unsigned i = 0; i < state.portalleafs; i++)
                    outbuffer[i >> 3] |= (1 << (i & 7));
            }
            for (unsigned i = 0; i < state.portalleafs; i++)
            {
                if (state.leafinfos[i].isskyboxpoint)
                    outbuffer[i >> 3] |= (1 << (i & 7));
            }

            int numvis = 0;
            for (unsigned i = 0; i < state.portalleafs; i++)
            {
                if (outbuffer[i >> 3] & (1 << (i & 7)))
                    numvis++;
            }
            state.totalvis += numvis;

            byte buffer2[limits::max_map_leafs / 8] = {};
            int diskbytes = (state.leafcount_all + 7) >> 3;
            std::memset(buffer2, 0, diskbytes);
            for (unsigned i = 0; i < state.portalleafs; i++)
            {
                for (int j = 0; j < state.leafcounts[i]; j++)
                {
                    int srcofs = i >> 3;
                    int srcbit = 1 << (i & 7);
                    int dstofs = (state.leafstarts[i] + j) >> 3;
                    int dstbit = 1 << ((state.leafstarts[i] + j) & 7);
                    if (outbuffer[srcofs] & srcbit)
                        buffer2[dstofs] |= dstbit;
                }
            }

            int compressed_size = compress_vis(buffer2, diskbytes, compressed, sizeof(compressed));
            byte *dest = state.vismap_p;
            state.vismap_p += compressed_size;

            if (state.vismap_p > state.visibility.data() + state.visibility.size())
                err::fatal("vismap expansion overflow");

            for (int j = 0; j < state.leafcounts[leafnum]; j++)
                state.map->leafs[state.leafstarts[leafnum] + j + 1].visofs =
                    (int)(dest - state.visibility.data());

            std::memcpy(dest, compressed, compressed_size);
        }

        void calc_portal_vis(vis_state &state)
        {
            if (state.options.fast)
            {
                for (int i = 0; i < state.numportals * 2; i++)
                {
                    state.portals[i].visbits = state.portals[i].mightsee;
                    state.portals[i].status = portal_status::done;
                }
                return;
            }

            // one portal per work item so the bar advances as portals finish
            // (the old leaf_thread loop drained the whole queue inside the
            // first few items, so the bar sat at 0% until the phase ended)
            // scheduling is unchanged: every item still pulls the cheapest
            // pending portal through get_next_portal
            threads::run_phase("computing visibility",
                               state.options.full ? "computing full visibility"
                                                  : "computing portal visibility",
                               state.numportals * 2, [&](int) {
                if (portal *p = get_next_portal(state))
                    portal_flow(state, p);
            });
        }

        void calc_vis(vis_state &state)
        {
            threads::run_phase("computing visibility", "computing base visibility",
                               state.numportals * 2, [&](int index) {
                base_portal_vis(state, index);
            });

            calc_portal_vis(state);

            for (unsigned i = 0; i < state.portalleafs; i++)
            {
                if (!state.leafinfos[i].additional_leaves.empty())
                {
                    for (int leaf_index : state.leafinfos[i].additional_leaves)
                    {
                        std::unordered_map<int, bool> exclude;
                        leaf_flow_neighbor_add_leaf(state, i, leaf_index, state.leafinfos[i].neighbor, exclude);
                    }
                }
            }

            for (unsigned i = 0; i < state.portalleafs; i++)
                leaf_flow(state, i);

            if (state.options.maxdistance)
                max_dist_vis(state, 0);
        }

        void fix_prt(const std::string &portalfile)
        {
            std::ifstream input(portalfile);
            if (!input)
                return;

            std::vector<std::string> lines;
            std::string line;
            while (std::getline(input, line))
            {
                if (!line.empty())
                    lines.push_back(line);
            }
            input.close();

            auto it = std::find_if(lines.begin(), lines.end(), [](const std::string &s) {
                return s.find('(') != std::string::npos;
            });
            if (lines.size() < 2 || lines[1] == "0" || it == lines.end() || it == lines.begin())
                return;

            lines.erase(lines.begin() + 2, it);

            std::ofstream output(portalfile);
            if (!output)
                return;
            for (const std::string &out : lines)
                output << out << "\n";
        }
    }

    int compress_vis(const byte *src, unsigned src_length, byte *dest, unsigned dest_length)
    {
        byte *dest_p = dest;
        unsigned current_length = 0;

        for (unsigned j = 0; j < src_length; j++)
        {
            current_length++;
            if (current_length > dest_length)
                err::fatal("compress_vis overflow");

            *dest_p = src[j];
            dest_p++;

            if (src[j])
                continue;

            unsigned char rep = 1;
            for (j++; j < src_length; j++)
            {
                if (src[j] || rep == 255)
                    break;
                rep++;
            }
            current_length++;
            if (current_length > dest_length)
                err::fatal("compress_vis overflow");

            *dest_p = rep;
            dest_p++;
            j--;
        }

        return (int)(dest_p - dest);
    }

    void run_vis(format::map_data &map, const vis_options &options)
    {
        vis_state state;
        state.map = &map;
        state.options = options;

        if (state.options.portal_path.empty())
            err::fatal("hltools vis requires a portal file path");

        collect_leaf_entities(state);
        load_portals(state, state.options.portal_path);
        state.uncompressed.assign(state.portalleafs * state.bitbytes, 0);

        calc_vis(state);

        size_t used = (size_t)(state.vismap_p - state.visibility.data());
        state.visibility.resize(used);
        map.visibility = state.visibility;

        char b1[32], b2[32];
        logging::file("  visdata %s (compressed from %s)\n",
                      str::human_bytes((long long)used, b1, sizeof(b1)),
                      str::human_bytes((long long)state.originalvismapsize, b2, sizeof(b2)));
        logging::file("  average leafs visible: %d\n",
                      state.portalleafs ? state.totalvis / (int)state.portalleafs : 0);

        if (!state.options.nofixprt)
            fix_prt(state.options.portal_path);
    }
}
