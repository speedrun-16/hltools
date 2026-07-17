#include "internal.h"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstring>
#include <map>
#include <numeric>
#include <tuple>
#include <utility>

#include "../common/binary.h"
#include "../common/error.h"
#include "../common/limits.h"
#include "../common/string_util.h"
#include "../common/log.h"
#include "format/bsp/face_extents.h"
#include "format/bsp/types.h"

// disk format emission: draw nodes, leafs, faces, clipnodes (with merging),
// the used plane and used texinfo compaction, unused texdata removal, and the
// ext extent file hlrad reads

namespace bsp
{
    namespace
    {
        constexpr int max_map_worldfaces = 32768;

        using plane_map = std::map<int, int>;
        using texinfo_map = std::map<int, int>;
        using clipnode_key = std::pair<int, std::pair<int, int>>;
        using clipnode_map = std::map<clipnode_key, int>;

        // writer bookkeeping shared by all models of one run, like the
        // reference file statics; begin_bsp_file resets it
        struct writer_state
        {
            plane_map planes_seen;
            std::vector<plane> mapped_planes;
            texinfo_map texinfos_seen;
            std::vector<format::texinfo_t> mapped_texinfos;
            int count_mergedclipnodes = 0;
        };

        writer_state g_writer;

        clipnode_key make_key(const format::dclipnode_t &c)
        {
            return std::make_pair((int)c.planenum,
                                  std::make_pair((int)c.children[0], (int)c.children[1]));
        }

        // hook for plane optimization: maps a used plane to its output slot
        int write_plane(bsp_state &state, int planenum)
        {
            planenum = planenum & (~1);

            if (state.options.noopt)
                return planenum;

            plane_map::iterator item = g_writer.planes_seen.find(planenum);
            if (item != g_writer.planes_seen.end())
                return item->second;

            if ((int)g_writer.mapped_planes.size() >= limits::max_map_planes)
                err::fatal("exceeded max_map_planes");
            g_writer.mapped_planes.push_back(state.planes[(size_t)planenum]);
            int out = (int)g_writer.mapped_planes.size() - 1;
            g_writer.planes_seen.emplace(planenum, out);
            return out;
        }

        int write_texinfo(bsp_state &state, int texinfo)
        {
            if (texinfo < 0 || texinfo >= (int)state.map->texinfo.size())
                err::fatal("bad texinfo number %d", texinfo);

            if (state.options.noopt)
                return texinfo;

            texinfo_map::iterator it = g_writer.texinfos_seen.find(texinfo);
            if (it != g_writer.texinfos_seen.end())
                return it->second;

            int c = (int)g_writer.mapped_texinfos.size();
            g_writer.mapped_texinfos.push_back(state.map->texinfo[(size_t)texinfo]);
            g_writer.texinfos_seen.emplace(texinfo, c);
            return c;
        }

        int write_clip_nodes_r(bsp_state &state, node *n, const node *portalleaf,
                               clipnode_map *outputmap)
        {
            if (n->isportalleaf)
            {
                if (n->contents == contents_solid)
                {
                    delete n;
                    return contents_solid;
                }
                portalleaf = n;
            }
            if (n->planenum == -1)
            {
                int num;
                if (n->iscontentsdetail)
                    num = contents_solid;
                else
                    num = portalleaf->contents;
                delete[] n->markfaces;
                delete n;
                return num;
            }

            // this clipnode lands in the lump only if it can't be merged
            format::dclipnode_t cn;
            int c = (int)state.map->clipnodes.size();
            state.map->clipnodes.resize((size_t)c + 1);
            if (n->planenum & 1)
                err::fatal("write_clip_nodes_r: odd planenum");
            cn.planenum = write_plane(state, n->planenum);
            for (int i = 0; i < 2; i++)
                cn.children[i] = (short)write_clip_nodes_r(state, n->children[i], portalleaf, outputmap);

            clipnode_map::iterator output = outputmap->find(make_key(cn));
            if (state.options.noclipnodemerge || output == outputmap->end())
            {
                if (c >= limits::max_map_clipnodes)
                    err::fatal("exceeded max_map_clipnodes");
                state.map->clipnodes[(size_t)c] = cn;
                (*outputmap)[make_key(cn)] = c;
            }
            else
            {
                g_writer.count_mergedclipnodes++;
                if ((int)state.map->clipnodes.size() != c + 1)
                    err::fatal("merge clipnodes: internal error");
                state.map->clipnodes.resize((size_t)c);
                c = output->second; // use the existing clipnode
            }

            delete n;
            return c;
        }

        int write_draw_leaf(bsp_state &state, node *n, const node *portalleaf)
        {
            int leafnum = (int)state.map->leafs.size();

            // emit a leaf
            if (leafnum >= limits::max_map_leafs)
                err::fatal("exceeded max_map_leafs");
            state.map->leafs.resize((size_t)leafnum + 1);
            format::dleaf_t *leaf_p = &state.map->leafs[(size_t)leafnum];

            leaf_p->contents = portalleaf->contents;

            // write bounding box info
            math::vec3v mins, maxs;
            if (n->isdetail)
            {
                // intersect the loose bounds with the parent portalleaf's
                // strict bounds
                for (int k = 0; k < 3; k++)
                {
                    mins[k] = portalleaf->mins[k] > n->loosemins[k] ? portalleaf->mins[k] : n->loosemins[k];
                    maxs[k] = portalleaf->maxs[k] < n->loosemaxs[k] ? portalleaf->maxs[k] : n->loosemaxs[k];
                }
            }
            else
            {
                mins = n->mins;
                maxs = n->maxs;
            }
            for (int k = 0; k < 3; k++)
            {
                int lo = (int)mins[k] < 32767 ? (int)mins[k] : 32767;
                int hi = (int)maxs[k] < 32767 ? (int)maxs[k] : 32767;
                leaf_p->mins[k] = (short)(lo > -32767 ? lo : -32767);
                leaf_p->maxs[k] = (short)(hi > -32767 ? hi : -32767);
            }

            leaf_p->visofs = -1; // no vis info yet

            // write the marksurfaces
            leaf_p->firstmarksurface = (unsigned short)state.map->marksurfaces.size();

            if (n->markfaces == nullptr)
                err::fatal("write_draw_leaf: empty solid");

            for (face **fp = n->markfaces; *fp; fp++)
            {
                // emit a marksurface
                face *f = *fp;
                do
                {
                    // fix face 0 being seen everywhere
                    if (f->output_number == -1)
                    {
                        f = f->original;
                        continue;
                    }
                    const char *name = texture_by_number(state, f->texturenum);
                    size_t len = std::strlen(name);
                    bool ishidden = len >= 7 && str::iequals(&name[len - 7], "_HIDDEN");
                    if (ishidden)
                    {
                        f = f->original;
                        continue;
                    }
                    if ((int)state.map->marksurfaces.size() >= limits::max_map_marksurfaces)
                        err::fatal("exceeded max_map_marksurfaces");
                    state.map->marksurfaces.push_back((unsigned short)f->output_number);
                    f = f->original; // grab tjunction split faces
                } while (f);
            }
            delete[] n->markfaces;
            n->markfaces = nullptr;

            leaf_p->nummarksurfaces =
                (unsigned short)(state.map->marksurfaces.size() - leaf_p->firstmarksurface);
            return leafnum;
        }

        bool face_is_unwritten(bsp_state &state, face *f)
        {
            return check_face_for_hint(state, f)
                || check_face_for_skip(state, f)
                || check_face_for_null(state, f)
                || check_face_for_discardable(state, f)
                || f->texturenum == -1
                // not referenced by any nonsolid leaf: completely covered by
                // func_details
                || f->referenced == 0
                || check_face_for_env_sky(state, f);
        }

        void write_face(bsp_state &state, face *f)
        {
            if (face_is_unwritten(state, f))
            {
                f->output_number = -1;
                return;
            }

            f->output_number = (int)state.map->faces.size();

            if ((int)state.map->faces.size() >= limits::max_map_faces)
                err::fatal("exceeded max_map_faces");
            state.map->faces.resize(state.map->faces.size() + 1);
            format::dface_t *df = &state.map->faces.back();

            df->planenum = (unsigned short)write_plane(state, f->planenum);
            df->side = (short)(f->planenum & 1);
            df->firstedge = (int)state.map->surfedges.size();
            df->numedges = (short)f->numpoints;

            df->texinfo = (short)write_texinfo(state, f->texturenum);

            for (int i = 0; i < f->numpoints; i++)
            {
                int e = f->output_edges[i];
                if ((int)state.map->surfedges.size() >= limits::max_map_surfedges)
                    err::fatal("exceeded max_map_surfedges");
                state.map->surfedges.push_back(e);
            }
            delete[] f->output_edges;
            f->output_edges = nullptr;
        }

        int write_draw_nodes_r(bsp_state &state, node *n, const node *portalleaf)
        {
            if (n->isportalleaf)
            {
                if (n->contents == contents_solid)
                    return -1;
                portalleaf = n;
                // make sure parent data has not been freed when writing children
            }
            if (n->planenum == -1)
            {
                if (n->iscontentsdetail)
                {
                    delete[] n->markfaces;
                    n->markfaces = nullptr;
                    return -1;
                }
                int leafnum = write_draw_leaf(state, n, portalleaf);
                return -1 - leafnum;
            }

            int nodenum = (int)state.map->nodes.size();

            // emit a node
            if (nodenum >= limits::max_map_nodes)
                err::fatal("exceeded max_map_nodes");
            state.map->nodes.resize((size_t)nodenum + 1);

            math::vec3v mins, maxs;
            if (n->isdetail)
            {
                // intersect the loose bounds with the parent portalleaf's
                // strict bounds
                for (int k = 0; k < 3; k++)
                {
                    mins[k] = portalleaf->mins[k] > n->loosemins[k] ? portalleaf->mins[k] : n->loosemins[k];
                    maxs[k] = portalleaf->maxs[k] < n->loosemaxs[k] ? portalleaf->maxs[k] : n->loosemaxs[k];
                }
            }
            else
            {
                mins = n->mins;
                maxs = n->maxs;
            }
            {
                format::dnode_t *dn = &state.map->nodes[(size_t)nodenum];
                for (int k = 0; k < 3; k++)
                {
                    int lo = (int)mins[k] < 32767 ? (int)mins[k] : 32767;
                    int hi = (int)maxs[k] < 32767 ? (int)maxs[k] : 32767;
                    dn->mins[k] = (short)(lo > -32767 ? lo : -32767);
                    dn->maxs[k] = (short)(hi > -32767 ? hi : -32767);
                }

                if (n->planenum & 1)
                    err::fatal("write_draw_nodes_r: odd planenum");
                dn->planenum = write_plane(state, n->planenum);
                dn->firstface = (unsigned short)state.map->faces.size();
            }

            for (face *f = n->faces; f; f = f->next)
                write_face(state, f);

            {
                // the vector may have grown while writing faces; re take the
                // pointer before finishing the record
                format::dnode_t *dn = &state.map->nodes[(size_t)nodenum];
                dn->numfaces = (unsigned short)((int)state.map->faces.size() - dn->firstface);

                // recursively output the other nodes
                for (int i = 0; i < 2; i++)
                {
                    int child = write_draw_nodes_r(state, n->children[i], portalleaf);
                    state.map->nodes[(size_t)nodenum].children[i] = (short)child;
                }
            }
            return nodenum;
        }

        void output_edges_face(bsp_state &state, face *f)
        {
            if (face_is_unwritten(state, f))
                return;
            f->output_edges = new int[(size_t)f->numpoints];
            for (int i = 0; i < f->numpoints; i++)
            {
                int e = get_edge(state, f->pts[i], f->pts[(i + 1) % f->numpoints], f);
                f->output_edges[i] = e;
            }
        }

        int output_edges_r(bsp_state &state, node *n, int detaillevel)
        {
            int next = -1;
            if (n->planenum == -1)
                return next;
            for (face *f = n->faces; f; f = f->next)
            {
                if (f->detail_level > detaillevel)
                {
                    if (next == -1 ? true : f->detail_level < next)
                        next = f->detail_level;
                }
                if (f->detail_level == detaillevel)
                    output_edges_face(state, f);
            }
            for (int i = 0; i < 2; i++)
            {
                int r = output_edges_r(state, n->children[i], detaillevel);
                if (r == -1 ? false : next == -1 ? true : r < next)
                    next = r;
            }
            return next;
        }

        void remove_covered_faces_r(node *n)
        {
            if (n->isportalleaf)
            {
                if (n->contents == contents_solid)
                    return; // stop here, don't go deeper into children
            }
            if (n->planenum == -1)
            {
                // this is a leaf
                if (n->iscontentsdetail)
                    return;
                for (face **fp = n->markfaces; *fp; fp++)
                {
                    for (face *f = *fp; f; f = f->original) // each tjunc subface
                        f->referenced++; // mark the face as referenced
                }
                return;
            }

            // this is a node
            for (face *f = n->faces; f; f = f->next)
                f->referenced = 0; // clear the mark

            remove_covered_faces_r(n->children[0]);
            remove_covered_faces_r(n->children[1]);
        }

        void write_extent_file(const bsp_state &state, const std::string &filename)
        {
            std::FILE *f = std::fopen(filename.c_str(), "w");
            if (!f)
                err::fatal("error opening %s", filename.c_str());
            std::fprintf(f, "%i\n", (int)state.map->faces.size());
            for (int i = 0; i < (int)state.map->faces.size(); i++)
            {
                int mins[2];
                int maxs[2];
                format::get_face_extents(*state.map, i, mins, maxs);
                std::fprintf(f, "%i %i %i %i\n", mins[0], mins[1], maxs[0], maxs[1]);
            }
            std::fclose(f);
        }

        // removes unused miptex data (keeping animation frames of used ones)
        void reduce_texdata(bsp_state &state)
        {
            std::vector<byte> &texdata = state.map->textures;
            if (texdata.size() < 4)
                return;
            byte *l = texdata.data();
            binary::reader texture_input(texdata);
            binary::writer texture_output(texdata);
            std::int32_t nummiptex;
            if (!texture_input.i32(nummiptex) || nummiptex < 0
                || (size_t)nummiptex > (texdata.size() - 4) / 4)
            {
                logging::warn("bad texdata structure");
                return;
            }
            auto dataofs = [&](int i) -> int
            {
                std::int32_t value = -1;
                texture_input.i32_at(4 + (size_t)i * 4, value);
                return value;
            };
            auto set_dataofs = [&](int i, int v)
            {
                err::require(texture_output.patch_i32(4 + (size_t)i * 4, v),
                             "reduce_texdata: could not patch texture offset");
            };

            std::vector<bool> used((size_t)nummiptex, false);
            std::vector<int> map_index((size_t)nummiptex, -1);
            std::vector<int> lumpsizes((size_t)nummiptex);
            int texdatasize = (int)texdata.size();
            const int header = 4 + nummiptex * 4;
            const int newdatasizemax = texdatasize - header;
            std::vector<byte> newdata((size_t)(newdatasizemax > 0 ? newdatasizemax : 0));
            int newdatasize = 0;
            int total = 0;
            for (int i = 0; i < nummiptex; i++)
            {
                if (dataofs(i) == -1)
                {
                    lumpsizes[(size_t)i] = -1;
                    continue;
                }
                lumpsizes[(size_t)i] = texdatasize - dataofs(i);
                for (int j = 0; j < nummiptex; j++)
                {
                    int lumpsize = dataofs(j) - dataofs(i);
                    if (dataofs(j) == -1 || lumpsize < 0 || (lumpsize == 0 && j <= i))
                        continue;
                    if (lumpsize < lumpsizes[(size_t)i])
                        lumpsizes[(size_t)i] = lumpsize;
                }
                total += lumpsizes[(size_t)i];
            }
            if (total != newdatasizemax)
            {
                logging::warn("bad texdata structure");
                return;
            }
            for (const format::texinfo_t &t : state.map->texinfo)
            {
                if (t.miptex < 0 || t.miptex >= nummiptex)
                {
                    logging::warn("bad miptex number %d", t.miptex);
                    return;
                }
                used[(size_t)t.miptex] = true;
            }
            for (int i = 0; i < nummiptex; i++)
            {
                if (dataofs(i) < 0)
                    continue;
                if (used[(size_t)i])
                {
                    // pull in the other animation frames of used textures
                    const char *m = (const char *)(l + dataofs(i));
                    if (m[0] != '+' && m[0] != '-')
                        continue;
                    char name[16];
                    str::copy(name, sizeof(name), m);
                    if (name[1] == '\0')
                        continue;
                    for (int j = 0; j < 20; j++)
                    {
                        name[1] = j < 10 ? (char)('0' + j) : (char)('A' + j - 10);
                        for (int k = 0; k < nummiptex; k++)
                        {
                            if (dataofs(k) < 0)
                                continue;
                            const char *m2 = (const char *)(l + dataofs(k));
                            if (str::iequals(name, m2))
                                used[(size_t)k] = true;
                        }
                    }
                }
            }
            int num = 0;
            for (int i = 0; i < nummiptex; i++)
            {
                if (used[(size_t)i])
                {
                    map_index[(size_t)i] = num;
                    num++;
                }
            }
            for (format::texinfo_t &t : state.map->texinfo)
                t.miptex = map_index[(size_t)t.miptex];
            int size = 4 + num * 4;
            for (int i = 0; i < nummiptex; i++)
            {
                if (!used[(size_t)i])
                    continue;
                if (lumpsizes[(size_t)i] == -1)
                {
                    set_dataofs(map_index[(size_t)i], -1);
                }
                else
                {
                    std::memcpy(newdata.data() + newdatasize, l + dataofs(i), (size_t)lumpsizes[(size_t)i]);
                    set_dataofs(map_index[(size_t)i], size);
                    newdatasize += lumpsizes[(size_t)i];
                    size += lumpsizes[(size_t)i];
                }
            }
            std::memcpy(l + 4 + (size_t)num * 4, newdata.data(), (size_t)newdatasize);
            logging::file("  reduced %d texdatas to %d (%d bytes to %d)\n",
                          nummiptex, num, texdatasize, size);
            err::require(texture_output.patch_i32(0, num),
                         "reduce_texdata: could not patch texture count");
            texdata.resize((size_t)size);
        }
    }

    void write_clip_nodes(bsp_state &state, node *nodes)
    {
        // only merge among the clipnodes of the same hull of the same model
        clipnode_map outputmap;
        write_clip_nodes_r(state, nodes, nullptr, &outputmap);
    }

    void write_draw_nodes(bsp_state &state, node *headnode)
    {
        remove_covered_faces_r(headnode); // fill the referenced values
        // higher detail levels must not compete for edge pairing with lower
        // detail levels
        int nextdetaillevel;
        for (int detaillevel = 0; detaillevel != -1; detaillevel = nextdetaillevel)
            nextdetaillevel = output_edges_r(state, headnode, detaillevel);
        write_draw_nodes_r(state, headnode, nullptr);
    }

    void optimize_face_order(bsp_state &state)
    {
        format::map_data &map = *state.map;
        if (map.faces.empty())
            return;

        struct face_size
        {
            bool lightmapped;
            long long area;
            int longest;
            int shortest;
            int width;
            int height;
        };

        std::vector<face_size> sizes(map.faces.size());
        for (int i = 0; i < (int)map.faces.size(); i++)
        {
            const format::dface_t &face = map.faces[(size_t)i];
            int texinfo = format::parse_texinfo_for_face(map, &face);
            err::require(texinfo >= 0 && texinfo < (int)map.texinfo.size(),
                         "optimize_face_order: invalid texinfo");
            const char *name = texture_by_number(state, texinfo);
            bool lightmapped = std::strncmp(name, "sky", 3) != 0
                && name[0] != '!'
                && !str::istarts_with(name, "water")
                && !str::istarts_with(name, "laser")
                && !(map.texinfo[(size_t)texinfo].flags & tex_special);
            int mins[2];
            int maxs[2];
            format::get_face_extents(map, i, mins, maxs);
            int width = maxs[0] - mins[0] + 1;
            int height = maxs[1] - mins[1] + 1;
            sizes[(size_t)i] = {
                lightmapped,
                (long long)width * height,
                width > height ? width : height,
                width < height ? width : height,
                width,
                height,
            };
        }

        std::vector<int> old_at_new(map.faces.size());
        std::iota(old_at_new.begin(), old_at_new.end(), 0);
        std::vector<int> original_order = old_at_new;
        for (const format::dnode_t &node : map.nodes)
        {
            size_t first = node.firstface;
            size_t count = node.numfaces;
            if (count < 2)
                continue;
            err::require(first <= map.faces.size() && count <= map.faces.size() - first,
                         "optimize_face_order: invalid node face range");
            std::stable_sort(old_at_new.begin() + (std::ptrdiff_t)first,
                             old_at_new.begin() + (std::ptrdiff_t)(first + count),
                             [&sizes](int a, int b)
                             {
                                 const face_size &left = sizes[(size_t)a];
                                 const face_size &right = sizes[(size_t)b];
                                 return std::tie(left.lightmapped, left.area, left.longest, left.shortest)
                                     > std::tie(right.lightmapped, right.area, right.longest, right.shortest);
                             });
        }

        auto count_pages = [&sizes](const std::vector<int> &order)
        {
            using skyline = std::array<int, limits::block_width>;
            std::vector<skyline> pages;
            for (int face_index : order)
            {
                const face_size &size = sizes[(size_t)face_index];
                if (!size.lightmapped)
                    continue;

                bool placed = false;
                for (skyline &page : pages)
                {
                    int best = limits::block_height;
                    int best_x = 0;
                    for (int x = 0; x < limits::block_width - size.width; x++)
                    {
                        int height = 0;
                        int i = 0;
                        for (; i < size.width; i++)
                        {
                            if (page[(size_t)(x + i)] >= best)
                                break;
                            if (page[(size_t)(x + i)] > height)
                                height = page[(size_t)(x + i)];
                        }
                        if (i == size.width)
                        {
                            best_x = x;
                            best = height;
                        }
                    }
                    if (best + size.height > limits::block_height)
                        continue;
                    for (int i = 0; i < size.width; i++)
                        page[(size_t)(best_x + i)] = best + size.height;
                    placed = true;
                    break;
                }
                if (!placed)
                {
                    pages.push_back({});
                    skyline &page = pages.back();
                    err::require(size.width < limits::block_width
                                     && size.height <= limits::block_height,
                                 "optimize_face_order: invalid lightmap size");
                    for (int i = 0; i < size.width; i++)
                        page[(size_t)i] = size.height;
                }
            }
            return (int)pages.size();
        };

        std::vector<int> baseline_order = old_at_new;
        std::vector<unsigned short> baseline_firstfaces;
        baseline_firstfaces.reserve(map.nodes.size());
        for (const format::dnode_t &node : map.nodes)
            baseline_firstfaces.push_back(node.firstface);

        struct node_group
        {
            size_t node_index;
            size_t first;
            size_t count;
            long long area;
        };

        for (const format::dmodel_t &model : map.models)
        {
            err::require(model.firstface >= 0 && model.numfaces >= 0,
                         "optimize_face_order: invalid model face range");
            size_t model_first = (size_t)model.firstface;
            size_t model_count = (size_t)model.numfaces;
            err::require(model_first <= map.faces.size()
                             && model_count <= map.faces.size() - model_first,
                         "optimize_face_order: invalid model face range");
            if (model_count < 2)
                continue;

            std::vector<unsigned char> covered(model_count, 0);
            std::vector<node_group> groups;
            for (size_t node_index = 0; node_index < map.nodes.size(); node_index++)
            {
                const format::dnode_t &node = map.nodes[node_index];
                size_t first = node.firstface;
                size_t count = node.numfaces;
                if (!count || first < model_first || first >= model_first + model_count)
                    continue;
                err::require(count <= model_first + model_count - first,
                             "optimize_face_order: node crosses model face range");

                node_group group = {node_index, first, count, 0};
                for (size_t i = first; i < first + count; i++)
                {
                    size_t relative = i - model_first;
                    err::require(!covered[relative],
                                 "optimize_face_order: overlapping node face ranges");
                    covered[relative] = 1;
                    long long area = sizes[(size_t)old_at_new[i]].area;
                    group.area += area;
                }
                groups.push_back(group);
            }
            for (unsigned char value : covered)
                err::require(value != 0, "optimize_face_order: face is not owned by a node");

            std::stable_sort(groups.begin(), groups.end(),
                             [](const node_group &left, const node_group &right)
                             {
                                 return left.area > right.area;
                             });

            std::vector<int> reordered_model;
            reordered_model.reserve(model_count);
            size_t next = model_first;
            for (const node_group &group : groups)
            {
                map.nodes[group.node_index].firstface = (unsigned short)next;
                reordered_model.insert(reordered_model.end(),
                                       old_at_new.begin() + (std::ptrdiff_t)group.first,
                                       old_at_new.begin() + (std::ptrdiff_t)(group.first + group.count));
                next += group.count;
            }
            err::require(reordered_model.size() == model_count,
                         "optimize_face_order: incomplete model face order");
            std::copy(reordered_model.begin(), reordered_model.end(),
                      old_at_new.begin() + (std::ptrdiff_t)model_first);
        }

        int best_pages = count_pages(original_order);
        const std::vector<int> *best_order = &original_order;
        int baseline_pages = count_pages(baseline_order);
        if (baseline_pages < best_pages)
        {
            best_pages = baseline_pages;
            best_order = &baseline_order;
        }
        bool grouped = count_pages(old_at_new) < best_pages;
        if (!grouped)
        {
            old_at_new = *best_order;
            for (size_t i = 0; i < map.nodes.size(); i++)
                map.nodes[i].firstface = baseline_firstfaces[i];
        }

        std::vector<int> new_for_old(map.faces.size());
        std::vector<format::dface_t> reordered(map.faces.size());
        for (size_t new_index = 0; new_index < old_at_new.size(); new_index++)
        {
            int old_index = old_at_new[new_index];
            reordered[new_index] = map.faces[(size_t)old_index];
            new_for_old[(size_t)old_index] = (int)new_index;
        }
        for (unsigned short &marksurface : map.marksurfaces)
        {
            err::require((size_t)marksurface < new_for_old.size(),
                         "optimize_face_order: invalid marksurface");
            marksurface = (unsigned short)new_for_old[(size_t)marksurface];
        }
        map.faces = std::move(reordered);
    }

    void begin_bsp_file(bsp_state &state)
    {
        // the loaded file may carry stale data, so clear explicitly
        g_writer = writer_state();

        state.map->models.clear();
        state.map->faces.clear();
        state.map->nodes.clear();
        state.map->clipnodes.clear();
        state.map->vertexes.clear();
        state.map->marksurfaces.clear();
        state.map->surfedges.clear();

        // edge 0 is not used, because 0 can't be negated
        state.map->edges.assign(1, format::dedge_t());

        // leaf 0 is common solid with no faces (the reference leaves every
        // other field zero initialized, including visofs)
        state.map->leafs.assign(1, format::dleaf_t());
        state.map->leafs[0].contents = contents_solid;
    }

    void finish_bsp_file(bsp_state &state)
    {
        format::map_data &map = *state.map;

        if (!map.models.empty() && map.models[0].visleafs > limits::max_map_leafs_engine)
        {
            logging::warn("Number of world leaves(%d) exceeded MAX_MAP_LEAFS(%d)\nIf you encounter problems when running your map, consider this the most likely cause",
                          map.models[0].visleafs, limits::max_map_leafs_engine);
        }
        if (!map.models.empty() && map.models[0].numfaces > max_map_worldfaces)
        {
            logging::warn("Number of world faces(%d) exceeded %d. Some faces will disappear in game.\nTo reduce world faces, change some world brushes (including func_details) to func_walls",
                          map.models[0].numfaces, max_map_worldfaces);
        }
        if (!state.options.noclipnodemerge)
        {
            logging::file("  reduced %d clipnodes to %d\n",
                          (int)map.clipnodes.size() + g_writer.count_mergedclipnodes,
                          (int)map.clipnodes.size());
        }
        if (!state.options.noopt)
        {
            logging::file("  reduced %d texinfos to %d\n",
                          (int)map.texinfo.size(), (int)g_writer.mapped_texinfos.size());
            map.texinfo = g_writer.mapped_texinfos;

            reduce_texdata(state);

            logging::file("  reduced %d planes to %d\n",
                          (int)state.planes.size(), (int)g_writer.mapped_planes.size());
            state.planes = g_writer.mapped_planes;
        }

        if (!state.options.nobrink)
        {
            fix_all_brinks(state);
        }

        optimize_face_order(state);
        write_extent_file(state, state.base_path + ".ext");

        // convert the double working planes to the disk format
        map.planes.resize(state.planes.size());
        for (size_t i = 0; i < state.planes.size(); i++)
        {
            const plane &mp = state.planes[i];
            format::dplane_t &dp = map.planes[i];
            dp.normal[0] = (float)mp.normal[0];
            dp.normal[1] = (float)mp.normal[1];
            dp.normal[2] = (float)mp.normal[2];
            dp.dist = (float)mp.dist;
            dp.type = mp.type;
        }
    }
}
