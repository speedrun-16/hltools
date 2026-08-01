#include "map.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <set>
#include <string>
#include <vector>

#include "entities.h"
#include "displacements.h"
#include "materials.h"
#include "format/map/document.h"
#include "format/vbsp/data.h"
#include "format/bsp/entity_lump.h"
#include "math/vector.h"
#include "math/winding.h"

namespace decompile
{
    namespace
    {
        using vec3 = math::vec3<double>;
        using winding = math::basic_winding<double>;

        constexpr double winding_range = 131072.0;
        constexpr double minimum_face_area = 0.1;
        constexpr int maximum_tree_depth = 8192;

        // source contents bit for brushes that were func_detail before vbsp merged
        // them into the world. the overwhelming majority of a detailed map is
        // detail; emitting it as world geometry makes every facet split visleafs
        // and generate portals, which overflows the compiler. routing it to
        // func_detail keeps it solid but out of the bsp/vis portal computation.
        constexpr int contents_detail = 0x8000000;
        // vbsp marks brushes carrying $translucent materials (glass) as window
        // content. goldsrc world faces cannot render translucent, so those become
        // func_wall entities with rendermode texture instead.
        constexpr int contents_window = 0x2;
        constexpr int contents_translucent = 0x10000000;
        // liquid volumes; goldsrc expresses these as func_water entities whose
        // sides all carry a '!'-prefixed texture
        constexpr int contents_water = 0x20;
        // volumes that stop the player but not bullets, goldsrc's CLIP texture
        constexpr int contents_playerclip = 0x10000;

        bool is_window_brush(int contents)
        {
            return (contents & (contents_window | contents_translucent)) != 0;
        }

        void set_error(std::string *error, const std::string &message)
        {
            if (error)
                *error = message;
        }

        vec3 negate(const vec3 &v)
        {
            return {-v.x, -v.y, -v.z};
        }

        std::string lowercase(std::string s)
        {
            for (char &c : s)
                c = (char)std::tolower((unsigned char)c);
            return s;
        }

        struct plane_side
        {
            vec3 normal;
            double dist = 0;
        };

        plane_side side_plane(const format::source_map_data &map, int planenum)
        {
            plane_side out;
            const format::source_dplane_t &p = map.planes[(std::size_t)planenum];
            out.normal = {p.normal[0], p.normal[1], p.normal[2]};
            out.dist = p.dist;
            return out;
        }

        // the visible polygon of one brush side: the side's plane clipped against
        // every other (non bevel) side of the same brush. mirrors bspsrc's
        // WindingFactory.fromSide and our goldsrc build_cell.
        winding brushside_winding(const format::source_map_data &map,
                                  const format::source_dbrush_t &brush, int side_index)
        {
            const format::source_dbrushside_t &side =
                map.brushsides[(std::size_t)side_index];
            plane_side self = side_plane(map, side.planenum);
            winding w = winding::from_plane(self.normal, self.dist, winding_range);

            for (int i = 0; i < brush.numsides && !w.empty(); i++)
            {
                int other_index = brush.firstside + i;
                if (other_index == side_index)
                    continue;
                const format::source_dbrushside_t &other =
                    map.brushsides[(std::size_t)other_index];
                if (other.bevel)
                    continue;
                plane_side op = side_plane(map, other.planenum);
                w.chop(negate(op.normal), -op.dist);
            }
            w.remove_colinear_points();
            return w;
        }

        struct side_texture
        {
            std::string name;
            float vecs[2][4];
            bool realigned = false;
        };

        // the six axial projections of the original tools' TextureAxisFromPlane,
        // one per dominant face direction: normal, then the u and v axes.
        const double base_axes[6][3][3] = {
            {{0, 0, 1}, {1, 0, 0}, {0, -1, 0}},  // floor
            {{0, 0, -1}, {1, 0, 0}, {0, -1, 0}}, // ceiling
            {{1, 0, 0}, {0, 1, 0}, {0, 0, -1}},  // west wall
            {{-1, 0, 0}, {0, 1, 0}, {0, 0, -1}}, // east wall
            {{0, 1, 0}, {1, 0, 0}, {0, 0, -1}},  // south wall
            {{0, -1, 0}, {1, 0, 0}, {0, 0, -1}}, // north wall
        };

        void world_aligned_axes(const vec3 &normal, float vecs[2][4])
        {
            int best = 0;
            double best_dot = -1e30;
            for (int i = 0; i < 6; i++)
            {
                double d = normal.x * base_axes[i][0][0]
                    + normal.y * base_axes[i][0][1] + normal.z * base_axes[i][0][2];
                if (d > best_dot)
                {
                    best_dot = d;
                    best = i;
                }
            }
            for (int axis = 0; axis < 2; axis++)
            {
                for (int k = 0; k < 3; k++)
                    vecs[axis][k] = (float)base_axes[best][axis + 1][k];
                vecs[axis][3] = 0;
            }
        }

        // true when the texture axes project onto the face plane; source reuses
        // texinfo across the sides of a brush, so a side can carry axes that are
        // edge-on to its own plane. vbsp never drew those sides, but goldsrc
        // does, and rad fatals ("Malformed face normal") building the lightmap
        // projection for them.
        bool axes_projectable(const float vecs[2][4], const vec3 &normal)
        {
            vec3 u{vecs[0][0], vecs[0][1], vecs[0][2]};
            vec3 v{vecs[1][0], vecs[1][1], vecs[1][2]};
            vec3 t = math::cross(u, v);
            if (math::normalize(t) == 0)
                return false;
            return std::fabs(math::dot(t, normal)) >= 0.01;
        }

        // resolves the goldsrc texture name and (uv-corrected) texture axes for a
        // brush side. surface flags win first, then the material catalog. when a
        // real texture was built its axes are scaled by the resize ratio so the
        // physical texel size is preserved.
        // engine-special textures (NULL/SKY/HINT/SKIP/CLIP/AAATRIGGER) are
        // consumed by the compiler, so their uv mapping is meaningless in game.
        // source's inherited axes are often edge-on to the side and use source's
        // 0.25 texel density, which renders as smears or a 4x tiled texture in
        // the editor; clean world-aligned axes at scale 1 match a freshly placed
        // texture instead.
        side_texture special_side(const std::string &name, const vec3 &normal)
        {
            side_texture st;
            st.name = name;
            world_aligned_axes(normal, st.vecs);
            return st;
        }

        side_texture resolve_side_texture(const format::source_map_data &map,
                                          const material_catalog &catalog,
                                          int texinfo_index, const vec3 &normal)
        {
            if (texinfo_index < 0 || (std::size_t)texinfo_index >= map.texinfo.size())
                return special_side("NULL", normal);

            const format::source_texinfo_t &ti = map.texinfo[(std::size_t)texinfo_index];
            if (ti.flags & format::vsurf_nodraw)
                return special_side("NULL", normal);
            if (ti.flags & (format::vsurf_sky | format::vsurf_sky2d))
                return special_side("SKY", normal);
            if (ti.flags & format::vsurf_hint)
                return special_side("HINT", normal);
            if (ti.flags & format::vsurf_skip)
                return special_side("SKIP", normal);

            const std::string &material = map.texinfo_material(texinfo_index);
            if (material.empty())
                return special_side("NULL", normal);
            const resolved_material &rm = catalog.resolve(material);
            if (rm.special)
                return special_side(rm.name, normal);

            side_texture st;
            st.name = rm.name;
            std::memcpy(st.vecs, ti.texture_vecs, sizeof(st.vecs));
            if (rm.has_texture)
            {
                for (int k = 0; k < 4; k++)
                {
                    st.vecs[0][k] = (float)(st.vecs[0][k] * rm.u_scale);
                    st.vecs[1][k] = (float)(st.vecs[1][k] * rm.v_scale);
                }
            }
            if (!axes_projectable(st.vecs, normal))
            {
                world_aligned_axes(normal, st.vecs);
                st.realigned = true;
            }
            return st;
        }

        struct brush_range
        {
            int first = 0;
            int last_exclusive = 0;
            bool empty() const { return last_exclusive <= first; }
        };

        // walks a model's bsp subtree collecting the min and max brush index found
        // in its leaves. valid=false means the subtree referenced no brushes.
        struct walk_state
        {
            int min_brush = 0;
            int max_brush = -1;
            bool valid = false;
        };

        void walk_brushes(const format::source_map_data &map, int node_index,
                          int depth, walk_state &state)
        {
            if (depth > maximum_tree_depth)
                return;
            if (node_index < 0)
            {
                int leaf_index = -1 - node_index;
                if (leaf_index < 0 || (std::size_t)leaf_index >= map.leaves.size())
                    return;
                const format::source_leaf_t &leaf = map.leaves[(std::size_t)leaf_index];
                for (int i = 0; i < leaf.numleafbrushes; i++)
                {
                    std::size_t ref = (std::size_t)leaf.firstleafbrush + (std::size_t)i;
                    if (ref >= map.leafbrushes.size())
                        break;
                    int brush = map.leafbrushes[ref];
                    if (brush < 0 || (std::size_t)brush >= map.brushes.size())
                        continue;
                    if (!state.valid)
                    {
                        state.min_brush = state.max_brush = brush;
                        state.valid = true;
                    }
                    else
                    {
                        state.min_brush = std::min(state.min_brush, brush);
                        state.max_brush = std::max(state.max_brush, brush);
                    }
                }
                return;
            }
            if ((std::size_t)node_index >= map.nodes.size())
                return;
            const format::source_dnode_t &node = map.nodes[(std::size_t)node_index];
            walk_brushes(map, node.children[0], depth + 1, state);
            walk_brushes(map, node.children[1], depth + 1, state);
        }

        // brushes are laid out contiguously per model: world brushes first, then
        // each brush model's block. the walk gives the block bounds; using the
        // contiguous range (not just leaf-referenced brushes) recovers brushes
        // that sit in solid space and appear in no leaf.
        std::vector<brush_range> assign_brush_ranges(const format::source_map_data &map)
        {
            std::vector<brush_range> ranges(map.models.size());
            int worldbrushes = 0;
            for (std::size_t m = 0; m < map.models.size(); m++)
            {
                walk_state state;
                walk_brushes(map, map.models[m].headnode, 0, state);
                if (m == 0)
                {
                    worldbrushes = state.valid ? state.max_brush + 1 : 0;
                    ranges[0] = {0, worldbrushes};
                }
                else if (state.valid)
                {
                    ranges[m] = {state.min_brush, state.max_brush + 1};
                }
            }
            return ranges;
        }

        // worldspawn's world_mins/world_maxs give the playable extent. a source
        // 3d skybox is a small separate scene outside that box, which goldsrc
        // cannot use: it would be unreachable brushes that still cost tree,
        // faces and lighting, and its 8192 unit sky slabs overflow a leaf's
        // face list once subdivided. so brushes outside the box are dropped.
        //
        // only when the map has a sky_camera, though. a map's own sky shell
        // also sits outside the playable extent, so culling without that check
        // deletes the sky and leaves no world brushes at all.
        struct world_box
        {
            bool valid = false;
            vec3 mins{}, maxs{};

            bool excludes(const vec3 &lo, const vec3 &hi) const
            {
                if (!valid)
                    return false;
                // a margin keeps anything merely touching the world
                constexpr double margin = 64.0;
                return hi.x < mins.x - margin || lo.x > maxs.x + margin
                    || hi.y < mins.y - margin || lo.y > maxs.y + margin
                    || hi.z < mins.z - margin || lo.z > maxs.z + margin;
            }
        };

        world_box read_world_box(const std::vector<format::entity> &entities)
        {
            world_box box;
            bool skybox = false;
            for (const format::entity &entity : entities)
                if (std::strcmp(entity.value("classname"), "sky_camera") == 0)
                {
                    skybox = true;
                    break;
                }
            if (!skybox)
                return box; // no 3d skybox to separate; keep every brush
            const format::entity &worldspawn = entities[0];
            if (std::sscanf(worldspawn.value("world_mins"), "%lf %lf %lf",
                            &box.mins.x, &box.mins.y, &box.mins.z) == 3
                && std::sscanf(worldspawn.value("world_maxs"), "%lf %lf %lf",
                               &box.maxs.x, &box.maxs.y, &box.maxs.z) == 3)
                box.valid = true;
            return box;
        }

        // true for a brush that is nothing but player clip.
        //
        // source uses tools/toolsplayerclip two ways: as a real clip volume, and
        // as plain nodraw on the hidden sides of ordinary brushes (the material
        // carries SURF_NODRAW, so vbsp never drew those faces either). only the
        // first is goldsrc's CLIP, and the distinction has to be made per brush:
        // CLIP rewrites the whole brush into the collision hulls, so mixing it
        // with visible faces would turn solid geometry into an invisible volume.
        bool is_clip_brush(const format::source_map_data &map,
                           const material_catalog &catalog,
                           const format::source_dbrush_t &brush)
        {
            if ((brush.contents & contents_playerclip) == 0)
                return false;
            int clip_sides = 0;
            for (int i = 0; i < brush.numsides; i++)
            {
                std::size_t side_index = (std::size_t)(brush.firstside + i);
                if (side_index >= map.brushsides.size())
                    continue;
                if (map.brushsides[side_index].bevel)
                    continue;
                const std::string &material =
                    map.texinfo_material(map.brushsides[side_index].texinfo);
                if (material.empty())
                    return false;
                const resolved_material &rm = catalog.resolve(material);
                if (!rm.special || rm.name != "CLIP")
                    return false;
                clip_sides++;
            }
            return clip_sides > 0;
        }

        // builds one document brush from a source brush: bevel and degenerate
        // sides are dropped, planes become canonical map sides (every face on a
        // shared source plane serializes identically, so csg collapses them
        // into one splitting plane), and texture axes come from the catalog.
        bool build_brush(const format::source_map_data &map, int brush_index,
                         const vec3 &model_origin, bool world_model,
                         const material_catalog &catalog, const world_box &box,
                         source_result &result, format::map_brush &out)
        {
            const format::source_dbrush_t &brush = map.brushes[(std::size_t)brush_index];
            out = format::map_brush{};
            out.sides.reserve((std::size_t)brush.numsides);

            // csg keys the clip hulls off the texture name, so a clip volume
            // has to carry CLIP on every side rather than the NULL its nodraw
            // surface flag would otherwise produce. only world geometry gets it:
            // CLIP rewrites a brush into the collision hulls alone, which would
            // leave a moving brush entity (a tracktrain carrying its model here)
            // with nothing to render or collide against. those stay NULL, which
            // is goldsrc's invisible-but-solid brush.
            bool clip = world_model && is_clip_brush(map, catalog, brush);

            vec3 lo{1e30, 1e30, 1e30};
            vec3 hi{-1e30, -1e30, -1e30};

            for (int i = 0; i < brush.numsides; i++)
            {
                int side_index = brush.firstside + i;
                if (side_index < 0 || (std::size_t)side_index >= map.brushsides.size())
                    continue;
                const format::source_dbrushside_t &side =
                    map.brushsides[(std::size_t)side_index];
                if (side.bevel)
                    continue;
                if ((std::size_t)side.planenum >= map.planes.size())
                    continue;

                // the winding only validates that this side actually bounds the
                // brush; the emitted plane comes from the source plane directly.
                winding w = brushside_winding(map, brush, side_index);
                if (w.size() < 3 || w.area() < minimum_face_area)
                {
                    result.skipped_sides++;
                    continue;
                }
                for (int p = 0; p < w.size(); p++)
                {
                    vec3 point = w[p] + model_origin;
                    lo.x = std::min(lo.x, point.x);
                    lo.y = std::min(lo.y, point.y);
                    lo.z = std::min(lo.z, point.z);
                    hi.x = std::max(hi.x, point.x);
                    hi.y = std::max(hi.y, point.y);
                    hi.z = std::max(hi.z, point.z);
                }

                plane_side plane = side_plane(map, side.planenum);
                plane.dist += math::dot(plane.normal, model_origin);
                side_texture texture =
                    clip ? special_side("CLIP", plane.normal)
                         : resolve_side_texture(map, catalog, side.texinfo, plane.normal);
                if (texture.realigned)
                    result.realigned_sides++;

                format::map_side out_side =
                    format::map_side::from_plane(plane.normal, plane.dist);
                out_side.texture = std::move(texture.name);
                for (int axis = 0; axis < 2; axis++)
                {
                    for (int k = 0; k < 4; k++)
                        out_side.axes[axis][k] = texture.vecs[axis][k];
                    out_side.axes[axis][3] -=
                        model_origin.x * out_side.axes[axis][0]
                        + model_origin.y * out_side.axes[axis][1]
                        + model_origin.z * out_side.axes[axis][2];
                }
                out.sides.push_back(std::move(out_side));
            }

            if (out.sides.size() < 4)
            {
                result.skipped_brushes++;
                return false;
            }
            if (box.excludes(lo, hi))
            {
                result.skybox_brushes++;
                return false;
            }
            if (clip)
                result.clip_brushes++;
            return true;
        }

        bool build_displacement_brush(
            const format::source_map_data &map, const material_catalog &catalog,
            const source_displacement_triangle &triangle, source_result &result,
            bool collision_only, format::map_brush &out)
        {
            constexpr double backing_depth = 8.0;

            const vec3 &a = triangle.points[0];
            const vec3 &b = triangle.points[1];
            const vec3 &c = triangle.points[2];
            vec3 normal = math::cross(b - a, c - a);
            if (math::normalize(normal) == 0)
                return false;
            if (math::dot(normal, triangle.base_normal) < 0)
                normal = -normal;

            vec3 inward = triangle.base_normal * -backing_depth;
            out = format::map_brush{};
            out.sides.reserve(5);

            side_texture texture = collision_only
                ? special_side("CLIP", normal)
                : resolve_side_texture(map, catalog, triangle.texinfo, normal);
            if (texture.realigned)
                result.realigned_sides++;
            format::map_side top =
                format::map_side::from_plane(normal, math::dot(normal, a));
            top.texture = std::move(texture.name);
            for (int axis = 0; axis < 2; axis++)
                for (int k = 0; k < 4; k++)
                    top.axes[axis][k] = texture.vecs[axis][k];
            out.sides.push_back(std::move(top));

            format::map_side underside =
                format::map_side::from_plane(
                    -triangle.base_normal,
                    -triangle.minimum_projection + backing_depth);
            side_texture underside_texture =
                special_side(collision_only ? "CLIP" : "NULL",
                             -triangle.base_normal);
            underside.texture = std::move(underside_texture.name);
            for (int axis = 0; axis < 2; axis++)
                for (int k = 0; k < 4; k++)
                    underside.axes[axis][k] = underside_texture.vecs[axis][k];
            out.sides.push_back(std::move(underside));

            const vec3 points[3] = {a, b, c};
            for (int edge = 0; edge < 3; edge++)
            {
                const vec3 &p = points[edge];
                const vec3 &q = points[(edge + 1) % 3];
                vec3 side_normal = math::cross(inward, q - p);
                if (math::normalize(side_normal) == 0)
                    return false;
                format::map_side side = format::map_side::from_plane(
                    side_normal, math::dot(side_normal, p));
                side_texture side_tex =
                    special_side(collision_only ? "CLIP" : "NULL", side_normal);
                side.texture = std::move(side_tex.name);
                for (int axis = 0; axis < 2; axis++)
                    for (int k = 0; k < 4; k++)
                        side.axes[axis][k] = side_tex.vecs[axis][k];
                out.sides.push_back(std::move(side));
            }
            return true;
        }

        void append_displacements(const format::source_map_data &map,
                                  const material_catalog &catalog,
                                  format::map_document &doc,
                                  source_result &result)
        {
            source_displacement_mesh mesh = build_source_displacement_mesh(map);
            result.displacement_faces = mesh.faces;
            result.skipped_displacement_faces = mesh.skipped_faces;
            result.skipped_displacement_triangles = mesh.skipped_triangles;

            for (const source_displacement_triangle &triangle : mesh.triangles)
            {
                format::map_brush brush;
                if (!build_displacement_brush(
                        map, catalog, triangle, result, false, brush))
                {
                    result.skipped_displacement_triangles++;
                    continue;
                }
                doc.add_brush(std::move(brush), "func_detail",
                              {{"zhlt_detaillevel", "1"},
                               {"zhlt_clipnodedetaillevel", "1"},
                               {"zhlt_noclip", "1"}});
                result.displacement_brushes++;
                result.detail_brushes++;
            }

            for (const source_displacement_triangle &triangle
                 : mesh.collision_triangles)
            {
                format::map_brush brush;
                if (!build_displacement_brush(
                        map, catalog, triangle, result, true, brush))
                {
                    result.skipped_displacement_triangles++;
                    continue;
                }
                doc.add_brush(std::move(brush), "func_detail",
                              {{"zhlt_detaillevel", "1"},
                               {"zhlt_clipnodedetaillevel", "1"}});
                result.displacement_collision_brushes++;
                result.detail_brushes++;
            }
        }

        // the water material of a liquid brush: the first side resolving to a
        // Water-shader material decides. sides facing away from the player are
        // nodraw in source, so usually exactly one side carries it. nullptr when
        // the brush has water contents but no recognizable water material.
        const resolved_material *water_material(const format::source_map_data &map,
                                                const material_catalog &catalog,
                                                const format::source_dbrush_t &brush)
        {
            if ((brush.contents & contents_water) == 0)
                return nullptr;
            for (int i = 0; i < brush.numsides; i++)
            {
                std::size_t side_index = (std::size_t)(brush.firstside + i);
                if (side_index >= map.brushsides.size())
                    continue;
                const std::string &material =
                    map.texinfo_material(map.brushsides[side_index].texinfo);
                if (material.empty())
                    continue;
                const resolved_material &rm = catalog.resolve(material);
                if (rm.water)
                    return &rm;
            }
            return nullptr;
        }

        // true when any side of the brush resolves to a '{' masked material. the
        // engine only punches the mask out on an entity set to rendermode solid,
        // so the owning entity has to be retagged.
        bool has_masked_material(const format::source_map_data &map,
                                 const material_catalog &catalog,
                                 const format::source_dbrush_t &brush)
        {
            for (int i = 0; i < brush.numsides; i++)
            {
                std::size_t side_index = (std::size_t)(brush.firstside + i);
                if (side_index >= map.brushsides.size())
                    continue;
                const std::string &material =
                    map.texinfo_material(map.brushsides[side_index].texinfo);
                if (material.empty())
                    continue;
                if (catalog.resolve(material).masked)
                    return true;
            }
            return false;
        }

        // the first translucent (non masked) material on the brush, or nullptr.
        // source puts the opacity in the material, goldsrc in the entity, so a
        // brush entity has to inherit it from whatever it is textured with.
        const resolved_material *translucent_material(const format::source_map_data &map,
                                                      const material_catalog &catalog,
                                                      const format::source_dbrush_t &brush)
        {
            for (int i = 0; i < brush.numsides; i++)
            {
                std::size_t side_index = (std::size_t)(brush.firstside + i);
                if (side_index >= map.brushsides.size())
                    continue;
                const std::string &material =
                    map.texinfo_material(map.brushsides[side_index].texinfo);
                if (material.empty())
                    continue;
                const resolved_material &rm = catalog.resolve(material);
                if (rm.translucent && !rm.masked)
                    return &rm;
            }
            return nullptr;
        }

        // the render amount reproducing a window brush's translucency: the first
        // translucent side material decides, matching how source shades the brush
        int window_render_amount(const format::source_map_data &map,
                                 const material_catalog &catalog,
                                 const format::source_dbrush_t &brush)
        {
            for (int i = 0; i < brush.numsides; i++)
            {
                std::size_t side_index = (std::size_t)(brush.firstside + i);
                if (side_index >= map.brushsides.size())
                    continue;
                const std::string &material =
                    map.texinfo_material(map.brushsides[side_index].texinfo);
                if (material.empty())
                    continue;
                const resolved_material &rm = catalog.resolve(material);
                if (rm.translucent)
                    return rm.render_amount;
            }
            return 128;
        }

        int entity_model(const format::entity &entity, std::size_t entity_index)
        {
            if (entity_index == 0)
                return 0;
            const char *model = entity.value("model");
            if (model[0] != '*')
                return -1;
            char *end = nullptr;
            long value = std::strtol(model + 1, &end, 10);
            if (!end || *end != '\0' || value < 0)
                return -1;
            return (int)value;
        }

        // a brush entity that only draws: nothing targets it, it fires no
        // outputs, and its classname carries no behaviour of its own. two of
        // these with the same keyvalues are indistinguishable in game.
        bool is_static_brush_entity(const format::map_entity &entity)
        {
            if (entity.brushes.empty())
                return false;
            const char *classname = entity.keyvalues.value("classname");
            if (std::strcmp(classname, "func_wall") != 0
                && std::strcmp(classname, "func_illusionary") != 0
                && std::strcmp(classname, "func_detail") != 0)
                return false;
            if (entity.keyvalues.value("targetname")[0] != '\0')
                return false;
            for (const format::entity::pair &kv : entity.keyvalues.pairs())
                if (kv.first.size() > 2 && kv.first[0] == 'O' && kv.first[1] == 'n')
                    return false; // a source output, so it is not inert
            return true;
        }

        // goldsrc counts every brush entity against a hard model limit, while
        // source happily emits one func_brush per piece of scenery. entities
        // that differ only in which brushes they hold are combined into one,
        // which is invisible in game and costs nothing.
        //
        // hammerid was already removed from every entity by the common remap
        // pass. Origin is dropped here because it would otherwise make every
        // entity unique, and goldsrc brush entities do not want an origin at
        // all (csg traces leaks from entity origins, and a centroid landing
        // in open space invents a leak).
        void merge_static_brush_entities(format::map_document &doc, source_result &result)
        {
            std::vector<format::map_entity> kept;
            std::vector<format::map_entity> merged;
            std::vector<std::string> signatures;
            kept.reserve(doc.entities.size());

            for (std::size_t i = 0; i < doc.entities.size(); i++)
            {
                format::map_entity &entity = doc.entities[i];
                if (i == 0 || !is_static_brush_entity(entity))
                {
                    kept.push_back(std::move(entity));
                    continue;
                }

                entity.keyvalues.remove("origin");

                std::string signature;
                for (const format::entity::pair &kv : entity.keyvalues.pairs())
                {
                    signature += kv.first;
                    signature += '\1';
                    signature += kv.second;
                    signature += '\2';
                }

                std::size_t slot = signatures.size();
                for (std::size_t s = 0; s < signatures.size(); s++)
                    if (signatures[s] == signature)
                    {
                        slot = s;
                        break;
                    }
                if (slot == signatures.size())
                {
                    signatures.push_back(std::move(signature));
                    merged.emplace_back();
                    merged.back().keyvalues = std::move(entity.keyvalues);
                }
                for (format::map_brush &brush : entity.brushes)
                    merged[slot].brushes.push_back(std::move(brush));
                result.merged_brush_entities++;
            }

            result.brush_entity_groups = merged.size();
            for (format::map_entity &entity : merged)
                kept.push_back(std::move(entity));
            doc.entities.swap(kept);
        }

        vec3 entity_origin(const format::entity &entity)
        {
            vec3 origin{};
            std::sscanf(entity.value("origin"), "%lf %lf %lf",
                        &origin.x, &origin.y, &origin.z);
            return origin;
        }

        std::string origin_near_player_start(const format::map_document &doc)
        {
            for (const format::map_entity &entity : doc.entities)
            {
                if (lowercase(entity.keyvalues.value("classname"))
                    != "info_player_start")
                    continue;
                vec3 origin = entity_origin(entity.keyvalues);
                origin.x += 32.0;
                char text[96];
                std::snprintf(text, sizeof(text), "%.6g %.6g %.6g",
                              origin.x, origin.y, origin.z);
                return text;
            }
            return {};
        }
    }

    bool port_source_map(const format::source_map_data &map, const source_options &options,
                         source_result &result, std::string *error)
    {
        result = source_result{};
        if (map.models.empty())
        {
            set_error(error, "source map contains no models");
            return false;
        }
        if (map.planes.empty() || map.brushes.empty())
        {
            set_error(error, "source map contains no brush geometry");
            return false;
        }
        std::vector<brush_range> ranges = assign_brush_ranges(map);

        material_catalog catalog;
        catalog.build(map, options.game_dirs, options.max_texture_size,
                      options.full_size_textures);
        result.converted_materials = catalog.converted();
        result.placeholder_materials = catalog.placeholders();
        result.masked_materials = catalog.masked();

        std::vector<format::entity> entities = format::parse_entities(map.entities);
        if (entities.empty())
        {
            set_error(error, "source map contains no worldspawn entity");
            return false;
        }

        world_box box = read_world_box(entities);

        entity_remap_stats spawn_stats;
        remap_source_entities(entities, spawn_stats);
        result.dropped_spawns = spawn_stats.dropped_spawns;
        result.player_start = spawn_stats.player_start;

        entities[0].set("mapversion", "220");
        {
            // the companion wad carries the converted materials, the tool wad the
            // engine textures; a ported map needs both on worldspawn's wad list
            std::string wad = entities[0].value("wad");
            auto append_wad = [&wad](const std::string &path)
            {
                if (path.empty())
                    return;
                if (!wad.empty() && wad.back() != ';')
                    wad.push_back(';');
                wad += path;
            };
            if (!catalog.textures().empty())
                append_wad(options.additional_wad);
            append_wad(options.tool_wad);
            if (!wad.empty())
                entities[0].set("wad", wad.c_str());
        }

        // the source 2d skybox becomes six gfx/env tgas. goldsrc resolves the sky
        // by name out of gfx/env with no subdirectory, so worldspawn is retagged
        // with the flattened name the exporter actually wrote.
        if (options.sky_size != 0)
        {
            std::string sky_name = entities[0].value("skyname");
            if (export_source_skybox(map, options.game_dirs, sky_name, options.sky_size,
                                     options.sky_exposure, result.skybox))
                entities[0].set("skyname", result.skybox.sky_name.c_str());
        }

        format::map_document doc;
        for (std::size_t index = 0; index < entities.size(); index++)
        {
            format::entity &entity = entities[index];
            int model_index = entity_model(entity, index);
            vec3 model_origin =
                model_index >= 1 ? entity_origin(entity) : vec3{};
            if (model_index >= (int)map.models.size())
            {
                set_error(error, "entity references a model outside the model lump");
                return false;
            }
            if (model_index >= 1)
                entity.remove("model");

            doc.entities.emplace_back();
            format::map_entity &out = doc.entities.back();
            out.keyvalues = std::move(entity);

            // worldspawn keeps only structural brushes; detail and window content
            // are routed to entities below. brush entities keep everything.
            if (model_index >= 0 && (std::size_t)model_index < ranges.size())
            {
                const brush_range &range = ranges[(std::size_t)model_index];
                bool masked = false;
                int translucent_amount = -1;
                for (int b = range.first; b < range.last_exclusive; b++)
                {
                    if (b < 0 || (std::size_t)b >= map.brushes.size())
                        continue;
                    int contents = map.brushes[(std::size_t)b].contents;
                    if (model_index == 0
                        && (is_window_brush(contents)
                            || (contents & contents_detail) != 0
                            || water_material(map, catalog,
                                              map.brushes[(std::size_t)b])
                                != nullptr))
                        continue;
                    format::map_brush brush;
                    if (!build_brush(map, b, model_origin, model_index == 0, catalog,
                                     box, result, brush))
                        continue;
                    if (has_masked_material(map, catalog, map.brushes[(std::size_t)b]))
                        masked = true;
                    else if (translucent_amount < 0)
                    {
                        const resolved_material *tm = translucent_material(
                            map, catalog, map.brushes[(std::size_t)b]);
                        if (tm)
                            translucent_amount = tm->render_amount;
                    }
                    out.brushes.push_back(std::move(brush));
                    if (model_index == 0)
                        result.world_brushes++;
                    else
                        result.entity_brushes++;
                }

                // source draws the transparency from the material, goldsrc from
                // the entity, so an entity whose brushes carry a see-through
                // material inherits the matching render keys. an entity that
                // already specifies its own rendermode is left alone.
                if (model_index >= 1)
                {
                    const char *mode = out.keyvalues.value("rendermode");
                    bool opaque = mode[0] == '\0' || std::strcmp(mode, "0") == 0;
                    if (masked)
                    {
                        // rendermode solid keeps every unmasked texel opaque
                        // while the palette's last index is punched out
                        out.keyvalues.set("rendermode", "4");
                        out.keyvalues.set("renderamt", "255");
                        result.masked_entities++;
                    }
                    else if (translucent_amount >= 0 && opaque)
                    {
                        out.keyvalues.set("rendermode", "2");
                        out.keyvalues.set("renderamt",
                                          std::to_string(translucent_amount).c_str());
                        result.translucent_entities++;
                    }
                }
            }
        }

        // the world's detail and translucent brushes become entities: func_detail
        // stays out of the bsp/vis portal computation (zhlt_detaillevel does the
        // marking; the classname alone merges like func_group), and window content
        // becomes func_wall with rendermode texture since goldsrc cannot render
        // world faces translucent. identical signatures share one entity.
        if (!ranges.empty())
        {
            for (int b = ranges[0].first; b < ranges[0].last_exclusive; b++)
            {
                if (b < 0 || (std::size_t)b >= map.brushes.size())
                    continue;
                const format::source_dbrush_t &source_brush =
                    map.brushes[(std::size_t)b];
                const resolved_material *water = water_material(map, catalog, source_brush);
                bool masked = !water && has_masked_material(map, catalog, source_brush);
                bool window = !water && !masked && is_window_brush(source_brush.contents);
                bool detail = !water && !masked && !window
                    && (source_brush.contents & contents_detail) != 0;
                if (!water && !masked && !window && !detail)
                    continue;
                format::map_brush brush;
                if (!build_brush(map, b, vec3{}, true, catalog, box, result, brush))
                    continue;
                if (water)
                {
                    // goldsrc derives brush contents from the texture name, so
                    // every side must carry the '!' water texture; the entity
                    // supplies translucency (and skin -3 keeps water contents
                    // for the engine's point queries)
                    for (format::map_side &side : brush.sides)
                        side.texture = water->name;
                    doc.add_brush(std::move(brush), "func_water",
                                  {{"rendermode", "2"},
                                   {"renderamt", std::to_string(water->render_amount)},
                                   {"skin", "-3"},
                                   {"WaveHeight", "0"}});
                    result.water_brushes++;
                }
                else if (masked)
                {
                    // rendermode solid keeps every non masked texel fully opaque
                    // while the palette's last index is punched out
                    doc.add_brush(std::move(brush), "func_wall",
                                  {{"rendermode", "4"}, {"renderamt", "255"}});
                    result.masked_brushes++;
                }
                else if (window)
                {
                    int amount = window_render_amount(map, catalog, source_brush);
                    doc.add_brush(std::move(brush), "func_wall",
                                  {{"rendermode", "2"},
                                   {"renderamt", std::to_string(amount)}});
                    result.window_brushes++;
                }
                else
                {
                    // zhlt_detaillevel keeps the brushes out of the visual hull's
                    // portal computation; zhlt_clipnodedetaillevel does the same
                    // for the collision hulls 1-3, whose leaf conflicts otherwise
                    // remain (collision itself is preserved either way)
                    doc.add_brush(std::move(brush), "func_detail",
                                  {{"zhlt_detaillevel", "1"},
                                   {"zhlt_clipnodedetaillevel", "1"}});
                    result.detail_brushes++;
                }
            }
        }

        append_displacements(map, catalog, doc, result);

        // studio models: rebuild each one for goldsrc, then place the static
        // props, which live in a game lump that goldsrc has no equivalent for
        // and so have to become entities.
        if (options.convert_models)
        {
            convert_source_models(map, options.game_dirs, result.models);
            for (const prop_placement &prop : result.models.props)
            {
                if (prop.model.empty())
                    continue;
                char origin[96];
                char angles[96];
                std::snprintf(origin, sizeof(origin), "%.6g %.6g %.6g",
                              prop.origin[0], prop.origin[1], prop.origin[2]);
                std::snprintf(angles, sizeof(angles), "%.6g %.6g %.6g",
                              prop.angles[0], prop.angles[1], prop.angles[2]);
                doc.add_entity("cycler_sprite", {{"model", prop.model},
                                                 {"origin", origin},
                                                 {"angles", angles},
                                                 {"framerate", "10"},
                                                 {"rendermode", "0"},
                                                 {"renderamt", "255"}});
                result.prop_entities++;
            }
        }

        merge_static_brush_entities(doc, result);

        // Source's UnlitGeneric shader does not sample a lightmap. RAD consumes
        // this compiler metadata and gives matching faces constant-white
        // lightmaps, reproducing fullbright modulation in the GoldSrc renderer.
        // Give the point entity a useful editor position beside the player start.
        std::set<std::string> unlit_textures;
        for (const std::string &material : map.material_names)
        {
            const resolved_material &resolved = catalog.resolve(material);
            if (resolved.unlit)
                unlit_textures.insert(resolved.name);
        }
        if (!unlit_textures.empty())
        {
            format::map_entity &metadata = doc.add_entity("info_unlittextures");
            std::string origin = origin_near_player_start(doc);
            if (!origin.empty())
                metadata.set("origin", origin);
            for (const std::string &texture : unlit_textures)
                metadata.set(texture.c_str(), "1");
            result.unlit_materials = unlit_textures.size();
        }

        result.entities = doc.entities.size();
        result.sides = doc.side_count();
        for (std::size_t i = 1; i < doc.entities.size(); i++)
            if (!doc.entities[i].brushes.empty())
                result.brush_models++;
        result.text = doc.write();
        result.wad_textures = catalog.textures();

        // every texture the map names has to exist in some wad for csg to size
        // it. the engine textures (NULL, SKY, AAATRIGGER, ...) come from no
        // source material, so they have to be supplied by a tool wad such as
        // sdhlt.wad, named with -toolwad and added to the worldspawn wad list.
        std::set<std::string> present;
        for (const format::mip_texture &texture : result.wad_textures)
            present.insert(lowercase(texture.name));
        for (const format::map_entity &entity : doc.entities)
            for (const format::map_brush &brush : entity.brushes)
                for (const format::map_side &side : brush.sides)
                    if (!present.count(lowercase(side.texture)))
                        result.engine_textures.insert(side.texture);
        return true;
    }
}
