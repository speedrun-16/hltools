#include "format/mdl/chunk_skins.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <tuple>

namespace format
{
    namespace
    {
        studio_vertex lerp_vertex(const studio_vertex &a, const studio_vertex &b, float t)
        {
            studio_vertex out = a;
            for (int k = 0; k < 3; k++)
            {
                out.position[k] = a.position[k] + (b.position[k] - a.position[k]) * t;
                out.normal[k] = a.normal[k] + (b.normal[k] - a.normal[k]) * t;
            }
            float normal_length = std::sqrt(out.normal[0] * out.normal[0]
                                          + out.normal[1] * out.normal[1]
                                          + out.normal[2] * out.normal[2]);
            if (normal_length > 1e-12f)
                for (int k = 0; k < 3; k++)
                    out.normal[k] /= normal_length;
            out.u = a.u + (b.u - a.u) * t;
            out.v = a.v + (b.v - a.v) * t;
            // goldsrc skinning is rigid, so a clipped vertex keeps the bone of
            // the edge it was cut from rather than blending ownership
            out.bone = a.bone;
            return out;
        }

        float coord(const studio_vertex &vertex, int axis)
        {
            return axis == 0 ? vertex.u : vertex.v;
        }

        // sutherland-hodgman against one axis aligned half plane. keep_upper
        // retains the side with coord >= value.
        void clip_half_plane(const std::vector<studio_vertex> &in, int axis, float value,
                             bool keep_upper, std::vector<studio_vertex> &out)
        {
            out.clear();
            if (in.empty())
                return;
            auto inside = [&](const studio_vertex &v)
            {
                float c = coord(v, axis);
                return keep_upper ? c >= value : c <= value;
            };
            for (std::size_t i = 0; i < in.size(); i++)
            {
                const studio_vertex &current = in[i];
                const studio_vertex &next = in[(i + 1) % in.size()];
                bool in_current = inside(current);
                bool in_next = inside(next);
                if (in_current)
                    out.push_back(current);
                if (in_current != in_next)
                {
                    float a = coord(current, axis);
                    float b = coord(next, axis);
                    float span = b - a;
                    if (std::fabs(span) > 1e-12f)
                    {
                        // always interpolate from the lower endpoint. adjacent
                        // triangles commonly traverse a shared edge in opposite
                        // directions; canonical ordering makes the cut vertex
                        // bit-identical on both sides so it can be reused.
                        if (a < b)
                            out.push_back(lerp_vertex(current, next,
                                                      (value - a) / span));
                        else
                            out.push_back(lerp_vertex(next, current,
                                                      (value - b) / -span));
                    }
                }
            }
        }

        // a vertex is reused only on an exact match, so clipped edges shared by
        // two triangles collapse back to one vertex instead of tearing
        struct vertex_key
        {
            float data[8];
            int bone;
            bool operator<(const vertex_key &other) const
            {
                for (int i = 0; i < 8; i++)
                {
                    if (data[i] < other.data[i]) return true;
                    if (data[i] > other.data[i]) return false;
                }
                return bone < other.bone;
            }
        };

        vertex_key key_of(const studio_vertex &v)
        {
            vertex_key key{};
            for (int k = 0; k < 3; k++)
            {
                key.data[k] = v.position[k];
                key.data[3 + k] = v.normal[k];
            }
            key.data[6] = v.u;
            key.data[7] = v.v;
            key.bone = v.bone;
            return key;
        }

        // the last tile of a row is pulled back so it ends at the image edge,
        // overlapping its neighbour instead of running past it. every tile is
        // therefore exactly the same size in texels, which is what keeps each
        // crop 1:1 with the source.
        float tile_origin(float core, int index)
        {
            float origin = (float)index * core;
            float last = 1.0f - core;
            if (last < 0.0f)
                last = 0.0f;
            return origin > last ? last : origin;
        }

        double polygon_area(const std::vector<studio_vertex> &poly)
        {
            if (poly.size() < 3)
                return 0;
            double area = 0;
            const studio_vertex &a = poly[0];
            for (std::size_t i = 1; i + 1 < poly.size(); i++)
            {
                const studio_vertex &b = poly[i];
                const studio_vertex &c = poly[i + 1];
                double e1[3], e2[3];
                for (int k = 0; k < 3; k++)
                {
                    e1[k] = b.position[k] - a.position[k];
                    e2[k] = c.position[k] - a.position[k];
                }
                double x = e1[1] * e2[2] - e1[2] * e2[1];
                double y = e1[2] * e2[0] - e1[0] * e2[2];
                double z = e1[0] * e2[1] - e1[1] * e2[0];
                area += 0.5 * std::sqrt(x * x + y * y + z * z);
            }
            return area;
        }
    }

    void chunk_model_skins(studio_model &model,
                           const std::vector<tile_layout> &layout_for_material,
                           float area_keep, std::vector<skin_chunk> &out_chunks)
    {
        out_chunks.clear();
        area_keep = std::max(0.0f, std::min(1.0f, area_keep));

        // rank tiles by the surface area that lands in them, so the ones that
        // barely show can share one untiled skin instead of each taking a tile
        std::map<std::tuple<int, int, int>, double> tile_area;
        if (area_keep < 1.0f)
        {
            std::vector<studio_vertex> poly, scratch;
            for (const studio_mesh &mesh : model.meshes)
            {
                int material = mesh.material;
                tile_layout layout;
                if (material >= 0 && (std::size_t)material < layout_for_material.size())
                    layout = layout_for_material[(std::size_t)material];
                if (layout.count_u <= 1 && layout.count_v <= 1)
                    continue;
                for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3)
                {
                    const studio_vertex &a = model.vertices[(std::size_t)mesh.indices[i]];
                    const studio_vertex &b = model.vertices[(std::size_t)mesh.indices[i + 1]];
                    const studio_vertex &c = model.vertices[(std::size_t)mesh.indices[i + 2]];
                    float min_u = std::min(a.u, std::min(b.u, c.u));
                    float max_u = std::max(a.u, std::max(b.u, c.u));
                    float min_v = std::min(a.v, std::min(b.v, c.v));
                    float max_v = std::max(a.v, std::max(b.v, c.v));
                    if (min_u < 0.0f || max_u > 1.0f
                        || min_v < 0.0f || max_v > 1.0f)
                        continue; // this triangle uses the untiled fallback

                    int first_u = std::max(0, (int)std::floor(min_u / layout.core_u));
                    int last_u = std::min(layout.count_u - 1,
                                          (int)std::floor((max_u - 1e-6f)
                                                          / layout.core_u));
                    int first_v = std::max(0, (int)std::floor(min_v / layout.core_v));
                    int last_v = std::min(layout.count_v - 1,
                                          (int)std::floor((max_v - 1e-6f)
                                                          / layout.core_v));
                    if (last_u < first_u) last_u = first_u;
                    if (last_v < first_v) last_v = first_v;
                    for (int tu = first_u; tu <= last_u; tu++)
                        for (int tv = first_v; tv <= last_v; tv++)
                        {
                            float hi_u = tu + 1 >= layout.count_u
                                ? 1.0f : (float)(tu + 1) * layout.core_u;
                            float hi_v = tv + 1 >= layout.count_v
                                ? 1.0f : (float)(tv + 1) * layout.core_v;
                            poly = {a, b, c};
                            clip_half_plane(poly, 0, (float)tu * layout.core_u,
                                            true, scratch);
                            clip_half_plane(scratch, 0, hi_u, false, poly);
                            clip_half_plane(poly, 1, (float)tv * layout.core_v,
                                            true, scratch);
                            clip_half_plane(scratch, 1, hi_v, false, poly);
                            double area = polygon_area(poly);
                            if (area > 1e-12)
                                tile_area[std::make_tuple(material, tu, tv)] += area;
                        }
                }
            }
        }

        // keep the largest tiles until they cover area_keep of each material
        std::map<int, double> material_total;
        for (const auto &kv : tile_area)
            material_total[std::get<0>(kv.first)] += kv.second;
        std::map<std::tuple<int, int, int>, bool> keep_tile;
        {
            std::map<int, std::vector<std::pair<double, std::tuple<int, int, int>>>> ranked;
            for (const auto &kv : tile_area)
                ranked[std::get<0>(kv.first)].push_back({kv.second, kv.first});
            for (auto &entry : ranked)
            {
                std::sort(entry.second.begin(), entry.second.end(),
                          [](const auto &x, const auto &y)
                          {
                              if (x.first != y.first)
                                  return x.first > y.first;
                              return x.second < y.second;
                          });
                double budget = material_total[entry.first] * (double)area_keep;
                double acc = 0;
                for (const auto &item : entry.second)
                {
                    bool keep = acc < budget;
                    keep_tile[item.second] = keep;
                    acc += item.first;
                }
            }
        }
        auto tile_kept = [&](int material, int tu, int tv)
        {
            if (area_keep >= 1.0f)
                return true;
            auto found = keep_tile.find(std::make_tuple(material, tu, tv));
            return found != keep_tile.end() && found->second;
        };

        std::vector<studio_vertex> vertices;
        std::map<vertex_key, int> seen;
        auto add_vertex = [&](const studio_vertex &v)
        {
            vertex_key key = key_of(v);
            auto found = seen.find(key);
            if (found != seen.end())
                return found->second;
            int index = (int)vertices.size();
            vertices.push_back(v);
            seen.emplace(key, index);
            return index;
        };

        // (source material, tile) -> new material index, so tiles are allocated
        // in a stable order and only when a triangle actually lands in one
        // the fourth component distinguishes the whole-image fallback from tile
        // (0,0). without it, -tilearea could make both geometries share whichever
        // texture happened to be created first.
        std::map<std::tuple<int, int, int, bool>, int> chunk_index;
        std::vector<std::string> materials;
        std::vector<studio_mesh> meshes;
        auto mesh_for = [&](int source_material, const tile_layout &layout,
                            int tu, int tv, bool fallback = false) -> studio_mesh &
        {
            auto key = std::make_tuple(source_material, tu, tv, fallback);
            auto found = chunk_index.find(key);
            if (found == chunk_index.end())
            {
                int index = (int)materials.size();
                chunk_index.emplace(key, index);
                std::string name = (std::size_t)source_material < model.materials.size()
                    ? model.materials[(std::size_t)source_material]
                    : std::string("skin");
                if (layout.count_u > 1 || layout.count_v > 1)
                    name += "_" + std::to_string(tu) + std::to_string(tv);
                materials.push_back(name);
                skin_chunk chunk;
                chunk.source_material = source_material;
                chunk.count_u = layout.count_u;
                chunk.count_v = layout.count_v;
                chunk.tile_u = tu;
                chunk.tile_v = tv;
                chunk.core_u = layout.core_u;
                chunk.core_v = layout.core_v;
                chunk.origin_u = tile_origin(layout.core_u, tu);
                chunk.origin_v = tile_origin(layout.core_v, tv);
                out_chunks.push_back(chunk);
                studio_mesh mesh;
                mesh.material = index;
                meshes.push_back(mesh);
                found = chunk_index.find(key);
            }
            return meshes[(std::size_t)found->second];
        };

        std::vector<studio_vertex> poly, scratch;
        for (const studio_mesh &mesh : model.meshes)
        {
            int source_material = mesh.material;
            tile_layout layout;
            if (source_material >= 0
                && (std::size_t)source_material < layout_for_material.size())
                layout = layout_for_material[(std::size_t)source_material];
            if (layout.count_u < 1)
                layout.count_u = 1;
            if (layout.count_v < 1)
                layout.count_v = 1;

            for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3)
            {
                const studio_vertex &a = model.vertices[(std::size_t)mesh.indices[i]];
                const studio_vertex &b = model.vertices[(std::size_t)mesh.indices[i + 1]];
                const studio_vertex &c = model.vertices[(std::size_t)mesh.indices[i + 2]];

                if (layout.count_u == 1 && layout.count_v == 1)
                {
                    studio_mesh &target = mesh_for(source_material, layout, 0, 0);
                    target.indices.push_back(add_vertex(a));
                    target.indices.push_back(add_vertex(b));
                    target.indices.push_back(add_vertex(c));
                    continue;
                }

                // triangles are assigned by an even division of uv space; only
                // the tile IMAGE is pulled back at the end of a row, so the cut
                // lines stay uniform and a shared edge is still cut identically
                // from both sides
                float u0 = std::min(a.u, std::min(b.u, c.u));
                float u1 = std::max(a.u, std::max(b.u, c.u));
                float v0 = std::min(a.v, std::min(b.v, c.v));
                float v1 = std::max(a.v, std::max(b.v, c.v));
                // wrapped or intentionally overscanned UVs cannot be represented
                // by a finite tile grid without duplicating wrap cells. keep the
                // entire triangle on the untiled fallback instead of clipping
                // away the part outside [0,1].
                if (u0 < 0.0f || u1 > 1.0f || v0 < 0.0f || v1 > 1.0f)
                {
                    tile_layout whole;
                    studio_mesh &target = mesh_for(source_material, whole, 0, 0, true);
                    target.indices.push_back(add_vertex(a));
                    target.indices.push_back(add_vertex(b));
                    target.indices.push_back(add_vertex(c));
                    continue;
                }

                int first_u = std::max(0, (int)std::floor(u0 / layout.core_u));
                int last_u = std::min(layout.count_u - 1,
                                      (int)std::floor((u1 - 1e-6f) / layout.core_u));
                int first_v = std::max(0, (int)std::floor(v0 / layout.core_v));
                int last_v = std::min(layout.count_v - 1,
                                      (int)std::floor((v1 - 1e-6f) / layout.core_v));
                if (last_u < first_u) last_u = first_u;
                if (last_v < first_v) last_v = first_v;

                for (int tu = first_u; tu <= last_u; tu++)
                    for (int tv = first_v; tv <= last_v; tv++)
                    {
                        float hi_u = tu + 1 >= layout.count_u
                            ? 1.0f : (float)(tu + 1) * layout.core_u;
                        float hi_v = tv + 1 >= layout.count_v
                            ? 1.0f : (float)(tv + 1) * layout.core_v;
                        poly = {a, b, c};
                        clip_half_plane(poly, 0, (float)tu * layout.core_u, true, scratch);
                        clip_half_plane(scratch, 0, hi_u, false, poly);
                        clip_half_plane(poly, 1, (float)tv * layout.core_v, true, scratch);
                        clip_half_plane(scratch, 1, hi_v, false, poly);
                        if (polygon_area(poly) <= 1e-12)
                            continue;

                        if (!tile_kept(source_material, tu, tv))
                        {
                            // keeps its original uvs and shares the material's
                            // one untiled skin
                            tile_layout whole;
                            studio_mesh &coarse =
                                mesh_for(source_material, whole, 0, 0, true);
                            int first = add_vertex(poly[0]);
                            for (std::size_t k = 1; k + 1 < poly.size(); k++)
                            {
                                coarse.indices.push_back(first);
                                coarse.indices.push_back(add_vertex(poly[k]));
                                coarse.indices.push_back(add_vertex(poly[k + 1]));
                            }
                            continue;
                        }

                        // into the tile's content region, then inside its border
                        float ou = tile_origin(layout.core_u, tu);
                        float ov = tile_origin(layout.core_v, tv);
                        float span_u = 1.0f - 2.0f * layout.inset_u;
                        float span_v = 1.0f - 2.0f * layout.inset_v;
                        for (studio_vertex &vertex : poly)
                        {
                            float lu = (vertex.u - ou) / layout.core_u;
                            float lv = (vertex.v - ov) / layout.core_v;
                            lu = std::min(1.0f, std::max(0.0f, lu));
                            lv = std::min(1.0f, std::max(0.0f, lv));
                            vertex.u = layout.inset_u + lu * span_u;
                            vertex.v = layout.inset_v + lv * span_v;
                        }

                        studio_mesh &target = mesh_for(source_material, layout, tu, tv);
                        int base = add_vertex(poly[0]);
                        for (std::size_t k = 1; k + 1 < poly.size(); k++)
                        {
                            target.indices.push_back(base);
                            target.indices.push_back(add_vertex(poly[k]));
                            target.indices.push_back(add_vertex(poly[k + 1]));
                        }
                    }
            }
        }

        // a tile that ended up with no triangles would still cost a skin
        std::vector<studio_mesh> kept_meshes;
        std::vector<std::string> kept_materials;
        std::vector<skin_chunk> kept_chunks;
        for (std::size_t i = 0; i < meshes.size(); i++)
        {
            if (meshes[i].indices.empty())
                continue;
            studio_mesh mesh = meshes[i];
            mesh.material = (int)kept_materials.size();
            kept_materials.push_back(materials[i]);
            kept_chunks.push_back(out_chunks[i]);
            kept_meshes.push_back(std::move(mesh));
        }

        model.vertices.swap(vertices);
        model.meshes.swap(kept_meshes);
        model.materials.swap(kept_materials);
        out_chunks.swap(kept_chunks);
    }
}
