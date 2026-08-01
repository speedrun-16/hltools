#include "model.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <set>

#include "common/binary.h"

namespace format
{
    namespace
    {
        constexpr std::size_t phy_header_size = 16;
        constexpr std::size_t compact_surface_header_size = 32;
        constexpr std::size_t compact_model_size = 48;
        constexpr std::size_t compact_node_size = 28;
        constexpr std::size_t compact_ledge_size = 16;
        constexpr std::size_t compact_triangle_size = 16;
        constexpr std::size_t compact_vertex_size = 16;
        constexpr std::size_t traversal_limit = 1000000;
        constexpr double metres_to_inches = 1.0 / 0.0254;

        bool range(std::size_t at, std::size_t count, std::size_t begin,
                   std::size_t end)
        {
            return at >= begin && at <= end && count <= end - at;
        }

        bool add_offset(std::size_t base, std::int32_t offset, std::size_t begin,
                        std::size_t end, std::size_t &result)
        {
            if (offset >= 0)
            {
                std::size_t amount = (std::size_t)offset;
                if (amount > end - base)
                    return false;
                result = base + amount;
            }
            else
            {
                // widen before negating so INT32_MIN is well defined
                std::uint64_t amount = (std::uint64_t)(-(std::int64_t)offset);
                if (amount > base - begin)
                    return false;
                result = base - (std::size_t)amount;
            }
            return result >= begin && result <= end;
        }

        bool fail(std::string *error, const char *message)
        {
            if (error)
                *error = message;
            return false;
        }

        bool parse_solid(const std::vector<byte> &data, std::size_t solid,
                         std::size_t solid_end, source_phy_model &out,
                         std::string *error)
        {
            binary::reader reader(data);
            if (!range(solid, compact_surface_header_size + compact_model_size,
                       solid, solid_end)
                || std::memcmp(data.data() + solid + 4, "VPHY", 4) != 0)
                return fail(error, "invalid VPHY compact-surface header");

            std::size_t model = solid + compact_surface_header_size;
            if (std::memcmp(data.data() + model + 44, "IVPS", 4) != 0)
                return fail(error, "invalid IVPS compact collision model");

            std::uint32_t tree_offset = 0;
            if (!reader.u32_at(model + 32, tree_offset)
                || tree_offset > solid_end - model)
                return fail(error, "invalid VPHY collision-tree offset");
            std::size_t root = model + tree_offset;
            if (!range(root, compact_node_size, solid, solid_end))
                return fail(error, "truncated VPHY collision tree");

            // every ledge indexes one shared vertex array. the root node's
            // ledge is the tree summary and owns the offset to that array.
            std::int32_t root_ledge_offset = 0;
            if (!reader.i32_at(root + 4, root_ledge_offset))
                return fail(error, "truncated VPHY root node");
            std::size_t root_ledge = 0;
            if (!add_offset(root, root_ledge_offset, solid, solid_end, root_ledge)
                || !range(root_ledge, compact_ledge_size, solid, solid_end))
                return fail(error, "invalid VPHY root ledge");
            std::uint32_t vertex_offset = 0;
            if (!reader.u32_at(root_ledge, vertex_offset)
                || vertex_offset > solid_end - root_ledge)
                return fail(error, "invalid VPHY vertex-array offset");
            std::size_t vertices = root_ledge + vertex_offset;

            std::vector<std::size_t> pending{root};
            std::set<std::size_t> visited;
            while (!pending.empty())
            {
                std::size_t node = pending.back();
                pending.pop_back();
                if (!visited.insert(node).second)
                    return fail(error, "cyclic VPHY collision tree");
                if (visited.size() > traversal_limit
                    || !range(node, compact_node_size, solid, solid_end))
                    return fail(error, "invalid VPHY collision tree");

                std::int32_t right_offset = 0, ledge_offset = 0;
                if (!reader.i32_at(node, right_offset)
                    || !reader.i32_at(node + 4, ledge_offset))
                    return fail(error, "truncated VPHY collision node");

                if (ledge_offset != 0)
                {
                    std::size_t ledge = 0;
                    if (!add_offset(node, ledge_offset, solid, solid_end, ledge)
                        || !range(ledge, compact_ledge_size, solid, solid_end))
                        return fail(error, "invalid VPHY convex ledge");
                    std::uint32_t flags = 0;
                    std::uint16_t triangle_count = 0;
                    if (!reader.u32_at(ledge + 8, flags)
                        || !reader.u16_at(ledge + 12, triangle_count))
                        return fail(error, "truncated VPHY convex ledge");

                    // low flag bits mark summary ledges with child ledges. only
                    // terminal ledges are individual convex collision pieces.
                    if ((flags & 3u) == 0)
                    {
                        std::size_t triangle_bytes =
                            (std::size_t)triangle_count * compact_triangle_size;
                        if (!range(ledge + compact_ledge_size, triangle_bytes,
                                   solid, solid_end))
                            return fail(error, "truncated VPHY triangle list");

                        source_phy_convex convex;
                        std::vector<std::uint16_t> source_ids;
                        for (unsigned t = 0; t < triangle_count; t++)
                        {
                            std::size_t triangle = ledge + compact_ledge_size
                                + (std::size_t)t * compact_triangle_size;
                            std::array<unsigned, 3> local{};
                            const std::size_t id_offsets[3] = {4, 8, 12};
                            for (int corner = 0; corner < 3; corner++)
                            {
                                std::int16_t signed_id = 0;
                                if (!reader.i16_at(triangle + id_offsets[corner], signed_id)
                                    || signed_id < 0)
                                    return fail(error, "invalid VPHY vertex index");
                                std::uint16_t id = (std::uint16_t)signed_id;
                                unsigned local_id = 0;
                                for (; local_id < source_ids.size(); local_id++)
                                    if (source_ids[local_id] == id)
                                        break;
                                if (local_id == source_ids.size())
                                {
                                    std::size_t vertex = vertices
                                        + (std::size_t)id * compact_vertex_size;
                                    if (!range(vertex, compact_vertex_size, solid, solid_end))
                                        return fail(error, "VPHY vertex index is out of range");
                                    float x = 0, y = 0, z = 0;
                                    if (!reader.f32_at(vertex, x)
                                        || !reader.f32_at(vertex + 4, y)
                                        || !reader.f32_at(vertex + 8, z)
                                        || !std::isfinite(x) || !std::isfinite(y)
                                        || !std::isfinite(z))
                                        return fail(error, "truncated VPHY vertex array");
                                    // coordinates use IVP metres in X/Z/-Y order.
                                    convex.vertices.push_back({x * metres_to_inches,
                                                               z * metres_to_inches,
                                                              -y * metres_to_inches});
                                    source_ids.push_back(id);
                                }
                                local[corner] = local_id;
                            }
                            convex.triangles.push_back(local);
                        }
                        if (!convex.triangles.empty())
                            out.convexes.push_back(std::move(convex));
                    }
                }

                if (right_offset != 0)
                {
                    std::size_t right = 0;
                    if (right_offset < 0
                        || !add_offset(node, right_offset, solid, solid_end, right)
                        || !range(node + compact_node_size, compact_node_size,
                                  solid, solid_end))
                        return fail(error, "invalid VPHY child-node offset");
                    pending.push_back(right);
                    pending.push_back(node + compact_node_size);
                }
            }
            return true;
        }
    }

    bool load_source_phy(const std::vector<byte> &data, source_phy_model &out,
                         std::string *error)
    {
        out = source_phy_model{};
        if (error)
            error->clear();
        if (data.size() < phy_header_size)
            return fail(error, "truncated Source PHY header");

        binary::reader reader(data);
        std::int32_t header_size = 0, solid_count = 0;
        if (!reader.i32_at(0, header_size) || header_size != (int)phy_header_size
            || !reader.i32_at(8, solid_count) || solid_count < 0
            || solid_count > 65536)
            return fail(error, "invalid Source PHY header");

        std::size_t solid = phy_header_size;
        for (int i = 0; i < solid_count; i++)
        {
            std::uint32_t size = 0;
            if (solid > data.size() || data.size() - solid < 4
                || !reader.u32_at(solid, size) || size < 76
                || size > data.size() - solid - 4)
                return fail(error, "invalid Source PHY solid size");
            std::size_t end = solid + 4 + size;
            if (!parse_solid(data, solid, end, out, error))
            {
                out = source_phy_model{};
                return false;
            }
            solid = end;
        }
        return true;
    }
}
