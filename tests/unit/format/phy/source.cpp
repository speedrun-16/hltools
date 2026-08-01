#include <cmath>
#include <cstring>
#include <vector>

#include "common/binary.h"
#include "format/phy/source/model.h"
#include "support/test.h"

namespace
{
    void raw(binary::writer &writer, const char *text, std::size_t size)
    {
        writer.raw(reinterpret_cast<const byte *>(text), size);
    }

    void triangle(binary::writer &writer, int a, int b, int c)
    {
        writer.u32(0);
        writer.i16((std::int16_t)a); writer.i16(0);
        writer.i16((std::int16_t)b); writer.i16(0);
        writer.i16((std::int16_t)c); writer.i16(0);
    }

    std::vector<byte> box_phy()
    {
        std::vector<byte> bytes;
        binary::writer writer(bytes);
        writer.i32(16); writer.i32(0); writer.i32(1); writer.i32(1234);
        std::size_t solid = writer.position();
        writer.u32(0);
        raw(writer, "VPHY", 4);
        writer.u16(0x100); writer.u16(0);
        writer.u32(412);
        writer.f32(1); writer.f32(1); writer.f32(1);
        writer.u32(0);

        for (int i = 0; i < 7; i++) writer.f32(0);
        writer.u32(0);
        writer.u32(384); // tree follows its summary ledge and shared vertices
        writer.u32(0); writer.u32(0);
        raw(writer, "IVPS", 4);

        writer.u32(208); // vertex array follows 12 compact triangles
        writer.u32(0); writer.u32(0);
        writer.u16(12); writer.u16(0);
        triangle(writer, 0, 2, 1); triangle(writer, 1, 2, 3);
        triangle(writer, 4, 5, 6); triangle(writer, 5, 7, 6);
        triangle(writer, 0, 1, 4); triangle(writer, 1, 5, 4);
        triangle(writer, 2, 6, 3); triangle(writer, 3, 6, 7);
        triangle(writer, 0, 4, 2); triangle(writer, 2, 4, 6);
        triangle(writer, 1, 3, 5); triangle(writer, 3, 7, 5);

        // coordinates use IVP metres and axes: Source = (x, z, -y) / 0.0254.
        constexpr float unit = 0.0254f;
        for (int z = 0; z < 2; z++)
            for (int y = 0; y < 2; y++)
                for (int x = 0; x < 2; x++)
                {
                    writer.f32(x * unit);
                    writer.f32(-y * unit);
                    writer.f32(z * unit);
                    writer.f32(0);
                }

        writer.i32(0);    // one-node tree
        writer.i32(-336); // terminal/root ledge precedes the tree
        writer.f32(0); writer.f32(0); writer.f32(0); writer.f32(1);
        writer.u8(0); writer.u8(0); writer.u8(0); writer.u8(0);
        writer.patch_u32(solid, (std::uint32_t)(writer.position() - solid - 4));
        return bytes;
    }
}

suite("unit.format.source_phy")
{
    test("reads a terminal IVP convex and converts metres and axes")
    {
        format::source_phy_model model;
        std::string error;
        require(format::load_source_phy(box_phy(), model, &error));
        expect(error.empty());
        require(model.convexes.size() == 1);
        expect(model.convexes[0].vertices.size() == 8);
        expect(model.convexes[0].triangles.size() == 12);

        bool found = false;
        for (const auto &vertex : model.convexes[0].vertices)
            if (std::fabs(vertex.x - 1) < 0.0001
                && std::fabs(vertex.y - 1) < 0.0001
                && std::fabs(vertex.z - 1) < 0.0001)
                found = true;
        expect(found);
    }

    test("rejects a truncated physics tree without partial output")
    {
        std::vector<byte> bytes = box_phy();
        bytes.resize(100);
        format::source_phy_model model;
        model.convexes.emplace_back();
        std::string error;
        expect_false(format::load_source_phy(bytes, model, &error));
        expect(model.convexes.empty());
        expect_false(error.empty());
    }
}
