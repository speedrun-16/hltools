#include <cstdint>
#include <string>
#include <vector>

#include "common/binary.h"
#include "common/types.h"
#include "decompile/source/skybox.h"
#include "format/vbsp/data.h"
#include "support/test.h"

namespace
{
    void pad(binary::writer &sink, std::size_t count)
    {
        for (std::size_t i = 0; i < count; i++)
            sink.u8(0);
    }

    void text(binary::writer &sink, const std::string &value)
    {
        sink.raw(reinterpret_cast<const byte *>(value.data()), value.size());
    }

    // a 4x4 rgb888 vtf whose top row is red and whose remaining rows are blue,
    // so the exported tga pins both the row order and the channel order.
    std::vector<byte> make_vtf()
    {
        std::vector<byte> vtf;
        binary::writer sink(vtf);
        text(sink, "VTF");
        sink.u8(0);
        sink.u32(7);  // version major
        sink.u32(2);  // version minor
        sink.u32(80); // header size
        sink.u16(4);  // width
        sink.u16(4);  // height
        sink.u32(0);  // flags
        sink.u16(1);  // frames
        sink.u16(0);  // first frame
        pad(sink, 4);
        sink.f32(1);
        sink.f32(1);
        sink.f32(1); // reflectivity
        pad(sink, 4);
        sink.f32(1);  // bump scale
        sink.i32(2);  // high res format: RGB888
        sink.u8(1);   // one mip level
        sink.i32(-1); // no low res image
        sink.u8(0);   // low res width
        sink.u8(0);   // low res height
        sink.u16(1);  // depth
        pad(sink, 15); // out to the declared 80 byte header

        for (int y = 0; y < 4; y++)
        {
            for (int x = 0; x < 4; x++)
            {
                sink.u8((byte)(y == 0 ? 255 : 0)); // r
                sink.u8(0);                        // g
                sink.u8((byte)(y == 0 ? 0 : 255)); // b
            }
        }
        return vtf;
    }

    struct zip_entry
    {
        std::string name;
        std::vector<byte> data;
    };

    // a minimal stored-only zip, the shape a source pakfile lump takes
    std::vector<byte> make_zip(const std::vector<zip_entry> &entries)
    {
        std::vector<byte> zip;
        binary::writer sink(zip);

        std::vector<std::size_t> local_offsets;
        for (const zip_entry &entry : entries)
        {
            local_offsets.push_back(sink.position());
            sink.u32(0x04034b50);
            sink.u16(20); // version needed
            sink.u16(0);  // flags
            sink.u16(0);  // method: stored
            sink.u16(0);  // mod time
            sink.u16(0);  // mod date
            sink.u32(0);  // crc32, not verified by the reader
            sink.u32((std::uint32_t)entry.data.size());
            sink.u32((std::uint32_t)entry.data.size());
            sink.u16((std::uint16_t)entry.name.size());
            sink.u16(0); // extra length
            text(sink, entry.name);
            sink.raw(entry.data);
        }

        std::size_t central = sink.position();
        for (std::size_t i = 0; i < entries.size(); i++)
        {
            const zip_entry &entry = entries[i];
            sink.u32(0x02014b50);
            sink.u16(20); // version made by
            sink.u16(20); // version needed
            sink.u16(0);  // flags
            sink.u16(0);  // method: stored
            sink.u16(0);  // mod time
            sink.u16(0);  // mod date
            sink.u32(0);  // crc32
            sink.u32((std::uint32_t)entry.data.size());
            sink.u32((std::uint32_t)entry.data.size());
            sink.u16((std::uint16_t)entry.name.size());
            sink.u16(0); // extra
            sink.u16(0); // comment
            sink.u16(0); // disk number
            sink.u16(0); // internal attributes
            sink.u32(0); // external attributes
            sink.u32((std::uint32_t)local_offsets[i]);
            text(sink, entry.name);
        }
        std::size_t central_size = sink.position() - central;

        sink.u32(0x06054b50);
        sink.u16(0);
        sink.u16(0);
        sink.u16((std::uint16_t)entries.size());
        sink.u16((std::uint16_t)entries.size());
        sink.u32((std::uint32_t)central_size);
        sink.u32((std::uint32_t)central);
        sink.u16(0);
        return zip;
    }

    std::vector<byte> vmt_text(const char *basetexture)
    {
        std::string value = std::string("\"UnlitGeneric\"\n{\n\t\"$basetexture\" \"")
            + basetexture + "\"\n}\n";
        return std::vector<byte>(value.begin(), value.end());
    }
}

suite("unit.decompile.source_skybox")
{
    test("source_skybox.exports a face as a bottom-up 24 bit tga")
    {
        format::source_map_data map;
        map.pakfile = make_zip({
            {"materials/skybox/skyauthor/testsky_bk.vmt",
             vmt_text("skybox/skyauthor/testsky_b")},
            {"materials/skybox/skyauthor/testsky_b.vtf", make_vtf()},
        });

        decompile::skybox_result sky;
        require(decompile::export_source_skybox(
            map, {}, "skyauthor/testsky_", 4, 1.0, sky));

        // goldsrc resolves the sky out of gfx/env with no subdirectory
        expect(sky.sky_name == "testsky_");
        require(sky.faces.size() == 1);
        expect(sky.missing == 5); // only the bk face is present in the fixture
        expect(sky.faces[0].filename == "testsky_bk.tga");

        const std::vector<byte> &tga = sky.faces[0].tga;
        require(tga.size() == 18 + 4u * 4u * 3u);
        expect(tga[0] == 0);  // no id field
        expect(tga[1] == 0);  // no colour map
        expect(tga[2] == 2);  // uncompressed truecolour
        expect(tga[12] == 4); // width
        expect(tga[13] == 0);
        expect(tga[14] == 4); // height
        expect(tga[15] == 0);
        expect(tga[16] == 24); // bits per pixel
        expect(tga[17] == 0);  // origin bottom left, matching the stock skies

        // the file's first row is the image's bottom row: blue, stored bgr
        expect(tga[18 + 0] == 255);
        expect(tga[18 + 1] == 0);
        expect(tga[18 + 2] == 0);

        // the file's last row is the image's top row: red, stored bgr
        std::size_t last = 18 + (std::size_t)3 * 4 * 3;
        expect(tga[last + 0] == 0);
        expect(tga[last + 1] == 0);
        expect(tga[last + 2] == 255);
    }

    test("source_skybox.applies explicit linear exposure")
    {
        format::source_map_data map;
        map.pakfile = make_zip({
            {"materials/skybox/test_bk.vmt", vmt_text("skybox/test_b")},
            {"materials/skybox/test_b.vtf", make_vtf()},
        });

        decompile::skybox_result sky;
        require(decompile::export_source_skybox(
            map, {}, "test_", 4, 0.5, sky));
        require(sky.faces.size() == 1);

        const std::vector<byte> &tga = sky.faces[0].tga;
        expect(tga[18 + 0] == 128); // blue bottom row after half exposure
        expect(tga[18 + 1] == 0);
        expect(tga[18 + 2] == 0);
    }

    test("source_skybox.reports no sky when the map declares none")
    {
        format::source_map_data map;
        decompile::skybox_result sky;
        expect_false(decompile::export_source_skybox(
            map, {}, "", 256, 1.0, sky));
        expect(sky.faces.empty());
    }
}
