#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "common/binary.h"
#include "common/filesystem.h"
#include "common/types.h"
#include "decompile/source/materials.h"
#include "format/vbsp/data.h"
#include "support/scratch.h"
#include "support/test.h"

namespace
{
    void pad(binary::writer &sink, std::size_t count)
    {
        for (std::size_t i = 0; i < count; i++)
            sink.u8(0);
    }

    std::vector<byte> make_rgb_vtf(unsigned dimension)
    {
        std::vector<byte> data;
        binary::writer sink(data);
        sink.raw(reinterpret_cast<const byte *>("VTF"), 3);
        sink.u8(0);
        sink.u32(7);
        sink.u32(2);
        sink.u32(80);
        sink.u16((std::uint16_t)dimension);
        sink.u16((std::uint16_t)dimension);
        sink.u32(0);
        sink.u16(1);
        sink.u16(0);
        pad(sink, 4);
        sink.f32(1);
        sink.f32(1);
        sink.f32(1);
        pad(sink, 4);
        sink.f32(1);
        sink.i32(2);  // RGB888
        sink.u8(1);   // one mip level
        sink.i32(-1); // no low-resolution image
        sink.u8(0);
        sink.u8(0);
        sink.u16(1);
        pad(sink, 15);
        for (unsigned pixel = 0; pixel < dimension * dimension; pixel++)
        {
            sink.u8(96);
            sink.u8(128);
            sink.u8(160);
        }
        return data;
    }

    void write_material(const std::filesystem::path &path, const char *shader)
    {
        std::string text = std::string("\"") + shader + "\"\n{\n"
            "\t\"$basetexture\" \"custom/missing\"\n"
            "\t\"$selfillum\" \"1\"\n"
            "}\n";
        require(fs::write_all(path.string(), text.data(), text.size()));
    }
}

suite("unit.decompile.source_materials")
{
    test("material_catalog distinguishes UnlitGeneric from self illuminated lightmapped materials")
    {
        std::filesystem::path game =
            test_support::scratch_directory("source_materials_unlit");
        std::filesystem::path materials = game / "materials" / "custom";
        require(fs::make_directory(materials.string()));
        write_material(materials / "unlit.vmt", "UnlitGeneric");
        write_material(materials / "masked_glow.vmt", "LightmappedGeneric");

        format::source_map_data map;
        map.material_names = {"custom/unlit", "custom/masked_glow"};

        decompile::material_catalog catalog;
        catalog.build(map, {game.string()});

        expect(catalog.resolve("CUSTOM/UNLIT").unlit);
        expect_false(catalog.resolve("custom/masked_glow").unlit);
    }

    test("material_catalog reduces texture dimensions and UV vectors together")
    {
        std::filesystem::path game =
            test_support::scratch_directory("source_materials_max_texture");
        std::filesystem::path materials = game / "materials" / "custom";
        require(fs::make_directory(materials.string()));
        write_material(materials / "scaled.vmt", "LightmappedGeneric");
        std::vector<byte> vtf = make_rgb_vtf(32);
        require(fs::write_all((materials / "missing.vtf").string(),
                              vtf.data(), vtf.size()));

        format::source_map_data map;
        map.material_names = {"custom/scaled"};

        decompile::material_catalog catalog;
        catalog.build(map, {game.string()}, 16);

        const decompile::resolved_material &material =
            catalog.resolve("custom/scaled");
        require(material.has_texture);
        expect(material.u_scale == 0.5);
        expect(material.v_scale == 0.5);
        require(catalog.textures().size() == 1);
        expect(catalog.textures()[0].width == 16);
        expect(catalog.textures()[0].height == 16);
    }

    test("material_catalog can preserve selected textures at native size")
    {
        std::filesystem::path game =
            test_support::scratch_directory("source_materials_full_texture");
        std::filesystem::path materials = game / "materials" / "custom";
        require(fs::make_directory(materials.string()));
        write_material(materials / "logo.vmt", "LightmappedGeneric");
        std::vector<byte> vtf = make_rgb_vtf(32);
        require(fs::write_all((materials / "missing.vtf").string(),
                              vtf.data(), vtf.size()));

        format::source_map_data map;
        map.material_names = {"custom/logo"};

        decompile::material_catalog catalog;
        catalog.build(map, {game.string()}, 16, {"{LOGO"});

        const decompile::resolved_material &material =
            catalog.resolve("custom/logo");
        require(material.has_texture);
        expect(material.u_scale == 1.0);
        expect(material.v_scale == 1.0);
        require(catalog.textures().size() == 1);
        expect(catalog.textures()[0].width == 32);
        expect(catalog.textures()[0].height == 32);
    }
}
