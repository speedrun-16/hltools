#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "common/binary.h"
#include "common/filesystem.h"
#include "format/mdl/studio_model.h"
#include "model/skybox_model.h"
#include "support/test.h"

namespace
{
    model::rgb_image face(unsigned size, byte seed)
    {
        model::rgb_image image;
        image.width = image.height = size;
        image.rgb.resize((std::size_t)size * size * 3);
        for (unsigned y = 0; y < size; y++)
            for (unsigned x = 0; x < size; x++)
            {
                std::size_t at = ((std::size_t)y * size + x) * 3;
                image.rgb[at + 0] = (byte)(seed + x);
                image.rgb[at + 1] = (byte)(seed + y);
                image.rgb[at + 2] = seed;
            }
        return image;
    }

    std::int32_t i32_at(const std::vector<byte> &data, std::size_t at)
    {
        std::int32_t value = 0;
        binary::reader(data).i32_at(at, value);
        return value;
    }

    std::int16_t i16_at(const std::vector<byte> &data, std::size_t at)
    {
        std::int16_t value = 0;
        binary::reader(data).i16_at(at, value);
        return value;
    }
}

suite("unit.model.skybox_model")
{
    test("loads bottom-origin BGR TGA into top-origin RGB")
    {
        std::vector<byte> tga(18, 0);
        binary::writer header(tga);
        header.patch_u32(0, 2u << 16); // image type 2 at byte 2
        header.patch_u16(12, 2);
        header.patch_u16(14, 2);
        tga[16] = 24;
        // Bottom row: blue, white. Top row: red, green.
        const byte pixels[] = {
            255, 0, 0, 255, 255, 255,
            0, 0, 255, 0, 255, 0,
        };
        tga.insert(tga.end(), std::begin(pixels), std::end(pixels));

        std::string path =
            std::string(HLTOOLS_TEST_SCRATCH_DIRECTORY) + "/skybox_loader.tga";
        require(fs::write_all(path, tga.data(), tga.size()));

        model::rgb_image image;
        std::string error;
        require(model::load_tga(path, image, &error));
        expect(image.width == 2);
        expect(image.height == 2);
        // Top-left red, top-right green, bottom-left blue.
        require(image.rgb.size() == 12);
        expect(image.rgb[0] == 255);
        expect(image.rgb[1] == 0);
        expect(image.rgb[2] == 0);
        expect(image.rgb[3] == 0);
        expect(image.rgb[4] == 255);
        expect(image.rgb[5] == 0);
        expect(image.rgb[6] == 0);
        expect(image.rgb[7] == 0);
        expect(image.rgb[8] == 255);
    }

    test("preserves a tiled sky across the 64-skin model boundary")
    {
        std::array<model::rgb_image, 6> faces;
        for (std::size_t i = 0; i < faces.size(); i++)
            faces[i] = face(64, (byte)(i * 24));

        model::skybox_model_options options;
        options.world_size = 131072;
        options.tile_size = 16; // 6 * 4 * 4 = 96 skins
        options.model_name = "testsky";

        std::vector<model::skybox_model_part> parts;
        std::string error;
        require(model::build_skybox_models(faces, options, parts, &error));
        require(parts.size() == 2);
        expect(parts[0].textures == 64);
        expect(parts[1].textures == 32);
        expect(parts[0].triangles == 128);
        expect(parts[1].triangles == 64);

        for (std::size_t i = 0; i < parts.size(); i++)
        {
            const std::vector<byte> &mdl = parts[i].data;
            require(mdl.size() > 244);
            expect(std::memcmp(mdl.data(), "IDST", 4) == 0);
            expect(i32_at(mdl, 180) == (std::int32_t)parts[i].textures);
            expect(i32_at(mdl, 140) == 1); // root bone
            expect(i32_at(mdl, 164) == 1); // idle sequence

            std::size_t texture_at = (std::size_t)i32_at(mdl, 184);
            require(texture_at + 80 <= mdl.size());
            expect(i32_at(mdl, 188) == i32_at(mdl, texture_at + 76));
            int flags = i32_at(mdl, texture_at + 64);
            expect((flags & format::studio_nf_flatshade) != 0);
            expect((flags & format::studio_nf_fullbright) != 0);
            expect((flags & format::studio_nf_nomips) != 0);

            std::size_t bodypart = (std::size_t)i32_at(mdl, 208);
            std::size_t model_at =
                (std::size_t)i32_at(mdl, bodypart + 72);
            std::size_t mesh_at =
                (std::size_t)i32_at(mdl, model_at + 76);
            for (std::size_t mesh = 0; mesh < parts[i].textures; mesh++)
            {
                std::size_t at = mesh_at + mesh * 20;
                expect(i32_at(mdl, at + 12) == 1);
                expect(i32_at(mdl, at + 16) == (std::int32_t)mesh);
            }

            if (i == 0)
            {
                // the two tile triangles share all four command attributes, so
                // the writer may submit them as one exact four-vertex strip.
                std::size_t commands =
                    (std::size_t)i32_at(mdl, mesh_at + 4);
                expect(i16_at(mdl, commands) == 4);
                std::array<std::pair<std::int16_t, std::int16_t>, 4> uv;
                for (std::size_t vertex = 0; vertex < uv.size(); vertex++)
                {
                    std::size_t at = commands + 2 + vertex * 8;
                    uv[vertex] = {i16_at(mdl, at + 4), i16_at(mdl, at + 6)};
                }
                std::sort(uv.begin(), uv.end());
                expect(uv[0] == std::make_pair<std::int16_t, std::int16_t>(1, 1));
                expect(uv[1] == std::make_pair<std::int16_t, std::int16_t>(1, 15));
                expect(uv[2] == std::make_pair<std::int16_t, std::int16_t>(15, 1));
                expect(uv[3] == std::make_pair<std::int16_t, std::int16_t>(15, 15));
            }
        }
    }

    test("rejects mismatched face dimensions")
    {
        std::array<model::rgb_image, 6> faces;
        for (model::rgb_image &image : faces)
            image = face(32, 0);
        faces[5] = face(64, 0);

        std::vector<model::skybox_model_part> parts;
        std::string error;
        expect_false(model::build_skybox_models(
            faces, model::skybox_model_options{}, parts, &error));
        expect(error.find("identical") != std::string::npos);
    }
}
