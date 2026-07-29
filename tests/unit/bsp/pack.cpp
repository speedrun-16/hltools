#include <filesystem>
#include <string>
#include <vector>

#include "bsp/pack.h"
#include "common/binary.h"
#include "common/filesystem.h"
#include "common/types.h"
#include "format/bsp/data.h"
#include "format/bsp/entity_lump.h"
#include "format/bsp/file.h"
#include "format/miptex/types.h"
#include "support/scratch.h"
#include "support/test.h"

namespace
{
    void write_file(const std::filesystem::path &path)
    {
        require(fs::make_directory(path.parent_path().string()));
        const char data[] = "fixture";
        require(fs::write_all(path.string(), data, sizeof(data) - 1));
    }

    std::vector<byte> external_texture_lump()
    {
        std::vector<byte> data;
        binary::writer sink(data);
        sink.i32(1);
        sink.i32(8);
        format::miptex_t texture{};
        std::string name = "CUSTOM";
        std::copy(name.begin(), name.end(), texture.name);
        texture.width = 16;
        texture.height = 16;
        sink.raw(reinterpret_cast<const byte *>(&texture), sizeof(texture));
        return data;
    }
}

suite("unit.bsp.pack")
{
    test("preserves resource paths and writes a GoldSrc res list")
    {
        namespace stdfs = std::filesystem;
        stdfs::path root = test_support::scratch_directory("bsp_pack");
        stdfs::path game = root / "cstrike";
        stdfs::path base_game = root / "valve";
        stdfs::path map_path = game / "maps" / "probe.bsp";
        stdfs::path output = root / "package";

        write_file(game / "custom.wad");
        write_file(game / "models" / "prop.mdl");
        write_file(game / "models" / "propT.mdl");
        write_file(game / "models" / "prop01.mdl");
        write_file(game / "sprites" / "glow.spr");
        write_file(game / "sound" / "ambience" / "wind.wav");
        write_file(game / "sound" / "manual.wav");
        write_file(game / "maps" / "probe.txt");
        write_file(game / "overviews" / "probe.txt");
        write_file(base_game / "models" / "stock.mdl");
        static const char *const suffixes[6] =
            {"bk", "dn", "ft", "lf", "rt", "up"};
        for (const char *suffix : suffixes)
            write_file(game / "gfx" / "env"
                       / (std::string("custom/sky") + suffix + ".tga"));

        format::entity world;
        world.append("classname", "worldspawn");
        world.append("wad", (game / "custom.wad").string());
        world.append("skyname", "custom/sky");
        format::entity model;
        model.append("classname", "cycler");
        model.append("model", "models/prop.mdl");
        format::entity sprite;
        sprite.append("classname", "env_sprite");
        sprite.append("model", "glow.spr");
        format::entity sound;
        sound.append("classname", "ambient_generic");
        sound.append("message", "ambience/wind.wav");
        format::entity missing;
        missing.append("classname", "cycler");
        missing.append("model", "models/missing.mdl");
        format::entity stock;
        stock.append("classname", "cycler");
        stock.append("model", "models/stock.mdl");

        format::map_data map;
        map.entities =
            format::write_entities({world, model, sprite, sound, missing, stock});
        map.textures = external_texture_lump();
        require(fs::make_directory(map_path.parent_path().string()));
        require(format::bsp_file::write(map_path.string(), map));
        std::string existing_res =
            "// a dependency that is not present in the entity lump\r\n"
            "sound/manual.wav\r\n";
        require(fs::write_all(
            (game / "maps" / "probe.res").string(),
            existing_res.data(), existing_res.size()));

        bsp::pack_options options;
        options.game_dir = game.string();
        options.base_dirs.push_back(base_game.string());
        bsp::pack_result result;
        std::string error;
        require(bsp::pack_map(
            map_path.string(), output.string(), options, result, &error));

        expect(fs::exists((output / "maps" / "probe.bsp").string()));
        expect(fs::exists((output / "custom.wad").string()));
        expect(fs::exists((output / "models" / "prop.mdl").string()));
        expect(fs::exists((output / "models" / "propT.mdl").string()));
        expect(fs::exists((output / "models" / "prop01.mdl").string()));
        expect(fs::exists((output / "sprites" / "glow.spr").string()));
        expect(fs::exists(
            (output / "sound" / "ambience" / "wind.wav").string()));
        expect(fs::exists((output / "sound" / "manual.wav").string()));
        expect(fs::exists(
            (output / "gfx" / "env" / "custom" / "skyup.tga").string()));
        require(result.missing.size() == 1);
        expect(result.missing[0] == "models/missing.mdl");
        require(result.provided_by_base.size() == 1);
        expect(result.provided_by_base[0] == "models/stock.mdl");
        expect_false(fs::exists((output / "models" / "stock.mdl").string()));

        std::vector<byte> res;
        require(fs::read_all((output / "maps" / "probe.res").string(), res));
        std::string text(res.begin(), res.end());
        expect(text.find("maps/probe.bsp\r\n") != std::string::npos);
        expect(text.find("custom.wad\r\n") != std::string::npos);
        expect(text.find("models/propT.mdl\r\n") != std::string::npos);
        expect(text.find("sound/manual.wav\r\n") != std::string::npos);
        expect(text.find("gfx/env/custom/skyup.tga\r\n") != std::string::npos);
        expect(text.find("models/missing.mdl") == std::string::npos);

        options.strict = true;
        bsp::pack_result strict_result;
        expect_false(bsp::pack_map(
            map_path.string(), (root / "strict").string(), options,
            strict_result, &error));
        expect(error.find("1 referenced resource") != std::string::npos);
        expect_false(fs::exists((root / "strict" / "maps" / "probe.bsp").string()));
    }
}
