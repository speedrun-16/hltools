#include <cstring>

#include "format/bsp/data.h"
#include "format/miptex/types.h"
#include "rad/internal.h"
#include "support/test.h"

namespace
{
    void add_texture_lump(format::map_data &map, const char *first,
                          const char *second)
    {
        const int count = 2;
        const int header_size = (int)(sizeof(int) + count * sizeof(int));
        map.textures.resize((std::size_t)header_size
                            + count * sizeof(format::miptex_t), 0);
        std::memcpy(map.textures.data(), &count, sizeof(count));

        int offsets[2] = {
            header_size,
            header_size + (int)sizeof(format::miptex_t),
        };
        std::memcpy(map.textures.data() + sizeof(int), offsets, sizeof(offsets));

        format::miptex_t textures[2] = {};
        std::strncpy(textures[0].name, first, sizeof(textures[0].name) - 1);
        std::strncpy(textures[1].name, second, sizeof(textures[1].name) - 1);
        std::memcpy(map.textures.data() + header_size, textures, sizeof(textures));
    }
}

suite("unit.rad.unlit_textures")
{
    test("info_unlittextures accepts enabled texture keys case insensitively")
    {
        rad::rad_state state;
        state.entities.emplace_back();
        state.entities[0].append("classname", "info_unlittextures");
        state.entities[0].append("Glow270", "1");
        state.entities[0].append("disabled", "0");

        rad::read_info_tex_and_minlights(state);

        expect(rad::is_unlit_texture(state, "Glow270"));
        expect(rad::is_unlit_texture(state, "gLoW270"));
        expect_false(rad::is_unlit_texture(state, "DISABLED"));
        expect_false(rad::is_unlit_texture(state, "other"));
    }

    test("reduce_lightmap bakes white samples only for unlit texture faces")
    {
        format::map_data map;
        add_texture_lump(map, "270", "WALL");
        map.texinfo.resize(2);
        map.texinfo[0].miptex = 0;
        map.texinfo[1].miptex = 1;

        map.faces.resize(2);
        map.faces[0].texinfo = 0;
        map.faces[0].styles[0] = 0;
        map.faces[0].styles[1] = 255;
        map.faces[0].styles[2] = 255;
        map.faces[0].styles[3] = 255;
        map.faces[0].lightofs = 0;
        map.faces[1].texinfo = 1;
        map.faces[1].styles[0] = 0;
        map.faces[1].styles[1] = 255;
        map.faces[1].styles[2] = 255;
        map.faces[1].styles[3] = 255;
        map.faces[1].lightofs = 3;
        map.lighting = {10, 20, 30, 40, 50, 60};

        rad::rad_state state;
        state.map = &map;
        state.unlittextures.emplace_back("270");
        state.facelights.resize(2);
        state.facelights[0].numsamples = 1;
        state.facelights[1].numsamples = 1;
        state.entities.emplace_back();
        state.face_entity = {&state.entities[0], &state.entities[0]};

        rad::reduce_lightmap(state);

        expect(map.faces[0].lightofs == 0);
        expect(map.faces[0].styles[0] == 0);
        for (std::size_t i = 1; i < sizeof(map.faces[0].styles); i++)
            expect(map.faces[0].styles[i] == 255);
        expect(map.faces[1].lightofs == 3);
        expect(map.faces[1].styles[0] == 0);
        require(map.lighting.size() == 6);
        expect(map.lighting[0] == 255);
        expect(map.lighting[1] == 255);
        expect(map.lighting[2] == 255);
        expect(map.lighting[3] == 40);
        expect(map.lighting[4] == 50);
        expect(map.lighting[5] == 60);
    }
}
