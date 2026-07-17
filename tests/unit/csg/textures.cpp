#include "common/string_util.h"
#include "csg/textures.h"
#include "support/test.h"

namespace
{
    csg::brush_texture texture(const char *name)
    {
        csg::brush_texture value;
        str::copy(value.name, sizeof(value.name), name);
        value.valve.scale[0] = 1;
        value.valve.scale[1] = 1;
        return value;
    }

    csg::brush_plane floor_plane()
    {
        csg::brush_plane floor;
        floor.normal = {0, 0, 1};
        return floor;
    }
}
suite("unit.csg.textures")
{
    test("textures.builds and reuses legacy texture axes")
    {
        csg::texinfo_store store;
        csg::brush_texture stone = texture("STONE");
        stone.valve.shift[0] = 8;
        stone.valve.shift[1] = 16;

        expect(store.texinfo_for_brush_texture(floor_plane(), stone, {}, 0) == 0);
        require(store.entries().size() == 1);
        const format::texinfo_t &info = store.entries()[0].info;
        expect(info.vecs[0][0] == 1.0f);
        expect(info.vecs[0][1] == 0.0f);
        expect(info.vecs[0][3] == 8.0f);
        expect(info.vecs[1][0] == 0.0f);
        expect(info.vecs[1][1] == -1.0f);
        expect(info.vecs[1][3] == 16.0f);
        expect(store.texinfo_for_brush_texture(floor_plane(), stone, {}, 0) == 0);
    }

    test("textures.applies Valve axes scale shift and origin")
    {
        csg::texinfo_store store;
        csg::brush_texture brick = texture("BRICK");
        brick.valve.u_axis = {2, 0, 0};
        brick.valve.v_axis = {0, -4, 0};
        brick.valve.scale[0] = 2;
        brick.valve.scale[1] = 4;
        brick.valve.shift[0] = 3;
        brick.valve.shift[1] = 5;

        expect(store.texinfo_for_brush_texture(
                   floor_plane(), brick, {10, 20, 30}, 220) == 0);
        require(store.entries().size() == 1);
        const format::texinfo_t &info = store.entries()[0].info;
        expect(info.vecs[0][0] == 1.0f);
        expect(info.vecs[0][3] == 13.0f);
        expect(info.vecs[1][1] == -1.0f);
        expect(info.vecs[1][3] == -15.0f);
    }

    test("textures.normalizes zero scales")
    {
        csg::texinfo_store store;
        csg::brush_texture value = texture("ZERO");
        value.valve.u_axis = {1, 0, 0};
        value.valve.v_axis = {0, 1, 0};
        value.valve.scale[0] = 0;
        value.valve.scale[1] = 0;
        store.texinfo_for_brush_texture(floor_plane(), value, {}, 220);
        expect(value.valve.scale[0] == 1);
        expect(value.valve.scale[1] == 1);
    }

    test("textures.marks special textures and omits null")
    {
        csg::texinfo_store store;
        csg::brush_texture sky = texture("SKY01");
        expect(store.texinfo_for_brush_texture(floor_plane(), sky, {}, 220) == 0);
        require(store.entries().size() == 1);
        expect((store.entries()[0].info.flags & csg::tex_special) != 0);

        csg::brush_texture null_texture = texture("NULL");
        expect(store.texinfo_for_brush_texture(
                   floor_plane(), null_texture, {}, 220) == -1);
    }
}
