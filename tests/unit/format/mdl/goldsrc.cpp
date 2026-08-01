#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include "common/binary.h"
#include "common/types.h"
#include "format/mdl/goldsrc/model.h"
#include "support/test.h"

namespace
{
    format::studio_model make_model()
    {
        format::studio_model model;
        model.name = "props/crate.mdl";
        model.bbmin[0] = -4;
        model.bbmax[2] = 9;

        format::studio_bone bone;
        bone.name = "root";
        bone.parent = -1;
        bone.position[2] = 10;
        model.bones.push_back(bone);

        for (int i = 0; i < 3; i++)
        {
            format::studio_vertex vertex;
            vertex.position[0] = (float)i;
            vertex.position[1] = (float)(i * 2);
            vertex.normal[2] = 1;
            vertex.u = i == 0 ? 0.0f : 1.0f;
            vertex.v = 0.5f;
            vertex.bone = 0;
            model.vertices.push_back(vertex);
        }

        format::studio_mesh mesh;
        mesh.material = 0;
        mesh.indices = {0, 1, 2};
        model.meshes.push_back(mesh);

        format::studio_texture texture;
        texture.name = "crate.bmp";
        texture.image.width = 32;
        texture.image.height = 16;
        texture.image.pixels.assign(32 * 16, 7);
        texture.image.palette[7] = {1, 2, 3};
        model.textures.push_back(texture);

        // two frames moving the bone up in z
        format::studio_sequence sequence;
        sequence.name = "bob";
        sequence.fps = 24;
        sequence.loop = true;
        for (int f = 0; f < 2; f++)
        {
            format::studio_bone_key key;
            key.position[2] = 10.0f + (float)f * 6.0f;
            sequence.frames.push_back({key});
        }
        model.sequences.push_back(sequence);
        return model;
    }

    std::int32_t i32_at(const std::vector<byte> &d, std::size_t at)
    {
        std::int32_t value = 0;
        binary::reader(d).i32_at(at, value);
        return value;
    }

    std::uint16_t u16_at(const std::vector<byte> &d, std::size_t at)
    {
        std::uint16_t value = 0;
        binary::reader(d).u16_at(at, value);
        return value;
    }

    float f32_at(const std::vector<byte> &d, std::size_t at)
    {
        float value = 0;
        binary::reader(d).f32_at(at, value);
        return value;
    }

    // the run length walk the engine performs, so the test reads the animation
    // back the way the engine would
    float decode_channel(const std::vector<byte> &d, std::size_t at, int frame)
    {
        int k = frame;
        for (;;)
        {
            int valid = d[at], total = d[at + 1];
            if (total > k)
            {
                int index = valid > k ? k : valid - 1;
                std::int16_t value = 0;
                binary::reader(d).i16_at(at + 2 + (std::size_t)index * 2, value);
                return (float)value;
            }
            k -= total;
            at += 2 * ((std::size_t)valid + 1);
        }
    }
}

suite("unit.format.goldsrc_mdl")
{
    test("goldsrc_mdl.writes a well formed version 10 header")
    {
        std::vector<byte> out;
        std::string error;
        require(format::write_goldsrc_model(make_model(), out, &error));

        expect(std::memcmp(out.data(), "IDST", 4) == 0);
        expect(i32_at(out, 4) == format::goldsrc_studio_version);
        // the length field must describe the whole file
        expect(i32_at(out, 72) == (std::int32_t)out.size());
        expect(std::strcmp((const char *)out.data() + 8, "props/crate.mdl") == 0);

        expect(i32_at(out, 140) == 1); // numbones
        expect(i32_at(out, 156) == 1); // generated extent hitbox
        expect(i32_at(out, 164) == 1); // numseq
        expect(i32_at(out, 172) == 1); // numseqgroups
        expect(i32_at(out, 180) == 1); // numtextures
        expect(i32_at(out, 204) == 1); // numbodyparts
        expect(i32_at(out, 196) == 1); // numskinfamilies
        std::size_t texture = (std::size_t)i32_at(out, 184);
        // texturedataindex names the first embedded pixel block, not the
        // texture-header array.
        expect(i32_at(out, 188) == i32_at(out, texture + 76));
        std::size_t hitbox = (std::size_t)i32_at(out, 160);
        expect(i32_at(out, hitbox) == 0); // root bone
        expect(f32_at(out, hitbox + 8) == -4.0f);
        expect(f32_at(out, hitbox + 8 + 5 * 4) == 9.0f);
    }

    test("goldsrc_mdl.emits geometry as a triangle command stream")
    {
        std::vector<byte> out;
        require(format::write_goldsrc_model(make_model(), out));

        std::size_t bodypart = (std::size_t)i32_at(out, 208);
        std::size_t model = (std::size_t)i32_at(out, bodypart + 72);
        expect(i32_at(out, model + 72) == 1); // one mesh
        expect(i32_at(out, model + 80) == 3); // three distinct positions
        expect(i32_at(out, model + 92) == 1); // all three share one normal

        std::size_t mesh = (std::size_t)i32_at(out, model + 76);
        expect(i32_at(out, mesh) == 1);     // one triangle
        expect(i32_at(out, mesh + 8) == 0); // skinref
        expect(i32_at(out, mesh + 12) == 1); // one normal in this mesh's slice
        expect(i32_at(out, mesh + 16) == 0); // slice starts at normal zero

        std::size_t commands = (std::size_t)i32_at(out, mesh + 4);
        std::int16_t count = 0;
        binary::reader(out).i16_at(commands, count);
        expect(count == 3); // a strip of three, i.e. one triangle

        // uvs become texel columns and rows against the 32x16 skin
        expect((std::int16_t)u16_at(out, commands + 2 + 4) == 0);   // u 0.0 -> s 0
        expect((std::int16_t)u16_at(out, commands + 2 + 6) == 8);   // v 0.5 -> t 8
        expect((std::int16_t)u16_at(out, commands + 2 + 8 + 4) == 32); // u 1.0 -> s 32

        // the stream is terminated
        std::int16_t last = 1;
        binary::reader(out).i16_at(commands + 2 + 3 * 8, last);
        expect(last == 0);
    }

    test("goldsrc_mdl.round trips an animated channel through the encoding")
    {
        std::vector<byte> out;
        require(format::write_goldsrc_model(make_model(), out));

        std::size_t seq = (std::size_t)i32_at(out, 168);
        expect(std::strcmp((const char *)out.data() + seq, "bob") == 0);
        expect(f32_at(out, seq + 32) == 24.0f); // fps
        expect(i32_at(out, seq + 36) == 1);     // looping
        expect(i32_at(out, seq + 56) == 2);     // numframes

        std::size_t bone = (std::size_t)i32_at(out, 144);
        float base = f32_at(out, bone + 64 + 2 * 4);  // value[2]
        float scale = f32_at(out, bone + 88 + 2 * 4); // scale[2]

        std::size_t anim = (std::size_t)i32_at(out, seq + 124);
        std::uint16_t offset = u16_at(out, anim + 2 * 2); // z channel
        require(offset != 0);

        float f0 = base + decode_channel(out, anim + offset, 0) * scale;
        float f1 = base + decode_channel(out, anim + offset, 1) * scale;
        expect(std::fabs(f0 - 10.0f) < 0.01f);
        expect(std::fabs(f1 - 16.0f) < 0.01f);

        // channels that never move are left at offset zero
        expect(u16_at(out, anim + 0 * 2) == 0);
        expect(u16_at(out, anim + 3 * 2) == 0);
    }

    test("goldsrc_mdl.splits geometry past the vertex ceiling across bodyparts")
    {
        format::studio_model model = make_model();
        model.vertices.clear();
        model.meshes[0].indices.clear();
        // 800 triangles of three distinct positions each: 2400 positions, so the
        // writer has to spill into a second bodypart
        constexpr int triangles = 800;
        for (int t = 0; t < triangles; t++)
            for (int k = 0; k < 3; k++)
            {
                format::studio_vertex vertex;
                vertex.position[0] = (float)t;
                vertex.position[1] = (float)k;
                vertex.normal[2] = 1;
                model.vertices.push_back(vertex);
                model.meshes[0].indices.push_back(t * 3 + k);
            }

        std::vector<byte> out;
        std::string error;
        require(format::write_goldsrc_model(model, out, &error));

        std::int32_t numbodyparts = i32_at(out, 204);
        expect(numbodyparts > 1);

        // every bodypart stays under the ceiling, and between them they carry
        // all of the original triangles
        std::size_t bodypart = (std::size_t)i32_at(out, 208);
        int total_tris = 0;
        for (std::int32_t bp = 0; bp < numbodyparts; bp++)
        {
            std::size_t at = bodypart + (std::size_t)bp * 76;
            expect(i32_at(out, at + 64) == 1); // one model per bodypart
            std::size_t model_at = (std::size_t)i32_at(out, at + 72);
            std::int32_t verts = i32_at(out, model_at + 80);
            expect(verts <= format::goldsrc_max_studio_verts);
            std::size_t mesh_at = (std::size_t)i32_at(out, model_at + 76);
            for (std::int32_t m = 0; m < i32_at(out, model_at + 72); m++)
                total_tris += i32_at(out, mesh_at + (std::size_t)m * 20);
        }
        expect(total_tris == triangles);
    }

    test("goldsrc_mdl.keeps a small model in a single bodypart")
    {
        std::vector<byte> out;
        require(format::write_goldsrc_model(make_model(), out));
        expect(i32_at(out, 204) == 1);
    }
}
