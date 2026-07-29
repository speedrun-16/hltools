#include <cstdint>
#include <string>
#include <vector>

#include "common/binary.h"
#include "common/types.h"
#include "format/mdl/source_mdl.h"
#include "support/test.h"

namespace
{
    void put(std::vector<byte> &buffer, std::size_t at, std::uint32_t value)
    {
        for (int i = 0; i < 4; i++)
            buffer[at + (std::size_t)i] = (byte)(value >> (i * 8));
    }

    void put_f(std::vector<byte> &buffer, std::size_t at, float value)
    {
        std::uint32_t raw;
        std::memcpy(&raw, &value, sizeof(raw));
        put(buffer, at, raw);
    }

    void put_str(std::vector<byte> &buffer, std::size_t at, const std::string &value)
    {
        for (std::size_t i = 0; i < value.size(); i++)
            buffer[at + i] = (byte)value[i];
    }

    // one bone, one bodypart, one model, one mesh, one triangle
    constexpr std::size_t bone_at = 512;
    constexpr std::size_t texture_at = 1024;
    constexpr std::size_t bodypart_at = 1280;
    constexpr std::size_t model_at = 1536;
    constexpr std::size_t mesh_at = 1792;
    constexpr std::size_t strings_at = 2048;
    constexpr std::size_t anim_at = 2560;
    constexpr std::size_t seq_at = 2816;
    constexpr std::size_t block_at = 3072;

    // terminator variant writes bone 255 into the animation block instead of a
    // real channel, the way a static prop's single frame idle is stored
    std::vector<byte> make_mdl(bool animated = true)
    {
        std::vector<byte> d(4096, 0);
        put_str(d, 0, "IDST");
        put(d, 4, 49);
        put_str(d, 12, "test/probe.mdl");
        put_f(d, 104, -8);  put_f(d, 108, -9);  put_f(d, 112, -10); // hull min
        put_f(d, 116, 11);  put_f(d, 120, 12);  put_f(d, 124, 13);  // hull max

        put(d, 156, 1);            // numbones
        put(d, 160, (std::uint32_t)bone_at);
        put(d, 204, 1);            // numtextures
        put(d, 208, (std::uint32_t)texture_at);
        put(d, 212, 1);            // numcdtextures
        put(d, 216, (std::uint32_t)(texture_at + 32));
        put(d, 232, 1);            // numbodyparts
        put(d, 236, (std::uint32_t)bodypart_at);

        // bone: name offset is relative to the record
        put(d, bone_at + 0, (std::uint32_t)(strings_at - bone_at));
        put(d, bone_at + 4, (std::uint32_t)-1); // parent
        put_f(d, bone_at + 32, 1);
        put_f(d, bone_at + 36, 2);
        put_f(d, bone_at + 40, 3);
        put_f(d, bone_at + 60, 0.5f);
        put_f(d, bone_at + 72, 0.5f); // posscale
        put_f(d, bone_at + 76, 0.5f);
        put_f(d, bone_at + 80, 0.5f);
        put_str(d, strings_at, "root_bone");

        // texture name, then the cdmaterials pointer table
        put(d, texture_at + 0, (std::uint32_t)(strings_at + 16 - texture_at));
        put_str(d, strings_at + 16, "skin_a");
        put(d, texture_at + 32, (std::uint32_t)(strings_at + 32));
        put_str(d, strings_at + 32, "models\\probe\\");

        // bodypart -> model -> mesh, each offset relative to its parent record
        put(d, bodypart_at + 4, 1); // nummodels
        put(d, bodypart_at + 12, (std::uint32_t)(model_at - bodypart_at));
        put_str(d, model_at, "probe.smd");
        put(d, model_at + 72, 1); // nummeshes
        put(d, model_at + 76, (std::uint32_t)(mesh_at - model_at));
        put(d, model_at + 80, 3); // numvertices
        put(d, model_at + 84, 0); // vertexindex, byte offset into the vvd pool
        put(d, mesh_at + 0, 0);   // material
        put(d, mesh_at + 8, 3);   // numvertices
        put(d, mesh_at + 12, 0);  // vertexoffset

        // one local animation and one sequence pointing at it through a blend
        // table holding a single anim id
        put(d, 180, 1); // numlocalanim
        put(d, 184, (std::uint32_t)anim_at);
        put(d, 188, 1); // numlocalseq
        put(d, 192, (std::uint32_t)seq_at);

        put(d, anim_at + 4, (std::uint32_t)(strings_at + 48 - anim_at));
        put_str(d, strings_at + 48, "@walk");
        put_f(d, anim_at + 8, 30);  // fps
        put(d, anim_at + 16, 2);    // numframes
        put(d, anim_at + 52, 0);    // animblock: inline
        put(d, anim_at + 56, (std::uint32_t)(block_at - anim_at));
        put(d, anim_at + 80, 0);    // sectionindex
        put(d, anim_at + 84, 0);    // sectionframes

        put(d, seq_at + 4, (std::uint32_t)(strings_at + 64 - seq_at));
        put_str(d, strings_at + 64, "walk");
        put(d, seq_at + 12, 1);   // flags: looping
        put(d, seq_at + 56, 1);   // numblends
        put(d, seq_at + 60, 200); // animindexindex, relative to the seqdesc
        put(d, seq_at + 200, 0);  // blend table entry: local anim 0

        d[block_at + 0] = animated ? 0 : 255; // bone, or the chain terminator
        d[block_at + 1] = animated ? 0x04 : 0; // STUDIO_ANIM_ANIMPOS
        put(d, block_at + 2, 0);               // nextoffset
        if (animated)
        {
            // position value pointers, offsets relative to the pointer record
            d[block_at + 4] = 0; d[block_at + 5] = 0;  // x: not animated
            d[block_at + 6] = 0; d[block_at + 7] = 0;  // y: not animated
            d[block_at + 8] = 6; d[block_at + 9] = 0;  // z: data 6 bytes along
            // run: two valid values covering two frames
            d[block_at + 10] = 2; // valid
            d[block_at + 11] = 2; // total
            put(d, block_at + 12, 100 | (200u << 16)); // the two shorts
        }
        return d;
    }

    std::vector<byte> make_vvd()
    {
        constexpr std::size_t header = 64;
        constexpr std::size_t vertex = 48;
        std::vector<byte> d(header + 3 * vertex + 3 * 16, 0);
        put_str(d, 0, "IDSV");
        put(d, 4, 4);   // version
        put(d, 8, 0);   // checksum
        put(d, 12, 1);  // numLODs
        put(d, 16, 3);  // numLODVertexes[0]
        put(d, 48, 0);  // numFixups
        put(d, 52, (std::uint32_t)header);
        put(d, 56, (std::uint32_t)header);
        put(d, 60, (std::uint32_t)(header + 3 * vertex));

        for (int i = 0; i < 3; i++)
        {
            std::size_t at = header + (std::size_t)i * vertex;
            put_f(d, at + 0, 1.0f); // weight[0]
            d[at + 12] = 0;         // bone[0]
            d[at + 15] = 1;         // numbones
            put_f(d, at + 16, (float)i);       // position
            put_f(d, at + 20, (float)(i * 2));
            put_f(d, at + 24, (float)(i * 3));
            put_f(d, at + 28, 0);              // normal
            put_f(d, at + 32, 0);
            put_f(d, at + 36, 1);
            put_f(d, at + 40, (float)i * 0.5f); // u
            put_f(d, at + 44, 0.25f);           // v
        }
        return d;
    }

    std::vector<byte> make_vtx()
    {
        // header 36, bodypart 8, model 8, lod 12, mesh 9, stripgroup 25,
        // then three 9-byte vertices and three 2-byte indices
        constexpr std::size_t bp = 36, model = 44, lod = 52, mesh = 64, group = 73;
        constexpr std::size_t verts = 98, indices = 125;
        std::vector<byte> d(indices + 3 * 2, 0);
        put(d, 0, 7);   // version
        put(d, 28, 1);  // numBodyParts
        put(d, 32, (std::uint32_t)bp);

        put(d, bp + 0, 1);
        put(d, bp + 4, (std::uint32_t)(model - bp));
        put(d, model + 0, 1); // numLODs
        put(d, model + 4, (std::uint32_t)(lod - model));
        put(d, lod + 0, 1);   // numMeshes
        put(d, lod + 4, (std::uint32_t)(mesh - lod));
        put(d, mesh + 0, 1);  // numStripGroups
        put(d, mesh + 4, (std::uint32_t)(group - mesh));

        put(d, group + 0, 3); // numVerts
        put(d, group + 4, (std::uint32_t)(verts - group));
        put(d, group + 8, 3); // numIndices
        put(d, group + 12, (std::uint32_t)(indices - group));
        put(d, group + 16, 0); // numStrips
        put(d, group + 20, 0);

        for (int i = 0; i < 3; i++)
        {
            std::size_t at = verts + (std::size_t)i * 9;
            d[at + 3] = 1;                  // numBones
            d[at + 4] = (byte)i;            // origMeshVertID
            d[at + 5] = 0;
        }
        for (int i = 0; i < 3; i++)
            d[indices + (std::size_t)i * 2] = (byte)i;
        return d;
    }
}

suite("unit.format.source_mdl")
{
    test("source_mdl.reads geometry, bones and material names")
    {
        format::studio_model model;
        std::string error;
        require(format::load_source_model(make_mdl(), make_vvd(), make_vtx(), model, &error));

        expect(model.name == "test/probe.mdl");
        require(model.bones.size() == 1);
        expect(model.bones[0].name == "root_bone");
        expect(model.bones[0].parent == -1);
        expect(model.bones[0].position[0] == 1);
        expect(model.bones[0].position[2] == 3);

        require(model.materials.size() == 1);
        expect(model.materials[0] == "skin_a");
        require(model.material_dirs.size() == 1);
        expect(model.material_dirs[0] == "models\\probe\\");

        // each pool vertex is emitted once and shared by index
        expect(model.vertices.size() == 3);
        require(model.meshes.size() == 1);
        expect(model.meshes[0].material == 0);
        expect(model.meshes[0].indices.size() == 3);
        expect(model.triangle_count() == 1);

        expect(model.vertices[1].position[0] == 1);
        expect(model.vertices[2].position[1] == 4);
        expect(model.vertices[1].u == 0.5f);
        expect(model.vertices[0].bone == 0);

        expect(model.bbmin[0] == -8);
        expect(model.bbmax[2] == 13);
    }

    test("source_mdl.decodes a run length encoded position channel")
    {
        format::studio_model model;
        std::string error;
        require(format::load_source_model(make_mdl(), make_vvd(), make_vtx(), model, &error));

        require(model.sequences.size() == 1);
        const format::studio_sequence &walk = model.sequences[0];
        expect(walk.name == "walk");
        expect(walk.fps == 30);
        expect(walk.loop);
        require(walk.frames.size() == 2);
        require(walk.frames[0].size() == 1);

        // z = bone position 3 + value * posscale 0.5
        expect(walk.frames[0][0].position[2] == 3.0f + 100.0f * 0.5f);
        expect(walk.frames[1][0].position[2] == 3.0f + 200.0f * 0.5f);
        // unanimated channels stay at the bind pose
        expect(walk.frames[0][0].position[0] == 1.0f);
        expect(walk.frames[1][0].position[1] == 2.0f);
    }

    test("source_mdl.treats bone 255 as the end of the animation chain")
    {
        format::studio_model model;
        std::string error;
        require(format::load_source_model(make_mdl(false), make_vvd(), make_vtx(),
                                          model, &error));

        // the sequence survives and every key sits at the bind pose
        require(model.sequences.size() == 1);
        require(model.sequences[0].frames.size() == 2);
        expect(model.sequences[0].frames[0][0].position[0] == 1.0f);
        expect(model.sequences[0].frames[0][0].position[2] == 3.0f);
        expect(model.sequences[0].frames[1][0].position[2] == 3.0f);
    }

    test("source_mdl.rejects files that are not studio models")
    {
        format::studio_model model;
        std::vector<byte> junk(512, 0);
        expect_false(format::load_source_model(junk, make_vvd(), make_vtx(), model));
        expect_false(format::load_source_model(make_mdl(), junk, make_vtx(), model));
        expect_false(format::load_source_model(make_mdl(), make_vvd(), junk, model));
    }
}
