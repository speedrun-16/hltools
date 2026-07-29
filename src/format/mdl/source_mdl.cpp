#include "source_mdl.h"

#include <array>
#include <cmath>
#include <cstring>

#include "common/binary.h"

namespace format
{
    namespace
    {
        // on disk record sizes. these are the historical valve layouts and are
        // fixed across the mdl versions this reader accepts; fields are pulled
        // out by explicit offset rather than by casting a struct, so alignment
        // and endianness never enter into it.
        constexpr std::size_t mdl_bone_size = 216;
        constexpr std::size_t mdl_bodypart_size = 16;
        constexpr std::size_t mdl_model_size = 148;
        constexpr std::size_t mdl_mesh_size = 116;
        constexpr std::size_t mdl_texture_size = 64;

        constexpr std::size_t vvd_header_size = 64;
        constexpr std::size_t vvd_vertex_size = 48;
        constexpr std::size_t vvd_fixup_size = 12;

        constexpr std::size_t vtx_bodypart_size = 8;
        constexpr std::size_t vtx_model_size = 8;
        constexpr std::size_t vtx_lod_size = 12;
        constexpr std::size_t vtx_mesh_size = 9;
        constexpr std::size_t vtx_vertex_size = 9;
        // the strip group record grew two fields (topology indices) in the newer
        // vtx revisions; which one a file uses is detected by walking it
        constexpr std::size_t vtx_stripgroup_size_classic = 25;
        constexpr std::size_t vtx_stripgroup_size_topology = 33;

        constexpr std::size_t mdl_animdesc_size = 100;
        constexpr std::size_t mdl_seqdesc_size = 212;

        constexpr int vvd_version = 4;
        constexpr int vtx_version = 7;

        // per bone animation channel flags
        constexpr int anim_rawpos = 0x01;
        constexpr int anim_rawrot = 0x02;
        constexpr int anim_animpos = 0x04;
        constexpr int anim_animrot = 0x08;
        constexpr int anim_delta = 0x10;
        constexpr int anim_rawrot2 = 0x20;

        constexpr int seq_looping = 0x0001;

        float half_to_float(std::uint16_t bits)
        {
            int sign = (bits >> 15) & 1;
            int exponent = (bits >> 10) & 0x1f;
            int mantissa = bits & 0x3ff;
            float value;
            if (exponent == 0)
                value = std::ldexp((float)mantissa, -24);
            else if (exponent == 31)
                value = mantissa ? 0.0f : 65504.0f; // nan/inf are not meaningful here
            else
                value = std::ldexp((float)(mantissa + 1024), exponent - 25);
            return sign ? -value : value;
        }

        void quaternion_to_euler(const float q[4], float out[3])
        {
            float x = q[0], y = q[1], z = q[2], w = q[3];
            out[0] = std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
            float sinp = 2.0f * (w * y - z * x);
            sinp = sinp > 1.0f ? 1.0f : (sinp < -1.0f ? -1.0f : sinp);
            out[1] = std::asin(sinp);
            out[2] = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
        }

        // the run length encoding both engines use for animation channels: a run
        // is a (valid, total) byte pair followed by `valid` shorts, where a frame
        // past the stored values repeats the last one.
        bool extract_anim_value(const std::vector<byte> &data, std::size_t at, int frame,
                                float &out)
        {
            binary::reader r(data);
            int k = frame;
            for (int guard = 0; guard < 65536; guard++)
            {
                if (at + 2 > data.size())
                    return false;
                int valid = data[at];
                int total = data[at + 1];
                if (total == 0)
                    return false;
                if (total > k)
                {
                    int index = valid > k ? k : valid - 1;
                    if (index < 0)
                        return false;
                    std::int16_t value = 0;
                    if (!r.i16_at(at + 2 + (std::size_t)index * 2, value))
                        return false;
                    out = (float)value;
                    return true;
                }
                k -= total;
                at += 2 * ((std::size_t)valid + 1);
            }
            return false;
        }

        void set_error(std::string *error, const std::string &message)
        {
            if (error)
                *error = message;
        }

        std::string read_cstring(const std::vector<byte> &data, std::size_t offset)
        {
            if (offset >= data.size())
                return std::string();
            std::size_t end = offset;
            while (end < data.size() && data[end] != 0)
                end++;
            return std::string((const char *)data.data() + offset, end - offset);
        }

        // a name stored as an offset relative to the record that holds it
        std::string read_indexed_name(const std::vector<byte> &data, std::size_t record,
                                      std::size_t field_offset)
        {
            binary::reader r(data);
            std::int32_t relative = 0;
            if (!r.i32_at(record + field_offset, relative) || relative == 0)
                return std::string();
            return read_cstring(data, record + (std::size_t)relative);
        }

        struct vtx_layout
        {
            std::size_t stripgroup_size = vtx_stripgroup_size_classic;
        };

        // decodes one bone's key for one frame out of a per bone animation block.
        // `at` points at the 4 byte mstudioanim_t header for that bone.
        void read_bone_key(const std::vector<byte> &mdl, std::size_t at, int flags,
                           int frame_in_section, const studio_bone &bone,
                           const float posscale[3], const float rotscale[3],
                           studio_bone_key &key)
        {
            binary::reader r(mdl);
            for (int k = 0; k < 3; k++)
            {
                key.position[k] = bone.position[k];
                key.rotation[k] = bone.rotation[k];
            }

            // the payload order valve's accessors imply: raw rotation, then raw
            // position, then the animated channel pointers (rotation first)
            std::size_t cursor = at + 4;
            if (flags & anim_rawrot)
            {
                std::uint16_t raw[3] = {};
                r.u16_at(cursor, raw[0]);
                r.u16_at(cursor + 2, raw[1]);
                r.u16_at(cursor + 4, raw[2]);
                float q[4];
                q[0] = ((float)raw[0] - 32768.0f) / 32768.0f;
                q[1] = ((float)raw[1] - 32768.0f) / 32768.0f;
                q[2] = ((float)(raw[2] & 0x7fff) - 16384.0f) / 16384.0f;
                float sum = q[0] * q[0] + q[1] * q[1] + q[2] * q[2];
                q[3] = sum < 1.0f ? std::sqrt(1.0f - sum) : 0.0f;
                if (raw[2] & 0x8000)
                    q[3] = -q[3];
                quaternion_to_euler(q, key.rotation);
                cursor += 6;
            }
            if (flags & anim_rawrot2)
            {
                std::uint32_t lo = 0, hi = 0;
                r.u32_at(cursor, lo);
                r.u32_at(cursor + 4, hi);
                std::uint64_t bits = (std::uint64_t)lo | ((std::uint64_t)hi << 32);
                float q[4];
                q[0] = ((float)(bits & 0x1fffff) - 1048576.0f) / 1048576.0f;
                q[1] = ((float)((bits >> 21) & 0x1fffff) - 1048576.0f) / 1048576.0f;
                q[2] = ((float)((bits >> 42) & 0x1fffff) - 1048576.0f) / 1048576.0f;
                float sum = q[0] * q[0] + q[1] * q[1] + q[2] * q[2];
                q[3] = sum < 1.0f ? std::sqrt(1.0f - sum) : 0.0f;
                if ((bits >> 63) & 1)
                    q[3] = -q[3];
                quaternion_to_euler(q, key.rotation);
                cursor += 8;
            }
            if (flags & anim_rawpos)
            {
                for (int k = 0; k < 3; k++)
                {
                    std::uint16_t raw = 0;
                    r.u16_at(cursor + (std::size_t)k * 2, raw);
                    key.position[k] = half_to_float(raw);
                }
                cursor += 6;
            }

            // animated channels: the rotation pointer comes first when present
            std::size_t rot_ptr = cursor;
            std::size_t pos_ptr = cursor + ((flags & anim_animrot) ? 6 : 0);
            if (flags & anim_animrot)
            {
                for (int k = 0; k < 3; k++)
                {
                    std::int16_t offset = 0;
                    if (!r.i16_at(rot_ptr + (std::size_t)k * 2, offset) || offset == 0)
                        continue;
                    float value = 0;
                    if (extract_anim_value(mdl, rot_ptr + (std::size_t)offset,
                                           frame_in_section, value))
                        key.rotation[k] = bone.rotation[k] + value * rotscale[k];
                }
            }
            if (flags & anim_animpos)
            {
                for (int k = 0; k < 3; k++)
                {
                    std::int16_t offset = 0;
                    if (!r.i16_at(pos_ptr + (std::size_t)k * 2, offset) || offset == 0)
                        continue;
                    float value = 0;
                    if (extract_anim_value(mdl, pos_ptr + (std::size_t)offset,
                                           frame_in_section, value))
                        key.position[k] = bone.position[k] + value * posscale[k];
                }
            }
        }

        // reads every local sequence and bakes it into explicit per frame keys.
        // sequences are best effort: a model whose animation cannot be decoded
        // still keeps its geometry, which is what the porter mostly needs.
        void read_sequences(const std::vector<byte> &mdl, studio_model &out)
        {
            binary::reader r(mdl);
            std::int32_t numlocalanim = 0, localanimindex = 0;
            std::int32_t numlocalseq = 0, localseqindex = 0;
            r.i32_at(180, numlocalanim);
            r.i32_at(184, localanimindex);
            r.i32_at(188, numlocalseq);
            r.i32_at(192, localseqindex);
            if (numlocalseq <= 0 || numlocalseq > 65536 || numlocalanim <= 0)
                return;

            // per bone quantisation scales live in the bone table
            std::vector<std::array<float, 3>> posscale(out.bones.size());
            std::vector<std::array<float, 3>> rotscale(out.bones.size());
            std::int32_t boneindex = 0;
            r.i32_at(160, boneindex);
            for (std::size_t b = 0; b < out.bones.size(); b++)
            {
                std::size_t at = (std::size_t)boneindex + b * mdl_bone_size;
                for (int k = 0; k < 3; k++)
                {
                    r.f32_at(at + 72 + (std::size_t)k * 4, posscale[b][(std::size_t)k]);
                    r.f32_at(at + 84 + (std::size_t)k * 4, rotscale[b][(std::size_t)k]);
                }
            }

            for (int s = 0; s < numlocalseq; s++)
            {
                std::size_t so = (std::size_t)localseqindex + (std::size_t)s * mdl_seqdesc_size;
                if (so + mdl_seqdesc_size > mdl.size())
                    break;

                studio_sequence sequence;
                sequence.name = read_indexed_name(mdl, so, 4);
                std::int32_t seq_flags = 0, animindexindex = 0;
                r.i32_at(so + 12, seq_flags);
                r.i32_at(so + 60, animindexindex);
                sequence.loop = (seq_flags & seq_looping) != 0;

                // a sequence points at its animations through a blend table; the
                // first entry is the one goldsrc can represent
                std::int16_t anim_id = 0;
                if (animindexindex == 0
                    || !r.i16_at(so + (std::size_t)animindexindex, anim_id))
                    continue;
                if (anim_id < 0 || anim_id >= numlocalanim)
                    continue;

                std::size_t ao = (std::size_t)localanimindex
                    + (std::size_t)anim_id * mdl_animdesc_size;
                if (ao + mdl_animdesc_size > mdl.size())
                    continue;

                std::int32_t numframes = 0, animblock = 0, animindex = 0;
                std::int32_t sectionindex = 0, sectionframes = 0;
                r.f32_at(ao + 8, sequence.fps);
                r.i32_at(ao + 16, numframes);
                r.i32_at(ao + 52, animblock);
                r.i32_at(ao + 56, animindex);
                r.i32_at(ao + 80, sectionindex);
                r.i32_at(ao + 84, sectionframes);
                if (numframes <= 0 || numframes > 65536 || animblock != 0 || animindex == 0)
                    continue; // an external animation block is not readable here
                if (sequence.fps <= 0)
                    sequence.fps = 30;

                sequence.frames.resize((std::size_t)numframes);
                bool ok = true;
                for (int f = 0; f < numframes && ok; f++)
                {
                    // long animations are split into fixed size sections, each
                    // with its own block of per bone channels
                    std::size_t block = ao + (std::size_t)animindex;
                    int frame_in_section = f;
                    if (sectionframes > 0 && sectionindex != 0)
                    {
                        int section = f / sectionframes;
                        frame_in_section = f - section * sectionframes;
                        std::int32_t section_animindex = 0;
                        if (!r.i32_at(ao + (std::size_t)sectionindex
                                          + (std::size_t)section * 8 + 4,
                                      section_animindex))
                        {
                            ok = false;
                            break;
                        }
                        block = ao + (std::size_t)section_animindex;
                    }

                    std::vector<studio_bone_key> &keys = sequence.frames[(std::size_t)f];
                    keys.resize(out.bones.size());
                    for (std::size_t b = 0; b < out.bones.size(); b++)
                    {
                        keys[b].position[0] = out.bones[b].position[0];
                        keys[b].position[1] = out.bones[b].position[1];
                        keys[b].position[2] = out.bones[b].position[2];
                        keys[b].rotation[0] = out.bones[b].rotation[0];
                        keys[b].rotation[1] = out.bones[b].rotation[1];
                        keys[b].rotation[2] = out.bones[b].rotation[2];
                    }

                    // the block is a chain of per bone records linked by offset
                    std::size_t at = block;
                    for (int guard = 0; guard < 4096; guard++)
                    {
                        if (at + 4 > mdl.size())
                        {
                            ok = false;
                            break;
                        }
                        int bone = mdl[at];
                        int flags = mdl[at + 1];
                        std::int16_t nextoffset = 0;
                        r.i16_at(at + 2, nextoffset);
                        // 255 terminates the chain. a block that starts with it
                        // simply animates nothing, which is how a static prop's
                        // single frame idle is stored; the bind pose already
                        // filled the keys.
                        if (bone == 255)
                            break;
                        if (bone < 0 || (std::size_t)bone >= out.bones.size())
                        {
                            ok = false;
                            break;
                        }
                        read_bone_key(mdl, at, flags, frame_in_section,
                                      out.bones[(std::size_t)bone],
                                      posscale[(std::size_t)bone].data(),
                                      rotscale[(std::size_t)bone].data(),
                                      keys[(std::size_t)bone]);
                        if (nextoffset == 0)
                            break;
                        at += (std::size_t)nextoffset;
                    }
                }
                if (ok)
                    out.sequences.push_back(std::move(sequence));
            }
        }

        struct mesh_indices
        {
            std::vector<int> indices; // relative to the owning mesh's vertices
        };

        // walks every strip group of lod 0 with the given record size, collecting
        // per mesh index lists. returns false as soon as anything lands out of
        // bounds, which is what lets the caller tell the two layouts apart.
        bool read_vtx_lod0(const std::vector<byte> &vtx, std::size_t stripgroup_size,
                           std::vector<std::vector<mesh_indices>> &out)
        {
            binary::reader r(vtx);
            std::int32_t num_bodyparts = 0, bodypart_offset = 0;
            if (!r.i32_at(28, num_bodyparts) || !r.i32_at(32, bodypart_offset))
                return false;
            if (num_bodyparts < 0 || num_bodyparts > 4096)
                return false;

            out.clear();
            out.resize((std::size_t)num_bodyparts);
            for (int bp = 0; bp < num_bodyparts; bp++)
            {
                std::size_t bpo = (std::size_t)bodypart_offset + (std::size_t)bp * vtx_bodypart_size;
                std::int32_t num_models = 0, model_offset = 0;
                if (!r.i32_at(bpo, num_models) || !r.i32_at(bpo + 4, model_offset))
                    return false;
                if (num_models < 0 || num_models > 4096)
                    return false;

                for (int m = 0; m < num_models; m++)
                {
                    std::size_t mo = bpo + (std::size_t)model_offset + (std::size_t)m * vtx_model_size;
                    std::int32_t num_lods = 0, lod_offset = 0;
                    if (!r.i32_at(mo, num_lods) || !r.i32_at(mo + 4, lod_offset))
                        return false;
                    if (num_lods <= 0)
                        return false;

                    // lod 0 only; goldsrc has no level of detail
                    std::size_t lo = mo + (std::size_t)lod_offset;
                    std::int32_t num_meshes = 0, mesh_offset = 0;
                    if (!r.i32_at(lo, num_meshes) || !r.i32_at(lo + 4, mesh_offset))
                        return false;
                    if (num_meshes < 0 || num_meshes > 65536)
                        return false;

                    for (int me = 0; me < num_meshes; me++)
                    {
                        std::size_t meo = lo + (std::size_t)mesh_offset + (std::size_t)me * vtx_mesh_size;
                        std::int32_t num_groups = 0, group_offset = 0;
                        if (!r.i32_at(meo, num_groups) || !r.i32_at(meo + 4, group_offset))
                            return false;
                        if (num_groups < 0 || num_groups > 65536)
                            return false;

                        mesh_indices mesh;
                        for (int sg = 0; sg < num_groups; sg++)
                        {
                            std::size_t sgo = meo + (std::size_t)group_offset
                                + (std::size_t)sg * stripgroup_size;
                            std::int32_t num_verts = 0, vert_offset = 0;
                            std::int32_t num_indices = 0, index_offset = 0;
                            if (!r.i32_at(sgo, num_verts) || !r.i32_at(sgo + 4, vert_offset)
                                || !r.i32_at(sgo + 8, num_indices)
                                || !r.i32_at(sgo + 12, index_offset))
                                return false;
                            if (num_verts < 0 || num_indices < 0 || num_indices % 3 != 0)
                                return false;

                            std::size_t verts_at = sgo + (std::size_t)vert_offset;
                            std::size_t indices_at = sgo + (std::size_t)index_offset;
                            if (verts_at + (std::size_t)num_verts * vtx_vertex_size > vtx.size())
                                return false;
                            if (indices_at + (std::size_t)num_indices * 2 > vtx.size())
                                return false;

                            mesh.indices.reserve(mesh.indices.size() + (std::size_t)num_indices);
                            for (int i = 0; i < num_indices; i++)
                            {
                                std::uint16_t local = 0;
                                if (!r.u16_at(indices_at + (std::size_t)i * 2, local))
                                    return false;
                                if (local >= (std::uint16_t)num_verts)
                                    return false;
                                // the strip group's vertex record points back at
                                // the mesh's own vertex range
                                std::uint16_t orig = 0;
                                if (!r.u16_at(verts_at + (std::size_t)local * vtx_vertex_size + 4,
                                              orig))
                                    return false;
                                mesh.indices.push_back((int)orig);
                            }
                        }
                        out[(std::size_t)bp].push_back(std::move(mesh));
                    }
                }
            }
            return true;
        }
    }

    bool load_source_model(const std::vector<byte> &mdl, const std::vector<byte> &vvd,
                           const std::vector<byte> &vtx, studio_model &out,
                           std::string *error)
    {
        out = studio_model{};

        if (mdl.size() < 408 || std::memcmp(mdl.data(), "IDST", 4) != 0)
        {
            set_error(error, "not a source studio model (.mdl)");
            return false;
        }
        binary::reader m(mdl);
        std::int32_t version = 0;
        m.i32_at(4, version);
        if (version < source_mdl_min_version || version > source_mdl_max_version)
        {
            set_error(error, "unsupported .mdl version " + std::to_string(version));
            return false;
        }

        if (vvd.size() < vvd_header_size || std::memcmp(vvd.data(), "IDSV", 4) != 0)
        {
            set_error(error, "not a source vertex file (.vvd)");
            return false;
        }
        binary::reader v(vvd);
        std::int32_t vvd_ver = 0;
        v.i32_at(4, vvd_ver);
        if (vvd_ver != vvd_version)
        {
            set_error(error, "unsupported .vvd version " + std::to_string(vvd_ver));
            return false;
        }
        binary::reader x(vtx);
        std::int32_t vtx_ver = 0;
        if (vtx.size() < 36 || !x.i32_at(0, vtx_ver) || vtx_ver != vtx_version)
        {
            set_error(error, "unsupported or missing .vtx index file");
            return false;
        }

        out.name = read_cstring(mdl, 12);
        m.f32_at(104, out.bbmin[0]);
        m.f32_at(108, out.bbmin[1]);
        m.f32_at(112, out.bbmin[2]);
        m.f32_at(116, out.bbmax[0]);
        m.f32_at(120, out.bbmax[1]);
        m.f32_at(124, out.bbmax[2]);

        std::int32_t numbones = 0, boneindex = 0;
        m.i32_at(156, numbones);
        m.i32_at(160, boneindex);
        if (numbones < 0 || numbones > 4096)
        {
            set_error(error, "implausible bone count in .mdl");
            return false;
        }
        out.bones.resize((std::size_t)numbones);
        for (int b = 0; b < numbones; b++)
        {
            std::size_t at = (std::size_t)boneindex + (std::size_t)b * mdl_bone_size;
            if (at + mdl_bone_size > mdl.size())
            {
                set_error(error, ".mdl bone table extends past the file");
                return false;
            }
            studio_bone &bone = out.bones[(std::size_t)b];
            bone.name = read_indexed_name(mdl, at, 0);
            std::int32_t parent = -1;
            m.i32_at(at + 4, parent);
            bone.parent = parent;
            for (int k = 0; k < 3; k++)
            {
                m.f32_at(at + 32 + (std::size_t)k * 4, bone.position[k]);
                m.f32_at(at + 60 + (std::size_t)k * 4, bone.rotation[k]);
            }
        }

        std::int32_t numtextures = 0, textureindex = 0;
        std::int32_t numcdtextures = 0, cdtextureindex = 0;
        m.i32_at(204, numtextures);
        m.i32_at(208, textureindex);
        m.i32_at(212, numcdtextures);
        m.i32_at(216, cdtextureindex);
        for (int t = 0; t < numtextures && numtextures < 4096; t++)
        {
            std::size_t at = (std::size_t)textureindex + (std::size_t)t * mdl_texture_size;
            out.materials.push_back(read_indexed_name(mdl, at, 0));
        }
        for (int c = 0; c < numcdtextures && numcdtextures < 4096; c++)
        {
            std::int32_t relative = 0;
            if (m.i32_at((std::size_t)cdtextureindex + (std::size_t)c * 4, relative))
                out.material_dirs.push_back(read_cstring(mdl, (std::size_t)relative));
        }

        std::int32_t lod_vertexes = 0, numfixups = 0, fixup_start = 0, vertex_start = 0;
        v.i32_at(16, lod_vertexes); // numLODVertexes[0]
        v.i32_at(48, numfixups);
        v.i32_at(52, fixup_start);
        v.i32_at(56, vertex_start);
        if (lod_vertexes < 0
            || (std::size_t)vertex_start + (std::size_t)lod_vertexes * vvd_vertex_size > vvd.size())
        {
            set_error(error, ".vvd vertex data extends past the file");
            return false;
        }

        auto read_vertex = [&](std::size_t index, studio_vertex &vertex) {
            std::size_t at = (std::size_t)vertex_start + index * vvd_vertex_size;
            // pick the dominant bone: goldsrc cannot blend, and the props this
            // targets are rigid anyway
            float best_weight = -1;
            int best_bone = 0;
            for (int i = 0; i < 3; i++)
            {
                float weight = 0;
                v.f32_at(at + (std::size_t)i * 4, weight);
                byte bone = at + 12 + (std::size_t)i < vvd.size() ? vvd[at + 12 + (std::size_t)i] : 0;
                if (weight > best_weight)
                {
                    best_weight = weight;
                    best_bone = (int)bone;
                }
            }
            vertex.bone = best_bone;
            for (int k = 0; k < 3; k++)
            {
                v.f32_at(at + 16 + (std::size_t)k * 4, vertex.position[k]);
                v.f32_at(at + 28 + (std::size_t)k * 4, vertex.normal[k]);
            }
            v.f32_at(at + 40, vertex.u);
            v.f32_at(at + 44, vertex.v);
        };

        // lod fixups reorder the pool; with none present it is already lod 0
        std::vector<int> pool_index;
        if (numfixups > 0)
        {
            for (int i = 0; i < numfixups; i++)
            {
                std::size_t at = (std::size_t)fixup_start + (std::size_t)i * vvd_fixup_size;
                std::int32_t lod = 0, source_id = 0, count = 0;
                if (!v.i32_at(at, lod) || !v.i32_at(at + 4, source_id)
                    || !v.i32_at(at + 8, count))
                {
                    set_error(error, ".vvd fixup table is truncated");
                    return false;
                }
                if (lod < 0) // fixups apply to every lod at or below their own
                    continue;
                for (int k = 0; k < count; k++)
                    pool_index.push_back(source_id + k);
            }
        }
        else
        {
            pool_index.reserve((std::size_t)lod_vertexes);
            for (int i = 0; i < lod_vertexes; i++)
                pool_index.push_back(i);
        }

        std::vector<std::vector<mesh_indices>> vtx_meshes;
        vtx_layout layout;
        if (!read_vtx_lod0(vtx, vtx_stripgroup_size_classic, vtx_meshes))
        {
            layout.stripgroup_size = vtx_stripgroup_size_topology;
            if (!read_vtx_lod0(vtx, vtx_stripgroup_size_topology, vtx_meshes))
            {
                set_error(error, ".vtx index data could not be read with either "
                                 "known strip group layout");
                return false;
            }
        }

        // pool slot -> index of the vertex already emitted for it, or -1
        std::vector<int> emitted((std::size_t)lod_vertexes, -1);

        std::int32_t numbodyparts = 0, bodypartindex = 0;
        m.i32_at(232, numbodyparts);
        m.i32_at(236, bodypartindex);
        if (numbodyparts < 0 || (std::size_t)numbodyparts > vtx_meshes.size())
        {
            set_error(error, ".mdl and .vtx disagree on the bodypart count");
            return false;
        }

        for (int bp = 0; bp < numbodyparts; bp++)
        {
            std::size_t bpo = (std::size_t)bodypartindex + (std::size_t)bp * mdl_bodypart_size;
            std::int32_t nummodels = 0, modelindex = 0;
            m.i32_at(bpo + 4, nummodels);
            m.i32_at(bpo + 12, modelindex);

            std::size_t vtx_mesh_cursor = 0;
            const std::vector<mesh_indices> &vtx_for_part = vtx_meshes[(std::size_t)bp];

            for (int mi = 0; mi < nummodels; mi++)
            {
                std::size_t mo = bpo + (std::size_t)modelindex + (std::size_t)mi * mdl_model_size;
                if (mo + mdl_model_size > mdl.size())
                {
                    set_error(error, ".mdl model table extends past the file");
                    return false;
                }
                std::int32_t nummeshes = 0, meshindex = 0, vertexindex = 0;
                m.i32_at(mo + 72, nummeshes);
                m.i32_at(mo + 76, meshindex);
                m.i32_at(mo + 84, vertexindex);
                // vertexindex is a byte offset into the vvd vertex pool
                std::size_t model_base = (std::size_t)vertexindex / vvd_vertex_size;

                for (int me = 0; me < nummeshes; me++)
                {
                    std::size_t meo = mo + (std::size_t)meshindex + (std::size_t)me * mdl_mesh_size;
                    if (meo + mdl_mesh_size > mdl.size())
                    {
                        set_error(error, ".mdl mesh table extends past the file");
                        return false;
                    }
                    std::int32_t material = 0, mesh_verts = 0, vertex_offset = 0;
                    m.i32_at(meo, material);
                    m.i32_at(meo + 8, mesh_verts);
                    m.i32_at(meo + 12, vertex_offset);

                    if (vtx_mesh_cursor >= vtx_for_part.size())
                    {
                        set_error(error, ".vtx is missing meshes the .mdl declares");
                        return false;
                    }
                    const mesh_indices &source = vtx_for_part[vtx_mesh_cursor++];

                    studio_mesh mesh;
                    mesh.material = material;
                    mesh.indices.reserve(source.indices.size());
                    for (int local : source.indices)
                    {
                        // vtx indices are mesh relative; walk out to the pool
                        std::size_t pool = model_base + (std::size_t)vertex_offset
                            + (std::size_t)local;
                        if (local >= mesh_verts || pool >= pool_index.size())
                        {
                            set_error(error, ".vtx references a vertex outside the .vvd pool");
                            return false;
                        }
                        std::size_t vvd_index = (std::size_t)pool_index[pool];
                        if (vvd_index >= (std::size_t)lod_vertexes)
                        {
                            set_error(error, ".vvd fixup table points outside the vertex pool");
                            return false;
                        }
                        // the pool is shared between triangles, so each source
                        // vertex is emitted once and referenced by index
                        int &slot = emitted[vvd_index];
                        if (slot < 0)
                        {
                            slot = (int)out.vertices.size();
                            studio_vertex vertex;
                            read_vertex(vvd_index, vertex);
                            out.vertices.push_back(vertex);
                        }
                        mesh.indices.push_back(slot);
                    }
                    out.meshes.push_back(std::move(mesh));
                }
            }
        }

        if (out.meshes.empty())
        {
            set_error(error, "source model has no lod 0 geometry");
            return false;
        }

        read_sequences(mdl, out);
        return true;
    }
}
