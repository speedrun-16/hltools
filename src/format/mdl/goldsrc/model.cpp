#include "model.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <map>

#include "common/binary.h"

namespace format
{
    namespace
    {
        // on disk record sizes for studio version 10
        constexpr std::size_t hdr_size = 244;
        constexpr std::size_t bone_size = 112;
        constexpr std::size_t seqgroup_size = 104;
        constexpr std::size_t seqdesc_size = 176;
        constexpr std::size_t anim_size = 12; // six unsigned short offsets
        constexpr std::size_t hitbox_size = 32;
        constexpr std::size_t bodypart_size = 76;
        constexpr std::size_t model_size = 112;
        constexpr std::size_t mesh_size = 20;
        constexpr std::size_t texture_size = 80;

        constexpr std::size_t bone_name_size = 32;
        constexpr std::size_t texture_name_size = 64;
        constexpr std::size_t model_name_size = 64;

        void set_error(std::string *error, const std::string &message)
        {
            if (error)
                *error = message;
        }

        void write_name(std::vector<byte> &out, std::size_t at, const std::string &name,
                        std::size_t capacity)
        {
            std::size_t count = std::min(name.size(), capacity - 1);
            for (std::size_t i = 0; i < count; i++)
                out[at + i] = (byte)name[i];
        }

        void pad_to(binary::writer &sink, std::vector<byte> &out, std::size_t alignment)
        {
            while (out.size() % alignment != 0)
                sink.u8(0);
        }

        struct vec3_key
        {
            float v[3];
            bool operator<(const vec3_key &other) const
            {
                for (int i = 0; i < 3; i++)
                {
                    if (v[i] < other.v[i])
                        return true;
                    if (v[i] > other.v[i])
                        return false;
                }
                return false;
            }
        };

        // GoldSrc indexes positions and normals through separate arrays, each
        // with its own byte of bone ownership. Positions are model-wide, but
        // normals are partitioned per mesh: mstudiomesh_t::normindex and
        // numnorms describe a contiguous slice. Some renderers ignore those two
        // fields and trust the triangle commands; editors and strict loaders do
        // not, so a globally de-duplicated normal pool is not sufficient.
        struct geometry
        {
            std::vector<float> positions; // 3 per entry
            std::vector<byte> position_bone;
            std::vector<float> normals;
            std::vector<byte> normal_bone;
            std::vector<int> vertex_to_position;
            std::vector<std::vector<int>> mesh_vertex_to_normal;
            std::vector<int> mesh_normal_start;
            std::vector<int> mesh_normal_count;
        };

        // de-duplicates the vertices reached by `meshes` only, so each bodypart
        // of a split model gets its own compact arrays
        geometry split_vertices(const studio_model &model,
                                const std::vector<studio_mesh> &meshes)
        {
            geometry out;
            std::map<vec3_key, int> position_ids;
            out.vertex_to_position.assign(model.vertices.size(), -1);
            out.mesh_vertex_to_normal.resize(meshes.size());
            out.mesh_normal_start.reserve(meshes.size());
            out.mesh_normal_count.reserve(meshes.size());

            for (std::size_t mesh_index = 0; mesh_index < meshes.size(); mesh_index++)
            {
                const studio_mesh &mesh = meshes[mesh_index];
                std::map<vec3_key, int> normal_ids;
                std::vector<int> &normal_map =
                    out.mesh_vertex_to_normal[mesh_index];
                normal_map.assign(model.vertices.size(), -1);
                int normal_start = (int)out.normal_bone.size();
                out.mesh_normal_start.push_back(normal_start);

                for (int index : mesh.indices)
                {
                    std::size_t i = (std::size_t)index;
                    if (i >= model.vertices.size())
                        continue;
                    const studio_vertex &vertex = model.vertices[i];

                    if (out.vertex_to_position[i] < 0)
                    {
                        vec3_key position{{vertex.position[0], vertex.position[1],
                                           vertex.position[2]}};
                        auto pit = position_ids.find(position);
                        if (pit == position_ids.end())
                        {
                            pit = position_ids
                                      .emplace(position,
                                               (int)out.position_bone.size())
                                      .first;
                            for (int k = 0; k < 3; k++)
                                out.positions.push_back(vertex.position[k]);
                            out.position_bone.push_back((byte)vertex.bone);
                        }
                        out.vertex_to_position[i] = pit->second;
                    }

                    if (normal_map[i] >= 0)
                        continue;
                    vec3_key normal{{vertex.normal[0], vertex.normal[1],
                                     vertex.normal[2]}};
                    auto nit = normal_ids.find(normal);
                    if (nit == normal_ids.end())
                    {
                        nit = normal_ids
                                  .emplace(normal,
                                           (int)out.normal_bone.size())
                                  .first;
                        for (int k = 0; k < 3; k++)
                            out.normals.push_back(vertex.normal[k]);
                        out.normal_bone.push_back((byte)vertex.bone);
                    }
                    normal_map[i] = nit->second;
                }
                out.mesh_normal_count.push_back(
                    (int)out.normal_bone.size() - normal_start);
            }
            return out;
        }

        // one drawable chunk: a set of meshes whose combined vertices fit under
        // the studio ceiling, emitted as its own bodypart
        struct part
        {
            std::vector<studio_mesh> meshes;
            geometry geo;
        };

        // greedily packs triangles into parts. a triangle is never split, and
        // material grouping is preserved inside each part, so the only visible
        // effect of a split is more bodyparts.
        std::vector<part> partition_model(const studio_model &model, int max_verts)
        {
            std::vector<part> parts;
            parts.emplace_back();
            std::map<vec3_key, int> used;

            auto flush = [&]() {
                if (!parts.back().meshes.empty())
                {
                    parts.emplace_back();
                    used.clear();
                }
            };

            for (const studio_mesh &source : model.meshes)
            {
                for (std::size_t t = 0; t + 2 < source.indices.size(); t += 3)
                {
                    vec3_key keys[3];
                    int fresh = 0;
                    for (int k = 0; k < 3; k++)
                    {
                        std::size_t i = (std::size_t)source.indices[t + (std::size_t)k];
                        if (i >= model.vertices.size())
                            continue;
                        const studio_vertex &vertex = model.vertices[i];
                        keys[k] = vec3_key{{vertex.position[0], vertex.position[1],
                                            vertex.position[2]}};
                        if (!used.count(keys[k]))
                        {
                            // a triangle can repeat a position within itself
                            bool duplicate = false;
                            for (int j = 0; j < k; j++)
                                if (!(keys[j] < keys[k]) && !(keys[k] < keys[j]))
                                    duplicate = true;
                            if (!duplicate)
                                fresh++;
                        }
                    }
                    if ((int)used.size() + fresh > max_verts)
                        flush();

                    for (int k = 0; k < 3; k++)
                        used.emplace(keys[k], 0);

                    // keep one mesh per material inside the part
                    std::vector<studio_mesh> &meshes = parts.back().meshes;
                    studio_mesh *target = nullptr;
                    for (studio_mesh &mesh : meshes)
                        if (mesh.material == source.material)
                        {
                            target = &mesh;
                            break;
                        }
                    if (target == nullptr)
                    {
                        studio_mesh mesh;
                        mesh.material = source.material;
                        meshes.push_back(std::move(mesh));
                        target = &meshes.back();
                    }
                    for (int k = 0; k < 3; k++)
                        target->indices.push_back(source.indices[t + (std::size_t)k]);
                }
            }

            if (parts.back().meshes.empty() && parts.size() > 1)
                parts.pop_back();
            for (part &p : parts)
                p.geo = split_vertices(model, p.meshes);
            return parts;
        }

        // one triangle per strip. the command stream also allows longer strips
        // and fans, which only saves draw time, never correctness.
        std::vector<std::int16_t> build_triangle_commands(const studio_model &model,
                                                          const studio_mesh &mesh,
                                                          const geometry &geo,
                                                          std::size_t mesh_index,
                                                          unsigned width, unsigned height)
        {
            std::vector<std::int16_t> commands;
            commands.reserve(mesh.indices.size() * 4 + mesh.indices.size() / 3 + 1);
            for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3)
            {
                commands.push_back(3);
                for (int k = 0; k < 3; k++)
                {
                    int index = mesh.indices[t + (std::size_t)k];
                    const studio_vertex &vertex = model.vertices[(std::size_t)index];
                    // source stores uvs normalized; goldsrc stores texel columns
                    // and rows directly in the command stream
                    float s = vertex.u * (float)width;
                    float v = vertex.v * (float)height;
                    commands.push_back((std::int16_t)geo.vertex_to_position[(std::size_t)index]);
                    commands.push_back((std::int16_t)
                        geo.mesh_vertex_to_normal[mesh_index][(std::size_t)index]);
                    commands.push_back((std::int16_t)std::lround(s));
                    commands.push_back((std::int16_t)std::lround(v));
                }
            }
            commands.push_back(0); // end of the stream
            return commands;
        }

        // the six degrees of freedom a studio bone animates, in file order
        float key_dof(const studio_bone_key &key, int dof)
        {
            return dof < 3 ? key.position[dof] : key.rotation[dof - 3];
        }

        float bone_dof(const studio_bone &bone, int dof)
        {
            return dof < 3 ? bone.position[dof] : bone.rotation[dof - 3];
        }

        // run length encodes one channel. a run is a (valid, total) byte pair
        // followed by `valid` shorts: the first `valid` frames read their own
        // value and the remaining `total - valid` repeat the last one.
        void encode_channel(const std::vector<std::int16_t> &values,
                            std::vector<std::int16_t> &out)
        {
            std::size_t i = 0;
            while (i < values.size())
            {
                std::size_t j = i;
                int valid = 0;
                while (j < values.size() && valid < 255
                       && (j == i || values[j] != values[j - 1]))
                {
                    valid++;
                    j++;
                }
                int total = valid;
                while (j < values.size() && total < 255 && values[j] == values[j - 1])
                {
                    total++;
                    j++;
                }
                out.push_back((std::int16_t)((unsigned)valid | ((unsigned)total << 8)));
                for (int k = 0; k < valid; k++)
                    out.push_back(values[i + (std::size_t)k]);
                i = j;
            }
        }
    }

    bool write_goldsrc_model(const studio_model &model, std::vector<byte> &out,
                             std::string *error)
    {
        out.clear();
        if (model.bones.empty())
        {
            set_error(error, "a studio model needs at least one bone");
            return false;
        }
        if (model.meshes.empty())
        {
            set_error(error, "a studio model needs at least one mesh");
            return false;
        }

        std::vector<part> parts = partition_model(model, goldsrc_max_studio_verts);
        if (parts.empty())
        {
            set_error(error, "a studio model needs at least one triangle");
            return false;
        }
        if (parts.size() > (std::size_t)goldsrc_max_studio_bodyparts)
        {
            set_error(error, "model needs " + std::to_string(parts.size())
                                 + " bodyparts to stay under the "
                                 + std::to_string(goldsrc_max_studio_verts)
                                 + " vertex limit, above the maximum of "
                                 + std::to_string(goldsrc_max_studio_bodyparts));
            return false;
        }

        // per bone default pose and the scale each channel quantises through
        std::size_t bones = model.bones.size();
        std::vector<std::array<float, 6>> value(bones);
        std::vector<std::array<float, 6>> scale(bones);
        for (std::size_t b = 0; b < bones; b++)
            for (int d = 0; d < 6; d++)
            {
                value[b][(std::size_t)d] = bone_dof(model.bones[b], d);
                scale[b][(std::size_t)d] = 0;
            }
        for (const studio_sequence &sequence : model.sequences)
            for (const std::vector<studio_bone_key> &frame : sequence.frames)
                for (std::size_t b = 0; b < bones && b < frame.size(); b++)
                    for (int d = 0; d < 6; d++)
                    {
                        float delta = std::fabs(key_dof(frame[b], d) - value[b][(std::size_t)d]);
                        scale[b][(std::size_t)d] =
                            std::max(scale[b][(std::size_t)d], delta);
                    }
        for (std::size_t b = 0; b < bones; b++)
            for (int d = 0; d < 6; d++)
            {
                // float noise in a channel that is really static would otherwise
                // produce an absurdly fine scale, so anything under a thousandth
                // of a unit counts as not moving and takes the default divisor
                constexpr float still = 1e-3f;
                float &s = scale[b][(std::size_t)d];
                s = s > still ? s / 32767.0f : (d < 3 ? 0.01f : 0.000001f);
            }

        out.reserve(65536);
        binary::writer sink(out);
        for (std::size_t i = 0; i < hdr_size; i++)
            sink.u8(0);
        std::memcpy(out.data(), "IDST", 4);
        binary::writer header(out);
        header.patch_u32(4, (std::uint32_t)goldsrc_studio_version);
        write_name(out, 8, model.name, model_name_size);
        for (int k = 0; k < 3; k++)
        {
            std::uint32_t lo, hi;
            std::memcpy(&lo, &model.bbmin[k], 4);
            std::memcpy(&hi, &model.bbmax[k], 4);
            header.patch_u32(88 + (std::size_t)k * 4, lo);   // min
            header.patch_u32(100 + (std::size_t)k * 4, hi);  // max
            header.patch_u32(112 + (std::size_t)k * 4, lo);  // bbmin
            header.patch_u32(124 + (std::size_t)k * 4, hi);  // bbmax
        }

        std::size_t bone_at = out.size();
        for (std::size_t i = 0; i < bones * bone_size; i++)
            sink.u8(0);
        for (std::size_t b = 0; b < bones; b++)
        {
            std::size_t at = bone_at + b * bone_size;
            write_name(out, at, model.bones[b].name, bone_name_size);
            header.patch_i32(at + 32, model.bones[b].parent);
            for (int d = 0; d < 6; d++)
            {
                // name[32] parent flags bonecontroller[6] value[6] scale[6]
                header.patch_i32(at + 40 + (std::size_t)d * 4, -1); // bonecontroller
                std::uint32_t raw;
                std::memcpy(&raw, &value[b][(std::size_t)d], 4);
                header.patch_u32(at + 64 + (std::size_t)d * 4, raw);
                std::memcpy(&raw, &scale[b][(std::size_t)d], 4);
                header.patch_u32(at + 88 + (std::size_t)d * 4, raw);
            }
        }
        header.patch_i32(140, (std::int32_t)bones);
        header.patch_i32(144, (std::int32_t)bone_at);

        // A generated extent hitbox is part of the conventional StudioMDL
        // output and of gchimp's model rebuild path. It also gives editors a
        // valid bone-bound extent instead of making them infer one from an
        // otherwise empty hitbox table.
        if (bones != 0)
        {
            std::size_t hitbox_at = out.size();
            sink.i32(0); // bone
            sink.i32(0); // group
            for (int k = 0; k < 3; k++)
                sink.f32(model.bbmin[k]);
            for (int k = 0; k < 3; k++)
                sink.f32(model.bbmax[k]);
            if (out.size() - hitbox_at != hitbox_size)
            {
                set_error(error, "internal hitbox record size mismatch");
                return false;
            }
            header.patch_i32(156, 1);
            header.patch_i32(160, (std::int32_t)hitbox_at);
        }

        std::size_t seqgroup_at = out.size();
        for (std::size_t i = 0; i < seqgroup_size; i++)
            sink.u8(0);
        write_name(out, seqgroup_at, "default", 32);
        header.patch_i32(172, 1);
        header.patch_i32(176, (std::int32_t)seqgroup_at);

        std::size_t seq_at = out.size();
        std::size_t numseq = model.sequences.size();
        for (std::size_t i = 0; i < numseq * seqdesc_size; i++)
            sink.u8(0);

        for (std::size_t s = 0; s < numseq; s++)
        {
            const studio_sequence &sequence = model.sequences[s];
            std::size_t frames = sequence.frames.size();
            std::size_t at = seq_at + s * seqdesc_size;

            write_name(out, at, sequence.name, 32);
            std::uint32_t fps_raw;
            std::memcpy(&fps_raw, &sequence.fps, 4);
            header.patch_u32(at + 32, fps_raw);
            header.patch_i32(at + 36, sequence.loop ? 1 : 0); // STUDIO_LOOPING
            header.patch_i32(at + 56, (std::int32_t)frames);
            header.patch_i32(at + 120, 1); // numblends
            header.patch_i32(at + 156, 0); // seqgroup
            for (int k = 0; k < 3; k++)
            {
                std::uint32_t lo, hi;
                std::memcpy(&lo, &model.bbmin[k], 4);
                std::memcpy(&hi, &model.bbmax[k], 4);
                header.patch_u32(at + 96 + (std::size_t)k * 4, lo);
                header.patch_u32(at + 108 + (std::size_t)k * 4, hi);
            }

            pad_to(sink, out, 4);
            std::size_t anim_at = out.size();
            header.patch_i32(at + 124, (std::int32_t)anim_at);
            for (std::size_t i = 0; i < bones * anim_size; i++)
                sink.u8(0);

            for (std::size_t b = 0; b < bones; b++)
            {
                for (int d = 0; d < 6; d++)
                {
                    if (frames == 0)
                        continue;
                    std::vector<std::int16_t> quantised(frames);
                    bool moves = false;
                    for (std::size_t f = 0; f < frames; f++)
                    {
                        float key = b < sequence.frames[f].size()
                            ? key_dof(sequence.frames[f][b], d)
                            : value[b][(std::size_t)d];
                        float delta = (key - value[b][(std::size_t)d])
                            / scale[b][(std::size_t)d];
                        delta = std::max(-32767.0f, std::min(32767.0f, delta));
                        quantised[f] = (std::int16_t)std::lround(delta);
                        if (quantised[f] != 0)
                            moves = true;
                    }
                    // an unmoving channel is left at offset 0, which the engine
                    // reads as "hold the bone's default value"
                    if (!moves)
                        continue;

                    std::vector<std::int16_t> encoded;
                    encode_channel(quantised, encoded);
                    std::size_t data_at = out.size();
                    std::size_t record = anim_at + b * anim_size;
                    header.patch_u16(record + (std::size_t)d * 2,
                                     (std::uint16_t)(data_at - record));
                    for (std::int16_t word : encoded)
                        sink.i16(word);
                }
            }
        }
        header.patch_i32(164, (std::int32_t)numseq);
        header.patch_i32(168, (std::int32_t)seq_at);

        // one bodypart per part, each holding a single model, so the engine
        // draws them all at once
        std::size_t bodypart_at = out.size();
        for (std::size_t i = 0; i < parts.size() * bodypart_size; i++)
            sink.u8(0);

        for (std::size_t p = 0; p < parts.size(); p++)
        {
            const part &piece = parts[p];
            const geometry &geo = piece.geo;
            std::size_t part_at = bodypart_at + p * bodypart_size;
            std::string label = parts.size() == 1
                ? std::string("body")
                : "body" + std::to_string(p);
            write_name(out, part_at, label, model_name_size);
            header.patch_i32(part_at + 64, 1); // nummodels
            header.patch_i32(part_at + 68, 1); // base

            std::size_t model_at = out.size();
            header.patch_i32(part_at + 72, (std::int32_t)model_at);
            for (std::size_t i = 0; i < model_size; i++)
                sink.u8(0);
            write_name(out, model_at, label, model_name_size);

            std::size_t mesh_at = out.size();
            std::size_t nummesh = piece.meshes.size();
            for (std::size_t i = 0; i < nummesh * mesh_size; i++)
                sink.u8(0);

            // vertex and normal ownership tables, then the coordinates
            pad_to(sink, out, 4);
            std::size_t vertinfo_at = out.size();
            for (byte bone : geo.position_bone)
                sink.u8(bone);
            pad_to(sink, out, 4);
            std::size_t norminfo_at = out.size();
            for (byte bone : geo.normal_bone)
                sink.u8(bone);
            pad_to(sink, out, 4);
            std::size_t vert_at = out.size();
            for (float component : geo.positions)
                sink.f32(component);
            std::size_t norm_at = out.size();
            for (float component : geo.normals)
                sink.f32(component);

            header.patch_i32(model_at + 72, (std::int32_t)nummesh);
            header.patch_i32(model_at + 76, (std::int32_t)mesh_at);
            header.patch_i32(model_at + 80, (std::int32_t)geo.position_bone.size());
            header.patch_i32(model_at + 84, (std::int32_t)vertinfo_at);
            header.patch_i32(model_at + 88, (std::int32_t)vert_at);
            header.patch_i32(model_at + 92, (std::int32_t)geo.normal_bone.size());
            header.patch_i32(model_at + 96, (std::int32_t)norminfo_at);
            header.patch_i32(model_at + 100, (std::int32_t)norm_at);

            for (std::size_t m = 0; m < nummesh; m++)
            {
                const studio_mesh &mesh = piece.meshes[m];
                unsigned width = 64, height = 64;
                if (mesh.material >= 0
                    && (std::size_t)mesh.material < model.textures.size())
                {
                    width = model.textures[(std::size_t)mesh.material].image.width;
                    height = model.textures[(std::size_t)mesh.material].image.height;
                    width = width ? width : 64;
                    height = height ? height : 64;
                }
                std::vector<std::int16_t> commands =
                    build_triangle_commands(model, mesh, geo, m, width, height);

                pad_to(sink, out, 4);
                std::size_t commands_at = out.size();
                for (std::int16_t word : commands)
                    sink.i16(word);

                std::size_t at = mesh_at + m * mesh_size;
                header.patch_i32(at + 0, (std::int32_t)(mesh.indices.size() / 3));
                header.patch_i32(at + 4, (std::int32_t)commands_at);
                header.patch_i32(at + 8, mesh.material);
                header.patch_i32(at + 12, geo.mesh_normal_count[m]);
                header.patch_i32(at + 16, geo.mesh_normal_start[m]);
            }
        }
        header.patch_i32(204, (std::int32_t)parts.size());
        header.patch_i32(208, (std::int32_t)bodypart_at);

        std::size_t numtextures = model.textures.size();
        pad_to(sink, out, 4);
        std::size_t texture_at = out.size();
        for (std::size_t i = 0; i < numtextures * texture_size; i++)
            sink.u8(0);

        std::size_t skin_at = out.size();
        for (std::size_t i = 0; i < numtextures; i++)
            sink.i16((std::int16_t)i);
        pad_to(sink, out, 4);
        header.patch_i32(192, (std::int32_t)numtextures); // numskinref
        header.patch_i32(196, 1);                          // numskinfamilies
        header.patch_i32(200, (std::int32_t)skin_at);

        std::size_t texture_data_at = 0;
        for (std::size_t t = 0; t < numtextures; t++)
        {
            const studio_texture &texture = model.textures[t];
            std::size_t at = texture_at + t * texture_size;
            write_name(out, at, texture.name, texture_name_size);
            header.patch_i32(at + 64, texture.flags);
            header.patch_i32(at + 68, (std::int32_t)texture.image.width);
            header.patch_i32(at + 72, (std::int32_t)texture.image.height);

            std::size_t pixels_at = out.size();
            if (t == 0)
                texture_data_at = pixels_at;
            header.patch_i32(at + 76, (std::int32_t)pixels_at);
            std::size_t expected =
                (std::size_t)texture.image.width * texture.image.height;
            for (std::size_t i = 0; i < expected; i++)
                sink.u8(i < texture.image.pixels.size() ? texture.image.pixels[i] : 0);
            for (std::size_t i = 0; i < 256; i++)
                for (int k = 0; k < 3; k++)
                    sink.u8(texture.image.palette[i][(std::size_t)k]);
        }
        header.patch_i32(180, (std::int32_t)numtextures);
        header.patch_i32(184, (std::int32_t)texture_at);
        header.patch_i32(188, (std::int32_t)texture_data_at);

        header.patch_i32(72, (std::int32_t)out.size()); // length
        return true;
    }
}
