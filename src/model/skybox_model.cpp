#include "skybox_model.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <filesystem>

#include "common/binary.h"
#include "common/filesystem.h"
#include "format/image/quantize.h"
#include "format/mdl/goldsrc_mdl.h"
#include "format/mdl/studio_model.h"

namespace model
{
    namespace
    {
        constexpr std::size_t max_textures_per_model = 64;
        const char *const suffixes[6] = {"up", "lf", "ft", "rt", "bk", "dn"};

        void set_error(std::string *error, const std::string &message)
        {
            if (error)
                *error = message;
        }

        bool power_of_two(unsigned value)
        {
            return value != 0 && (value & (value - 1)) == 0;
        }

        struct point
        {
            float x, y, z;
        };

        point flip_horizontal(point p)
        {
            p.x = -p.x;
            return p;
        }

        point flip_vertical(point p)
        {
            p.y = -p.y;
            return p;
        }

        // These are row-vector quarter turns. Keeping them exact avoids tiny
        // floating point cracks between adjacent tiles.
        point rotate_x_negative(point p)
        {
            return {p.x, -p.z, p.y};
        }

        point rotate_x_positive(point p)
        {
            return {p.x, p.z, -p.y};
        }

        point rotate_y_negative(point p)
        {
            return {p.z, p.y, -p.x};
        }

        point rotate_y_positive(point p)
        {
            return {-p.z, p.y, p.x};
        }

        point rotate_z_positive(point p)
        {
            return {p.y, -p.x, p.z};
        }

        // Port of gchimp SkyMod's rotate_matrix_by_index_relative_to_down.
        point orient(unsigned face, point p)
        {
            switch (face)
            {
            case 0: // up
                p = flip_horizontal(p);
                p = rotate_x_negative(p);
                p = rotate_x_negative(p);
                p = rotate_z_positive(p);
                return rotate_z_positive(p);
            case 1: // left
                return rotate_x_negative(flip_horizontal(p));
            case 2: // front
                return rotate_x_negative(rotate_y_negative(flip_horizontal(p)));
            case 3: // right
                return rotate_x_positive(flip_vertical(p));
            case 4: // back
                return rotate_x_negative(rotate_y_positive(flip_horizontal(p)));
            default: // down
                return flip_vertical(p);
            }
        }

        std::array<float, 3> face_normal(unsigned face)
        {
            static const float normals[6][3] = {
                {0, 0, -1}, {0, -1, 0}, {-1, 0, 0},
                {0, 1, 0},  {1, 0, 0},  {0, 0, 1},
            };
            return {normals[face][0], normals[face][1], normals[face][2]};
        }

        std::string safe_texture_stem(std::string name)
        {
            name = std::filesystem::path(name).stem().string();
            for (char &c : name)
            {
                unsigned char u = (unsigned char)c;
                if (!(std::isalnum(u) || c == '_' || c == '-'))
                    c = '_';
            }
            // suffix + tile coordinates + ".bmp" need twelve bytes.
            if (name.size() > 50)
                name.resize(50);
            return name.empty() ? "skybox" : name;
        }

        format::studio_model make_part(const skybox_model_options &options,
                                       std::size_t index)
        {
            format::studio_model model;
            model.name = options.model_name + std::to_string(index) + ".mdl";
            float half = options.world_size * 0.5f;
            for (int axis = 0; axis < 3; axis++)
            {
                model.bbmin[axis] = -half;
                model.bbmax[axis] = half;
            }

            format::studio_bone root;
            root.name = "root";
            root.parent = -1;
            model.bones.push_back(root);

            format::studio_sequence idle;
            idle.name = "idle";
            idle.fps = 1;
            idle.loop = true;
            idle.frames.push_back({format::studio_bone_key{}});
            model.sequences.push_back(std::move(idle));
            return model;
        }

        void add_vertex(format::studio_model &model, point position,
                        const std::array<float, 3> &normal, float u, float v)
        {
            format::studio_vertex vertex;
            vertex.position[0] = position.x;
            vertex.position[1] = position.y;
            vertex.position[2] = position.z;
            vertex.normal[0] = normal[0];
            vertex.normal[1] = normal[1];
            vertex.normal[2] = normal[2];
            vertex.u = u;
            vertex.v = v;
            vertex.bone = 0;
            model.vertices.push_back(vertex);
        }
    }

    bool load_tga(const std::string &path, rgb_image &out, std::string *error)
    {
        out = rgb_image{};
        std::vector<byte> data;
        if (!fs::read_all(path, data))
        {
            set_error(error, "could not read '" + path + "'");
            return false;
        }
        if (data.size() < 18)
        {
            set_error(error, "'" + path + "' has a truncated TGA header");
            return false;
        }

        binary::reader reader(data);
        byte id_length = 0, colour_map = 0, image_type = 0;
        std::uint16_t ignored16 = 0, width = 0, height = 0;
        byte ignored8 = 0, depth = 0, descriptor = 0;
        if (!reader.u8(id_length) || !reader.u8(colour_map) || !reader.u8(image_type)
            || !reader.u16(ignored16) || !reader.u16(ignored16) || !reader.u8(ignored8)
            || !reader.u16(ignored16) || !reader.u16(ignored16)
            || !reader.u16(width) || !reader.u16(height)
            || !reader.u8(depth) || !reader.u8(descriptor))
        {
            set_error(error, "'" + path + "' has a truncated TGA header");
            return false;
        }
        if (colour_map != 0 || image_type != 2 || (depth != 24 && depth != 32))
        {
            set_error(error, "'" + path
                + "' must be an uncompressed 24-bit or 32-bit true-colour TGA");
            return false;
        }
        if (width == 0 || height == 0)
        {
            set_error(error, "'" + path + "' has zero image dimensions");
            return false;
        }

        std::size_t bytes_per_pixel = depth / 8;
        std::size_t pixels_at = 18 + id_length;
        std::size_t pixel_count = (std::size_t)width * height;
        if (pixels_at > data.size()
            || pixel_count > (data.size() - pixels_at) / bytes_per_pixel)
        {
            set_error(error, "'" + path + "' has truncated pixel data");
            return false;
        }

        out.width = width;
        out.height = height;
        out.rgb.resize(pixel_count * 3);
        bool top_origin = (descriptor & 0x20) != 0;
        bool right_origin = (descriptor & 0x10) != 0;
        for (unsigned file_y = 0; file_y < height; file_y++)
            for (unsigned file_x = 0; file_x < width; file_x++)
            {
                unsigned x = right_origin ? width - 1 - file_x : file_x;
                unsigned y = top_origin ? file_y : height - 1 - file_y;
                std::size_t source =
                    pixels_at + ((std::size_t)file_y * width + file_x) * bytes_per_pixel;
                std::size_t target = ((std::size_t)y * width + x) * 3;
                out.rgb[target + 0] = data[source + 2];
                out.rgb[target + 1] = data[source + 1];
                out.rgb[target + 2] = data[source + 0];
            }
        return true;
    }

    bool build_skybox_models(const std::array<rgb_image, 6> &faces,
                             const skybox_model_options &options,
                             std::vector<skybox_model_part> &out,
                             std::string *error)
    {
        out.clear();
        unsigned face_size = faces[0].width;
        if (face_size < 16 || !power_of_two(face_size)
            || faces[0].height != face_size)
        {
            set_error(error, "sky faces must be square power-of-two images at least 16 pixels");
            return false;
        }
        for (std::size_t i = 0; i < faces.size(); i++)
        {
            const rgb_image &face = faces[i];
            if (face.width != face_size || face.height != face_size)
            {
                set_error(error, "all six sky faces must have identical square dimensions");
                return false;
            }
            if (face.rgb.size() != (std::size_t)face_size * face_size * 3)
            {
                set_error(error, "a sky face has incomplete RGB pixel data");
                return false;
            }
        }
        if (options.tile_size < 16 || options.tile_size > 512
            || !power_of_two(options.tile_size))
        {
            set_error(error, "tile size must be a power of two from 16 through 512");
            return false;
        }
        unsigned tile_size = std::min(options.tile_size, face_size);
        if (face_size % tile_size != 0)
        {
            set_error(error, "sky face size must be evenly divisible by the tile size");
            return false;
        }
        if (!std::isfinite(options.world_size) || options.world_size < 1024.0f
            || options.world_size > 262144.0f)
        {
            set_error(error, "world size must be from 1024 through 262144 units");
            return false;
        }

        unsigned tiles_per_side = face_size / tile_size;
        std::size_t texture_count =
            (std::size_t)6 * tiles_per_side * tiles_per_side;
        std::size_t part_count =
            (texture_count + max_textures_per_model - 1) / max_textures_per_model;
        std::vector<format::studio_model> models;
        models.reserve(part_count);
        for (std::size_t i = 0; i < part_count; i++)
            models.push_back(make_part(options, i));

        std::string texture_stem = safe_texture_stem(options.model_name);
        float half = options.world_size * 0.5f;
        float world_tile = options.world_size / (float)tiles_per_side;
        // One texel inset matches gchimp SkyMod and prevents the studio
        // renderer wrapping the opposite texture edge at a tile boundary.
        float inset = 1.0f / (float)tile_size;
        // gchimp's StudioMdl layer flips SMD V coordinates during compilation.
        // These are the resulting MDL coordinates, after that flip.
        const float uv[4][2] = {
            {inset, inset}, {inset, 1.0f - inset},
            {1.0f - inset, 1.0f - inset}, {1.0f - inset, inset},
        };

        std::size_t overall = 0;
        for (unsigned face_index = 0; face_index < 6; face_index++)
            for (unsigned tile_y = 0; tile_y < tiles_per_side; tile_y++)
                for (unsigned tile_x = 0; tile_x < tiles_per_side; tile_x++, overall++)
                {
                    std::size_t part_index = overall / max_textures_per_model;
                    format::studio_model &model = models[part_index];
                    unsigned local_material = (unsigned)model.textures.size();

                    std::vector<byte> tile((std::size_t)tile_size * tile_size * 3);
                    const rgb_image &source = faces[face_index];
                    for (unsigned y = 0; y < tile_size; y++)
                    {
                        std::size_t source_at =
                            ((std::size_t)(tile_y * tile_size + y) * face_size
                             + tile_x * tile_size) * 3;
                        std::size_t target_at = (std::size_t)y * tile_size * 3;
                        std::copy_n(source.rgb.data() + source_at,
                                    (std::size_t)tile_size * 3,
                                    tile.data() + target_at);
                    }

                    format::studio_texture texture;
                    char coordinates[16];
                    std::snprintf(coordinates, sizeof(coordinates), "%02u%02u",
                                  tile_x, tile_y);
                    texture.name = texture_stem + suffixes[face_index]
                        + coordinates + ".bmp";
                    texture.flags = format::studio_nf_flatshade
                        | format::studio_nf_fullbright | format::studio_nf_nomips;
                    if (!format::quantize_rgb(tile.data(), tile_size, tile_size,
                                              texture.image))
                    {
                        set_error(error, "could not quantize a sky texture tile");
                        return false;
                    }
                    model.textures.push_back(std::move(texture));
                    model.materials.push_back(model.textures.back().name);

                    float min_x = half - world_tile * tile_x;
                    float min_y = half - world_tile * tile_y;
                    point quad[4] = {
                        {min_x, min_y, -half},
                        {min_x, min_y - world_tile, -half},
                        {min_x - world_tile, min_y - world_tile, -half},
                        {min_x - world_tile, min_y, -half},
                    };
                    std::array<float, 3> normal = face_normal(face_index);
                    int first_vertex = (int)model.vertices.size();
                    for (int vertex = 0; vertex < 4; vertex++)
                        add_vertex(model, orient(face_index, quad[vertex]), normal,
                                   uv[vertex][0], uv[vertex][1]);

                    format::studio_mesh mesh;
                    mesh.material = (int)local_material;
                    // StudioMdl reverses every SMD triangle before writing the
                    // GoldSrc command stream. The inward-facing cube depends on
                    // this winding for back-face culling.
                    mesh.indices = {
                        first_vertex + 2, first_vertex + 1, first_vertex + 0,
                        first_vertex + 3, first_vertex + 2, first_vertex + 0,
                    };
                    model.meshes.push_back(std::move(mesh));
                }

        for (std::size_t i = 0; i < models.size(); i++)
        {
            skybox_model_part part;
            part.name = models[i].name;
            part.textures = models[i].textures.size();
            part.triangles = models[i].triangle_count();
            std::string write_error;
            if (!format::write_goldsrc_model(models[i], part.data, &write_error))
            {
                set_error(error, "could not write sky model part "
                    + std::to_string(i) + ": " + write_error);
                return false;
            }
            out.push_back(std::move(part));
        }
        return true;
    }
}
