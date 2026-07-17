#include <cmath>
#include <cstring>

#include "../common/binary.h"
#include "../common/error.h"
#include "../common/filesystem.h"
#include "../common/log.h"
#include "../common/string_util.h"
#include "internal.h"

// loads the texture pixels rad needs for reflectivity and translucency the
// bsp's miptex lump usually carries only names (csg strips pixel data unless
// wadincluded), so missing textures come from the csg temp wad "<map>wa_" or,
// failing that, from wad files named by the worldspawn wad key inside the
// -wadinclude search folders

namespace rad
{
    namespace
    {
        // last path component of a windows or unix style path
        std::string extract_file(const std::string &path)
        {
            size_t pos = path.find_last_of("/\\:");
            if (pos == std::string::npos)
                return path;
            return path.substr(pos + 1);
        }

        // append the extension unless the name already carries one
        std::string default_extension(const std::string &name, const char *ext)
        {
            size_t slash = name.find_last_of("/\\");
            size_t dot = name.find_last_of('.');
            if (dot != std::string::npos && (slash == std::string::npos || dot > slash))
                return name;
            return name + ext;
        }

        void open_wad_file(rad_state &state, const std::string &name, bool fullpath = false)
        {
            format::wad_archive wad;
            std::string error;
            if (fullpath)
            {
                if (!wad.load(name, &error))
                {
                    err::fatal("Couldn't open %s (%s)", name.c_str(), error.c_str());
                }
            }
            else
            {
                bool found = false;
                for (const std::string &dir : state.options.wad_folders)
                {
                    std::string path = dir + "\\" + name;
                    if (wad.load(path, nullptr))
                    {
                        found = true;
                        break;
                    }
                }
                if (!found)
                {
                    err::fatal("Could not locate wad file %s", name.c_str());
                }
            }
            logging::info("Using Wadfile: %s\n", wad.path().c_str());
            state.wad_files.push_back(std::move(wad));
        }

        void default_texture(rad_texture *tex, const char *name)
        {
            int i;
            tex->width = 16;
            tex->height = 16;
            std::strncpy(tex->name, name, sizeof(tex->name));
            tex->name[16 - 1] = '\0';
            tex->canvas.assign((size_t)(tex->width * tex->height), 0);
            for (i = 0; i < 256; i++)
            {
                tex->palette[i][0] = 0x80;
                tex->palette[i][1] = 0x80;
                tex->palette[i][2] = 0x80;
            }
        }

        void load_texture(rad_texture *tex, const format::miptex_t *mt, int size)
        {
            int i, j;
            const format::miptex_t *header = mt;
            const byte *data = (const byte *)mt;
            tex->width = (int)header->width;
            tex->height = (int)header->height;
            std::strncpy(tex->name, header->name, sizeof(tex->name));
            tex->name[16 - 1] = '\0';
            if (tex->width <= 0 || tex->height <= 0 ||
                tex->width % (2 * 1 << (format::mip_levels - 1)) != 0 || tex->height % (2 * (1 << (format::mip_levels - 1))) != 0)
            {
                err::fatal("Texture '%s': dimension (%dx%d) is not multiple of %d.", tex->name, tex->width, tex->height, 2 * (1 << (format::mip_levels - 1)));
            }
            int mipsize;
            for (mipsize = 0, i = 0; i < format::mip_levels; i++)
            {
                if ((int)mt->offsets[i] != (int)sizeof(format::miptex_t) + mipsize)
                {
                    err::fatal("Texture '%s': unexpected miptex offset.", tex->name);
                }
                mipsize += (tex->width >> i) * (tex->height >> i);
            }
            if (size < (int)sizeof(format::miptex_t) + mipsize + 2 + 256 * 3)
            {
                err::fatal("Texture '%s': no enough data.", tex->name);
            }
            unsigned short palette_size;
            std::memcpy(&palette_size, &data[sizeof(format::miptex_t) + mipsize], sizeof(palette_size));
            if (palette_size != 256)
            {
                err::fatal("Texture '%s': palette size is not 256.", tex->name);
            }
            tex->canvas.resize((size_t)(tex->width * tex->height));
            for (i = 0; i < tex->height; i++)
            {
                for (j = 0; j < tex->width; j++)
                {
                    tex->canvas[(size_t)(i * tex->width + j)] = data[sizeof(format::miptex_t) + i * tex->width + j];
                }
            }
            for (i = 0; i < 256; i++)
            {
                for (j = 0; j < 3; j++)
                {
                    tex->palette[i][j] = data[sizeof(format::miptex_t) + mipsize + 2 + i * 3 + j];
                }
            }
        }

        bool terminated_string(const char *buffer, int size)
        {
            for (int x = 0; x < size; x++)
            {
                if (buffer[x] == 0)
                    return true;
            }
            return false;
        }

        void load_texture_from_wad(rad_state &state, rad_texture *tex, const format::miptex_t *header)
        {
            tex->width = (int)header->width;
            tex->height = (int)header->height;
            std::strncpy(tex->name, header->name, sizeof(tex->name));
            tex->name[16 - 1] = '\0';
            size_t w;
            for (w = 0; w < state.wad_files.size(); w++)
            {
                const format::wad_archive &wad = state.wad_files[w];
                const format::wad_lump *found = wad.find_texture(tex->name);
                if (found)
                {
                    // 67 is the miptex lump type in wad3 files
                    if (found->type != 67 || found->compression != 0)
                        continue;
                    if (found->disksize < (int)sizeof(format::miptex_t))
                    {
                        logging::warn("Texture '%s': invalid texture data in '%s'.", tex->name, wad.path().c_str());
                        continue;
                    }
                    const format::miptex_t *mt = (const format::miptex_t *)found->data.data();
                    if (!terminated_string(mt->name, 16))
                    {
                        logging::warn("Texture '%s': invalid texture data in '%s'.", tex->name, wad.path().c_str());
                        continue;
                    }
                    if (!str::iequals(mt->name, tex->name))
                    {
                        logging::warn("Texture '%s': texture name '%s' differs from its reference name '%s' in '%s'.", tex->name, mt->name, tex->name, wad.path().c_str());
                    }
                    load_texture(tex, mt, found->disksize);
                    break;
                }
            }
            if (w == state.wad_files.size())
            {
                logging::warn("Texture '%s': texture is not found in wad files.", tex->name);
                default_texture(tex, tex->name);
                return;
            }
        }
    }

    // texture name for a texinfo number, read straight from the miptex lump
    const char *texture_by_number(const rad_state &state, int texinfo)
    {
        if (texinfo == -1)
            return "";

        const format::texinfo_t *info = &state.map->texinfo[(size_t)texinfo];
        if (info->miptex < 0)
            return "";
        binary::reader textures(state.map->textures);
        std::int32_t ofs;
        if (!textures.i32_at(sizeof(int) + (size_t)info->miptex * sizeof(int), ofs)
            || ofs < 0 || (size_t)ofs + sizeof(format::miptex_t) > state.map->textures.size())
            return "";
        const format::miptex_t *mt = (const format::miptex_t *)&state.map->textures[(size_t)ofs];
        return mt->name;
    }

    void try_open_wad_files(rad_state &state)
    {
        if (!state.wad_files_opened)
        {
            state.wad_files_opened = true;
            std::string filename = state.base_path + ".wa_";
            if (fs::exists(filename))
            {
                open_wad_file(state, filename, true);
            }
            else
            {
                logging::warn("Couldn't open %s", filename.c_str());
                logging::info("Opening wad files from directories:\n");
                if (state.options.wad_folders.empty())
                {
                    logging::warn("No wad directories have been set.");
                }
                else
                {
                    for (const std::string &dir : state.options.wad_folders)
                    {
                        logging::info("  %s\n", dir.c_str());
                    }
                }
                const char *value = state.entities.empty() ? "" : state.entities[0].value("wad");
                std::string path;
                for (size_t i = 0; i < std::strlen(value) + 1; i++)
                {
                    if (value[i] == ';' || value[i] == '\0')
                    {
                        if (!path.empty())
                        {
                            std::string name = extract_file(path);
                            name = default_extension(name, ".wad");
                            open_wad_file(state, name);
                        }
                        path.clear();
                    }
                    else
                    {
                        path.push_back(value[i]);
                    }
                }
            }
        }
    }

    void try_close_wad_files(rad_state &state)
    {
        if (state.wad_files_opened)
        {
            state.wad_files_opened = false;
            state.wad_files.clear();
        }
    }

    void load_textures(rad_state &state)
    {
        const format::map_data &map = *state.map;
        std::int32_t numtextures = 0;
        binary::reader texture_input(map.textures);
        if (!texture_input.i32(numtextures) || numtextures < 0
            || (size_t)numtextures > texture_input.remaining() / sizeof(std::int32_t))
        {
            logging::warn("Invalid texture directory in '%s'.", state.base_path.c_str());
            state.textures.clear();
            return;
        }
        state.textures.resize((size_t)numtextures);
        for (int i = 0; i < numtextures; i++)
        {
            std::int32_t offset = -1;
            texture_input.i32_at(sizeof(int) + (size_t)i * sizeof(int), offset);
            int size = (int)map.textures.size() - offset;
            rad_texture *tex = &state.textures[(size_t)i];
            if (state.options.notextures)
            {
                default_texture(tex, "DEFAULT");
            }
            else if (offset < 0 || size < (int)sizeof(format::miptex_t))
            {
                logging::warn("Invalid texture data in '%s'.", state.base_path.c_str());
                default_texture(tex, "");
            }
            else
            {
                const format::miptex_t *mt = (const format::miptex_t *)&map.textures[(size_t)offset];
                if (mt->offsets[0])
                {
                    load_texture(tex, mt, size);
                }
                else
                {
                    try_open_wad_files(state);
                    load_texture_from_wad(state, tex, mt);
                }
            }
            {
                vec3v total{};
                for (int j = 0; j < tex->width * tex->height; j++)
                {
                    vec3v reflectivity;
                    if (tex->name[0] == '{' && tex->canvas[(size_t)j] == 0xFF)
                    {
                        reflectivity = vec3v{0.0, 0.0, 0.0};
                    }
                    else
                    {
                        for (int k = 0; k < 3; k++)
                        {
                            reflectivity[k] = (vec_t)(tex->palette[tex->canvas[(size_t)j]][k] * (1.0 / 255.0));
                        }
                        for (int k = 0; k < 3; k++)
                        {
                            reflectivity[k] = (vec_t)std::pow((double)reflectivity[k], state.options.texreflectgamma);
                        }
                        math::scale(reflectivity, state.options.texreflectscale, reflectivity);
                    }
                    math::add(total, reflectivity, total);
                }
                math::scale(total, 1.0 / (double)(tex->width * tex->height), total);
                math::copy(total, tex->reflectivity);
                vec_t maximum = tex->reflectivity[0] > tex->reflectivity[1]
                    ? (tex->reflectivity[0] > tex->reflectivity[2] ? tex->reflectivity[0] : tex->reflectivity[2])
                    : (tex->reflectivity[1] > tex->reflectivity[2] ? tex->reflectivity[1] : tex->reflectivity[2]);
                if (maximum > 1.0 + math::normal_epsilon)
                {
                    logging::warn("Texture '%s': reflectivity (%f,%f,%f) greater than 1.0.", tex->name, tex->reflectivity[0], tex->reflectivity[1], tex->reflectivity[2]);
                }
            }
        }
        if (!state.options.notextures)
        {
            logging::info("\n  %-14s %d referenced\n", "textures", numtextures);
            try_close_wad_files(state);
        }
    }
}
