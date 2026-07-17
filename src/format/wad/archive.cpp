#include "archive.h"

#include <cctype>
#include <climits>
#include <cstring>

#include "common/binary.h"
#include "common/filesystem.h"

namespace format
{
    namespace
    {
        constexpr int wad_header_size = 12;
        constexpr int wad_lump_size = 32;

        void cleanup_name(const char *in, char out[max_wad_name])
        {
            int i = 0;
            for (; i < max_wad_name; i++)
            {
                if (!in[i])
                    break;
                unsigned char c = (unsigned char)in[i];
                out[i] = (char)std::toupper(c);
            }
            for (; i < max_wad_name; i++)
                out[i] = 0;
        }

        void set_error(std::string *error, const char *message)
        {
            if (error)
                *error = message;
        }

    }

    bool wad_archive::load(const std::string &path, std::string *error)
    {
        path_ = path;
        lumps_.clear();

        std::vector<byte> data;
        if (!fs::read_all(path, data))
        {
            set_error(error, "could not open wad file");
            return false;
        }
        if (data.size() < wad_header_size)
        {
            set_error(error, "wad file is too small");
            return false;
        }
        if (std::memcmp(data.data(), "WAD2", 4) != 0 && std::memcmp(data.data(), "WAD3", 4) != 0)
        {
            set_error(error, "wad file has an invalid header");
            return false;
        }

        binary::reader header(data);
        std::int32_t num_lumps;
        std::int32_t info_table_ofs;
        if (!header.seek(4) || !header.i32(num_lumps) || !header.i32(info_table_ofs))
        {
            set_error(error, "wad file has a truncated header");
            return false;
        }
        if (num_lumps < 0 || info_table_ofs < 0)
        {
            set_error(error, "wad file has invalid directory values");
            return false;
        }

        size_t directory = (size_t)info_table_ofs;
        size_t directory_size = (size_t)num_lumps * wad_lump_size;
        if (directory > data.size() || directory_size > data.size() - directory)
        {
            set_error(error, "wad directory is outside the file");
            return false;
        }

        lumps_.reserve((size_t)num_lumps);
        for (int i = 0; i < num_lumps; i++)
        {
            size_t offset = directory + (size_t)i * wad_lump_size;
            wad_lump lump;
            binary::reader entry(data.data() + offset, wad_lump_size);
            if (!entry.i32(lump.filepos) || !entry.i32(lump.disksize)
                || !entry.i32(lump.size))
            {
                set_error(error, "wad directory entry is truncated");
                return false;
            }
            lump.type = (char)data[offset + 12];
            lump.compression = (char)data[offset + 13];
            lump.pad1 = (char)data[offset + 14];
            lump.pad2 = (char)data[offset + 15];
            cleanup_name(reinterpret_cast<const char *>(data.data() + offset + 16), lump.name);

            if (lump.filepos < 0 || lump.disksize < 0)
            {
                set_error(error, "wad lump has invalid bounds");
                return false;
            }
            size_t filepos = (size_t)lump.filepos;
            size_t disksize = (size_t)lump.disksize;
            if (filepos > data.size() || disksize > data.size() - filepos)
            {
                set_error(error, "wad lump data is outside the file");
                return false;
            }
            lump.data.assign(data.begin() + filepos, data.begin() + filepos + disksize);
            lumps_.push_back(std::move(lump));
        }

        return true;
    }

    const wad_lump *wad_archive::find_texture(const char *name) const
    {
        char clean[max_wad_name];
        cleanup_name(name, clean);
        for (const auto &lump : lumps_)
        {
            if (std::strncmp(lump.name, clean, max_wad_name) == 0)
                return &lump;
        }
        return nullptr;
    }

    bool mip_texture_from_lump(const wad_lump &lump, mip_texture &out, std::string *error)
    {
        if ((unsigned char)lump.type != (unsigned char)wad_miptex_type)
        {
            set_error(error, "lump is not a GoldSrc miptex texture");
            return false;
        }
        if (lump.compression != 0)
        {
            set_error(error, "compressed WAD textures are not supported");
            return false;
        }
        return decode_mip_texture(lump.data, out, error);
    }

    bool write_wad3(const std::string &path, const std::vector<mip_texture> &textures,
                    std::string *error)
    {
        if (textures.size() > (size_t)INT_MAX)
        {
            set_error(error, "too many textures for a WAD3 archive");
            return false;
        }

        std::vector<byte> out(12, 0);
        std::vector<byte> directory;
        binary::writer output(out);
        binary::writer directory_output(directory);
        directory.reserve(textures.size() * (size_t)wad_lump_size);
        for (const mip_texture &texture : textures)
        {
            if (texture.name.empty() || texture.name.size() >= max_wad_name)
            {
                set_error(error, "texture name must contain 1 to 15 characters");
                return false;
            }
            mip_texture checked;
            std::string detail;
            if (!decode_mip_texture(texture.data, checked, &detail))
            {
                if (error)
                    *error = std::string("texture '") + texture.name + "': " + detail;
                return false;
            }
            if (out.size() > (size_t)INT_MAX
                || texture.data.size() > (size_t)INT_MAX
                || texture.data.size() > (size_t)INT_MAX - out.size())
            {
                set_error(error, "WAD3 archive exceeds its 32-bit size limit");
                return false;
            }
            directory_output.i32((int)out.size());
            directory_output.i32((int)texture.data.size());
            directory_output.i32((int)texture.data.size());
            directory.push_back((byte)wad_miptex_type);
            directory.push_back(0);
            directory.push_back(0);
            directory.push_back(0);
            char name[max_wad_name] = {};
            std::memcpy(name, texture.name.data(), texture.name.size());
            directory.insert(directory.end(), name, name + max_wad_name);
            output.raw(texture.data);
        }
        if (out.size() > (size_t)INT_MAX || directory.size() > (size_t)INT_MAX - out.size())
        {
            set_error(error, "WAD3 directory exceeds its 32-bit size limit");
            return false;
        }
        int directory_offset = (int)out.size();
        output.raw(directory);
        std::memcpy(out.data(), "WAD3", 4);
        if (!output.patch_i32(4, (int)textures.size())
            || !output.patch_i32(8, directory_offset))
        {
            set_error(error, "could not finalize WAD3 header");
            return false;
        }
        if (!fs::write_all(path, out.data(), out.size()))
        {
            set_error(error, "could not write WAD3 file");
            return false;
        }
        return true;
    }
}
