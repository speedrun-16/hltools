#include "vpk.h"

#include <cctype>
#include <cstdio>
#include <cstring>
#include <fstream>

#include "common/filesystem.h"

namespace format
{
    namespace
    {
        constexpr unsigned vpk_signature = 0x55aa1234;
        // an entry with this archive index stores its data in the dir file
        // itself, after the tree
        constexpr unsigned vpk_dir_archive = 0x7fff;

        std::string normalize_path(const std::string &path)
        {
            std::string out = path;
            for (char &c : out)
            {
                if (c == '\\')
                    c = '/';
                else
                    c = (char)std::tolower((unsigned char)c);
            }
            return out;
        }

        template <typename T>
        bool read_value(const std::vector<byte> &data, std::size_t &pos, T &out)
        {
            if (data.size() - pos < sizeof(T))
                return false;
            std::memcpy(&out, data.data() + pos, sizeof(T));
            pos += sizeof(T);
            return true;
        }

        bool read_string(const std::vector<byte> &data, std::size_t &pos,
                         std::string &out)
        {
            out.clear();
            while (pos < data.size() && data[pos] != 0)
                out.push_back((char)data[pos++]);
            if (pos >= data.size())
                return false;
            pos++; // consume the terminator
            return true;
        }
    }

    bool vpk_archive::open(const std::string &dir_path)
    {
        entries_.clear();
        dir_path_ = dir_path;

        std::vector<byte> data;
        if (!fs::read_all(dir_path, data))
            return false;

        std::size_t pos = 0;
        unsigned signature = 0, version = 0, tree_size = 0;
        if (!read_value(data, pos, signature) || !read_value(data, pos, version)
            || !read_value(data, pos, tree_size))
            return false;
        if (signature != vpk_signature || (version != 1 && version != 2))
            return false;
        if (version == 2)
            pos += 16; // file data / md5 / signature section sizes
        std::size_t header_size = pos;
        data_base_ = (unsigned)(header_size + tree_size);

        // the tree nests extension -> directory -> file name; empty strings
        // close each level and a single space means "no directory"
        std::string extension, directory, name;
        while (true)
        {
            if (!read_string(data, pos, extension))
                return false;
            if (extension.empty())
                break;
            while (true)
            {
                if (!read_string(data, pos, directory))
                    return false;
                if (directory.empty())
                    break;
                while (true)
                {
                    if (!read_string(data, pos, name))
                        return false;
                    if (name.empty())
                        break;

                    unsigned crc = 0, entry_offset = 0, entry_length = 0;
                    unsigned short preload_size = 0, archive_index = 0, terminator = 0;
                    if (!read_value(data, pos, crc)
                        || !read_value(data, pos, preload_size)
                        || !read_value(data, pos, archive_index)
                        || !read_value(data, pos, entry_offset)
                        || !read_value(data, pos, entry_length)
                        || !read_value(data, pos, terminator))
                        return false;
                    if (data.size() - pos < preload_size)
                        return false;

                    entry e;
                    e.archive_index = archive_index;
                    e.offset = entry_offset;
                    e.length = entry_length;
                    e.preload.assign(data.begin() + (std::ptrdiff_t)pos,
                                     data.begin() + (std::ptrdiff_t)(pos + preload_size));
                    pos += preload_size;

                    std::string full = directory == " "
                        ? name + "." + extension
                        : directory + "/" + name + "." + extension;
                    entries_[normalize_path(full)] = std::move(e);
                }
            }
        }
        return true;
    }

    std::string vpk_archive::archive_path(unsigned index) const
    {
        std::size_t tag = dir_path_.rfind("_dir.vpk");
        if (tag == std::string::npos)
            return {};
        char suffix[16];
        std::snprintf(suffix, sizeof(suffix), "_%03u.vpk", index);
        return dir_path_.substr(0, tag) + suffix;
    }

    bool vpk_archive::extract(const std::string &path, std::vector<byte> &out) const
    {
        auto it = entries_.find(normalize_path(path));
        if (it == entries_.end())
            return false;
        const entry &e = it->second;

        out = e.preload;
        if (e.length == 0)
            return true;

        std::string source;
        std::size_t offset = e.offset;
        if (e.archive_index == vpk_dir_archive)
        {
            source = dir_path_;
            offset += data_base_;
        }
        else
        {
            source = archive_path(e.archive_index);
        }

        std::ifstream input(source, std::ios::binary);
        if (!input.seekg((std::streamoff)offset))
            return false;
        std::size_t preload_size = out.size();
        out.resize(preload_size + e.length);
        if (!input.read((char *)out.data() + preload_size, e.length))
        {
            out.clear();
            return false;
        }
        return true;
    }
}
