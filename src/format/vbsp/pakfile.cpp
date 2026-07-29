#include "pakfile.h"

#include <algorithm>
#include <cstring>

#include "common/binary.h"

namespace format
{
    namespace
    {
        constexpr unsigned sig_eocd = 0x06054b50;
        constexpr unsigned sig_central = 0x02014b50;
        constexpr unsigned sig_local = 0x04034b50;

        void set_error(std::string *error, const char *message)
        {
            if (error)
                *error = message;
        }

        std::string to_lower(std::string value)
        {
            std::transform(value.begin(), value.end(), value.begin(),
                           [](unsigned char c) { return (char)std::tolower(c); });
            // normalize separators so lookups are slash agnostic
            std::replace(value.begin(), value.end(), '\\', '/');
            return value;
        }

        // locates the end of central directory record by scanning backwards over
        // the (usually empty) trailing comment.
        bool find_eocd(const std::vector<byte> &data, std::size_t &eocd)
        {
            if (data.size() < 22)
                return false;
            std::size_t max_back = std::min<std::size_t>(data.size(), 22 + 65535);
            for (std::size_t i = 0; i <= max_back - 22; i++)
            {
                std::size_t pos = data.size() - 22 - i;
                unsigned sig;
                std::memcpy(&sig, data.data() + pos, 4);
                if (sig == sig_eocd)
                {
                    eocd = pos;
                    return true;
                }
            }
            return false;
        }
    }

    bool pakfile::open(std::vector<byte> archive, std::string *error)
    {
        archive_ = std::move(archive);
        entries_.clear();
        if (archive_.empty())
            return true;

        std::size_t eocd = 0;
        if (!find_eocd(archive_, eocd))
        {
            set_error(error, "pakfile has no zip end-of-central-directory record");
            return false;
        }

        binary::reader head(archive_);
        std::uint16_t total_entries = 0;
        std::uint32_t central_offset = 0;
        if (!head.u16_at(eocd + 10, total_entries)
            || !head.u32_at(eocd + 16, central_offset))
        {
            set_error(error, "pakfile end-of-central-directory record is truncated");
            return false;
        }

        std::size_t cursor = central_offset;
        for (unsigned i = 0; i < total_entries; i++)
        {
            if (cursor + 46 > archive_.size())
            {
                set_error(error, "pakfile central directory is truncated");
                return false;
            }
            unsigned sig;
            std::memcpy(&sig, archive_.data() + cursor, 4);
            if (sig != sig_central)
            {
                set_error(error, "pakfile central directory header is malformed");
                return false;
            }

            binary::reader rec(archive_);
            std::uint16_t method = 0, name_len = 0, extra_len = 0, comment_len = 0;
            std::uint32_t comp_size = 0, uncomp_size = 0, local_offset = 0;
            rec.u16_at(cursor + 10, method);
            rec.u32_at(cursor + 20, comp_size);
            rec.u32_at(cursor + 24, uncomp_size);
            rec.u16_at(cursor + 28, name_len);
            rec.u16_at(cursor + 30, extra_len);
            rec.u16_at(cursor + 32, comment_len);
            rec.u32_at(cursor + 42, local_offset);

            std::size_t name_pos = cursor + 46;
            if (name_pos + name_len > archive_.size())
            {
                set_error(error, "pakfile entry name is truncated");
                return false;
            }
            std::string name((const char *)archive_.data() + name_pos, name_len);

            entry e;
            e.local_header_offset = local_offset;
            e.compressed_size = comp_size;
            e.uncompressed_size = uncomp_size;
            e.method = method;
            if (!name.empty() && name.back() != '/') // skip directory records
                entries_[to_lower(name)] = e;

            cursor = name_pos + name_len + extra_len + comment_len;
        }
        return true;
    }

    bool pakfile::contains(const std::string &name) const
    {
        return entries_.find(to_lower(name)) != entries_.end();
    }

    bool pakfile::extract(const std::string &name, std::vector<byte> &out,
                          std::string *error) const
    {
        auto it = entries_.find(to_lower(name));
        if (it == entries_.end())
        {
            set_error(error, "pakfile entry not found");
            return false;
        }
        const entry &e = it->second;
        if (e.method != 0)
        {
            set_error(error, "pakfile entry is compressed (deflate unsupported yet)");
            return false;
        }

        // the local header's name/extra lengths can differ from the central
        // record's, so the data offset must be read from the local header.
        std::size_t pos = e.local_header_offset;
        if (pos + 30 > archive_.size())
        {
            set_error(error, "pakfile local header is truncated");
            return false;
        }
        unsigned sig;
        std::memcpy(&sig, archive_.data() + pos, 4);
        if (sig != sig_local)
        {
            set_error(error, "pakfile local header is malformed");
            return false;
        }
        binary::reader rec(archive_);
        std::uint16_t name_len = 0, extra_len = 0;
        rec.u16_at(pos + 26, name_len);
        rec.u16_at(pos + 28, extra_len);

        std::size_t data_pos = pos + 30 + name_len + extra_len;
        if (data_pos + e.compressed_size > archive_.size())
        {
            set_error(error, "pakfile entry data extends past the archive");
            return false;
        }
        out.assign(archive_.begin() + data_pos,
                   archive_.begin() + data_pos + e.compressed_size);
        return true;
    }
}
