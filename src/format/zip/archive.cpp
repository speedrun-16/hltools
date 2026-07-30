#include "archive.h"

#include <cctype>
#include <limits>

#include <zip.h>

namespace format
{
    namespace
    {
        void set_error(std::string *out, const char *message)
        {
            if (out)
                *out = message ? message : "unknown ZIP error";
        }

        void set_error(std::string *out, zip_error_t *error)
        {
            set_error(out, zip_error_strerror(error));
        }

        bool map_name(const char *name)
        {
            if (!name || name[0] == '\0')
                return false;
            std::string value(name);
            if (value.find('/') != std::string::npos
                || value.find('\\') != std::string::npos
                || value.size() < 4)
                return false;
            size_t n = value.size();
            return value[n - 4] == '.'
                && std::tolower((unsigned char)value[n - 3]) == 'm'
                && std::tolower((unsigned char)value[n - 2]) == 'a'
                && std::tolower((unsigned char)value[n - 1]) == 'p';
        }

        bool read_open_entry(zip_t *archive, const std::string &name,
                             std::vector<byte> &out, std::string *error)
        {
            zip_stat_t stat;
            zip_stat_init(&stat);
            if (zip_stat(archive, name.c_str(), ZIP_FL_ENC_UTF_8, &stat) < 0
                || stat.size > std::numeric_limits<size_t>::max())
            {
                set_error(error, zip_get_error(archive));
                return false;
            }
            zip_file_t *file =
                zip_fopen(archive, name.c_str(), ZIP_FL_ENC_UTF_8);
            if (!file)
            {
                set_error(error, zip_get_error(archive));
                return false;
            }
            out.resize((size_t)stat.size);
            size_t position = 0;
            while (position < out.size())
            {
                zip_int64_t read =
                    zip_fread(file, out.data() + position, out.size() - position);
                if (read <= 0)
                    break;
                position += (size_t)read;
            }
            zip_fclose(file);
            if (position != out.size())
            {
                set_error(error, "could not read ZIP entry");
                out.clear();
                return false;
            }
            return true;
        }
    }

    bool create_zip(const std::vector<zip_entry> &entries, std::vector<byte> &out,
                    std::string *error)
    {
        out.clear();
        zip_error_t zip_error;
        zip_error_init(&zip_error);
        zip_source_t *backing =
            zip_source_buffer_create(nullptr, 0, 0, &zip_error);
        if (!backing)
        {
            set_error(error, &zip_error);
            zip_error_fini(&zip_error);
            return false;
        }

        zip_source_keep(backing);
        zip_t *archive =
            zip_open_from_source(backing, ZIP_CREATE | ZIP_TRUNCATE, &zip_error);
        if (!archive)
        {
            set_error(error, &zip_error);
            zip_source_free(backing);
            zip_source_free(backing);
            zip_error_fini(&zip_error);
            return false;
        }

        for (const zip_entry &entry : entries)
        {
            zip_source_t *source =
                zip_source_buffer(archive, entry.second.data(),
                                  entry.second.size(), 0);
            if (!source)
            {
                set_error(error, zip_get_error(archive));
                zip_discard(archive);
                zip_source_free(backing);
                zip_error_fini(&zip_error);
                return false;
            }
            zip_int64_t index =
                zip_file_add(archive, entry.first.c_str(), source,
                             ZIP_FL_ENC_UTF_8 | ZIP_FL_OVERWRITE);
            if (index < 0)
            {
                set_error(error, zip_get_error(archive));
                zip_source_free(source);
                zip_discard(archive);
                zip_source_free(backing);
                zip_error_fini(&zip_error);
                return false;
            }
            if (zip_set_file_compression(archive, (zip_uint64_t)index,
                                         ZIP_CM_DEFLATE, 9) < 0)
            {
                set_error(error, zip_get_error(archive));
                zip_discard(archive);
                zip_source_free(backing);
                zip_error_fini(&zip_error);
                return false;
            }
        }

        if (zip_close(archive) < 0)
        {
            set_error(error, zip_get_error(archive));
            zip_discard(archive);
            zip_source_free(backing);
            zip_error_fini(&zip_error);
            return false;
        }

        zip_stat_t stat;
        zip_stat_init(&stat);
        if (zip_source_open(backing) < 0 || zip_source_stat(backing, &stat) < 0
            || stat.size > std::numeric_limits<size_t>::max())
        {
            set_error(error, zip_source_error(backing));
            zip_source_free(backing);
            zip_error_fini(&zip_error);
            return false;
        }
        out.resize((size_t)stat.size);
        size_t position = 0;
        while (position < out.size())
        {
            zip_int64_t read = zip_source_read(
                backing, out.data() + position, out.size() - position);
            if (read <= 0)
                break;
            position += (size_t)read;
        }
        zip_source_close(backing);
        zip_source_free(backing);
        zip_error_fini(&zip_error);
        if (position != out.size())
        {
            set_error(error, "could not read completed ZIP archive");
            out.clear();
            return false;
        }
        return true;
    }

    bool read_zip_entry(const std::vector<byte> &bytes, const std::string &name,
                        std::vector<byte> &out, std::string *error)
    {
        out.clear();
        zip_error_t zip_error;
        zip_error_init(&zip_error);
        zip_source_t *source =
            zip_source_buffer_create(bytes.data(), bytes.size(), 0, &zip_error);
        if (!source)
        {
            set_error(error, &zip_error);
            zip_error_fini(&zip_error);
            return false;
        }
        zip_t *archive = zip_open_from_source(source, ZIP_RDONLY, &zip_error);
        if (!archive)
        {
            set_error(error, &zip_error);
            zip_source_free(source);
            zip_error_fini(&zip_error);
            return false;
        }

        bool ok = read_open_entry(archive, name, out, error);
        zip_close(archive);
        zip_error_fini(&zip_error);
        return ok;
    }

    bool read_zip_map(const std::vector<byte> &bytes, std::vector<byte> &out,
                      std::string *entry_name, std::string *error)
    {
        out.clear();
        zip_error_t zip_error;
        zip_error_init(&zip_error);
        zip_source_t *source =
            zip_source_buffer_create(bytes.data(), bytes.size(), 0, &zip_error);
        if (!source)
        {
            set_error(error, &zip_error);
            zip_error_fini(&zip_error);
            return false;
        }
        zip_t *archive = zip_open_from_source(source, ZIP_RDONLY, &zip_error);
        if (!archive)
        {
            set_error(error, &zip_error);
            zip_source_free(source);
            zip_error_fini(&zip_error);
            return false;
        }

        std::string selected;
        zip_int64_t preferred_index = zip_name_locate(
            archive, embedded_map_name, ZIP_FL_ENC_UTF_8 | ZIP_FL_NOCASE);
        if (preferred_index >= 0)
        {
            const char *actual = zip_get_name(
                archive, (zip_uint64_t)preferred_index, ZIP_FL_ENC_UTF_8);
            if (actual)
                selected = actual;
        }
        else
        {
            zip_int64_t count = zip_get_num_entries(archive, 0);
            for (zip_int64_t i = 0; i < count; i++)
            {
                const char *name =
                    zip_get_name(archive, (zip_uint64_t)i, ZIP_FL_ENC_UTF_8);
                if (map_name(name))
                {
                    selected = name;
                    break;
                }
            }
        }

        if (selected.empty())
        {
            set_error(error, "embedded ZIP contains no top-level .map source");
            zip_close(archive);
            zip_error_fini(&zip_error);
            return false;
        }
        bool ok = read_open_entry(archive, selected, out, error);
        zip_close(archive);
        zip_error_fini(&zip_error);
        if (ok && entry_name)
            *entry_name = selected;
        return ok;
    }
}
