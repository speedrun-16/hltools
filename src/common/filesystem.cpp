#include "filesystem.h"

#include <filesystem>
#include <fstream>

namespace stdfs = std::filesystem;

namespace fs
{
    bool exists(const std::string &path)
    {
        std::error_code ec;
        return stdfs::exists(path, ec);
    }

    long long size(const std::string &path)
    {
        std::error_code ec;
        auto n = stdfs::file_size(path, ec);
        return ec ? -1 : (long long)n;
    }

    bool read_all(const std::string &path, std::vector<unsigned char> &out)
    {
        std::ifstream in(path, std::ios::binary | std::ios::ate);
        if (!in)
            return false;
        std::streamsize len = in.tellg();
        if (len < 0)
            return false;
        in.seekg(0, std::ios::beg);
        out.resize((size_t)len);
        if (len > 0)
            in.read(reinterpret_cast<char *>(out.data()), len);
        return !in.fail();
    }

    bool write_all(const std::string &path, const void *data, size_t len)
    {
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        if (!out)
            return false;
        if (len > 0)
            out.write(reinterpret_cast<const char *>(data), (std::streamsize)len);
        return out.good();
    }

    std::string with_extension(const std::string &path, const char *ext)
    {
        return stdfs::path(path).replace_extension(ext).string();
    }

    std::string strip_extension(const std::string &path)
    {
        stdfs::path p(path);
        return (p.parent_path() / p.stem()).string();
    }

    std::string directory(const std::string &path)
    {
        return stdfs::path(path).parent_path().string();
    }

    std::string filename(const std::string &path)
    {
        return stdfs::path(path).filename().string();
    }

    bool make_directory(const std::string &path)
    {
        std::error_code ec;
        stdfs::create_directories(path, ec);
        return stdfs::exists(path, ec);
    }
}
