#pragma once

#include <string>
#include <vector>

// path and file helpers built on std::filesystem, replacing the old filelib
// cold path only, so std::string and vector are fine here

namespace fs
{
    bool exists(const std::string &path);
    long long size(const std::string &path);

    // read a whole file into memory; returns false if it cannot be opened
    bool read_all(const std::string &path, std::vector<unsigned char> &out);
    bool write_all(const std::string &path, const void *data, size_t len);

    // "maps/foomap" -> "maps/foobsp" (replaces the final extension, adding one
    // if absent) ext includes the dot, for example "bsp"
    std::string with_extension(const std::string &path, const char *ext);

    // "maps/foomap" -> "maps/foo" (strips the final extension only)
    std::string strip_extension(const std::string &path);

    // directory portion, for example "maps/foomap" -> "maps"
    std::string directory(const std::string &path);

    // final path component, for example "maps/foomap" -> "foomap"
    std::string filename(const std::string &path);

    // create the directory (and any missing parents); true if it exists after
    bool make_directory(const std::string &path);
}
