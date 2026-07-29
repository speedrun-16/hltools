#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "common/types.h"

namespace format
{
    // read-only view of a valve pack (vpk) directory archive, the container
    // source games use for stock content. the *_dir.vpk file carries the file
    // tree (plus optional preload bytes); file data lives either in numbered
    // companion archives (*_000.vpk, ...) or embedded after the tree.
    class vpk_archive
    {
    public:
        // parses a *_dir.vpk tree. returns false when the file is missing or
        // is not a v1/v2 vpk directory.
        bool open(const std::string &dir_path);

        // extracts a file by its archive-relative path (case insensitive,
        // forward or back slashes). returns false when absent.
        bool extract(const std::string &path, std::vector<byte> &out) const;

        std::size_t file_count() const { return entries_.size(); }

    private:
        struct entry
        {
            unsigned archive_index = 0;
            unsigned offset = 0;
            unsigned length = 0;
            std::vector<byte> preload;
        };

        // path of a numbered companion archive for this directory
        std::string archive_path(unsigned index) const;

        std::string dir_path_;
        unsigned data_base_ = 0; // start of embedded data within the dir file
        std::unordered_map<std::string, entry> entries_;
    };
}
