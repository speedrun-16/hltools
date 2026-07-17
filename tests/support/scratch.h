#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>

namespace test_support
{
    inline std::filesystem::path scratch_directory(const char *name)
    {
        std::string component = name ? name : "";
        if (component.empty() || component == "." || component == ".."
            || component.find_first_of("/\\:") != std::string::npos)
        {
            throw std::runtime_error("invalid test scratch directory name");
        }

        std::filesystem::path path =
            std::filesystem::path(HLTOOLS_TEST_SCRATCH_DIRECTORY) / component;
        std::error_code error;
        std::filesystem::remove_all(path, error);
        if (error)
            throw std::runtime_error("could not clear test scratch directory");
        std::filesystem::create_directories(path, error);
        if (error)
            throw std::runtime_error("could not create test scratch directory");
        return path;
    }
}
