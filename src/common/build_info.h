#pragma once

#include <string>

namespace build_info
{
    const char *version();
    const char *commit();
    const std::string &date();
    const std::string &compiler();
    std::string compiler(const char *scope);
}
