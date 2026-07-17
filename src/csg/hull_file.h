#pragma once

#include <string>

#include "brush.h"

namespace csg
{
    void load_hull_file(const std::string &path, brush_build_options &options);
}
