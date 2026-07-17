#pragma once

#include <vector>

#include "csg.h"

namespace csg
{
    struct brush_union_warning
    {
        int entity_num = 0;
        int brush_num = 0;
        int other_brush_num = 0;
        vec_t percent = 0;
    };

    std::vector<brush_union_warning> calculate_brush_union_warnings(const csg_result &result,
                                                                    vec_t threshold,
                                                                    vec_t world_extent);
}
