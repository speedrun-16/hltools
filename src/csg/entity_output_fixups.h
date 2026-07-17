#pragma once

#include <vector>

#include "format/bsp/entity_lump.h"

namespace csg
{
    // apply the compiler only entity conversions immediately before csg writes
    // the bsp entity lump
    void apply_entity_output_fixups(std::vector<format::entity> &entities,
                                    bool optimize_lights);
}
