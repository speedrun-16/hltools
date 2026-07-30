#pragma once

#include <set>
#include <string>

#include "brush.h"

// the reference performs a large amount of work while the map file is still
// being parsed: brush contents are classified and stored, invisible surfaces
// are renamed, origin and boundingbox brushes are fully built (registering
// their planes and texinfos before any normal brush), sky brushes grow clip
// copies, func_group and func_detail merge into worldspawn and info_hullshape
// entities register their shape and disappear the new parser reads raw syntax
// only, so this walk replays those fixups in the exact reference order

namespace csg
{
    struct map_post_options
    {
        // duplicate every sky brush as an invisible clip brush (-noskyclip off)
        bool sky_clip = true;

        // classnames and targetnames whose faces turn invisible (-nullfile)
        std::set<std::string> invisible_items;

        brush_build_options brush;
        bool compile_parameters_consumed = false;
    };

    void post_process_map(map_source &map,
                          plane_store &planes,
                          texinfo_store *texinfos,
                          hull_shape_library &hull_shapes,
                          const map_post_options &options);

    // reads a -nullfile null entity list: one classname or targetname per line
    std::set<std::string> load_invisible_items(const std::string &path);
}
