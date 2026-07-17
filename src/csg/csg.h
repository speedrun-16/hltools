#pragma once

#include <string>
#include <vector>

#include "brush.h"
#include "map_parser.h"
#include "map_post.h"
#include "textures.h"
#include "wad_path.h"

namespace csg
{
    constexpr vec_t default_brush_union_threshold = (vec_t)0.0;

    struct csg_options
    {
        brush_build_options brush;
        wad_build_options wad;
        std::string hull_file_path;
        vec_t brush_union_threshold = default_brush_union_threshold;

        // -noskyclip disables the invisible clip copy grown by sky brushes
        bool sky_clip = true;

        // -nullfile: classnames and targetnames whose faces turn invisible
        std::set<std::string> invisible_items;
    };

    struct csg_result
    {
        map_source map;
        hull_shape_library hull_shapes;
        texinfo_store texinfos;
        std::vector<brush_plane> planes;
        std::vector<byte> textures;
        std::vector<byte> temp_wad;
        std::vector<built_entity> entities;
    };

    bool parse_clip_type(const char *value, clip_type &out);
    const char *clip_type_name(clip_type type);

    csg_result run_csg(map_source map, const csg_options &options);
}
