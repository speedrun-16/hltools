#pragma once

#include <string>

#include "format/bsp/data.h"
#include "csg.h"

namespace csg
{
    // mutates the map's entities (model numbers, zhlt_usemodel reordering,
    // light styles) before converting, so call it after the intermediate
    // files are written optimize_lights is -nolightopt off (the default)
    format::map_data build_bsp_data(csg_result &result, bool optimize_lights = true);

    // -onlyents: runs the entity passes and swaps only the entity lump into
    // an existing bsp, keeping its wad key and all geometry lumps
    void replace_entities(csg_result &result, format::map_data &existing,
                          bool optimize_lights = true);

    // the csg -> bsp handoff, byte for byte what the intermediate files hold
    // (text buffers use \n; the disk writer converts to \r\n) the bsp stage
    // parses these with the same tokenizer whether they come from memory or
    // disk, so the %58f vertex quantization round trip is preserved either way
    struct intermediate_data
    {
        std::string surfaces[num_hulls]; // p0 through p3 per hull face text
        std::string brushes[num_hulls];  // b0 through b3 per hull detail brush text
        std::string hull_sizes;          // hsz text
        std::vector<byte> planes;        // pln binary plane records
    };

    intermediate_data build_intermediate_data(const csg_result &result,
                                              const brush_build_options &options);

    // writes the intermediate data plus the temp wad (wa_) next to base_path
    bool write_intermediate_files(const std::string &base_path,
                                  const intermediate_data &data,
                                  const csg_result &result);
    bool write_intermediate_files(const std::string &base_path,
                                  const csg_result &result,
                                  const brush_build_options &options);
}
