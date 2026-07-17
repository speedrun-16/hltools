// round trip proof for the format layer: load a bsp into map_data and write it
// straight back out the result must be byte identical to the input, which
// compile_diffpy verifies this format check before any stage logic
// starts reading or mutating map_data

#include <cstdio>

#include "format/bsp/data.h"
#include "format/bsp/file.h"
#include "format/bsp/usage_chart.h"

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::printf("usage: bsp_roundtrip <in.bsp> <out.bsp>\n");
        return 2;
    }

    format::map_data map;
    if (!format::bsp_file::load(argv[1], map))
    {
        std::printf("load failed: %s\n", argv[1]);
        return 1;
    }

    std::printf("loaded %s: %zu planes, %zu faces, %zu leafs, %zu bytes lighting, "
                "%zu bytes entities\n",
                argv[1], map.planes.size(), map.faces.size(), map.leafs.size(),
                map.lighting.size(), map.entities.size());

    if (!format::bsp_file::write(argv[2], map))
    {
        std::printf("write failed: %s\n", argv[2]);
        return 1;
    }
    std::printf("wrote %s\n", argv[2]);

    // the usage chart runs over the loaded map, exercising format/chart
    format::print_usage_chart(map);
    return 0;
}
