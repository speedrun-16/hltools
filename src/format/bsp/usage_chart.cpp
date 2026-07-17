#include "usage_chart.h"

#include <cstdio>

#include "common/limits.h"
#include "common/log.h"
#include "common/string_util.h"
#include "data.h"

namespace format
{
    namespace
    {
        long long g_total = 0;
        bool g_full_console = false;

        // a fixed record lump: count against a max element count the full chart
        // goes to the logfile; the console shows only the rows worth watching
        void array_row(const char *name, long long count, long long max, long long elem_size)
        {
            double pct = max ? count * 100.0 / max : 0.0;
            g_total += count * elem_size;
            char b1[32], b2[32], line[128];
            std::snprintf(line, sizeof(line), "  %-16s %10s / %-10s (%4.1f%%)\n", name,
                          str::with_commas(count, b1, sizeof(b1)),
                          str::with_commas(max, b2, sizeof(b2)), pct);
            logging::file("%s", line);
            if (g_full_console || pct >= 15.0)
                logging::console("%s", line);
        }

        // a variable byte lump: size against a byte ceiling
        void blob_row(const char *name, long long bytes, long long max)
        {
            double pct = max ? bytes * 100.0 / max : 0.0;
            g_total += bytes;
            char b1[32], b2[32], line[128];
            std::snprintf(line, sizeof(line), "  %-16s %10s / %-10s (%4.1f%%)\n", name,
                          str::human_bytes(bytes, b1, sizeof(b1)),
                          str::human_bytes(max, b2, sizeof(b2)), pct);
            logging::file("%s", line);
            if (g_full_console || bytes > 0)
                logging::console("%s", line);
        }
    }

    void print_usage_chart(const map_data &map, bool full_console, int alloc_block_pages)
    {
        g_total = 0;
        g_full_console = full_console;

        logging::file("\nbsp usage (full chart):\n");
        if (full_console)
            logging::console("\nbsp usage:\n");
        else
            logging::console("\nbsp usage (rows over 15%% full; the full chart is in the logfile):\n");

        long long world_faces = map.models.empty() ? 0 : map.models[0].numfaces;
        long long world_leaves = map.models.empty() ? 0 : map.models[0].visleafs;

        array_row("models", map.models.size(), limits::max_map_models, sizeof(dmodel_t));
        array_row("planes", map.planes.size(), limits::max_map_planes, sizeof(dplane_t));
        array_row("vertexes", map.vertexes.size(), limits::max_map_verts, sizeof(dvertex_t));
        array_row("nodes", map.nodes.size(), limits::max_map_nodes, sizeof(dnode_t));
        array_row("texinfos", map.texinfo.size(), limits::max_map_texinfo, sizeof(texinfo_t));
        array_row("faces", map.faces.size(), limits::max_map_faces, sizeof(dface_t));
        array_row("* worldfaces", world_faces, limits::max_map_worldfaces, 0);
        array_row("clipnodes", map.clipnodes.size(), limits::max_map_clipnodes, sizeof(dclipnode_t));
        array_row("leaves", map.leafs.size(), limits::max_map_leafs, sizeof(dleaf_t));
        array_row("* worldleaves", world_leaves, limits::max_map_leafs_engine, 0);
        array_row("marksurfaces", map.marksurfaces.size(), limits::max_map_marksurfaces,
                  sizeof(unsigned short));
        array_row("surfedges", map.surfedges.size(), limits::max_map_surfedges, sizeof(int));
        array_row("edges", map.edges.size(), limits::max_map_edges, sizeof(dedge_t));

        blob_row("texdata", map.textures.size(), limits::max_map_miptex);
        blob_row("lightdata", map.lighting.size(), limits::max_map_lightdata);
        blob_row("visdata", map.visibility.size(), limits::max_map_visibility);
        blob_row("entdata", map.entities.size(), limits::max_map_entstring);
        if (alloc_block_pages >= 0)
            array_row("* lightmap atlas", alloc_block_pages, limits::max_alloc_block_pages, 0);

        char b1[32];
        logging::info("  %-16s %s (total)\n", "bsp data", str::human_bytes(g_total, b1, sizeof(b1)));
    }
}
