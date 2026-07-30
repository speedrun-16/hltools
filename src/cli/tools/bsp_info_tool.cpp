#include "bsp_info_tool.h"

#include <cstring>
#include <string>
#include <vector>

#include "../../common/binary.h"
#include "../../common/cmdline.h"
#include "../../common/filesystem.h"
#include "../../common/log.h"
#include "../../common/string_util.h"
#include "format/bsp/data.h"
#include "format/bsp/entity_lump.h"
#include "format/bsp/file.h"
#include "format/bsp/usage_chart.h"
#include "../../rad/rad.h"

namespace tools
{
    namespace
    {
        void print_bsp_info_help()
        {
            logging::console(
                "usage\n"
                "  hltools bsp info <map[.bsp]>\n"
                "\n"
                "  reports a compiled bsp's contents: compiled stages, entity and\n"
                "  texture counts, the wads it references, and how full every lump\n"
                "  is against its engine limit. read-only.\n");
        }

        // the miptex lump: count, and how many entries carry their pixel data
        // in the bsp versus load from a wad at runtime (zeroed mip offsets)
        void miptex_summary(const std::vector<byte> &lump, int &total, int &embedded)
        {
            total = 0;
            embedded = 0;
            if (lump.size() < sizeof(int))
                return;
            binary::reader input(lump);
            std::int32_t count = 0;
            if (!input.i32(count))
                return;
            if (count <= 0 || lump.size() < sizeof(int) * (size_t)(count + 1))
                return;
            total = count;
            for (int i = 0; i < count; i++)
            {
                std::int32_t ofs = 0;
                if (!input.i32(ofs))
                    return;
                if (ofs < 0 || (size_t)ofs + sizeof(format::miptex_t) > lump.size())
                    continue;
                format::miptex_t header;
                std::memcpy(&header, lump.data() + (size_t)ofs, sizeof(header));
                if (header.offsets[0] != 0)
                    embedded++;
            }
        }
    }

    int run_bsp_info_tool(int argc, char **argv)
    {
        cli::args args(argc, argv);
        if (args.empty() || args.map_name().empty() || args.has("-h") || args.has("-help")
            || args.has("--help"))
        {
            print_bsp_info_help();
            return (args.has("-h") || args.has("-help") || args.has("--help")) ? 0 : 1;
        }

        std::string bsp_path = fs::with_extension(args.map_name(), ".bsp");
        format::map_data map;
        if (!format::bsp_file::load(bsp_path, map))
        {
            logging::console("bsp info: could not load '%s'\n", bsp_path.c_str());
            return 1;
        }

        std::vector<format::entity> entities = format::parse_entities(map.entities);

        int brush_models = 0;
        std::string wad_value;
        for (const format::entity &ent : entities)
        {
            const char *model = ent.value("model");
            if (model[0] == '*')
                brush_models++;
                    if (wad_value.empty() && str::iequals(ent.value("classname"), "worldspawn"))
                wad_value = ent.value("wad");
        }

        int miptex_total = 0, miptex_embedded = 0;
        miptex_summary(map.textures, miptex_total, miptex_embedded);

        const bool has_vis = !map.visibility.empty();
        const bool has_light = !map.lighting.empty();

        char b1[32];
        logging::console("\nbsp info: %s\n\n", bsp_path.c_str());
        logging::console("  %-14s 30 (goldsrc)\n", "version");
        logging::console("  %-14s %s\n", "file size",
                         str::human_bytes(fs::size(bsp_path), b1, sizeof(b1)));
        logging::console("  %-14s csg+bsp%s%s%s\n", "stages",
                         has_vis ? ", vis" : "", has_light ? ", rad" : "",
                         (has_vis && has_light) ? "" : "  (not fully compiled)");
        logging::console("  %-14s %zu (%d brush model%s)\n", "entities", entities.size(),
                         brush_models, brush_models == 1 ? "" : "s");
        if (miptex_total > 0)
            logging::console("  %-14s %d (%d embedded, %d from wads)\n", "textures",
                             miptex_total, miptex_embedded, miptex_total - miptex_embedded);

        // the worldspawn wad list: full editor paths joined with ';' - show
        // just the file names, which is what the engine resolves anyway
        if (!wad_value.empty())
        {
            std::string names;
            size_t start = 0;
            while (start < wad_value.size())
            {
                size_t end = wad_value.find(';', start);
                if (end == std::string::npos)
                    end = wad_value.size();
                if (end > start)
                {
                    std::string name = fs::filename(wad_value.substr(start, end - start));
                    if (!name.empty())
                    {
                        if (!names.empty())
                            names += ", ";
                        names += name;
                    }
                }
                start = end + 1;
            }
            if (!names.empty())
                logging::console("  %-14s %s\n", "wads", names.c_str());
        }

        format::print_usage_chart(map, true, rad::count_alloc_blocks(map));
        return 0;
    }
}
