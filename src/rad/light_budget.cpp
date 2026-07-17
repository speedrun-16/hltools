#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

#include "../common/binary.h"
#include "../common/error.h"
#include "../common/limits.h"
#include "../common/log.h"
#include "../common/string_util.h"
#include "format/bsp/face_extents.h"
#include "internal.h"

// the engine packs every lightmapped face into a fixed set of 64 128x128
// atlas pages ("allocblock") overflowing it aborts map load with
// "allocblock: full", so rad counts the pages up front and fails fast with a
// per texture breakdown before the expensive lighting passes pure
// diagnostics: it never touches lightdata

namespace rad
{
    namespace
    {
        struct lightmapblock
        {
            lightmapblock *next;
            bool used;
            int allocated[limits::block_width];
        };

        void do_alloc_block(lightmapblock *blocks, int w, int h)
        {
            lightmapblock *block;
            // code from quake
            int i, j;
            int best, best2;
            int x = 0, y;
            (void)y;
            if (w < 1 || h < 1)
            {
                err::fatal("do_alloc_block: internal error.");
            }
            for (block = blocks; block; block = block->next)
            {
                best = limits::block_height;
                for (i = 0; i < limits::block_width - w; i++)
                {
                    best2 = 0;
                    for (j = 0; j < w; j++)
                    {
                        if (block->allocated[i + j] >= best)
                            break;
                        if (block->allocated[i + j] > best2)
                            best2 = block->allocated[i + j];
                    }
                    if (j == w)
                    {
                        x = i;
                        best = best2;
                    }
                }
                if (best + h <= limits::block_height)
                {
                    block->used = true;
                    for (i = 0; i < w; i++)
                    {
                        block->allocated[x + i] = best + h;
                    }
                    return;
                }
                if (!block->next)
                {
                    // need to allocate a new block
                    if (!block->used)
                    {
                        logging::warn("CountBlocks: invalid extents %dx%d", w, h);
                        return;
                    }
                    block->next = (lightmapblock *)malloc(sizeof(lightmapblock));
                    err::require(block->next != nullptr, "do_alloc_block: out of memory");
                    memset(block->next, 0, sizeof(lightmapblock));
                }
            }
        }

        // per texture lightmap footprint for the breakdown report
        struct texlmusage
        {
            std::string name;
            int faces;
            long long luxels;
        };

        const char *texture_name(const format::map_data &map, int texinfo)
        {
            if (texinfo == -1)
                return "";

            const format::texinfo_t *info = &map.texinfo[(size_t)texinfo];
            if (info->miptex < 0)
                return "";
            binary::reader textures(map.textures);
            std::int32_t ofs;
            if (!textures.i32_at(sizeof(int) + (size_t)info->miptex * sizeof(int), ofs)
                || ofs < 0 || (size_t)ofs + sizeof(format::miptex_t) > map.textures.size())
                return "";
            const format::miptex_t *mt = (const format::miptex_t *)&map.textures[(size_t)ofs];
            return mt->name;
        }

        int count_blocks(const format::map_data &map,
                         std::vector<texlmusage> *breakdown = nullptr,
                         long long *total_luxels = nullptr)
        {
            std::unordered_map<std::string, size_t> texindex; // texname to slot in breakdown
            lightmapblock *blocks;
            blocks = (lightmapblock *)malloc(sizeof(lightmapblock));
            err::require(blocks != nullptr, "count_blocks: out of memory");
            memset(blocks, 0, sizeof(lightmapblock));
            int k;
            for (k = 0; k < (int)map.faces.size(); k++)
            {
                const format::dface_t *f = &map.faces[(size_t)k];
                const char *texname = texture_name(map, format::parse_texinfo_for_face(map, f));
                if (!strncmp(texname, "sky", 3) // sky: no lightmap allocation
                    || !strncmp(texname, "!", 1) || str::istarts_with(texname, "water") || str::istarts_with(texname, "laser") // water: no lightmap allocation
                    || (map.texinfo[(size_t)format::parse_texinfo_for_face(map, f)].flags & tex_special)) // aaatrigger
                {
                    continue;
                }
                int extents[2];
                vec3v point;
                {
                    int bmins[2];
                    int bmaxs[2];
                    int i;
                    format::get_face_extents(map, k, bmins, bmaxs);
                    for (i = 0; i < 2; i++)
                    {
                        extents[i] = (bmaxs[i] - bmins[i]) * texture_step;
                    }

                    math::clear(point);
                    if (f->numedges > 0)
                    {
                        int e = map.surfedges[(size_t)f->firstedge];
                        const format::dvertex_t *v = &map.vertexes[map.edges[(size_t)abs(e)].v[e >= 0 ? 0 : 1]];
                        point = vec3v{v->point[0], v->point[1], v->point[2]};
                    }
                }
                int maxextent = 512 > limits::max_surface_extent * texture_step ? 512 : limits::max_surface_extent * texture_step;
                if (extents[0] < 0 || extents[1] < 0 || extents[0] > maxextent || extents[1] > maxextent)
                {
                    logging::warn("Bad surface extents %d/%d at position (%.0f,%.0f,%.0f)",
                                  extents[0], extents[1], point[0], point[1], point[2]);
                    continue;
                }
                int w = (extents[0] / texture_step) + 1;
                int h = (extents[1] / texture_step) + 1;
                do_alloc_block(blocks, w, h);
                if (breakdown)
                {
                    long long luxels = (long long)w * h;
                    auto it = texindex.find(texname);
                    size_t slot;
                    if (it == texindex.end())
                    {
                        slot = breakdown->size();
                        texindex[texname] = slot;
                        texlmusage entry;
                        entry.name = texname;
                        entry.faces = 0;
                        entry.luxels = 0;
                        breakdown->push_back(entry);
                    }
                    else
                    {
                        slot = it->second;
                    }
                    (*breakdown)[slot].faces++;
                    (*breakdown)[slot].luxels += luxels;
                    if (total_luxels)
                    {
                        *total_luxels += luxels;
                    }
                }
            }
            int numblocks = 0;
            lightmapblock *b;
            for (b = blocks; b; b = blocks)
            {
                if (b->used)
                {
                    numblocks++;
                }
                blocks = b->next;
                free(b);
            }
            return numblocks;
        }

        void print_alloc_block_report(int numblocks, int maxblocks, std::vector<texlmusage> &breakdown, long long total_luxels)
        {
            const int warn_blocks = (maxblocks * 95) / 100; // start reporting past 95% of the budget
            if (numblocks <= warn_blocks)
            {
                return;
            }
            bool overflow = numblocks > maxblocks;
            const long long budget_luxels = (long long)maxblocks * limits::block_width * limits::block_height;
            double usage_pct = maxblocks ? numblocks * 100.0 / maxblocks : 0.0;

            // lead with the verdict, then the breakdown explaining which textures caused it
            if (overflow)
            {
                logging::console("\n!!! ERROR: LIGHTMAP ATLAS OVERFLOW - map exceeds the engine's %d page limit\n", maxblocks);
                logging::console("    usage    %d / %d pages (%.0f%%)\n", numblocks, maxblocks, usage_pct);
                logging::console("    cause    too many lightmapped luxels; the engine aborts with \"AllocBlock: full\"\n");
                logging::console("    action   raise the texture scale on the biggest consumers below, or make them smaller / VL\n");
                logging::file("\nLightmap atlas overflow (AllocBlock): %d of %d pages used. The map will not load until this is under the limit.\n", numblocks, maxblocks);
                logging::set_error();
            }
            else
            {
                logging::console("\nWarning: lightmap atlas %.0f%% full (%d / %d pages) - approaching the engine limit\n", usage_pct, numblocks, maxblocks);
                logging::add_warning_summary("lightmap atlas approaching the 64 page limit (see the breakdown below)");
            }

            // biggest luxel footprint first: that is what a mapper needs to shrink
            std::sort(breakdown.begin(), breakdown.end(),
                      [](const texlmusage &a, const texlmusage &b) { return a.luxels > b.luxels; });

            const size_t show = 12;
            logging::console("\n    Lightmap atlas budget by texture (top consumers):\n");
            logging::console("      %-20s %9s %12s  %8s\n", "texture", "faces", "luxels", "% budget");
            logging::console("      --------------------------------------------------------------\n");
            int total_faces = 0;
            size_t i;
            for (i = 0; i < breakdown.size(); i++)
            {
                total_faces += breakdown[i].faces;
            }
            for (i = 0; i < breakdown.size() && i < show; i++)
            {
                double pct = budget_luxels ? breakdown[i].luxels * 100.0 / budget_luxels : 0.0;
                logging::console("      %-20s %9d %12lld   %6.1f%%\n", breakdown[i].name.c_str(),
                                 breakdown[i].faces, breakdown[i].luxels, pct);
            }
            if (breakdown.size() > show)
            {
                long long rest_luxels = 0;
                int rest_faces = 0;
                for (i = show; i < breakdown.size(); i++)
                {
                    rest_luxels += breakdown[i].luxels;
                    rest_faces += breakdown[i].faces;
                }
                double pct = budget_luxels ? rest_luxels * 100.0 / budget_luxels : 0.0;
                char label[32];
                snprintf(label, sizeof(label), "(%d others)", (int)(breakdown.size() - show));
                logging::console("      %-20s %9d %12lld   %6.1f%%\n", label, rest_faces, rest_luxels, pct);
            }
            logging::console("      --------------------------------------------------------------\n");
            {
                double pct = budget_luxels ? total_luxels * 100.0 / budget_luxels : 0.0;
                logging::console("      %-20s %9d %12lld   %6.1f%%\n", "total", total_faces, total_luxels, pct);
            }
        }
    }

    int count_alloc_blocks(const format::map_data &map)
    {
        return count_blocks(map);
    }

    // early budget check, run before the expensive lighting passes so an
    // unloadable map fails fast returns true on overflow, in which case the
    // caller should skip lighting (the compile error flag has been set)
    bool check_alloc_block_budget(rad_state &state, int *alloc_block_pages)
    {
        std::vector<texlmusage> breakdown;
        long long luxels = 0;
        int numblocks = count_blocks(*state.map, &breakdown, &luxels);
        if (alloc_block_pages)
        {
            *alloc_block_pages = numblocks;
        }
        if (numblocks <= 0)
        {
            return false;
        }
        print_alloc_block_report(numblocks, limits::max_alloc_block_pages, breakdown, luxels);
        return numblocks > limits::max_alloc_block_pages;
    }
}
