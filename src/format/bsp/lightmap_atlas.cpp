#include "lightmap_atlas.h"

#include <algorithm>
#include <cstdint>
#include <cstring>

#include "types.h"
#include "face_extents.h"
#include "data.h"

namespace format
{
    namespace
    {
        constexpr unsigned preferred_width = 1024;
        constexpr unsigned padding = 1;

        struct tile
        {
            unsigned x;
            unsigned y;
            unsigned width;
            unsigned height;
            size_t light_offset;
        };

        void set_error(std::string *error, const std::string &message)
        {
            if (error)
                *error = message;
        }
    }

    bool build_lightmap_atlas(const map_data &map, lightmap_atlas &atlas,
                              std::string *error)
    {
        atlas = lightmap_atlas{};
        if (map.lighting.empty())
        {
            set_error(error, "BSP contains no lighting data");
            return false;
        }

        struct pending_tile
        {
            unsigned width;
            unsigned height;
            size_t light_offset;
        };
        std::vector<pending_tile> pending;
        unsigned widest = 0;
        for (size_t facenum = 0; facenum < map.faces.size(); facenum++)
        {
            const dface_t &face = map.faces[facenum];
            if (face.lightofs < 0)
                continue;
            int styles = 0;
            while (styles < max_lightmaps && face.styles[styles] != 255)
                styles++;
            if (!styles)
                continue;

            int mins[2], maxs[2];
            get_face_extents(map, (int)facenum, mins, maxs);
            int signed_width = maxs[0] - mins[0] + 1;
            int signed_height = maxs[1] - mins[1] + 1;
            if (signed_width <= 0 || signed_height <= 0)
            {
                set_error(error, "face " + std::to_string(facenum)
                                  + " has invalid lightmap extents");
                return false;
            }
            unsigned width = (unsigned)signed_width;
            unsigned height = (unsigned)signed_height;
            if ((size_t)width > SIZE_MAX / height
                || (size_t)width * height > SIZE_MAX / 3)
            {
                set_error(error, "face " + std::to_string(facenum)
                                  + " lightmap dimensions overflow");
                return false;
            }
            size_t style_bytes = (size_t)width * height * 3;
            for (int slot = 0; slot < styles; slot++)
            {
                size_t offset = (size_t)face.lightofs + (size_t)slot * style_bytes;
                if (offset > map.lighting.size()
                    || style_bytes > map.lighting.size() - offset)
                {
                    set_error(error, "face " + std::to_string(facenum)
                                      + " lightmap is outside the lighting lump");
                    return false;
                }
                pending.push_back({width, height, offset});
                widest = std::max(widest, width);
            }
            atlas.faces++;
        }
        if (pending.empty())
        {
            set_error(error, "BSP contains no lit faces");
            return false;
        }

        atlas.width = std::max(preferred_width, widest + padding * 2);
        unsigned x = padding;
        unsigned y = padding;
        unsigned row_height = 0;
        std::vector<tile> tiles;
        tiles.reserve(pending.size());
        for (const pending_tile &item : pending)
        {
            if (x + item.width + padding > atlas.width)
            {
                x = padding;
                y += row_height + padding;
                row_height = 0;
            }
            tiles.push_back({x, y, item.width, item.height, item.light_offset});
            x += item.width + padding;
            row_height = std::max(row_height, item.height);
        }
        if (y > UINT32_MAX - row_height - padding)
        {
            set_error(error, "lightmap atlas is too tall");
            return false;
        }
        atlas.height = y + row_height + padding;
        if ((size_t)atlas.width > SIZE_MAX / atlas.height
            || (size_t)atlas.width * atlas.height > SIZE_MAX / 3)
        {
            set_error(error, "lightmap atlas is too large");
            return false;
        }
        atlas.pixels.assign((size_t)atlas.width * atlas.height * 3, 0);
        for (const tile &item : tiles)
        {
            for (unsigned row = 0; row < item.height; row++)
            {
                size_t source = item.light_offset + (size_t)row * item.width * 3;
                size_t destination = ((size_t)(item.y + row) * atlas.width + item.x) * 3;
                std::memcpy(atlas.pixels.data() + destination,
                            map.lighting.data() + source, (size_t)item.width * 3);
            }
        }
        atlas.tiles = (int)tiles.size();
        return true;
    }
}
