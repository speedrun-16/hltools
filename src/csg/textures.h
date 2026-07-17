#pragma once

#include <string>
#include <vector>

#include "format/bsp/types.h"
#include "brush.h"

namespace csg
{
    constexpr int tex_special = 1;

    struct texinfo_entry
    {
        format::texinfo_t info = {};
        std::string texture_name;
    };

    class texinfo_store
    {
    public:
        int texinfo_for_brush_texture(const brush_plane &plane,
                                      brush_texture &texture,
                                      const math::vec3v &origin,
                                      int map_file_version);

        const std::vector<texinfo_entry> &entries() const {
            return entries_;
        }

        void set_miptex_index(int texinfo, int miptex);

    private:
        std::vector<texinfo_entry> entries_;
    };
}
