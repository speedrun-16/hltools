#pragma once

#include "data.h"

// the engine exact lightmap extent math shared by hlbsp (ext file) and hlrad
// (lightmap sizing) the product is sequenced one operation at a time through
// volatile doubles because the goldsrc engine computes it exactly that way, and
// the tools must agree with the engine about every face's lightmap size

namespace format
{
    // resolves a face's original texinfo through an embedded lightmap texture
    // name ("?_radnnn"), identity on fresh compiler output
    int parse_texinfo_for_face(const map_data &map, const dface_t *f);

    // the reference's carefully sequenced float product, matching the engine's
    // calcfaceextents arithmetic exactly
    float calculate_point_vecs_product(const volatile float *point, const volatile float *vecs);

    void get_face_extents(const map_data &map, int facenum, int mins_out[2], int maxs_out[2]);
}
