#pragma once

#include <string>
#include <vector>

// texlight definitions parsed from a lightsrad file each names a texture and
// the light colour its faces emit rad reads one or more of these files plus the
// in map info_texlights entity; later definitions override earlier ones by name

namespace format
{
    struct texlight
    {
        std::string name;
        float value[3]; // r, g, b as fed into the lighting solve
        std::string source; // file the definition came from, for override reports
    };

    // parse a lightsrad file, merging into out (a later definition of the same
    // texture replaces an earlier one) returns the number of definitions read,
    // or -1 if the file could not be opened
    //
    // line format, matching the reference reader:
    //   texname v            grayscale, r = g = b = v
    //   texname r g b        explicit colour
    //   texname r g b scale  colour scaled by scale/255
    // a "//" starts a comment to end of line
    int read_rad_file(const std::string &path, std::vector<texlight> &out);
}
