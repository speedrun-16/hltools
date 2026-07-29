#pragma once

#include <string>
#include <vector>

#include "format/bmp/image.h"

// the engine neutral studio model the converters talk through. a source model
// (mdl + vvd + vtx) is read into this, and the goldsrc v10 writer serializes it
// out again, so neither side needs to know the other's layout.
//
// goldsrc skinning is rigid: one bone per vertex, no weights. source vertices
// carry up to three weighted bones, so the reader keeps the dominant one; for
// the single bone props this porter targets that is lossless.

namespace format
{
    struct studio_vertex
    {
        float position[3] = {};
        float normal[3] = {};
        float u = 0, v = 0; // normalized, origin top left as source stores it
        int bone = 0;
    };

    struct studio_mesh
    {
        int material = 0;         // index into studio_model::materials
        std::vector<int> indices; // triangle list into studio_model::vertices
    };

    struct studio_bone
    {
        std::string name;
        int parent = -1;
        float position[3] = {};
        float rotation[3] = {}; // euler radians, as both formats store it
    };

    // one bone's placement in one frame, in the same terms as the bind pose
    struct studio_bone_key
    {
        float position[3] = {};
        float rotation[3] = {}; // euler radians
    };

    struct studio_sequence
    {
        std::string name;
        float fps = 30;
        bool loop = false;
        // frames[frame][bone]; every frame carries a key for every bone, so the
        // writer never has to reason about which channels were animated
        std::vector<std::vector<studio_bone_key>> frames;
    };

    // goldsrc studio texture render flags (mstudiotexture_t::flags)
    constexpr int studio_nf_flatshade = 0x0001;
    constexpr int studio_nf_chrome = 0x0002;
    constexpr int studio_nf_fullbright = 0x0004;
    constexpr int studio_nf_nomips = 0x0008;
    constexpr int studio_nf_alpha = 0x0010;
    constexpr int studio_nf_additive = 0x0020;
    // the palette's last entry is punched out, the model equivalent of a '{'
    // masked world texture
    constexpr int studio_nf_masked = 0x0040;

    // a goldsrc model carries its skins inside the file, palettized. the source
    // reader leaves this empty: resolving material names to images is the
    // caller's job, since only it knows where the vmt/vtf live.
    struct studio_texture
    {
        std::string name; // as referenced by the model, e.g. "bark.bmp"
        int flags = 0;
        indexed_image image;
    };

    struct studio_model
    {
        std::string name;
        std::vector<studio_bone> bones;
        std::vector<studio_sequence> sequences;
        std::vector<studio_texture> textures;
        std::vector<studio_vertex> vertices;
        std::vector<studio_mesh> meshes;
        // material names as the model references them, plus the search
        // directories the source model recorded; resolving them to real images
        // is the caller's job
        std::vector<std::string> materials;
        std::vector<std::string> material_dirs;

        float bbmin[3] = {};
        float bbmax[3] = {};

        std::size_t triangle_count() const
        {
            std::size_t total = 0;
            for (const studio_mesh &mesh : meshes)
                total += mesh.indices.size() / 3;
            return total;
        }
    };
}
