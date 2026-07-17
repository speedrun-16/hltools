#include "file.h"

#include <cstring>
#include <vector>

#include "common/filesystem.h"
#include "common/log.h"
#include "data.h"

namespace format
{
    namespace
    {
        // copy a fixed record lump into a typed vector
        template <typename T>
        void read_lump(const std::vector<byte> &file, const dheader_t &h, int id, std::vector<T> &out)
        {
            const lump_t &l = h.lumps[id];
            size_t count = (size_t)l.filelen / sizeof(T);
            out.resize(count);
            if (count > 0)
                std::memcpy(out.data(), file.data() + l.fileofs, count * sizeof(T));
        }

        void read_blob(const std::vector<byte> &file, const dheader_t &h, int id, std::vector<byte> &out)
        {
            const lump_t &l = h.lumps[id];
            out.assign(file.begin() + l.fileofs, file.begin() + l.fileofs + l.filelen);
        }

        // append a lump: record its offset and length, then write the bytes
        // padded up to the next 4 byte boundary with zeros, exactly as the
        // reference writer did
        void add_bytes(std::vector<byte> &out, dheader_t &h, int id, const void *ptr, size_t len)
        {
            h.lumps[id].fileofs = (int)out.size();
            h.lumps[id].filelen = (int)len;
            const byte *p = (const byte *)ptr;
            out.insert(out.end(), p, p + len);
            while ((out.size() & 3) != 0)
                out.push_back(0);
        }

        template <typename T>
        void add_lump(std::vector<byte> &out, dheader_t &h, int id, const std::vector<T> &v)
        {
            add_bytes(out, h, id, v.data(), v.size() * sizeof(T));
        }
    }

    bool bsp_file::load(const std::string &path, map_data &out)
    {
        std::vector<byte> file;
        if (!fs::read_all(path, file))
        {
            logging::warn("could not read bsp '%s'", path.c_str());
            return false;
        }
        if (file.size() < sizeof(dheader_t))
        {
            logging::warn("bsp '%s' is too small to be valid", path.c_str());
            return false;
        }

        dheader_t header;
        std::memcpy(&header, file.data(), sizeof(header));
        if (header.version != bsp_version)
        {
            logging::warn("bsp '%s' has version %d, expected %d", path.c_str(),
                          header.version, bsp_version);
            return false;
        }

        read_lump(file, header, lump_models, out.models);
        read_lump(file, header, lump_planes, out.planes);
        read_lump(file, header, lump_vertexes, out.vertexes);
        read_lump(file, header, lump_nodes, out.nodes);
        read_lump(file, header, lump_texinfo, out.texinfo);
        read_lump(file, header, lump_faces, out.faces);
        read_lump(file, header, lump_clipnodes, out.clipnodes);
        read_lump(file, header, lump_leafs, out.leafs);
        read_lump(file, header, lump_marksurfaces, out.marksurfaces);
        read_lump(file, header, lump_edges, out.edges);
        read_lump(file, header, lump_surfedges, out.surfedges);

        read_blob(file, header, lump_lighting, out.lighting);
        read_blob(file, header, lump_visibility, out.visibility);
        read_blob(file, header, lump_textures, out.textures);

        const lump_t &ent = header.lumps[lump_entities];
        out.entities.assign((const char *)file.data() + ent.fileofs, (size_t)ent.filelen);
        return true;
    }

    bool bsp_file::write(const std::string &path, const map_data &data)
    {
        dheader_t header{};
        header.version = bsp_version;

        std::vector<byte> out;
        out.resize(sizeof(dheader_t)); // header placeholder, overwritten at the end

        // the exact order the reference writer used; the physical layout depends
        // on it, so it must not change
        add_lump(out, header, lump_planes, data.planes);
        add_lump(out, header, lump_leafs, data.leafs);
        add_lump(out, header, lump_vertexes, data.vertexes);
        add_lump(out, header, lump_nodes, data.nodes);
        add_lump(out, header, lump_texinfo, data.texinfo);
        add_lump(out, header, lump_faces, data.faces);
        add_lump(out, header, lump_clipnodes, data.clipnodes);
        add_lump(out, header, lump_marksurfaces, data.marksurfaces);
        add_lump(out, header, lump_surfedges, data.surfedges);
        add_lump(out, header, lump_edges, data.edges);
        add_lump(out, header, lump_models, data.models);

        add_bytes(out, header, lump_lighting, data.lighting.data(), data.lighting.size());
        add_bytes(out, header, lump_visibility, data.visibility.data(), data.visibility.size());
        add_bytes(out, header, lump_entities, data.entities.data(), data.entities.size());
        add_bytes(out, header, lump_textures, data.textures.data(), data.textures.size());

        std::memcpy(out.data(), &header, sizeof(header));
        return fs::write_all(path, out.data(), out.size());
    }
}
