#include "file.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include "common/filesystem.h"
#include "common/log.h"
#include "data.h"

namespace format
{
    namespace
    {
        struct bspx_header
        {
            char id[4];
            std::uint32_t num_lumps;
        };

        struct bspx_lump_header
        {
            char name[24];
            std::uint32_t file_offset;
            std::uint32_t file_length;
        };

        struct embed_locator
        {
            std::uint16_t version;
            std::uint16_t header_size;
            std::uint32_t flags;
            std::uint64_t zip_offset;
        };

        static_assert(sizeof(bspx_header) == 8, "BSPX header layout");
        static_assert(sizeof(bspx_lump_header) == 32, "BSPX lump layout");
        static_assert(sizeof(embed_locator) == 16, "embed locator layout");

        size_t aligned4(size_t value)
        {
            return (value + 3) & ~(size_t)3;
        }

        bool valid_range(size_t offset, size_t length, size_t size)
        {
            return offset <= size && length <= size - offset;
        }

        size_t vanilla_end(const dheader_t &header)
        {
            size_t end = sizeof(dheader_t);
            for (int i = 0; i < header_lumps; i++)
            {
                const lump_t &lump = header.lumps[i];
                if (lump.fileofs < 0 || lump.filelen < 0)
                    return 0;
                size_t offset = (size_t)lump.fileofs;
                size_t length = (size_t)lump.filelen;
                if (offset > std::numeric_limits<size_t>::max() - length)
                    return 0;
                end = std::max(end, offset + length);
            }
            return aligned4(end);
        }

        const bspx_lump *find_bspx(const std::vector<bspx_lump> &lumps,
                                   const char *name)
        {
            for (const bspx_lump &lump : lumps)
                if (lump.name == name)
                    return &lump;
            return nullptr;
        }

        bool read_bspx(const std::vector<byte> &file, const dheader_t &header,
                       map_data &out)
        {
            const size_t offset = vanilla_end(header);
            if (offset == 0 || offset == file.size())
                return true;
            if (!valid_range(offset, sizeof(bspx_header), file.size()))
                return true; // ordinary trailing bytes, not BSPX

            bspx_header xheader;
            std::memcpy(&xheader, file.data() + offset, sizeof(xheader));
            if (std::memcmp(xheader.id, "BSPX", 4) != 0)
                return true;

            const size_t directory_offset = offset + sizeof(xheader);
            if (xheader.num_lumps > (file.size() - directory_offset)
                    / sizeof(bspx_lump_header))
            {
                logging::warn("bsp has an invalid BSPX directory");
                return false;
            }

            out.bspx.clear();
            size_t bspx_end = directory_offset
                + (size_t)xheader.num_lumps * sizeof(bspx_lump_header);
            for (std::uint32_t i = 0; i < xheader.num_lumps; i++)
            {
                bspx_lump_header xentry;
                std::memcpy(&xentry,
                            file.data() + directory_offset
                                + (size_t)i * sizeof(xentry),
                            sizeof(xentry));
                if (!valid_range(xentry.file_offset, xentry.file_length, file.size()))
                {
                    logging::warn("bsp has an invalid BSPX lump at index %u", i);
                    return false;
                }
                bspx_end = std::max(
                    bspx_end,
                    (size_t)xentry.file_offset + (size_t)xentry.file_length);

                size_t name_length = 0;
                while (name_length < sizeof(xentry.name)
                       && xentry.name[name_length] != '\0')
                    name_length++;
                bspx_lump lump;
                lump.name.assign(xentry.name, name_length);
                lump.data.assign(file.begin() + xentry.file_offset,
                                 file.begin() + xentry.file_offset
                                     + xentry.file_length);
                out.bspx.push_back(std::move(lump));
            }

            const bspx_lump *locator =
                find_bspx(out.bspx, bsp_file::embed_locator_lump);
            if (!locator)
                return true;
            if (locator->data.size() < sizeof(embed_locator))
            {
                logging::warn("bsp has a truncated %s lump",
                              bsp_file::embed_locator_lump);
                return false;
            }

            embed_locator value;
            std::memcpy(&value, locator->data.data(), sizeof(value));
            if (value.version != 1 || value.header_size < sizeof(embed_locator)
                || value.header_size > locator->data.size()
                || value.flags != 0 || value.zip_offset < bspx_end
                || value.zip_offset > file.size())
            {
                logging::warn("bsp has an unsupported %s lump",
                              bsp_file::embed_locator_lump);
                return false;
            }
            out.embedded_zip.assign(file.begin() + (size_t)value.zip_offset,
                                    file.end());
            return true;
        }

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

        void append_bspx(std::vector<byte> &out, const map_data &data)
        {
            std::vector<bspx_lump> lumps = data.bspx;
            lumps.erase(std::remove_if(lumps.begin(), lumps.end(),
                         [](const bspx_lump &lump)
                         {
                             return lump.name == bsp_file::embed_locator_lump;
                         }),
                        lumps.end());
            if (!data.embedded_zip.empty())
            {
                bspx_lump locator;
                locator.name = bsp_file::embed_locator_lump;
                locator.data.resize(sizeof(embed_locator));
                lumps.push_back(std::move(locator));
            }
            if (lumps.empty())
                return;

            while ((out.size() & 3) != 0)
                out.push_back(0);

            bspx_header header{{'B', 'S', 'P', 'X'}, (std::uint32_t)lumps.size()};
            out.insert(out.end(), (const byte *)&header,
                       (const byte *)&header + sizeof(header));
            const size_t directory_offset = out.size();
            out.resize(out.size() + lumps.size() * sizeof(bspx_lump_header), 0);

            std::vector<bspx_lump_header> directory(lumps.size());
            size_t locator_index = lumps.size();
            for (size_t i = 0; i < lumps.size(); i++)
            {
                while ((out.size() & 3) != 0)
                    out.push_back(0);
                bspx_lump_header &entry = directory[i];
                std::memset(&entry, 0, sizeof(entry));
                std::memcpy(entry.name, lumps[i].name.data(),
                            std::min(lumps[i].name.size(), sizeof(entry.name)));
                entry.file_offset = (std::uint32_t)out.size();
                entry.file_length = (std::uint32_t)lumps[i].data.size();
                if (lumps[i].name == bsp_file::embed_locator_lump)
                    locator_index = i;
                out.insert(out.end(), lumps[i].data.begin(), lumps[i].data.end());
            }

            while ((out.size() & 3) != 0)
                out.push_back(0);
            if (!data.embedded_zip.empty())
            {
                embed_locator locator{1, (std::uint16_t)sizeof(embed_locator),
                                      0, (std::uint64_t)out.size()};
                const bspx_lump_header &entry = directory[locator_index];
                std::memcpy(out.data() + entry.file_offset, &locator, sizeof(locator));
                out.insert(out.end(), data.embedded_zip.begin(),
                           data.embedded_zip.end());
            }
            std::memcpy(out.data() + directory_offset, directory.data(),
                        directory.size() * sizeof(directory[0]));
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

        for (int i = 0; i < header_lumps; i++)
        {
            const lump_t &lump = header.lumps[i];
            if (lump.fileofs < 0 || lump.filelen < 0
                || !valid_range((size_t)lump.fileofs, (size_t)lump.filelen,
                                file.size()))
            {
                logging::warn("bsp '%s' has an invalid lump at index %d",
                              path.c_str(), i);
                return false;
            }
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
        out.bspx.clear();
        out.embedded_zip.clear();
        return read_bspx(file, header, out);
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
        append_bspx(out, data);
        return fs::write_all(path, out.data(), out.size());
    }
}
