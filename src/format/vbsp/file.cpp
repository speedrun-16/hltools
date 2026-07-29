#include "file.h"

#include <cstring>
#include <string>

#include "common/binary.h"
#include "common/filesystem.h"

namespace format
{
    namespace
    {
        const std::string empty_material;

        void set_error(std::string *error, const std::string &message)
        {
            if (error)
                *error = message;
        }

        // validates a lump's directory entry against the file image and rejects
        // lzma compressed lumps (payloads that begin with the 'LZMA' ident).
        // returns the lump bounds through ofs/len on success.
        bool locate_lump(const std::vector<byte> &file, const vbsp_header_t &header,
                         int id, const char *name, std::size_t &ofs, std::size_t &len,
                         std::string *error)
        {
            const vbsp_lump_t &lump = header.lumps[id];
            if (lump.filelen < 0 || lump.fileofs < 0)
            {
                set_error(error, std::string("lump ") + name + " has a negative range");
                return false;
            }
            ofs = (std::size_t)lump.fileofs;
            len = (std::size_t)lump.filelen;
            if (len == 0)
                return true; // absent lump, nothing to read
            if (ofs > file.size() || len > file.size() - ofs)
            {
                set_error(error, std::string("lump ") + name + " extends past the file");
                return false;
            }
            if (len >= 4)
            {
                unsigned ident;
                std::memcpy(&ident, file.data() + ofs, 4);
                if (ident == vbsp_lzma_ident)
                {
                    set_error(error, std::string("lump ") + name
                                         + " is lzma compressed (unsupported yet)");
                    return false;
                }
            }
            return true;
        }

        template <typename T>
        bool read_records(const std::vector<byte> &file, const vbsp_header_t &header,
                          int id, const char *name, std::vector<T> &out,
                          std::string *error)
        {
            std::size_t ofs = 0, len = 0;
            if (!locate_lump(file, header, id, name, ofs, len, error))
                return false;
            if (len % sizeof(T) != 0)
            {
                set_error(error, std::string("lump ") + name
                                     + " length is not a whole number of records");
                return false;
            }
            std::size_t count = len / sizeof(T);
            out.resize(count);
            if (count)
                std::memcpy(out.data(), file.data() + ofs, len);
            return true;
        }

        bool read_blob(const std::vector<byte> &file, const vbsp_header_t &header,
                       int id, const char *name, std::vector<byte> &out,
                       std::string *error)
        {
            std::size_t ofs = 0, len = 0;
            if (!locate_lump(file, header, id, name, ofs, len, error))
                return false;
            out.assign(file.begin() + ofs, file.begin() + ofs + len);
            return true;
        }

        bool read_text(const std::vector<byte> &file, const vbsp_header_t &header,
                       int id, const char *name, std::string &out, std::string *error)
        {
            std::size_t ofs = 0, len = 0;
            if (!locate_lump(file, header, id, name, ofs, len, error))
                return false;
            out.assign((const char *)file.data() + ofs, len);
            return true;
        }

        // the game lump is a directory of sub lumps, each pointing at an
        // absolute file offset rather than one relative to the lump. only 'sprp'
        // (the static prop list) matters to the porter; a map without it simply
        // has no static props, which is not an error.
        void read_static_props(const std::vector<byte> &file, const vbsp_header_t &header,
                               source_map_data &out)
        {
            const vbsp_lump_t &lump = header.lumps[vlump_game_lump];
            if (lump.filelen < 4 || lump.fileofs < 0
                || (std::size_t)lump.fileofs + (std::size_t)lump.filelen > file.size())
                return;

            binary::reader reader(file);
            std::int32_t count = 0;
            if (!reader.i32_at((std::size_t)lump.fileofs, count) || count <= 0
                || count > 4096)
                return;

            for (int i = 0; i < count; i++)
            {
                std::size_t at = (std::size_t)lump.fileofs + 4 + (std::size_t)i * 16;
                std::int32_t id = 0, ofs = 0, len = 0;
                std::uint16_t version = 0;
                if (!reader.i32_at(at, id) || !reader.u16_at(at + 6, version)
                    || !reader.i32_at(at + 8, ofs) || !reader.i32_at(at + 12, len))
                    return;
                // the game lump id is a fourcc packed most significant byte
                // first, so 'sprp' is 's'<<24 | 'p'<<16 | 'r'<<8 | 'p'
                if (id != 0x73707270)
                    continue;
                if (ofs < 0 || len < 0
                    || (std::size_t)ofs + (std::size_t)len > file.size())
                    return;
                out.static_props.assign(file.begin() + ofs, file.begin() + ofs + len);
                out.static_prop_version = (int)version;
                return;
            }
        }

        // leaves come in two on disk sizes (56 bytes with a trailing ambient
        // light cube for lump version 0, 32 bytes without for version 1+), but
        // the brush fields sit at the same offset in both. read them by stride.
        bool read_leaves(const std::vector<byte> &file, const vbsp_header_t &header,
                         source_map_data &out, std::string *error)
        {
            std::size_t ofs = 0, len = 0;
            if (!locate_lump(file, header, vlump_leafs, "leafs", ofs, len, error))
                return false;
            std::size_t stride = header.lumps[vlump_leafs].version == 0 ? 56 : 32;
            if (len % stride != 0)
            {
                set_error(error, "leafs length is not a whole number of records");
                return false;
            }
            constexpr std::size_t first_brush_offset = 24; // after contents/area/bbox/faces
            std::size_t count = len / stride;
            out.leaves.resize(count);
            binary::reader input(file.data() + ofs, len);
            for (std::size_t i = 0; i < count; i++)
            {
                std::uint16_t first = 0, num = 0;
                if (!input.seek(i * stride + first_brush_offset)
                    || !input.u16(first) || !input.u16(num))
                {
                    set_error(error, "leafs lump is truncated");
                    return false;
                }
                out.leaves[i].firstleafbrush = first;
                out.leaves[i].numleafbrushes = num;
            }
            return true;
        }

        void build_material_names(source_map_data &out)
        {
            out.material_names.assign(out.texdata.size(), std::string{});
            for (std::size_t i = 0; i < out.texdata.size(); i++)
            {
                int id = out.texdata[i].name_string_table_id;
                if (id < 0 || (std::size_t)id >= out.string_table.size())
                    continue;
                int offset = out.string_table[(std::size_t)id];
                if (offset < 0 || (std::size_t)offset >= out.string_data.size())
                    continue;
                const char *start = out.string_data.c_str() + offset;
                std::size_t max = out.string_data.size() - (std::size_t)offset;
                out.material_names[i].assign(start, ::strnlen(start, max));
            }
        }

        void detect_displacements(source_map_data &out)
        {
            // only faces carry the -1 "no displacement" sentinel; brushside
            // dispinfo uses a different convention (0 for ordinary sides) and is
            // not a reliable signal, so it must not be used here.
            for (const source_dface_t &face : out.faces)
            {
                if (face.dispinfo >= 0)
                {
                    out.has_displacements = true;
                    return;
                }
            }
        }
    }

    const std::string &source_map_data::texinfo_material(int texinfo_index) const
    {
        if (texinfo_index < 0 || (std::size_t)texinfo_index >= texinfo.size())
            return empty_material;
        int texdata_index = texinfo[(std::size_t)texinfo_index].texdata;
        if (texdata_index < 0 || (std::size_t)texdata_index >= material_names.size())
            return empty_material;
        return material_names[(std::size_t)texdata_index];
    }

    bool source_bsp_file::parse(const std::vector<byte> &file, source_map_data &out,
                                std::string *error)
    {
        out = source_map_data{};
        if (file.size() < sizeof(vbsp_header_t))
        {
            set_error(error, "file is too small to be a vbsp map");
            return false;
        }

        vbsp_header_t header;
        std::memcpy(&header, file.data(), sizeof(header));
        if ((unsigned)header.ident != vbsp_ident)
        {
            set_error(error, "not a vbsp map (bad ident)");
            return false;
        }
        if (header.version < vbsp_min_version || header.version > vbsp_max_version)
        {
            set_error(error, "unsupported vbsp version " + std::to_string(header.version));
            return false;
        }
        out.version = header.version;
        out.map_revision = header.map_revision;

        std::vector<byte> string_table_bytes;
        if (!read_records(file, header, vlump_planes, "planes", out.planes, error)
            || !read_records(file, header, vlump_vertexes, "vertexes", out.vertexes, error)
            || !read_records(file, header, vlump_edges, "edges", out.edges, error)
            || !read_records(file, header, vlump_surfedges, "surfedges", out.surfedges, error)
            || !read_records(file, header, vlump_faces, "faces", out.faces, error)
            || !read_records(file, header, vlump_texinfo, "texinfo", out.texinfo, error)
            || !read_records(file, header, vlump_texdata, "texdata", out.texdata, error)
            || !read_records(file, header, vlump_models, "models", out.models, error)
            || !read_records(file, header, vlump_brushes, "brushes", out.brushes, error)
            || !read_records(file, header, vlump_brushsides, "brushsides", out.brushsides, error)
            || !read_records(file, header, vlump_dispinfo, "dispinfo", out.dispinfo, error)
            || !read_records(file, header, vlump_disp_verts, "disp verts",
                             out.disp_verts, error)
            || !read_records(file, header, vlump_nodes, "nodes", out.nodes, error)
            || !read_records(file, header, vlump_leafbrushes, "leafbrushes", out.leafbrushes, error)
            || !read_records(file, header, vlump_texdata_string_table, "texdata string table",
                             out.string_table, error)
            || !read_text(file, header, vlump_texdata_string_data, "texdata string data",
                          out.string_data, error)
            || !read_text(file, header, vlump_entities, "entities", out.entities, error)
            || !read_blob(file, header, vlump_pakfile, "pakfile", out.pakfile, error))
        {
            return false;
        }
        if (!read_leaves(file, header, out, error))
            return false;
        read_static_props(file, header, out);

        build_material_names(out);
        detect_displacements(out);
        return true;
    }

    bool source_bsp_file::load(const std::string &path, source_map_data &out,
                               std::string *error)
    {
        std::vector<byte> file;
        if (!fs::read_all(path, file))
        {
            set_error(error, "could not read '" + path + "'");
            return false;
        }
        return parse(file, out, error);
    }
}
