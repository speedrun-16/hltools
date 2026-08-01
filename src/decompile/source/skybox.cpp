#include "skybox.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <system_error>

#include "common/binary.h"
#include "common/filesystem.h"
#include "format/vbsp/data.h"
#include "format/vbsp/pakfile.h"
#include "format/vmt/vmt.h"
#include "format/vpk/vpk.h"
#include "format/vtf/vtf.h"

namespace decompile
{
    namespace
    {
        // goldsrc and source name the six faces identically, so the suffix maps
        // straight across. the order here is only the order the files come out in.
        const char *const face_suffixes[6] = {"bk", "dn", "ft", "lf", "rt", "up"};

        bool iequals(const std::string &a, const char *b)
        {
            std::size_t i = 0;
            for (; i < a.size(); i++)
            {
                if (b[i] == '\0'
                    || std::tolower((unsigned char)a[i]) != std::tolower((unsigned char)b[i]))
                    return false;
            }
            return b[i] == '\0';
        }

        std::string to_lower(std::string s)
        {
            for (char &c : s)
                c = (char)std::tolower((unsigned char)c);
            return s;
        }

        // vmt texture references are game relative even when authored with a
        // leading slash or backslashes; pak and vpk entries carry neither.
        std::string normalize_reference(std::string path)
        {
            std::replace(path.begin(), path.end(), '\\', '/');
            while (!path.empty() && path.front() == '/')
                path.erase(path.begin());
            return path;
        }

        // pakfile first, then loose files under the content dir, then the game's
        // vpk archives: the engine's search order.
        bool load_asset(const format::pakfile &pak,
                        const std::vector<std::string> &game_dirs,
                        const std::vector<format::vpk_archive> &vpks,
                        const std::string &relative, std::vector<byte> &out)
        {
            if (pak.extract(relative, out))
                return true;
            for (const std::string &dir : game_dirs)
                if (!dir.empty() && fs::read_all(dir + "/" + relative, out))
                    return true;
            for (const format::vpk_archive &vpk : vpks)
                if (vpk.extract(relative, out))
                    return true;
            return false;
        }

        std::vector<format::vpk_archive>
        open_game_vpks(const std::vector<std::string> &game_dirs)
        {
            std::vector<format::vpk_archive> vpks;
            for (const std::string &game_dir : game_dirs)
            {
                if (game_dir.empty())
                    continue;
                std::error_code ec;
                for (const auto &item : std::filesystem::directory_iterator(game_dir, ec))
                {
                    std::string path = item.path().string();
                    if (path.size() < 8
                        || !iequals(path.substr(path.size() - 8), "_dir.vpk"))
                        continue;
                    format::vpk_archive vpk;
                    if (vpk.open(path))
                        vpks.push_back(std::move(vpk));
                }
            }
            return vpks;
        }

        // the vtf a face's material draws from. patch materials override their
        // base, matching how source resolves $basetexture.
        std::string face_basetexture(const format::pakfile &pak, const std::vector<std::string> &game_dirs,
                                     const std::vector<format::vpk_archive> &vpks,
                                     const std::string &material_path)
        {
            std::vector<byte> text;
            if (!load_asset(pak, game_dirs, vpks, material_path, text))
                return std::string();

            format::vmt_material vmt;
            if (!parse_vmt(text, vmt))
                return std::string();

            std::string value = vmt.get("$basetexture");
            if (value.empty() && vmt.is_patch())
            {
                std::vector<byte> base_text;
                format::vmt_material base;
                std::string include = normalize_reference(vmt.patch_include());
                if (!include.empty()
                    && load_asset(pak, game_dirs, vpks, include, base_text)
                    && parse_vmt(base_text, base))
                    value = base.get("$basetexture");
            }
            return normalize_reference(value);
        }

        std::vector<byte> resample_nearest(const std::vector<byte> &src, unsigned sw,
                                           unsigned sh, unsigned dw, unsigned dh)
        {
            std::vector<byte> out((std::size_t)dw * dh * 3);
            for (unsigned y = 0; y < dh; y++)
            {
                unsigned sy = sh ? (y * sh) / dh : 0;
                for (unsigned x = 0; x < dw; x++)
                {
                    unsigned sx = sw ? (x * sw) / dw : 0;
                    std::size_t s = ((std::size_t)sy * sw + sx) * 3;
                    std::size_t o = ((std::size_t)y * dw + x) * 3;
                    out[o] = src[s];
                    out[o + 1] = src[s + 1];
                    out[o + 2] = src[s + 2];
                }
            }
            return out;
        }

        void apply_exposure(std::vector<byte> &rgb, double exposure)
        {
            if (exposure == 1.0)
                return;
            for (byte &channel : rgb)
            {
                double value = (double)channel * exposure;
                channel = (byte)std::clamp((int)std::lround(value), 0, 255);
            }
        }

        // uncompressed 24 bit truecolor tga, bottom row first: byte for byte the
        // layout of the stock goldsrc gfx/env skies.
        std::vector<byte> write_tga(const std::vector<byte> &rgb, unsigned width,
                                    unsigned height)
        {
            std::vector<byte> out;
            out.reserve(18 + (std::size_t)width * height * 3);

            binary::writer sink(out);
            sink.u8(0); // no id field
            sink.u8(0); // no colour map
            sink.u8(2); // uncompressed truecolour
            sink.u16(0);
            sink.u16(0);
            sink.u8(0); // colour map specification, unused
            sink.u16(0);
            sink.u16(0); // image origin
            sink.u16((std::uint16_t)width);
            sink.u16((std::uint16_t)height);
            sink.u8(24); // bits per pixel
            sink.u8(0);  // origin bottom left, no alpha bits

            std::vector<byte> row((std::size_t)width * 3);
            for (unsigned y = 0; y < height; y++)
            {
                // source rows run top to bottom, the file runs bottom to top
                const byte *src = rgb.data() + (std::size_t)(height - 1 - y) * width * 3;
                for (unsigned x = 0; x < width; x++)
                {
                    row[x * 3 + 0] = src[x * 3 + 2]; // b
                    row[x * 3 + 1] = src[x * 3 + 1]; // g
                    row[x * 3 + 2] = src[x * 3 + 0]; // r
                }
                sink.raw(row);
            }
            return out;
        }

        // goldsrc looks the sky up as gfx/env/<skyname><suffix>.tga, with no
        // subdirectory, so only the final component of a source sky name survives.
        std::string goldsrc_sky_name(const std::string &source_name)
        {
            std::string name = normalize_reference(source_name);
            std::size_t slash = name.find_last_of('/');
            if (slash != std::string::npos)
                name = name.substr(slash + 1);
            for (char &c : name)
            {
                if (!(std::isalnum((unsigned char)c) || c == '_' || c == '-'))
                    c = '_';
            }
            return to_lower(name);
        }
    }

    bool export_source_skybox(const format::source_map_data &map,
                              const std::vector<std::string> &game_dirs,
                              const std::string &source_sky_name,
                              unsigned face_size, double exposure,
                              skybox_result &out)
    {
        out = skybox_result{};
        if (source_sky_name.empty() || face_size == 0)
            return false;

        out.sky_name = goldsrc_sky_name(source_sky_name);
        if (out.sky_name.empty())
            return false;

        format::pakfile pak;
        pak.open(map.pakfile);
        std::vector<format::vpk_archive> vpks = open_game_vpks(game_dirs);

        std::string base = "materials/skybox/" + normalize_reference(source_sky_name);
        for (const char *suffix : face_suffixes)
        {
            std::string reference =
                face_basetexture(pak, game_dirs, vpks, base + suffix + ".vmt");

            std::vector<byte> vtf;
            // a face without a usable vmt may still ship the vtf under the
            // material's own name, which is enough to rebuild the face
            if (reference.empty()
                || !load_asset(pak, game_dirs, vpks, "materials/" + reference + ".vtf", vtf))
            {
                if (!load_asset(pak, game_dirs, vpks, base + suffix + ".vtf", vtf))
                {
                    out.missing++;
                    continue;
                }
            }

            format::vtf_image image;
            if (!decode_vtf(vtf, face_size, image) || image.width == 0 || image.height == 0)
            {
                out.missing++;
                continue;
            }

            const std::vector<byte> *rgb = &image.rgb;
            std::vector<byte> resized;
            if (image.width != face_size || image.height != face_size)
            {
                resized = resample_nearest(image.rgb, image.width, image.height,
                                           face_size, face_size);
                rgb = &resized;
            }

            std::vector<byte> exposed = *rgb;
            apply_exposure(exposed, exposure);

            skybox_face face;
            face.filename = out.sky_name + suffix + ".tga";
            face.tga = write_tga(exposed, face_size, face_size);
            out.faces.push_back(std::move(face));
        }

        return !out.faces.empty();
    }
}
