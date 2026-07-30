#include "test.h"

#include <cstring>
#include <string>
#include <vector>

#include "common/filesystem.h"
#include "format/bsp/data.h"
#include "format/bsp/file.h"
#include "format/map/document.h"
#include "format/zip/archive.h"

namespace
{
    std::string scratch(const char *name)
    {
        return std::string(HLTOOLS_TEST_SCRATCH_DIRECTORY) + "/" + name;
    }
}

suite("unit.format.bsp_extensions")
{
    test("round trips BSPX lumps and an embedded ZIP tail")
    {
        format::map_data source;
        source.entities = "{\n\"classname\" \"worldspawn\"\n}\n";
        source.bspx.push_back({
            "ABCDEFGHIJKLMNOPQRSTUVWX", {1, 2, 3, 4, 5}
        });
        std::string zip_error;
        require(format::create_zip({
            {format::embedded_map_name, {'s', 'o', 'u', 'r', 'c', 'e'}}
        }, source.embedded_zip, &zip_error));

        const std::string path = scratch("bsp_extensions.bsp");
        require(format::bsp_file::write(path, source));

        format::map_data loaded;
        require(format::bsp_file::load(path, loaded));
        require(loaded.bspx.size() == 2);
        expect(loaded.bspx[0].name == "ABCDEFGHIJKLMNOPQRSTUVWX");
        expect(loaded.bspx[0].data == source.bspx[0].data);
        expect(loaded.bspx[1].name == format::bsp_file::embed_locator_lump);
        expect(loaded.embedded_zip == source.embedded_zip);
    }

    test("vanilla maps do not acquire BSPX data")
    {
        format::map_data source;
        source.entities = "{\n\"classname\" \"worldspawn\"\n}\n";
        const std::string first = scratch("bsp_vanilla_first.bsp");
        const std::string second = scratch("bsp_vanilla_second.bsp");
        require(format::bsp_file::write(first, source));

        format::map_data loaded;
        require(format::bsp_file::load(first, loaded));
        require(format::bsp_file::write(second, loaded));

        std::vector<unsigned char> a;
        std::vector<unsigned char> b;
        require(fs::read_all(first, a));
        require(fs::read_all(second, b));
        expect(a == b);
    }

    test("creates and reads a Deflate ZIP in memory")
    {
        std::vector<unsigned char> source(4096, 'a');
        std::vector<unsigned char> archive;
        std::string error;
        require(format::create_zip({
            {format::embedded_map_name, source}
        }, archive, &error));
        expect(archive.size() < source.size());

        std::vector<unsigned char> extracted;
        std::string name;
        require(format::read_zip_map(archive, extracted, &name, &error));
        expect(name == format::embedded_map_name);
        expect(extracted == source);

        std::vector<unsigned char> original;
        expect(!format::read_zip_entry(
            archive, "hltools/original.map", original));

        require(format::create_zip({
            {"renamed.map", source}
        }, archive, &error));
        require(format::read_zip_map(archive, extracted, &name, &error));
        expect(name == "renamed.map");
        expect(extracted == source);
    }

    test("parses erases and appends MAP entities without rewriting source")
    {
        std::string edited =
            "// source comment\n"
            "{\n\"classname\" \"worldspawn\"\n"
            "{\n( 0 0 0 ) ( 1 0 0 ) ( 0 1 0 ) NULL [ 1 0 0 0 ] "
            "[ 0 1 0 0 ] 0 1 1\n}\n}\n"
            "{\n\"classname\" \"info_compile_parameters\"\n"
            "\"rad_bounce\" \"2\"\n}\n"
            "{\n\"classname\" \"info_texlights\"\n"
            "\"UNUSED\" \"255 0 0\"\n}\n";

        const auto parsed = format::parse_map_source_entities(edited);
        require(parsed.size() == 3);
        expect(std::string(parsed[0].keyvalues.value("classname"))
               == "worldspawn");

        format::erase_map_entities(
            edited, {"info_compile_parameters", "info_texlights"});
        format::entity parameters;
        parameters.append("classname", "info_compile_parameters");
        parameters.append("rad_bounce", "8");
        format::append_map_entity(edited, parameters);

        format::entity texlights;
        texlights.append("classname", "info_texlights");
        texlights.append("USED", "1 2 3");
        format::append_map_entity(edited, texlights);

        expect(edited.find("// source comment") != std::string::npos);
        expect(edited.find("\"rad_bounce\" \"2\"") == std::string::npos);
        expect(edited.find("\"UNUSED\"") == std::string::npos);
        expect(edited.find("\"rad_bounce\" \"8\"") != std::string::npos);
        expect(edited.find("\"USED\" \"1 2 3\"") != std::string::npos);
    }
}
