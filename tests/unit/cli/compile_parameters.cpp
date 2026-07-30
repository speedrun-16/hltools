#include "test.h"

#include <string>
#include <vector>

#include "cli/tools/compile_parameters.h"
#include "common/cmdline.h"
#include "format/bsp/entity_lump.h"
#include "format/map/document.h"
#include "format/zip/archive.h"

suite("unit.cli.compile_parameters")
{
    test("uses one registry for stage membership persistence and arity")
    {
        const cli::option_spec *scale = cli::find_option_spec("-scale");
        require(scale != nullptr);
        expect(scale->arity == 1);
        expect(scale->persist);
        expect(cli::option_applies_to(cli::compiler_stage::csg, "-scale", true));
        expect(cli::option_applies_to(cli::compiler_stage::rad, "-scale", true));
        expect(!cli::option_applies_to(cli::compiler_stage::vis, "-scale", true));

        const cli::option_spec *ambient = cli::find_option_spec("-ambient");
        require(ambient != nullptr);
        expect(ambient->arity == 3);
        expect(cli::option_applies_to(
            cli::compiler_stage::rad, "-ambient", true));

        expect(cli::option_applies_to(
            cli::compiler_stage::csg, "-onlyents"));
        expect(!cli::option_applies_to(
            cli::compiler_stage::csg, "-onlyents", true));
    }

    test("prepends entity defaults so explicit stage arguments win")
    {
        const std::string source =
            "{\n\"classname\" \"worldspawn\"\n}\n"
            "{\n\"classname\" \"info_compile_parameters\"\n"
            "\"threads\" \"2\"\n"
            "\"rad_fast\" \"1\"\n"
            "\"rad_bounce\" \"4\"\n"
            "\"vis_fast\" \"1\"\n}\n";
        const tools::compile_parameters parameters =
            tools::compile_parameters_from_map(source);

        std::vector<std::string> actual = {
            "hltools", "-bounce", "9", "sample"
        };
        std::vector<char *> argv;
        for (std::string &value : actual)
            argv.push_back(&value[0]);
        tools::staged_arguments staged(
            (int)actual.size(), argv.data(), parameters,
            cli::compiler_stage::rad);
        cli::args args(staged.argc(), staged.argv.data());

        expect(args.has("-fast"));
        expect(args.int_value("-threads", 0) == 2);
        expect(args.int_value("-bounce", 0) == 9);
        expect(args.map_name() == "sample");
    }

    test("prefers embedded parameters and removes runtime metadata")
    {
        format::entity world;
        world.append("classname", "worldspawn");
        format::entity runtime;
        runtime.append("classname", "info_compile_parameters");
        runtime.append("rad_bounce", "3");

        format::map_data map;
        map.entities = format::write_entities({world, runtime});
        const std::vector<format::map_source_entity> original_entities =
            format::parse_map_source_entities(map.entities);
        require(original_entities.size() == 2);
        const std::string original_world(
            map.entities, original_entities[0].begin,
            original_entities[0].end - original_entities[0].begin);
        const std::string embedded =
            "{\n\"classname\" \"worldspawn\"\n}\n"
            "{\n\"classname\" \"info_compile_parameters\"\n"
            "\"rad_bounce\" \"7\"\n}\n";
        std::string error;
        require(format::create_zip({
            {format::embedded_map_name,
             std::vector<byte>(embedded.begin(), embedded.end())}
        }, map.embedded_zip, &error));

        const tools::compile_parameters parameters =
            tools::compile_parameters_from_bsp(map);
        require(parameters.size() == 1);
        expect(parameters[0].first == "rad_bounce");
        expect(parameters[0].second == "7");

        tools::erase_runtime_compile_parameters(map);
        const std::vector<format::map_source_entity> remaining_entities =
            format::parse_map_source_entities(map.entities);
        require(remaining_entities.size() == 1);
        expect(std::string(
            map.entities, remaining_entities[0].begin,
            remaining_entities[0].end - remaining_entities[0].begin)
            == original_world);
    }
}
