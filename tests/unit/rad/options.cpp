#include <string>
#include <vector>

#include "cli/tools/rad_tool.h"
#include "common/cmdline.h"
#include "common/filesystem.h"
#include "support/scratch.h"
#include "support/test.h"

namespace
{
    rad::rad_options parse(const std::string &base)
    {
        std::vector<std::string> storage = {"hlrad", base};
        std::vector<char *> argv;
        for (std::string &argument : storage)
            argv.push_back(argument.data());
        cli::args args((int)argv.size(), argv.data());
        return tools::parse_rad_options(args, (int)argv.size(), argv.data(), base);
    }

    void write_definition(const std::filesystem::path &path)
    {
        const char definition[] = "LIGHT 255\n";
        require(fs::write_all(path.string(), definition, sizeof(definition) - 1));
    }
}
suite("unit.rad.options")
{
    test("options.tracks a generic lights.rad fallback")
    {
        std::filesystem::path scratch = test_support::scratch_directory("rad_options");
        std::string base = (scratch / "sample").string();
        std::string generic = (scratch / "lights.rad").string();
        write_definition(generic);

        rad::rad_options options = parse(base);
        require(options.rad_files.size() == 1);
        expect(options.rad_files[0] == generic);
        expect(options.generic_rad_file == generic);
    }

    test("options.loads map-specific definitions after the generic fallback")
    {
        std::filesystem::path scratch = test_support::scratch_directory("rad_options");
        std::string base = (scratch / "sample").string();
        std::string generic = (scratch / "lights.rad").string();
        std::string specific = base + ".rad";
        write_definition(generic);
        write_definition(specific);

        rad::rad_options options = parse(base);
        require(options.rad_files.size() == 2);
        expect(options.rad_files[0] == generic);
        expect(options.rad_files[1] == specific);
        expect(options.generic_rad_file == generic);
    }
}
