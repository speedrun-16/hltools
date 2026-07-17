#include <fstream>
#include <vector>

#include "format/rad/texlights.h"
#include "support/scratch.h"
#include "support/test.h"

namespace
{
    const format::texlight *find(const std::vector<format::texlight> &lights,
                                 const char *name)
    {
        for (const format::texlight &light : lights)
            if (light.name == name)
                return &light;
        return nullptr;
    }
}
suite("unit.format.rad_file")
{
    test("rad_file.parses colours scaling comments and overrides")
    {
        std::filesystem::path path =
            test_support::scratch_directory("rad_file") / "lights.rad";
        std::ofstream output(path);
        output << "// a comment line\n";
        output << "GRAY 200\n";
        output << "COLOR 10 20 30\n";
        output << "SCALED 255 255 255 128\n";
        output << "COLOR 1 2 3 // override\n";
        output << "junk\n";
        output.close();

        std::vector<format::texlight> lights;
        expect(format::read_rad_file(path.string(), lights) == 4);

        const format::texlight *gray = find(lights, "GRAY");
        require(gray != nullptr);
        expect(gray->value[0] == 200);
        expect(gray->value[1] == 200);
        expect(gray->value[2] == 200);

        const format::texlight *color = find(lights, "COLOR");
        require(color != nullptr);
        expect(color->value[0] == 1);
        expect(color->value[1] == 2);
        expect(color->value[2] == 3);

        const format::texlight *scaled = find(lights, "SCALED");
        require(scaled != nullptr);
        expect(scaled->value[0] == (float)(255 * (128 / 255.0)));
    }
}
