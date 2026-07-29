#include "common/build_info.h"
#include "support/test.h"

suite("unit.common.build_info")
{
    test("build_info.formats compiler metadata with and without a scope")
    {
        const std::string suffix = " (" + build_info::date() + ", "
            + build_info::commit() + ")";
        expect(build_info::compiler() == std::string("hltools ")
            + build_info::version() + suffix);
        expect(build_info::compiler("rad") == std::string("hltools ")
            + build_info::version() + " - rad " + suffix);
    }

    test("build_info.formats the compile instant as UTC ISO 8601")
    {
        const std::string &value = build_info::compiled_at();
        require(value.size() == 20);
        expect(value[4] == '-');
        expect(value[7] == '-');
        expect(value[10] == 'T');
        expect(value[13] == ':');
        expect(value[16] == ':');
        expect(value[19] == 'Z');
    }
}
