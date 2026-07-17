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
}
