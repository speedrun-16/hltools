#include <filesystem>
#include <string>
#include <vector>

#include "common/filesystem.h"
#include "common/types.h"
#include "decompile/source/models.h"
#include "support/scratch.h"
#include "support/test.h"

suite("unit.decompile.source_models")
{
    test("loose conversion locates an unsuffixed vtx sibling and reports decoder errors")
    {
        std::filesystem::path root =
            test_support::scratch_directory("source_model_convert");
        std::filesystem::path base = root / "probe";
        std::vector<byte> invalid(512, 0);
        invalid[0] = 'I';
        invalid[1] = 'D';
        invalid[2] = 'S';
        invalid[3] = 'T';

        require(fs::write_all((base.string() + ".mdl"), invalid.data(), invalid.size()));
        require(fs::write_all((base.string() + ".vvd"), invalid.data(), invalid.size()));
        require(fs::write_all((base.string() + ".vtx"), invalid.data(), invalid.size()));

        decompile::source_model_conversion result;
        result.data.push_back(1);
        result.vertices = 99;
        std::string error;
        expect_false(decompile::convert_source_model(
            base.string() + ".mdl", {}, result, &error));
        expect(result.data.empty());
        expect(result.vertices == 0);
        expect(error == "unsupported .mdl version 0");
    }

    test("loose conversion names a missing vvd sidecar")
    {
        std::filesystem::path root =
            test_support::scratch_directory("source_model_missing_vvd");
        std::filesystem::path mdl = root / "probe.mdl";
        std::vector<byte> invalid(64, 0);
        require(fs::write_all(mdl.string(), invalid.data(), invalid.size()));

        decompile::source_model_conversion result;
        std::string error;
        expect_false(decompile::convert_source_model(mdl.string(), {}, result, &error));
        expect(error.find("probe.vvd") != std::string::npos);
    }
}
