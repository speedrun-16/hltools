#pragma once

#include <string>
#include <vector>

#include "common/cmdline.h"
#include "format/bsp/data.h"
#include "format/bsp/entity_lump.h"

namespace tools
{
    using compile_parameters = std::vector<format::entity::pair>;

    // reads every info_compile_parameters pair from editable map text
    compile_parameters compile_parameters_from_map(const std::string &source);

    // prefers the editable map in the embedded source zip, then falls back to
    // an info_compile_parameters entity carried in an intermediate bsp
    compile_parameters compile_parameters_from_bsp(const format::map_data &map);

    // prepends a stage's entity derived defaults to the real argv so existing
    // command line arguments remain last and win for scalar values
    struct staged_arguments
    {
        std::vector<std::string> storage;
        std::vector<char *> argv;

        staged_arguments(
            int argc, char **actual, const compile_parameters &parameters,
            cli::compiler_stage stage);

        int argc() const {
            return (int)storage.size();
        }
    };

    // produces the normalized stage prefixed recipe embedded in the map
    compile_parameters canonical_compile_parameters(
        int argc, char **argv, const compile_parameters &existing);

    // the entity may travel through traditional intermediate bsps so later
    // stage commands can consume it, but must not remain in the runtime bsp
    void erase_runtime_compile_parameters(format::map_data &map);
}
