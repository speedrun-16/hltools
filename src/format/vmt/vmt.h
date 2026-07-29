#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "common/types.h"

namespace format
{
    // a parsed valve material. only the top level shader name and its immediate
    // key/values are kept (nested blocks such as proxies are skipped). keys are
    // lowercased for case insensitive lookup. patch materials expose their base
    // include path so the caller can resolve and merge it.
    struct vmt_material
    {
        std::string shader;
        std::unordered_map<std::string, std::string> values;

        bool is_patch() const;
        // the include target of a patch material, or "".
        std::string patch_include() const;
        // value for a key (case insensitive), or "" when absent.
        std::string get(const std::string &key) const;
    };

    bool parse_vmt(const std::vector<byte> &text, vmt_material &out,
                   std::string *error = nullptr);
}
