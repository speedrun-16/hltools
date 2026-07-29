#pragma once

#include <string>
#include <utility>
#include <vector>

// entity key/value data parsed from the entities lump text a stage that only
// passes entities through leaves map_dataentities untouched; a stage that edits
// them parses to these, mutates, and writes back
//
// pair order is load bearing for byte identical output the reference stored
// epairs in a prepended linked list, so a key read from the text ended up ahead
// of the keys read before it, and that reversed order is what got written back
// this class matches that: pairs_ is head first (index 0 is written first), the
// parser prepends, set() prepends a new key, and value() returns the first match

namespace format
{
    class entity
    {
    public:
        using pair = std::pair<std::string, std::string>;

        // value for a key, or "" if absent (mirrors the old valueforkey)
        const char *value(const char *key) const;
        bool has(const char *key) const;

        // replace the key in place if present, otherwise prepend it an empty
        // value removes the key, matching setkeyvalue
        void set(const char *key, const char *value);
        void remove(const char *key);

        bool empty() const {
            return pairs_.empty();
        }
        const std::vector<pair> &pairs() const {
            return pairs_;
        }

        // parser hook: add a pair at the head, as the reference linked list did
        void prepend(std::string key, std::string value) {
            pairs_.emplace(pairs_.begin(), std::move(key), std::move(value));
        }

        // generator hook: add a pair at the tail, for code that builds entities
        // top-down and wants the written order to match the call order
        void append(std::string key, std::string value) {
            pairs_.emplace_back(std::move(key), std::move(value));
        }

    private:
        std::vector<pair> pairs_;
    };

    // parse the entities lump text into entities
    std::vector<entity> parse_entities(const std::string &text);

    // serialize entities back to the exact lump byte layout the reference wrote:
    // "{\n" then "\"key\" \"value\"\n" per pair then "}\n" per entity, empty
    // entities skipped, and a single trailing null so the lump length matches
    std::string write_entities(const std::vector<entity> &entities);

}
