#include <cstring>
#include <string>

#include "format/bsp/entity_lump.h"
#include "support/test.h"

suite("unit.format.entity_lump")
{
    test("entity_lump.parses entity keys and missing values")
    {
        const char *text = "{\n"
                           "\"classname\" \"worldspawn\"\n"
                           "\"wad\" \"x.wad\"\n"
                           "\"message\" \"hello\"\n"
                           "}\n";

        auto entities = format::parse_entities(text);
        require(entities.size() == 1);
        expect(std::strcmp(entities[0].value("classname"), "worldspawn") == 0);
        expect(std::strcmp(entities[0].value("message"), "hello") == 0);
        expect(entities[0].value("missing")[0] == '\0');
    }

    test("entity_lump.writes reversed key order with a trailing null")
    {
        const char *text = "{\n"
                           "\"classname\" \"worldspawn\"\n"
                           "\"wad\" \"x.wad\"\n"
                           "\"message\" \"hello\"\n"
                           "}\n";
        const char *expected = "{\n"
                               "\"message\" \"hello\"\n"
                               "\"wad\" \"x.wad\"\n"
                               "\"classname\" \"worldspawn\"\n"
                               "}\n";

        std::string output = format::write_entities(format::parse_entities(text));
        require(output.size() == std::strlen(expected) + 1);
        expect(std::memcmp(output.data(), expected, std::strlen(expected)) == 0);
        expect(output.back() == '\0');
    }

    test("entity_lump.replaces prepends and removes keys")
    {
        auto entities = format::parse_entities("{\n\"classname\" \"worldspawn\"\n}\n");
        require(entities.size() == 1);

        entities[0].set("classname", "changed");
        expect(std::strcmp(entities[0].value("classname"), "changed") == 0);
        entities[0].set("newkey", "v");
        expect(entities[0].pairs().front().first == "newkey");
        entities[0].set("newkey", "");
        expect_false(entities[0].has("newkey"));
    }
}
