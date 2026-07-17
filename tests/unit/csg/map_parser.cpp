#include <cstring>
#include <fstream>

#include "common/build_info.h"
#include "csg/map_parser.h"
#include "support/scratch.h"
#include "support/test.h"

suite("unit.csg.map_parser")
{
    test("map_parser.applies tool textures and clamps detail settings")
    {
        std::filesystem::path path =
            test_support::scratch_directory("csg_map_parser") / "source.map";
        std::ofstream file(path, std::ios::binary);
        file <<
            "{\n"
            "\"classname\" \"worldspawn\"\n"
            "\"mapversion\" \"220\"\n"
            "\"targetname\" \"first\"\n"
            "\"targetname\" \"\"\n"
            "\"zhlt_detaillevel\" \"-1\"\n"
            "\"zhlt_chopdown\" \"-1\"\n"
            "\"zhlt_chopup\" \"-1\"\n"
            "\"zhlt_clipnodedetaillevel\" \"-1\"\n"
            "{\n"
            "( 0 0 0 ) ( 128 0 0 ) ( 0 128 0 ) CLIPBEVELBRUSH "
            "[ 1 0 0 0 ] [ 0 -1 0 0 ] 0 1 1 //TX1\n"
            "( 0 0 128 ) ( 0 128 128 ) ( 128 0 128 ) BEVELHINT "
            "[ 1 0 0 0 ] [ 0 -1 0 0 ] 0 1 1\n"
            "}\n"
            "}\n";
        file.close();

        csg::map_source map = csg::load_map_file(path.string());
        require(map.entities.size() == 1);
        require(map.brushes.size() == 1);
        require(map.sides.size() == 2);
        expect(std::strcmp(map.entities[0].value("targetname"), "") == 0);
        expect(std::strcmp(map.entities[0].value("compiler"),
                           build_info::compiler().c_str()) == 0);
        expect(map.brushes[0].bevel);
        expect(map.brushes[0].cliphull != 0);
        expect(map.brushes[0].detail_level == 0);
        expect(map.brushes[0].chop_down == 0);
        expect(map.brushes[0].chop_up == 0);
        expect(map.brushes[0].clipnode_detail_level == 0);
        expect(std::strcmp(map.sides[0].texture.name, "SKIP") == 0);
        expect(map.sides[0].bevel);
        expect(map.sides[0].texture.txcommand == '1');
        expect(std::strcmp(map.sides[1].texture.name, "NULL") == 0);
        expect(map.sides[1].bevel);
    }
}
