#include <cstdio>
#include <string>
#include <vector>

#include "bsp/internal.h"

namespace
{
    // an axis aligned quad in the z = 0 plane from (x0, y0) to (x1, y1), drawn with texinfo *tex*
    void add_quad(format::map_data &map, float x0, float y0, float x1, float y1, short tex)
    {
        unsigned short first_vertex = (unsigned short)map.vertexes.size();
        const float points[4][2] = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
        for (const auto &point : points)
        {
            format::dvertex_t vertex = {};
            vertex.point[0] = point[0];
            vertex.point[1] = point[1];
            map.vertexes.push_back(vertex);
        }
        int first_edge = (int)map.surfedges.size();
        for (int i = 0; i < 4; i++)
        {
            format::dedge_t edge = {};
            const bool reverse = i % 2 != 0;
            edge.v[reverse ? 1 : 0] = (unsigned short)(first_vertex + i);
            edge.v[reverse ? 0 : 1] = (unsigned short)(first_vertex + (i + 1) % 4);
            const int edge_index = (int)map.edges.size();
            map.surfedges.push_back(reverse ? -edge_index : edge_index);
            map.edges.push_back(edge);
        }
        format::dface_t face = {};
        face.firstedge = first_edge;
        face.numedges = 4;
        face.texinfo = tex;
        map.faces.push_back(face);
    }

    bool expect(bool condition, const char *message)
    {
        if (!condition)
            std::printf("%s failed\n", message);
        return condition;
    }
}

int main()
{
    format::map_data map;
    map.edges.push_back({}); // edge 0 is reserved, like in a real bsp
    const std::vector<std::string> names = {"!water1", "concrete", "sky", "WATER_dark"};

    add_quad(map, 0, 0, 1024, 1024, 0);            // 0: water inside the range
    add_quad(map, 0, 11000, 1024, 12000, 0);       // 1: water wholly beyond +9999 in y
    add_quad(map, 0, 9800, 1024, 10400, 0);        // 2: water crossing +9999 (its pieces past it recurse)
    add_quad(map, 0, 20000, 1024, 21000, 1);       // 3: an ordinary wall far out: not warped, fine
    add_quad(map, -12000, 0, -11000, 512, 2);      // 4: sky beyond -9999 in x
    add_quad(map, -9999, -9999, 9999, 9999, 3);    // 5: water exactly on the limit: fine
    add_quad(map, 9900, 0, 9999, 512, 3);        // 6: water submodel with one vertex past +9999
    map.vertexes[map.vertexes.size() - 3].point[0] = 9999.25f; // starts the reversed surfedge
    format::dmodel_t world = {};
    world.firstface = 0;
    world.numfaces = 5;
    map.models.push_back(world);
    format::dmodel_t water_entity = {};
    water_entity.firstface = 5;
    water_entity.numfaces = 2;
    map.models.push_back(water_entity);

    std::vector<bsp::warped_face_outside_range> found = bsp::find_warped_faces_outside_range(map, names);

    bool ok = true;
    ok &= expect(found.size() == 4, "four warped faces outside the range");
    ok &= expect(found.size() > 0 && found[0].face == 1 && found[0].texture == "!water1", "water beyond +y");
    ok &= expect(found.size() > 1 && found[1].face == 2, "water crossing the limit");
    ok &= expect(found.size() > 2 && found[2].face == 4 && found[2].model == 0, "sky beyond -x");
    ok &= expect(found.size() > 3 && found[3].face == 6 && found[3].model == 1
                     && found[3].maxs[0] == 9999.25f,
                 "fractional bound beyond the limit on a submodel");
    ok &= expect(found.size() > 0 && found[0].mins[1] == 11000 && found[0].maxs[1] == 12000, "bounds reported");
    ok &= expect(bsp::texture_is_warped("!lava") && bsp::texture_is_warped("SKY")
                     && bsp::texture_is_warped("water_blue") && !bsp::texture_is_warped("concrete"),
                 "warped texture names");
    return ok ? 0 : 1;
}
