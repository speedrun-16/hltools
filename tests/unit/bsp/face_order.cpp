#include <cstdio>

#include "bsp/internal.h"

namespace
{
    void add_face(format::map_data &map, int lightmap_width, int lightmap_height,
                  unsigned short identity)
    {
        unsigned short first_vertex = (unsigned short)map.vertexes.size();
        float width = (float)((lightmap_width - 1) * 16);
        float height = (float)((lightmap_height - 1) * 16);
        const float points[4][3] = {
            {0, 0, 0},
            {width, 0, 0},
            {width, height, 0},
            {0, height, 0},
        };
        for (const auto &point : points)
        {
            format::dvertex_t vertex = {};
            vertex.point[0] = point[0];
            vertex.point[1] = point[1];
            vertex.point[2] = point[2];
            map.vertexes.push_back(vertex);
        }

        int first_edge = (int)map.edges.size();
        for (int i = 0; i < 4; i++)
        {
            format::dedge_t edge = {};
            edge.v[0] = (unsigned short)(first_vertex + i);
            edge.v[1] = (unsigned short)(first_vertex + (i + 1) % 4);
            map.edges.push_back(edge);
            map.surfedges.push_back(first_edge + i);
        }

        format::dface_t face = {};
        face.planenum = identity;
        face.firstedge = first_edge;
        face.numedges = 4;
        face.texinfo = 0;
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
    format::texinfo_t texinfo = {};
    texinfo.vecs[0][0] = 1;
    texinfo.vecs[1][1] = 1;
    map.texinfo.push_back(texinfo);

    const int rectangles[][2] = {
        {24, 28}, {20, 28}, {8, 24}, {16, 4},
        {20, 32}, {20, 32}, {32, 12}, {16, 16}, {12, 4},
        {28, 28}, {24, 32}, {28, 24}, {28, 16}, {8, 8}, {12, 4},
        {28, 32}, {28, 28}, {32, 24}, {16, 28}, {24, 12}, {16, 8},
        {32, 24}, {32, 24}, {32, 20}, {20, 28}, {16, 24}, {12, 24},
    };
    const int group_counts[] = {4, 5, 6, 6, 6};
    int face_index = 0;
    for (int count : group_counts)
    {
        format::dnode_t node = {};
        node.firstface = (unsigned short)face_index;
        node.numfaces = (unsigned short)count;
        map.nodes.push_back(node);
        for (int i = 0; i < count; i++, face_index++)
        {
            add_face(map, rectangles[face_index][0], rectangles[face_index][1],
                     (unsigned short)face_index);
        }
    }

    format::dmodel_t model = {};
    model.firstface = 0;
    model.numfaces = face_index;
    map.models.push_back(model);
    map.marksurfaces = {0, 4, 9, 15, 21, 26};

    bsp::bsp_state state;
    state.map = &map;
    bsp::optimize_face_order(state);

    bool ok = true;
    ok &= expect(map.faces[0].planenum == 21, "largest node first");
    ok &= expect(map.faces[6].planenum == 15, "second largest node second");
    ok &= expect(map.faces[12].planenum == 9, "middle node third");
    ok &= expect(map.faces[18].planenum == 4, "second smallest node fourth");
    ok &= expect(map.faces[23].planenum == 0, "smallest node last");
    ok &= expect(map.marksurfaces == std::vector<unsigned short>({23, 18, 12, 6, 0, 5}),
                 "marksurfaces remapped");
    ok &= expect(map.nodes[0].firstface == 23 && map.nodes[0].numfaces == 4,
                 "first node face range moved");
    ok &= expect(map.nodes[4].firstface == 0 && map.nodes[4].numfaces == 6,
                 "last node face range moved");
    ok &= expect(map.models[0].firstface == 0 && map.models[0].numfaces == 27,
                 "model face range preserved");
    return ok ? 0 : 1;
}
