#include <cmath>

#include "decompile/source/displacements.h"
#include "format/vbsp/data.h"
#include "support/test.h"

namespace
{
    format::source_dvertex_t vertex(float x, float y, float z)
    {
        format::source_dvertex_t out{};
        out.point[0] = x;
        out.point[1] = y;
        out.point[2] = z;
        return out;
    }

    bool near(double a, double b)
    {
        return std::fabs(a - b) < 0.00001;
    }
}

suite("source displacement reconstruction")
{
    test("rotates the base quad to startPosition and emits the full power grid")
    {
        format::source_map_data map;
        format::source_dplane_t plane{};
        plane.normal[2] = 1;
        map.planes.push_back(plane);

        map.vertexes = {
            vertex(0, 0, 0),
            vertex(0, 4, 0),
            vertex(4, 4, 0),
            vertex(4, 0, 0),
        };
        map.edges = {{{0, 1}}, {{1, 2}}, {{2, 3}}, {{3, 0}}};
        map.surfedges = {0, 1, 2, 3};

        format::source_dface_t face{};
        face.planenum = 0;
        face.firstedge = 0;
        face.numedges = 4;
        face.texinfo = 7;
        face.dispinfo = 0;
        map.faces.push_back(face);

        format::source_ddispinfo_t info{};
        info.start_position[0] = 0;
        info.start_position[1] = 4;
        info.start_position[2] = 0;
        info.disp_vert_start = 0;
        info.power = 2;
        info.map_face = 0;
        map.dispinfo.push_back(info);

        map.disp_verts.resize(25);
        for (format::source_ddispvert_t &disp : map.disp_verts)
        {
            disp.vector[2] = 1;
            disp.dist = 1;
        }
        map.disp_verts[0].dist = 5;

        decompile::source_displacement_mesh mesh =
            decompile::build_source_displacement_mesh(map);
        require(mesh.faces == 1);
        expect(mesh.skipped_faces == 0);
        expect(mesh.skipped_triangles == 0);
        require(mesh.triangles.size() == 32);
        require(mesh.collision_triangles.size() == 32);

        bool found_start = false;
        for (const decompile::source_displacement_triangle &triangle
             : mesh.triangles)
        {
            expect(triangle.texinfo == 7);
            expect(near(triangle.minimum_projection, 1));
            math::vec3<double> edge0 =
                triangle.points[1] - triangle.points[0];
            math::vec3<double> edge1 =
                triangle.points[2] - triangle.points[0];
            expect(math::dot(math::cross(edge0, edge1),
                             triangle.base_normal) > 0);
            for (const math::vec3<double> &point : triangle.points)
            {
                expect(point.x >= 0);
                expect(point.x <= 4);
                expect(point.y >= 0);
                expect(point.y <= 4);
                if (near(point.x, 0) && near(point.y, 4) && near(point.z, 5))
                    found_start = true;
            }
        }
        expect(found_start);
    }
}
