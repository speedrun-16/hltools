#include "math/winding.h"
#include "support/test.h"

suite("unit.math.winding")
{
    using winding = math::basic_winding<double>;
    using vector = math::vec3<double>;

    test("winding.builds a base winding and recovers its plane")
    {
        winding base = winding::from_plane(vector{0, 0, 1}, 0, 1000);
        require(base.size() == 4);
        expect(base.area() == 4000000.0);

        vector normal;
        double distance = 0;
        base.plane(normal, distance);
        expect(normal.x == 0);
        expect(normal.y == 0);
        expect(normal.z == 1);
        expect(distance == 0);
    }

    test("winding.splits a winding without losing area")
    {
        winding base = winding::from_plane(vector{0, 0, 1}, 0, 1000);
        winding front;
        winding back;
        base.clip(vector{1, 0, 0}, 0, front, back);

        require(front.size() == 4);
        require(back.size() == 4);
        expect(front.area() + back.area() == base.area());
    }

    test("winding.uses on-epsilon when classifying points")
    {
        winding near_split({{-1, 0, 0}, {0.005, 0, 0}, {0.005, 1, 0}, {-1, 1, 0}});
        winding near_front;
        winding near_back;
        near_split.clip(vector{1, 0, 0}, 0, near_front, near_back);
        expect(near_front.empty());
        expect(near_back.size() == 4);

        winding wide_split({{-1, 0, 0}, {0.05, 0, 0}, {0.05, 1, 0}, {-1, 1, 0}});
        winding wide_front;
        winding wide_back;
        wide_split.clip(vector{1, 0, 0}, 0, wide_front, wide_back);
        expect(wide_front.size() == 4);
        expect(wide_back.size() == 4);
    }
}
