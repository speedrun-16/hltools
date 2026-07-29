from __future__ import annotations

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

import remove_map_group_volume as remover


def face(point, u, v, texture="TEST"):
    p0 = tuple(point[i] + u[i] for i in range(3))
    p1 = point
    p2 = tuple(point[i] + v[i] for i in range(3))
    values = " ".join(
        f"( {p[0]} {p[1]} {p[2]} )" for p in (p0, p1, p2)
    )
    return f"{values} {texture} [ 1 0 0 0 ] [ 0 1 0 0 ] 0 1 1\n"


def box(minimum, maximum, texture="TEST"):
    x0, y0, z0 = minimum
    x1, y1, z1 = maximum
    sides = [
        ((x0, y0, z0), (0, 0, 1), (0, 1, 0)),  # -x
        ((x1, y0, z0), (0, 1, 0), (0, 0, 1)),  # +x
        ((x0, y0, z0), (1, 0, 0), (0, 0, 1)),  # -y
        ((x0, y1, z0), (0, 0, 1), (1, 0, 0)),  # +y
        ((x0, y0, z0), (0, 1, 0), (1, 0, 0)),  # -z
        ((x0, y0, z1), (1, 0, 0), (0, 1, 0)),  # +z
    ]
    return "{\n" + "".join(face(p, u, v, texture) for p, u, v in sides) + "}\n"


def entity(number, keyvalues, brushes=()):
    keys = "".join(f'"{key}" "{value}"\n' for key, value in keyvalues)
    body = "".join(f"// brush {index}\n{brush}" for index, brush in enumerate(brushes))
    return f"// entity {number}\n{{\n{keys}{body}}}\n"


def fixture_map():
    world = entity(
        0,
        (("classname", "worldspawn"),),
        (
            box((20, 20, 20), (30, 30, 30), "INSIDE"),
            box((110, 110, 110), (120, 120, 120), "OUTSIDE"),
            box((5, 40, 40), (11, 50, 50), "CROSSING"),
        ),
    )
    shell = (
        box((0, 0, 0), (10, 100, 100), "MARKER"),
        box((90, 0, 0), (100, 100, 100), "MARKER"),
        box((0, 0, 0), (100, 10, 100), "MARKER"),
        box((0, 90, 0), (100, 100, 100), "MARKER"),
        box((0, 0, 0), (100, 100, 10), "MARKER"),
        box((0, 0, 90), (100, 100, 100), "MARKER"),
    )
    marker = entity(
        1,
        (
            ("classname", "func_group"),
            ("_tb_type", "_tb_group"),
            ("_tb_name", "cut_here"),
        ),
        shell,
    )
    point_inside = entity(
        2, (("classname", "info_target"), ("origin", "50 50 50"))
    )
    point_outside = entity(
        3, (("classname", "info_target"), ("origin", "150 150 150"))
    )
    brush_inside = entity(
        4,
        (("classname", "func_wall"),),
        (box((60, 60, 60), (70, 70, 70), "ENTITY_INSIDE"),),
    )
    return world + marker + point_inside + point_outside + brush_inside


class RemoveMapGroupVolumeTests(unittest.TestCase):
    def test_safe_mode_removes_contained_content_and_preserves_crossers(self):
        source = fixture_map()
        output, report = remover.remove_group_volume(
            source, source, "cut_here", mode="contained"
        )

        self.assertNotIn('"_tb_name" "cut_here"', output)
        self.assertNotIn("INSIDE ", output)
        self.assertNotIn("ENTITY_INSIDE ", output)
        self.assertNotIn('"origin" "50 50 50"', output)
        self.assertIn("OUTSIDE ", output)
        self.assertIn("CROSSING ", output)
        self.assertIn('"origin" "150 150 150"', output)
        self.assertEqual(report.group_brushes, 6)
        self.assertEqual(report.contained_brushes, 2)
        self.assertEqual(report.removed_entities, 3)
        self.assertEqual(report.crossing_brushes, 1)
        remover.parse_map(output)

    def test_intersecting_mode_removes_boundary_crossers(self):
        source = fixture_map()
        output, report = remover.remove_group_volume(
            source, source, "cut_here", mode="intersecting"
        )

        self.assertNotIn("CROSSING ", output)
        self.assertEqual(report.crossing_brushes, 0)
        self.assertEqual(report.contained_brushes, 3)

    def test_group_source_may_be_separate(self):
        source = fixture_map()
        target = source.replace(
            entity(
                1,
                (
                    ("classname", "func_group"),
                    ("_tb_type", "_tb_group"),
                    ("_tb_name", "cut_here"),
                ),
                (
                    box((0, 0, 0), (10, 100, 100), "MARKER"),
                    box((90, 0, 0), (100, 100, 100), "MARKER"),
                    box((0, 0, 0), (100, 10, 100), "MARKER"),
                    box((0, 90, 0), (100, 100, 100), "MARKER"),
                    box((0, 0, 0), (100, 100, 10), "MARKER"),
                    box((0, 0, 90), (100, 100, 100), "MARKER"),
                ),
            ),
            "",
        )
        output, report = remover.remove_group_volume(
            target, source, "cut_here", mode="contained"
        )

        self.assertEqual(report.group_brushes, 0)
        self.assertNotIn("INSIDE ", output)
        self.assertIn("OUTSIDE ", output)

    def test_rejects_non_box_group(self):
        source = entity(
            0, (("classname", "worldspawn"),), (box((0, 0, 0), (1, 1, 1)),)
        ) + entity(
            1,
            (("classname", "func_group"), ("_tb_name", "bad")),
            (box((0, 0, 0), (10, 10, 10)),),
        )
        with self.assertRaisesRegex(remover.MapError, "exactly six"):
            remover.remove_group_volume(source, source, "bad")


if __name__ == "__main__":
    unittest.main()
