#!/usr/bin/env python3
"""Remove map content inside a box-shaped TrenchBroom group.

The named group is expected to be a func_group containing the six slab
brushes of a hollow box.  Its inner bounds become the removal volume.

With neither --output nor --in-place the command is a read-only dry run.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import shutil
import sys
import tempfile
from collections import Counter
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Iterable, Sequence


NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
POINT_RE = re.compile(
    rf"\(\s*({NUMBER})\s+({NUMBER})\s+({NUMBER})\s*\)"
)
KEYVALUE_RE = re.compile(r'^"([^"]+)"\s+"([^"]*)"')
ENTITY_COMMENT_RE = re.compile(r"^//\s*entity\s+\d+\s*$", re.IGNORECASE)
BRUSH_COMMENT_RE = re.compile(r"^//\s*brush\s+\d+\s*$", re.IGNORECASE)


class MapError(ValueError):
    """A malformed map or unsuitable marker group."""


@dataclass(frozen=True)
class Vec3:
    x: float
    y: float
    z: float

    def __add__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> "Vec3":
        return Vec3(self.x * scalar, self.y * scalar, self.z * scalar)

    def __truediv__(self, scalar: float) -> "Vec3":
        return Vec3(self.x / scalar, self.y / scalar, self.z / scalar)

    def component(self, axis: int) -> float:
        return (self.x, self.y, self.z)[axis]


@dataclass(frozen=True)
class Plane:
    normal: Vec3
    distance: float


@dataclass(frozen=True)
class Bounds:
    minimum: Vec3
    maximum: Vec3
    center: Vec3

    def size(self, axis: int) -> float:
        return self.maximum.component(axis) - self.minimum.component(axis)


@dataclass
class Brush:
    start: int
    end: int
    comment_start: int


@dataclass
class Entity:
    start: int
    end: int
    comment_start: int
    keyvalues: dict[str, str] = field(default_factory=dict)
    brushes: list[Brush] = field(default_factory=list)

    @property
    def classname(self) -> str:
        return self.keyvalues.get("classname", "<unknown>")


@dataclass
class MapDocument:
    lines: list[str]
    entities: list[Entity]


@dataclass(frozen=True)
class GroupVolume:
    outer: Bounds
    inner: Bounds
    shell_bounds: tuple[Bounds, ...]
    group_entity: Entity


@dataclass
class RemovalReport:
    group_name: str
    mode: str
    volume: GroupVolume
    group_brushes: int = 0
    contained_brushes: int = 0
    removed_entities: int = 0
    crossing_brushes: int = 0
    invalid_brushes: int = 0
    class_counts: Counter[str] = field(default_factory=Counter)
    crossing_details: list[str] = field(default_factory=list)

    @property
    def removed_brushes(self) -> int:
        return self.group_brushes + self.contained_brushes


def dot(a: Vec3, b: Vec3) -> float:
    return a.x * b.x + a.y * b.y + a.z * b.z


def cross(a: Vec3, b: Vec3) -> Vec3:
    return Vec3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    )


def length(v: Vec3) -> float:
    return math.sqrt(dot(v, v))


def _comment_start(lines: Sequence[str], start: int, pattern: re.Pattern[str]) -> int:
    if start > 0 and pattern.match(lines[start - 1].strip()):
        return start - 1
    return start


def parse_map(text: str) -> MapDocument:
    """Parse entity/brush spans and entity keyvalues from a Valve 220 map."""

    lines = text.splitlines(keepends=True)
    entities: list[Entity] = []
    depth = 0
    entity: Entity | None = None
    brush: Brush | None = None

    for line_number, raw in enumerate(lines):
        stripped = raw.strip()
        if stripped == "{":
            if depth == 0:
                entity = Entity(
                    start=line_number,
                    end=-1,
                    comment_start=_comment_start(
                        lines, line_number, ENTITY_COMMENT_RE
                    ),
                )
            elif depth == 1:
                if entity is None:
                    raise MapError(f"line {line_number + 1}: brush outside entity")
                brush = Brush(
                    start=line_number,
                    end=-1,
                    comment_start=_comment_start(
                        lines, line_number, BRUSH_COMMENT_RE
                    ),
                )
            depth += 1
            continue

        if stripped == "}":
            if depth <= 0:
                raise MapError(f"line {line_number + 1}: unmatched closing brace")
            if depth == 2:
                if entity is None or brush is None:
                    raise MapError(f"line {line_number + 1}: malformed brush")
                brush.end = line_number
                entity.brushes.append(brush)
                brush = None
            elif depth == 1:
                if entity is None:
                    raise MapError(f"line {line_number + 1}: malformed entity")
                entity.end = line_number
                entities.append(entity)
                entity = None
            depth -= 1
            continue

        if depth == 1 and entity is not None:
            match = KEYVALUE_RE.match(stripped)
            if match:
                entity.keyvalues[match.group(1)] = match.group(2)

    if depth != 0 or entity is not None or brush is not None:
        raise MapError("map ended with an unclosed entity or brush")
    if not entities:
        raise MapError("map contains no entities")
    return MapDocument(lines=lines, entities=entities)


def brush_planes(document: MapDocument, brush: Brush) -> list[Plane]:
    planes: list[Plane] = []
    for raw in document.lines[brush.start + 1 : brush.end]:
        if not raw.lstrip().startswith("("):
            continue
        matches = list(POINT_RE.finditer(raw))
        if len(matches) < 3:
            continue
        points = [
            Vec3(float(match.group(1)), float(match.group(2)), float(match.group(3)))
            for match in matches[:3]
        ]
        normal = cross(points[0] - points[1], points[2] - points[1])
        magnitude = length(normal)
        if magnitude < 1e-9:
            continue
        normal = normal / magnitude
        planes.append(Plane(normal=normal, distance=dot(normal, points[0])))
    return planes


def intersect_planes(a: Plane, b: Plane, c: Plane) -> Vec3 | None:
    bc = cross(b.normal, c.normal)
    determinant = dot(a.normal, bc)
    if abs(determinant) < 1e-8:
        return None
    ca = cross(c.normal, a.normal)
    ab = cross(a.normal, b.normal)
    return (
        bc * a.distance + ca * b.distance + ab * c.distance
    ) / determinant


def brush_bounds(
    document: MapDocument, brush: Brush, epsilon: float = 0.05
) -> Bounds | None:
    planes = brush_planes(document, brush)
    if len(planes) < 4:
        return None

    vertices: list[Vec3] = []
    for a in range(len(planes) - 2):
        for b in range(a + 1, len(planes) - 1):
            for c in range(b + 1, len(planes)):
                point = intersect_planes(planes[a], planes[b], planes[c])
                if point is None:
                    continue
                if any(
                    dot(plane.normal, point) - plane.distance > epsilon
                    for plane in planes
                ):
                    continue
                if not any(length(point - old) <= 1e-5 for old in vertices):
                    vertices.append(point)

    if not vertices:
        return None

    minimum = Vec3(
        min(point.x for point in vertices),
        min(point.y for point in vertices),
        min(point.z for point in vertices),
    )
    maximum = Vec3(
        max(point.x for point in vertices),
        max(point.y for point in vertices),
        max(point.z for point in vertices),
    )
    center = Vec3(
        sum(point.x for point in vertices) / len(vertices),
        sum(point.y for point in vertices) / len(vertices),
        sum(point.z for point in vertices) / len(vertices),
    )
    return Bounds(minimum=minimum, maximum=maximum, center=center)


def _bounds_from_extents(minimum: Vec3, maximum: Vec3) -> Bounds:
    return Bounds(minimum, maximum, (minimum + maximum) / 2.0)


def find_group_volume(
    document: MapDocument, group_name: str, epsilon: float = 0.05
) -> GroupVolume:
    matches = [
        entity
        for entity in document.entities
        if entity.keyvalues.get("_tb_name") == group_name
    ]
    if not matches:
        raise MapError(f"TrenchBroom group {group_name!r} was not found")
    if len(matches) > 1:
        raise MapError(f"multiple TrenchBroom groups are named {group_name!r}")

    group = matches[0]
    if len(group.brushes) != 6:
        raise MapError(
            f"group {group_name!r} must contain exactly six box-shell brushes; "
            f"found {len(group.brushes)}"
        )

    shell: list[Bounds] = []
    for index, brush in enumerate(group.brushes):
        bounds = brush_bounds(document, brush, epsilon)
        if bounds is None:
            raise MapError(
                f"could not calculate bounds for group brush {index}"
            )
        shell.append(bounds)

    outer_min = Vec3(
        min(item.minimum.x for item in shell),
        min(item.minimum.y for item in shell),
        min(item.minimum.z for item in shell),
    )
    outer_max = Vec3(
        max(item.maximum.x for item in shell),
        max(item.maximum.y for item in shell),
        max(item.maximum.z for item in shell),
    )

    inner_min_values: list[float] = []
    inner_max_values: list[float] = []
    selected_slabs: set[int] = set()
    for axis in range(3):
        lower = [
            (index, item)
            for index, item in enumerate(shell)
            if abs(item.minimum.component(axis) - outer_min.component(axis))
            <= epsilon * 2
        ]
        upper = [
            (index, item)
            for index, item in enumerate(shell)
            if abs(item.maximum.component(axis) - outer_max.component(axis))
            <= epsilon * 2
        ]
        if not lower or not upper:
            raise MapError(
                f"group {group_name!r} does not provide both slabs on axis {axis}"
            )
        lower_index, lower_slab = min(lower, key=lambda pair: pair[1].size(axis))
        upper_index, upper_slab = min(upper, key=lambda pair: pair[1].size(axis))
        selected_slabs.update((lower_index, upper_index))
        inner_min_values.append(lower_slab.maximum.component(axis))
        inner_max_values.append(upper_slab.minimum.component(axis))

    if len(selected_slabs) != 6:
        raise MapError(
            f"group {group_name!r} is not a six-sided hollow box"
        )

    inner_min = Vec3(*inner_min_values)
    inner_max = Vec3(*inner_max_values)
    if any(
        inner_min.component(axis) >= inner_max.component(axis) - epsilon
        for axis in range(3)
    ):
        raise MapError(f"group {group_name!r} has no positive inner volume")

    return GroupVolume(
        outer=_bounds_from_extents(outer_min, outer_max),
        inner=_bounds_from_extents(inner_min, inner_max),
        shell_bounds=tuple(shell),
        group_entity=group,
    )


def point_inside(point: Vec3, bounds: Bounds, epsilon: float) -> bool:
    return all(
        bounds.minimum.component(axis) - epsilon
        <= point.component(axis)
        <= bounds.maximum.component(axis) + epsilon
        for axis in range(3)
    )


def bounds_contained(candidate: Bounds, container: Bounds, epsilon: float) -> bool:
    return all(
        candidate.minimum.component(axis) >= container.minimum.component(axis) - epsilon
        and candidate.maximum.component(axis)
        <= container.maximum.component(axis) + epsilon
        for axis in range(3)
    )


def bounds_intersect(a: Bounds, b: Bounds, epsilon: float) -> bool:
    return all(
        a.maximum.component(axis) >= b.minimum.component(axis) - epsilon
        and a.minimum.component(axis) <= b.maximum.component(axis) + epsilon
        for axis in range(3)
    )


def same_bounds(a: Bounds, b: Bounds, epsilon: float) -> bool:
    return all(
        abs(a.minimum.component(axis) - b.minimum.component(axis)) <= epsilon * 2
        and abs(a.maximum.component(axis) - b.maximum.component(axis)) <= epsilon * 2
        for axis in range(3)
    )


def parse_origin(value: str) -> Vec3 | None:
    parts = value.split()
    if len(parts) < 3:
        return None
    try:
        return Vec3(float(parts[0]), float(parts[1]), float(parts[2]))
    except ValueError:
        return None


def _format_vec(vector: Vec3) -> str:
    return f"({vector.x:.3f}, {vector.y:.3f}, {vector.z:.3f})"


def _mark_range(remove: set[int], start: int, end: int) -> None:
    remove.update(range(start, end + 1))


def remove_group_volume(
    map_text: str,
    group_text: str,
    group_name: str,
    *,
    mode: str = "contained",
    epsilon: float = 0.05,
) -> tuple[str, RemovalReport]:
    """Return a rewritten map and a detailed removal report."""

    if mode not in {"contained", "centroid", "intersecting"}:
        raise ValueError(f"unknown selection mode {mode!r}")

    group_document = parse_map(group_text)
    volume = find_group_volume(group_document, group_name, epsilon)
    document = parse_map(map_text)
    report = RemovalReport(group_name=group_name, mode=mode, volume=volume)
    remove_lines: set[int] = set()

    for entity_index, entity in enumerate(document.entities):
        named_group = entity.keyvalues.get("_tb_name") == group_name
        if named_group:
            _mark_range(remove_lines, entity.comment_start, entity.end)
            report.group_brushes += len(entity.brushes)
            report.removed_entities += 1
            report.class_counts[entity.classname] += len(entity.brushes)
            continue

        selected: list[Brush] = []
        for brush_index, brush in enumerate(entity.brushes):
            bounds = brush_bounds(document, brush, epsilon)
            if bounds is None:
                report.invalid_brushes += 1
                continue

            shell = any(
                same_bounds(bounds, candidate, epsilon)
                for candidate in volume.shell_bounds
            )
            intersects = bounds_intersect(bounds, volume.inner, epsilon)
            if mode == "contained":
                chosen = bounds_contained(bounds, volume.inner, epsilon)
            elif mode == "centroid":
                chosen = point_inside(bounds.center, volume.inner, epsilon)
            else:
                chosen = intersects
            chosen = shell or chosen

            if chosen:
                selected.append(brush)
                report.contained_brushes += 1
                report.class_counts[entity.classname] += 1
            elif intersects:
                report.crossing_brushes += 1
                report.crossing_details.append(
                    f"entity {entity_index} {entity.classname}, brush {brush_index}: "
                    f"{_format_vec(bounds.minimum)} - {_format_vec(bounds.maximum)}, "
                    f"center {_format_vec(bounds.center)}"
                )

        if entity.brushes and len(selected) == len(entity.brushes) and entity_index != 0:
            _mark_range(remove_lines, entity.comment_start, entity.end)
            report.removed_entities += 1
            continue

        for brush in selected:
            _mark_range(remove_lines, brush.comment_start, brush.end)

        if not entity.brushes and entity_index != 0:
            origin = parse_origin(entity.keyvalues.get("origin", ""))
            if origin is not None and point_inside(origin, volume.inner, epsilon):
                _mark_range(remove_lines, entity.comment_start, entity.end)
                report.removed_entities += 1
                report.class_counts[entity.classname] += 1

    output = "".join(
        line for index, line in enumerate(document.lines) if index not in remove_lines
    )
    return output, report


def _bounds_line(label: str, bounds: Bounds) -> str:
    return (
        f"{label}: {_format_vec(bounds.minimum)} - "
        f"{_format_vec(bounds.maximum)}"
    )


def report_lines(report: RemovalReport) -> Iterable[str]:
    yield _bounds_line("outer bounds", report.volume.outer)
    yield _bounds_line("inner bounds", report.volume.inner)
    yield f"selection mode: {report.mode}"
    yield f"group shell brushes selected: {report.group_brushes}"
    yield f"other brushes selected: {report.contained_brushes}"
    yield f"total brushes selected: {report.removed_brushes}"
    yield f"entities selected: {report.removed_entities}"
    yield f"boundary-crossing brushes preserved: {report.crossing_brushes}"
    yield f"brushes with unreadable bounds preserved: {report.invalid_brushes}"
    for detail in report.crossing_details:
        yield f"  {detail}"
    for classname, count in sorted(report.class_counts.items()):
        yield f"  {classname}: {count}"


def _safe_group_stem(group_name: str) -> str:
    stem = re.sub(r"[^A-Za-z0-9_.-]+", "-", group_name).strip(".-")
    return stem or "group"


def _atomic_write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="") as stream:
            stream.write(text)
        os.replace(temporary_name, path)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map", type=Path, help="map to inspect or rewrite")
    parser.add_argument("group", help="value of the marker entity's _tb_name")
    parser.add_argument(
        "--group-source",
        type=Path,
        help="read the marker from another map, such as a TrenchBroom autosave",
    )
    destination = parser.add_mutually_exclusive_group()
    destination.add_argument("-o", "--output", type=Path, help="write a new map")
    destination.add_argument(
        "--in-place",
        action="store_true",
        help="replace the input map after creating a timestamped backup",
    )
    parser.add_argument(
        "--mode",
        choices=("contained", "centroid", "intersecting"),
        default="contained",
        help=(
            "brush selection rule: fully contained (safe default), centroid "
            "(legacy), or any bounding-box intersection"
        ),
    )
    parser.add_argument(
        "--epsilon",
        type=float,
        default=0.05,
        help="geometry comparison tolerance in map units (default: 0.05)",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="replace an existing --output file",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    if args.epsilon <= 0:
        parser.error("--epsilon must be positive")

    map_path: Path = args.map.resolve()
    group_path: Path = (
        args.group_source.resolve() if args.group_source else map_path
    )
    try:
        map_text = map_path.read_text(encoding="utf-8-sig")
        group_text = group_path.read_text(encoding="utf-8-sig")
        output_text, report = remove_group_volume(
            map_text,
            group_text,
            args.group,
            mode=args.mode,
            epsilon=args.epsilon,
        )
    except (OSError, UnicodeError, MapError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    for line in report_lines(report):
        print(line)

    if args.in_place:
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        backup = map_path.with_name(
            f"{map_path.name}.{timestamp}.before-remove-"
            f"{_safe_group_stem(args.group)}.bak"
        )
        try:
            shutil.copy2(map_path, backup)
            _atomic_write(map_path, output_text)
        except OSError as error:
            print(f"error: {error}", file=sys.stderr)
            return 2
        print(f"backup: {backup}")
        print(f"wrote: {map_path}")
    elif args.output:
        output_path: Path = args.output.resolve()
        if output_path == map_path:
            parser.error("use --in-place when the output is the input map")
        if output_path.exists() and not args.force:
            print(
                f"error: output exists: {output_path} (pass --force to replace it)",
                file=sys.stderr,
            )
            return 2
        try:
            _atomic_write(output_path, output_text)
        except OSError as error:
            print(f"error: {error}", file=sys.stderr)
            return 2
        print(f"wrote: {output_path}")
    else:
        print("dry run: no map was written")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
