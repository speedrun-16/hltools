#!/usr/bin/env python3
"""generates the feature map and wad fixture

the map exercises a rotating door with an origin brush, func_detail and
func_group merging, a sky brush, water, clip geometry, mixed clip and visible
textures, an aaatrigger, a bounding box brush, zhlt_usemodel, named and
switchable lights, and the vis leaf entities

pass the output directory as the first argument
"""

import os
import struct
import sys


def cube_sides(mins, maxs):
    x0, y0, z0 = mins
    x1, y1, z1 = maxs
    # (three plane points, texture axis kind) per face; the point pattern
    # matches the winding the compiler test maps use
    return [
        (((x0, y0, z0), (x0, y1, z0), (x0, y0, z1)), "x"),
        (((x1, y0, z0), (x1, y0, z1), (x1, y1, z0)), "x"),
        (((x0, y0, z0), (x0, y0, z1), (x1, y0, z0)), "y"),
        (((x0, y1, z0), (x1, y1, z0), (x0, y1, z1)), "y"),
        (((x0, y0, z0), (x1, y0, z0), (x0, y1, z0)), "z"),
        (((x0, y0, z1), (x0, y1, z1), (x1, y0, z1)), "z"),
    ]


AXES = {
    "x": "[ 0 1 0 0 ] [ 0 0 -1 0 ]",
    "y": "[ 1 0 0 0 ] [ 0 0 -1 0 ]",
    "z": "[ 1 0 0 0 ] [ 0 -1 0 0 ]",
}


def brush(mins, maxs, texture, face_textures=None):
    lines = ["{"]
    for i, (points, axis_kind) in enumerate(cube_sides(mins, maxs)):
        name = face_textures[i] if face_textures else texture
        pts = " ".join("( %g %g %g )" % p for p in points)
        lines.append("%s %s %s 0 1 1" % (pts, name, AXES[axis_kind]))
    lines.append("}")
    return "\n".join(lines)


def entity(keys, brushes=()):
    lines = ["{"]
    for key, value in keys:
        lines.append('"%s" "%s"' % (key, value))
    for b in brushes:
        lines.append(b)
    lines.append("}")
    return "\n".join(lines)


def build_map():
    parts = []

    # worldspawn: sealed room, interior x/y -256256, z 0256, walls 16 thick
    world_brushes = [
        brush((-272, -272, -16), (272, 272, 0), "STONE"),      # floor
        brush((-272, -272, 256), (272, 272, 272), "SKY"),      # sky ceiling
        brush((-272, -272, 0), (-256, 272, 256), "STONE"),     # west wall
        brush((256, -272, 0), (272, 272, 256), "STONE"),       # east wall
        brush((-272, -272, 0), (256, -256, 256), "STONE"),     # south wall
        brush((-272, 256, 0), (272, 272, 256), "STONE"),       # north wall
        brush((-8, -256, 0), (8, 64, 256), "STONE"),           # divider, gap at y 64256
        brush((-192, -192, 0), (-64, -64, 64), "!WATER"),      # water pool on the floor
        brush((-224, 192, 0), (-208, 224, 64), "CLIP"),        # pure clip brush
        # mixed clip and visible texture: splits into a clip part and a solid part
        brush((-224, 128, 0), (-208, 160, 64), "CLIP",
              ["CLIP", "CLIP", "CLIP", "CLIP", "CLIP", "STONE"]),
    ]
    parts.append(entity([
        ("mapversion", "220"),
        ("wad", "features.wad"),
        ("classname", "worldspawn"),
    ], world_brushes))

    # rotating door in the divider gap, with an origin brush at the hinge
    parts.append(entity([
        ("classname", "func_door_rotating"),
        ("targetname", "door1"),
        ("speed", "100"),
    ], [
        brush((-8, 64, 0), (8, 256, 256), "STONE"),
        brush((-8, 56, 120), (8, 72, 136), "ORIGIN"),
    ]))

    # merges into worldspawn at parse time
    parts.append(entity([
        ("classname", "func_detail"),
        ("zhlt_detaillevel", "1"),
    ], [brush((96, 96, 0), (160, 160, 128), "STONE")]))

    parts.append(entity([
        ("classname", "func_group"),
    ], [brush((96, -160, 0), (160, -96, 128), "STONE")]))

    # switchable texlight: style -1 allocates an engine style number
    parts.append(entity([
        ("classname", "func_wall"),
        ("targetname", "wall1"),
        ("style", "-1"),
    ], [brush((-160, 96, 0), (-96, 160, 64), "STONE")]))

    # borrows wall1's model; its own geometry is just the origin brush
    parts.append(entity([
        ("classname", "func_illusionary"),
        ("zhlt_usemodel", "wall1"),
    ], [brush((192, -208, 24), (208, -192, 40), "ORIGIN")]))

    # boundingbox brush overrides the entity bounds
    parts.append(entity([
        ("classname", "func_wall_toggle"),
        ("targetname", "bbox1"),
    ], [
        brush((192, 192, 0), (224, 224, 32), "STONE"),
        brush((176, 176, 0), (240, 240, 48), "BOUNDINGBOX"),
    ]))

    parts.append(entity([
        ("classname", "trigger_once"),
        ("target", "door1"),
    ], [brush((32, -64, 0), (64, -32, 64), "AAATRIGGER")]))

    # named light: gets a switched style the plain one stays style 0
    parts.append(entity([
        ("classname", "light"),
        ("targetname", "light1"),
        ("origin", "0 -128 200"),
        ("_light", "255 255 128 200"),
    ]))
    parts.append(entity([
        ("classname", "light"),
        ("origin", "128 128 200"),
        ("_light", "255 255 255 150"),
    ]))
    parts.append(entity([
        ("classname", "light_environment"),
        ("origin", "0 0 240"),
        ("_light", "255 255 255 100"),
        ("angles", "0 0 0"),
        ("pitch", "-60"),
    ]))

    # vis leaf entities
    parts.append(entity([
        ("classname", "info_overview_point"),
        ("origin", "0 160 128"),
    ]))
    parts.append(entity([
        ("classname", "info_portal"),
        ("origin", "-128 -128 100"),
        ("target", "leafA"),
        ("neighbor", "1"),
    ]))
    parts.append(entity([
        ("classname", "info_leaf"),
        ("targetname", "leafA"),
        ("origin", "32 -128 64"),
    ]))

    parts.append(entity([
        ("classname", "info_player_start"),
        ("origin", "-32 192 36"),
        ("angles", "0 0 0"),
    ]))

    return "\n".join(parts) + "\n"


def miptex_bytes(name):
    # a 16 by 16 texture with all four mip levels and a full palette of 256 colors
    # palette (hlrad loads the pixels and requires the palette block)
    data = bytearray(40)
    data[: len(name)] = name.encode("ascii")
    struct.pack_into("<6i", data, 16, 16, 16, 40, 296, 360, 376)
    data.extend(bytes([7]) * (380 - 40))  # the four mip levels
    data.extend(struct.pack("<h", 256))  # palette colour count
    data.extend(bytes(i % 256 for i in range(768)))  # 256 rgb entries
    data.extend(bytes(2))  # trailing pad
    return bytes(data)


def build_wad(textures):
    lumps = b""
    infos = b""
    offset = 12
    for name in textures:
        mip = miptex_bytes(name)
        infos += struct.pack("<3i4b", offset, len(mip), len(mip), 0x43, 0, 0, 0)
        infos += name.encode("ascii").ljust(16, b"\0")
        lumps += mip
        offset += len(mip)
    header = b"WAD3" + struct.pack("<2i", len(textures), offset)
    return header + lumps + infos


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else "."
    os.makedirs(out_dir, exist_ok=True)

    with open(os.path.join(out_dir, "features.map"), "w", newline="\n") as f:
        f.write(build_map())
    with open(os.path.join(out_dir, "features.wad"), "wb") as f:
        f.write(build_wad(["STONE", "SKY", "!WATER", "ORIGIN", "BOUNDINGBOX", "NULL", "SKIP"]))
    print("wrote features.map and features.wad to", out_dir)


if __name__ == "__main__":
    main()
