#!/usr/bin/env python3
"""generates the lighting showcase fixtures for -ao and -lightradius

two maps, each built so that one effect is the only thing changing between an
off and an on compile, and both framed for a screenshot from a fixed vantage:

  ao_room     concave geometry lit almost entirely by bounce and -ambient, the
              two terms ambient occlusion is allowed to darken

  soft_room   six sealed cells: four plates at rising heights over their own
              light, a pole, and a spot light with occluders at two distances

neither map carries "_radius" or an "-ao" flavoured key, so the off and on
builds compile the same source and differ only by the command line

EVERY brush coordinate is a multiple of GRID (16), which is checked on the way
out. that is what makes the floor grid meaningful: a texture line, a luxel
centre and a brush edge all land on the same world coordinate, so a shadow edge
is read against the grid instead of floating between two rulers

pass the output directory as the first argument
"""

import os
import struct
import sys

GRID = 16          # the world grid everything snaps to
LUXEL = 16         # default texture step: one luxel per 16 world units

# ===== textures =====

# a 240 step grey ramp, so a texture is authored in plain luminance, plus a few
# accents for the surfaces that should show coloured bounce
GREY_LEVELS = 240
ACCENTS = [
    (196, 96, 48),   # 240 warm orange
    (56, 96, 176),   # 241 cool blue
    (96, 160, 88),   # 242 green
    (168, 56, 56),   # 243 red
]

SIZE = 256         # texture edge; at scale 1 one tile covers 256 world units


def palette():
    entries = [(round(i * 255 / (GREY_LEVELS - 1)),) * 3 for i in range(GREY_LEVELS)]
    entries.extend(ACCENTS)
    while len(entries) < 256:
        entries.append((0, 0, 0))
    return entries


def grey(value):
    """the palette index whose grey ramp entry is closest to value (0..255)"""
    return max(0, min(GREY_LEVELS - 1, round(value * (GREY_LEVELS - 1) / 255)))


def tex_grid(base, minor, major, axis):
    """a three tier ruler, world aligned

    at texture scale 1 with a zero texture offset, texture column c sits at
    world coordinate c, so a line at c % 16 == 0 lands exactly on the 16 unit
    world grid - and therefore on a luxel centre and on every brush edge in
    these maps. the tiers let a distance be read without counting every line:
    16 (one luxel), 64 (four luxels, the default -chop), 256

    only floors carry this. walls and ceilings take the 64 unit tier instead
    (tex_wall), which is dense enough to measure against and quiet enough not to
    compete with the floor, where the shadows actually land
    """
    b, mn, mj, ax = grey(base), grey(minor), grey(major), grey(axis)
    pixels = []
    for y in range(SIZE):
        for x in range(SIZE):
            if x % 256 == 0 or y % 256 == 0:
                pixels.append(ax)
            elif x % 64 == 0 or y % 64 == 0:
                pixels.append(mj)
            elif x % 16 == 0 or y % 16 == 0:
                pixels.append(mn)
            else:
                pixels.append(b)
    return pixels


def tex_wall(base, line):
    """walls carry the ruler at the 64 unit tier only, so they read as a surface
    and stay measurable without competing with the floor"""
    b, l = grey(base), grey(line)
    return [
        l if (x % 64 == 0 or y % 64 == 0) else b
        for y in range(SIZE)
        for x in range(SIZE)
    ]


def tex_flat(value):
    return [grey(value)] * (SIZE * SIZE)


def tex_solid(index):
    return [index] * (SIZE * SIZE)


def downsample(pixels, size):
    """point sampling every other texel

    a box filter would average the grid lines away by mip 2; keeping the line
    sharp matters more here than aliasing does, because the grid is the
    measuring instrument
    """
    half = size // 2
    return [pixels[(y * 2) * size + (x * 2)] for y in range(half) for x in range(half)]


def miptex_bytes(name, pixels):
    assert len(name) < 16
    mips = [pixels]
    for level in range(3):
        mips.append(downsample(mips[-1], SIZE >> level))

    header = bytearray(40)
    header[: len(name)] = name.encode("ascii")
    offsets = []
    cursor = 40
    for mip in mips:
        offsets.append(cursor)
        cursor += len(mip)
    struct.pack_into("<6i", header, 16, SIZE, SIZE, *offsets)

    out = bytes(header)
    for mip in mips:
        out += bytes(mip)
    out += struct.pack("<h", 256)
    for r, g, b in palette():
        out += bytes((r, g, b))
    out += bytes(2)
    return out


TEXTURES = [
    # the ao room is deliberately bright: ambient occlusion only has bounced
    # light to darken if the room actually bounces
    ("FLOOR", tex_grid(176, 148, 120, 88)),
    ("WALL", tex_wall(188, 164)),
    ("CEIL", tex_wall(124, 92)),
    ("TRIM", tex_wall(124, 104)),
    # the soft shadow room wants the opposite: dark, barely bouncing surfaces,
    # so a penumbra is read against black rather than against wall fill
    ("SFLOOR", tex_grid(92, 74, 58, 42)),
    ("SWALL", tex_wall(76, 56)),
    ("SCEIL", tex_wall(76, 56)),
    ("ACCENT", tex_solid(240)),
    ("SKY", tex_flat(255)),
]


def build_wad(textures):
    lumps = b""
    infos = b""
    offset = 12
    for name, pixels in textures:
        mip = miptex_bytes(name, pixels)
        infos += struct.pack("<3i4b", offset, len(mip), len(mip), 0x43, 0, 0, 0)
        infos += name.encode("ascii").ljust(16, b"\0")
        lumps += mip
        offset += len(mip)
    header = b"WAD3" + struct.pack("<2i", len(textures), offset)
    return header + lumps + infos


# ===== map geometry =====

AXES = {
    "x": "[ 0 1 0 0 ] [ 0 0 -1 0 ]",
    "y": "[ 1 0 0 0 ] [ 0 0 -1 0 ]",
    "z": "[ 1 0 0 0 ] [ 0 -1 0 0 ]",
}


def cube_sides(mins, maxs):
    x0, y0, z0 = mins
    x1, y1, z1 = maxs
    return [
        (((x0, y0, z0), (x0, y1, z0), (x0, y0, z1)), "x"),
        (((x1, y0, z0), (x1, y0, z1), (x1, y1, z0)), "x"),
        (((x0, y0, z0), (x0, y0, z1), (x1, y0, z0)), "y"),
        (((x0, y1, z0), (x1, y1, z0), (x0, y1, z1)), "y"),
        (((x0, y0, z0), (x1, y0, z0), (x0, y1, z0)), "z"),
        (((x0, y0, z1), (x0, y1, z1), (x1, y0, z1)), "z"),
    ]


def brush(mins, maxs, texture, face_textures=None):
    for value in tuple(mins) + tuple(maxs):
        if value % GRID:
            raise ValueError("off grid brush coordinate %r in %r %r"
                             % (value, mins, maxs))
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


def shell(x0, y0, x1, y1, z0, z1, thickness=GRID,
          ceiling="CEIL", floor="FLOOR", wall="WALL"):
    """a sealed box room; the arguments give the interior extent"""
    t = thickness
    return [
        brush((x0 - t, y0 - t, z0 - t), (x1 + t, y1 + t, z0), floor),
        brush((x0 - t, y0 - t, z1), (x1 + t, y1 + t, z1 + t), ceiling),
        brush((x0 - t, y0 - t, z0), (x0, y1 + t, z1), wall),
        brush((x1, y0 - t, z0), (x1 + t, y1 + t, z1), wall),
        brush((x0 - t, y0 - t, z0), (x1, y0, z1), wall),
        brush((x0 - t, y1, z0), (x1 + t, y1 + t, z1), wall),
    ]


def light(origin, colour="255 250 240", brightness=300):
    return entity([
        ("classname", "light"),
        ("origin", "%g %g %g" % origin),
        ("_light", "%s %d" % (colour, brightness)),
    ])


def player_start(origin, yaw):
    """the screenshot vantage

    the engine drops the player to the floor and ignores a start pitch, so the
    position frames the geometry from standing eye height and the yaw aims it
    """
    return entity([
        ("classname", "info_player_start"),
        ("origin", "%g %g %g" % origin),
        ("angles", "0 %g 0" % yaw),
    ])


# ===== ao_room =====

AO_HALF = 768      # interior 1536 units square
AO_TOP = 640


def build_ao_room():
    """
    concave geometry under bounce dominated lighting

    the lights sit high and in the open, so direct light is broad and nearly
    shadowless; what fills the corners is bounce off the bright walls plus
    whatever -ambient the compile adds. those are exactly the two terms -ao
    modulates, so switching it on changes the cavities and nothing else

    the cavity widths straddle the 64 unit default -chop on purpose: the slot
    is 32 units and radiosity cannot resolve it at all, the niches are 128 and
    it resolves them coarsely, the room corners are larger still and it already
    has those right
    """
    h = AO_HALF
    world = shell(-h, -h, h, h, 0, AO_TOP)

    # columns down the west wall: each base is a three plane corner, the case
    # radiosity cannot resolve below -chop
    for cy in (-512, -256, 0, 256):
        world.append(brush((-h, cy - 32, 0), (-h + 128, cy + 32, AO_TOP), "TRIM"))

    # stepped stack: three concave ledges of decreasing size, so one object
    # shows occlusion at three cavity widths at once
    world.append(brush((-448, -448, 0), (-64, -64, 64), "WALL"))
    world.append(brush((-384, -384, 64), (-128, -128, 128), "WALL"))
    world.append(brush((-320, -320, 128), (-192, -192, 192), "WALL"))

    # niches: pilasters standing 128 proud of the east wall leave six recesses
    # 128 wide, with a base and a header closing the ends
    world.append(brush((h - 128, -h, 0), (h, h, 64), "TRIM"))
    world.append(brush((h - 128, -h, 512), (h, h, AO_TOP), "WALL"))
    for ny in range(-768, 768, 256):
        world.append(brush((h - 128, ny, 0), (h, ny + 128, AO_TOP), "WALL"))

    # the slot: two blocks 32 units apart, half the default -chop, so nothing
    # in the radiosity solve can darken between them
    world.append(brush((256, 256, 0), (448, 448, 384), "WALL"))
    world.append(brush((256, 480, 0), (448, 672, 384), "WALL"))

    # overhanging shelf on the north wall: a hard contact shadow underneath
    world.append(brush((-640, h - 128, 320), (-128, h, 352), "TRIM"))

    # corner cluster in the accent colour, so its occlusion picks up the albedo
    # aware multibounce tint instead of going flat grey
    world.append(brush((384, 592, 0), (624, h, 128), "ACCENT"))
    world.append(brush((448, 656, 128), (624, h, 224), "WALL"))

    parts = []

    for lx in (-384, 384):
        for ly in (-384, 384):
            parts.append(light((lx, ly, 384), brightness=420))

    # viewing balcony along the south wall, with a stair down the centreline.
    # occlusion collects in floor cavities and under ledges, so it is read
    # looking down into the room rather than across it at eye level
    world.append(brush((-h, -h, 0), (640, -640, 256), "TRIM"))
    world.append(brush((-192, -640, 0), (192, -576, 192), "TRIM"))
    world.append(brush((-192, -576, 0), (192, -512, 128), "TRIM"))
    world.append(brush((-192, -512, 0), (192, -448, 64), "TRIM"))

    # the vantage: on the balcony, looking north down the room centreline. the
    # stack is ahead, the columns run away down the left wall, the niches down
    # the right, the slot sits right of centre and the shelf closes the far left
    # corner - every cavity in the map in one frame, all seen from above
    parts.append(player_start((0, -704, 292), 90))

    parts.insert(0, entity([
        ("mapversion", "220"),
        ("wad", "lighting.wad"),
        ("classname", "worldspawn"),
    ], world))

    return "\n".join(parts) + "\n"


# ===== soft_room =====

CELL_HALF = 384
CELL_Y0, CELL_Y1 = -512, 512
SOFT_TOP = 640
LIGHT_Z = 512            # H in the penumbra formula
PLATE_HALF = 48          # a
BALCONY_TOP = 192

PLATE_HEIGHTS = (64, 192, 320, 416)
CELLS = {"pole": -1920, 64: -1152, 192: -384, 320: 384, 416: 1152, "spot": 1920}
DIVIDERS = (-1536, -768, 0, 768, 1536)


def penumbra_table(radius=64):
    """the analytic prediction the rig is built to show

    for an emitter of radius r at height H over a floor and an occluder of half
    width a at height h, with k = H / (H - h):
        penumbra width   = 2 * r * (k - 1)
        umbra half width = a * k - r * (k - 1)
    so the umbra survives only while a * k > r * (k - 1)
    """
    rows = []
    for h in PLATE_HEIGHTS:
        k = LIGHT_Z / (LIGHT_Z - h)
        pen = 2 * radius * (k - 1)
        umbra = PLATE_HALF * k - radius * (k - 1)
        rows.append((h, k, pen, pen / LUXEL, umbra))
    return rows


def build_soft_room():
    """
    penumbra width against occluder height

    six cells in a row, each sealed from its neighbours by a full height
    divider whose only opening is a doorway at balcony level, so no cell's
    light reaches another and every shadow in view was cast by one source

    the four middle cells are the analytic rig: one 96 unit square plate at 64,
    192, 320 and 416 units above the floor, each under its own light at z 512.
    the umbra half width falls 46 -> 38 -> 21 -> gone while the penumbra widens
    18 -> 77 -> 213 -> 555 units, which is 1 -> 5 -> 13 -> 35 luxels

    every cell has a balcony along its south wall at z 192, reached by two
    steps, with the doorways running between the balconies. the shadows are on
    the floor, so they want to be looked down at rather than across
    """
    x0, x1 = CELLS["pole"] - CELL_HALF, CELLS["spot"] + CELL_HALF
    world = shell(x0, CELL_Y0, x1, CELL_Y1, 0, SOFT_TOP,
                  ceiling="SCEIL", floor="SFLOOR", wall="SWALL")

    for dx in DIVIDERS:
        # solid either side of a doorway that only opens at balcony level
        world.append(brush((dx - 16, CELL_Y0 - 16, BALCONY_TOP + 112),
                           (dx + 16, CELL_Y1 + 16, SOFT_TOP), "SWALL"))
        world.append(brush((dx - 16, -384, 0),
                           (dx + 16, CELL_Y1 + 16, SOFT_TOP), "SWALL"))
        world.append(brush((dx - 16, CELL_Y0 - 16, 0),
                           (dx + 16, CELL_Y0, SOFT_TOP), "SWALL"))
        world.append(brush((dx - 16, CELL_Y0, 0),
                           (dx + 16, -384, BALCONY_TOP), "SWALL"))

    # per cell: the viewing balcony and its two steps down to the floor
    for cx in CELLS.values():
        world.append(brush((cx - CELL_HALF, CELL_Y0, 0),
                           (cx + CELL_HALF, -384, BALCONY_TOP), "SWALL"))
        world.append(brush((cx - CELL_HALF, -384, 0),
                           (cx + CELL_HALF, -320, 128), "SWALL"))
        world.append(brush((cx - CELL_HALF, -320, 0),
                           (cx + CELL_HALF, -256, 64), "SWALL"))

    parts = []

    # the rig
    for h in PLATE_HEIGHTS:
        cx = CELLS[h]
        world.append(brush((cx - PLATE_HALF, -PLATE_HALF, h),
                           (cx + PLATE_HALF, PLATE_HALF, h + 16), "ACCENT"))
        parts.append(light((cx, 0, LIGHT_Z), brightness=640))

    # pole cell: the light is pushed into the south corner so the shadow is
    # thrown the length of the cell, sharp at the base and soft at the tip
    px = CELLS["pole"]
    world.append(brush((px - 16, -224, 0), (px + 16, -192, 384), "TRIM"))
    parts.append(light((px, -448, 608), colour="255 240 220", brightness=760))

    # spot cell: two identical bars, one 96 units off the wall they are thrown
    # against and one 480 units off, so one frame holds both ends of the ramp
    sx = CELLS["spot"]
    world.append(brush((sx - 160, 384, 64), (sx - 128, 416, 448), "TRIM"))
    world.append(brush((sx + 128, 0, 64), (sx + 160, 32, 448), "TRIM"))
    parts.append(entity([
        ("classname", "light_spot"),
        ("origin", "%d -256 512" % sx),
        ("_light", "255 250 240 1900"),
        ("target", "spot_wall"),
        ("_cone", "36"),
        ("_cone2", "44"),
    ]))
    parts.append(entity([
        ("classname", "info_target"),
        ("targetname", "spot_wall"),
        ("origin", "%d 512 256" % sx),
    ]))

    parts.insert(0, entity([
        ("mapversion", "220"),
        ("wad", "lighting.wad"),
        ("classname", "worldspawn"),
    ], world))

    # the vantage: on the balcony of the h 320 cell, where the change is
    # unmistakable and the shadow still fits the frame, looking north
    parts.append(player_start((CELLS[320], -448, BALCONY_TOP + 36), 90))

    return "\n".join(parts) + "\n"


# ===== output =====

README = '''# lighting showcase fixtures

generated by `tests/fixtures/generators/lighting.py` - regenerate, do not edit.

every brush coordinate is a multiple of 16, checked by the generator. that is
what makes the floor grid a usable ruler: a texture line, a luxel centre and a
brush edge all land on the same world coordinate. the grid has three tiers -
every 16 units (one luxel), every 64 (four luxels, the default `-chop`), and
every 256.

## ao_room

concave geometry lit almost entirely by bounce and `-ambient`, the two terms
`-ao` is allowed to darken. direct light already carries an exact shadow ray per
luxel, so `-aodirect` defaults to 0 and switching `-ao` on must not touch it.

the cavity widths straddle the 64 unit default `-chop` on purpose:

| feature | width | what radiosity can do with it |
|---|---|---|
| slot between the two east blocks | 32 u | nothing: half a patch |
| wall niches | 128 u | resolves it coarsely |
| column bases, stack ledges | corner | the large scale only |

    hltools compile -extra -ambient 0.18 0.18 0.18          ao_room_off.map
    hltools compile -extra -ambient 0.18 0.18 0.18 -ao      ao_room_on.map
    hltools compile -extra -ambient 0.18 0.18 0.18 -aodebug ao_room_debug.map

`-aodebug` bakes the measured occlusion straight into the lightmap: the view to
screenshot when the question is what the sampler saw, not what the room looks
like.

the spawn is on a balcony at z 256 along the south wall, looking north down the
centreline, with a stair down to the floor. the stack is ahead, the columns run
down the left wall, the niches down the right, the slot is right of centre, the
shelf closes the far left corner - every cavity in one frame, seen from above.

## soft_room

six sealed cells. the four middle ones are the analytic rig: a 96 unit square
plate (a = 48) at four heights, each under its own light at H = 512. for an
emitter of radius r, with k = H / (H - h):

    penumbra width = 2*r*(k-1)      umbra half width = a*k - r*(k-1)

which at r = 64 gives

PENUMBRA_TABLE

the umbra shrinks to nothing by the fourth cell while the penumbra grows past
the width of the plate itself. the outer cells are the qualitative pair: a pole
whose shadow hardens at its base and softens along its length, and a spot light
with two bars at different distances from the wall.

    hltools compile -extra                                  soft_room_off.map
    hltools compile -extra -lightradius 64  -softsamples 64 soft_room_on.map
    hltools compile -extra -lightradius 128 -softsamples 96 soft_room_wide.map

each cell has a balcony at z 192 along its south wall, two steps down to the
floor, and doorways running between the balconies. floor shadows want to be
looked down at. the spawn is on the h 320 balcony.

lighting is kept clear of the 255 clip on purpose: a clipped lit floor flattens
the top of every penumbra ramp and the gradient stops being measurable.

## comparing without the game

    hltools lightmap <map>.bsp <map>_lm.bmp -force

exports every lit face to one deterministic atlas, so an off/on pair can be
diffed pixel by pixel rather than eyeballed.

## in game

set `gl_overbright 0` and leave gamma and brightness alone across a pair. the
contact shadow case differs by only a few units per channel, which is less than
one notch of the brightness slider.
'''


def readme_text():
    rows = ["    h     k       penumbra                umbra half width",
            "    " + "-" * 58]
    for h, k, pen, luxels, umbra in penumbra_table():
        rows.append("    %3d   %5.3f   %6.1f u (%4.1f luxels)   %s"
                    % (h, k, pen, luxels,
                       "%.1f u" % umbra if umbra > 0 else "none"))
    return README.replace("PENUMBRA_TABLE", "\n".join(rows))


MAPS = {
    "ao_room": build_ao_room,
    "soft_room": build_soft_room,
}


def main():
    if len(sys.argv) != 2:
        sys.exit("usage: lighting.py OUTPUT_DIRECTORY")
    out_dir = sys.argv[1]
    os.makedirs(out_dir, exist_ok=True)

    for name, build in MAPS.items():
        path = os.path.join(out_dir, name + ".map")
        with open(path, "w", newline="\n") as f:
            f.write(build())
        print("wrote", path)

    with open(os.path.join(out_dir, "README.md"), "w", newline="\n") as f:
        f.write(readme_text())

    with open(os.path.join(out_dir, "lighting.wad"), "wb") as f:
        f.write(build_wad(TEXTURES))
    print("wrote lighting.wad and README.md to", out_dir)


if __name__ == "__main__":
    main()
