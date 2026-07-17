#!/usr/bin/env python3
"""compile_diff.py - byte-compare every lump of two GoldSrc (BSP v30) files.

The regression harness for the compiler rewrite: it proves a refactored tool
produces output byte-identical to the reference build. It compares all 15 lumps
of two .bsp files and reports, per lump, whether they match. When the lighting
lump differs it drops into the same per-face lightmap analysis as lightmap_diff.py
so you can see how far off the lighting is.

Usage:
    python compile_diff.py reference.bsp candidate.bsp [--lightmap-detail] [--quiet]

Exit code: 0 if every lump is byte-identical, 1 if any lump differs,
2 on a structural mismatch (bad version, unreadable header).
"""

import argparse
import struct
import sys
import zlib
from pathlib import Path

import numpy as np

BSP_VERSION = 30

# lump index -> name in storage order
LUMP_NAMES = [
    "entities",
    "planes",
    "textures",
    "vertexes",
    "visibility",
    "nodes",
    "texinfo",
    "faces",
    "lighting",
    "clipnodes",
    "leafs",
    "marksurfaces",
    "edges",
    "surfedges",
    "models",
]
NUM_LUMPS = len(LUMP_NAMES)
LUMP_ENTITIES = 0
LUMP_VERTEXES = 3
LUMP_LIGHTING = 8
LUMP_TEXINFO = 6
LUMP_FACES = 7
LUMP_EDGES = 12
LUMP_SURFEDGES = 13

TEXTURE_STEP = 16.0


# ============================================================================
# bsp container
# ============================================================================

class Bsp:
    def __init__(self, path):
        self.path = Path(path)
        data = self.path.read_bytes()
        if len(data) < 4 + 8 * NUM_LUMPS:
            raise ValueError(f"{path}: file too small to be a BSP ({len(data)} bytes)")
        version = struct.unpack_from("<i", data, 0)[0]
        if version != BSP_VERSION:
            raise ValueError(f"{path}: unsupported BSP version {version}")
        self.lumps = []
        for i in range(NUM_LUMPS):
            ofs, ln = struct.unpack_from("<ii", data, 4 + 8 * i)
            if ofs < 0 or ln < 0 or ofs + ln > len(data):
                raise ValueError(f"{path}: lump {i} ({LUMP_NAMES[i]}) out of range")
            self.lumps.append(data[ofs:ofs + ln])

    # parsed views used only for detailed lighting comparisons

    def _parse_geometry(self):
        self.vertexes = np.frombuffer(self.lumps[LUMP_VERTEXES], dtype="<f4").reshape(-1, 3)
        self.edges = np.frombuffer(self.lumps[LUMP_EDGES], dtype="<u2").reshape(-1, 2)
        self.surfedges = np.frombuffer(self.lumps[LUMP_SURFEDGES], dtype="<i4")
        ti = np.frombuffer(self.lumps[LUMP_TEXINFO], dtype="<f4").reshape(-1, 10)
        self.texinfo_vecs = ti[:, :8].reshape(-1, 2, 4)
        fdata = self.lumps[LUMP_FACES]
        self.numfaces = len(fdata) // 20
        self.faces = []
        for i in range(self.numfaces):
            firstedge, numedges, texinfo = struct.unpack_from("<xxxxiHh", fdata, i * 20)
            styles = struct.unpack_from("<4B", fdata, i * 20 + 12)
            lightofs = struct.unpack_from("<i", fdata, i * 20 + 16)[0]
            self.faces.append((firstedge, numedges, texinfo, styles, lightofs))
        self.lightdata = np.frombuffer(self.lumps[LUMP_LIGHTING], dtype=np.uint8)

    def face_extents(self, facenum):
        """replicates GetFaceExtents: float32 min/max of double-summed dot products"""
        firstedge, numedges, texinfo, _styles, _lightofs = self.faces[facenum]
        se = self.surfedges[firstedge:firstedge + numedges]
        vidx = np.where(se >= 0, self.edges[np.abs(se), 0], self.edges[np.abs(se), 1])
        pts = self.vertexes[vidx].astype(np.float64)
        vecs = self.texinfo_vecs[texinfo].astype(np.float64)
        vals = ((pts @ vecs[:, :3].T) + vecs[:, 3]).astype(np.float32)
        mins = np.minimum(np.float32(999999), vals.min(axis=0))
        maxs = np.maximum(np.float32(-99999), vals.max(axis=0))
        bmins = np.floor(mins / TEXTURE_STEP).astype(int)
        bmaxs = np.ceil(maxs / TEXTURE_STEP).astype(int)
        return int(bmaxs[0] - bmins[0]) + 1, int(bmaxs[1] - bmins[1]) + 1

    def face_lightmaps(self, facenum):
        _fe, _ne, _texinfo, styles, lightofs = self.faces[facenum]
        if lightofs < 0:
            return None
        nstyles = sum(1 for s in styles if s != 255)
        if nstyles == 0:
            return None
        w, h = self.face_extents(facenum)
        size = w * h * 3
        out = {}
        for k in range(nstyles):
            ofs = lightofs + k * size
            block = self.lightdata[ofs:ofs + size]
            if len(block) < size:
                block = np.pad(block, (0, size - len(block)))
            out[styles[k]] = block.reshape(h, w, 3)
        return w, h, out


# ============================================================================
# helpers
# ============================================================================

def human(n):
    if n >= 1024 * 1024:
        return f"{n / (1024 * 1024):.2f} MB"
    if n >= 1024:
        return f"{n / 1024:.1f} KB"
    return f"{n} B"


def first_diff_offset(a, b):
    """byte offset of the first difference, or -1 if the common prefix matches"""
    n = min(len(a), len(b))
    aa = np.frombuffer(a[:n], dtype=np.uint8)
    bb = np.frombuffer(b[:n], dtype=np.uint8)
    ne = np.nonzero(aa != bb)[0]
    return int(ne[0]) if len(ne) else -1


def count_diff_bytes(a, b):
    n = min(len(a), len(b))
    aa = np.frombuffer(a[:n], dtype=np.uint8)
    bb = np.frombuffer(b[:n], dtype=np.uint8)
    return int(np.count_nonzero(aa != bb)) + abs(len(a) - len(b))


def entities_first_diff_line(a, b):
    """for the text entities lump, the 1-based line number where they first diverge"""
    la = a.split(b"\n")
    lb = b.split(b"\n")
    for i in range(min(len(la), len(lb))):
        if la[i] != lb[i]:
            return i + 1, la[i][:80], lb[i][:80]
    if len(la) != len(lb):
        return min(len(la), len(lb)) + 1, b"<end>", b"<end>"
    return None


# ============================================================================
# lighting details when the lighting lump differs
# ============================================================================

def lighting_detail(a, b, p):
    a._parse_geometry()
    b._parse_geometry()
    if a.numfaces != b.numfaces:
        p(f"    face count differs ({a.numfaces} vs {b.numfaces}); skipping per-face detail")
        return
    n_diff = 0
    total_luxels = 0
    diff_luxels = 0
    global_max = 0
    sum_abs = 0.0
    worst = []
    for i in range(a.numfaces):
        la = a.face_lightmaps(i)
        lb = b.face_lightmaps(i)
        if (la is None) != (lb is None):
            n_diff += 1
            worst.append((999, i))
            continue
        if la is None:
            continue
        wa, ha, mapsa = la
        wb, hb, mapsb = lb
        if (wa, ha) != (wb, hb) or set(mapsa) != set(mapsb):
            n_diff += 1
            worst.append((999, i))
            continue
        total_luxels += wa * ha
        face_max = 0
        for style, img_a in mapsa.items():
            img_b = mapsb[style]
            if np.array_equal(img_a, img_b):
                continue
            d = np.abs(img_a.astype(np.int16) - img_b.astype(np.int16))
            face_max = max(face_max, int(d.max()))
            sum_abs += float(d.sum())
            diff_luxels += int((d.max(axis=2) > 0).sum())
        if face_max > 0:
            n_diff += 1
            global_max = max(global_max, face_max)
            worst.append((face_max, i))
    worst.sort(reverse=True)
    p(f"    faces differing   : {n_diff}/{a.numfaces}")
    p(f"    differing luxels  : {diff_luxels}/{total_luxels}"
      f" ({100.0 * diff_luxels / max(1, total_luxels):.3f}%)")
    p(f"    max channel delta : {global_max}")
    p(f"    mean abs delta    : {sum_abs / max(1, total_luxels * 3):.5f}")
    p(f"    worst faces       : {', '.join(str(fn) for _m, fn in worst[:8])}")


# ============================================================================
# main
# ============================================================================

def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("reference")
    ap.add_argument("candidate")
    ap.add_argument("--lightmap-detail", action="store_true",
                    help="on a lighting-lump difference, print the per-face lightmap analysis")
    ap.add_argument("--quiet", action="store_true", help="only print the final verdict")
    args = ap.parse_args()

    def p(*msg):
        if not args.quiet:
            print(*msg)

    try:
        a = Bsp(args.reference)
        b = Bsp(args.candidate)
    except ValueError as e:
        print(f"STRUCTURAL MISMATCH: {e}")
        return 2

    p(f"reference: {a.path}")
    p(f"candidate: {b.path}")
    p("")
    p(f"  {'lump':<13} {'reference':>11} {'candidate':>11}  status")
    p(f"  {'-' * 13} {'-' * 11} {'-' * 11}  {'-' * 24}")

    differing = []
    for i in range(NUM_LUMPS):
        la, lb = a.lumps[i], b.lumps[i]
        name = LUMP_NAMES[i]
        if la == lb:
            status = "identical"
        else:
            differing.append(i)
            if len(la) != len(lb):
                status = f"DIFFER (size {len(la)} vs {len(lb)})"
            else:
                status = f"DIFFER ({count_diff_bytes(la, lb)} bytes, @+{first_diff_offset(la, lb)})"
        p(f"  {name:<13} {human(len(la)):>11} {human(len(lb)):>11}  {status}")

    if not differing:
        print("\nVERDICT: IDENTICAL (all 15 lumps are byte-identical)")
        return 0

    p("")
    for i in differing:
        if i == LUMP_ENTITIES:
            hit = entities_first_diff_line(a.lumps[i], b.lumps[i])
            if hit:
                line, ta, tb = hit
                p(f"  entities first differ at line {line}:")
                p(f"    reference: {ta.decode('latin-1', 'replace')}")
                p(f"    candidate: {tb.decode('latin-1', 'replace')}")
        if i == LUMP_LIGHTING and args.lightmap_detail:
            p("  lighting lump detail:")
            lighting_detail(a, b, p)

    print(f"\nVERDICT: DIFFERENT - {len(differing)}/{NUM_LUMPS} lumps differ "
          f"({', '.join(LUMP_NAMES[i] for i in differing)})")
    return 1


if __name__ == "__main__":
    sys.exit(main())
