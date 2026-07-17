#!/usr/bin/env python3
"""lightmap_diff.py - compare lightmaps between two GoldSrc (BSP v30) files.

Used to verify that HLRAD optimizations do not change the compiled lightmaps.

Usage:
    python lightmap_diff.py old.bsp new.bsp [--png-dir DIR] [--top N] [--quiet]
                                            [--stats] [--gate gpu-bsp|gpu-rt]

--stats prints a per-luxel delta histogram (luxel delta = max channel delta).
--gate applies the GPU acceptance gates
(A is the CPU reference, B the candidate): >=99.5%% of luxels within 1,
>=99.99%% within 4, hard cap 16 - except that gpu-rt additionally allows >16
on luxels whose reference neighborhood already contains a shadow edge.

Exit code: 0 if lightmaps are identical (or the gate passes), 1 if they
differ (or the gate fails), 2 on structural mismatch.
"""

import argparse
import struct
import sys
import zlib
from pathlib import Path

import numpy as np

BSP_VERSION = 30
LUMP_ENTITIES = 0
LUMP_PLANES = 1
LUMP_TEXTURES = 2
LUMP_VERTEXES = 3
LUMP_VISIBILITY = 4
LUMP_NODES = 5
LUMP_TEXINFO = 6
LUMP_FACES = 7
LUMP_LIGHTING = 8
LUMP_CLIPNODES = 9
LUMP_LEAFS = 10
LUMP_MARKSURFACES = 11
LUMP_EDGES = 12
LUMP_SURFEDGES = 13
LUMP_MODELS = 14
NUM_LUMPS = 15

TEXTURE_STEP = 16.0
MAXLIGHTMAPS = 4


class Bsp:
    def __init__(self, path):
        self.path = Path(path)
        data = self.path.read_bytes()
        version = struct.unpack_from("<i", data, 0)[0]
        if version != BSP_VERSION:
            raise ValueError(f"{path}: unsupported BSP version {version}")
        self.lumps = []
        for i in range(NUM_LUMPS):
            ofs, ln = struct.unpack_from("<ii", data, 4 + 8 * i)
            self.lumps.append(data[ofs:ofs + ln])

        # vertexes: 3 floats each
        self.vertexes = np.frombuffer(self.lumps[LUMP_VERTEXES], dtype="<f4").reshape(-1, 3)
        # edges: 2 uint16
        self.edges = np.frombuffer(self.lumps[LUMP_EDGES], dtype="<u2").reshape(-1, 2)
        # surfedges: int32
        self.surfedges = np.frombuffer(self.lumps[LUMP_SURFEDGES], dtype="<i4")
        # texinfo: vecs[2][4] float, miptex int32, flags int32 -> 40 bytes
        ti = np.frombuffer(self.lumps[LUMP_TEXINFO], dtype="<f4").reshape(-1, 10)
        self.texinfo_vecs = ti[:, :8].reshape(-1, 2, 4)
        ti_i = np.frombuffer(self.lumps[LUMP_TEXINFO], dtype="<i4").reshape(-1, 10)
        self.texinfo_flags = ti_i[:, 9]
        # faces: planenum u2, side i2, firstedge i4, numedges i2, texinfo i2,
        #        styles 4*u1, lightofs i4 -> 20 bytes
        fdata = self.lumps[LUMP_FACES]
        self.numfaces = len(fdata) // 20
        self.faces = []
        for i in range(self.numfaces):
            planenum, side, firstedge, numedges, texinfo = struct.unpack_from("<HhiHh", fdata, i * 20)
            styles = struct.unpack_from("<4B", fdata, i * 20 + 12)
            lightofs = struct.unpack_from("<i", fdata, i * 20 + 16)[0]
            self.faces.append((firstedge, numedges, texinfo, styles, lightofs))
        self.lightdata = np.frombuffer(self.lumps[LUMP_LIGHTING], dtype=np.uint8)

    def face_extents(self, facenum):
        """Replicates GetFaceExtents in bspfile.cpp (float32 min/max of double-summed dot products)."""
        firstedge, numedges, texinfo, _styles, _lightofs = self.faces[facenum]
        se = self.surfedges[firstedge:firstedge + numedges]
        vidx = np.where(se >= 0, self.edges[np.abs(se), 0], self.edges[np.abs(se), 1])
        pts = self.vertexes[vidx].astype(np.float64)  # (n, 3)
        vecs = self.texinfo_vecs[texinfo].astype(np.float64)  # (2, 4)
        # calculate point vector products with double accumulation then cast to float32
        vals = (pts @ vecs[:, :3].T) + vecs[:, 3]  # (n, 2)
        vals = vals.astype(np.float32)
        mins = np.minimum(np.float32(999999), vals.min(axis=0))
        maxs = np.maximum(np.float32(-99999), vals.max(axis=0))
        bmins = np.floor(mins / TEXTURE_STEP).astype(int)
        bmaxs = np.ceil(maxs / TEXTURE_STEP).astype(int)
        w = int(bmaxs[0] - bmins[0]) + 1
        h = int(bmaxs[1] - bmins[1]) + 1
        return w, h

    def face_lightmaps(self, facenum):
        """Returns (w, h, {style: HxWx3 uint8 array}) or None if unlit."""
        _fe, _ne, texinfo, styles, lightofs = self.faces[facenum]
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


def write_png(path, rgb):
    """Minimal PNG writer for a HxWx3 uint8 array (no external deps)."""
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag, payload):
        c = struct.pack(">I", len(payload)) + tag + payload
        return c + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF)

    ihdr = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)
    png = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr)
           + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))
    Path(path).write_bytes(png)


def upscale(img, factor=8):
    return np.repeat(np.repeat(img, factor, axis=0), factor, axis=1)


def edge_mask(img, threshold=32):
    """Luxels whose 8-neighborhood in img contains a channel jump > threshold
    (i.e. the reference already has a shadow edge there)."""
    h, w, _ = img.shape
    ref = img.astype(np.int16)
    mask = np.zeros((h, w), dtype=bool)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy == 0 and dx == 0:
                continue
            shifted = np.roll(np.roll(ref, dy, axis=0), dx, axis=1)
            d = np.abs(ref - shifted).max(axis=2)
            # roll wraps around; mask out the wrapped border rows/cols
            valid = np.ones((h, w), dtype=bool)
            if dy == 1:
                valid[0, :] = False
            elif dy == -1:
                valid[-1, :] = False
            if dx == 1:
                valid[:, 0] = False
            elif dx == -1:
                valid[:, -1] = False
            mask |= (d > threshold) & valid
    return mask


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("bsp_a")
    ap.add_argument("bsp_b")
    ap.add_argument("--png-dir", help="dump PNGs (a / b / amplified diff) for the worst differing faces")
    ap.add_argument("--top", type=int, default=10, help="number of worst faces to list/dump (default 10)")
    ap.add_argument("--quiet", action="store_true", help="only print the final verdict")
    ap.add_argument("--stats", action="store_true", help="print the per-luxel delta histogram")
    ap.add_argument("--gate", choices=["gpu-bsp", "gpu-rt"],
                    help="apply the design-doc acceptance gates (implies --stats)")
    args = ap.parse_args()
    if args.gate:
        args.stats = True

    a = Bsp(args.bsp_a)
    b = Bsp(args.bsp_b)

    def p(*msg):
        if not args.quiet:
            print(*msg)

    p(f"A: {a.path}  ({a.numfaces} faces, lighting {len(a.lightdata)} bytes)")
    p(f"B: {b.path}  ({b.numfaces} faces, lighting {len(b.lightdata)} bytes)")

    if a.numfaces != b.numfaces:
        print(f"STRUCTURAL MISMATCH: face count {a.numfaces} vs {b.numfaces} - not comparable")
        return 2

    # fast path when the whole lump and face metadata are identical
    same_lump = (len(a.lightdata) == len(b.lightdata)
                 and np.array_equal(a.lightdata, b.lightdata))
    same_meta = all(fa[3] == fb[3] and fa[4] == fb[4]
                    for fa, fb in zip(a.faces, b.faces))
    if same_lump and same_meta:
        print("VERDICT: IDENTICAL (lighting lump and face styles/offsets are byte-identical)")
        return 0

    # detailed comparison by face
    n_diff = 0
    n_style_diff = 0
    n_geo_diff = 0
    total_luxels = 0
    diff_luxels = 0
    sum_abs = 0.0
    worst = []  # (maxdelta, meandelta, facenum)
    global_max = 0
    # histogram of maximum channel delta for each luxel across styles
    hist = {"1": 0, "2-4": 0, "5-16": 0, ">16": 0}
    gt16_edge = 0
    gt16_flat = 0

    for i in range(a.numfaces):
        la = a.face_lightmaps(i)
        lb = b.face_lightmaps(i)
        if (la is None) != (lb is None):
            n_style_diff += 1
            n_diff += 1
            worst.append((999, 999, i))
            continue
        if la is None:
            continue
        wa, ha, mapsa = la
        wb, hb, mapsb = lb
        if (wa, ha) != (wb, hb):
            n_geo_diff += 1
            n_diff += 1
            continue
        total_luxels += wa * ha
        if set(mapsa) != set(mapsb):
            n_style_diff += 1
            n_diff += 1
            worst.append((999, 999, i))
            continue
        face_max = 0
        face_sum = 0.0
        face_delta = np.zeros((ha, wa), dtype=np.int16)
        face_edge = np.zeros((ha, wa), dtype=bool)
        for style, img_a in mapsa.items():
            img_b = mapsb[style]
            if np.array_equal(img_a, img_b):
                continue
            d = np.abs(img_a.astype(np.int16) - img_b.astype(np.int16))
            face_max = max(face_max, int(d.max()))
            face_sum += float(d.sum())
            dl = d.max(axis=2)
            face_delta = np.maximum(face_delta, dl)
            if args.stats and face_max > 16:
                face_edge |= edge_mask(img_a)
        if face_max > 0:
            n_diff += 1
            diff_luxels += int((face_delta > 0).sum())
            sum_abs += face_sum
            global_max = max(global_max, face_max)
            worst.append((face_max, face_sum / (wa * ha * 3), i))
            if args.stats:
                hist["1"] += int((face_delta == 1).sum())
                hist["2-4"] += int(((face_delta >= 2) & (face_delta <= 4)).sum())
                hist["5-16"] += int(((face_delta >= 5) & (face_delta <= 16)).sum())
                over = face_delta > 16
                hist[">16"] += int(over.sum())
                gt16_edge += int((over & face_edge).sum())
                gt16_flat += int((over & ~face_edge).sum())

    if n_diff == 0:
        # metadata layout change only with the same data at different light offsets
        print("VERDICT: IDENTICAL CONTENT (byte layout differs, per-face lightmaps equal)")
        return 0

    worst.sort(reverse=True)
    print(f"VERDICT: DIFFERENT - {n_diff}/{a.numfaces} faces differ")
    p(f"  style/offset mismatches : {n_style_diff}")
    p(f"  extent mismatches       : {n_geo_diff}")
    p(f"  differing luxels        : {diff_luxels}/{total_luxels}"
      f" ({100.0 * diff_luxels / max(1, total_luxels):.3f}%)")
    p(f"  max channel delta       : {global_max}")
    p(f"  mean abs delta (global) : {sum_abs / max(1, total_luxels * 3):.5f}")
    p(f"  worst {min(args.top, len(worst))} faces (maxdelta, meandelta, facenum):")
    for md, mean, fn in worst[:args.top]:
        p(f"    face {fn:6d}  max {md:3}  mean {mean:.4f}")

    gate_result = None
    if args.stats:
        n0 = total_luxels - diff_luxels
        n_le1 = n0 + hist["1"]
        n_le4 = n_le1 + hist["2-4"]
        n_le16 = n_le4 + hist["5-16"]
        t = max(1, total_luxels)
        print("  luxel delta histogram (max channel delta, max over styles):")
        print(f"    0      : {n0:10d} ({100.0 * n0 / t:8.4f}%)")
        print(f"    1      : {hist['1']:10d}   cumulative <=1  {100.0 * n_le1 / t:8.4f}%")
        print(f"    2-4    : {hist['2-4']:10d}   cumulative <=4  {100.0 * n_le4 / t:8.4f}%")
        print(f"    5-16   : {hist['5-16']:10d}   cumulative <=16 {100.0 * n_le16 / t:8.4f}%")
        print(f"    >16    : {hist['>16']:10d}   ({gt16_edge} at shadow edges, {gt16_flat} flat)")

        if args.gate:
            failures = []
            if n_style_diff or n_geo_diff:
                failures.append("structural mismatches (styles/offsets/extents must be identical)")
            if n_le1 / t < 0.995:
                failures.append(f"<=1 coverage {100.0 * n_le1 / t:.4f}% (need >= 99.5%)")
            if n_le4 / t < 0.9999:
                failures.append(f"<=4 coverage {100.0 * n_le4 / t:.4f}% (need >= 99.99%)")
            over_budget = hist[">16"] if args.gate == "gpu-bsp" else gt16_flat
            if over_budget > 0:
                kind = "luxels" if args.gate == "gpu-bsp" else "flat (non-shadow-edge) luxels"
                failures.append(f"{over_budget} {kind} exceed the 16/255 cap")
            gate_result = (len(failures) == 0)
            if gate_result:
                print(f"GATE {args.gate}: PASS")
            else:
                print(f"GATE {args.gate}: FAIL")
                for f in failures:
                    print(f"  - {f}")

    if args.png_dir:
        outdir = Path(args.png_dir)
        outdir.mkdir(parents=True, exist_ok=True)
        for md, mean, fn in worst[:args.top]:
            la = a.face_lightmaps(fn)
            lb = b.face_lightmaps(fn)
            if la is None or lb is None:
                continue
            wa, ha, mapsa = la
            _wb, _hb, mapsb = lb
            for style in mapsa:
                if style not in mapsb:
                    continue
                img_a, img_b = mapsa[style], mapsb[style]
                if img_a.shape != img_b.shape:
                    continue
                d = np.abs(img_a.astype(np.int16) - img_b.astype(np.int16))
                amp = np.clip(d * 16, 0, 255).astype(np.uint8)
                write_png(outdir / f"face{fn}_s{style}_a.png", upscale(img_a))
                write_png(outdir / f"face{fn}_s{style}_b.png", upscale(img_b))
                write_png(outdir / f"face{fn}_s{style}_diff_x16.png", upscale(amp))
        p(f"  PNGs written to {outdir}")

    if gate_result is not None:
        return 0 if gate_result else 1
    return 1


if __name__ == "__main__":
    sys.exit(main())
