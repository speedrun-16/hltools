#!/usr/bin/env python3
"""gather_diff.py - compare two hlrad -dumpgather files (<base>.gather).

The dump holds every face's per-style direct-lighting gather results, the
seam replaced by the GPU backend.
Comparing a cpu dump against a gpu dump bisects a lightmap divergence to a
specific (face, style, sample) before the blur/bounce pipeline smears it.

Usage:
    python gather_diff.py a.gather b.gather [--tolerance T] [--top N]

Exit code: 0 if equal within tolerance, 1 if not, 2 on structural mismatch.
"""

import argparse
import struct
import sys

import numpy as np


def read_gather(path):
    with open(path, "rb") as f:
        data = f.read()
    if data[:8] != b"GATHER1\x00":
        raise ValueError(f"{path}: not a GATHER1 dump")
    numfaces = struct.unpack_from("<I", data, 8)[0]
    ofs = 12
    faces = []
    for _ in range(numfaces):
        (blob_len,) = struct.unpack_from("<I", data, ofs)
        ofs += 4
        blob = data[ofs:ofs + blob_len]
        ofs += blob_len
        if blob_len == 0:
            faces.append(None)  # unlit face
            continue
        numsamples, numstyles = struct.unpack_from("<II", blob, 0)
        pos = 8
        styles = {}
        for _ in range(numstyles):
            (style,) = struct.unpack_from("<I", blob, pos)
            pos += 4
            vals = np.frombuffer(blob, dtype="<f4", count=numsamples * 3, offset=pos)
            styles[style] = vals.reshape(numsamples, 3)
            pos += numsamples * 12
        faces.append((numsamples, styles))
    return faces


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("gather_a")
    ap.add_argument("gather_b")
    ap.add_argument("--tolerance", type=float, default=0.0,
                    help="max allowed per-channel delta in raw light units (default 0 = exact)")
    ap.add_argument("--top", type=int, default=10, help="worst samples to list (default 10)")
    args = ap.parse_args()

    a = read_gather(args.gather_a)
    b = read_gather(args.gather_b)
    if len(a) != len(b):
        print(f"STRUCTURAL MISMATCH: {len(a)} vs {len(b)} faces")
        return 2

    total = 0
    exceed = 0
    max_delta = 0.0
    worst = []  # (delta, face, style, sample)
    structural = 0

    for fn, (fa, fb) in enumerate(zip(a, b)):
        if (fa is None) != (fb is None):
            structural += 1
            continue
        if fa is None:
            continue
        na, sa = fa
        nb, sb = fb
        if na != nb or set(sa) != set(sb):
            structural += 1
            continue
        for style, va in sa.items():
            vb = sb[style]
            total += na
            d = np.abs(va - vb)
            dmax_per_sample = d.max(axis=1)
            m = float(dmax_per_sample.max()) if na else 0.0
            if m > max_delta:
                max_delta = m
            over = np.nonzero(dmax_per_sample > args.tolerance)[0]
            exceed += len(over)
            for idx in over[np.argsort(-dmax_per_sample[over])][:3]:
                worst.append((float(dmax_per_sample[idx]), fn, style, int(idx)))

    print(f"A: {args.gather_a}")
    print(f"B: {args.gather_b}")
    print(f"face/style structural mismatches : {structural}")
    print(f"samples compared                 : {total}")
    print(f"samples over tolerance ({args.tolerance:g})     : {exceed}")
    print(f"max channel delta                : {max_delta:g}")
    if worst:
        worst.sort(reverse=True)
        print(f"worst {min(args.top, len(worst))}:")
        for delta, fn, style, idx in worst[:args.top]:
            print(f"  face {fn:6d} style {style:3d} sample {idx:5d}  delta {delta:g}")

    if structural:
        return 2
    if exceed:
        print("VERDICT: DIFFERENT")
        return 1
    print("VERDICT: EQUAL (within tolerance)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
