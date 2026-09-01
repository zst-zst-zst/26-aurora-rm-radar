#!/usr/bin/env python3
"""从高密点云提取裁判系场地高度图（max-Z per cell），一次性输出缓存。

作用：为 visualize_field_surface.py / npz_to_heightgrid.py 提供缓存，避免重复扫几千万点。
输入：config/outputs/RMUC2026_field_dense.pcd 等裁判系点云。
输出：config/outputs/ground_surface_dense.{pkl,npz}

用法：
    python3 tools/extract_field_surface.py                          # 默认路径
    python3 tools/extract_field_surface.py --pcd <in.pcd> --grid-size 0.01
"""

import argparse
import os
import pickle
import re
import sys
import time

import numpy as np


def load_pcd(path):
    with open(path, "rb") as f:
        hdr = b""
        while True:
            line = f.readline()
            hdr += line
            if line.startswith(b"DATA"):
                break
        txt = hdr.decode()
        n = int(re.search(r"POINTS (\d+)", txt).group(1))
        fmt = re.search(r"DATA (\w+)", txt).group(1)
        if fmt == "binary":
            arr = np.frombuffer(f.read(), dtype=np.float32).reshape(n, -1)[:, :3]
        else:
            arr = np.loadtxt(f, max_rows=n)[:, :3]
    return arr.astype(np.float32, copy=False)


def extract_max_z_grid(pts, x_range, y_range, grid_size):
    """Return (X, Y, Z) meshgrids of the max-Z surface, vectorized."""
    x_min, x_max = x_range
    y_min, y_max = y_range
    nx = int(np.ceil((x_max - x_min) / grid_size))
    ny = int(np.ceil((y_max - y_min) / grid_size))

    x_centers = x_min + (np.arange(nx) + 0.5) * grid_size
    y_centers = y_min + (np.arange(ny) + 0.5) * grid_size
    X, Y = np.meshgrid(x_centers, y_centers)

    # Filter to ROI
    mask = (
        (pts[:, 0] >= x_min) & (pts[:, 0] < x_max) &
        (pts[:, 1] >= y_min) & (pts[:, 1] < y_max)
    )
    p = pts[mask]
    if len(p) == 0:
        return X, Y, np.full_like(X, np.nan)

    ix = np.minimum(((p[:, 0] - x_min) / grid_size).astype(np.int64), nx - 1)
    iy = np.minimum(((p[:, 1] - y_min) / grid_size).astype(np.int64), ny - 1)
    flat = iy * nx + ix
    z = p[:, 2].astype(np.float64)

    # Sort by flat index, then group-reduce with maximum.reduceat
    order = np.argsort(flat, kind="stable")
    flat_s = flat[order]
    z_s = z[order]
    uniq, starts = np.unique(flat_s, return_index=True)
    max_z = np.maximum.reduceat(z_s, starts)

    Z_flat = np.full(nx * ny, np.nan, dtype=np.float64)
    Z_flat[uniq] = max_z
    Z = Z_flat.reshape(ny, nx)
    return X, Y, Z


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument(
        "--input-pcd",
        default="/home/zst/T/config/outputs/RMUC2026_field_dense.pcd",
        help="Input point cloud (must already be in referee frame).",
    )
    ap.add_argument(
        "--out",
        default="/home/zst/T/config/outputs/ground_surface_dense",
        help="Output prefix (without extension).",
    )
    ap.add_argument("--grid-size", type=float, default=0.05, help="Grid resolution in meters (default 0.05).")
    ap.add_argument("--x-min", type=float, default=0.0)
    ap.add_argument("--x-max", type=float, default=28.0)
    ap.add_argument("--y-min", type=float, default=0.0)
    ap.add_argument("--y-max", type=float, default=15.0)
    ap.add_argument("--clip-to-field", action="store_true",
                    help="Restrict the grid to [0,28]x[0,15]. Off by default so the "
                         "official-STEP overshoot remains visible in the cache.")
    args = ap.parse_args()

    if not os.path.exists(args.input_pcd):
        sys.exit(f"input PCD not found: {args.input_pcd}")

    print(f"[1/3] loading PCD: {args.input_pcd}")
    t0 = time.time()
    pts = load_pcd(args.input_pcd)
    print(f"      {len(pts):,} points  bbox "
          f"x=[{pts[:,0].min():.2f},{pts[:,0].max():.2f}] "
          f"y=[{pts[:,1].min():.2f},{pts[:,1].max():.2f}] "
          f"z=[{pts[:,2].min():.2f},{pts[:,2].max():.2f}]  "
          f"({time.time()-t0:.1f}s)")

    if args.clip_to_field:
        x_range = (args.x_min, args.x_max)
        y_range = (args.y_min, args.y_max)
    else:
        # Cover the full bbox aligned to the referee frame, with a small pad.
        x_range = (min(args.x_min, float(pts[:, 0].min())),
                   max(args.x_max, float(pts[:, 0].max())))
        y_range = (min(args.y_min, float(pts[:, 1].min())),
                   max(args.y_max, float(pts[:, 1].max())))

    print(f"[2/3] extracting max-Z grid @ {args.grid_size} m  "
          f"x={x_range}  y={y_range}")
    t0 = time.time()
    X, Y, Z = extract_max_z_grid(pts, x_range, y_range, args.grid_size)
    valid = int(np.sum(~np.isnan(Z)))
    print(f"      grid {Z.shape[1]}x{Z.shape[0]} = {Z.size:,} cells, "
          f"valid={valid:,} ({100*valid/Z.size:.1f}%)  "
          f"z=[{np.nanmin(Z):.2f},{np.nanmax(Z):.2f}]  "
          f"({time.time()-t0:.1f}s)")

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    pkl_path = args.out + ".pkl"
    npz_path = args.out + ".npz"
    meta = {
        "X": X, "Y": Y, "Z": Z,
        "grid_size": args.grid_size,
        "x_range": list(x_range),
        "y_range": list(y_range),
        "z_range": [float(np.nanmin(Z)), float(np.nanmax(Z))],
        "valid_cells": valid,
        "total_cells": int(Z.size),
        "source_pcd": os.path.realpath(args.input_pcd),
        "frame": "referee (X 0..28, Y 0..15, Z up)",
    }
    with open(pkl_path, "wb") as f:
        pickle.dump(meta, f)
    np.savez(npz_path, X=X, Y=Y, Z=Z)
    print(f"[3/3] saved:\n      {pkl_path}\n      {npz_path}")


if __name__ == "__main__":
    main()
