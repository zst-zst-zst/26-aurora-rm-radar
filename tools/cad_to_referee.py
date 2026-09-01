#!/usr/bin/env python3
"""把 CAD 居中系的 PCD 点云平移到 RMUC2026 裁判坐标系。

作用：CAD 导出的 PCD 以场地对称中心为原点，需平移到裁判系（(0,0,0) 在红方角）才能与雷达/相机脚本对齐。
参数在下方 CAD_TO_REFEREE_T 中硬编码，不要改。

CAD 居中系（原始 STEP/STL 转出）：
    x ∈ [-14.88, 14.88], y ∈ [-6.40, 9.65], z ∈ [-1.84, 1.86]
裁判坐标系（场地 28 × 15 m，对称中心 (14, 7.5)）：
    x_referee = x_centered + 14.0
    y_referee = y_centered + 5.87
    z_referee = z_centered + 1.64
纯平移，无旋转/缩放。

用法：
    python3 tools/cad_to_referee.py <in.pcd> <out.pcd>
    python3 tools/cad_to_referee.py <in.pcd> <out.pcd> --inverse   # 裁判系 → CAD 系
"""

import argparse
import os
import re
import struct
import sys
import time

import numpy as np


# ---- 标准平移常量（不要改）------------------------------------------
CAD_TO_REFEREE_T = np.array([14.0, 5.87, 1.64], dtype=np.float64)


def cad_to_referee(pts_centered: np.ndarray) -> np.ndarray:
    """CAD 居中系 (N,3) -> 裁判系 (N,3)."""
    return pts_centered.astype(np.float64, copy=True) + CAD_TO_REFEREE_T


def referee_to_cad(pts_referee: np.ndarray) -> np.ndarray:
    """裁判系 (N,3) -> CAD 居中系 (N,3) (反向)."""
    return pts_referee.astype(np.float64, copy=True) - CAD_TO_REFEREE_T


def load_pcd(path: str) -> np.ndarray:
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
    return np.ascontiguousarray(arr.astype(np.float32))


def save_pcd(path: str, pts: np.ndarray) -> None:
    n = len(pts)
    header = (
        "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode())
        f.write(pts.astype(np.float32).tobytes())


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("input_pcd", help="CAD 居中系的 PCD 文件")
    ap.add_argument("output_pcd", help="平移后的裁判系 PCD 文件")
    ap.add_argument("--inverse", action="store_true",
                    help="反向：裁判系 -> CAD 居中系")
    args = ap.parse_args()

    t0 = time.time()
    pts = load_pcd(args.input_pcd)
    print(f"loaded {len(pts):,} points  bbox "
          f"x=[{pts[:,0].min():.2f},{pts[:,0].max():.2f}] "
          f"y=[{pts[:,1].min():.2f},{pts[:,1].max():.2f}] "
          f"z=[{pts[:,2].min():.2f},{pts[:,2].max():.2f}]")

    if args.inverse:
        pts2 = referee_to_cad(pts).astype(np.float32)
        print(f"applied inverse shift {-CAD_TO_REFEREE_T}")
    else:
        pts2 = cad_to_referee(pts).astype(np.float32)
        print(f"applied shift {+CAD_TO_REFEREE_T}")

    print(f"after: bbox "
          f"x=[{pts2[:,0].min():.2f},{pts2[:,0].max():.2f}] "
          f"y=[{pts2[:,1].min():.2f},{pts2[:,1].max():.2f}] "
          f"z=[{pts2[:,2].min():.2f},{pts2[:,2].max():.2f}]")

    os.makedirs(os.path.dirname(args.output_pcd) or ".", exist_ok=True)
    save_pcd(args.output_pcd, pts2)
    print(f"saved {args.output_pcd}  ({time.time()-t0:.1f}s)")


if __name__ == "__main__":
    main()
