#!/usr/bin/env python3
"""利用 RM 场地中心对称，把单视角 map 镜像翻转生成对面视角 map，并合并成全场地 map。

用法:
  # 已录红方 map，生成蓝方镜像并合并:
  python3 test/mirror_map.py config/map_red.pcd \
      --mirrored config/map_blue.pcd \
      --merged config/map.pcd

  # 自定义场地尺寸（默认 28 x 15）:
  python3 test/mirror_map.py input.pcd --field-x 28 --field-y 15
"""
import argparse
import sys
from pathlib import Path

import numpy as np


def load_pcd(path: str) -> np.ndarray:
    """加载 PCD（ascii / binary，仅 xyz）。返回 (N, 3) 数组。"""
    with open(path, "rb") as f:
        header = b""
        while not header.endswith(b"DATA ascii\n") and not header.endswith(b"DATA binary\n"):
            line = f.readline()
            if not line:
                raise RuntimeError("未找到 DATA 行")
            header += line
        is_binary = header.endswith(b"DATA binary\n")
        text = header.decode(errors="replace")

        fields, sizes, points = [], [], 0
        for line in text.splitlines():
            if line.startswith("FIELDS"):
                fields = line.split()[1:]
            elif line.startswith("SIZE"):
                sizes = [int(x) for x in line.split()[1:]]
            elif line.startswith("POINTS"):
                points = int(line.split()[1])

        idx = {n: i for i, n in enumerate(fields)}
        if not all(k in idx for k in ("x", "y", "z")):
            raise RuntimeError(f"PCD 缺少 xyz 字段: {fields}")

        if is_binary:
            point_size = sum(sizes)
            data = f.read(points * point_size)
            arr = np.frombuffer(data, dtype=np.uint8).reshape(points, point_size)
            xs = np.frombuffer(arr[:, sum(sizes[: idx["x"]]):
                                      sum(sizes[: idx["x"]]) + 4].tobytes(), dtype=np.float32)
            ys = np.frombuffer(arr[:, sum(sizes[: idx["y"]]):
                                      sum(sizes[: idx["y"]]) + 4].tobytes(), dtype=np.float32)
            zs = np.frombuffer(arr[:, sum(sizes[: idx["z"]]):
                                      sum(sizes[: idx["z"]]) + 4].tobytes(), dtype=np.float32)
            return np.stack([xs, ys, zs], axis=1)
        else:
            pts = []
            for line in f.read().decode().splitlines():
                if not line.strip():
                    continue
                v = line.split()
                pts.append([float(v[idx["x"]]), float(v[idx["y"]]), float(v[idx["z"]])])
            return np.array(pts, dtype=np.float32)


def save_pcd(path: str, pts: np.ndarray):
    """保存 ascii PCD（xyz 字段）。"""
    n = len(pts)
    header = (
        "# .PCD v0.7 - Point Cloud Data\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA ascii\n"
    )
    with open(path, "w") as f:
        f.write(header)
        for p in pts:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")


def mirror(pts: np.ndarray, field_x: float, field_y: float) -> np.ndarray:
    """关于场地中心做点对称：(x, y, z) → (field_x - x, field_y - y, z)。"""
    out = pts.copy()
    out[:, 0] = field_x - out[:, 0]
    out[:, 1] = field_y - out[:, 1]
    return out


def voxel_downsample(pts: np.ndarray, voxel: float) -> np.ndarray:
    if voxel <= 0 or len(pts) == 0:
        return pts
    idx = np.floor(pts / voxel).astype(np.int32)
    _, unique = np.unique(idx, axis=0, return_index=True)
    return pts[unique]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", help="原始 map.pcd（单视角录制）")
    ap.add_argument("--mirrored", default=None,
                    help="镜像后的 map 输出路径（默认 不保存）")
    ap.add_argument("--merged", default="config/map.pcd",
                    help="合并后的全场地 map 输出路径")
    ap.add_argument("--field-x", type=float, default=28.0,
                    help="场地 X 长度（米），默认 28")
    ap.add_argument("--field-y", type=float, default=15.0,
                    help="场地 Y 长度（米），默认 15")
    ap.add_argument("--voxel", type=float, default=0.05,
                    help="合并后下采样体素大小（米），默认 0.05")
    args = ap.parse_args()

    if not Path(args.input).exists():
        print(f"找不到 {args.input}", file=sys.stderr)
        sys.exit(1)

    print(f"▶ 加载 {args.input}")
    pts = load_pcd(args.input)
    print(f"  原始点数: {len(pts)}")

    print(f"▶ 镜像（中心 ({args.field_x/2:.1f}, {args.field_y/2:.1f})）")
    mirrored = mirror(pts, args.field_x, args.field_y)

    if args.mirrored:
        save_pcd(args.mirrored, mirrored)
        print(f"  镜像点云已保存: {args.mirrored} ({len(mirrored)} 点)")

    print(f"▶ 合并 + 体素下采样（{args.voxel}m）")
    merged = np.vstack([pts, mirrored])
    print(f"  合并前: {len(merged)} 点")
    merged = voxel_downsample(merged, args.voxel)
    print(f"  下采样后: {len(merged)} 点")

    save_pcd(args.merged, merged)
    print(f"✓ 全场地 map 已保存: {args.merged}")


if __name__ == "__main__":
    main()
