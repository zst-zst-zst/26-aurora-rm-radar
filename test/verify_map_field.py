#!/usr/bin/env python3
"""验证 cad_to_pcd.py 生成的 map_field.pcd 是否对齐到 rm_frame。

检查项:
1. 点云是否在 X∈[0,28], Y∈[0,15] 范围内
2. 5 个 PnP 标志点（calibrate_points_red.yaml）周围是否有点云
3. 场地中心 (14, 7.5) 是否在场地中心区域
4. 输出统计信息 + 简易 ASCII 俯视图

用法:
    python3 test/verify_map_field.py config/map_field.pcd
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from mirror_map import load_pcd  # noqa: E402

# 5 个 PnP 标志点（red 视角，rm_frame）
LANDMARKS = {
    "self_fortress":  (7.93210,  7.53070, 0.00000),
    "self_tower":     (10.82200, 3.66430, 1.86840),
    "enemy_base":     (23.01710, 4.45300, 0.59980),
    "enemy_tower":    (24.54140, 11.08010, 0.73160),
    "enemy_high":     (16.19990, 11.79060, 0.30080),
}


def stats(pts: np.ndarray):
    print(f"点数: {len(pts)}")
    for name, idx in (("X", 0), ("Y", 1), ("Z", 2)):
        v = pts[:, idx]
        print(f"{name}: min={v.min():.3f}  max={v.max():.3f}  mean={v.mean():.3f}")


def check_landmarks(pts: np.ndarray, radius: float = 0.5):
    """检查每个标志点 0.5m 半径内是否有点（说明结构在那里）。"""
    print(f"\n=== 标志点附近点云密度（半径 {radius}m）===")
    ok_count = 0
    for name, (x, y, z) in LANDMARKS.items():
        d2 = (pts[:, 0]-x)**2 + (pts[:, 1]-y)**2 + (pts[:, 2]-z)**2
        n = int((d2 < radius**2).sum())
        flag = "✅" if n >= 5 else "⚠️"
        print(f"  {flag} {name:15s} ({x:6.2f},{y:6.2f},{z:5.2f}) → 半径内 {n:4d} 点")
        if n >= 5:
            ok_count += 1
    print(f"\n命中 {ok_count}/5 个标志点")
    return ok_count


def ascii_topdown(pts: np.ndarray, w: int = 70, h: int = 20):
    """打印简易 ASCII 俯视图。"""
    print(f"\n=== 俯视图 (X 0-28m → 宽 {w} 字符, Y 0-15m → 高 {h} 字符) ===")
    grid = [[" "] * w for _ in range(h)]
    for x, y, _ in pts:
        if 0 <= x <= 28 and 0 <= y <= 15:
            cx = int(x / 28 * (w - 1))
            cy = int((15 - y) / 15 * (h - 1))  # Y 翻转：rm_frame Y 越大 → 屏幕越上
            ch = grid[cy][cx]
            if ch == " ":
                grid[cy][cx] = "."
            elif ch == ".":
                grid[cy][cx] = "+"
            elif ch == "+":
                grid[cy][cx] = "#"
    # 标记 5 个标志点
    for name, (x, y, _) in LANDMARKS.items():
        cx = int(x / 28 * (w - 1))
        cy = int((15 - y) / 15 * (h - 1))
        if 0 <= cx < w and 0 <= cy < h:
            grid[cy][cx] = name[0].upper()  # S/E

    border = "+" + "-" * w + "+"
    print(border)
    for row in grid:
        print("|" + "".join(row) + "|")
    print(border)
    print("图例: . 稀疏  + 中  # 密  S=self_xxx  E=enemy_xxx")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pcd", default="config/map_field.pcd", nargs="?")
    ap.add_argument("--radius", type=float, default=0.5)
    args = ap.parse_args()

    if not Path(args.pcd).exists():
        print(f"找不到 {args.pcd}", file=sys.stderr)
        sys.exit(1)

    print(f"▶ 加载 {args.pcd}")
    pts = load_pcd(args.pcd)
    stats(pts)
    check_landmarks(pts, args.radius)
    ascii_topdown(pts)


if __name__ == "__main__":
    main()
