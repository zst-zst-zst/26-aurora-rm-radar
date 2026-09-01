#!/usr/bin/env python3
"""可视化猜点表叠加到 RMUC2026 场地俯视图。

作用：调试 config/guess_pts.yaml 时检查盲区覆盖效果。每兵种输出一张 PNG，
另生成全兵种汇总图，底图优先用 config/outputs/RMUC2026_topview_dense.png，
没有则回退到 PCD 俯视投影。

用法：
    python3 tools/visualize_guess_pts.py                       # 默认参数
    python3 tools/visualize_guess_pts.py --yaml config/guess_pts_strategic.yaml \\
                                         --out_dir tools/guess_pts_viz_strategic

参数:
  --pcd       场地 PCD (binary float32 xyz)
  --yaml      guess_pts yaml
  --out_dir   输出目录 (默认 tools/guess_pts_viz)
  --zmin/zmax 投影时保留的 Z 范围 (默认 0.05~3.0m, 去地面零飘 + 去天花板)
"""
import argparse
import os
import struct
from pathlib import Path

import numpy as np
import yaml
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle


# ── 场地物理常数 (RMUC2026, 红方原点) ────────────────────────────────────
ARENA_W = 28.0
ARENA_H = 15.0

# 重要 landmark (来自 yaml 注释)
LANDMARKS = {
    "self_tower":   (10.82, 3.66),
    "cross_tower":  (17.18, 11.34),
    "red_outpost":  (9.20, 7.50),
    "blue_outpost": (18.80, 7.50),
    "red_base":     (3.00, 7.50),
    "blue_base":    (25.00, 7.50),
    "assembly":     (14.00, 7.50),
    "red_castle":   (6.60, 7.50),
    "blue_castle":  (21.40, 7.50),
}


def read_pcd_binary_xyz(path: str) -> np.ndarray:
    """读 binary PCD (FIELDS x y z, TYPE F F F)。返回 (N, 3) float32。"""
    with open(path, "rb") as f:
        # 解析 header
        header = []
        while True:
            line = f.readline()
            header.append(line)
            if line.startswith(b"DATA"):
                data_kind = line.strip().split()[1]
                break
        if data_kind != b"binary":
            raise RuntimeError(f"only DATA binary supported, got {data_kind}")
        # 找 POINTS / FIELDS / SIZE
        meta = {}
        for line in header:
            s = line.decode("ascii", errors="ignore").strip()
            if s.startswith("FIELDS"):
                meta["fields"] = s.split()[1:]
            elif s.startswith("SIZE"):
                meta["size"] = [int(x) for x in s.split()[1:]]
            elif s.startswith("TYPE"):
                meta["type"] = s.split()[1:]
            elif s.startswith("POINTS"):
                meta["n"] = int(s.split()[1])
        assert meta["fields"][:3] == ["x", "y", "z"], meta["fields"]
        assert meta["size"][:3] == [4, 4, 4]
        assert meta["type"][:3] == ["F", "F", "F"]
        n = meta["n"]
        stride = sum(meta["size"])
        raw = f.read(stride * n)
    arr = np.frombuffer(raw, dtype=np.uint8).reshape(n, stride)
    xyz = np.frombuffer(arr[:, :12].tobytes(), dtype=np.float32).reshape(n, 3)
    return xyz


def make_background(xyz: np.ndarray, zmin: float, zmax: float,
                    res: float = 0.05) -> tuple:
    """生成俯视图 2D 密度图。返回 (img, extent)。"""
    mask = (xyz[:, 2] >= zmin) & (xyz[:, 2] <= zmax) \
         & (xyz[:, 0] >= 0) & (xyz[:, 0] <= ARENA_W) \
         & (xyz[:, 1] >= 0) & (xyz[:, 1] <= ARENA_H)
    pts = xyz[mask]
    nx = int(round(ARENA_W / res))
    ny = int(round(ARENA_H / res))
    hist, _, _ = np.histogram2d(
        pts[:, 0], pts[:, 1],
        bins=[nx, ny],
        range=[[0, ARENA_W], [0, ARENA_H]],
        weights=pts[:, 2],  # 用 z 当权重 → 越高的结构越显眼
    )
    cnt, _, _ = np.histogram2d(
        pts[:, 0], pts[:, 1],
        bins=[nx, ny],
        range=[[0, ARENA_W], [0, ARENA_H]],
    )
    # avg z per cell (越高的越亮)
    img = np.zeros_like(hist)
    nz = cnt > 0
    img[nz] = hist[nz] / cnt[nz]
    img = img.T  # (ny, nx) → 让 imshow origin='lower' 显示 (x→右, y→上)
    extent = (0, ARENA_W, 0, ARENA_H)
    return img, extent


def draw_background(ax, bg_img, extent, title: str):
    if bg_img.ndim == 3:
        # 彩色 PNG (RGB) — 不需要 cmap
        ax.imshow(bg_img, origin="lower", extent=extent,
                  aspect="equal", alpha=1.0)
    else:
        ax.imshow(bg_img, origin="lower", extent=extent,
                  cmap="bone", aspect="equal", alpha=0.85)
    # landmark 标注
    for name, (x, y) in LANDMARKS.items():
        ax.plot(x, y, marker="*", color="gold", markersize=9,
                markeredgecolor="black", markeredgewidth=0.5, zorder=4)
        ax.annotate(name, (x, y), fontsize=7, color="yellow",
                    xytext=(4, 4), textcoords="offset points", zorder=5)
    # 红方半场 (左) 红色描线
    ax.add_patch(plt.Rectangle((0, 0), ARENA_W / 2, ARENA_H,
                               fill=False, edgecolor="red", linewidth=2.5,
                               zorder=3))
    ax.text(ARENA_W / 4, ARENA_H - 0.5, "RED",
            color="red", fontsize=14, fontweight="bold",
            ha="center", zorder=6)
    # 蓝方半场 (右) 蓝色描线
    ax.add_patch(plt.Rectangle((ARENA_W / 2, 0), ARENA_W / 2, ARENA_H,
                               fill=False, edgecolor="cyan", linewidth=2.5,
                               zorder=3))
    ax.text(ARENA_W * 3 / 4, ARENA_H - 0.5, "BLUE",
            color="cyan", fontsize=14, fontweight="bold",
            ha="center", zorder=6)
    # 中线 (分界线)
    ax.axvline(x=ARENA_W / 2, color="white", linewidth=1.0,
               linestyle="--", alpha=0.6, zorder=3)
    # 场地外边界
    ax.add_patch(plt.Rectangle((0, 0), ARENA_W, ARENA_H,
                               fill=False, edgecolor="white", linewidth=1.0))
    ax.set_xlim(-0.5, ARENA_W + 0.5)
    ax.set_ylim(-0.5, ARENA_H + 0.5)
    ax.set_xlabel("X (m, red→blue)")
    ax.set_ylabel("Y (m)")
    ax.set_title(title)
    ax.grid(False)


# 兵种颜色
ROBOT_COLORS = {
    "R1": "#FF3030", "R2": "#FF8030", "R3": "#FF40A0", "R4": "#FFC040", "R5": "#FFFF40", "R7": "#FF0000",
    "B1": "#3060FF", "B2": "#30A0FF", "B3": "#40FFC0", "B4": "#40C0FF", "B5": "#80FFFF", "B7": "#0000FF",
}


def plot_single_robot(bg_img, extent, robot: str, pts: list, out_path: str):
    fig, ax = plt.subplots(figsize=(14, 7.5), dpi=100)
    draw_background(ax, bg_img, extent,
                    f"{robot} candidates ({len(pts)} pts)")
    color = ROBOT_COLORS.get(robot, "magenta")
    for i, (x, y) in enumerate(pts):
        ax.plot(x, y, "o", color=color, markersize=9,
                markeredgecolor="white", markeredgewidth=1.0, zorder=10)
        ax.annotate(f"{i}: ({x:.1f},{y:.1f})", (x, y),
                    fontsize=7, color="white",
                    xytext=(6, -3), textcoords="offset points", zorder=11)
    fig.tight_layout()
    fig.savefig(out_path, dpi=120, bbox_inches="tight",
                facecolor="black")
    plt.close(fig)


def plot_all_robots(bg_img, extent, robots: dict, out_path: str, side: str):
    """side: 'R' or 'B', 画一张总图"""
    fig, ax = plt.subplots(figsize=(16, 8.5), dpi=100)
    title_side = "Red" if side == "R" else "Blue"
    draw_background(ax, bg_img, extent,
                    f"All {title_side}-side robot guess points")
    for robot, pts in robots.items():
        if not robot.startswith(side):
            continue
        color = ROBOT_COLORS.get(robot, "magenta")
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ax.scatter(xs, ys, c=color, s=60, edgecolors="white",
                   linewidths=0.8, label=f"{robot} ({len(pts)})", zorder=10)
    ax.legend(loc="upper left", fontsize=9, framealpha=0.8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=120, bbox_inches="tight",
                facecolor="black")
    plt.close(fig)


def load_topview_png(path: str) -> tuple:
    """加载官方俯视图 PNG，裁剪掉青色/黑色边距后映射到 (0,28)x(0,15)。"""
    from PIL import Image
    img = np.array(Image.open(path).convert("RGB"))
    # 通过非青色非黑色像素探测真实场地边界
    mid_row = img[img.shape[0] // 2]
    mid_col = img[:, img.shape[1] // 2]

    def is_field(rgb):
        r, g, b = int(rgb[0]), int(rgb[1]), int(rgb[2])
        if r < 10 and g < 10 and b < 10:
            return False  # 黑色边框
        if r < 10 and g > 200 and b > 200:
            return False  # 青色背景
        return True

    x_left = next(x for x in range(img.shape[1]) if is_field(mid_row[x]))
    x_right = next(x for x in range(img.shape[1] - 1, -1, -1)
                   if is_field(mid_row[x]))
    y_top = next(y for y in range(img.shape[0]) if is_field(mid_col[y]))
    y_bot = next(y for y in range(img.shape[0] - 1, -1, -1)
                 if is_field(mid_col[y]))
    # 裁剪
    img = img[y_top:y_bot + 1, x_left:x_right + 1]
    # PIL y 是从顶部往下, 但场地坐标 y 是从底部往上 → flipud
    img = np.flipud(img)
    extent = (0, ARENA_W, 0, ARENA_H)
    return img, extent


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pcd", default="/home/zst/T/config/map/RMUC2026_ground_only_1cm.pcd")
    ap.add_argument("--bg_img", default="/home/zst/T/config/outputs/RMUC2026_topview_dense.png",
                    help="官方俯视图 PNG (优先) — 空字符串则改用 PCD 投影")
    ap.add_argument("--yaml", default="/home/zst/T/config/guess_pts.yaml")
    ap.add_argument("--out_dir", default="/home/zst/T/tools/guess_pts_viz")
    ap.add_argument("--zmin", type=float, default=0.05)
    ap.add_argument("--zmax", type=float, default=3.0)
    ap.add_argument("--res", type=float, default=0.05)
    args = ap.parse_args()

    Path(args.out_dir).mkdir(parents=True, exist_ok=True)

    if args.bg_img and os.path.exists(args.bg_img):
        print(f"[1/4] using official topview as background: {args.bg_img}")
        bg, extent = load_topview_png(args.bg_img)
    else:
        print(f"[1/4] reading PCD: {args.pcd}")
        xyz = read_pcd_binary_xyz(args.pcd)
        print(f"     loaded {len(xyz):,} points, "
              f"x=[{xyz[:,0].min():.2f},{xyz[:,0].max():.2f}], "
              f"y=[{xyz[:,1].min():.2f},{xyz[:,1].max():.2f}], "
              f"z=[{xyz[:,2].min():.2f},{xyz[:,2].max():.2f}]")
        print(f"[2/4] making top-down background (z in [{args.zmin},{args.zmax}])")
        bg, extent = make_background(xyz, args.zmin, args.zmax, args.res)

    print(f"[3/4] loading guess pts: {args.yaml}")
    with open(args.yaml, "r") as f:
        cfg = yaml.safe_load(f)
    robots = cfg.get("guess_points", {})
    print(f"     {len(robots)} robot types: {list(robots.keys())}")

    print(f"[4/4] drawing into {args.out_dir}")
    for robot, pts in robots.items():
        out = os.path.join(args.out_dir, f"{robot}.png")
        plot_single_robot(bg, extent, robot, pts, out)
        print(f"     wrote {out} ({len(pts)} pts)")

    # 全方汇总图
    for side in ("R", "B"):
        out = os.path.join(args.out_dir, f"_all_{side}.png")
        plot_all_robots(bg, extent, robots, out, side)
        print(f"     wrote {out}")

    print("done.")


if __name__ == "__main__":
    main()
