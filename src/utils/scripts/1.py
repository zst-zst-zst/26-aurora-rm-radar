#!/usr/bin/env python3
import argparse
import math
import os
import re

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

FIELD_X_MIN = 0.0
FIELD_X_MAX = 28.0
FIELD_Y_MIN = 0.0
FIELD_Y_MAX = 15.0

HOLLOW_POLYGON_POINTS = [
    ("Red_Base", (7.93210, 7.53070, 0.00000)),
    ("Red_Outpost_High", (10.82200, 3.66430, 1.86840)),
    ("Blue_Trapezoid_Highland", (23.01710, 4.45300, 0.59980)),
    ("Blue_Left_90_Barrier", (24.54140, 11.08010, 0.73160)),
    ("Blue_Outpost_Lamp_Base", (16.19990, 11.79060, 0.30080)),
]


def parse_map_points(file_path):
    with open(file_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    section = None
    region_heights = {}
    regions = {}

    header_pattern = re.compile(r"^([A-Za-z0-9_]+):\s*$")
    point_pattern = re.compile(
        r"\{\s*x:\s*([+-]?\d+(?:\.\d+)?),\s*y:\s*([+-]?\d+(?:\.\d+)?),\s*z:\s*([+-]?\d+(?:\.\d+)?)\s*\}"
    )

    in_heights = False
    for raw in lines:
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if line in ("%YAML:1.0", "---"):
            continue

        m = header_pattern.match(line)
        if m:
            section = m.group(1)
            if section != "Region_Heights":
                regions.setdefault(section, [])
            in_heights = section == "Region_Heights"
            continue

        if in_heights:
            if line.startswith("{") or line.startswith("}"):
                continue
            if ":" in line:
                key, value = line.rstrip(",").split(":", 1)
                region_heights[key.strip()] = float(value.strip())
            continue

        if section and section != "Region_Heights":
            pm = point_pattern.search(line)
            if pm:
                x, y, z = map(float, pm.groups())
                regions[section].append((x, y, z))

    regions = {k: v for k, v in regions.items() if v}
    return region_heights, regions


def to_referee_xy(x, y):
    return x + 14.0, y + 5.85566


def mirror_referee_xy(x, y):
    return FIELD_X_MAX - x, FIELD_Y_MAX - y


def swap_red_blue_prefix(label):
    if label.startswith("Red_"):
        return "Blue_" + label[len("Red_"):]
    if label.startswith("Blue_"):
        return "Red_" + label[len("Blue_"):]
    return label


def region_colors(name):
    if name.startswith("Red_"):
        return "#f9caca", "#c54141"
    if name.startswith("Blue_"):
        return "#cfe6ff", "#3f79c9"
    return "#e8e8e8", "#7d7d7d"


def prepare_polygons(regions, input_frame):
    polygons_xy = {}
    polygons_xyz = {}
    for name, pts in regions.items():
        xy = []
        xyz = []
        for x, y, z in pts:
            if input_frame == "center":
                x, y = to_referee_xy(x, y)
            xy.append((x, y))
            xyz.append((x, y, z))
        polygons_xy[name] = xy
        polygons_xyz[name] = xyz
    return polygons_xy, polygons_xyz


def get_hollow_polygon(input_frame, self_color):
    hollow_xy = []
    hollow_xyz = []
    labels = []
    for label, (x, y, z) in HOLLOW_POLYGON_POINTS:
        if input_frame == "center":
            x, y = to_referee_xy(x, y)
        if self_color == "blue":
            x, y = mirror_referee_xy(x, y)
            label = swap_red_blue_prefix(label)
        hollow_xy.append((x, y))
        hollow_xyz.append((x, y, z))
        labels.append(label)
    return hollow_xy, hollow_xyz, labels


def polygon_centroid(poly):
    x = sum(p[0] for p in poly) / len(poly)
    y = sum(p[1] for p in poly) / len(poly)
    return x, y


def validate_referee_bounds(polygons, strict=True, eps=1e-6):
    violations = []
    for region, pts in polygons.items():
        for i, (x, y) in enumerate(pts):
            if (x < FIELD_X_MIN - eps or x > FIELD_X_MAX + eps or
                    y < FIELD_Y_MIN - eps or y > FIELD_Y_MAX + eps):
                violations.append((region, i, x, y))

    if not violations:
        return

    lines = [
        "Found out-of-field points in referee frame (x in [0,28], y in [0,15]):"
    ]
    for region, idx, x, y in violations:
        lines.append(f"  - {region}[{idx}] = ({x:.5f}, {y:.5f})")
    msg = "\n".join(lines)

    if strict:
        raise ValueError(msg)

    print("[WARN] " + msg.replace("\n", "\n[WARN] "))


# ===================== 2D 绘图（完美原版） =====================
def draw_2d(polygons_xy, heights, hollow_xy, hollow_labels, output_path):
    fig, ax = plt.subplots(figsize=(14, 8), dpi=150)
    fig.patch.set_facecolor("#fafafa")
    ax.set_facecolor("#ffffff")

    for name, poly_xy in polygons_xy.items():
        fill_color, edge_color = region_colors(name)
        xy = np.array(poly_xy)
        ax.fill(xy[:, 0], xy[:, 1], color=fill_color, alpha=0.75, linewidth=0)
        ax.plot(xy[:, 0], xy[:, 1], color=edge_color, linewidth=1.2)
        ax.plot([xy[-1, 0], xy[0, 0]], [xy[-1, 1], xy[0, 1]], color=edge_color, linewidth=1.2)

        cx, cy = polygon_centroid(poly_xy)
        h = heights.get(name, float("nan"))
        label = f"{name} ({h:.2f}m)" if not math.isnan(h) else name
        ax.text(cx, cy, label, fontsize=7, color="#333333", ha="center", va="center")

    hollow_arr = np.array(hollow_xy)
    ax.plot(
        np.r_[hollow_arr[:, 0], hollow_arr[0, 0]],
        np.r_[hollow_arr[:, 1], hollow_arr[0, 1]],
        linestyle="--",
        linewidth=2.0,
        color="#1f1f1f",
        marker="o",
        markerfacecolor="none",
        markeredgewidth=1.5,
        markeredgecolor="#1f1f1f",
    )
    for i, label in enumerate(hollow_labels):
        ax.text(hollow_arr[i, 0], hollow_arr[i, 1] + 0.2, label, fontsize=8, color="#111111")

    ax.set_xlim(FIELD_X_MIN, FIELD_X_MAX)
    ax.set_ylim(FIELD_Y_MIN, FIELD_Y_MAX)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(alpha=0.25)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Map Regions 2D (Filled) + Top 5 Points Hollow Polygon")

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved 2D visualization: {output_path}")


# ===================== 3D 绘图（增强版：挤出+支柱+排序） =====================
def draw_3d(polygons_xyz, hollow_xyz, hollow_labels, heights, output_path):
    """
    立体挤出 + 顶点排序 + 空心点垂直支柱和地面阴影
    """
    fig = plt.figure(figsize=(14, 9), dpi=150, facecolor="#f8f8f8")
    ax = fig.add_subplot(111, projection="3d")
    ax.set_proj_type("persp")  # 透视投影

    # ---- 地面网格 ----
    floor_x, floor_y = np.meshgrid(
        np.linspace(FIELD_X_MIN, FIELD_X_MAX, 40),
        np.linspace(FIELD_Y_MIN, FIELD_Y_MAX, 30),
    )
    floor_z = np.zeros_like(floor_x)
    ax.plot_surface(floor_x, floor_y, floor_z, color="#e0e0e0", alpha=0.25, linewidth=0, shade=True, zorder=0)
    for x in np.linspace(FIELD_X_MIN, FIELD_X_MAX, 15):
        ax.plot([x, x], [FIELD_Y_MIN, FIELD_Y_MAX], [0, 0], color="#cccccc", linewidth=0.4, alpha=0.5)
    for y in np.linspace(FIELD_Y_MIN, FIELD_Y_MAX, 12):
        ax.plot([FIELD_X_MIN, FIELD_X_MAX], [y, y], [0, 0], color="#cccccc", linewidth=0.4, alpha=0.5)

    # ---- 对每个区域：顶点排序 + 挤出体 ----
    for name, poly_pts in polygons_xyz.items():
        h = heights.get(name, np.mean([p[2] for p in poly_pts]))
        if h <= 0:
            continue

        # 按角度排序多边形顶点，保证侧面连续
        xy_center = np.mean([(p[0], p[1]) for p in poly_pts], axis=0)
        xy_with_angle = [(p[0], p[1], math.atan2(p[1] - xy_center[1], p[0] - xy_center[0])) for p in poly_pts]
        xy_sorted = sorted(xy_with_angle, key=lambda t: t[2])
        xy_ring = [(x, y) for x, y, _ in xy_sorted]
        if xy_ring[0] != xy_ring[-1]:
            xy_ring.append(xy_ring[0])

        fill_color, edge_color = region_colors(name)

        # 顶面
        top_verts = [(x, y, h) for (x, y) in xy_ring[:-1]]
        ax.add_collection3d(Poly3DCollection([top_verts], alpha=0.85, facecolor=fill_color,
                                             edgecolor=edge_color, linewidth=1.2, zorder=2))
        # 底面
        bottom_verts = [(x, y, 0) for (x, y) in xy_ring[:-1]]
        ax.add_collection3d(Poly3DCollection([bottom_verts], alpha=0.25, facecolor=fill_color,
                                             edgecolor="none", zorder=0))
        # 侧面
        for i in range(len(xy_ring) - 1):
            x1, y1 = xy_ring[i]
            x2, y2 = xy_ring[i + 1]
            quad_verts = [(x1, y1, 0), (x2, y2, 0), (x2, y2, h), (x1, y1, h)]
            ax.add_collection3d(Poly3DCollection([quad_verts], alpha=0.7, facecolor=fill_color,
                                                 edgecolor=edge_color, linewidth=0.8, zorder=1))

    # ---- 空心多边形（Top5点）处理：垂直支柱 + 地面阴影 ----
    hollow_arr = np.array(hollow_xyz)
    # 主体折线
    ax.plot(
        np.r_[hollow_arr[:, 0], hollow_arr[0, 0]],
        np.r_[hollow_arr[:, 1], hollow_arr[0, 1]],
        np.r_[hollow_arr[:, 2], hollow_arr[0, 2]],
        linestyle="--", linewidth=2.5, color="#2c2c2c",
        marker="o", markerfacecolor="white", markeredgewidth=1.8,
        markeredgecolor="#2c2c2c", markersize=6, zorder=5
    )
    # 每个点：垂直实线支柱 + 地面半透明圆盘
    for (x, y, z) in hollow_arr:
        ax.plot([x, x], [y, y], [0, z], color="#666666", linewidth=1.5, alpha=0.7, zorder=3)
        # 地面投影小圆盘
        u = np.linspace(0, 2 * np.pi, 20)
        circle_x = x + 0.08 * np.cos(u)
        circle_y = y + 0.08 * np.sin(u)
        circle_z = np.zeros_like(u)
        ax.plot_trisurf(circle_x, circle_y, circle_z, color="#888888", alpha=0.3, shade=False)
    # 标签略高于点
    for i, label in enumerate(hollow_labels):
        x, y, z = hollow_arr[i]
        ax.text(x, y, z + 0.12, label, fontsize=8, color="#111111",
                ha="center", va="bottom", weight="bold")

    # ---- 坐标轴与视图优化 ----
    all_heights = [heights.get(name, 0) for name in polygons_xyz.keys()] + list(hollow_arr[:, 2])
    max_z = max(all_heights) if all_heights else 1.0
    ax.set_zlim(0.0, max_z + 0.5)
    ax.set_xlim(FIELD_X_MIN, FIELD_X_MAX)
    ax.set_ylim(FIELD_Y_MIN, FIELD_Y_MAX)
    ax.set_xlabel("X (m)", fontsize=10, labelpad=6)
    ax.set_ylabel("Y (m)", fontsize=10, labelpad=6)
    ax.set_zlabel("Z (m)", fontsize=10, labelpad=8)
    ax.view_init(elev=32, azim=-56)   # 优化视角，便于观察点位置
    ax.set_box_aspect((FIELD_X_MAX - FIELD_X_MIN,
                       FIELD_Y_MAX - FIELD_Y_MIN,
                       max_z * 1.2))
    ax.set_xticks(np.arange(FIELD_X_MIN, FIELD_X_MAX + 1, 2.0))
    ax.set_yticks(np.arange(FIELD_Y_MIN, FIELD_Y_MAX + 1, 2.0))
    ax.tick_params(axis="both", labelsize=7, pad=1)
    ax.tick_params(axis="z", labelsize=7, pad=2)
    ax.grid(True, linestyle="--", linewidth=0.4, alpha=0.4)
    ax.set_title("Map Regions 3D (Extruded) + Top 5 Points with Pillars", fontsize=12, pad=20)

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    fig.savefig(output_path, bbox_inches="tight", dpi=150)
    plt.close(fig)
    print(f"Saved 3D visualization: {output_path}")


def build_output_paths(output_path):
    base, ext = os.path.splitext(output_path)
    if not ext:
        ext = ".png"
    return f"{base}_2d{ext}", f"{base}_3d{ext}"


def visualize(map_points_path, output_path, input_frame, self_color, show, strict):
    heights, regions = parse_map_points(map_points_path)
    if not regions:
        raise RuntimeError("No region polygons parsed from map_points file")

    polygons_xy, polygons_xyz = prepare_polygons(regions, input_frame)
    hollow_xy, hollow_xyz, hollow_labels = get_hollow_polygon(input_frame, self_color)

    validate_referee_bounds(polygons_xy, strict=strict)
    validate_referee_bounds({"Hollow_Top5": hollow_xy}, strict=strict)

    output_2d, output_3d = build_output_paths(output_path)
    draw_2d(polygons_xy, heights, hollow_xy, hollow_labels, output_2d)
    draw_3d(polygons_xyz, hollow_xyz, hollow_labels, heights, output_3d)

    if show:
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Visualize config/map/map_points.yaml with referee coordinate axes"
    )
    parser.add_argument(
        "--map-points",
        default="config/map/map_points.yaml",
        help="Path to map_points.yaml",
    )
    parser.add_argument(
        "--output",
        default="config/outputs/map_points_referee.png",
        help="Output image path",
    )
    parser.add_argument(
        "--input-frame",
        choices=["center", "referee"],
        default="referee",
        help="Coordinate frame of input map_points (default: referee)",
    )
    parser.add_argument(
        "--self-color",
        choices=["red", "blue"],
        default="red",
        help="Calibration side perspective for hollow points (default: red)",
    )
    parser.add_argument(
        "--no-strict",
        action="store_true",
        help="Do not fail on out-of-field points; print warnings instead",
    )
    parser.add_argument("--show", action="store_true", help="Show window after saving")
    args = parser.parse_args()

    visualize(
        args.map_points,
        args.output,
        args.input_frame,
        args.self_color,
        args.show,
        strict=(not args.no_strict),
    )


if __name__ == "__main__":
    main()