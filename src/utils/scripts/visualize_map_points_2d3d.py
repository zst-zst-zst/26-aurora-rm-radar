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

BLUE_CALIB_POINTS = [
    ("B1_self_fortress", (20.06790, 7.46930, 0.00020)),
    ("B2_self_tower", (17.17800, 11.33570, 1.86860)),
    ("B3_enemy_base", (4.98290, 10.54700, 0.60000)),
    ("B4_enemy_tower", (3.45860, 3.91990, 0.73180)),
    ("B5_cross_tower", (10.82200, 3.66430, 1.86860)),
]

RED_CALIB_POINTS = [
    ("R1_self_fortress", (7.93210, 7.53070, 0.00000)),
    ("R2_self_tower", (10.82200, 3.66430, 1.86840)),
    ("R3_enemy_base", (23.01710, 4.45300, 0.59980)),
    ("R4_enemy_tower", (24.54140, 11.08010, 0.73160)),
    ("R5_cross_tower", (17.17800, 11.33570, 1.86840)),
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


def get_polygon_points(points, input_frame):
    poly_xy = []
    poly_xyz = []
    labels = []
    for label, (x, y, z) in points:
        if input_frame == "center":
            x, y = to_referee_xy(x, y)
        poly_xy.append((x, y))
        poly_xyz.append((x, y, z))
        labels.append(label)
    return poly_xy, poly_xyz, labels


def sort_polygon_points_xy(points_xyz, labels):
    points = np.array(points_xyz, dtype=float)
    center = np.mean(points[:, :2], axis=0)
    angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    order = np.argsort(angles)

    sorted_points = [tuple(points_xyz[i]) for i in order]
    sorted_labels = [labels[i] for i in order]
    return sorted_points, sorted_labels


def draw_2d(polygons_xy, heights, blue_xy, blue_labels, red_xy, red_labels, output_path):
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

    blue_arr = np.array(blue_xy)
    ax.plot(
        np.r_[blue_arr[:, 0], blue_arr[0, 0]],
        np.r_[blue_arr[:, 1], blue_arr[0, 1]],
        linestyle="--",
        linewidth=2.0,
        color="#1a5fb4",
        marker="o",
        markerfacecolor="none",
        markeredgewidth=1.5,
        markeredgecolor="#1a5fb4",
    )
    for i, label in enumerate(blue_labels):
        ax.text(blue_arr[i, 0], blue_arr[i, 1] + 0.2, label, fontsize=8, color="#1a5fb4")

    red_arr = np.array(red_xy)
    ax.plot(
        np.r_[red_arr[:, 0], red_arr[0, 0]],
        np.r_[red_arr[:, 1], red_arr[0, 1]],
        linestyle="--",
        linewidth=2.0,
        color="#c01c28",
        marker="o",
        markerfacecolor="none",
        markeredgewidth=1.5,
        markeredgecolor="#c01c28",
    )
    for i, label in enumerate(red_labels):
        ax.text(red_arr[i, 0], red_arr[i, 1] + 0.2, label, fontsize=8, color="#c01c28")

    ax.set_xlim(FIELD_X_MIN, FIELD_X_MAX)
    ax.set_ylim(FIELD_Y_MIN, FIELD_Y_MAX)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(alpha=0.25)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Map Regions 2D (Filled) + Blue/Red Calibration 5-Point Polygons")

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved 2D visualization: {output_path}")


def draw_3d(polygons_xyz, blue_xyz, blue_labels, red_xyz, red_labels, output_path):
    fig = plt.figure(figsize=(14, 8), dpi=150)
    ax = fig.add_subplot(111, projection="3d")
    ax.set_proj_type("persp")

    floor_x, floor_y = np.meshgrid(
        np.linspace(FIELD_X_MIN, FIELD_X_MAX, 24),
        np.linspace(FIELD_Y_MIN, FIELD_Y_MAX, 16),
    )
    floor_z = np.zeros_like(floor_x)
    ax.plot_surface(
        floor_x,
        floor_y,
        floor_z,
        color="#f2f2f2",
        alpha=0.12,
        linewidth=0,
        shade=False,
        zorder=0,
    )

    for name, poly_xyz in polygons_xyz.items():
        fill_color, edge_color = region_colors(name)
        poly = Poly3DCollection([poly_xyz], alpha=0.68, facecolor=fill_color, edgecolor=edge_color, linewidth=1.0)
        poly.set_zsort("average")
        ax.add_collection3d(poly)

    blue_arr = np.array(blue_xyz)
    ax.plot(
        np.r_[blue_arr[:, 0], blue_arr[0, 0]],
        np.r_[blue_arr[:, 1], blue_arr[0, 1]],
        np.r_[blue_arr[:, 2], blue_arr[0, 2]],
        linestyle="--",
        linewidth=2.0,
        color="#1a5fb4",
        marker="o",
        markerfacecolor="none",
        markeredgecolor="#1a5fb4",
    )
    for i, label in enumerate(blue_labels):
        ax.text(blue_arr[i, 0], blue_arr[i, 1], blue_arr[i, 2] + 0.05, label, fontsize=7, color="#1a5fb4")

    red_arr = np.array(red_xyz)
    ax.plot(
        np.r_[red_arr[:, 0], red_arr[0, 0]],
        np.r_[red_arr[:, 1], red_arr[0, 1]],
        np.r_[red_arr[:, 2], red_arr[0, 2]],
        linestyle="--",
        linewidth=2.0,
        color="#c01c28",
        marker="o",
        markerfacecolor="none",
        markeredgecolor="#c01c28",
    )
    for i, label in enumerate(red_labels):
        ax.text(red_arr[i, 0], red_arr[i, 1], red_arr[i, 2] + 0.05, label, fontsize=7, color="#c01c28")

    ax.set_xlim(FIELD_X_MIN, FIELD_X_MAX)
    ax.set_ylim(FIELD_Y_MIN, FIELD_Y_MAX)
    max_z = float(max(max(p[2] for p in poly) for poly in polygons_xyz.values()))
    max_z = max(max_z, float(np.max(blue_arr[:, 2])), float(np.max(red_arr[:, 2])))
    ax.set_zlim(0.0, max_z + 0.4)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("Map Regions 3D (Filled) + Blue/Red Calibration 5-Point Polygons")
    ax.view_init(elev=24, azim=-52)
    ax.set_xticks(np.arange(FIELD_X_MIN, FIELD_X_MAX + 0.1, 1.0))
    ax.set_yticks(np.arange(FIELD_Y_MIN, FIELD_Y_MAX + 0.1, 1.0))
    ax.tick_params(axis="x", labelsize=6, pad=1)
    ax.tick_params(axis="y", labelsize=6, pad=1)
    ax.tick_params(axis="z", labelsize=7, pad=2)
    ax.grid(True, linestyle="--", linewidth=0.35, alpha=0.35)
    # Keep field plane readable while restoring stronger 3D perception.
    # This only affects rendering ratio, not the underlying coordinates.
    ax.set_box_aspect((28.0, 15.0, 7.5))

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved 3D visualization: {output_path}")


def build_output_paths(output_path):
    base, ext = os.path.splitext(output_path)
    if not ext:
        ext = ".png"
    return f"{base}_2d{ext}", f"{base}_3d{ext}"


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


def visualize(map_points_path, output_path, input_frame, show, strict):
    heights, regions = parse_map_points(map_points_path)
    if not regions:
        raise RuntimeError("No region polygons parsed from map_points file")

    polygons_xy, polygons_xyz = prepare_polygons(regions, input_frame)
    blue_xy, blue_xyz, blue_labels = get_polygon_points(BLUE_CALIB_POINTS, input_frame)
    red_xy, red_xyz, red_labels = get_polygon_points(RED_CALIB_POINTS, input_frame)

    validate_referee_bounds(polygons_xy, strict=strict)
    validate_referee_bounds({"Blue_Calib_Top5": blue_xy}, strict=strict)
    validate_referee_bounds({"Red_Calib_Top5": red_xy}, strict=strict)

    output_2d, output_3d = build_output_paths(output_path)
    draw_2d(polygons_xy, heights, blue_xy, blue_labels, red_xy, red_labels, output_2d)
    draw_3d(polygons_xyz, blue_xyz, blue_labels, red_xyz, red_labels, output_3d)

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
        args.show,
        strict=(not args.no_strict),
    )


if __name__ == "__main__":
    main()
