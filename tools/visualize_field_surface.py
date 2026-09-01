#!/usr/bin/env python3
"""渲染裁判系高密度场地高度图为 3D PNG。

作用：读 extract_field_surface.py 生成的高度图缓存，画出与
``config/outputs/map_points_referee_3d.png`` 同构的 3D 图，便于对齐检查。不会重新提取表面。
场地对称中心 (X=14, Y=7.5)。

用法：
    # 先一次性生成缓存：
    python3 tools/extract_field_surface.py
    # 再渲染：
    python3 tools/visualize_field_surface.py
"""

import argparse
import os
import pickle
import re

import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image, ImageChops


# ----- Field constants (referee frame) ----------------------------------
FIELD_X_MIN, FIELD_X_MAX = 0.0, 28.0
FIELD_Y_MIN, FIELD_Y_MAX = 0.0, 15.0
FIELD_CENTER = (14.0, 7.5)


# ----- I/O helpers ------------------------------------------------------
def load_surface(prefix):
    pkl = prefix + ".pkl"
    if os.path.exists(pkl):
        with open(pkl, "rb") as f:
            return pickle.load(f)
    npz = prefix + ".npz"
    if os.path.exists(npz):
        d = np.load(npz)
        return {"X": d["X"], "Y": d["Y"], "Z": d["Z"]}
    raise FileNotFoundError(f"no surface cache at {prefix}.pkl or {prefix}.npz; "
                            f"run extract_field_surface.py first")


def load_calibrate_points(yaml_path):
    with open(yaml_path) as f:
        content = f.read()
    if content.lstrip().startswith("%YAML"):
        content = "\n".join(content.splitlines()[1:])
    data = yaml.safe_load(content)
    return {k: list(v) for k, v in data.items() if isinstance(v, list)}


# ----- Plot -------------------------------------------------------------
CALIB_KEYS = ["self_fortress", "self_tower", "enemy_base", "enemy_tower", "cross_tower"]
CALIB_COLOR = "#c01c28"   # red side
CALIB_MARKER = {"self_fortress": "o", "self_tower": "^", "enemy_base": "s",
                "enemy_tower": "D", "cross_tower": "*"}


def add_reference_grid_lines(ax, z_min, z_max):
    """Draw a few clean reference lines on the ground plane and as faint
    vertical guides; these are *separate* lines, not a wireframe wall."""
    z0 = float(z_min)
    z_top = float(z_max)

    # Floor lines: X=14 (X-symmetry), X=15 (highland reference), X=28 (boundary)
    for x_pos, color, lw, ls in [
        (FIELD_X_MIN, "k", 1.2, "-"),
        (14.0,        "#2ca02c", 1.2, "--"),
        (15.0,        "#c01c28", 1.5, "--"),
        (FIELD_X_MAX, "k", 1.2, "-"),
    ]:
        ax.plot([x_pos, x_pos], [FIELD_Y_MIN, FIELD_Y_MAX], [z0, z0],
                color=color, linewidth=lw, linestyle=ls, alpha=0.85, zorder=5)
        # faint vertical guide
        ax.plot([x_pos, x_pos], [FIELD_Y_MIN, FIELD_Y_MIN], [z0, z_top],
                color=color, linewidth=0.7, linestyle=":", alpha=0.5)

    # Floor lines: Y=7.5 (Y-symmetry), Y=15 (boundary)
    for y_pos, color, lw, ls in [
        (FIELD_Y_MIN, "k", 1.2, "-"),
        (7.5,         "#1a5fb4", 1.5, "--"),
        (FIELD_Y_MAX, "k", 1.2, "-"),
    ]:
        ax.plot([FIELD_X_MIN, FIELD_X_MAX], [y_pos, y_pos], [z0, z0],
                color=color, linewidth=lw, linestyle=ls, alpha=0.85, zorder=5)

    # Mark the symmetry center
    ax.scatter([FIELD_CENTER[0]], [FIELD_CENTER[1]], [z0],
               c="#2ca02c", s=180, marker="x", linewidths=3.5,
               zorder=6, label=f"center ({FIELD_CENTER[0]}, {FIELD_CENTER[1]})")
    ax.text(FIELD_CENTER[0], FIELD_CENTER[1], z0 + 0.05,
            f"  ({FIELD_CENTER[0]}, {FIELD_CENTER[1]})",
            fontsize=8, color="#2ca02c", fontweight="bold", zorder=7)


def render(surface, calib, output_path, *, elev=25, azim=-52,
           cmap="terrain", show_calib_labels=True):
    X, Y, Z = surface["X"], surface["Y"], surface["Z"]
    # Clip to the official field [0, 28] x [0, 15] so the visual symmetry
    # center is exactly (14, 7.5). Cells outside the arena are set to NaN
    # (the cache still contains them for reference).
    Z = np.where(
        (X >= FIELD_X_MIN) & (X <= FIELD_X_MAX) &
        (Y >= FIELD_Y_MIN) & (Y <= FIELD_Y_MAX),
        Z, np.nan,
    )
    z_min = float(np.nanmin(Z))
    z_max = float(np.nanmax(Z))

    fig = plt.figure(figsize=(14, 6.5), dpi=150)
    ax = fig.add_subplot(111, projection="3d")
    ax.set_proj_type("persp")
    fig.subplots_adjust(left=-0.15, right=0.90, top=1.05, bottom=-0.12)

    # --- surface ---
    surf = ax.plot_surface(
        X, Y, Z, cmap=cmap, alpha=0.85,
        linewidth=0, antialiased=True,
        rcount=140, ccount=240, shade=True, zorder=1,
    )
    cbar = fig.colorbar(surf, ax=ax, shrink=0.5, aspect=20, pad=0.06)
    cbar.set_label("Z (m)", rotation=270, labelpad=14)

    # --- reference lines (X=14/15/28, Y=7.5/15, etc.) ---
    add_reference_grid_lines(ax, z_min - 0.05, z_max + 0.4)

    # --- calibration points (red side) ---
    # Per-point label offsets (dx, dy, dz, ha) to avoid overlap
    label_offsets = {
        "self_fortress": (0, -1.2, 0.15, "center"),
        "self_tower":    (0,  1.0, 0.25, "center"),
        "enemy_base":    (0, -0.8, 0.15, "center"),
        "enemy_tower":   (0,  0.8, 0.15, "center"),
        "cross_tower":   (0,  0.8, 0.15, "center"),
    }
    for key in CALIB_KEYS:
        if key not in calib:
            continue
        pt = np.asarray(calib[key], dtype=float).reshape(-1)[:3]
        ax.scatter(pt[0], pt[1], pt[2], c=CALIB_COLOR,
                   marker=CALIB_MARKER.get(key, "o"),
                   s=140, edgecolors="black", linewidth=1.2,
                   zorder=8, label=key)
        if show_calib_labels:
            dx, dy, dz, ha = label_offsets.get(key, (0, 0, 0.18, "center"))
            ax.text(pt[0] + dx, pt[1] + dy, pt[2] + dz, key,
                    fontsize=8, color=CALIB_COLOR, ha=ha,
                    fontweight="bold", zorder=9)

    # --- axes / layout to match map_points_referee_3d.png ---
    ax.set_xlim(FIELD_X_MIN, FIELD_X_MAX)
    ax.set_ylim(FIELD_Y_MIN, FIELD_Y_MAX)
    ax.set_zlim(0.0, max(z_max + 0.4, 2.2))
    AXIS_X_COLOR = "#d62728"   # red
    AXIS_Y_COLOR = "#2ca02c"   # green
    AXIS_Z_COLOR = "#1f77b4"   # blue

    ax.set_xlabel("X (m)", fontsize=13, fontweight="bold", color=AXIS_X_COLOR, labelpad=6)
    ax.set_ylabel("Y (m)", fontsize=13, fontweight="bold", color=AXIS_Y_COLOR, labelpad=6)
    ax.set_zlabel("Z (m)", fontsize=13, fontweight="bold", color=AXIS_Z_COLOR, labelpad=6)
    ax.set_xticks(np.arange(FIELD_X_MIN, FIELD_X_MAX + 0.1, 1.0))
    ax.set_yticks(np.arange(FIELD_Y_MIN, FIELD_Y_MAX + 0.1, 1.0))
    ax.tick_params(axis="x", labelsize=6, pad=1, colors=AXIS_X_COLOR)
    ax.tick_params(axis="y", labelsize=6, pad=1, colors=AXIS_Y_COLOR)
    ax.tick_params(axis="z", labelsize=7, pad=2, colors=AXIS_Z_COLOR)

    # Light, long colored axis arrows anchored at the true origin (0, 0, 0).
    z_top = max(z_max + 0.6, 2.4)
    x_end = FIELD_X_MAX + 1.5
    y_end = FIELD_Y_MAX + 1.5

    axis_kw = dict(linewidth=1.6, alpha=0.55, zorder=12, clip_on=False)
    ax.plot([0.0, x_end], [0.0, 0.0], [0.0, 0.0],
            color=AXIS_X_COLOR, **axis_kw)
    ax.plot([0.0, 0.0], [0.0, y_end], [0.0, 0.0],
            color=AXIS_Y_COLOR, **axis_kw)
    ax.plot([0.0, 0.0], [0.0, 0.0], [0.0, z_top],
            color=AXIS_Z_COLOR, **axis_kw)
    # Slim arrow heads at axis ends
    head_kw = dict(s=80, zorder=13, clip_on=False, alpha=0.75)
    ax.scatter([x_end], [0.0], [0.0], c=AXIS_X_COLOR, marker=">", **head_kw)
    ax.scatter([0.0], [y_end], [0.0], c=AXIS_Y_COLOR, marker="<", **head_kw)
    ax.scatter([0.0], [0.0], [z_top], c=AXIS_Z_COLOR, marker="^", **head_kw)
    # Mark the true origin (0, 0, 0)
    ax.scatter([0.0], [0.0], [0.0], c="black", marker="o", s=60,
               zorder=14, edgecolors="white", linewidths=1.0)
    ax.text(0.0, 0.0, 0.0, "  O(0,0,0)", fontsize=8, fontweight="bold",
            color="black", zorder=15)
    ax.grid(True, linestyle="--", linewidth=0.35, alpha=0.35)
    ax.view_init(elev=elev, azim=azim)
    ax.set_box_aspect((28.0, 15.0, 5.0))
    ax.set_title("RMUC2026 Field Ground Surface", fontsize=10, pad=-14)
    ax.legend(loc="upper left", fontsize=6, framealpha=0.8,
             borderpad=0.3, handletextpad=0.3, labelspacing=0.2)

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    fig.savefig(output_path, facecolor="white", bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)

    # Auto-crop whitespace (threshold-based to ignore faint grid lines)
    img = Image.open(output_path).convert("RGB")
    arr = np.array(img)
    # Treat pixels with all channels > 245 as "white"
    non_white = np.any(arr < 245, axis=2)
    rows = np.any(non_white, axis=1)
    cols = np.any(non_white, axis=0)
    if rows.any() and cols.any():
        r0, r1 = np.where(rows)[0][[0, -1]]
        c0, c1 = np.where(cols)[0][[0, -1]]
        h = r1 - r0
        # Trim top extra 1/5 of content height (empty perspective space)
        r0 = min(r0 + h // 5, r1 - 10)
        pad_top, pad_bot, pad_left, pad_right = 4, 8, 8, 40
        r0 = max(0, r0 - pad_top)
        c0 = max(0, c0 - pad_left)
        r1 = min(arr.shape[0], r1 + pad_bot)
        c1 = min(arr.shape[1], c1 + pad_right)
        Image.fromarray(arr[r0:r1, c0:c1]).save(output_path)
    print(f"saved: {output_path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument(
        "--surface",
        default="/home/zst/T/config/outputs/ground_surface_dense",
        help="Surface cache prefix (without extension).",
    )
    ap.add_argument(
        "--calib",
        default="/home/zst/T/config/red/calibrate_points_red.yaml",
        help="Calibration points YAML (red-as-self).",
    )
    ap.add_argument(
        "--out",
        default="/home/zst/T/config/outputs/ground_surface_dense.png",
    )
    ap.add_argument("--elev", type=float, default=24.0)
    ap.add_argument("--azim", type=float, default=-52.0)
    args = ap.parse_args()

    surface = load_surface(args.surface)
    calib = load_calibrate_points(args.calib)
    render(surface, calib, args.out, elev=args.elev, azim=args.azim)


if __name__ == "__main__":
    main()
