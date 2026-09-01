#!/usr/bin/env python3
import argparse
import math
import os
import re

import cv2
import numpy as np


FIELD_X_MIN = 0.0
FIELD_X_MAX = 28.0
FIELD_Y_MIN = 0.0
FIELD_Y_MAX = 15.0


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
    # Legacy center-like frame -> referee frame.
    # Keep for backward compatibility only.
    return x + 14.0, y + 5.85566


def build_canvas_xy(points_xy, width=1800, height=1050, margin=80):
    xs = [p[0] for p in points_xy]
    ys = [p[1] for p in points_xy]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    span_x = max(max_x - min_x, 1e-6)
    span_y = max(max_y - min_y, 1e-6)
    scale = min((width - 2 * margin) / span_x, (height - 2 * margin) / span_y)

    def to_px(x, y):
        px = int(round(margin + (x - min_x) * scale))
        py = int(round(height - margin - (y - min_y) * scale))
        return px, py

    return to_px, (min_x, max_x, min_y, max_y), scale


def draw_axes(img, to_px, bounds, input_frame):
    min_x, max_x, min_y, max_y = bounds
    color = (180, 180, 180)

    # Always draw referee-frame axes from referee origin (0, 0).
    x0, y0 = 0.0, 0.0
    x1, y1 = 28.0, 0.0
    x2, y2 = 0.0, 15.0

    if input_frame == "center":
        label = "Referee Frame: origin at field corner, x right, y up (input converted: center->referee)"
    else:
        label = "Referee Frame: origin at field corner, x right, y up"

    if min_x - 1 <= x0 <= max_x + 1 and min_y - 1 <= y0 <= max_y + 1:
        p0 = to_px(x0, y0)
        p1 = to_px(x1, y1)
        p2 = to_px(x2, y2)
        cv2.arrowedLine(img, p0, p1, color, 2, tipLength=0.02)
        cv2.arrowedLine(img, p0, p2, color, 2, tipLength=0.02)
        cv2.putText(img, "X", (p1[0] + 8, p1[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.putText(img, "Y", (p2[0] + 8, p2[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.putText(img, "(0,0)", (p0[0] + 8, p0[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

    if input_frame == "center":
        # Mark where the center-frame origin maps in referee coordinates.
        c = to_px(14.0, 7.5)
        cv2.circle(img, c, 4, (140, 140, 255), -1)
        cv2.putText(img, "center origin -> (14,7.5)", (c[0] + 10, c[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 255), 1)

    cv2.putText(img, label, (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (230, 230, 230), 2)


def draw_field_boundary(img, to_px):
    corners = np.array(
        [
            to_px(0.0, 0.0),
            to_px(28.0, 0.0),
            to_px(28.0, 15.0),
            to_px(0.0, 15.0),
        ],
        dtype=np.int32,
    )
    cv2.polylines(img, [corners], True, (120, 120, 120), 1)
    cv2.putText(img, "Field 28m x 15m", (corners[0][0] + 8, corners[0][1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (170, 170, 170), 1)


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

    polygons = {}
    all_points = []
    for name, pts in regions.items():
        xy = []
        for x, y, _ in pts:
            if input_frame == "center":
                x, y = to_referee_xy(x, y)
            xy.append((x, y))
            all_points.append((x, y))
        polygons[name] = xy

    validate_referee_bounds(polygons, strict=strict)

    # Keep full referee field in view for stable coordinate understanding.
    all_points.extend([(0.0, 0.0), (28.0, 0.0), (28.0, 15.0), (0.0, 15.0)])

    to_px, bounds, _ = build_canvas_xy(all_points)
    img = np.full((1050, 1800, 3), 30, dtype=np.uint8)

    draw_axes(img, to_px, bounds, input_frame)
    draw_field_boundary(img, to_px)

    for idx, (name, poly_xy) in enumerate(polygons.items()):
        hue = (37 * idx) % 180
        color = cv2.cvtColor(np.uint8([[[hue, 180, 230]]]), cv2.COLOR_HSV2BGR)[0][0]
        color = (int(color[0]), int(color[1]), int(color[2]))

        poly_px = np.array([to_px(x, y) for x, y in poly_xy], dtype=np.int32)
        cv2.fillPoly(img, [poly_px], color)
        cv2.polylines(img, [poly_px], True, (20, 20, 20), 2)

        cx, cy = polygon_centroid(poly_xy)
        tx, ty = to_px(cx, cy)
        h = heights.get(name, float("nan"))
        label = f"{name}  h={h:.2f}m" if not math.isnan(h) else name
        cv2.putText(img, label, (tx - 80, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    cv2.imwrite(output_path, img)
    print(f"Saved visualization: {output_path}")

    if show:
        cv2.imshow("map_points_visualization", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


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
