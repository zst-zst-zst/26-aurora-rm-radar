#!/usr/bin/env python3
import argparse
import math
import statistics
from collections import defaultdict

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2


ROBOT_NAMES = ["hero", "engineer", "infantry3", "infantry4", "sentry"]


def open_reader(bag_path):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="")
    converter_options = rosbag2_py.ConverterOptions("cdr", "cdr")
    reader.open(storage_options, converter_options)
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, topic_types


def detect_points(msg, stamp_ns, camera_topic):
    points = []
    for color, xs, ys in (("blue", msg.blue_x, msg.blue_y), ("red", msg.red_x, msg.red_y)):
        for slot, (x, y) in enumerate(zip(xs, ys)):
            if math.isfinite(x) and math.isfinite(y) and x > 0.0 and y > 0.0:
                points.append({
                    "t": stamp_ns,
                    "topic": camera_topic,
                    "color": color,
                    "slot": slot,
                    "robot": ROBOT_NAMES[slot] if slot < len(ROBOT_NAMES) else str(slot),
                    "x": float(x),
                    "y": float(y),
                })
    return points


def cloud_points(msg, max_points=200000):
    pts = []
    for p in point_cloud2.read_points(msg, field_names=("x", "y"), skip_nans=True):
        x = float(p[0])
        y = float(p[1])
        if math.isfinite(x) and math.isfinite(y):
            pts.append((x, y))
            if len(pts) >= max_points:
                break
    if not pts:
        return np.empty((0, 2), dtype=np.float32)
    return np.asarray(pts, dtype=np.float32)


def nearest_distance(point_xy, cloud_xy):
    if cloud_xy.size == 0:
        return None
    dx = cloud_xy[:, 0] - point_xy[0]
    dy = cloud_xy[:, 1] - point_xy[1]
    d2 = dx * dx + dy * dy
    return float(np.sqrt(np.min(d2)))


def percentile(values, pct):
    if not values:
        return None
    arr = sorted(values)
    k = (len(arr) - 1) * pct / 100.0
    lo = int(math.floor(k))
    hi = int(math.ceil(k))
    if lo == hi:
        return arr[lo]
    return arr[lo] * (hi - k) + arr[hi] * (k - lo)


def summarize(label, rows, thresholds):
    dists = [r["dist"] for r in rows if r["dist"] is not None]
    print(f"\n## {label}")
    print(f"camera_points: {len(rows)}")
    print(f"with_lidar_sample: {len(dists)}")
    if not dists:
        print("no matched lidar samples in time window")
        return
    print(f"mean_dist_m: {statistics.fmean(dists):.3f}")
    print(f"median_dist_m: {percentile(dists, 50):.3f}")
    print(f"p90_dist_m: {percentile(dists, 90):.3f}")
    print(f"p95_dist_m: {percentile(dists, 95):.3f}")
    print(f"max_dist_m: {max(dists):.3f}")
    for th in thresholds:
        hit = sum(1 for d in dists if d <= th)
        print(f"overlap_<=_{th:.2f}m: {hit}/{len(rows)} = {hit / max(1, len(rows)) * 100:.1f}%")


def main():
    parser = argparse.ArgumentParser(description="Compute LiDAR-camera overlap from a ROS2 bag.")
    parser.add_argument("bag", help="Path to ROS2 bag directory")
    parser.add_argument("--camera-topic", default="/resolve_result")
    parser.add_argument("--lidar-topic", default="/livox/lidar_cluster")
    parser.add_argument("--fallback-lidar-topic", default="/kalman_detect")
    parser.add_argument("--time-window-ms", type=float, default=120.0)
    parser.add_argument("--thresholds", type=float, nargs="+", default=[0.5, 0.8, 1.0])
    parser.add_argument("--segment-sec", type=float, default=10.0)
    args = parser.parse_args()

    reader, topic_types = open_reader(args.bag)
    lidar_topic = args.lidar_topic
    if lidar_topic not in topic_types and args.fallback_lidar_topic in topic_types:
        lidar_topic = args.fallback_lidar_topic
    if args.camera_topic not in topic_types:
        raise RuntimeError(f"camera topic not found: {args.camera_topic}")
    if lidar_topic not in topic_types:
        raise RuntimeError(f"lidar topic not found: {args.lidar_topic} or {args.fallback_lidar_topic}")

    camera_type = get_message(topic_types[args.camera_topic])
    lidar_type = get_message(topic_types[lidar_topic])
    topics = {args.camera_topic, lidar_topic}

    camera_msgs = []
    lidar_msgs = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic not in topics:
            continue
        if topic == args.camera_topic:
            msg = deserialize_message(data, camera_type)
            stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
            if stamp_ns <= 0:
                stamp_ns = int(t)
            pts = detect_points(msg, stamp_ns, args.camera_topic)
            if pts:
                camera_msgs.extend(pts)
        elif topic == lidar_topic:
            msg = deserialize_message(data, lidar_type)
            stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
            if stamp_ns <= 0:
                stamp_ns = int(t)
            if topic_types[lidar_topic] == "vision_interface/msg/DetectResult":
                pts = detect_points(msg, stamp_ns, lidar_topic)
                arr = np.asarray([(p["x"], p["y"]) for p in pts], dtype=np.float32)
                if arr.size == 0:
                    arr = np.empty((0, 2), dtype=np.float32)
                lidar_msgs.append((stamp_ns, arr))
            else:
                lidar_msgs.append((stamp_ns, cloud_points(msg)))

    if not camera_msgs:
        raise RuntimeError("no valid camera points")
    if not lidar_msgs:
        raise RuntimeError("no lidar messages")

    lidar_times = np.asarray([t for t, _ in lidar_msgs], dtype=np.int64)
    window_ns = int(args.time_window_ms * 1e6)
    rows = []
    for p in camera_msgs:
        t = p["t"]
        idx = int(np.searchsorted(lidar_times, t))
        candidates = []
        if idx < len(lidar_msgs):
            candidates.append(idx)
        if idx > 0:
            candidates.append(idx - 1)
        best_idx = None
        best_dt = None
        for ci in candidates:
            dt = abs(int(lidar_times[ci]) - int(t))
            if best_dt is None or dt < best_dt:
                best_dt = dt
                best_idx = ci
        row = dict(p)
        row["dt_ms"] = None
        row["dist"] = None
        if best_idx is not None and best_dt is not None and best_dt <= window_ns:
            row["dt_ms"] = best_dt / 1e6
            row["dist"] = nearest_distance((p["x"], p["y"]), lidar_msgs[best_idx][1])
        rows.append(row)

    print(f"# LiDAR-Camera Overlap Report")
    print(f"bag: {args.bag}")
    print(f"camera_topic: {args.camera_topic}")
    print(f"lidar_topic: {lidar_topic}")
    print(f"time_window_ms: {args.time_window_ms:.1f}")
    print(f"thresholds_m: {', '.join(f'{x:.2f}' for x in args.thresholds)}")
    summarize("overall", rows, args.thresholds)

    by_slot = defaultdict(list)
    for r in rows:
        by_slot[(r["color"], r["slot"], r["robot"])].append(r)
    for key in sorted(by_slot.keys()):
        color, slot, robot = key
        summarize(f"{color}[{slot}] {robot}", by_slot[key], args.thresholds)

    t0 = min(r["t"] for r in rows)
    by_segment = defaultdict(list)
    seg_ns = int(args.segment_sec * 1e9)
    if seg_ns > 0:
        for r in rows:
            seg = int((r["t"] - t0) // seg_ns)
            by_segment[seg].append(r)
        for seg in sorted(by_segment.keys()):
            lo = seg * args.segment_sec
            hi = (seg + 1) * args.segment_sec
            summarize(f"time {lo:.0f}-{hi:.0f}s", by_segment[seg], args.thresholds)


if __name__ == "__main__":
    main()
