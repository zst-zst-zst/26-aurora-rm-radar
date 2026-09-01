#!/usr/bin/env python3
"""
从现场录制的 rosbag 构建 map.pcd（用于 GICP 定位）。

用法:
    python3 tools/build_map_from_bag.py <bag_dir> <output_pcd> [--voxel 0.05] [--topic /livox/lidar]

示例:
    python3 tools/build_map_from_bag.py bags/empty_field config/map/map.pcd
    python3 tools/build_map_from_bag.py bags/empty_field config/map/map.pcd --voxel 0.03

流程:
    1. 读取 rosbag 中的 PointCloud2 消息
    2. 合并所有帧
    3. 体素降采样 (默认 0.05m)
    4. 估算法线 (GICP 需要)
    5. 保存为 PCD (binary)
"""

import argparse
import sys
import struct
import numpy as np

def main():
    parser = argparse.ArgumentParser(description='Build map.pcd from rosbag')
    parser.add_argument('bag_dir', help='Path to rosbag directory')
    parser.add_argument('output_pcd', help='Output PCD file path')
    parser.add_argument('--voxel', type=float, default=0.05,
                        help='Voxel size for downsampling (default: 0.05m)')
    parser.add_argument('--topic', default='/livox/lidar',
                        help='Point cloud topic name (default: /livox/lidar)')
    parser.add_argument('--max-frames', type=int, default=0,
                        help='Max frames to use (0 = all)')
    parser.add_argument('--filter-robots', action='store_true',
                        help='Filter out starting zones (where robots are parked before match)')
    args = parser.parse_args()

    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import PointCloud2
        import sensor_msgs_py.point_cloud2 as pc2
    except ImportError:
        print("ERROR: ROS2 packages not found. Source ROS2 first:")
        print("  source /opt/ros/jazzy/setup.bash")
        sys.exit(1)

    # --- 1. Read rosbag ---
    print(f"Reading rosbag: {args.bag_dir}")
    print(f"Topic: {args.topic}")

    storage_options = rosbag2_py.StorageOptions(uri=args.bag_dir, storage_id='')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Filter to our topic
    filter_ = rosbag2_py.StorageFilter(topics=[args.topic])
    reader.set_filter(filter_)

    all_points = []
    frame_count = 0

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic != args.topic:
            continue

        msg = deserialize_message(data, PointCloud2)
        points = np.array(list(pc2.read_points(msg, field_names=('x', 'y', 'z'),
                                                skip_nans=True)),
                          dtype=np.float32)
        if len(points) > 0:
            all_points.append(points)
            frame_count += 1

        if args.max_frames > 0 and frame_count >= args.max_frames:
            break

        if frame_count % 50 == 0:
            total = sum(len(p) for p in all_points)
            print(f"  Frame {frame_count}: {total:,} points so far")

    if not all_points:
        print(f"ERROR: No point cloud messages found on topic '{args.topic}'")
        sys.exit(1)

    merged = np.vstack(all_points)
    print(f"Merged {frame_count} frames → {len(merged):,} points")

    # --- Filter starting zones (robots parked before match) ---
    if args.filter_robots:
        print("Filtering starting zones (robot parking areas)...")
        before = len(merged)
        # Red starting zone: hexagonal area around red base
        # Approximate as bounding box x=[0.5, 4.5], y=[5.2, 9.8]
        # Blue starting zone: symmetric x=[23.5, 27.5], y=[5.2, 9.8]
        # Only filter points above ground (z > 0.15) to keep floor
        # Only filter robot-height points (z < 1.5) to keep base structure above
        red_start = (
            (merged[:, 0] > 0.5) & (merged[:, 0] < 4.5) &
            (merged[:, 1] > 5.2) & (merged[:, 1] < 9.8) &
            (merged[:, 2] > 0.15) & (merged[:, 2] < 1.5)
        )
        blue_start = (
            (merged[:, 0] > 23.5) & (merged[:, 0] < 27.5) &
            (merged[:, 1] > 5.2) & (merged[:, 1] < 9.8) &
            (merged[:, 2] > 0.15) & (merged[:, 2] < 1.5)
        )
        merged = merged[~red_start & ~blue_start]
        print(f"  Removed {before - len(merged):,} points from starting zones")

    # --- 2. Voxel downsample ---
    print(f"Voxel downsampling ({args.voxel}m)...")
    voxel = args.voxel
    quantized = (merged / voxel).astype(np.int32)
    _, unique_idx = np.unique(quantized, axis=0, return_index=True)

    # Average within each voxel for smoother result
    voxel_keys = quantized[unique_idx]
    downsampled = []
    key_to_idx = {}
    for i, key in enumerate(quantized):
        k = tuple(key)
        if k not in key_to_idx:
            key_to_idx[k] = []
        key_to_idx[k].append(i)

    result_points = np.zeros((len(key_to_idx), 3), dtype=np.float32)
    for i, (k, indices) in enumerate(key_to_idx.items()):
        result_points[i] = merged[indices].mean(axis=0)

    print(f"Downsampled: {len(merged):,} → {len(result_points):,} points")

    # --- 3. Estimate normals ---
    print("Estimating normals (KNN=20)...")
    try:
        from scipy.spatial import cKDTree
    except ImportError:
        print("WARNING: scipy not available, using zero normals")
        normals = np.zeros_like(result_points)
    else:
        tree = cKDTree(result_points)
        normals = np.zeros_like(result_points)
        k = min(20, len(result_points))

        batch_size = 10000
        for start in range(0, len(result_points), batch_size):
            end = min(start + batch_size, len(result_points))
            batch = result_points[start:end]
            _, idx = tree.query(batch, k=k)

            for j in range(end - start):
                neighbors = result_points[idx[j]]
                centroid = neighbors.mean(axis=0)
                cov = (neighbors - centroid).T @ (neighbors - centroid) / k
                try:
                    eigenvalues, eigenvectors = np.linalg.eigh(cov)
                    normals[start + j] = eigenvectors[:, 0]  # smallest eigenvalue
                except np.linalg.LinAlgError:
                    normals[start + j] = [0, 0, 1]

            if start % 50000 == 0 and start > 0:
                print(f"  {start:,}/{len(result_points):,} normals computed")

    # Normalize
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    norms[norms < 1e-6] = 1.0
    normals = normals / norms

    # --- 4. Save PCD ---
    print(f"Saving to {args.output_pcd}...")
    n = len(result_points)
    header = (
        f"# .PCD v0.7 - Point Cloud Data file format\n"
        f"VERSION 0.7\n"
        f"FIELDS x y z normal_x normal_y normal_z\n"
        f"SIZE 4 4 4 4 4 4\n"
        f"TYPE F F F F F F\n"
        f"COUNT 1 1 1 1 1 1\n"
        f"WIDTH {n}\n"
        f"HEIGHT 1\n"
        f"VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        f"DATA binary\n"
    )

    import os
    os.makedirs(os.path.dirname(os.path.abspath(args.output_pcd)), exist_ok=True)

    with open(args.output_pcd, 'wb') as f:
        f.write(header.encode('ascii'))
        for i in range(n):
            f.write(struct.pack('ffffff',
                                result_points[i, 0], result_points[i, 1], result_points[i, 2],
                                normals[i, 0], normals[i, 1], normals[i, 2]))

    file_size = os.path.getsize(args.output_pcd) / (1024 * 1024)
    print(f"\nDone! {args.output_pcd}")
    print(f"  Points: {n:,}")
    print(f"  Size: {file_size:.1f} MB")
    print(f"  Fields: x y z normal_x normal_y normal_z")
    print(f"  Voxel: {args.voxel}m")
    x_range = result_points[:, 0]
    y_range = result_points[:, 1]
    print(f"  X: [{x_range.min():.2f}, {x_range.max():.2f}]")
    print(f"  Y: [{y_range.min():.2f}, {y_range.max():.2f}]")


if __name__ == '__main__':
    main()
