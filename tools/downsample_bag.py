#!/usr/bin/env python3
"""
对 rosbag 中的 /livox/lidar 点云做体素降采样，使其点数接近 Mid-70 (~10000 pts/frame)。

用法:
    python3 tools/downsample_bag.py <输入包路径> [输出包路径]

如果未指定输出路径，默认在输入包同级目录创建 <原名>_mid70 文件夹。
"""

import sys
import os
import shutil
import numpy as np
from pathlib import Path

import rosbag2_py
from rclpy.serialization import serialize_message, deserialize_message
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import struct

# ── 目标参数 ──────────────────────────────────────────────────────────
TARGET_POINTS_PER_FRAME = 10000  # Mid-70 典型每帧点数
VOXEL_SIZE = 0.12                # 体素大小(m)，根据 24000->10000 粗略估算
#  24k pts in ~70.4° FOV / 4m range ≈ 0.10-0.15 voxel works
# ──────────────────────────────────────────────────────────────────────


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """体素降采样: 返回采样后的 N×3 数组"""
    if len(points) == 0:
        return points
    # 计算体素索引
    indices = np.floor(points[:, :3] / voxel_size).astype(np.int64)
    # 用结构化数组做 unique (快速)
    dtype = np.dtype([('x', np.int64), ('y', np.int64), ('z', np.int64)])
    structured = np.empty(len(indices), dtype=dtype)
    structured['x'] = indices[:, 0]
    structured['y'] = indices[:, 1]
    structured['z'] = indices[:, 2]
    _, unique_idx = np.unique(structured, return_index=True)
    return points[unique_idx]


def downsample_one_msg(msg: PointCloud2, target: int) -> PointCloud2:
    """将一帧点云降采样到接近 target 个点，保留所有原始字段"""
    # 读取所有字段
    field_names = [f.name for f in msg.fields]
    points_list = list(pc2.read_points(msg, field_names=field_names, skip_nans=False))
    if not points_list:
        return msg
    # structured array -> plain 2D float array
    all_points = np.array([tuple(p) for p in points_list], dtype=np.float32)
    if len(all_points) == 0:
        return msg

    # Step 1: 体素降采样 (保持空间分布)
    points_xyz = all_points[:, :3]
    voxel_idx = np.floor(points_xyz / VOXEL_SIZE).astype(np.int64)
    dtype_s = np.dtype([('x', np.int64), ('y', np.int64), ('z', np.int64)])
    structured = np.empty(len(voxel_idx), dtype=dtype_s)
    structured['x'] = voxel_idx[:, 0]
    structured['y'] = voxel_idx[:, 1]
    structured['z'] = voxel_idx[:, 2]
    _, unique_idx = np.unique(structured, return_index=True)
    sampled = all_points[unique_idx]

    # Step 2: 如果还是超过目标，随机采样
    if len(sampled) > target:
        idx = np.random.choice(len(sampled), target, replace=False)
        sampled = sampled[idx]

    # Step 3: 重建 PointCloud2 消息
    new_msg = PointCloud2()
    new_msg.header = msg.header
    new_msg.height = 1
    new_msg.width = len(sampled)
    new_msg.fields = msg.fields
    new_msg.is_bigendian = msg.is_bigendian
    new_msg.point_step = msg.point_step
    new_msg.row_step = new_msg.point_step * new_msg.width
    new_msg.is_dense = msg.is_dense

    # 序列化点数据
    buffer = bytearray()
    for pt in sampled:
        for fi, f in enumerate(msg.fields):
            val = pt[fi] if isinstance(pt, (list, tuple, np.ndarray)) else pt[f.name]
            if f.datatype == PointField.FLOAT32:
                buffer.extend(struct.pack('<f', float(val)))
            elif f.datatype == PointField.FLOAT64:
                buffer.extend(struct.pack('<d', float(val)))
            elif f.datatype in (PointField.UINT8, PointField.INT8):
                buffer.extend(struct.pack('<b', int(val)))
            elif f.datatype in (PointField.UINT16, PointField.INT16):
                buffer.extend(struct.pack('<h', int(val)))
            elif f.datatype == PointField.UINT32:
                buffer.extend(struct.pack('<I', int(val)))
            elif f.datatype == PointField.INT32:
                buffer.extend(struct.pack('<i', int(val)))
            else:
                buffer.extend(struct.pack('<f', float(val)))
    new_msg.data = bytes(buffer)

    return new_msg


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    in_path = sys.argv[1]
    if not os.path.isdir(in_path):
        # 也支持直接传 mcap 文件所在目录
        in_path = in_path

    if len(sys.argv) >= 3:
        out_path = sys.argv[2]
    else:
        out_path = in_path.rstrip('/') + '_mid70'

    if os.path.exists(out_path):
        print(f"错误: 输出路径 {out_path} 已存在，请先删除或指定其他路径")
        sys.exit(1)

    print(f"输入: {in_path}")
    print(f"输出: {out_path}")
    print(f"体素大小: {VOXEL_SIZE}m, 目标点数: {TARGET_POINTS_PER_FRAME}")

    # ── 读取输入包 ────────────────────────────────────────────────────
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=in_path, storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))

    topic_types = {}
    for t in reader.get_all_topics_and_types():
        topic_types[t.name] = t.type
        print(f"  话题: {t.name} -> {t.type}")

    # ── 创建输出包 ────────────────────────────────────────────────────
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=out_path, storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))

    for t_name, t_type in topic_types.items():
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0, name=t_name, type=t_type, serialization_format='cdr'))

    # ── 逐消息处理 ────────────────────────────────────────────────────
    total_in = 0
    total_out = 0
    frame_count = 0
    lidar_count = 0

    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic == '/livox/lidar':
            msg = deserialize_message(data, PointCloud2)
            n_in = msg.width
            new_msg = downsample_one_msg(msg, TARGET_POINTS_PER_FRAME)
            n_out = new_msg.width
            writer.write(topic, serialize_message(new_msg), ts)
            total_in += n_in
            total_out += n_out
            lidar_count += 1
            if lidar_count % 500 == 0:
                print(f"  已处理 {lidar_count} 帧点云 ({n_in} → {n_out} pts)")
        else:
            writer.write(topic, data, ts)
            if topic not in ('/compressed_image',):
                frame_count += 1

    avg_in = total_in / lidar_count if lidar_count else 0
    avg_out = total_out / lidar_count if lidar_count else 0
    print(f"\n完成! 处理了 {lidar_count} 帧点云")
    print(f"  平均: {avg_in:.0f} → {avg_out:.0f} pts/frame")
    print(f"  压缩率: {avg_out/avg_in*100:.1f}%")
    print(f"  输出包: {out_path}")


if __name__ == '__main__':
    main()
