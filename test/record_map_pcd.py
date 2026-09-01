#!/usr/bin/env python3
"""
录制场地静态点云并保存为 map.pcd (rm_frame 坐标系)

使用最远点网格过滤，即使场上有机器人也能得到干净的静态地图。

用法:
  # 步骤 1: 启动 Livox 驱动
  ros2 launch livox_ros2_driver livox_lidar_launch.py

  # 步骤 2: 录制（即使场上有机器人也行）
  python3 test/record_map_pcd.py --duration 60

  # 需要提供激光雷达在场地中的位置信息，用于坐标变换:
  #   --lidar-x      激光雷达在裁判坐标系中的 X 位置（米）
  #   --lidar-y      激光雷达在裁判坐标系中的 Y 位置（米）
  #   --lidar-z      激光雷达在裁判坐标系中的 Z 高度（米）
  #   --lidar-yaw    激光雷达相对于裁判 X 轴的偏航角（度）
  #
  # 最简单: 指定红方或蓝方，自动使用规则手册中的雷达基座位置
  python3 test/record_map_pcd.py --duration 60 --team red
  python3 test/record_map_pcd.py --duration 60 --team blue
  #
  # 也可以手动指定位置（覆盖默认值）:
  python3 test/record_map_pcd.py --duration 60 \
      --lidar-x 0.0 --lidar-y 7.5 --lidar-z 3.7 --lidar-yaw 0
  #
  # 如果不提供坐标参数也不指定队伍，保存原始坐标（需后续用 align_map_pcd.py 对齐）。

坐标系说明:
  裁判系 (rm_frame):  X = 场地长边 (0→28m)，Y = 场地短边 (0→15m)，Z = 高度
  激光雷达 (livox):    X = 前方，Y = 左方，Z = 上方

  雷达基座位置 (来自 RMUC2026 规则手册 4.2.5):
    平台: 3.4m × 1.16m，中心与场地中心轴线对齐 (Y=7.5m)
    平台高度: ~2.5m，传感器支架建议 ≥1.2m (总高 ~3.7m)
    红方: X≈0, Y=7.5, Z≈3.7, yaw=0°   (面朝 X+)
    蓝方: X≈28, Y=7.5, Z≈3.7, yaw=180° (面朝 X-)
"""
import argparse
import math
import sys
import time
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    import sensor_msgs_py.point_cloud2 as pc2
except ImportError:
    print("ERROR: 需要 ROS2 环境。请先 source /opt/ros/jazzy/setup.bash && source install/setup.bash")
    sys.exit(1)


def farthest_point_grid_filter(points: np.ndarray, grid_deg: float = 0.1) -> np.ndarray:
    """
    最远点网格过滤（与 localization.cpp 相同算法）。
    将点云按角度分为网格，每个网格只保留最远的点。
    效果：近处的机器人被过滤掉，远处的墙壁和结构保留。
    """
    if len(points) == 0:
        return points

    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    azimuth = np.arctan2(y, x)
    horizontal_dist = np.sqrt(x**2 + y**2)
    elevation = np.arctan2(z, horizontal_dist)
    distance = np.sqrt(x**2 + y**2 + z**2)

    az_idx = np.floor(np.degrees(azimuth) / grid_deg).astype(np.int32)
    el_idx = np.floor(np.degrees(elevation) / grid_deg).astype(np.int32)

    grid_key = az_idx.astype(np.int64) * 100000 + el_idx.astype(np.int64)

    # 每个网格保留最远的点
    best = {}
    for i in range(len(points)):
        k = grid_key[i]
        d = distance[i]
        if k not in best or d > best[k][1]:
            best[k] = (i, d)

    indices = [v[0] for v in best.values()]
    return points[indices]


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    if voxel_size <= 0 or len(points) == 0:
        return points
    indices = np.floor(points / voxel_size).astype(np.int32)
    _, unique_idx = np.unique(indices, axis=0, return_index=True)
    return points[unique_idx]


def make_transform(tx, ty, tz, yaw_deg):
    """创建从 livox_frame 到 rm_frame 的 4x4 变换矩阵"""
    yaw = math.radians(yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    T = np.eye(4, dtype=np.float64)
    T[0, 0] = c;  T[0, 1] = -s
    T[1, 0] = s;  T[1, 1] = c
    T[0, 3] = tx
    T[1, 3] = ty
    T[2, 3] = tz
    return T


def transform_points(points: np.ndarray, T: np.ndarray) -> np.ndarray:
    ones = np.ones((len(points), 1), dtype=np.float32)
    pts4 = np.hstack([points, ones])  # Nx4
    result = (T @ pts4.T).T  # Nx4
    return result[:, :3].astype(np.float32)


def write_pcd(path: str, points: np.ndarray):
    with open(path, 'w') as f:
        f.write('# .PCD v0.7 - Point Cloud Data\n')
        f.write('VERSION 0.7\n')
        f.write('FIELDS x y z\n')
        f.write('SIZE 4 4 4\n')
        f.write('TYPE F F F\n')
        f.write('COUNT 1 1 1\n')
        f.write(f'WIDTH {len(points)}\n')
        f.write('HEIGHT 1\n')
        f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
        f.write(f'POINTS {len(points)}\n')
        f.write('DATA ascii\n')
        for p in points:
            f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n')


class MapRecorder(Node):
    def __init__(self, duration: float):
        super().__init__('map_pcd_recorder')
        self.duration = duration
        self.all_points = []
        self.start_time = None
        self.frame_count = 0
        self.done = False

        self.sub = self.create_subscription(
            PointCloud2, '/livox/lidar', self.callback, 10)
        self.get_logger().info(f'等待 /livox/lidar 话题... (录制 {duration}s)')

    def callback(self, msg: PointCloud2):
        if self.done:
            return

        if self.start_time is None:
            self.start_time = time.time()
            self.get_logger().info('收到第一帧点云，开始累积...')

        elapsed = time.time() - self.start_time
        if elapsed > self.duration:
            self.done = True
            return

        points = []
        for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            points.append([p[0], p[1], p[2]])

        if points:
            self.all_points.extend(points)
            self.frame_count += 1

        if self.frame_count % 20 == 0:
            self.get_logger().info(
                f'  累积 {self.frame_count} 帧, {len(self.all_points)} 点, '
                f'剩余 {max(0, self.duration - elapsed):.0f}s')


def main():
    parser = argparse.ArgumentParser(
        description='录制场地静态点云 map.pcd',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    parser.add_argument('--duration', type=float, default=60.0,
                        help='录制时长（秒），默认 60。场上有机器人时建议 >= 60')
    parser.add_argument('--output', type=str, default='config/map.pcd',
                        help='输出文件路径，默认 config/map.pcd')
    parser.add_argument('--voxel', type=float, default=0.1,
                        help='下采样体素大小（米），默认 0.1')
    parser.add_argument('--grid-deg', type=float, default=0.1,
                        help='最远点网格角度分辨率（度），默认 0.1')
    parser.add_argument('--team', type=str, default=None, choices=['red', 'blue'],
                        help='队伍颜色，自动使用规则手册中的雷达基座默认位置')
    parser.add_argument('--lidar-x', type=float, default=None,
                        help='激光雷达在裁判坐标系中的 X（米），覆盖 --team 默认值')
    parser.add_argument('--lidar-y', type=float, default=None,
                        help='激光雷达在裁判坐标系中的 Y（米），覆盖 --team 默认值')
    parser.add_argument('--lidar-z', type=float, default=None,
                        help='激光雷达在裁判坐标系中的 Z（米），覆盖 --team 默认值')
    parser.add_argument('--lidar-yaw', type=float, default=None,
                        help='激光雷达偏航角（度），0=面朝 X+，覆盖 --team 默认值')
    args = parser.parse_args()

    # RMUC2026 规则手册 4.2.5: 雷达基座位置默认值
    TEAM_DEFAULTS = {
        'red':  {'x': 0.0,  'y': 7.5, 'z': 3.7, 'yaw': 0.0},
        'blue': {'x': 28.0, 'y': 7.5, 'z': 3.7, 'yaw': 180.0},
    }

    if args.team and not all(v is not None for v in [args.lidar_x, args.lidar_y, args.lidar_z]):
        defaults = TEAM_DEFAULTS[args.team]
        if args.lidar_x is None: args.lidar_x = defaults['x']
        if args.lidar_y is None: args.lidar_y = defaults['y']
        if args.lidar_z is None: args.lidar_z = defaults['z']
        if args.lidar_yaw is None: args.lidar_yaw = defaults['yaw']
        print(f'使用 {args.team} 方雷达基座默认位置: '
              f'x={args.lidar_x}, y={args.lidar_y}, z={args.lidar_z}, yaw={args.lidar_yaw}°')

    if args.lidar_yaw is None:
        args.lidar_yaw = 0.0

    has_transform = all(v is not None for v in [args.lidar_x, args.lidar_y, args.lidar_z])

    rclpy.init()
    node = MapRecorder(args.duration)

    start = time.time()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
            if time.time() - start > args.duration + 30:
                node.get_logger().warn('超时')
                break
    except KeyboardInterrupt:
        node.get_logger().info('手动中断')

    if not node.all_points:
        node.get_logger().error('没有收到任何点云数据！')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    pts = np.array(node.all_points, dtype=np.float32)
    print(f'\n原始点数: {len(pts)}')

    # 最远点网格过滤（过滤机器人）
    pts = farthest_point_grid_filter(pts, args.grid_deg)
    print(f'最远点过滤后: {len(pts)}')

    # 坐标变换
    if has_transform:
        T = make_transform(args.lidar_x, args.lidar_y, args.lidar_z, args.lidar_yaw)
        pts = transform_points(pts, T)
        print(f'已变换到裁判坐标系 (lidar @ x={args.lidar_x}, y={args.lidar_y}, '
              f'z={args.lidar_z}, yaw={args.lidar_yaw}°)')
    else:
        print('⚠ 未提供 --lidar-x/y/z，输出为激光雷达原始坐标')
        print('  后续需要手动对齐到裁判坐标系 (0-28m × 0-15m)')
        print('  或重新运行并加上: --lidar-x X --lidar-y Y --lidar-z Z --lidar-yaw YAW')

    # 体素下采样
    pts = voxel_downsample(pts, args.voxel)
    print(f'体素下采样后: {len(pts)}')

    # 统计
    print(f'范围: X=[{pts[:,0].min():.1f}, {pts[:,0].max():.1f}], '
          f'Y=[{pts[:,1].min():.1f}, {pts[:,1].max():.1f}], '
          f'Z=[{pts[:,2].min():.1f}, {pts[:,2].max():.1f}]')

    if has_transform:
        in_field = ((pts[:, 0] >= 0) & (pts[:, 0] <= 28) &
                    (pts[:, 1] >= 0) & (pts[:, 1] <= 15))
        ratio = in_field.sum() / len(pts) * 100
        print(f'在场地范围内 (0-28 × 0-15) 的点: {in_field.sum()} ({ratio:.0f}%)')
        if ratio < 50:
            print('⚠ 场地范围内的点少于 50%，请检查 --lidar-x/y/z/yaw 参数是否正确')

    write_pcd(args.output, pts)
    print(f'\n✓ 保存成功: {args.output} ({len(pts)} 点)')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
