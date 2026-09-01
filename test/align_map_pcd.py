#!/usr/bin/env python3
"""
交互式对齐 map.pcd 到裁判坐标系 (rm_frame)

用法:
  # 步骤 1: 先用 record_map_pcd.py 录制原始点云（不加 --lidar-x/y/z）
  python3 test/record_map_pcd.py --duration 60 --output config/map_raw.pcd

  # 步骤 2: 用本脚本交互式对齐
  python3 test/align_map_pcd.py config/map_raw.pcd

  脚本会显示俯视图，你用鼠标点击两个已知点（比如场地两个角），
  脚本自动计算变换并保存为 config/map.pcd。

原理:
  你知道场地是 28m × 15m。如果你能在点云中找到两个坐标已知的点
  （比如两个底线角），脚本就能算出旋转+平移。
"""
import sys
import numpy as np
import math


def read_pcd(path):
    """读取 ASCII 格式的 PCD 文件"""
    points = []
    data_start = False
    with open(path, 'r') as f:
        for line in f:
            if data_start:
                parts = line.strip().split()
                if len(parts) >= 3:
                    points.append([float(parts[0]), float(parts[1]), float(parts[2])])
            elif line.strip().startswith('DATA'):
                data_start = True
    return np.array(points, dtype=np.float32)


def write_pcd(path, points):
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


def compute_transform_2points(src_pts, dst_pts):
    """
    从两对对应点计算 2D 变换 (旋转 + 平移，保持 Z 不变)。
    src_pts: [[x1,y1], [x2,y2]]  点云中的两个点 (原始坐标)
    dst_pts: [[x1,y1], [x2,y2]]  对应的裁判坐标
    """
    s1, s2 = np.array(src_pts[0]), np.array(src_pts[1])
    d1, d2 = np.array(dst_pts[0]), np.array(dst_pts[1])

    # 源向量和目标向量
    sv = s2 - s1
    dv = d2 - d1

    # 缩放（应该接近 1）
    scale = np.linalg.norm(dv) / np.linalg.norm(sv)

    # 旋转角
    angle_src = math.atan2(sv[1], sv[0])
    angle_dst = math.atan2(dv[1], dv[0])
    angle = angle_dst - angle_src

    # 构建变换
    c, s = math.cos(angle), math.sin(angle)
    R = np.array([[c, -s], [s, c]])

    # 平移: dst = R * scale * src + t  =>  t = dst - R * scale * src
    t = d1 - scale * R @ s1

    return R, t, scale, math.degrees(angle)


def apply_transform(points, R, t, scale):
    result = points.copy()
    xy = points[:, :2]
    xy_transformed = scale * (xy @ R.T) + t
    result[:, 0] = xy_transformed[:, 0]
    result[:, 1] = xy_transformed[:, 1]
    return result


def interactive_mode(pts):
    """用 matplotlib 交互选择对应点"""
    try:
        import matplotlib
        matplotlib.use('TkAgg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("ERROR: 需要 matplotlib。pip install matplotlib")
        return None, None

    # 俯视图 (XY 平面)
    fig, ax = plt.subplots(1, 1, figsize=(12, 8))
    ax.scatter(pts[:, 0], pts[:, 1], s=0.3, c=pts[:, 2], cmap='viridis', alpha=0.5)
    ax.set_aspect('equal')
    ax.set_title('俯视图 - 左键点击两个已知点 (关闭窗口完成选择)')
    ax.set_xlabel('X (原始)')
    ax.set_ylabel('Y (原始)')
    ax.grid(True, alpha=0.3)

    clicked = []

    def onclick(event):
        if event.button == 1 and event.inaxes == ax and len(clicked) < 2:
            clicked.append([event.xdata, event.ydata])
            ax.plot(event.xdata, event.ydata, 'ro', markersize=10)
            ax.annotate(f'P{len(clicked)} ({event.xdata:.1f}, {event.ydata:.1f})',
                        (event.xdata, event.ydata), fontsize=10, color='red',
                        xytext=(10, 10), textcoords='offset points')
            fig.canvas.draw()
            if len(clicked) == 2:
                print(f'\n选择了两个点:')
                print(f'  P1: ({clicked[0][0]:.2f}, {clicked[0][1]:.2f})')
                print(f'  P2: ({clicked[1][0]:.2f}, {clicked[1][1]:.2f})')
                print('关闭窗口继续...')

    fig.canvas.mpl_connect('button_press_event', onclick)
    plt.show()

    if len(clicked) < 2:
        print('未选择足够的点')
        return None, None

    return clicked[0], clicked[1]


def main():
    if len(sys.argv) < 2:
        print("用法: python3 test/align_map_pcd.py <输入.pcd> [输出.pcd]")
        print()
        print("交互模式: 显示俯视图，鼠标点击两个已知点")
        print("手动模式: 加 --manual 直接输入坐标")
        print()
        print("例:")
        print("  python3 test/align_map_pcd.py config/map_raw.pcd")
        print("  python3 test/align_map_pcd.py config/map_raw.pcd config/map.pcd --manual")
        sys.exit(1)

    input_path = sys.argv[1]
    output_path = sys.argv[2] if len(sys.argv) > 2 and not sys.argv[2].startswith('--') else 'config/map.pcd'
    manual_mode = '--manual' in sys.argv

    pts = read_pcd(input_path)
    print(f'读取 {len(pts)} 个点')
    print(f'范围: X=[{pts[:,0].min():.1f}, {pts[:,0].max():.1f}], '
          f'Y=[{pts[:,1].min():.1f}, {pts[:,1].max():.1f}], '
          f'Z=[{pts[:,2].min():.1f}, {pts[:,2].max():.1f}]')

    if manual_mode:
        print('\n手动模式: 输入两对对应点')
        print('提示: 选择你能在点云中辨认的场地特征（角落、柱子等）')
        print()

        print('点云中的第一个点 (原始坐标):')
        sx1 = float(input('  X = '))
        sy1 = float(input('  Y = '))
        print('这个点在裁判坐标系中的位置:')
        dx1 = float(input('  X (0-28) = '))
        dy1 = float(input('  Y (0-15) = '))

        print('点云中的第二个点 (原始坐标):')
        sx2 = float(input('  X = '))
        sy2 = float(input('  Y = '))
        print('这个点在裁判坐标系中的位置:')
        dx2 = float(input('  X (0-28) = '))
        dy2 = float(input('  Y (0-15) = '))

        src_pts = [[sx1, sy1], [sx2, sy2]]
        dst_pts = [[dx1, dy1], [dx2, dy2]]
    else:
        print('\n交互模式: 请在弹出的窗口中点击两个已知点')
        p1, p2 = interactive_mode(pts)
        if p1 is None:
            sys.exit(1)

        print('\n现在输入这两个点在裁判坐标系中的实际位置:')
        print(f'P1 在点云中: ({p1[0]:.2f}, {p1[1]:.2f})')
        dx1 = float(input('  P1 裁判坐标 X (0-28) = '))
        dy1 = float(input('  P1 裁判坐标 Y (0-15) = '))
        print(f'P2 在点云中: ({p2[0]:.2f}, {p2[1]:.2f})')
        dx2 = float(input('  P2 裁判坐标 X (0-28) = '))
        dy2 = float(input('  P2 裁判坐标 Y (0-15) = '))

        src_pts = [p1, p2]
        dst_pts = [[dx1, dy1], [dx2, dy2]]

    R, t, scale, angle = compute_transform_2points(src_pts, dst_pts)
    print(f'\n计算得到的变换:')
    print(f'  旋转: {angle:.1f}°')
    print(f'  缩放: {scale:.4f} (应接近 1.0)')
    print(f'  平移: ({t[0]:.2f}, {t[1]:.2f})')

    if abs(scale - 1.0) > 0.1:
        print(f'⚠ 缩放 {scale:.3f} 偏离 1.0 较多，对应点可能选错了')

    result = apply_transform(pts, R, t, scale)

    print(f'\n变换后范围: X=[{result[:,0].min():.1f}, {result[:,0].max():.1f}], '
          f'Y=[{result[:,1].min():.1f}, {result[:,1].max():.1f}], '
          f'Z=[{result[:,2].min():.1f}, {result[:,2].max():.1f}]')

    in_field = ((result[:, 0] >= 0) & (result[:, 0] <= 28) &
                (result[:, 1] >= 0) & (result[:, 1] <= 15))
    ratio = in_field.sum() / len(result) * 100
    print(f'在场地范围内的点: {in_field.sum()} ({ratio:.0f}%)')

    write_pcd(output_path, result)
    print(f'\n✓ 保存成功: {output_path} ({len(result)} 点)')

    # 显示结果
    try:
        import matplotlib
        matplotlib.use('TkAgg')
        import matplotlib.pyplot as plt
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
        ax1.scatter(pts[:, 0], pts[:, 1], s=0.3, c='gray', alpha=0.3)
        ax1.set_title('变换前 (原始)')
        ax1.set_aspect('equal')
        ax1.grid(True, alpha=0.3)

        ax2.scatter(result[:, 0], result[:, 1], s=0.3, c=result[:, 2], cmap='viridis', alpha=0.3)
        ax2.axhline(0, color='r', linewidth=0.5); ax2.axhline(15, color='r', linewidth=0.5)
        ax2.axvline(0, color='r', linewidth=0.5); ax2.axvline(28, color='r', linewidth=0.5)
        ax2.set_title('变换后 (裁判坐标系)')
        ax2.set_aspect('equal')
        ax2.grid(True, alpha=0.3)
        ax2.set_xlim(-2, 30)
        ax2.set_ylim(-2, 17)
        plt.tight_layout()
        plt.savefig('config/outputs/map_alignment.png', dpi=150)
        print(f'对齐结果图: config/outputs/map_alignment.png')
        plt.show()
    except Exception:
        pass


if __name__ == '__main__':
    main()
