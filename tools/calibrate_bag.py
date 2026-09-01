#!/home/zst/T/.venv/bin/python
"""
calibrate_bag.py — 交互式外参标定工具（针对 rosbag 录制帧）
==============================================================
用途：
  从 rosbag 中提取一帧图像，让用户依次点击 5 个已知三维标定点，
  通过 solvePnP 求解相机外参（rvec / tvec），保存到
  config/map/bag_extrinsics.yaml，供 compare_methods.py 使用。

相机内参：使用东北大学 T-DT config/camera_params.yaml（包录制时的相机）
标定点顺序（蓝方视角，RM2026 裁判坐标系）：
  1. 我方基地近角        self_fortress  [20.068, 7.469, 0.000]
  2. 我方前哨站顶角      self_tower     [17.178, 11.336, 1.869]
  3. 敌方能量机关        enemy_base     [4.983, 10.547, 0.600]
  4. 敌方基地左侧        enemy_tower    [3.459, 3.920, 0.732]
  5. 敌方前哨站顶角      cross_tower    [10.822, 3.664, 1.868]

操作说明：
  • 左键单击 → 记录当前点
  • 右键单击 → 撤销上一个点
  • 鼠标滚轮 → 缩放图像
  • 中键拖拽 → 平移图像
  • Enter / s  → 完成标定并保存（需已点击全部 5 个点）
  • r          → 重置所有点，重新标定
  • q / Esc   → 退出（不保存）

用法：
  cd /home/zst/T
  source /opt/ros/jazzy/setup.bash
  python3 tools/calibrate_bag.py [--bag bags/latest] [--frame 30]
"""

import sys
import os

# ROS2 PYTHONPATH 会把系统 cv2 (numpy 1.x) 排在 venv 前面，提前修正路径
_venv_sp = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        '.venv', 'lib', 'python3.12', 'site-packages')
if os.path.isdir(_venv_sp):
    if _venv_sp in sys.path:
        sys.path.remove(_venv_sp)
    sys.path.insert(0, _venv_sp)

import argparse
import struct

import cv2
import numpy as np
import yaml

# ── 东北大学 T-DT 内参 ───────────────────────────────────────────────────────
K_TDT = np.array(
    [[3617.422853, 0.0,         2046.972383],
     [0.0,         3617.561167, 1515.242535],
     [0.0,         0.0,         1.0        ]], dtype=np.float64
)
DIST_TDT = np.array([-0.064321, 0.114219, -0.000746, 0.000801, 0.0], dtype=np.float64)

# ── RM2026 蓝方标定点（裁判坐标系，单位 m） ──────────────────────────────────
WORLD_POINTS = {
    'self_fortress': [20.06790, 7.46930, 0.00020],
    'self_tower':    [17.17800, 11.33570, 1.86860],
    'enemy_base':    [4.98290,  10.54700, 0.60000],
    'enemy_tower':   [3.45860,  3.91990,  0.73180],
    'cross_tower':   [10.82200, 3.66430,  1.86840],
}
POINT_ORDER = ['self_fortress', 'self_tower', 'enemy_base', 'enemy_tower', 'cross_tower']
POINT_LABELS_CN = [
    '① 我方基地近角',
    '② 我方前哨站顶角',
    '③ 敌方能量机关',
    '④ 敌方基地左侧',
    '⑤ 敌方前哨站顶角',
]
WORLD_PTS_ARR = np.array([WORLD_POINTS[k] for k in POINT_ORDER], dtype=np.float64)

OUT_YAML = 'config/map/bag_extrinsics.yaml'


# ── 从 bag 提取第 N 帧 ────────────────────────────────────────────────────────
def extract_frame(bag_path: str, frame_index: int = 30) -> np.ndarray:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import CompressedImage
    except ImportError:
        print("[ERR] 需要 source /opt/ros/jazzy/setup.bash 后执行")
        sys.exit(1)

    reader = rosbag2_py.SequentialReader()
    storage_opts = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    conv_opts    = rosbag2_py.ConverterOptions('cdr', 'cdr')
    reader.open(storage_opts, conv_opts)

    reader.set_filter(rosbag2_py.StorageFilter(topics=['/compressed_image']))

    count = 0
    img = None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        count += 1
        if count == frame_index or not reader.has_next():
            msg = deserialize_message(data, CompressedImage)
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            break

    if img is None:
        print(f"[ERR] 无法从 {bag_path} 提取第 {frame_index} 帧")
        sys.exit(1)

    print(f"[OK] 提取第 {count} 帧：{img.shape[1]}×{img.shape[0]}  from {bag_path}")
    return img


# ── 交互标定窗口 ──────────────────────────────────────────────────────────────
class CalibWindow:
    WIN = 'calibrate_bag'
    COLORS = [(255, 100, 50), (50, 220, 50), (50, 150, 255),
              (255, 200, 50), (200, 50, 255)]

    def __init__(self, img: np.ndarray):
        self.orig = img.copy()
        self.h, self.w = img.shape[:2]
        self.scale  = min(1.0, 1400 / self.w, 900 / self.h)
        self.offset = np.array([0.0, 0.0])   # translation in display coords
        self.clicks = []      # list of (u, v) in original image coords
        self._drag_start = None
        self._drag_offset = None

    # ── 坐标转换 ──────────────────────────────────────────────────────────────
    def img_to_display(self, u, v):
        x = u * self.scale + self.offset[0]
        y = v * self.scale + self.offset[1]
        return int(x), int(y)

    def display_to_img(self, x, y):
        u = (x - self.offset[0]) / self.scale
        v = (y - self.offset[1]) / self.scale
        return u, v

    # ── 绘制 ─────────────────────────────────────────────────────────────────
    def _render(self):
        dw = int(self.w * self.scale)
        dh = int(self.h * self.scale)
        disp = cv2.resize(self.orig, (dw, dh), interpolation=cv2.INTER_LINEAR)

        # 把图像放到画布上
        canvas_w = max(dw + int(abs(self.offset[0])) + 20, 1400)
        canvas_h = max(dh + int(abs(self.offset[1])) + 20, 900)
        canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)

        ox, oy = int(self.offset[0]), int(self.offset[1])
        ox = max(0, ox)
        oy = max(0, oy)
        ex = min(ox + dw, canvas_w)
        ey = min(oy + dh, canvas_h)
        sx = ex - ox
        sy = ey - oy
        canvas[oy:ey, ox:ex] = disp[:sy, :sx]

        # 已标定的点
        for i, (u, v) in enumerate(self.clicks):
            cx, cy = self.img_to_display(u, v)
            cx += 0; cy += 0  # offset already baked into img_to_display via offset
            col = self.COLORS[i % len(self.COLORS)]
            cv2.circle(canvas, (cx + ox - int(self.offset[0]), cy + oy - int(self.offset[1])), 8, col, -1)
            cv2.circle(canvas, (cx + ox - int(self.offset[0]), cy + oy - int(self.offset[1])), 8, (255,255,255), 1)
            cv2.putText(canvas, str(i+1), (cx + ox - int(self.offset[0]) + 10, cy + oy - int(self.offset[1]) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)

        # 重投影误差（如果已标定全5点）
        reproj_err_str = ''
        if len(self.clicks) == 5:
            img_pts = np.array(self.clicks, dtype=np.float64)
            ok, rvec, tvec = cv2.solvePnP(WORLD_PTS_ARR, img_pts, K_TDT, DIST_TDT,
                                          flags=cv2.SOLVEPNP_EPNP)
            if ok:  # 进一步迭代精化
                ok, rvec, tvec = cv2.solvePnP(WORLD_PTS_ARR, img_pts, K_TDT, DIST_TDT,
                                              rvec, tvec, useExtrinsicGuess=True,
                                              flags=cv2.SOLVEPNP_ITERATIVE)
            if ok:
                proj, _ = cv2.projectPoints(WORLD_PTS_ARR, rvec, tvec, K_TDT, DIST_TDT)
                errs = np.linalg.norm(proj.reshape(-1,2) - img_pts, axis=1)
                reproj_err_str = f'重投影误差: {errs.mean():.2f}px (max {errs.max():.2f}px)'
                # 绘制重投影点
                for j, p in enumerate(proj.reshape(-1, 2)):
                    pu, pv = int(p[0]), int(p[1])
                    pdx, pdy = self.img_to_display(pu, pv)
                    col = self.COLORS[j % len(self.COLORS)]
                    cv2.drawMarker(canvas, (pdx, pdy), col, cv2.MARKER_CROSS, 14, 2)

        # HUD
        next_idx = len(self.clicks)
        if next_idx < 5:
            msg = f'点击: {POINT_LABELS_CN[next_idx]}  ({POINT_ORDER[next_idx]})'
            cv2.putText(canvas, msg, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,220,255), 2)
        else:
            cv2.putText(canvas, '全部5点已完成！按 Enter/s 保存  r 重置',
                        (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,80), 2)

        if reproj_err_str:
            cv2.putText(canvas, reproj_err_str, (12, 62),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,200,50), 2)

        cv2.putText(canvas, '右键=撤销  滚轮=缩放  中键拖拽=平移  q=退出',
                    (12, canvas_h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100,100,100), 1)
        return canvas

    # ── 鼠标回调 ─────────────────────────────────────────────────────────────
    def mouse_cb(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(self.clicks) < 5:
                u, v = self.display_to_img(x, y)
                self.clicks.append((u, v))
                label = POINT_LABELS_CN[len(self.clicks)-1]
                print(f'  [{len(self.clicks)}] {label}: pixel=({u:.1f}, {v:.1f})')
        elif event == cv2.EVENT_RBUTTONDOWN:
            if self.clicks:
                self.clicks.pop()
                print(f'  [undo] 撤销上一个点，现有 {len(self.clicks)} 个')
        elif event == cv2.EVENT_MOUSEWHEEL:
            factor = 1.1 if flags > 0 else 0.9
            new_scale = max(0.1, min(4.0, self.scale * factor))
            # 以鼠标位置为中心缩放
            u, v = self.display_to_img(x, y)
            self.scale = new_scale
            # 调整 offset 使鼠标下的图像点不变
            self.offset[0] = x - u * self.scale
            self.offset[1] = y - v * self.scale
        elif event == cv2.EVENT_MBUTTONDOWN:
            self._drag_start  = (x, y)
            self._drag_offset = self.offset.copy()
        elif event == cv2.EVENT_MOUSEMOVE and self._drag_start:
            dx = x - self._drag_start[0]
            dy = y - self._drag_start[1]
            self.offset = self._drag_offset + np.array([dx, dy], dtype=float)
        elif event == cv2.EVENT_MBUTTONUP:
            self._drag_start = None

    # ── 主循环 ───────────────────────────────────────────────────────────────
    def run(self):
        cv2.namedWindow(self.WIN, cv2.WINDOW_NORMAL)
        init_w = min(1400, self.w)
        init_h = int(init_w / self.w * self.h)
        cv2.resizeWindow(self.WIN, init_w, init_h)
        cv2.setMouseCallback(self.WIN, self.mouse_cb)

        print('\n── 标定操作指南 ──────────────────────────────────────')
        for i, (key, cn) in enumerate(zip(POINT_ORDER, POINT_LABELS_CN)):
            wp = WORLD_POINTS[key]
            print(f'  {cn}: {wp}  m')
        print('  左键=记录点  右键=撤销  Enter/s=保存  r=重置  q=退出')
        print('─────────────────────────────────────────────────────\n')

        while True:
            canvas = self._render()
            cv2.imshow(self.WIN, canvas)
            key = cv2.waitKey(30) & 0xFF

            if key in (ord('q'), 27):
                print('[退出] 未保存')
                cv2.destroyAllWindows()
                return None, None

            elif key in (13, ord('s')):   # Enter 或 s
                if len(self.clicks) < 5:
                    print(f'[!] 还需点击 {5 - len(self.clicks)} 个点')
                else:
                    cv2.destroyAllWindows()
                    return self._solve_and_return()

            elif key == ord('r'):
                self.clicks.clear()
                print('[重置] 已清空所有点')

        cv2.destroyAllWindows()
        return None, None

    def _solve_and_return(self):
        img_pts = np.array(self.clicks, dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(
            WORLD_PTS_ARR, img_pts, K_TDT, DIST_TDT,
            flags=cv2.SOLVEPNP_EPNP
        )
        if ok:  # 迭代精化
            ok, rvec, tvec = cv2.solvePnP(
                WORLD_PTS_ARR, img_pts, K_TDT, DIST_TDT,
                rvec, tvec, useExtrinsicGuess=True,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
        if not ok:
            print('[ERR] solvePnP 失败')
            return None, None

        proj, _ = cv2.projectPoints(WORLD_PTS_ARR, rvec, tvec, K_TDT, DIST_TDT)
        errs = np.linalg.norm(proj.reshape(-1, 2) - img_pts, axis=1)
        print(f'\n── solvePnP 结果 ───────────────────────────')
        print(f'  rvec: {rvec.flatten().tolist()}')
        print(f'  tvec: {tvec.flatten().tolist()}')
        print(f'  重投影误差: 均值={errs.mean():.3f}px  最大={errs.max():.3f}px')
        for i, (name, err) in enumerate(zip(POINT_ORDER, errs)):
            print(f'    {POINT_LABELS_CN[i]}: {err:.3f}px')
        print('────────────────────────────────────────────\n')

        return rvec.flatten(), tvec.flatten()


# ── 保存外参 yaml ─────────────────────────────────────────────────────────────
def save_extrinsics(rvec, tvec, out_path):
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    data = {
        'comment': 'RM2026 蓝方外参，由 calibrate_bag.py 交互标定生成',
        'calibration_points': 'config/blue/calibrate_points_blue.yaml',
        'rvec': rvec.tolist(),
        'tvec': tvec.tolist(),
        'K': K_TDT.tolist(),
        'dist': DIST_TDT.tolist(),
        'coord_frame': 'T-DT_world (x=x_ref, y=-y_ref)',
    }
    with open(out_path, 'w') as f:
        yaml.dump(data, f, allow_unicode=True, default_flow_style=False)
    print(f'[保存] 外参已写入: {out_path}')
    print('       compare_methods.py 将自动加载此文件')


# ── 主函数 ────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description='交互式 rosbag 外参标定工具')
    ap.add_argument('--bag',   default='bags/latest', help='rosbag 路径')
    ap.add_argument('--frame', type=int, default=60,         help='提取第几帧图像（跳过开头模糊帧）')
    ap.add_argument('--out',   default=OUT_YAML,             help='输出 yaml 路径')
    args = ap.parse_args()

    print(f'[calibrate_bag] 从 {args.bag} 提取第 {args.frame} 帧…')
    img = extract_frame(args.bag, args.frame)

    win = CalibWindow(img)
    rvec, tvec = win.run()

    if rvec is not None:
        save_extrinsics(rvec, tvec, args.out)
        print('\n下一步：')
        print('  python3 tools/compare_methods.py')
        print('  （compare_methods.py 会自动读取新外参）')
    else:
        print('[跳过] 未保存任何外参')


if __name__ == '__main__':
    main()
