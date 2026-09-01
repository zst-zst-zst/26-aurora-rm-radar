#!/usr/bin/env python3
"""
作用：从PCD点云仿真雷达盲区遮挡 (三维射线投影到机器人高度平面)。
输出被 visualize_guess_pts.py 加载为盲区底图。

用法：
    python3 tools/simulate_shadow.py
    # 读 config/map/RMUC2026_ground_only_1cm.pcd，写出下方 OUT_DIR 下的 npy + png。

原理:
  雷达在 (rx,ry,rz) 处,  障碍物点在 (ox,oy,oz),  机器人高度 z_tgt
  遮挡发生条件: 障碍物点 oz > 当前射线在此水平距离处的高度
  投影: t = (rz - z_tgt)/(rz - oz), 遮影落点 = radar + t*(obs - radar)

输出:
  config/outputs/shadow_red.npy   红方视角遮挡图 (0=遮挡, 1=可见)
  config/outputs/shadow_blue.npy  蓝方视角
  config/outputs/shadow_vis.png   可视化对比图
"""
import numpy as np
import os
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import warnings; warnings.filterwarnings('ignore')

# ── 配置 ──────────────────────────────────────────────────────────────────────
ROOT      = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
# STL派生 PCD (裁判坐标系, 12.99M点, 能量机关等空心结构正确建模)
PCD_PATH  = os.path.join(ROOT, 'config', 'map', 'RMUC2026_ground_only_1cm.pcd')
OUT_DIR   = os.path.join(ROOT, 'config', 'outputs')
TOPVIEW   = os.path.join(OUT_DIR, 'RMUC2026_topview_dense.png')

FIELD_W, FIELD_H = 28.0, 15.0
GRID_RES  = 0.05        # m/cell (5cm 精度)
Z_RADAR   = 3.9         # 雷达实际安裃高度 (m) — 相机基本平视
                         # 下俧角: 3.9m到0.5m目标 @28m远只有≈6.9°
Z_TARGET  = 0.0         # 装甲板高度 (m) — 有的车装甲板基本贴地

# 雷达位置 — 根据实际安装调整
# 红方: 场地左上角 (x≈0.5, y≈13) 或 左下角
# 雷达基座 [11] 实测安裃位置 (referee 坐标系)
# 红方: x=-1 (侧墙外 1m), y=6, z=3.9m
# 蓝方: 对称 x=28-(-1)=29, y=15-6=9
RADAR_RED  = np.array([-1.0, 6.0, Z_RADAR])
RADAR_BLUE = np.array([29.0, 9.0, Z_RADAR])

# 障碍物高度过滤: 只统计足够挡住机器人的高度层
OBS_Z_MIN = 0.30   # 降低至 0.30m 以捕获中央高地边缘 (0.35-0.40m) 的阴影
OBS_Z_MAX = 4.0                # 排除天花板/场外杂点

# ── PCD 解析 ──────────────────────────────────────────────────────────────────
def load_pcd(path):
    """解析 binary PCD, 返回 Nx3 float32 xyz (载入时即过滤 z>OBS_Z_MIN 以减少内存)"""
    with open(path, 'rb') as f:
        n_pts, n_fields = 0, 3
        while True:
            line = f.readline().decode('ascii', errors='ignore').strip()
            if line.startswith('POINTS'):
                n_pts = int(line.split()[1])
            if line.startswith('FIELDS'):
                n_fields = len(line.split()) - 1
            if line.startswith('DATA'):
                break
        raw = f.read(n_pts * n_fields * 4)
    pts = np.frombuffer(raw, dtype=np.float32).reshape(-1, n_fields)[:, :3]
    # 就地过滤: 只保留能产生遮挡的高度层
    mask = (pts[:, 2] >= OBS_Z_MIN) & (pts[:, 2] <= OBS_Z_MAX)
    return pts[mask].copy()

# ── 2D 高度图 (每格最大Z) ─────────────────────────────────────────────────────
def build_height_map(pts, res=GRID_RES, fw=FIELD_W, fh=FIELD_H):
    gw = int(np.ceil(fw / res))
    gh = int(np.ceil(fh / res))
    hmap = np.full((gh, gw), -np.inf, dtype=np.float32)

    # 场地内有效点
    mask = (pts[:,0] >= 0) & (pts[:,0] < fw) & \
           (pts[:,1] >= 0) & (pts[:,1] < fh) & \
           (pts[:,2] >= OBS_Z_MIN) & (pts[:,2] <= OBS_Z_MAX)
    p = pts[mask]
    xi = np.clip((p[:,0] / res).astype(np.int32), 0, gw-1)
    yi = np.clip((p[:,1] / res).astype(np.int32), 0, gh-1)

    # 逐点更新最大Z (比 scatter 慢但正确)
    np.maximum.at(hmap, (yi, xi), p[:,2])
    return hmap

# ── 遮挡图计算 ────────────────────────────────────────────────────────────────
def compute_occlusion(radar, hmap, res=GRID_RES, fw=FIELD_W, fh=FIELD_H):
    """
    对高度图每格做射线遮挡判断, 返回同尺寸 bool 可见图 (True=可见).
    算法: 角度扫描 — 对每个方位角射线, 沿射线传播, 追踪最大"仰角",
          若当前格仰角 < 已见最大仰角则被遮挡.
    """
    rx, ry, rz = radar
    gh, gw = hmap.shape

    visible = np.ones((gh, gw), dtype=bool)
    visited = np.zeros((gh, gw), dtype=bool)

    # 以 rx,ry 为中心, 发射 N_ANGLES 条射线
    N_ANGLES = 3600
    for i in range(N_ANGLES):
        theta = 2 * np.pi * i / N_ANGLES
        dx = np.cos(theta)
        dy = np.sin(theta)

        x, y = rx, ry
        max_elev = -np.inf   # 已见最高仰角 (从雷达水平面看障碍物顶)

        step = res * 0.6
        # 增加 max_steps: 雷达在场外时需要额外步数穿越边界
        extra = max(abs(rx), abs(ry - fh), abs(rx - fw), abs(ry)) + 2
        max_steps = int((max(fw, fh) + extra) / step)

        for _ in range(max_steps):
            x += dx * step
            y += dy * step
            # 扩展边界: 允许从场外 (-2m margin) 开始遍历
            if x < -2 or x >= fw + 2 or y < -2 or y >= fh + 2:
                break

            # 场外段: 仍追踪仰角但不标记格子
            in_field = (0 <= x < fw) and (0 <= y < fh)

            # 当前格到雷达水平距离
            hdist = np.sqrt((x - rx)**2 + (y - ry)**2)
            if hdist < 0.1:
                continue

            if not in_field:
                continue   # 场外: 无格子可标记, 继续前进入场

            xi = int(x / res)
            yi = int(y / res)
            if xi >= gw or yi >= gh:
                break

            # 机器人目标仰角 (从雷达看机器人顶, 往下为负)
            tgt_elev = (Z_TARGET - rz) / hdist  # tan(elevation angle)

            # 如果已有更高障碍物把这个方向压住
            if tgt_elev < max_elev:
                # 目标被遮挡
                if not visited[yi, xi]:
                    visible[yi, xi] = False
            else:
                visible[yi, xi] = True

            visited[yi, xi] = True

            # 更新最高障碍物仰角
            oz = hmap[yi, xi]
            if oz > -np.inf:
                obs_elev = (oz - rz) / hdist
                if obs_elev > max_elev:
                    max_elev = obs_elev

    return visible

# ── 主流程 ─────────────────────────────────────────────────────────────────────
print(f'加载 PCD: {PCD_PATH}  ...', end='', flush=True)
pts = load_pcd(PCD_PATH)
print(f'  {len(pts):,} 点')

print('构建高度图 ...', end='', flush=True)
hmap = build_height_map(pts)

# ── 能量机关区域: 仅保留两根立柱阴影, 移除旋转臂冻结快照 ────────────────────
# 高度图分析确认: 结构斜置约45°
#   立柱1 中心 ≈ (13.50, 6.75)  立柱2 中心 ≈ (14.62, 8.00)
#   旋转臂快照在中间 (z>3.4m 的单元就是臂尖)
# 做法: EM 区域内 z>1.8m 的单元, 不在立柱圆内的全部清除
_r = GRID_RES
_xi0, _xi1 = int(12.0/_r), int(16.0/_r)
_yi0, _yi1 = int( 6.0/_r), int( 9.0/_r)

_wx = np.arange(_xi0, _xi1) * _r          # (n_x,)
_wy = np.arange(_yi0, _yi1) * _r          # (n_y,)
_WX, _WY = np.meshgrid(_wx, _wy)           # (n_y, n_x)

# 两根立柱位置 (从高度图直接读取)
_PILLARS = [(13.50, 6.75), (14.62, 8.00)]
_PILLAR_R = 0.40   # 立柱包围半径 (m)
_pillar_mask = np.zeros((_yi1-_yi0, _xi1-_xi0), dtype=bool)
for _px, _py in _PILLARS:
    _pillar_mask |= np.sqrt((_WX-_px)**2 + (_WY-_py)**2) <= _PILLAR_R

_region = hmap[_yi0:_yi1, _xi0:_xi1]
# 清除非立柱区域中 z>1.8m 的旋转臂/横梁结构
hmap[_yi0:_yi1, _xi0:_xi1] = np.where(
    (_region > 1.8) & ~_pillar_mask, -np.inf, _region)
print(' done')

print('仿真红方遮挡 ...', end='', flush=True)
vis_red  = compute_occlusion(RADAR_RED,  hmap)
print(' done')
print('仿真蓝方遮挡 ...', end='', flush=True)
vis_blue = compute_occlusion(RADAR_BLUE, hmap)
print(' done')

# ── 隐道盲区: 隐道内机器人完全隐藏 (高度图捕不到 0.25m 屋顶) ──────────
# 隐道尺寸 (图 4-34): 内宿 700mm×250mm, 内部高度仅 250mm
# 坐标来自 STL 高度图捕识和公路区尺寸 (采用保守估计)
_TUNNELS = [
    # (x0, x1, y0, y1) — 每条隐道居守盲区
    # 安全地: 基于公路区实测模型位置 (约 x≈8-12, y≈3-4/11-12)
    ( 8.0, 12.0,  3.0,  4.2),   # 红方下隐道
    ( 8.0, 12.0, 10.8, 12.0),   # 红方上隐道
    (16.0, 20.0,  3.0,  4.2),   # 蓝方下隐道 (对称)
    (16.0, 20.0, 10.8, 12.0),   # 蓝方上隐道 (对称)
]
for _tx0, _tx1, _ty0, _ty1 in _TUNNELS:
    _txi0, _txi1 = int(_tx0/GRID_RES), int(_tx1/GRID_RES)
    _tyi0, _tyi1 = int(_ty0/GRID_RES), int(_ty1/GRID_RES)
    vis_red [ _tyi0:_tyi1, _txi0:_txi1] = False  # 强制盲区
    vis_blue[ _tyi0:_tyi1, _txi0:_txi1] = False

os.makedirs(OUT_DIR, exist_ok=True)
np.save(os.path.join(OUT_DIR, 'shadow_red.npy'),  vis_red)
np.save(os.path.join(OUT_DIR, 'shadow_blue.npy'), vis_blue)
# 同时保存 C++ 可直接读取的 raw uint8 二进制 (行优先, 1=可见, 0=盲区)
vis_red.astype(np.uint8).tofile(os.path.join(OUT_DIR, 'shadow_red.bin'))
vis_blue.astype(np.uint8).tofile(os.path.join(OUT_DIR, 'shadow_blue.bin'))
print('已保存 shadow_red.npy / shadow_blue.npy / shadow_red.bin / shadow_blue.bin')

# ── 可视化 ──────────────────────────────────────────────────────────────────────
bg = None
if os.path.exists(TOPVIEW):
    bg = mpimg.imread(TOPVIEW)[::-1]   # y-flip

fig, axes = plt.subplots(1, 2, figsize=(20, 8))
fig.patch.set_facecolor('#0d1117')
fig.suptitle(
    f'LiDAR遮挡仿真  z_radar={Z_RADAR}m  z_target={Z_TARGET}m\n'
    f'红方雷达({RADAR_RED[0]},{RADAR_RED[1]})  蓝方雷达({RADAR_BLUE[0]},{RADAR_BLUE[1]})',
    color='#e6edf3', fontsize=11, fontweight='bold')

gh, gw = hmap.shape
extent_2d = [0, FIELD_W, 0, FIELD_H]

for ax, vis, title, radar, color in [
    (axes[0], vis_red,  '红方视角遮挡', RADAR_RED,  '#ff3333'),
    (axes[1], vis_blue, '蓝方视角遮挡', RADAR_BLUE, '#3399ff'),
]:
    ax.set_facecolor('#0d1117')
    ax.set_xlim(0, FIELD_W); ax.set_ylim(0, FIELD_H)
    ax.set_aspect('equal')
    ax.set_title(title, color='#e6edf3', fontsize=10, fontweight='bold')
    ax.tick_params(colors='#666', labelsize=7)
    for sp in ax.spines.values(): sp.set_color('#30363d')
    ax.set_xlabel('X (m)', color='#666', fontsize=8)
    ax.set_ylabel('Y (m)', color='#666', fontsize=8)

    # 底图
    if bg is not None:
        ax.imshow(bg, extent=extent_2d, aspect='auto', zorder=0, alpha=0.35, origin='lower')

    # 遮挡叠加 (半透明红色)
    # Upsample to plot extent
    occ_img = (~vis).astype(np.float32)   # occluded = 1
    ax.imshow(occ_img, extent=extent_2d, aspect='auto', zorder=2,
              alpha=0.55, origin='lower', cmap='Reds', vmin=0, vmax=1)

    # 雷达位置
    ax.plot(radar[0], radar[1], '*', ms=14, color=color, zorder=8, label='雷达位置')

    # 中线
    ax.axvline(FIELD_W/2, color='white', lw=0.8, ls='--', alpha=0.3)

    ax.legend(loc='upper right', fontsize=8, facecolor='#0d1117',
              edgecolor='#30363d', labelcolor='#c9d1d9')

plt.tight_layout(rect=[0,0,1,0.93])
out_png = os.path.join(OUT_DIR, 'shadow_vis.png')
plt.savefig(out_png, dpi=120, bbox_inches='tight', facecolor=fig.get_facecolor())
print(f'已保存: {out_png}')
