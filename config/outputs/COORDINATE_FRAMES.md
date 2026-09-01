# RMUC2026 坐标系定义与转换（标准）

> **保留此文件**。`config/outputs/` 下所有点云、PNG、`.pkl/.npz` 缓存均使用这里定义的**裁判坐标系**。

## 1. 裁判坐标系（Referee Frame）

| 项 | 值 |
|----|----|
| X 轴 | 0 → 28 m（场地长边） |
| Y 轴 | 0 → 15 m（场地短边） |
| Z 轴 | 0 m 为比赛地面，+Z 向上 |
| 原点 O | (0, 0, 0)，场地左前下角 |
| **对称中心** | **(14, 7.5, ?)**（X、Y 都是 1/2 处） |
| 红蓝互换 | `x_red = 28 - x_blue`，`y_red = 15 - y_blue`，`z_red = z_blue` |

## 2. 源坐标系（CAD 居中系）

官方 STEP 文件 `RMUC2026_V1.2.0.step` 经 FreeCAD 镶嵌后得到的 STL/PCD 默认在该系下：

| 轴 | 范围 |
|----|------|
| X | [-14.88, 14.88] m |
| Y | [-6.40, 9.65] m |
| Z | [-1.84, 1.86] m |

> 注意 X 轴关于 0 对称（场地中心在 x=0），但 Y、Z 不对称——CAD 把 z=0 设在了塔顶水平面附近，而不是地面。

## 3. 标准变换（CAD → 裁判系）

**仅平移，无旋转/缩放，单位米：**

```
x_referee = x_centered + 14.00
y_referee = y_centered + 5.87
z_referee = z_centered + 1.64
```

向量形式：

```python
T = np.array([14.0, 5.87, 1.64])
pts_referee = pts_centered + T
```

平移后裁判系下的实际 bbox（保留官方 STEP 的略微外溢，不要再裁）：

| 轴 | 范围 |
|----|------|
| X | [-0.88, 28.88] m |
| Y | [-0.53, 15.52] m |
| Z | [-0.20, 3.50] m |

> 官方 STEP 比 28×15 略大，bbox 故意保留这部分外溢，以避免裁切边缘的标定特征。坐标值本身完全正确。

## 4. 配套工具

- `tools/cad_to_referee.py <in.pcd> <out.pcd>` — 把 CAD 居中系 PCD 平移到裁判系（加 `--inverse` 反向）
- `tools/_step_to_stl.py` — STEP → STL（FreeCAD）
- `tools/_stl_to_pcd.py <stl> <pcd> <n_samples> <scale_to_m> <voxel_m>` — STL → CAD 系 PCD
- `tools/step_to_pcd.sh` — 一键 STEP → STL → PCD（CAD 系）
- `tools/extract_field_surface.py --grid-size 0.01` — 裁判系 PCD → max-Z 表面缓存
- `tools/visualize_field_surface.py` — 缓存 → PNG

## 5. 当前缓存（裁判系，1 cm 网格）

| 文件 | 内容 |
|------|------|
| `outputs/RMUC2026_field_dense.pcd` → `config/RMUC2026_ground_only_1cm.pcd` | 12.99 M 点（30 M 采样 → voxel 0.01 m） |
| `outputs/ground_surface_dense.pkl/.npz` | 2977×1606 = 4.78 M 格子，1 cm 网格，max-Z 提取，Z=0 钳位（45.2%） |
| `outputs/ground_surface_dense.png` | 渲染图，三轴汇于 O(0,0,0)，对称中心 (14, 7.5) 标注 |

## 6. 红方 5 个标定点（裁判系）

| 键名 | (X, Y, Z) m |
|------|-------------|
| `self_fortress` | (7.93210, 7.53070, 0.00000) |
| `self_tower`    | (10.82200, 3.66430, 1.86840) |
| `enemy_base`    | (23.01710, 4.45300, 0.59980) |
| `enemy_tower`   | (24.54140, 11.08010, 0.73160) |
| `cross_tower`   | (17.17800, 11.33570, 1.86840) — `self_tower` 关于 (14, 7.5) 的对称点 |

蓝方点 = 红方点关于 (14, 7.5) 对称。详见 `config/red/calibrate_points_red.yaml` 与 `config/blue/`。

## 7. 完整重建流水线（从 STEP 开始）

```bash
# 1) STEP → STL (≈3 min, FreeCAD, dev=5mm 是最高密度)
tools/step_to_pcd.sh /home/zst/RMUC2026_V1.2.0.step  # 默认参数太低
# 推荐手动：
freecadcmd tools/_step_to_stl.py \
    /home/zst/RMUC2026_V1.2.0.step \
    /home/zst/RMUC2026_V1.2.0.dev5.0.stl 5.0

# 2) STL → CAD 系 PCD（30 M 采样, voxel 0.01 m, ≈70 s）
python3 tools/_stl_to_pcd.py \
    /home/zst/RMUC2026_V1.2.0.dev5.0.stl \
    /tmp/RMUC2026_dense_centered.pcd \
    30000000 0.001 0.01

# 3) CAD 系 → 裁判系（≈0.5 s）
python3 tools/cad_to_referee.py \
    /tmp/RMUC2026_dense_centered.pcd \
    config/RMUC2026_ground_only_1cm.pcd

ln -sf $(realpath config/RMUC2026_ground_only_1cm.pcd) \
       config/outputs/RMUC2026_field_dense.pcd

# 4) 表面提取 @ 1 cm 网格（≈1.5 s）
python3 tools/extract_field_surface.py --grid-size 0.01

# 5) 渲染（≈3 s）
python3 tools/visualize_field_surface.py
```
