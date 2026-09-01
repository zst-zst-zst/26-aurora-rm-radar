# RMUC2026 场地输出文件

本目录存放所有下游工具可以直接取用的产出物。
两条主要流水线：**(a)** 一次性地面表面缓存 和 **(b)** 可直接使用的可视化图像，
均使用与 `map_points_referee_3d.png` 相同的裁判坐标系。

裁判坐标系定义：
- X: 0 → 28 m
- Y: 0 → 15 m
- Z: 0 m 为比赛场地地面，+Z 向上
- 对称中心：**(14, 7.5)**
- 红蓝互换：`x_red = 28 - x_blue`, `y_red = 15 - y_blue`, `z_red = z_blue`

## 源 CAD 文件（符号链接）

| 文件 | 链接目标 | 说明 |
|------|----------|------|
| `RMUC2026_V1.2.0.step` | `/home/zst/RMUC2026_V1.2.0.step` (993 MB) | 官方原始 STEP 文件 |
| `RMUC2026_V1.2.0.dev5.0.stl` | `/home/zst/RMUC2026_V1.2.0.dev5.0.stl` (527 MB) | 最高密度 STL（FreeCAD tessellate 偏差 = 5 mm） |
| `RMUC2026_field_dense.pcd` | `/home/zst/T/config/RMUC2026_ground_only.pcd` (22 MB) | 裁判坐标系下最密点云（1 861 210 点） |

> 官方 STEP 模型比实际场地 28 × 15 m 稍大，因此 PCD 覆盖范围为
> `x ∈ [-0.88, 28.88]`、`y ∈ [-0.53, 15.53]`。
> 刻意保留这个超出部分，以避免场边标定特征被裁切。坐标值本身是正确的。

## 缓存的地面表面（只需提取一次）

由 `tools/extract_field_surface.py` 从 `RMUC2026_field_dense.pcd` 生成。

| 文件 | 说明 |
|------|------|
| `ground_surface_dense.pkl` | 包含 `X`、`Y`、`Z` meshgrid 和元数据的字典 |
| `ground_surface_dense.npz` | 同样的数组，numpy 格式 |

统计（默认参数）：网格 0.05 m，596 × 322 = 191 912 个格子，97.5% 有效，
Z ∈ [-0.20, 3.50] m。只有源 PCD 或所需分辨率改变时才需要重新运行。

```bash
# 重新生成缓存（一般不需要）
python3 tools/extract_field_surface.py \
    --input-pcd config/outputs/RMUC2026_field_dense.pcd \
    --grid-size 0.05
```

## 可视化图像

| 文件 | 生成脚本 | 备注 |
|------|----------|------|
| `ground_surface_dense.png` | `tools/visualize_field_surface.py` | 裁判坐标系 3D 表面；布局/刻度/视角/box-aspect 与 `map_points_referee_3d.png` 一致；红方标定点叠加显示；在 X = 0, 14, 15, 28 和 Y = 0, 7.5, 15 处单独画了参考网格线 |
| `map_points_referee*.png` | `src/utils/scripts/visualize_map_points_2d3d.py` | 同一坐标系下的区域多边形参考 |
| `grid_lines_x15_y7.5.png`、`high_density_grid.png`、`coordinate_grid_with_points.png` | `tools/create_coordinate_grids.py` | 2D 网格参考 |

```bash
# 从缓存重新渲染图片（很快，~1 秒）
python3 tools/visualize_field_surface.py
```

## 红方标定点

来源：`config/red/calibrate_points_red.yaml`。
从 `cali.png` 中提取（四个与官方 STEP 一一对应的点 + cross_tower 为 self_tower
关于场地中心的对称点，替换掉了原来 Y = -1340.3 mm 的那个点）。
顺序为 `cali.png` 中最左边的点开始，逆时针排列。

| 键名 | (X, Y, Z) m |
|------|-------------|
| `self_fortress` | (7.93210, 7.53070, 0.00000) |
| `self_tower`    | (10.82200, 3.66430, 1.86840) |
| `enemy_base`    | (23.01710, 4.45300, 0.59980) |
| `enemy_tower`   | (24.54140, 11.08010, 0.73160) |
| `cross_tower`   | (17.17800, 11.33570, 1.86840) — `self_tower` 的对称点 |

## 遗留文件（保留供参考）

`comprehensive_surface.pcd`、`high_density_surface.pcd`、
`_step_to_stl.py`、`_stl_to_pcd.py`、`step_to_pcd.sh` — 早期转换产物。
标准转换流水线统一放在 `tools/` 下：

- `tools/_step_to_stl.py`       — FreeCAD: STEP → STL
- `tools/_stl_to_pcd.py`        — STL → PCD（面积加权采样 + 体素降采样）
- `tools/step_to_pcd.sh`        — 完整 STEP → STL → PCD 流水线
- `tools/extract_field_surface.py` — PCD → max-Z 网格缓存（本目录）
- `tools/visualize_field_surface.py` — 缓存 → PNG 可视化（本目录）
俯视图 (Top View XY)
能清楚看到各检测区与 3D 地图的对应关系：

红色菱形 (FORBIDDEN) — 中心障碍禁区，旋转 45° 的矩形精确框住了中央高低地障碍物。地图上灰色结构与红色框吻合 ✅
橙色小方块 (DART) — 飞镖检测区，在蓝方基地前方 x≈27.2-27.4，Y=3.64-4.23 的狭缝中。橙色虚线箭头是飞镖飞行路径。可以看到检测区恰好在蓝方基地结构旁边 ✅
红/黄/绿三层 (FLY ALARM/WARN/SAFE) — 飞机检测区的三级警报，Y 方向覆盖 [0.2, 5.24]（包含红方飞坡 + 安全绳范围）。青色填充的 Red FlySlope 在检测区内 ✅
紫色区域 (HERO) — 英雄检测区在蓝方堡垒到基地之间、y > 10.57（红方梯形高地下界）。紫色竖虚线 x=9.2 是 hero_threshold ✅
侧视图 (Side View XZ)
Y=3.6~4.3 截面的侧视：

橙色方块 (DART) 在 x≈27、z=[1.71, 2.47]，正好在基地顶部 (1.38m) 和围挡 (2.4m) 之间
青色区域 (FLY) 在 z=[1.7, 3.0]，只检测空中目标，不会误检地面机器人
可以看到蓝方基地结构（x≈25-27，高耸灰色点）恰好在飞镖检测区左侧
