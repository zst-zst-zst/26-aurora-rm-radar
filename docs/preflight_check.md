# 赛前功能测试 — 主程序验证清单

> 用 `match.launch.py` 实际运行，逐项确认各子系统工作正常。
> 建议在适应性训练日 / 赛前热身时完整跑一遍。

---

## 系统链路总览

`ros2 launch tdt_vision match.launch.py` 启动后的完整调用链：

```
radar_runtime.yaml (配置入口)
        │
        ▼
match.launch.py ─── 读取配置 → 根据 team 选择标定文件 → 启动以下节点:

╔══════════════════════════════════════════════════════════════╗
║ camera_detector_container (单进程, 零拷贝通信)               ║
║                                                              ║
║   NodeCamera ──image──→ Detect ──detect_result──→ Resolve    ║
║   (海康相机)            (YOLO推理     (像素→世界坐标          ║
║                          +3D框叠加)    +HeightGrid地形)       ║
║                                                              ║
║   [BagRecorder] ← 可选自动录包                                ║
╚══════════════════════════════════════════════════════════════╝
                                                    │
                                              /resolve_result
                                                    │
╔═══════════════════════════════════════╗            ▼
║ localization_container                ║   ┌──────────────────┐
║   Localization (CUDA ICP点云配准→TF)  ║   │ radar_serial_node│
╠═══════════════════════════════════════╣   │ (裁判系统串口)    │
║ dynamic_cloud_container               ║   │ 收: match_info   │
║   DynamicCloud (动态点提取)           ║   │ 发: 0x0305 ≤5Hz  │
╠═══════════════════════════════════════╣   └──────────────────┘
║ cluster_container                     ║           │
║   Cluster (聚类) → KalmanFilter (跟踪)║           ▼
╚═══════════════════════════════════════╝   裁判系统小地图显示

╔══════════════════════╗    ╔══════════════════╗
║ map_server (nav2)    ║    ║ debug_map (可选) ║
║ 提供地图服务          ║    ║ 本地minimap显示  ║
╚══════════════════════╝    ╚══════════════════╝
```

### 数据流简述

1. **相机** → 图像 → **Detect** (YOLO检测 + armor识别 + 3D框) → 检测结果
2. **Detect** → **Resolve** (用外参将像素投到世界坐标, HeightGrid修正地形高度)
3. **LiDAR** → **Localization** (ICP配准) → **DynamicCloud** → **Cluster** → **KalmanFilter** (融合跟踪)
4. **Resolve + KalmanFilter** → **radar_serial_node** → 裁判系统 (0x0305, ≤5Hz)
5. 裁判系统 → **radar_serial_node** (接收 match_info / 比赛状态)

### 关键配置文件对应

| 节点 | 读取的配置 |
|------|-----------|
| NodeCamera | `config/camera_driver_{day,night}.yaml` |
| Detect | 内参 `hik.yaml` + 外参 `out_matrix_*.yaml` (3D框) |
| Resolve | 外参 + `field_mesh.bin` + `map_points.yaml` |
| Localization | `config/map/map.pcd` (场地点云) |
| radar_serial | `serial_port` + `serial_baud` (从 runtime.yaml) |

---

## 准备

```bash
cd /home/zst/T && source /opt/ros/jazzy/setup.bash && source install/setup.bash
nano config/radar_runtime.yaml   # 确认 team 正确
```

---

## 1. 相机链路

```bash
ros2 launch tdt_vision match.launch.py
```

**验证**：
- [ ] 日志出现 `Load yolo engine success!` 和 `Load armor_yolo engine success!`
- [ ] 日志出现 `Detect node has been started`
- [ ] `armor id:` 行持续输出（说明推理在跑）
- [ ] detect 窗口画面正常、无花屏

**异常处理**：
- 无画面 → 检查 USB 3.0 连接，`ls /dev/video*`
- CUDA 报错 → `nvidia-smi` 确认 GPU 可用

---

## 2. 3D 投影（外参标定）

**验证**：
- [ ] 日志出现 `3D box overlay enabled: camera=..., extrinsics=...`
- [ ] detect 窗口中机器人周围有蓝色 3D 长方体框
- [ ] 框贴合机器人轮廓，无明显偏移

**异常处理**：
- 无 3D 框 → 检查 `out_matrix_*.yaml` 是否存在、路径是否正确
- 框偏移严重 → 重新标定 `ros2 launch tdt_vision calib.launch.py`

---

## 3. LiDAR 链路

**验证**：
- [ ] 日志出现 `CUDA ICP: target=... source=... CONVERGED`
- [ ] ICP 在前几帧内收敛（≤10 frames）
- [ ] 无持续 `TF rm_frame ← livox_frame unavailable` 警告（前几帧可忽略）

**异常处理**：
- 无 ICP 输出 → `ping 192.168.1.1xx` 检查网线连接
- 持续不收敛 → 检查点云地图 `config/map/map.pcd` 是否为当前场地

---

## 4. 串口通信（裁判系统）

**验证**：
- [ ] 日志出现 `Referee RX detected`
- [ ] 日志出现 `TX radar packet` 且 `active_enemies > 0`
- [ ] 裁判系统客户端小地图上出现雷达标记点

**异常处理**：
- 无 `Referee RX` → `ls -l /dev/gimbal` 确认设备存在
- 设备不存在 → 检查 USB-TTL 接线、换备用模块
- 有 RX 无 TX → 检测到机器人但未成功发送，看 CRC 错误日志

---

## 5. Resolve 定位融合

**验证**：
- [ ] 日志出现 `Resolve calibration: out_matrix=... camera_world=(...)`
- [ ] 日志出现 `HeightGrid Loaded config/map/field_mesh.bin`
- [ ] `cam_bias=` 输出值在 (±1.0, ±1.0) 范围内

**异常处理**：
- cam_bias 过大 → 外参可能偏了，重新标定
- HeightGrid 加载失败 → 检查 `config/map/field_mesh.bin` 是否存在

---

## 6. 整体联调确认

启动后运行 30 秒以上，确认：

- [ ] detect 窗口持续显示检测结果 + 3D 框
- [ ] 裁判小地图有点位且位置合理
- [ ] 无 crash / segfault / 进程异常退出
- [ ] topic 频率正常：

```bash
ros2 topic hz /detect_result --window 5    # 应 ≥1 Hz
ros2 topic hz /radar2sentry --window 5     # 应 ≤5 Hz 且 >0
```

---

## 7. 关机前

```bash
# Ctrl+C 停止主程序，确认所有进程退出
pkill -f rclcpp_components 2>/dev/null
```

---

## 快速通过标准

全部 ✓ = 可以上场比赛。任一项 ✗ = 定位问题后修复再重跑本清单。
