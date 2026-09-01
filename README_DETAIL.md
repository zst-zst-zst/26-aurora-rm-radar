# T 雷达站 — RMUC2026 详细说明

## 1. 启动（30 秒）

```bash
cd /home/zst/T && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch tdt_vision calib.launch.py
ros2 launch tdt_vision match.launch.py
```

确认：日志出现 `Referee RX detected` + `active_enemies > 0`，裁判小地图有点位即可。

---

## 2. 接线

| 设备 | 接口 | 备注 |
|------|------|------|
| 海康 MV-CS060 (6MP) | USB 3.0 → 主机 | 内参 `camera/config/hik.yaml` |
| Livox Mid-70 | 以太网直连 `eno1` | 主机自动配 `192.168.1.50/24` |
| 裁判系统串口 | USB-TTL → `/dev/gimbal` | 115200 8N1，热熔胶固定接头 |

**雷达基座**：上层放传感器（距地 2.5m），下层有 220V 电源 + HDMI 显示器。
上层**无电源**——必须带 4-5m 插线板从下方穿线孔引电上去给 LiDAR 供电。

---

## 3. 配置：`config/radar_runtime.yaml`

只改 `pre_match` 段：

| 字段 | 值 | 说明 |
|------|---|------|
| **`team`** | `0`红 / `1`蓝 | 比赛前必改 |
| `serial_port` | `/dev/gimbal` | 串口路径 |

---

## 4. 标定（5 点 PnP，~2 分钟）

每次雷达重新上架后做：

```bash
ros2 launch tdt_vision calib.launch.py
```

依次点 5 个点：我方堡垒 → 我方前哨顶角 → 敌方基地 → 敌方前哨顶角 → 中央高塔顶角。
结果写入 `config/{red,blue}/out_matrix_*.yaml`。

验证：detect 窗口 3D 框贴合机器人轮廓。歪了就重标。

---

## 5. 比赛日流程

### 时间线

```
T-60 min  检录（带卷尺、备用 USB-TTL、杜邦线、尖口钳、裁判系统模块）
T-10 min  候场区等待
T-3  min  进场 — 三分钟准备阶段
T-30 s    必须上电，人员离场
```

### 三分钟准备 checklist

```
□ 上一场队伍雷达搬下来后立刻把我方雷达放上去
□ 穿线：网线 + 串口 + 电源（插线板）从线槽孔穿过，线头标签区分
□ 两人配合：一人上面接传感器，一人下面接主机
□ 主机上电 → 运行已准备好的启动命令（排队时提前打开终端）：
    ros2 launch tdt_vision match.launch.py
□ 比赛不随主链路启动 debug_map / Foxglove；需要本机调试小地图时另开终端：
    ros2 launch debug_map map.launch.py
□ 改 team → 验证日志 → 看裁判小地图
□ T-15s 前一切就绪，否则申请技术暂停
```

### 每场之间（中场）

```
□ 改 team（BO 可能换边）
□ 重做 PnP 标定（雷达被搬动过，外参失效）：
    ros2 launch tdt_vision calib.launch.py
□ 启动比赛：
    ros2 launch tdt_vision match.launch.py
□ 验证 3D 框 + 裁判小地图
```

**时间窗口**：BO3/BO5 局间 3 分钟；第二/四局后 10+3=13 分钟。

### 实战注意事项（来自 2025 赛季总结）

- **USB-TTL 接头打热熔胶**，防止被扯脱
- **带 4-5m 插线板** + 卷线盘收纳，LiDAR 电源从下层引上去
- **线从孔里穿时留好接口端**，别全掉下去跑上跑下浪费时间
- **两人协作**：相机对焦需要一人上面看画面一人下面操作电脑（或 HDMI 延长线 + 采集卡到手机）
- **排队时就把程序调到"插线回车就跑"**，3 分钟很紧
- **每个参数修改必须赛前测试验证**，不能临场随意改

---

## 6. 故障排查

| 现象 | 处置 |
|------|------|
| 小地图无点位 | team 设错 / `ls /dev/gimbal` 确认串口 |
| detect 无输出 | `nvidia-smi` 看 GPU |
| LiDAR 无数据 | `ping 192.168.1.1xx` 检查网线 |
| 3D 框歪 | 重新标定 |
| 坐标全反 | team 红蓝反了 |
| 进程残留 | `pkill -9 -f rclcpp_components` |

---

## 7. 回放

```bash
ros2 launch tdt_vision calib_bag.launch.py
ros2 launch tdt_vision bag.launch.py
# 配置见 radar_runtime.yaml -> bag 段
# bag 与实时共用 hik.yaml 内参；bag_out_matrix_*.yaml 只隔离回放外参
```

### Foxglove 看回放雷达/点云

比赛实时不用 Foxglove；只在回放/离线排查时开。

开两个终端：

```bash
ros2 launch tdt_vision bag.launch.py
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

Foxglove 连接：

```text
ws://localhost:8765
```

3D 面板设置：

```text
Fixed frame = rm_frame
```

常用显示项：

```text
/map
/livox/map
/livox/lidar
/livox/lidar_cluster
/livox/lidar_kalman
/camera_point3D
/tf
```

如果没有 `rm_frame`，先确认回放仍在运行，并检查：

```bash
ros2 topic echo /tf --once
```

正常应看到 `frame_id: rm_frame`、`child_frame_id: livox_frame`。

---

## 8. 串口协议要点（官方 V1.3.0）

实现：`src/fusion/kalman_filter/src/radar_serial_node.cpp`

- 帧格式：SOF(0xA5) + len + seq + CRC8/CRC16
- 发送 cmd_id=0x0305，48B，频率 ≤5Hz
- 单位 cm，x=y=0 视为未发送
- 标记精度：`<0.8m` 准确(+1)，`0.8-1.6m` 半准(+0.5)，`≥1.6m` 错误(-0.8)
- 标记进度 P：100→15% 易伤，120→20% 易伤
- 双倍易伤：每局 2 次，每次 30s


ros2 launch tdt_vision calib_bag.launch.py \
  rosbag_file:=bags/match_20260522_195954 \
  self_color_override:=2 \
  show_image:=true
