## 1. 比赛启动

```bash
cd /home/zst/T
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch tdt_vision calib.launch.py

ros2 launch tdt_vision match.launch.py map:=1
```

比赛不开画面但开小地图：

```bash
ros2 launch tdt_vision match.launch.py show_image:=false map:=1
```

## 需要本机小地图调试时另开终端：

```bash
ros2 launch debug_map map.launch.py
```

---

## 2. 红蓝方设置

通常比赛前提前设置好；换边/BO 切边时再改。

改：`config/radar_runtime.yaml`

```yaml
pre_match:
  team: 0   # 0=红方，1=蓝方
```

---

## 3. 标定点顺序

```text
我方堡垒 -> 我方前哨顶角 -> 敌方基地 -> 敌方前哨顶角 -> 中央高塔顶角
```

结果写入：

```text
config/red/out_matrix_red.yaml
config/blue/out_matrix_blue.yaml
```

---

## 4. 比赛可能出现的问题

### 相机明明接了，但是打不开

程序已支持相机自动重试打开/自动重连；启动后先等 5-10 秒。

如果一直没有画面，先看系统有没有识别到相机：

```bash
lsusb
```

看到 `Hikrobot MV-CS060-10UC-PRO` 就继续等自动恢复；看不到再处理 USB：

```bash
pkill -9 -u $USER -f rclcpp_components
pkill -9 -u $USER -f camera
pkill -9 -u $USER -f ros2
```

拔插相机 USB3.0 -> 换 USB3.0 口 -> 确认不用 HUB。

```bash
ros2 launch tdt_vision calib.launch.py
```

### 裁判串口没点位

```bash
ls /dev/gimbal
```

如果没有 `/dev/gimbal`：

```text
重新插拔 USB-TTL -> 检查热熔胶/杜邦线 -> 重启主程序
```

### LiDAR 没数据

```bash
ros2 topic list | grep livox
ping 192.168.1.1
```

处理：

```text
检查网线/电源 -> 等 10 秒 -> 重启主程序
```

### 程序卡住或进程残留

```bash
pkill -9 -f rclcpp_components
pkill -9 -f ros2
```

然后重新运行比赛启动命令。

### 3D 框明显歪

```bash
ros2 launch tdt_vision calib.launch.py
```

重新标定外参。

---

## 5. 回放

回放入口：

```bash
ros2 launch tdt_vision calib_bag.launch.py
ros2 launch tdt_vision bag.launch.py
```

Foxglove 回放入口：

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

---

## 6. 关键文件

```text
config/radar_runtime.yaml                         # 比赛/回放主配置
src/tdt_vision/camera/config/hik.yaml             # HIK 相机内参
config/red/out_matrix_red.yaml                    # 红方比赛外参
config/blue/out_matrix_blue.yaml                  # 蓝方比赛外参
config/red/bag_out_matrix_red.yaml                # 红方回放外参
config/blue/bag_out_matrix_blue.yaml              # 蓝方回放外参
README_DETAIL.md                                  # 详细说明
```

---

## 7. 回放发送串口数据测试

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch tdt_vision bag.launch.py \
  rosbag_file:=bags/match_20260522_195954 \
  start_offset_sec:=180 \
  enable_lidar_pipeline:=true \
  enable_serial:=true \
  serial_dry_run:=false \
  serial_port:=/dev/gimbal \
  serial_use_resolve_fallback:=true \
  self_color_override:=2 \
  show_image:=false
```
