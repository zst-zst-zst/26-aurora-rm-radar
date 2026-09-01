#!/usr/bin/env python3
"""
串口测试脚本 —— 绕过识别和 map，直接向 radar_serial_node 发送机器人位置，
用于验证串口能否正常给裁判系统发 0x0305 map_robot_data。

发送内容：
  - 红方步兵 3 号  →  (7.0, 7.5)   红方半场中心
  - 蓝方步兵 3 号  →  (21.5, 7.5)  蓝方半场中心

使用方法：
  1) 先启动 radar_serial_node（需要设置正确的 self_color_override）：
       ros2 launch kalman_filter radar_serial.launch.py self_color_override:=0
     其中 0=蓝方, 2=红方。

  2) 再运行本脚本（self_color 需要和 serial node 一致）：
       source /home/zst/T/install/setup.bash
       python3 /home/zst/T/send_fake_positions.py --self-color 0

  Ctrl+C 停止。
"""

import argparse

import rclpy
from rclpy.node import Node
from vision_interface.msg import Radar2Sentry


# ---------- 固定测试坐标（单位：米，基于 28×15 地图） ----------
RED_INFANTRY3_X = 7.0
RED_INFANTRY3_Y = 7.5
BLUE_INFANTRY3_X = 21.5
BLUE_INFANTRY3_Y = 7.5
# Radar2Sentry 数组 slot: 0=英雄 1=工程 2=步兵3 3=步兵4 4=空中 5=哨兵
INFANTRY3_SLOT = 2


class TestSerialSender(Node):
    def __init__(self, self_color: int, hz: float):
        super().__init__("test_serial_sender")
        self.self_color = self_color
        self.pub = self.create_publisher(Radar2Sentry, "/radar2sentry", 10)
        self.timer = self.create_timer(1.0 / hz, self.on_timer)
        color_str = "蓝方" if self_color == 0 else "红方"
        self.get_logger().info(
            f"测试发送节点已启动  己方={color_str}  发送频率={hz:.1f}Hz"
        )
        self.get_logger().info(
            f"红方步兵3 → ({RED_INFANTRY3_X}, {RED_INFANTRY3_Y})  "
            f"蓝方步兵3 → ({BLUE_INFANTRY3_X}, {BLUE_INFANTRY3_Y})"
        )

    def on_timer(self):
        msg = Radar2Sentry()
        # float32[6] 数组默认全 0

        if self.self_color == 0:
            # 己方=蓝 → enemy=红, self=蓝
            msg.radar_enemy_x[INFANTRY3_SLOT] = RED_INFANTRY3_X
            msg.radar_enemy_y[INFANTRY3_SLOT] = RED_INFANTRY3_Y
            msg.radar_self_x[INFANTRY3_SLOT] = BLUE_INFANTRY3_X
            msg.radar_self_y[INFANTRY3_SLOT] = BLUE_INFANTRY3_Y
        else:
            # 己方=红 → enemy=蓝, self=红
            msg.radar_enemy_x[INFANTRY3_SLOT] = BLUE_INFANTRY3_X
            msg.radar_enemy_y[INFANTRY3_SLOT] = BLUE_INFANTRY3_Y
            msg.radar_self_x[INFANTRY3_SLOT] = RED_INFANTRY3_X
            msg.radar_self_y[INFANTRY3_SLOT] = RED_INFANTRY3_Y

        self.pub.publish(msg)
        self.get_logger().info(
            f"已发布 → enemy_slot{INFANTRY3_SLOT}="
            f"({msg.radar_enemy_x[INFANTRY3_SLOT]:.1f}, "
            f"{msg.radar_enemy_y[INFANTRY3_SLOT]:.1f})  "
            f"self_slot{INFANTRY3_SLOT}="
            f"({msg.radar_self_x[INFANTRY3_SLOT]:.1f}, "
            f"{msg.radar_self_y[INFANTRY3_SLOT]:.1f})",
            throttle_duration_sec=3.0,
        )


def main():
    parser = argparse.ArgumentParser(
        description="串口测试：直接发送机器人位置给 radar_serial_node"
    )
    parser.add_argument(
        "--self-color",
        type=int,
        default=0,
        choices=[0, 2],
        help="己方颜色：0=蓝方, 2=红方（默认 0）",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=5.0,
        help="发送频率，Hz（默认 5.0，协议限制 0x0305 最高 5Hz）",
    )
    args = parser.parse_args()

    rclpy.init()
    node = TestSerialSender(args.self_color, args.hz)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl+C，正在退出...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
