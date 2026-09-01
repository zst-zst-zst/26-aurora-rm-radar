#!/usr/bin/env python3
"""
Publish mp4 video frames as sensor_msgs/Image on 'camera_image' topic.
Usage:
  ros2 run ... / python3 tools/video_publisher.py <video.mp4> [--rate-factor 1.0]
"""
import sys
import time
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class VideoPublisher(Node):
    def __init__(self, video_path: str, rate_factor: float = 1.0, width: int = 1280, start_sec: float = 0.0):
        super().__init__('video_publisher')
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
        self.pub    = self.create_publisher(Image, 'camera_image', qos)
        self.bridge = CvBridge()
        self.resize_w = width
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            self.get_logger().error(f'Cannot open: {video_path}')
            rclpy.shutdown(); return
        fps   = cap.get(cv2.CAP_PROP_FPS) or 15.0
        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        orig_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        orig_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        pub_w  = width if width > 0 else orig_w
        pub_h  = int(orig_h * pub_w / orig_w) if width > 0 else orig_h
        self.get_logger().info(
            f'{video_path}  {total} frames @ {fps:.1f} fps  '
            f'publish={pub_w}x{pub_h}  rate_factor={rate_factor}')
        if start_sec > 0:
            cap.set(cv2.CAP_PROP_POS_MSEC, start_sec * 1000)
            self.get_logger().info(f'Seeking to {start_sec:.1f}s')

        # Shared latest frame (background reader fills it, timer drains it)
        self._latest: Image | None = None
        self._lock   = threading.Lock()
        self._done   = False
        self._fidx   = 0
        self._total  = total

        def _make_msg(frame) -> Image:
            msg = Image()
            msg.height = frame.shape[0]
            msg.width  = frame.shape[1]
            msg.encoding = 'bgr8'
            msg.is_bigendian = 0
            msg.step = frame.shape[1] * 3
            msg.data = frame.tobytes()   # expensive copy – done in bg thread
            return msg

        def _reader():
            frame_period = 1.0 / (fps * rate_factor)
            t_next = time.monotonic()
            while True:
                ret, frame = cap.read()
                if not ret:
                    with self._lock:
                        self._done = True
                    break
                if width > 0:
                    h, w = frame.shape[:2]
                    frame = cv2.resize(frame, (width, int(h * width / w)),
                                       interpolation=cv2.INTER_AREA)
                msg = _make_msg(frame)   # build while timer sleeps
                with self._lock:
                    self._latest = msg
                    self._fidx  += 1
                t_next += frame_period
                sleep = t_next - time.monotonic()
                if sleep > 0:
                    time.sleep(sleep)

        self._thread = threading.Thread(target=_reader, daemon=True)
        self._thread.start()
        self.timer = self.create_timer(1.0 / (fps * rate_factor), self.tick)

    def tick(self):
        with self._lock:
            msg  = self._latest
            self._latest = None
            done = self._done
            fidx = self._fidx
        if msg is None:
            if done:
                self.get_logger().info('Video finished.')
                self.timer.cancel(); rclpy.shutdown()
            return
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)   # tick() is now near-instant
        if fidx % 150 == 0:
            self.get_logger().info(f'  frame {fidx}/{self._total}')


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('video')
    ap.add_argument('--rate-factor', type=float, default=1.0,
                    help='Playback speed multiplier (0.5 = half speed, 2.0 = 2x)')
    ap.add_argument('--width', type=int, default=1280,
                    help='Resize width before publish (1280=YOLO native, 0=original 20MP)')
    ap.add_argument('--start', default='0',
                    help='Start time: seconds (180) or mm:ss (3:00)')
    args, ros_args = ap.parse_known_args()
    # parse --start
    s = args.start
    if ':' in s:
        m, sec = s.split(':', 1)
        start_sec = int(m) * 60 + float(sec)
    else:
        start_sec = float(s)
    rclpy.init(args=ros_args)
    node = VideoPublisher(args.video, args.rate_factor, args.width, start_sec)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
