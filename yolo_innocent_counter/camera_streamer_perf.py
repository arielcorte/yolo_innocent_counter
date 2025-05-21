#!/usr/bin/env python3
"""
camera_streamer.py

A ROS2 node that captures frames from a webcam (e.g. /dev/video0)
and publishes them as sensor_msgs/Image on /camera/image_raw.

Performance tweaks:
  - 640×480 @ MJPG @ locked FPS
  - MultiThreadedExecutor
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraStreamer(Node):
    def __init__(self):
        super().__init__('camera_streamer')

        # --- parameters ---
        self.declare_parameter('video_device', '/dev/video0')
        dev = self.get_parameter('video_device')\
                  .get_parameter_value().string_value

        self.declare_parameter('publish_rate', 30.0)
        self.rate_hz = self.get_parameter('publish_rate')\
                          .get_parameter_value().double_value

        # --- open camera via V4L2 ---
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open video device: {dev}')
            raise RuntimeError(f'Cannot open {dev}')

        # ↓ force 640×480 to cut down pixels
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # ↓ ask for MJPG so the camera/USB does most compression
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        # ↓ lock the camera to our desired FPS
        self.cap.set(cv2.CAP_PROP_FPS, self.rate_hz)

        # --- publisher & bridge ---
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.br = CvBridge()

        # --- timer for frame grabbing ---
        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f'Streaming {dev} at {self.rate_hz:.1f} Hz '
            '(640×480, MJPG) → /camera/image_raw'
        )

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Frame grab failed, retrying...')
            return

        # Convert BGR→ROS Image and publish
        msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamer()

    # spin with a MultiThreadedExecutor to avoid any back-pressure
    exec = MultiThreadedExecutor()
    exec.add_node(node)
    try:
        exec.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
