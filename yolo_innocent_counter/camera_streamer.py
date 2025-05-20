#!/usr/bin/env python3
"""
camera_streamer.py

A ROS2 node that captures frames from a webcam (e.g. /dev/video0)
and publishes them as sensor_msgs/Image on /camera/image_raw.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraStreamer(Node):
    def __init__(self):
        super().__init__('camera_streamer')
        # Parameter: video device or file
        self.declare_parameter('video_device', '/dev/video0')
        dev = self.get_parameter('video_device')\
                  .get_parameter_value().string_value

        # Parameter: publish rate (Hz)
        self.declare_parameter('publish_rate', 30.0)
        self.rate_hz = self.get_parameter('publish_rate')\
                          .get_parameter_value().double_value

        # OpenCV capture
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open video device: {dev}')
            raise RuntimeError(f'Cannot open {dev}')

        # Publisher
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.br = CvBridge()

        # Timer
        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f'Streaming {dev} at {self.rate_hz} Hz → /camera/image_raw')

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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
