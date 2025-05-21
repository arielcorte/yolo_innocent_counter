#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from picamera2 import Picamera2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('rpicamera2_publisher')
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # Initialize Picamera2
        self.picam2 = Picamera2()
        # Configure for 640×480 preview (you can adjust resolution)
        config = self.picam2.create_preview_configuration(
            main={'size': (640, 480)})
        self.picam2.configure(config)
        self.picam2.start()

        # Timer to capture at ~30 Hz
        timer_period = 1.0 / 30.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('rpicamera2 publisher started')

    def timer_callback(self):
        # Capture a frame as a numpy array (RGB)
        frame = self.picam2.capture_array()
        # Convert to ROS Image message
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.picam2.stop()
        node.picam2.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
