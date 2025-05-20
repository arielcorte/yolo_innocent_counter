#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

class YoloInnocentNode(Node):
    def __init__(self):
        super().__init__('yolo_innocent_counter')
        self.declare_parameter('image_topic', '/camera/image_raw')
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        self.sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)
        self.pub = self.create_publisher(Int32, 'innocent', 10)

        self.br = CvBridge()
        self.model = YOLO('yolov8s.pt')
        self.get_logger().info(f'Listening on "{image_topic}"')

    def image_callback(self, msg: Image):
        try:
            frame = self.br.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CVBridge: {e}')
            return

        results = self.model.predict(frame, classes=[0, 77], verbose=False)
        if results[0].boxes:
            count = len(results[0].boxes)
            out = Int32()
            out.data = count
            self.pub.publish(out)
            self.get_logger().info(f'Detected {count} innocent friend(s)')
        else:
            out = Int32()
            out.data = 0
            self.pub.publish(out)
            self.get_logger().info(f'No innocents detected')


def main(args=None):
    rclpy.init(args=args)
    node = YoloInnocentNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

