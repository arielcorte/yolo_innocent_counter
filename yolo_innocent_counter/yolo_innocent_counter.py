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
        self.person_pub = self.create_publisher(Int32, 'person', 10)
        self.teddy_pub = self.create_publisher(Int32, 'teddy_bear', 10)

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

        boxes = results[0].boxes

        # initialize counts
        person_count = 0
        teddy_count = 0

        if boxes:
            # boxes.cls is a tensor of class‐ids; convert to numpy ints
            cls_ids = boxes.cls
            person_count = int((cls_ids == 0).sum())
            teddy_count = int((cls_ids == 77).sum())

        # publish person count
        person_msg = Int32()
        person_msg.data = person_count
        self.person_pub.publish(person_msg)
        self.get_logger().info(f'Detected {person_count} person(s)')

        # publish teddy‐bear count
        teddy_msg = Int32()
        teddy_msg.data = teddy_count
        self.teddy_pub.publish(teddy_msg)
        self.get_logger().info(f'Detected {teddy_count} teddy bear(s)')


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

