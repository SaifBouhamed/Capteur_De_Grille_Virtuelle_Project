from typing import Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class WebcamPublisherNode(Node):
    def __init__(self):
        super().__init__('webcam_publisher_node')

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Impossible d’ouvrir la webcam')
            return

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info('WebcamPublisherNode initialisé')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Frame webcam non lue')
            return

        # frame is already in BGR 
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_pub.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = WebcamPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()