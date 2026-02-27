import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
import pyrealsense2 as rs
from cv_bridge import CvBridge


class RealSenseRGBPublisher(Node):
    def __init__(self):
        super().__init__('realsense_rgb_publisher')

        self.publisher_ = self.create_publisher(Image, 'camera/color/image_raw', 10)
        self.bridge = CvBridge()

        # RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)

        # 30 FPS = 0.033s
        self.timer = self.create_timer(0.033, self.timer_callback)

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        img = np.asanyarray(color_frame.get_data())
        ros_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.publisher_.publish(ros_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseRGBPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
