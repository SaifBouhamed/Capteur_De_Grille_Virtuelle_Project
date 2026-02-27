#!/usr/bin/env python3
import socket
import struct
import msgpack
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge


def send_msg(sock, obj):
    body = msgpack.packb(obj, use_bin_type=True)
    sock.sendall(struct.pack("!I", len(body)) + body)


def recvall(sock, n):
    data = b""
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            return None
        data += chunk
    return data


def recv_msg(sock):
    hdr = recvall(sock, 4)
    if not hdr:
        return None
    (length,) = struct.unpack("!I", hdr)
    body = recvall(sock, length)
    if body is None:
        return None
    return msgpack.unpackb(body, raw=False)


class PersonDetectorClient(Node):
    def __init__(self):
        super().__init__("person_detector_node")

        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter("image_topic", "/camera/rgb/image_raw")
        self.declare_parameter("server_host", "10.111.229.90")  # VM IP
        self.declare_parameter("server_port", 9900)
        self.declare_parameter("send_every", 5)
        self.declare_parameter("jpeg_quality", 75)
        self.declare_parameter("socket_timeout", 4.0)

        # Detection thresholds
        self.declare_parameter("min_area", 0.03)

        self.image_topic = self.get_parameter("image_topic").value
        self.server_host = self.get_parameter("server_host").value
        self.server_port = int(self.get_parameter("server_port").value)
        self.send_every = int(self.get_parameter("send_every").value)
        self.jpeg_q = int(self.get_parameter("jpeg_quality").value)
        self.sock_timeout = float(self.get_parameter("socket_timeout").value)

        self.min_area = float(self.get_parameter("min_area").value)

        # Publishers
        self.pub_human = self.create_publisher(Bool, "/human_detected", 10)
        self.pub_cx = self.create_publisher(Float32, "/person_cx", 10)
        self.pub_area = self.create_publisher(Float32, "/person_area", 10)

        self.frame_count = 0
        self.busy = False

        self.sub = self.create_subscription(Image, self.image_topic, self.cb, 10)

        self.get_logger().info(f"Subscribing: {self.image_topic}")
        self.get_logger().info(f"DeepLab server: {self.server_host}:{self.server_port}")

    def cb(self, msg: Image):
        if self.busy:
            return

        self.frame_count += 1
        if self.frame_count % self.send_every != 0:
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        self.busy = True
        try:
            ok, jpg = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_q])
            if not ok:
                return

            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(self.sock_timeout)
            s.connect((self.server_host, self.server_port))

            send_msg(s, {"img_jpg": jpg.tobytes()})
            resp = recv_msg(s)
            s.close()

            if not resp or not resp.get("ok", False):
                self.get_logger().warn(f"Bad server response: {resp}")
                return

            person = bool(resp.get("person", False))
            cx = float(resp.get("cx", 0.5))
            area = float(resp.get("area", 0.0))

            # Apply area gate here to reduce false positives
            human = person and (area >= self.min_area)

            self.pub_human.publish(Bool(data=human))
            self.pub_cx.publish(Float32(data=cx))
            self.pub_area.publish(Float32(data=area))

        except Exception as e:
            self.get_logger().warn(f"Server call failed: {e}")
            # Fail-safe: publish false if server unreachable
            self.pub_human.publish(Bool(data=False))
        finally:
            self.busy = False


def main():
    rclpy.init()
    node = PersonDetectorClient()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
