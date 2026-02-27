import struct
import zlib
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from rclpy.time import Time

from std_msgs.msg import ByteMultiArray
from sensor_msgs.msg import Image, PointCloud2, PointField


def _make_point_fields_xyzrgb():
    return [
        PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]


def _pack_rgb_to_float32(r, g, b):
    rgb_uint32 = (r.astype(np.uint32) << 16) | (g.astype(np.uint32) << 8) | b.astype(np.uint32)
    return rgb_uint32.view(np.float32)


class RCMBReceiver(Node):
    def __init__(self):
        super().__init__('rcmb_receiver')

        # Topics
        self.in_topic = self.declare_parameter('in_topic', '/camera/combined').value
        self.out_image = self.declare_parameter('out_image', '/camera/color/image_raw').value
        self.out_cloud = self.declare_parameter('out_cloud', '/camera/depth/color/points').value

        # If true, use (u,v) to sample RGB from image and publish XYZRGB pointcloud
        self.colorize = bool(self.declare_parameter('colorize', True).value)

        # Frames for RViz
        self.color_frame = self.declare_parameter('color_frame', 'base_link').value
        self.cloud_frame = self.declare_parameter('cloud_frame', 'base_link').value

        # Throttle output (Hz). 0 = no throttle.
        self.max_hz = float(self.declare_parameter('max_hz', 10.0).value)
        self._last_pub_ns = 0

        # IMPORTANT: use the timestamp from the packet (prevents RViz TF queue overflow)
        self.use_packet_stamp = bool(self.declare_parameter('use_packet_stamp', True).value)
        
        self.level_cloud = bool(self.declare_parameter('level_cloud', True).value)
        # Extra pitch correction in degrees (if camera is mounted angled)
        self.extra_pitch_deg = float(self.declare_parameter('extra_pitch_deg', 0.0).value)
        self.z_offset = float(self.declare_parameter('z_offset', 0.0).value)
        
        # Subscriber QoS (match robot publisher): BestEffort sensor-style
        self.sub_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # Publisher QoS: RELIABLE so RViz connects, depth=1 to avoid lag
        self.pub_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # Subscribe
        self.sub = self.create_subscription(
            ByteMultiArray,
            self.in_topic,
            self.cb,
            self.sub_qos
        )

        # Publish
        self.pub_img = self.create_publisher(Image, self.out_image, self.pub_qos)
        self.pub_pc = self.create_publisher(PointCloud2, self.out_cloud, self.pub_qos)

        self.get_logger().info(f"Listening: {self.in_topic}")
        self.get_logger().info(f"Publishing image: {self.out_image}")
        self.get_logger().info(f"Publishing cloud: {self.out_cloud} (colorize={self.colorize})")
        self.get_logger().info(
            f"QoS sub=BEST_EFFORT depth={self.sub_qos.depth} | pub=RELIABLE depth={self.pub_qos.depth} | "
            f"max_hz={self.max_hz} | use_packet_stamp={self.use_packet_stamp}"
        )
    def _rotate_pitch(self, xyz: np.ndarray, pitch_deg: float) -> np.ndarray:
        if pitch_deg == 0.0:
            return xyz
        a = np.deg2rad(pitch_deg)
        ca, sa = np.cos(a), np.sin(a)
        R = np.array([[ ca, 0.0,  sa],
                      [0.0, 1.0, 0.0],
                      [-sa, 0.0,  ca]], dtype=np.float32)
        return (xyz @ R.T).astype(np.float32)
    
    def _normalize_bytes(self, d) -> bytes:
        if isinstance(d, (bytes, bytearray)):
            return bytes(d)
        try:
            return bytes(d)
        except TypeError:
            return b''.join(d)

    def _throttle_ok(self) -> bool:
        if self.max_hz <= 0.0:
            return True
        now_ns = self.get_clock().now().nanoseconds
        period_ns = int(1e9 / self.max_hz)
        if (now_ns - self._last_pub_ns) < period_ns:
            return False
        self._last_pub_ns = now_ns
        return True

    def cb(self, msg: ByteMultiArray):
        if not self._throttle_ok():
            return

        try:
            data = self._normalize_bytes(msg.data)
        except Exception as e:
            self.get_logger().warning(f"Failed to normalize payload: {e}")
            return

        # Minimum packet size check
        min_sz = 4 + 4 + 8 + 4 + 4 + 12 + 4 + 4
        if len(data) < min_sz:
            self.get_logger().warning("RCMB packet too small")
            return

        # Header parse
        off = 0
        magic = data[off:off + 4]
        off += 4
        if magic != b'RCMB':
            self.get_logger().warning(f"Bad magic: {magic}")
            return

        ver, = struct.unpack_from('<I', data, off)
        off += 4
        if ver != 2:
            self.get_logger().warning(f"Unsupported version: {ver}")
            return

        stamp_ns, = struct.unpack_from('<Q', data, off)
        off += 8

        _width, _height = struct.unpack_from('<II', data, off)
        off += 8

        # yaw,pitch,roll (unused)
        _yaw, _pitch, _roll = struct.unpack_from('<fff', data, off)
        off += 12

        jpeg_sz, pc_sz = struct.unpack_from('<II', data, off)
        off += 8

        if len(data) < off + jpeg_sz + pc_sz:
            self.get_logger().warning("Packet truncated")
            return

        jpeg_bytes = data[off:off + jpeg_sz]
        off += jpeg_sz
        pc_z = data[off:off + pc_sz]

        if jpeg_sz == 0:
            self.get_logger().warning("No JPEG in packet")
            return

        # Decode JPEG -> BGR image
        jpg = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        bgr = cv2.imdecode(jpg, cv2.IMREAD_COLOR)
        if bgr is None:
            self.get_logger().warning("JPEG decode failed")
            return

        # Use packet timestamp to match robot TF timeline (prevents RViz filter queue overflow)
        if self.use_packet_stamp and stamp_ns > 0:
            stamp_msg = Time(nanoseconds=int(stamp_ns)).to_msg()
        else:
            stamp_msg = self.get_clock().now().to_msg()

        # Publish Image (bgr8)
        img_msg = Image()
        img_msg.header.stamp = stamp_msg
        img_msg.header.frame_id = self.color_frame
        img_msg.height, img_msg.width = bgr.shape[:2]
        img_msg.encoding = 'bgr8'
        img_msg.is_bigendian = 0
        img_msg.step = img_msg.width * 3
        img_msg.data = bgr.tobytes()
        self.pub_img.publish(img_msg)

        # Decompress point payload
        if pc_sz == 0:
            return

        try:
            pc_raw = zlib.decompress(pc_z)
        except Exception as e:
            self.get_logger().warning(f"zlib decompress failed: {e}")
            return

        if len(pc_raw) < 4:
            self.get_logger().warning("pc_raw too small")
            return

        count, = struct.unpack_from('<I', pc_raw, 0)
        expected = 4 + count * 5 * 4
        if len(pc_raw) < expected:
            self.get_logger().warning(f"pc_raw truncated: got={len(pc_raw)} expected>={expected}")
            return



        pts = np.frombuffer(pc_raw, dtype=np.float32, offset=4, count=count * 5).reshape((-1, 5))
        xyz = pts[:, 0:3] 
        uv = pts[:, 3:5]

        if self.level_cloud:
            # Optional extra pitch correction for camera mounting angle
            xyz = self._rotate_pitch(xyz, self.extra_pitch_deg)
            
        xyz[:, 2] += self.z_offset

        pc_msg = PointCloud2()
        pc_msg.header.stamp = stamp_msg
        pc_msg.header.frame_id = self.cloud_frame
        pc_msg.height = 1
        pc_msg.width = int(xyz.shape[0])
        pc_msg.is_bigendian = False
        pc_msg.is_dense = True

        if self.colorize:
            h, w = bgr.shape[:2]
            u = np.clip(uv[:, 0], 0.0, 1.0)
            v = np.clip(uv[:, 1], 0.0, 1.0)
            px = (u * (w - 1)).astype(np.int32)
            py = (v * (h - 1)).astype(np.int32)

            b = bgr[py, px, 0].astype(np.uint8)
            g = bgr[py, px, 1].astype(np.uint8)
            r = bgr[py, px, 2].astype(np.uint8)

            rgb_f = _pack_rgb_to_float32(r, g, b)

            cloud = np.zeros(
                (xyz.shape[0],),
                dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.float32)]
            )
            cloud['x'] = xyz[:, 0]
            cloud['y'] = xyz[:, 1]
            cloud['z'] = xyz[:, 2]
            cloud['rgb'] = rgb_f

            pc_msg.fields = _make_point_fields_xyzrgb()
            pc_msg.point_step = 16
            pc_msg.row_step = pc_msg.point_step * pc_msg.width
            pc_msg.data = cloud.tobytes()
        else:
            pc_msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            pc_msg.point_step = 12
            pc_msg.row_step = pc_msg.point_step * pc_msg.width
            pc_msg.data = xyz.astype(np.float32).tobytes()

        self.pub_pc.publish(pc_msg)


def main():
    rclpy.init()
    node = RCMBReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
