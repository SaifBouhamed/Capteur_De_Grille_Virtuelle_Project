#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
import numpy as np
import time
import struct


def get_one_frame(grid_n: int, ratio_taille: float, skip: int):
    node = rclpy.create_node('grid_worker_phoenix')

    try:
        topic = '/camera/depth/color/points_decomp'

        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, node)
        pub = node.create_publisher(Marker, '/smart_grid/visual', 10)

        captured_msg = {'data': None}

        def callback(msg):
            captured_msg['data'] = msg

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        sub = node.create_subscription(PointCloud2, topic, callback, qos)

        start_time = time.time()
        success = False

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

            if time.time() - start_time > 2.0:
                break

            if captured_msg['data'] is not None:
                msg = captured_msg['data']

                # translator already publishes in base_link, keep TF check for safety
                if tf_buffer.can_transform('base_link', msg.header.frame_id, Time()):
                    trans = tf_buffer.lookup_transform('base_link', msg.header.frame_id, Time())
                    points = process_cloud(msg, skip=skip)

                    if len(points) > 0:
                        publish_marker(points, trans, pub, node, grid_n=grid_n, ratio_taille=ratio_taille)
                        success = True
                        break

    finally:
        node.destroy_node()
        return success


def process_cloud(cloud_msg: PointCloud2, skip: int = 50) -> np.ndarray:
    try:
        raw = cloud_msg.data
        step = cloud_msg.point_step

        off_x = 0
        off_y = 4
        off_z = 8
        for f in cloud_msg.fields:
            if f.name == 'x':
                off_x = f.offset
            if f.name == 'y':
                off_y = f.offset
            if f.name == 'z':
                off_z = f.offset

        points = []
        limit = len(raw)
        stride = step * max(1, int(skip))

        unpack = struct.unpack_from

        for i in range(0, limit, stride):
            if i + off_z + 4 > limit:
                break
            x, y, z = unpack('fff', raw, i + off_x)
            if x == x and y == y and z == z:  # NaN check
                points.append([x, y, z])

        return np.array(points, dtype=np.float32)
    except Exception:
        return np.array([], dtype=np.float32)


def publish_marker(xyz_points: np.ndarray, tf, pub, node, grid_n: int = 40, ratio_taille: float = 0.3):
    # Transform
    rx = tf.transform.rotation.x
    ry = tf.transform.rotation.y
    rz = tf.transform.rotation.z
    rw = tf.transform.rotation.w
    tx = tf.transform.translation.x
    ty = tf.transform.translation.y
    tz = tf.transform.translation.z

    R = np.array([
        [1 - 2*ry*ry - 2*rz*rz, 2*rx*ry - 2*rz*rw,     2*rx*rz + 2*ry*rw],
        [2*rx*ry + 2*rz*rw,     1 - 2*rx*rx - 2*rz*rz, 2*ry*rz - 2*rx*rw],
        [2*rx*rz - 2*ry*rw,     2*ry*rz + 2*rx*rw,     1 - 2*rx*rx - 2*ry*ry]
    ], dtype=np.float32)

    xyz_robot = (xyz_points @ R.T) + np.array([tx, ty, tz], dtype=np.float32)

    # Grid (denser if grid_n is bigger)
    grid_size = 1.0   # 1 meter total length
    cell = grid_size / float(grid_n)
    
    base_h = 0.00
    h_map = {}

    # Filter zone 2m x 2m in front
    mask = (xyz_robot[:, 0] > 0) & (xyz_robot[:, 0] < 1.0) & (xyz_robot[:, 1] > -0.5) & (xyz_robot[:, 1] < 0.5)
    xyz_crop = xyz_robot[mask]
    if len(xyz_crop) == 0:
        return

    idx_x = (xyz_crop[:, 0] / cell).astype(np.int32)
    idx_y = ((xyz_crop[:, 1] + 0.5) / cell).astype(np.int32)

    for k in range(len(xyz_crop)):
        ix = int(idx_x[k])
        iy = int(idx_y[k])
        if 0 <= ix < grid_n and 0 <= iy < grid_n:
            z = float(xyz_crop[k, 2])
            if (ix, iy) not in h_map or z > h_map[(ix, iy)]:
                h_map[(ix, iy)] = z

    # Marker
    m = Marker()
    m.header.frame_id = 'base_link'
    m.header.stamp = node.get_clock().now().to_msg()
    m.ns = "pin_art"
    m.id = 0

    m.type = Marker.SPHERE_LIST
    m.action = Marker.ADD
    m.lifetime.sec = 0

    # Smaller spheres
    ratio_taille = float(np.clip(ratio_taille, 0.05, 1.0))
    m.scale.x = cell * ratio_taille
    m.scale.y = cell * ratio_taille
    m.scale.z = cell * ratio_taille
    m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

    pts = []
    for ix in range(grid_n):
        for iy in range(grid_n):
            z = base_h
            if (ix, iy) in h_map:
                z = max(base_h, h_map[(ix, iy)])

            cx = ix * cell + cell / 2.0
            cy = -0.5 + iy * cell + cell / 2.0
            pts.append(Point(x=float(cx), y=float(cy), z=float(z)))

    m.points = pts
    pub.publish(m)


def main(args=None):
    rclpy.init(args=args)

    # You can change these three values easily here:
    grid_n = 40        # higher => denser points (less spacing)
    ratio_taille = 0.65  # lower => smaller spheres
    skip = 0            # lower => use more points from the cloud

    print(f"--- PHOENIX NODE DÉMARRÉ (grid_n={grid_n}, ratio={ratio_taille}, skip={skip}) ---")

    frame_id = 0
    while rclpy.ok():
        success = get_one_frame(grid_n=grid_n, ratio_taille=ratio_taille, skip=skip)

        if success:
            frame_id += 1
            print(f"\rFrame #{frame_id} générée", end="")
        else:
            print(".", end="", flush=True)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
