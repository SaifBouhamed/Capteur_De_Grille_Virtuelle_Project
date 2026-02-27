#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
import numpy as np
import math

class GridProjectionNode(Node):
    def __init__(self):
        super().__init__('grid_projection_node')

        # Paramètres
        self.declare_parameter('grid_size_m', 2.0)
        self.declare_parameter('grid_resolution_n', 50)
        self.declare_parameter('cloud_topic', '/depth_camera/points')
        
        self.grid_size_m = float(self.get_parameter('grid_size_m').value)
        self.grid_resolution_n = int(self.get_parameter('grid_resolution_n').value)
        self.cell_size = self.grid_size_m / self.grid_resolution_n
        self.cloud_topic = self.get_parameter('cloud_topic').value

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS Robustesse
        qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            history=HistoryPolicy.KEEP_LAST, 
            depth=1
        )
        
        self.subscription = self.create_subscription(
            PointCloud2, 
            self.cloud_topic, 
            self.point_cloud_callback, 
            qos_policy
        )
        
        self.grid_publisher = self.create_publisher(OccupancyGrid, '/virtual_sensor/heightmap', 10)

        self.get_logger().info('--- GRID PROJECTION: FULL MANUAL MODE ---')

    def point_cloud_callback(self, msg):
        try:
            # 1. Vérifier si la transformation est dispo
            if not self.tf_buffer.can_transform('grid_frame', msg.header.frame_id, rclpy.time.Time()):
                return

            # Récupérer la transfo (Position + Rotation)
            trans_stamped = self.tf_buffer.lookup_transform('grid_frame', msg.header.frame_id, rclpy.time.Time())
            tx = trans_stamped.transform.translation.x
            ty = trans_stamped.transform.translation.y
            tz = trans_stamped.transform.translation.z
            rx = trans_stamped.transform.rotation.x
            ry = trans_stamped.transform.rotation.y
            rz = trans_stamped.transform.rotation.z
            rw = trans_stamped.transform.rotation.w

            # 2. Lire les points BRUTS (Sans passer par les outils ROS qui plantent)
            xyz = self.read_xyz_manual(msg)
            if xyz.shape[0] == 0: return

            # 3. Appliquer la transformation MANUELLEMENT (Maths pures)
            # Rotation (Quaternion -> Matrice)
            # Formule standard de rotation 3D
            R = np.array([
                [1 - 2*ry*ry - 2*rz*rz, 2*rx*ry - 2*rz*rw,     2*rx*rz + 2*ry*rw],
                [2*rx*ry + 2*rz*rw,     1 - 2*rx*rx - 2*rz*rz, 2*ry*rz - 2*rx*rw],
                [2*rx*rz - 2*ry*rw,     2*ry*rz + 2*rx*rw,     1 - 2*rx*rx - 2*ry*ry]
            ])
            
            # Application : P_new = R * P_old + Translation
            # xyz est de forme (N, 3), on transpose pour multiplier
            xyz_trans = np.dot(xyz, R.T) 
            xyz_trans[:, 0] += tx
            xyz_trans[:, 1] += ty
            xyz_trans[:, 2] += tz

            # 4. Filtrage (Zone Grille)
            half = self.grid_size_m / 2.0
            mask = (xyz_trans[:, 0] > -half) & (xyz_trans[:, 0] < half) & \
                   (xyz_trans[:, 1] > -half) & (xyz_trans[:, 1] < half) & \
                   (xyz_trans[:, 2] > -1.0) & (xyz_trans[:, 2] < 3.0)
            
            xyz_cropped = xyz_trans[mask]
            if xyz_cropped.shape[0] == 0: return

            # 5. Projection Grille
            idx_x = ((xyz_cropped[:, 0] + half) / self.cell_size).astype(int)
            idx_y = ((xyz_cropped[:, 1] + half) / self.cell_size).astype(int)
            
            idx_x = np.clip(idx_x, 0, self.grid_resolution_n - 1)
            idx_y = np.clip(idx_y, 0, self.grid_resolution_n - 1)

            N = self.grid_resolution_n
            heightmap = np.full((N, N), -100.0, dtype=np.float32)
            np.maximum.at(heightmap, (idx_x, idx_y), xyz_cropped[:, 2])

            # 6. Publication
            self.publish_occupancy_grid(heightmap)

        except Exception as e:
            self.get_logger().error(f"Manual Math Error: {e}")

    def read_xyz_manual(self, cloud_msg):
        """ Extraction binaire directe des points X, Y, Z """
        # Trouver les offsets
        x_off = -1; y_off = -1; z_off = -1
        for f in cloud_msg.fields:
            if f.name == 'x': x_off = f.offset
            if f.name == 'y': y_off = f.offset
            if f.name == 'z': z_off = f.offset

        if x_off < 0 or y_off < 0 or z_off < 0:
            return np.array([])

        # Lecture brutale du buffer
        raw_data = np.frombuffer(cloud_msg.data, dtype=np.uint8)
        point_step = cloud_msg.point_step
        num_points = cloud_msg.width * cloud_msg.height
        
        if len(raw_data) != num_points * point_step:
            return np.array([]) # Buffer incomplet

        reshaped = raw_data.reshape(num_points, point_step)

        # On suppose du float32 (cas standard ROS/Gazebo)
        x = reshaped[:, x_off:x_off+4].view(np.float32).flatten()
        y = reshaped[:, y_off:y_off+4].view(np.float32).flatten()
        z = reshaped[:, z_off:z_off+4].view(np.float32).flatten()
        
        # Assemblage et nettoyage des NaN
        xyz = np.column_stack((x, y, z))
        return xyz[~np.isnan(xyz).any(axis=1)]

    def publish_occupancy_grid(self, heightmap):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'grid_frame' 

        msg.info.resolution = self.cell_size
        msg.info.width = self.grid_resolution_n
        msg.info.height = self.grid_resolution_n
        msg.info.origin.position.x = -self.grid_size_m / 2.0
        msg.info.origin.position.y = -self.grid_size_m / 2.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        grid_data = np.full_like(heightmap, -1, dtype=np.int8)
        valid_mask = (heightmap > -99.0)
        
        if np.any(valid_mask):
            h_valid = heightmap[valid_mask]
            z_min, z_max = np.min(h_valid), np.max(h_valid)
            if z_max > z_min:
                norm = (h_valid - z_min) / (z_max - z_min) * 100
                grid_data[valid_mask] = norm.astype(np.int8)
            else:
                grid_data[valid_mask] = 100 

        msg.data = grid_data.flatten().tolist()
        self.grid_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GridProjectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()