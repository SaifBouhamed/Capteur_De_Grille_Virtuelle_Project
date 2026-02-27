#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, TransformStamped
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from rclpy.time import Time
import numpy as np

class SmartGridNode(Node):
    """
    MASTER NODE : Gère la TF (Position) + Le Visuel (Morphing) + La Correction de Hauteur
    """
    def __init__(self):
        super().__init__('smart_grid_node')

        # --- PARAMÈTRES ---
        self.declare_parameter('grid_size_m', 2.0)
        self.declare_parameter('grid_resolution_n', 30)
        self.declare_parameter('base_height', 0.05) # Hauteur de base (5cm) pour ne pas toucher le sol
        
        self.grid_size_m = float(self.get_parameter('grid_size_m').value)
        self.grid_n = int(self.get_parameter('grid_resolution_n').value)
        self.base_h = float(self.get_parameter('base_height').value)
        self.cell_size = self.grid_size_m / self.grid_n

        # --- OUTILS ROS ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publisher pour les points rouges
        self.marker_publisher = self.create_publisher(Marker, '/smart_grid/visual', 10)

        # Abonnement Caméra (Mode Rapide)
        qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(
            PointCloud2, '/depth_camera/points', self.point_cloud_callback, qos_policy)

        # Timer pour la TF (La grille suit le robot)
        self.timer = self.create_timer(0.02, self.publish_grid_tf) # 50Hz

        self.get_logger().info(f'--- GRILLE INTELLIGENTE ACTIVÉE (Base: {self.base_h}m) ---')

    def publish_grid_tf(self):
        # On attache la grille 1 mètre devant le robot
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'        # Parent : Le Robot
        t.child_frame_id = 'grid_frame'        # Enfant : La Grille
        
        t.transform.translation.x = 1.0        # 1m devant
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)

        # Si on ne reçoit pas de nuage (ex: début), on affiche quand même la grille plate
        # C'est géré par le callback, mais on pourrait forcer ici si besoin.

    def point_cloud_callback(self, msg):
        try:
            # 1. Vérif TF
            if not self.tf_buffer.can_transform('grid_frame', 'base_link', Time()): return
            t_cam_base = self.tf_buffer.lookup_transform('base_link', msg.header.frame_id, Time())
            t_base_grid = self.tf_buffer.lookup_transform('grid_frame', 'base_link', Time())

            # 2. Lecture Données
            xyz_raw = self.read_xyz_manual(msg, skip=20)
            
            height_map = {}

            # 3. Calcul (si données dispo)
            if xyz_raw.shape[0] > 0:
                xyz_base = self.apply_transform(xyz_raw, t_cam_base.transform)
                xyz_grid = self.apply_transform(xyz_base, t_base_grid.transform)

                half = self.grid_size_m / 2.0
                # Filtre large
                mask = (xyz_grid[:, 0] > -half) & (xyz_grid[:, 0] < half) & \
                       (xyz_grid[:, 1] > -half) & (xyz_grid[:, 1] < half) & \
                       (xyz_grid[:, 2] > 0.02) & (xyz_grid[:, 2] < 2.0)
                xyz_cropped = xyz_grid[mask]

                if xyz_cropped.shape[0] > 0:
                    idx_x = ((xyz_cropped[:, 0] + half) / self.cell_size).astype(int)
                    idx_y = ((xyz_cropped[:, 1] + half) / self.cell_size).astype(int)
                    
                    for k in range(len(xyz_cropped)):
                        ix, iy = idx_x[k], idx_y[k]
                        z = xyz_cropped[k, 2]
                        if (ix, iy) not in height_map or z > height_map[(ix, iy)]:
                            height_map[(ix, iy)] = z

            # 4. Dessin FINAL
            self.publish_smart_grid(height_map)

        except Exception:
            pass

    def publish_smart_grid(self, height_map):
        marker = Marker()
        marker.header.frame_id = 'grid_frame'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "smart_grid"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.lifetime.sec = 0; marker.lifetime.nanosec = 0

        # Taille des points
        marker.scale.x = self.cell_size * 0.85
        marker.scale.y = self.cell_size * 0.85
        marker.scale.z = self.cell_size * 0.85

        half = self.grid_size_m / 2.0
        step = self.cell_size

        # C'est ici que le Morphing opère
        for ix in range(self.grid_n):
            for iy in range(self.grid_n):
                cx = -half + ix * step + step/2
                cy = -half + iy * step + step/2
                
                # PAR DÉFAUT : On met le point à 5cm du sol (base_h)
                z = self.base_h
                
                # SI OBSTACLE : Le point monte à la hauteur de l'obstacle
                if (ix, iy) in height_map:
                    z = max(self.base_h, height_map[(ix, iy)]) # On prend le max pour pas descendre sous 5cm
                
                marker.points.append(Point(x=cx, y=cy, z=z))
        
        # Couleur Rouge Vif
        c = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
        marker.color = c

        self.marker_publisher.publish(marker)

    def read_xyz_manual(self, cloud_msg, skip=20):
        # Lecture binaire standard (inchangée)
        try:
            raw_data = cloud_msg.data; point_step = cloud_msg.point_step
            offset_x = 0; offset_y = 4; offset_z = 8
            for f in cloud_msg.fields:
                if f.name=='x': offset_x=f.offset
                if f.name=='y': offset_y=f.offset
                if f.name=='z': offset_z=f.offset
            points = []; limit = len(raw_data); stride = point_step * skip
            import struct; unpack = struct.unpack_from
            for i in range(0, limit, stride):
                if i + 12 > limit: break
                x, y, z = unpack('fff', raw_data, i + offset_x)
                if x == x and y == y and z == z: points.append([x, y, z])
            return np.array(points)
        except: return np.array([])

    def apply_transform(self, xyz, transform):
        # Maths TF (inchangées)
        rx = transform.rotation.x; ry = transform.rotation.y
        rz = transform.rotation.z; rw = transform.rotation.w
        tx = transform.translation.x; ty = transform.translation.y; tz = transform.translation.z
        R = np.array([
            [1 - 2*ry*ry - 2*rz*rz, 2*rx*ry - 2*rz*rw,     2*rx*rz + 2*ry*rw],
            [2*rx*ry + 2*rz*rw,     1 - 2*rx*rx - 2*rz*rz, 2*ry*rz - 2*rx*rw],
            [2*rx*rz - 2*ry*rw,     2*ry*rz + 2*rx*rw,     1 - 2*rx*rx - 2*ry*ry]
        ])
        return np.dot(xyz, R.T) + np.array([tx, ty, tz])

def main(args=None):
    rclpy.init(args=args)
    node = SmartGridNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()