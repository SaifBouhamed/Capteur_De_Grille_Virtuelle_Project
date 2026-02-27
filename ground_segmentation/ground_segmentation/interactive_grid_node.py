#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interactive_markers import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import Point, Pose, TransformStamped
from tf2_ros import TransformBroadcaster

class InteractiveGridNode(Node):
    """
    Contrôleur : Affiche une grille de points ROUGES et gère le déplacement.
    """
    def __init__(self):
        super().__init__('interactive_grid_node')

        # Paramètres
        self.declare_parameter('grid_size_m', 2.0)
        self.declare_parameter('grid_resolution_n', 30) # 30x30 points
        self.declare_parameter('frame_id', 'base_link')

        self.grid_size_m = float(self.get_parameter('grid_size_m').value)
        self.grid_n = int(self.get_parameter('grid_resolution_n').value)
        self.frame_id = self.get_parameter('frame_id').value

        # Position initiale
        self.grid_pose = Pose()
        self.grid_pose.position.x = 1.0 
        self.grid_pose.position.y = 0.0
        self.grid_pose.position.z = 0.0
        self.grid_pose.orientation.w = 1.0

        # Outils ROS
        self.server = InteractiveMarkerServer(self, 'grid_control')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.publish_grid_tf)

        self.make_interactive_marker()
        self.server.applyChanges()
        
        self.get_logger().info('Contrôleur Grille : Prêt (Grille ROUGE).')

    def make_interactive_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.frame_id
        int_marker.name = "grid_sensor_3d"
        int_marker.description = "Zone de Detection"
        int_marker.scale = 1.0
        int_marker.pose = self.grid_pose

        # 1. Visuel : La grille de points ROUGE
        points_marker = self.create_base_points_marker()
        control_visual = InteractiveMarkerControl()
        control_visual.always_visible = True
        control_visual.markers.append(points_marker)
        int_marker.controls.append(control_visual)

        # 2. Contrôles de mouvement
        self.add_controls(int_marker)

        self.server.insert(int_marker, feedback_callback=self.process_feedback)

    def create_base_points_marker(self):
        # Crée une grille de points plats
        marker = Marker()
        marker.pose.orientation.w = 1.0 
        marker.type = Marker.SPHERE_LIST
        
        # Taille des points (ajuste selon tes goûts)
        marker.scale.x = 0.03 
        marker.scale.y = 0.03
        marker.scale.z = 0.01
        
        # --- C'EST ICI QU'ON CHANGE LA COULEUR ---
        # ROUGE (Red=1.0, Green=0, Blue=0)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        # Transparence (0.6 = assez visible mais on voit un peu le sol à travers)
        marker.color.a = 0.6 

        half = self.grid_size_m / 2.0
        step = self.grid_size_m / self.grid_n

        # Génère les points au sol (z=0)
        for i in range(self.grid_n):
            x = -half + i * step + step/2.0
            for j in range(self.grid_n):
                y = -half + j * step + step/2.0
                marker.points.append(Point(x=x, y=y, z=0.0))
        return marker

    def add_controls(self, int_marker):
        # Flèche Rouge (X)
        c_x = InteractiveMarkerControl()
        c_x.name = "move_x"
        c_x.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        c_x.orientation.w = 1.0; c_x.orientation.x = 1.0; c_x.orientation.y = 0.0; c_x.orientation.z = 0.0
        int_marker.controls.append(c_x)

        # Flèche Verte (Y)
        c_y = InteractiveMarkerControl()
        c_y.name = "move_y"
        c_y.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        c_y.orientation.w = 1.0; c_y.orientation.x = 0.0; c_y.orientation.y = 0.0; c_y.orientation.z = 1.0
        int_marker.controls.append(c_y)

        # Flèche Bleue (Z)
        c_z = InteractiveMarkerControl()
        c_z.name = "move_z"
        c_z.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        c_z.orientation.w = 1.0; c_z.orientation.x = 0.0; c_z.orientation.y = 1.0; c_z.orientation.z = 0.0
        int_marker.controls.append(c_z)
        
        # Rotation (Autour de Z)
        c_rot = InteractiveMarkerControl()
        c_rot.name = "rotate_z"
        c_rot.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        c_rot.orientation.w = 1.0; c_rot.orientation.x = 0.0; c_rot.orientation.y = 1.0; c_rot.orientation.z = 0.0
        int_marker.controls.append(c_rot)

    def process_feedback(self, feedback):
        self.grid_pose = feedback.pose
        self.server.applyChanges()

    def publish_grid_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = 'grid_frame'
        t.transform.translation.x = self.grid_pose.position.x
        t.transform.translation.y = self.grid_pose.position.y
        t.transform.translation.z = self.grid_pose.position.z
        t.transform.rotation = self.grid_pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = InteractiveGridNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()