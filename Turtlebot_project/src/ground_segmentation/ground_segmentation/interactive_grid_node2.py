#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interactive_markers import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import Point, Pose, TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np


class InteractiveGridNode(Node):
    """
    Nœud qui crée une grille 3D interactive N×N dans RViz2.
    """

    def __init__(self):
        super().__init__('interactive_grid_node')

        self.declare_parameter('grid_size_m', 2.0)
        self.declare_parameter('grid_resolution_n', 20)
        self.declare_parameter('grid_height', 0.0)
        self.declare_parameter('frame_id', 'odom')

        self.grid_size_m = float(self.get_parameter('grid_size_m').value)
        self.grid_n = int(self.get_parameter('grid_resolution_n').value)
        self.grid_height = float(self.get_parameter('grid_height').value)
        self.frame_id = self.get_parameter('frame_id').value


        self.grid_pose = Pose()
        self.grid_pose.position.x = 0.0
        self.grid_pose.position.y = 0.0
        self.grid_pose.position.z = self.grid_height
        self.grid_pose.orientation.w = 1.0


        self.server = InteractiveMarkerServer(self, 'grid_control')


        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_grid_tf)


        self.make_interactive_grid()
        self.server.applyChanges()

        self.get_logger().info(f'Interactive Grid Node initialized')
        self.get_logger().info(f'  grid_size_m={self.grid_size_m}')
        self.get_logger().info(f'  grid_resolution_n={self.grid_n}')
        self.get_logger().info(f'  frame_id={self.frame_id}')



    def make_interactive_grid(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.frame_id
        int_marker.name = "grid_sensor"
        int_marker.description = f"Virtual Grid {self.grid_n}x{self.grid_n}"
        int_marker.pose = self.grid_pose
        int_marker.scale = self.grid_size_m


        grid_marker = self.create_grid_marker()
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(grid_marker)
        int_marker.controls.append(control)


        self.add_6dof_controls(int_marker)

   
        self.server.insert(int_marker, feedback_callback=self.process_feedback)

    def create_grid_marker(self):
        marker = Marker()
        marker.type = Marker.LINE_LIST
        marker.scale.x = 0.01  

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.7

        half = self.grid_size_m / 2.0
        cell = self.grid_size_m / self.grid_n

        
        for i in range(self.grid_n + 1):
            x = -half + i * cell
            p1 = Point(x=x, y=-half, z=0.0)
            p2 = Point(x=x, y=half, z=0.0)
            marker.points.append(p1)
            marker.points.append(p2)

        
        for j in range(self.grid_n + 1):
            y = -half + j * cell
            p1 = Point(x=-half, y=y, z=0.0)
            p2 = Point(x=half, y=y, z=0.0)
            marker.points.append(p1)
            marker.points.append(p2)

        return marker

    def add_6dof_controls(self, int_marker):
       
        c = InteractiveMarkerControl()
        c.orientation.w = 1.0
        c.orientation.x = 1.0
        c.orientation.y = 0.0
        c.orientation.z = 0.0
        c.name = "move_x"
        c.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(c)

       
        c = InteractiveMarkerControl()
        c.orientation.w = 1.0
        c.orientation.x = 0.0
        c.orientation.y = 0.0
        c.orientation.z = 1.0
        c.name = "move_y"
        c.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(c)

        
        c = InteractiveMarkerControl()
        c.orientation.w = 1.0
        c.orientation.x = 0.0
        c.orientation.y = 1.0
        c.orientation.z = 0.0
        c.name = "move_z"
        c.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(c)

        
        c = InteractiveMarkerControl()
        c.orientation.w = 1.0
        c.orientation.x = 1.0
        c.orientation.y = 0.0
        c.orientation.z = 0.0
        c.name = "rotate_x"
        c.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(c)

        
        c = InteractiveMarkerControl()
        c.orientation.w = 1.0
        c.orientation.x = 0.0
        c.orientation.y = 1.0
        c.orientation.z = 0.0
        c.name = "rotate_y"
        c.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(c)

       
        c = InteractiveMarkerControl()
        c.orientation.w = 1.0
        c.orientation.x = 0.0
        c.orientation.y = 0.0
        c.orientation.z = 1.0
        c.name = "rotate_z"
        c.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(c)



    def process_feedback(self, feedback):
        self.grid_pose = feedback.pose
        self.get_logger().info(
            f'Grid moved: x={self.grid_pose.position.x:.2f}, '
            f'y={self.grid_pose.position.y:.2f}, '
            f'z={self.grid_pose.position.z:.2f}'
        )
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
