#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import math
import random

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.command_subscription = self.create_subscription(String, '/robot_operation_mode', self.command_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Paramètres de base
        self.linear_speed = 0.2
        self.angular_speed_base = 0.2
        self.safe_distance = 0.4

        self.state = 'IDLE'
        self.action_timer = 0
        self.manual_cmd_vel = Twist()

        # --- Variables Odométrie (Rotation) ---
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.rotation_angle = 10
        self.precision_tolerance_rad = math.radians(1.0)

        # --- Variables Odométrie (Distance Linéaire) ---
        self.current_x = 0.0
        self.current_y = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.target_distance = 0.10
        self.move_direction = 1
        self.dist_tolerance = 0.01

        self.get_logger().info("Robot prêt (Rotation et Distance précises).")


    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def command_callback(self, msg: String):
        command = msg.data.lower().strip()
        self.get_logger().info(f"Message reçu : {command}")

        if command == "stop":
            self.state = 'IDLE'
            self.stop_robot()
            return
        elif command == "listen":
            self.state = 'LISTENING'
            self.stop_robot()
            return
        elif command == "start":
            self.state = 'EXPLORE_FWD'
            self.manual_cmd_vel = Twist()
            self.action_timer = 0
            return

        if self.state != 'LISTENING':
            self.get_logger().warn("Je suis occupé, commande ignorée.")
            return

        if command == "left":
            angle_rad = math.radians(self.rotation_angle)
            self.target_yaw = self.normalize_angle(self.current_yaw + angle_rad)
            self.state = 'ROTATING_PRECISE'
            self.get_logger().info(f"Rotation gauche {self.rotation_angle}°")
            return

        elif command == "right":
            angle_rad = math.radians(self.rotation_angle)
            self.target_yaw = self.normalize_angle(self.current_yaw - angle_rad)
            self.state = 'ROTATING_PRECISE'
            self.get_logger().info(f"Rotation droite {self.rotation_angle}°")
            return

        elif command == "forward":
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.move_direction = 1
            self.state = 'MOVING_PRECISE'
            self.get_logger().info(f"Avance de {self.target_distance*100:.0f}cm")
            return

        elif command == "backward":
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.move_direction = -1
            self.state = 'MOVING_PRECISE'
            self.get_logger().info(f"Recule de {self.target_distance*100:.0f}cm...")
            return

    def scan_callback(self, msg: LaserScan):
        cmd = Twist()

        if self.state == 'IDLE' or self.state == 'LISTENING':
            self.publisher.publish(Twist())
            return

        # Lecture distance avant pour sécurité
        valid_ranges = [r for r in (msg.ranges[0:30] + msg.ranges[-30:]) if not (r == float('inf'))]
        front_dist = min(valid_ranges) if valid_ranges else 10.0

        if self.state == 'ROTATING_PRECISE':
            error = self.normalize_angle(self.target_yaw - self.current_yaw)
            if abs(error) < self.precision_tolerance_rad:
                self.stop_robot()
                self.state = 'LISTENING'
                self.get_logger().info("Rotation terminée.")
            else:
                kp = 1.5
                angular_z = kp * error
                angular_z = max(min(angular_z, 0.5), -0.5)
                if 0 < angular_z < 0.1: angular_z = 0.1
                if -0.1 < angular_z < 0: angular_z = -0.1

                cmd.angular.z = angular_z
                self.publisher.publish(cmd)
            return

        if self.state == 'MOVING_PRECISE':
            # Sécurité : Si on avance et qu'il y a un obstacle
            if self.move_direction == 1 and front_dist < self.safe_distance:
                self.stop_robot()
                self.state = 'LISTENING'
                self.get_logger().warn("Obstacle détecté ! Arrêt d'urgence.")
                return

            dx = self.current_x - self.start_x
            dy = self.current_y - self.start_y
            distance_traveled = math.sqrt(dx*dx + dy*dy)

            remaining = self.target_distance - distance_traveled

            if remaining <= self.dist_tolerance:
                self.stop_robot()
                self.state = 'LISTENING'
                self.get_logger().info("Distance 10cm atteinte.")
            else:
                kp_dist = 2.0
                speed = kp_dist * remaining

                speed = max(min(speed, 0.2), 0.05)

                cmd.linear.x = speed * self.move_direction
                self.publisher.publish(cmd)
            return

        if self.state == 'MANUAL':
            self.publisher.publish(self.manual_cmd_vel)
            self.action_timer -= 1
            if self.action_timer <= 0:
                self.state = 'LISTENING'
                self.stop_robot()
            return

        if self.state == 'EXPLORE_FWD':
            if front_dist > self.safe_distance:
                cmd.linear.x = self.linear_speed
            else:
                self.state = 'EXPLORE_TURN'
                direction = random.choice([-1, 1])
                self.manual_cmd_vel = Twist()
                self.manual_cmd_vel.angular.z = self.angular_speed_base * direction
                self.action_timer = random.randint(15, 40)

        elif self.state == 'EXPLORE_TURN':
            cmd.angular.z = self.manual_cmd_vel.angular.z
            self.action_timer -= 1
            if self.action_timer <= 0:
                self.state = 'EXPLORE_FWD'

        self.publisher.publish(cmd)

    def stop_robot(self):
        self.publisher.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()