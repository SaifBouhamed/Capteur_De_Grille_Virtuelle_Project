#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
# C'est ce profil qui force Gazebo à envoyer les données
from rclpy.qos import qos_profile_sensor_data 

class GazeboActivator(Node):
    def __init__(self):
        super().__init__('gazebo_force_wake')
        
        self.get_logger().info('--- ACTIVATEUR DÉMARRÉ : Je maintiens Gazebo éveillé ! ---')
        
        # On s'abonne juste pour dire "J'écoute".
        # On ne fait rien des données, on les jette.
        self.subscription = self.create_subscription(
            PointCloud2,
            '/depth_camera/points',
            self.callback,
            qos_profile_sensor_data # LE PASS VIP
        )

    def callback(self, msg):
        # On reçoit le message pour maintenir la connexion active
        # Mais on ne fait aucun calcul pour ne pas charger le PC
        pass

def main(args=None):
    rclpy.init(args=args)
    node = GazeboActivator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()