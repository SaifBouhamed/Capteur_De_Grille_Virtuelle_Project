#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from rclpy.time import Time
import numpy as np
import time

def get_one_frame():
    node = rclpy.create_node('grid_worker_phoenix')
    
    try:
        topic = '/depth_camera/points' 
        
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
                
                if tf_buffer.can_transform('base_link', msg.header.frame_id, Time()):
                    trans = tf_buffer.lookup_transform('base_link', msg.header.frame_id, Time())
                    points = process_cloud(msg)
                    
                    if len(points) > 0:
                        publish_marker(points, trans, pub, node)
                        success = True
                        break 
                
    finally:
        node.destroy_node()
        return success

def process_cloud(cloud_msg):
    try:
        raw = cloud_msg.data
        step = cloud_msg.point_step
        skip = 50 
        
        off_x=0; off_y=4; off_z=8
        for f in cloud_msg.fields:
            if f.name=='x': off_x=f.offset
            if f.name=='y': off_y=f.offset
            if f.name=='z': off_z=f.offset
            
        import struct
        unpack = struct.unpack_from
        points = []
        limit = len(raw)
        stride = step * skip
        
        for i in range(0, limit, stride):
            if i + 12 > limit: break
            x, y, z = unpack('fff', raw, i + off_x)
            if x==x and y==y and z==z:
                points.append([x,y,z])
        return np.array(points)
    except:
        return np.array([])

def publish_marker(xyz_points, tf, pub, node):
    rx=tf.transform.rotation.x; ry=tf.transform.rotation.y
    rz=tf.transform.rotation.z; rw=tf.transform.rotation.w
    tx=tf.transform.translation.x; ty=tf.transform.translation.y; tz=tf.transform.translation.z
    
    R = np.array([
        [1 - 2*ry*ry - 2*rz*rz, 2*rx*ry - 2*rz*rw,     2*rx*rz + 2*ry*rw],
        [2*rx*ry + 2*rz*rw,     1 - 2*rx*rx - 2*rz*rz, 2*ry*rz - 2*rx*rw],
        [2*rx*rz - 2*ry*rw,     2*ry*rz + 2*rx*rw,     1 - 2*rx*rx - 2*ry*ry]
    ])
    
    xyz_robot = np.dot(xyz_points, R.T) + np.array([tx, ty, tz])
    
    # Grille
    grid_n = 20
    cell = 1.7 / grid_n
    base_h = 0.00
    h_map = {}
    
    mask = (xyz_robot[:,0]>0) & (xyz_robot[:,0]<2) & (xyz_robot[:,1]>-1) & (xyz_robot[:,1]<1)
    xyz_crop = xyz_robot[mask]
    
    if len(xyz_crop) == 0: return

    idx_x = ((xyz_crop[:,0] - 0) / cell).astype(int)
    idx_y = ((xyz_crop[:,1] + 1) / cell).astype(int)
    
    for k in range(len(xyz_crop)):
        ix, iy = idx_x[k], idx_y[k]
        if 0<=ix<grid_n and 0<=iy<grid_n:
            z = xyz_crop[k,2]
            if (ix,iy) not in h_map or z > h_map[(ix,iy)]:
                h_map[(ix,iy)] = z

    # Marker
    m = Marker()
    m.header.frame_id = 'base_link'
    m.header.stamp = node.get_clock().now().to_msg()
    m.ns = "pin_art"
    m.id = 0
    m.type = Marker.SPHERE_LIST
    m.action = Marker.ADD
    m.lifetime.sec = 0 
    

    ratio_taille = 0.4 
    
    m.scale.x = cell * ratio_taille
    m.scale.y = cell * ratio_taille
    m.scale.z = cell * ratio_taille
    
    m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
    
    pts = []
    for ix in range(grid_n):
        for iy in range(grid_n):
            z = base_h
            if (ix,iy) in h_map: z = max(base_h, h_map[(ix,iy)])
            
            cx = 0 + ix*cell + cell/2
            cy = -1 + iy*cell + cell/2
            pts.append(Point(x=cx, y=cy, z=z))
            
    m.points = pts
    pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    print("--- PHOENIX NODE DÉMARRÉ ---")
    
    frame_id = 0
    while rclpy.ok():
        success = get_one_frame()
        if success:
            frame_id += 1
            print(f"\rFrame #{frame_id} générée", end="")
        else:
            print(".", end="") 

    rclpy.shutdown()

if __name__ == '__main__':
    main()