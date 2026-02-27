#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import ByteMultiArray, Header
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import struct
import zlib

# --- LA FONCTION MAGIQUE QUI CORRIGE LE CRASH ---
def to_bytes_any(data) -> bytes:
    if isinstance(data, (bytes, bytearray, memoryview)):
        return bytes(data)
    try:
        return bytes(data)
    except TypeError:
        pass
    out = bytearray()
    for x in data:
        if isinstance(x, int):
            out.append(x & 0xFF)
        elif isinstance(x, (bytes, bytearray, memoryview)):
            if len(x):
                out.append(x[0])
        else:
            out.append(int(x) & 0xFF)
    return bytes(out)


class PointCloudTranslator(Node):
    def __init__(self):
        super().__init__('pc_translator_node')
        
        self.sub = self.create_subscription(
            ByteMultiArray, 
            '/camera/combined', 
            self.decoder_callback, 
            qos_profile_sensor_data
        )
        
        self.pub = self.create_publisher(PointCloud2, '/real_pointcloud', 10)
        
        self.MAGIC = b"RCMB"
        self.HDR_FMT = "<4sIQIIfffII"
        self.HDR_SIZE = struct.calcsize(self.HDR_FMT)

        self.get_logger().info('--- TRADUCTEUR ACTIF : /camera/combined ---> /real_pointcloud ---')

    def decoder_callback(self, msg):
        # On utilise la fonction de nettoyage ici !
        b = to_bytes_any(msg.data)
        
        if len(b) < self.HDR_SIZE: return

        try:
            magic, ver, stamp_ns, w, h, yaw, pitch, roll, jpeg_sz, pc_z_sz = struct.unpack_from(self.HDR_FMT, b, 0)
        except Exception:
            return

        if magic != self.MAGIC or pc_z_sz == 0: return

        # Extraction de la partie zlib
        off = self.HDR_SIZE + int(jpeg_sz)
        pc_z = b[off : off + int(pc_z_sz)]

        try:
            pc_raw = zlib.decompress(pc_z)
            if len(pc_raw) >= 4:
                count = struct.unpack_from("<I", pc_raw, 0)[0]
                expect = 4 + count * 5 * 4
                
                if len(pc_raw) == expect:
                    # Extraction des coordonnées XYZ
                    points_5d = np.frombuffer(pc_raw, dtype=np.float32, offset=4).reshape((count, 5))
                    xyz = points_5d[:, 0:3]

                    # Création du message PointCloud2 standard pour RViz
                    header = Header()
                    header.stamp = self.get_clock().now().to_msg()
                    header.frame_id = 'base_link' 
                    
                    # Conversion en format RViz
                    pc2_msg = pc2.create_cloud_xyz32(header, xyz.tolist())
                    
                    # On publie pour RViz !
                    self.pub.publish(pc2_msg)
        except Exception as e:
            pass

def main():
    rclpy.init()
    node = PointCloudTranslator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
