
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import pyrealsense2 as rs
import struct

class RSPointCloud(Node):
    def __init__(self):
        super().__init__('rs_pointcloud')
       # Empfohlenes QoS-Profil für Sensor-Daten (RELIABLE + depth=10)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=1
        )      
        self.publisher_ = self.create_publisher(PointCloud2, 'camera/points', qos)

        # RealSense Pipeline Setup
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 15)

        self.align = rs.align(rs.stream.color)
        self.pc = rs.pointcloud()

        self.pipe.start(cfg)
        self.get_logger().info("RealSense pipeline started")

        self.timer = self.create_timer(1.0 / 15.0, self.loop)

    def loop(self):
        t0 = time.time() # time  debug
        frames = self.align.process(self.pipe.wait_for_frames())
        depth = frames.get_depth_frame()
        color = frames.get_color_frame()

        if not depth or not color:
            self.get_logger().warn("Incomplete frame")
            return

        self.pc.map_to(color)
        points = self.pc.calculate(depth)

        # xyz        
        verts_struct = np.asarray(points.get_vertices())
        verts = np.vstack([verts_struct['f0'], verts_struct['f1'], verts_struct['f2']]).T.astype(np.float32)

        # rgb as packed float
        rgb_img = np.asarray(color.get_data(), dtype=np.uint8).reshape(-1, 3)
        rgb_packed = np.array([
            struct.unpack('f', struct.pack('I', r << 16 | g << 8 | b))[0]
            for r, g, b in rgb_img
        ], dtype=np.float32).reshape(-1, 1)

        cloud_data = np.hstack((verts, rgb_packed))

        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"  # <– wichtig für RViz

        msg = pc2.create_cloud(header, fields, cloud_data)
        t1 = time.time()
        self.get_logger().info(f"Loop took {t1 - t0:.3f} s")
        self.get_logger().info(f"Publishing frame at {self.get_clock().now().nanoseconds}") # log time  debugger
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = RSPointCloud()
    try:
        rclpy.spin(node)
    finally:
        node.pipe.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()