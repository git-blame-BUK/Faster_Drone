
#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import pyrealsense2 as rs


class RSPointCloud(Node):
    def __init__(self):
        super().__init__("rs_pointcloud")
        self.pub = self.create_publisher(PointCloud2, "camera/points", 10)

        # RealSense pipeline --------------------------------------------------
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

        self.pipe  = rs.pipeline()
        self.pc    = rs.pointcloud()           # C++ point-cloud object
        self.align = rs.align(rs.stream.color) # keep depth & RGB registered
        self.pipe.start(cfg)

        # 30 Hz timer
        self.timer = self.create_timer(1.0/30, self.loop)

    # ------------------------------------------------------------------------
    def loop(self):
        frames = self.align.process(self.pipe.wait_for_frames())
        depth  = frames.get_depth_frame()
        color  = frames.get_color_frame()
        if not depth:              # camera warming up
            return

        # GPU/ISA: depth → XYZ -----------------------------------------------
        points = self.pc.calculate(depth)    # runs in C++
        self.pc.map_to(color)

        # C-array views – zero copy                                           
        verts = np.asanyarray(points.get_vertices(), dtype=np.float32)\
                  .reshape(-1, 3)        # (N,3) xyz
        rgb   = np.asanyarray(color.get_data(),   dtype=np.uint8)\
                  .reshape(-1, 3)         # (H*W,3) BGR

        stamp = self.get_clock().now().to_msg()
        cloud = pc2.create_cloud_xyzrgb(
            header=pc2.Header(stamp=stamp, frame_id="camera_link"),
            xyz=verts,
            rgb=rgb
        )
        self.pub.publish(cloud)


def main():
    rclpy.init()
    node = RSPointCloud()
    try:
        rclpy.spin(RSPointCloud())  
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        node.pipe.stop()


if __name__ == "__main__":
    main()
