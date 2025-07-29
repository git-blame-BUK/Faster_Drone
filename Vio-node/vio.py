


#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import pyrealsense2 as rs


class RSPointCloud(Node):
    def __init__(self):
        super().__init__("rs_pointcloud")

        self.pub = self.create_publisher(PointCloud2, "camera/points", 10)

        # RealSense Setup -----------------------------------------------------
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

        self.pipe = rs.pipeline()
        self.pc = rs.pointcloud()
        self.align = rs.align(rs.stream.color)

        self.pipe.start(cfg)
        self.get_logger().info("RealSense pipeline started")

        self.timer = self.create_timer(1.0 / 30, self.loop)

    def loop(self):
        frames = self.align.process(self.pipe.wait_for_frames())
        depth = frames.get_depth_frame()
        color = frames.get_color_frame()

        if not depth or not color:
            self.get_logger().warn("Frames not ready")
            return

        self.pc.map_to(color)
        points = self.pc.calculate(depth)

        verts = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)  # (N,3)

        # Convert RGB image to packed float32 per point
        rgb = np.asanyarray(color.get_data(), dtype=np.uint8).reshape(-1, 3)  # (N,3)
        rgb_uint32 = (rgb[:, 0].astype(np.uint32) << 16) | \
                     (rgb[:, 1].astype(np.uint32) << 8) | \
                     rgb[:, 2].astype(np.uint32)
        rgb_float = rgb_uint32.view(np.float32)

        # Combine xyz and rgb into a single array
        points_rgb = np.hstack((verts, rgb_float.reshape(-1, 1)))  # (N,4)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_link"

        cloud_msg = pc2.create_cloud(header, [
            PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1)
        ], points_rgb)

        self.pub.publish(cloud_msg)


def main():
    rclpy.init()
    node = RSPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipe.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()