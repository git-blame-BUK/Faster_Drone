
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
            return

        points = self.pc.calculate(depth)
        self.pc.map_to(color)

        verts = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
        rgb = np.asanyarray(color.get_data(), dtype=np.uint8).reshape(-1, 3)

        # Pack RGB into float32 field as required by PointCloud2
        rgb_packed = np.left_shift(rgb[:, 2], 16) + np.left_shift(rgb[:, 1], 8) + rgb[:, 0]
        rgb_float = rgb_packed.view(np.float32)

        points_rgb = np.hstack((verts, rgb_float.reshape(-1, 1)))

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_link"

        cloud = pc2.create_cloud(
            header,
            fields=[
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('rgb', 12, PointField.FLOAT32, 1),
            ],
            points=points_rgb
        )
        self.pub.publish(cloud)


def main():
    rclpy.init()
    node = RSPointCloud()
    try:
        rclpy.spin(node)
    finally:
        node.pipe.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
