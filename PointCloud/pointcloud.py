import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu, Image
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Vector3, Point, Pose, PoseWithCovariance, Twist, TwistWithCovariance
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import struct

class RealSensePointCloudPublisher(Node):
    def __init__(self):
        super().__init__('rs_pointcloud_pub')
        self.pub_pc = self.create_publisher(PointCloud2, '/camera/pointcloud', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        
        # RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 6)
        self.pipeline.start(config)
        self.bridge = CvBridge()

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            return

        width, height = depth_frame.get_width(), depth_frame.get_height()
        intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

        # Collect 3D points
        points = []
        for y in range(0, height, 4):
            for x in range(0, width, 4):
                dist = depth_frame.get_distance(x, y)
                if dist == 0.0:
                    continue
                dx, dy, dz = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], dist)
                points.append([dx, dy, dz])

        header = rclpy.time.Time().to_msg()
        pc2_msg = point_cloud2.create_cloud_xyz32(
            frame_id='camera_depth_optical_frame',
            points=points
        )
        pc2_msg.header.stamp = self.get_clock().now().to_msg()
        pc2_msg.header.frame_id = 'camera_depth_optical_frame'
        self.pub_pc.publish(pc2_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RealSensePointCloudPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
