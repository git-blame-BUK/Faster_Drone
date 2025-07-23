import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import pyrealsense2 as rs
import cv2
import numpy as np

class RealSensePointCloudPublisher:
    def __init__(self):
        self.pub_pc = rospy.Publisher('/camera/pointcloud', PointCloud2, queue_size=10)
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)  # 10Hz

        # RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 6)
        self.pipeline.start(config)

    def timer_callback(self, event):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            return

        width, height = depth_frame.get_width(), depth_frame.get_height()
        intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

        # Convert depth to numpy array for optional processing
        depth_image = np.asanyarray(depth_frame.get_data())

        # Collect 3D points
        points = []
        for y in range(0, height, 4):
            for x in range(0, width, 4):
                dist = depth_frame.get_distance(x, y)
                if dist == 0.0:
                    continue
                dx, dy, dz = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], dist)
                points.append([dx, dy, dz])

        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'camera_depth_optical_frame'
        pc2_msg = point_cloud2.create_cloud_xyz32(header, points)
        self.pub_pc.publish(pc2_msg)

        # Optional: visualize depth for debugging
        cv2.imshow("Depth", depth_image)
        cv2.waitKey(1)

def main():
    rospy.init_node('rs_pointcloud_pub')
    RealSensePointCloudPublisher()
    rospy.spin()

if __name__ == '__main__':
    main()
