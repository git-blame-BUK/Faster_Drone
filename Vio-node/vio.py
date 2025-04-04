#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import tf.transformations as tf_trans
import threading

class VisualInertialOdometryNode:
    def __init__(self):
        rospy.init_node('vio_node')
        self.bridge = CvBridge()

        # Camera intrinsics (adjust for your OAK-D Lite calibration)
        self.fx, self.fy = 459.889, 459.889
        self.cx, self.cy = 320.0, 240.0

        self.prev_gray_image = None
        self.tracked_features_prev_frame = None

        self.rotation_matrix = np.eye(3)
        self.current_position = np.zeros((3,))
        self.last_imu_timestamp = None

        self.lock = threading.Lock()

        # ROS subscribers
        rospy.Subscriber('/depth/image_raw', Image, self.on_depth_image)
        rospy.Subscriber('/mavros/imu/data', Imu, self.on_imu_data)

        # ROS publishers
        self.pose_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
        self.local_pose_pub = rospy.Publisher('/mavros/local_position/pose', PoseStamped, queue_size=1)

        rospy.loginfo("🔧 VIO Node initialized and running.")

    def on_imu_data(self, msg):
        with self.lock:
            now = msg.header.stamp.to_sec()
            if self.last_imu_timestamp is None:
                self.last_imu_timestamp = now
                return

            dt = now - self.last_imu_timestamp
            self.last_imu_timestamp = now

            # Angular velocity
            gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
            omega = np.array([[0, -gz, gy],
                              [gz, 0, -gx],
                              [-gy, gx, 0]])

            self.rotation_matrix = self.rotation_matrix @ (np.eye(3) + omega * dt)

    def on_depth_image(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if depth_image.dtype == np.uint16:
                depth_image = depth_image.astype(np.float32) / 1000.0  # Convert mm → meters

            gray_image = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

            if self.prev_gray_image is None:
                self.prev_gray_image = gray_image
                self.tracked_features_prev_frame = cv2.goodFeaturesToTrack(gray_image, 200, 0.01, 7)
                return

            # Track features
            tracked_features_curr_frame, status, _ = cv2.calcOpticalFlowPyrLK(
                self.prev_gray_image, gray_image, self.tracked_features_prev_frame, None)
            
            good_prev = self.tracked_features_prev_frame[status.flatten() == 1]
            good_curr = tracked_features_curr_frame[status.flatten() == 1]

            # Estimate 3D motion
            translation_estimate = self.estimate_translation_from_flow(good_prev, good_curr, depth_image)
            if translation_estimate is not None:
                with self.lock:
                    self.current_position += self.rotation_matrix @ translation_estimate

            self.publish_fused_pose()

            # Prepare for next frame
            self.prev_gray_image = gray_image
            self.tracked_features_prev_frame = cv2.goodFeaturesToTrack(gray_image, 200, 0.01, 7)

        except Exception as e:
            rospy.logwarn("❗ Depth image callback error: %s", e)

    def estimate_translation_from_flow(self, features_prev, features_curr, depth_img):
        motions = []
        for (pt1, pt2) in zip(features_prev, features_curr):
            x1, y1 = int(pt1[0]), int(pt1[1])
            x2, y2 = int(pt2[0]), int(pt2[1])
            if depth_img[y1, x1] == 0 or depth_img[y2, x2] == 0:
                continue
            Z1, Z2 = depth_img[y1, x1], depth_img[y2, x2]
            X1 = (x1 - self.cx) * Z1 / self.fx
            Y1 = (y1 - self.cy) * Z1 / self.fy
            X2 = (x2 - self.cx) * Z2 / self.fx
            Y2 = (y2 - self.cy) * Z2 / self.fy
            motions.append([X2 - X1, Y2 - Y1, Z2 - Z1])

        if not motions:
            return None
        return np.median(np.array(motions), axis=0)

    def publish_fused_pose(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"

        pose_msg.pose.position.x = self.current_position[0]
        pose_msg.pose.position.y = self.current_position[1]
        pose_msg.pose.position.z = self.current_position[2]

        # Rotation matrix to quaternion
        T = np.eye(4)
        T[:3, :3] = self.rotation_matrix
        quat = tf_trans.quaternion_from_matrix(T)

        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        self.pose_pub.publish(pose_msg)
        self.local_pose_pub.publish(pose_msg)

if __name__ == '__main__':
    try:
        VisualInertialOdometryNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

