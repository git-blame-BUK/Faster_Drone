#!/usr/bin/env python3
"""
Lightweight stereo‑depth + IMU EKF VIO for Jetson Nano.
Publiziert SnapStack‑`State` direkt auf `/state`, wie es FASTER‑Planner
im UAV‑Modus erwartet (Launch‑Remap `~state -> state`).

Subscribes:
  * /camera/depth/image_rect_raw  (sensor_msgs/Image, Depth16)
  * /mavros/imu/data              (sensor_msgs/Imu)

Publishes:
  * /state                        (snapstack_msgs/State)
  * /camera/cloud wird vom launch file erwartet ist aber nur simuliert
  * /hier muss faster_launch bearbeitet werden --> bereits kommentier in faster_launch (realsense_ros published /camera/depth/color/points)

"""
import rospy, cv2, numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Imu, Image
from snapstack_msgs.msg import State
from message_filters import ApproximateTimeSynchronizer, Subscriber
from geometry_msgs.msg import Quaternion, Vector3

bridge = CvBridge()
KF_P = np.diag([1e-3]*6)      # initial covariance
KF_Q = np.diag([1e-4]*6)      # process noise
KF_R = np.diag([1e-2]*6)      # measurement noise

state = np.zeros(6)           # x,y,z, roll,pitch,yaw
P = KF_P.copy()


def sync_cb(depth_img_msg, imu_msg):
    global state, P
    depth_img = bridge.imgmsg_to_cv2(depth_img_msg).astype(np.float32) / 1000.0
    z = np.nanmean(depth_img)
    if np.isnan(z):
        return
    z_meas = np.array([0, 0, z, 0, 0, 0])

    dt = 1.0 / 30.0
    ax = imu_msg.linear_acceleration
    state[:3] += np.array([0, 0, -ax.z]) * 0.5 * dt**2
    P += KF_Q

    # Kalman update
    y = z_meas - state
    S = P + KF_R
    K = P @ np.linalg.inv(S)
    state += K @ y
    P = (np.eye(6) - K) @ P

    # Build State message
    st = State()
    st.header = depth_img_msg.header
    st.pos = Vector3(*state[:3])
    st.vel = Vector3()                # unknown → 0
    st.acc = Vector3()
    st.att = Quaternion(w=1.0)        # simple orientation stub
    pub.publish(st)

rospy.init_node("light_vio_state")
sub_depth = Subscriber("/camera/depth/image_rect_raw", Image)
sub_imu   = Subscriber("/mavros/imu/data", Imu)
ats = ApproximateTimeSynchronizer([sub_depth, sub_imu], queue_size=20, slop=0.03)
ats.registerCallback(sync_cb)

pub = rospy.Publisher("/state", State, queue_size=5)
rospy.loginfo("Lightweight VIO node for FASTER UAV‑mode started.")
rospy.spin()