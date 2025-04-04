

import rospy
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge

# Assume you know intrinsics:
fx = 459.889
fy = 459.889
cx = 320.0
cy = 240.0

bridge = CvBridge()
pub = rospy.Publisher("/depth/points", PointCloud2, queue_size=1)

def depth_callback(msg):
    depth = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    points = []
    for v in range(0, depth.shape[0], 4):  # downsample
        for u in range(0, depth.shape[1], 4):
            Z = float(depth[v, u]) / 1000.0  # mm → m
            if Z == 0: continue
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy
            points.append([X, Y, Z])

    header = msg.header
    cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
    pub.publish(cloud_msg)

rospy.init_node("depth_to_cloud")
rospy.Subscriber("/depth/image_raw", Image, depth_callback)
rospy.spin()
