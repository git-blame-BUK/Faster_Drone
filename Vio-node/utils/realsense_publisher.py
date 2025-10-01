
#!/usr/bin/env python3
# Publisht: /cam0/image_raw, /cam1/image_raw, /cam0/camera_info, /cam1/camera_info

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
import pyrealsense2 as rs
import numpy as np
import cv2
from cv_bridge import CvBridge
import yaml
from pathlib import Path



class RealSenseStereoNode(Node):
    def __init__(self):
        super().__init__('realsense_stereo_publisher')

        # Parameter
        self.declare_parameter('width', 848)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('serial', '')
        self.declare_parameter('cam0_yaml', '')  # Path too cam.yml
        self.declare_parameter('cam1_yaml', '')  
        self.declare_parameter('use_device_time', True)  # Timestamp from realsense 

        self.width  = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps    = int(self.get_parameter('fps').value)
        self.serial = str(self.get_parameter('serial').value)
        self.cam0_yaml = str(self.get_parameter('cam0_yaml').value)
        self.cam1_yaml = str(self.get_parameter('cam1_yaml').value)
        self.use_device_time = bool(self.get_parameter('use_device_time').value)

        qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        #Publisher
        self.pub_cam0 = self.create_publisher(Image, '/cam0/image_raw', qos)
        self.pub_cam1 = self.create_publisher(Image, '/cam1/image_raw', qos)
        self.pub_ci0  = self.create_publisher(CameraInfo, '/cam0/camera_info', qos)
        self.pub_ci1  = self.create_publisher(CameraInfo, '/cam1/camera_info', qos)

        self.bridge = CvBridge()

        # RealSense Pipeline
        cfg = rs.config()
        if self.serial:
            cfg.enable_device(self.serial)

        # D435: IR1 = left ir cam, IR2 = right ir cam
        cfg.enable_stream(rs.stream.infrared, 1, self.width, self.height, rs.format.y8, self.fps)
        cfg.enable_stream(rs.stream.infrared, 2, self.width, self.height, rs.format.y8, self.fps)

        self.pipe = rs.pipeline()
        self.profile = self.pipe.start(cfg)

        # Optional: Auto-Exposure finetuning 
        self.get_logger().info('RealSense D435 Stereo gestartet (IR1/IR2).')
        self.timer = self.create_timer(0.0, self.loop_once)  

    def loop_once(self):
        frames = self.pipe.poll_for_frames()
        if not frames:
            return

        ir1 = frames.get_infrared_frame(1)
        ir2 = frames.get_infrared_frame(2)
        if not (ir1 and ir2):
            return

        # Timestamp
        if self.use_device_time:
            # RealSense Timestamp milisec-> ROS2 Sek+Nanosec
            t_ms = ir1.get_timestamp()
            sec = int(t_ms // 1000)
            nsec = int((t_ms % 1000) * 1e6)
            stamp = rclpy.time.Time(seconds=sec, nanoseconds=nsec).to_msg()
        else:
            stamp = self.get_clock().now().to_msg()

        # In numpy
        img0 = np.asanyarray(ir1.get_data())  # Y8
        img1 = np.asanyarray(ir2.get_data())

        # too ROS Image
        msg0 = self.bridge.cv2_to_imgmsg(img0, encoding='mono8')
        msg1 = self.bridge.cv2_to_imgmsg(img1, encoding='mono8')

        msg0.header = Header(stamp=stamp, frame_id='cam0')
        msg1.header = Header(stamp=stamp, frame_id='cam1')

        # CameraInfo with Timestamp
        self.ci0.header.stamp = stamp
        self.ci1.header.stamp = stamp

        # Publish
        self.pub_cam0.publish(msg0)
        self.pub_cam1.publish(msg1)
        self.pub_ci0.publish(self.ci0)
        self.pub_ci1.publish(self.ci1)

    def destroy_node(self):
        try:
            self.pipe.stop()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = RealSenseStereoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
