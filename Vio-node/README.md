# How to Vio ?
- utilize Nvidia Isaac gpu accelerated Vslam to feed into PX4 EKF
- VIO: Visual Inertial Odometry --> Pose Estimation
- ROS: Robot Operating System --> Internal Networking of Flight Controller Camera and Onboard Computer (ROS2 Humble)

## Tools
1. debug.py: checks for all important Datapoints from realsense 
2. poc.py: checks for realsense devices on host
3. imu_publisher.py: Systemstime stamped imu publisher for ROS

