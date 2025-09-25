
# imu_publisher.py  — ROS 2 / rclpy / pymavlink
# Publiziert Pixhawk-IMU als sensor_msgs/Imu mit konfigurierbarem Zeit-Offset.

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from pymavlink import mavutil

class PixhawkImuNode(Node):
    def __init__(self):
        super().__init__('pixhawk_imu_node')

        # --- Parameter ---
        self.conn = self.declare_parameter('mavlink_url', 'udpin:0.0.0.0:14600').get_parameter_value().string_value 
        self.frame_id = self.declare_parameter('frame_id', 'imu_link').get_parameter_value().string_value
        # sinnvoller Startwert: +4 ms (Jetson Empfangszeit leicht nach vorn schieben)
        self.offset_sec = float(self.declare_parameter('imu_time_offset', 0.004).get_parameter_value().double_value)

        # --- Publisher ---
        self.pub = self.create_publisher(Imu, '/imu0', qos_profile_sensor_data)

        # --- MAVLink verbinden ---
        self.mav = mavutil.mavlink_connection(self.conn)
        # self.get_logger().info(f'Pixhawk via MAVLink: {self.port} @ {self.baud}')

        # Optional: Datenstrom anfragen (manche FC senden eh automatisch)
        try:
            self.mav.mav.request_data_stream_send(
                self.mav.target_system,
                self.mav.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,  # enthält HIGHRES_IMU
                200,  # ~200 Hz
                1)
        except Exception:
            pass

        # Timer: so schnell wie möglich pollen
        self.timer = self.create_timer(0.0, self.poll)

        # Simple Covariances (später kalibrieren/anpassen)
        self.gyro_cov = [0.0005,0.0,0.0, 0.0,0.0005,0.0, 0.0,0.0,0.0005]
        self.acc_cov  = [0.02,0.0,0.0,    0.0,0.02,0.0,    0.0,0.0,0.02]

    def poll(self):
        # Bevorzugt HIGHRES_IMU, fällt ggf. auf SCALED_IMU2/3 zurück
        msg = self.mav.recv_match(type=['HIGHRES_IMU','SCALED_IMU2','SCALED_IMU','SCALED_IMU3'], blocking=False)
        if not msg:
            return

        imu = Imu()
        # Zeitstempel: Systemzeit + konfigurierter Offset
        t = self.get_clock().now() + Duration(seconds=self.offset_sec)
        imu.header = Header()
        imu.header.stamp = t.to_msg()
        imu.header.frame_id = self.frame_id

        # HIGHRES_IMU: xacc/yacc/zacc [m/s^2], xgyro/ygyro/zgyro [rad/s]
        if msg.get_type() == 'HIGHRES_IMU':
            imu.angular_velocity.x = float(msg.xgyro)
            imu.angular_velocity.y = float(msg.ygyro)
            imu.angular_velocity.z = float(msg.zgyro)
            imu.linear_acceleration.x = float(msg.xacc)
            imu.linear_acceleration.y = float(msg.yacc)
            imu.linear_acceleration.z = float(msg.zacc)

        else:
            # SCALED_IMU*: acc in mg, gyro in mrad/s → umrechnen
            # Doku: https://mavlink.io/en/messages/common.html#SCALED_IMU
            imu.angular_velocity.x = float(msg.xgyro) / 1000.0  # mrad/s → rad/s
            imu.angular_velocity.y = float(msg.ygyro) / 1000.0
            imu.angular_velocity.z = float(msg.zgyro) / 1000.0
            imu.linear_acceleration.x = float(msg.xacc) * 9.80665 / 1000.0  # mg → m/s^2
            imu.linear_acceleration.y = float(msg.yacc) * 9.80665 / 1000.0
            imu.linear_acceleration.z = float(msg.zacc) * 9.80665 / 1000.0

        imu.angular_velocity_covariance = self.gyro_cov
        imu.linear_acceleration_covariance = self.acc_cov
        imu.orientation_covariance[0] = -1.0  # keine Orientierung

        self.pub.publish(imu)

def main():
    rclpy.init()
    node = PixhawkImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
