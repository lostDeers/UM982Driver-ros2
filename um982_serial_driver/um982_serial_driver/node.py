import serial

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import RTCM
from sensor_msgs.msg import NavSatFix
import transforms3d
from rclpy.qos import qos_profile_system_default
from rclpy.logging import get_logger

from .decoder import Decoder


class GNSSNode(Node):
    def __init__(self) -> None:
        Node.__init__(self, "gnss_node")
        self.declare_parameter("odometry_topic", "odom")
        self.declare_parameter("nav_sat_fix_topic", "nav_sat_fix")
        self.declare_parameter("gngga_topic", "gngga")
        self.declare_parameter("rtcm_topic", "rtcm")
        self.declare_parameter("serial_path", "/dev/ttyUSB0")
        self.declare_parameter("serial_baudrate", 115200)
        self.declare_parameter("frame_id", "gnss_link")
        self.declare_parameter("decoder_rate", 10)
        sp = self.get_parameter("serial_path").get_parameter_value().string_value
        baudrate = (
            self.get_parameter("serial_baudrate").get_parameter_value().integer_value
        )
        
        # add logger
        self.logger = get_logger("gnss_node")

        try:
            ser = serial.Serial(sp, baudrate)
            self.logger.info(f"connected to {sp} at {baudrate} baud")
        except serial.SerialException as e:
            self.logger.error(f"failed to connect to {sp} at {baudrate} baud: {e}")
            ser = None
            
        
        # member variables
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.ser = ser
        self.decoder = Decoder()
        self.isrunning = True

        # topics
        odom_topic = (
            self.get_parameter("odometry_topic").get_parameter_value().string_value
        )
        nav_sat_fix_topic = (
            self.get_parameter("nav_sat_fix_topic").get_parameter_value().string_value
        )
        gngga_topic = (
            self.get_parameter("gngga_topic").get_parameter_value().string_value
        )
        rtcm_topic = self.get_parameter("rtcm_topic").get_parameter_value().string_value

        # publishers
        self.odom_publisher = self.create_publisher(
            Odometry, odom_topic, qos_profile_system_default
        )
        self.nav_sat_fix_publisher = self.create_publisher(
            NavSatFix, nav_sat_fix_topic, qos_profile_system_default
        )
        self.gngga_publisher = self.create_publisher(
            String, gngga_topic, qos_profile_system_default
        )

        # timers
        decoder_rate = self.get_parameter("decoder_rate").get_parameter_value().double_value
        self.decoder_timer = self.create_timer(decoder_rate, self.decode_next_msgs)
        self.odom_pub_timer = self.create_timer(0.5, self.nmea_msg_to_odom)
        self.nav_sat_fix_pub_timer = self.create_timer(0.1, self.nmea_msg_to_nav_sat_fix)
        self.gngga_pub_timer = self.create_timer(0.5, self.pub_GNGGA)

        # subscriptions
        self.RTCM_subscription = self.create_subscription(
            RTCM, rtcm_topic, self.send_RTCM, qos_profile_system_default
        )

    def decode_next_msgs(self) -> bool:
        if self.ser is None:
            self.logger.error("COM port is not open or not connected.")
            return False

        try:
            msg = self.ser.read_until(b"\r\n")
            if len(msg) == 0:
                return False
            else:
                
                self.decoder.decode(msg.decode())
                return True
        except serial.SerialException as e:
            self.logger.error(f"failed to read from serial port: {e}")
            return False

    def nmea_msg_to_odom(self) -> None:
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = "base_link"

        # set position
        msg.pose.pose.position.x = float(self.decoder.utm_x)
        msg.pose.pose.position.y = float(self.decoder.utm_y)
        msg.pose.pose.position.z = float(self.decoder.bestpos_hgt)

        # RPY -> orientation
        q = transforms3d.euler.euler2quat(
            self.decoder.roll, self.decoder.pitch, self.decoder.heading
        )
        msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # fix covariance of position
        msg.pose.covariance[0] = self.decoder.bestpos_latstd**2  # position.x的方差
        msg.pose.covariance[7] = self.decoder.bestpos_lonstd**2  # position.y的方差
        msg.pose.covariance[14] = self.decoder.bestpos_hgtstd**2  # position.z的方差

        # fix covariance of orientation with small value
        small_var = 1e-6
        msg.pose.covariance[21] = small_var
        msg.pose.covariance[28] = small_var
        msg.pose.covariance[35] = small_var

        # velocity
        msg.twist.twist.linear.x = float(self.decoder.vel_north)
        msg.twist.twist.linear.y = float(self.decoder.vel_east)
        msg.twist.twist.linear.z = float(self.decoder.vel_up)

        # covariance of velocity
        msg.twist.covariance[0] = self.decoder.vel_east_std**2
        msg.twist.covariance[1] = (
            self.decoder.vel_hor_cov**2
        )
        msg.twist.covariance[6] = (
            self.decoder.vel_hor_cov**2
        )
        msg.twist.covariance[7] = self.decoder.vel_north_std**2
        msg.twist.covariance[14] = self.decoder.vel_up_std**2

        self.odom_publisher.publish(msg)

    def nmea_msg_to_nav_sat_fix(self) -> None:
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.latitude = float(self.decoder.bestpos_lat)
        msg.longitude = float(self.decoder.bestpos_lon)
        msg.altitude = float(self.decoder.bestpos_hgt)
        msg.position_covariance[0] = 0.1*self.decoder.bestpos_latstd**2
        msg.position_covariance[4] = 0.1*self.decoder.bestpos_lonstd**2
        msg.position_covariance[8] = 0.1*self.decoder.bestpos_hgtstd**2
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        if msg.latitude==0 or msg.longitude==0:
            self.logger.error("Invalid GPS data received, skipping publishing.")
            return
        self.nav_sat_fix_publisher.publish(msg)

    def pub_GNGGA(self) -> None:
        msg = String()
        msg.data = self.decoder.GNGGA_msg
        self.gngga_publisher.publish(msg)

    def send_RTCM(self, msg: RTCM) -> None:
        if self.ser is not None:
            self.ser.write(bytes(msg.data))
        else:
            self.logger.error("COM port is not open or not connected, cannot send RTCM data.")


def main():
    rclpy.init()
    gnss = GNSSNode()
    rclpy.spin(gnss)
    gnss.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
