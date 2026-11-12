"""Gazebo hardware adapter. Navigation and localization remain the real code."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan

class Adapter(Node):

    def __init__(self):
        super().__init__('sim_hardware')
        self.velocity = self.create_publisher(Twist, '/sim/cmd_vel', 1)
        self.odom = self.create_publisher(Odometry, '/odom/raw', 10)
        self.imu = self.create_publisher(Imu, '/imu_data_ros', 10)
        self.scan = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.command, 1)
        self.create_subscription(Odometry, '/sim/wheel_odom', self.wheel, qos_profile_sensor_data)
        self.create_subscription(Imu, '/sim/imu', self.inertial, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/sim/scan', self.laser, qos_profile_sensor_data)
        self.last_command = self.get_clock().now().nanoseconds
        self.create_timer(0.05, self.watchdog)

    def command(self, msg):
        self.last_command = self.get_clock().now().nanoseconds
        self.velocity.publish(msg)

    def watchdog(self):
        if (self.get_clock().now().nanoseconds - self.last_command) / 1000000000.0 > 0.8:
            self.velocity.publish(Twist())

    def wheel(self, msg):
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.twist.covariance[0] = 0.03 ** 2
        msg.twist.covariance[7] = 0.02 ** 2
        self.odom.publish(msg)

    def inertial(self, msg):
        msg.header.frame_id = 'gyro_link'
        for i in (0, 4, 8):
            msg.orientation_covariance[i] = 0.2 ** 2
            msg.angular_velocity_covariance[i] = 0.03 ** 2
            msg.linear_acceleration_covariance[i] = 0.1 ** 2
        self.imu.publish(msg)

    def laser(self, msg):
        msg.header.frame_id = 'laser'
        self.scan.publish(msg)
if __name__ == '__main__':
    rclpy.init()
    node = Adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.velocity.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
