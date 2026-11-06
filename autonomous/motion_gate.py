"""Final command gate: stop on expired input, missing pose or mission pause."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class Gate(Node):

    def __init__(self):
        super().__init__('motion_gate')
        self.times = {}
        self.velocity = Twist()
        self.enabled = False
        self.covariance = False
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.create_subscription(Twist, '/cmd_vel_smoothed', self.command, 1)
        self.create_subscription(Bool, '/motion_enabled', self.permission, 1)
        self.create_subscription(Odometry, '/odom', lambda m: self.stamp('odom', m), qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', lambda m: self.stamp('scan', m), qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose, 10)
        self.create_timer(0.02, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds / 1000000000.0

    def stamp(self, key, msg):
        self.times[key] = msg.header.stamp.sec + msg.header.stamp.nanosec / 1000000000.0

    def command(self, msg):
        self.velocity = msg
        self.times['cmd'] = self.now()

    def permission(self, msg):
        self.enabled = msg.data
        self.times['enable'] = self.now()

    def pose(self, msg):
        self.stamp('pose', msg)
        c = msg.pose.covariance
        self.covariance = all((math.isfinite(c[i]) and 0 <= c[i] < 0.1 ** 2 for i in (0, 7))) and 0 <= c[35] < math.radians(20) ** 2

    def tick(self):
        limits = {'odom': 0.5, 'scan': 0.5, 'pose': 1.0, 'cmd': 0.3, 'enable': 0.3}
        now = self.now()
        valid = self.enabled and self.covariance and all((-0.1 <= now - self.times.get(k, -1000000000.0) < v for k, v in limits.items()))
        self.pub.publish(self.velocity if valid else Twist())
if __name__ == '__main__':
    rclpy.init()
    node = Gate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
