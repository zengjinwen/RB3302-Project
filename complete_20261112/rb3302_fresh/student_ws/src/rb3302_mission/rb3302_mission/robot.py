"""AMCL turns, TF straight-line pose, fused IMU rate and ROS I/O."""
import math
import time
from pathlib import Path
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from .grid import MapPlanner


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class Robot(Node):
    def __init__(self):
        super().__init__('rb3302_mission')
        folder = Path(get_package_share_directory('rb3302_mission'))
        defaults = yaml.safe_load((folder/'config/control.yaml').read_text())['rb3302_mission']['ros__parameters']
        defaults.update(start='', bearing='', goals='', course=str(folder/'config/course.yaml'))
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key:self.get_parameter(key).value for key in defaults}
        self.points = yaml.safe_load(Path(self.p['course']).read_text())['landmarks']
        self.start = self.p['start'].upper().removeprefix('CELL_')
        bearing = self.p['bearing'].upper().removeprefix('DIR_')
        self.goals = self.p['goals'].upper().split(',')
        compass = dict(E=0, NE=45, N=90, NW=135, W=180, SW=-135, S=-90, SE=-45)
        if self.start not in self.points or bearing not in compass or len(self.goals)!=4 or any(g not in self.points for g in self.goals):
            raise ValueError('填写起点A-X、八方向朝向、四个逗号分隔的目标。')
        self.start_yaw = math.radians(compass[bearing])
        self.planner = self.amcl = None
        self.initial_sent = False
        self.odom = None
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 1)
        self.initial = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.update = self.create_client(Empty, '/request_nomotion_update')
        self.update_future = None
        self.command_stopped = True
        self.create_subscription(OccupancyGrid, '/map', self.read_map, QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m:setattr(self,'amcl',m), 1)
        self.create_subscription(LaserScan, '/scan', self.read_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', lambda m:setattr(self,'odom',m), qos_profile_sensor_data)

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec+msg.header.stamp.nanosec/1e9

    def read_map(self, msg):
        if self.planner is None:
            self.planner = MapPlanner(msg,self.points,self.p['stop_distance'])

    def read_scan(self, msg):
        # Moving scans update normally (AMCL movement thresholds are zero).
        # Request stationary updates only while commanding a stop.
        if self.command_stopped and self.initial_sent and self.update.service_is_ready() and (self.update_future is None or self.update_future.done()):
            self.update_future = self.update.call_async(Empty.Request())

    def pose(self):
        p = self.amcl.pose.pose
        return p.position.x,p.position.y,yaw(p.orientation)

    def straight_sample(self):
        # EKF already rotates the IMU into base_link; never use commanded yaw rate.
        if not self.ready() or self.odom is None:
            return None
        if self.odom.child_frame_id != 'base_link' or not -.1 <= self.now()-self.stamp(self.odom) < 1.:
            return None
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except TransformException:
            return None
        if not -.1 <= self.now()-self.stamp(transform) < 1.:
            return None
        p = transform.transform
        values = (p.translation.x, p.translation.y, yaw(p.rotation), self.odom.twist.twist.angular.z)
        return values if all(map(math.isfinite, values)) else None

    def ready(self):
        if self.amcl is None or not -.1 <= self.now()-self.stamp(self.amcl) < 1.:
            return False
        p = self.amcl.pose.pose
        return self.amcl.header.frame_id=='map' and all(map(math.isfinite,self.pose())) and .5 < sum(v*v for v in (p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)) < 1.5

    def wait_ready(self):
        while not self.ready():
            self.command()
            self.pause(.05)

    def pause(self, seconds=.02):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(self,timeout_sec=min(.02,max(0.,end-time.monotonic())))

    def command(self, linear=0., angular=0.):
        msg = Twist()
        msg.linear.x,msg.angular.z = float(linear),float(angular)
        self.command_stopped = linear == 0. and angular == 0.
        self.cmd.publish(msg)

    def halt(self, seconds=.3):
        end = self.now()+seconds
        while self.now() < end:
            self.command()
            self.pause()

