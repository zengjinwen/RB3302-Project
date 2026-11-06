"""Sequence goals using Nav2 actions. No steering, odometry integration or planner."""
import math
import json
import os
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from collections import deque
from rclpy.qos import qos_profile_sensor_data
from initial_alignment import InitialAlignment
from std_msgs.msg import Bool, String
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener, TransformException
DIRECTIONS = {'E': 0, 'NE': 45, 'N': 90, 'NW': 135, 'W': 180, 'SW': -135, 'S': -90, 'SE': -45}

class Mission(Node):

    def __init__(self):
        super().__init__('autonomous_mission')
        self.points = yaml.safe_load((Path(__file__).parent / 'course.yaml').read_text())['landmarks']
        self.events = self.create_publisher(String, '/mission_events', QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.start = os.environ['NAV_START']
        self.heading = math.radians(DIRECTIONS[os.environ['NAV_BEARING']])
        self.goals = os.environ['NAV_GOALS'].split(',')
        if len(self.goals) != 4 or any((k not in self.points for k in [self.start] + self.goals)):
            raise ValueError('Provide a valid start, eight-way bearing and exactly four goals')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.initial = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.enable = self.create_publisher(Bool, '/motion_enabled', 10)
        self.update = self.create_client(Empty, '/request_nomotion_update')
        self.update_future = None
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.localization, 10)
        self.create_subscription(Odometry, '/odom', self.odometry, 10)
        self.aligner = None
        self.scans = deque(maxlen=3)
        self.create_subscription(OccupancyGrid, '/map', self.map_received, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(LaserScan, '/scan', lambda msg: self.scans.append(msg), qos_profile_sensor_data)
        self.amcl = None
        self.odom = None
        self.index = 0
        self.state = 'initialize'
        self.since = 0.0
        self.started = None
        self.goal_handle = None
        self.create_timer(0.1, self.tick)

    def map_received(self, msg):
        if self.aligner is None:
            self.aligner = InitialAlignment(msg)

    def aligned_start(self):
        points = []
        for scan in self.scans:
            try:
                t = self.tf.lookup_transform('base_link', scan.header.frame_id, rclpy.time.Time()).transform
            except TransformException:
                return None
            q = t.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            stride = max(1, len(scan.ranges)//180)
            for i in range(0, len(scan.ranges), stride):
                d = scan.ranges[i]
                if math.isfinite(d) and scan.range_min < d < min(scan.range_max, 5.):
                    a = yaw + scan.angle_min + i*scan.angle_increment
                    points.append((t.translation.x+d*math.cos(a), t.translation.y+d*math.sin(a)))
        if len(points) < 30:
            return None
        pose, residual = self.aligner.align(points, self.points[self.start], self.heading)
        self.log(f'INITIAL ALIGNMENT x={pose[0]:.3f} y={pose[1]:.3f} yaw={math.degrees(pose[2]):.1f} residual={residual:.3f}')
        return pose

    def now(self):
        return self.get_clock().now().nanoseconds / 1000000000.0

    def localization(self, msg):
        self.amcl = msg

    def odometry(self, msg):
        self.odom = msg

    def log(self, text):
        self.get_logger().info(text)
        self.events.publish(String(data=json.dumps({'event': text, 'time': self.now()})))

    def change(self, state):
        self.state = state
        self.since = self.now()

    def position(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time()).transform.translation
            return (t.x, t.y)
        except TransformException:
            return None

    def send(self):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = self.points[self.goals[self.index]]
        goal.pose.pose.orientation.w = 1.0
        self.change('navigating')
        self.log('TARGET ' + self.goals[self.index])
        self.client.send_goal_async(goal).add_done_callback(self.accepted)

    def accepted(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.change('retry')
            return
        if self.state == 'failed':
            self.goal_handle.cancel_goal_async()
            return
        self.goal_handle.get_result_async().add_done_callback(self.finished)

    def finished(self, future):
        if self.state == 'failed':
            return
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.change('verify')
        else:
            self.log('REPLAN ' + result.result.error_msg)
            self.change('retry')

    def tick(self):
        now = self.now()
        self.enable.publish(Bool(data=self.state == 'navigating'))
        if self.state in ('done', 'failed'):
            return
        if self.started is not None and now - self.started > 180.0:
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self.change('failed')
            self.enable.publish(Bool(data=False))
            self.log('FAILED: 180 second mission deadline')
            return
        if self.update.service_is_ready() and (self.update_future is None or self.update_future.done()):
            self.update_future = self.update.call_async(Empty.Request())
        if self.state == 'initialize':
            if self.initial.get_subscription_count() == 0 or self.odom is None or self.aligner is None or len(self.scans) < 3:
                return
            initial = self.aligned_start()
            if initial is None:
                return
            x, y, heading = initial
            p = PoseWithCovarianceStamped()
            p.header.frame_id = 'map'
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose.pose.position.x, p.pose.pose.position.y = x, y
            p.pose.pose.orientation.z = math.sin(heading / 2)
            p.pose.pose.orientation.w = math.cos(heading / 2)
            p.pose.covariance[0] = p.pose.covariance[7] = 0.025 ** 2
            p.pose.covariance[35] = math.radians(2) ** 2
            self.initial.publish(p)
            self.started = now
            self.change('localize')
            self.log('MISSION START')
            return
        if self.amcl is None or self.odom is None:
            return
        stamp = self.amcl.header.stamp
        age = now - stamp.sec - stamp.nanosec / 1000000000.0
        if age > 1.0 or self.position() is None:
            return
        if self.state == 'localize':
            c = self.amcl.pose.covariance
            if now - self.since > 0.5 and max(c[0], c[7]) < 0.08 ** 2 and (c[35] < math.radians(15) ** 2) and self.client.server_is_ready():
                self.send()
        elif self.state == 'retry' and now - self.since > 0.5:
            self.send()
        elif self.state == 'verify':
            if now - self.since < 0.8 or stamp.sec + stamp.nanosec / 1000000000.0 < self.since:
                return
            v = self.odom.twist.twist
            if abs(v.linear.x) > 0.02 or abs(v.angular.z) > 0.04:
                return
            error = math.dist(self.position(), self.points[self.goals[self.index]])
            if error <= 0.05:
                self.log(f'GOAL {self.goals[self.index]} error={error:.3f}m')
                self.change('dwell')
            else:
                self.send()
        elif self.state == 'dwell' and now - self.since >= 3.0:
            self.index += 1
            if self.index == len(self.goals):
                self.change('done')
                self.log(f'MISSION COMPLETE elapsed={now - self.started:.2f}s')
            else:
                self.send()
if __name__ == '__main__':
    rclpy.init()
    node = Mission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.enable.publish(Bool(data=False))
        node.destroy_node()
        rclpy.try_shutdown()
