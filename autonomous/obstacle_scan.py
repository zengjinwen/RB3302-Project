"""Separate new obstacles from known walls, without changing AMCL's raw scan."""
import copy
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener, TransformException

def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

class ObstacleScan(Node):

    def __init__(self):
        super().__init__('obstacle_scan')
        self.declare_parameter('wall_match_distance', 0.1)
        self.margin = self.get_parameter('wall_match_distance').value
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.info = None
        self.walls = set()
        self.pub = self.create_publisher(LaserScan, '/obstacle_scan', qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self.map_received, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(LaserScan, '/scan', self.scan_received, qos_profile_sensor_data)

    def map_received(self, msg):
        self.info = msg.info
        self.walls.clear()
        r = msg.info.resolution
        cells = math.ceil(self.margin / r)
        offsets = [(x, y) for x in range(-cells, cells + 1) for y in range(-cells, cells + 1) if math.hypot(x, y) * r <= self.margin + r / 2]
        for i, value in enumerate(msg.data):
            if value >= 50:
                x, y = (i % msg.info.width, i // msg.info.width)
                self.walls.update(((x + dx, y + dy) for dx, dy in offsets))

    def scan_received(self, msg):
        if self.info is None:
            return
        try:
            t = self.tf.lookup_transform('map', msg.header.frame_id, rclpy.time.Time.from_msg(msg.header.stamp)).transform
        except TransformException:
            return
        filtered = copy.deepcopy(msg)
        angles = yaw(t.rotation)
        origin = self.info.origin
        a = yaw(origin.orientation)
        c, s = (math.cos(a), math.sin(a))
        r = self.info.resolution
        for i, d in enumerate(msg.ranges):
            if not math.isfinite(d):
                continue
            angle = angles + msg.angle_min + i * msg.angle_increment
            dx = t.translation.x + d * math.cos(angle) - origin.position.x
            dy = t.translation.y + d * math.sin(angle) - origin.position.y
            pixel = (math.floor((c * dx + s * dy) / r), math.floor((-s * dx + c * dy) / r))
            if pixel in self.walls:
                filtered.ranges[i] = float('inf')
        self.pub.publish(filtered)
if __name__ == '__main__':
    rclpy.init()
    node = ObstacleScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
