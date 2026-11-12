"""RViz-only display of the static inflated grid used by the original A*."""
import math
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray


def inflated_data(msg, clearance):
    resolution = msg.info.resolution
    width, height = msg.info.width, msg.info.height
    n = math.ceil(clearance/resolution)
    offsets = [(x,y) for x in range(-n,n+1) for y in range(-n,n+1)
               if math.hypot(x,y)*resolution <= clearance+1e-6]
    data = [0]*(width*height)
    for i,value in enumerate(msg.data):
        if value < 0 or value >= 50:
            for dx,dy in offsets:
                x,y = i%width+dx,i//width+dy
                if 0 <= x < width and 0 <= y < height:
                    data[y*width+x] = 100
    return data


class CostmapView(Node):
    def __init__(self):
        super().__init__('rb3302_costmap_view')
        config = Path(__file__).resolve().parents[1]/'config/control.yaml'
        self.clearance = yaml.safe_load(config.read_text())['rb3302_mission']['ros__parameters']['stop_distance']
        qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.labels = self.create_publisher(MarkerArray,'/grid_labels',qos)
        self.points = yaml.safe_load((config.parent/'course.yaml').read_text())['landmarks']
        self.publisher = self.create_publisher(OccupancyGrid,'/planning_costmap',qos)
        self.subscription = self.create_subscription(OccupancyGrid,'/map',self.show,qos)

    def show(self, msg):
        labels = MarkerArray()
        for index,(name,(x,y)) in enumerate(self.points.items()):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.ns,marker.id = 'grid_letters',index
            marker.type,marker.action = Marker.TEXT_VIEW_FACING,Marker.ADD
            marker.pose.position.x,marker.pose.position.y = float(x),float(y)
            marker.pose.position.z = .08
            marker.pose.orientation.w = 1.
            marker.scale.z = .14
            marker.color.r,marker.color.g,marker.color.b,marker.color.a = .05,.15,.85,1.
            marker.text = name
            labels.markers.append(marker)
        self.labels.publish(labels)
        output = OccupancyGrid()
        output.header,output.info = msg.header,msg.info
        output.data = inflated_data(msg,self.clearance)
        self.publisher.publish(output)
        self.get_logger().info(f'A*静态膨胀地图已显示，膨胀距离{self.clearance:.2f}m；仅显示，不参与控制。')


def main():
    rclpy.init()
    node = CostmapView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
