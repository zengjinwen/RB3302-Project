"""Strict static-wall exclusion for the two experiments; baseline is unchanged."""
import math
from rclpy.time import Time
from tf2_ros import TransformException
from .node import Mission
from .robot import yaw
from .obstacle import CellObstacle
from .obstacle_node import ObstacleMission


class NewSurfaceObstacle(CellObstacle):
    def detect_cells(self, hits, laser, cells):
        # Width, line fit and stability may only use genuinely new returns.
        new_hits = [hit for hit in hits if self.static_free(*hit)]
        return super().detect_cells(new_hits,laser,cells)


class ExperimentalObstacleMission(ObstacleMission):
    def read_map(self, msg):
        Mission.read_map(self,msg)
        if self.obstacles is None:
            self.obstacles = NewSurfaceObstacle(self.points,msg,
                self.p['obstacle_confirm_time'],self.p['obstacle_min_width'],
                self.p['obstacle_boundary_tolerance'])

    def read_scan(self, msg):
        Mission.read_scan(self, msg)
        if not self.initial_sent or self.obstacles is None or not self.ready():
            return
        if not 0. <= self.now()-self.stamp(msg)<1.:
            return
        x,y = self.pose()[:2]
        current = min(self.points,key=lambda cell:math.dist((x,y),self.points[cell]))
        # A wall already separating two centres cannot become a new obstacle.
        cells = self.planner.neighbors[current]
        try:
            tf = self.tf_buffer.lookup_transform('map',msg.header.frame_id,Time.from_msg(msg.header.stamp))
        except TransformException:
            return  # Never project an old scan using a new robot pose.
        p = tf.transform.translation
        heading = yaw(tf.transform.rotation)
        hits = []
        for i,distance in enumerate(msg.ranges):
            if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                angle = heading+msg.angle_min+i*msg.angle_increment
                hits.append((p.x+distance*math.cos(angle),p.y+distance*math.sin(angle)))
        observed = self.obstacles.detect_cells(hits,(p.x,p.y),
                                               cells)
        if set(observed)!=set(self.observed_cells):
            self.get_logger().info('扫描障碍候选：'+(','.join(observed) or '无')+
                                   '；仅四邻格内稳定候选参与规划。')
        self.observed_cells = observed
        # A still-visible confirmed obstacle stays a valid candidate. Do not
        # discard it merely because it is already blocked, then pick a rival.
        blocked = self.obstacles.blocked
        candidate = blocked if blocked in observed else next(iter(observed),None)
        if candidate is not None and self.active_goal is not None:
            if not self.planner.plan(current,self.active_goal,candidate):
                candidate = None  # User's course rule: one obstacle must leave a route.
        previous = self.obstacles.blocked
        if self.obstacles.observe(candidate,self.stamp(msg)):
            action = '挡住剩余路线，准备重新规划。' if self.blocks_route() else '原路线可走，继续执行。'
            self.get_logger().warning(f'确认整格障碍：{candidate}，替换旧障碍{previous}；{action}')

