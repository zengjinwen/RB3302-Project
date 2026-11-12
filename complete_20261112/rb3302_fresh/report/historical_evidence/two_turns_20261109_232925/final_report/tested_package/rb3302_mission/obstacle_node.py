"""Successful in-place controller plus one confirmed obstacle cell and A*."""
import math
from rclpy.time import Time
from tf2_ros import TransformException
from .node import Mission, main as original_main
from .robot import yaw
from .grid import wrap
from .obstacle import CellObstacle


class Replan(Exception):
    pass


class ObstacleMission(Mission):
    def __init__(self):
        self.obstacles = None
        self.observed_cells = []
        self.route_cells = []
        self.active_goal = None
        self.segment_cells = []
        self.monitor_route = False
        super().__init__()

    def read_map(self, msg):
        super().read_map(msg)
        if self.obstacles is None:
            self.obstacles = CellObstacle(self.points,msg,self.p['obstacle_confirm_time'],
                                          self.p['obstacle_min_width'],self.p['obstacle_boundary_tolerance'])

    def read_scan(self, msg):
        super().read_scan(msg)
        if not self.initial_sent or self.obstacles is None or not self.ready():
            return
        if not 0. <= self.now()-self.stamp(msg)<1.:
            return
        x,y = self.pose()[:2]
        current = min(self.points,key=lambda cell:math.dist((x,y),self.points[cell]))
        cx,cy = self.points[current]
        cells = [cell for cell,(px,py) in self.points.items()
                 if cell!=self.obstacles.blocked and
                 ((math.isclose(px,cx,abs_tol=1e-6) and math.isclose(abs(py-cy),.58,abs_tol=1e-6))
                  or (math.isclose(py,cy,abs_tol=1e-6) and math.isclose(abs(px-cx),.58,abs_tol=1e-6)))]
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
                                               [cell for cell in self.points if cell!=current])
        if set(observed)!=set(self.observed_cells):
            self.get_logger().info('扫描障碍候选：'+(','.join(observed) or '无')+
                                   '；仅四邻格内稳定候选参与规划。')
        self.observed_cells = observed
        candidate = next((cell for cell in observed if cell in cells),None)
        if candidate is not None and self.active_goal is not None:
            if not self.planner.plan(current,self.active_goal,candidate):
                candidate = None  # User's course rule: one obstacle must leave a route.
        previous = self.obstacles.blocked
        if self.obstacles.observe(candidate,self.stamp(msg)):
            action = '挡住剩余路线，准备重新规划。' if self.blocks_route() else '原路线可走，继续执行。'
            self.get_logger().warning(f'确认整格障碍：{candidate}，替换旧障碍{previous}；{action}')

    def blocks_route(self):
        blocked = self.obstacles.blocked
        if blocked not in self.route_cells:
            return False
        if blocked in self.segment_cells:
            x,y = self.pose()[:2]
            bx,by = self.points[blocked]
            return (bx-x)*math.cos(self.route_heading)+(by-y)*math.sin(self.route_heading)>0.
        return True

    def command(self, linear=0., angular=0.):
        if linear>0 and self.monitor_route and self.blocks_route():
            super().command()
            raise Replan()
        super().command(linear,angular)

    def face_route(self, heading):
        turns = round(wrap(heading-self.route_heading)/(math.pi/2))
        if turns:
            self.turn_relative(turns*math.pi/2)
        self.route_heading = heading

    def replan_at_centre(self, segment, goal):
        self.wait_ready()
        sample = self.straight_sample()
        while sample is None:
            self.command()
            self.pause()
            sample = self.straight_sample()
        x,y = sample[:2]
        # Only use centres on this straight segment before the blocked cell.
        if self.obstacles.blocked in segment:
            segment = segment[:segment.index(self.obstacles.blocked)]
        ux,uy = math.cos(self.route_heading),math.sin(self.route_heading)
        def progress(cell):
            px,py = self.points[cell]
            return (px-x)*ux+(py-y)*uy
        # Finish at the next safe centre ahead, rather than the nearest behind.
        forward = [cell for cell in segment if progress(cell)>=0.]
        anchor = min(forward,key=progress) if forward else max(segment,key=progress)
        blocked = self.obstacles.blocked
        route = self.planner.plan(anchor,goal,blocked)
        if route:
            self.get_logger().info('已重新规划：'+' → '.join(route)+f'；先走到{anchor}中心，再执行绕行。')
        else:
            anchor = segment[-1]  # Last safe centre before the blocked cell.
            self.get_logger().warning(f'暂时无路：先到障碍前{anchor}格中心，再停车重算。')
        target = self.points[anchor]
        along = (target[0]-x)*ux+(target[1]-y)*uy
        if abs(along)>1e-6:
            heading = self.route_heading if along>0 else wrap(self.route_heading+math.pi)
            self.face_route(heading)
            start = (target[0]-.58*math.cos(heading),target[1]-.58*math.sin(heading))
            self.drive_line(start,target,end_speed=self.p['drive_speed'])
            self.halt(.3)
        self.get_logger().info(f'重新规划起点：{anchor}；唯一障碍格：{self.obstacles.blocked}。')
        return anchor,route,blocked

    def run(self):
        self.initialize()
        current,self.route_heading = self.start,self.start_yaw
        for number,goal in enumerate(self.goals,1):
            self.active_goal = goal
            pending = None
            while True:
                if pending is not None and pending[1]==self.obstacles.blocked:
                    route = pending[0]
                else:
                    route = self.planner.plan(current,goal,self.obstacles.blocked)
                pending = None
                if not route:
                    self.get_logger().warning(f'在{current}中心等待：封锁{self.obstacles.blocked}后暂时无路，每秒重算。')
                    while not route:
                        self.command()
                        self.pause(1.)
                        route = self.planner.plan(current,goal,self.obstacles.blocked)
                self.get_logger().info('58cm路线：'+' → '.join(route))
                corners = self.planner.turning_points(route)
                interrupted = False
                for source,destination in zip(corners,corners[1:]):
                    start,end = self.points[source],self.points[destination]
                    begin,finish = route.index(source),route.index(destination)
                    segment = route[begin:finish+1]
                    self.segment_cells = segment
                    self.route_cells = route[begin+1:]
                    heading = math.atan2(end[1]-start[1],end[0]-start[0])
                    self.face_route(heading)
                    self.monitor_route = True
                    try:
                        end_speed = self.p['approach_speed'] if destination==goal else self.p['drive_speed']
                        target = end
                        self.get_logger().info(f'走向{destination}：'+('goal中心停车。' if destination==goal else '到格子中心停车，再转弯。'))
                        self.drive_line(start,target,end_speed=end_speed)
                    except Replan:
                        self.monitor_route = False
                        current,planned,blocked = self.replan_at_centre(segment,goal)
                        pending = (planned,blocked)
                        interrupted = True
                        break
                    finally:
                        self.monitor_route = False
                    self.halt(.3)
                    current = destination
                if not interrupted:
                    break
            self.route_cells = []
            self.get_logger().info('GOAL '+goal)
            self.halt(3.)
            self.get_logger().info(f'目标 {number}/4：{goal}，已停三秒。')
            current = goal
        self.get_logger().info('MISSION COMPLETE')


def main(args=None):
    original_main(args,mission_class=ObstacleMission)
