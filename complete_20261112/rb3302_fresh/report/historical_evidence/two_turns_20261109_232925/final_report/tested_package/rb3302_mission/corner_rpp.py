"""RPP geometry only around grid corners, with measured turn-rate trim.

This is a local curvature-regulated pure-pursuit implementation, NOT the
Nav2 controller plugin. No Nav2 costmap, recovery or goal policy is imported.
"""
import math
from .turn_feedback import FeedbackMission, RateTrim
from .grid import wrap
from .corner_geometry import turn_windows, lengths, project, point_at, pursuit
from .obstacle_node import Replan
from .node import main as mission_main


class CornerMission(FeedbackMission):
    def __init__(self):
        self.in_curve = False
        super().__init__()

    def blocks_route(self):
        if self.in_curve:
            return self.obstacles.blocked in self.route_cells
        return super().blocks_route()

    def follow_corner(self, cells, remaining_route, goal):
        points = [self.points[n] for n in cells]
        distances = lengths(points)
        final_heading = math.atan2(points[-1][1]-points[-2][1],points[-1][0]-points[-2][0])
        ux,uy = math.cos(final_heading),math.sin(final_heading)
        # Use this course's scale, not another robot's controller settings.
        cell_size = min(math.dist(a,b) for a,b in zip(points,points[1:]))
        body_length = .20
        lookahead = min(body_length,cell_size/2.)
        # Design acceleration: nominal speed around a half-cell radius.
        lateral_accel = self.p['drive_speed']**2/(cell_size/2.)
        speed = self.p['drive_speed']
        last_log = 0.
        self.in_curve = self.rate_active = True
        self.rate_trim = RateTrim()
        self.rate_time = None
        try:
            while True:
                self.pause()
                sample = self.straight_sample()
                if sample is None:
                    self.command()
                    continue
                x,y,heading,_ = sample
                edge,progress = project((x,y),points,distances)
                # Existing replan code must see the CURRENT straight grid edge,
                # never a diagonal shortcut across a corner.
                a,b = points[edge:edge+2]
                self.route_heading = math.atan2(b[1]-a[1],b[0]-a[0])
                self.segment_cells = cells[edge:edge+2]
                self.route_cells = cells[edge+1:]+remaining_route
                remaining = (points[-1][0]-x)*ux+(points[-1][1]-y)*uy
                if edge == len(points)-2 and remaining <= 0.:
                    self.route_heading = final_heading
                    if cells[-1] == goal:
                        self.command()
                    return
                path_left = distances[-1]-progress
                desired = self.p['drive_speed']
                if cells[-1] == goal:
                    fraction = min(1.,path_left/self.p['slowdown_distance'])
                    desired = self.p['approach_speed']+(desired-self.p['approach_speed'])*fraction
                target = point_at(points,distances,progress+lookahead)
                heading_error = wrap(math.atan2(target[1]-y,target[0]-x)-heading)
                # Align first if the target is more sideways than forward.
                if abs(heading_error) > math.pi/4.:
                    angular = self.p['turn_speed']*min(1.,abs(heading_error)/(math.pi/2.))
                    self.command(0.,math.copysign(angular,heading_error))
                    if self.now() >= last_log:
                        self.get_logger().info(f'CORNER RPP align heading={math.degrees(heading_error):+.1f}deg')
                        last_log = self.now()+1.
                    continue
                curvature = pursuit((x,y),heading,target)
                # v^2*|curvature| <= chosen lateral acceleration,
                # v*|curvature| <= our existing requested turn-speed limit.
                speed = desired
                if abs(curvature) > 1e-9:
                    speed = min(speed,math.sqrt(lateral_accel/abs(curvature)),
                                self.p['turn_speed']/abs(curvature))
                self.command(speed,speed*curvature)
                if self.now() >= last_log:
                    self.get_logger().info(f'CORNER RPP {cells[0]}->{cells[-1]} remaining={path_left:.3f} v={speed:.3f} w={speed*curvature:+.3f}')
                    last_log = self.now()+1.
        finally:
            self.in_curve = self.rate_active = False
            self.rate_trim = RateTrim()

    def run(self):
        self.initialize()
        current,self.route_heading = self.start,self.start_yaw
        for number,goal in enumerate(self.goals,1):
            self.active_goal = goal
            pending = None
            while True:
                route = pending[0] if pending is not None and pending[1]==self.obstacles.blocked else self.planner.plan(current,goal,self.obstacles.blocked)
                pending = None
                while not route:
                    self.command()
                    self.pause(1.)
                    route = self.planner.plan(current,goal,self.obstacles.blocked)
                self.get_logger().info('58cm路线：'+' → '.join(route))
                interrupted = False
                for kind,begin,finish in turn_windows(route,self.points):
                    cells = route[begin:finish+1]
                    start,end = self.points[cells[0]],self.points[cells[-1]]
                    first = self.points[cells[1]]
                    heading = math.atan2(first[1]-start[1],first[0]-start[0])
                    self.segment_cells = cells
                    self.route_cells = route[begin+1:]
                    # Only mission start/replan/goal departure can need a pivot.
                    if kind == 'line':
                        self.face_route(heading)
                    else:
                        self.route_heading = heading
                    self.monitor_route = True
                    try:
                        if kind == 'rpp':
                            self.follow_corner(cells,route[finish+1:],goal)
                        else:
                            final = cells[-1]==goal
                            self.drive_line(start,end,stop=final,
                                end_speed=self.p['approach_speed'] if final else self.p['drive_speed'])
                    except Replan:
                        self.monitor_route = False
                        current,planned,blocked = self.replan_at_centre(self.segment_cells,goal)
                        pending = (planned,blocked)
                        interrupted = True
                        break
                    finally:
                        self.monitor_route = False
                    current = cells[-1]
                if not interrupted:
                    break
            self.route_cells = []
            self.get_logger().info('GOAL '+goal)
            self.halt(3.)
            self.get_logger().info(f'目标 {number}/4：{goal}，已停三秒。')
            current = goal
        self.get_logger().info('MISSION COMPLETE')


def main(args=None):
    mission_main(args,mission_class=CornerMission)
