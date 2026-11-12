"""RPP at grid corners and goal cells, with measured turn-rate trim.

This is a local curvature-regulated pure-pursuit implementation, NOT the
Nav2 controller plugin. No Nav2 costmap, recovery or goal policy is imported.
"""
import math
from .turn_feedback import FeedbackMission, RateTrim
from .grid import wrap
from .corner_geometry import turn_windows, lengths, project, point_at, pursuit, smooth_corners
from .obstacle_node import Replan
from .node import main as mission_main


class CornerMission(FeedbackMission):
    goal_turn_speed = .8
    motion_scale = 1.2

    def command(self, linear=0., angular=0.):
        # Scale both components together to preserve the requested curvature.
        super().command(linear*self.motion_scale, angular*self.motion_scale)

    def initialize(self):
        super().initialize()
        started = self.now()
        last_stamp = self.stamp(self.amcl)
        stable = 0
        next_log = 0.
        while True:
            self.command()
            self.pause()
            now = self.now()
            if self.straight_sample() is None:
                stable = 0
                if now >= next_log:
                    self.get_logger().info('START LOCALIZATION: 静止等待新的有效定位')
                    next_log = now+1.
                continue
            stamp = self.stamp(self.amcl)
            if stamp <= last_stamp:
                continue
            last_stamp = stamp
            c = self.amcl.pose.covariance
            # Largest XY covariance eigenvalue handles correlated uncertainty.
            xx,yy,xy = c[0],c[7],(c[1]+c[6])/2.
            values = (xx,yy,xy,c[35])
            valid = all(math.isfinite(v) for v in values) and min(xx,yy,c[35]) >= 0.
            position = math.sqrt(max(0.,(xx+yy+math.hypot(xx-yy,2.*xy))/2.)) if valid else math.inf
            angle = math.sqrt(c[35]) if valid else math.inf
            stable = stable+1 if position <= .05 and angle <= math.radians(5.) else 0
            if now >= next_log:
                self.get_logger().info(f'START LOCALIZATION: 位置标准差={position*100:.1f}cm 朝向标准差={math.degrees(angle):.1f}deg 连续合格={stable}/5')
                next_log = now+1.
            if now-started >= 3. and stable >= 5:
                self.get_logger().info('START LOCALIZATION READY: AMCL收敛检查通过，先对准首段路线再出发（运动速度1.2倍）')
                return

    def face_route(self, heading):
        # Every departure uses measured yaw, not a nominal relative quarter turn.
        self.face_heading(heading)

    def drive_line(self, start, end, stop=True, end_speed=None, arrival_distance=0.):
        # Use the same measured-rate trim as RPP and pivots.
        self.rate_trim = RateTrim()
        self.rate_time = None
        self.rate_active = True
        try:
            return super().drive_line(start,end,stop,end_speed,arrival_distance)
        finally:
            self.rate_active = False
            self.rate_trim = RateTrim()

    def settle_goal(self, goal):
        # Hold only: localization keeps updating; no position correction.
        self.halt(4.)
        sample = self.straight_sample()
        if sample is not None:
            error = math.dist(sample[:2],self.points[goal])
            self.get_logger().info(f'GOAL HOLD {goal}: 已停4秒，二维误差={error:.4f}m；不纠正')

    def report_arc(self, centre, radius, clear):
        cell = min(self.points,key=lambda n:math.dist(self.points[n],centre))
        if clear:
            self.get_logger().info(f'CORNER ARC {cell}: 四分之一圆，半径={radius*100:.1f}cm')
        else:
            self.get_logger().warning(f'CORNER ARC {cell}: 半径{radius*100:.1f}cm圆弧未通过原地图墙体间距检查，保留直角路径')

    def follow_corner(self, cells, remaining_route, goal):
        self.goal_changed_heading = False  # RPP handles actual departure yaw itself.
        centres = [self.points[n] for n in cells]
        grid_distances = lengths(centres)
        points,rounded = smooth_corners(centres,self.planner.clear_line,report=self.report_arc)
        distances = lengths(points)
        self.get_logger().info(f'CORNER PATH {cells[0]}->{cells[-1]}: {rounded} smooth corners, {len(points)} samples')
        final_heading = math.atan2(points[-1][1]-points[-2][1],points[-1][0]-points[-2][0])
        ux,uy = math.cos(final_heading),math.sin(final_heading)
        # Use this course's scale, not another robot's controller settings.
        cell_size = min(math.dist(a,b) for a,b in zip(centres,centres[1:]))
        lookahead = min(.20,cell_size/2.)
        # Design acceleration: nominal speed around a half-cell radius.
        lateral_accel = self.p['drive_speed']**2/(cell_size/2.)
        speed = self.p['drive_speed']
        # A rounded corner is passed only after its outgoing tangent section,
        # not when nearest-point projection jumps to the next grid edge.
        passed_at = [0.]
        for i in range(1,len(centres)-1):
            a,b,c = centres[i-1:i+2]
            incoming=(b[0]-a[0],b[1]-a[1])
            outgoing=(c[0]-b[0],c[1]-b[1])
            if abs(incoming[0]*outgoing[1]-incoming[1]*outgoing[0])>1e-9:
                reach=min(.30,math.dist(a,b)/2.,math.dist(b,c)/2.)
                mark=(b[0]+reach*outgoing[0]/math.dist(b,c),b[1]+reach*outgoing[1]/math.dist(b,c))
            else:
                mark=b
            passed_at.append(project(mark,points,distances)[1])
        passed_at.append(distances[-1])
        anchor=0
        last_log = 0.
        self.rate_active = True
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
                _,progress = project((x,y),points,distances)
                edge,_ = project((x,y),centres,grid_distances)
                # Existing replan code must see the CURRENT straight grid edge,
                # never a diagonal shortcut across a corner.
                while anchor < len(cells)-2 and progress >= passed_at[anchor+1]:
                    anchor+=1
                a,b = centres[edge:edge+2]
                self.route_heading = math.atan2(b[1]-a[1],b[0]-a[0])
                self.segment_cells = cells[edge:edge+2]
                self.route_cells = cells[anchor+1:]+remaining_route
                remaining = (points[-1][0]-x)*ux+(points[-1][1]-y)*uy
                if edge == len(centres)-2 and remaining <= 0.:
                    self.route_heading = final_heading
                    if cells[-1] == goal:
                        self.command()
                    return
                path_left = distances[-1]-progress
                desired = self.p['drive_speed']
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
                    self.get_logger().info(f'CORNER RPP {cells[0]}->{cells[-1]} remaining={path_left:.3f} heading={math.degrees(heading_error):+.1f}deg v={speed:.3f} w={speed*curvature:+.3f}')
                    last_log = self.now()+1.
        finally:
            self.rate_active = False
            self.rate_trim = RateTrim()

    def run(self):
        self.initialize()
        current,self.route_heading = self.start,self.start_yaw
        for number,goal in enumerate(self.goals,1):
            self.active_goal = goal
            pending = None
            departure = True
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
                    if departure:
                        self.face_route(heading)
                    else:
                        self.route_heading = heading
                    self.monitor_route = True
                    try:
                        if kind == 'rpp':
                            self.follow_corner(cells,route[finish+1:],goal)
                        else:
                            final = cells[-1]==goal
                            if final:
                                # Switch at the goal-cell boundary without stopping.
                                # Keep the original final centreline as RPP reference.
                                previous = self.points[cells[-2]]
                                entry = tuple((a+b)/2. for a,b in zip(previous,end))
                                self.drive_line(start,entry,stop=False,
                                    end_speed=self.p['drive_speed'])
                                self.get_logger().info(f'GOAL RPP {goal}: 进入目标格，沿中心线跟踪至终点')
                                self.follow_corner(cells[-2:],[],goal)
                            else:
                                self.drive_line(start,end,stop=False,
                                    end_speed=self.p['drive_speed'])
                    except Replan:
                        self.monitor_route = False
                        current,planned,blocked = self.replan_at_centre(self.segment_cells,goal)
                        pending = (planned,blocked)
                        departure = True
                        interrupted = True
                        break
                    finally:
                        self.monitor_route = False
                    current = cells[-1]
                    departure = False
                if not interrupted:
                    break
            self.route_cells = []
            self.settle_goal(goal)
            self.get_logger().info('GOAL '+goal)
            self.get_logger().info(f'目标 {number}/4：{goal}，已停四秒。')
            current = goal
        self.get_logger().info('MISSION COMPLETE')


def main(args=None):
    mission_main(args,mission_class=CornerMission)
