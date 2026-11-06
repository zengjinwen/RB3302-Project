"""Rolling 90-degree corners; preserve the original mission as a selectable version."""
import math
from .node import Mission, main as original_main
from .grid import wrap
from .arc import corner_arc, arc_points, arc_control


class ArcMission(Mission):
    def drive_arc(self, arc):
        radius,speed = self.p['arc_radius'],self.p['arc_speed']
        next_log = self.now()
        self.get_logger().info(f'弧线转弯：半径{radius:.2f}m，前进{speed:.2f}m/s，左右差速通过90度弯。')
        while True:
            self.pause()
            sample = self.straight_sample()
            if sample is None:
                self.command()
                continue
            linear,omega,done,error = arc_control(arc,radius,speed,sample[:3],
                                          self.p['arc_heading_gain'],self.p['arc_cross_gain'],
                                          self.p['arc_turn_limit'],
                                          math.radians(self.p['arc_exit_heading_degrees']),
                                          self.p['turn_min_speed'])
            if done:
                self.get_logger().info('弧线完成：位置到出口且朝向已对准下一段。')
                self.command()
                return
            self.command(linear,omega)
            if self.now() >= next_log:
                self.get_logger().info(f'弧线跟踪：径向误差{error:+.3f}m，出弯朝向误差{math.degrees(wrap(arc[5]-sample[2])):+.1f}度，前进{linear:.3f}m/s，角速度{omega:+.3f}rad/s。')
                next_log = self.now()+1.

    def run(self):
        self.initialize()
        current,previous_heading = self.start,self.start_yaw
        for number,goal in enumerate(self.goals,1):
            route = self.planner.plan(current,goal)
            if not route:
                raise RuntimeError('静态地图没有通路。')
            self.get_logger().info('58cm路线：'+' → '.join(route))
            route = self.planner.turning_points(route)
            for i in range(len(route)-1):
                start,end = self.points[route[i]],self.points[route[i+1]]
                heading = math.atan2(end[1]-start[1],end[0]-start[0])
                turns = round(wrap(heading-previous_heading)/(math.pi/2))
                if turns:
                    self.get_logger().info('起步换向或掉头：保留原地转弯。')
                    self.turn_relative(turns*math.pi/2)
                arc = None
                if i+2 < len(route):
                    arc = corner_arc(start,end,self.points[route[i+2]],self.p['arc_radius'])
                    if arc is not None:
                        samples = arc_points(arc,self.p['arc_radius'])
                        if not all(self.planner.clear_line(a,b) for a,b in zip(samples,samples[1:])):
                            self.get_logger().warning('该弯地图净空不足，保留原地转弯。')
                            arc = None
                if arc is not None:
                    self.drive_line(start,arc[0],stop=False,end_speed=self.p['drive_speed'])
                    self.drive_arc(arc)
                    previous_heading = arc[5]
                else:
                    end_speed = self.p['approach_speed'] if i+1 == len(route)-1 else self.p['drive_speed']
                    self.drive_line(start,end,end_speed=end_speed)
                    self.halt(.3)
                    previous_heading = heading
            self.get_logger().info('GOAL '+goal)
            self.halt(3.)
            self.get_logger().info(f'目标 {number}/4：{goal}，已停三秒。')
            current = goal
        self.get_logger().info('MISSION COMPLETE')


def main(args=None):
    original_main(args,mission_class=ArcMission)
