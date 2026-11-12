"""大格子A* → 匀速中心线 → 转弯格子中心停车 → 90度转弯 → 到点停3秒。"""
import math
import signal
import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseWithCovarianceStamped
from .robot import Robot
from .grid import clamp,wrap


class Mission(Robot):
    transition_delay = .3

    def align_goal(self, goal):
        pass

    def initialize(self):
        while self.planner is None or self.initial.get_subscription_count()==0:
            self.command()
            self.pause(.05)
        for topic in ('/cmd_vel','/amcl_pose','/odom'):
            if len(self.get_publishers_info_by_topic(topic))>1:
                raise RuntimeError(topic+'有多个发布者，请关闭另一套导航。')
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x,msg.pose.pose.position.y = self.points[self.start]
        msg.pose.pose.orientation.z,msg.pose.pose.orientation.w = math.sin(self.start_yaw/2),math.cos(self.start_yaw/2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = .2**2/3
        msg.pose.covariance[35] = (math.pi/4)**2/3
        self.initial.publish(msg)
        self.initial_sent = True
        self.get_logger().info('已发送起点和朝向，等待AMCL。')
        self.wait_ready()

    def turn_relative(self, angle):
        early_stop = math.radians(self.p['turn_early_stop_degrees'])
        if abs(angle)<=early_stop:
            return
        self.wait_ready()
        previous_yaw = self.pose()[2]
        target = wrap(previous_yaw+angle)
        direction = math.copysign(1.,angle)
        turned,speed = 0.,0.
        previous = self.now()
        self.get_logger().info(f'AMCL相对转弯：{math.degrees(angle):+.1f}度，目标朝向{math.degrees(target):+.1f}度，提前{math.degrees(early_stop):.1f}度发停车。')
        try:
            while True:
                self.pause()
                if not self.ready():
                    self.wait_ready()
                    speed,previous = 0.,self.now()
                now = self.now()
                dt,previous = max(.001,now-previous),now
                yaw = self.pose()[2]
                # Unwrap AMCL heading across +/-pi; no IMU/odom angle integration.
                turned += wrap(yaw-previous_yaw)
                previous_yaw = yaw
                remaining = direction*(angle-turned)-early_stop
                if remaining<=0.:
                    break
                wanted = min(self.p['turn_speed'],2.*remaining)
                speed += clamp(wanted-speed,.8*dt)
                self.command(0.,direction*speed)
        finally:
            self.command()
        stop_stamp = self.stamp(self.amcl)
        self.get_logger().info(f'AMCL转弯结束：已转{math.degrees(turned):+.1f}度，过冲{math.degrees(direction*turned-abs(angle)):+.1f}度，已发停车。')
        self.halt(self.transition_delay)
        if self.transition_delay and self.ready() and self.stamp(self.amcl)>stop_stamp:
            after = turned+wrap(self.pose()[2]-previous_yaw)
            self.get_logger().info(f'转弯停车后0.3秒AMCL：已转{math.degrees(after):+.1f}度，过冲{math.degrees(direction*after-abs(angle)):+.1f}度。')

    def drive_line(self, start, end, stop=True, end_speed=None, arrival_distance=0.):
        heading = math.atan2(end[1]-start[1],end[0]-start[0])
        ux,uy = math.cos(heading),math.sin(heading)
        speed = self.p['drive_speed']
        if end_speed is None:
            end_speed = self.p['approach_speed']
        previous = self.now()
        yaw_rate = integral = 0.
        next_log = self.now()
        while True:
            self.pause()
            sample = self.straight_sample()
            if sample is None:
                self.command()
                previous = self.now()
                yaw_rate = integral = 0.
                continue
            x,y,a,measured_rate = sample
            now = self.now()
            dt,previous = max(.001,now-previous),now
            remaining = (end[0]-x)*ux+(end[1]-y)*uy
            if remaining<=arrival_distance:
                if stop:
                    self.command()
                    ax,ay,aa = self.pose()
                    cross = -uy*(x-start[0])+ux*(y-start[1])
                    amcl_remaining = (end[0]-ax)*ux+(end[1]-ay)*uy
                    amcl_cross = -uy*(ax-start[0])+ux*(ay-start[1])
                    self.get_logger().info(
                        f'停车诊断：目标=({end[0]:.3f},{end[1]:.3f})m，'
                        f'路线朝向={math.degrees(heading):+.1f}度；'
                        f'TF控制位置=({x:.3f},{y:.3f})m，朝向={math.degrees(a):+.1f}度，'
                        f'纵向剩余={remaining:+.3f}m，横向={cross:+.3f}m，'
                        f'距目标={math.hypot(end[0]-x,end[1]-y):.3f}m；'
                        f'AMCL最新位置=({ax:.3f},{ay:.3f})m，朝向={math.degrees(aa):+.1f}度，'
                        f'纵向剩余={amcl_remaining:+.3f}m，横向={amcl_cross:+.3f}m，'
                        f'消息年龄={self.now()-self.stamp(self.amcl):.3f}s。'
                        '正剩余=尚未走到；负剩余=已越过；正横向=路线左侧。'
                        'TF与AMCL为各自最新值，并非同一采样时刻。')
                return
            cross = -uy*(x-start[0])+ux*(y-start[1])
            heading_error = wrap(heading-a)
            front_cross = cross-self.p['stanley_front_offset']*math.sin(heading_error)
            correction = math.atan2(self.p['stanley_cross_gain']*front_cross,
                                    speed+self.p['stanley_soft_speed'])
            lateral = -self.p['stanley_angular_gain']*correction
            yaw_rate += dt/(self.p['heading_d_filter_time']+dt)*(measured_rate-yaw_rate)
            candidate = clamp(integral+self.p['heading_ki']*heading_error*dt,.05)
            output = self.p['heading_kp']*heading_error+candidate-self.p['heading_kd']*yaw_rate+lateral
            if abs(output)<=self.p['drive_turn_limit'] or heading_error*output<0.:
                integral = candidate
            pid = self.p['heading_kp']*heading_error+integral-self.p['heading_kd']*yaw_rate
            omega = clamp(pid+lateral,self.p['drive_turn_limit'])
            fraction = min(1., remaining/self.p['slowdown_distance'])
            linear_speed = end_speed+(speed-end_speed)*fraction
            self.command(linear_speed,omega)
            now = self.now()
            if now>=next_log:
                self.get_logger().info(f'PID+Stanley直线（TF位置/IMU融合转速）：剩余{remaining:.3f}m，前进{linear_speed:.3f}m/s，横向{cross:+.3f}m，朝向误差{math.degrees(heading_error):+.1f}度，PID项{pid:+.3f}，横向项{lateral:+.3f}，角速度{omega:+.3f}rad/s。')
                next_log = now+1.

    def run(self):
        self.initialize()
        current = self.start
        previous_heading = self.start_yaw
        for index,goal in enumerate(self.goals,1):
            route = self.planner.plan(current,goal)
            if not route:
                raise RuntimeError('静态地图没有通路。')
            self.get_logger().info('58cm路线：'+' → '.join(route))
            route = self.planner.turning_points(route)
            self.get_logger().info('保存转弯点和终点：'+' → '.join(route))
            for source,destination in zip(route,route[1:]):
                start,end = self.points[source],self.points[destination]
                heading = math.atan2(end[1]-start[1],end[0]-start[0])
                # Route headings persist across goals; AMCL never changes the route.
                quarter_turns = round(wrap(heading-previous_heading)/(math.pi/2))
                if quarter_turns:
                    self.turn_relative(quarter_turns*math.pi/2)
                is_goal = destination == goal
                target = end
                self.get_logger().info(f'走向{destination}：'+('goal前15cm减速停车。' if is_goal else '到格子中心停车转弯。'))
                end_speed = self.p['approach_speed'] if is_goal else self.p['drive_speed']
                self.drive_line(start,target,end_speed=end_speed)
                self.halt(.3)
                previous_heading = heading
            self.get_logger().info('GOAL '+goal)
            self.halt(3.)
            self.get_logger().info(f'目标 {index}/4：{goal}，已停三秒。')
            current = goal
        self.get_logger().info('MISSION COMPLETE')


def main(args=None, mission_class=Mission):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupt)
    signal.signal(signal.SIGINT,interrupt)
    node,code = None,0
    try:
        node = mission_class()
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as error:
        if node is not None:
            node.get_logger().error(f'任务错误：{error}')
        else:
            print(error)
        code = 1
    finally:
        if node is not None:
            node.command()
            node.pause(.1)
            node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(code)
