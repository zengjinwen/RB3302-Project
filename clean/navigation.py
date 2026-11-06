"""Single timer state machine: localize, plan, turn, track, dwell, replan."""
import math
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener, TransformException
from planning import Planner, wrap


def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

class Navigation(Node):
    def __init__(self):
        super().__init__('rb3302_navigation')
        root=Path(__file__).resolve().parents[1]
        defaults=yaml.safe_load((root/'clean/navigation.yaml').read_text())['rb3302_navigation']['ros__parameters']
        defaults.update(start='X',bearing='N',goals='X,P,G,X')
        for k,v in defaults.items():self.declare_parameter(k,v)
        self.p={k:self.get_parameter(k).value for k in defaults}
        self.points=yaml.safe_load((root/'student_ws/src/rb3302_mission/config/course.yaml').read_text())['landmarks']
        self.start=self.p['start'].removeprefix('cell_');self.bearing=self.p['bearing'].removeprefix('dir_')
        self.goals=self.p['goals'].split(',')
        if any(k not in self.points for k in [self.start]+self.goals):raise ValueError('Unknown letter')
        self.start_yaw={'E':0.,'N':math.pi/2,'W':math.pi,'S':-math.pi/2}[self.bearing]
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.update_client=self.create_client(Empty,'/request_nomotion_update');self.update_future=None
        self.cmd=self.create_publisher(Twist,'/cmd_vel',10)
        self.initial=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        self.route_pub=self.create_publisher(RosPath,'/plan',10)
        self.cost_pub=self.create_publisher(OccupancyGrid,'/planning_costmap',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(OccupancyGrid,'/map',self.map_received,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry,'/odom',self.odom_received,10)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.amcl_received,10)
        self.create_subscription(LaserScan,'/scan',self.scan_received,qos_profile_sensor_data)
        self.planner=None;self.odom=None;self.amcl=None;self.scan=None
        self.state='initialize';self.goal_index=0;self.sent=0.;self.route=[];self.targets=[]
        self.candidate=None;self.candidate_since=0.;self.near_obstacle=False
        self.last_status=0.;self.state_since=self.now();self.turn_previous=0.;self.turn_progress=0.
        self.create_timer(.02,self.tick)

    def now(self):return self.get_clock().now().nanoseconds/1e9
    def log(self,text):self.get_logger().info(text)
    def command(self,v=0.,w=0.):
        msg=Twist();msg.linear.x=float(v);msg.angular.z=float(w);self.cmd.publish(msg)
    def transition(self,state):self.state=state;self.state_since=self.now()
    def map_received(self,msg):
        if self.planner is not None:return
        self.planner=Planner(msg,self.points,self.p['clearance'])
        display=OccupancyGrid();display.header=msg.header;display.info=msg.info
        display.data=[100 if (i%msg.info.width,i//msg.info.width) in self.planner.occupied else 0 for i in range(len(msg.data))]
        self.cost_pub.publish(display);self.log('60cm A* ready; footprint clearance 16cm.')
    def odom_received(self,msg):self.odom=msg
    def amcl_received(self,msg):self.amcl=msg
    def pose(self):
        try:
            t=self.tf.lookup_transform('map','base_link',rclpy.time.Time()).transform
            return t.translation.x,t.translation.y,yaw(t.rotation)
        except TransformException:return None
    def fresh(self,msg):
        if msg is None:return False
        t=msg.header.stamp;return -.1<=self.now()-t.sec-t.nanosec/1e9<1.
    def ready(self):
        if not (self.fresh(self.odom) and self.fresh(self.scan) and self.fresh(self.amcl)):
            return False
        # If 3-sigma spans half a grid or a cardinal direction, stop and rescan.
        c=self.amcl.pose.covariance
        localized=all(math.isfinite(c[i]) and 0<=c[i]<(.3/3)**2 for i in (0,7)) and 0<=c[35]<(math.pi/2/3)**2
        return localized and self.pose() is not None

    def scan_received(self,msg):
        self.scan=msg
        if self.update_client.service_is_ready() and (self.update_future is None or self.update_future.done()):
            self.update_future=self.update_client.call_async(Empty.Request())
        if self.planner is None:return
        try:t=self.tf.lookup_transform('map',msg.header.frame_id,rclpy.time.Time.from_msg(msg.header.stamp)).transform
        except TransformException:return
        a=yaw(t.rotation);groups={};self.near_obstacle=False
        robot=self.pose()
        if robot is None:return
        for i,d in enumerate(msg.ranges):
            if not math.isfinite(d) or not msg.range_min<d<min(msg.range_max,1.0):continue
            angle=a+msg.angle_min+i*msg.angle_increment
            p=(t.translation.x+d*math.cos(angle),t.translation.y+d*math.sin(angle))
            # Ignore returns near known walls, including small map/scan mismatches.
            if self.planner.pixel(p) in self.planner.wall_band:continue
            key=min(self.points,key=lambda k:math.dist(p,self.points[k]))
            c=self.points[key]
            if abs(p[0]-c[0])<.22 and abs(p[1]-c[1])<.22:groups.setdefault(key,[]).append(p)
            dx,dy=p[0]-robot[0],p[1]-robot[1]
            front=dx*math.cos(robot[2])+dy*math.sin(robot[2]);side=-dx*math.sin(robot[2])+dy*math.cos(robot[2])
            if 0<front<.20 and abs(side)<.12:self.near_obstacle=True
        candidates=[]
        for k,pts in groups.items():
            width=max(max(p[j] for p in pts)-min(p[j] for p in pts) for j in (0,1))
            if len(pts)>=5 and width>=self.p['obstacle_width']:candidates.append(k)
        key=min(candidates,key=lambda k:math.dist(robot[:2],self.points[k])) if candidates else None
        if key!=self.candidate:self.candidate=key;self.candidate_since=self.now()
        if key and self.now()-self.candidate_since>=self.p['obstacle_confirm'] and key!=self.planner.blocked:
            self.planner.blocked=key;self.log('Confirmed occupied cell '+key)
            if key in self.route and self.state not in ('initialize','dwell','done','failed'):
                self.command();self.transition('plan')

    def begin_turn(self,angle):
        self.turn_target=angle;self.turn_progress=0.;self.turn_previous=yaw(self.odom.pose.pose.orientation)
        self.transition('turn');self.log(f'Turn {math.degrees(angle):+.1f} deg')

    def plan(self,pose):
        goal=self.goals[self.goal_index];self.route=self.planner.path(pose[:2],goal)
        if not self.route:
            self.command()
            if self.now()-self.last_status>2:self.log('No route to '+goal);self.last_status=self.now()
            return
        self.log('Route: '+' -> '.join(self.route))
        names=self.planner.corners(self.route)
        if math.dist(pose[:2],self.points[names[0]])<=self.p['goal_tolerance']:names=names[1:]
        self.targets=[(k,tuple(self.points[k])) for k in names]
        path=RosPath();path.header.frame_id='map';path.header.stamp=self.get_clock().now().to_msg()
        for k in self.route:
            p=PoseStamped();p.header=path.header;p.pose.position.x,p.pose.position.y=self.points[k];p.pose.orientation.w=1.;path.poses.append(p)
        self.route_pub.publish(path)
        self.line_start=pose[:2];self.transition('track')

    def tick(self):
        if self.state in ('done','failed'):self.command();return
        if self.state=='initialize':
            self.command()
            if self.planner is None or self.initial.get_subscription_count()==0:return
            if self.sent==0.:
                p=PoseWithCovarianceStamped();p.header.frame_id='map';p.header.stamp=self.get_clock().now().to_msg()
                p.pose.pose.position.x,p.pose.pose.position.y=self.points[self.start]
                p.pose.pose.orientation.z=math.sin(self.start_yaw/2);p.pose.pose.orientation.w=math.cos(self.start_yaw/2)
                p.pose.covariance[0]=p.pose.covariance[7]=.03**2;p.pose.covariance[35]=.05**2
                self.initial.publish(p);self.sent=self.now();return
            if self.ready():self.transition('plan')
            return
        if not self.ready():self.command();return
        pose=self.pose()
        if self.state=='plan':self.plan(pose);return
        if self.state=='turn':
            current=yaw(self.odom.pose.pose.orientation)
            self.turn_progress+=wrap(current-self.turn_previous);self.turn_previous=current
            error=self.turn_target-self.turn_progress
            if abs(error)<=self.p['heading_tolerance'] or error*self.turn_target<0:
                self.command();self.transition('settle');return
            if self.now()-self.state_since>12.:
                self.command();self.log('FAILED: turn feedback did not reach angle');self.transition('failed');return
            self.command(0.,math.copysign(min(self.p['turn_speed'],max(.08,abs(error))),error));return
        if self.state=='settle':
            self.command()
            if self.now()-self.state_since>=.3:self.transition('track')
            return
        if self.state=='verify':
            self.command()
            stamp=self.amcl.header.stamp
            if self.now()-self.state_since<.3 or stamp.sec+stamp.nanosec/1e9<=self.state_since:return
            goal=self.points[self.goals[self.goal_index]]
            error=math.dist(pose[:2],goal)
            if error<=self.p['goal_tolerance']:
                self.log(f'GOAL {self.goals[self.goal_index]} error={error:.3f}m');self.transition('dwell')
            else:
                self.targets=[(self.goals[self.goal_index],tuple(goal))];self.line_start=pose[:2];self.transition('track')
            return
        if self.state=='dwell':
            self.command()
            if self.now()-self.state_since>=3.:
                self.goal_index+=1
                if self.goal_index==len(self.goals):self.log('MISSION COMPLETE');self.transition('done')
                else:self.transition('plan')
            return
        if not self.targets:
            goal=self.points[self.goals[self.goal_index]]
            if math.dist(pose[:2],goal)<=self.p['goal_tolerance']:
                self.command();self.transition('verify')
            else:self.targets=[(self.goals[self.goal_index],tuple(goal))];self.line_start=pose[:2]
            return
        name,end=self.targets[0];distance=math.dist(pose[:2],end)
        if distance<=self.p['goal_tolerance']/2:
            self.command();self.targets.pop(0);self.line_start=end
            if name in self.route:self.route=self.route[self.route.index(name)+1:]
            return
        dx,dy=end[0]-self.line_start[0],end[1]-self.line_start[1];length=math.hypot(dx,dy)
        along=((pose[0]-self.line_start[0])*dx+(pose[1]-self.line_start[1])*dy)/max(length,.001)
        ahead=min(length,max(0.,along)+self.p['lookahead'])
        target=(self.line_start[0]+dx*ahead/max(length,.001),self.line_start[1]+dy*ahead/max(length,.001))
        alpha=wrap(math.atan2(target[1]-pose[1],target[0]-pose[0])-pose[2])
        if abs(alpha)>math.pi/3:
            self.command();self.begin_turn(alpha);return
        if self.near_obstacle:self.command();return
        v=min(self.p['speed'],.6*distance)*max(0.,math.cos(alpha))
        w=2*v*math.sin(alpha)/max(.05,math.dist(pose[:2],target))
        self.command(v,max(-.6,min(.6,w)))

if __name__=='__main__':
    rclpy.init();node=Navigation()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.command();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
