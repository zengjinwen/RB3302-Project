"""Read-only Gazebo evidence. Ground truth is never used by navigation."""
import csv,json,math,os,time,sys
from pathlib import Path
from collections import deque
import xml.etree.ElementTree as ET
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, Path as RosPath, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import String
from rcl_interfaces.msg import Log
import re
from rosgraph_msgs.msg import Clock

class Evidence(Node):
    def __init__(self,out,root):
        super().__init__('autonomous_evidence');self.out=out;self.root=root
        self.points=yaml.safe_load((root/'student_ws/src/rb3302_mission/config/course.yaml').read_text())['landmarks']
        self.now=0.;self.events=[];self.trace=[];self.commands=[];self.errors=[];self.history=deque(maxlen=1000)
        self.boxes=None;self.collision_frames=0;self.latest=None;self.done=False;self.paths=[];self.audit={};self.obstacle_seen=False;self.scan=None;self.goal_scans=[];self.estimates=[]
        self.stream=out.with_suffix('.jsonl').open('w')
        self.create_subscription(Clock,'/clock',lambda m:setattr(self,'now',m.clock.sec+m.clock.nanosec/1e9),10)
        self.create_subscription(LaserScan,'/scan',lambda m:setattr(self,'scan',m),qos_profile_sensor_data)
        self.create_subscription(Odometry,'/sim/ground_truth',self.truth,qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.amcl,qos_profile_sensor_data)
        self.create_subscription(Twist,'/cmd_vel',lambda m:self.commands.append((self.now,m.linear.x,m.angular.z)),10)
        self.create_subscription(RosPath,'/plan',self.path,10)
        self.create_subscription(OccupancyGrid,'/global_costmap/costmap',self.costmap,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.requested=os.environ['CASE_GOALS'].split(',');self.goal_index=0;self.turns=[];self.active_turn=None;self.began=False
        self.create_subscription(Log,'/rosout',self.legacy_log,100)
    def event(self,text,when=None):
        self.log(String(data=json.dumps({'event':text,'time':self.now if when is None else when})))
    def legacy_log(self,msg):
        if msg.name!='rb3302_mission':return
        text=msg.msg
        if text.startswith('已发送起点') and not self.began:
            self.began=True;self.event('MISSION START')
        elif text.startswith('60cm路线'):
            self.event('TARGET '+self.requested[self.goal_index])
        elif text.startswith('GOAL '):
            self.event(text);self.goal_index+=1
        elif text=='MISSION COMPLETE':
            self.event(text)
        elif text.startswith('IMU相对转弯'):
            self.active_turn={'start':self.now,'requested_degrees':float(re.search(r'：([+\-\d.]+)',text)[1])}
        elif text.startswith('IMU转弯结束') and self.active_turn is not None:
            self.active_turn['end']=self.now;self.turns.append(self.active_turn);self.active_turn=None
        elif text.startswith('确认稳定大障碍'):
            self.obstacle_seen=True;self.event(text)
        elif text.startswith('任务停止'):
            self.event('FAILED '+text)
    @staticmethod
    def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    def truth(self,msg):
        p=msg.pose.pose.position;a=self.yaw(msg.pose.pose.orientation);t=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        v=msg.twist.twist
        row=(t,p.x,p.y,a,v.linear.x,v.linear.y,v.angular.z)
        self.latest=row;self.history.append(row);self.trace.append(row)
        if self.boxes is None:
            self.boxes=[]
            for model in ET.parse(os.environ['TEST_WORLD']).findall('./world/model'):
                if model.get('name')!='aligned_map_walls' and not model.get('name').startswith('obstacle_'):continue
                mx,my,*_=map(float,model.findtext('pose','0 0 0 0 0 0').split())
                for b in model.findall('./link/collision'):
                    x,y,*_=map(float,b.findtext('pose').split());sx,sy,*_=map(float,b.findtext('geometry/box/size').split())
                    self.boxes.append((mx+x,my+y,sx/2,sy/2))
        c,s=math.cos(a),math.sin(a)
        body=[(p.x+c*x-s*y,p.y+s*x+c*y) for x,y in [(-.1,-.1),(.1,-.1),(.1,.1),(-.1,.1)]]
        for x,y,hx,hy in self.boxes:
            wall=[(x-hx,y-hy),(x+hx,y-hy),(x+hx,y+hy),(x-hx,y+hy)]
            hit=True
            for ax,ay in [(1,0),(0,1),(c,s),(-s,c)]:
                bb=[x*ax+y*ay for x,y in body];ww=[x*ax+y*ay for x,y in wall]
                if max(bb)<=min(ww) or max(ww)<=min(bb):hit=False;break
            if hit:self.collision_frames+=1;break
    def amcl(self,msg):
        if not self.history:return
        t=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        truth=min(self.history,key=lambda r:abs(r[0]-t))
        if abs(truth[0]-t)>.08:return
        p=msg.pose.pose.position
        self.estimates.append((t,p.x,p.y,self.yaw(msg.pose.pose.orientation)))
        dyaw=self.yaw(msg.pose.pose.orientation)-truth[3]
        self.errors.append((math.hypot(p.x-truth[1],p.y-truth[2]),math.atan2(math.sin(dyaw),math.cos(dyaw))))
    def path(self,msg):
        pts=[(p.pose.position.x,p.pose.position.y) for p in msg.poses]
        if pts:self.paths.append({'time':self.now,'points':pts})
    def costmap(self,msg):
        key=os.environ.get('SIM_OBSTACLE')
        if not key:return
        x,y=self.points[key];info=msg.info
        # A visible surface (not the unseen centre) should be marked lethal.
        r=float(os.environ.get('SIM_OBSTACLE_SIZE','.4'))/2
        for sy in (y-r,y,y+r):
            for sx in (x-r,x,x+r):
                ix=int((sx-info.origin.position.x)/info.resolution);iy=int((sy-info.origin.position.y)/info.resolution)
                if 0<=ix<info.width and 0<=iy<info.height and msg.data[iy*info.width+ix]>=99:self.obstacle_seen=True
    def log(self,msg):
        e=json.loads(msg.data)
        if e['event'].startswith('GOAL') and self.scan is not None:
            m=self.scan
            self.goal_scans.append({'event':e,'stamp':m.header.stamp.sec+m.header.stamp.nanosec/1e9,'angle_min':m.angle_min,'angle_increment':m.angle_increment,'range_min':m.range_min,'range_max':m.range_max,'ranges':[float(r) if math.isfinite(r) else None for r in m.ranges]})
        self.events.append(e);self.stream.write(json.dumps(e)+'\n');self.stream.flush();print(json.dumps(e),flush=True)
        if e['event'].startswith(('MISSION COMPLETE','FAILED')):self.done=True
    def save(self):
        for t in (() if not rclpy.ok() else ('/sim/ground_truth','/scan','/odom/raw','/odom','/amcl_pose','/cmd_vel','/cmd_vel_nav')):
            self.audit[t]={'publishers':[v.node_name for v in self.get_publishers_info_by_topic(t)],'subscribers':[v.node_name for v in self.get_subscriptions_info_by_topic(t)]}
        # Match recorded event time to truth, independent of DDS callback ordering.
        for e in self.events:
            if not self.trace:continue
            row=min(self.trace,key=lambda r:abs(r[0]-e['time']))
            e['truth']=row
            if e['event'].startswith('GOAL'):
                p=self.points[e['event'].split()[1]];_,x,y,a,*_=row
                e['true_goal_error']=math.dist((x,y),p)
                dx,dy=p[0]-x,p[1]-y
                e['centre_under_body']=abs(dx*math.cos(a)+dy*math.sin(a))<=.1 and abs(-dx*math.sin(a)+dy*math.cos(a))<=.1
        goals=[e for e in self.events if e['event'].startswith('GOAL')]
        start=next((e['time'] for e in self.events if e['event']=='MISSION START'),None)
        finish=next((e['time'] for e in self.events if e['event'].startswith('MISSION COMPLETE')),None)
        holds=[]
        for goal in goals:
            next_event=next((e for e in self.events if e['time']>goal['time'] and e['event'].startswith(('TARGET','MISSION COMPLETE'))),None)
            if next_event:
                span=[r for r in self.trace if goal['time']<=r[0]<=next_event['time']]
                travel=max((math.hypot(r[1]-goal['truth'][1],r[2]-goal['truth'][2]) for r in span),default=0.)
                holds.append({'goal':goal['event'].split()[1],'duration':next_event['time']-goal['time'],'max_translation':travel})
        result={'completed':finish is not None,'elapsed_sim_seconds':None if finish is None or start is None else finish-start,'goals':goals,'events':self.events,'holds':holds,'collision_frames':self.collision_frames,'obstacle_seen_in_costmap':self.obstacle_seen,'plans':len(self.paths),'amcl_position_rmse':math.sqrt(sum(p*p for p,a in self.errors)/max(1,len(self.errors))),'amcl_yaw_rmse_deg':math.degrees(math.sqrt(sum(a*a for p,a in self.errors)/max(1,len(self.errors)))),'topic_audit':self.audit}
        expected=os.environ['CASE_GOALS'].split(',')
        result['passed']=bool(finish is not None and finish-start<=180. and [e['event'].split()[1] for e in goals]==expected and self.collision_frames==0 and all(e['centre_under_body'] for e in goals) and len(holds)==4 and all(h['duration']>=2.9 and h['max_translation']<.02 for h in holds) and (not os.environ.get('SIM_OBSTACLE') or self.obstacle_seen))
        self.out.with_suffix('.scans.json').write_text(json.dumps(self.goal_scans));self.out.with_suffix('.amcl.json').write_text(json.dumps(self.estimates))
        for turn in self.turns:
            first=min(self.trace,key=lambda r:abs(r[0]-turn['start']));last=min(self.trace,key=lambda r:abs(r[0]-turn['end']))
            measured=math.degrees(math.atan2(math.sin(last[3]-first[3]),math.cos(last[3]-first[3])))
            turn['true_degrees']=measured
            turn['error_degrees']=math.degrees(math.atan2(math.sin(math.radians(measured-turn['requested_degrees'])),math.cos(math.radians(measured-turn['requested_degrees']))))
        result['turns']=self.turns
        self.out.write_text(json.dumps(result,indent=2));self.out.with_suffix('.paths.json').write_text(json.dumps(self.paths))
        for suffix,rows,header in [('.csv',self.trace,['sim_time','x','y','yaw','vx','vy','w']),('.commands.csv',self.commands,['sim_time','v','w'])]:
            with self.out.with_suffix(suffix).open('w') as f:w=csv.writer(f);w.writerow(header);w.writerows(rows)
        self.stream.close()
if __name__=='__main__':
    rclpy.init();node=Evidence(Path(sys.argv[1]),Path(os.environ['TEST_ROOT']));start=time.monotonic()
    try:
        while not node.done and time.monotonic()-start<600:rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:pass
    finally:node.save();node.destroy_node();rclpy.try_shutdown()
