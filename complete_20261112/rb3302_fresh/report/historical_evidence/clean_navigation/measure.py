import json,time,math,sys,os,csv
import xml.etree.ElementTree as ET
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.msg import Log
import yaml
rclpy.init();n=Node('simulation_evidence');points=yaml.safe_load(Path('/home/rb3302-7/rb3302_fresh/student_ws/src/rb3302_mission/config/course.yaml').read_text())['landmarks'];latest={};events=[];done=False;trace=[];collisions=0;boxes=None
out=Path(sys.argv[1]);stream=out.with_suffix('.jsonl').open('w')
def pose(key,msg):
 global boxes,collisions
 p=msg.pose.pose.position;latest[key]=[p.x,p.y]
 if key!='truth':return
 q=msg.pose.pose.orientation;angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
 trace.append([msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,p.x,p.y,angle])
 if boxes is None:
  world=Path(os.environ['TEST_WORLD'])
  if not world.exists():return
  boxes=[]
  for model in ET.parse(world).findall('./world/model'):
   if model.get('name')!='aligned_map_walls' and not model.get('name').startswith('obstacle_'):continue
   pose_text=model.findtext('pose','0 0 0 0 0 0');mx,my,*_=map(float,pose_text.split())
   for box in model.findall('./link/collision'):
    bx,by,*_=map(float,box.findtext('pose').split());sx,sy,*_=map(float,box.findtext('geometry/box/size').split())
    boxes.append((mx+bx,my+by,sx/2,sy/2))
 c,ss=math.cos(angle),math.sin(angle)
 body=[(p.x+c*x-ss*y,p.y+ss*x+c*y) for x,y in [(-.1,-.1),(.1,-.1),(.1,.1),(-.1,.1)]]
 for x,y,hx,hy in boxes:
  wall=[(x-hx,y-hy),(x+hx,y-hy),(x+hx,y+hy),(x-hx,y+hy)]
  hit=True
  for ax,ay in [(1,0),(0,1),(c,ss),(-ss,c)]:
   first=[xx*ax+yy*ay for xx,yy in body];second=[xx*ax+yy*ay for xx,yy in wall]
   if max(first)<=min(second) or max(second)<=min(first):hit=False;break
  if hit:collisions+=1;break

def log(msg):
 global done
 if msg.name!='rb3302_navigation':return
 if msg.msg.startswith(('GOAL','Confirmed','Route:','Turn','FAILED','MISSION')):
  event={'event':msg.msg,'poses':dict(latest)}
  if msg.msg.startswith('GOAL') and 'truth' in latest:
   k=msg.msg.split()[1];event['true_goal_error']=math.dist(latest['truth'],points[k])
  events.append(event);stream.write(json.dumps(event)+'\n');stream.flush();print(json.dumps(event),flush=True)
 if msg.msg=='MISSION COMPLETE' or msg.msg.startswith('FAILED'):done=True
n.create_subscription(Odometry,'/sim/ground_truth',lambda m:pose('truth',m),qos_profile_sensor_data)
n.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',lambda m:pose('amcl',m),qos_profile_sensor_data)
n.create_subscription(Log,'/rosout',log,100)
start=time.monotonic()
while not done and time.monotonic()-start<600:rclpy.spin_once(n,timeout_sec=.1)
out.write_text(json.dumps({'completed':any(e['event']=='MISSION COMPLETE' for e in events),'events':events,'collision_frames':collisions,'truth_samples':len(trace)},indent=2));stream.close()
with out.with_suffix('.csv').open('w') as f:
 writer=csv.writer(f);writer.writerow(['sim_time','x','y','yaw']);writer.writerows(trace)
n.destroy_node();rclpy.shutdown()
