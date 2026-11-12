"""ROS integration check: expire localization while other inputs stay fresh."""
import os,sys,time,json,signal,subprocess
from pathlib import Path
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
root=Path(__file__).resolve().parents[1]
env=os.environ.copy();env.update(ROS_DOMAIN_ID='98',ROS_LOCALHOST_ONLY='1')
os.environ.update(ROS_DOMAIN_ID='98',ROS_LOCALHOST_ONLY='1')
child=subprocess.Popen([sys.executable,str(root/'motion_gate.py'),'--ros-args','-p','use_sim_time:=true'],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
rclpy.init();node=Node('gate_test');latest=[]
pubs={topic:node.create_publisher(typ,topic,10) for topic,typ in [('/clock',Clock),('/cmd_vel_smoothed',Twist),('/motion_enabled',Bool),('/odom',Odometry),('/scan',LaserScan),('/amcl_pose',PoseWithCovarianceStamped)]}
node.create_subscription(Twist,'/cmd_vel',lambda m:latest.append(m.linear.x),10)
simtime=1.
def advance(seconds,localization):
 global simtime
 for _ in range(round(seconds/.05)):
  simtime+=.05;clock=Clock();clock.clock.sec=int(simtime);clock.clock.nanosec=int((simtime-int(simtime))*1e9);pubs['/clock'].publish(clock)
  cmd=Twist();cmd.linear.x=.1;pubs['/cmd_vel_smoothed'].publish(cmd);pubs['/motion_enabled'].publish(Bool(data=True))
  for topic,typ in [('/odom',Odometry),('/scan',LaserScan)]+([('/amcl_pose',PoseWithCovarianceStamped)] if localization else []):
   msg=typ();msg.header.stamp=clock.clock
   if topic=='/amcl_pose':msg.pose.pose.orientation.w=1.
   pubs[topic].publish(msg)
  end=time.monotonic()+.025
  while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.003)
 return latest[-5:]
try:
 deadline=time.monotonic()+5
 while pubs['/clock'].get_subscription_count()==0 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
 normal=advance(2.,True);expired=advance(2.,False);restored=advance(2.,True)
 result={'normal':normal,'expired_localization':expired,'restored':restored,'passed':len(normal)==5 and all(v==.1 for v in normal) and len(expired)==5 and all(v==0 for v in expired) and len(restored)==5 and all(v==.1 for v in restored)}
 (root/'evidence/gate_test.json').write_text(json.dumps(result,indent=2));print(json.dumps(result));assert result['passed']
finally:
 child.send_signal(signal.SIGINT);child.wait(timeout=10);node.destroy_node();rclpy.try_shutdown()
