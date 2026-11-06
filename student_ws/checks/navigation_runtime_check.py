#!/usr/bin/python3
"""Isolated software-only check; never launch alongside a physical robot.
Starts and stops a headless planning_rpp launch on localhost ROS_DOMAIN_ID=231.
Publishes ideal synthetic sensors and integrates cmd_vel; NOT mission code.
"""
import os
assert os.environ.get('ROS_DOMAIN_ID') == '231'
assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'LOCALHOST'
import json, math, time, subprocess, signal
from pathlib import Path
import numpy as np
from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from nav2_msgs.action import ComputePathToPose, NavigateToPose, FollowWaypoints
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters
from tf2_ros import StaticTransformBroadcaster

ROOT = Path(__file__).resolve().parents[1]
class TestRobot(Node):
    def __init__(self):
        super().__init__('software_test_robot')
        self.x, self.y, self.yaw = -.462, .01, 0.
        self.v = self.w = 0.
        self.max_v = 0.
        self.last = time.monotonic()
        self.command_time = 0.
        self.grid = np.asarray(Image.open(ROOT / 'maps/rb3302_map.pgm'))
        self.odom = self.create_publisher(Odometry, '/odom/raw', 10)
        self.imu = self.create_publisher(Imu, '/imu_data_ros', 10)
        self.scan = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.initial = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(Twist, '/cmd_vel', self.command, 10)
        self.estimate = None
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.estimated, 10)
        self.tf = StaticTransformBroadcaster(self)
        transforms=[]
        for child in ['laser','gyro_link']:
            t=TransformStamped(); t.header.stamp=self.get_clock().now().to_msg()
            t.header.frame_id='base_link'; t.child_frame_id=child; t.transform.rotation.w=1.
            transforms.append(t)
        self.tf.sendTransform(transforms)
        self.create_timer(.02, self.sensors)
        self.create_timer(.1, self.laser)
    def estimated(self,m): self.estimate=m.pose.pose
    def command(self,m):
        self.v,self.w=m.linear.x,m.angular.z
        self.max_v=max(self.max_v,abs(self.v)); self.command_time=time.monotonic()
    def sensors(self):
        now=time.monotonic(); dt=min(now-self.last,.1); self.last=now
        if now-self.command_time>1.: self.v=self.w=0.
        self.x+=dt*self.v*math.cos(self.yaw); self.y+=dt*self.v*math.sin(self.yaw); self.yaw+=dt*self.w
        stamp=self.get_clock().now().to_msg()
        o=Odometry(); o.header.stamp=stamp; o.header.frame_id='odom'; o.child_frame_id='base_link'
        o.pose.pose.position.x=self.x+.462; o.pose.pose.position.y=self.y-.01
        o.pose.pose.orientation.z=math.sin(self.yaw/2); o.pose.pose.orientation.w=math.cos(self.yaw/2)
        o.twist.twist.linear.x=self.v; o.twist.twist.angular.z=self.w
        for k in range(6): o.twist.covariance[k*7]=.001; o.pose.covariance[k*7]=.001
        self.odom.publish(o)
        m=Imu(); m.header.stamp=stamp; m.header.frame_id='gyro_link'; m.orientation=o.pose.pose.orientation
        m.angular_velocity.z=self.w; m.linear_acceleration.z=9.81
        for k in [0,4,8]:
            m.orientation_covariance[k]=.001; m.angular_velocity_covariance[k]=.001; m.linear_acceleration_covariance[k]=.01
        self.imu.publish(m)
    def laser(self):
        angles=np.arange(360)*2*math.pi/360-math.pi
        ds=np.arange(.05,6.01,.025)
        xs=self.x+np.cos(angles[:,None]+self.yaw)*ds
        ys=self.y+np.sin(angles[:,None]+self.yaw)*ds
        cs=np.floor((xs+1.487)/.05).astype(int); rs=self.grid.shape[0]-1-np.floor((ys+1.015)/.05).astype(int)
        valid=(cs>=0)&(cs<self.grid.shape[1])&(rs>=0)&(rs<self.grid.shape[0])
        hits=~valid | (self.grid[np.clip(rs,0,self.grid.shape[0]-1),np.clip(cs,0,self.grid.shape[1]-1)]<250)
        idx=hits.argmax(axis=1); ranges=np.where(hits.any(axis=1),ds[idx],6.)
        m=LaserScan(); m.header.stamp=self.get_clock().now().to_msg(); m.header.frame_id='laser'
        m.angle_min=-math.pi; m.angle_increment=2*math.pi/360; m.angle_max=m.angle_min+359*m.angle_increment
        m.range_min=.05; m.range_max=6.; m.scan_time=.1; m.ranges=ranges.tolist(); self.scan.publish(m)
    def seed(self):
        m=PoseWithCovarianceStamped(); m.header.frame_id='map'; m.header.stamp=self.get_clock().now().to_msg()
        m.pose.pose.position.x=self.x; m.pose.pose.position.y=self.y; m.pose.pose.orientation.w=1.
        m.pose.covariance[0]=.0001; m.pose.covariance[7]=.0001; m.pose.covariance[35]=.0001
        self.initial.publish(m)
    def pump(self,sec):
        end=time.monotonic()+sec
        while time.monotonic()<end: rclpy.spin_once(self,timeout_sec=.02)
    def future(self,f,timeout=30.):
        end=time.monotonic()+timeout
        while not f.done() and time.monotonic()<end: rclpy.spin_once(self,timeout_sec=.02)
        if not f.done(): raise TimeoutError('ROS request timed out')
        return f.result()
    def service(self,kind,name,request):
        c=self.create_client(kind,name); end=time.monotonic()+15
        while not c.service_is_ready() and time.monotonic()<end: self.pump(.1)
        assert c.service_is_ready(), name+' unavailable'
        result=self.future(c.call_async(request)); self.destroy_client(c); return result
    def pose(self,x,y):
        p=PoseStamped(); p.header.frame_id='map'; p.header.stamp=self.get_clock().now().to_msg()
        p.pose.position.x=x; p.pose.position.y=y; p.pose.orientation.w=1.; return p
    def action(self,kind,name,goal,timeout=45.):
        c=ActionClient(self,kind,name); end=time.monotonic()+10
        while not c.server_is_ready() and time.monotonic()<end: self.pump(.1)
        assert c.server_is_ready(), name+' unavailable'
        handle=self.future(c.send_goal_async(goal)); assert handle.accepted, name+' rejected'
        start=time.monotonic()
        try: result=self.future(handle.get_result_async(),timeout)
        except Exception:
            self.future(handle.cancel_goal_async(),5); raise
        elapsed=time.monotonic()-start
        c.destroy(); return result,elapsed

def main():
    rclpy.init(); n=TestRobot(); report={}
    logfile=open(ROOT/'log/runtime_launch.log','w')
    proc=subprocess.Popen(['ros2','launch','planning_rpp','planning.launch.py','rviz:=false','set_initial_pose:=true','initial_pose_x:=-0.462','initial_pose_y:=0.01','initial_pose_yaw:=0.0'],stdout=logfile,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        n.pump(5); n.seed(); n.pump(7)
        names=['map_server','amcl','planner_server','controller_server','smoother_server','behavior_server','bt_navigator','waypoint_follower','velocity_smoother']
        report['lifecycle']={name:n.service(GetState,'/'+name+'/get_state',GetState.Request()).current_state.label for name in names}
        print('LIFECYCLE',report['lifecycle'],flush=True)
        assert all(v=='active' for v in report['lifecycle'].values())
        for node,params in [('controller_server',['goal_checker.plugin','goal_checker.xy_goal_tolerance','goal_checker.trans_stopped_velocity']),('waypoint_follower',['wait_at_waypoint.waypoint_pause_duration']),('global_costmap/global_costmap',['footprint']),('local_costmap/local_costmap',['footprint'])]:
            req=GetParameters.Request(); req.names=params
            vals=n.service(GetParameters,'/'+node+'/get_parameters',req).values
            report[node]={k: v.string_value if v.type==4 else v.double_value if v.type==3 else v.integer_value for k,v in zip(params,vals)}
        print('PARAMETERS',report,flush=True)
        target_x,target_y=-.162,.01
        g=ComputePathToPose.Goal(); g.start=n.pose(n.x,n.y); g.goal=n.pose(target_x,target_y); g.use_start=True; g.planner_id='GridBased'
        r,t=n.action(ComputePathToPose,'/compute_path_to_pose',g)
        report['plan']={'status':r.status,'error_code':r.result.error_code,'poses':len(r.result.path.poses)}
        print('PATH',report['plan'],flush=True); assert r.status==4 and len(r.result.path.poses)>1
        g=NavigateToPose.Goal(); g.pose=n.pose(target_x,target_y)
        r,t=n.action(NavigateToPose,'/navigate_to_pose',g,60)
        n.pump(1)
        report['navigate']={'status':r.status,'error_code':r.result.error_code,'seconds':t,'ground_truth_error_m':math.hypot(n.x-target_x,n.y-target_y),'max_cmd_linear':n.max_v,'final_v':n.v,'final_w':n.w,'amcl_received':n.estimate is not None}
        print('NAVIGATE',report['navigate'],flush=True); assert r.status==4 and n.max_v>0 and abs(n.v)<.02 and abs(n.w)<.05
        g=FollowWaypoints.Goal(); g.poses=[n.pose(target_x,target_y),n.pose(target_x,target_y)]
        r,t=n.action(FollowWaypoints,'/follow_waypoints',g,30)
        report['waypoints']={'status':r.status,'error_code':r.result.error_code,'missed':len(r.result.missed_waypoints),'seconds':t}
        print('WAYPOINTS',report['waypoints'],flush=True); assert r.status==4 and not r.result.missed_waypoints and t>=6.
        report['passed']=True
    except Exception as exc:
        report['passed']=False; report['failure']=repr(exc); raise
    finally:
        (ROOT/'log/runtime_result.json').write_text(json.dumps(report,indent=2)+'\n')
        os.killpg(proc.pid,signal.SIGINT)
        try: proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid,signal.SIGTERM); proc.wait(timeout=5)
        logfile.close()
        n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
