import ast,math,threading
from types import SimpleNamespace as N
from pathlib import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time
src=Path('/home/rb3302-7/rb3302_fresh/robot_ws/src/waverover_base/waverover_base/waverover_driver.py')
c=next(x for x in ast.parse(src.read_text()).body if isinstance(x,ast.ClassDef) and x.name=='WaveRoverDriver')
names={'_process_feedback','_publish_odom','_yaw_radians','_estimate_battery_percentage'}
now=[10.];env={'math':math,'threading':threading,'time':N(monotonic=lambda:now[0]),'json':__import__('json'),'Odometry':Odometry,'TransformStamped':TransformStamped}
exec(compile(ast.Module(body=[f for f in c.body if isinstance(f,ast.FunctionDef) and f.name in names],type_ignores=[]),str(src),'exec'),env)
r=N(feedback_lock=threading.Lock(),feedback_sample=None,previous_sample=None,feedback_sequence=0,published_sequence=0,last_feedback_time=0.,yaw_units='radians',feedback_timeout=1.,odom_x=0.,odom_y=0.,odom_frame='odom',base_frame='base_link',current_linear=-.1,current_angular=9.,publish_odom_tf=False,linear_variance=.03**2,angular_variance=.1**2,last_motion_log=0.,dist_L=0.,dist_R=0.,dist_centre=0.,yaw_odom=0.,yaw_imu=0.,enc_A=0,enc_B=0,voltage=12.,battery_full_voltage=12.6,battery_empty_voltage=9.)
for name in names:setattr(r,name,lambda *a,name=name:env[name](r,*a))
r.get_clock=lambda:N(now=lambda:N(to_msg=lambda:Time(sec=int(now[0]),nanosec=int((now[0]%1)*1e9))))
logs=[];r.get_logger=lambda:N(info=logs.append);messages=[];r.odom_pub=N(publish=messages.append)
def packet(d,yaw=0.):r._process_feedback({'T':1001,'dist_centre':d,'yaw_odom':yaw})
packet(0.);r._publish_odom();assert not messages
now[0]+=.1;packet(-.01);r._publish_odom()
assert abs(messages[-1].twist.twist.linear.x+.1)<1e-8
assert messages[-1].twist.covariance[0]>0
assert messages[-1].twist.twist.angular.z==0. # not command=9
r._publish_odom();r._publish_odom();assert len(messages)==1
now[0]+=.1;packet(-.02,.02);r._publish_odom();assert abs(messages[-1].twist.twist.angular.z-.2)<1e-8
stamp=r.last_feedback_time;r._process_feedback({'T':1001});assert r.last_feedback_time==stamp
r.yaw_units='degrees';assert abs(r._yaw_radians(5)-math.radians(5))<1e-12
r.yaw_units='radians';assert r._yaw_radians(7)==7
print('PASS driver: fresh measurements only; reverse sign preserved; sample interval; positive covariance; measured angular velocity; explicit units; partial feedback ignored')
print('Example motion log:',logs[0])
# All Python files changed in this pass parse without importing/starting hardware.
paths=[src,Path('/home/rb3302-7/rb3302_fresh/student_ws/src/rb3302_mission/rb3302_mission/node.py'),Path('/home/rb3302-7/rb3302_fresh/student_ws/src/rb3302_mission/rb3302_mission/robot.py'),Path('/home/rb3302-7/rb3302_fresh/robot_ws/src/rplidar_ros/launch/rplidar_launch.py')]
for p in paths:ast.parse(p.read_text())
print('PASS changed Python files parse')
