import ast,math,time,threading
from pathlib import Path
from types import SimpleNamespace as N
s=Path('/home/rb3302-7/rb3302_fresh/robot_ws/src/waverover_base/waverover_base/waverover_driver.py').read_text()
cls=next(c for c in ast.parse(s).body if isinstance(c,ast.ClassDef) and c.name=='WaveRoverDriver')
funcs=[f for f in cls.body if isinstance(f,ast.FunctionDef) and f.name in ('_publish_odom','_yaw_radians')]
class Stamp:
 def __init__(self,t):self.nanoseconds=int(t*1e9)
 def __sub__(self,o):return Stamp((self.nanoseconds-o.nanoseconds)/1e9)
 def to_msg(self):return self
class Odometry:
 def __init__(self):
  self.header=N();self.pose=N(pose=N(position=N(),orientation=N()),covariance=[0.]*36)
  self.twist=N(twist=N(linear=N(),angular=N()),covariance=[0.]*36)
ns={'math':math,'time':time,'Odometry':Odometry};exec(compile(ast.Module(body=funcs,type_ignores=[]),'<actual-driver>','exec'),ns)
print('Yaw conversion for degree-valued feedback 5,6,7:',[round(ns['_yaw_radians'](None,x),6) for x in (5,6,7)])
r=N(last_feedback_time=time.monotonic(),feedback_timeout=1.,feedback_lock=threading.Lock(),dist_centre=0.,yaw_odom=0.,prev_dist_centre=0.,prev_odom_time=Stamp(0),odom_x=0.,odom_y=0.,odom_frame='odom',base_frame='base_link',current_angular=0.,publish_odom_tf=False)
r._yaw_radians=lambda a:ns['_yaw_radians'](r,a)
now=[0.];r.get_clock=lambda:N(now=lambda:Stamp(now[0]));out=[];r.odom_pub=N(publish=out.append)
# One 1mm feedback arrives; same measurement is reused by following 100Hz ticks.
for t in [.01,.02,.03]:
 now[0]=t;r.dist_centre=.001;ns['_publish_odom'](r)
print('One feedback reused over 3 timer ticks, published vx:',[round(m.twist.twist.linear.x,4) for m in out])
print('Published vx measurement variance:',out[0].twist.covariance[0])
