import ast,math
from pathlib import Path
from types import SimpleNamespace as N
src=Path('/home/rb3302-7/rb3302_fresh/student_ws/src/rb3302_mission/rb3302_mission/node.py')
f=next(n for c in ast.parse(src.read_text()).body if isinstance(c,ast.ClassDef) for n in c.body if isinstance(n,ast.FunctionDef) and n.name=='drive_line')
wrap=lambda a:math.atan2(math.sin(a),math.cos(a))
clamp=lambda x,l:max(-l,min(l,x))
class Timeout(Exception):pass
class Sim:
 def __init__(self,x,y,a):
  self.x,self.y,self.a=x,y,a;self.t=0.;self.v=0.;self.w=0.;self.maxerr=abs(y);self.anchor=(0.,0.,0.);self.wait_reason=None
  self.drive_velocity=0.;self.steering=0.;self.messages={'odom':N(pose=N(pose=N(position=N(x=x,y=y))))}
  self.p={'lookahead_distance':.3,'drive_speed':.1,'drive_acceleration':.2,'drive_turn_limit':.35,'steer_acceleration':.5}
  self.line_pid=N(reset=lambda:None,update=lambda e,w,dt:1.5*e-.05*w)
 def pause(self,dt):
  self.x+=self.v*math.cos(self.a)*dt;self.y+=self.v*math.sin(self.a)*dt;self.a=wrap(self.a+self.w*dt);self.t+=dt;self.maxerr=max(self.maxerr,abs(self.y))
  self.messages['odom'].pose.pose.position=N(x=self.x,y=self.y)
  if self.t>90:raise Timeout()
 def ready(self):return True
 def pose(self):return self.x,self.y,self.a
 def at(self,end):return math.hypot(self.x-end[0],self.y-end[1])<=.05
 def amcl_position(self):return self.x,self.y
 def blocked(self,d):return False
 def odom_yaw_rate(self):return self.w
 def command(self,v=0.,w=0.):self.v,self.w=v,w
for initial in [(0,0,0),(0,.10,0),(0,.10,math.pi),(.04,.04,0),(.0,.25,0),(.0,.25,math.pi),(.0,.40,0)]:
 r=Sim(*initial);ns={'math':math,'time':N(monotonic=lambda:r.t),'wrap':wrap,'clamp':clamp};exec(compile(ast.Module(body=[f],type_ignores=[]),str(src),'exec'),ns)
 try:ns['drive_line'](r,(0,0),(.6,0));result='ARRIVED'
 except Timeout:result='TIMEOUT'
 print(initial,result,'time',round(r.t,2),'xy',round(r.x,3),round(r.y,3),'max lateral',round(r.maxerr,3))
# Exact contradictory command case: raw AMCL is beyond target, but fused pose is behind.
r=Sim(.5,.10,0);r.messages['odom'].pose.pose.position=N(x=0.,y=0.);steps=0
orig_pause=r.pause
# Direct geometry used by drive_line after odom_done: AMCL says x=.65 while fused x=.5.
x,y,yaw=.5,.10,0.;end=.6
bearing=math.atan2(-y,end-x);error=wrap(bearing-yaw)
if abs(error)>math.pi/2:error=wrap(error+math.pi)
v=-.03;w=1.5*error
print('Mixed-position overshoot: v',v,'angular request',w,'y after 0.1s',y+v*math.sin(w*.1)*.1,'initial y',y)
