"""检查60cm路径、转弯点目标、恒速直走；不启动硬件。"""
import math
from pathlib import Path
import sys
from types import SimpleNamespace as S
from unittest.mock import patch
import yaml
from PIL import Image
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'student_ws/src/rb3302_mission'))
from rb3302_mission.grid import MapPlanner, AnglePID
from rb3302_mission.node import Mission
pkg=ROOT/'student_ws/src/rb3302_mission'
params=yaml.safe_load((pkg/'config/control.yaml').read_text())['rb3302_mission']['ros__parameters']
points=yaml.safe_load((pkg/'config/course.yaml').read_text())['landmarks']
cfg=yaml.safe_load((ROOT/'student_ws/maps/rb3302_refined_aligned.yaml').read_text())
im=Image.open(ROOT/'student_ws/maps'/cfg['image']);w,h=im.size;pixels=list(im.getdata())
data=[0 if pixels[y*w+x]>250 else 100 if pixels[y*w+x]<10 else -1 for y in range(h-1,-1,-1) for x in range(w)]
msg=S(info=S(resolution=.05,width=w,height=h,origin=S(position=S(x=cfg['origin'][0],y=cfg['origin'][1]))),data=data)
planner=MapPlanner(msg,points)
for goal in points:
    route=planner.plan('A',goal)
    assert route and route[0]=='A' and route[-1]==goal
    for a,b in zip(route,route[1:]):
        assert b in planner.neighbors[a] and math.isclose(math.dist(points[a],points[b]),.6)
steps=[];stops=[]
f=S(start='A',goals=['B','C','I','G'],points=points,p=params,planner=planner,initialize=lambda:None,
    pose=lambda:(0.,0.,0.),turn=lambda angle:True,drive_line=lambda a,b:steps.append((a,b)),halt=lambda secs:stops.append(secs),
    get_logger=lambda:S(info=lambda text:None))
Mission.run(f)
assert planner.turning_points(['A','B','C']) == ['A','C']
assert planner.turning_points(['A']) == ['A']
assert planner.turning_points(['A','B','A']) == ['A','B','A']
assert planner.turning_points(planner.plan('I','G')) == ['I','C','A','G']
expected=[(points[a],points[b]) for a,b in [('A','B'),('B','C'),('C','I'),('I','C'),('C','A'),('A','G')]]
assert steps==expected and stops==[3.]*4

class Straight:
    at=Mission.at
    def __init__(self,x=0.,y=0.,yaw=0.):
        self.p=params;self.x=x;self.y=y;self.yaw=yaw;self.t=0.;self.v=0.;self.w=0.;self.commands=[];self.wait_reason=None
        self.line_pid=AnglePID(params['heading_kp'],params['heading_ki'],params['heading_kd'])
        self.steering=0.;self.drive_velocity=0.
    def ready(self): return True
    def blocked(self,direction=1.): return False
    def amcl_position(self): return self.x,self.y
    def pose(self): return self.x,self.y,self.yaw
    def odom_yaw_rate(self): return self.w
    def command(self,v=0.,w=0.):self.v=v;self.w=w;self.commands.append((v,w))
    def pause(self,dt=.05):
        self.t+=dt;self.x+=self.v*math.cos(self.yaw)*dt;self.y+=self.v*math.sin(self.yaw)*dt;self.yaw+=self.w*dt
        assert self.t<60.,'未到点'
for initial in [(0.,0.,0.),(0.,.1,0.),(0.,0.,math.pi),(.75,.1,0.),(.6,.1,0.)]:
    straight=Straight(*initial)
    with patch('rb3302_mission.node.time.monotonic',lambda:straight.t):
        Mission.drive_line(straight,(0.,0.),(.6,0.))
    assert math.dist(straight.amcl_position(),(.6,0.))<=params['center_tolerance']
    assert all(abs(v)<=params['drive_speed']+1e-9 for v,w in straight.commands)
    assert all(abs(b[0]-a[0])<=params['drive_acceleration']*.02+1e-9 for a,b in zip(straight.commands,straight.commands[1:]))
    if initial in [(0.,0.,math.pi),(.75,.1,0.)]:assert any(v<0 for v,w in straight.commands)
f=S(p=params,laser_points=[(-.15,0.)])
assert Mission.blocked(f,-1.) and not Mission.blocked(f,1.)
pid=AnglePID(0.,0.,.05)
assert pid.update(.3,0.,.05)==pid.update(-.3,0.,.05)==0.
assert pid.update(0.,.2,.05)<0.
print('PASS: 24 centres connected; every route step is 60cm; straight runs merged; turns and requested goals retained; forward/reverse/overshoot recovery and rear obstacle checks; measured-rate damping.')
print('A→B→C→I→G:',steps)
