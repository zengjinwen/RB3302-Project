import ast,math
from types import SimpleNamespace as N
from pathlib import Path
p=Path('/home/rb3302-7/rb3302_fresh/student_ws/src/rb3302_mission/rb3302_mission/node.py')
s=p.read_text(); tree=ast.parse(s)
f=next(x for c in tree.body if isinstance(c,ast.ClassDef) for x in c.body if isinstance(x,ast.FunctionDef) and x.name=='drive_line')
ns={'math':math,'time':N(monotonic=lambda:0.),'wrap':lambda a:(a+math.pi)%(2*math.pi)-math.pi,'clamp':lambda x,l:max(-l,min(l,x))}
exec(compile(ast.Module(body=[f],type_ignores=[]),str(p),'exec'),ns)
class Robot:
    def __init__(self):
        self.messages={'odom':N(pose=N(pose=N(position=N(x=0.,y=0.))))}
        self.anchor=(0.,0.,0.); self.local=0.; self.drive_velocity=0.; self.steering=0.
        self.p={'lookahead_distance':.3,'drive_speed':.1,'drive_acceleration':1000.,'drive_turn_limit':.35,'steer_acceleration':.5}
        self.line_pid=N(update=lambda *a:0.)
        self.samples=iter([(.1,.6),(.2,.7),(.61,.5),(.59,.7),(.59,.6)])
        self.commands=[]
    def pause(self,dt):
        o,self.local=next(self.samples); self.messages['odom'].pose.pose.position.x=o
    def ready(self):return True
    def at(self,end):return abs(self.local-end[0])<=.05
    def amcl_position(self):return self.local,0.
    def pose(self):return self.local,0.,0.
    def blocked(self,d):return False
    def odom_yaw_rate(self):return 0.
    def command(self,v=0.,w=0.):self.commands.append(v)
r=Robot(); ns['drive_line'](r,(0.,0.),(.6,0.))
expected=[.07,.04,.03,-.03,0.]
assert all(abs(a-b)<1e-9 for a,b in zip(r.commands,expected)) and len(r.commands)==5,r.commands
print('PASS: AMCL arrived first still +70%; overshoot +40%; odom exits permanently; localization +30%/-30%; both arrived stop.')
