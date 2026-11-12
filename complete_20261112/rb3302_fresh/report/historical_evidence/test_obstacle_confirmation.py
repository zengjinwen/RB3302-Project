import ast,time
from pathlib import Path
from types import SimpleNamespace as NS
p=Path('/home/rb3302-7/rb3302_fresh/student_ws/src/rb3302_mission/rb3302_mission/node.py')
t=ast.parse(p.read_text());fn=next(n for c in t.body if isinstance(c,ast.ClassDef) and c.name=='Mission' for n in c.body if isinstance(n,ast.FunctionDef) and n.name=='confirm_obstacle')
clock=NS(t=0.);Obstacle=type('Obstacle',(Exception,),{});ns={'time':NS(monotonic=lambda:clock.t),'Obstacle':Obstacle}
exec(compile(ast.Module(body=[fn],type_ignores=[]),'confirm','exec'),ns)
class Sim:
 def __init__(self,sequence):
  self.sequence=sequence;self.i=-1;self.received={'scan':0};self.planner=NS(blocked_cell='B',new_obstacle=lambda *a:self.sequence[min(self.i,len(self.sequence)-1)])
 def command(self):pass
 def get_logger(self):return NS(info=lambda text:None)
 def halt(self,t):
  clock.t+=.13;self.i+=1;self.received['scan']=self.i+1
  assert self.i<20
 def ready(self):return True
 def laser_in_map(self):return []
 def pose(self):return (0,0,0)
s=Sim(['F']*8)
try:ns['confirm_obstacle'](s)
except Obstacle:pass
else:raise AssertionError('Expected replan')
assert s.planner.blocked_cell=='F' and s.i>=4
s=Sim(['F',None]);ns['confirm_obstacle'](s);assert s.planner.blocked_cell=='B'
s=Sim(['F','E','F','E',None]);ns['confirm_obstacle'](s);assert s.planner.blocked_cell=='B'
print('PASS: shared confirmation raises replan only for stable new cell; transient/changing cells preserve old state')
