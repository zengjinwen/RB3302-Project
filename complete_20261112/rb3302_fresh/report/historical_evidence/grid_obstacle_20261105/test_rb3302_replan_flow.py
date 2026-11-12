exec(open('/tmp/test_rb3302_obstacle.py').read().split('# Every static')[0])
from rb3302_mission.obstacle_node import ObstacleMission,Replan
class Fake:
 run=ObstacleMission.run
 face_route=ObstacleMission.face_route
 return_to_grid=ObstacleMission.return_to_grid
 def __init__(self,inject):
  self.points=pts;self.planner=planner;self.start='X';self.start_yaw=math.pi/2
  self.goals=['X','U','N','X'];self.obstacles=S(blocked=None);self.p={'approach_speed':.04,'drive_speed':.1}
  self.monitor_route=False;self.inject=inject;self.logs=[];self.position=(*pts['X'],math.pi/2,0.)
 def initialize(self):pass
 def wait_ready(self):pass
 def straight_sample(self):return self.position
 def pause(self):pass
 def command(self,*args):pass
 def halt(self,*args):pass
 def turn_relative(self,*args):pass
 def get_logger(self):return self
 def info(self,x):self.logs.append(x)
 def warning(self,x):self.logs.append(x)
 def drive_line(self,start,end,**kwargs):
  if self.inject and self.monitor_route:
   self.inject=False;self.obstacles.blocked='W'
   self.position=(3.10,.29,math.pi,0.)
   raise Replan()
  self.position=(*end,self.route_heading,0.)
for inject in [False,True]:
 f=Fake(inject);f.run()
 assert [x for x in f.logs if x.startswith('GOAL ')]==['GOAL X','GOAL U','GOAL N','GOAL X']
 assert f.logs[-1]=='MISSION COMPLETE'
 if inject:assert any('重新规划起点' in x for x in f.logs)
print('Mission flow: baseline unchanged; interrupted segment returns to grid, replans around W and completes goals in order.')
