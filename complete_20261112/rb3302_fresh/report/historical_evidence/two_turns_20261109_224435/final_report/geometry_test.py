import ast, math, sys, importlib.util
from pathlib import Path
root=Path('/home/rb3302-7/rb3302_fresh')
code=root/'student_ws/src/rb3302_mission/rb3302_mission'
for p in code.glob('*.py'): ast.parse(p.read_text(),filename=str(p))
for name in ['node.py','obstacle_node.py','obstacle.py','robot.py','grid.py']:
    assert (code/name).read_bytes()==(root/'backups/before_two_turn_versions_20261109_223644/rb3302_mission/rb3302_mission'/name).read_bytes(),name
spec=importlib.util.spec_from_file_location('geom',code/'corner_geometry.py'); g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
# Exercise actual rate controller and actual corner follower without ROS hardware.
ns={'math':math,'clamp':lambda x,n:max(-n,min(n,x))}
tree=ast.parse((code/'turn_feedback.py').read_text())
exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='RateTrim'],type_ignores=[]),'rate','exec'),ns)
RateTrim=ns['RateTrim']
r=RateTrim()
assert r.update(0.,2.,.02)==0.
assert r.update(.6,.2,.02)>r.update(.6,1.2,.02)
assert r.update(-.6,-.2,.02)<0.
assert r.update(.6,None,.02)==.6
for _ in range(1000): assert abs(r.update(1.8,0.,.02))<=3.
class Base: pass
ns.update(FeedbackMission=Base,RateTrim=RateTrim,wrap=lambda a:math.atan2(math.sin(a),math.cos(a)))
ns.update({k:getattr(g,k) for k in ['turn_windows','lengths','project','point_at','pursuit']})
tree=ast.parse((code/'corner_rpp.py').read_text())
exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n,ast.ClassDef)],type_ignores=[]),'corner','exec'),ns)
class Log:
    def info(self,*args): pass
class Fake(ns['CornerMission']):
    def __init__(self,points):
        self.points={str(i):p for i,p in enumerate(points)}
        self.p={'drive_speed':.24,'approach_speed':.07,'slowdown_distance':.15,'turn_speed':1.8}
        self.turn_calibration=None
        self.x,self.y=points[0]
        self.heading=math.atan2(points[1][1]-self.y,points[1][0]-self.x)
        self.t=0.; self.v=self.w=0.; self.trace=[]
    def now(self): return self.t
    def get_logger(self): return Log()
    def straight_sample(self): return self.x,self.y,self.heading,self.w
    def pause(self):
        if self.t>90: raise AssertionError('corner did not complete')
        self.t+=.02
        self.x+=self.v*math.cos(self.heading)*.02
        self.y+=self.v*math.sin(self.heading)*.02
        self.heading+=self.w*.02
        self.trace.append((self.x,self.y))
    def command(self,v=0.,w=0.):
        assert v>=0.,'unexpected reverse'
        self.v,self.w=v,w
cases=[[(0,0),(.58,0),(.58,.58)],[(0,0),(.58,0),(.58,-.58)],[(0,0),(.58,0),(.58,.58),(1.16,.58)],[(0,0),(.58,0),(.58,.58),(0,.58)]]
for points in cases:
    bot=Fake(points);cells=list(bot.points)
    actions=g.turn_windows(cells,bot.points)
    assert actions==[('rpp',0,len(cells)-1)],actions
    bot.follow_corner(cells,[],cells[-1])
    error=math.dist((bot.x,bot.y),points[-1])
    assert error<.05,(points,error)
    assert bot.v==bot.w==0.
    print('ideal kinematic corner:',len(points),'points',round(bot.t,2),'seconds endpoint error',round(error,4))
points={str(i):p for i,p in enumerate([(0,0),(.58,0),(1.16,0),(1.16,.58),(1.16,1.16)])}
assert g.turn_windows(list(points),points)==[('line',0,1),('rpp',1,3),('line',3,4)]
assert g.turn_windows(['0'],points)==[]
print('PASS baseline unchanged, geometry windows, rate feedback signs/braking/stop and ideal corner tracking. NOT Gazebo or real hardware.')
