import ast,importlib.util,math
from pathlib import Path
from types import SimpleNamespace as NS
b=Path('/home/rb3302-7/rb3302_fresh/student_ws/src/rb3302_mission/rb3302_mission')
def load(n):
 s=importlib.util.spec_from_file_location(n,b/(n+'.py'));m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
O=load('obstacles').CellObstacle;g=load('grid')
msg=NS(info=NS(width=100,height=100,resolution=.05,origin=NS(position=NS(x=0.,y=0.))),data=[0]*10000)
points={'A':(1.,1.),'B':(1.6,1.)}
a=[(.85,.88+i*.02) for i in range(13)];bb=[(1.45,.88+i*.02) for i in range(13)]
def detector():return O(msg,points)
def feed(o,scan,t):return o.observe(scan,(1.2,.8),(1.2,.8),t)
o=detector();assert [feed(o,a,t*.125) for t in range(5)]==[False]*4+[True]
for i in range(5,30):assert not feed(o,a+bb,i*.125)
for i in range(30,35):result=feed(o,bb,i*.125)
assert result and o.cell=='B'
for label,scan in [('small',a[:7]),('edge',[(.74,y) for x,y in a]),('scattered',a[:3]+a[-3:])]:
 o=detector()
 for i in range(12):assert not feed(o,scan,i*.125),label
# Same letter is insufficient: an object centroid wandering >5cm must not confirm.
o=detector()
for i in range(12):
 scan=[(x+(.10 if i%2 else 0),y) for x,y in a]
 assert not feed(o,scan,i*.125)
o=detector()
for i in range(12):assert not feed(o,a,1.)
print('Detector passed: duration+position stability, large connected surface, single object persistence/replacement, outside-centre/small/scattered/duplicate rejection.')
p=g.MapPlanner(msg,points);p.blocked_cell='A'
assert not p.edge_clear((.5,1.2),(1.5,1.2)) # crosses area, misses centre
assert p.edge_clear((.5,1.4),(1.5,1.4))
assert p.edge_clear((1.,1.),(1.6,1.)) # exit occupied region
assert not p.edge_clear((1.,1.),(1.1,1.))
print('Full-cell geometry passed: crossing off-centre rejected; external edge and outward escape accepted.')
# Execute actual steering block, including sign in all cardinal directions.
tree=ast.parse((b/'node.py').read_text());method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='drive_line')
loop=next(n for n in method.body if isinstance(n,ast.While));nodes=loop.body
start=next(i for i,n in enumerate(nodes) if isinstance(n,ast.Assign) and ast.unparse(n.targets[0])=='(x, y, yaw)')
end=next(i for i,n in enumerate(nodes) if isinstance(n,ast.Assign) and ast.unparse(n.targets[0])=='error')
code=compile(ast.fix_missing_locations(ast.Module(body=nodes[start:end+1],type_ignores=[])),'steering','exec')
for angle in (0.,math.pi/2,math.pi,-math.pi/2):
 ux,uy=math.cos(angle),math.sin(angle)
 for offset in (-.1,.1):
  pose=[-uy*offset,ux*offset,angle];steering=0.;pid=g.AnglePID(1.5,0.,.05)
  for _ in range(300):
   fake=NS(pose=lambda:pose,p={'cross_track_gain':2.,'lookahead_distance':.3})
   env=dict(self=fake,math=math,wrap=g.wrap,ux=ux,uy=uy,start=(0.,0.),end=(2*ux,2*uy),precise=False,odom_done=False)
   exec(code,env)
   target=g.clamp(pid.update(env['error'],steering,.05),.5)
   steering+=g.clamp(target-steering,.5*.05)
   pose[0]+=.1*math.cos(pose[2])*.05;pose[1]+=.1*math.sin(pose[2])*.05;pose[2]+=steering*.05
  cross=ux*pose[1]-uy*pose[0];assert abs(cross)<.01,(angle,offset,cross)
print('Actual steering block: ideal forward model recovers +/-10cm lateral error for all four headings; hardware behaviour still untested.')
