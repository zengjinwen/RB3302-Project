import math,importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
root=Path('/home/rb3302-7/rb3302_fresh/student_ws/src/rb3302_mission/rb3302_mission')
spec=importlib.util.spec_from_file_location('grid',root/'grid.py');g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
msg=NS(info=NS(resolution=.05,width=80,height=80,origin=NS(position=NS(x=-1,y=-1))),data=[0]*6400)
# A real static wall patch, separate from our cell-centred object.
msg.data[40*80+40]=100
points={'A':(0.,0.),'B':(.6,0.),'C':(1.2,0.),'D':(0.,.6),'E':(.6,.6),'F':(1.2,.6)}
p=g.MapPlanner(msg,points)
laser=[(.42,i*.02) for i in range(-6,7)]
assert p.new_obstacle(laser,(0,0))=='B'
assert p.new_obstacle([(.42,0)],(0,0)) is None
assert p.new_obstacle([(1.+i*.005,1.) for i in range(5)],(1.,1.)) is None
assert p.plan('A','C')==['A','B','C']
p.blocked_cell='B';route=p.plan('A','C');assert route==['A','D','E','F','C'],route
assert p.plan('A','B')==[]
assert p.new_obstacle(laser,(0,0)) is None
new=[(1.02,.6+i*.02) for i in range(-6,7)]
assert p.new_obstacle(new,(.6,.6))=='F'
p.blocked_cell='F';assert p.plan('A','C')==['A','B','C']
p.blocked_cell='B';start=p.attach((.05,0.));route=p.plan(start,'C');assert route and 'B' not in route
assert p.clear_observed_cell([(.9,i*.01) for i in range(-4,5)],(0,0))
assert not p.clear_observed_cell(laser,(0,0))
print('PASS: centre obstacle detection, noise/static wall rejection, reroute, latest replaces old, blocked goal waits, virtual start, actual clear rays required')
