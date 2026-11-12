from pathlib import Path
from types import SimpleNamespace as S
import sys,math,yaml,numpy as np
from PIL import Image
r=Path('/home/rb3302-7/rb3302_fresh');sys.path.insert(0,str(r/'student_ws/src/rb3302_mission'))
from rb3302_mission.grid import MapPlanner
from rb3302_mission.obstacle import CellObstacle
m=r/'student_ws/maps';meta=yaml.safe_load((m/'rb3302_refined_58cm.yaml').read_text());a=np.array(Image.open(m/meta['image']))[::-1];res=meta['resolution'];ox,oy,_=meta['origin'];pts=yaml.safe_load((r/'student_ws/src/rb3302_mission/config/course.yaml').read_text())['landmarks'];msg=S(info=S(resolution=res,width=a.shape[1],height=a.shape[0],origin=S(position=S(x=ox,y=oy))),data=np.where(a==0,100,np.where(a==254,0,-1)).ravel().tolist())
planner=MapPlanner(msg,pts)
# Every static wall endpoint must be rejected; no dynamic obstacle in the map.
hits=[(ox+(ix+.5)*res,oy+(iy+.5)*res) for iy,ix in np.argwhere(a==0)]
for name,point in pts.items():
 detector=CellObstacle(pts,msg)
 assert detector.detect(hits,point,[c for c in pts if c!=name]) is None
# A full-cell front face from each accessible neighbour, versus a small object.
count=0
for target,neighbors in planner.neighbors.items():
 for source in neighbors:
  detector=CellObstacle(pts,msg);cx,cy=pts[target];dx,dy=np.array(pts[source])-pts[target];axis=0 if abs(dx)>abs(dy) else 1;direction=math.copysign(1.,dx if axis==0 else dy)
  def face(width):
   result=[]
   for lateral in np.linspace(-width/2,width/2,41):
    hit=[cx,cy];hit[axis]+=direction*.29;hit[1-axis]+=lateral;result.append(tuple(hit))
   return result
  assert detector.detect(face(.58),pts[source],[target])==target,(source,target)
  assert detector.detect(face(.15),pts[source],[target]) is None
  count+=1
# Confirmation needs elapsed time and fresh scans, not loop iterations.
detector=CellObstacle(pts,msg)
assert not detector.observe('W',1.)
for _ in range(10):assert not detector.observe('W',1.)
for t in [1.125,1.25,1.375]:assert not detector.observe('W',t)
assert detector.observe('W',1.5) and detector.blocked=='W'
for t in [2.,2.125,2.25,2.375]:assert not detector.observe('Q',t)
assert detector.observe('Q',2.5) and detector.blocked=='Q'
assert not detector.observe(None,3.) and detector.blocked=='Q'
# All reachable re-plans exclude the blocked cell; removed old cell is traversable.
checked=0
for blocked in pts:
 for start in pts:
  for goal in pts:
   route=planner.plan(start,goal,blocked)
   assert blocked not in route
   if route:assert route[0]==start and route[-1]==goal
   checked+=1
assert planner.plan('X','U','W') and 'W' not in planner.plan('X','U','W')
assert planner.plan('X','W','W')==[]
assert planner.plan('X','U','Q')
print('No-obstacle wall rejection:24 poses; large vs small faces:',count,'directions; fresh/stable/latest-only checks passed; planner cases:',checked)
