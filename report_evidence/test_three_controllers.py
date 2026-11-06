import sys,math,importlib,types
from pathlib import Path as FilePath
from PIL import Image
import yaml
controller_dir=FilePath('/home/rb3302-7/rb3302_fresh/student_ws/src/rb3302_mission/rb3302_mission/controllers')
if not controller_dir.exists():controller_dir=FilePath('/tmp/rb3302_controllers')
pkg=types.ModuleType('controllers');pkg.__path__=[str(controller_dir)];sys.modules['controllers']=pkg
from controllers.geometry import Path,CollisionMap,advance
root=FilePath('/home/rb3302-7/rb3302_fresh/student_ws')
spec=importlib.util.spec_from_file_location('grid',root/'src/rb3302_mission/rb3302_mission/grid.py');g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
config=yaml.safe_load((root/'maps/rb3302_refined_aligned.yaml').read_text());im=Image.open(root/'maps'/config['image']);width,height=im.size
pixels=list(im.getdata());pixels=[pixels[(height-1-y)*width+x] for y in range(height) for x in range(width)]
msg=types.SimpleNamespace(info=types.SimpleNamespace(width=width,height=height,resolution=.05,origin=types.SimpleNamespace(position=types.SimpleNamespace(x=config['origin'][0],y=config['origin'][1]),orientation=types.SimpleNamespace(x=0,y=0,z=0,w=1))),data=[100 if (255-v)/255>.65 else 0 if (255-v)/255<.196 else -1 for v in pixels])
points=yaml.safe_load((root/'src/rb3302_mission/config/course.yaml').read_text())['landmarks']
planner=g.MapPlanner(msg,points);world=CollisionMap(msg)
for name in ['pure_pursuit','stanley','dwa']:
 controller=importlib.import_module('controllers.'+name).Controller()
 pose=(*points['X'],math.pi/2);current='X'
 for goal in ['P','G','X']:
  route=planner.plan(current,goal);path=Path([points[n] for n in planner.turning_points(route)])
  print(name,current,goal,route, 'pathclear',all(world.free(p) for p in path.points),flush=True)
  assert all(world.free(p) for p in path.points)
  progress=0.;previous=(0.,0.);blocked=0
  for step in range(4000):
   if math.dist(pose[:2],points[goal])<=.05:break
   _,progress,_,_=path.project(pose[:2],progress,min(path.length,progress+.35))
   cmd=controller.command(pose,path,progress,previous,.05,lambda v,w:world.safe(pose,v,w,1.5))
   if cmd is None:
    print('BLOCKED',pose,'progress',progress,'of',path.length,flush=True);break
   assert cmd[0]>=0 and abs(cmd[1])<=.50001
   pose=advance(pose,*cmd,.05);previous=cmd
  distance=math.dist(pose[:2],points[goal]);print('result',step,'error',round(distance,3),'yaw',round(math.degrees(pose[2]),1),flush=True)
  assert distance<=.05, (name,current,goal,distance)
  assert previous[0]<.045, ('goal approach too fast',name,previous)
  current=goal
