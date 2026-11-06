"""Build Gazebo walls directly from the aligned ROS occupancy map."""
import argparse
import os
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from PIL import Image


def tag(parent, element, text=None, **attrs):
    node = ET.SubElement(parent,element,attrs)
    if text is not None:
        node.text = str(text)
    return node


def box(link, name, pose, size, color, collision=True):
    for kind in (('collision','visual') if collision else ('visual',)):
        node = tag(link,kind,name=name+'_'+kind)
        tag(node,'pose',pose)
        tag(tag(tag(node,'geometry'),'box'),'size',size)
        if kind == 'visual':
            mat = tag(node,'material');tag(mat,'ambient',color);tag(mat,'diffuse',color)


def plugin(parent, filename, name, values=()):
    node = tag(parent,'plugin',filename=filename,name=name)
    for key,value in values:
        tag(node,key,value)
    return node


def rectangles(mask):
    """Merge equal occupied row runs; preserves every occupied map pixel exactly."""
    active, result = {}, []
    for y,row in enumerate(mask+[[]]):
        runs=[];x=0
        while x<len(row):
            if not row[x]:x+=1;continue
            start=x
            while x<len(row) and row[x]:x+=1
            runs.append((start,x))
        for run,first in active.items():
            if run not in runs:result.append((*run,first,y))
        active={run:active.get(run,y) for run in runs}
    return result


def build(root, output, start, bearing, obstacle=None):
    config=yaml.safe_load((root/'student_ws/maps/rb3302_refined_aligned.yaml').read_text())
    image=Image.open(root/'student_ws/maps'/config['image']).convert('L')
    w,h=image.size;pixels=list(image.getdata());res=config['resolution'];ox,oy,angle=config['origin']
    mask=[[(pixels[(h-1-y)*w+x]/255 if config.get('negate',0) else 1-pixels[(h-1-y)*w+x]/255)>config['occupied_thresh'] for x in range(w)] for y in range(h)]
    points=yaml.safe_load((root/'student_ws/src/rb3302_mission/config/course.yaml').read_text())['landmarks']
    directions={'E':0,'NE':math.pi/4,'N':math.pi/2,'NW':3*math.pi/4,'W':math.pi,'SW':-3*math.pi/4,'S':-math.pi/2,'SE':-math.pi/4}
    if start not in points or bearing not in directions:raise ValueError('Unknown start/bearing')
    if obstacle and obstacle not in points:raise ValueError('Unknown obstacle cell')
    sdf=ET.Element('sdf',version='1.9');world=tag(sdf,'world',name='rb3302')
    physics=tag(world,'physics',name='physics',type='ignored');tag(physics,'max_step_size',.001);tag(physics,'real_time_factor',1.)
    tag(world,'gravity','0 0 -9.81')
    for stem,system in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('imu','Imu')]:
        plugin(world,'gz-sim-'+stem+'-system','gz::sim::systems::'+system)
    plugin(world,'gz-sim-sensors-system','gz::sim::systems::Sensors',[('render_engine','ogre2')])
    light=tag(world,'light',name='sun',type='directional');tag(light,'pose','0 0 8 0 0 0');tag(light,'diffuse','.8 .8 .8 1');tag(light,'direction','-.3 -.4 -1')
    floor=tag(world,'model',name='floor');tag(floor,'static','true');link=tag(floor,'link',name='floor')
    box(link,'floor','2 1 -.05 0 0 0','10 8 .1','.75 .75 .75 1')
    walls=tag(world,'model',name='aligned_map_walls');tag(walls,'static','true');tag(walls,'pose',f'{ox} {oy} 0 0 0 {angle}');link=tag(walls,'link',name='walls')
    rects=rectangles(mask)
    for i,(x0,x1,y0,y1) in enumerate(rects):
        box(link,f'wall_{i}',f'{(x0+x1)*res/2} {(y0+y1)*res/2} .25 0 0 0',f'{(x1-x0)*res} {(y1-y0)*res} .5','.18 .18 .18 1')
    markers=tag(world,'model',name='grid_centres');tag(markers,'static','true');link=tag(markers,'link',name='markers')
    for name,(x,y) in points.items():box(link,'cell_'+name,f'{x} {y} .003 0 0 0','.025 .025 .004','.1 .65 .15 1',False)
    if obstacle:
        x,y=points[obstacle];size=float(os.environ.get('SIM_OBSTACLE_SIZE','.4'));m=tag(world,'model',name='obstacle_'+obstacle);tag(m,'static','true');link=tag(m,'link',name='box');box(link,'box',f'{x} {y} .15 0 0 0',f'{size} {size} .3','.8 .2 .1 1')
    x,y=points[start];x+=float(os.environ.get('SIM_SPAWN_DX','0'));y+=float(os.environ.get('SIM_SPAWN_DY','0'));heading=directions[bearing]+math.radians(float(os.environ.get('SIM_SPAWN_YAW','0')));robot=tag(world,'model',name='rover');tag(robot,'pose',f'{x} {y} .075 0 0 {heading}')
    tag(robot,'self_collide','false');body=tag(robot,'link',name='base_link')
    inertial=tag(body,'inertial');tag(inertial,'mass',5.);inertia=tag(inertial,'inertia')
    for key,value in [('ixx',.018),('iyy',.018),('izz',.033)]:tag(inertia,key,value)
    box(body,'body','0 0 0 0 0 0','.20 .20 .06','.25 .35 .65 1')
    for side,sign in [('left',1),('right',-1)]:
        for end,x in [('front',.065),('rear',-.065)]:
            name=side+'_'+end;wheel=tag(robot,'link',name=name);tag(wheel,'pose',f'{x} {sign*.09} -.042 {math.pi/2} 0 0')
            inertial=tag(wheel,'inertial');tag(inertial,'mass',.15);inertia=tag(inertial,'inertia')
            for key,value in [('ixx',.000045),('iyy',.000045),('izz',.000082)]:tag(inertia,key,value)
            for kind in ('collision','visual'):
                item=tag(wheel,kind,name=name+'_'+kind);cyl=tag(tag(item,'geometry'),'cylinder');tag(cyl,'radius',.033);tag(cyl,'length',.02)
                if kind=='collision':
                    ode=tag(tag(tag(item,'surface'),'friction'),'ode');tag(ode,'mu',1.);tag(ode,'mu2',.1);tag(ode,'fdir1','1 0 0')
            joint=tag(robot,'joint',name=name+'_joint',type='revolute');tag(joint,'parent','base_link');tag(joint,'child',name);axis=tag(joint,'axis');tag(axis,'xyz','0 0 -1')
    drive=plugin(robot,'gz-sim-diff-drive-system','gz::sim::systems::DiffDrive')
    for side in ('left','right'):
        for end in ('front','rear'):tag(drive,side+'_joint',side+'_'+end+'_joint')
    for key,value in [('wheel_separation',.18),('wheel_radius',.033),('topic','/sim/cmd_vel'),('odom_topic','/sim/wheel_odom'),('tf_topic','/sim/wheel_tf_unused'),('frame_id','odom'),('child_frame_id','base_link'),('odom_publish_frequency',100)]:tag(drive,key,value)
    for name,kind,pose,rate,topic in [('imu','imu',f'0 0 .05 {math.pi} 0 0',200,'/sim/imu'),('lidar','gpu_lidar',f'-.052 0 .09 0 0 {math.pi}',10,'/sim/scan')]:
        sensor=tag(body,'sensor',name=name,type=kind);tag(sensor,'pose',pose);tag(sensor,'always_on','true');tag(sensor,'update_rate',rate);tag(sensor,'topic',topic)
        if kind=='imu' and float(os.environ.get('SIM_GYRO_NOISE','0'))>0:
            angular=tag(tag(sensor,'imu'),'angular_velocity')
            for axis in ('x','y','z'):
                noise=tag(tag(angular,axis),'noise',type='gaussian');tag(noise,'mean',0);tag(noise,'stddev',os.environ['SIM_GYRO_NOISE'])
        if kind=='gpu_lidar':
            lidar=tag(sensor,'lidar');horizontal=tag(tag(lidar,'scan'),'horizontal')
            for key,value in [('samples',720),('resolution',1),('min_angle',-math.pi),('max_angle',math.pi)]:tag(horizontal,key,value)
            ranges=tag(lidar,'range')
            for key,value in [('min',.10),('max',12.),('resolution',.01)]:tag(ranges,key,value)
            noise=tag(lidar,'noise');tag(noise,'type','gaussian');tag(noise,'mean',0);tag(noise,'stddev',os.environ.get('SIM_LIDAR_NOISE','.005'))
    plugin(robot,'gz-sim-odometry-publisher-system','gz::sim::systems::OdometryPublisher',[('odom_frame','map'),('robot_base_frame','base_link'),('odom_topic','/sim/ground_truth'),('odom_publish_frequency',30),('dimensions',3)])
    ET.indent(sdf);output.parent.mkdir(parents=True,exist_ok=True);ET.ElementTree(sdf).write(output,encoding='unicode',xml_declaration=True)
    return len(rects),sum(map(sum,mask))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1]);p.add_argument('--output',type=Path,required=True);p.add_argument('--start',default='X');p.add_argument('--bearing',default='N');p.add_argument('--obstacle',default=None)
    a=p.parse_args();rects,cells=build(a.root,a.output,a.start,a.bearing,a.obstacle);print(f'World: {a.output}; {cells} occupied pixels -> {rects} wall boxes.')
