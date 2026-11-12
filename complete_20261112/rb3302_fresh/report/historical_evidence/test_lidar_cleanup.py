import os,runpy,tempfile,pathlib,subprocess,time,signal
from launch import LaunchContext
from launch.utilities import perform_substitutions
root=pathlib.Path(tempfile.mkdtemp(prefix='rb3302_lidar_test_'))
bin=root/'bin';bin.mkdir();(root/'lib/rplidar_ros').mkdir(parents=True)
log=root/'events'
for name,body in {
 'systemctl':'#!/bin/sh\necho "service $*" >> "$TEST_EVENTS"\nexit 0\n',
 'sudo':'#!/bin/sh\n[ "$1" = "-n" ] && shift\nexec "$@"\n'}.items():
 p=bin/name;p.write_text(body);p.chmod(0o755)
node=root/'lib/rplidar_ros/rplidar_node'
node.write_text('''#!/usr/bin/env python3
import signal,time,os
with open(os.environ['TEST_EVENTS'],'a') as f:f.write('child_started\\n')
def stop(*args):
 with open(os.environ['TEST_EVENTS'],'a') as f:f.write('child_stopped\\n')
 raise SystemExit(0)
signal.signal(signal.SIGINT,stop)
signal.signal(signal.SIGTERM,stop)
while True:time.sleep(.05)
''');node.chmod(0o755)
mod=runpy.run_path('/home/rb3302-7/rb3302_fresh/robot_ws/src/rplidar_ros/launch/rplidar_launch.py')
fn=mod['generate_launch_description'];fn.__globals__['get_package_prefix']=lambda _:str(root)
action=fn().entities[0];ctx=LaunchContext();cmd=[perform_substitutions(ctx,part) for part in action.cmd]
env=dict(os.environ,PATH=str(bin)+':'+os.environ['PATH'],TEST_EVENTS=str(log))
p=subprocess.Popen(cmd,env=env,start_new_session=True)
try:
 deadline=time.monotonic()+5
 while time.monotonic()<deadline and (not log.exists() or 'child_started' not in log.read_text()):time.sleep(.05)
 assert 'child_started' in log.read_text()
 # Signal the wrapper alone: it must terminate its own child and restore once.
 p.send_signal(signal.SIGTERM);p.wait(timeout=4)
 events=log.read_text();assert events.count('service start rplidar-motor.service')==1,events
 assert events.index('child_stopped')<events.index('service start'),events
 print('PASS lidar wrapper: child exits before service restored; cleanup runs once')
finally:
 if p.poll() is None:os.killpg(p.pid,signal.SIGKILL);p.wait()
