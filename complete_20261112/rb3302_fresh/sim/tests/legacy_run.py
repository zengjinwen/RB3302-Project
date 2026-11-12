import os,sys,time,signal,subprocess,json
from pathlib import Path
root=Path('/home/rb3302-7/rb3302_fresh');out=root/'report_evidence'/('legacy_turn_'+time.strftime('%Y%m%d_%H%M%S'));out.mkdir(parents=True)
case=sys.argv[1] if len(sys.argv)>1 else 'baseline'
env=os.environ.copy();env.update(ROS_DOMAIN_ID='77',ROS_LOCALHOST_ONLY='1',SIM_HEADLESS='1',SIM_RVIZ='0',ROS_LOG_DIR=str(out/'ros'),TEST_WORLD=str(root/'sim/generated/world.sdf'),TEST_ROOT=str(root),CASE_GOALS='X,O,Q,X')
if case=='obstacle':env.update(SIM_OBSTACLE='Q',SIM_OBSTACLE_SIZE='.4',CASE_GOALS='X,P,G,X')
procs=[]
def stop(p):
 if p.poll() is None:
  os.killpg(p.pid,signal.SIGINT)
  try:p.wait(timeout=10)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
try:
 with (out/'measure.log').open('w') as mf,(out/'launch.log').open('w') as lf:
  measure=subprocess.Popen([sys.executable,str(root/'sim/tests/legacy_measure.py'),str(out/'result.json')],env=env,stdout=mf,stderr=subprocess.STDOUT,start_new_session=True);procs.append(measure)
  sim=subprocess.Popen(['bash',str(root/'sim.sh'),'X','N',env['CASE_GOALS']],env=env,stdout=lf,stderr=subprocess.STDOUT,start_new_session=True);procs.append(sim)
  print('START',case,out,flush=True);end=time.monotonic()+450
  while measure.poll() is None and sim.poll() is None and time.monotonic()<end:
   time.sleep(2)
   if 'Traceback (most recent call last)' in (out/'launch.log').read_text(errors='replace'):break
finally:
 for p in procs[::-1]:stop(p)
if (out/'result.json').exists():
 d=json.loads((out/'result.json').read_text());print({k:d[k] for k in ('passed','completed','elapsed_sim_seconds','collision_frames')},flush=True)
 print('turns',d['turns'],flush=True)
print('RESULTS',out,flush=True)
