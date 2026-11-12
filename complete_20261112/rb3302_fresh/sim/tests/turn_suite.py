"""Sequential isolated Gazebo/AMCL tests of both turn experiments."""
import os,sys,time,signal,subprocess,json
from pathlib import Path
root=Path(__file__).resolve().parents[2]
out=root/'report_evidence'/('two_turns_'+time.strftime('%Y%m%d_%H%M%S'));out.mkdir(parents=True)
print('EVIDENCE',out,flush=True)
cases=[('feedback_clear','mission_feedback',''),('rpp_clear','mission_corner_rpp',''),
       ('feedback_obstacle','mission_feedback','Q'),('rpp_obstacle','mission_corner_rpp','Q')]
if len(sys.argv)>1:cases=[case for case in cases if case[0] in sys.argv[1:]]
def stop(p):
 if p.poll() is None:
  os.killpg(p.pid,signal.SIGINT)
  try:p.wait(timeout=12)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
summary=[]
for name,executable,obstacle in cases:
 if obstacle:obstacle=os.environ.get('TEST_OBSTACLE_CELL',obstacle)
 folder=out/name;folder.mkdir()
 env=os.environ.copy();env.update(ROS_DOMAIN_ID='77',ROS_LOCALHOST_ONLY='1',SIM_HEADLESS='1',SIM_RVIZ='0',ROS_LOG_DIR=str(folder/'ros'),TEST_WORLD=str(root/'sim/generated/world.sdf'),TEST_ROOT=str(root),CASE_GOALS='X,P,G,X',RB3302_MISSION=executable,SIM_OBSTACLE=obstacle,SIM_OBSTACLE_SIZE='.58')
 procs=[];began=time.monotonic()
 try:
  with (folder/'measure.log').open('w') as mf,(folder/'launch.log').open('w') as lf:
   measure=subprocess.Popen([sys.executable,str(root/'sim/tests/turn_measure.py'),str(folder/'result.json')],env=env,stdout=mf,stderr=subprocess.STDOUT,start_new_session=True);procs.append(measure)
   sim=subprocess.Popen(['bash',str(root/'sim.sh'),'X','N',env['CASE_GOALS']],env=env,stdout=lf,stderr=subprocess.STDOUT,start_new_session=True);procs.append(sim)
   print('START',name,flush=True)
   while measure.poll() is None and sim.poll() is None and time.monotonic()-began<610:
    time.sleep(2)
    text=(folder/'launch.log').read_text(errors='replace')
    if 'Traceback (most recent call last)' in text or 'Failed to create motion model' in text:break
 finally:
  for p in procs[::-1]:stop(p)
 result=json.loads((folder/'result.json').read_text()) if (folder/'result.json').exists() else {'passed':False,'error':'no result'}
 compact={k:result.get(k) for k in ['passed','completed','elapsed_sim_seconds','collision_frames','amcl_position_rmse','obstacle_seen_in_costmap']}
 compact.update(case=name,wall_seconds=round(time.monotonic()-began,2))
 summary.append(compact);(out/'summary.json').write_text(json.dumps(summary,indent=2))
 print('RESULT',json.dumps(compact),flush=True)
print('FINISHED',out,flush=True)
