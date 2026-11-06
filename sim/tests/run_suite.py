"""Run actual Gazebo cases in isolated ROS domains; preserve every failure."""
import os,sys,json,subprocess,signal,time
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
root=Path(__file__).resolve().parents[2];out=root/'report_evidence'/('matrix_'+time.strftime('%Y%m%d_%H%M%S'));out.mkdir(parents=True,exist_ok=True)
cases=[
 dict(name='01_A_east_clear',start='A',bearing='E',goals='A,F,S,A',env={}),
 dict(name='02_X_west_offset_noise',start='X',bearing='W',goals='X,O,Q,X',env={'SIM_SPAWN_DX':'.04','SIM_SPAWN_DY':'-.02','SIM_SPAWN_YAW':'5','SIM_LIDAR_NOISE':'.02','SIM_GYRO_NOISE':'.02'}),
 dict(name='03_S_north_U35',start='S',bearing='N',goals='S,C,F,S',env={'SIM_OBSTACLE':'U','SIM_OBSTACLE_SIZE':'.35'}),
 dict(name='04_F_west_C40',start='F',bearing='W',goals='F,A,P,F',env={'SIM_OBSTACLE':'C','SIM_OBSTACLE_SIZE':'.40'}),
 dict(name='05_P_east_Q30',start='P',bearing='E',goals='P,L,G,P',env={'SIM_OBSTACLE':'Q','SIM_OBSTACLE_SIZE':'.30'}),
 dict(name='06_X_north_Q40_repeat',start='X',bearing='N',goals='X,P,G,X',env={'SIM_OBSTACLE':'Q','SIM_OBSTACLE_SIZE':'.40'}),
]
def stop(p):
 if p.poll() is None:
  os.killpg(p.pid,signal.SIGINT)
  try:p.wait(timeout=15)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
def run(item):
 index,case=item;name=case['name'];env=os.environ.copy();domain=str(81+index)
 env.update(SIM_DOMAIN=domain,ROS_DOMAIN_ID=domain,ROS_LOCALHOST_ONLY='1',SIM_HEADLESS='1',SIM_RVIZ='0',ROS_LOG_DIR=str(out/'roslogs'),TEST_WORLD=str(root/f'sim/generated/world_{domain}.sdf'))
 env.update(case['env']);result_path=out/(name+'.json')
 (out/(name+'_case.json')).write_text(json.dumps(case,indent=2))
 print('START '+name,flush=True)
 with (out/(name+'_measure.log')).open('w') as mlog,(out/(name+'.log')).open('w') as log:
  measure=subprocess.Popen([sys.executable,str(Path(__file__).with_name('measure.py')),str(result_path)],env=env,stdout=mlog,stderr=subprocess.STDOUT,start_new_session=True)
  sim=subprocess.Popen(['bash',str(root/'sim_clean.sh'),case['start'],case['bearing'],case['goals']],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  try:
   end=time.monotonic()+620
   while measure.poll() is None and sim.poll() is None and time.monotonic()<end:
    time.sleep(2)
    txt=(out/(name+'.log')).read_text(errors='replace')
    if 'Traceback (most recent call last)' in txt:break
  finally:stop(sim);stop(measure)
 result=json.loads(result_path.read_text()) if result_path.exists() else {'completed':False,'error':'Process ended before evidence completed'}
 goals=[x for x in result.get('events',[]) if x['event'].startswith('GOAL')]
 found=[x['event'].removeprefix('Confirmed occupied cell ') for x in result.get('events',[]) if x['event'].startswith('Confirmed')]
 expected=case['env'].get('SIM_OBSTACLE')
 ok=result.get('completed') and len(goals)==4 and result.get('collision_frames')==0 and (found==[expected] if expected else not found)
 result.update(case=case,passed=bool(ok));result_path.write_text(json.dumps(result,indent=2));print(('PASS ' if ok else 'FAIL ')+name,flush=True)
 return result
with ThreadPoolExecutor(max_workers=2) as pool:results=list(pool.map(run,enumerate(cases)))
(out/'SUMMARY.json').write_text(json.dumps(results,indent=2));print('SUITE FINISHED',sum(r['passed'] for r in results),'/',len(results),flush=True)
