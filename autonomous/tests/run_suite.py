"""Reproducible Gazebo integration tests, with no ground-truth navigation input."""
import os,sys,json,time,signal,subprocess,threading,hashlib
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
root=Path(os.environ.get('TEST_ROOT',str(Path(__file__).resolve().parents[1])))
out=root/'evidence'/time.strftime('%Y%m%d_%H%M%S');out.mkdir(parents=True)
stopping=threading.Event()
signal.signal(signal.SIGINT,lambda *args:stopping.set())
signal.signal(signal.SIGTERM,lambda *args:stopping.set())
(out/'CODE_SHA256.json').write_text(json.dumps({str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in root.iterdir() if p.suffix in ('.py','.yaml','.xml','.urdf','.sh')},indent=2))
cases=[
 {'name':'01_baseline','start':'X','bearing':'N','goals':'X,O,Q,X','env':{}},
 {'name':'02_blocked_Q','start':'X','bearing':'N','goals':'X,P,G,X','env':{'SIM_OBSTACLE':'Q','SIM_OBSTACLE_SIZE':'.4'}},
 {'name':'03_blocked_C','start':'F','bearing':'W','goals':'F,A,P,F','env':{'SIM_OBSTACLE':'C','SIM_OBSTACLE_SIZE':'.4'}},
 {'name':'04_diagonal_offset','start':'X','bearing':'NW','goals':'O,Q,P,X','env':{'SIM_SPAWN_DX':'.10','SIM_SPAWN_DY':'-.10','SIM_SPAWN_YAW':'35','SIM_LIDAR_NOISE':'.02','SIM_GYRO_NOISE':'.02'}},
 {'name':'05_other_start','start':'A','bearing':'SE','goals':'F,C,S,A','env':{'SIM_SPAWN_DX':'.08','SIM_SPAWN_DY':'-.06','SIM_SPAWN_YAW':'-35'}},
 {'name':'06_small_obstacle','start':'P','bearing':'E','goals':'P,L,G,P','env':{'SIM_OBSTACLE':'Q','SIM_OBSTACLE_SIZE':'.3'}},
]
def stop(p):
 if p.poll() is None:
  os.killpg(p.pid,signal.SIGINT)
  try:p.wait(timeout=12)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
def run(item):
 i,case=item
 if stopping.is_set():return {'case':case,'passed':False,'completed':False,'reason':'Cancelled'}
 domain=str(92+i);name=case['name'];env=os.environ.copy()
 env.update(TEST_ROOT=str(root.parent),SIM_DOMAIN=domain,ROS_DOMAIN_ID=domain,ROS_LOCALHOST_ONLY='1',SIM_HEADLESS='1',NAV_RVIZ='0',ROS_LOG_DIR=str(out/(name+'_ros')),TEST_WORLD=str(root/f'generated/world_{domain}.sdf'),CASE_GOALS=case['goals'])
 env.update(case['env']);dest=out/(name+'.json');(out/(name+'_case.json')).write_text(json.dumps(case,indent=2));print('START',name,flush=True)
 procs=[]
 try:
  with (out/(name+'.log')).open('w') as f,(out/(name+'_measure.log')).open('w') as mf:
   measure=subprocess.Popen([sys.executable,str(root/'tests/evaluate.py'),str(dest)],env=env,stdout=mf,stderr=subprocess.STDOUT,start_new_session=True);procs.append(measure)
   sim=subprocess.Popen(['bash',str(root/'run.sh'),'sim',case['start'],case['bearing'],case['goals']],env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);procs.append(sim)
   deadline=time.monotonic()+620
   while not stopping.is_set() and measure.poll() is None and sim.poll() is None and time.monotonic()<deadline:
    time.sleep(2)
    if 'Traceback (most recent call last)' in (out/(name+'.log')).read_text(errors='replace'):break
 finally:
  for p in procs[::-1]:stop(p)
 result=json.loads(dest.read_text()) if dest.exists() else {'passed':False,'completed':False,'reason':'No evaluation result'}
 result['case']=case;dest.write_text(json.dumps(result,indent=2));print('PASS' if result['passed'] else 'FAIL',name,'elapsed',result.get('elapsed_sim_seconds'),'collisions',result.get('collision_frames'),flush=True)
 return result
if __name__=='__main__':
 # One Gazebo at a time by default: avoid CPU contention changing sensor timing.
 selected=[(i,c) for i,c in enumerate(cases) if len(sys.argv)==1 or c['name'] in sys.argv[1:]]
 with ThreadPoolExecutor(max_workers=int(os.environ.get('TEST_WORKERS','1'))) as pool:results=list(pool.map(run,selected))
 (out/'SUMMARY.json').write_text(json.dumps(results,indent=2));print('RESULTS',out,'PASS',sum(bool(r['passed']) for r in results),'/',len(results),flush=True)
