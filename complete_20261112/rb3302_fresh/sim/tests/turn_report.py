from pathlib import Path
import json,csv,math,re
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image
import yaml
root=Path('/home/rb3302-7/rb3302_fresh')
first=root/'report_evidence/two_turns_20261109_224435'
import sys
last=Path(sys.argv[1]) if len(sys.argv)>1 else first
out=first/'final_report';out.mkdir(exist_ok=True)
config=yaml.safe_load((root/'student_ws/maps/rb3302_refined_58cm.yaml').read_text())
image=Image.open(root/'student_ws/maps'/config['image']);res=config['resolution'];ox,oy,_=config['origin']
points=yaml.safe_load((root/'student_ws/src/rb3302_mission/config/course.yaml').read_text())['landmarks']
fig,axes=plt.subplots(2,2,figsize=(11,8));rows=[]
for ax,name in zip(axes.flat,['feedback_clear','rpp_clear','feedback_obstacle','rpp_obstacle']):
 folder=(last/name) if (last/name/'result.json').exists() else first/name
 d=json.loads((folder/'result.json').read_text())
 trace=[[float(v) for v in r] for r in list(csv.reader((folder/'result.csv').open()))[1:]]
 detected=[re.search('确认整格障碍：(.)',e['event'])[1] for e in d['events'] if e['event'].startswith('确认整格障碍：')]
 false_cells=sorted(set(detected)-({'Q'} if 'obstacle' in name else set()))
 row=dict(case=name,elapsed_seconds=d['elapsed_sim_seconds'],completed=d['completed'],collision_frames=d['collision_frames'],maximum_goal_error_m=max(e['true_goal_error'] for e in d['goals']),false_blocked_cells=false_cells,detected_actual_obstacle=('Q' in detected) if 'obstacle' in name else None,within_180_seconds=d['elapsed_sim_seconds'] is not None and d['elapsed_sim_seconds']<=180,base_checks_passed=d['passed'],source=str(folder))
 row['overall_passed']=d['passed'] and not false_cells and (row['detected_actual_obstacle'] is not False)
 rows.append(row)
 ax.imshow(image,cmap='gray',origin='upper',extent=[ox,ox+image.width*res,oy,oy+image.height*res],vmin=0,vmax=255)
 ax.plot([r[1] for r in trace],[r[2] for r in trace],color='#0077bb',linewidth=1.5,label='Gazebo truth')
 for letter,(x,y) in points.items():ax.text(x,y,letter,fontsize=7,ha='center',va='center',color='#117733')
 if 'obstacle' in name:
  x,y=points['Q'];ax.add_patch(plt.Rectangle((x-.29,y-.29),.58,.58,color='#ee7733',alpha=.6))
 for e in d['goals']:
  ax.plot(e['truth'][1],e['truth'][2],'ro',markersize=3)
 ax.set_xlim(-.1,3.6);ax.set_ylim(-.1,2.45);ax.set_aspect('equal');ax.set_title(f"{name}: {d['elapsed_sim_seconds'] or 0:.1f}s, overlaps={d['collision_frames']}");ax.set_xlabel('map x (m)');ax.set_ylabel('map y (m)')
fig.tight_layout();fig.savefig(out/'trajectories.png',dpi=160);plt.close(fig)
(out/'summary.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2))
lines=['Gazebo tests: X (North), goals X,P,G,X; obstacle Q = 0.58 x 0.58m box.', 'Real AMCL and EKF; truth used only for evaluation. 5ms physics step.', 'Gazebo wheel response/friction is not a measured replica of the physical rover.', 'Collision frames are planar 20cm-square body overlap samples, not contact-force measurements.', 'Goal acceptance for test: goal centre under body, not a 5cm accuracy guarantee.','']
for row in rows:lines.append(json.dumps(row,ensure_ascii=False))
(out/'RESULTS.txt').write_text('\n'.join(lines)+'\n')
print(json.dumps(rows,ensure_ascii=False,indent=2))
print(out)
