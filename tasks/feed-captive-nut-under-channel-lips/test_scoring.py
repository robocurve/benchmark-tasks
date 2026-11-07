from pathlib import Path
from types import SimpleNamespace
import sys,math,numpy as np
root=Path(__file__).resolve().parent
sys.path[:0]=[str(root/'runtime'),str(root)]
import task
signs=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
for yaw in (-4,0,4):
 angle=math.radians(yaw);q=np.array([0.,0.,math.sin(angle/2),math.cos(angle/2)])
 rotation=task.quat_xyzw_to_matrix(q);origin=np.array([.26,0.,task.TOP+.025])
 points=[(signs*np.asarray(part['size'])/2+part['pos'])@rotation.T+origin for part in task.NUT.parts]
 all_points=np.concatenate(points)
 state=SimpleNamespace(pos=(all_points.min(axis=0)+all_points.max(axis=0))/2,quat_xyzw=q,low=all_points.min(axis=0),high=all_points.max(axis=0),lin_vel=np.zeros(3),ang_vel=np.zeros(3),resting=lambda *args:True)
 assert state.high[2]>task.TOP+.065,'The handle must expose the old whole-object height check'
 assert abs(task.head_top(state)-points[0][:,2].max())<1e-12
 task.reset_history();task._entered=True
 assert task.score({'nut':state})==.5
 task._entered=False
 assert task.score({'nut':state})==0.
print('PASS nut head height and entry requirement')
