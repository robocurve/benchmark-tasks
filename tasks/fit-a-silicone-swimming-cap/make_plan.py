from pathlib import Path
import json,numpy as np
from planner import Plan
from scipy.spatial.transform import Rotation as Rot
from geometry import HEAD,CAP,HEADROT
from scipy.optimize import least_squares
from geometry import POINTS,RIM,M
from pad_covers import configure_kinematics
configure_kinematics()
from yam_arena import ik,station
from yam_arena.constants import READY
from common import down
GRIPROT=Rot.from_euler('z',-90,degrees=True).as_matrix()
RL=Rot.from_euler('y',-30,degrees=True).as_matrix()@down(0)
RR=RL.copy()
a=np.r_[READY,.096,READY,.096]
for side,o,node in [('left',0,RIM[M//4]),('right',7,RIM[3*M//4])]:
 choices=[]
 # Keep the two arms on their respective sides of the forward-facing head.
 R=GRIPROT@Rot.from_euler('y',-20 if side=='left' else 20,degrees=True).as_matrix()@down(0)
 target=POINTS[node]+.012*R[:,2]
 def f(q):
  pos,r=ik.tcp_pose(q,station.BASES[side]);return np.r_[pos-target,.3*ik._rot_error(R,r)]
 for seed in [a[o:o+6],(ik.LOWER+ik.UPPER)/2]:
  sol=least_squares(f,seed,bounds=(ik.LOWER,ik.UPPER),max_nfev=180)
  choices.append((np.linalg.norm(sol.fun),sol.x.copy(),R.copy()))
 error,q,R=min(choices,key=lambda item:item[0]);assert error<.0003,(side,error);a[o:o+6]=q
 if side=='left':RL=R
 else:RR=R
Path(__file__).with_name('initial-action.json').write_text(json.dumps({'action14':a.tolist(),'description':'Both hands hold the separate cap rim at real opposed contacts before frame zero; material is not repositioned.'},indent=2)+'\n')
p=Plan(a)
p.add(2,'hold the loose cap by its rim',{'left':(ik.tcp_pose(a[:6],station.BASES['left'])[0],RL,.075),'right':(ik.tcp_pose(a[7:13],station.BASES['right'])[0],RR,.075)})
def point(dx,z,y=0.):return HEAD+GRIPROT@np.array([dx,y,z])+.012*(RL if dx<0 else RR)[:,2]
p.add(2,'stretch the loose cap opening',{'left':(point(-.10,.105),RL,.075),'right':(point(.10,.105),RR,.075)})
p.add(4,'settle the stretched opening around the crown',{'left':(point(-.10,.105),RL,.075),'right':(point(.10,.105),RR,.075)})
p.add(8,'lower both sides of the rim',{'left':(point(-.085,.050),RL,.075),'right':(point(.085,.050),RR,.075)})
p.add(2,'open both grips in place',{'left':(point(-.085,.050),RL,.17),'right':(point(.085,.050),RR,.17)})
p.add(3,'slide the fingers below the rim',{'left':(point(-.085,.050)+.08*RL[:,2],RL,.17),'right':(point(.085,.050)+.08*RR[:,2],RR,.17)})
p.add(3,'clear the cap laterally',{'left':(point(-.17,-.025),RL,.17),'right':(point(.17,-.025),RR,.17)})
p.add(2,'open fully away from the cap',{'left':(point(-.17,-.025),RL,.7),'right':(point(.17,-.025),RR,.7)})
p.add(4,'inspect after release',{})
print(json.dumps(p.save(Path(__file__).with_name('demo-plan.json'))))
