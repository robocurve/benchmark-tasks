from pathlib import Path
import numpy as np,json
from planner import Plan
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from geometry import POINTS,GRIP_IDS,HEAD,TOWEL
from common import TABLE
from pad_covers import configure_kinematics
configure_kinematics()
from yam_arena import ik,station
from yam_arena.constants import READY
RL=np.array([[0,-1,0],[0,0,-1],[1,0,0.]])
RR=np.array([[0,-1,0],[0,0,1],[-1,0,0.]])
RL=Rotation.from_euler('z',20,degrees=True).as_matrix()@RL
RR=Rotation.from_euler('z',-20,degrees=True).as_matrix()@RR
a=np.r_[READY,.096,READY,.096]
for side,o,i,R in [('left',0,GRIP_IDS[0],RL),('right',7,GRIP_IDS[1],RR)]:
 target=POINTS[i]-R@np.array([0,0,-.006])
 def f(q):
  pp,rr=ik.tcp_pose(q,station.BASES[side]);return np.r_[pp-target,.3*ik._rot_error(R,rr)]
 sols=[least_squares(f,seed,bounds=(ik.LOWER,ik.UPPER),max_nfev=300) for seed in [a[o:o+6],(ik.LOWER+ik.UPPER)/2]]
 sol=min(sols,key=lambda x:np.linalg.norm(x.fun));assert np.linalg.norm(sol.fun)<.0003,(side,sol.fun);a[o:o+6]=sol.x
Path(__file__).with_name('initial-action.json').write_text(json.dumps({'action14':a.tolist()},indent=2)+'\n')
p=Plan(a)
p.add(2,'hold the open towel',{'left':(ik.tcp_pose(a[:6],station.BASES['left'])[0],RL,.065),'right':(ik.tcp_pose(a[7:13],station.BASES['right'])[0],RR,.065)})
for z in [TABLE+.43,TABLE+.35,TABLE+.27]:
 p.add(4,'bring the towel around the next section',{'left':([.36,.09,z],RL,.065),'right':([.36,-.09,z],RR,.065)})
 p.add(4,'wrap the towel against the section',{'left':([.44,.085,z],RL,.065),'right':([.44,-.085,z],RR,.065)})
 p.add(8,'blot without rubbing',{})
 p.add(3,'open the towel clear of the section',{'left':([.32,.09,z],RL,.065),'right':([.32,-.09,z],RR,.065)})
p.add(4,'lower the towel away from the hair',{'left':([.27,.09,TABLE+.25],RL,.065),'right':([.27,-.09,TABLE+.25],RR,.065)})
p.add(2,'release the towel',{'left':([.27,.09,TABLE+.25],RL,.6),'right':([.27,-.09,TABLE+.25],RR,.6)})
p.add(4,'withdraw clear',{'left':([.25,.18,TABLE+.36],RL,.7),'right':([.25,-.18,TABLE+.36],RR,.7)})
p.add(4,'inspect the released result',{})
print(json.dumps(p.save(Path(__file__).with_name('demo-plan.json'))))
