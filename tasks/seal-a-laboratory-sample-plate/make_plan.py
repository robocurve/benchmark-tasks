from pathlib import Path
import numpy as np,json
from planner import Plan
from common import down
from scipy.spatial.transform import Rotation
PICK_R=down(0)@Rotation.from_euler("x",50,degrees=True).as_matrix()
ROLL_R=Rotation.from_euler("z",180,degrees=True).as_matrix()@Rotation.from_euler("x",-90,degrees=True).as_matrix()@PICK_R
from geometry import *
p=Plan();L=np.array([[0,-1,0],[0,0,-1],[1,0,0]]);R=np.array([[0,1,0],[0,0,1],[1,0,0]])
a=FILM+[DX/2,(NY-1)/2*DY+.006,0];b=FILM+[DX/2,-(NY-1)/2*DY-.006,0]
p.add(4,'approach film edges',{'left':(a+[0,.02,0],L,.12),'right':(b+[0,-.02,0],R,.12)},'joint')
p.add(3,'put fingers around film edges',{'left':(a,L,.08),'right':(b,R,.08)})
p.add(2,'pinch film edges',{'left':(a,L,0.),'right':(b,R,0.)})
p.add(3,'lift film clear of release backing',{'left':(a+[0,0,.065],L,0.),'right':(b+[0,0,.065],R,0.)})
c=PLATE+[DX/2,(NY-1)/2*DY+.006,.0295];d=PLATE+[DX/2,-(NY-1)/2*DY-.006,.0295]
p.add(4,'carry film over the sample plate',{'left':(c+[0,0,.065],L,0.),'right':(d+[0,0,.065],R,0.)})
p.add(3,'place loose film on the well rims',{'left':(c,L,0.),'right':(d,R,0.)})
p.add(2,'release film edges',{'left':(c,L,.04),'right':(d,R,.04)})
p.add(2,'withdraw sideways from film',{'left':(c+[0,.020,0],L,.04),'right':(d+[0,-.020,0],R,.04)})
p.add(2,'lift hands clear of film',{'left':(c+[0,.020,.065],L,.6),'right':(d+[0,-.020,.065],R,.6)})
p.add(4,'approach roller handle',{'right':(TOOL+[0,0,.08],PICK_R,.4)},'joint')
p.add(2,'settle above roller handle',{'right':(TOOL+[0,0,.08],PICK_R,.4)})
p.add(3,'put fingers around roller handle',{'right':(TOOL,PICK_R,.4)})
p.add(2,'grip roller handle',{'right':(TOOL,PICK_R,.08)})
p.add(3,'lift roller',{'right':(TOOL+[0,0,.10],PICK_R,.08)})
# Roller bottom = handle Z - 60 mm; 1 mm rubber compression gives about 0.8 N.
for lane,x in enumerate([-.040,0,.040]):
 target=PLATE+[x,-.040,.017+.060-.001]
 p.add(3,'approach rolling lane '+str(lane),{'right':(target+[0,0,.035],ROLL_R,.10)},'joint')
 p.add(2,'load roller lane '+str(lane),{'right':(target,ROLL_R,.10)})
 target[1]=PLATE[1]+.040
 p.add(6,'press rolling lane '+str(lane),{'right':(target,ROLL_R,.10)})
 p.add(2,'lift roller from lane '+str(lane),{'right':(target+[0,0,.035],ROLL_R,.10)})
p.add(3,'return roller above table',{'right':(TOOL+[0,0,.10],PICK_R,.08)})
p.add(3,'put roller on table',{'right':(TOOL,PICK_R,.08)})
p.add(2,'release roller',{'right':(TOOL,PICK_R,.5)})
from yam_arena import ik,station
from yam_arena.constants import READY
clear={side:(*ik.tcp_pose(np.asarray(READY),station.BASES[side]),.6) for side in ['left','right']}
p.add(3,'leave both hands clear',clear,'joint')
p.add(4,'inspect the individual well seals',{})
print(json.dumps(p.save(Path(__file__).with_name('demo-plan.json'))))
