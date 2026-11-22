from pathlib import Path
import numpy as np,json
from planner import Plan
from common import down
from geometry import *
p=Plan();L=down(0)@Rotation.from_euler('x',-30,degrees=True).as_matrix();R=down(-90)@Rotation.from_euler('x',75,degrees=True).as_matrix();RP=down(-90)@Rotation.from_euler('x',65,degrees=True).as_matrix();OPEN=.38;CLOSE=.28
# Complete and release each row before approaching the next strip.
for j in [0]:
 i=j%2;grip_index=[7,12,16][j];long=ORIGINS[i]+[0,(grip_index-(N-1)/2)*DX+.0015,.0043];cross=ORIGINS[2+j]+ROTATIONS[2+j].apply([0,(N-1)/2*DX,0])-RP@np.array([0,0,.0028])
 targets={'right':(cross+[0,0,.08],RP,OPEN)}
 targets['left']=(long+[0,0,.08],L,OPEN)
 p.add(3,f'{j}:approach',targets,'joint')
 targets={'right':(cross,RP,OPEN)}
 targets['left']=(long,L,OPEN)
 p.add(3,f'{j}:align',targets)
 targets={'right':(cross,RP,CLOSE)}
 targets['left']=(long,L,CLOSE)
 p.add(3,f'{j}:pinch',targets)
 targets={'right':(cross,RP,CLOSE)}
 targets['left']=(long+[0,0,.060],L,CLOSE)
 p.add(4,f'{j}:lift longitudinal',targets)
 p.add(3,f'{j}:lift cross strip',{'right':(cross+[0,0,.015],RP,CLOSE)})
 entry=PIE+[-.070,CROSS_Y[j],.0295];end=PIE+[.0875,CROSS_Y[j],.0295]
 p.add(4,f'{j}:feed onto pie',{'right':(entry,R,CLOSE)})
 p.add(5,f'{j}:draw across',{'right':(end,R,CLOSE)})
 p.add(2,f'{j}:lower cross strip',{'right':(end,R,CLOSE)})
 p.add(2,f'{j}:release cross strip',{'right':(end,R,.45)})
 p.add(3,f'{j}:clear right hand',{'right':(entry+[-.040,0,0],R,.6)})
 p.add(2,f'{j}:retreat right',{'right':(np.array([entry[0]-.040,CROSS_Y[j]+PIE[1],PIE[2]+.09]),R,.6)})
 p.add(2,f'{j}:park right',{'right':(np.array([.20,-.18,PIE[2]+.09]),R,.6)})
 p.add(4,f'{j}:replace longitudinal',{'left':(long+[0,0,.020],L,CLOSE)})
 p.add(2,f'{j}:release longitudinal',{'left':(long+[0,0,.020],L,.45)})
 p.add(2,f'{j}:clear left hand',{'left':(long+[0,0,.08],L,.6)})
from yam_arena import ik,station
from yam_arena.constants import READY
p.add(3,'hands clear',{s:(*ik.tcp_pose(np.asarray(READY),station.BASES[s]),.7) for s in ['left','right']},'joint')
p.add(4,'inspect',{})
print(json.dumps(p.save(Path(__file__).with_name('demo-plan.json'))))
