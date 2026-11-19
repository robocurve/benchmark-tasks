from pathlib import Path
import json,numpy as np
from common import down,TABLE
from geometry import BRUSH,TOOL
from planner import Plan
from scipy.spatial.transform import Rotation as Rot
p=Plan();R=down();RL=down(180);L=BRUSH+[0,.105,.010]; lift=0.
p.add(3,'approach brush handle and comb',{'left':(L+[0,0,.06],RL,.4),'right':(TOOL+[0,0,.06],R,.4)},'joint')
p.add(2,'lower to ordinary handles',{'left':(L,RL,.35),'right':(TOOL,R,.35)})
p.add(2,'close the comb grip and prepare to brace the brush',{'left':(L,RL,.24),'right':(TOOL,R,0.)})
p.add(2,'brace the brush and lift the comb',{'left':(L+[0,0,lift],RL,.24),'right':(TOOL+[0,0,.09],R,0.)})
insert_R=Rot.from_euler('x',-3,degrees=True).as_matrix()@R
insert_z=.0123
p.add(3,'place pin tail in a lane between bristles',{'right':([.3013,-.17,BRUSH[2]+lift+insert_z],insert_R,0.)})
p.add(6,'slide the tail beneath the trapped hair',{'right':([.3013,-.065,BRUSH[2]+lift+insert_z],insert_R,0.)})
tilt=insert_R
p.add(3,'lift the tail and trapped hair clear of the pins',{'right':([.3013,-.065,BRUSH[2]+lift+insert_z+.060],tilt,0.)})
# Loosen each pin lane; then empty the brush over the dish.
for lane in [-.0035,-.0125]:
 p.add(2,'withdraw the tail above the pins',{'right':([BRUSH[0]+lane,-.17,BRUSH[2]+insert_z+.075],insert_R,0.)})
 p.add(2,'lower the tail into the next lane',{'right':([BRUSH[0]+lane,-.17,BRUSH[2]+insert_z],insert_R,0.)})
 p.add(6,'slide beneath the next section of hair',{'right':([BRUSH[0]+lane,-.065,BRUSH[2]+insert_z],insert_R,0.)})
 p.add(3,'lift and loosen that section',{'right':([BRUSH[0]+lane,-.065,BRUSH[2]+insert_z+.060],insert_R,0.)})
p.add(3,'park the comb',{'right':(TOOL+[0,0,.025],R,0.)})
p.add(1,'release the comb',{'right':(TOOL+[0,0,.025],R,.6)})
p.add(2,'clear the comb',{'right':(TOOL+[0,0,.10],R,.7)})
centre=np.array([.43,.15,TABLE+.16]);handle=L-BRUSH
p.add(5,'lift the brush over the dish',{'left':(centre+handle,RL,.24)})
for angle in np.linspace(0,-90,7)[1:]:
 turn=Rot.from_euler('x',angle,degrees=True).as_matrix()
 p.add(2,'turn the brush to empty the loosened hair',{'left':(centre+turn@handle,turn@RL,.24)},'joint')
p.add(3,'let the loosened hair fall into the dish',{})
for angle in np.linspace(-90,0,7)[1:]:
 turn=Rot.from_euler('x',angle,degrees=True).as_matrix()
 p.add(2,'turn the brush upright',{'left':(centre+turn@handle,turn@RL,.24)},'joint')
p.add(5,'return the brush to its mat',{'left':(L,RL,.24)})
p.add(1,'release the brush',{'left':(L,RL,.6)})
p.add(3,'move both hands clear',{'left':(L+[0,0,.10],RL,.7),'right':(TOOL+[0,0,.10],R,.7)})
p.add(4,'released inspection',{})
print(json.dumps(p.save(Path(__file__).with_name('demo-plan.json'))))
