from pathlib import Path
import numpy as np,json
from scipy.spatial.transform import Rotation
from planner import Plan
from common import down,TABLE
p=Plan();R=Rotation.from_euler('x',45,degrees=True).as_matrix()@down(0);GRIP=np.array([.34,-.17,TABLE+.140])-.008*R[:,2]
p.add(3,'approach masher handle',{'right':(GRIP+[0,0,.10],R,.5)},'joint')
p.add(2,'enclose handle',{'right':(GRIP,R,.5)})
p.add(1,'grip handle',{'right':(GRIP,R,.05)})
p.add(2,'lift masher',{'right':(GRIP+[0,0,.09],R,.05)})
C=GRIP+[0,.17,0]
p.add(2,'move above potato',{'right':(C+[0,0,.13],R,.05)})
local_head=R.T@(np.array([.34,-.17,TABLE+.003])-GRIP)
for i in range(20):
 Ri=R
 Ci=np.array([.34,0,TABLE+.003])-Ri@local_head
 p.add(.75,'position over potato '+str(i),{'right':(Ci+[0,0,.040],Ri,.05)})
 p.add(2.,'press potato '+str(i),{'right':(Ci+[0,0,-.004],Ri,.05)})
 p.add(1.,'hold compressive load '+str(i),{'right':(Ci+[0,0,-.004],Ri,.05)})
 p.add(1.,'lift for next press '+str(i),{'right':(Ci+[0,0,.040],Ri,.05)})
p.add(2,'let loose potato fall back into the bowl',{})
p.add(2,'return masher',{'right':(GRIP+[0,0,.12],R,.05)})
p.add(2,'lower masher to bench',{'right':(GRIP,R,.05)})
p.add(1,'release masher',{'right':(GRIP,R,.6)})
p.add(2,'withdraw',{'right':(GRIP+[0,0,.09],R,.6)})
p.add(4,'released outcome',{})
print(json.dumps(p.save(Path(__file__).with_name('demo-plan.json'))))
