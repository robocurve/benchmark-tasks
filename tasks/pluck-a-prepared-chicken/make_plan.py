from pathlib import Path
import numpy as np,json
from planner import Plan
from common import down
from geometry import TIPS,AXIS,TABLE,C,breast_z
from scipy.spatial.transform import Rotation
p=Plan();R=Rotation.from_euler('y',-30,degrees=True).as_matrix()@down(0);L=Rotation.from_euler('x',-45,degrees=True).as_matrix()@down(0)
left=C+[.012,-.0485,breast_z(.005,-.045)+.0095]
p.add(3,'approach the prepared breast',{'left':(left+[0,0,.035],L,.15),'right':(TIPS[1]+AXIS*.035,R,.15)},'joint')
p.add(2,'support skin beside feathers',{'left':(left,L,.15)})
for group in range(3):
 target=TIPS[group*3+1]+AXIS*.0015
 support=C+[.012,(group-1.5)*.030-.0035,breast_z(.005,(group-1.5)*.030)+.0095]
 if group:p.add(1,'lift supporting fingers',{'left':(left+[0,0,.025],L,.15)})
 p.add(1,'support skin beside group '+str(group+1),{'left':(support,L,.15)})
 left=support
 p.add(2,'approach group '+str(group+1),{'right':(target+AXIS*.035,R,.15)})
 p.add(2,'place fingers around group '+str(group+1),{'right':(target,R,.15)})
 p.add(1,'align group '+str(group+1),{'right':(target,R,.15)})
 p.add(2,'grip group '+str(group+1),{'right':(target,R,.050)})
 p.add(2,'pluck along the quills',{'right':(target+AXIS*.026,R,.050)})
 p.add(2,'lift the freed feathers',{'right':(target+AXIS*.026+[0,0,.05],R,.050)})
 p.add(2,'carry feathers to clear tray area',{'right':(np.array([.23,-.13,TABLE+.09]),R,.050)})
 p.add(1,'release the feathers',{'right':(np.array([.23,-.13,TABLE+.09]),R,.3)})
p.add(3,'leave hands clear',{'left':(left+[0,.10,.055],L,.7),'right':(np.array([.24,-.18,TABLE+.22]),R,.7)})
p.add(4,'inspect released carcass',{})
print(json.dumps(p.save(Path(__file__).with_name('demo-plan.json'))))
