from pathlib import Path
import numpy as np,json
from scipy.spatial.transform import Rotation
from planner import Plan
from geometry import BASE,REST,BUTTON,FRAME
from common import down
R=FRAME@np.array([[0,0,-1],[1,0,0],[0,-1,0.]])
RL=Rotation.from_euler('x',30,degrees=True).as_matrix()@np.array([[0,-1,0],[0,0,-1],[1,0,0.]])
p=Plan();G=REST+[0,0,.003]
p.add(3,'approach the free end of the extension',{'right':(G+FRAME@np.array([.07,0,.025]),R,.35),'left':(BUTTON+[0,0,.08],RL,.03)},'joint')
p.add(3,'enclose the distal end',{'right':(G,R,.35)})
p.add(2,'grip the extension',{'right':(G,R,.16)})
p.add(3,'lift the prepared extension',{'right':(G+[0,0,.035],R,.16)})
# Align the matched curved surfaces, then lower under gentle compression.
A=FRAME@Rotation.from_euler('y',-10,degrees=True).as_matrix()@FRAME.T;D=BASE+[0,0,.0032]
p.add(3,'align in front of the lamp opening',{'right':(D+FRAME@np.array([.050,0,.010]),R,.16)})
p.add(3,'enter level above the nail',{'right':(D+[0,0,.010],R,.16)})
p.add(3,'lower level over the prepared nail',{'right':(D+[0,0,.001],R,.16)})
p.add(4,'seat the matched curved tip gently',{'right':(D,R,.16)})
p.add(10,'hold the tip seated before curing',{})
p.add(4,'approach the lamp switch from above',{'left':(BUTTON+[0,0,.08],RL,.03)},'joint')
p.add(2,'press the lamp switch',{'left':(BUTTON+[0,0,.012],RL,.03)})
p.add(2,'start the cure',{'left':(BUTTON+[0,0,.0025],RL,.03)})
p.add(2,'clear the lamp switch',{'left':(BUTTON+[0,.08,.10],RL,.7)})
p.add(42,'hold the seated tip through its cure',{})
p.add(1,'release the extension',{'right':(D,R,.35)})
p.add(3,'withdraw horizontally from the free edge',{'right':(D+FRAME@np.array([.09,0,0]),R,.7)})
p.add(19,'inspect the released bond and proof load',{})
print(json.dumps(p.save(Path(__file__).with_name('demo-plan.json'))))
