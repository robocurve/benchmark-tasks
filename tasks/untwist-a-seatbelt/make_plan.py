"""Fixed joint plan: grip the tongue, turn it through the trapped half twist, release."""
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from planner import Plan
from common import down
from model import Model,CENTER
from installation import world,R as INSTALL_R
if __name__=='__main__':
 Model.configure_station();m=Model();p=Plan();R=Rotation.from_rotvec(INSTALL_R[:,1]*np.radians(315)).as_matrix()@np.column_stack((INSTALL_R[:,0],INSTALL_R[:,2],-INSTALL_R[:,1]));centre=world(CENTER)
 grip=world(m.pull_point())-.008*R[:,2]
 p.add(3,'approach the tongue beside the seat',{'right':(grip+[-.03,0,.03],R,.5)},'joint')
 p.add(2,'enclose the metal tongue',{'right':(grip,R,.5)})
 p.add(1,'grip the tongue',{'right':(grip,R,.025)})
 for k in range(1,25):
  Q=Rotation.from_rotvec(INSTALL_R[:,1]*(np.pi*k/24)).as_matrix()
  target=centre+Q@(grip-centre)
  p.add(16/24,'turn the tongue and feed out the trapped twist',{'right':(target,Q@R,.025)})
 p.add(1,'hold the straightened belt',{})
 p.add(1,'release the tongue',{'right':(target,Q@R,.8)})
 p.add(2,'withdraw without disturbing the belt',{'right':(target+[-.08,-.04,.02],Q@R,.8)})
 p.add(4,'released flatness check',{})
 print(p.save(Path(__file__).with_name('demo-plan.json')))
