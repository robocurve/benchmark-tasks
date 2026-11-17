from pathlib import Path
import numpy as np,json
from scipy.spatial.transform import Rotation
from planner import Plan
from common import down,TABLE
from geometry import Z
p=Plan();RL=Rotation.from_euler('x',-45,degrees=True).as_matrix()@down(0);RR=Rotation.from_euler('x',45,degrees=True).as_matrix()@down(0)
L=np.array([.34,.119,Z])-.008*RL[:,2];R=np.array([.34,.066,Z])-.008*RR[:,2]
p.add(3,'approach the stalk and leaf base',{'left':(L+[0,0,.12],RL,.65),'right':(R+[0,0,.12],RR,.9)},'joint')
p.add(2,'enclose stalk and leaf base',{'left':(L,RL,.65),'right':(R,RR,.9)})
p.add(1,'hold stalk and surround its leaf',{'left':(L,RL,.04),'right':(R,RR,.075)})
p.add(3,'draw stalk through the stripping hand',{'left':(L+[0,.14,.015],RL,.04),'right':(R,RR,.075)})
p.add(3,'finish stripping to the tip',{'left':(L+[0,.29,.025],RL,.04),'right':(R,RR,.075)})
p.add(1,'let the stripped leaf rest on the board',{'right':(R,RR,.8)})
p.add(2,'put the stalk beside the leaf',{'left':(L+[.13,.10,0],RL,.04),'right':(R+[0,-.12,.14],RR,.8)})
p.add(1,'release the separate stalk',{'left':(L+[.13,.10,0],RL,.8)})
p.add(2,'withdraw',{'left':(L+[0,.10,.15],RL,.8)})
p.add(4,'released outcome',{})
print(json.dumps(p.save(Path(__file__).with_name('demo-plan.json'))))
