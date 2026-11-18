"""Contact pickup candidate. Threading and closure require native validation."""
from pathlib import Path
import numpy as np,json
from scipy.spatial.transform import Rotation
from planner import Plan
from common import down,TABLE
from geometry import PIN
p=Plan();RL=Rotation.from_euler('x',10,degrees=True).as_matrix()@np.array([[0,-1,0],[0,0,-1],[1,0,0.]]);RR=Rotation.from_euler('x',45,degrees=True).as_matrix()@down(0)
L=np.array([.34,.111,TABLE+.067])-.006*RL[:,2];R=PIN+.008*RR[:,2]
p.add(3,'approach outside the loose hem and above the pin coil',{'left':(L+[0,.045,0],RL,.25),'right':(R+[0,0,.10],RR,.4)},'joint')
p.add(2,'enclose the turned hem and coil',{'left':(L,RL,.25),'right':(R,RR,.4)})
p.add(1,'grip the two fabric layers and pin',{'left':(L,RL,0.),'right':(R,RR,0.)})
p.add(3,'lift the hem clear of the back of the leg',{'left':(L+[0,0,.060],RL,0.),'right':(R+[0,0,.22],RR,0.)})
p.add(2,'hold for contact inspection',{})
p.add(39,'threading path candidate',{})
p.add(28,'close and release the threaded pin',{})
print(json.dumps(p.save(Path(__file__).with_name('demo-plan.json'))))
