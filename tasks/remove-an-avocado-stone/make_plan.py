from pathlib import Path
import numpy as np,json
from planner import Plan
from common import down,TABLE
from geometry import C,PIT
from scipy.spatial.transform import Rotation
p=Plan();RL=Rotation.from_euler('x',-45,degrees=True).as_matrix()@down(0);R=Rotation.from_euler('x',45,degrees=True).as_matrix()@down(0);L=C+[0,.036,0]-.008*RL[:,2];Q=PIT+[0,0,.012]-.008*R[:,2]
p.add(3.,'approach fruit and stone',{'left':(L+[0,0,.12],RL,1.),'right':(Q+[0,0,.12],R,.65)},'joint')
p.add(2.,'enclose skin and protruding stone',{'left':(L,RL,1.),'right':(Q,R,.65)})
p.add(1.,'hold gently',{'left':(L,RL,.42),'right':(Q,R,.20)})
p.add(3.,'lift the stone while supporting fruit',{'left':(L,RL,.42),'right':(Q+[0,0,.10],R,.20)})
p.add(3.,'carry stone to waste dish',{'right':(np.array([.34,-.18,TABLE+.10]),R,.20)})
p.add(2.,'lower over dish',{'right':(np.array([.34,-.18,TABLE+.06]),R,.20)})
p.add(1.,'release fruit and stone',{'left':(L,RL,1.),'right':(np.array([.34,-.18,TABLE+.06]),R,.8)})
p.add(2.,'withdraw hands',{'left':(L+[0,.05,.15],RL,1.),'right':(np.array([.34,-.18,TABLE+.22]),R,.8)})
p.add(4.,'released outcome',{})
print(json.dumps(p.save(Path(__file__).with_name('demo-plan.json'))))
