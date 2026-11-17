from pathlib import Path
import numpy as np,json
from scipy.spatial.transform import Rotation
from planner import Plan
from common import down,TABLE
from geometry import PAPER,SPONGE,ARM,ARM_RADIUS
p=Plan();RL=Rotation.from_euler('x',10,degrees=True).as_matrix()@np.array([[0,-1,0],[0,0,-1],[1,0,0.]]);RR=Rotation.from_euler('x',45,degrees=True).as_matrix()@down(0)
L=PAPER+[0,.0275,0]-.008*RL[:,2];Q=SPONGE-.008*RR[:,2]
p.add(3,'approach transfer paper and wet sponge',{'left':(L+[0,0,.04],RL,.3),'right':(Q+[0,0,.10],RR,.7)},'joint')
p.add(2,'enclose the overhanging paper edge and sponge',{'left':(L,RL,.3),'right':(Q,RR,.7)})
p.add(1,'hold paper edge and sponge',{'left':(L,RL,0.),'right':(Q,RR,.35)})
p.add(2,'lift the paper',{'left':(L+[0,0,.10],RL,0.),'right':(Q+[0,0,.10],RR,.35)})
T=ARM+[0,.0275,.028]-.008*RL[:,2];S=ARM+[0,0,ARM_RADIUS+.018]-.008*RR[:,2]
p.add(3,'place image against the curved form',{'left':(T,RL,0.),'right':(S+[0,0,.05],RR,.35)})
p.add(2,'press the wet sponge onto the backing',{'right':(S+[0,0,-.008],RR,.35)})
p.add(45,'hold even wet contact',{})
p.add(2,'lift sponge after wetting',{'right':(S+[0,0,.07],RR,.35)})
p.add(1,'release the blank margin for regripping',{'left':(T,RL,.25)})
p.add(2,'enclose the wet backing margin',{'left':(T,RL,.25)})
p.add(1,'grip the wet backing margin',{'left':(T,RL,0.)})
p.add(4,'peel the backing steadily',{'left':(T+[0,.07,.06],RL,0.)})
p.add(3,'return sponge and removed backing',{'left':(L+[0,0,.04],RL,0.),'right':(Q,RR,.35)})
p.add(1,'release tools',{'left':(L+[0,0,.04],RL,.7),'right':(Q,RR,.7)})
p.add(2,'withdraw',{'left':(L+[-.05,0,.07],RL,.7),'right':(Q+[0,0,.12],RR,.7)})
p.add(4,'released outcome',{})
print(json.dumps(p.save(Path(__file__).with_name('demo-plan.json'))))
