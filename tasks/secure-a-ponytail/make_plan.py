from pathlib import Path
import json,numpy as np
from planner import Plan
from geometry import ELASTIC,HEAD
p=Plan()
RL=np.array([[0,-1,0],[0,0,-1],[1,0,0.]])
RR=np.array([[0,-1,0],[0,0,1],[-1,0,0.]])
L=ELASTIC+[0,.006,.002];R=ELASTIC+[0,-.006,.002]
p.add(3,'approach the plain elastic from its two sides',{'left':(L+[0,.045,0.],RL,.18),'right':(R+[0,-.045,0.],RR,.18)},'joint')
p.add(2,'place the pads around opposite parts of the elastic',{'left':(L,RL,.18),'right':(R,RR,.18)})
p.add(2,'grip the elastic',{'left':(L,RL,.065),'right':(R,RR,.065)})
p.add(1,'lift the elastic clear of the open notches',{'left':(L+[0,0,.015],RL,.065),'right':(R+[0,0,.015],RR,.065)})
p.add(3,'stretch the elastic below the hair tips',{'left':(ELASTIC+[0,.023,.015],RL,.065),'right':(ELASTIC+[0,-.023,.015],RR,.065)})
p.add(4,'place the opening beneath the prepared hair ends',{'left':([.289,.023,HEAD[2]-.235],RL,.065),'right':([.289,-.023,HEAD[2]-.235],RR,.065)})
p.add(7,'bring the prepared hair through the elastic',{'left':([.289,.023,HEAD[2]-.100],RL,.065),'right':([.289,-.023,HEAD[2]-.100],RR,.065)})
p.add(3,'let the elastic gather the ponytail',{'left':([.289,.012,HEAD[2]-.100],RL,.065),'right':([.289,-.012,HEAD[2]-.100],RR,.065)})
p.add(2,'let the gathered band settle',{})
p.add(3,'release the right side while the left steadies the elastic',{'left':([.289,.012,HEAD[2]-.100],RL,.065),'right':([.289,-.012,HEAD[2]-.100],RR,.18)})
p.add(2,'withdraw the right hand',{'right':([.289,-.11,HEAD[2]-.100],RR,.18)})
p.add(3,'let the held elastic settle around the hair',{'left':([.289,.010,HEAD[2]-.100],RL,.065)})
p.add(2,'pause before the final release',{})
p.add(2,'release the left side',{'left':([.289,.010,HEAD[2]-.100],RL,.18)})
p.add(3,'withdraw the left hand',{'left':([.289,.11,HEAD[2]-.100],RL,.18)})
p.add(1,'open clear of the head',{'left':([.289,.11,HEAD[2]-.100],RL,.6),'right':([.289,-.11,HEAD[2]-.100],RR,.6)})
p.add(9,'released inspection and gentle retention test',{})
print(json.dumps(p.save(Path(__file__).with_name('demo-plan.json'))))
