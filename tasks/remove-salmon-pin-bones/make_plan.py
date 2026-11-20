from pathlib import Path
import numpy as np,json
from planner import Plan
from common import down
from geometry import TIPS,AXIS,TABLE,DISH
from scipy.spatial.transform import Rotation
p=Plan();R=Rotation.from_euler("y",-45,degrees=True).as_matrix()@down(0);L=Rotation.from_euler("x",-45,degrees=True).as_matrix()@down(0)
# Tweezer axis follows the bone; the other hand steadies the local flesh.
left=np.array([.375,TIPS[0][1]+.003,TABLE+.0375])
p.add(3,'approach the fillet',{'left':(left+[0,0,.08],L,.12),'right':(TIPS[0]+[0,0,.08],R,.12)},'joint')
p.add(2,'steady the fillet',{'left':(left,L,.12)})
for i,tip in enumerate(TIPS):
 target=tip+AXIS*.0025
 support=np.array([.375,tip[1]+.003,TABLE+.0375])
 if i:
  p.add(1,'lift the supporting hand',{'left':(left+[0,0,.025],L,.12)})
  p.add(1,'support beside the next bone',{'left':(support,L,.12)})
 left=support
 p.add(2,'approach bone '+str(i+1),{'right':(target+[0,0,.035],R,.12)})
 p.add(2,'place tweezers around bone '+str(i+1),{'right':(target,R,.12)})
 p.add(2,'align tweezers on bone '+str(i+1),{'right':(target,R,.12)})
 p.add(2,'grip bone '+str(i+1),{'right':(target,R,.0215)})
 p.add(3,'draw bone along its direction',{'right':(target+AXIS*.036,R,.0215)})
 p.add(2,'lift the extracted bone',{'right':(target+AXIS*.036+[0,0,.065],R,.0215)})
 p.add(3,'carry the bone to the waste dish',{'right':(DISH+[0,0,.065],R,.0215)})
 p.add(1,'release the bone in the dish',{'right':(DISH+[0,0,.065],R,.15)})
p.add(3,'leave both hands clear',{'left':(left+[.03,.10,.14],L,.6),'right':(DISH+[0,-.03,.15],R,.6)})
p.add(4,'inspect the fillet and recovered bones',{})
print(json.dumps(p.save(Path(__file__).with_name('demo-plan.json'))))
