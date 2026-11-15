from pathlib import Path
import numpy as np,json
from planner import Plan
if __name__=='__main__':
 p=Plan();R=np.array([[0,0,1],[0,-1,0],[1,0,0.]])
 L=np.array([.328,.18,1.02]);Q=np.array([.328,-.18,1.02])
 p.add(3.,'approach the handle',{'left':(L+[-.08,0,0],R,.8),'right':(Q+[-.08,0,0],R,.8)},'joint')
 p.add(2.,'enclose the handle',{'left':(L,R,.8),'right':(Q,R,.8)})
 p.add(1.,'hold the handle',{'left':(L,R,.2),'right':(Q,R,.2)})
 for i in range(10):
  p.add(1.6,'roll forward',{'left':(L+[.20,0,0],R,.2),'right':(Q+[.20,0,0],R,.2)})
  p.add(1.6,'roll back',{'left':(L,R,.2),'right':(Q,R,.2)})
 p.add(2.,'bring to rest',{'left':(L,R,.2),'right':(Q,R,.2)})
 p.add(1.,'release handle',{'left':(L,R,.8),'right':(Q,R,.8)})
 p.add(2.,'hands clear',{'left':(L+[-.09,0,.07],R,.8),'right':(Q+[-.09,0,.07],R,.8)})
 p.add(4.,'stationary judging',{})
 print(json.dumps(p.save(Path(__file__).with_name('demo-plan.json'))))
