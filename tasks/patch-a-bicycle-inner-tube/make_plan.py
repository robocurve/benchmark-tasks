from pathlib import Path
import numpy as np
from planner import Plan
from common import down
from model import TARGET,PATCH,HOLD
if __name__=='__main__':
 p=Plan();L=down(-45);R=down()
 a=HOLD-.014*L[:,2]
 b=PATCH+np.array([0,-.006,.0070])-.008*R[:,2]
 p.add(3,'approach tube and patch',{'left':(a+[0,0,.08],L,.7),'right':(b+[0,0,.08],R,.7)},'joint')
 p.add(2,'lower to both objects',{'left':(a,L,.7),'right':(b,R,.7)})
 p.add(1,'steady tube and pinch patch',{'left':(a,L,.025),'right':(b,R,.025)})
 p.add(2,'lift patch',{'right':(b+[0,0,.065],R,.025)})
 dest=TARGET+np.array([0,-.006,.004])-.008*R[:,2]
 p.add(3,'place patch over marked puncture',{'right':(dest+[0,0,.055],R,.025)})
 p.add(2,'lower adhesive face onto rubber',{'right':(dest,R,.025)})
 p.add(1,'release patch',{'right':(dest,R,.55)})
 p.add(1,'clear patch',{'right':(dest+[0,0,.04],R,.55)})
 # Close away from the patch; the contact strokes use finger pads, not a new grip.
 p.add(1,'prepare pad pressure',{'right':(dest+[0,0,.04],R,.12)})
 for x in [-.006,.006]:
  q=TARGET+np.array([x,0,-.001])-R[:,2]*.01284
  p.add(1.2,'press patch evenly',{'right':(q,R,.12)})
  p.add(.8,'hold pressure',{})
 p.add(1,'withdraw pressing hand',{'right':(dest+[0,0,.08],R,.12)})
 p.add(1,'release tube',{'left':(a,L,.7)})
 p.add(2,'clear both hands',{'left':(a+[0,0,.1],L,.8),'right':(dest+[0,0,.1],R,.8)})
 p.add(4,'released seal check',{})
 print(p.save(Path(__file__).with_name('demo-plan.json')))
