from pathlib import Path
import numpy as np
from planner import Plan
from common import down,TABLE
from model import CENTER,RADIUS,LENGTH,BOARD,rot

if __name__=='__main__':
 p=Plan();R=down();x=CENTER[0]
 # Pads at the middle of the tube, away from the open-end grasp sites.
 press=np.array([x,0.,BOARD+2*RADIUS-.012+.01284])
 p.add(3,'approach peeler',{'right':(press+[0,0,.09],R,.08)},'joint')
 p.add(2,'press compliant tube',{'right':(press,R,.08)})
 for i in range(5):
  p.add(1.6,'roll forward under pressure',{'right':(press+[.06,0,0],R,.08)})
  p.add(1.6,'roll back under pressure',{'right':(press,R,.08)})
 p.add(1,'lift from tube',{'right':(press+[0,0,.10],R,.7)})
 grasp=np.array([x,.046,BOARD+RADIUS+.005])
 p.add(2,'approach open end',{'right':(grasp+[0,0,.07],R,.7)})
 p.add(1.5,'lower round open end',{'right':(grasp,R,.7)})
 p.add(1,'grasp tube end',{'right':(grasp,R,.24)})
 p.add(2,'lift tube',{'right':(grasp+[0,0,.11],R,.24)})
 tipped=rot([1,0,0],np.radians(65))@R
 p.add(2,'tip to empty peeled clove',{'right':(grasp+[0,-.025,.13],tipped,.24)})
 p.add(2,'let clove slide out',{})
 p.add(2,'return tube level',{'right':(grasp+[-.08,0,.08],R,.24)})
 p.add(2,'set empty tube down',{'right':(grasp+[-.08,0,.007],R,.24)})
 p.add(1,'release tube',{'right':(grasp+[-.08,0,.007],R,.8)})
 p.add(2,'clear work',{'right':(grasp+[-.08,0,.10],R,.8)})
 p.add(4,'released final inspection',{})
 print(p.save(Path(__file__).with_name('demo-plan.json')))
