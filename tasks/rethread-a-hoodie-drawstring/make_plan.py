from pathlib import Path
import numpy as np
from planner import Plan
from common import down
from model import arc,LENGTH

def target(s,side):
 point,t=arc(s-.020);yaw=-90+np.clip(s,0,LENGTH)/.18*180/np.pi
 R=down(yaw+(180 if side=='left' else 0))
 return point-.014*R[:,2],R

if __name__=='__main__':
 p=Plan();a=arc(.48)[0]+[0,0,.014];L=down(180);b,R=target(.015,'right')
 p.add(3,'approach bodkin and casing',{'left':(a+[0,0,.08],L,.7),'right':(b+[0,0,.08],R,.7)},'joint')
 p.add(2,'lower to cloth and bodkin',{'left':(a,L,.7),'right':(b,R,.7)})
 p.add(1,'hold casing and pinch bodkin through cloth',{'left':(a,L,.025),'right':(b,R,.025)})
 for s in np.linspace(.015,.29,8)[1:]:
  b,R=target(s,'right');p.add(1.3,'feed bodkin through gathered casing',{'right':(b,R,.025)})
 p.add(1,'leave bodkin within casing',{'right':(b,R,.7)})
 p.add(1,'clear right hand',{'right':(b+[0,-.10,.09],R,.7)})
 p.add(1,'release left fabric hold',{'left':(a,L,.7)})
 p.add(1,'lift left hand',{'left':(a+[0,0,.08],L,.7)})
 b,L=target(.29,'left');q=arc(.10)[0]+[0,0,.014];R=down(-90)
 p.add(2,'change hands over the middle of the hood',{'left':(b+[0,0,.06],L,.7),'right':(q+[0,0,.08],R,.7)})
 p.add(2,'lower to bodkin and casing',{'left':(b,L,.7),'right':(q,R,.7)})
 p.add(1,'hold bodkin and steady fabric',{'left':(b,L,.025),'right':(q,R,.025)})
 for s in np.linspace(.29,LENGTH+.13,10)[1:]:
  b,L=target(s,'left');p.add(1.3,'feed and draw bodkin from the far opening',{'left':(b,L,.025)})
 p.add(1,'release both ends',{'left':(b,L,.7),'right':(q,R,.7)})
 p.add(2,'withdraw from the hood',{'left':(b+[0,0,.08],L,.8),'right':(q+[0,0,.10],R,.8)})
 p.add(4,'leave usable cord ends',{})
 print(p.save(Path(__file__).with_name('demo-plan.json')))
