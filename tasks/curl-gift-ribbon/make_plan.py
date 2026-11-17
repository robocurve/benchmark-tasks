from pathlib import Path
import numpy as np
from planner import Plan
from common import down,TABLE
from model import EDGE

if __name__=='__main__':
 p=Plan();L=down(-45);R=down(45)
 a=np.array([.235,.21,TABLE+.010]);b=np.array([.235,-.13,TABLE+.010])
 p.add(3,'approach ribbon ends',{'left':(a+[0,0,.08],L,.65),'right':(b+[0,0,.08],R,.65)},'joint')
 p.add(2,'lower to ribbon',{'left':(a,L,.65),'right':(b,R,.65)})
 p.add(1,'pinch ribbon',{'left':(a,L,.025),'right':(b,R,.025)})
 p.add(3,'lift straight ribbon',{'left':(a+[0,0,.13],L,.025),'right':(b+[0,0,.13],R,.025)})
 p.add(2,'carry above curling edge',{'left':(a+[.085,0,.13],L,.025),'right':(b+[.085,0,.13],R,.025)})
 height=.035;path=.34*1.025
 def targets(y):
  right=-np.sqrt((path-np.sqrt(y*y+height*height))**2-height*height)
  return np.array([EDGE[0],y,TABLE+.030]),np.array([EDGE[0],right,TABLE+.030])
 a1,b1=targets(.21)
 p.add(3,'lower ribbon under tension onto edge',{'left':(a1,L,.025),'right':(b1,R,.025)})
 for y in np.linspace(.21,.055,17)[1:]:
  a2,b2=targets(y)
  p.add(.7,'draw loaded ribbon across edge',{'left':(a2,L,.025),'right':(b2,R,.025)})
 a2=a2+[-.10,0,.13];b2=b2+[-.10,0,.13]
 p.add(3,'lift processed ribbon clear of the ruler',{'left':(a2,L,.025),'right':(b2,R,.025)})
 p.add(1,'release curled ribbon',{'left':(a2,L,.7),'right':(b2,R,.7)})
 p.add(2,'clear ribbon',{'left':(a2+[.05,.07,.04],L,.8),'right':(b2+[.05,-.03,.04],R,.8)})
 p.add(10,'allow free curl to settle',{})
 print(p.save(Path(__file__).with_name('demo-plan.json')))
