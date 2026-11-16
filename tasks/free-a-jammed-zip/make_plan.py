from pathlib import Path
import numpy as np
from planner import Plan
from common import down
from model import X,Z,START

if __name__=='__main__':
 p=Plan();L=down(-45);c=np.sqrt(.5);R=np.array([[1,0,0],[0,-c,c],[0,-c,-c]])
 a=np.array([X-.070,.063,Z+.002])-.008*L[:,2];b=np.array([X,START-.012,Z+.011])-.008*R[:,2]
 def both(t,label,l,r,g,mode='cartesian'):p.add(t,label,{'left':(l,L,g),'right':(r,R,g)},mode)
 both(3,'approach lining and slider',a+[0,0,.09],b+[0,0,.09],.7,'joint')
 both(2,'lower to the lining and pull',a,b,.7)
 both(1,'pinch lining and pull tab',a,b,.025)
 both(2,'back the slider away from the trapped lining',a,b+[0,-.050,0],.025)
 both(2,'ease the lining sideways',a+[-.035,0,.008],b+[0,-.050,0],.025)
 both(3,'run the zip beyond the freed section',a+[-.035,0,.008],b+[0,.075,0],.025)
 both(1,'release both',a+[-.035,0,.008],b+[0,.075,0],.7)
 both(2,'withdraw',a+[-.045,0,.10],b+[.04,.075,.10],.8)
 p.add(3,'leave the repaired zip',{})
 print(p.save(Path(__file__).with_name('demo-plan.json')))
