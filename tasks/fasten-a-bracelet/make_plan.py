from pathlib import Path
import numpy as np,json
from planner import Plan
from common import TABLE
from bracelet_geometry import END_Y
from scipy.spatial.transform import Rotation
if __name__=='__main__':
 c=np.sqrt(.5);Rs={'left':np.array([[1,0,0],[0,-c,-c],[0,c,-c]]),'right':np.array([[1,0,0],[0,-c,c],[0,-c,-c]])}
 G=END_Y+.004;A=G-.034;B=G-.016;C=G-.0055
 L=np.array([.34,G,TABLE+.0135])-.008*Rs['left'][:,2];Q=np.array([.34,-G,TABLE+.0135])-.008*Rs['right'][:,2];p=Plan()
 def both(sec,label,a,b,opening,mode='cartesian'):p.add(sec,label,{'left':(a,Rs['left'],opening),'right':(b,Rs['right'],opening)},mode)
 both(3.,'approach the outer ends of the clasp',L+[0,0,.08],Q+[0,0,.08],.8,'joint')
 both(2.,'enclose the clasp bodies',L,Q,.8)
 both(1.,'hold both ends',L,Q,.04)
 both(2.,'lift around the wrist',L+[0,-A,.084],Q+[0,A,.084],.04)
 old=Rs['left'].copy();Rs['left']=Rotation.from_rotvec([0,-np.radians(40),0]).as_matrix()@old
 L=L+.008*(old[:,2]-Rs['left'][:,2])+[-.0025,0,0]
 old=Rs['right'].copy();Rs['right']=Rotation.from_rotvec([0,np.radians(20),0]).as_matrix()@old
 Q=Q+.008*(old[:,2]-Rs['right'][:,2])
 both(2.,'turn the holding hand for insertion clearance',L+[0,-A,.084],Q+[0,A,.084],.04)
 both(3.,'align pin and socket',L+[0,-B,.084],Q+[0,B,.084],.04)
 both(16.,'push pin into socket',L+[0,-C,.084],Q+[0,C,.084],.04)
 p.add(3.,'seat the aligned pin',{'left':(L+[0,-C,.084],Rs['left'],.04),'right':(Q+[0,C,.084],Rs['right'],.04)})
 p.add(2.,'hold the closed clasp',{})
 p.add(2.,'lower the clasp onto the wrist',{})
 p.add(1.,'release clasp',{'left':(L+[0,-C,.084],Rs['left'],.18),'right':(Q+[0,C,.084],Rs['right'],.18)})

 both(2.,'withdraw',L+[0,.04,.07],Q+[0,-.04,.07],.18)
 both(1.,'open hands clear of the bracelet',L+[0,.04,.07],Q+[0,-.04,.07],.8)
 p.add(5.,'released retention',{})
 print(json.dumps(p.save(Path(__file__).with_name('demo-plan.json'))))
