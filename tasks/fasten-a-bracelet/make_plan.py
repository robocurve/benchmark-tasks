from pathlib import Path
import numpy as np,json
from planner import Plan
from common import TABLE
if __name__=='__main__':
 c=np.sqrt(.5);Rs={'left':np.array([[1,0,0],[0,-c,-c],[0,c,-c]]),'right':np.array([[1,0,0],[0,-c,c],[0,-c,-c]])}
 L=np.array([.34,.109,TABLE+.0035])-.008*Rs['left'][:,2];Q=np.array([.34,-.109,TABLE+.0035])-.008*Rs['right'][:,2];p=Plan()
 def both(sec,label,a,b,opening,mode='cartesian'):p.add(sec,label,{'left':(a,Rs['left'],opening),'right':(b,Rs['right'],opening)},mode)
 both(3.,'approach the outer ends of the clasp',L+[0,0,.08],Q+[0,0,.08],.8,'joint')
 both(2.,'enclose the clasp bodies',L,Q,.8)
 both(1.,'hold both ends',L,Q,.04)
 both(2.,'lift around the wrist',L+[0,-.070,.1015],Q+[0,.070,.1015],.04)
 both(3.,'align pin and socket',L+[0,-.093,.1015],Q+[0,.093,.1015],.04)
 both(2.,'push pin into socket',L+[0,-.0995,.1015],Q+[0,.0995,.1015],.04)
 p.add(2.,'seat the aligned pin',{'left':(L+[0,-.0995,.1015],Rs['left'],.04),'right':(Q+[.003,.1055,.1025],Rs['right'],.04)})
 p.add(1.5,'hold the closed clasp',{})
 p.add(1.,'release clasp',{'left':(L+[0,-.0995,.1015],Rs['left'],.8),'right':(Q+[.003,.1055,.1025],Rs['right'],.8)})
 # Native grasp calibration is a fixed offline offset, not prop feedback.
 # both(1.,'release clasp',L+[0,-.0995,.1015],Q+[0,.0995,.1015],.8)
 both(2.,'withdraw',L+[0,.12,.12],Q+[0,-.12,.12],.8)
 p.add(5.,'released retention',{})
 print(json.dumps(p.save(Path(__file__).with_name('demo-plan.json'))))
