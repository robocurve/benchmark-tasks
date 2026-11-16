from pathlib import Path
import numpy as np
from planner import Plan
from common import down
from model import Model,CENTER,COLS
if __name__=='__main__':
 m=Model();p=Plan();L=down(135);c=np.sqrt(.5);R=np.array([[1,0,0],[0,-c,c],[0,-c,-c]])
 a=m.soft.p[26*COLS+6]-.014*L[:,2];b=m.pull_point()-.008*R[:,2]
 p.add(3,'approach webbing and tongue',{'left':(a+[0,0,.08],L,.4),'right':(b+[0,0,.08],R,.4)},'joint')
 p.add(2,'lower hands',{'left':(a,L,.4),'right':(b,R,.4)})
 p.add(1,'hold webbing edge and tongue',{'left':(a,L,.025),'right':(b,R,.025)})
 theta=np.radians(25);L=np.array([[1,0,0],[0,np.cos(theta),np.sin(theta)],[0,-np.sin(theta),np.cos(theta)]])@down(135)
 lift=m.soft.p[26*COLS+6]+[0,0,0.025]-.014*L[:,2]
 p.add(2,'lift the edge and angle the holding hand clear',{'left':(lift,L,.025)})
 fold=m.soft.p[26*COLS]+np.array([0.,-.012,.0013+0.025])-.014*L[:,2]
 p.add(2,'make a diagonal fold in the webbing',{'left':(fold,L,.025)})
 p.add(1.8,'feed the diagonal fold into the slot',{'right':(b+[0,.035,0],R,.025)})
 p.add(.6,'release fabric before the tongue reaches the fingers',{'left':(fold+[0,.02,.02],L,.7)})
 p.add(1.2,'clear left hand',{'left':(a+[-.06,.16,.14],down(135),.8)})
 p.add(4,'slide tongue over the fold',{'right':(b+[0,.13,0],R,.025)})
 p.add(1,'release tongue',{'right':(b+[0,.13,0],R,.7)})
 p.add(2,'clear right hand',{'right':(b+[0,.13,.10],R,.8)})
 p.add(4,'released flatness check',{})
 # Fixed grasp coordinates were refined from a prior native cloth recording.
 # They change only the scripted pickup, not the cloth state or task scorer.
 L=down(135);R=down()
 gl=np.array([0.30843058855497246, 0.0954015391442675, 0.7741684587181443])
 gr=np.array([0.3253766606024856, -0.055961514408360585, 0.7678799371101135])
 p.add(2,'approach opposite webbing edges',{'left':(gl+[0,0,.07],L,.7),'right':(gr+[0,0,.07],R,.7)})
 p.add(2,'lower to the released webbing',{'left':(gl,L,.7),'right':(gr,R,.7)})
 p.add(1,'take the two edges',{'left':(gl,L,.025),'right':(gr,R,.025)})
 ul=CENTER+np.array([-.0235,.065,.030])-.014*L[:,2]
 ur=CENTER+np.array([.0235,-.065,.030])-.014*R[:,2]
 p.add(3,'spread the webbing flat',{'left':(ul,L,.025),'right':(ur,R,.025)})
 p.add(2,'lay the straightened webbing down',{'left':(ul+[0,0,-.055],L,.025),'right':(ur+[0,0,-.055],R,.025)})
 p.add(1,'release the flat belt',{'left':(ul+[0,0,-.055],L,.7),'right':(ur+[0,0,-.055],R,.7)})
 p.add(2,'withdraw both hands',{'left':(ul+[0,0,.06],L,.8),'right':(ur+[0,0,.06],R,.8)})
 p.add(4,'released final check',{})
 print(p.save(Path(__file__).with_name('demo-plan.json')))
