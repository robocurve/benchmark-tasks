import json
from pathlib import Path
import numpy as np
from planner import Plan
from common import down,TABLE
if __name__=='__main__':
 p=Plan();D=down(-90);a=np.radians(-60);R=np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]])@down(-90);c=np.sqrt(.5);S=np.array([[1,0,0],[0,-c,-c],[0,c,-c]]);offset=-.008*R[:,2];left=np.array([.2225,.078,TABLE+.002])-.008*S[:,2];right=np.array([.20,-.078,TABLE+.002])+offset;L=np.array([.3825,.078,.8662])-.008*S[:,2];Q=np.array([.36,-.078,.8706])+offset
 p.add(3.,'approach the strip',{'left':(left+[0,0,.10],S,.8),'right':(right+[0,0,.10],R,.8)},'joint')
 p.add(2.,'reach its ends',{'left':(left,S,.8),'right':(right,R,.8)})
 p.add(1.,'pick up the strip',{'left':(left,S,.01),'right':(right,R,.01)})
 p.add(2.,'lift',{'left':(left+[0,0,.17],S,.01),'right':(right+[0,0,.17],R,.01)})
 p.add(2.,'carry over the leg',{'left':(L+[0,0,.06],S,.01),'right':(Q+[0,0,.06],R,.01)})
 p.add(2.,'lay onto the leg',{'left':(L,S,.01),'right':(Q,R,.01)})
 p.add(1.,'keep the dry edge held',{'right':(Q+[0,0,.03],D,.7)})
 p.add(1.,'press the far end',{'right':(np.array([.344,-.072,.877]),D,.36)})
 p.add(3.,'smooth towards the held edge',{'right':(np.array([.344,.035,.877]),D,.36)})
 p.add(2.,'smooth back along the strip',{'right':(np.array([.357,-.072,.877]),D,.36)})
 p.add(3.,'smooth the other long edge',{'right':(np.array([.372,.035,.877]),D,.36)})
 p.add(2.,'clear the other hand for peeling',{'right':(np.array([.24,-.32,1.02]),R,.8)})
 p.add(.9,'peel back close to the leg',{'left':(L+[0,-.270,.025]+.008*S[:,2]-.008*R[:,2],R,.01)})
 p.add(2.,'carry removed strip away',{'left':(np.array([.18,-.18,.88]),R,.01)})
 p.add(1.,'release strip',{'left':(np.array([.18,-.18,.88]),R,.8)})
 p.add(2.,'clear the leg',{'left':(np.array([.18,.22,1.0]),R,.8),'right':(np.array([.25,-.23,1.0]),R,.8)})
 p.add(3.,'released judging',{})
 print(json.dumps(p.save(Path(__file__).with_name('demo-plan.json'))))
