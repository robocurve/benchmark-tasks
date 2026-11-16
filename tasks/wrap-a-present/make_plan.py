from pathlib import Path
import numpy as np,json
from planner import Plan
from common import down,TABLE
if __name__=='__main__':
 p=Plan();D=down(-90);a=np.radians(-55);R=np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]])@down(-90);offset=-.008*R[:,2]
 def move(side,point,opening=.02,t=2.,label='move',orientation=None):
  orientation=R if orientation is None else orientation
  pad_offset=.013 if np.allclose(orientation,D) else .008
  p.add(t,label,{side:(np.array(point)-pad_offset*orientation[:,2],orientation,opening)})
 z=TABLE+.0004;top=z+.040
 p.add(3.,'approach paper',{'left':(np.array([.35,.22,z+.10])+offset,R,.8),'right':(np.array([.35,-.22,z+.10])+offset,R,.8)},'joint')
 for k,(side,base,vec,length) in enumerate([('left',[.35,.08,z],[0,1,0],.10),('right',[.35,-.08,z],[0,-1,0],.10),('left',[.41,0,z],[1,0,0],.08),('right',[.29,0,z],[-1,0,0],.08)]):
  b=np.array(base)+([-0.05,0,0] if k<2 else [0,.04 if side=='left' else -.04,0]);v=np.array(vec);tip=b+v*(.04+length*np.cos(.30))+[0,0,length*np.sin(.30)]
  move(side,tip,.8,2.,'reach paper edge');move(side,tip,.02,1.,'grasp paper edge')
  # Lift the side crease first, then fold the top panel inward.
  for ang in np.linspace(0,np.pi/2,9)[1:]:move(side,b+v*(.04*np.cos(ang)+length*np.cos(.30))+[0,0,.04*np.sin(ang)+length*np.sin(.30)],.02,.3,'raise side fold')
  for ang in np.linspace(.30,np.pi,17)[1:]:move(side,b+v*(length*np.cos(ang))+[0,0,.04+length*np.sin(ang)],.02,.2,'turn flap over box')
  if k%2==0:
   move(side,b-v*length+[0,0,.04],.8,1.,'release supported flap');move(side,[.22,.26 if side=='left' else -.26,1.0],.8,1.5,'clear other arm')
  else:
   move(side,b-v*length+[0,0,.04],.7,.5,'hold the fold down with open fingers')
   tape_side='left' if side=='right' else 'right';j=k//2;pick=np.array([.22,.14 if j==0 else .24,TABLE+.001])
   move(tape_side,pick+[0,0,.10],.8,2.,'approach tape',orientation=D);move(tape_side,pick,.8,1.5,'reach tape',orientation=D);move(tape_side,pick,.02,1.,'pick tape',orientation=D)
   move(tape_side,pick+[0,0,.10],.02,1.5,'lift tape',orientation=D);move(tape_side,[.39 if j==0 else .37,.02 if j==0 else .05,top+.04],.02,2.,'carry tape over seam',orientation=D);move(tape_side,[.39 if j==0 else .37,.02 if j==0 else .05,top+.002],.02,1.5,'press tape across overlap',orientation=D);move(tape_side,[.39 if j==0 else .37,.02 if j==0 else .05,top+.002],.02,.7,'smooth tape',orientation=D)
   move(tape_side,[.39 if j==0 else .37,.02 if j==0 else .05,top+.002],.8,1.,'release tape',orientation=D);move(tape_side,[.22,.26 if tape_side=='left' else -.26,1.0],.8,2.,'clear seam',orientation=R)
   move(side,b-v*length+[0,0,.04],.8,1.,'release held flap');move(side,[.22,.26 if side=='left' else -.26,1.0],.8,2.,'withdraw')
 p.add(4.,'released judging',{})
 print(json.dumps(p.save(Path(__file__).with_name('demo-plan.json'))))
