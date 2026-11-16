"""Fixed ordinary joint trajectory, using setup coordinates only."""
from pathlib import Path
import numpy as np,json
from planner import Plan
from common import down
from model import REST,MOUTH,GRIP_LOCAL

def ry(a):
 a=np.radians(a);return np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]])

if __name__=='__main__':
 p=Plan();D=ry(-50)@down(-90);offset=np.array([0,0,.004]);grip=REST+GRIP_LOCAL+offset
 def move(center,angle=0.,opening=.08,t=2.,label='carry spoon',mode='cartesian'):
  R=ry(angle);tcp=np.asarray(center)+R@(GRIP_LOCAL+offset)
  p.add(t,label,{'right':(tcp,R@D,opening)},mode)
 move(REST+[0,0,.10],opening=.8,t=3.,label='approach spoon',mode='joint')
 move(REST,opening=.8,t=2.,label='lower around padded handle')
 move(REST,opening=.08,t=1.,label='grasp handle')
 move(REST+[0,0,.065],t=2.,label='lift level spoon')
 target=MOUTH+[-.035,0,.004]
 move(target+[-.025,0,.025],t=4.,label='carry liquid level')
 move(target,t=3.,label='approach open mouth slowly')
 for angle in np.linspace(0,32,9)[1:]:
  # Keep the pouring rim inside the lip opening while rotating about it.
  R=ry(angle);center=MOUTH+np.array([.006,0,-.001])-R@np.array([.024,0,0])
  move(center,angle,t=.65,label='tip spoon gently')
 move(center,32,t=4.,label='allow liquid to drain')
 move(center+[-.025,0,.020],32,t=2.,label='withdraw spoon')
 move(REST+[0,0,.065],0,t=4.,label='return spoon to rest')
 move(REST,0,t=2.,label='lower spoon')
 move(REST,0,.8,1.,'release spoon')
 move(REST+[0,0,.10],0,.8,2.,'clear spoon')
 p.add(2.,'released judging',{})
 print(json.dumps(p.save(Path(__file__).with_name('demo-plan.json'))))
