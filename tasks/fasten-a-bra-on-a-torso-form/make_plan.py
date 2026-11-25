from pathlib import Path
import numpy as np,json
from planner import Plan
from scipy.spatial.transform import Rotation
from geometry import X,Z,GAP
p=Plan();L=np.array([[1,0,0],[0,0,-1],[0,1,0]]);R=np.array([[-1,0,0],[0,0,1],[0,1,0]])
L=L@Rotation.from_euler('x',20,degrees=True).as_matrix();R=R@Rotation.from_euler('x',20,degrees=True).as_matrix()
a=np.array([X,GAP+.010,Z]);b=np.array([X,-GAP-.010,Z])
p.add(4,'approach the open back closure',{'left':(a+[0,.045,.0164],L,.2),'right':(b+[0,-.045,.0164],R,.2)})
p.add(3,'put fingers around the two closure tapes',{'left':(a,L,.12),'right':(b,R,.12)})
p.add(2,'pinch both tapes',{'left':(a,L,.022),'right':(b,R,.022)})
a=np.array([X-.008,.010,Z]);b=np.array([X,-.010,Z])
p.add(5,'stretch the wings and put the hooks past the eyes',{'left':(a,L,.022),'right':(b,R,.022)})
a[0]=X-.0045
p.add(3,'pass both eye bars into the hook openings',{'left':(a,L,.022)})
a[1]=.0142
p.add(3,'let band tension seat both hooks',{'left':(a,L,.022)})
p.add(2,'release the two tapes',{'left':(a,L,.20),'right':(b,R,.20)})
p.add(3,'withdraw from the closure',{'left':(a+[0,.08,.03],L,.6),'right':(b+[0,-.08,.03],R,.6)})
p.add(4,'check the released fastening',{})
print(json.dumps(p.save(Path(__file__).with_name('demo-plan.json'))))
