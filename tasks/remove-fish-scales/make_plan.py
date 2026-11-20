from pathlib import Path
import numpy as np,json
from planner import Plan
from common import down
from geometry import *
GRIP_R=down(0)@Rotation.from_euler("x",50,degrees=True).as_matrix()
def tool_surface(x,y):
 # Continue across the actual last skin facet rather than tip the scaler
 # down the underlying ellipsoid beyond that facet's centre.
 yc=max(y,-.060) if abs(x)<.02 else y
 q,Q=surface(x,yc)
 if y<yc:q=q+np.array([0.,y-yc,-Q[1,2]/Q[2,2]*(y-yc)])
 if x>0:Q=Q@Rotation.from_euler("z",-10,degrees=True).as_matrix()
 return q,Q
p=Plan();L=down(0);R=GRIP_R;tail=TAIL.copy();tool=TOOL.copy()
p.add(3,'approach tail and scaler',{'left':(tail+[0,0,.08],L,.35),'right':(tool+[0,0,.08],R,.30)},'joint')
p.add(3,'place fingers around tail and handle',{'left':(tail,L,.25),'right':(tool,R,.20)})
p.add(2,'grip tail and scaler handle',{'left':(tail,L,.18),'right':(tool,R,.09)})
p.add(2,'lift the scaler',{'right':(tool+[0,0,.10],R,.09)})
for lane,x in enumerate([-.022,-.011,0.,.011,.022]):
 start=.045 if abs(x)>.02 else .060;end=-.045 if abs(x)>.02 else -.078
 q,Q=tool_surface(x,start);target=C+q+Q[:,2]*.0045-Q@HEAD_OFFSET
 p.add(3,'approach scraping lane '+str(lane),{'right':(target+[0,0,.045],Q@GRIP_R,.09)},'joint' if lane==0 else 'cartesian')
 p.add(5,'load scraping lane '+str(lane),{'right':(target,Q@GRIP_R,.09)})
 ys=np.linspace(start,end,13)[1:]
 if lane in (2,3):ys=ys[:9]
 for j,y in enumerate(ys):
  q,Q=tool_surface(x,float(y));target=C+q+Q[:,2]*.0045-Q@HEAD_OFFSET
  p.add(2/3,'scrape lane '+str(lane)+' point '+str(j),{'right':(target,Q@GRIP_R,.09)})
 if lane in (2,3):
  # The rounded head rises beyond this contact interval. Use short loaded
  # strokes behind it rather than drive the scaler onto the head.
  for repeat in range(8):
   for seconds,y,label in [(.35,-.051,'forward'),(.30,-.045,'reset')]:
    q,Q=tool_surface(x,y);target=C+q+Q[:,2]*.0045-Q@HEAD_OFFSET
    p.add(seconds,'scrape lane '+str(lane)+' finish '+str(repeat)+' '+label,{'right':(target,Q@GRIP_R,.09)})
 p.add(2,'lift from scraping lane '+str(lane),{'right':(target+[0,0,.035],Q@GRIP_R,.09)})
p.add(3,'return scaler above its starting place',{'right':(tool+[0,0,.065],R,.09)})
p.add(2,'put scaler on the table',{'right':(tool+[0,0,.003],R,.09)})
p.add(2,'release scaler and fish',{'left':(tail,L,.45),'right':(tool+[0,0,.003],R,.45)})
from yam_arena import ik,station
from yam_arena.constants import READY
clear={side:(*ik.tcp_pose(np.asarray(READY),station.BASES[side]),.6) for side in ['left','right']}
p.add(3,'leave hands clear',clear,'joint')
p.add(4,'inspect remaining scales and intact skin',{})
print(json.dumps(p.save(Path(__file__).with_name('demo-plan.json'))))
