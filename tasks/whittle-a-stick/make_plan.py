"""Author robot joint actions for support, shaving and stock rotation."""
from pathlib import Path
from geometry import *
from common import down
from planner import Plan
p=Plan();grip=np.array([0,.055,0.]);held=np.array([.27,0,TABLE+.12])
def wood(origin,Q=Q0,g=.10):return (np.array(origin)+Q@grip,Q@down(180)@Rotation.from_euler("x",40,degrees=True).as_matrix(),g)
def knife(origin,Q=Q0,g=.14):return (np.array(origin)+Q@HANDLE,Q@down(-90)@Rotation.from_euler("x",40,degrees=True).as_matrix(),g)
p.add(3,'approach',{'left':wood(STICK+[0,0,.10],g=.8),'right':knife(KNIFE+[0,0,.10],g=.8)},mode='joint')
p.add(2,'align',{'left':wood(STICK,g=.8),'right':knife(KNIFE,g=.8)})
p.add(2,'grip',{'left':wood(STICK),'right':knife(KNIFE)})
p.add(4,'lift',{'left':wood(held),'right':knife(KNIFE+[0,0,.15])})
for i in range(5):
 Q=Q0
 origin=held+Q0@(np.array([0,-.044,.007])-EDGE)
 p.add(4,'approach cut '+str(i),{'right':knife(origin+[0,0,.025])})
 p.add(2,'set edge '+str(i),{'right':knife(origin)})
 p.add(4,'shave '+str(i),{'right':knife(held+Q0@(np.array([0,-.089,.006-(i+1)*.002*(44/45)+.0002])-EDGE))})
 p.add(2,'clear edge '+str(i),{'right':knife(held+Q0@(np.array([0,-.12,.04])-EDGE))})
p.add(4,'unturn',{'left':wood(held),'right':knife(KNIFE+[0,0,.15])})
p.add(4,'place',{'left':wood(STICK),'right':knife(KNIFE)})
p.add(1,'release',{'left':wood(STICK,g=.8),'right':knife(KNIFE,g=.8)})
p.add(3,'clear',{'left':wood(STICK+[0,0,.15],g=.8),'right':knife(KNIFE+[0,0,.15],g=.8)})
p.add(2,'settle',{})
print(p.save(Path(__file__).with_name('demo-plan.json')))
