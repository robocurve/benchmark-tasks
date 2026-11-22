"""Ordinary scoop pickup, confined sweep and tipping into the bucket."""
from pathlib import Path
from planner import Plan
from geometry import *
from common import down
p=Plan();GRIP_R=Rotation.from_euler("x",30,degrees=True).as_matrix()@down(0)
def target(origin,Q=None,g=.14):
 Q=np.eye(3) if Q is None else Q
 return {'right':(np.asarray(origin)+Q@HANDLE,Q@GRIP_R,g)}
p.add(3,'approach',target(SCOOP+[0,0,.10],g=.7),mode='joint')
p.add(2,'align',target(SCOOP,g=.7))
p.add(2,'grip',target(SCOOP))
p.add(2,'lift',target(SCOOP+[0,0,.12]))
entry=GUTTER+[0,.165,-RADIUS-.0010]
Q=Rotation.from_euler('x',-5,degrees=True).as_matrix()
p.add(4,'position',target(entry+[0,0,.065],Q))
p.add(3,'enter',target(entry,Q))
p.add(6,'scoop',target(entry+[0,.075,0],Q))
exitQ=Rotation.from_euler('y',25,degrees=True).as_matrix()@Rotation.from_euler('x',15,degrees=True).as_matrix()
p.add(3,'lift leaves',target(entry+[0,.075,.060],exitQ))
p.add(3,'withdraw from roof',target(entry+[-.15,.075,.09],exitQ))
over=BUCKET+[0,0,.10]
carryQ=exitQ
p.add(3,'carry',target(over,carryQ))
p.add(3,'level over bucket',target(over,np.eye(3)))
p.add(3,'put scoop in bucket',target(BUCKET+[0,0,.020],np.eye(3)))
p.add(2,'release',target(BUCKET+[0,0,.020],np.eye(3),g=.8))
p.add(3,'clear',target(BUCKET+[0,0,.14],np.eye(3),g=.8))
p.add(2,'settle',{})
print(p.save(Path(__file__).with_name('demo-plan.json')))
