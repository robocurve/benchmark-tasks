"""Cut on a board: support the cutter and align the cigar vertically above it."""
from pathlib import Path
import numpy as np,json
from scipy.spatial.transform import Rotation
from planner import Plan
from common import down,TABLE
from geometry import CIGAR,CUTTER,CAP
p=Plan();RL=Rotation.from_euler('x',-45,degrees=True).as_matrix()@down(0);RR=Rotation.from_euler('x',45,degrees=True).as_matrix()@down(0)
L=CIGAR+[0,.025,0]-.008*RL[:,2];R=CUTTER-.008*RR[:,2]
p.add(3,'approach cigar and cutter',{'left':(L+[0,0,.10],RL,.6),'right':(R+[0,0,.10],RR,1.)},'joint')
p.add(2,'enclose cigar and open handles',{'left':(L,RL,.6),'right':(R,RR,1.)})
p.add(1,'hold cigar and support cutter on board',{'left':(L,RL,.185),'right':(R,RR,.85)})
p.add(3,'lift cigar',{'left':(L+[0,0,.16],RL,.185)})
turn=Rotation.from_euler('z',-90,degrees=True).as_matrix()@Rotation.from_euler('x',90,degrees=True).as_matrix();LC=turn@RL
G=CUTTER-turn@(CAP-(CIGAR+[0,.025,0]));P=G-.008*LC[:,2]
p.add(4,'point the cap down over the cutter',{'left':(P+[0,0,.04],LC,.185)})
p.add(3,'place cap through aperture',{'left':(P,LC,.185)})
p.add(3,'close guillotine blades',{'right':(R,RR,.4)})
p.add(1,'hold cut',{})
p.add(2,'open cutter',{'right':(R,RR,.85)})
p.add(2,'lift cut cigar clear',{'left':(P+[0,0,.08],LC,.185)})
p.add(4,'return cigar above board',{'left':(L+[0,0,.10],RL,.185)})
p.add(2,'lower cigar',{'left':(L,RL,.185)})
p.add(1,'release both',{'left':(L,RL,.8),'right':(R,RR,1.)})
p.add(3,'withdraw',{'left':(L+[0,0,.15],RL,.8),'right':(R+[0,0,.15],RR,1.)})
p.add(4,'released outcome',{})
print(json.dumps(p.save(Path(__file__).with_name('demo-plan.json'))))
