"""Original full-size five-card friction case; units m, kg, seconds."""
from dataclasses import dataclass
import numpy as np
CARD_SIZE=np.array([.0856,.054,.00076])
CARD_DENSITY=1400.
CARD_MASS=float(np.prod(CARD_SIZE)*CARD_DENSITY)
N=5
TABLE=.7475
ORIGIN=np.array([.34,-.045,TABLE+.050])
ROOF=.014
PLATE_TOP=ROOF-N*CARD_SIZE[2]
SPRING_K=2500.
SPRING_REST=.0008
SLOT_BOTTOM=.01285
SLOT_HEIGHT=ROOF-SLOT_BOTTOM
TRAY=np.array([.50,-.10,TABLE+.001])
CASE_COLOR=(.16,.30,.44)
@dataclass(frozen=True)
class Shape:
 name:str
 center:tuple
 size:tuple
 density:float=1050.
 color:tuple=CASE_COLOR
 @property
 def mass(self):return float(np.prod(self.size)*self.density)
def case_shapes():
 # Opening faces +X. Roof aperture 66x40mm exposes only top card.
 return [Shape('floor',(0,0,.001),(.100,.064,.002)),
 Shape('left',(0,.0305,.009),(.100,.003,.014)),
 Shape('right',(0,-.0305,.009),(.100,.003,.014)),
 Shape('rear',(-.0485,0,.009),(.003,.058,.014)),
 Shape('front_lower',(.0485,0,(.002+SLOT_BOTTOM)/2),(.003,.058,SLOT_BOTTOM-.002)),
 Shape('roof_L',(0,.0245,.015),(.100,.009,.002)),
 Shape('roof_R',(0,-.0245,.015),(.100,.009,.002)),
 Shape('roof_rear',(-.0415,0,.015),(.017,.040,.002)),
 Shape('roof_front',(.0415,0,.015),(.017,.040,.002))]
def follower_shapes():
 return [Shape('plate',(0,0,PLATE_TOP-.0005),(.084,.052,.001),density=2700.,color=(.65,.65,.65))]
def initial_cards():
 return {f'card_{i}':ORIGIN+np.array([0,0,PLATE_TOP+(i+.5)*CARD_SIZE[2]])for i in range(N)}
def engineering():
 return dict(card=dict(size_m=CARD_SIZE.tolist(),density_kg_m3=CARD_DENSITY,mass_kg=CARD_MASS,count=N,material='rigid PVC 0.76mm loyalty cards; not paper'),
 case=dict(exterior_m=[.100,.064,.016],roof_bottom_m=ROOF,thumb_window_m=[.066,.040],single_slot_height_m=SLOT_HEIGHT,single_card_clearance_m=SLOT_HEIGHT-CARD_SIZE[2],two_card_interference_m=2*CARD_SIZE[2]-SLOT_HEIGHT,
 spring=dict(rate_N_per_m=SPRING_K,free_target_above_initial_m=SPRING_REST,initial_force_N=SPRING_K*SPRING_REST,travel_m=[-.005,.0038]),
 parts=[dict(name=s.name,center_m=s.center,size_m=s.size,density_kg_m3=s.density,mass_kg=s.mass)for s in case_shapes()+follower_shapes()]),
 provenance='Original manufacturable geometry. 0.76mm card format is sourced; friction .3, spring 2500N/m and clearances are declared design values, not OEM calibration.',
 table_top_m=TABLE,origin_m=ORIGIN.tolist(),runtime_physics=None)
def rotation(q):
 x,y,z,w=q
 return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def corners(p,q,size):
 import itertools
 return np.array(list(itertools.product(*[[-x/2,x/2]for x in size])))@rotation(q).T+p
