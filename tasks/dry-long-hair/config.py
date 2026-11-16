"""Refined articulated bundles and conserved coarse thermal totals; not fibres."""
TOP=.7475
HEAD=(.43,0.,1.045)
# User-facing rest frame: yaw180 from the original nose-to-world-minus-X pose.
# Exact XYZW quaternion avoids a nearly-zero scalar ambiguity at pi.
HEAD_QUAT=(0.,0.,1.,0.)
HEAD_RADIUS=.065
# Split each previous 14mm bundle into two 7mm bundles and each 50mm
# rigid link into two 25mm links. This is a physical contact/bending refinement,
# not just extra render hairs. Total lock length and lateral exchange area stay
# fixed; effective density is not a measured individual-fibre material.
REFERENCE_LOCKS=12
REFERENCE_SEGMENTS=5
REFERENCE_LENGTH=.050
REFERENCE_RADIUS=.007
LOCKS=24
AXIAL_REFINEMENT=2
SEGMENTS=REFERENCE_SEGMENTS*AXIAL_REFINEMENT
LENGTH=REFERENCE_LENGTH/AXIAL_REFINEMENT
RADIUS=REFERENCE_RADIUS*REFERENCE_LOCKS/LOCKS
# Distributed elastic/viscous bending: splitting a bundle divides its EI;
# halving link length doubles the corresponding angular spring coefficient.
# Alternating one-axis joints remain a reduced approximation, not calibrated hair.
BEND_STIFFNESS=.00006*(REFERENCE_LOCKS/LOCKS)*AXIAL_REFINEMENT
BEND_DAMPING=.000015*(REFERENCE_LOCKS/LOCKS)*AXIAL_REFINEMENT
BEND_LIMIT_DEG=70./AXIAL_REFINEMENT
DRY_MASS_PER_SEGMENT=.032/(LOCKS*SEGMENTS)
WATER_PER_PATCH=.0064/(LOCKS*SEGMENTS*4)
PATCHES_PER_SEGMENT=4
# Ordinary low-heat dryer setting; candidate physical source, uncalibrated.
AIR_TEMPERATURE=45.
HEATER_LIMIT=800.
MAX_TEMPERATURE=50.
DRY_FRACTION=.10
DRYER_START=(.23,0.,TOP+.21)
SWITCH_START=(DRYER_START[0]-.037,DRYER_START[1],DRYER_START[2]-.045)
SWITCH_TRAVEL=.004
# Candidate passive detent approximation, not a calibrated switch mechanism.
SWITCH_DETENT_STIFFNESS=100.  # N/m
SWITCH_DETENT_DAMPING=.05    # N s/m

def coarse_points_local(k):
 import math
 a=-2.12+4.24*k/(LOCKS-1)
 phi=math.radians(35. if k%2==0 else 60.)
 radius=(HEAD_RADIUS+.011)*math.sin(phi)
 z=(HEAD_RADIUS+.011)*math.cos(phi)
 result=[(radius*math.cos(a),radius*math.sin(a),z)]
 for j in range(REFERENCE_SEGMENTS):
  next_radius=.080 if j<2 or k%2 else .058
  dr=next_radius-radius
  if abs(dr)>=REFERENCE_LENGTH:raise ValueError('invalid rest strand segment')
  z-=math.sqrt(REFERENCE_LENGTH*REFERENCE_LENGTH-dr*dr);radius=next_radius
  result.append((radius*math.cos(a),radius*math.sin(a),z))
 return result

def points_local(k):
 coarse=coarse_points_local(k)
 # Subdivision leaves every old polyline endpoint fixed and each fine link
 # exactly LENGTH long. No rest-shape relaxation or hidden pose maintenance.
 result=[]
 for a,b in zip(coarse,coarse[1:]):
  for j in range(AXIAL_REFINEMENT):
   u=j/AXIAL_REFINEMENT
   result.append(tuple(x+(y-x)*u for x,y in zip(a,b)))
 return result+[coarse[-1]]

def bend_axis(k,n):return 'X' if (k+n//AXIAL_REFINEMENT)%2 else 'Y'

def points(k):return [tuple(c+d for c,d in zip(HEAD,rotate(HEAD_QUAT,p))) for p in points_local(k)]
def root_local(k):return points_local(k)[0]
def root(k):return points(k)[0]
def segment(k,j):return tuple((a+b)/2 for a,b in zip(points(k)[j],points(k)[j+1]))
def segment_quat(k,j):
 import math
 a,b=points_local(k)[j:j+2];t=[(x-y)/LENGTH for x,y in zip(a,b)];q=(-t[1],t[0],0.,1+t[2]);n=math.sqrt(sum(x*x for x in q));return multiply(HEAD_QUAT,tuple(x/n for x in q))
def conjugate(q):return (-q[0],-q[1],-q[2],q[3])
def multiply(a,b):
 x,y,z,w=a;X,Y,Z,W=b
 return (w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,w*Z+x*Y-y*X+z*W,w*W-x*X-y*Y-z*Z)
def rotate(q,v):
 x,y,z,w=q;cross=(y*v[2]-z*v[1],z*v[0]-x*v[2],x*v[1]-y*v[0]);second=(y*cross[2]-z*cross[1],z*cross[0]-x*cross[2],x*cross[1]-y*cross[0]);return tuple(a+2*w*b+2*c for a,b,c in zip(v,cross,second))
