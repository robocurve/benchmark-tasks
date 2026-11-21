"""Full-scale compact dryer, free body with passive folding handle and loose cord."""
import numpy as np
from scipy.spatial.transform import Rotation
CONTROL_HZ=20
MAX_STEPS=1000
INSTRUCTION="Fold the unplugged travel hair dryer's handle fully against its barrel. Leave the dryer resting on the counter with the handle folded, release both hands, and let it settle. The remote part of the cable is already coiled and tied; the loose tail remains attached to the handle. Do not pull or remove the cable."
LINKS=('head','handle',*(f'cord_{i}' for i in range(6)),'cord_bundle')
N=len(LINKS)
HEAD,HANDLE,BUNDLE=0,1,8
HINGE=np.array((.035,0,-.050))
GRIP_LOCAL=np.array((0,0,-.045))
R0=Rotation.from_euler('x',90,degrees=True).as_matrix()
Q0=Rotation.from_matrix(R0).as_quat()
ORIGIN=np.array((.28,-.10,.045))
HANDLE_ORIGIN=ORIGIN+R0@HINGE
CABLE_ANCHOR_LOCAL=np.array((0,0,-.0505))
CABLE_START=HANDLE_ORIGIN+R0@CABLE_ANCHOR_LOCAL
LINK_LENGTH=.05
CABLE_RADIUS=.0025
COIL_RADIUS=.04
COIL_HALF_HEIGHT=.0125
# Six 50 mm free links make a slack 300 mm tail, with a gently falling initial
# profile. The remaining coiled cable is bound into a movable rigid bundle.
DROP=(CABLE_START[2]-COIL_HALF_HEIGHT)/6
HORIZONTAL=np.sqrt(LINK_LENGTH**2-DROP**2)
NODES=[CABLE_START.copy()]
for a in np.radians((90,45,-45,-90,0,0)):
 NODES.append(NODES[-1]+[HORIZONTAL*np.cos(a),HORIZONTAL*np.sin(a),-DROP])
NODES=np.asarray(NODES)
BUNDLE_ORIGIN=NODES[-1]+[COIL_RADIUS,0,0]
def align_z(d):
 d=np.asarray(d,float);d/=np.linalg.norm(d)
 if d[2]<-.999999:return np.array((1.,0.,0.,0.))
 q=np.r_[np.cross((0.,0.,1.),d),1+d[2]];return q/np.linalg.norm(q)
START_POSES=np.array([(*ORIGIN,*Q0),(*HANDLE_ORIGIN,*Q0),*[(*((NODES[i]+NODES[i+1])/2),*align_z(NODES[i+1]-NODES[i])) for i in range(6)],(*BUNDLE_ORIGIN,0,0,0,1)])
RESET_POSES=dict(zip(LINKS,START_POSES))
MASSES=np.array((.285,.083,*([.00125]*6),.068))
TABLE_BOUNDS=(-.10,.85,-.50,.50)
HINGE_DAMPING=50.
HINGE_FRICTION_CAP=.025
HINGE_LIMIT=90.
def quat_matrix(q):return Rotation.from_quat(q).as_matrix()
def qmul(a,b):return (Rotation.from_quat(a)*Rotation.from_quat(b)).as_quat()
