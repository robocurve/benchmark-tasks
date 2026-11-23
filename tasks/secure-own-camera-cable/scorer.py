"""Observable routing, strap placement and unsupported-release rubric."""
import numpy as np
from scipy.spatial.transform import Rotation
MOUNT=np.array([-.05,.007,-.16]);B=np.array([[0,0,-1],[1,0,0],[0,-1,0.]])
STRAP_ROOT = (np.array([-.05 + .044*np.cos(2.8), -.0335 + .029*np.sin(2.8), -.16]) - MOUNT) @ B
STRAP_HALF_LENGTH = .007
CABLE_RADIUS = .003


def encloses_cable(cable_samples, strap, centre, basis, pad_indices):
    """Test the routed cable at the middle of the strap in the peg frame."""
    if not pad_indices or len(strap) < 2:
        return False
    rotations = Rotation.from_quat(strap[:, 3:]).as_matrix()
    centres = (strap[:, :3] - centre) @ basis
    tangents = rotations[:, :, 0] @ basis
    inward = rotations[:, :, 2] @ basis
    starts = centres - STRAP_HALF_LENGTH * tangents
    ends = centres + STRAP_HALF_LENGTH * tangents
    rail = cable_samples.reshape(-1, 3)
    crosses = (rail[:-1, 1] <= 0) != (rail[1:, 1] <= 0)
    a, b = rail[:-1][crosses], rail[1:][crosses]
    section = a + (b - a) * (-a[:, 1:2] / (b[:, 1:2] - a[:, 1:2]))
    x, _, z = section.T
    section = section[(x > -.006) & (x < .028) & (z > .005)
                      & (np.abs(z - .013) < .002)]

    for i in pad_indices:
        gaps = np.vstack([starts[0] - STRAP_ROOT, starts[1:i+1] - ends[:i]])
        if np.any(np.linalg.norm(gaps, axis=1) >= 2*CABLE_RADIUS):
            continue
        # The cable centre must lie under the strip's width, with room for its radius.
        if np.any(np.abs(np.r_[starts[:i+1, 1], ends[:i, 1], centres[i, 1]])
                  > .010 - CABLE_RADIUS):
            continue
        face = centres + .00115 * inward
        joints = (ends[:i] + starts[1:i+1]) / 2 + .00115 * inward[:i]
        # The arm-side backing and cap close the cross-section down to
        # the peg axis. Only cable crossings resting on the shaft are tested.
        boundary = np.vstack([STRAP_ROOT, joints, face[i],
                              [face[i, 0], 0, 0], [STRAP_ROOT[0], 0, 0]])[:, [0, 2]]
        x0, z0 = boundary.T
        x1, z1 = np.roll(boundary, -1, axis=0).T
        for x, _, z in section:
            hits = (x0 > x) != (x1 > x)
            heights = z0[hits] + (z1[hits]-z0[hits]) * (x-x0[hits]) / (x1[hits]-x0[hits])
            if np.count_nonzero(heights > z) % 2 and np.all(np.abs(heights-z) >= CABLE_RADIUS):
                return True
    return False


class Scorer:
 def __init__(self):self.route_time=0.;self.strap_time=0.;self.release_time=0.;self.best=0.
 def update(self,cable,strap,T,right_tcp,gap,dt):
  c=(T@np.r_[MOUNT,1])[:3];basis=T[:3,:3]@B
  cable=np.asarray(cable);axes=Rotation.from_quat(cable[:,3:]).as_matrix()[:,:,0];half=.7217926530318888/40/2
  gap_error=float(np.linalg.norm((cable[:-1,:3]+axes[:-1]*half)-(cable[1:,:3]-axes[1:]*half),axis=1).max())
  intact=gap_error<.020
  # Cable must actually rest on the upper shaft, behind the cap.
  sampled=[]
  for pos,axis in zip(cable[:,:3],axes):
   sampled.extend((np.array([pos+axis*t for t in np.linspace(-half,half,5)])-c)@basis)
  sampled=np.array(sampled);radial=np.linalg.norm(sampled[:,1:],axis=1)
  contact=(sampled[:,0]>-.006)&(sampled[:,0]<.028)&(sampled[:,2]>.005)&(abs(radial-.013)<.002)
  crossing=bool(np.any(sampled[:,1]<-.008) and np.any(sampled[:,1]>.008))
  routed=bool(intact and crossing and contact.any())
  pad_indices=[]
  strap=np.asarray(strap)
  for i,row in enumerate(strap):
   R=Rotation.from_quat(row[3:]).as_matrix();p=(np.array(row[:3])+R[:,2]*.00115-c)@basis
   if abs(p[0]-.0355)<=.0015 and np.linalg.norm(p[1:])<.017 and R[:,2]@basis[:,0]<-.866:pad_indices.append(i)
  overlap=bool(pad_indices)
  retained=routed and encloses_cable(sampled,strap,c,basis,pad_indices)
  distance=float(np.linalg.norm(np.asarray(right_tcp)-c));clear=bool(distance>.12 and gap>.080)
  self.route_time=self.route_time+dt if routed else 0.
  self.strap_time=self.strap_time+dt if retained else 0.
  self.release_time=self.release_time+dt if retained and clear else 0.
  score=1. if self.release_time>=2. else .75 if self.strap_time>=1. else .5 if self.route_time>=1. else 0.
  self.best=max(self.best,score)
  return dict(score=score,best_score=self.best,routed=routed,strap_overlap=overlap,cable_enclosed=retained,right_hand_clear=clear,right_tcp_distance_m=distance,release_hold_s=self.release_time)
