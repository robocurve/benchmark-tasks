"""Latched elastic-domain diagnostic for the native rim (not a tear renderer).

The native material callback checks this at every substep. The0.3% axial/bending bound
corresponds to207MPa with E=69GPa, below Hydro's240MPa minimum0.2% proof stress
for6061-T6 extrusions. It is a conservative engineering validity choice, not a
commercial wheel calibration. Torsion is limited separately; the simplified
ring has no validated warping or permanent rim-damage model.

Reference: https://www.hydro.com/globalassets/01-products--services/extruded-profiles/americas/ena-resources/alloy-data-sheets/hydro_2019_data_sheet_6061.pdf
"""
import math
import numpy as np
from mechanics import WheelSpec


def rim_strain(centres,rotations,spec=None):
    s=spec or WheelSpec();p=np.asarray(centres,float);R=np.asarray(rotations,float)
    if p.shape!=(s.spokes,3) or R.shape!=(s.spokes,3,3) or not np.isfinite(p).all() or not np.isfinite(R).all():
        raise ValueError('Finite native rim centres and rotations required')
    chord=2*s.rim_radius_m*math.sin(math.pi/s.spokes);a=math.pi/s.spokes
    Z=np.array(((math.cos(a),-math.sin(a),0.),(math.sin(a),math.cos(a),0.),(0.,0.,1.)))
    other=np.roll(np.arange(s.spokes),-1)
    f0=R@Z;f1=R[other]@Z.T
    delta=p[other]-R[other,:,0]*chord/2-p-R[:,:,0]*chord/2
    gap=np.einsum('nji,nj->ni',f0,delta)
    relative=np.swapaxes(f0,1,2)@f1
    cosine=np.clip((np.trace(relative,axis1=1,axis2=2)-1)/2,-1.,1.)
    angle=np.arccos(cosine)
    if np.any(angle>math.pi-.001):raise ValueError('Folded rim joint outside elastic model')
    scale=np.full(s.spokes,.5)
    mask=angle>1e-7;scale[mask]=angle[mask]/(2*np.sin(angle[mask]))
    rotation_vector=np.stack((relative[:,2,1]-relative[:,1,2],relative[:,0,2]-relative[:,2,0],relative[:,1,0]-relative[:,0,1]),axis=1)*scale[:,None]
    # Axis x is tangent; y radial, z along axle. Take the conservative maximum
    # absolute axial strain over the four corners of the20×22mm rim envelope.
    strain=np.abs(gap[:,0])/chord+(np.abs(rotation_vector[:,1])*.011+np.abs(rotation_vector[:,2])*.010)/chord
    return {'maximum_axial_bending_strain':float(strain.max()),
            'maximum_twist_rad':float(np.max(np.abs(rotation_vector[:,0]))),
            'maximum_locked_joint_gap_m':float(np.max(np.linalg.norm(gap[:,1:],axis=1)))}


class RimDomainGuard:
    def __init__(self):
        self.failed=False;self.first_failure=None;self.steps=0
        self.peaks={name:0. for name in ('maximum_axial_bending_strain',
            'maximum_twist_rad','maximum_locked_joint_gap_m')}
        self.latest={}

    def summary(self):
        return {'valid':not self.failed,'steps':self.steps,'first_failure':self.first_failure,
                **self.latest,'peaks':dict(self.peaks)}
    def update(self,centres,rotations):
        self.steps+=1
        try:
            values=rim_strain(centres,rotations)
            valid=(values['maximum_axial_bending_strain']<=.003 and
                   values['maximum_twist_rad']<=.01 and values['maximum_locked_joint_gap_m']<=.0001)
        except ValueError as error:
            values={'error':str(error)};valid=False
        self.latest=values
        for name in self.peaks:
            if name in values:self.peaks[name]=max(self.peaks[name],values[name])
        if not valid:
            self.failed=True;self.first_failure=self.first_failure or {'step':self.steps,**values}
        return self.summary()
