"""Experimental elastic-perfectly-plastic crease memory, not calibrated paper.

Rest angles change only after actual relative deformation exceeds the elastic
range. No task goal, robot action, time schedule or success state enters this law.
Native joint drives supply the forces; this class never writes body poses.
"""
import numpy as np
from scipy.spatial.transform import Rotation

STIFFNESS_NM_PER_DEG = .0005
YIELD_TORQUE_NM = .001
DAMPING = .00001


class CreaseMemory:
    def __init__(self, hinges, names, initial_poses):
        index={n:i for i,n in enumerate(names)}
        self.hinges=[h for h in hinges if h[0] not in ('fin_minus','fin_plus')]
        self.parents=np.array([index[h[1]] for h in self.hinges])
        self.children=np.array([index[h[2]] for h in self.hinges])
        self.axes=np.array([(np.asarray(h[4])-h[3])/np.linalg.norm(np.asarray(h[4])-h[3]) for h in self.hinges])
        self.previous=self.raw_angles(initial_poses)
        self.rest=self.previous.copy()
        self.plastic_work_J=0.
        self.plastic_rotation_deg=np.zeros(len(self.hinges))

    def raw_angles(self, poses):
        q=np.asarray(poses)
        r=Rotation.from_quat(q[self.parents,3:]).inv()*Rotation.from_quat(q[self.children,3:])
        quat=r.as_quat()
        angle=np.rad2deg(2*np.arctan2(np.sum(quat[:,:3]*self.axes,axis=1),quat[:,3]))
        return (angle+180)%360-180

    def update(self, poses):
        angle=self.previous+(self.raw_angles(poses)-self.previous+180)%360-180
        elastic_range=YIELD_TORQUE_NM/STIFFNESS_NM_PER_DEG
        error=angle-self.rest
        change=np.sign(error)*np.maximum(np.abs(error)-elastic_range,0.)
        self.rest+=change
        self.plastic_rotation_deg+=np.abs(change)
        self.plastic_work_J+=YIELD_TORQUE_NM*np.deg2rad(np.abs(change)).sum()
        self.previous=angle
        return self.rest.copy()

    def native_drives(self, stage, joint_root):
        from pxr import UsdPhysics
        drives=[]
        for h,rest in zip(self.hinges,self.rest):
            prim=stage.GetPrimAtPath(joint_root+'/'+h[0])
            d=UsdPhysics.DriveAPI.Apply(prim,'angular')
            d.CreateTypeAttr('force');d.CreateStiffnessAttr(STIFFNESS_NM_PER_DEG)
            d.CreateDampingAttr(DAMPING);d.CreateMaxForceAttr(YIELD_TORQUE_NM)
            d.CreateTargetVelocityAttr(0.);d.CreateTargetPositionAttr(float(rest))
            drives.append(d.GetTargetPositionAttr())
        return drives
