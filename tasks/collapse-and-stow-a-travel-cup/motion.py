"""Cartesian joint-target planning for the two standard YAM arms."""

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp

HOME = np.array((0., 1.047, 1.047, -0.35, 0., 0.))
SIDES = ("left", "right")
GRIP_FEATURE = np.array((.044, 0, .001))


def frame(approach, closing):
    z = np.array(approach, dtype=float)
    z /= np.linalg.norm(z)
    y = np.array(closing, dtype=float)
    y -= y@z*z
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.column_stack((x,y,z))


DOWN = frame((0,0,-1), (0,1,0))
FROM_LEFT = frame((0,-1,0), (1,0,0))
FROM_RIGHT = frame((0,1,0), (-1,0,0))


class Motion:
    def __init__(self, model):
        self.model = model
        self.calculation = mujoco.MjData(model)
        self.jac_pos = np.zeros((3, model.nv))
        self.jac_rot = np.zeros((3, model.nv))
        self.axes = {}
        for side in SIDES:
            joints = [model.joint(f"{side}_joint{i}") for i in range(1,7)]
            self.axes[side] = (np.array([j.qposadr[0] for j in joints]),
                               np.array([j.dofadr[0] for j in joints]),
                               np.array([j.range for j in joints]))

    def solve(self, side, target, orientation, previous, angular_tolerance=.052):
        qpos, dofs, limits = self.axes[side]
        data = self.calculation
        site = self.model.site(f"{side}_grasp_site").id
        keep = .001
        target_site = np.array(target) - orientation @ GRIP_FEATURE

        def residual(joints):
            data.qpos[qpos] = joints
            mujoco.mj_kinematics(self.model, data)
            actual = data.site_xmat[site].reshape(3,3)
            rot_error = Rotation.from_matrix(actual @ orientation.T).as_rotvec()
            return np.r_[data.site_xpos[site]-target_site, .115*rot_error, keep*(joints-previous)]

        def jacobian(joints):
            data.qpos[qpos] = joints
            mujoco.mj_kinematics(self.model, data)
            mujoco.mj_comPos(self.model, data)
            mujoco.mj_jacSite(self.model, data, self.jac_pos, self.jac_rot, site)
            return np.vstack((self.jac_pos[:,dofs], .115*self.jac_rot[:,dofs], keep*np.eye(6)))

        result = least_squares(residual, np.clip(previous,limits[:,0]+1.e-7,limits[:,1]-1.e-7),
                               jac=jacobian, bounds=limits.T, max_nfev=50, gtol=1e-7)
        miss = residual(result.x)
        if np.linalg.norm(miss[:3])>.003 or np.linalg.norm(miss[3:6])>.115*angular_tolerance:
            raise ValueError(f"{side} unreachable {np.round(target,3)} at {np.round(result.x,2)}, "
                             f"offset {np.round(miss[:3],4)}, angle {np.linalg.norm(miss[3:6])/.115:.3f}")
        return result.x

    @staticmethod
    def feature(data, side):
        site = data.site(f"{side}_grasp_site")
        rotation = site.xmat.copy().reshape(3,3)
        return site.xpos.copy()+rotation@GRIP_FEATURE, rotation
