"""Position servo and Cartesian IK for the calibrated dual-YAM arms."""

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

ARMS = ('left', 'right')
HOME = np.array([0., 1., 1., -.6, 0., 0., 1.])


def direction(yaw, pitch, flip=False):
    a, b = np.deg2rad([yaw, pitch])
    approach = np.array([-np.sin(a)*np.cos(b), -np.cos(a)*np.cos(b), -np.sin(b)])
    tangent = np.cross([0, 0, 1], approach)
    tangent /= np.linalg.norm(tangent)
    closure = np.cross(approach, tangent)
    if flip:
        tangent, closure = -tangent, -closure
    return np.column_stack((tangent, closure, approach))


class Arm:
    def __init__(self, model, side):
        self.model = model
        self.site = model.site(f'{side}_tcp').id
        self.joints = [model.joint(f'{side}_joint{i}') for i in range(1, 7)]
        self.address = [j.qposadr[0] for j in self.joints]
        self.dofs = [j.dofadr[0] for j in self.joints]
        self.limits = np.stack([j.range for j in self.joints])
        self.finger_names = [f'{side}_finger_{i}' for i in (-1, 1)]
        self.actuators = [model.actuator(j.name).id for j in self.joints]
        self.fingers = [model.actuator(f).id for f in self.finger_names]
        self.side = side

    def command(self, data, action):
        data.ctrl[self.actuators] = action[:6]
        data.ctrl[self.fingers] = action[6]*.0475

    def reset(self, data):
        data.qpos[self.address] = HOME[:6]
        for name in self.finger_names:
            data.joint(name).qpos[:] = .0475
        self.command(data, HOME)

    def inverse(self, data, target, rotation, seed):
        q = seed.copy()
        j, j_rot = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
        for _ in range(150):
            data.qpos[self.address] = q
            mj.mj_kinematics(self.model, data)
            mj.mj_comPos(self.model, data)
            ep = target - data.site_xpos[self.site]
            er = Rotation.from_matrix(rotation @ data.site_xmat[self.site].reshape(3, 3).T).as_rotvec()
            if np.linalg.norm(ep) < .00012 and np.linalg.norm(er) < .002:
                return q
            mj.mj_jacSite(self.model, data, j, j_rot, self.site)
            jac = np.vstack((j[:, self.dofs], .24*j_rot[:, self.dofs]))
            q += np.clip(jac.T @ np.linalg.solve(jac @ jac.T + .013**2*np.eye(6), np.r_[ep, .24*er]), -.24, .24)
            q = np.clip(q, self.limits[:, 0], self.limits[:, 1])
        raise ValueError(f'{self.side} misses {target} by {np.linalg.norm(ep):.4f}m and {np.linalg.norm(er):.4f}rad')
