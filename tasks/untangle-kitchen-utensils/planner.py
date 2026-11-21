"""YAM waypoint planner using the public robot model and a private FK workspace."""

import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from station_model import add_arms


def frame(approach, closing):
    z = np.array(approach, dtype=float)
    z /= np.linalg.norm(z)
    x = np.array(closing, dtype=float)
    x -= z * (x @ z)
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


class Planner:
    """Only the robot's public kinematic model enters the planner."""

    def __init__(self, joint_pos):
        robot = ET.fromstring(
            '<mujoco><compiler angle="radian"/><asset/><worldbody/><contact/><actuator/></mujoco>'
        )
        add_arms(robot)
        self.model = model = mujoco.MjModel.from_xml_string(ET.tostring(robot, encoding="unicode"))
        self.kin = mujoco.MjData(model)
        self.q = np.array(joint_pos, dtype=float).copy().reshape(2, 7)
        self.joints = []
        self.dofs = []
        self.sites = []
        for arm, side in enumerate(("left", "right")):
            joints = [model.joint(f"{side}_joint{i}") for i in range(1, 7)]
            self.joints.append([j.qposadr[0] for j in joints])
            self.dofs.append([j.dofadr[0] for j in joints])
            self.sites.append(model.site(side + "_tcp").id)
            self.kin.qpos[self.joints[arm]] = self.q[arm, :6]
        mujoco.mj_kinematics(model, self.kin)

    def pose(self, arm):
        site = self.sites[arm]
        return self.kin.site_xpos[site].copy(), self.kin.site_xmat[site].reshape(3, 3).copy()

    def ik(self, arm, pos, rot):
        site = self.sites[arm]
        j = self.joints[arm]
        dof = self.dofs[arm]
        jp, jr = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
        bounds = self.model.jnt_range[[self.model.dof_jntid[i] for i in dof]]
        for _ in range(25):
            self.kin.qpos[j] = self.q[arm, :6]
            mujoco.mj_kinematics(self.model, self.kin)
            mujoco.mj_comPos(self.model, self.kin)
            now, orientation = self.pose(arm)
            err = np.r_[pos - now, 0.22 * Rotation.from_matrix(rot @ orientation.T).as_rotvec()]
            if np.linalg.norm(err) < 0.000035:
                break
            mujoco.mj_jacSite(self.model, self.kin, jp, jr, site)
            jac = np.vstack((jp[:, dof], 0.22 * jr[:, dof]))
            update = jac.T @ np.linalg.solve(jac @ jac.T + np.eye(6) * 0.00008, err)
            self.q[arm, :6] = np.clip(
                self.q[arm, :6] + np.clip(update, -0.18, 0.18), bounds[:, 0], bounds[:, 1]
            )

    def move(self, arm, pos, rot, opening, steps):
        p0, r0 = self.pose(arm)
        g0 = self.q[arm, 6]
        relative = Rotation.from_matrix(rot @ r0.T).as_rotvec()
        for k in range(1, steps + 1):
            u = k / steps
            s = u * u * (3 - 2 * u)
            self.ik(arm, p0 + s * (pos - p0), Rotation.from_rotvec(relative * s).as_matrix() @ r0)
            self.q[arm, 6] = g0 + s * (opening - g0)
            yield self.q.ravel().copy()
