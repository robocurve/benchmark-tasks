"""Damped least-squares IK for one YAM arm's tool-centre site (``tcp_<side>``).

Tool frame: +z is the approach axis, +y is the finger slide axis. A top-down
grasp is z = world -z with the fingers along a chosen yaw.
"""

from __future__ import annotations

import mujoco
import numpy as np

from robosci.scene import ARM_JOINTS


def topdown_rot(yaw: float) -> np.ndarray:
    """Rotation whose z points down and whose y (fingers) points along ``yaw``."""
    y = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    z = np.array([0.0, 0.0, -1.0])
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


def _rot_err(r_cur: np.ndarray, r_tgt: np.ndarray) -> np.ndarray:
    q_cur, q_tgt, q_err = np.zeros(4), np.zeros(4), np.zeros(4)
    mujoco.mju_mat2Quat(q_cur, r_cur.ravel())
    mujoco.mju_mat2Quat(q_tgt, r_tgt.ravel())
    neg = np.zeros(4)
    mujoco.mju_negQuat(neg, q_cur)
    mujoco.mju_mulQuat(q_err, q_tgt, neg)
    v = np.zeros(3)
    mujoco.mju_quat2Vel(v, q_err, 1.0)
    return v


class ArmIK:
    """IK solver bound to one arm of a compiled model. Uses a scratch MjData."""

    def __init__(self, model: mujoco.MjModel, side: str) -> None:
        self.m = model
        self.d = mujoco.MjData(model)
        self.site = model.site(f"tcp_{side}").id
        self.jids = [model.joint(j).id for j in ARM_JOINTS[side]]
        self.qadr = np.array([model.jnt_qposadr[j] for j in self.jids])
        self.dadr = np.array([model.jnt_dofadr[j] for j in self.jids])
        self.lo = model.jnt_range[self.jids, 0]
        self.hi = model.jnt_range[self.jids, 1]

    def fk(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Tool position and rotation for arm joints ``q``."""
        self.d.qpos[self.qadr] = q
        mujoco.mj_kinematics(self.m, self.d)
        mujoco.mj_comPos(self.m, self.d)
        return self.d.site_xpos[self.site].copy(), self.d.site_xmat[self.site].reshape(3, 3).copy()

    def solve(self, pos: np.ndarray, rot: np.ndarray, q0: np.ndarray, *, iters: int = 300,
              tol: float = 1e-3, restarts: int = 6, seed: int = 0) -> tuple[np.ndarray, float]:
        """Return joints reaching ``pos``/``rot`` and the final position error (m)."""
        rng = np.random.default_rng(seed)
        best_q, best_err = q0.copy(), np.inf
        starts = [np.clip(q0, self.lo, self.hi)]
        starts += [rng.uniform(self.lo, self.hi) for _ in range(restarts)]
        jp, jr = np.zeros((3, self.m.nv)), np.zeros((3, self.m.nv))
        for q in starts:
            q = q.copy()
            for _ in range(iters):
                p, r = self.fk(q)
                e = np.concatenate([pos - p, 0.5 * _rot_err(r, rot)])
                if np.linalg.norm(e[:3]) < tol and np.linalg.norm(e[3:]) < 0.01:
                    break
                mujoco.mj_jacSite(self.m, self.d, jp, jr, self.site)
                jac = np.vstack([jp[:, self.dadr], 0.5 * jr[:, self.dadr]])
                dq = jac.T @ np.linalg.solve(jac @ jac.T + 1e-4 * np.eye(6), e)
                q = np.clip(q + np.clip(dq, -0.2, 0.2), self.lo, self.hi)
            p, r = self.fk(q)
            err = np.linalg.norm(pos - p) + 0.05 * np.linalg.norm(_rot_err(r, rot))
            if err < best_err:
                best_q, best_err = q, err
            if err < 2 * tol:
                break
        return best_q, best_err
