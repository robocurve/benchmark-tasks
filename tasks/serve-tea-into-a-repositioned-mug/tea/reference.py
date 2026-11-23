"""State-feedback baseline. The liquid ledger and mug pose are privileged reference observations."""

import numpy as np
import mujoco as mj
from scipy.spatial.transform import Rotation
from .scene import DOWN, GRIP, HOME, LIP, MUG, TABLE
from .model import smooth


class IK:
    def __init__(self, env):
        self.m = env.m
        self.d = mj.MjData(env.m)
        self.d.qpos[:] = env.d.qpos
        self.site = self.m.site("left_tcp").id
        self.qadr = np.array(
            [self.m.joint("left_j" + str(i)).qposadr[0] for i in range(1, 7)]
        )
        self.dof = np.array(
            [self.m.joint("left_j" + str(i)).dofadr[0] for i in range(1, 7)]
        )
        self.limits = self.m.jnt_range[
            [self.m.joint("left_j" + str(i)).id for i in range(1, 7)]
        ]
        self.worst = 0.0

    def solve(self, pos, rot):
        d = self.d
        for _ in range(24):
            mj.mj_kinematics(self.m, d)
            mj.mj_comPos(self.m, d)
            ep = pos - d.site_xpos[self.site]
            er = Rotation.from_matrix(
                rot @ d.site_xmat[self.site].reshape(3, 3).T
            ).as_rotvec()
            if np.linalg.norm(ep) < 0.0002 and np.linalg.norm(er) < 0.004:
                break
            jp = np.zeros((3, self.m.nv))
            jr = jp.copy()
            mj.mj_jacSite(self.m, d, jp, jr, self.site)
            jac = np.vstack((jp[:, self.dof], 0.23 * jr[:, self.dof]))
            delta = jac.T @ np.linalg.solve(
                jac @ jac.T + 0.00012 * np.eye(6), np.r_[ep, 0.23 * er]
            )
            d.qpos[self.qadr] = np.clip(
                d.qpos[self.qadr] + np.clip(delta, -0.10, 0.10),
                self.limits[:, 0],
                self.limits[:, 1],
            )
        self.worst = max(self.worst, np.linalg.norm(ep))
        return d.qpos[self.qadr].copy()


class Server:
    def __init__(self, env, mode="adaptive"):
        self.env = env
        self.mode = mode
        self.ik = IK(env)
        self.start = env.state().copy()
        self.p0 = env.d.site("left_tcp").xpos.copy()
        self.r0 = env.d.site("left_tcp").xmat.reshape(3, 3).copy()
        self.home = np.array(env.d.xpos[env.pot]) + GRIP
        self.pitch = 0.0
        self.finish_at = None
        self.last_pose = None
        self.last_rot = None
        self.shift_start = None
        self.previous = None
        self.recovered = False

    def command(self):
        e = self.env
        t = e.d.time + 0.05
        if self.mode == "idle":
            return self.start.copy()
        cup = e.d.body("receiving_hand").xpos.copy()
        if self.mode == "contact_probe":
            goal = cup + [0.077, 0, 0.048]
            high = goal + [0, 0, 0.12]
            pos = (
                self.p0 + (high - self.p0) * smooth(t / 2)
                if t <= 2
                else high + (goal - high) * smooth((t - 2) / 2)
            )
            f = smooth(t / 2)
            rot = (
                Rotation.from_rotvec(
                    f * Rotation.from_matrix(DOWN @ self.r0.T).as_rotvec()
                ).as_matrix()
                @ self.r0
            )
            out = self.start.copy()
            out[:6] = self.ik.solve(pos, rot)
            out[6] = 0.35
            return out
        displaced = np.linalg.norm((cup - MUG)[:2]) > 0.0035
        speed = (
            0.0
            if self.previous is None
            else np.linalg.norm((cup - self.previous)[:2]) / 0.05
        )
        self.previous = cup.copy()
        if displaced:
            self.shift_start = t if self.shift_start is None else self.shift_start
        pause = speed > 0.009 or (
            self.shift_start is not None and self.mode == "pause_only"
        )
        if self.mode == "track":
            pause = False
        # Aim over the actual mug. Keep the angled spout above the rim through wrist roll.
        if (
            self.finish_at is None
            and e.cup + sum(p[2] for p in e.packets) + 0.24 * e.rate >= 120
        ):
            self.finish_at = t
            self.last_pose = e.d.site("left_tcp").xpos.copy()
            self.last_rot = e.d.site("left_tcp").xmat.reshape(3, 3).copy()
        approach = self.home + [-0.078, 0, 0.035]
        rest = self.home + [0, 0, 0.001]
        if t <= 1.2:
            f = smooth(t / 1.2)
            pos = self.p0 * (1 - f) + approach * f
            rot = (
                Rotation.from_rotvec(
                    f * Rotation.from_matrix(DOWN @ self.r0.T).as_rotvec()
                ).as_matrix()
                @ self.r0
            )
            gap = 0.36
        elif t <= 2.2:
            pos = approach + (rest - approach) * smooth((t - 1.2))
            rot = DOWN
            gap = 0.36
        elif t <= 2.95:
            pos = rest
            rot = DOWN
            gap = 0.0
        elif self.finish_at is None:
            goal_angle = 0.0 if t < 5.5 or pause else np.radians(81)
            self.pitch += np.clip(goal_angle - self.pitch, -0.115, 0.020)
            r = Rotation.from_rotvec([-self.pitch, 0, 0]).as_matrix()
            # Calibrated geometric offset for downward flight at the outlet.
            mouth = cup + [0, -0.008, 0.212]
            pose = mouth - r @ (LIP - GRIP)
            high = self.home + [0, 0, 0.143]
            if t < 4.0:
                pos = rest + (high - rest) * smooth((t - 2.95) / 1.05)
            else:
                pos = high + (pose - high) * smooth((t - 4.0) / 1.35)
            rot = r @ DOWN
            gap = 0.0
        else:
            z = t - self.finish_at
            rest = np.array([0.330, 0.085, TABLE]) + GRIP + [0, 0, 0.001]
            up = rest + [0, 0, 0.15]
            vertical = cup + [0, -0.008, 0.212] - (LIP - GRIP)
            if z < 0.52:
                f = smooth(z / 0.52)
                pos = self.last_pose + (vertical - self.last_pose) * f
                rot = (
                    Rotation.from_rotvec(
                        f * Rotation.from_matrix(DOWN @ self.last_rot.T).as_rotvec()
                    ).as_matrix()
                    @ self.last_rot
                )
                gap = 0.0
            elif self.mode == "early_release":
                pos = vertical
                rot = DOWN
                gap = 1.0
            elif self.mode == "hold":
                pos = vertical
                rot = DOWN
                gap = 0.0
            elif z < 2.3:
                pos = vertical + (up - vertical) * smooth((z - 0.52) / 1.78)
                rot = DOWN
                gap = 0.0
            elif z < 3.6:
                pos = up + (rest - up) * smooth((z - 2.3) / 1.3)
                rot = DOWN
                gap = 0.0
            else:
                # Free the jaw inside the loop at set-down height. Full opening at the
                # handle can wedge the other jaw against the pitcher under Inspect's slew limit.
                elapsed = z - 3.6
                exit_handle = 0.060 * smooth((elapsed - 0.60) / 0.55)
                lift = 0.060 * smooth((elapsed - 1.90) / 0.95)
                retreat = smooth((elapsed - 2.85) / 0.9)
                pos = rest + [-exit_handle - 0.031 * retreat, -0.035 * retreat,
                              lift + 0.033 * retreat]
                rot = DOWN
                gap = 0.36 if elapsed < 1.15 else 1.0
                if self.mode == "park_closed" and elapsed > 3.4:
                    gap = 0.0
        a = self.start.copy()
        a[:6] = self.ik.solve(pos, rot)
        a[6] = gap
        return a
