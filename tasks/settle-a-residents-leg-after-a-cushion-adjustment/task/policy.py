"""Joint targets from Cartesian waypoints. The feedback controller observes the lifted calf."""

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

from .station import HZ, READY


class Push:
    def __init__(self, model, mode):
        self.model, self.mode = model, mode
        self.ikdata = mj.MjData(model)
        self.joints = READY.copy()
        self.start = None
        self.home = None
        self.release = None
        self.last = None
        self.rotation = Rotation.from_matrix(
            [[0, -0.5, 0.8660254], [-1, 0, 0], [0, -0.8660254, -0.5]]
        ).as_matrix()

    def solve(self, target):
        if self.last is not None and np.linalg.norm(target - self.last) < 1e-7:
            return self.joints
        self.last = target.copy()
        d, m = self.ikdata, self.model
        tcp = m.site("tcp").id
        jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        for _ in range(35):
            d.qpos[:6] = self.joints
            mj.mj_kinematics(m, d)
            mj.mj_comPos(m, d)
            error = np.r_[
                target - d.site_xpos[tcp],
                0.24
                * Rotation.from_matrix(
                    self.rotation @ d.site_xmat[tcp].reshape(3, 3).T
                ).as_rotvec(),
            ]
            if np.linalg.norm(error) < 0.0003:
                break
            mj.mj_jacSite(m, d, jp, jr, tcp)
            J = np.vstack((jp[:, :6], 0.24 * jr[:, :6]))
            dq = J.T @ np.linalg.solve(J @ J.T + np.eye(6) * 0.00018, error)
            self.joints = np.clip(
                self.joints + np.clip(dq, -0.14, 0.14),
                m.jnt_range[:6, 0],
                m.jnt_range[:6, 1],
            )
        return self.joints

    def command(self, tick, obs):
        if self.home is None:
            self.home = np.array(obs["tcp"])
        t = tick / HZ
        # Direct measurement of posture. Lift onset is deliberately variable between trials.
        lift_ready = t >= 1.6 if self.mode == "clock" else obs["heel_gap_mm"] > 105
        if self.start is None and lift_ready:
            self.start = t
        y = 0.038 if self.mode == "offset" else 0.020
        hover = np.array([0.055, y, 0.16])
        ready = np.array([0.055, y, 0.040])
        dest = np.array([0.338, y, 0.040])
        if self.mode == "short":
            dest[0] = 0.175
        if self.mode == "edge":
            dest[0] = 0.265
        park = np.array([0.12, y, 0.18])
        grip = -0.029
        if self.mode == "idle":
            return np.r_[READY, grip, grip]
        travel = 2.65 if self.mode == "slow" else 1.85
        start = max(1.55, self.start if self.start is not None else t + 1)
        end = start + travel
        poses = [
            (0.0, self.home),
            (0.85, hover),
            (1.5, ready),
            (start, ready),
            (end, dest),
        ]
        if self.release is None and t >= end + 0.32:
            settled = (
                obs["settled_support"]
                if "settled_support" in obs
                else (
                    obs["prox_n"] + obs["dist_n"] > 12
                    and abs(obs["knee_velocity"]) < 0.08
                )
            )
            if settled or self.mode in ("clock", "short", "edge") or t >= 9.6:
                self.release = t
        if self.mode != "linger" and self.release is not None:
            release = self.release
            poses.extend(
                [
                    (release, dest),
                    (release + 0.45, dest + [0, 0, 0.115]),
                    (release + 1.4, park),
                ]
            )
        if self.mode == "bed_touch" and self.release is not None:
            # Briefly touch an empty portion of the bed after clearing the cushion.
            poses.extend(
                [
                    (10.5, park),
                    (11.0, np.array([0.10, y, -0.010])),
                    (11.45, np.array([0.10, y, -0.010])),
                    (11.90, park),
                ]
            )
        points = np.array([p for _, p in poses])
        times = [s for s, _ in poses]
        target = np.array([np.interp(t, times, col) for col in points.T])
        if self.mode == "park_closed" and t > end + 1.5:
            grip = 0.0
        return np.r_[self.solve(target), grip, grip]
