"""A world without contact physics for dry runs of the demonstration and of the scorer. NumPy only.

The arms follow their joint commands with a first-order lag plus a constant joint offset (sag). The funnel
stands on its wipe until the left pads close on its body (fingertip centre within 6 mm of the funnel's axis,
at a height on its body); it then moves with the hand, except that the base's flange stops it from below.
Let go on the base it stays where it is; let go anywhere else it falls (a failure). The clamp lies shut until
the right pads close on its handles at the grip point; held fully closed, the jaws open to 27 degrees. Let go
with its ring within 3 mm of the axis and 2 mm of the joint's height, it closes on the flanges (and, if the
funnel is not held, shoves it by the ring's own offset, as the MuJoCo probe saw; README "Physics probe");
anywhere else it falls. This checks reach, joint limits, control flow and the scorer's logic; it says nothing
about finger contact, the clamp's spring or the glass's friction.
"""

import numpy as np

from rdb_filter_clamp import fixed_scene as fixture
from rdb_filter_clamp import layout as L

OPEN = .04695
SPAN = 2 * OPEN


class World:
    def __init__(self, ik, *, lag_s=.12, sag=(0., .02, 0., .05, 0., 0.), slip_mm=(0., 0., 0.), clearance=None):
        from yam_arena.constants import READY

        self.ik = ik
        self.blend = 1 - np.exp(-(1 / fixture.CONTROL_HZ) / lag_s)
        self.sag = np.asarray(sag, dtype=float)
        self.slip = np.asarray(slip_mm, dtype=float) / 1000
        self.q = {arm: np.asarray(READY, dtype=float) + self.sag for arm in fixture.ARMS}
        self.reading = dict.fromkeys(fixture.ARMS, 1.)           # commanded gripper reading, followed with a lag
        self.holder = (fixture.HOLDER_START.copy(), np.eye(3))
        self.funnel = [fixture.FUNNEL_START.copy(), np.eye(3)]
        self.clamp = [fixture.CLAMP_START.copy(), np.eye(3)]
        self.jaw = L.THETA_MIN
        self.funnel_mode, self.clamp_mode = "wipe", "bench"       # wipe/held/seated/fallen; bench/held/on/fallen
        self.funnel_grip = self.clamp_grip = None
        self.clearance = clearance
        self.report = {"worst_limit_margin_deg": 180., "min_arm_gap_mm": np.inf, "min_gap_table_mm": np.inf,
                       "min_right_to_funnel_mm": np.inf, "min_left_to_clamp_ring_mm": np.inf,
                       "min_right_to_fitted_clamp_mm": np.inf, "events": []}
        self.tick = 0

    # -- what the demonstration and the scorer read ------------------------------------------------------
    def joint_pos(self):
        out = []
        for arm in fixture.ARMS:
            shown = self.reading[arm]
            if arm == "left" and self.funnel_mode == "held":
                shown = max(shown, 2 * L.BODY_R / SPAN)
            if arm == "right" and self.clamp_mode == "held":
                shown = max(shown, .0115 / SPAN)
            out.append(np.r_[self.q[arm], shown])
        return np.concatenate(out)

    def hand(self, arm):
        return self.ik.tcp_pose(self.q[arm], fixture.BASES[arm])

    def state(self):
        return {"holder": (self.holder[0].copy(), self.holder[1].copy()), "funnel": (self.funnel[0].copy(), self.funnel[1].copy()),
                "clamp": (self.clamp[0].copy(), self.clamp[1].copy()),
                "jaw_open_deg": [float(np.degrees(self.jaw))] * 2}

    def grade_state(self):
        joints = self.joint_pos()
        tcps, rots, openings = [], [], []
        for i, arm in enumerate(fixture.ARMS):
            p, r = self.hand(arm)
            tcps.append(p)
            rots.append(r)
            openings.append(1000 * SPAN * float(joints[7 * i + 6]))
        s = self.state()
        return {"holder_pos": s["holder"][0], "holder_rot": s["holder"][1], "funnel_pos": s["funnel"][0],
                "funnel_rot": s["funnel"][1], "clamp_pos": s["clamp"][0], "clamp_rot": s["clamp"][1],
                "jaw_open_deg": s["jaw_open_deg"], "tcp": tcps, "tcp_rot": rots, "opening_mm": openings}

    # -- one control step ----------------------------------------------------------------------------
    def step(self, command):
        self.tick += 1
        command = np.asarray(command, dtype=float)
        for i, arm in enumerate(fixture.ARMS):
            q = command[7 * i:7 * i + 6]
            if np.any(q < self.ik.LOWER - 1e-6) or np.any(q > self.ik.UPPER + 1e-6):
                raise RuntimeError(f"{arm} arm commanded past a joint limit at tick {self.tick}: {q}")
            self.report["worst_limit_margin_deg"] = min(self.report["worst_limit_margin_deg"], float(
                np.degrees(np.minimum(q - self.ik.LOWER, self.ik.UPPER - q).min())))
            self.q[arm] = self.q[arm] + self.blend * (q + self.sag - self.q[arm])
            self.reading[arm] = self.reading[arm] + .6 * (command[7 * i + 6] - self.reading[arm])
        self._funnel()
        self._clamp()
        if self.clearance is not None and self.tick % 3 == 0:
            self._gaps()

    def _funnel(self):
        point, rot = self.hand("left")
        gap = self.reading["left"] * SPAN
        if self.funnel_mode in ("wipe", "seated"):
            local = self.funnel[1].T @ (point - self.funnel[0])
            if gap < 2 * L.BODY_R - .001 and np.hypot(local[0], local[1]) < .006 and .03 < local[2] < L.FUNNEL_H:
                self.funnel_grip = (rot.T @ (self.funnel[0] - point) + self.slip, rot.T @ self.funnel[1])
                self.funnel_mode = "held"
                self.report["events"].append((self.tick, "funnel gripped"))
        elif self.funnel_mode == "held":
            if gap > 2 * L.BODY_R + .002:
                seat = L.funnel_on_base(*self.holder, *self.funnel)
                self.funnel_mode = "seated" if seat["gap"] < .001 and seat["offset"] < L.FLANGE_R - .005 else "fallen"
                if self.funnel_mode == "fallen":
                    self.funnel[0] = self.funnel[0] * (1, 1, 0) + (0., 0., .0)
                    self.funnel[1] = np.array(((1., 0., 0.), (0., 0., -1.), (0., 1., 0.)))     # lying on its side
                self.report["events"].append((self.tick, f"funnel let go: {self.funnel_mode}"))
                return
            k, r = self.funnel_grip
            p = point + rot @ k
            # the base's flange stops it from below where the two overlap
            if np.hypot(*(p[:2] - self.holder[0][:2])) < 2 * L.FLANGE_R:
                p[2] = max(p[2], L.JOINT_Z)
            self.funnel[0], self.funnel[1] = p, np.eye(3) if np.hypot(*(p[:2] - self.holder[0][:2])) < .01 and p[2] < L.JOINT_Z + .002 else rot @ r

    def _clamp(self):
        point, rot = self.hand("right")
        reading = self.reading["right"]
        if self.clamp_mode == "bench":
            local = self.clamp[1].T @ (point - self.clamp[0]) - L.GRIP_IN_CLAMP
            if reading < .2 and np.linalg.norm(local) < .006:
                self.clamp_grip = (rot.T @ (self.clamp[0] - point), rot.T @ self.clamp[1])
                self.clamp_mode = "held"
                self.report["events"].append((self.tick, "clamp gripped"))
        if self.clamp_mode == "held":
            k, r = self.clamp_grip
            self.clamp[0], self.clamp[1] = point + rot @ k, rot @ r
            # fully squeezed the handles touch and the jaws stand at their widest; opening the pads lets them close
            self.jaw = L.THETA_MAX if reading < .2 else max(L.THETA_MIN, L.THETA_MAX - (reading - .2) * 3.)
            if reading > .35:
                ring = L.clamp_on_joint(*self.holder, *self.clamp, [0., 0.])
                if ring["off_axis"] < .003 and abs(ring["height"]) < .002 and self.funnel_mode in ("seated", "held"):
                    self.clamp_mode = "on"
                    if self.funnel_mode == "seated":                # nobody holds it: the closing jaws shove it
                        shove = self.clamp[0][:2] - self.holder[0][:2]
                        self.funnel[0][:2] -= shove
                    self.clamp[0] = self.holder[0] + (0., 0., L.JOINT_Z)
                    self.clamp[1] = np.eye(3)
                    self.jaw = np.radians(1.4)
                else:
                    self.clamp_mode = "fallen"
                    self.clamp[0] = np.r_[self.clamp[0][:2], .009]
                    self.jaw = L.THETA_MIN
                self.report["events"].append((self.tick, f"clamp let go: {self.clamp_mode}"))

    def _gaps(self):
        C = self.clearance
        tips = ("gripper", "tip_left", "tip_right", "link6", "link5", "link4")
        left = C.arm_cloud(self.q["left"], self.reading["left"], fixture.BASES["left"])
        right = C.arm_cloud(self.q["right"], self.reading["right"], fixture.BASES["right"])
        self._least("min_arm_gap_mm", 1000 * C.arm_gap(left, right))
        for cloud in (C.arm_cloud(self.q["left"], self.reading["left"], fixture.BASES["left"], links=tips),
                      C.arm_cloud(self.q["right"], self.reading["right"], fixture.BASES["right"], links=tips)):
            self._least("min_gap_table_mm", 1000 * float(cloud[:, 2].min()))
        if self.funnel_mode in ("seated", "held") and self.clamp_mode in ("held", "on"):
            fp = self.funnel[0]
            r = C.arm_cloud(self.q["right"], self.reading["right"], fixture.BASES["right"])
            band = r[(r[:, 2] > fp[2] + L.NECK_TOP) & (r[:, 2] < fp[2] + L.FUNNEL_H)]
            if len(band):
                self._least("min_right_to_funnel_mm", 1000 * float((np.hypot(band[:, 0] - fp[0], band[:, 1] - fp[1]) - L.BODY_R).min()))
            cp = self.clamp[0]
            band = left[np.abs(left[:, 2] - cp[2]) < L.HALF_H + .003]
            if len(band):
                self._least("min_left_to_clamp_ring_mm", 1000 * float((np.hypot(band[:, 0] - cp[0], band[:, 1] - cp[1]) - L.OUT_R).min()))

        if self.clamp_mode == "on" and self.reading["right"] > .85:   # the right arm leaving a fitted clamp (run 2: it hooked a handle)
            cp = self.clamp[0]
            pts = np.vstack([L.jaw_points(side, np.radians(1.4)) for side in (1, -1)]) + cp
            from scipy.spatial import cKDTree
            self._least("min_right_to_fitted_clamp_mm", 1000 * float(cKDTree(pts).query(right)[0].min()))

    def _least(self, key, value):
        if value < self.report[key]:
            self.report[key] = value
            self.report[key.replace("min_", "tick_of_min_")] = self.tick
