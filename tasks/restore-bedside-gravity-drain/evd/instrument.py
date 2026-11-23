"""Take a surveyed reading from live body poses. Supply volume is an integrated trainer model."""

import gzip
import json
import math
from collections import deque
from dataclasses import asdict

import numpy as np
from scipy.spatial.transform import Rotation

from checks.scorer import Frame
from evd.gripper_clearance import lateral_gap_mm
from evd.layout import DATUM, LAYOUTS, LINKS, ROOT
from evd.visual import PATIENT_LINE, loop_points

BOXES = json.loads((ROOT / "assets/link_bounds.json").read_text())["bounds"]
CENTERS = np.asarray([BOXES[n][0] for n in LINKS] * 2)
EXTENTS = np.asarray([BOXES[n][1] for n in LINKS] * 2)
# Analytical bounding boxes enclose every contact surface of the mannequin head, neck and torso.
ANATOMY = np.array(
    [
        [0.538, 0.22, 0.012, 0.095, 0.075, 0.0775],
        [0.483, 0.161, 0.003, 0.017, 0.0105, 0.026],
        [0.478, 0.152, 0, 0.0055, 0.0045, 0.006],
        [0.518, 0.209, 0.086, 0.0185, 0.016, 0.0185],
        [0.631, 0.221, -0.025, 0.04, 0.047, 0.0475],
        [0.775, 0.221, -0.039, 0.135, 0.12, 0.0615],
    ]
)


class Monitor:
    def __init__(self, case, blocked=False, output=None):
        self.case = case
        self.blocked = blocked
        self.ear0, self.zero0 = LAYOUTS[case]
        self.history = deque(maxlen=7201)
        self.open_count = self.closed_count = self.sealed = 0
        self.tick = -1
        self.volume = 0.0
        self.previous_rate = 0.0
        self.fault = None
        self.score = 0.0
        self.first_fault_tick = None
        self.range_z = [1000.0, -1000.0]
        self.minimum_gripper_gap = {}
        self.max_angle = 0.0
        self.first_open = None
        self.trace = gzip.open(output, "wt", compresslevel=1) if output else None

    def close(self):
        if self.trace is not None:
            self.trace.close()
            self.trace = None

    def sample(self, tick, poses, robot):
        if tick != self.tick + 1:
            raise ValueError("Nonconsecutive PhysX samples")
        self.tick = tick
        scene = np.array(
            [poses[n] for n in ("patient", "carriage", "brake", "tap")], dtype=float
        )
        arms = np.concatenate([robot[s] for s in ("left", "right")])
        if (
            not np.isfinite(scene).all()
            or arms.shape != (18, 7)
            or not np.isfinite(arms).all()
        ):
            raise ValueError("Missing live body pose")
        qnorm = np.linalg.norm(np.vstack([scene[:, 3:], arms[:, 3:]]), axis=1)
        if np.any(abs(qnorm - 1) > 0.002):
            raise ValueError("Invalid body quaternion")
        rot = Rotation.from_quat(scene[:, 3:]).as_matrix()
        ear = (scene[0, 2] - DATUM) * 1000
        zero = (scene[1, 2] - DATUM) * 1000
        sill = zero + rot[1, 2, 2] * 100
        taprot = rot[1].T @ rot[3]
        raw_angle = math.degrees(math.atan2(taprot[1, 2], taprot[2, 2]))
        slide_local = rot[1].T @ (scene[2, :3] - scene[1, :3])
        raw_gap = float(slide_local[0] * 1000)
        # Project solver compliance at the two manufactured hard stops, retain raw excursions in the trace.
        if not -4 <= raw_angle <= 94 or not -1 <= raw_gap <= 9.5:
            raise ValueError("Measured mechanical stop overtravel")
        angle = float(np.clip(raw_angle, 0, 90))
        gap = float(np.clip(raw_gap, 0, 8))
        anchor = scene[1, :3] + rot[1] @ np.array([-0.015, -0.085, -0.050])
        intact = (
            np.linalg.norm(anchor - scene[3, :3]) <= 0.002
            and np.linalg.norm(slide_local[1:] - [-0.065, 0.045]) <= 0.002
        )
        arm_rot = Rotation.from_quat(arms[:, 3:]).as_matrix()
        gripper_gap = lateral_gap_mm(arms[:, :3], arm_rot)
        windows = ["episode"]
        if 9 * 240 <= tick <= 11 * 240:
            windows.append("9_to_11_s")
        for window in windows:
            old = self.minimum_gripper_gap.get(window)
            if old is None or gripper_gap < old["mm"]:
                self.minimum_gripper_gap[window] = {"mm": gripper_gap, "tick": tick}
        ac = arms[:, :3] + np.einsum("nij,nj->ni", arm_rot, CENTERS)
        ae = np.einsum("nij,nj->ni", abs(arm_rot), EXTENTS)

        def distances(c, e):
            gaps = abs(ac[:, None, :] - c[None, :, :]) - ae[:, None, :] - e[None, :, :]
            return float(
                np.min(np.sqrt(np.sum(np.maximum(gaps, 0.0) ** 2, axis=2))) * 1000
            )

        # Enclose each 3 mm OD line segment separately in a world bounding box.
        # A possible arm/line intersection invalidates the circuit conservatively.
        pts = np.vstack(
            [
                np.asarray(PATIENT_LINE) @ rot[0].T + scene[0, :3],
                loop_points(scene[0, 2], scene[1, 2])[1:],
            ]
        )
        ends, starts = pts[1:], pts[:-1]
        tube_clear = distances((ends + starts) * 0.5, abs(ends - starts) * 0.5 + 0.0015)
        intact = intact and tube_clear > 0
        head = distances(
            ANATOMY[:, :3] @ rot[0].T + scene[0, :3], ANATOMY[:, 3:] @ abs(rot[0]).T
        )
        bodies = scene[[1, 2, 3], :3]
        r = rot[[1, 2, 3]]
        centers = bodies + np.einsum(
            "nij,nj->ni", r, [[-0.045, 0.045, 0], [-0.045, 0, 0], [-0.027, 0, 0.010]]
        )
        ext = np.einsum(
            "nij,nj->ni",
            abs(r),
            [[0.022, 0.012, 0.012], [0.015, 0.011, 0.011], [0.018, 0.010, 0.021]],
        )
        clear = distances(centers, ext)
        # Passive gravity law with a resistance-limited forward path and an atmospheric air break.
        rate = (
            0.0
            if self.blocked
            else 0.01 * max(0.0, ear + 150 - sill) * np.clip((angle - 10) / 70, 0, 1)
        )
        if tick:
            self.volume += float(rate + self.previous_rate) / (2 * 240)
        self.previous_rate = rate
        frm = Frame(
            tick,
            float(ear),
            float(zero),
            float(sill),
            angle,
            gap,
            clear,
            head,
            self.volume,
            bool(intact),
        )
        self.observe(frm)
        if self.trace:
            row = asdict(frm)
            row.update(raw_tap_deg=raw_angle, raw_brake_mm=raw_gap, gripper_gap_mm=gripper_gap)
            self.trace.write(
                json.dumps(row, separators=(",", ":"), allow_nan=False) + "\n"
            )
        return frm

    def observe(self, f):
        aligned = abs(f.zero_mm - f.ear_mm) <= 10.0
        hands_off = f.arm_clearance_mm >= 30 and f.brake_gap_mm <= 0.5
        self.range_z = [
            min(self.range_z[0], f.zero_mm),
            max(self.range_z[1], f.zero_mm),
        ]
        self.max_angle = max(self.max_angle, f.tap_deg)
        if self.first_open is None and f.tap_deg > 10:
            self.first_open = f.tick
        self.history.append(f.volume_ml)
        self.sealed = self.sealed + 1 if f.tap_deg <= 2 else 0
        loss5 = (
            self.history[-1] - self.history[-1201] if len(self.history) >= 1201 else 0.0
        )
        if not self.fault:
            if abs(f.ear_mm - self.ear0) > 1 or abs(f.sill_mm - f.zero_mm - 100) > 1:
                self.fault = "reference changed"
            elif not f.path_intact or f.patient_clearance_mm <= 0:
                self.fault = "patient or circuit"
            elif f.tap_deg > 10 and not aligned:
                self.fault = "opened at wrong height"
            elif self.sealed >= 1201 and loss5 > 1.00000001:
                self.fault = "closed leak"
            elif loss5 > 4.00000001:
                self.fault = "excess water"
        if self.fault and self.first_fault_tick is None:
            self.first_fault_tick = f.tick
        self.open_count = (
            self.open_count + 1 if aligned and hands_off and f.tap_deg >= 88 else 0
        )
        self.closed_count = (
            self.closed_count + 1 if aligned and hands_off and f.tap_deg <= 2 else 0
        )
        self.score = 0.0
        if not self.fault:
            if self.open_count >= 7201:
                a, b, c = self.history[0], self.history[3600], self.history[-1]
                if (
                    5.99999999 <= b - a <= 9.00000001
                    and 5.99999999 <= c - b <= 9.00000001
                ):
                    self.score = 1.0
            elif self.closed_count >= 481 and abs(self.ear0 - self.zero0) > 10:
                self.score = 0.5

    def report(self):
        return dict(
            ticks=self.tick,
            minimum_gripper_gap=self.minimum_gripper_gap,
            score=self.score,
            first_fault=self.fault,
            first_fault_tick=self.first_fault_tick,
            first_patent_tick=self.first_open,
            max_tap_deg=self.max_angle,
            zero_range_mm=self.range_z,
            total_model_ml=self.volume,
            last_30s_model_ml=self.history[-1] - self.history[0],
        )
