"""Read loaded geometry and integrate each channel's guarded water check."""

import math

import numpy as np
from yam_arena.ik import quat_xyzw_to_matrix

from . import config as C
from .motion import rotation

TUBE_CENTER = C.TABLE + C.BED_Z + C.TUBE_R
SAMPLE_Y = C.HEAD[1] + np.array([-0.046, -0.022, 0.0, 0.022, 0.046])
CONTROL_HZ = 20
GUARD_STEPS = CONTROL_HZ // 2
CHECK_STEPS = 3 * CONTROL_HZ
PUMP_ML_PER_STEP = 1.7 * 24 / 60 / CONTROL_HZ


def tube_geometry(links):
    points = np.array([links[f"Link{i}"][:3] for i in range(C.LINKS)])
    last = np.asarray(links[f"Link{C.LINKS - 1}"])
    outlet = last[:3] + quat_xyzw_to_matrix(last[3:])[:, 1] * (C.TUBE_L / 2)
    y = points[:, 1]
    spans = y[0] < SAMPLE_Y[0] - 0.025 and y[-1] > SAMPLE_Y[-1] + 0.025
    continuous = bool(np.all(np.diff(y) > 0.008))
    x, z = [np.interp(SAMPLE_Y, y, points[:, axis]) for axis in (0, 2)]
    return outlet, x, z, bool(spans and continuous)


def inspect_track(geometry, lane, receiver):
    outlet, x, z, through = geometry
    lateral = 1e3 * np.max(np.abs(x - C.HEAD[0] - lane))
    depth = 1e3 * np.max(np.abs(z - TUBE_CENTER))
    captures = np.linalg.norm(outlet[:2] - receiver) <= 0.009 and outlet[2] > C.TABLE + 0.0315
    return (bool(through and lateral < 4.1 and depth < 5.0), float(lateral),
            float(depth), bool(captures))


class PumpCheck:
    def __init__(self):
        self.volumes = np.zeros(2)
        self.flow_steps = np.zeros(2, dtype=int)
        self.steady = np.zeros(2, dtype=int)
        self.assignment = (0, 1)
        self.active = np.zeros(2, dtype=bool)
        self.outlets = np.zeros((2, 3))

    def observe(self, state):
        joints = state["objects"]["head"]["joints"]
        angle = float(joints["CamClose"])
        closed = abs(angle) < math.radians(4)
        tubes = [tube_geometry(state["objects"][name]["links"]) for name in C.TUBE_NAMES]
        options = [
            [inspect_track(tubes[j], C.LANES[i], C.RECEIVERS[i]) for i, j in enumerate(order)]
            for order in ((0, 1), (1, 0))
        ]
        best = max(range(2), key=lambda j: (sum(row[0] and row[3] for row in options[j]),
                                           sum(row[0] for row in options[j]),
                                           -sum(row[1] for row in options[j])))
        order = (0, 1) if best == 0 else (1, 0)
        aligned = options[best]
        compressed = np.array([float(joints[f"Compression{i}"]) for i in range(2)])
        lever = C.HEAD + C.PIVOT + rotation("Y", angle) @ C.LEVER
        pump = np.array([C.HEAD[0], C.HEAD[1], TUBE_CENTER])
        clear = all(
            np.linalg.norm(np.asarray(point) - pump) > 0.093
            and np.linalg.norm(np.asarray(point) - lever) > 0.055
            for hand in state["hands"].values() for point in hand["points"]
        )
        checkable = np.array([clear and closed and ok and capture and 0.0012 < travel < 0.0065
                              for (ok, _, _, capture), travel in zip(aligned, compressed)])
        if order != self.assignment:
            self.steady[:] = 0
        self.assignment = order
        self.outlets = np.array([tubes[i][0] for i in order])
        self.steady = np.where(checkable, self.steady + 1, 0)
        self.flow_steps = np.clip(self.steady - GUARD_STEPS, 0, CHECK_STEPS)
        self.active = (self.steady > GUARD_STEPS) & (self.steady <= GUARD_STEPS + CHECK_STEPS)
        self.volumes = self.flow_steps * PUMP_ML_PER_STEP
        passed = (self.flow_steps == CHECK_STEPS) & (self.volumes >= 1.0)
        score = float(0.5 * np.count_nonzero(passed))
        return {
            "score": score, "success": score == 1.0, "valve_angle_deg": math.degrees(angle),
            "track_lateral_error_mm": [row[1] for row in aligned],
            "track_height_error_mm": [row[2] for row in aligned],
            "outlet_over_receiver": [row[3] for row in aligned],
            "occlusion_mm": (1e3 * compressed).tolist(), "water_ml": self.volumes.tolist(),
            "hands_clear": clear, "flow_check_steps": self.flow_steps.tolist(),
        }
