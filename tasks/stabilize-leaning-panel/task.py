"""Brace the leaning wooden panel with the orange stopper, then withdraw both hands.

The panel leans against a wall on a smooth base. Its foot can slide toward the
robot. Hold the panel while seating the stopper in the three-sided socket at its
foot. The stopper must carry the load through physical contact, not a virtual
lock. After release, a standard 0.6 N outward pull at the panel foot tests the
restraint for 0.4 s; the panel must then remain stable for 2 s. Use run.py.
"""

from collections import deque

import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import ArmPlan, grip_rotation
from yam_arena.task_file import TABLE_TOP, Box
from yam_arena.urdf import quat_xyzw_to_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]
ANGLE = np.radians(20.0)
LENGTH, WIDTH, THICKNESS = 0.22, 0.340, 0.012
BASE_X = 0.270
BASE_Z = TOP + 0.004
PANEL_QUAT = (0.0, float(np.sin(ANGLE / 2)), 0.0, float(np.cos(ANGLE / 2)))
PANEL_CENTER = (
    BASE_X + LENGTH / 2 * np.sin(ANGLE),
    0.0,
    BASE_Z + LENGTH / 2 * np.cos(ANGLE) + THICKNESS / 2 * np.sin(ANGLE),
)
WALL_FRONT = (
    PANEL_CENTER[0] + LENGTH / 2 * np.sin(ANGLE) + THICKNESS / 2 * np.cos(ANGLE)
)
PANEL = Box(
    "panel",
    pos=PANEL_CENTER,
    size=(THICKNESS, WIDTH, LENGTH),
    mass=0.15,
    color=(0.76, 0.56, 0.31),
)
STOPPER = Box(
    "stopper",
    pos=(0.16, -0.18, BASE_Z + 0.025),
    size=(0.040, 0.044, 0.050),
    mass=0.10,
    color=(0.95, 0.36, 0.05),
)
STOP_GOAL = np.array((0.233, 0.0, BASE_Z + 0.025))
FIXTURES = [
    Box(
        "base_plate",
        pos=(0.28, 0, TOP + 0.002),
        size=(0.36, 0.50, 0.004),
        color=(0.65, 0.68, 0.70),
    ),
    Box(
        "back_wall",
        pos=(WALL_FRONT + 0.01, 0, BASE_Z + 0.14),
        size=(0.02, 0.40, 0.28),
        color=(0.48, 0.52, 0.56),
    ),
    Box(
        "socket_front",
        pos=(0.208, 0, BASE_Z + 0.006),
        size=(0.008, 0.064, 0.012),
        color=(0.16, 0.38, 0.22),
    ),
    *[
        Box(
            "socket_" + name,
            pos=(0.226, y, BASE_Z + 0.006),
            size=(0.028, 0.006, 0.012),
            color=(0.16, 0.38, 0.22),
        )
        for name, y in [("left", 0.027), ("right", -0.027)]
    ],
]
FIXTURES += [
    Box(
        "foot_guide_" + name,
        pos=(0.31, y, BASE_Z + 0.006),
        size=(0.12, 0.008, 0.012),
        color=(0.40, 0.44, 0.48),
    )
    for name, y in [("left", 0.176), ("right", -0.176)]
]
OBJECTS = [*FIXTURES, PANEL, STOPPER]
NAMES = ("panel", "stopper")
SUCCESS_HOLD_S = 0.0
SCORE_OVER = "end"
PULL_N, PULL_S, SETTLE_S = 0.6, 0.4, 2.0


def rotation(state):
    return quat_xyzw_to_matrix(state.quat_xyzw)


def panel_foot(state):
    return state.pos + rotation(state) @ np.array((0, 0, -LENGTH / 2))


def tilt(state):
    return float(np.degrees(np.arccos(np.clip(rotation(state)[2, 2], -1, 1))))


def seated(state, *, require_rest=True):
    r = rotation(state)
    return bool(
        np.all(np.abs(state.pos[:2] - STOP_GOAL[:2]) <= 0.004)
        and abs(state.low[2] - BASE_Z) <= 0.003
        and tilt(state) <= 5
        and abs(np.degrees(np.arctan2(r[1, 0], r[0, 0]))) <= 5
        and (not require_rest or state.resting(max_speed=0.015, max_spin=0.15))
    )


class Monitor:
    """Per-episode safety latch and release/pull/settle state machine.

    run.py samples safety at every physics step and supplies measured gripper
    clearance at 20 Hz. Score hooks only read state; reset is explicit.
    """

    def __init__(self):
        self.armed = False
        self.reasons = set()
        self.clear = False
        self.time = 0.0
        self.proof_start = None
        self.complete = False
        self.history = deque()
        self.max_tilt = 0.0
        self.max_foot_drift = 0.0

    @property
    def pulling(self):
        return (
            self.proof_start is not None and 0 <= self.time - self.proof_start < PULL_S
        )

    def physics(self, objects, dt):
        if not self.armed:
            return
        self.time += dt
        for name in NAMES:
            s = objects[name]
            if not np.all(
                np.isfinite(
                    np.r_[s.pos, s.quat_xyzw, s.lin_vel, s.ang_vel, s.low, s.high]
                )
            ):
                self.reasons.add("nonfinite_state")
                return
            if s.low[2] < TOP - 0.005 or not (
                0.05 < s.pos[0] < 0.50 and abs(s.pos[1]) < 0.40
            ):
                self.reasons.add(name + "_outside_workspace")
        p = objects["panel"]
        self.max_tilt = max(self.max_tilt, tilt(p))
        foot = panel_foot(p)
        drift = float(np.linalg.norm(foot[:2] - (BASE_X, 0)))
        self.max_foot_drift = max(self.max_foot_drift, drift)
        if not 10 <= tilt(p) <= 32:
            self.reasons.add("panel_unsafe_tilt")
        if (
            drift > 0.018
            or abs(foot[2] - (BASE_Z + THICKNESS / 2 * np.sin(ANGLE))) > 0.010
        ):
            self.reasons.add("panel_foot_displaced")
        if self.proof_start is not None:
            if not self.clear:
                self.reasons.add("hands_returned_during_proof")
            if not seated(objects["stopper"], require_rest=False):
                self.reasons.add("stopper_lost_during_proof")
            if self.time - self.proof_start >= PULL_S:
                self.history.append(
                    (self.time, p.pos.copy(), objects["stopper"].pos.copy())
                )
                while self.history and self.history[0][0] < self.time - SETTLE_S:
                    self.history.popleft()
                if self.history and self.time - self.history[0][0] >= SETTLE_S - 2 * dt:
                    positions = np.array([np.r_[a, b] for _, a, b in self.history])
                    self.complete = bool(
                        np.max(np.ptp(positions, axis=0)) <= 0.003
                        and p.resting(max_speed=0.01, max_spin=0.1)
                        and objects["stopper"].resting(max_speed=0.01, max_spin=0.1)
                        and not self.reasons
                    )

    def control(self, objects, clear):
        self.clear = bool(clear)
        if (
            self.proof_start is None
            and clear
            and seated(objects["stopper"])
            and not self.reasons
        ):
            self.proof_start = self.time


MONITOR = Monitor()


def score(objects):
    if not MONITOR.armed or MONITOR.reasons:
        return 0.0
    if MONITOR.complete:
        return 1.0
    return 0.5 if seated(objects["stopper"]) else 0.0


def success(objects):
    return score(objects) == 1.0


HORIZONTAL = ik.matrix_to_quat_xyzw(
    np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
)


class Waypoint:
    def __init__(self, duration, pos, finger, label="", quat=HORIZONTAL):
        self.duration, self.pos, self.finger, self.label = (
            duration,
            tuple(pos),
            finger,
            label,
        )
        self.quat = np.asarray(quat)


class PanelArmPlan(ArmPlan):
    def target(self, t):
        p, r, f = self.start_pos, self.start_rot, self.finger_start
        for w in self.waypoints:
            goal, rot = np.asarray(w.pos), quat_xyzw_to_matrix(w.quat)
            if t < w.duration:
                s = ik.smoothstep(t / w.duration)
                return (
                    p + s * (goal - p),
                    ik.slerp_matrix(r, rot, s),
                    f + s * (w.finger - f),
                    w.label,
                )
            t -= w.duration
            p, r, f = goal, rot, w.finger
        return p, r, f, "done"


def demo():
    z = BASE_Z + 0.140
    front = BASE_X + 0.14 * np.tan(ANGLE) - THICKNESS / (2 * np.cos(ANGLE))
    support = (front - 0.012, 0.150, z)
    approach = (support[0] - 0.030, support[1], z + 0.020)
    park_l = (0.30, 0.30, TOP + 0.20)
    park_r = (0.18, -0.24, TOP + 0.16)
    left = [
        Waypoint(2, approach, FINGER_CLOSED, "approach panel side"),
        Waypoint(2, support, FINGER_CLOSED, "support panel against wall"),
        Waypoint(16, support, FINGER_CLOSED, "support while seating stopper"),
        Waypoint(5, park_l, FINGER_OPEN, "withdraw and park clear"),
        Waypoint(5, park_l, FINGER_OPEN, "wait for proof"),
    ]
    down = ik.matrix_to_quat_xyzw(grip_rotation(np.pi / 2))
    pick = (STOPPER.pos[0], STOPPER.pos[1], STOPPER.pos[2] + 0.004)
    goal = tuple(STOP_GOAL + (0, 0, 0.006))
    hover_pick = (*pick[:2], TOP + 0.12)
    hover_goal = (*goal[:2], TOP + 0.12)
    right = [
        Waypoint(5, park_r, FINGER_OPEN, "wait for support", down),
        Waypoint(2, hover_pick, FINGER_OPEN, "above stopper", down),
        Waypoint(2, pick, FINGER_OPEN, "lower", down),
        Waypoint(1, pick, FINGER_CLOSED, "grasp stopper", down),
        Waypoint(2, hover_pick, FINGER_CLOSED, "lift", down),
        Waypoint(2, hover_goal, FINGER_CLOSED, "over socket", down),
        Waypoint(2, goal, FINGER_CLOSED, "seat stopper", down),
        Waypoint(1, goal, FINGER_OPEN * 0.70, "release stopper", down),
        Waypoint(2, hover_goal, FINGER_OPEN * 0.70, "retreat", down),
        Waypoint(2, park_r, FINGER_OPEN, "park clear", down),
        Waypoint(9, park_r, FINGER_OPEN, "wait for proof", down),
    ]
    return {"left": left, "right": right}
