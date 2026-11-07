"""Sort six rigid bottle surrogates onto the shelf matching their color.

A narrow red lower shelf and blue upper shelf form a vertical rack. Each holds
three closely spaced bottles, only 6 mm from its open front edge. A roof blocks
overhead grasps: enter from the front with the gripper horizontal. Swap the two
misplaced center bottles via the gray staging pedestal. Preserve all four
neighbors. Finish with all bottles upright, still, wholly on the correct shelf,
and both grippers withdrawn.
Use run.py for the continuous pose safety monitor and release/stability window.
This task evaluates tipping and falls; contact force is telemetry, not a score
condition, and glass fracture is not simulated.
"""

from collections import deque

import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import ArmPlan
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # SafetyMonitor already verifies a two-second pose window.
SCORE_OVER = "end"
RADIUS = 0.018
HEIGHT = 0.10
MASS = 0.08
MAX_TILT_DEG = 20.0
STABILITY_S = 2.0
MAX_DRIFT_M = 0.003
EDGE_MARGIN = 0.003
RED = (0.80, 0.08, 0.07)
BLUE = (0.06, 0.28, 0.85)

# Front is -X. Both occupied shelves and the roof have the same XY footprint.
RACK_FRONT = 0.27
RACK_BACK = 0.40
SHELVES = {
    "red": Box(
        "red_shelf", pos=(0.335, 0, TOP + 0.044), size=(0.13, 0.17, 0.012), color=RED
    ),
    "blue": Box(
        "blue_shelf", pos=(0.335, 0, TOP + 0.204), size=(0.13, 0.17, 0.012), color=BLUE
    ),
}
FIXTURES = [
    Box(
        "rack_roof",
        pos=(0.335, 0, TOP + 0.364),
        size=(0.13, 0.17, 0.012),
        color=(0.56, 0.47, 0.35),
    ),
    Box(
        "rack_back",
        pos=(0.406, 0, TOP + 0.21),
        size=(0.012, 0.194, 0.32),
        color=(0.40, 0.34, 0.27),
    ),
    *[
        Box(
            "rack_side_" + side,
            pos=(0.335, y, TOP + 0.185),
            size=(0.13, 0.012, 0.37),
            color=(0.46, 0.39, 0.30),
        )
        for side, y in [("left", 0.091), ("right", -0.091)]
    ],
]
STAGE = Box(
    "staging_pad",
    pos=(0.25, 0.23, TOP + 0.08),
    size=(0.08, 0.08, 0.16),
    color=(0.52, 0.54, 0.58),
)


def surface(shelf):
    return shelf.pos[2] + shelf.size[2] / 2


def bottle(name, x, y, shelf, color):
    # run.py replaces these validated Cylinder specs with compound bottle shapes.
    return Cylinder(
        name,
        pos=(x, y, surface(shelf) + HEIGHT / 2),
        radius=RADIUS,
        height=HEIGHT,
        mass=MASS,
        color=color,
    )


BOTTLES = [
    bottle("blue_swap", 0.294, 0.0, SHELVES["red"], BLUE),
    bottle("red_left", 0.294, 0.060, SHELVES["red"], RED),
    bottle("red_right", 0.294, -0.060, SHELVES["red"], RED),
    bottle("red_swap", 0.294, 0.0, SHELVES["blue"], RED),
    bottle("blue_left", 0.294, 0.060, SHELVES["blue"], BLUE),
    bottle("blue_right", 0.294, -0.060, SHELVES["blue"], BLUE),
]
OBJECTS = [*SHELVES.values(), *FIXTURES, STAGE, *BOTTLES]
NAMES = tuple(b.name for b in BOTTLES)
GOALS = {name: name.split("_")[0] for name in NAMES}
INITIAL_SORTED = 4
MAX_FALL_SPEED = 0.35
# Local +Z approaches along world +X; local X closes the jaws along world Y.
HORIZONTAL_ROTATION = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
HORIZONTAL_QUAT = ik.matrix_to_quat_xyzw(HORIZONTAL_ROTATION)


def tilt_deg(state):
    q = state.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (q[0] ** 2 + q[1] ** 2), -1, 1))))


def supported_on(state, shelf):
    low = np.asarray(shelf.pos) - np.asarray(shelf.size) / 2
    high = np.asarray(shelf.pos) + np.asarray(shelf.size) / 2
    return bool(
        np.all(state.low[:2] >= low[:2] + EDGE_MARGIN)
        and np.all(state.high[:2] <= high[:2] - EDGE_MARGIN)
        and abs(state.low[2] - high[2]) <= 0.004
    )


def sorted_bottle(name, state):
    return bool(supported_on(state, SHELVES[GOALS[name]]) and tilt_deg(state) <= 5.0)


class SafetyMonitor:
    """Episode-local, irreversible safety latch; updated at 240 Hz by run.py.

    Scoring uses observable pose, motion and release conditions. Contact force
    is retained as diagnostic telemetry only; rigid bottles do not fracture.
    """

    def __init__(self):
        self.reasons = set()
        self.peak_force_n = {name: 0.0 for name in NAMES}
        self.max_tilt_deg = {name: 0.0 for name in NAMES}
        self.released = False
        self.armed = False
        self.stable = False
        self._poses = deque()

    def update(self, objects, forces=None):
        if not self.armed:
            return
        for name in NAMES:
            state = objects[name]
            tilt = tilt_deg(state)
            force = float((forces or {}).get(name, 0.0))
            self.max_tilt_deg[name] = max(self.max_tilt_deg[name], tilt)
            if np.isfinite(force):
                self.peak_force_n[name] = max(self.peak_force_n[name], force)
            if not np.all(
                np.isfinite(
                    np.r_[
                        state.pos,
                        state.quat_xyzw,
                        state.lin_vel,
                        state.ang_vel,
                        state.low,
                        state.high,
                    ]
                )
            ):
                self.reasons.add(f"{name}:invalid_state")
            if tilt > MAX_TILT_DEG:
                self.reasons.add(f"{name}:tipped")
            if state.lin_vel[2] < -MAX_FALL_SPEED:
                self.reasons.add(f"{name}:uncontrolled_fall")
            if state.low[2] < min(surface(s) for s in SHELVES.values()) - 0.006:
                self.reasons.add(f"{name}:fell_below_shelves")
            if not (
                0.07 <= state.pos[0] <= 0.44
                and -0.15 <= state.pos[1] <= 0.27
                and state.high[2] <= TOP + 0.43
            ):
                self.reasons.add(f"{name}:outside_workspace")

    def observe_control(self, objects, time_s, released):
        """Observe a two-second window of bottle centers at the control rate.

        A ruler and calibrated video can reproduce this test. Rotation about the
        vertical bottle axis is deliberately irrelevant for these unlabeled bottles.
        """
        self.released = bool(released)
        if (
            not released
            or not self.safe
            or any(tilt_deg(objects[n]) > 5 for n in NAMES)
        ):
            self._poses.clear()
            self.stable = False
            return
        now = float(time_s)
        positions = np.array([objects[n].pos.copy() for n in NAMES])
        if self._poses and now <= self._poses[-1][0]:
            self._poses.clear()
        self._poses.append((now, positions))
        while len(self._poses) > 1 and self._poses[1][0] <= now - STABILITY_S + 1e-9:
            self._poses.popleft()
        self.stable = bool(
            now - self._poses[0][0] >= STABILITY_S - 1e-9
            and all(
                np.all(
                    np.linalg.norm(p - self._poses[0][1], axis=1) <= MAX_DRIFT_M + 1e-9
                )
                for _, p in self._poses
            )
        )

    @property
    def safe(self):
        return self.armed and not self.reasons


# The stock loader deliberately cannot award credit without instrumented run.py.
MONITOR = SafetyMonitor()


def success(objects):
    return bool(
        MONITOR.safe
        and MONITOR.released
        and MONITOR.stable
        and all(sorted_bottle(n, objects[n]) for n in NAMES)
    )


def score(objects):
    if not MONITOR.safe or not MONITOR.released or not MONITOR.stable:
        return 0.0
    return max(0, sum(sorted_bottle(n, objects[n]) for n in NAMES) - INITIAL_SORTED) / 2


def distance(objects):
    return float(
        sum(abs(objects[n].low[2] - surface(SHELVES[GOALS[n]])) for n in NAMES)
        / len(NAMES)
    )


class RackWaypoint:
    """A Cartesian target with an explicit quaternion, including horizontal grips."""

    def __init__(self, duration, pos, finger, label="", quat=HORIZONTAL_QUAT):
        self.duration = duration
        self.pos = tuple(pos)
        self.finger = finger
        self.label = label
        self.quat = np.asarray(quat)


class RackArmPlan(ArmPlan):
    """Use the stock IK/action layout with full 3D gripper orientation targets."""

    def target(self, t):
        from yam_arena.urdf import quat_xyzw_to_matrix

        prev_pos, prev_rot, prev_finger = (
            self.start_pos,
            self.start_rot,
            self.finger_start,
        )
        for w in self.waypoints:
            goal = np.asarray(w.pos)
            rot = quat_xyzw_to_matrix(w.quat)
            if t < w.duration:
                s = ik.smoothstep(t / w.duration)
                return (
                    prev_pos + s * (goal - prev_pos),
                    ik.slerp_matrix(prev_rot, rot, s),
                    prev_finger + s * (w.finger - prev_finger),
                    w.label,
                )
            t -= w.duration
            prev_pos, prev_rot, prev_finger = goal, rot, w.finger
        return prev_pos, prev_rot, prev_finger, "done"


def demo():
    """Enter horizontally, lift 20 mm, withdraw fully, then change shelf level."""
    blue = BOTTLES[0]
    red = BOTTLES[3]
    stage = (STAGE.pos[0], STAGE.pos[1], surface(STAGE) + HEIGHT / 2)
    red_goal = (blue.pos[0], 0, surface(SHELVES["red"]) + HEIGHT / 2)
    blue_goal = (red.pos[0], 0, surface(SHELVES["blue"]) + HEIGHT / 2)
    parks = {"left": (0.23, 0.23, TOP + 0.20), "right": (0.23, -0.23, TOP + 0.20)}
    opening = FINGER_OPEN * 0.55
    plans = {
        side: [RackWaypoint(2, pos, opening, "park horizontally")]
        for side, pos in parks.items()
    }

    def move(pick, place):
        px, py, pz = pick
        qx, qy, qz = place
        # At x=0.20 the whole carried bottle is outside the shelf front.
        # The staging pad is approached from x=0.21.
        ex = 0.20 if px > RACK_FRONT else 0.21
        fx = 0.20 if qx > RACK_FRONT else 0.21
        return [
            RackWaypoint(2, (ex, py, pz), opening, "align outside shelf"),
            RackWaypoint(2, pick, opening, "enter horizontally between neighbors"),
            RackWaypoint(1, pick, FINGER_CLOSED, "grasp bottle"),
            RackWaypoint(1, (px, py, pz + 0.020), FINGER_CLOSED, "lift 20 mm"),
            RackWaypoint(
                2, (ex, py, pz + 0.020), FINGER_CLOSED, "withdraw horizontally"
            ),
            RackWaypoint(
                3, (fx, qy, qz + 0.020), FINGER_CLOSED, "transfer outside rack"
            ),
            RackWaypoint(2, (qx, qy, qz + 0.020), FINGER_CLOSED, "insert horizontally"),
            RackWaypoint(1, (qx, qy, qz + 0.001), FINGER_CLOSED, "set down"),
            RackWaypoint(1, (qx, qy, qz + 0.001), opening, "release carefully"),
            RackWaypoint(2, (fx, qy, qz + 0.001), opening, "withdraw empty gripper"),
        ]

    for side, pick, place in [
        ("left", blue.pos, stage),
        ("right", red.pos, red_goal),
        ("left", stage, blue_goal),
    ]:
        motion = move(pick, place)
        motion.append(RackWaypoint(2, parks[side], opening, "park clear"))
        other = "right" if side == "left" else "left"
        plans[side].extend(motion)
        plans[other].append(
            RackWaypoint(
                sum(w.duration for w in motion), parks[other], opening, "wait clear"
            )
        )
    for side, plan in plans.items():
        plan.append(
            RackWaypoint(2, parks[side], FINGER_OPEN, "open grippers and verify")
        )
        plan.append(RackWaypoint(3, parks[side], FINGER_OPEN, "verify stability"))
    return plans
