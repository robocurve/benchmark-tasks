"""Version 3: score visible pickup, release and a marked-face turnover.

The physical rubric uses a ruler and frame-by-frame video, not acceleration or
contact-force estimation:

* 0.25: hold the cube's bottom at least 30 mm above the tabletop for 0.20 s.
* 0.50: after that pickup, observe its bottom at least 10 mm above the table
  with both gripper tool centres at least 50 mm from the cube centre.
* 1.00: after release, observe any marked face within 30 degrees of upward,
  then that same face within 30 degrees of downward. Subsequently, the cube
  must rest on a face for 1.00 s: bottom within 3 mm of the tabletop, a face
  within 10 degrees of horizontal, both tool centres at least 60 mm away,
  and centre/orientation within 2 mm/5 degrees of one fixed resting pose.

A tool centre is the point midway between its finger pads. For the measured
simulation audit, "held" means a tool centre within 35 mm and gripper opening
below 0.30. No velocity or acceleration threshold changes the score. Once
released, a tool centre returning within 50 mm, or the cube leaving the
permitted tabletop area, permanently erases all credit. The final score counts.

Face identity is physical, so world-yaw and small repeated rocking cannot
substitute for an upward-to-downward inversion. Only observations at or after
the first visibly separated release can start a turnover; held rotation and
the held-to-free boundary interval never count. Inputs are measured poses and
hand states, not action targets. The runner retains them for offline replay.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

SCORE_VERSION = "roll-a-die-v3"
TABLE_TOP = 0.7475
EDGE = 0.016
LIFT_CLEARANCE = 0.03
LIFT_HOLD_S = 0.20
RELEASE_CLEARANCE = 0.01
RELEASE_HAND_DISTANCE = 0.05
FACE_DIRECTION_COS = math.cos(math.radians(30))
REST_HOLD_S = 1.0
REST_TRANSLATION = 0.002
REST_ROTATION = math.radians(5)
TABLE_X = (-0.17, 0.89)
TABLE_Y = (-0.63, 0.63)
EPSILON = 1e-9

# The scene paints these rigidly attached faces. Opposite pips sum to seven.
FACES = (
    ("+X", 0, 1, 2, "red"), ("-X", 0, -1, 5, "cyan"),
    ("+Y", 1, 1, 3, "green"), ("-Y", 1, -1, 4, "purple"),
    ("+Z", 2, 1, 1, "white"), ("-Z", 2, -1, 6, "yellow"),
)


def tilt_separation(first_up, second_up):
    """Separation of world-vertical directions in the die's own coordinates."""
    return math.acos(float(np.clip(np.dot(first_up, second_up), -1.0, 1.0)))


def rotation(q):
    """Rotation matrix for an XYZW quaternion, invariant to its sign."""
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all():
        raise ValueError("die quaternion must contain four finite numbers")
    norm = float(np.linalg.norm(q))
    if norm < 1e-9:
        raise ValueError("invalid die quaternion")
    x, y, z, w = q / norm
    return np.array([
        [1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y)],
    ])


def orientation_separation(first, second):
    """Full orientation distance, including yaw, in radians."""
    return math.acos(float(np.clip((np.trace(first.T @ second) - 1) / 2, -1, 1)))


def vector(value, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain three finite numbers")
    return result


@dataclass
class RollTracker:
    """One tracker per episode. Durations use simulator time, not wall time."""

    lifted: bool = False
    released: bool = False
    tumble: float = 0.0
    max_tilt_excursion: float = 0.0
    release_up: np.ndarray | None = None
    invalid: str | None = None
    lift_hold: float = 0.0
    rest_hold: float = 0.0
    previously_held: bool = False
    last_time: float | None = None
    last_up: np.ndarray | None = None
    face_up_times: dict = field(default_factory=dict)
    face_down_times: dict = field(default_factory=dict)
    rest_start: float | None = None
    rest_position: np.ndarray | None = None
    rest_rotation: np.ndarray | None = None
    value: float = 0.0
    events: list = field(default_factory=list)

    def update(self, sample: dict) -> float:
        t = float(sample["time"])
        if not math.isfinite(t) or t < 0:
            raise ValueError("sample time must be finite and nonnegative")
        if self.last_time is not None and t <= self.last_time:
            raise ValueError("sample times must increase; reset the tracker for each episode")
        dt = 0.0 if self.last_time is None else t - self.last_time
        if dt > 0.075 + EPSILON:
            raise ValueError("scoring expects 20 Hz observations with no gap above 75 ms")
        p = vector(sample["pos"], "die position")
        lo = vector(sample["low"], "die lower bounds")
        hi = vector(sample["high"], "die upper bounds")
        # Retained in the audit but deliberately not used to award credit.
        vector(sample["lin_vel"], "die linear velocity")
        vector(sample["ang_vel"], "die angular velocity")
        r = rotation(sample["quat_xyzw"])
        up = r[2, :]  # world vertical expressed in the die frame
        hands = sample["hands"]
        if len(hands) != 2:
            raise ValueError("two finite measured hand states are required")
        distances = [float(np.linalg.norm(p - vector(h["tcp"], "tool centre"))) for h in hands]
        openings = [float(h["opening"]) for h in hands]
        if not np.isfinite(openings).all():
            raise ValueError("two finite measured hand states are required")
        self.last_time = t

        if (lo[2] < TABLE_TOP - 0.015 - EPSILON
                or lo[0] < TABLE_X[0] - EPSILON or hi[0] > TABLE_X[1] + EPSILON
                or lo[1] < TABLE_Y[0] - EPSILON or hi[1] > TABLE_Y[1] + EPSILON):
            self.invalid = self.invalid or "die left the permitted tabletop area"

        if not self.lifted:
            near_closed_hand = any(d < 0.035 and opening < 0.30 for d, opening in zip(distances, openings))
            held = lo[2] + EPSILON >= TABLE_TOP + LIFT_CLEARANCE and near_closed_hand
            self.lift_hold = self.lift_hold + dt if held and self.previously_held else 0.0
            self.previously_held = bool(held)
            if self.lift_hold + EPSILON >= LIFT_HOLD_S:
                self.lifted = True
                self.events.append({"time": t, "event": "pickup"})

        newly_released = False
        if (self.lifted and not self.released
                and lo[2] + EPSILON >= TABLE_TOP + RELEASE_CLEARANCE
                and min(distances) + EPSILON >= RELEASE_HAND_DISTANCE):
            self.released = True
            newly_released = True
            self.release_up = up.copy()
            self.events.append({"time": t, "event": "release", "credited_from": t})

        if self.released:
            if min(distances) + EPSILON < RELEASE_HAND_DISTANCE:
                self.invalid = self.invalid or "gripper tool centre returned within 50 mm after release"
            if self.last_up is not None and not newly_released:
                angle = tilt_separation(up, self.last_up)
                if angle >= math.radians(0.5):
                    self.tumble += angle
            self.max_tilt_excursion = max(self.max_tilt_excursion, tilt_separation(up, self.release_up))
            for face, axis, sign, pips, color in FACES:
                normal_z = float(sign * up[axis])
                if normal_z + EPSILON >= FACE_DIRECTION_COS and face not in self.face_up_times:
                    self.face_up_times[face] = t
                    self.events.append({"time": t, "event": "face_up", "face": face,
                                        "pips": pips, "color": color, "direction": "up",
                                        "normal_z": normal_z})
                if (normal_z - EPSILON <= -FACE_DIRECTION_COS
                        and face in self.face_up_times and t > self.face_up_times[face]
                        and face not in self.face_down_times):
                    self.face_down_times[face] = t
                    self.events.append({"time": t, "event": "face_down", "face": face,
                                        "pips": pips, "color": color, "direction": "down",
                                        "normal_z": normal_z, "up_time": self.face_up_times[face]})

        resting = (
            bool(self.face_down_times)
            and abs(lo[2] - TABLE_TOP) <= 0.003 + EPSILON
            and max(abs(up)) + EPSILON >= math.cos(math.radians(10))
            and min(distances) + EPSILON >= 0.06
        )
        if not resting:
            self.rest_start = None
            self.rest_position = self.rest_rotation = None
            self.rest_hold = 0.0
        elif (self.rest_start is None
                or np.linalg.norm(p - self.rest_position) > REST_TRANSLATION + EPSILON
                or orientation_separation(self.rest_rotation, r) > REST_ROTATION + EPSILON):
            # Anchor one fixed pose. Comparing only consecutive frames would
            # incorrectly accept arbitrarily long slow sliding or yawing.
            self.rest_start = t
            self.rest_position, self.rest_rotation = p.copy(), r.copy()
            self.rest_hold = 0.0
        else:
            self.rest_hold = t - self.rest_start

        self.value = (1.0 if self.rest_hold + EPSILON >= REST_HOLD_S
                      else 0.5 if self.released else 0.25 if self.lifted else 0.0)
        if self.invalid:
            self.value = 0.0
        self.last_up = up.copy()
        return self.value

    def summary(self):
        return {
            "score_version": SCORE_VERSION,
            "score": self.value, "lifted": self.lifted, "released": self.released,
            "turned_over": bool(self.face_down_times),
            "turnover_faces": list(self.face_down_times),
            "face_up_times": dict(self.face_up_times), "face_down_times": dict(self.face_down_times),
            "max_tilt_excursion_degrees": math.degrees(self.max_tilt_excursion),
            "cumulative_tumble_degrees": math.degrees(self.tumble),
            # Historical consumers retain this diagnostic; it never awards credit.
            "tumble_degrees": math.degrees(self.tumble), "rest_seconds": self.rest_hold,
            "invalid": self.invalid, "events": self.events,
        }
