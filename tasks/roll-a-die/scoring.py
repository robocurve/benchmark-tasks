"""Temporal scoring for a physical roll, independent of Isaac Sim.

Inputs are measured object poses and measured gripper states, never action targets.
The runner writes these inputs to JSONL so a score can be reproduced offline.

Version 2 requires a 90-degree tilt excursion from a verified free-flight pose.
Cumulative angular travel remains diagnostic only: repeated small rocking cannot
substitute for a turnover, and a complete revolution need not finish tilted.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

SCORE_VERSION = "roll-a-die-v2"
TABLE_TOP = 0.7475
EDGE = 0.016
LIFT_CLEARANCE = 0.03
LIFT_HOLD_S = 0.20
MIN_TUMBLE = math.pi / 2
REST_HOLD_S = 1.0
TABLE_X = (-0.17, 0.89)
TABLE_Y = (-0.63, 0.63)
FINGER_TRAVEL = 0.04695  # metres per finger at normalized opening 1 (YAM linear gripper)
JAW_CLEARANCE = 0.002    # required empty space between each jaw and the die


def tilt_separation(first_up, second_up):
    """Separation of world-vertical directions in the die's own coordinates."""
    return math.acos(float(np.clip(np.dot(first_up, second_up), -1.0, 1.0)))


def rotation(q):
    """Rotation matrix for an XYZW quaternion, invariant to its sign."""
    q = np.asarray(q, dtype=float)
    norm = float(np.linalg.norm(q))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid die quaternion")
    x, y, z, w = q / norm
    return np.array([
        [1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y)],
    ])


def clear_of_hand(hand, position, die_rotation, distance):
    """Whether measured jaw geometry excludes a pinch, or the hand is distant.

    ``closing_axis`` is the unit world vector along the gripper's local X axis.
    Project both the die's half-width and its displacement from the TCP onto it;
    an opening alone is insufficient if the die is off-centre between the jaws.
    Older audit samples without this field retain the conservative 55% rule.
    """
    if distance >= 0.07:
        return True
    if "closing_axis" not in hand:
        return float(hand["opening"]) >= 0.55
    axis = np.asarray(hand["closing_axis"], dtype=float)
    if axis.shape != (3,) or not np.isfinite(axis).all():
        raise ValueError("closing_axis must contain three finite numbers")
    norm = float(np.linalg.norm(axis))
    if norm < 1e-9:
        raise ValueError("closing_axis must be nonzero")
    axis = axis / norm
    offset = abs(float(np.dot(position - np.asarray(hand["tcp"], dtype=float), axis)))
    half_width = EDGE / 2 * float(np.abs(axis @ die_rotation).sum())
    half_gap = float(hand["opening"]) * FINGER_TRAVEL
    return half_gap - offset - half_width >= JAW_CLEARANCE


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
    previously_resting: bool = False
    last_time: float | None = None
    last_up: np.ndarray | None = None
    last_velocity: np.ndarray | None = None
    last_clear_airborne: bool = False
    pending_tumble: float = 0.0
    pending_start: float | None = None
    pending_up: np.ndarray | None = None
    pending_max_tilt_excursion: float = 0.0
    value: float = 0.0
    events: list = field(default_factory=list)

    def update(self, sample: dict) -> float:
        t = float(sample["time"])
        if not math.isfinite(t) or t < 0:
            raise ValueError("sample time must be finite and nonnegative")
        if self.last_time is not None and t <= self.last_time:
            raise ValueError("sample times must increase; reset the tracker for each episode")
        dt = 0.0 if self.last_time is None else t - self.last_time
        if dt > 0.075:
            raise ValueError("scoring expects 20 Hz observations with no gap above 75 ms")
        previous_time = self.last_time
        self.last_time = t
        p = np.asarray(sample["pos"], dtype=float)
        lo = np.asarray(sample["low"], dtype=float)
        hi = np.asarray(sample["high"], dtype=float)
        velocity = np.asarray(sample["lin_vel"], dtype=float)
        spin = np.asarray(sample["ang_vel"], dtype=float)
        r = rotation(sample["quat_xyzw"])
        up = r[2, :]  # world vertical expressed in the die frame; yaw does not change it
        hands = sample["hands"]
        distances = [float(np.linalg.norm(p - np.asarray(h["tcp"]))) for h in hands]
        openings = [float(h["opening"]) for h in hands]
        if len(hands) != 2 or not np.isfinite(np.r_[p, lo, hi, velocity, spin, distances, openings]).all():
            raise ValueError("two finite measured hand states and a finite die state are required")

        if lo[2] < TABLE_TOP - 0.015 or lo[0] < TABLE_X[0] or hi[0] > TABLE_X[1] or lo[1] < TABLE_Y[0] or hi[1] > TABLE_Y[1]:
            self.invalid = "die left the permitted tabletop area"

        near_closed_hand = any(d < 0.035 and opening < 0.30 for d, opening in zip(distances, openings))
        if not self.lifted:
            held = lo[2] >= TABLE_TOP + LIFT_CLEARANCE and np.linalg.norm(velocity) <= 0.15 and near_closed_hand
            self.lift_hold = self.lift_hold + dt if held and self.previously_held else 0.0
            self.previously_held = bool(held)
            if self.lift_hold + 1e-9 >= LIFT_HOLD_S:
                self.lifted = True
                self.events.append({"time": t, "event": "pickup"})

        hands_released = all(clear_of_hand(hand, p, r, d) for hand, d in zip(hands, distances))
        clear_airborne = hands_released and lo[2] >= TABLE_TOP + 0.012
        ballistic_interval = False
        if self.last_velocity is not None and dt > 0:
            acceleration = (velocity - self.last_velocity) / dt
            ballistic_interval = abs(acceleration[2] + 9.81) <= 2.5 and np.linalg.norm(acceleration[:2]) <= 3.0
        angle = 0.0
        if self.last_up is not None:
            angle = tilt_separation(up, self.last_up)
            if angle < math.radians(0.5):
                angle = 0.0
        newly_released = False
        if self.lifted and not self.released:
            # Preserve only intervals with clear jaws and an airborne die at
            # BOTH endpoints, plus measured gravitational acceleration. This
            # includes upward free flight without crediting a held-to-free
            # orientation jump. Any unverified interval breaks this history.
            if clear_airborne and self.last_clear_airborne and ballistic_interval:
                if self.pending_start is None:
                    self.pending_start = previous_time
                    self.pending_up = self.last_up.copy()
                self.pending_tumble += angle
                self.pending_max_tilt_excursion = max(
                    self.pending_max_tilt_excursion,
                    tilt_separation(up, self.pending_up),
                )
            else:
                self.pending_tumble = 0.0
                self.pending_start = None
                self.pending_up = None
                self.pending_max_tilt_excursion = 0.0
            if clear_airborne and min(distances) >= 0.022 and ballistic_interval:
                self.released = True
                newly_released = True
                self.tumble += self.pending_tumble
                self.release_up = (up if self.pending_up is None else self.pending_up).copy()
                self.max_tilt_excursion = self.pending_max_tilt_excursion
                self.events.append({
                    "time": t, "event": "release",
                    "credited_from": self.pending_start if self.pending_start is not None else t,
                })
                self.pending_tumble = 0.0
                self.pending_start = None
                self.pending_up = None
                self.pending_max_tilt_excursion = 0.0

        if self.released:
            if not hands_released or min(distances) < 0.020:
                self.invalid = "gripper re-entered the die clearance after release"
            if self.last_up is not None and not newly_released:
                self.tumble += angle
            self.max_tilt_excursion = max(
                self.max_tilt_excursion, tilt_separation(up, self.release_up),
            )

        rolled = self.released and self.max_tilt_excursion + 1e-9 >= MIN_TUMBLE
        resting = (
            rolled
            and abs(lo[2] - TABLE_TOP) <= 0.003
            and np.linalg.norm(velocity) < 0.01
            and np.linalg.norm(spin) < 0.25
            and max(abs(up)) >= math.cos(math.radians(10))
            and min(distances) >= 0.06
        )
        self.rest_hold = self.rest_hold + dt if resting and self.previously_resting else 0.0
        self.previously_resting = bool(resting)
        self.value = 1.0 if self.rest_hold + 1e-9 >= REST_HOLD_S else 0.5 if rolled else 0.25 if self.lifted else 0.0
        if self.invalid:
            self.value = 0.0
        self.last_up, self.last_velocity = up.copy(), velocity.copy()
        self.last_clear_airborne = bool(clear_airborne)
        return self.value

    def summary(self):
        return {
            "score_version": SCORE_VERSION,
            "score": self.value, "lifted": self.lifted, "released": self.released,
            "max_tilt_excursion_degrees": math.degrees(self.max_tilt_excursion),
            "cumulative_tumble_degrees": math.degrees(self.tumble),
            # Keep the original telemetry name for older analysis consumers.
            # It no longer determines whether the roll earns credit.
            "tumble_degrees": math.degrees(self.tumble), "rest_seconds": self.rest_hold,
            "invalid": self.invalid, "events": self.events,
        }
