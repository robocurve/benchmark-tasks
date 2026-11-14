"""The scorer, as plain functions of the clipper's states and the arms' state. NumPy only.

``Tracker.update(states, arms)`` -> Result(score, success, details), fed one observation per control step. It reads the
clipper's mechkit AssemblyState (``clipper``, one articulation: the base (lower blade, tail block and upper blade),
the pin and the lever) and both arms through rigstate. Nothing is latched: the score is
that of the last step. Every direction is taken in the clipper's own frame (from the base link's pose), so turning the
whole clipper over proves nothing.

    u           the lever's hinge-to-tip direction; u_plan its part in the lower blade's plane (x_C, y_C)
    TO_TAIL     |u_plan| of at least 20 mm per 57 and u_plan within 10 deg of +x_C
    PAST_JAWS   the same length, u_plan within 30 deg of -x_C
    HEEL_AWAY   the heel-side normal n . z_C > 0: the heel side faces away from the lower blade
    FLAT        the gap from the lever tip's blade-side face (centre of the tip edge) to the upper blade's top plane
                is at most 2.0 mm
    IN_SLOT     the four bottom corners of the lower blade each within 1.5 mm of the slot floor
    CLEAR       rigstate.released with the clipper's own distance (every shape of both assemblies): each gripper's
                finger zone at least 30 mm away; unreadable arms, or a distance that cannot be computed, are NOT clear
    STILL       over the last 20 steps the lever tip and the two end centres of the lower blade moved at most 1.0 mm
                from where they were at the window's start

    1.0    TO_TAIL, HEEL_AWAY, FLAT, IN_SLOT and CLEAR on each of the last 20 steps, and STILL   (success)
    0.5    TO_TAIL and HEEL_AWAY
    0.25   PAST_JAWS
    0      anything else, and any step whose clipper state is missing or not finite (the windows start again)
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

import layout as L
import rigstate

CLIPPER = "clipper"
PIN, LEVER = "pin", "lever"

U0 = np.array([math.cos(L.E0), 0.0, math.sin(L.E0)])  # the lever's direction in the base frame, joints at 0
N0 = np.array([math.sin(L.E0), 0.0, -math.cos(L.E0)])  # its heel-side normal
Z0 = -N0  # the face away from the heel
HINGE = np.array([L.PIN_X, 0.0, L.ZH]) * L.MM
TIP_FACE = HINGE + L.LEV * L.MM * U0 + (L.LEV_T / 2) * L.MM * Z0  # the blade-side face's tip-edge centre, at rest
BLADE_TOP_PT = np.array([L.PIV[0], 0.0, L.blade_top(L.PIV[0])]) * L.MM  # on the sloping top, base frame
BLADE_NORMAL0 = np.array([math.sin(L.ALPHA), 0.0, math.cos(L.ALPHA)])
CORNERS = np.array([(x, y, 0.0) for x in (0.0, L.LEN) for y in (-L.WIDTH / 2, L.WIDTH / 2)]) * L.MM
ENDS = np.array([(0.0, 0.0, 1.0), (L.LEN, 0.0, 1.0)]) * L.MM


def turn_deg(r0: np.ndarray, r1: np.ndarray) -> float:
    """The angle (deg) of the rotation from orientation r0 to r1."""
    c = (float(np.trace(r0.T @ r1)) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


@dataclass(frozen=True)
class Reading:
    """What the scorer reads from one clipper state (m, deg)."""
    origin: np.ndarray  # the lower blade's frame origin (world)
    rot: np.ndarray  # and its rotation
    pin: float  # joint readings (rad)
    lever: float
    u_c: np.ndarray  # the lever's direction in the clipper frame
    n_c: np.ndarray  # its heel-side normal in the clipper frame
    gap: float  # tip face to the upper blade's top plane (m)
    corner_z: np.ndarray  # the four bottom corners' heights over the slot floor (m)
    track: np.ndarray  # (3, 3): the lever tip's blade-side face and the lower blade's two end centres (world)


def read(sc) -> Reading | None:
    """A Reading, or None when the state is missing, incomplete or not finite."""
    if sc is None:
        return None
    try:
        r_c = np.asarray(sc.rotation(), dtype=float)
        origin = np.asarray(sc.origin, dtype=float).ravel()
        pin, lever = float(sc.joints[PIN]), float(sc.joints[LEVER])
        r_l, _ = sc.link_pose(LEVER)
        r_l = np.asarray(r_l, dtype=float)
        tip = np.asarray(sc.link_to_world(LEVER, TIP_FACE), dtype=float).ravel()
        top_pt = np.asarray(sc.to_world(BLADE_TOP_PT[None, :]), dtype=float).ravel()
        corners = np.asarray(sc.to_world(CORNERS), dtype=float)
        ends = np.asarray(sc.to_world(ENDS), dtype=float)
    except (AttributeError, KeyError, TypeError, ValueError, RuntimeError):
        return None
    if r_c.shape != (3, 3) or r_l.shape != (3, 3) or origin.size != 3 or top_pt.size != 3:
        return None
    arrays = (r_c, r_l, origin, tip, top_pt, corners, ends)
    if not all(np.all(np.isfinite(a)) for a in arrays) or not (math.isfinite(pin) and math.isfinite(lever)):
        return None
    normal = r_c @ BLADE_NORMAL0
    gap = float((tip - top_pt) @ normal)
    return Reading(origin, r_c, pin, lever, r_c.T @ (r_l @ U0), r_c.T @ (r_l @ N0), gap,
                   corners[:, 2] - L.FLOOR_Z, np.vstack([tip[None, :], ends]))


def plan_angles(u_c: np.ndarray) -> tuple[float, float, float]:
    """(plan length in mm per the lever's full length, deg from +x_C, deg from -x_C) of u_plan."""
    plan = float(math.hypot(u_c[0], u_c[1]))
    ang = math.degrees(math.atan2(u_c[1], u_c[0]))
    return plan * L.LEV, abs(ang), 180.0 - abs(ang)


def clipper_distance(sc):
    """The distance (m) from world points to every shape of the clipper, for rigstate."""

    def distance(points):
        return sc.distance(points)

    return distance


@dataclass
class Result:
    score: float
    success: bool
    details: dict = field(default_factory=dict)


def tier(to_tail: bool, past_jaws: bool, heel_away: bool, full: bool, still: bool) -> float:
    if full and still:
        return 1.0
    if to_tail and heel_away:
        return 0.5
    if past_jaws:
        return 0.25
    return 0.0


def _mm(v) -> float | None:
    return None if v is None or not math.isfinite(v) else round(float(v) * 1000.0, 2)


class Tracker:
    """The last 20 steps' readings (stillness) and verdicts (the 1.0 conditions)."""

    def __init__(self, control_hz: float = L.CONTROL_HZ) -> None:
        self.window = max(1, int(round(L.STILL_WINDOW_S * control_hz)))  # 20 steps at 20 Hz
        self.resets = 0  # fallback resets seen (teleports back to the start), kept across resets
        self.reset()

    def reset(self) -> None:
        self.steps = 0
        self.bad_steps = 0
        self.hist: deque = deque(maxlen=self.window + 1)
        self.ok: deque = deque(maxlen=self.window)
        self.prev: Reading | None = None
        self.last = Result(0.0, False, {})

    # -- reset fallback ----------------------------------------------------------------------------------------
    @staticmethod
    def at_start(r: Reading) -> bool:
        """The clipper on its start pose with the lever as drawn (the cutting position)."""
        if float(np.linalg.norm(r.origin - np.array(L.ORIGIN, dtype=float))) > L.RESET_START_TOL:
            return False
        if turn_deg(np.eye(3), r.rot) > L.RESET_AT_START_DEG:
            return False
        limit = math.radians(L.RESET_AT_START_DEG)
        return abs(r.pin) <= limit and abs(r.lever) <= limit

    @classmethod
    def teleported(cls, prev: Reading, now: Reading) -> bool:
        """The simulator put the clipper back on its start pose with a jump no trial makes in one control step."""
        if not cls.at_start(now):
            return False
        return (math.degrees(abs(now.pin - prev.pin)) >= L.RESET_TURN_DEG
                or math.degrees(abs(now.lever - prev.lever)) >= L.RESET_TURN_DEG
                or float(np.linalg.norm(now.origin - prev.origin)) >= L.RESET_JUMP
                or turn_deg(prev.rot, now.rot) >= L.RESET_TURN_DEG)

    # -- per step ------------------------------------------------------------------------------------------------
    def update(self, states: dict, arms) -> Result:
        self.steps += 1
        sc = states.get(CLIPPER) if isinstance(states, dict) else None
        r = read(sc)
        if r is None:
            self.bad_steps += 1  # a blow-up or a missing state: 0, and the 20-step windows start again
            self.hist.clear()
            self.ok.clear()
            self.prev = None
            self.last = Result(0.0, False, {"valid_state": False, "bad_steps": self.bad_steps})
            return self.last
        if self.prev is not None and self.teleported(self.prev, r):
            self.reset()
            self.steps = 1
            self.resets += 1
        self.hist.append(r)
        self.prev = r
        try:
            dist = clipper_distance(sc)
            clear = bool(rigstate.released(arms, dist, L.LET_GO))
        except Exception:  # noqa: BLE001 - no distance: not clear
            dist, clear = None, False
        self.last = self.judge(r, arms, dist, clear)
        return self.last

    def motion(self) -> float | None:
        """Over the last 20 steps: the largest distance (m) any tracked point is from where it was at the window's
        start; None before 20 steps of valid readings."""
        if len(self.hist) < self.window + 1:
            return None
        h = list(self.hist)
        return max(float(np.linalg.norm(x.track - h[0].track, axis=1).max()) for x in h[1:])

    def judge(self, r: Reading, arms, dist, clear: bool) -> Result:
        plan_mm, from_tail, from_jaws = plan_angles(r.u_c)
        long_enough = plan_mm >= L.PLAN_MIN_MM
        to_tail = long_enough and from_tail <= L.TAIL_PLAN_DEG + L.EPS
        past_jaws = long_enough and from_jaws <= L.PAST_PLAN_DEG + L.EPS
        heel_away = bool(r.n_c[2] > 0.0)
        flat = r.gap <= L.FLAT_GAP + L.EPS
        in_slot = bool(np.all(np.abs(r.corner_z) <= L.SLOT_TOL + L.EPS))
        self.ok.append(bool(to_tail and heel_away and flat and in_slot and clear))
        full = len(self.ok) == self.window and all(self.ok)
        m = self.motion()
        still = bool(m is not None and m <= L.STILL_MOVE + L.EPS)
        level = tier(to_tail, past_jaws, heel_away, full, still)
        gap = None
        if dist is not None:
            try:
                gap = rigstate.hands_gap(arms, dist)
            except Exception:  # noqa: BLE001 - trace only
                gap = None
        details = {
            "valid_state": True, "score": level, "to_tail": bool(to_tail), "past_jaws": bool(past_jaws),
            "heel_away": heel_away, "flat": bool(flat), "in_slot": in_slot, "clear_now": bool(clear),
            "ok_20": bool(full), "still_20": still, "plan_mm": round(plan_mm, 2), "from_tail_deg": round(from_tail, 2),
            "from_jaws_deg": round(from_jaws, 2), "tip_gap_mm": _mm(r.gap), "pin_deg": round(math.degrees(r.pin), 2),
            "lever_deg": round(math.degrees(r.lever), 2), "corner_mm": [_mm(v) for v in r.corner_z],
            "motion_mm": None if m is None else _mm(m), "zone_mm": _mm(gap), "arms_readable": rigstate.readable(arms),
        }
        return Result(level, bool(level == 1.0), details)
