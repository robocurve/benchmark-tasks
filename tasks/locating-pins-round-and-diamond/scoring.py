"""The scorer, as plain functions of the pins', the bracket's and the arms' state. NumPy only.

``Tracker.update(states, arms)`` -> Result(score, success, details), fed one observation per control step. It reads the
mechkit AssemblyState of each loose part: the three pins (base frame at the centre of the flange's underside, local z
up the axis, local x along the diamond head's wide axis) and the bracket (base frame at the centre of its underside,
local z up, local y along its length).

    HOME        a pin is home when its flange is within 0.5 mm of the plate's top (its origin 20.0 mm above the table,
                1 mm of slack below), its axis is within 3 degrees of vertical, head up, and its axis is within 1 mm
                of a hole's centre. A home pin is locked (task.py's weld); the scorer reads only the pose.
    LOCATED SET one round pin home in one hole and the diamond pin home in the other, its wide axis (the lands) within
                15 degrees of square to the line between the holes (along x)
    SEATED      the bracket's hole over the round pin has its centre within 0.3 mm of the pin's axis, the bracket's
                underside is within 0.3 mm of the rest pads' top (25.0 mm), it is level within 0.3 degrees and its
                length is square to the line between the holes within 0.3 degrees (either way round)
    RELEASED    on every step of the last 1 s both grippers' finger zones are at least 25 mm from the bracket and from
                every pin (rigstate.released with the parts' own distances; unreadable arms: not released)
    STILL       over the last 1 s the bracket's origin and both hole centres moved under 0.3 mm from where they were
                at the window's start and the bracket turned under 0.3 degrees
    OFF TABLE   now: a pin or the bracket with its centre outside the table's outline seen from above, or its lowest
                point more than 10 mm below the table top

    0      off the table at the end
    1.0    located set, bracket seated, released and still        (the trial ends once this has held 1 s)
    0.5    located set (the bracket is not seated)
    0.25   any one pin home, but no located set (a bracket seated on such pins earns only this)
    0      anything else
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

import layout as L
import rigstate

BRACKET = "bracket"
PARTS = (*L.PIN_NAMES, BRACKET)


def rotation(quat_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def turn_deg(r0: np.ndarray, r1: np.ndarray) -> float:
    c = (float(np.trace(r0.T @ r1)) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def tilt_deg(r: np.ndarray) -> float:
    """How far (deg) the body's z axis leans from straight up (180 for upside down)."""
    return math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))


def square_err_deg(vec_xy, axis: str) -> float:
    """The angle (deg, 0..90) between a horizontal direction and the world x axis (axis "x") or y axis ("y"), either
    way along it."""
    a = math.degrees(math.atan2(float(vec_xy[1]), float(vec_xy[0])))
    ref = 0.0 if axis == "x" else 90.0
    d = abs((a - ref + 90.0) % 180.0 - 90.0)
    return d


@dataclass(frozen=True)
class Reading:
    """What the scorer reads from one step (m and deg). Positions are in the world frame with the table top at 0."""
    pos: dict  # part -> its base frame's origin (x, y, z above the table)
    rot: dict  # part -> rotation
    low: dict  # part -> lowest point above the table top
    centre: dict  # part -> bounding-box centre (x, y), for the table outline


def read(states) -> Reading | None:
    """A Reading, or None when any of the four states is missing, malformed or not finite."""
    if not isinstance(states, dict):
        return None
    pos, rot, low, centre = {}, {}, {}, {}
    try:
        for name in PARTS:
            s = states.get(name)
            if s is None:
                return None
            o = np.asarray(s.origin, dtype=float).ravel()
            r = rotation(s.quat_xyzw)
            lo = np.asarray(s.low, dtype=float).ravel()
            c = np.asarray(s.pos, dtype=float).ravel()
            if o.size != 3 or lo.size != 3 or c.size != 3:
                return None
            if not (np.all(np.isfinite(o)) and np.all(np.isfinite(r)) and np.all(np.isfinite(lo))
                    and np.all(np.isfinite(c))):
                return None
            pos[name] = o - np.array([0.0, 0.0, L.TOP])
            rot[name] = r
            low[name] = float(lo[2] - L.TOP)
            centre[name] = c[:2]
    except (AttributeError, TypeError, ValueError, KeyError):
        return None
    return Reading(pos, rot, low, centre)


def home_hole(r: Reading, name: str) -> str | None:
    """The hole ("A" or "B") a pin is home in, or None."""
    p, rot = r.pos[name], r.rot[name]
    if tilt_deg(rot) > L.HOME_TILT_DEG:
        return None
    if not (-0.001 <= p[2] - L.PIN_SEAT_Z <= L.HOME_TOL + L.EPS):
        return None
    for h, (hx, hy) in L.HOLES.items():
        if math.hypot(p[0] - hx, p[1] - hy) <= L.HOME_XY_TOL + L.EPS:
            return h
    return None


def diamond_turn_err(r: Reading) -> float:
    """How far (deg) the diamond's wide axis (its lands) is from square to the line between the holes (along y), i.e.
    from the world x axis."""
    return square_err_deg(r.rot[L.DIAMOND][:2, 0], "x")


def located_set(r: Reading):
    """(round pin name, its hole, diamond's hole) if a located set is home, else None."""
    holes = {n: home_hole(r, n) for n in L.PIN_NAMES}
    d = holes[L.DIAMOND]
    if d is None or diamond_turn_err(r) > L.TURN_TOL_DEG + L.EPS:
        return None
    for n in L.ROUND:
        if holes[n] is not None and holes[n] != d:
            return n, holes[n], d
    return None


def seat(r: Reading, round_name: str) -> dict:
    """The bracket's pose against its located pose over ``round_name``: the offset (m) of the hole over the round pin
    from the pin's axis, the underside's height error (m), tilt and the length's turn from square to the line (deg)."""
    o, rot = r.pos[BRACKET], r.rot[BRACKET]
    holes = [o[:2] + rot[:2, 1] * (s * L.BR_HOLE_Y) for s in (1.0, -1.0)]
    pin = r.pos[round_name][:2]
    off = min(float(np.linalg.norm(h - pin)) for h in holes)
    return {"hole_off": off, "z_err": float(o[2] - L.PAD_TOP), "tilt": tilt_deg(rot),
            "square": square_err_deg(rot[:2, 1], "y")}


def is_seated(s: dict) -> bool:
    return (s["hole_off"] <= L.SEAT_POS_TOL + L.EPS and abs(s["z_err"]) <= L.SEAT_Z_TOL + L.EPS
            and s["tilt"] <= L.SEAT_TURN_DEG + L.EPS and s["square"] <= L.SEAT_TURN_DEG + L.EPS)


def off_table(r: Reading) -> bool:
    for n in PARTS:
        x, y = r.centre[n]
        if not (L.TABLE_X0 <= x <= L.TABLE_X1 and L.TABLE_Y0 <= y <= L.TABLE_Y1):
            return True
        if r.low[n] < -L.OFF_TABLE_DROP:
            return True
    return False


def tier(off: bool, located, seated: bool, released: bool, still: bool, any_home: bool) -> float:
    if off:
        return 0.0
    if located is not None:
        return 1.0 if (seated and released and still) else 0.5
    return 0.25 if any_home else 0.0


@dataclass
class Result:
    score: float
    success: bool
    details: dict = field(default_factory=dict)


def _mm(v) -> float | None:
    return None if v is None or not math.isfinite(v) else round(float(v) * 1000.0, 3)


class Tracker:
    """The last 1 s of readings (stillness) and of the release check, and the whole-trial reset fallback."""

    def __init__(self, control_hz: float = L.CONTROL_HZ) -> None:
        self.window = max(1, int(round(L.STILL_WINDOW_S * control_hz)))
        self.resets = 0
        self.reset()

    def reset(self) -> None:
        self.steps = 0
        self.bad_steps = 0
        self.hist: deque = deque(maxlen=self.window + 1)
        self.released: deque = deque(maxlen=self.window)
        self.prev: Reading | None = None
        self.last = Result(0.0, False, {})

    # -- reset fallback ---------------------------------------------------------------------------------------
    @staticmethod
    def at_start(r: Reading) -> bool:
        if float(np.linalg.norm(r.pos[BRACKET][:2] - np.array(L.BR_START))) > L.START_TOL:
            return False
        return all(home_hole(r, n) is None for n in L.PIN_NAMES)

    @classmethod
    def teleported(cls, prev: Reading, now: Reading) -> bool:
        """The bracket and every pin put back on their starting marks with a jump no trial makes in one step."""
        if not cls.at_start(now):
            return False
        return any(float(np.linalg.norm(now.pos[n] - prev.pos[n])) >= L.RESET_JUMP for n in PARTS)

    # -- per step ---------------------------------------------------------------------------------------------
    def update(self, states, arms) -> Result:
        self.steps += 1
        r = read(states)
        if r is None:
            self.bad_steps += 1  # a blow-up or a missing state: 0, and the 1 s windows start again
            self.hist.clear()
            self.released.clear()
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
            dists = [states[n].distance for n in PARTS]

            def dist(points):
                return np.min([d(points) for d in dists], axis=0)
        except Exception:  # noqa: BLE001 - no distance: not released
            dist = None
        ok = False
        if dist is not None:
            try:
                ok = bool(rigstate.released(arms, dist, L.LET_GO))
            except Exception:  # noqa: BLE001
                ok = False
        self.released.append(ok)
        self.last = self.judge(r, arms, dist)
        return self.last

    def motion(self) -> dict | None:
        """Over the last 1 s: the largest move of the bracket's origin and of its two hole centres from where they were
        at the window's start (m) and the largest turn (deg); None before 1 s of valid readings."""
        if len(self.hist) < self.window + 1:
            return None
        h = list(self.hist)
        first = h[0]

        def holes(x):
            o, rot = x.pos[BRACKET], x.rot[BRACKET]
            return [o + rot[:, 1] * (s * L.BR_HOLE_Y) for s in (1.0, -1.0)]

        h0 = holes(first)
        move = 0.0
        turn = 0.0
        for x in h[1:]:
            move = max(move, float(np.linalg.norm(x.pos[BRACKET] - first.pos[BRACKET])),
                       *(float(np.linalg.norm(a - b)) for a, b in zip(holes(x), h0)))
            turn = max(turn, turn_deg(first.rot[BRACKET], x.rot[BRACKET]))
        return {"move": move, "turn": turn}

    def judge(self, r: Reading, arms, dist) -> Result:
        off = off_table(r)
        homes = {n: home_hole(r, n) for n in L.PIN_NAMES}
        loc = located_set(r)
        seat_info = seat(r, loc[0]) if loc is not None else None
        seated = bool(seat_info is not None and is_seated(seat_info))
        released = len(self.released) == self.window and all(self.released)
        m = self.motion()
        still = bool(m is not None and m["move"] <= L.STILL_MOVE + L.EPS and m["turn"] <= L.STILL_TURN_DEG + L.EPS)
        level = tier(off, loc, seated, released, still, any(h is not None for h in homes.values()))
        gap = None
        if dist is not None:
            try:
                gap = rigstate.hands_gap(arms, dist)
            except Exception:  # noqa: BLE001 - trace only
                gap = None
        details = {
            "valid_state": True, "score": level, "off_table": bool(off), "homes": homes,
            "located": None if loc is None else {"round": loc[0], "round_hole": loc[1], "diamond_hole": loc[2]},
            "diamond_turn_err_deg": round(diamond_turn_err(r), 3),
            "pin_z_mm": {n: _mm(r.pos[n][2] - L.PIN_SEAT_Z) for n in L.PIN_NAMES},
            "bracket_z_mm": _mm(r.pos[BRACKET][2] - L.PAD_TOP), "bracket_tilt_deg": round(tilt_deg(r.rot[BRACKET]), 3),
            "bracket_xy_mm": [_mm(r.pos[BRACKET][0] - L.FIX_X), _mm(r.pos[BRACKET][1] - L.FIX_Y)],
            "seat": None if seat_info is None else {"hole_off_mm": _mm(seat_info["hole_off"]),
                                                    "z_err_mm": _mm(seat_info["z_err"]),
                                                    "tilt_deg": round(seat_info["tilt"], 3),
                                                    "square_deg": round(seat_info["square"], 3)},
            "seated": seated, "released_now": bool(self.released[-1]), "released_1s": bool(released), "still_1s": still,
            "motion": None if m is None else {"move_mm": _mm(m["move"]), "turn_deg": round(m["turn"], 3)},
            "zone_mm": _mm(gap), "arms_readable": rigstate.readable(arms),
        }
        return Result(level, bool(level == 1.0), details)
