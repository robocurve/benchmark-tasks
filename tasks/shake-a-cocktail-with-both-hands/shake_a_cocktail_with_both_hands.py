"""On the station table, a steel cocktail shaker tin (19 cm tall, 7.6 cm across) stands upright in front of the arms on
the centre line, holding 12 beads (6 mm) for the ice and the drink. Ahead of it and a little to the right, its cap rests on
a steel stand as tall as the tin: a lid with a skirt that fits over the tin's mouth and a tall square knob on top. The
tin, the cap and the beads are loose; the stand is fixed.

Task: put the cap on the tin with one hand, hold the tin with the other, and shake it with both hands, tipped over on
its side, without losing a single bead; then stand the tin back on the table. A cap that is only resting on the tin
falls off as soon as it tips, so the capping hand has to keep holding it.

Run it:  inspect-robots run --embodiment yam_arena -E task=shake_a_cocktail_with_both_hands.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, on_reset, plate, rotation  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Cylinder  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the tin (origin at the centre of its underside, axis up) and the cap (origin at the centre of its top disc's
# underside, knob up, skirt down) ---------------------------------------------------------------------------------
TIN_R_I, TIN_T, TIN_H, TIN_FLOOR = 0.035, 0.003, 0.190, 0.005
TIN_R_O = TIN_R_I + TIN_T
TIN_MASS = 0.20
CAP_GAP, CAP_T, SKIRT_H, DISC_T = 0.004, 0.003, 0.015, 0.004  # the skirt clears the tin by CAP_GAP all round at the top
FLARE = 0.005  # ... and flares out by this much at its bottom edge, a lead-in that guides the cap onto the tin
SKIRT_R_I = TIN_R_O + CAP_GAP
KNOB_X, KNOB_Y, KNOB_H = 0.022, 0.030, 0.052  # a tall square knob: flat faces left and right for the fingers
CAP_MASS = 0.08

# -- layout (world frame: x forward from the arm bases, y left, z up; metres) ----------------------------------------
TIN_XY = (0.22, 0.0)
STAND_XY = (0.33, -0.05)  # ahead and to the right: not behind the tin, where the capping hand's body ends up
STAND_R = 0.020
STAND_H = TIN_H  # the cap sits on the stand at the height it will sit on the tin

BEAD_R, BEAD_MASS, N_BEADS = 0.003, 0.0008, 12

STEEL, CAP_C, BEAD_C, STAND_C = (0.72, 0.73, 0.75), (0.80, 0.81, 0.83), (0.85, 0.20, 0.30), (0.35, 0.36, 0.38)

# -- scoring --------------------------------------------------------------------------------------------------------
TIPPED_OVER = math.radians(80)  # the tin must have been tipped this far ...
SHAKE_TIP = math.radians(60)  # ... and, while tipped past SHAKE_TIP, moved back and forth along its own axis:
TURNS = 4  # at least this many changes of direction,
LEG = 0.02  # each after a leg of at least this much (tipping it over and back alone makes at most one such turn)
HALF_LOST = 2
UPRIGHT = math.radians(10)
SUCCESS_HOLD_S = 1.0


def _wall(n: int, r_mid: float, z0: float, z1: float, t: float, color) -> list:
    out = []
    for k in range(n):
        a = 2 * math.pi * k / n
        u = (math.cos(a), math.sin(a), 0.0)
        out.append(plate((r_mid * u[0], r_mid * u[1], z0), (r_mid * u[0], r_mid * u[1], z1),
                         2 * r_mid * math.tan(math.pi / n) + 0.001, t, u, color))
    return out


def _tin() -> Body:
    shapes = [CylShape((0.0, 0.0, TIN_FLOOR / 2), TIN_R_O, TIN_FLOOR, STEEL)]
    shapes += _wall(20, TIN_R_I + TIN_T / 2, TIN_FLOOR, TIN_H, TIN_T, STEEL)
    return Body.build("tin", pos=(*TIN_XY, TOP + 0.0005), mass=TIN_MASS, shapes=shapes, friction=(0.7, 0.6), color=STEEL)


def _skirt(n: int, color) -> list:
    """The cap's skirt: a cone of n panels, CAP_GAP clear of the tin at the disc, FLARE wider at its bottom edge. Each
    panel is as wide as the cone's widest chord, so neighbours overlap and leave no slot."""
    r0, r1 = SKIRT_R_I + CAP_T / 2, SKIRT_R_I + FLARE + CAP_T / 2
    out = []
    for k in range(n):
        a = 2 * math.pi * k / n
        u = (math.cos(a), math.sin(a), 0.0)
        out.append(plate((r0 * u[0], r0 * u[1], 0.0), (r1 * u[0], r1 * u[1], -SKIRT_H),
                         2 * r1 * math.tan(math.pi / n) + 0.001, CAP_T, u, color))
    return out


def _cap() -> Body:
    shapes = [CylShape((0.0, 0.0, DISC_T / 2), SKIRT_R_I + CAP_T, DISC_T, CAP_C),
              BoxShape((0.0, 0.0, DISC_T + KNOB_H / 2), (KNOB_X, KNOB_Y, KNOB_H), CAP_C)]
    shapes += _skirt(24, CAP_C)
    return Body.build("cap", pos=(*STAND_XY, TOP + STAND_H + 0.0005), mass=CAP_MASS, shapes=shapes, friction=(0.7, 0.6),
                      color=CAP_C)


def _bead_spots() -> list[tuple[float, float, float]]:
    d = 2 * BEAD_R + 0.0008
    rmax = TIN_R_I - BEAD_R - 0.0015
    z = TIN_FLOOR + BEAD_R + 0.0005
    pts = [((i + 0.5 * (j % 2)) * d, j * d * math.sqrt(3) / 2, z) for j in range(-6, 7) for i in range(-6, 7)]
    pts = sorted([p for p in pts if math.hypot(p[0], p[1]) <= rmax], key=lambda p: (round(math.hypot(p[0], p[1]), 6), p))
    return pts[:N_BEADS]


def _beads() -> list[Body]:
    tx, ty = TIN_XY
    return [Body.build(f"ice_{i:02d}", pos=(tx + x, ty + y, TOP + 0.0005 + z), mass=BEAD_MASS, color=BEAD_C,
                       friction=(0.3, 0.25), shapes=[SphereShape((0.0, 0.0, 0.0), BEAD_R, BEAD_C)], damping=(0.05, 0.3))
            for i, (x, y, z) in enumerate(_bead_spots())]


TIN = _tin()
CAP = _cap()
BEADS = _beads()
OBJECTS = [Cylinder("cap_stand", pos=(*STAND_XY, TOP + STAND_H / 2), radius=STAND_R, height=STAND_H, color=STAND_C),
           TIN, CAP, *BEADS]
BEAD_NAMES = [b.name for b in BEADS]


# -- scoring --------------------------------------------------------------------------------------------------------

_EPISODE = {"max_tip": 0.0, "turns": 0, "last": None, "s": 0.0, "ext": 0.0, "lo": 0.0, "hi": 0.0, "dir": 0}


@on_reset
def _new_episode() -> None:
    _EPISODE.update(max_tip=0.0, turns=0, last=None)
    _restart_stroke()


def _restart_stroke() -> None:
    _EPISODE.update(s=0.0, ext=0.0, lo=0.0, hi=0.0, dir=0)


def _track_shake(centre, axis, tip: float) -> None:
    """Count the turns of the tin's back-and-forth along its own axis while it is tipped past SHAKE_TIP: s integrates
    the centre's motion along the axis (so turning the tin about its cap or its base hardly moves s), and a turn counts
    once s has come back LEG or more from its last extreme."""
    e = _EPISODE
    last, e["last"] = e["last"], centre
    if tip < SHAKE_TIP or last is None:
        _restart_stroke()
        return
    e["s"] += float(np.dot(centre - last, axis))
    s = e["s"]
    if e["dir"] == 0:  # the first leg: its direction is set once s has moved LEG either way
        e["lo"], e["hi"] = min(e["lo"], s), max(e["hi"], s)
        if e["hi"] - e["lo"] >= LEG:
            e["dir"], e["ext"] = (1, s) if s >= e["hi"] else (-1, s)
    elif (s - e["ext"]) * e["dir"] > 0:  # still going the same way: a new extreme
        e["ext"] = s
    elif abs(s - e["ext"]) >= LEG:  # came back far enough: a turn
        e["turns"] += 1
        e["dir"], e["ext"] = -e["dir"], s


def _tilt(rot) -> float:
    return float(math.acos(max(-1.0, min(1.0, rot[2, 2]))))


def _tally(objects) -> dict:
    tin, cap = objects["tin"], objects["cap"]
    t_rot, c_rot = rotation(tin.quat_xyzw), rotation(cap.quat_xyzw)
    inside = 0
    for name in BEAD_NAMES:
        w = np.asarray(objects[name].pos, float)
        t = t_rot.T @ (w - tin.pos)
        c = c_rot.T @ (w - cap.pos)
        in_tin = math.hypot(t[0], t[1]) < TIN_R_I and 0.0 < t[2] < TIN_H
        in_cap = math.hypot(c[0], c[1]) < SKIRT_R_I and -SKIRT_H < c[2] < 0.0
        inside += in_tin or in_cap
    tip = _tilt(t_rot)
    _EPISODE["max_tip"] = max(_EPISODE["max_tip"], tip)
    _track_shake(np.asarray(tin.pos, float) + t_rot[:, 2] * TIN_H / 2, t_rot[:, 2], tip)
    return {"inside": inside, "lost": N_BEADS - inside, "tip": tip, "max_tip": _EPISODE["max_tip"],
            "turns": _EPISODE["turns"],
            "upright_on_table": tip < UPRIGHT and abs(tin.pos[2] - TOP) < 0.01}


def _points(t) -> float:
    if not t["upright_on_table"]:
        return 0.0
    tipped = t["max_tip"] >= TIPPED_OVER
    shaken = tipped and t["turns"] >= TURNS
    if shaken and t["lost"] == 0:
        return 1.0
    if shaken and t["lost"] <= HALF_LOST:
        return 0.5
    if tipped and t["lost"] == 0:
        return 0.25
    return 0.0


_DEBUG = {"on": bool(os.environ.get("SHAKE_DEBUG")), "calls": 0}


def score(objects) -> float:
    """0 unless the tin ends standing upright on the table. Otherwise 1.0 if it was tipped 80 degrees or more and,
    while tipped past 60 degrees, shaken back and forth along its axis (4 or more changes of direction, each after a leg
    of 2 cm or more), with none of the 12 beads lost (outside both the tin and the cap); 0.5 if so shaken with at most 2
    lost; 0.25 if tipped 80 degrees but shaken less, with none lost; else 0."""
    t = _tally(objects)
    if _DEBUG["on"]:  # SHAKE_DEBUG=1: one line a second (at 20 Hz) on stderr
        _DEBUG["calls"] += 1
        if _DEBUG["calls"] % 20 == 0:
            tin, cap = objects["tin"], objects["cap"]
            gap = np.asarray(cap.pos) - (np.asarray(tin.pos) + rotation(tin.quat_xyzw)[:, 2] * TIN_H)
            print(f"[shake] t={_DEBUG['calls'] / 20:5.1f}s inside={t['inside']} tip={math.degrees(t['tip']):.0f}deg "
                  f"max_tip={math.degrees(t['max_tip']):.0f}deg turns={t['turns']} "
                  f"cap_off_rim_cm={np.round(gap * 100, 1).tolist()} upright={t['upright_on_table']} "
                  f"score={_points(t)}", file=sys.stderr, flush=True)
    return _points(t)


def success(objects) -> bool:
    """Full score with the tin, the cap and every bead still."""
    if _points(_tally(objects)) < 1.0:
        return False
    return all(objects[n].resting() for n in ("tin", "cap", *BEAD_NAMES))


# -- scripted demo --------------------------------------------------------------------------------------------------
# Level grips from each arm's own side, fingers closing front and back: the right one on the cap's knob, the left one on
# the tin's body. The right hand comes down on the knob, lifts the cap off its stand and sets it on the tin; the left
# slides in and grips the tin. Then the shaker follows one shared path, given as the knob grip's position and how far
# the shaker is tipped forward (about world y); both wrists roll with it.


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


SIDE_GRIP = {
    "left": _rx(-math.pi / 2) @ grip_rotation(-math.pi / 2),  # pointing -y
    "right": _rx(math.pi / 2) @ grip_rotation(math.pi / 2),  # pointing +y
}
KNOB_GRIP = DISC_T + 0.037  # above the cap's origin: the fingers' roots reach 3.4 cm below the grip, so grip high
BODY_GRIP = 0.145  # above the tin's underside
OUT = 0.07
ABOVE = 0.05
SHAKE_TIP_TO = math.radians(100)  # the mouth a little below level: the beads lie against the cap
STROKE, STROKES, STROKE_S = 0.025, 4, 0.45  # each stroke 2.5 cm either side, straight forward and back
LIFT = 0.03
CAP_LEVEL = math.radians(4.5)  # the fingers' pads are not quite parallel: the gripped cap hangs 4.5 deg off level
TIN_LEVEL = math.radians(-6.0)  # ... and the lifted tin would lean in the left hand: turn that hand to match
SQUEEZE = 0.012  # while the shaker is off the table each hand aims this far into the other along the tin's axis, so
# the soft arms press the cap onto the rim (about 10 N) instead of letting it lift off
LEAD_S, LEAD_STEPS = 3.0, 15

KNOB_ON_STAND = np.array([STAND_XY[0], STAND_XY[1], TOP + STAND_H + KNOB_GRIP])
KNOB_ON_TIN = np.array([TIN_XY[0], TIN_XY[1], TOP + TIN_H + KNOB_GRIP])  # the cap's disc sits on the tin's rim
CAP_CLEAR = SKIRT_H + 0.012  # lift the cap this much to clear the stand and the tin's rim


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4), (0.0,)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed), base_pos=station.BASES[arm],
                             iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _lead_in(arm: str, pos, rot, label: str) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), FINGER_OPEN, label, r))
    return out


def _shaker_pose(arm: str, knob_at, tip: float, squeeze: float = 0.0):
    rot = _ry(tip)
    axis = rot @ np.array([0.0, 0.0, 1.0])  # the tin's axis, toward the cap
    pos = np.asarray(knob_at, float) + (axis if arm == "left" else -axis) * squeeze
    if arm == "left":
        pos = pos + rot @ np.array([0.0, 0.0, BODY_GRIP - (TIN_H + KNOB_GRIP)])
        return pos, rot @ _ry(TIN_LEVEL) @ SIDE_GRIP[arm]
    return pos, rot @ _ry(CAP_LEVEL) @ SIDE_GRIP[arm]  # the right hand turned so that the cap it holds sits level


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


# shared shaker path after both grips hold: (seconds, knob position, tip, label)
LIFTED = KNOB_ON_TIN + [0.0, 0.0, LIFT]


def _shake_path() -> list:
    path = [(1.5, LIFTED, 0.0, "lift the shaker"), (2.5, LIFTED, SHAKE_TIP_TO, "tip it on its side")]
    axis = np.array([1.0, 0.0, 0.0])  # one fixed, level direction: forward (toward the cap) and back
    for k in range(STROKES):
        path.append((STROKE_S, LIFTED + axis * STROKE, SHAKE_TIP_TO, "shake: toward the cap"))
        path.append((STROKE_S, LIFTED - axis * STROKE, SHAKE_TIP_TO, "shake: toward the bottom"))
    path += [(STROKE_S, LIFTED, SHAKE_TIP_TO, "shake: back to the middle"),
             (2.5, LIFTED, 0.0, "stand it up"), (1.5, KNOB_ON_TIN + [0.0, 0.0, 0.003], 0.0, "set it down"),
             (2.0, KNOB_ON_TIN + [0.0, 0.0, 0.003], 0.0, "hold still: the trial ends once the full score has held 1 s")]
    return path


SHAKE = _shake_path()
CAPPING = [1.2, 0.8, 1.0, 2.0, 1.0]  # the right hand: down, grip, lift, over the tin, cap it
CLOSING = [1.5, 0.8]  # then the left hand: slide in, grip; the right holds the cap on meanwhile


def _right() -> list[Waypoint]:
    rot = SIDE_GRIP["right"]
    level = _ry(CAP_LEVEL) @ rot
    on_tin = KNOB_ON_TIN - [0.0, 0.0, 0.002]  # press it down 2 mm onto the rim
    out = _lead_in("right", KNOB_ON_STAND + [0.0, 0.0, ABOVE], rot, "level, over the cap's knob")
    out += [
        _w(CAPPING[0], KNOB_ON_STAND, FINGER_OPEN, "down round the knob", rot),
        _w(CAPPING[1], KNOB_ON_STAND, FINGER_CLOSED, "grip the knob", rot),
        _w(CAPPING[2], KNOB_ON_STAND + [0.0, 0.0, CAP_CLEAR], FINGER_CLOSED, "lift the cap off its stand, level it", level),
        _w(CAPPING[3], KNOB_ON_TIN + [0.0, 0.0, CAP_CLEAR], FINGER_CLOSED, "over the tin", level),
        _w(CAPPING[4], on_tin, FINGER_CLOSED, "cap the tin", level),
        _w(sum(CLOSING), on_tin, FINGER_CLOSED, "hold the cap on", level),
    ]
    for k, (dt, knob, tip, label) in enumerate(SHAKE):
        pos, r = _shaker_pose("right", knob, tip, SQUEEZE if k < len(SHAKE) - 2 else 0.0)
        out.append(_w(dt, pos, FINGER_CLOSED, label, r))
    pos, r = _shaker_pose("right", KNOB_ON_TIN + [0.0, 0.0, 0.003], 0.0)
    out += [_w(0.8, pos, FINGER_OPEN, "let go of the cap", r), _w(1.2, pos + [0.0, 0.0, ABOVE], FINGER_OPEN, "up", r)]
    return out


def _left() -> list[Waypoint]:
    side = np.array([0.0, OUT, 0.0])
    grip, rot = _shaker_pose("left", KNOB_ON_TIN, 0.0)
    out = _lead_in("left", grip + side, rot, "level, beside the tin")
    out += [
        _w(sum(CAPPING), grip + side, FINGER_OPEN, "wait while the tin is capped", rot),
        _w(CLOSING[0], grip, FINGER_OPEN, "slide in round the tin", rot),
        _w(CLOSING[1], grip, FINGER_CLOSED, "grip the tin", rot),
    ]
    for k, (dt, knob, tip, label) in enumerate(SHAKE):
        pos, r = _shaker_pose("left", knob, tip, SQUEEZE if k < len(SHAKE) - 2 else 0.0)
        out.append(_w(dt, pos, FINGER_CLOSED, label, r))
    pos, r = _shaker_pose("left", KNOB_ON_TIN + [0.0, 0.0, 0.003], 0.0)
    out += [_w(0.8, pos, FINGER_OPEN, "let go of the tin", r), _w(1.2, pos + side, FINGER_OPEN, "slide out", r)]
    return out


def demo():
    return {"left": _left(), "right": _right()}
