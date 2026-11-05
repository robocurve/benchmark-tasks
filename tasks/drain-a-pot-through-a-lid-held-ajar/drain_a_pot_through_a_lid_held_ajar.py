"""On the station table, a saucepan with its lid on stands on a fixed stove block in front of the arms, its handle
pointing back toward the arms. In it are six dumplings (24 mm beads) in their cooking water (twenty 5 mm beads). In
front of the stove is a fixed sink basin. The pan, the lid and the beads are loose.

Task: drain the water into the sink but keep every dumpling in the pan: hold the lid on with one hand, slid back a
little so that a narrow gap opens at the front rim, while the other hand lifts the pan by its handle and tips it forward
over the sink; then set the pan back on the stove. Tipped with the lid loose, the lid slides off and the dumplings follow
the water.

Run it:  inspect-robots run --embodiment yam_arena -E task=drain_a_pot_through_a_lid_held_ajar.py \\
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
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the pan (origin at the centre of its underside) and the lid (origin at the centre of its disc's underside) -------
R_IN, WALL_T, WALL_H, FLOOR_T = 0.055, 0.003, 0.075, 0.005
R_OUT = R_IN + WALL_T
POT_H = FLOOR_T + WALL_H
POT_MASS = 0.40
HANDLE_LEN, HANDLE_W, HANDLE_H = 0.070, 0.025, 0.022  # long (pointing back, -x), wide (y), tall
HANDLE_Z = POT_H - 0.005 - HANDLE_H / 2  # its centre, just under the rim
LID_R, DISC_T, LID_MASS = R_OUT + 0.010, 0.004, 0.08  # the lid overhangs the rim by 1 cm
KNOB_X, KNOB_Y, KNOB_H = 0.022, 0.030, 0.052  # a tall square knob: flat faces front and back for the fingers

# -- layout (world frame: x forward from the arm bases, y left, z up; metres) ----------------------------------------
STOVE_XY, STOVE_R, STOVE_H = (0.27, -0.01), 0.050, 0.130
STOVE_TOP = TOP + STOVE_H
SINK_XY, SINK_IN, SINK_T, SINK_H = (0.41, -0.01), 0.16, 0.004, 0.07  # square basin: inside width, wall, height

WATER_R, WATER_MASS, N_WATER = 0.0025, 0.0005, 20
DUMPLING_R, DUMPLING_MASS, N_DUMPLING = 0.012, 0.008, 6

STEEL, DARK, LID_C, STONE = (0.72, 0.73, 0.75), (0.10, 0.10, 0.11), (0.80, 0.81, 0.83), (0.30, 0.30, 0.32)
WATER_C, DUMPLING_C, SINK_C = (0.30, 0.60, 0.95), (0.95, 0.85, 0.45), (0.85, 0.86, 0.88)

# -- scoring --------------------------------------------------------------------------------------------------------
FULL_DRAINED, HALF_DRAINED, SOME_DRAINED = 16, 10, 5  # of the 20 water beads in the sink
ON_STOVE, UPRIGHT = 0.04, math.radians(10)  # the pan back on the stove: centre within this, tilted less than this
SUCCESS_HOLD_S = 1.0


def _wall(n: int, r_in: float, t: float, z0: float, z1: float, color) -> list:
    """A round wall of n panels, each as wide as the outer chord (neighbours overlap: no slots)."""
    r_mid, r_out = r_in + t / 2, r_in + t
    out = []
    for k in range(n):
        a = 2 * math.pi * k / n
        u = (math.cos(a), math.sin(a), 0.0)
        out.append(plate((r_mid * u[0], r_mid * u[1], z0), (r_mid * u[0], r_mid * u[1], z1),
                         2 * r_out * math.tan(math.pi / n) + 0.001, t, u, color))
    return out


def _pot() -> Body:
    shapes = [CylShape((0.0, 0.0, FLOOR_T / 2), R_OUT, FLOOR_T, STEEL)]
    shapes += _wall(28, R_IN, WALL_T, FLOOR_T, POT_H, STEEL)
    shapes.append(BoxShape((-(R_OUT + HANDLE_LEN / 2 - 0.002), 0.0, HANDLE_Z), (HANDLE_LEN, HANDLE_W, HANDLE_H), DARK))
    return Body.build("pan", pos=(*STOVE_XY, STOVE_TOP + 0.0005), mass=POT_MASS, shapes=shapes, friction=(0.6, 0.5),
                      color=STEEL)


def _lid() -> Body:
    shapes = [CylShape((0.0, 0.0, DISC_T / 2), LID_R, DISC_T, LID_C),
              BoxShape((0.0, 0.0, DISC_T + KNOB_H / 2), (KNOB_X, KNOB_Y, KNOB_H), DARK)]
    return Body.build("lid", pos=(*STOVE_XY, STOVE_TOP + POT_H + 0.001), mass=LID_MASS, shapes=shapes,
                      friction=(0.5, 0.4), color=LID_C)


def _beads() -> list[Body]:
    sx, sy = STOVE_XY
    base = STOVE_TOP + FLOOR_T + 0.001
    out = []
    for i in range(N_DUMPLING):  # a ring on the floor
        a = 2 * math.pi * i / N_DUMPLING
        out.append(Body.build(f"dumpling_{i}", pos=(sx + 0.027 * math.cos(a), sy + 0.027 * math.sin(a), base + DUMPLING_R),
                              mass=DUMPLING_MASS, color=DUMPLING_C, friction=(0.5, 0.4),
                              shapes=[SphereShape((0.0, 0.0, 0.0), DUMPLING_R, DUMPLING_C)]))
    d = 2 * WATER_R + 0.003  # the water: a loose grid above the dumplings, to settle among them
    spots = [(i * d, j * d) for i in range(-4, 5) for j in range(-4, 5) if math.hypot(i * d, j * d) < R_IN - 0.006]
    spots.sort(key=lambda p: (round(math.hypot(*p), 6), p))
    for i, (x, y) in enumerate(spots[:N_WATER]):
        out.append(Body.build(f"water_{i:02d}", pos=(sx + x, sy + y, base + 2 * DUMPLING_R + 0.004 + WATER_R),
                              mass=WATER_MASS, color=WATER_C, friction=(0.2, 0.15), damping=(0.05, 0.2),
                              shapes=[SphereShape((0.0, 0.0, 0.0), WATER_R, WATER_C)]))
    return out


def _sink() -> list:
    x, y = SINK_XY
    half = SINK_IN / 2 + SINK_T / 2
    span = SINK_IN + 2 * SINK_T
    z = TOP + SINK_H / 2
    return [Box("sink_floor", pos=(x, y, TOP + 0.002), size=(span, span, 0.004), color=SINK_C),
            Box("sink_front", pos=(x + half, y, z), size=(SINK_T, span, SINK_H), color=SINK_C),
            Box("sink_back", pos=(x - half, y, z), size=(SINK_T, span, SINK_H), color=SINK_C),
            Box("sink_left", pos=(x, y + half, z), size=(span, SINK_T, SINK_H), color=SINK_C),
            Box("sink_right", pos=(x, y - half, z), size=(span, SINK_T, SINK_H), color=SINK_C)]


POT = _pot()
LID = _lid()
BEADS = _beads()
OBJECTS = [Cylinder("stove", pos=(*STOVE_XY, TOP + STOVE_H / 2), radius=STOVE_R, height=STOVE_H, color=STONE),
           *_sink(), POT, LID, *BEADS]
WATER_NAMES = [b.name for b in BEADS if b.name.startswith("water")]
DUMPLING_NAMES = [b.name for b in BEADS if b.name.startswith("dumpling")]


# -- scoring ------------------------------------------------------------------------------------------------------

def _tilt(rot) -> float:
    return float(math.acos(max(-1.0, min(1.0, rot[2, 2]))))


def _in_pot(p, pot) -> bool:
    local = rotation(pot.quat_xyzw).T @ (np.asarray(p, float) - np.asarray(pot.pos, float))
    return math.hypot(local[0], local[1]) < R_IN and 0.0 < local[2] < POT_H + 0.01


def _in_sink(p) -> bool:
    return (abs(p[0] - SINK_XY[0]) < SINK_IN / 2 and abs(p[1] - SINK_XY[1]) < SINK_IN / 2
            and TOP < p[2] < TOP + SINK_H)


def _tally(objects) -> dict:
    pot = objects["pan"]
    drained = sum(_in_sink(objects[n].pos) for n in WATER_NAMES)
    kept = sum(_in_pot(objects[n].pos, pot) for n in DUMPLING_NAMES)
    p = np.asarray(pot.pos, float)
    on_stove = (_tilt(rotation(pot.quat_xyzw)) < UPRIGHT and abs(p[2] - STOVE_TOP) < 0.01
                and math.hypot(p[0] - STOVE_XY[0], p[1] - STOVE_XY[1]) < ON_STOVE)
    return {"drained": int(drained), "kept": int(kept), "on_stove": on_stove}


def _points(t) -> float:
    if not t["on_stove"]:
        return 0.0
    if t["kept"] == N_DUMPLING and t["drained"] >= FULL_DRAINED:
        return 1.0
    if t["kept"] >= N_DUMPLING - 1 and t["drained"] >= HALF_DRAINED:
        return 0.5
    if t["kept"] == N_DUMPLING and t["drained"] >= SOME_DRAINED:
        return 0.25
    return 0.0


_DEBUG = {"on": bool(os.environ.get("DRAIN_DEBUG")), "calls": 0}


@on_reset
def _new_episode() -> None:
    _DEBUG["calls"] = 0


def score(objects) -> float:
    """0 unless the pan ends standing upright on the stove. Otherwise 1.0 with at least 16 of the 20 water beads in the
    sink and all 6 dumplings in the pan; 0.5 with at least 10 in the sink and at least 5 dumplings in the pan; 0.25 with at
    least 5 in the sink and all 6 dumplings in the pan; else 0."""
    t = _tally(objects)
    if _DEBUG["on"]:  # DRAIN_DEBUG=1: one line a second (at 20 Hz) on stderr
        _DEBUG["calls"] += 1
        if _DEBUG["calls"] % 20 == 0:
            pot, lid = objects["pan"], objects["lid"]
            rel = rotation(pot.quat_xyzw).T @ (np.asarray(lid.pos) - np.asarray(pot.pos)) - np.array([0.0, 0.0, POT_H])
            in_pot = sum(_in_pot(objects[n].pos, pot) for n in WATER_NAMES)
            lid_tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(rotation(pot.quat_xyzw)[:, 2] @ rotation(lid.quat_xyzw)[:, 2])))))
            print(f"[drain] t={_DEBUG['calls'] / 20:5.1f}s drained={t['drained']} water_in_pan={in_pot} lid_tilt={lid_tilt:.1f}deg "
                  f"dumplings_kept={t['kept']} pan_tip={math.degrees(_tilt(rotation(pot.quat_xyzw))):.0f}deg "
                  f"lid_vs_pan_cm={np.round(rel * 100, 1).tolist()} on_stove={t['on_stove']} score={_points(t)}",
                  file=sys.stderr, flush=True)
    return _points(t)


def success(objects) -> bool:
    """Full score with the pan and every bead still."""
    if _points(_tally(objects)) < 1.0:
        return False
    return all(objects[n].resting() for n in ("pan", *WATER_NAMES, *DUMPLING_NAMES))


# -- scripted demo --------------------------------------------------------------------------------------------------
# Side grips from each arm's own side: the left one across the handle, its wrist rolled so the fingers close top and
# bottom (the pan's weight then turns it about an axis the pads hold); the right one level, high on the lid's knob,
# fingers closing front and back. The right hand slides the lid 2.5 cm back (a gap of about 1.2 cm opens at the front
# rim) and keeps it pressed on; the left hand slides in from the left round the handle and grips it. Then the pan
# follows one shared path, given as the position of its front rim (the pouring lip) and how far it is tipped forward
# about that lip. Off the stove each hand aims a little into the other along the pan's axis, and the left one aims the
# pan 10 degrees back, against the sag of its soft wrist joints, so that the lid stays where the right hand put it.


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
KNOB_GRIP = DISC_T + 0.037  # above the lid's origin: the fingers' roots reach 3.4 cm below the grip, so grip high
SHIFT = 0.025  # slide the lid back this much: the front gap is SHIFT - (LID_R - R_IN) = 12 mm
HANDLE_GRIP_X = -(R_OUT + HANDLE_LEN / 2)  # the TCP on the middle of the handle
CAP_LEVEL = math.radians(4.5)  # the fingers' pads are not quite parallel: a gripped knob hangs 4.5 deg off level
HANDLE_ROLL = math.radians(-90)  # the left wrist rolled so its fingers close top and bottom on the handle: the pan's
# weight then turns it about an axis the pads hold (about the fingers' closing axis only friction would)
SAG = math.radians(10)  # off the stove the pan sags this far forward in the left hand (soft wrist joints): aim it back
SQUEEZE = 0.010  # off the stove, each hand aims this far into the other along the pan's axis: the lid stays pressed on
OUT, ABOVE, LIFT = 0.07, 0.05, 0.03
DRAIN_TIP = math.radians(100)
FORWARD = 0.04  # carry the pan this far forward: its lip ends up over the sink's back half
LEAD_S, LEAD_STEPS = 3.0, 15

POT0 = np.array([STOVE_XY[0], STOVE_XY[1], STOVE_TOP])  # the pan's origin on the stove
LIP = np.array([R_OUT, 0.0, POT_H])  # in the pan's frame
GRIPS = {"left": np.array([HANDLE_GRIP_X, 0.0, HANDLE_Z]),  # in the pan's frame
         "right": np.array([-SHIFT, 0.0, POT_H + KNOB_GRIP])}
LIP0 = POT0 + LIP


def _pose(arm: str, lip_at, tip: float, squeeze: float = 0.0):
    """The TCP pose of ``arm`` with the pan's lip at ``lip_at`` and the pan tipped ``tip`` forward about it."""
    rot = _ry(tip)
    axis = rot @ np.array([0.0, 0.0, 1.0])
    pos = np.asarray(lip_at, float) + rot @ (GRIPS[arm] - LIP) + (axis if arm == "left" else -axis) * squeeze
    level = HANDLE_ROLL if arm == "left" else CAP_LEVEL
    return pos, rot @ _ry(level) @ SIDE_GRIP[arm]


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


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


LIFTED = LIP0 + [0.0, 0.0, LIFT]
OVER = LIFTED + [FORWARD, 0.0, 0.0]
# (seconds, lip position, tip, squeeze?, label) for both hands together once both grips hold
PATH = [(1.5, LIFTED, 0.0, True, "lift the pan off the stove"),
        (2.0, OVER, 0.0, True, "carry it over the sink"),
        (2.5, OVER, DRAIN_TIP, True, "tip it forward: the water runs out under the lid"),
        (2.0, OVER, DRAIN_TIP, True, "let it drain"),
        *[(0.5, OVER, DRAIN_TIP - math.radians(15 if k % 2 == 0 else 0), True, "jiggle it to free the trapped water")
          for k in range(4)],
        (1.5, OVER, DRAIN_TIP, True, "let it drain"),
        (2.5, OVER, 0.0, True, "tip it back up"),
        (2.0, LIFTED, 0.0, True, "carry it back"),
        (1.5, LIP0 + [0.0, 0.0, 0.003], 0.0, False, "set it down on the stove"),
        (2.0, LIP0 + [0.0, 0.0, 0.003], 0.0, False, "hold still: the trial ends once the full score has held 1 s")]
RIGHT_PREP = [1.2, 0.8, 1.0]  # down round the knob, grip, slide the lid back
LEFT_PREP = [1.5, 0.8]  # slide in round the handle, grip


def _right() -> list[Waypoint]:
    centred, rot = _pose("right", LIP0, 0.0)
    centred = centred + [SHIFT, 0.0, 0.0]  # the knob before the lid is slid back
    back = centred - [SHIFT, 0.0, 0.002]  # slid back, pressed 2 mm onto the rim
    out = _lead_in("right", centred + [0.0, 0.0, ABOVE], rot, "level, over the knob")
    out += [_w(RIGHT_PREP[0], centred, FINGER_OPEN, "down round the knob", rot),
            _w(RIGHT_PREP[1], centred, FINGER_CLOSED, "grip the knob", rot),
            _w(RIGHT_PREP[2], back, FINGER_CLOSED, "slide the lid back: a gap opens at the front", rot),
            _w(sum(LEFT_PREP), back, FINGER_CLOSED, "hold the lid there", rot)]
    for dt, lip, tip, sq, label in PATH:
        pos, r = _pose("right", lip, tip, SQUEEZE if sq else 0.0)
        out.append(_w(dt, pos, FINGER_CLOSED, label, r))
    return out


def _left() -> list[Waypoint]:
    side = np.array([0.0, OUT, 0.0])
    grip, rot = _pose("left", LIP0, 0.0)
    out = _lead_in("left", grip + side, rot, "level, beside the handle")
    out += [_w(sum(RIGHT_PREP), grip + side, FINGER_OPEN, "wait while the lid is set ajar", rot),
            _w(LEFT_PREP[0], grip, FINGER_OPEN, "slide in round the handle", rot),
            _w(LEFT_PREP[1], grip, FINGER_CLOSED, "grip the handle", rot)]
    for dt, lip, tip, sq, label in PATH:
        pos, r = _pose("left", lip, tip - (SAG if sq else 0.0), SQUEEZE if sq else 0.0)
        out.append(_w(dt, pos, FINGER_CLOSED, label, r))
    return out


def demo():
    return {"left": _left(), "right": _right()}
