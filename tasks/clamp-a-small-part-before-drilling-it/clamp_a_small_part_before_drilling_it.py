"""On the station table stands a small drill press over its table, which carries a waxed MDF spoil board. A light aluminium bar lies
loose on the plate, its punch mark under the drill's bit; a screw hold-down clamp stands over the bar's right end.

Task: one hand holds the bar in place while the other turns the clamp's knob down until its pad grips the bar. The
pad turns with the screw and drags a loose bar round with it, off the mark, unless the bar is held. Then, with both
hands at least 5 cm from the bit, one hand pushes the drill's feed knob down to drill through the mark, and lets it
back up.

Run it:  inspect-robots run --embodiment yam_arena -E task=clamp_a_small_part_before_drilling_it.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, on_reset,  # noqa: E402
                     on_step, rotation, set_slide_target, set_slider_drive, set_thread_on, slider, thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the drill table's spoil board: waxed melamine-faced MDF (slick: the bar slides and turns on it easily) ----------
PX, PY = (0.23, 0.33), (-0.15, 0.11)
PT = TOP + 0.010  # its top
DRILL_XY = np.array([0.28, 0.0])  # the bit's axis

# -- the bar: aluminium, its punch mark under the bit; origin at its middle ------------------------------------------
BAR_L, BAR_W, BAR_T = 0.20, 0.020, 0.012
BAR_Y = (-0.12, 0.08)
BAR_MASS = 0.12
BAR_TOP = PT + BAR_T
MARK_IN_BAR = np.array([0.0, DRILL_XY[1] - (BAR_Y[0] + BAR_Y[1]) / 2])  # the mark, from the bar's middle

# -- the clamp: a post behind the bar with an arm over its right end; a screw through the arm, a knob on top and a pad
# underneath. Turning the knob clockwise (from above) drives the screw down PITCH per turn. Origin of the screw: the
# middle of its pad's underside
CLAMP_XY = np.array([DRILL_XY[0], -0.10])
PAD_R, PAD_H = 0.010, 0.004
GAP0 = 0.004  # the pad starts this far above the bar
SCREW0 = np.array([*CLAMP_XY, BAR_TOP + GAP0])
ARM_Z = (0.024, 0.034)  # the arm, in the screw's starting frame (from the pad's underside)
KNOB_W, KNOB_Z = 0.016, (0.044, 0.066)  # a square knob, turned 45 degrees on the screw
PITCH = 0.008  # m per turn: a quick thread
TRAVEL = 0.014
SCREW_K, SCREW_D, SCREW_MAX = 3000.0, 20.0, 30.0
THREAD_DRAG = 0.01  # N m s/rad
TIGHT_N = 6.0  # the pad pressing on the bar with at least this force, i.e. the knob turned a quarter turn (2 mm)
# past the pad meeting the bar: clamped (the soft simulated spring presses 6 N there; a real hand-tight clamp ~100 N)

# -- the drill: a column behind, a head over the bit, and the quill (bit, chuck, spindle, feed knob) sliding down in the
# head on a return spring. Origin of the quill: the bit's tip
TIP0 = PT + BAR_T + 0.008  # the bit's tip at rest, 8 mm above the bar
BIT_R, BIT_L = 0.0025, 0.025
CHUCK_R, CHUCK_L = 0.008, 0.015
SPINDLE_R = 0.006
FEED_KNOB_R, FEED_KNOB_H = 0.015, 0.010
FEED_TOP = 0.095  # the feed knob's top, above the tip
HEAD_Z = (TOP + 0.072, TOP + 0.088)
FEED_TRAVEL = 0.024
FEED_K = 150.0  # N/m, the return spring
QUILL_MASS = 0.1
RUNNING = 0.003  # the quill this far down: the motor runs and the bit spins
DRILL_DEPTH = 0.0135  # the tip this far below the bar's top: through the 12 mm bar, 1.5 mm into the spoil board

STEEL, ALU, DARK, RED, BLACK, BRASS = (0.55, 0.57, 0.60), (0.80, 0.81, 0.83), (0.20, 0.21, 0.23), \
    (0.85, 0.12, 0.10), (0.05, 0.05, 0.05), (0.75, 0.62, 0.30)
BOARD = (0.86, 0.85, 0.80)  # white melamine

# -- scoring ----------------------------------------------------------------------------------------------------------
SAFE = 0.05  # every gripper's TCP at least this far from the bit while it spins
ON_MARK = 0.005  # the hole (or, before drilling, the mark) within this of where it should be
FLING = (0.6, 25.0)  # an unclamped bar the bit catches: flung at this speed (m/s) and spin (rad/s)
SUCCESS_HOLD_S = 1.0


def _plate() -> Body:
    # a waxed melamine-faced MDF spoil board on the drill press's table: the bit drills through the bar into it
    shapes = [BoxShape(((PX[0] + PX[1]) / 2, (PY[0] + PY[1]) / 2, 0.005), (PX[1] - PX[0], PY[1] - PY[0], 0.010), BOARD),
              BoxShape(((PX[0] + PX[1]) / 2, (PY[0] + PY[1]) / 2, 0.0003), (PX[1] - PX[0] + 0.03, PY[1] - PY[0] + 0.03,
                                                                            0.0006), DARK, collide=False)]  # the table
    return Body.build("plate", pos=(0.0, 0.0, TOP), mass=1.0, shapes=shapes, kinematic=True, friction=(0.05, 0.05),
                      slick=True, color=BOARD)


def _bar() -> Body:
    m = MARK_IN_BAR
    shapes = [BoxShape((0.0, 0.0, 0.0), (BAR_W, BAR_L, BAR_T), ALU),
              BoxShape((m[0], m[1], BAR_T / 2 + 0.0002), (0.010, 0.0012, 0.0004), BLACK, collide=False),  # the mark
              BoxShape((m[0], m[1], BAR_T / 2 + 0.0002), (0.0012, 0.010, 0.0004), BLACK, collide=False)]
    return Body.build("bar", pos=(DRILL_XY[0], (BAR_Y[0] + BAR_Y[1]) / 2, PT + BAR_T / 2), mass=BAR_MASS, shapes=shapes,
                      friction=(1.0, 0.9), color=ALU)


def _clamp() -> Body:
    """The clamp's post and arm (fixed). Origin at the screw's starting origin."""
    post_x = 0.035  # the post, behind the bar
    shapes = [BoxShape((post_x, 0.0, (PT - SCREW0[2] + ARM_Z[1]) / 2), (0.014, 0.016, ARM_Z[1] - (PT - SCREW0[2])),
                       DARK),
              BoxShape((post_x / 2 - 0.004, 0.0, (ARM_Z[0] + ARM_Z[1]) / 2), (post_x + 0.022, 0.016,
                                                                               ARM_Z[1] - ARM_Z[0]), DARK)]
    return Body.build("clamp", pos=tuple(SCREW0.tolist()), mass=1.0, shapes=shapes, kinematic=True, color=DARK)


def _screw() -> Body:
    shapes = [CylShape((0.0, 0.0, PAD_H / 2), PAD_R, PAD_H, BLACK),
              CylShape((0.0, 0.0, (PAD_H + KNOB_Z[0]) / 2), 0.004, KNOB_Z[0] - PAD_H, STEEL),
              BoxShape((0.0, 0.0, (KNOB_Z[0] + KNOB_Z[1]) / 2), (KNOB_W, KNOB_W, KNOB_Z[1] - KNOB_Z[0]), RED,
                       yaw=math.pi / 4)]
    return Body.build("screw", pos=tuple(SCREW0.tolist()), mass=0.03, shapes=shapes, friction=(1.2, 1.1), color=STEEL,
                      com=(0.0, 0.0, 0.03))


def _drill() -> Body:
    """The drill press's column and head (fixed), behind the bit. Origin at the bit's axis on the table."""
    cx = 0.10  # the column, behind the bit
    shapes = [BoxShape((cx, 0.0, 0.010), (0.06, 0.07, 0.020), DARK),
              BoxShape((cx, 0.0, (HEAD_Z[1] - TOP + 0.02) / 2), (0.03, 0.03, HEAD_Z[1] - TOP + 0.02), DARK),
              BoxShape((cx / 2, 0.0, (HEAD_Z[0] + HEAD_Z[1]) / 2 - TOP), (cx + 0.04, 0.04, HEAD_Z[1] - HEAD_Z[0]),
                       DARK)]
    return Body.build("drill", pos=(*DRILL_XY, TOP), mass=5.0, shapes=shapes, kinematic=True, color=DARK)


def _quill() -> Body:
    shapes = [CylShape((0.0, 0.0, BIT_L / 2), BIT_R, BIT_L, STEEL, collide=False),  # the bit
              CylShape((0.0, 0.0, BIT_L + CHUCK_L / 2), CHUCK_R, CHUCK_L, STEEL),
              CylShape((0.0, 0.0, (BIT_L + CHUCK_L + FEED_TOP - FEED_KNOB_H) / 2), SPINDLE_R,
                       FEED_TOP - FEED_KNOB_H - BIT_L - CHUCK_L, STEEL),
              CylShape((0.0, 0.0, FEED_TOP - FEED_KNOB_H / 2), FEED_KNOB_R, FEED_KNOB_H, BLACK)]
    return Body.build("quill", pos=(*DRILL_XY, TIP0), mass=QUILL_MASS, shapes=shapes, friction=(1.0, 0.9),
                      color=STEEL, com=(0.0, 0.0, 0.05))


def _marker(name: str, shape, at) -> Body:
    """A small fixed marker the task file moves into view (a hole drilled; the motor's running light)."""
    return Body.build(name, pos=tuple(float(v) for v in at), mass=0.01, shapes=[shape], kinematic=True,
                      color=shape.color)


HIDE_HOLE = np.array([DRILL_XY[0], 0.05, TOP - 0.02])  # under the table
LAMP_ON = np.array([DRILL_XY[0] - 0.021, 0.0, (HEAD_Z[0] + HEAD_Z[1]) / 2])  # on the head's front
LAMP_OFF = LAMP_ON + [0.008, 0.0, 0.0]  # inside the head
OBJECTS = [_plate(), _bar(), _clamp(), _screw(), _drill(), _quill(),
           _marker("hole", CylShape((0.0, 0.0, 0.0), 0.003, 0.0006, BLACK, collide=False), HIDE_HOLE),
           _marker("motor_light", SphereShape((0.0, 0.0, 0.0), 0.004, RED, collide=False), LAMP_OFF)]
thread("clamp_screw", "clamp", "screw", anchor0=(0.0, 0.0, 0.0), axis0=(0.0, 0.0, -1.0), anchor1=(0.0, 0.0, 0.0),
       axis1=(0.0, 0.0, -1.0), slide_limits=(0.0, TRAVEL), turn_damping=THREAD_DRAG, slide_stiffness=SCREW_K,
       slide_damping=SCREW_D, slide_max_force=SCREW_MAX, always_on=True)
slider("feed", "drill", "quill", anchor=(*DRILL_XY, TIP0), axis="Z", limits=(-FEED_TRAVEL, 0.0), stiffness=FEED_K,
       damping=5.0, target=0.008, max_force=50.0)  # the target past the top stop: it rests up against it


# -- followed after every control step: the clamp's thread, the drill ----------------------------------------------

def _fresh() -> dict:
    return {"phi": None, "turns": 0.0, "tcps": None, "unsafe": False, "drilled": None, "caught": False,
            "hole_ok": False, "lamp": False, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("DRILL_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    set_thread_on("clamp_screw", True)
    set_slide_target("clamp_screw", 0.0)


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _pose(base, origin, name) -> tuple[np.ndarray, np.ndarray]:
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _teleport(base, origin, name, at) -> None:
    import torch

    asset = base.scene[name]
    p = np.asarray(at, float) + origin
    asset.write_root_pose_to_sim(torch.tensor([[*p.tolist(), 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32,
                                              device=asset.device))


def _fling(base, b_rot) -> None:
    import torch

    asset = base.scene["bar"]
    v = np.array([-FLING[0], 0.0, 0.05])
    asset.write_root_velocity_to_sim(torch.tensor([[*v.tolist(), 0.0, 0.0, FLING[1]]], dtype=torch.float32,
                                                  device=asset.device))


def _dist_to_bit(p, tip) -> float:
    """From a point to the bit (the segment from its tip up its length)."""
    v = np.asarray(p, float) - tip
    h = min(max(float(v[2]), 0.0), BIT_L)
    return float(np.linalg.norm(v - np.array([0.0, 0.0, h])))


def _state(b_pos, b_rot, s_pos, s_rot, q_pos, target) -> dict:
    """The clamp and the drill as they are."""
    slide = float(SCREW0[2] - s_pos[2])  # how far the screw has come down
    force = min(SCREW_K * max(target - slide, 0.0), SCREW_MAX)
    pad = b_rot.T @ (s_pos - b_pos)  # the pad's underside, in the bar's frame
    on_bar = abs(pad[0]) < BAR_W / 2 + 0.002 and abs(pad[1]) < BAR_L / 2 and abs(pad[2] - BAR_T / 2) < 0.002
    clamped = on_bar and force >= TIGHT_N
    mark = b_pos + b_rot @ np.array([MARK_IN_BAR[0], MARK_IN_BAR[1], BAR_T / 2])
    down = float(TIP0 - q_pos[2])
    return {"slide": slide, "force": force, "on_bar": on_bar, "clamped": clamped, "down": down,
            "running": down >= RUNNING, "mark_off": float(np.linalg.norm(mark[:2] - DRILL_XY)),
            "flat": float(b_rot[2, 2]) > math.cos(math.radians(10.0))}


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    b_pos, b_rot = _pose(base, origin, "bar")
    s_pos, s_rot = _pose(base, origin, "screw")
    q_pos, _ = _pose(base, origin, "quill")
    _EP["tcps"] = _tcps(base, origin)
    # the thread: the screw comes down PITCH per clockwise turn (seen from above)
    phi = math.atan2(float(s_rot[1, 0]), float(s_rot[0, 0]))
    if _EP["phi"] is not None:
        _EP["turns"] -= ((phi - _EP["phi"] + math.pi) % (2 * math.pi) - math.pi) / (2 * math.pi)
    _EP["phi"] = phi
    target = min(max(PITCH * _EP["turns"], 0.0), TRAVEL)
    set_slide_target("clamp_screw", target)
    st = _state(b_pos, b_rot, s_pos, s_rot, q_pos, target)
    _EP["s"] = st
    # the drill: the motor runs while the quill is down; a hand near the spinning bit is unsafe
    if st["running"] != _EP["lamp"]:
        _EP["lamp"] = st["running"]
        _teleport(base, origin, "motor_light", LAMP_ON if st["running"] else LAMP_OFF)
    if st["running"] and any(_dist_to_bit(p, q_pos) < SAFE for p in _EP["tcps"]):
        _EP["unsafe"] = True
    if _EP["drilled"] is None and not _EP["caught"]:
        tip = b_rot.T @ (q_pos - b_pos)  # the bit's tip in the bar's frame
        if (abs(tip[0]) < BAR_W / 2 and abs(tip[1]) < BAR_L / 2 and tip[2] <= BAR_T / 2 - DRILL_DEPTH
                and tip[2] > -BAR_T / 2 - 0.02):
            if st["clamped"]:
                _EP["drilled"] = np.array(tip[:2])
                _EP["hole_ok"] = float(np.linalg.norm(tip[:2] - MARK_IN_BAR)) <= ON_MARK
                _teleport(base, origin, "hole", b_pos + b_rot @ np.array([tip[0], tip[1], BAR_T / 2 + 0.0003]))
            else:  # the bit catches a loose bar and flings it
                _EP["caught"] = True
                _fling(base, b_rot)
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = _EP["tcps"]
            sp = s_pos + [0.0, 0.0, (KNOB_Z[0] + KNOB_Z[1]) / 2]
            print(f"[drill] t={_DEBUG['steps'] / 20:5.1f}s turns={_EP['turns']:5.2f} slide={st['slide'] * 1000:5.1f}mm "
                  f"target={target * 1000:5.1f}mm force={st['force']:5.1f}N on_bar={st['on_bar']} "
                  f"clamped={st['clamped']} bar={np.round(b_pos * 1000).astype(int).tolist()} "
                  f"bar_yaw={math.degrees(math.atan2(b_rot[1, 0], b_rot[0, 0])):6.1f} mark_off={st['mark_off'] * 1000:5.1f}mm "
                  f"down={st['down'] * 1000:5.1f}mm running={st['running']} unsafe={_EP['unsafe']} "
                  f"drilled={None if _EP['drilled'] is None else np.round(_EP['drilled'] * 1000, 1).tolist()} "
                  f"hole_ok={_EP['hole_ok']} caught={_EP['caught']} "
                  f"bit_dist=({_dist_to_bit(tl, q_pos) * 1000:.0f},{_dist_to_bit(tr, q_pos) * 1000:.0f})mm "
                  f"rightTCP-knob={np.round((tr - sp) * 1000).astype(int).tolist()} "
                  f"leftTCP-bar={np.round((tl - b_pos) * 1000).astype(int).tolist()} score={_points(_status())}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    st = _EP["s"]
    return {"drilled_ok": _EP["drilled"] is not None and _EP["hole_ok"], "unsafe": _EP["unsafe"],
            "caught": _EP["caught"], "clamped": bool(st.get("clamped")),
            "in_place": bool(st.get("clamped")) and st.get("mark_off", 1.0) <= ON_MARK and bool(st.get("flat")),
            "quill_up": st.get("down", 0.0) < RUNNING}


def _points(s) -> float:
    if s["caught"]:
        return 0.0
    if not s["unsafe"]:
        if s["drilled_ok"] and s["clamped"] and s["quill_up"]:
            return 1.0
        if s["drilled_ok"] or s["in_place"]:
            return 0.5
    return 0.25 if s["clamped"] or s["drilled_ok"] else 0.0


def score(objects) -> float:
    """Clamped: the pad on the bar pressing at least 6 N (the knob a quarter turn past contact). Drilled: the bit's tip
    13.5 mm below the bar's top, through the 12 mm bar into the spoil board.
    1.0 with a hole drilled through the bar within 5 mm of its mark while it was clamped, no gripper's TCP within
    5 cm of the bit while it spun, the bar still clamped and the drill back up; 0.5 drilled so (safely) but the drill
    still down or the clamp loosened, or the bar clamped with its mark under the bit (within 5 mm) but not drilled yet;
    0.25 the bar clamped but off its mark, or clamped or drilled with a hand near the spinning bit; 0.0 otherwise, or
    if the bit caught a loose bar."""
    return _points(_status())


def success(objects) -> bool:
    """Full score with the bar, the drill's quill and the clamp's screw still: the trial ends once it has held 1 s."""
    return score(objects) >= 1.0 and all(objects[n].resting() for n in ("bar", "quill", "screw"))


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the bar's left part, fingers closed, and presses it onto the plate, holding it.
# The right hand comes straight down on the clamp's knob, fingers closing across it, and turns it clockwise in strokes
# (grip, roll the wrist a quarter turn, let go, roll back, grip again) until the pad is tight on the bar. Then the left
# lets go and moves well away, and the right rises, goes over the drill's feed knob and pushes it straight down,
# fingers closed, drilling through the mark, then lets it back up.

PRESS_L = grip_rotation(0.0)  # straight down, fingers closed: pressing the bar
L_PRESS = 0.008  # the left aims this far into the bar: the arm gives, pressing it down with a few newtons
PRESS_Y = 0.065
KNOB_IN_HAND = KNOB_Z[1] - 0.006  # the right TCP this far above the screw's origin: 6 mm below the knob's top
STROKE = math.radians(90.0)
STROKES = 4
TURN_S, RELEASE_S, BACK_S, REGRIP_S = 1.6, 0.4, 1.0, 0.5
PRESS = 0.004  # aimed this far into what a closed hand presses on
LEAD_S, LEAD_STEPS = 3.0, 15


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


KNOB_GRIP = _rz(math.pi / 4) @ grip_rotation(0.0)  # straight down, fingers closing across the knob's flats (at 45
# degrees): each quarter turn rolls the wrist from +45 to -45 degrees, inside its range


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4), (0.0,)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _knob_at(turned: float) -> np.ndarray:
    """The right TCP on the knob after the screw has turned ``turned`` (rad); it stops on the bar."""
    down = min(PITCH * turned / (2 * math.pi), GAP0)
    return SCREW0 + [0.0, 0.0, KNOB_IN_HAND - down]


def _right_clamp() -> list[Waypoint]:
    out = _lead_in("right", _knob_at(0.0) + [0.0, 0.0, 0.04], KNOB_GRIP, "over the clamp's knob", FINGER_OPEN)
    out += [_w(1.0, _knob_at(0.0), FINGER_OPEN, "down round the knob", KNOB_GRIP),
            _w(0.6, _knob_at(0.0), FINGER_CLOSED, "grip it", KNOB_GRIP)]
    turned = 0.0
    for k in range(STROKES):
        for j in range(1, 5):  # a quarter of the stroke at a time
            out.append(_w(TURN_S / 4, _knob_at(turned + STROKE * j / 4), FINGER_CLOSED, f"turn it clockwise ({k + 1})",
                          _rz(-STROKE * j / 4) @ KNOB_GRIP))
        turned += STROKE
        at = _knob_at(turned)
        out.append(_w(RELEASE_S, at, FINGER_OPEN, "let go", _rz(-STROKE) @ KNOB_GRIP))
        if k < STROKES - 1:
            for j in range(1, 5):
                out.append(_w(BACK_S / 4, at, FINGER_OPEN, "turn back, open", _rz(-STROKE * (1 - j / 4)) @ KNOB_GRIP))
            out.append(_w(REGRIP_S, at, FINGER_CLOSED, "take the knob again", KNOB_GRIP))
    out.append(_w(1.0, _knob_at(turned) + [0.0, 0.0, 0.05], FINGER_OPEN, "rise", _rz(-STROKE) @ KNOB_GRIP))
    return out


def _right_drill(start: float) -> list[Waypoint]:
    top = np.array([*DRILL_XY, TIP0 + FEED_TOP + 0.013])  # the right TCP with its closed fingertips on the feed knob
    rot = grip_rotation(0.0)
    return [_w(max(start, 1.5), top + [0.0, 0.0, 0.015], FINGER_CLOSED, "over the drill's feed knob", rot),
            _w(0.8, top, FINGER_CLOSED, "onto the knob", rot),
            _w(2.5, top - [0.0, 0.0, DRILL_DEPTH + 0.008 + 0.006 + PRESS], FINGER_CLOSED,
               "push it down: the bit drills through the mark", rot),
            _w(0.8, top - [0.0, 0.0, DRILL_DEPTH + 0.008 + 0.006 + PRESS], FINGER_CLOSED, "through", rot),
            _w(1.5, top + [0.0, 0.0, 0.015], FINGER_CLOSED, "let it back up", rot)]


def _left(release_at: float, total: float) -> list[Waypoint]:
    press = np.array([DRILL_XY[0], PRESS_Y, BAR_TOP + 0.013 - L_PRESS])
    away = np.array([0.24, 0.22, BAR_TOP + 0.12])
    out = _lead_in("left", press + [0.0, 0.0, 0.04], PRESS_L, "over the bar's left part", FINGER_CLOSED)
    out += [_w(1.0, press + [0.0, 0.0, L_PRESS], FINGER_CLOSED, "down onto it", PRESS_L),
            _w(0.6, press, FINGER_CLOSED, "press it firmly onto the board: it holds the bar still", PRESS_L)]
    t = sum(w.duration for w in out)
    out += [_w(max(release_at - t, 0.3), press, FINGER_CLOSED, "hold it while the clamp is tightened", PRESS_L),
            _w(0.8, press + [0.0, 0.0, 0.05], FINGER_CLOSED, "rise", PRESS_L),
            _w(1.5, away, FINGER_CLOSED, "well away from the drill", PRESS_L)]
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), away, FINGER_CLOSED,
                  "hold still: the trial ends once the full score has held 1 s", PRESS_L))
    return out


def demo():
    right = _right_clamp()
    clamped_at = sum(w.duration for w in right)
    left_free = clamped_at - 1.0 + 0.8 + 1.5  # the left is away by then
    right += _right_drill(left_free - clamped_at + 0.5)
    total = sum(w.duration for w in right) + 3.0
    right.append(_w(3.0, right[-1].pos, FINGER_CLOSED, "hold still", right[-1].yaw))
    return {"left": _left(clamped_at - 1.0, total), "right": right}
