"""On the station table, a small wall cabinet (an open-fronted shelf box) to be hung on a French cleat. At the back of the
table stands a wall with a cleat screwed to it: a strip whose top edge is bevelled at 45 degrees, sloping down toward
the wall. The cabinet stands on the table in front of it, with the matching cleat along the top of its back, its bevel
facing down toward the wall.

Task: hang the cabinet on the cleat. Lift it by its two side panels, carry it level to the wall with its cleat above the
wall's, and lower it so that its cleat drops behind the wall cleat's edge and the two bevels wedge it against the wall;
then let go. The cabinet is 32 cm wide and has nothing to hold in its middle: held by one side it swings down, and its
cleat must come down on the wall cleat along its whole length at once. Both hands carry it level and hook it on
together, the way a carpenter hangs a wall cabinet.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- wall: fixed, its face at x 0.44, 60 cm wide and 15 cm tall (a section of wall).
- wall cleat: fixed, pine, 40 cm long (y), 1.5 cm thick, 3 cm tall, its sharp top edge 11 cm above the table, the top
  bevelled 45 degrees down toward the wall.
- cabinet: loose, pine, 32.4 cm wide (y), 11.5 cm deep (x) and 12 cm tall, 0.8 kg: a back panel, two side panels, a
  bottom, a 1.5 cm cleat along the top of its back (bevelled to match) and a 1.5 cm spacer along the bottom of its back,
  so that it hangs plumb. It starts upright on the table, centred at x 0.30 on the centre line, its back to the wall.

Full score: the cabinet hung on the cleat (its back within 3 mm of the cleat's face, its cleat down on the wall cleat
within 3 mm, level within 1.5 degrees), both hands clear (5 cm or more from the cabinet), still for 1 s. The cabinet
tipping over (more than 30 degrees) scores 0 for the rest of the run.
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, allow_matrix_grips, as_numpy, on_reset, on_step, plate  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
# The file keeps the best stage reached itself and a broken rule latches 0, so the score is read at the end of the run.
SCORE_OVER = "end"

# -- layout (world frame) -----------------------------------------------------------------------------------------------
WALL_X = 0.44
T_CL = 0.015  # cleat thickness (and the bevel's depth)
ZC = TOP + 0.11  # the wall cleat's sharp top edge (room side)
CLEAT_LEN, WALL_CLEAT_H = 0.40, 0.030
CAB_W, CAB_D, CAB_H = 0.30, 0.10, 0.120  # inside width (y); depth (x) from the back panel's face; height
PANEL, BACK, CAB_CLEAT_H = 0.012, 0.006, 0.030  # side/bottom panel, back panel; the cabinet cleat's height above ZC
CAB_KG = 0.8
WOOD_MU = (0.5, 0.45)
PINE_C, WALL_C, BACK_C = (0.80, 0.66, 0.44), (0.86, 0.85, 0.82), (0.70, 0.56, 0.36)

# The cabinet's body origin: the middle of its back panel's face (x = WALL_X - T_CL when hung), mid-height, centre line.
HUNG_TOP = ZC + CAB_CLEAT_H  # the cabinet's top when hung
HUNG_POS = np.array([WALL_X - T_CL, 0.0, HUNG_TOP - CAB_H / 2])


def _cabinet_shapes() -> list:
    """In the body frame: x toward the wall, y along the width, z up; origin at the back panel's face, mid-height."""
    half_h = CAB_H / 2
    out = [BoxShape((-BACK / 2, 0.0, 0.0), (BACK, CAB_W + 2 * PANEL, CAB_H), BACK_C)]  # back panel
    for s in (1, -1):  # side panels, from the back panel's face to the front
        out.append(BoxShape((-CAB_D / 2, s * (CAB_W / 2 + PANEL / 2), 0.0), (CAB_D, PANEL, CAB_H), PINE_C))
    out.append(BoxShape((-CAB_D / 2, 0.0, -half_h + PANEL / 2), (CAB_D, CAB_W, PANEL), PINE_C))  # bottom
    # Cleat on the back, behind the panel (x 0..T_CL), from its bevel up to the cabinet's top. Its bevel runs from the
    # back panel's face (x 0) at the height of the wall cleat's edge down to the wall (x T_CL), so that hung, the two
    # bevels meet along one line; the face is a thin plate whose lower face lies on that line.
    zc_rel = ZC - HUNG_POS[2]
    n = np.array([1.0, 0.0, 1.0]) / math.sqrt(2.0)  # from the wall cleat into the cabinet cleat
    out.append(BoxShape((T_CL / 2, 0.0, (zc_rel + half_h) / 2), (T_CL, CLEAT_LEN - 0.08, half_h - zc_rel), PINE_C))
    out.append(plate(np.array([0.0, 0.0, zc_rel]) + n * 0.001, np.array([T_CL, 0.0, zc_rel - T_CL]) + n * 0.001,
                     CLEAT_LEN - 0.08, 0.002, n, PINE_C))
    out.append(BoxShape((T_CL / 2, 0.0, -half_h + 0.0075), (T_CL, CLEAT_LEN - 0.08, 0.015), PINE_C))  # spacer
    return out


def _wall() -> list:
    wall = Box("wall", pos=(WALL_X + 0.01, 0.0, TOP + 0.075), size=(0.02, 0.60, 0.15), color=WALL_C)
    return [wall]


_N = np.array([1.0, 0.0, 1.0]) / math.sqrt(2.0)
WALL_CLEAT = Body.build(
    "wall_cleat", pos=(WALL_X - T_CL / 2, 0.0, ZC - T_CL), mass=1.0, fixed=True, friction=WOOD_MU, color=PINE_C,
    contact_offset=0.001,
    shapes=[BoxShape((0.0, 0.0, -(WALL_CLEAT_H - T_CL) / 2), (T_CL, CLEAT_LEN, WALL_CLEAT_H - T_CL), PINE_C),  # body
            BoxShape((-T_CL / 2 + 0.001, 0.0, (T_CL - 0.0025) / 2), (0.002, CLEAT_LEN, T_CL - 0.0025), PINE_C),  # room face,
            # stopping just under the bevel so that its corner does not stand proud of it
            plate(np.array([-T_CL / 2, 0.0, T_CL]) - _N * 0.001, np.array([T_CL / 2, 0.0, 0.0]) - _N * 0.001, CLEAT_LEN,
                  0.002, _N, PINE_C)])  # the bevel's face (its upper face on the line), from the edge down to the wall

START = np.array([0.30 + CAB_D / 2, 0.0, TOP + CAB_H / 2])  # body origin (back face) when standing on the table
CABINET = Body.build("cabinet", pos=tuple(START + [0.0, 0.0, 0.0005]), mass=CAB_KG, friction=WOOD_MU, color=PINE_C,
                     damping=(0.0, 0.3), contact_offset=0.001, solver_iterations=24, shapes=_cabinet_shapes())
OBJECTS = [*_wall(), WALL_CLEAT, CABINET]

# -- scoring ------------------------------------------------------------------------------------------------------------
HUNG_MM, LEVEL_DEG = 3.0, 1.5
CLEAR_HANDS = 0.05
STILL_STEPS, STILL_MM = 20, 1.0
_DEBUG = os.environ.get("CLEAT_DEBUG") == "1"
_DEBUG_EVERY = int(os.environ.get("CLEAT_DEBUG_EVERY", "10"))
_EP: dict = {}
_TCP: dict = {}
_ARM_IDS: dict = {}


@on_reset
def _reset() -> None:
    _EP.update(step=0, tick=0, counted=-1, best=0.0, failed="", recent=[])
    _TCP.clear()


_reset()


@on_step
def _track_grippers(env) -> None:
    """Each gripper's centre (TCP) from its arm's joint positions: "let go" needs it."""
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS

    _EP["tick"] += 1  # one control step: success() and score() both tally it, the history takes it once
    base = getattr(env, "unwrapped", env)
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        if side not in _ARM_IDS:
            _ARM_IDS[side] = list(robot.find_joints(ARM_JOINTS, preserve_order=True)[0])
        q = as_numpy(robot.data.joint_pos)[0]
        _TCP[side] = np.asarray(ik.tcp_pose(q[_ARM_IDS[side]], station.BASES[side])[0], float)


def _tally(o) -> dict:
    c = o["cabinet"]
    r = c.rotation()
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, r[2, 2]))))
    d = c.pos - HUNG_POS
    lowest = c.low[2]
    t = {"tilt": tilt, "dx": d[0], "dy": d[1], "dz": d[2], "lowest": lowest - TOP}
    if tilt > 30.0:
        _EP["failed"] = _EP["failed"] or "the cabinet tipped over"
    lifted = lowest > TOP + 0.01 and tilt < 5.0
    above = abs(d[0]) < 0.010 and 0.0 < d[2] < 0.040 and abs(d[1]) < 0.03 and tilt < 5.0  # cleat over the wall cleat
    hung = abs(d[0]) < HUNG_MM / 1000 and abs(d[2]) < HUNG_MM / 1000 and abs(d[1]) < 0.03 and tilt < LEVEL_DEG
    stage = 0.75 if hung else 0.5 if above else 0.25 if lifted else 0.0
    if not _EP["failed"]:
        _EP["best"] = max(_EP["best"], stage)
    recent = _EP["recent"]
    if _EP["counted"] != _EP["tick"]:
        _EP["counted"] = _EP["tick"]
        recent.append(np.array(c.pos, float))
        del recent[:-STILL_STEPS]
    still = len(recent) == STILL_STEPS and all(np.abs(p - recent[-1]).max() * 1000 < STILL_MM for p in recent)
    lo, hi = np.array([-CAB_D, -(CAB_W / 2 + PANEL), -CAB_H / 2]), np.array([T_CL, CAB_W / 2 + PANEL, CAB_H / 2])
    gaps = []
    for q in _TCP.values():
        p = r.T @ (q - c.pos)
        gaps.append(float(np.linalg.norm(np.maximum(np.maximum(lo - p, p - hi), 0.0))))
    t["hands"] = min(gaps) if len(_TCP) == 2 else 0.0
    t.update(stage=stage, still=still, done=bool(hung and t["hands"] >= CLEAR_HANDS and still))
    return t


def _debug(o, t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    print(f"DBG step={_EP['step']} cabinet d=({t['dx'] * 1000:+.1f},{t['dy'] * 1000:+.1f},{t['dz'] * 1000:+.1f})mm "
          f"tilt={t['tilt']:.2f} lowest={t['lowest'] * 1000:.1f}mm hands={t['hands']:.3f} still={t['still']} "
          f"stage={t['stage']} best={_EP['best']} failed={_EP['failed'] or '-'}", flush=True)
    if len(_TCP) == 2:
        print("DBG   " + " | ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{v[2] - TOP:.3f})" for k, v in _TCP.items()),
              flush=True)


def score(objects) -> float:
    """The best stage reached (0 for the rest of the run once the cabinet tips over more than 30 degrees): 0.25 the
    cabinet lifted off the table, level within 5 degrees; 0.5 its cleat right above the wall cleat (within 1 cm of the
    hung position front to back, at most 4 cm above it); 0.75 hung on the cleat (back and cleat within 3 mm, level
    within 1.5 degrees). Full score (1.0) is success()."""
    t = _tally(objects)
    if _DEBUG:
        _debug(objects, t)
    return 0.0 if _EP["failed"] else _EP["best"]


def success(objects) -> bool:
    """The cabinet hung on the cleat, level, both hands clear, still for 1 s."""
    t = _tally(objects)
    return bool(t["done"] and not _EP["failed"])


# -- scripted demo ------------------------------------------------------------------------------------------------------
# Each gripper comes down over the top edge of its side panel, above the cabinet's centre of mass, fingers either side
# of the panel, and
# clamps it. Together they lift the cabinet until its cleat clears the wall cleat, carry it level to the wall, lower it
# so that its cleat drops behind the wall cleat's edge onto the bevel, let go and back away.

DOWN = {"left": grip_rotation(0.0), "right": grip_rotation(0.0)}  # fingers closing along y, on the side panels
OPEN, CLAMP = -0.017, float(os.environ.get("CLEAT_CLAMP", str(-(PANEL / 2 - 0.003))))
GRIP_X = -0.028  # over the cabinet's centre of mass (body frame x), so that it hangs level between the grips
GRIP_DZ = float(os.environ.get("CLEAT_GRIP_DZ", "-0.006"))  # TCP this far below the panel's top edge
SAG = float(os.environ.get("CLEAT_SAG", "0.008"))
LIFT_CLEAR = float(os.environ.get("CLEAT_LIFT_CLEAR", "0.012"))  # the cabinet cleat's tip this far above the edge
SET_DZ = float(os.environ.get("CLEAT_SET_DZ", "-0.010"))  # aim this far below the hung height when lowering


def _grip(arm: str, origin) -> np.ndarray:
    s = 1.0 if arm == "left" else -1.0
    return np.asarray(origin, float) + np.array([GRIP_X, s * (CAB_W / 2 + PANEL / 2), CAB_H / 2 + GRIP_DZ])


def _good_ik(arm: str, pos, rot):
    from yam_arena import ik, station

    best = None
    for sd in itertools.product((-0.4, 0.0, 0.4), (0.8, 1.3), (0.6, 1.2), (-0.8, 0.8), (-1.2, 0.0, 1.2),
                                (-1.5, 0.0, 1.5)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.asarray(sd, float),
                             base_pos=station.BASES[arm], iters=150)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} arm at {pos}")
    return best[0]


def _lead_in(arm: str, pos, rot, finger: float, label: str, seconds: float = 3.0, steps: int = 15) -> list[Waypoint]:
    """From READY along a straight line in joint space (the wrist's big turn stays on one IK branch)."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, station.BASES[arm])
        out.append(Waypoint(seconds / steps, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, p, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in p), finger, label, rot)


def _arm(arm: str) -> list[Waypoint]:
    rot = DOWN[arm]
    side = 1.0 if arm == "left" else -1.0
    # The cabinet cleat's lowest tip sits at ZC - T_CL when hung; carry it LIFT_CLEAR above the wall cleat's edge (ZC).
    carry_z = HUNG_POS[2] + T_CL + LIFT_CLEAR
    start = _grip(arm, START)
    lifted = _grip(arm, [START[0], 0.0, carry_z]) + [0.0, 0.0, SAG]
    over = _grip(arm, [HUNG_POS[0], 0.0, carry_z]) + [0.0, 0.0, SAG]
    hung = _grip(arm, [HUNG_POS[0], 0.0, HUNG_POS[2] + SET_DZ]) + [0.0, 0.0, SAG]
    out = _lead_in(arm, start + [0.0, 0.0, 0.05], rot, OPEN, "above its side panel, fingers either side of it")
    out += [_w(1.5, start, OPEN, "down over the panel's top edge", rot),
            _w(1.0, start, CLAMP, "clamp the panel", rot),
            _w(2.5, lifted, CLAMP, "lift the cabinet until its cleat clears the wall cleat", rot),
            _w(3.0, over, CLAMP, "carry it level to the wall", rot),
            _w(0.8, over, CLAMP, "steady", rot),
            _w(2.5, hung, CLAMP, "lower it: its cleat drops behind the wall cleat's edge", rot),
            _w(1.0, hung, CLAMP, "settle", rot),
            _w(1.0, hung, OPEN, "let go", rot),
            _w(1.5, hung + [-0.03, side * 0.06, 0.04], OPEN, "back away, out to the side", rot),
            _w(3.5, hung + [-0.03, side * 0.06, 0.04], OPEN, "wait", rot)]
    return out


def _idle(arm: str, seconds: float) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return [Waypoint(seconds, tuple(float(v) for v in pos), OPEN, "stay put", rot)]


def demo():
    """CLEAT_ONE_ARM=1 keeps the right arm still: lifted by one side panel, the cabinet swings down at the other side."""
    if os.environ.get("CLEAT_ONE_ARM") == "1":
        return {"left": _arm("left"), "right": _idle("right", 22.0)}
    return {"left": _arm("left"), "right": _arm("right")}
