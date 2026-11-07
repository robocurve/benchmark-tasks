"""On the station table, a tomato plant's stem rises at the back with one truss hanging from it at working height: a
peduncle running across the table with three tomatoes hanging from it on short pedicels, the two nearest the stem red
(ripe) and the one at the far end green. A shallow harvest tray stands on the table under the truss.

Task: pick the ripe tomatoes and put them in the crate, leaving the green ones on the truss. A tomato's pedicel has a
joint, the knuckle (its abscission zone), where it snaps cleanly when the fruit is bent up or twisted past it, leaving
the calyx on the fruit; it does not give when the fruit is pulled along its stem. A picker therefore takes the fruit in
the hand and bends it up and over, toward the truss, until the knuckle snaps; pulling drags the stem and tears the
truss.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- plant: fixed. A stem 1.8 cm thick at (0.37, -0.06), and the truss: a peduncle 5 mm thick from the stem along +y at
  x 0.31, about 16 cm above the table, with a 5 mm pedicel hanging 3 cm from it to each tomato's knuckle.
- tomatoes: loose, 60 g, 4.8 cm across, centred 8.5 cm above the table at x 0.31 and y 0.03 and 0.12 (red) and 0.21
  (green), each with a green calyx and a 12 mm stub of pedicel up to its knuckle. The knuckle lets a tomato swing up to
  35 degrees each way; bent further, it snaps (0.12 N m). Pulled along its stem it holds, and the stem stretches on a
  spring (1,000 N/m).
- tray: fixed, black plastic, 13 x 26 cm inside and 2.5 cm deep, centred at (0.29, 0.10), under the truss.

Score: 0 for the rest of the run if the green tomato comes off the truss, a tomato is pulled 6 mm along its stem (the
truss torn), or a tomato falls onto the table outside the tray. Otherwise 0.25 for each red tomato off the truss and 0.25 more for each one in the tray, at most 0.75; full
score: both red tomatoes in the tray, the green one still on the truss, both grippers 5 cm or more from every tomato,
still for 1 s.
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, Joint, SphereShape, allow_matrix_grips, as_numpy,  # noqa: E402
                     on_reset, on_step, segment)
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
# The file keeps the best stage reached itself and a broken rule latches 0, so the score is read at the end of the run.
SCORE_OVER = "end"

# -- layout (world frame) ---------------------------------------------------------------------------------------------
R_F = 0.024  # tomato radius
FX, FZ = 0.31, TOP + 0.085  # tomato centres
STUB = 0.012  # pedicel stub between the fruit's top and the knuckle
KZ = FZ + R_F + STUB  # the knuckles
FRUIT_Y = (0.03, 0.12, 0.21)
RIPE = (True, True, False)
FRUIT_KG = 0.06
KNUCKLE_DEG = 35.0  # how far a tomato swings on its knuckle before the knuckle carries torque
SNAP_NM = 0.12  # the torque that then snaps it
PULL_N = 200.0  # it never comes off by pulling: the pedicel stretches instead (a spring along it)
STRETCH_K, STRETCH_MAX = 1000.0, 0.012  # N/m, m: pulled 6 mm along its stem (6 N), the truss is torn
TORN_MM = 6.0
RED, GREEN, CALYX = (0.80, 0.12, 0.08), (0.42, 0.66, 0.24), (0.20, 0.45, 0.16)
STEM_C = (0.30, 0.50, 0.20)

STEM_XY = (0.37, -0.06)
PED_Z0, PED_DROP = TOP + 0.165, 0.025  # peduncle height at the stem, and how much it falls across the truss
TRUSS_END = 0.24


def _ped_z(y: float) -> float:
    """The peduncle's height above y: it falls slightly from the stem toward its free end."""
    return PED_Z0 - PED_DROP * (y - STEM_XY[1]) / (TRUSS_END - STEM_XY[1])


def _plant_shapes() -> list:
    """Body frame = world frame shifted to the stem's foot."""
    sx, sy = STEM_XY
    org = np.array([sx, sy, TOP])
    out = [CylShape((0.0, 0.0, 0.13), 0.009, 0.26, STEM_C)]  # stem
    knee = np.array([FX, sy + 0.02, _ped_z(sy + 0.02)])
    out.append(segment(np.array([sx - 0.006, sy, PED_Z0 + 0.004]) - org, knee - org, 0.006, STEM_C))
    out.append(segment(knee - org, np.array([FX, TRUSS_END, _ped_z(TRUSS_END)]) - org, 0.005, STEM_C))
    for y in FRUIT_Y:  # pedicels down to the knuckles, each with a slight swelling at the knuckle
        top = np.array([FX, y, _ped_z(y)])
        out.append(segment(top - org, np.array([FX, y, KZ + 0.002]) - org, 0.005, STEM_C))
        out.append(CylShape(tuple(np.array([FX, y, KZ + 0.003]) - org), 0.0035, 0.004, STEM_C, collide=False))
    return out


PLANT = Body.build("plant", pos=(STEM_XY[0], STEM_XY[1], TOP), mass=1.0, fixed=True, friction=(0.6, 0.5),
                   color=STEM_C, shapes=_plant_shapes())


def _tomato_shapes(color) -> list:
    """Body frame: origin at the fruit's centre; z up, the knuckle at z R_F + STUB."""
    out = [SphereShape((0.0, 0.0, 0.0), R_F, color),
           CylShape((0.0, 0.0, R_F + STUB / 2 - 0.001), 0.0025, STUB + 0.002, CALYX)]
    for k in range(5):  # sepals of the calyx, lying on the fruit's shoulder (visual only)
        a = 2 * math.pi * k / 5
        out.append(BoxShape((0.010 * math.cos(a), 0.010 * math.sin(a), R_F - 0.0025), (0.016, 0.005, 0.0015), CALYX,
                            yaw=a, collide=False))
    return out


TOMATOES = [
    Body.build(f"tomato_{i + 1}", pos=(FX, y, FZ), mass=FRUIT_KG, friction=(0.9, 0.8), color=RED if ripe else GREEN,
               damping=(0.05, 1.5), contact_offset=0.001, solver_iterations=32,
               shapes=_tomato_shapes(RED if ripe else GREEN),
               joint=Joint(anchor=(FX, y, KZ), kind="ball", parent="plant", limits=(KNUCKLE_DEG,),
                           break_torque=SNAP_NM, break_force=PULL_N, child_first=True,
                           axial=(STRETCH_K, 5.0, STRETCH_MAX)))
    for i, (y, ripe) in enumerate(zip(FRUIT_Y, RIPE))
]

CRATE_XY = (0.29, 0.10)
CRATE_IN, CRATE_H, CRATE_T = (0.13, 0.26), 0.025, 0.004
CRATE_C = (0.10, 0.10, 0.11)


def _crate_shapes() -> list:
    lx, ly = CRATE_IN[0] + 2 * CRATE_T, CRATE_IN[1] + 2 * CRATE_T
    out = [BoxShape((0.0, 0.0, CRATE_T / 2), (lx, ly, CRATE_T), CRATE_C)]
    for s in (1, -1):
        out.append(BoxShape((s * (CRATE_IN[0] / 2 + CRATE_T / 2), 0.0, CRATE_H / 2), (CRATE_T, ly, CRATE_H), CRATE_C))
        out.append(BoxShape((0.0, s * (CRATE_IN[1] / 2 + CRATE_T / 2), CRATE_H / 2), (CRATE_IN[0], CRATE_T, CRATE_H),
                            CRATE_C))
    return out


CRATE = Body.build("tray", pos=(CRATE_XY[0], CRATE_XY[1], TOP), mass=1.0, fixed=True, friction=(0.7, 0.6),
                   color=CRATE_C, shapes=_crate_shapes())
OBJECTS = [PLANT, *TOMATOES, CRATE]

# -- scoring ----------------------------------------------------------------------------------------------------------
OFF_MM = 4.0  # a tomato whose knuckle has moved this far from the pedicel's end has come off
CLEAR_HANDS = 0.05
STILL_STEPS, STILL_MM = 20, 1.0
_DEBUG = os.environ.get("TOMATO_DEBUG") == "1"
_DEBUG_EVERY = int(os.environ.get("TOMATO_DEBUG_EVERY", "10"))
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
    crate = o["tray"]
    fruits = []
    for i, (y, ripe) in enumerate(zip(FRUIT_Y, RIPE)):
        f = o[f"tomato_{i + 1}"]
        rf = f.rotation()
        knuckle = f.pos + rf @ np.array([0.0, 0.0, R_F + STUB])
        gap = rf.T @ (np.array([FX, y, KZ]) - knuckle)  # the pedicel's end seen from the fruit's knuckle
        stretch, lateral = float(gap[2]) * 1000, float(math.hypot(gap[0], gap[1])) * 1000
        off = lateral > OFF_MM or abs(stretch) > STRETCH_MAX * 1000 + 2.0
        if not off and stretch > TORN_MM:
            _EP["failed"] = _EP["failed"] or f"tomato_{i + 1} was pulled along its stem, tearing the truss"
        d = f.pos - crate.pos
        in_crate = off and abs(d[0]) < CRATE_IN[0] / 2 and abs(d[1]) < CRATE_IN[1] / 2 and d[2] < CRATE_H + R_F
        on_table = off and not in_crate and f.low[2] < TOP + 0.003
        if off and not ripe:
            _EP["failed"] = _EP["failed"] or f"the green tomato (tomato_{i + 1}) came off the truss"
        if on_table:
            _EP["failed"] = _EP["failed"] or f"tomato_{i + 1} fell onto the table"
        fruits.append(dict(off=off, in_crate=in_crate, ripe=ripe, pos=np.array(f.pos, float)))
    ripe = [f for f in fruits if f["ripe"]]
    n_off, n_in = sum(f["off"] for f in ripe), sum(f["in_crate"] for f in ripe)
    stage = min(0.75, 0.25 * n_off + 0.25 * n_in)
    if not _EP["failed"]:
        _EP["best"] = max(_EP["best"], stage)
    recent = _EP["recent"]
    if _EP["counted"] != _EP["tick"]:
        _EP["counted"] = _EP["tick"]
        recent.append(np.concatenate([f["pos"] for f in ripe]))
        del recent[:-STILL_STEPS]
    still = len(recent) == STILL_STEPS and all(np.abs(p - recent[-1]).max() * 1000 < STILL_MM for p in recent)
    gaps = [float(np.linalg.norm(v - f["pos"])) - R_F for v in _TCP.values() for f in fruits]
    hands = min(gaps) if len(_TCP) == 2 else 0.0
    greens_on = all(not f["off"] for f in fruits if not f["ripe"])
    done = n_in == len(ripe) and greens_on and hands >= CLEAR_HANDS and still
    return dict(fruits=fruits, n_off=n_off, n_in=n_in, stage=stage, still=still, hands=hands, done=bool(done))


def _debug(o, t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    parts = []
    for i, f in enumerate(t["fruits"]):
        p = f["pos"]
        parts.append(f"t{i + 1}=({p[0]:.3f},{p[1]:.3f},{p[2] - TOP:.3f}){'off' if f['off'] else 'on'}"
                     f"{'/crate' if f['in_crate'] else ''}")
    print(f"DBG step={_EP['step']} {' '.join(parts)} hands={t['hands']:.3f} still={t['still']} stage={t['stage']} "
          f"best={_EP['best']} failed={_EP['failed'] or '-'}", flush=True)
    if len(_TCP) == 2:
        print("DBG   " + " | ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{v[2] - TOP:.3f})" for k, v in _TCP.items()),
              flush=True)


def score(objects) -> float:
    """The best stage reached (0 for the rest of the run once the green tomato has come off the truss, a tomato has been
    pulled 6 mm along its stem or a tomato has fallen onto the table outside the tray): 0.25 for each red tomato off the truss and 0.25 more for each one in the
    tray, at most 0.75. Full score (1.0) is success()."""
    t = _tally(objects)
    if _DEBUG:
        _debug(objects, t)
    return 0.0 if _EP["failed"] else _EP["best"]


def success(objects) -> bool:
    """Both red tomatoes in the tray, the green one on the truss, both grippers 5 cm or more from every tomato, still
    for 1 s, no rule broken."""
    t = _tally(objects)
    return bool(t["done"] and not _EP["failed"])


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right gripper comes at each red tomato from the front, level, fingers either side of it, and closes on it. It
# then swings the tomato up and toward itself around the knuckle, bending the pedicel until the knuckle snaps, draws
# it back and lowers it into the tray, lets go and backs out. The left arm stays where it is. TOMATO_PULL=1 pulls the
# first tomato straight down along its stem instead of bending it: the stem stretches and the truss counts as torn.


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


SIDE = _ry(-math.pi / 2) @ grip_rotation(0.0)  # pointing forward, level, fingers closing along y
OPEN, PINCH = -0.032, float(os.environ.get("TOMATO_PINCH", str(-(R_F - 0.003))))
SAG = float(os.environ.get("TOMATO_SAG", "0.0015"))
BEND_DEG = float(os.environ.get("TOMATO_BEND_DEG", "55"))
RELEASE_Z = TOP + float(os.environ.get("TOMATO_RELEASE_DZ", "0.07"))  # TCP height when letting go over the tray


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


def _w(t, p, finger, label, rot=SIDE) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in p), finger, label, rot)


def _idle(arm: str, seconds: float) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return [Waypoint(seconds, tuple(float(v) for v in pos), OPEN, "stay put", rot)]


def _pick(y: float, first: bool) -> list[Waypoint]:
    centre = np.array([FX, y, FZ + SAG])
    front = centre + [-0.08, 0.0, 0.0]
    knuckle = np.array([FX, y, KZ + SAG])
    out = (_lead_in("right", front, SIDE, OPEN, "in front of the red tomato, level") if first
           else [_w(2.5, front, OPEN, "in front of the next red tomato, level")])
    out += [_w(2.0, centre, OPEN, "forward, fingers either side of the tomato"),
            _w(1.0, centre, PINCH, "close on it")]
    if os.environ.get("TOMATO_PULL") == "1":
        return out + [_w(2.0, centre + [0.0, 0.0, -0.03], PINCH, "pull it straight down, along its stem"),
                      _w(1.0, centre + [0.0, 0.0, -0.03], OPEN, "open")]
    for deg in (15.0, 30.0, 42.0, BEND_DEG):  # swing it up and toward the robot around the knuckle
        a = math.radians(deg)
        p = knuckle + (R_F + STUB) * np.array([-math.sin(a), 0.0, -math.cos(a)])
        out.append(_w(1.0, p, PINCH, f"bend it up around the knuckle ({deg:.0f} degrees)"))
    low = np.array([FX - 0.035, y, RELEASE_Z])
    out += [_w(1.5, low, PINCH, "draw it back and down over the tray"),
            _w(0.8, low, OPEN, "let it go into the tray"),
            _w(1.2, low + [-0.07, 0.0, 0.0], OPEN, "back out"),
            _w(1.0, low + [-0.10, 0.0, 0.05], OPEN, "back out and up")]
    return out


def _right() -> list[Waypoint]:
    out = []
    for y, ripe in zip(FRUIT_Y, RIPE):
        if ripe:
            out += _pick(y, first=not out)
            if os.environ.get("TOMATO_PULL") == "1":
                break
    out.append(_w(3.0, out[-1].pos, OPEN, "wait"))
    return out


def demo():
    right = _right()
    return {"left": _idle("left", sum(w.duration for w in right)), "right": right}
