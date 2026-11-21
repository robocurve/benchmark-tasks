"""On the station table: the weight stack of a selectorized cable machine (the plates, the selector stem through them,
the two guide rods and the base they rest on; the machine's frame, cable and handle are not modelled). Twelve 10-lb
steel plates (28 x 15 x 2.5 cm) are stacked on two guide rods; a selector stem (a 25 mm square steel bar with a
10.5 mm cross hole at the middle of each plate) runs down through the 30 mm square hole in the middle of every plate,
and the top plate is fixed to it. Each plate has a 13 mm bore running in from its front face, across the stem's cross
hole, to 2.5 cm short of its back face, and its weight (10 ... 120 lb) printed on its front. The selector pin (a 9.5 mm
(3/8 in) steel shaft 11.5 cm long with a rounded tip and a 44 mm red plastic knob with a magnet in it) sits in the
40-lb plate (the 4th plate from the top). Task (instruction): set the stack to 70 lb. The instruction does not say how.

How it is done: the pin is pulled out of the plate it is in and pushed into the bore of the plate wanted; the stem
then lifts that plate and every plate above it, so 70 lb is the 7th plate from the top. Owner's manuals insist that
it goes fully in: "Make sure that the weight pin is inserted fully into the weight stack before you exercise" (Gold's
Gym GGSY30580 manual, Important Precautions); "Be certain that weight pins are completely inserted prior to
exercising" (Nautilus NS200 manual). A pin left only part-way in lifts only the top plate.

The hand skill (one arm, as a person does it): the pin is pulled straight out against its magnet (6 N) along its axis,
then carried down three plates (7.6 cm) and pushed home into the 7th plate's bore. The stem's cross hole is 6 cm inside
the plate, hidden: only 0.5 mm wider all round than the pin. The pin must go in level and square to the plate; the
bore lets it tilt about 3 deg, enough for its tip to catch the stem's edge, so the hand has to keep it straight and let
its rounded tip find the hole while it pushes. A pin pushed in at the wrong height hits a plate's face; one in the
6th or 8th plate selects 60 or 80 lb.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- base: fixed steel frame foot 19 x 34 x 5.25 cm under the stack, x 0.305-0.495; the two 1-inch guide rods rise
  from it at y -0.135 and 0.075 to a top bar (visual only: the plates ride on slide joints).
- plates 2-12: each on its guide rods (a slide joint along z, free to rise, resting on the one below); front faces at
  x 0.325, middle at y -0.03; plate k (1 = top) sits with its middle 5.25 + (12 - k) x 2.53 + 1.25 cm above the
  table (the 7th at 19.2 cm, the 4th at 26.8 cm). 1.0 kg each in the simulation (a real 10-lb plate is 4.5 kg: with
  4.5 kg plates the light pin, squeezed between the stepped stem and the plates, was pushed through the plate's
  bore wall in the simulator; the robot never moves the plates, only the lift test does).
- stem with the top plate: fixed in place (kinematic), moved only by the lift test below.
- pin: loose, 0.11 kg, in the 4th plate's bore with its knob against the plate's face, held there by its magnet (a
  joint to the stem that lets go for good under 6 N or 0.5 N m).

Score (at the end of the run):
- The lift test (the cable pulling the stem, as a person checks it before a set): once the pin has been touched and
  then left alone (no finger within 10 mm of it) for 1 s, the stem rises 4 cm, holds 0.5 s and comes back down. The
  plates that rose 2 cm or more with it are counted (the stem is stepped 0.5 mm per control step, 1 cm/s). The test
  runs again after every new touch of the pin (a finger point within 4 mm of it).
- 1.0 (success, held 1 s): the latest lift test, with the pin not touched since, raised exactly 7 plates (70 lb),
  and the pin is fully home (its knob's face within 5 mm of the plate's face).
- 0.5: the same, but the pin is not fully home (it carries the stack by the tip of its shaft in the stem's hole, the
  partly inserted pin the manuals warn about).
- 0: the latest such test raised 2-6 or 8-12 plates (a wrong weight is selected), or the pin was never drawn out of
  the 40-lb plate.
- 0.3: otherwise, if the pin was drawn fully out of the 40-lb plate (its tip in front of the plate's face) at some
  point: the old setting is cleared but no weight, or not 70 lb, is selected (a test raised only the top plate, the
  pin was still held at the end, or no test ran).
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, Joint, SphereShape, allow_matrix_grips  # noqa: E402
from bodykit import as_numpy, on_reset, on_step, rotation  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
MM = 0.001

# -- the layout ----------------------------------------------------------------------------------------------------
N_PLATES = 12
T = 0.025  # plate thickness
GAP = 0.0003  # between plates in the sim (they touch on a real stack)
PITCH = T + GAP
PW, PD = 0.28, 0.15  # plate width (y), depth (x)
X_F = 0.325  # the plates' front faces
Y_S = -0.03  # the stack's middle line
X_C = X_F + PD / 2  # the stem's axis
BASE_H = 0.0525  # the base's top above the table
BORE = 0.013  # the pin's bore in each plate (square, 13 mm)
BORE_L = 0.125  # bore depth from the front face
STEM = 0.025  # the stem's square section
STEM_HOLE_PLATE = 0.030  # the plates' middle hole (square)
XHOLE = float(os.environ.get("GYM_XHOLE", "10.5")) * MM  # the stem's cross holes (square)
PIN_R = 0.00475  # 9.5 mm (3/8 in)
PIN_L = 0.115  # knob face to tip
KNOB_R, KNOB_L = 0.022, 0.025
PIN_MASS = 0.11
PLATE_MASS = float(os.environ.get("GYM_PLATE_KG", "1.0"))  # a real plate is 4.5 kg (10 lb); see the fidelity note
DETENT_N = float(os.environ.get("GYM_DETENT", "6.0"))  # the knob's magnet lets go at this pull (N)
OLD, TARGET = 4, 7  # the pin starts in the 40-lb plate; 70 lb is the 7th plate

BLACK, STEEL, DARKSTEEL, WHITE, RED, RUBBER, CHROME = (
    (0.10, 0.10, 0.11), (0.62, 0.64, 0.68), (0.30, 0.31, 0.33), (0.95, 0.95, 0.93), (0.80, 0.08, 0.06),
    (0.06, 0.06, 0.06), (0.82, 0.83, 0.86))


def plate_z(k: int) -> float:
    """The middle of plate ``k`` (1 = top) in world z."""
    return TOP + BASE_H + (N_PLATES - k) * PITCH + T / 2 + GAP


def _v(p) -> tuple:
    return tuple(float(x) for x in p)


# 7-segment digits for the printed weights (segments a-g)
_SEG = {"0": "abcdef", "1": "bc", "2": "abged", "3": "abgcd", "4": "fgbc", "5": "afgcd", "6": "afgedc",
        "7": "abc", "8": "abcdefg", "9": "abcdfg"}


def _digits(text: str, y0: float, x: float, z: float, h: float = 0.014, color=WHITE) -> list:
    """Visual 7-segment digits on a face at ``x`` (facing -x), the first digit's left edge at ``y0`` (reading from
    the front, left = +y), centred at ``z``."""
    w, s = h * 0.55, 0.0018
    out = []
    for i, ch in enumerate(text):
        yl = y0 - i * (w + 0.004)  # left edge (larger y) of this digit
        yc = yl - w / 2
        segs = {"a": (yc, z + h / 2, w, s), "g": (yc, z, w, s), "d": (yc, z - h / 2, w, s),
                "f": (yl, z + h / 4, s, h / 2), "b": (yl - w, z + h / 4, s, h / 2),
                "e": (yl, z - h / 4, s, h / 2), "c": (yl - w, z - h / 4, s, h / 2)}
        for name in _SEG[ch]:
            cy, cz, sy, sz = segs[name]
            out.append(BoxShape((x, cy, cz), (0.0006, sy, sz), color, collide=False))
    return out


def _plate_shapes(k: int, o) -> list:
    """Plate ``k``'s boxes, relative to the body origin ``o`` (world point), with the plate's middle at
    (X_C, Y_S, plate_z(k))."""
    c = np.array([X_C, Y_S, plate_z(k)]) - np.asarray(o, float)
    b, hy, hd, ht, sh = BORE / 2, PW / 2, PD / 2, T / 2, STEM_HOLE_PLATE / 2
    xb = -hd + BORE_L  # the bore's end (plate frame)

    def box(x0, x1, y0, y1, z0, z1, color=BLACK):
        return BoxShape(_v(c + [(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]), (x1 - x0, y1 - y0, z1 - z0), color)

    sh_ = [box(-hd, hd, -hy, -sh, -ht, ht), box(-hd, hd, sh, hy, -ht, ht),
           box(-hd, -sh, -sh, -b, -ht, ht), box(-hd, -sh, b, sh, -ht, ht),
           box(-hd, -sh, -b, b, b, ht), box(-hd, -sh, -b, b, -ht, -b),
           box(sh, hd, -sh, -b, -ht, ht), box(sh, hd, b, sh, -ht, ht),
           box(sh, xb, -b, b, b, ht), box(sh, xb, -b, b, -ht, -b), box(xb, hd, -b, b, -ht, ht)]
    # the weight printed on the front, left of the bore, and a white rim round the bore's mouth
    fx = c[0] - hd - 0.0004
    sh_ += _digits(str(10 * k), c[1] + 0.085, fx, c[2])
    sh_ += [BoxShape(_v([fx, c[1], c[2] + b + 0.001]), (0.0006, BORE + 0.004, 0.0015), WHITE, collide=False),
            BoxShape(_v([fx, c[1], c[2] - b - 0.001]), (0.0006, BORE + 0.004, 0.0015), WHITE, collide=False)]
    return sh_


def _stem_shapes(o) -> list:
    """The selector stem: a 25 mm square bar with a 10.5 mm square cross hole (along x) at the middle of each plate,
    from the base to 6 cm above the top plate, and the cable's clevis on top."""
    c0 = np.array([X_C, Y_S, 0.0]) - np.asarray(o, float)
    hs, hh = STEM / 2, XHOLE / 2
    out = []
    z_lo = TOP + BASE_H + 0.002
    for k in range(N_PLATES, 0, -1):
        zc = plate_z(k)
        # solid from z_lo to the hole's bottom, then the two side walls beside the hole
        out.append(BoxShape(_v(c0 + [0.0, 0.0, (z_lo + zc - hh) / 2]), (STEM, STEM, zc - hh - z_lo), STEEL))
        for sgn in (-1, 1):
            out.append(BoxShape(_v(c0 + [0.0, sgn * (hh + (hs - hh) / 2), zc]), (STEM, hs - hh, XHOLE), STEEL))
        z_lo = zc + hh
    z_top = plate_z(1) + T / 2 + 0.06
    out.append(BoxShape(_v(c0 + [0.0, 0.0, (z_lo + z_top) / 2]), (STEM, STEM, z_top - z_lo), STEEL))
    out.append(CylShape(_v(c0 + [0.0, 0.0, z_top + 0.015]), 0.012, 0.03, DARKSTEEL, collide=False))
    out.append(CylShape(_v(c0 + [0.0, 0.0, z_top + 0.10]), 0.0025, 0.14, RUBBER, collide=False))  # the cable
    return out


STEM_O = (X_C, Y_S, TOP + BASE_H)
STEM_BODY = Body.build("stem", pos=STEM_O, mass=6.0, color=STEEL, fixed=True,
                       shapes=_stem_shapes(STEM_O) + _plate_shapes(1, STEM_O), friction=(0.4, 0.35))


def _plate(k: int) -> Body:
    o = (X_C, Y_S, plate_z(k))
    return Body.build(f"plate_{k}", pos=o, mass=PLATE_MASS, color=BLACK, shapes=_plate_shapes(k, o),
                      friction=(0.4, 0.35), damping=(0.5, 0.5),
                      joint=Joint(anchor=o, kind="slide", axis="Z", axial=(0.0, 5.0, 0.1), slide_range=(0.0, 0.10)))


def _base() -> Body:
    o = (X_C, Y_S, TOP + BASE_H / 2)
    sh = [BoxShape((0.0, 0.0, 0.0), (0.19, 0.34, BASE_H), DARKSTEEL)]
    # rubber bumpers on top (visual), the two 1-inch guide rods up to a top bar (visual only: the plates ride on
    # their slide joints)
    for sgn in (-1, 1):
        sh.append(CylShape((0.0, sgn * 0.105, BASE_H / 2 + 0.0002), 0.02, 0.0004, RUBBER, collide=False))
        sh.append(CylShape((0.0, sgn * 0.105, BASE_H / 2 + 0.25), 0.0127, 0.50, CHROME, collide=False))
    sh.append(BoxShape((0.0, 0.0, BASE_H / 2 + 0.505), (0.05, 0.27, 0.02), DARKSTEEL, collide=False))
    return Body.build("base", pos=o, mass=20.0, color=DARKSTEEL, fixed=True, shapes=sh)


def _pin() -> Body:
    """The selector pin, its origin on its axis at the knob's face; along +x."""
    o = (X_F - 0.0003, Y_S, plate_z(OLD))
    sh = [CylShape((PIN_L / 2 - PIN_R / 2, 0.0, 0.0), PIN_R, PIN_L - PIN_R, CHROME, axis="X"),
          SphereShape((PIN_L - PIN_R, 0.0, 0.0), PIN_R, CHROME),
          CylShape((-KNOB_L / 2, 0.0, 0.0), KNOB_R, KNOB_L, RED, axis="X"),
          CylShape((-KNOB_L + 0.002, 0.0, 0.0), KNOB_R - 0.002, 0.004, (0.55, 0.05, 0.04), axis="X", collide=False)]
    return Body.build("pin", pos=o, mass=PIN_MASS, color=RED, shapes=sh, friction=(0.35, 0.3), contact_offset=0.001,
                      solver_iterations=32,
                      joint=Joint(anchor=o, kind="fixed", parent="stem", break_force=DETENT_N, break_torque=0.5))


BASE = _base()
PLATES = [_plate(k) for k in range(2, N_PLATES + 1)]
PIN = _pin()
OBJECTS = [BASE, STEM_BODY] + PLATES + [PIN]

# -- tracking and scoring -------------------------------------------------------------------------------------------
LIFT = 0.04  # the lift test raises the stem this far
LIFT_STEP = float(os.environ.get("GYM_LIFT_STEP", "0.0005"))  # per control step (1 cm/s)
HOLD_STEPS = 10
HOME = 0.005  # the knob within this of the plate's face: fully home
RAISED = 0.02  # a plate that rose this far went up with the stem
QUIET_STEPS = 20  # 1 s with no finger within RELEASE of the pin
RELEASE = 0.010
TOUCH = 0.004
_DEBUG = os.environ.get("GYM_DEBUG") == "1"
_DEBUG_EVERY = int(os.environ.get("GYM_DEBUG_EVERY", "5"))
_EP: dict = {}
_IDS: dict = {}
PIN_SAMPLES = np.array([[x, 0.0, 0.0] for x in np.linspace(0.0, PIN_L, 12)]
                       + [[x, KNOB_R * math.cos(a), KNOB_R * math.sin(a)] for x in (-KNOB_L, -KNOB_L / 2, 0.0)
                          for a in np.linspace(0, 2 * math.pi, 12, endpoint=False)]
                       + [[-KNOB_L, 0.0, 0.0]])


@on_reset
def _reset() -> None:
    _EP.update(tick=0, step=0, phase="idle", lift=0.0, hold=0, quiet=0, touched_ever=False, tested=False,
               result=None, valid=False, pulled=False, near=1.0, tip=np.zeros(3), raised=[], tcp={}, gaps={},
               tests=[], last_lift_written=None, axis_tilt=0.0)


_reset()


def _finger_points() -> dict:
    import struct
    import xml.etree.ElementTree as ET

    from yam_arena import YAM_URDF
    from yam_arena.urdf import rpy_matrix

    root = pathlib.Path(YAM_URDF).parent
    out = {}
    for link in ET.parse(YAM_URDF).getroot().iter("link"):
        name = link.get("name")
        if name not in ("tip_left", "tip_right"):
            continue
        pts = []
        for col in link.findall("collision"):
            o = col.find("origin")
            xyz = np.array([float(v) for v in (o.get("xyz") or "0 0 0").split()]) if o is not None else np.zeros(3)
            rpy = np.array([float(v) for v in (o.get("rpy") or "0 0 0").split()]) if o is not None else np.zeros(3)
            b = (root / col.find("geometry").find("mesh").get("filename")).read_bytes()
            n = struct.unpack("<I", b[80:84])[0]
            tri = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
            v = tri["v"].reshape(-1, 3).astype(float)
            pts.append(np.unique(np.round(v, 4), axis=0) @ rpy_matrix(rpy).T + xyz)
        out[name] = np.concatenate(pts)
    return out


def _write_stem(base, origin, lift: float) -> None:
    """Move the (kinematic) stem to ``lift`` above its rest, 0.5 mm per control step."""
    import torch

    p = np.asarray(STEM_O, float) + origin + np.array([0.0, 0.0, lift])
    pose = np.array([[p[0], p[1], p[2], 0.0, 0.0, 0.0, 1.0]])
    obj = base.scene["stem"]
    obj.write_root_link_pose_to_sim_index(root_pose=torch.tensor(pose, dtype=torch.float32, device=base.device))
    obj.write_root_com_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), dtype=torch.float32,
                                                                       device=base.device))


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("GYM_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _track_inner(env) -> None:
    from scipy.spatial import cKDTree

    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    _EP["tick"] += 1
    base = getattr(env, "unwrapped", env)
    origin = as_numpy(base.scene.env_origins[0])
    if "fingers" not in _IDS:
        _IDS["fingers"] = _finger_points()
    fingers = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        if side not in _IDS:
            _IDS[side] = list(robot.find_joints(ARM_JOINTS, preserve_order=True)[0])
            _IDS[side + "_f"] = list(robot.find_joints(FINGER_JOINTS, preserve_order=True)[0])
            _IDS[side + "_tips"] = [robot.body_names.index(n) for n in ("tip_left", "tip_right")]
        q = as_numpy(robot.data.joint_pos)[0]
        _EP["tcp"][side] = np.asarray(ik.tcp_pose(q[_IDS[side]], station.BASES[side])[0], float)
        _EP["gaps"][side] = float(-q[_IDS[side + "_f"]].sum())
        poses = as_numpy(robot.data.body_link_pose_w)[0]
        fingers.append(np.concatenate([_IDS["fingers"][n] @ rotation(poses[i][3:7]).T + poses[i][:3] - origin
                                       for n, i in zip(("tip_left", "tip_right"), _IDS[side + "_tips"])]))
    pp = as_numpy(base.scene["pin"].data.root_link_pose_w)[0]
    pr = rotation(pp[3:7])
    pin_o = pp[:3] - origin
    tip = pin_o + pr @ np.array([PIN_L, 0.0, 0.0])
    near = float(cKDTree(pin_o + PIN_SAMPLES @ pr.T).query(np.concatenate(fingers), k=1)[0].min())
    _EP.update(near=near, tip=tip, pin_o=pin_o, axis_tilt=math.degrees(math.acos(min(1.0, abs(float(pr[0, 0]))))))
    # the old setting cleared: the pin's tip came out in front of the plates' faces
    if tip[0] < X_F - 0.002:
        _EP["pulled"] = True
    raised = []
    for k in range(2, N_PLATES + 1):
        z = float(as_numpy(base.scene[f"plate_{k}"].data.root_link_pose_w)[0][2] - origin[2])
        raised.append(z - plate_z(k))
    _EP["raised"] = raised
    # touches of the pin cancel any earlier test result
    if near < RELEASE:
        _EP["quiet"] = 0
        if near < TOUCH:
            _EP["touched_ever"] = True
            _EP["tested"] = False
            _EP["valid"] = False
    else:
        _EP["quiet"] += 1
    ph = _EP["phase"]
    if ph == "idle":
        if _EP["touched_ever"] and not _EP["tested"] and _EP["quiet"] >= QUIET_STEPS:
            _EP["phase"] = "up"
            _EP["valid"] = True  # stays valid unless the pin is touched during or after the test
    elif ph == "up":
        _EP["lift"] = min(LIFT, _EP["lift"] + LIFT_STEP)
        if _EP["lift"] >= LIFT - 1e-9:
            _EP["phase"], _EP["hold"] = "hold", 0
    elif ph == "hold":
        _EP["hold"] += 1
        if _EP["hold"] >= HOLD_STEPS:
            count = 1 + sum(1 for r in raised if r >= RAISED)  # the top plate is fixed to the stem
            _EP["result"] = count
            _EP["tests"].append(count)
            _EP["phase"] = "down"
    elif ph == "down":
        _EP["lift"] = max(0.0, _EP["lift"] - LIFT_STEP)
        if _EP["lift"] <= 1e-9:
            _EP["phase"] = "idle"
            _EP["tested"] = True
    if _EP["last_lift_written"] != _EP["lift"]:
        _write_stem(base, origin, _EP["lift"])
        _EP["last_lift_written"] = _EP["lift"]


def _home() -> bool:
    """The pin fully home: its knob's face within HOME of the plates' faces."""
    po = _EP.get("pin_o")
    return po is not None and float(po[0]) >= X_F - 0.0003 - HOME


def _state() -> str:
    """'set' (70 lb, pin fully home), 'part' (70 lb, pin not fully home), 'wrong' (another weight), 'cleared' (pin out
    of the 40-lb plate, nothing else) or 'none'."""
    res = _EP["result"] if (_EP["valid"] and _EP["tested"]) else None
    if res is not None and res == TARGET:
        return "set" if _home() else "part"
    if res is not None and res != 1:
        return "wrong"
    return "cleared" if _EP["pulled"] else "none"


def _debug() -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    tip, po = _EP["tip"], _EP.get("pin_o", np.zeros(3))
    up = " ".join(f"{r * 1000:.0f}" for r in _EP["raised"])
    print(f"DBG step={_EP['step']} phase={_EP['phase']} lift={_EP['lift'] * 1000:.0f}mm state={_state()} home={int(_home())} "
          f"tests={_EP['tests']} valid={int(_EP['valid'])} tested={int(_EP['tested'])} pulled={int(_EP['pulled'])} "
          f"near={_EP['near'] * 1000:.0f}mm quiet={_EP['quiet']} pin_o=({po[0]:.4f},{po[1]:.4f},{po[2] - TOP:.4f}) "
          f"tip=({tip[0]:.4f},{tip[1]:.4f},{tip[2] - TOP:.4f}) tilt={_EP['axis_tilt']:.1f} raised[2..12]=[{up}] | "
          + " ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{v[2] - TOP:.3f}) gap={_EP['gaps'][k] * 1000:.0f}"
                     for k, v in _EP["tcp"].items()), flush=True)


def score(objects) -> float:
    """1.0 for 70 lb selected with the pin fully home (success), 0.5 for 70 lb with the pin not fully home, 0.3 for the
    old setting cleared only, 0 otherwise."""
    if _DEBUG:
        _debug()
    s = _state()
    return {"set": 1.0, "part": 0.5, "cleared": 0.3}.get(s, 0.0)


def success(objects) -> bool:
    return _state() == "set"


# -- scripted demo --------------------------------------------------------------------------------------------------
# Open loop, from the known layout; the left arm stays at the ready pose. The right gripper, level and pointing at the
# stack (fingers closing across the knob, along y), comes in round the knob of the pin in the 40-lb plate, closes on
# it and draws it straight out 14 cm (its tip 2.7 cm clear of the plates); it carries the pin down 7.6 cm to the
# 70-lb plate's height, 1.5 cm in front of its face, and pushes it home slowly (aiming 1 cm past the seat), waits,
# opens, backs off and stays clear while the lift test runs.
# GYM_WAY=plate6 pins the 60-lb plate instead (a wrong weight: 0); GYM_WAY=tilt pushes the pin in pitched 4 deg (it
# binds part-way: see the README).
OPEN_C = -0.047
ARM = "right"
WAY = os.environ.get("GYM_WAY", "demo")
PINCH = float(os.environ.get("GYM_PINCH", "-0.0185"))  # closed on the 44 mm knob (3.5 mm inside each face)
LEVEL = np.column_stack([(0.0, -1.0, 0.0), (0.0, 0.0, -1.0), (1.0, 0.0, 0.0)])  # pointing +x, closing along y
GRIP_DX = float(os.environ.get("GYM_GRIP_DX", "-0.0125"))  # the TCP this far in front of the knob's face (its middle)
PULL_OVER = float(os.environ.get("GYM_PULL_OVER", "0.015"))
PUSH_OVER = float(os.environ.get("GYM_PUSH_OVER", "0.010"))
PUSH_S = float(os.environ.get("GYM_PUSH_S", "5.0"))
DZ_AIM = float(os.environ.get("GYM_DZ", "0.0"))  # height trim when aiming at the bore


def knob_at(k: int, dx: float = 0.0, dz: float = 0.0) -> np.ndarray:
    """The TCP on the knob with the pin seated in plate ``k`` (dx > 0: further in)."""
    return np.array([X_F - 0.0003 + GRIP_DX + dx, Y_S, plate_z(k) + dz])


def _good_ik(arm: str, pos, rot):
    from yam_arena import ik, station

    best = None
    b = station.BASES[arm]
    j1 = math.atan2(float(pos[1]) - b[1], float(pos[0]) - b[0])
    for sd in itertools.product((j1,), (0.8, 1.3), (0.6, 1.2), (-0.8, 0.8), (0.0,), (-1.8, -0.9, 0.0, 0.9, 1.8)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.asarray(sd, float),
                             base_pos=station.BASES[arm], iters=150)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
                if room > 0.3:
                    break
    if best is None:
        raise ValueError(f"no IK solution for the {arm} arm at {pos}")
    return best[0]


def _lead_in(arm: str, pos, rot, finger: float, label: str, seconds: float = 3.0, steps: int = 15) -> list:
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


def _stay(arm: str, t: float = 120.0) -> list:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return [Waypoint(t, tuple(float(v) for v in pos), OPEN_C, "stay put", rot)]


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _arm_path() -> list:
    r = LEVEL
    tgt = 6 if WAY == "plate6" else TARGET
    out_dx = -(PIN_L + 0.012)  # the pin's tip 1.2 cm in front of the plates
    out = _lead_in(ARM, knob_at(OLD, -0.10) + np.array([0.0, -0.12, 0.0]), r, OPEN_C, "off to the right, open")
    out += [_w(2.0, knob_at(OLD, -0.10), OPEN_C, "in front of the pin's knob", r),
            _w(2.0, knob_at(OLD), OPEN_C, "round the knob", r),
            _w(0.5, knob_at(OLD), OPEN_C, "settle", r),
            _w(1.0, knob_at(OLD), PINCH, "take the knob", r),
            _w(0.5, knob_at(OLD), PINCH, "hold", r),
            _w(3.0, knob_at(OLD, out_dx - PULL_OVER), PINCH, "draw the pin straight out", r),
            _w(1.0, knob_at(OLD, out_dx - PULL_OVER), PINCH, "let the arm catch up", r),
            _w(2.5, knob_at(tgt, out_dx, DZ_AIM), PINCH, "carry it down to the 70-lb plate", r),
            _w(1.0, knob_at(tgt, out_dx, DZ_AIM), PINCH, "line it up", r)]
    rp = r
    if WAY == "tilt":
        rp = _ry(math.radians(-4.0)) @ r  # the pin's tip pitched up
        out += [_w(1.0, knob_at(tgt, out_dx + 0.01, DZ_AIM - 0.003), PINCH, "tilt", rp)]
    out += [_w(PUSH_S, knob_at(tgt, PUSH_OVER, DZ_AIM), PINCH, "push it home", rp),
            _w(1.5, knob_at(tgt, PUSH_OVER, DZ_AIM), PINCH, "hold it home", rp),
            _w(1.0, knob_at(tgt, 0.0, DZ_AIM), OPEN_C, "let go", rp),
            _w(2.0, knob_at(tgt, -0.08, DZ_AIM), OPEN_C, "back off", rp),
            _w(15.0, knob_at(tgt, -0.10, 0.03), OPEN_C, "stay clear while the stack is test-lifted", rp)]
    return out


def demo():
    other = "left" if ARM == "right" else "right"
    return {ARM: _arm_path(), other: _stay(other)}
