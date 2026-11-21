"""A full-size interior door beside the station table, its knob under a child-safety knob cover.

A wall stands along the table's right-hand edge (the right arm's side) with a prehung 30 x 80 in hollow-core interior
door in it. The door opens away from the robot (push to open). Its passage knob is 914 mm (36 in) above the floor,
about 155 mm above the table top. Over the knob is a child-safety knob cover: a round plastic shell that spins
freely on the knob, with two grip zones on opposite sides (darker) that flex in when squeezed hard. The latch bolt
holds the door shut until the knob is turned. The task: open the door at least 100 mm at its latch edge, using the
knob, with the cover left on.

Frame: x forward from the arm bases, y left, z up, metres.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402  (hooks yam_arena so multi-shape bodies and joints are simulated)
from bodykit import Body, BoxShape, CylShape  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

bodykit.allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
FLOOR = TOP - 0.76  # the station's floor (table top 760 mm above it)
SCORE_OVER = "best"
CONTROL_HZ = 20.0
IN = 0.0254

# -- the door (Masonite-style 30 x 80 in hollow-core flush interior door, 1-3/8 in thick) -------------------------------
DOOR_W, DOOR_T, DOOR_H = 30 * IN, 1.375 * IN, 80 * IN
DOOR_MASS = 11.3  # kg (about 25 lb for a 30 in hollow-core slab)
UNDERCUT = 0.5 * IN  # gap under the door
WALL_FACE_Y = -0.652  # the wall's robot-side face, 2 mm past the table's right edge (y -0.650)
JAMB_D = 4.5625 * IN  # 4-9/16 in jamb (2x4 wall with 1/2 in drywall each side)
YD = WALL_FACE_Y - JAMB_D / 2  # the door's centre plane (y), the door centred in its jamb
Y_FACE = YD + DOOR_T / 2  # the door's robot-side face
KNOB_X = 0.300  # the knob axis (x), nominal
KNOB_Z = FLOOR + 36 * IN  # knob centre 36 in above the floor
BACKSET = 2.375 * IN
XL = KNOB_X - BACKSET  # the door's latch edge (x)
XH = XL + DOOR_W  # its hinge edge
HINGE_Y = YD - DOOR_T / 2  # hinge pins on the far face (the door swings away from the robot)
DOOR_Z0 = FLOOR + UNDERCUT
DOOR_ZC = DOOR_Z0 + DOOR_H / 2
GAP = 0.003  # door edge to jamb
JAMB_T = 0.75 * IN
FRAME_MASS = 500.0

# -- the Kwikset Tylo passage knob (1-7/8 in knob, 2-5/8 in rose, 2-1/2 in projection) ------------------------------------
ROSE_R, ROSE_T = 2.625 * IN / 2, 0.010
NECK_R, NECK_Y = 0.011, (ROSE_T, 0.035)
KNOB_R, KNOB_Y = 1.875 * IN / 2, (0.035, 2.5 * IN)
KNOB_LIMIT = 50.0  # degrees each way to the knob's stop
KNOB_FULL = 45.0  # the bolt is fully in at this turn
KNOB_DEAD = 3.0  # free play before the cam starts to draw the bolt
KNOB_TORQUE_FULL = 0.50  # N m at KNOB_FULL (return spring and latch spring together, est.)
KNOB_SPRING = KNOB_TORQUE_FULL / KNOB_FULL  # N m per degree
THROW = 0.5 * IN  # latch bolt throw (1/2 in)
LATCH_K = 800.0  # N/m: the bolt spring and the cam drive, 10 N at full throw (est.)

# -- the knob cover (child-safety door knob cover: round shell, two flexing grip zones) ---------------------------------
SHELL_R, SHELL_Y = 0.031, (0.012, 0.036)  # 62 mm across behind the grip zones
WAIST_R, WAIST_Y = 0.029, (0.036, 0.062)  # under the grip zones
NOSE_R, NOSE_Y = 0.026, (0.062, 0.070)  # the rounded front, 52 mm across (the shell is 58 mm deep in all)
PAD_OUT, PAD_IN = 0.033, 0.0275  # grip zones stand 2 mm proud: 66 mm across them; inner faces 3.7 mm off the knob
PAD_W, PAD_L = 0.026, 0.024  # tangential width, axial length
PAD_YC = 0.049  # axial centre of the grip zones (on the knob's widest part)
PAD_TRAVEL = 0.0065  # how far a grip zone can flex in (it meets the knob after 5.7 mm)
PAD_K = 1500.0  # N/m: 5.6 N to flex a grip zone onto the knob
COVER_MASS, PAD_MASS = 0.030, 0.010

WHITE, OFFWHITE = (0.93, 0.93, 0.91), (0.86, 0.85, 0.80)
NICKEL, BRASS = (0.74, 0.74, 0.72), (0.78, 0.62, 0.30)
COVER_C, PAD_C = (0.95, 0.95, 0.93), (0.45, 0.62, 0.78)
INK = (0.08, 0.08, 0.08)  # the far side's dial marks and the knob's index line


def _box(lo, hi, color, collide: bool = True) -> BoxShape:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return BoxShape(tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, collide=collide)


def _ycyl(y0: float, y1: float, r: float, color, x: float = 0.0, z: float = 0.0, collide: bool = True) -> CylShape:
    return CylShape((x, (y0 + y1) / 2, z), r, abs(y1 - y0), color, collide=collide, axis="Y")


FRAME_POS = (XL, YD, KNOB_Z)  # the frame body's origin: latch edge line, door centre plane, knob height


def _frame_shapes() -> list:
    """Jambs, stops, strike pocket, walls and casing, in the frame body (origin FRAME_POS)."""
    j = JAMB_D / 2
    z0, ztop = FLOOR - KNOB_Z, DOOR_Z0 + DOOR_H - KNOB_Z  # floor and door top, frame-relative
    zw = FLOOR + 2.2 - KNOB_Z  # wall top
    xj0, xj1 = -GAP - JAMB_T, -GAP  # latch jamb in x
    pz, py, pd = 0.0165, 0.0075, 0.016  # strike pocket half height, half width, depth
    s = [
        # latch-side jamb with the strike pocket the bolt enters
        _box((xj0, -j, z0), (xj1, j, -pz), WHITE),
        _box((xj0, -j, pz), (xj1, j, ztop + GAP), WHITE),
        _box((xj0, -j, -pz), (xj1, -py, pz), WHITE),
        _box((xj0, py, -pz), (xj1, j, pz), WHITE),
        _box((xj0, -py, -pz), (xj1 - pd, py, pz), (0.2, 0.2, 0.2)),
        _box((xj1 - 0.0008, -0.0135, -0.034), (xj1, -py, 0.034), BRASS),  # strike plate (lip on the far side)
        _box((xj1 - 0.0008, py, -0.034), (xj1, 0.0135, 0.034), BRASS, collide=False),
        # stop moulding (3/8 x 1-1/4 in) on the robot's side: the door shuts against it
        _box((xj1, DOOR_T / 2 + 0.001, z0 + 0.01), (xj1 + 0.0095, DOOR_T / 2 + 0.001 + 1.25 * IN, ztop + GAP), WHITE),
        # head jamb and its stop, hinge jamb
        _box((xj0, -j, ztop + GAP), (DOOR_W + GAP + JAMB_T, j, ztop + GAP + JAMB_T), WHITE),
        _box((xj1, DOOR_T / 2 + 0.001, ztop + GAP - 0.0095), (DOOR_W + GAP, DOOR_T / 2 + 0.001 + 1.25 * IN, ztop + GAP),
             WHITE),
        _box((DOOR_W + GAP, -j, z0), (DOOR_W + GAP + JAMB_T, j, ztop + GAP + JAMB_T), WHITE),
        # walls (painted drywall), each side of the opening and over it
        _box((xj0 - 1.0, -j, z0), (xj0, j, zw), OFFWHITE),
        _box((DOOR_W + GAP + JAMB_T, -j, z0), (DOOR_W + GAP + JAMB_T + 0.6, j, zw), OFFWHITE),
        _box((xj0, -j, ztop + GAP + JAMB_T), (DOOR_W + GAP + JAMB_T, j, zw), OFFWHITE),
        # casing (2-1/4 in trim) on the robot's side, visual only (it overhangs the table's edge line)
        _box((xj0 - 0.051, j, z0), (xj0 + 0.006, j + 0.0175, ztop + GAP + JAMB_T + 0.051), WHITE, collide=False),
        _box((DOOR_W + GAP + JAMB_T - 0.006, j, z0), (DOOR_W + GAP + JAMB_T + 0.051, j + 0.0175,
                                                       ztop + GAP + JAMB_T + 0.051), WHITE, collide=False),
        _box((xj0 - 0.051, j, ztop + GAP + JAMB_T - 0.006), (DOOR_W + GAP + JAMB_T + 0.051, j + 0.0175,
                                                             ztop + GAP + JAMB_T + 0.051), WHITE, collide=False),
    ]
    for zh in (DOOR_Z0 + 0.18, DOOR_ZC, DOOR_Z0 + DOOR_H - 0.28):  # three 3-1/2 in hinges (visual)
        s.append(_box((DOOR_W - 0.004, -DOOR_T / 2 - 0.004, zh - KNOB_Z - 0.0445),
                      (DOOR_W + GAP + 0.004, -DOOR_T / 2, zh - KNOB_Z + 0.0445), BRASS, collide=False))
    return s


DOOR_POS = (XL + DOOR_W / 2, YD, DOOR_ZC)


DIAL_DEG = (0.0, 15.0, -15.0, 45.0, -45.0)  # marks on the far side's printed dial (degrees from level)
DIAL_R = 0.045  # the marks' radius on the dial (m), outside the 67 mm rose


def _dial_marks(y: float, kx: float, kz: float) -> list:
    """Square marks of the printed dial round the far side's rose (the 0, 15 and 45 degree lines), door frame."""
    out = []
    for a in DIAL_DEG:
        x, z = kx + DIAL_R * math.sin(math.radians(a)), kz + DIAL_R * math.cos(math.radians(a))
        out.append(_box((x - 0.002, y - 0.0005, z - 0.002), (x + 0.002, y, z + 0.002), INK, collide=False))
    return out


def _far_knob_shapes() -> list:
    """The far side's knob, on the same spindle as the covered one (no cover), with a paint-pen index line on its end
    face that points at the dial's 0 mark when the knob is level. Visual only: nothing touches it."""
    y = -DOOR_T  # the far face, in the knob body's frame (its origin is on the robot-side face)
    end = y - KNOB_Y[1]
    return [_ycyl(y - ROSE_T, y - KNOB_Y[0], NECK_R, NICKEL, collide=False),
            _ycyl(y - KNOB_Y[0], end, KNOB_R, NICKEL, collide=False),
            _box((-0.0015, end - 0.0005, 0.0), (0.0015, end, KNOB_R - 0.002), INK, collide=False)]


def _door_shapes() -> list:
    kx, kz = KNOB_X - DOOR_POS[0], KNOB_Z - DOOR_ZC
    t = DOOR_T / 2
    return [
        _box((-DOOR_W / 2, -t, -DOOR_H / 2), (DOOR_W / 2, t, DOOR_H / 2), WHITE),
        _ycyl(t, t + ROSE_T, ROSE_R, NICKEL, kx, kz),  # rose on the robot's side
        _ycyl(-t, -t - ROSE_T, ROSE_R, NICKEL, kx, kz, collide=False),  # the far side's rose (visual)
        *_dial_marks(-t, kx, kz),  # the printed dial taped round the far rose (visual)
        _box((-DOOR_W / 2, -0.0125, kz - 0.0143), (-DOOR_W / 2 + 0.0008, 0.0125, kz + 0.0143), BRASS,
             collide=False),  # latch faceplate
    ]


KNOB_POS = (KNOB_X, Y_FACE, KNOB_Z)  # the knob's origin: on its axis, at the door face
BOLT_POS = (XL, YD, KNOB_Z)


def _build() -> list:
    # the wall and frame: a heavy free body standing on the floor (so each trial's start can move it), 500 kg
    frame = Body.build("frame", pos=FRAME_POS, mass=FRAME_MASS, shapes=_frame_shapes(), color=WHITE,
                       friction=(1.0, 0.9), solver_iterations=32, damping=(0.5, 0.5))
    door = Body.build("door", pos=DOOR_POS, mass=DOOR_MASS, shapes=_door_shapes(), color=WHITE, friction=(0.5, 0.4),
                      solver_iterations=32, damping=(0.0, 0.0))
    bolt = Body.build("bolt", pos=BOLT_POS, mass=0.02, color=BRASS, friction=(0.3, 0.25), solver_iterations=32,
                      shapes=[_box((-THROW, -0.0055, -0.0125), (0.012, 0.0055, 0.0125), BRASS)], ignore=("door",))
    knob = Body.build("knob", pos=KNOB_POS, mass=0.20, color=NICKEL, friction=(0.5, 0.45), solver_iterations=32,
                      shapes=[_ycyl(*NECK_Y, NECK_R, NICKEL), _ycyl(*KNOB_Y, KNOB_R, NICKEL), *_far_knob_shapes()],
                      ignore=("door", "bolt"))
    cover = Body.build("cover", pos=KNOB_POS, mass=COVER_MASS, color=COVER_C, friction=(0.45, 0.4),
                       solver_iterations=32, shapes=[_ycyl(*SHELL_Y, SHELL_R, COVER_C), _ycyl(*WAIST_Y, WAIST_R, COVER_C),
                                                     _ycyl(*NOSE_Y, NOSE_R, COVER_C)],
                       ignore=("door", "bolt", "knob"))
    rc = (PAD_OUT + PAD_IN) / 2
    pads = []
    for name, sz in (("pad_a", 1.0), ("pad_b", -1.0)):
        pos = (KNOB_X, Y_FACE + PAD_YC, KNOB_Z + sz * rc)
        pads.append(Body.build(name, pos=pos, mass=PAD_MASS, color=PAD_C, friction=(0.6, 0.5), solver_iterations=32,
                               shapes=[BoxShape((0.0, 0.0, 0.0), (PAD_W, PAD_L, PAD_OUT - PAD_IN), PAD_C)],
                               ignore=("door", "bolt", "cover", "pad_a")))
        bodykit.slider(f"flex_{name[-1]}", "cover", name, anchor=pos, axis="Z",
                       limits=(-PAD_TRAVEL, 0.0) if sz > 0 else (0.0, PAD_TRAVEL), stiffness=PAD_K, damping=1.0,
                       target=0.0, max_force=200.0)
    bodykit.hinge("door_hinge", "frame", "door", anchor=(XH, HINGE_Y, DOOR_ZC), axis="Z", limits_deg=(0.0, 95.0),
                  damping=0.002)
    bodykit.slider("latch", "door", "bolt", anchor=BOLT_POS, axis="X", limits=(-0.0002, THROW + 0.0005),
                   stiffness=LATCH_K, damping=5.0, target=0.0, max_force=60.0)
    bodykit.hinge("spindle", "door", "knob", anchor=KNOB_POS, axis="Y", limits_deg=(-KNOB_LIMIT, KNOB_LIMIT),
                  spring=KNOB_SPRING, damping=0.0003)
    bodykit.hinge("cover_spin", "knob", "cover", anchor=KNOB_POS, axis="Y", damping=2e-6)
    return [frame, door, bolt, knob, cover, *pads]


OBJECTS = _build()
NAMES = [o.name for o in OBJECTS]


# -- reading the mechanism from poses ------------------------------------------------------------------------------------
def _ry(a: float) -> np.ndarray:
    return np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]])


def _rz(a: float) -> np.ndarray:
    return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])


def _quat(r: np.ndarray) -> tuple:
    return bodykit._quat_from_matrix(r)


def y_angle(rel: np.ndarray) -> float:
    """Degrees about +y of a relative rotation."""
    return math.degrees(math.atan2(rel[0, 2], rel[0, 0]))


def door_deg(st) -> float:
    r = st["frame"].rotation().T @ st["door"].rotation()
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def knob_deg(st) -> float:
    return y_angle(st["door"].rotation().T @ st["knob"].rotation())


def cover_deg(st) -> float:
    """The cover's turn on the knob (degrees)."""
    return y_angle(st["knob"].rotation().T @ st["cover"].rotation())


def bolt_in(st) -> float:
    """How far the latch bolt has been drawn into the door (m)."""
    door = st["door"]
    return float((door.rotation().T @ (np.asarray(st["bolt"].pos, float) - np.asarray(door.pos, float)))[0]
                 - (BOLT_POS[0] - DOOR_POS[0]))


def pad_travel(st, p: str) -> float:
    """How far a grip zone has flexed in toward the knob (m)."""
    cover = st["cover"]
    local = cover.rotation().T @ (np.asarray(st[f"pad_{p}"].pos, float) - np.asarray(cover.pos, float))
    return float((PAD_OUT + PAD_IN) / 2 - abs(local[2]))


def edge_open_mm(st) -> float:
    """How far the door's latch edge (its robot-side corner at knob height) has moved from where it is shut (mm):
    what a tape measures from the stop moulding to the door's edge."""
    frame, door = st["frame"], st["door"]
    corner = np.array([-DOOR_W / 2, DOOR_T / 2, 0.0])
    shut = np.asarray(frame.pos, float) + frame.rotation() @ (np.array(DOOR_POS) - np.array(FRAME_POS) + corner)
    now = np.asarray(door.pos, float) + door.rotation() @ corner
    return float(np.linalg.norm((now - shut)[:2]) * 1000.0)


def bolt_target(knob: float) -> float:
    """The latch's cam: the bolt is drawn in as the spindle turns either way (m)."""
    s = (abs(knob) - KNOB_DEAD) / (KNOB_FULL - KNOB_DEAD)
    return THROW * float(np.clip(s, 0.0, 1.0))


# -- episode state, the latch rule, the score -----------------------------------------------------------------------------
_EP: dict = {}
_DBG = open(os.environ["KNOB_DEBUG"], "w") if os.environ.get("KNOB_DEBUG") else None
POINTS = ((100.0, 1.0), (20.0, 0.6))  # latch edge open at least this far (mm): score
KNOB_DRAWN_DEG, KNOB_MIN_DEG = KNOB_FULL, 15.0  # read on the far side's dial: bolt fully drawn (45); first turn


def _blank() -> None:
    _EP.clear()
    _EP.update(step=0, knob_max=0.0, bolt_max=0.0, open_max=0.0, pad_max=0.0, st=None, report={})


_blank()


@bodykit.on_reset
def _reset() -> None:
    _blank()


def _track(st) -> None:
    k = knob_deg(st)
    bodykit.set_slider_drive("latch", target=bolt_target(k))
    _EP["knob_max"] = max(_EP["knob_max"], abs(k))
    _EP["bolt_max"] = max(_EP["bolt_max"], bolt_in(st) * 1000.0)
    _EP["open_max"] = max(_EP["open_max"], edge_open_mm(st))
    _EP["pad_max"] = max(_EP["pad_max"], min(pad_travel(st, "a"), pad_travel(st, "b")) * 1000.0)


@bodykit.on_step
def _watch(env) -> None:
    from yam_arena import scene as _scene

    st = _scene.object_states(env, NAMES)
    _EP["st"] = st
    _track(st)
    _EP["step"] += 1
    if _DBG is not None:
        _debug(st)


def report(objects=None) -> dict:
    e = _EP
    if e["open_max"] >= POINTS[0][0]:
        s = 1.0
    elif e["open_max"] >= POINTS[1][0]:
        s = 0.6
    elif e["knob_max"] >= KNOB_DRAWN_DEG:
        s = 0.3
    elif e["knob_max"] >= KNOB_MIN_DEG:
        s = 0.1
    else:
        s = 0.0
    return dict(score=s, open_mm=e["open_max"], bolt_mm=e["bolt_max"], knob_deg=e["knob_max"], pad_mm=e["pad_max"])


def score(objects) -> float:
    """1.0: the latch edge opened at least 100 mm; 0.6: at least 20 mm (unlatched); 0.3: the knob turned at least 45
    degrees (the bolt fully drawn), as the far side's knob shows on its dial; 0.1: at least 15 degrees; otherwise 0.
    The best reached during the trial."""
    rep = report(objects)
    _EP["report"] = rep
    return float(rep["score"])


def distance(objects) -> float:
    return max(0.0, (POINTS[0][0] - _EP["open_max"]) / 1000.0)


def _debug(st) -> None:
    rep = _EP.get("report") or {}
    _DBG.write(f"{_EP['step']:4d} knob={knob_deg(st):7.2f} cover={cover_deg(st):8.2f} "
               f"pads=({pad_travel(st, 'a') * 1000:4.1f},{pad_travel(st, 'b') * 1000:4.1f}) "
               f"bolt={bolt_in(st) * 1000:5.2f} door={door_deg(st):6.2f} open={edge_open_mm(st):6.1f} "
               f"frame=({st['frame'].pos[0]:.4f},{st['frame'].pos[1]:.4f}) score={rep.get('score')}\n")
    _DBG.flush()


# -- the seeded start ----------------------------------------------------------------------------------------------------
SHIFT_X = 0.040  # the wall and door move up to this far along x (m), each trial
SHIFT_Y = (-0.010, 0.0)  # and this far away from the table (m)
FAIL = os.environ.get("KNOB_FAIL", "")  # wrong outcomes on purpose (README): fixed, light, early, push


def layout(seed: int | None) -> dict:
    """The seeded start: the door assembly's shift (m) and the cover's turn on the knob (degrees)."""
    rng = np.random.default_rng(seed) if seed else None
    if rng is None:
        return dict(dx=0.0, dy=0.0, cover=0.0)
    if os.environ.get("KNOB_NOSHIFT"):  # development: the door where the setup text puts it, the cover still turned
        return dict(dx=0.0, dy=0.0, cover=float(rng.uniform(0.0, 180.0)))
    return dict(dx=float(rng.uniform(-SHIFT_X, SHIFT_X)), dy=float(rng.uniform(*SHIFT_Y)),
                cover=float(rng.uniform(0.0, 180.0)))


def poses(lay: dict) -> dict:
    """World poses (position, quaternion xyzw) of every body for a layout."""
    d = np.array([lay["dx"], lay["dy"], 0.0])
    out = {}
    for o in OBJECTS:
        out[o.name] = (np.asarray(o.pos, float) + d, (0.0, 0.0, 0.0, 1.0))
    r = _ry(math.radians(lay["cover"]))
    axis = np.asarray(KNOB_POS, float) + d
    for name in ("cover", "pad_a", "pad_b"):
        p = np.asarray(out[name][0], float)
        out[name] = (axis + r @ (p - axis), _quat(r))
    return out


@bodykit.on_reset_env
def _lay_out(emb, seed):
    """Put the bodies where this trial's seed says, let them settle, and return the first observation."""
    import torch
    from yam_arena import inspect_embodiment as ie
    from yam_arena import scene as _scene

    base = emb._base
    lay = layout(seed)
    _blank()
    for name, (pos, quat) in poses(lay).items():
        obj = base.scene[name]
        pose = obj.data.default_root_pose.torch.clone()
        pose[:, :3] = torch.tensor(pos, dtype=pose.dtype, device=pose.device) + base.scene.env_origins
        pose[:, 3:7] = torch.tensor(quat, dtype=pose.dtype, device=pose.device)
        obj.write_root_pose_to_sim_index(root_pose=pose)
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(obj.data.default_root_vel.torch))
    bodykit.set_slider_drive("latch", target=0.0)
    for _ in range(max(1, round(getattr(ie, "SETTLE_S", 0.5) / base.step_dt))):
        emb._env.step(emb._ready)
    for _ in range(getattr(ie, "RESET_RENDERS", 3)):
        base.sim.render()
    _blank()
    _EP["st"] = _scene.object_states(emb._env, NAMES)
    _EP["layout"] = lay
    if _DBG is not None:
        _DBG.write(f"seed {seed}: layout {lay}\n")
        _debug(_EP["st"])
    return emb._observation()


# -- the scripted demo (reads object poses from the simulator to plan; moves the parts only through the grippers) --------
TIP = 0.013  # pad tips past the TCP
SQUEEZE_IN = float(os.environ.get("KNOB_SQUEEZE_MM", "8.0")) / 1000.0  # finger target past the knob-contact width
TURN = float(os.environ.get("KNOB_TURN_DEG", "54"))
OPEN_DEG = float(os.environ.get("KNOB_OPEN_DEG", "9.0"))  # door angle the push goes to (100 mm is 7.5 degrees)
UNLATCH_DEG = float(os.environ.get("KNOB_UNLATCH_DEG", "3.4"))  # let go of the knob here (latch edge 45 mm out)


def ready_pose(arm: str) -> tuple[np.ndarray, np.ndarray]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), station.BASES[arm])


def park(arm: str, seconds: float, finger: float = FINGER_OPEN, label: str = "ready pose") -> Waypoint:
    p, r = ready_pose(arm)
    return Waypoint(seconds, tuple(p.tolist()), finger, label, r)


def demo():
    """Both arms wait at the ready pose; the real plan is made on the first control step (knobdemo.py)."""
    return {arm: [park(arm, 0.5, label="look")] for arm in ("left", "right")}


def _grip_rot(d: np.ndarray) -> np.ndarray:
    """Approach along -y (toward the door), fingers closing along ``d`` (perpendicular to y)."""
    z = np.array([0.0, -1.0, 0.0])
    x = np.asarray(d, float) / np.linalg.norm(d)
    return np.column_stack([x, np.cross(z, x), z])


def _turn_cost(p, r0, r1) -> float:
    """IK error of the grip and of the turned grip solved on from it, plus a little for the final wrist roll."""
    from yam_arena import ik, station

    b = station.BASES["right"]
    q0, pe0, re0 = ik.solve(p, ik.matrix_to_quat_xyzw(r0), q_init=[0, 1, 1, -0.6, 0, 0], base_pos=b, iters=400)
    q1, pe1, re1 = ik.solve(p, ik.matrix_to_quat_xyzw(r1), q_init=q0, base_pos=b, iters=400)
    return pe0 + pe1 + 0.02 * (re0 + re1) + 1e-4 * abs(float(q1[5]))


def plan_from_state(st, lay=None) -> dict[str, list[Waypoint]]:
    """The right hand's plan, from the start poses of the door, knob and cover."""
    if FAIL == "fixed":  # wrong on purpose: the reference layout of the setup text, not the poses
        p_axis, cover = np.array(KNOB_POS, float), 0.0
        hinge = np.array([XH, HINGE_Y])
    else:
        p_axis = np.asarray(st["knob"].pos, float)
        cover = knob_deg(st) + cover_deg(st)
        f = st["frame"]
        hinge = (np.asarray(f.pos, float) + f.rotation() @ (np.array([XH, HINGE_Y, KNOB_Z]) - np.array(FRAME_POS)))[:2]
    d0 = _ry(math.radians(cover)) @ np.array([0.0, 0.0, 1.0])  # grip zone A's direction
    y_tcp = p_axis[1] + PAD_YC + TIP - 0.011  # finger pads on the grip zones' inner part, clear of the nose
    grab = np.array([p_axis[0], y_tcp, p_axis[2]])
    # choose the finger axis sign and the turn's direction that the arm reaches best
    best = None
    for sgn in (1.0, -1.0):
        r0 = _grip_rot(sgn * d0)
        for beta in (TURN, -TURN):
            r1 = _ry(math.radians(beta)) @ r0
            cost = _turn_cost(grab, r0, r1)
            if best is None or cost < best[0]:
                best = (cost, r0, beta)
    _, r0, beta = best
    r1 = _ry(math.radians(beta)) @ r0
    contact_half = KNOB_R + (PAD_OUT - PAD_IN)  # each finger's distance from the axis with the zone on the knob
    squeeze = -(contact_half - SQUEEZE_IN)
    if FAIL == "light":
        squeeze = -(PAD_OUT - 0.0015)  # fingers just closed on the grip zones: about 7 N each, no flex to the knob
    open_f = FINGER_OPEN
    pre = grab + np.array([0.0, 0.07, 0.0])
    right = [
        Waypoint(2.0, tuple(pre.tolist()), open_f, "line up with the cover", r0),
        Waypoint(1.5, tuple(grab.tolist()), open_f, "fingers round the grip zones", r0),
        Waypoint(1.0, tuple(grab.tolist()), squeeze, "squeeze", r0),
        Waypoint(0.5, tuple(grab.tolist()), squeeze, "hold the squeeze", r0),
        Waypoint(2.5, tuple(grab.tolist()), squeeze, "turn", r1),
        Waypoint(0.5, tuple(grab.tolist()), squeeze, "hold the turn", r1),
    ]
    if FAIL == "early":  # let the knob turn back before pushing
        right.append(Waypoint(1.5, tuple(grab.tolist()), squeeze, "turn back", r0))
        r1 = r0
    if FAIL == "push":  # wrong on purpose: no turn at all
        right = right[:4]
        r1 = r0
    def on_door(point, deg):  # a point that turns with the door, ``deg`` open
        return np.r_[hinge + _rz(math.radians(deg))[:2, :2] @ (np.asarray(point)[:2] - hinge), point[2]]

    # 1) still squeezing and turned, push until the bolt is past the strike (the latch edge about 45 mm out)
    for i in range(1, 5):
        deg = UNLATCH_DEG * i / 4
        right.append(Waypoint(0.6, tuple(on_door(grab, deg).tolist()), squeeze, "push past the strike",
                              _rz(math.radians(deg)) @ r1))
    rz = _rz(math.radians(UNLATCH_DEG))
    at = on_door(grab, UNLATCH_DEG)
    right.append(Waypoint(0.8, tuple(at.tolist()), open_f, "let go", rz @ r1))
    out = at + rz @ np.array([0.0, 0.06, 0.0])
    right.append(Waypoint(0.8, tuple(out.tolist()), open_f, "back off the cover", rz @ r1))
    # 2) closed fingers, upright, on the middle of the cover's front: push the door the rest of the way
    rv = _grip_rot(np.array([0.0, 0.0, 1.0]))
    nose = np.array([p_axis[0], p_axis[1] + NOSE_Y[1] + TIP - 0.004, p_axis[2]])  # 4 mm of press
    right.append(Waypoint(0.8, tuple((on_door(nose, UNLATCH_DEG) + rz @ np.array([0.0, 0.03, 0.0])).tolist()),
                          0.0, "close the fingers in front of the cover", rz @ rv))
    for i in range(0, 7):
        deg = UNLATCH_DEG + (OPEN_DEG - UNLATCH_DEG) * i / 6
        right.append(Waypoint(0.6 if i else 0.8, tuple(on_door(nose, deg).tolist()), 0.0, "push the door open",
                              _rz(math.radians(deg)) @ rv))
    rz = _rz(math.radians(OPEN_DEG))
    back = np.asarray(right[-1].pos) + rz @ np.array([0.0, 0.08, 0.0])
    right.append(Waypoint(1.0, tuple(back.tolist()), 0.0, "back off", rz @ rv))
    right.append(park("right", 2.0, label="back to the ready pose"))
    total = sum(w.duration for w in right)
    print(f"knobdemo: knob axis ({p_axis[0]:.4f}, {p_axis[1]:.4f}, {p_axis[2]:.4f}), cover {cover:.1f} deg, "
          f"turn {beta:+.0f} deg, plan {total:.1f} s", flush=True)
    return {"left": [park("left", total, label="wait")], "right": right}


import knobdemo  # noqa: E402  (the demo's control hooks for yam_scripted)

knobdemo.install()
knobdemo_task = True
