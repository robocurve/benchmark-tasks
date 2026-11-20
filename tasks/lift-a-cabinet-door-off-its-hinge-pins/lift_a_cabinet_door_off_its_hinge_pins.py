"""On the station table: a small wall cabinet hangs on a plywood wall panel at the right of the bench, its bottom 25 mm
above the bench top, its open side facing left. Its door (30 x 20 cm, 18 mm painted pine, 0.55 kg) stands open 90 deg,
hung on two 2 in. (51 mm) steel loose-pin butt hinges. The bottom hinge's pin has already been taken out and lies in a
steel parts dish; the top hinge's ball-tipped pin is still in, so the door hangs on that one hinge. Two 1x2 pine
battens (19 x 38 mm, 32 cm long) are taped flat on the bench in front of the cabinet. Task (instruction): take the door
off its hinges and lay it flat across the two battens, with the hinge pin in the dish. The instruction does not say how.

How it is done (Residence Supply, "How to Remove a Door from Its Hinges"): "Working from the bottom up keeps the top
hinge carrying the load until the very end." "Remove the top pin last." "With the bottom (and middle) pins out, hold
the door with one hand while tapping the top pin free." Then the door is lifted off and laid down flat.

The hand skill: with one pin left, the door hangs on a single hinge, its weight 15 cm out from the hinge axis, so that
pin carries the door's turning load. Pulled with nobody holding the door, it binds in the knuckles and the ball tip
slips out of the fingers. One hand takes the door's weight at its top edge while the other draws the pin straight up out
of the five knuckles (51 mm) by its 11 mm ball tip. Once it is out, the door's knuckles only sit on the frame's: a door
let go of swings down off level on them and drops onto the bench. The hands then lift the door clear together, stand its bottom edge
on the far batten, lower it face down onto both battens, turning their grips with it, and draw their fingers out.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475. Heights
below are above the table top.
- cabinet (fixed): carcass 30 cm (x) x 15 cm (y) x 20 cm tall, 15 mm panels, from x 0.2435 to 0.5435, front edge at
  y -0.125, bottom at z 0.025, on a plywood wall panel behind it (y -0.293 to -0.275). The two hinges' frame knuckles
  (1, 3 and 5 of 5) on its near side panel's front edge; hinge axis vertical at (0.236, -0.118). Top hinge from z 0.159
  to 0.210, bottom hinge from z 0.050 to 0.101.
- door (loose, 0.55 kg): 30 cm along y (y -0.111 to 0.189), 20 cm tall (z 0.025 to 0.225), 18 mm thick (x 0.211 to
  0.229), with the door knuckles (2 and 4 of 5) of both hinges; knuckles 8 mm across, bores 4.4 mm square round a pin
  shank 3.8 mm square (0.3 mm clearance a side).
- top hinge pin (loose, 8 g): 51 mm shank in the top hinge's five knuckles, 11 mm ball tip sitting on the top knuckle.
- bottom hinge pin (loose, 8 g): lying in the parts dish.
- parts dish (loose, 0.20 kg): steel, 12 cm across, 15 mm deep, centre (0.130, -0.190).
- battens (fixed: taped): 19 mm tall, 38 mm wide, 32 cm long along y (y -0.105 to 0.215), centres x 0.119 and 0.262.

Score (0 for the rest of the run once the door falls):
- 0: the door fell: its centre moved down faster than 0.30 m/s at any moment, or its lowest point hit the bench top
  moving down faster than 0.10 m/s (a door lowered by hand moves far slower and goes onto the battens; a door let go of
  once the pin is out swings down off level on its knuckles and its free corner hits the bench at about 0.3 m/s).
- otherwise the best stage reached: 0.25 the top pin out of the hinge (its lower end above the top knuckle); 0.5 the door
  also off the hinges (its knuckles 15 mm or more from the hinge axis); 0.75 the door also lying flat across both
  battens (face within 5 deg of level, its underside within 3 mm of the battens' tops, overlapping each by 15 mm or more).
- 1.0 (success, held 1 s): the door lying flat across both battens, both hinge pins in the dish, every finger point 2 cm
  or more from the door, and the door still (under 1 mm in 1 s).
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, on_reset  # noqa: E402
from bodykit import on_step, rotation  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
MM = 0.001

# -- the layout ----------------------------------------------------------------------------------------------------
X_AX, Y_AX = 0.236, -0.118  # the hinge axis (vertical)
KN = 0.004  # knuckle half-width (8 mm knuckles)
BORE = float(os.environ.get("DOOR_BORE", "0.0022"))  # bore half-width (4.4 mm: 0.3 mm clearance a side)
PIN_H = 0.0019  # pin shank half-width (3.8 mm)
HL = 0.0508  # 2 in. hinge
KL = HL / 5  # five knuckles
KGAP = 0.0003
GAP = 0.003  # knuckle edge to door face / door edge (the leaf's room)
DOOR_W, DOOR_H, DOOR_T = 0.300, 0.200, 0.018
ZD0 = float(os.environ.get("DOOR_ZD0", "0.025"))  # the door's (and the carcass's) bottom above the bench
DOOR_X0 = X_AX - KN - GAP - DOOR_T  # the door's outside face (facing the robot)
DOOR_Y0 = Y_AX + KN + GAP  # the door's hinge edge
DOOR_C = np.array([DOOR_X0 + DOOR_T / 2, DOOR_Y0 + DOOR_W / 2, ZD0 + DOOR_H / 2])
DOOR_MASS = 0.55
H_TOP = ZD0 + DOOR_H - 0.015 - HL  # the top hinge's lower end
H_BOT = ZD0 + 0.025  # the bottom hinge's lower end
BALL_R = 0.0055
PIN_MASS = 0.008
PIN_BOT0 = H_TOP + 0.0003  # the top pin's lower end, seated
BALL0 = H_TOP + HL + 0.0002 + BALL_R  # the ball's centre, seated
CAB_Y0 = Y_AX - KN - GAP  # the carcass's front edge
CAB_X0 = X_AX + KN + GAP + 0.0005  # the carcass's near side
CAB_W, CAB_D, PANEL = 0.300, 0.150, 0.015
WALL_Y = CAB_Y0 - CAB_D  # the wall panel's face
DISH_C = (0.130, -0.190)
DISH_R, DISH_H = 0.060, 0.015
BAT_T, BAT_W, BAT_L = 0.019, 0.038, 0.320
BAT_X = (0.119, 0.262)
BAT_Y = 0.055

PINE, WHITE, STEEL, BRASS, PLY, WALLC, DARK, BAT = (
    (0.80, 0.66, 0.45), (0.93, 0.92, 0.88), (0.70, 0.72, 0.75), (0.72, 0.60, 0.30), (0.78, 0.68, 0.52),
    (0.82, 0.82, 0.80), (0.25, 0.25, 0.27), (0.86, 0.74, 0.52))


def _v(p) -> tuple:
    return tuple(float(x) for x in p)


def _ring(cx, cy, z0, z1, color, outer=KN, bore=BORE) -> list:
    """A knuckle: four walls round a square bore, plus a round visual sleeve (body frame)."""
    h, zc = z1 - z0, (z0 + z1) / 2
    w = outer - bore
    out = [BoxShape((cx + bore + w / 2, cy, zc), (w, 2 * outer, h), color),
           BoxShape((cx - bore - w / 2, cy, zc), (w, 2 * outer, h), color),
           BoxShape((cx, cy + bore + w / 2, zc), (2 * bore, w, h), color),
           BoxShape((cx, cy - bore - w / 2, zc), (2 * bore, w, h), color),
           CylShape((cx, cy, zc), outer + 0.0002, h, color, collide=False)]
    return out


def _knuckles(h0: float, which, origin) -> list:
    out = []
    for k in which:
        z0 = h0 + k * KL + KGAP / 2
        out += _ring(X_AX - origin[0], Y_AX - origin[1], z0 - origin[2], z0 + KL - KGAP - origin[2], STEEL)
    return out


def _door() -> Body:
    o = DOOR_C + np.array([0.0, 0.0, TOP])
    rel = lambda p: _v(np.asarray(p, float) - DOOR_C)  # noqa: E731
    sh = [BoxShape(rel(DOOR_C), (DOOR_T, DOOR_W, DOOR_H), WHITE),
          # a routed groove round the face, drawn on the outside face (visual)
          BoxShape(rel((DOOR_X0 - 0.0002, DOOR_C[1], ZD0 + 0.025)), (0.0004, DOOR_W - 0.05, 0.003), (0.80, 0.80, 0.77),
                   collide=False),
          BoxShape(rel((DOOR_X0 - 0.0002, DOOR_C[1], ZD0 + DOOR_H - 0.025)), (0.0004, DOOR_W - 0.05, 0.003),
                   (0.80, 0.80, 0.77), collide=False),
          BoxShape(rel((DOOR_X0 - 0.0002, DOOR_Y0 + 0.025, DOOR_C[2])), (0.0004, 0.003, DOOR_H - 0.05),
                   (0.80, 0.80, 0.77), collide=False),
          BoxShape(rel((DOOR_X0 - 0.0002, DOOR_Y0 + DOOR_W - 0.025, DOOR_C[2])), (0.0004, 0.003, DOOR_H - 0.05),
                   (0.80, 0.80, 0.77), collide=False)]
    for h0 in (H_TOP, H_BOT):
        # the door leaf: on the door's hinge edge, joined to the knuckles (visual), and the knuckles 2 and 4
        sh.append(BoxShape(rel((X_AX - KN - GAP / 2 - 0.0045, Y_AX + KN + 0.0012, h0 + HL / 2)), (0.016, 0.0015, HL),
                           STEEL, collide=False))
        sh.append(BoxShape(rel((X_AX - KN - 0.0015, Y_AX + KN + 0.0010, h0 + HL / 2)), (0.003, 0.002, HL), STEEL,
                           collide=False))
        sh += _knuckles(h0, (1, 3), DOOR_C)
    return Body.build("door", pos=_v(o), mass=DOOR_MASS, color=WHITE, friction=(0.5, 0.4), shapes=sh,
                      contact_offset=0.001, solver_iterations=48)


def _cabinet() -> Body:
    c = np.array([CAB_X0 + CAB_W / 2, CAB_Y0 - CAB_D / 2, ZD0 + DOOR_H / 2])  # carcass centre
    rel = lambda p: _v(np.asarray(p, float) - c)  # noqa: E731
    x0, x1, y0, y1, z0, z1 = CAB_X0, CAB_X0 + CAB_W, CAB_Y0 - CAB_D, CAB_Y0, ZD0, ZD0 + DOOR_H
    sh = [BoxShape(rel((x0 + PANEL / 2, (y0 + y1) / 2, (z0 + z1) / 2)), (PANEL, CAB_D, DOOR_H), WHITE),
          BoxShape(rel((x1 - PANEL / 2, (y0 + y1) / 2, (z0 + z1) / 2)), (PANEL, CAB_D, DOOR_H), WHITE),
          BoxShape(rel(((x0 + x1) / 2, (y0 + y1) / 2, z0 + PANEL / 2)), (CAB_W - 2 * PANEL, CAB_D, PANEL), WHITE),
          BoxShape(rel(((x0 + x1) / 2, (y0 + y1) / 2, z1 - PANEL / 2)), (CAB_W - 2 * PANEL, CAB_D, PANEL), WHITE),
          BoxShape(rel(((x0 + x1) / 2, y0 + 0.003, (z0 + z1) / 2)), (CAB_W - 2 * PANEL, 0.006, DOOR_H), WHITE),
          BoxShape(rel(((x0 + x1) / 2, (y0 + y1) / 2 + 0.005, (z0 + z1) / 2)), (CAB_W - 2 * PANEL, CAB_D - 0.02,
                                                                                0.012), WHITE),  # the shelf
          # the plywood wall panel the cabinet hangs on, standing on the bench
          BoxShape(rel(((x0 + x1) / 2 - 0.01, WALL_Y - 0.009, 0.16)), (CAB_W + 0.10, 0.018, 0.32), WALLC)]
    for h0 in (H_TOP, H_BOT):
        # the frame leaf on the near side panel's front edge (visual), and the knuckles 1, 3 and 5
        sh.append(BoxShape(rel((X_AX + KN + GAP / 2 + 0.006, Y_AX - KN - 0.0012, h0 + HL / 2)), (0.018, 0.0015, HL),
                           STEEL, collide=False))
        sh.append(BoxShape(rel((X_AX + KN + 0.0015, Y_AX - KN - 0.0010, h0 + HL / 2)), (0.003, 0.002, HL), STEEL,
                           collide=False))
        sh += _knuckles(h0, (0, 2, 4), c)
    return Body.build("cabinet", pos=_v(c + np.array([0.0, 0.0, TOP])), mass=5.0, color=WHITE, fixed=True,
                      shapes=sh, friction=(0.5, 0.4), contact_offset=0.001)


def _pin(name: str, pos, lying: bool) -> Body:
    """A loose hinge pin: a 3.8 mm square-cored shank (round sleeve shown) 51 mm long with an 11 mm ball tip. Its
    origin is the ball's centre; lying, its shank points along +x."""
    if lying:
        sh = [BoxShape((-BALL_R - HL / 2, 0.0, 0.0), (HL, 2 * PIN_H, 2 * PIN_H), STEEL),
              CylShape((-BALL_R - HL / 2, 0.0, 0.0), 0.002, HL, STEEL, collide=False, axis="X"),
              SphereShape((0.0, 0.0, 0.0), BALL_R, BRASS)]
        quat = (0.0, 0.0, 1.0, 0.0)  # turned half round, so the shank points along +x
    else:
        sh = [BoxShape((0.0, 0.0, -BALL_R - HL / 2), (2 * PIN_H, 2 * PIN_H, HL), STEEL),
              CylShape((0.0, 0.0, -BALL_R - HL / 2), 0.002, HL, STEEL, collide=False),
              SphereShape((0.0, 0.0, 0.0), BALL_R, BRASS)]
        quat = None
    return Body.build(name, pos=_v(pos), mass=PIN_MASS, color=STEEL, shapes=sh, friction=(0.45, 0.35),
                      contact_offset=0.001, solver_iterations=48, init_quat=quat, damping=(0.0, 0.2))


def _dish() -> Body:
    n = 16
    sh = [CylShape((0.0, 0.0, 0.001), DISH_R, 0.002, STEEL)]
    for k in range(n):
        a = 2 * math.pi * k / n
        sh.append(BoxShape(((DISH_R - 0.001) * math.cos(a), (DISH_R - 0.001) * math.sin(a), DISH_H / 2),
                           (0.002, 2 * DISH_R * math.sin(math.pi / n) + 0.001, DISH_H), STEEL, yaw=a))
    return Body.build("dish", pos=(DISH_C[0], DISH_C[1], TOP + 0.0005), mass=0.20, color=STEEL, shapes=sh,
                      friction=(0.5, 0.4), contact_offset=0.001)


def _batten(name: str, x: float) -> Body:
    sh = [BoxShape((0.0, 0.0, 0.0), (BAT_W, BAT_L, BAT_T), BAT),
          BoxShape((0.0, BAT_L / 2 - 0.02, BAT_T / 2 + 0.0001), (BAT_W, 0.02, 0.0002), (0.85, 0.2, 0.15), collide=False),
          BoxShape((0.0, -BAT_L / 2 + 0.02, BAT_T / 2 + 0.0001), (BAT_W, 0.02, 0.0002), (0.85, 0.2, 0.15),
                   collide=False)]
    return Body.build(name, pos=(x, BAT_Y, TOP + BAT_T / 2), mass=0.12, color=BAT, shapes=sh, fixed=True,
                      friction=(0.5, 0.4), contact_offset=0.001)


CABINET = _cabinet()
DOOR = _door()
if os.environ.get("DOOR_PINOUT") == "1":  # a check only: the top pin starts out of the hinge, lying on the bench
    PIN = _pin("pin", (0.10, -0.10, TOP + PIN_H + 0.001), lying=True)
else:
    PIN = _pin("pin", (X_AX, Y_AX, TOP + BALL0), lying=False)
DISH = _dish()
PIN2 = _pin("pin2", (DISH_C[0] - 0.03, DISH_C[1] + 0.015, TOP + 0.0025 + BALL_R + 0.001), lying=True)
BATTENS = [_batten("batten_far", BAT_X[1]), _batten("batten_near", BAT_X[0])]
OBJECTS = [CABINET, DOOR, PIN, DISH, PIN2] + BATTENS

# -- tracking and scoring -------------------------------------------------------------------------------------------
FALL_V = 0.30  # m/s: the door's centre moving down faster than this: it fell
HIT_V = 0.10  # m/s: the door's lowest point reaching the bench top moving down faster than this: it dropped on it
BENCH_TOUCH = 0.001
OFF = 0.015  # the door's knuckles this far from the hinge axis: off the hinges
FLAT_DEG = 5.0
ON_BATTEN = 0.003
SPAN = 0.015  # the door overlaps each batten by this much or more
CLEAR = 0.02
STILL_STEPS, STILL_MM = 20, 1.0
_DEBUG = os.environ.get("DOOR_DEBUG") == "1"
_DEBUG_EVERY = int(os.environ.get("DOOR_DEBUG_EVERY", "5"))
_EP: dict = {}
_IDS: dict = {}
KNUCKLE_REL = np.array([X_AX, Y_AX, H_TOP + HL / 2]) - DOOR_C  # the top hinge's axis point, in the door's frame
HALF = np.array([DOOR_T, DOOR_W, DOOR_H]) / 2


@on_reset
def _reset() -> None:
    _EP.update(tick=0, step=0, counted=-1, best=0.0, failed="", recent=[], prev_z=None, vz=0.0, min_vz=0.0,
               pin_rise=0.0, pin_out=False, off=False, flat=False, near=1.0, tilt=90.0, kn_off=0.0, low=0.0, min_lowv=0.0,
               in_dish=0, tcp={}, gaps={}, door=DOOR_C.copy())


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


def _in_dish(p, dish_p) -> bool:
    d = np.asarray(p) - dish_p
    return bool(math.hypot(d[0], d[1]) < DISH_R - 0.004 and d[2] < DISH_H)


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("DOOR_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _track_inner(env) -> None:
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
        fingers += [_IDS["fingers"][n] @ rotation(poses[i][3:7]).T + poses[i][:3] - origin
                    for n, i in zip(("tip_left", "tip_right"), _IDS[side + "_tips"])]
    pose = lambda name: as_numpy(base.scene[name].data.root_link_pose_w)[0]  # noqa: E731
    d = pose("door")
    dp, dr = d[:3] - origin, rotation(d[3:7])
    p = pose("pin")
    pp, pr = p[:3] - origin, rotation(p[3:7])
    q2 = pose("pin2")
    p2 = q2[:3] - origin
    ds = pose("dish")
    dish_p = ds[:3] - origin
    # the door's centre falling
    if _EP["prev_z"] is not None:
        _EP["vz"] = (dp[2] - _EP["prev_z"]) / 0.05
        _EP["min_vz"] = min(_EP["min_vz"], _EP["vz"])
    _EP["prev_z"] = float(dp[2])
    # the pin: its lower end (the shank's end) above the top knuckle
    pin_end = pp + pr @ np.array([0.0, 0.0, -BALL_R - HL])
    rise = float(pin_end[2] - TOP - PIN_BOT0)
    pin_out = bool(pin_end[2] - TOP > H_TOP + HL + 0.001 or math.hypot(pin_end[0] - X_AX, pin_end[1] - Y_AX) > 0.01)
    # the door's knuckles from the hinge axis
    kn = dp + dr @ KNUCKLE_REL
    kn_off = math.hypot(kn[0] - X_AX, kn[1] - Y_AX)
    # flat across both battens: the face normal (door's x axis) near vertical, underside on both battens
    tilt = math.degrees(math.acos(min(1.0, abs(float(dr[2, 0])))))
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * HALF
    cw = dp + corners @ dr.T
    low = float(cw[:, 2].min() - TOP)
    xs = (float(cw[:, 0].min()), float(cw[:, 0].max()))
    spans = all(min(xs[1], bx + BAT_W / 2) - max(xs[0], bx - BAT_W / 2) >= SPAN for bx in BAT_X) and \
        float(cw[:, 1].min()) <= BAT_Y + BAT_L / 2 and float(cw[:, 1].max()) >= BAT_Y - BAT_L / 2
    flat = bool(tilt <= FLAT_DEG and abs(low - BAT_T) <= ON_BATTEN and spans)
    # fingers' clearance from the door (distance to its box)
    pts = np.concatenate(fingers)
    loc = (pts - dp) @ dr
    out = np.maximum(np.abs(loc) - HALF, 0.0)
    near = float(np.sqrt((out ** 2).sum(1)).min())
    mid = lambda c, r: c + r @ np.array([0.0, 0.0, -BALL_R - HL / 2])  # noqa: E731  (the shank's middle)
    in_dish = int(_in_dish(mid(pp, pr), dish_p)) + int(_in_dish(mid(p2, rotation(q2[3:7])), dish_p))
    low_v = (low - _EP["low"]) / 0.05 if _EP["tick"] > 1 else 0.0  # the door's lowest point's vertical speed
    _EP["min_lowv"] = min(_EP.get("min_lowv", 0.0), low_v)
    if not _EP["failed"] and _EP["vz"] < -FALL_V:
        _EP["failed"] = f"the door fell (its centre moving down at {-_EP['vz']:.2f} m/s)"
    if not _EP["failed"] and low <= BENCH_TOUCH and low_v < -HIT_V:
        _EP["failed"] = f"the door dropped onto the bench (its lowest point hit it at {-low_v:.2f} m/s)"
    _EP["pin_out"] = _EP["pin_out"] or pin_out
    _EP["off"] = _EP["off"] or (_EP["pin_out"] and kn_off >= OFF)
    stage = 0.0
    if _EP["pin_out"]:
        stage = 0.25
        if _EP["off"]:
            stage = 0.5
            if flat:
                stage = 0.75
    _EP["best"] = max(_EP["best"], stage)
    _EP.update(pin_rise=rise, flat=flat, near=near, tilt=tilt, kn_off=kn_off, low=low, in_dish=in_dish, door=dp,
               pin=pp, pin2=p2)


def _tally(o) -> dict:
    recent = _EP["recent"]
    if _EP["counted"] != _EP["tick"]:
        _EP["counted"] = _EP["tick"]
        recent.append(np.asarray(_EP["door"], float).copy())
        del recent[:-STILL_STEPS]
    still = len(recent) == STILL_STEPS and all(np.abs(p - recent[-1]).max() * 1000 < STILL_MM for p in recent)
    done = bool(_EP["off"] and _EP["flat"] and _EP["in_dish"] == 2 and _EP["near"] >= CLEAR and still)
    return dict(still=still, done=done)


def _debug(t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    dp = _EP["door"]
    print(f"DBG step={_EP['step']} vz={_EP['vz']:+.3f} minvz={_EP['min_vz']:+.3f} pin_rise={_EP['pin_rise'] * 1000:.1f}mm "
          f"pin_out={int(_EP['pin_out'])} kn_off={_EP['kn_off'] * 1000:.1f}mm off={int(_EP['off'])} "
          f"lowv_min={_EP['min_lowv']:+.3f} door=({dp[0]:.3f},{dp[1]:.3f},{dp[2] - TOP:.3f}) tilt={_EP['tilt']:.1f} low={_EP['low'] * 1000:.1f}mm "
          f"flat={int(_EP['flat'])} dish={_EP['in_dish']} near={_EP['near'] * 1000:.0f}mm still={int(t['still'])} "
          f"best={_EP['best']} failed={_EP['failed'] or '-'} | "
          + " ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{v[2] - TOP:.3f}) gap={_EP['gaps'][k] * 1000:.0f}"
                     for k, v in _EP["tcp"].items()), flush=True)


def score(objects) -> float:
    """0 for the rest of the run once the door has fallen or dropped onto the bench; otherwise the best stage reached (0.25 pin out, 0.5 door
    off the hinges, 0.75 door flat across both battens). Full score is success()."""
    t = _tally(objects)
    if _DEBUG:
        _debug(t)
    if _EP["failed"]:
        return 0.0
    return _EP["best"]


def success(objects) -> bool:
    t = _tally(objects)
    return bool(t["done"] and not _EP["failed"])


# -- scripted demo --------------------------------------------------------------------------------------------------
OPEN_C = -0.047
MODE = os.environ.get("DOOR_MODE", "demo")  # demo | nohold, letgo (wrong ways, see README) | stay


def _rax(axis, a: float) -> np.ndarray:
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]])
    return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * (k @ k)


D1 = np.array([1.0, 1.0, 0.0]) / math.sqrt(2)  # across the hinge corner (from outside the cabinet to the door's inside)
D2 = np.array([1.0, -1.0, 0.0]) / math.sqrt(2)
PIN_GRIP = _rax(D2, math.radians(float(os.environ.get("DOOR_PIN_PITCH", "20")))) @ grip_rotation(3 * math.pi / 4)
HOLD_TILT = math.radians(float(os.environ.get("DOOR_HOLD_TILT", "20")))
# the hands on the door's top edge: fingers closing along x (across its thickness), leaning in the door's plane with the
# wrists outboard (the left hand's fingers point down and to the right, the right hand's down and to the left)
DOWN_L = _rax((1.0, 0.0, 0.0), -HOLD_TILT) @ grip_rotation(-math.pi / 2)
DOWN_R = _rax((1.0, 0.0, 0.0), HOLD_TILT) @ grip_rotation(math.pi / 2)
PIN_PINCH = float(os.environ.get("DOOR_PIN_PINCH", "-0.004"))
DOOR_PINCH = float(os.environ.get("DOOR_PINCH", "-0.002"))  # 18 mm door: commanded 10 mm gap
PIN_DZ = float(os.environ.get("DOOR_PIN_DZ", "0.0095"))  # TCP above the ball's centre
EDGE_DZ = float(os.environ.get("DOOR_EDGE_DZ", "0.012"))  # TCP below the door's top edge
HOLD_Y = {"left": float(os.environ.get("DOOR_HOLD_YL", "0.100")), "right": float(os.environ.get("DOOR_HOLD_YR", "-0.070"))}
PULL = float(os.environ.get("DOOR_PULL", "0.058"))  # how far the pin is drawn up
LIFT = float(os.environ.get("DOOR_LIFT", "0.020"))
TAKE = float(os.environ.get("DOOR_TAKE", "0.006"))  # the holding hand raised this far: the door's weight off the pin
SHIFT_Y = float(os.environ.get("DOOR_SHIFT_Y", "0.016"))
X_PIVOT = float(os.environ.get("DOOR_X_PIVOT", "0.268"))  # the door's outside face at the far batten


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


def _edge(y: float, dz: float = 0.0, dx: float = 0.0, dy: float = 0.0) -> np.ndarray:
    """The TCP on the hanging door's top edge at ``y`` (mid-thickness, EDGE_DZ below the top)."""
    return np.array([DOOR_C[0] + dx, y + dy, TOP + ZD0 + DOOR_H - EDGE_DZ + dz])


def _tip_pose(arm: str, beta: float):
    """The grip on the top edge with the door stood on the far batten and tipped ``beta`` (rad) toward the robot
    about its outside bottom edge."""
    pivot = np.array([X_PIVOT, 0.0, TOP + BAT_T])
    r = np.array([DOOR_T / 2, 0.0, DOOR_H - EDGE_DZ])
    c, s = math.cos(beta), math.sin(beta)
    off = np.array([r[0] * c - r[2] * s, 0.0, r[0] * s + r[2] * c])
    p = pivot + off
    p[1] = HOLD_Y[arm] + SHIFT_Y
    rot = _rax((0.0, 1.0, 0.0), -beta) @ (DOWN_L if arm == "left" else DOWN_R)
    return p, rot


T_PIN = 9.0  # s: when the left hand holds the door and the right hand starts on the pin


def _left_path() -> list:
    y = HOLD_Y["left"]
    out = _lead_in("left", _edge(y, 0.04), DOWN_L, OPEN_C, "above the door's top edge, open")
    out += [_w(2.5, _edge(y), OPEN_C, "fingers down either side of the top edge", DOWN_L),
            _w(1.0, _edge(y), DOOR_PINCH, "grip the top edge", DOWN_L),
            _w(1.5, _edge(y, TAKE), DOOR_PINCH, "take the door's weight off the pin", DOWN_L),
            _w(3.0 + 22.0, _edge(y, TAKE), DOOR_PINCH, "hold the door while the pin comes out", DOWN_L)]
    if MODE == "letgo":  # wrong: let go of the door once the pin is out, before taking it off the knuckles
        out += [_w(1.0, _edge(y, TAKE), OPEN_C, "let go of the door", DOWN_L),
                _w(2.0, _edge(y, 0.03), OPEN_C, "rise", DOWN_L),
                _w(15.0, _edge(y, 0.03), OPEN_C, "watch", DOWN_L)]
        return out
    out += _carry("left")
    return out


def _right_path() -> list:
    ball = np.array([X_AX, Y_AX, TOP + BALL0])
    at = ball + np.array([0.0, 0.0, PIN_DZ])
    out = _lead_in("right", at + np.array([-0.02, -0.02, 0.05]), PIN_GRIP, OPEN_C, "above the pin's ball tip, open")
    out += [_w(T_PIN - 3.0, at + np.array([-0.02, -0.02, 0.05]), OPEN_C, "wait for the door to be held", PIN_GRIP),
            _w(2.0, at + np.array([0.0, 0.0, 0.02]), OPEN_C, "over the ball tip", PIN_GRIP),
            _w(1.5, at, OPEN_C, "fingers round the ball tip", PIN_GRIP),
            _w(1.5, at, OPEN_C, "settle", PIN_GRIP),
            _w(0.8, at, -0.012, "close in on the ball tip", PIN_GRIP),
            _w(1.2, at, PIN_PINCH, "pinch the ball tip", PIN_GRIP),
            _w(4.0, at + np.array([0.0, 0.0, PULL]), PIN_PINCH, "draw the pin straight up out of the knuckles",
               PIN_GRIP),
            _w(0.5, at + np.array([0.0, 0.0, PULL]), PIN_PINCH, "settle", PIN_GRIP)]
    if MODE == "nohold":
        out += [_w(20.0, at + np.array([-0.03, -0.03, PULL]), PIN_PINCH, "hold the pin", PIN_GRIP)]
        return out
    dish = np.array([DISH_C[0], DISH_C[1], TOP + 0.06])
    out += [_w(3.0, dish + np.array([0.0, 0.0, 0.03]), PIN_PINCH, "carry the pin over the dish", PIN_GRIP),
            _w(1.5, dish, PIN_PINCH, "lower it into the dish", PIN_GRIP),
            _w(1.0, dish, OPEN_C, "let go of it", PIN_GRIP),
            _w(1.0, dish + np.array([0.0, 0.0, 0.04]), OPEN_C, "rise", PIN_GRIP)]
    if MODE == "letgo":
        out += [_w(20.0, dish + np.array([0.0, 0.0, 0.04]), OPEN_C, "watch", PIN_GRIP)]
        return out
    y = HOLD_Y["right"]
    out += [_w(1.5, np.array([0.150, -0.110, TOP + 0.150]), -0.012, "back up, clear of the door's face", DOWN_R),
            _w(2.0, np.array([0.170, y, TOP + 0.255]), -0.012, "up over the top edge's height", DOWN_R),
            _w(2.0, _edge(y, 0.04), -0.012, "above the door's top edge by the hinge", DOWN_R),
            _w(0.8, _edge(y, 0.04), OPEN_C, "open", DOWN_R),
            _w(2.0, _edge(y), OPEN_C, "fingers down either side of the top edge", DOWN_R),
            _w(1.0, _edge(y), DOOR_PINCH, "grip the top edge", DOWN_R),
            _w(1.0, _edge(y, 0.002), DOOR_PINCH, "take the door's weight", DOWN_R)]
    out += _carry("right")
    return out


def _carry(arm: str) -> list:
    """Both hands: lift the door clear of the knuckles, carry it out over the far batten, stand it on that batten and
    lower it face down onto both battens, then draw the fingers out from under its edge. Times are matched between
    the arms by the waypoints' durations (the left arm's hold is padded to end when the right arm has the door)."""
    y = HOLD_Y[arm]
    rot = DOWN_L if arm == "left" else DOWN_R
    up = _edge(y, LIFT)
    side = _edge(y, LIFT, dy=SHIFT_Y)
    over = np.array([X_PIVOT + DOOR_T / 2, y + SHIFT_Y, TOP + BAT_T + DOOR_H - EDGE_DZ + 0.012])
    p0, _ = _tip_pose(arm, 0.0)
    out = [_w(2.0, up, DOOR_PINCH, "lift the door off the knuckles' seats", rot),
           _w(2.0, side, DOOR_PINCH, "draw it off the hinge axis", rot),
           _w(4.0, over, DOOR_PINCH, "carry it out over the far batten", rot),
           _w(2.0, p0 + np.array([0.0, 0.0, -0.002]), DOOR_PINCH, "stand it on the far batten", rot),
           _w(0.5, p0 + np.array([0.0, 0.0, -0.002]), DOOR_PINCH, "settle", rot)]
    n = 18
    for k in range(1, n + 1):
        b = math.radians(90.0) * k / n
        p, r = _tip_pose(arm, b)
        out.append(_w(7.0 / n, p + np.array([0.0, 0.0, -0.002]), DOOR_PINCH, "lower it face down onto the battens", r))
    p, r = _tip_pose(arm, math.radians(90.0))
    back = r @ np.array([0.0, 0.0, -float(os.environ.get("DOOR_BACK", "0.040"))])  # along the fingers, out from under the edge
    out += [_w(1.0, p, DOOR_PINCH, "settle", r),
            _w(1.0, p, OPEN_C, "let go", r),
            _w(2.0, p + back, OPEN_C, "draw the fingers out", r),
            _w(2.0, p + back + np.array([0.0, 0.0, 0.07]), OPEN_C, "rise clear", r),
            _w(1.0, p + back + np.array([0.0, 0.0, 0.07]), -0.015, "close the empty fingers", r),
            _w(2.0, p + back + np.array([0.0, 0.0, 0.11]), -0.015, "rise higher", r),
            _w(6.0, p + back + np.array([0.0, 0.0, 0.11]), -0.015, "stay clear", r)]
    return out


def demo():
    if MODE == "stay":
        return {"left": _stay("left"), "right": _stay("right")}
    if MODE == "nohold":
        return {"left": _stay("left"), "right": _right_path()}
    if MODE == "letgo":
        return {"left": _left_path(), "right": _right_path()}
    left = _left_path()
    right = _right_path()
    # pad the left arm's hold so both arms start the carry together
    tl = sum(w.duration for w in left[:-len(_carry("left"))])
    tr = sum(w.duration for w in right[:-len(_carry("right"))])
    hold_i = len(left) - len(_carry("left")) - 1
    w = left[hold_i]
    left[hold_i] = Waypoint(w.duration + (tr - tl), w.pos, w.finger, w.label, w.yaw)
    return {"left": left, "right": right}
