"""On the station table stands a beech mitre box: a channel between a front wall and a back fence, with saw slots cut
through both walls at 90 degrees and at 45 degrees either way, on a long base board. In front of it lies a length of
black-lacquered pine picture-frame moulding with a gold lip, a pencil line across it at 45 degrees near its right end.
To the right a backsaw (a steel blade with a brass back and a turned handle) stands in its wooden rest, blade down.

Task: saw the moulding right through at its pencil line in the mitre box's 45-degree slot, the moulding against the
box's back fence; leave the saw standing back in its rest, the cut moulding lying on the table in front of the box and
both arms back where they started. The moulding is light and slides in the box: the saw's strokes drag it along with
the blade unless it is held. The wood is not cut in simulation: counted strokes stand in for the kerf's depth (real
trials measure it; see task.json).

Run it:  inspect-robots run --embodiment yam_arena -E task=saw_a_mitre_in_a_mitre_box_holding_the_moulding_to_the_fence.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402,F401
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, glue, on_reset, on_step,  # noqa: E402
                     rotation, set_glue_on, set_slider_drive, slider)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the mitre box: the channel's centre line runs along y at XC; the 45-degree slots cross it at YX ------------------
XC, YX = 0.350, -0.050
IN_HALF = 0.030  # the channel is 6 cm wide inside
WT, WH = 0.015, 0.050  # wall thickness, wall height over the floor
FT = 0.015  # the floor (base board) thickness
FLOOR = TOP + FT
BOX_Y0, BOX_Y1 = -0.150, 0.080  # where the walls run
BASE_Y1 = 0.280  # the base board runs on to the left, under the moulding's far end
SX0, SX1 = XC - IN_HALF - WT - 0.008, XC + IN_HALF + WT + 0.008
Y90 = 0.030  # the 90-degree slot
GAP = 0.014  # each slot's opening across one 3 mm slice of a wall
FENCE = XC + IN_HALF  # the back fence's face
D = np.array([math.sqrt(0.5), math.sqrt(0.5), 0.0])  # along the 45-degree slot used (front right to back left)
N = np.array([-math.sqrt(0.5), math.sqrt(0.5), 0.0])  # across it
# -- the moulding: its frame on its centre line at the pencil mark, at its bottom; it lies along y ---------------------
MW, MH = 0.026, 0.022
L_MAIN, L_OFF = 0.290, 0.050  # left of the mark (kept), right of it (the offcut)
MX = FENCE - MW / 2  # its centre line when it lies against the fence
YM = YX + (MX - XC)  # where the 45-degree slot crosses its centre line: the mark lies here in the box
MX0 = 0.215  # its centre line where it lies at the start, on the table in front of the box
M0 = (MX0, TOP)
M_MASS, OFF_MASS = 0.080, 0.014
NSL = 8  # slices across its width that make its 45-degree ends
# -- the backsaw: its frame at the middle of the blade's toothed edge, the blade along x -----------------------------
BL, BH = 0.240, 0.050
G = np.array([-BL / 2 - 0.058, 0.0, BH])  # where a hand holds its handle
SAW_MASS = 0.16
SAW_COM = (-0.100, 0.0, 0.042)
# -- the saw rest: two slotted blocks; the saw stands in them, blade along x -----------------------------------------
XR, YR = 0.380, -0.210
REST_Z = TOP + 0.010  # the slots' floor: where the saw's teeth rest
PIN = (0.40, -0.10, 0.30)  # hidden, under the table: where the drawn-only bodies wait
# -- the rules -----------------------------------------------------------------------------------------------------------
MARK_Y, MARK_X, MARK_YAW, MARK_Z = 0.010, 0.006, 4.0, 0.004  # laid at the mark, against the fence
LAID_S = 0.5
LAT, SAW_YAW, SAW_ROLL = 0.009, 8.0, 12.0  # the saw in the 45-degree slot
ON_KERF_UP, ON_KERF_DOWN = 0.007, 0.008  # its teeth this near the bottom of the cut
STROKE = 0.030  # a stroke: the saw pushed this far along the slot
MOVE_TOL = 0.004  # ... while the moulding moves at most this much
STROKES_DONE = 8
DEPTH_FULL = MH + 0.001
STEP = DEPTH_FULL / STROKES_DONE
REST_POS, REST_ZTOL, REST_YAW = 0.020, 0.012, 10.0
OUT_Z = 0.012

HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0

BEECH, BEECH_DARK, PLY, KERF = (0.66, 0.45, 0.27), (0.50, 0.34, 0.20), (0.56, 0.50, 0.42), (0.20, 0.13, 0.08)
LACQUER, GOLD, BARE, PENCIL = (0.07, 0.07, 0.08), (0.86, 0.70, 0.30), (0.93, 0.84, 0.66), (0.86, 0.86, 0.88)
STEEL, TEETH, BRASS, ROSEWOOD = (0.76, 0.78, 0.81), (0.46, 0.47, 0.50), (0.80, 0.62, 0.26), (0.42, 0.22, 0.13)
DUST, SLIT, SCREW = (0.94, 0.86, 0.68), (0.16, 0.10, 0.06), (0.35, 0.35, 0.37)


def _slot_gaps(xc: float, w: float) -> list[tuple[float, float]]:
    """The openings (y ranges) the three slots cut through a 3 mm slice of a wall centred at xc."""
    gaps = [(YX + (xc - XC), GAP + w), (YX - (xc - XC), GAP + w), (Y90, GAP)]
    out = sorted((c - g / 2, c + g / 2) for c, g in gaps)
    merged = [list(out[0])]
    for a, b in out[1:]:
        if a <= merged[-1][1]:
            merged[-1][1] = max(merged[-1][1], b)
        else:
            merged.append([a, b])
    return [tuple(m) for m in merged]


def _box() -> Body:
    """The mitre box on its base board (frame at (XC, 0) on the table)."""
    shapes = [BoxShape(((SX0 + SX1) / 2 - XC, (BOX_Y0 + BASE_Y1) / 2, FT / 2), (SX1 - SX0, BASE_Y1 - BOX_Y0, FT), PLY)]
    for x0w in (XC - IN_HALF - WT, XC + IN_HALF):  # front wall, back fence: 5 slices each (collision only)
        for k in range(5):
            x0, x1 = x0w + k * WT / 5, x0w + (k + 1) * WT / 5
            xc = (x0 + x1) / 2
            y = BOX_Y0
            for a, b in _slot_gaps(xc, x1 - x0) + [(BOX_Y1, BOX_Y1)]:
                if a > y:
                    shapes.append(BoxShape((xc - XC, (y + a) / 2, FT + WH / 2), (x1 - x0, a - y, WH), None))
                y = max(y, b)
        # the wall as it looks: solid, with the thin saw kerfs of its three slots
        xm = x0w + WT / 2 - XC
        shapes.append(BoxShape((xm, (BOX_Y0 + BOX_Y1) / 2, FT + WH / 2), (WT, BOX_Y1 - BOX_Y0, WH), BEECH,
                               collide=False))
        for yk, yaw in ((YX + xm, math.pi / 4), (YX - xm, -math.pi / 4), (Y90, 0.0)):
            ln = WT * (math.sqrt(2) if yaw else 1.0) + 0.001
            shapes.append(BoxShape((xm, yk, FT + WH + 0.0003), (ln, 0.0014, 0.0006), KERF, collide=False, yaw=yaw))
            for side in (-1.0, 1.0):  # down the wall's inner and outer faces
                xf = xm + side * (WT / 2 + 0.0003)
                shapes.append(BoxShape((xf, YX + (xf if yaw > 0 else -xf) if yaw else Y90, FT + WH / 2),
                                       (0.0006, 0.0014 * (math.sqrt(2) if yaw else 1.0), WH), KERF, collide=False))
    # the slots' dark kerfs in the floor, from use, and the screws holding the walls (drawn only)
    for s in (1.0, -1.0):
        shapes.append(BoxShape((0.0, YX, FT + 0.0002), (2 * IN_HALF * math.sqrt(2), 0.0015, 0.0004), KERF,
                               collide=False, yaw=s * math.pi / 4))
    shapes.append(BoxShape((0.0, Y90, FT + 0.0002), (2 * IN_HALF, 0.0015, 0.0004), KERF, collide=False))
    for xw in (-IN_HALF - WT / 2, IN_HALF + WT / 2):
        for y in (BOX_Y0 + 0.02, BOX_Y1 - 0.02):
            shapes.append(CylShape((xw, y, FT + WH + 0.0002), 0.003, 0.0004, SCREW, collide=False))
    return Body.build("mitre_box", pos=(XC, 0.0, TOP), mass=3.0, kinematic=True, color=BEECH, shapes=shapes,
                      friction=(0.20, 0.18), slick=True)


def _vee(x: float, y0: float, z0: float, half: float, h: float, length: float, deg: float) -> list:
    """Two cheeks leaning out at `deg` from upright, `half` apart at the bottom (z0) either side of y0: a V that
    funnels a blade or a handle into its bottom."""
    th, t = math.radians(deg), 0.006
    out = []
    for s in (1.0, -1.0):
        phi = -s * th
        up = np.array([0.0, s * math.sin(th), math.cos(th)])
        outw = np.array([0.0, s * math.cos(th), -math.sin(th)])
        c = np.array([x, y0 + s * half, z0]) + up * (h / 2) + outw * (t / 2)
        out.append(BoxShape(tuple(c), (length, t, h), BEECH, quat_xyzw=(math.sin(phi / 2), 0.0, 0.0, math.cos(phi / 2))))
    return out


def _rest() -> Body:
    """The saw rest (frame at the saw's start): two V-blocks under the blade and a V-cradle under the handle's end."""
    shapes = [BoxShape((-0.075, 0.0, 0.002), (0.310, 0.060, 0.004), BEECH_DARK)]
    z0 = REST_Z - TOP
    for x in (-0.105, 0.060):
        shapes.append(BoxShape((x, 0.0, (0.004 + z0) / 2), (0.030, 0.050, z0 - 0.004), BEECH))
        shapes += _vee(x, 0.0, z0, 0.004, 0.026, 0.030, 25.0)
    # the cradle: its V's bottom just under the handle's end
    xh, zh = G[0] - 0.040, z0 + G[2] - 0.0136  # the handle (1 cm round) then sits clear of both cheeks
    shapes.append(BoxShape((xh, 0.0, (0.004 + zh) / 2), (0.016, 0.050, zh - 0.004), BEECH))
    shapes += _vee(xh, 0.0, zh, 0.003, 0.018, 0.016, 35.0)
    return Body.build("rest", pos=(XR, YR, TOP), mass=1.0, kinematic=True, color=BEECH, shapes=shapes)


def _pin() -> Body:
    return Body.build("pin", pos=PIN, mass=1.0, kinematic=True,
                      shapes=[BoxShape((0.0, 0.0, 0.0), (0.01, 0.01, 0.01), None, collide=False)])


def _moulding_shapes(main: bool) -> list:
    """One piece of the moulding: the kept length left of the 45-degree line through the mark (main) or the offcut
    right of it, built as slices across its width so the cut end runs at 45 degrees."""
    shapes = []
    w = MW / NSL
    for i in range(NSL):
        xc = -MW / 2 + (i + 0.5) * w
        y0, y1 = (xc, L_MAIN) if main else (-L_OFF, xc)
        shapes.append(BoxShape((xc, (y0 + y1) / 2, MH / 2), (w, y1 - y0, MH), LACQUER))
        if i < 2:  # the gold lip along its inner (front) edge
            shapes.append(BoxShape((xc, (y0 + y1) / 2, MH + 0.0003), (w, y1 - y0, 0.0006), GOLD, collide=False))
    # the bare wood of the cut face, hidden while the two pieces are one
    c = 0.0006 if main else -0.0006
    shapes.append(BoxShape((-c, c, MH / 2), (MW * math.sqrt(2) - 0.0015, 0.0006, MH - 0.002), BARE, collide=False,
                           yaw=math.pi / 4))
    if main:  # the pencil line across the top, and ticks down both sides
        shapes.append(BoxShape((0.0, 0.0, MH + 0.0004), (MW * math.sqrt(2) - 0.001, 0.0012, 0.0004), PENCIL,
                               collide=False, yaw=math.pi / 4))
        for s in (-1.0, 1.0):
            shapes.append(BoxShape((s * (MW / 2 + 0.0003), s * MW / 2, MH - 0.004), (0.0004, 0.0012, 0.006), PENCIL,
                                   collide=False))
    return shapes


def _moulding() -> Body:
    return Body.build("moulding", pos=(M0[0], YM, M0[1]), mass=M_MASS, shapes=_moulding_shapes(True), color=LACQUER,
                      friction=(0.6, 0.5), com=(0.0, L_MAIN / 2, MH / 2))


def _offcut() -> Body:
    return Body.build("offcut", pos=(M0[0], YM, M0[1]), mass=OFF_MASS, shapes=_moulding_shapes(False), color=LACQUER,
                      friction=(0.6, 0.5), com=(0.0, -L_OFF / 2, MH / 2))


def _kerf() -> Body:
    """The bottom of the saw cut (not drawn): a plate the saw's teeth ride on, sliding down inside the moulding as the
    cut deepens, and dragging the moulding along with the saw."""
    return Body.build("kerf", pos=(M0[0], YM, M0[1] + MH), mass=0.01,
                      shapes=[BoxShape((0.0, 0.0, -0.0015), (0.046, 0.034, 0.003), None, yaw=math.pi / 4)],
                      friction=(1.2, 1.1), ignore=("mitre_box", "offcut"))


def _saw() -> Body:
    """A backsaw: a thin steel blade with a brass back along its top, and a turned rosewood handle in line with it."""
    shapes = [BoxShape((0.0, 0.0, BH / 2), (BL, 0.002, BH), STEEL),
              BoxShape((0.0, 0.0, 0.002), (BL, 0.0024, 0.004), TEETH, collide=False),
              BoxShape((0.0, 0.0, BH + 0.001), (BL, 0.006, 0.012), BRASS),
              CylShape((-BL / 2 - 0.006, 0.0, BH), 0.0115, 0.012, BRASS, axis="X"),
              CylShape((-BL / 2 - 0.063, 0.0, BH), 0.0100, 0.100, ROSEWOOD, axis="X"),
              CylShape((-BL / 2 - 0.115, 0.0, BH), 0.0115, 0.006, ROSEWOOD, axis="X")]
    for x in (-0.095, -0.065):  # the brass back's rivets (drawn only)
        for s in (-1.0, 1.0):
            shapes.append(CylShape((x, s * 0.0032, BH + 0.001), 0.002, 0.0006, BRASS, collide=False, axis="Y"))
    return Body.build("saw", pos=(XR, YR, REST_Z), mass=SAW_MASS, shapes=shapes, color=STEEL, friction=(1.2, 1.1),
                      com=SAW_COM, damping=(0.0, 0.2), ignore=("moulding", "offcut"))


def _slit() -> Body:
    """The saw cut across the moulding's top, drawn once sawing starts (moulding's frame)."""
    return Body.build("slit", pos=PIN, mass=0.01, kinematic=True, shapes=[
        BoxShape((0.0, 0.0, MH + 0.0006), (MW * math.sqrt(2) - 0.001, 0.0018, 0.0006), SLIT, collide=False,
                 yaw=math.pi / 4)])


def _dust(name: str, bits) -> Body:
    """Sawdust on the box's floor in front of the cut (box's frame)."""
    shapes = [BoxShape((x, y, FT + h / 2), (s, s * 0.8, h), DUST, collide=False, yaw=a) for x, y, s, h, a in bits]
    return Body.build(name, pos=PIN, mass=0.01, kinematic=True, color=DUST, shapes=shapes)


slider("kerf", "moulding", "kerf", anchor=(M0[0], YM, M0[1] + MH), axis="Z", limits=(-(MH + 0.004), 0.002),
       stiffness=3000.0, damping=30.0, target=0.0)
glue("cut", "moulding", "offcut", anchor=(M0[0], YM, M0[1] + MH / 2))

_DX = MX - XC - MW / 2 - 0.012  # in front of the moulding, under the cut
OBJECTS = [_box(), _rest(), _pin(), _moulding(), _offcut(), _kerf(), _saw(), _slit(),
           _dust("dust1", [(_DX, YX - 0.012, 0.012, 0.0015, 0.3), (_DX - 0.006, YX - 0.004, 0.008, 0.001, 1.1)]),
           _dust("dust2", [(_DX - 0.004, YX - 0.020, 0.016, 0.0025, 0.7), (_DX + 0.002, YX - 0.002, 0.012, 0.002, 0.2),
                           (_DX - 0.010, YX - 0.012, 0.010, 0.0015, 1.4)])]


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"laid": False, "at_mark_s": 0.0, "strokes": 0, "push": None, "cut": False, "retracted": False,
            "shown": set(), "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("MITRE_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0


def _arm_state(base, origin) -> tuple[list, list]:
    tcps, fingers = [], []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        tcps.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
        fingers.append(float(as_numpy(robot.data.joint_pos)[0][robot.joint_names.index("joint7")]))
    return tcps, fingers


def _write_pose(base, name, pose) -> None:
    import torch

    asset = base.scene[name]
    asset.write_root_pose_to_sim(torch.tensor([list(map(float, pose))], dtype=torch.float32, device=asset.device))


def _yaw(r) -> float:
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def _pose(base, name, origin):
    raw = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return raw, raw[:3] - origin, rotation(raw[3:7])


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    mraw, mp, mr = _pose(base, "moulding", origin)
    _, sp, sr = _pose(base, "saw", origin)
    _, kp, _ = _pose(base, "kerf", origin)
    tcps, fingers = _arm_state(base, origin)
    myaw = _yaw(mr)
    at_mark = (abs(float(mp[1]) - YM) <= MARK_Y and float(mp[0]) >= MX - MARK_X and abs(myaw) <= MARK_YAW
               and abs(float(mp[2]) - FLOOR) <= MARK_Z and not _EP["cut"])
    _EP["at_mark_s"] = _EP["at_mark_s"] + 0.05 if at_mark else 0.0
    if _EP["at_mark_s"] >= LAID_S:
        _EP["laid"] = True
    # the saw in the 45-degree slot, its teeth on the bottom of the cut
    rel = sp - np.array([XC, YX, 0.0])
    s, lat = float(rel @ D), float(rel @ N)
    syaw = _yaw(sr)
    roll = math.degrees(math.asin(max(-1.0, min(1.0, float(sr[2, 1])))))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, float(sr[2, 0])))))
    mpitch = math.degrees(math.asin(max(-1.0, min(1.0, float(mr[2, 1])))))
    kerf_top = float(kp[2])
    u = float((sr.T @ (kp - sp))[0])  # where along the blade the bottom of the cut lies
    up = float((sp + sr @ np.array([u, 0.0, 0.0]))[2]) - kerf_top if abs(u) <= BL / 2 else 1.0
    in_slot = (abs(lat) <= LAT and abs(syaw - 45.0) <= SAW_YAW and abs(roll) <= SAW_ROLL and -0.07 <= s <= 0.07
               and -ON_KERF_DOWN <= up <= ON_KERF_UP)
    st = _EP["push"]
    if in_slot and at_mark:
        if st is None or s < st["smin"] or s < st["smax"] - 0.010:  # drawn back: the next push starts from here
            st = {"smin": s, "smax": s, "m0": mp.copy(), "moved": 0.0, "counted": False}
        st["smax"] = max(st["smax"], s)
        st["moved"] = max(st["moved"], float(np.linalg.norm(mp - st["m0"])))
        if not st["counted"] and s - st["smin"] >= STROKE:
            st["counted"] = True
            if st["moved"] <= MOVE_TOL and _EP["strokes"] < STROKES_DONE:
                _EP["strokes"] += 1
                set_slider_drive("kerf", target=-STEP * _EP["strokes"])
                if _EP["strokes"] >= STROKES_DONE:
                    _EP["cut"] = True
                    set_glue_on("cut", False)
    else:
        st = None
    _EP["push"] = st
    # once the saw is out of the finished cut, the plate goes back up inside the moulding (out of everything's way)
    if _EP["cut"] and not _EP["retracted"] and float(sp[2]) > FLOOR + 0.045:
        _EP["retracted"] = True
        set_slider_drive("kerf", target=-0.008)
    # drawn only: the saw cut across the top while sawing, and sawdust
    if 1 <= _EP["strokes"] and not _EP["cut"]:
        _write_pose(base, "slit", mraw)
        _EP["shown"].add("slit")
    elif "slit" in _EP["shown"]:
        _write_pose(base, "slit", list(np.array(PIN) + origin) + [0.0, 0.0, 0.0, 1.0])
        _EP["shown"].discard("slit")
    braw = as_numpy(base.scene["mitre_box"].data.root_link_pose_w)[0]
    for name, n in (("dust1", 2), ("dust2", 6)):
        if _EP["strokes"] >= n and name not in _EP["shown"]:
            _write_pose(base, name, braw)
            _EP["shown"].add(name)
    _EP["tcps"] = tcps
    _EP["s"] = {"at_mark": at_mark, "s": s, "lat": lat, "up": up}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0:
            tl, trc = tcps
            push = _EP["push"]
            print(f"[mitre] t={_DEBUG['steps'] / 20:5.1f}s mould={np.round((mp - [MX, YM, FLOOR]) * 1000).astype(int).tolist()}"
                  f" myaw={myaw:5.1f} mpitch={mpitch:5.1f} at_mark={at_mark} laid={_EP['laid']}"
                  f" saw s={s * 1000:6.1f} lat={lat * 1000:5.1f} up={up * 1000:6.1f} yaw={syaw:5.1f} roll={roll:5.1f} pitch={pitch:5.1f}"
                  f" in_slot={in_slot} moved={(push['moved'] * 1000 if push else -1):5.1f} strokes={_EP['strokes']}"
                  f" cut={_EP['cut']} kerf={(kerf_top - FLOOR) * 1000:5.1f}"
                  f" sawpos={np.round((sp - [XR, YR, REST_Z]) * 1000).astype(int).tolist()}"
                  f" fing={[round(f * 1000, 1) for f in fingers]}"
                  f" L={np.round((tl - [0, 0, TOP]) * 1000).astype(int).tolist()}"
                  f" R={np.round((trc - [0, 0, TOP]) * 1000).astype(int).tolist()} score={score(None)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """1.0: the moulding cut through, the saw back in its rest, the cut moulding out of the box on the table and both
    arms back where they started, at the end; 0.75: cut through; 0.5: at least half the strokes (a kerf at least 11 mm
    deep); 0.25: the moulding laid in the box against the fence with its mark at the 45-degree slot; else 0.0."""
    if _EP["cut"]:
        if objects is not None and _finished(objects) and _arms_home():
            return 1.0
        return 0.75
    if _EP["strokes"] >= STROKES_DONE // 2:
        return 0.5
    return 0.25 if _EP["laid"] else 0.0


def _finished(objects) -> bool:
    """The saw standing back in its rest and the cut moulding lying on the table out in front of the box."""
    saw, m = objects["saw"], objects["moulding"]
    sp, sr = np.asarray(saw.pos, float), rotation(saw.quat_xyzw)
    rest = (float(np.linalg.norm(sp[:2] - [XR, YR])) <= REST_POS and abs(float(sp[2]) - REST_Z) <= REST_ZTOL
            and abs(_yaw(sr)) <= REST_YAW)
    mp, mr = np.asarray(m.pos, float), rotation(m.quat_xyzw)
    far = mp + mr @ np.array([0.0, L_MAIN / 2, 0.0])
    out = abs(float(mp[2]) - TOP) <= OUT_Z and float(mp[0]) < SX0 - 0.005 and float(far[0]) < SX0 - 0.005
    return rest and out


_HOME: dict = {}


def _arms_home() -> bool:
    """Both arms back where they started (each TCP within HOME_WITHIN of its start)."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    tcps = _EP.get("tcps")
    if tcps is None:
        return False
    if "tcps" not in _HOME:
        _HOME["tcps"] = [np.asarray(ik.tcp_pose(np.array(READY, float), station.BASES[a])[0], float)
                         for a in ("left", "right")]
    return all(float(np.linalg.norm(t - h)) < HOME_WITHIN for t, h in zip(tcps, _HOME["tcps"]))


def success(objects) -> bool:
    """Full score with the saw and both pieces of moulding still: the trial ends there."""
    return (score(objects) >= 1.0 and objects["saw"].resting() and objects["moulding"].resting()
            and objects["offcut"].resting())


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes down on the moulding's far part, fingers closing across it, lifts it over the box's front wall,
# lays it in the channel against the back fence with its pencil line at the 45-degree slot and holds it there, pressing
# it down. The right hand takes the saw from its rest by the handle, lifts it, turns it to the slot and sets it in;
# it saws eight strokes, each a little deeper, until the moulding is cut through, lifts the saw out and stands it back
# in its rest. The left then lifts the cut moulding out of the box and lays it on the table in front of the box; both
# arms go back to where they started.

GRIP_Y = 0.120  # the left hand holds the moulding here (y)
M_PRE = -(MW / 2 + 0.006)
M_GRIP_Z = MH - 0.007  # TCP above the moulding's bottom
SAW_PRE = -(0.010 + 0.006)
PRESS_M = 0.002  # the moulding pressed this far down into the floor
M_OFF = 0.011  # a gripped moulding sits this far +x of the TCP (the first finger to close shoves it)
PUSH_X = 0.012  # laid 7 mm clear of the fence, then slid back this far (the fence stops it)
PRESS_S = -0.008  # the saw aimed this far below the bottom of the cut (above it: the arm sags about 6 mm, and the
#   saw's own weight rests it on the cut)
N_STROKES = STROKES_DONE + 4  # the demo saws four strokes more than needed: the first ones go while the saw settles
SAW_LAT = 0.012  # a gripped saw sits this far to its own left (+y) of where the TCP holds it (measured): aim against it
S_BACK, S_FWD = -0.030, 0.035
CARRY = 0.105  # TCP height (above the table) carrying the moulding over the wall
SAW_UP = 0.115  # the saw's teeth this high (above the table) carrying it
LEAD_S, LEAD_STEPS = 3.0, 15
T_LAID = 10.5  # the right hand sets the saw in the slot once the moulding is in place (s)


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.7, 0.0, 0.7), (1.3, 1.9), (1.0, 1.7), (-1.3, -0.8), (-0.4, 0.0, 0.4), (-0.4, 0.4)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _joint_path(arm: str, q0, q1, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _left() -> list[Waypoint]:
    from yam_arena.constants import READY

    rot = grip_rotation(-math.pi / 2)  # fingers closing along x, across the moulding
    pick = np.array([MX0, GRIP_Y, TOP + M_GRIP_Z])
    put = np.array([MX - M_OFF - 0.007, GRIP_Y, FLOOR + M_GRIP_Z])
    out = _joint_path("left", np.array(READY, float), _good_ik("left", pick + [0.0, 0.0, 0.05], rot),
                      "over the moulding", M_PRE)
    out.append(_w(1.0, pick, M_PRE, "down round the moulding", rot))
    out.append(_w(0.6, pick, FINGER_CLOSED, "take hold of the moulding", rot))
    lift = np.array([MX0, GRIP_Y, TOP + CARRY])
    out.append(_w(1.0, lift, FINGER_CLOSED, "lift it", rot))
    out.append(_w(1.4, [put[0], GRIP_Y, TOP + CARRY], FINGER_CLOSED, "carry it over the box's front wall", rot))
    out.append(_w(1.2, put, FINGER_CLOSED, "lay it in the box, the line at the 45-degree slot", rot))
    out.append(_w(0.8, put + [PUSH_X, 0.0, -PRESS_M], FINGER_CLOSED, "slide it back against the fence and hold it down",
                  rot))
    return out


def _saw_tcp(origin, yaw: float, lat: float = 0.0) -> np.ndarray:
    """The TCP that puts the saw's frame at `origin`, turned `yaw`; `lat`: how far the saw sits across its handle
    (its own y) from where the TCP holds it."""
    return np.asarray(origin, float) + _rz(yaw) @ (G - [0.0, lat, 0.0])


def _right() -> tuple[list[Waypoint], float]:
    """The right hand's part, and the time it lifts the saw out of the finished cut."""
    from yam_arena.constants import READY

    r0, r45 = grip_rotation(0.0), _rz(math.pi / 4) @ grip_rotation(0.0)
    at_rest = np.array([XR, YR, REST_Z])
    t_rest = _saw_tcp(at_rest, 0.0)
    out = _joint_path("right", np.array(READY, float), _good_ik("right", t_rest + [0.0, 0.0, 0.05], r0),
                      "over the saw's handle", SAW_PRE)
    out.append(_w(1.0, t_rest, SAW_PRE, "down round the saw's handle", r0))
    out.append(_w(0.6, t_rest, FINGER_CLOSED, "take hold of the saw", r0))
    up = at_rest + [0.0, 0.0, TOP + SAW_UP - REST_Z]
    out.append(_w(1.2, _saw_tcp(up, 0.0), FINGER_CLOSED, "lift it out of its rest", r0))
    kerf0 = FLOOR + MH

    def slot(s, z):
        return np.array([XC, YX, z]) + s * D

    over = slot(S_BACK, TOP + SAW_UP)
    t_now = sum(w.duration for w in out)
    out.append(_w(max(0.3, T_LAID - 1.6 - t_now), _saw_tcp(up, 0.0), FINGER_CLOSED,
                  "wait for the moulding to be in place", r0))
    out.append(_w(1.6, _saw_tcp(over, math.pi / 4, SAW_LAT), FINGER_CLOSED, "turn it over the 45-degree slot", r45))
    out.append(_w(1.2, _saw_tcp(slot(S_BACK, kerf0 - PRESS_S), math.pi / 4, SAW_LAT), FINGER_CLOSED,
                  "set the saw in the slot, its teeth on the pencil line", r45))
    for k in range(N_STROKES):
        z = kerf0 - STEP * min(k, STROKES_DONE) - PRESS_S
        for j in range(1, 4):
            out.append(_w(0.30, _saw_tcp(slot(S_BACK + (S_FWD - S_BACK) * j / 3, z), math.pi / 4, SAW_LAT), FINGER_CLOSED,
                          f"push the saw along the slot ({k + 1} of {N_STROKES})", r45))
        z2 = kerf0 - STEP * min(k + 1, STROKES_DONE) - PRESS_S
        out.append(_w(0.6, _saw_tcp(slot(S_BACK, max(z2, FLOOR - PRESS_S)), math.pi / 4, SAW_LAT), FINGER_CLOSED,
                      "draw it back", r45))
    t_out = sum(w.duration for w in out)
    out.append(_w(1.0, _saw_tcp(slot(S_BACK, TOP + SAW_UP), math.pi / 4, SAW_LAT), FINGER_CLOSED,
                  "lift the saw out of the cut", r45))
    out.append(_w(1.6, _saw_tcp(up, 0.0, SAW_LAT), FINGER_CLOSED, "carry it back over its rest", r0))
    back = _saw_tcp(at_rest + [0.0, 0.0, 0.003], 0.0, SAW_LAT)
    out.append(_w(1.2, back, FINGER_CLOSED, "stand it back in its rest", r0))
    out.append(_w(0.5, back, SAW_PRE, "let go", r0))
    out.append(_w(0.8, t_rest + [0.0, 0.0, 0.05], SAW_PRE, "rise clear of the handle", r0))
    out += _joint_path("right", _good_ik("right", t_rest + [0.0, 0.0, 0.05], r0), np.array(READY, float),
                       "back to where it started", SAW_PRE)
    return out, t_out


def demo():
    from yam_arena.constants import READY

    left = _left()
    right, t_out = _right()
    rot = grip_rotation(-math.pi / 2)
    hold = np.array([MX - M_OFF - 0.007 + PUSH_X, GRIP_Y, FLOOR + M_GRIP_Z - PRESS_M])
    t_now = sum(w.duration for w in left)
    left.append(_w(max(0.3, t_out + 0.8 - t_now), hold, FINGER_CLOSED, "hold the moulding while it is sawn", rot))
    left.append(_w(0.8, [MX, GRIP_Y, TOP + CARRY], FINGER_CLOSED, "lift the cut moulding out of the box", rot))
    put = np.array([MX0, GRIP_Y, TOP + M_GRIP_Z + 0.001])
    left.append(_w(1.4, [MX0, GRIP_Y, TOP + CARRY], FINGER_CLOSED, "carry it over the front wall", rot))
    left.append(_w(1.0, put, FINGER_CLOSED, "lay it on the table in front of the box", rot))
    left.append(_w(0.5, put, M_PRE, "let go", rot))
    left.append(_w(0.8, put + [0.0, 0.0, 0.05], M_PRE, "rise clear of it", rot))
    left += _joint_path("left", _good_ik("left", put + [0.0, 0.0, 0.05], rot), np.array(READY, float),
                        "back to where it started", M_PRE)
    for plan in (left, right):
        last = plan[-1]
        plan.append(_w(3.0, last.pos, last.finger,
                       "hold still: the trial ends once the full score has held 1 s with both arms home", last.yaw))
    tl, tr = sum(w.duration for w in left), sum(w.duration for w in right)
    short = left if tl < tr else right
    last = short[-1]
    short[-1] = _w(last.duration + abs(tl - tr), last.pos, last.finger, last.label, last.yaw)
    return {"left": left, "right": right}
