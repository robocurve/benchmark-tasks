"""Square a crooked cabinet door: turn its top cup hinge's side screw with a screwdriver until the gap is even.

Two frameless cabinets (18 mm white melamine, 300 wide x 300 deep x 450 high) stand side by side on a 30 mm plinth, open sides toward the robot, each with a
19 mm full-overlay door (298 x 448 mm, 1.9 kg) on two Blum CLIP top 110-degree hinges (71T3550) on 0 mm cam plates (173H7100), with a bar pull. Both doors are
right-hinged: the right door on the right cabinet's outer side, the left door on the middle partition, so it opens toward the middle. The right door is square,
2.5 mm from the left. The left door hangs crooked: its top hinge's side screw is backed off, so the gap between the doors is 1.7 mm 20 mm below the top and
2.6 mm 20 mm above the bottom. The pair stands right of the table's middle. A long-reach PH2 screwdriver (300 mm blade) lies on the cabinets' top, its handle over the left cabinet's left edge.

The door is a hinged body on the real 110-degree swing, held open by the hinge's friction above 25 degrees and drawn shut by its self-close spring below that. Each hinge arm's side
screw is a real part with a cross recess on a stiff seat spring; its thread and the bit's drive in the recess are code (declared below, force-gated): the screwdriver's turn about its
own axis turns the head only with the tip seated in the recess, the driver within 15 degrees of the screw's axis and pushed in, against a resisting torque, and moves that hinge's arm sideways by
the screw's pitch; the door follows through its hinge line. Everything else is physics: the hands, the door, the bar, the driver.

    python demokit.py task.py OUT --mode full
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import gatekit as gk  # noqa: E402
import mechkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = ("The left door hangs crooked. Use the screwdriver on its hinge to even the gap between the doors, 2 to 3 mm, then shut the door and let go.")
SUCCESS_HOLD_S = 0.0  # the scorer counts its own 1 s hold
HOLD_STEPS = 20
TOP = TABLE_TOP[SCENE]

# -- the cabinets, station metres (x forward from the arm bases, y left from the midline, z up from the table top)
XC, CD, CH, PL, T = 0.40, 0.30, 0.45, 0.03, 0.018  # front face, depth, height, plinth, board
DY = -0.250  # the pair stands right of the middle: the cabinets and the left door are authored about y 0 and placed this far along y
OFF = np.array([0.0, DY, 0.0])
Z1 = PL + CH
DT, DW, DH = 0.019, 0.298, 0.448  # the doors
DX1 = XC - 0.002  # the door's back face: 2 mm in front of the carcase (the door bumpers)
DX0 = DX1 - DT
ZD0 = PL + 0.001
REVEAL = 0.0025  # the doors' nominal gap
LY0, LY1 = REVEAL / 2, REVEAL / 2 + DW  # left door, across y
XH, YH = DX0 + 0.002, LY0 + 0.00275  # its hinge line, on the middle partition side: near the door's front face, so its edge clears the right door as it swings
ZB, ZT = ZD0 + 0.080, ZD0 + DH - 0.080  # the two hinges' heights: 80 mm from the door's bottom and top
SPAN = ZT - ZB
YI = 0.300 - T  # the left side panel's inner face
YM = T  # the middle partition's left face (the left cabinet's right side panel), where the left door's plates are screwed
PLATE = 0.0085
YARM0 = YM + PLATE  # the arm lies on the plate: its face on the plate
YARM1 = YARM0 + 0.010  # and its face toward the cabinet's inside, where the screw heads are
SCREW_X = XC + 0.032  # the side-adjust screw on each arm, 32 mm back from the front
DEPTH_X = XC + 0.060  # the depth screw, behind it
HEAD_R, HEAD_L = 0.0045, 0.003
REC_R, REC_D = 0.002, 0.0015  # the cross recess as the bit meets it: a pocket this wide and deep, whose wall holds a seated tip sideways
YFACE = YARM1 + 0.0003 + HEAD_L  # the head's face, where the bit seats: the heads face +y, into the left cabinet
EIN = np.array([0.0, -1.0, 0.0])  # the screw's axis, into the partition: the bit's direction when seated
GAP_AT = 0.020  # the gap is read 20 mm from the door's top and bottom edges
GAP_Z = (ZD0 + GAP_AT, ZD0 + DH - GAP_AT)
PITCH = 0.0010  # m of side travel per turn: assumed (Blum gives +-2 mm of travel, not the thread)
TRAVEL = 0.0020
START_GAP = (2.6e-3, 1.7e-3)  # gap at the bottom and top, at the start
GAP_LO, GAP_HI, GAP_EVEN = 0.0020, 0.0030, 0.0005
SHUT_DEG = 2.0
HOLD_OPEN_DEG = 25.0  # the hinge's spring draws the door shut below this; above it friction holds the door where it is left
HINGE_K, HINGE_DRAG, HINGE_DAMP = 1.2, 0.15, 1.5  # N m/rad: the self-close spring; N m of friction; N m s/rad
CLEAR = 0.03
STRIP_TURNS = 0.5  # a screw forced this far past its stop strips (the arm comes off its plate)
ENGAGE_DEG = 15.0  # the bit drives the screw only with the driver within this of the screw's axis (PH2 cams out beyond it)
ENGAGE_LAT = 0.0015  # and its tip within this of the recess's centre
PUSH_MIN = 10.0  # and pushed into the head with at least this, N
SEAT_K = 20000.0  # N/m: the head's seat spring, through which the push is read (10 N presses it 0.5 mm)
RESIST = 0.3  # N m: the screw's resisting torque, held against the driver's turn while the bit drives it
GRIP_N = 35.0  # the demo's grip on the handle, N a finger
PUSH_GOAL = 13.0  # the push the demo keeps on the bit, N
TURN_MIN = 0.25  # tier 1: the top screw turned a quarter turn the right way

DOOR_OPEN = 55.0  # the right hand opens the door this far toward the middle: enough for the bit and the hand to pass, and within the right arm's reach
# -- the screwdriver: a long-reach PH2 screwdriver. Its 300 mm blade (6 mm shaft) runs along local +x from the handle's front end (origin) to the tip;
# its 100 mm handle (octagonal, 31 mm across the corners, 28.6 across the flats) runs along local -x. The hand holds the handle in line with its own axis, so the
# push on the bit runs along the blade and the wrist's roll turns it. The blade reaches the screw from outside the cabinet, 10 degrees off the screw's axis.
SD_R, SD_L, SD_SR, SD_SL = 0.0155, 0.100, 0.003, 0.300
SD_TIP = 0.004  # the bit's point: a cone from the blade's 3 mm radius to 1 mm
SD_FLAT = 2 * SD_R * math.cos(math.radians(22.5))  # the handle's width across its flats, 28.6 mm
BIT = SD_SL  # the tip, along local x
APPROACH = 10.0  # degrees off the screw's axis: the least that keeps the blade clear of the cabinet's left side at the opening
SD_YAW = -90.0 + APPROACH  # it lies on the cabinets' top pointing right, as it is held at the screw
SD_START = np.array([0.43, DY + 0.30 - 0.043, Z1 + SD_FLAT / 2 + 0.001])  # the handle's front end, 43 mm in from the left cabinet's left edge: the rest of the handle over the edge, the blade along the top
GRIP_AT = 0.060  # the hand holds the handle this far behind its front end, where it is over the edge

LOOKS = {
    "melamine": m.Look(color=m.rgb(0.93, 0.93, 0.91), roughness=0.4),
    "door": m.Look(color=m.rgb(0.85, 0.84, 0.80), roughness=0.45),
    "ply": m.Look(color=m.rgb(0.74, 0.60, 0.42), roughness=0.75),
    "nickel": m.Look(color=m.rgb(0.76, 0.76, 0.77), metallic=0.9, roughness=0.3),
    "screw": m.Look(color=m.rgb(0.55, 0.55, 0.56), metallic=0.9, roughness=0.35),
    "dark": m.Look(color=m.rgb(0.08, 0.08, 0.09), roughness=0.6),
    "handle": m.Look(color=m.rgb(0.85, 0.15, 0.10), roughness=0.5),
    "steel": m.Look(color=m.rgb(0.70, 0.70, 0.72), metallic=1.0, roughness=0.3),
    "pull": m.Look(color=m.rgb(0.30, 0.30, 0.33), metallic=0.8, roughness=0.35),
    "stand": m.Look(color=m.rgb(0.30, 0.30, 0.32), roughness=0.6),
}


def _bx(x, y, z) -> m.Mesh:
    return m.box((x[1] - x[0], y[1] - y[0], z[1] - z[0]), ((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (z[0] + z[1]) / 2))


def _cbox(name, x, y, z) -> str:
    return m.col_box(name, (x[1] - x[0], y[1] - y[0], z[1] - z[0]), ((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (z[0] + z[1]) / 2))


def _pull(prefix: str, y: float, parts, cols):
    """A bar pull on a door's outer face, near its free edge: two standoffs and a 12 mm vertical bar 30 mm off the face."""
    xb = DX0 - 0.030
    zc = 0.255
    parts.append((f"{prefix}_bar", m.cylinder(0.006, 0.120, (xb, y, zc), axis="z", segments=16), "pull"))
    cols.append(m.col_cylinder(f"{prefix}_bar_c", 0.006, 0.120, (xb, y, zc)))
    for dz in (-0.045, 0.045):
        x = (xb, DX0)
        parts.append((f"{prefix}_so{int(dz > 0)}", _bx(x, (y - 0.004, y + 0.004), (zc + dz - 0.004, zc + dz + 0.004)), "pull"))
        cols.append(_cbox(f"{prefix}_so{int(dz > 0)}_c", x, (y - 0.004, y + 0.004), (zc + dz - 0.004, zc + dz + 0.004)))


def _cup(prefix: str, y: float, parts):
    """A hinge's cup on a door's back face, with its boss going back to the arm (looks only)."""
    for i, h in enumerate((ZB, ZT)):
        parts.append((f"{prefix}_cup{i}", m.cylinder(0.0175, 0.0008, (DX1 + 0.0004, y, h), axis="x", segments=28), "nickel"))
        parts.append((f"{prefix}_boss{i}", _bx((DX1, DX1 + 0.016), (y - 0.003, y + 0.003), (h - 0.010, h + 0.010)), "nickel"))


def _cabinets():
    parts, cols = [], []
    X = (XC, XC + CD)
    boxes = {"plinth": (X, (-0.30, 0.30), (0.0, PL)), "top": (X, (-0.30, 0.30), (Z1 - T, Z1)), "bottom": (X, (-0.30, 0.30), (PL, PL + T)),
             "back": ((XC + CD - 0.006, XC + CD), (-0.30, 0.30), (PL, Z1)), "side_l": (X, (YI, 0.30), (PL, Z1)), "side_r": (X, (-0.30, -YI), (PL, Z1)),
             "mid": (X, (-T, T), (PL, Z1))}
    for n, (x, y, z) in boxes.items():
        parts.append((n, _bx(x, y, z), "ply" if n == "plinth" else "melamine"))
        cols.append(_cbox(f"{n}_c", x, y, z))
    # the right door, square: static
    ry = (-LY1, -LY0)
    parts.append(("door_r", _bx((DX0, DX1), ry, (ZD0, ZD0 + DH)), "door"))
    cols.append(_cbox("door_r_c", (DX0, DX1), ry, (ZD0, ZD0 + DH)))
    _pull("pull_r", -0.030, parts, cols)
    _cup("hw_r", -0.2777, parts)
    for i, h in enumerate((ZB, ZT)):  # its plates and arms, on the right side panel's inner face
        parts.append((f"plate_r{i}", _bx((XC + 0.020, XC + 0.070), (-YI, -YI + PLATE), (h - 0.0225, h + 0.0225)), "nickel"))
        parts.append((f"arm_r{i}", _bx((XC + 0.012, XC + 0.082), (-YI + PLATE, -YI + PLATE + 0.010), (h - 0.007, h + 0.007)), "nickel"))
    return m.fixed("cabinets", (0.0, DY, TOP), parts, cols, LOOKS, friction=(0.4, 0.3))


def _frame_hardware():
    """The left door's plates and arms on the middle partition's left face, with the depth screws (fixed, as the thread is not modelled) and the plate cams."""
    parts, cols = [], []
    for i, h in enumerate((ZB, ZT)):
        px = (XC + 0.020, XC + 0.070)
        parts.append((f"plate{i}", _bx(px, (YM, YARM0), (h - 0.0225, h + 0.0225)), "nickel"))
        cols.append(_cbox(f"plate{i}_c", px, (YM, YARM0), (h - 0.0225, h + 0.0225)))
        ax = (XC + 0.012, XC + 0.082)
        parts.append((f"arm{i}", _bx(ax, (YARM0, YARM1), (h - 0.007, h + 0.007)), "nickel"))
        cols.append(_cbox(f"arm{i}_c", ax, (YARM0, YARM1), (h - 0.007, h + 0.007)))
        dy = YARM1 + 0.0003 + HEAD_L / 2
        parts.append((f"depth{i}", m.cylinder(HEAD_R, HEAD_L, (DEPTH_X, dy, h), axis="y", segments=16), "screw"))
        cols.append(m.col_hull(f"depth{i}_c", m.cylinder(HEAD_R, HEAD_L, (DEPTH_X, dy, h), axis="y", segments=12)))
        parts.append((f"cam{i}", m.cylinder(0.0035, 0.0008, (XC + 0.012, YARM0 + 0.0004, h), axis="y", segments=16), "screw"))
    return parts, cols


def _screws():
    links = []
    for k, h in (("t", ZT), ("b", ZB)):
        yc = YARM1 + 0.0003 + HEAD_L / 2
        links.append(m.slider(f"seat_{k}", "Y", -0.003, 0.0005, [], [], at=(SCREW_X, yc, h), mass=0.002, spring=SEAT_K, damping=20.0, effort=200.0, friction=0.0, inertia=(1e-7, 1e-7, 1e-7)))
        head = m.cylinder(HEAD_R, HEAD_L, (SCREW_X, yc, h), axis="y", segments=24)
        yr = YFACE - 0.0003
        recess = [m.box((0.0055, 0.0006, 0.0012), (SCREW_X, yr, h)), m.box((0.0012, 0.0006, 0.0055), (SCREW_X, yr, h))]
        cols = [m.col_hull(f"head_{k}_c", m.cylinder(HEAD_R, HEAD_L - REC_D, (SCREW_X, yc - REC_D / 2, h), axis="y", segments=16))]
        w = HEAD_R - REC_R
        cols += [m.col_hull(f"rim_{k}{i}", m.box((w, REC_D, 2 * HEAD_R * math.tan(math.radians(22.5))), (REC_R + w / 2, 0.0, 0.0)).turned(45.0 * i, "y").moved((SCREW_X, YFACE - REC_D / 2, h)))
                 for i in range(8)]  # the pocket's wall, in eight pieces round it
        links.append(m.hinge(f"screw_{k}", "Y", (SCREW_X, yc, h), -1e5, 1e5, [(f"head_{k}", head, "screw"), (f"rec_{k}a", recess[0], "dark"), (f"rec_{k}b", recess[1], "dark")],
                             cols, mass=0.004, spring=0.05, damping=0.002, friction=0.0, effort=5.0, inertia=(1e-7, 1e-7, 1e-7), parent=f"seat_{k}"))
    return links


def _door_link():
    cy = (LY0 + LY1) / 2
    parts = [("slab", _bx((DX0, DX1), (LY0, LY1), (ZD0, ZD0 + DH)), "door")]
    cols = [_cbox("slab_c", (DX0, DX1), (LY0, LY1), (ZD0, ZD0 + DH))]
    _pull("pull_l", LY1 - 0.030, parts, cols)
    _cup("hw_l", LY0 + 0.0215, parts)
    mass = 1.9
    inertia = (mass * (DW ** 2 + DH ** 2) / 12, mass * (DT ** 2 + DH ** 2) / 12, mass * (DT ** 2 + DW ** 2) / 12)
    return m.hinge("swing", "Z", (XH, YH, ZB), 0.0, 110.0, parts, cols, mass=mass, gravity=True, com=((DX0 + DX1) / 2, cy, ZD0 + DH / 2), spring=HINGE_K, damping=HINGE_DAMP,
                   effort=1.5, friction=0.0, armature=0.002, inertia=inertia, parent="tilt")


def _door():
    hw_parts, hw_cols = _frame_hardware()
    shift = m.slider("shift", "Y", -0.01, 0.01, [], [], at=(XH, YH, ZB), mass=0.2, spring=4e5, damping=2000.0, effort=4000.0, friction=0.0, inertia=(1e-3, 1e-3, 1e-3))
    tilt = m.hinge("tilt", "X", (XH, YH, ZB), -10.0, 10.0, [], [], mass=0.2, spring=3e4, damping=300.0, effort=3000.0, friction=0.0, inertia=(1e-3, 1e-3, 1e-3), parent="shift")
    return m.mechanism("door", (0.0, DY, TOP), [shift, tilt, _door_link(), *_screws()], LOOKS, frame_parts=hw_parts, frame_colliders=hw_cols, friction=(0.4, 0.3), solver_iters=64)


def _driver():
    hx = -SD_L / 2
    tip = m.lathe([(0.0, 0.0), (SD_SR, 0.0), (0.001, SD_TIP), (0.0, SD_TIP)], segments=12).aimed("+X").moved((SD_SL - SD_TIP, 0.0, 0.0))  # the PH2 point
    parts = [("handle", m.cylinder(SD_R, SD_L, (hx, 0.0, 0.0), axis="x", segments=8).turned(22.5, "x"), "handle"),  # flats up, down and to the sides
             ("band", m.cylinder(SD_R + 0.0005, 0.008, (-0.012, 0.0, 0.0), axis="x", segments=8).turned(22.5, "x"), "dark"),
             ("ferrule", m.cylinder(0.006, 0.014, (0.007, 0.0, 0.0), axis="x", segments=16), "steel"),
             ("shaft", m.cylinder(SD_SR, SD_SL - SD_TIP, ((SD_SL - SD_TIP) / 2, 0.0, 0.0), axis="x", segments=12), "steel"),
             ("bit", tip, "steel")]
    cols = [m.col_hull("handle_c", m.cylinder(SD_R, SD_L, (hx, 0.0, 0.0), axis="x", segments=8).turned(22.5, "x")),
            m.col_hull("shaft_c", m.cylinder(SD_SR, SD_SL - SD_TIP - 0.004, ((SD_SL - SD_TIP) / 2 + 0.002, 0.0, 0.0), axis="x", segments=8)),
            m.col_hull("bit_c", tip)]
    return parts, cols


_dp, _dc = _driver()
DRIVER = m.free("driver", tuple(SD_START + np.array([0.0, 0.0, TOP])), _dp, _dc, LOOKS, mass=0.15, friction=(0.8, 0.7), yaw_deg=SD_YAW)
OBJECTS = [_cabinets(), _door(), DRIVER]


# -- the screws' thread: declared code -----------------------------------------------------------------------------

def _s0() -> tuple[float, float]:
    """The hinges' side offsets at the start (m, positive toward +y: the gap widens): found from the start gaps at the two reading heights."""
    k = lambda z: (z - ZB) / SPAN  # noqa: E731  lateral(z) = sb + (st - sb) k
    a = np.array([[1 - k(GAP_Z[0]), k(GAP_Z[0])], [1 - k(GAP_Z[1]), k(GAP_Z[1])]])
    st_sb = np.linalg.solve(a, np.array(START_GAP) - REVEAL)
    return float(st_sb[0]), float(st_sb[1])  # (sb, st)


SB0, ST0 = _s0()


def lateral(st: float, sb: float, z: float) -> float:
    return sb + (st - sb) * (z - ZB) / SPAN


def gaps_from(st: float, sb: float) -> tuple[float, float]:
    """The gap (bottom, top) from the two hinges' offsets, door shut."""
    return REVEAL + lateral(st, sb, GAP_Z[0]), REVEAL + lateral(st, sb, GAP_Z[1])


def _roll(R) -> float:
    """The screwdriver's turn about its own axis (its local x), from its rotation: positive is clockwise seen from behind the handle."""
    ax, side = R[:, 0], R[:, 1]
    ref = np.array([0.0, 0.0, 1.0]) - ax * ax[2]
    ref = ref / (np.linalg.norm(ref) + 1e-9)
    return math.atan2(float(np.dot(np.cross(ref, side), ax)), float(np.dot(ref, side)))


def _torque(env, part: str, tau) -> None:
    """Hold torque `tau` (N m, station frame) on a free part until changed: the screw's resistance on the seated bit."""
    import torch

    asset = env.unwrapped.scene[part]
    t = torch.tensor([[[float(v) for v in tau]]], device=asset.device)
    asset.permanent_wrench_composer.set_forces_and_torques_index(forces=torch.zeros_like(t), torques=t, body_ids=[0], is_global=True)
    m._PUSHED.add((part, 0))


HEAD = {"t": np.array([SCREW_X, YFACE + DY, TOP + ZT]), "b": np.array([SCREW_X, YFACE + DY, TOP + ZB])}  # each head's face centre, station frame


class Thread:
    """Turns of each side screw from its start, and the door's hinge targets that follow."""

    def __init__(self) -> None:
        self.reset()

    def reset(self, env=None) -> None:
        self.turns = {"t": 0.0, "b": 0.0}
        self.last = None
        self.engaged = None
        self.over = 0.0  # turns forced past a stop
        self.push = {"t": 0.0, "b": 0.0}
        self.stripped = False

    def s(self, k: str) -> float:
        return (ST0 if k == "t" else SB0) + self.turns[k] * PITCH

    def step(self, env) -> None:
        drv = m.state(env, "driver")
        R = drv.rotation()
        tip = drv.to_world(np.array([[BIT, 0.0, 0.0]]))[0]
        axis = R[:, 0]
        roll = _roll(R)
        self.engaged = None
        door = m.state(env, "door")
        for k, f in HEAD.items():
            self.push[k] = SEAT_K * max(0.0, -door.joint(f"seat_{k}"))  # the bit's push on the head (the seat pressed toward -y), N
            lat = math.hypot(tip[0] - f[0], tip[2] - f[2])
            ax = f[1] - tip[1]  # how far the tip is past the face, into the head
            if lat <= ENGAGE_LAT and -0.001 < ax < 0.004 and float(np.dot(axis, EIN)) >= math.cos(math.radians(ENGAGE_DEG)) and self.push[k] >= PUSH_MIN:
                self.engaged = k
        w = float(np.dot(drv.ang_vel, axis)) if self.engaged else 0.0  # the driver's turn rate about its axis
        _torque(env, "driver", -RESIST * float(np.clip(w / 0.2, -1.0, 1.0)) * axis)  # the screw resists the turn
        if self.last is not None and self.engaged is not None:
            d = (roll - self.last + math.pi) % (2 * math.pi) - math.pi
            k = self.engaged
            new = self.turns[k] + d / (2 * math.pi)
            lo = (-TRAVEL - (ST0 if k == "t" else SB0)) / PITCH
            hi = (TRAVEL - (ST0 if k == "t" else SB0)) / PITCH
            if new > hi or new < lo:  # against the end of the slot: the screw turns no further
                self.over += abs(d) / (2 * math.pi)
                new = min(max(new, lo), hi)
                if self.over > STRIP_TURNS:
                    self.stripped = True
            else:
                self.over = max(0.0, self.over - abs(d) / (2 * math.pi))
            self.turns[k] = new
        self.last = roll
        st, sb = self.s("t"), self.s("b")
        for k in ("t", "b"):
            asset, j = m._joint(env, "door", f"screw_{k}")
            asset.set_joint_position_target(m._tensor(asset, self.turns[k] * 2 * math.pi), joint_ids=[j])
        for name, val in (("shift", sb), ("tilt", -(st - sb) / SPAN)):
            asset, j = m._joint(env, "door", name)
            asset.set_joint_position_target(m._tensor(asset, val), joint_ids=[j])


THREAD = Thread()
m.on_step(THREAD.step)
m.on_reset(THREAD.reset)


class Swing:
    """The hinge's own mechanism: friction holds the door where it is left above HOLD_OPEN_DEG; below it the self-close spring draws it shut."""

    def __init__(self) -> None:
        self.reset()

    def reset(self, env=None) -> None:
        self.rest = 0.0

    def step(self, env) -> None:
        q = m.state(env, "door").joint("swing")  # the joint turns anticlockwise (positive about +z) to open: q is the opening
        slip = HINGE_DRAG / HINGE_K
        if math.degrees(q) < HOLD_OPEN_DEG:
            self.rest = 0.0
        elif abs(q - self.rest) > slip:
            self.rest = q - math.copysign(slip, q - self.rest)
        m.set_target(env, "door", "swing", self.rest)


SWING = Swing()
m.on_step(SWING.step)
m.on_reset(SWING.reset)


# -- the scorer ------------------------------------------------------------------------------------

def link_pose(env, link: str):
    base = env.unwrapped
    asset = base.scene["door"]
    pose = m._np(asset.data.body_link_pose_w)[0, asset.body_names.index(link)]
    return m.quat_matrix(pose[3:7]), pose[:3] - m._np(base.scene.env_origins[0])


def door_gaps(env) -> tuple[float, float]:
    """The gap between the two doors (bottom, top), read at the left door's inner edge 20 mm from its bottom and top, from the door link's own pose."""
    R, t = link_pose(env, "swing")
    pts = np.array([[(DX0 + DX1) / 2, LY0, GAP_Z[0]], [(DX0 + DX1) / 2, LY0, GAP_Z[1]]])
    w = pts @ R.T + t
    return float(w[0, 1] - DY + REVEAL / 2), float(w[1, 1] - DY + REVEAL / 2)


def box_distance(pts: np.ndarray, lo, hi) -> float:
    d = np.maximum(np.maximum(np.asarray(lo) - pts, pts - np.asarray(hi)), 0.0)
    return float(np.linalg.norm(d, axis=1).min())


CAB_BOX = (np.array([XC - 0.060, DY - 0.30, TOP]), np.array([XC + CD, DY + 0.30, TOP + Z1]))  # both cabinets with their doors and pulls


def moment(c: dict) -> float:
    """The scoring text, on one moment's facts."""
    if c["stripped"]:
        return 0.0
    if c["shut"] and c["gaps_ok"]:
        return 1.0 if c["released"] else 0.5
    return 0.25 if c["turned"] else 0.0


class Trial:
    def __init__(self) -> None:
        self.reset()

    def reset(self, env=None) -> None:
        self.steps = 0
        self.end = gk.EndHold(HOLD_STEPS)
        self.score = 0.0
        self.report: dict = {}

    def step(self, env) -> None:
        self.steps += 1
        door = m.state(env, "door")
        drv = m.state(env, "driver")
        gb, gt = door_gaps(env)
        swing = door.deg("swing")  # the opening, degrees
        shut = abs(swing) <= SHUT_DEG
        gaps_ok = bool(GAP_LO <= gb <= GAP_HI and GAP_LO <= gt <= GAP_HI and abs(gt - gb) <= GAP_EVEN)
        tips = np.concatenate([m.fingertips(env), m.hand_body(env)])  # each whole gripper: fingers, pads, palm, housing and the wrist link
        R, t = link_pose(env, "swing")
        local = (tips - t) @ R  # in the door's frame as authored
        hands = min(box_distance(tips, *CAB_BOX), box_distance(local, (DX0 - 0.060, LY0, ZD0), (DX1 + 0.02, LY1, ZD0 + DH)),
                    box_distance(drv.to_local(tips), (-SD_L - 0.01, -SD_R, -SD_R), (BIT + 0.01, SD_R, SD_R)))
        tool_clear = min(float(np.linalg.norm(drv.to_world(np.array([[BIT, 0.0, 0.0]]))[0] - f)) for f in HEAD.values()) > 0.02
        still = abs(float(door.ang_vel[2])) < 0.05 and float(np.linalg.norm(drv.lin_vel)) < 0.02
        released = bool(hands >= CLEAR and tool_clear and still)
        c = {"stripped": THREAD.stripped, "shut": shut, "gaps_ok": gaps_ok, "turned": THREAD.turns["t"] >= TURN_MIN, "released": released}
        mo = moment(c)
        self.end.step(mo == 1.0)
        self.score = 1.0 if self.end.count >= HOLD_STEPS else min(mo, 0.5)  # every tier on the state now; only the gate (a stripped thread) holds
        self.report = {"step": self.steps, **c, "gap_mm": [round(1000 * gb, 2), round(1000 * gt, 2)], "swing": round(swing, 2), "hands_mm": round(1000 * hands, 1),
                       "turns": {k: round(v, 3) for k, v in THREAD.turns.items()}, "engaged": THREAD.engaged, "push_n": {k: round(v, 1) for k, v in THREAD.push.items()}, "over": round(THREAD.over, 2), "score": self.score}


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.end.count >= HOLD_STEPS


def score(objects) -> float:
    return TRIAL.score


# -- the demo ------------------------------------------------------------------------------------

def demo_script(ctx):
    """The left hand takes the screwdriver by its handle, in line with the hand, from the cabinets' top and holds it up. The right hand takes the left door's bar pull,
    swings the door open 55 degrees toward the middle and keeps hold of the pull. The left hand lowers the driver in front of the cabinet, brings it onto the
    line 10 degrees off the top hinge's side screw's axis, slides the bit through the opening onto the head, presses it in (about 13 N, read at the head's seat) and turns
    it with the wrist's roll; between strokes it backs the bit off and winds the wrist back. It lays the driver back on the top and moves clear; the right hand, still holding the pull,
    swings the door shut and lets go. It steers on the door's pose, the head's angle and the driver's pose, all in view.

    Check modes (TASK_DEMO): none; door_only (opens and shuts the door, never touches a screw); hover (the bit pressed into the top screw, never turned); bottom (turns the
    bottom hinge's screw instead); reverse (turns the top screw the wrong way); open_end (turns the top screw right but leaves the door open); no_push (the bit 1.5 mm short of the head, turned: hover-turn);
    off_centre (the tip pushed 2.8 mm off the recess's centre, turned: an unseated turn)."""
    import demokit as d
    from yam_arena import ik
    from yam_arena.constants import FINGER_OPEN
    from yam_arena.urdf import axis_angle_matrix

    env = ctx.emb._env
    l, r = ctx.left, ctx.right
    mode = ctx.mode or "full"
    if mode not in ("full", "none", "door_only", "hover", "bottom", "reverse", "open_end", "no_push", "off_centre", "pick"):
        raise ValueError(f"unknown mode {mode!r}")
    for _ in range(10):
        yield
    if mode == "none":
        for _ in range(100):
            yield
        return
    hinge = np.array([XH, YH + DY, TOP])
    up = np.array([0.0, 0.0, 1.0])
    ex = np.array([1.0, 0.0, 0.0])
    u = np.array([math.cos(math.radians(SD_YAW)), math.sin(math.radians(SD_YAW)), 0.0])  # the tool's axis at the screw, handle to tip
    dr = lambda: ctx.state("driver")  # noqa: E731
    tip = lambda: dr().to_world(np.array([[BIT, 0.0, 0.0]]))[0]  # noqa: E731
    swing_deg = lambda: math.degrees(ctx.state("door").joint("swing"))  # noqa: E731
    head_turns = lambda k: ctx.state("door").joint(f"screw_{k}") / (2 * math.pi)  # noqa: E731  the head's own angle, as a camera shows it
    tilt = lambda: math.degrees(math.acos(float(np.clip(dr().rotation()[:, 0] @ EIN, -1, 1))))  # noqa: E731  the bit's angle off the screw's axis
    in_hand = lambda: math.degrees(math.acos(float(np.clip(l.grip.rot[:, 2] @ dr().rotation()[:, 0], -1, 1))))  # noqa: E731  the driver's angle off the hand's axis

    def say(label):
        print(f"[hinge] t {ctx.t:5.1f} {label}: door {swing_deg():.1f} deg, bit {tilt():.1f} deg off the screw, {in_hand():.1f} off the hand, left tcp {np.round(l.tcp() - [0, 0, TOP], 3)} "
              f"tip {np.round(tip() - [0, 0, TOP], 4)}, {TRIAL.report}", flush=True)

    # -- the right hand: the door, by its pull
    def pull_now():
        R, t = link_pose(env, "swing")
        p = R @ np.array([DX0 - 0.030, LY1 - 0.030, 0.255]) + t
        f = R @ np.array([1.0, 0.0, 0.0])  # the door's own x: into its outer face
        return p, d.level(math.degrees(math.atan2(f[1], f[0])) + 15.0)  # 15 degrees round the bar, so the hand ends on it

    def grab_pull():
        p, g = pull_now()
        a = g.rot[:, 2]
        yield from r.move(p - 0.075 * a, grip=g, finger=d.FINGER_OPEN, t=2.5, label="right: in front of the pull")
        yield from r.move(p - 0.013 * a, t=1.2, label="right: round the pull")
        yield from r.close(0.012, 30.0, t=0.6, label="right: take the pull")

    def swing_to(deg):
        n = max(1, int(abs(deg - swing_deg()) // 7.5))
        step = (deg - swing_deg()) / n
        for _ in range(n):
            yield from r.arc(hinge, up, step, t=0.8, label="right: swing the door")

    def let_go_right():
        a = r.grip.rot[:, 2]
        yield from r.open(t=0.5, label="right: let go of the pull")
        yield from r.move(r.at() - 0.07 * a, t=1.0, label="right: off the pull")
        yield from r.move(r.at() + np.array([-0.06, -0.08, 0.04]), t=1.2, label="right: back toward its base")

    # -- the left hand: the screwdriver, held in line with the hand
    gr = d.level(SD_YAW)
    h0 = dr().to_world(np.array([[-GRIP_AT, 0.0, 0.0]]))[0]  # the hand's point on the handle
    yield from l.move(h0 + 0.10 * up, grip=gr, finger=d.FINGER_OPEN, t=3.0, label="left: over the screwdriver's handle")
    if mode != "door_only":
        yield from l.move(h0, t=1.2, label="left: fingers either side of the handle")
        yield from l.servo(h0, tol=0.002, timeout=1.0, label="left: at the handle")
        yield from l.close(SD_FLAT, GRIP_N, t=0.8, label="left: grip the handle by two flats")
        yield from l.move(l.at() + 0.04 * up, t=1.0, label="left: lift")
        say("lifted")
        if mode == "pick":
            return
    yield from grab_pull()
    yield from swing_to(DOOR_OPEN)
    yield from r.wait(0.5)
    say("door open")  # the right hand keeps the pull, holding the door open
    if mode == "door_only":
        yield from grab_pull()
        yield from swing_to(0.0)
        yield from r.wait(1.0)
        yield from let_go_right()
        yield from r.wait(2.0)
        say("shut")
        return

    def turn_onto(a, b):
        a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
        v, c = np.cross(a, b), float(np.dot(a, b))
        k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        return np.eye(3) + k + k @ k / (1.0 + c)

    def square():
        """Turn the hand so the driver, as it sits in the fingers, points along the tool's line."""
        for _ in range(2):
            g = d._as_grip(turn_onto(dr().rotation()[:, 0], u) @ l.grip.rot)
            yield from l.move(l.at(), grip=g, t=0.6, label="left: square the screwdriver")

    k = "b" if mode == "bottom" else "t"
    face = lambda: HEAD[k].copy()  # noqa: E731  the head's face centre, in view
    aim = (lambda: face() + np.array([0.0, 0.0, 0.0028])) if mode == "off_centre" else face  # noqa: E731  off_centre: the tip 2.8 mm above the recess's centre
    tip_to = lambda p: l.at() + (p - tip())  # noqa: E731  the hand's command that puts the tip at p
    near = lambda: face() - 0.006 * u  # noqa: E731  the tip on the line, 6 mm short of the head
    front = lambda: near() - 0.055 * ex  # noqa: E731  the same, out in front of the cabinet
    yield from l.move(tip_to(np.array([front()[0], front()[1], TOP + Z1 + 0.05])), t=2.5, label="left: the bit out over the cabinets' front, above the screw")
    yield from square()
    yield from l.move(tip_to(front()), t=2.5, label="left: the driver down in front of the cabinet")
    yield from l.servo(front(), tol=0.002, timeout=1.5, read=tip, label="left: the bit in front of the screw")
    yield from square()
    yield from l.move(tip_to(near()), t=2.0, label="left: the bit through the opening, onto the screw's line")
    yield from l.servo(near(), tol=0.0012, timeout=2.0, read=tip, label="left: the bit on the screw's line")
    say("on the line")

    sign = -1.0 if mode == "reverse" else 1.0
    push_to = 0.0 if mode == "no_push" else PUSH_GOAL  # the push the hand keeps on the bit, read at the head's seat
    ax = l.grip.rot[:, 2]  # which way joint 6 (the wrist roll, +-120 degrees) turns the hand about its approach axis
    q_try = ik.solve(l.at(), ik.matrix_to_quat_xyzw(axis_angle_matrix(ax, math.radians(10.0)) @ l.grip.rot), l.q, base_pos=l.base, iters=60)[0]
    jdir = sign * (1.0 if q_try[5] > l.q[5] else -1.0)  # joint 6's sense for the right way
    J6 = math.radians(112.0)  # each stroke runs joint 6 from -J6 to +J6 the right way
    st = {"lead": -0.006, "fix": np.zeros(3)}  # the tip's command along the line, m past the head's face; the hand's running correction

    def drive(q6, lead_to, t, label):
        """Joint 6 to `q6`, the hand turned about the driver's own axis while it holds the tip on the head's centre: `lead_to` along the line (None: pressing by `push_to`, read at the head's seat)."""
        qo = ctx.observed("left")[:6]
        p_o, R_o = ik.tcp_pose(qo, l.base)
        v = R_o.T @ (tip() - p_o)  # the tip in the hand's frame
        lag = ik.tcp_pose(l.q, l.base)[0] - p_o  # commanded less observed: the arm's droop here
        q0, l0 = l.q.copy(), st["lead"]
        R0 = ik.tcp_pose(q0, l.base)[1]
        ad = dr().rotation()[:, 0]  # the driver's axis as it sits in the fingers: the hand turns about it, so the bit keeps its line
        qa = q0.copy()
        qa[5] += 0.1
        dR = ik.tcp_pose(qa, l.base)[1] @ R0.T
        sg = 1.0 if np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]) @ ad > 0 else -1.0  # joint 6's sense about that axis
        worst = 0.0
        n = max(1, round(t * d.CONTROL_HZ))
        for i in range(1, n + 1):
            s_ = ik.smoothstep(i / n)
            qq = l.q.copy()
            qq[5] = q0[5] + (q6 - q0[5]) * s_
            Rn = axis_angle_matrix(ad, sg * (qq[5] - q0[5])) @ R0
            if lead_to is None:
                e = aim() - tip()
                e -= u * float(e @ u)  # across the line only: the push sets the depth
                if push_to > 0:
                    on = float(np.linalg.norm(e)) < ENGAGE_LAT  # pressed further only with the tip on the recess
                    st["lead"] = float(np.clip(st["lead"] + (0.00006 * (push_to - THREAD.push[k]) if on or THREAD.push[k] > push_to else 0.0), -0.004, 0.015))
                else:
                    st["lead"] = -0.0015
            else:
                st["lead"] = l0 + (lead_to - l0) * s_
                e = aim() + st["lead"] * u - tip()  # the tip where the lead puts it, in all three directions
            if lead_to is not None or THREAD.push[k] < 2.0:  # a seated tip is held by the recess: the hand keeps its place rather than lean the driver over
                st["fix"] = np.clip(st["fix"] + np.clip(0.3 * e, -0.001, 0.001), -0.04, 0.04)
            q, pe, _ = ik.solve(aim() - Rn @ v + st["fix"] + lag + st["lead"] * u, ik.matrix_to_quat_xyzw(Rn), qq, base_pos=l.base, iters=60)
            worst = max(worst, pe)
            l.q = q
            l.cmd = np.r_[q, float(np.clip(l.finger / FINGER_OPEN, 0.0, 1.0))]
            yield
        l.grip = d._as_grip(ik.tcp_pose(l.q, l.base)[1])
        e = tip() - aim()
        ho = ik.tcp_pose(ctx.observed("left")[:6], l.base)[1][:, 2]
        print(f"[hinge] {label}: j6 {math.degrees(q0[5]):.0f} to {math.degrees(l.q[5]):.0f} (obs {math.degrees(ctx.observed('left')[5]):.0f}), ik worst {1000 * worst:.1f} mm, across {1000 * np.linalg.norm(e - u * (e @ u)):.1f} mm, "
              f"along {1000 * (e @ u):.1f} mm, push {THREAD.push[k]:.1f} N, head {head_turns(k):+.3f}, bit {tilt():.1f} deg, in hand {math.degrees(math.acos(float(np.clip(ho @ dr().rotation()[:, 0], -1, 1)))):.1f} deg", flush=True)

    q6_home = l.q[5]
    yield from drive(q6_home - jdir * J6, -0.006, 2.0, "left: the wrist wound back, the bit short of the head")
    for _ in range(3):
        yield from drive(l.q[5], None, 1.5, "left: the bit pressed into the recess")
        if THREAD.push[k] >= push_to - 2.0:
            break
    say("pressed")
    print(f"[hinge] pressed: push {THREAD.push[k]:.1f} N, lead {1000 * st['lead']:.1f} mm, engaged {THREAD.engaged}", flush=True)
    if mode == "hover":
        yield from drive(l.q[5], None, 6.0, "hover: pressed, never turned")
        say("hover end")
        return
    need = 0.63  # 0.28 to 0.99 turns puts both gaps in the window; 0.63 squares the door
    for n in range(8):
        left_turns = need - head_turns(k) * sign
        if left_turns < 0.15:
            break
        go = min(2 * J6, math.radians(left_turns * 360.0))
        yield from drive(l.q[5] + jdir * go, None, 1.5 * go + 1.0, "left: turn the bit with the wrist")
        print(f"[hinge] stroke {n}: push {THREAD.push[k]:.1f} N, head {head_turns(k):+.3f} turns, bit {tilt():.1f} deg, engaged {THREAD.engaged}, tip to face {np.round(1000 * (tip() - face()), 1)} mm", flush=True)
        if need - head_turns(k) * sign < 0.15 or head_turns(k) * sign >= 0.35:  # from 0.28 turns both gaps are in the window
            break
        yield from drive(l.q[5], -0.012, 1.0, "left: the bit off the head")
        yield from drive(q6_home - jdir * J6, -0.012, 2.5, "left: the wrist wound back")
        yield from drive(l.q[5], None, 1.5, "left: the bit pressed in again")
    say("turned")
    # -- the driver out and back on the top; the door shut; both hands clear
    yield from drive(l.q[5], -0.010, 0.6, "left: the bit off the head")
    yield from l.move(tip_to(near()), t=1.0, label="left: the bit on the line")
    yield from l.move(tip_to(front()), t=1.5, label="left: the bit out of the cabinet")
    yield from l.move(tip_to(np.array([front()[0], front()[1], TOP + Z1 + 0.05])), t=2.0, label="left: up past the cabinets' top")
    q_from = l.q.copy()
    n = round(2.0 * d.CONTROL_HZ)
    for i in range(1, n + 1):  # the wrist unwound in the air, joint 6 alone
        l.q = q_from.copy()
        l.q[5] = q_from[5] + (q6_home - q_from[5]) * ik.smoothstep(i / n)
        l.cmd = np.r_[l.q, float(np.clip(l.finger / FINGER_OPEN, 0.0, 1.0))]
        yield
    l.grip = d._as_grip(ik.tcp_pose(l.q, l.base)[1])
    hp = lambda: dr().to_world(np.array([[-GRIP_AT, 0.0, 0.0]]))[0]  # noqa: E731  the hand's point on the handle, as the driver sits now
    lie = np.array([math.cos(math.radians(SD_YAW)), math.sin(math.radians(SD_YAW)), 0.0])
    Rd = turn_onto(dr().rotation()[:, 0], lie)  # the hand turned so the driver lies as it lay
    w = Rd @ (hp() - l.at())
    yield from l.move(h0 + 0.04 * up - w, grip=d._as_grip(Rd @ l.grip.rot), t=2.5, label="left: back over where it lay")
    yield from l.move(l.at() + (h0 + 0.004 * up - hp()), t=1.2, label="left: down")
    yield from l.open(t=0.6, label="left: let go of the driver")
    yield from l.move(l.at() + 0.10 * up, t=1.0, label="left: up off the handle")
    yield from l.move(l.at() + np.array([-0.10, 0.04, 0.0]), t=1.2, label="left: back toward its base")
    say("driver down")
    if mode == "open_end":
        yield from l.wait(3.0)
        return
    yield from swing_to(0.0)
    yield from r.wait(1.0)
    yield from let_go_right()
    yield from r.wait(1.5)
    say("door shut")
    for _ in range(60):
        yield
    say("end")
