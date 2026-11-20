"""Fit the crevice tool to a cordless vacuum used as a handheld, switch it on, and vacuum the grit out of a narrow gap.

On the station table a Dyson V8 Cyclone's main body (the handheld, no wand) stands on a wooden block, its inlet
pointing forward. In front of the inlet its quick-release crevice tool rests in a wooden cradle, its collar 30 mm from
the inlet. To the right a gap 30 mm wide, 100 mm deep and 260 mm long runs between two wooden boards (like the gap
between a car seat and the centre console), closed at the far end and open towards the robot but for a 6 mm lip;
2-3 mm quartz grit lies along its floor.

What makes it hard: the tool's collar has to go into the inlet square until its catch clicks (Dyson's manual: tools
"click" on); the machine is heavy (about 1.7 kg) and long (with the tool on, the tip is 44 cm from the handle), and
held by its handle in a two-finger grip it sags and turns; the gap is too deep for fingers and too narrow for the bare
inlet, so the tool has to go in tilted and be drawn along the floor of the gap from end to end without catching on the
boards.

Simulation: the vacuum, the tool and the catch are rigid bodies (partkit), the grit about 1,400 rigid spheres
(soilkit). Suction is not simulated as airflow: while the machine is on, a grain whose surface comes within 4 mm of
the opening (the tool's while it is clicked on, else the bare inlet's) is taken into the machine (CleanControlling:
suction inlets work 2-6 mm from the surface). The vacuum's sizes are scaled from the drawing in Dyson's V8 Cyclone
user manual to the 320 mm body length.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 ...
"""

from __future__ import annotations

import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import partkit  # noqa: E402
import soilkit  # noqa: E402
from partkit import Slider, part  # noqa: E402
from scenekit import Shape, on_reset, on_step  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = "Fit the crevice tool to the vacuum, switch it on and vacuum the grit out of the gap."

MM = 0.001
TOP = TABLE_TOP[SCENE]
WOOD = (0.72, 0.58, 0.40)
GREY, PURPLE, RED, DARK = (0.80, 0.80, 0.82), (0.55, 0.35, 0.65), (0.85, 0.10, 0.10), (0.25, 0.25, 0.27)

# -- the vacuum: Dyson V8 Cyclone main body ------------------------------------------------------------------------
# Part frame: x along the inlet (forward), z up, origin on the base plane right below the inlet's front end. Sizes are
# scaled from the side view in Dyson's V8 Cyclone user manual ("Machine body", p. 2) to the body's 320 mm length
# (Smart Home Fox, V8 handheld "32 cm plus attachment"): 0.708 mm per drawing pixel at 300 dpi. Widths across (y) are
# not in a side view: the round parts are taken as round, handle and battery as drawn below.
S = 0.320 / 452


def _x(px):
    return (px - 482) * S


def _z(py):
    return (288 - py) * S


AXIS_Z = _z(158.5)  # the inlet's axis, 91.7 mm above the base
BORE_R = 18.0 * MM  # the inlet's bore (0.5 mm clearance round the tool's 35 mm collar)
BORE_D = 30 * MM  # how far the collar goes in
WALL = 4 * MM
TOOTH_X = -10 * MM  # the catch's tooth, on top of the bore, this far inside the inlet's mouth
TOOTH_REST_R = 17.6 * MM  # its underside at rest, from the axis (0.1 mm above the collar's top)
HANDLE_C = (_x(135), 0.0, _z(190))  # the handle's middle, where the hand holds it
HANDLE_TILT = np.radians(20)  # the handle leans back 20 deg at its foot
BUTTON = (_x(120), 0.0, _z(20))  # the power button on top of the motor housing
# The centre of mass is not published: the V8 balances in the hand, with the battery under the handle and the motor
# above it; taken over the handle, 30 mm above its middle.
VAC_COM = (HANDLE_C[0], 0.0, HANDLE_C[2] + 0.030)
VAC_MASS = 1.74  # kg: V8 handheld 1.58 kg (Dyson spec, TechRadar) + 0.16 kg (V8 Cyclone 2.70 kg vs V8 2.54 kg whole)


def _ring(x0, x1, r_in, wall, color, gap_top=0.0):
    """An octagonal tube along x (part frame) at the inlet's axis; ``gap_top``: leave a channel this wide on top."""
    out = []
    width = 2 * (r_in + wall) * np.tan(np.pi / 8) + 1 * MM
    for k in range(8):
        a = k * np.pi / 4
        c = np.array([(x0 + x1) / 2, (r_in + wall / 2) * np.sin(a), AXIS_Z + (r_in + wall / 2) * np.cos(a)])
        q = (np.cos(-a / 2), np.sin(-a / 2), 0.0, 0.0)  # about x
        if k == 0 and gap_top:
            for side in (-1, 1):  # two half-plates either side of the channel
                w = (width - gap_top) / 2
                out.append(((c[0], side * (gap_top + w) / 2, c[2]), (x1 - x0, w, wall), color))
            continue
        out.append((tuple(c), (x1 - x0, width, wall), color, q))
    return out


def _qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def _chamfer(r_in, wall, color, length=9 * MM, flare=3.3 * MM):
    """A lead-in at the socket's mouth: eight plates flaring out from the bore over ``length`` ahead of it."""
    phi = np.arctan2(flare, length)
    width = 2 * (r_in + flare + wall) * np.tan(np.pi / 8) + 1 * MM
    out = []
    for k in range(1, 8):  # none on top: the collar's rib runs in the socket's top channel
        a = k * np.pi / 4
        # the plate's inner face runs from the bore's edge at the mouth (x 0, r_in) to (length, r_in + flare)
        r = r_in + flare / 2 + wall / 2 * np.cos(phi)
        c = (length / 2 - wall / 2 * np.sin(phi), r * np.sin(a), AXIS_Z + r * np.cos(a))
        q = _qmul((np.cos(-a / 2), np.sin(-a / 2), 0.0, 0.0), (np.cos(-phi / 2), 0.0, np.sin(-phi / 2), 0.0))
        out.append((c, (length / np.cos(phi), width, wall), color, q))
    return out


VACUUM_BOXES = [
    # handle, leaning back at its foot
    (HANDLE_C, (40 * MM, 34 * MM, 83 * MM), GREY, (np.cos(HANDLE_TILT / 2), 0.0, np.sin(HANDLE_TILT / 2), 0.0)),
    ((_x(130), 0.0, _z(266.5)), (149 * MM, 60 * MM, 30 * MM), DARK),  # battery, under the handle
    ((_x(225), 0.0, _z(80)), (30 * MM, 50 * MM, 60 * MM), GREY),  # neck between motor and cyclone
    *_ring(-BORE_D, 0.0, BORE_R, WALL, PURPLE, gap_top=8 * MM),  # the inlet's socket
    *_chamfer(BORE_R, WALL, PURPLE),  # and its lead-in
    ((TOOTH_X, 0.0, AXIS_Z + BORE_R + WALL + 4 * MM), (16 * MM, 16 * MM, 8 * MM), PURPLE),  # the catch's housing
]
VACUUM_CYLINDERS = [
    ((_x(122.5), 0.0, _z(75)), 39 * MM, 131 * MM, GREY, "X"),  # motor and filter
    ((_x(312.5), 0.0, 95 * MM), 51 * MM, 190 * MM, GREY, "Z"),  # cyclone and bin
    (((_x(345) - BORE_D) / 2, 0.0, AXIS_Z), 26 * MM, -BORE_D - _x(345), PURPLE, "X"),  # duct, behind the socket
]
_TOOTH_H = 6 * MM
TOOTH = Slider("catch", rest=(TOOTH_X, 0.0, AXIS_Z + TOOTH_REST_R + _TOOTH_H / 2),
               boxes=(((TOOTH_X, 0.0, AXIS_Z + TOOTH_REST_R + _TOOTH_H / 2), (6 * MM, 8 * MM, _TOOTH_H), RED),),
               travel=3 * MM, rate=600.0, damping=1.0, mass=0.002, axis="Z", sign=1)
BUTTON_SLIDER = Slider("button", rest=(BUTTON[0], 0.0, BUTTON[2] + 2 * MM),
                       cylinders=(((BUTTON[0], 0.0, BUTTON[2] + 2 * MM), 7 * MM, 4 * MM, RED, "Z"),),
                       travel=2 * MM, rate=2500.0, damping=5.0, mass=0.002, axis="Z", sign=-1)

# -- the crevice tool (Dyson 967612-01) ------------------------------------------------------------------------------
# Part frame: x along the tool from its collar's back end (which goes into the inlet), on its axis. Dyson: "reaches up
# to 175 mm into tight, narrow spaces as small as 17.2 mm"; collar 35 mm across (vhbw/ManoMano listings, and the
# manual's drawing scaled to the 175 mm reach); blade 24.7 mm deep (same drawing). Mass not published: 40 g, a 1.5 mm
# ABS shell (1.05 g/cm3) of this size.
COLLAR_R, COLLAR_L = 17.5 * MM, 55 * MM
BLADE_T, BLADE_W, REACH = 17.2 * MM, 24.7 * MM, 175 * MM
TOOL_L = COLLAR_L + REACH
TOOL_MASS = 0.040
NOTCH = (16 * MM, 24 * MM)  # the groove in the collar's top rib the catch's tooth drops into (from the back end)
RIB_TOP = 18.6 * MM  # the rib on the collar's top, ahead of the groove, lifts the tooth 1.0 mm on its way in
_ramp = np.arctan2(RIB_TOP - 17.3 * MM, 4 * MM)
TOOL_BOXES = [
    ((NOTCH[0] / 2 + 2 * MM, 0.0, (COLLAR_R + RIB_TOP) / 2 - 0.5 * MM), (NOTCH[0] - 4 * MM, 6 * MM, RIB_TOP - COLLAR_R + MM),
     PURPLE),  # the rib from 4 mm to the groove
    ((2 * MM, 0.0, (17.3 * MM + RIB_TOP) / 2 - 0.6 * MM), (4.4 * MM, 6 * MM, 1.2 * MM), PURPLE,
     (np.cos(-_ramp / 2), 0.0, np.sin(-_ramp / 2), 0.0)),  # its ramp, rising from the back end
    ((COLLAR_L + REACH / 2, 0.0, 0.0), (REACH, BLADE_T, BLADE_W), GREY),  # the blade, thin side across (y)
]
TOOL_CYLINDERS = [((COLLAR_L / 2, 0.0, 0.0), COLLAR_R, COLLAR_L, PURPLE, "X")]

# -- the layout ------------------------------------------------------------------------------------------------------
BLOCK_H = 90 * MM  # the wooden block the vacuum stands on
VAC_Y = -0.060
HANDLE_X = 0.250  # the handle's middle at the start
VAC_X = HANDLE_X - HANDLE_C[0]  # the part origin (inlet's front end)
BLOCK = Shape("block", pos=(VAC_X + _x(200), VAC_Y, TOP + BLOCK_H / 2), size=(0.28, 0.14, BLOCK_H), color=WOOD)
VACUUM = part("vacuum", pos=(VAC_X, VAC_Y, TOP + BLOCK_H + 0.5 * MM), boxes=VACUUM_BOXES, cylinders=VACUUM_CYLINDERS,
              sliders=(TOOTH, BUTTON_SLIDER), mass=VAC_MASS, color=GREY, friction=0.8, com=VAC_COM, torsional_patch=0.010)
GAP_TO_TOOL = 30 * MM
LIFT = 6 * MM  # the vacuum is carried this far above its block while the tool goes on
TOOL0 = np.array([VAC_X + GAP_TO_TOOL, VAC_Y, TOP + BLOCK_H + 0.5 * MM + LIFT + AXIS_Z])  # the tool's back end, on its axis
TOOL = part("crevice", pos=tuple(float(v) for v in TOOL0), boxes=TOOL_BOXES, cylinders=TOOL_CYLINDERS,
            mass=TOOL_MASS, color=PURPLE, friction=0.4)  # ABS on ABS


def _cradle():
    """Two notched wooden posts under the collar and the blade, and a stop in front of the tip."""
    ax, (tx, ty, tz) = TOOL0[2], TOOL0
    out = []
    for name, x0, x1, half, under in (("collar", 40 * MM, 54 * MM, COLLAR_R, COLLAR_R),
                                      ("blade", 150 * MM, 180 * MM, BLADE_T / 2, BLADE_W / 2)):
        seat = ax - under - 0.3 * MM
        cx, lx = tx + (x0 + x1) / 2, x1 - x0
        out.append(Shape(f"cradle_{name}", pos=(cx, ty, (TOP + seat) / 2), size=(lx, 0.05, seat - TOP), color=WOOD))
        for side in (-1, 1):
            out.append(Shape(f"cradle_{name}_cheek{side + 1}", pos=(cx, ty + side * (half + 2 * MM + 5 * MM), seat + 6 * MM),
                             size=(lx, 10 * MM, 12 * MM), color=WOOD))
    stop_x = tx + TOOL_L + 1 * MM + 5 * MM
    out.append(Shape("cradle_stop", pos=(stop_x, ty, (TOP + tz + 15 * MM) / 2), size=(10 * MM, 0.04, tz + 15 * MM - TOP),
                     color=WOOD))
    return out


CRADLE = _cradle()

# the gap: two 18 mm boards 30 mm apart, 100 mm tall, 260 mm long, on the table; closed at the far end, and at the
# near end (towards the robot) only by a 6 mm lip, as a car seat's gap opens towards the footwell
GAP_Y, GAP_W, GAP_D = -0.225, 30 * MM, 100 * MM
GAP_X0, GAP_X1 = 0.500, 0.760
BOARD_T = 18 * MM
LIP_H = 6 * MM
RIG = [
    *(Shape(f"board_{n}", pos=((GAP_X0 + GAP_X1) / 2, GAP_Y + side * (GAP_W + BOARD_T) / 2, TOP + GAP_D / 2),
            size=(GAP_X1 - GAP_X0 + 2 * BOARD_T, BOARD_T, GAP_D), color=WOOD) for n, side in (("left", 1), ("right", -1))),
    Shape("end_far", pos=(GAP_X1 + BOARD_T / 2, GAP_Y, TOP + GAP_D / 2), size=(BOARD_T, GAP_W, GAP_D), color=WOOD),
    Shape("lip_near", pos=(GAP_X0 - 3 * MM, GAP_Y, TOP + LIP_H / 2), size=(6 * MM, GAP_W, LIP_H), color=WOOD),
]

# the grit: rounded quartz aquarium gravel, 2-3 mm (Olibetta Crystal Quartz 2-3 mm), quartz 2.65 g/cm3
GRIT = soilkit.Soil("grit", pos=((GAP_X0 + GAP_X1) / 2, GAP_Y, TOP + 3.5 * MM),
                    size=(GAP_X1 - GAP_X0 - 4 * MM, GAP_W - 2 * MM, 7 * MM), r_min=1.0 * MM, r_max=1.5 * MM,
                    density=2650.0, friction=(0.6, 0.5), angular_damping=2.0,
                    colors=((0.85, 0.83, 0.78), (0.70, 0.68, 0.64), (0.92, 0.90, 0.86), (0.55, 0.53, 0.50)),
                    start_file=os.environ.get("GRIT_START", "assets/grit_start.npz"))
BIN_AT = np.array([-0.30, -0.70, TOP - 0.40])  # where a vacuumed grain goes (out of sight below the table)

OBJECTS = [BLOCK, VACUUM, TOOL, *CRADLE, *RIG, GRIT]

# -- state: the power switch, the catch, suction ---------------------------------------------------------------------
CAPTURE = 4 * MM  # a grain whose surface comes within this of the opening (the tool's, or the bare inlet's) is
# taken: the middle of the 2-6 mm that suction inlets work at (CleanControlling)
SEATED = 2.5 * MM  # the collar's back end at most this far from the socket's floor (the groove's 2 mm play, held
# out against the catch by the tool's weight, plus 0.5 mm)
ALIGNED = np.radians(3)
PRESSED = -1.0 * MM
RELEASED = -0.3 * MM  # and released again above this  # the button counts as pressed below this
_STATE: dict = {"on": False, "pressed": False, "taken": set(), "clicked": False, "t": 0.0, "last": -1.0}


def tool_in_vacuum(vac, tool):
    """(the tool's back end in the vacuum's frame, angle between their axes)."""
    back = vac.to_local(tool.pos)
    axis = vac.rotation().T @ tool.rotation()[:, 0]
    return back, float(np.arccos(np.clip(axis[0], -1.0, 1.0)))


def seated(vac, tool) -> bool:
    """The collar fully home in the inlet, square to it: its back end on the socket's floor, on the axis."""
    back, angle = tool_in_vacuum(vac, tool)
    on_axis = np.hypot(back[1], back[2] - AXIS_Z) < 1.5 * MM
    return bool(abs(back[0] + BORE_D) < SEATED and on_axis and angle < ALIGNED)


def clicked(vac, tool) -> bool:
    """Seated with the catch's tooth down in the collar's groove (it rose over the rib and dropped back)."""
    return seated(vac, tool) and vac.sliders.get("catch", 0.0) < 0.5 * MM


def opening(vac, tool, attached: bool):
    """Where the suction acts: (centre, its frame's rotation, half sizes across y and z) of the tool's opening when
    the tool is on, else of the bare inlet."""
    if attached:
        R = tool.rotation()
        return tool.to_world((TOOL_L, 0.0, 0.0)), R, (BLADE_T / 2, BLADE_W / 2)
    R = vac.rotation()
    return vac.to_world((0.0, 0.0, AXIS_Z)), R, (BORE_R, BORE_R)


def captured(points, radii, vac, tool, attached: bool) -> np.ndarray:
    """Which grains are at the opening: their surface within CAPTURE of its face (or up to 3 mm inside it) and of its
    outline."""
    c, R, (hy, hz) = opening(vac, tool, attached)
    local = (np.asarray(points) - c) @ R
    reach = radii + CAPTURE
    return ((local[:, 0] > -3 * MM) & (local[:, 0] < reach)
            & (np.abs(local[:, 1]) < hy + reach) & (np.abs(local[:, 2]) < hz + reach))


def in_gap(points) -> np.ndarray:
    p = np.asarray(points)
    return ((p[:, 0] > GAP_X0) & (p[:, 0] < GAP_X1) & (np.abs(p[:, 1] - GAP_Y) < GAP_W / 2)
            & (p[:, 2] > TOP - 5 * MM) & (p[:, 2] < TOP + GAP_D + 20 * MM))


def in_bin(points) -> np.ndarray:
    """Vacuumed: taken into the machine (the simulation moves a vacuumed grain to BIN_AT, below the table)."""
    return np.asarray(points)[:, 2] < TOP - 0.2


def vacuumed_fraction(points, radii) -> float:
    """The share of the grit's mass in the bin, as a real trial weighs it (the grains are of one density)."""
    m = np.asarray(radii) ** 3
    return float(m[in_bin(points)].sum() / m.sum())


def score(objects, grains=None) -> float:
    """By the share of the grit's mass vacuumed into the machine, provided the tool is clicked on at the end. Grit pushed out of the gap or dropped elsewhere does not count.
    0 if the tool is not on (or under 25 % vacuumed); 0.25 for 25 %; 0.5 for 50 %; 0.75 for 75 %; 1.0 for 85 % or more.
    """
    vac, tool = objects["vacuum"], objects["crevice"]
    p, r = grains if grains is not None else soilkit.grains("grit")
    if not clicked(vac, tool):
        return 0.0
    f = vacuumed_fraction(p, r)
    for need, s in ((0.85, 1.0), (0.75, 0.75), (0.50, 0.5), (0.25, 0.25)):
        if f >= need:
            return s
    return 0.0


def success(objects) -> bool:
    return score(objects) == 1.0


@on_reset
def _reset(environment) -> None:
    _STATE.update(on=False, pressed=False, taken=set(), clicked=False, t=0.0, last=-1.0)


@on_step
def _vacuum(environment) -> None:
    """The power button toggles on each press; while on, grains at the opening go to the bin."""
    try:
        vac = partkit._part_state(environment, VACUUM)
        tool = partkit._part_state(environment, TOOL)
        b = vac.sliders["button"]
        pressed = b < PRESSED or (_STATE["pressed"] and b < RELEASED)
        if pressed and not _STATE["pressed"]:
            _STATE["on"] = not _STATE["on"]
        _STATE["pressed"] = pressed
        _STATE["clicked"] = clicked(vac, tool)
        attached = seated(vac, tool)
        if _STATE["on"]:
            p, r = soilkit.grains("grit")
            take = np.flatnonzero(captured(p, r, vac, tool, attached))
            take = [i for i in take if i not in _STATE["taken"]]
            if take:
                n0 = len(_STATE["taken"])
                spots = [BIN_AT + np.array([((n0 + k) % 20) * 4 * MM, ((n0 + k) // 20) * 4 * MM, 0.0])
                         for k in range(len(take))]
                soilkit.move("grit", environment, take, spots)
                _STATE["taken"].update(take)
        _STATE["t"] += environment.unwrapped.step_dt
        if os.environ.get("VAC_LOG") and _STATE["t"] - _STATE["last"] >= 0.5:
            _STATE["last"] = _STATE["t"]
            back, angle = tool_in_vacuum(vac, tool)
            tip = tool.to_world((TOOL_L, 0, 0))
            print(f"LOG t={_STATE['t']:.1f} on={_STATE['on']} button={vac.sliders['button'] * 1000:.1f}mm "
                  f"catch={vac.sliders['catch'] * 1000:.2f}mm back=({back[0] * 1000:.1f},{back[1] * 1000:.1f},"
                  f"{(back[2] - AXIS_Z) * 1000:.1f})mm angle={np.degrees(angle):.1f}deg clicked={_STATE['clicked']} "
                  f"tip=({tip[0]:.3f},{tip[1]:.3f},{(tip[2] - TOP) * 1000:.0f}mm) "
                  f"vac=({vac.pos[0]:.3f},{vac.pos[1]:.3f},{(vac.pos[2] - TOP) * 1000:.0f}mm) "
                  f"heading={np.degrees(np.arctan2(vac.rotation()[1, 0], vac.rotation()[0, 0])):.1f}deg "
                  f"pitch={np.degrees(np.arcsin(-vac.rotation()[2, 0])):.1f}deg "
                  f"taken={len(_STATE['taken'])} vacuumed={vacuumed_fraction(*soilkit.grains('grit')) * 100:.0f}% "
                  f"{_hand(environment, vac)} {_left(soilkit.grains('grit')[0])}",
                  flush=True)
    except Exception:
        import traceback

        traceback.print_exc()
        raise


def _left(points) -> str:
    """Grit still in the gap, by 5 mm bands across it (right to left), for the log."""
    p = np.asarray(points)
    p = p[in_gap(p)]
    h = np.histogram(p[:, 1] - GAP_Y, bins=np.arange(-0.015, 0.0151, 0.005))[0]
    return f"left_in_gap={h.tolist()}"


def _hand(environment, vac) -> str:
    """The right gripper's finger roots in the vacuum's frame, relative to the handle's middle (mm), for the log."""
    from scenekit import robot_links

    r = robot_links(environment)["right"]
    tl, tr = (vac.to_local(r[k]) - np.array(HANDLE_C) for k in ("tip_left", "tip_right"))
    return f"fingers_l={np.round(tl * 1000).astype(int).tolist()} r={np.round(tr * 1000).astype(int).tolist()}"


# -- the scripted demo ---------------------------------------------------------------------------------------------
PRESS_IN = float(os.environ.get("PRESS_IN", "0.020"))  # the fingertips' target past the button's top (m)
PITCH = np.radians(float(os.environ.get("PITCH", "40")))  # nose-down tilt while vacuuming


WALL_PRESS = float(os.environ.get("WALL_PRESS", "0.007"))  # how far past touching a board the blade is aimed (m)
TIP_UP = float(os.environ.get("TIP_UP", "0.007"))  # the opening's lower edge this far above the floor (planned)
AIM_YAW = np.radians(float(os.environ.get("AIM_YAW", "0.0")))
AIM_DY = float(os.environ.get("AIM_DY", "0.002"))


def _rot_z(a) -> np.ndarray:
    return np.array([[np.cos(a), -np.sin(a), 0.0], [np.sin(a), np.cos(a), 0.0], [0.0, 0.0, 1.0]])


def _rot_y(a) -> np.ndarray:
    return np.array([[np.cos(a), 0.0, np.sin(a)], [0.0, 1.0, 0.0], [-np.sin(a), 0.0, np.cos(a)]])


def demo():
    """Left arm: press the power button on top with its closed fingertips, leaning over from its side. Right arm, then:
    hold the handle from behind, push the inlet onto the tool's collar until it clicks, lift the tool out of its
    cradle, tilt nose-down into the gap at its far end and draw the tip along its floor to the near end and back."""
    from yam_arena.constants import FINGER_CLOSED as C
    from yam_arena.constants import FINGER_OPEN as O
    from yam_arena.demo import Waypoint as W

    import tiltkit

    tiltkit.install()
    park = (0.20, 0.10, TOP + 0.20)
    c = np.cos(np.radians(45))
    tip_dir = np.array([0.0, -c, -c])
    press = tiltkit.rotation(tip_dir, (-1, 0, 0))
    reach = 13 * MM  # the fingertips' ends ahead of the TCP (pads 26.4 mm long, centred on it)
    button_top = np.array([VAC_X + BUTTON[0], VAC_Y, TOP + BLOCK_H + 0.5 * MM + BUTTON[2] + 4 * MM])

    def fingertips_at(back):
        return tuple(float(v) for v in button_top - tip_dir * (reach + back))

    left = [
        W(3.0, fingertips_at(0.02), C, "over the power button", tiltkit.Tilt(press, joint=True)),
        W(1.2, fingertips_at(-PRESS_IN), C, "press", tiltkit.Tilt(press)),
        W(0.8, fingertips_at(0.02), C, "let go", tiltkit.Tilt(press)),
        W(2.5, park, C, "back", tiltkit.Tilt(None, joint=True)),
    ]
    t_left = sum(w.duration for w in left)

    level = tiltkit.rotation((1, 0, 0), (0, -1, 0))  # approach forward, fingers closing across the handle
    grip = tiltkit.Tilt(level)
    # what the hand carries (the arm sags under it; the plan aims past it): the vacuum, then the vacuum and the tool
    c_vac = np.array(VAC_COM) - np.array(HANDLE_C)  # in the vacuum's frame, from the hand
    c_tool = np.array([-BORE_D + TOOL_L / 2, 0.0, AXIS_Z]) - np.array(HANDLE_C)
    m_both = VAC_MASS + TOOL_MASS
    vac_only = (VAC_MASS, level.T @ c_vac)
    with_tool = (m_both, level.T @ ((VAC_MASS * c_vac + TOOL_MASS * c_tool) / m_both))
    carry, carry_tool = tiltkit.Tilt(level, load=vac_only), tiltkit.Tilt(level, load=with_tool)
    hx, hy, hz = VAC_X + HANDLE_C[0], VAC_Y, TOP + BLOCK_H + 0.5 * MM + HANDLE_C[2]
    squeeze = C  # fully closed: the 34 mm handle stops the fingers, which then press with the gripper's full force
    lift, push = LIFT, GAP_TO_TOOL + BORE_D + 1 * MM
    right = [
        W(t_left - 1.0, (0.20, -0.30, TOP + 0.15), O, "wait for the left arm", 0.0),
        W(3.0, (hx - 0.06, hy - 0.11, hz + 0.010), O, "beside and behind the machine", tiltkit.Tilt(level, joint=True)),
        W(1.5, (hx - 0.06, hy, hz), O, "behind the handle", grip),
        W(1.5, (hx, hy, hz), O, "round the handle", grip),
        W(1.0, (hx, hy, hz), squeeze, "grip", grip),
        W(1.0, (hx, hy, hz + lift), squeeze, "lift off the block", carry),
        W(2.5, (hx + push, hy, hz + lift), squeeze, "push onto the collar until it clicks", carry),
        W(0.8, (hx + push, hy, hz + lift), squeeze, "hold", carry_tool),
        W(0.8, (hx + push - 4 * MM, hy, hz + lift), squeeze, "back off the stop", carry_tool),
        W(1.5, (hx + push - 4 * MM, hy, hz + lift + 0.040), squeeze, "lift the tool out of its cradle", carry_tool),
    ]
    # vacuuming: the handle (TCP) such that the tip is at (x, GAP_Y, just above the floor), pitched nose-down
    # Aim correction, measured in the demo's own runs: carrying the machine tilted, the soft wrist joints let the
    # hand turn AIM_YAW about the vertical and drift AIM_DY sideways beyond what the gravity model above predicts,
    # so the plan aims that much the other way (as an operator would by eye).
    tip_from_grip = _rot_y(PITCH) @ (np.array([TOOL_L - BORE_D, 0.0, AXIS_Z]) - np.array(HANDLE_C))
    tip_from_grip += np.array([0.0, AIM_DY, 0.0])  # the hand is commanded AIM_DY to the right of where it should end
    Rv = _rot_z(-AIM_YAW) @ _rot_y(PITCH)  # and turned AIM_YAW clockwise
    tilt = tiltkit.Tilt(Rv @ level, load=with_tool)

    def at_tip(x, dz=0.0, lane=0.0):
        tip = np.array([x, GAP_Y + lane, TOP + BLADE_W / 2 * np.cos(PITCH) + TIP_UP + dz])
        return tuple(float(v) for v in tip - tip_from_grip)

    far, near, mid = GAP_X1 - 14 * MM, GAP_X0 + 14 * MM, (GAP_X0 + GAP_X1) / 2
    # Down into the middle of the gap (the most forgiving place to aim), then along each board in turn with the
    # blade pushed sideways against it (WALL past touching it, which the arm's give turns into a light press), so
    # each pass covers the strip along that board.
    wall = GAP_W / 2 - BLADE_T / 2 + WALL_PRESS
    right += [
        W(6.0, at_tip(mid, 0.11), squeeze, "over the middle of the gap, tilted",
          tiltkit.Tilt(Rv @ level, joint=True, load=with_tool)),
        W(1.5, at_tip(mid, 0.11), squeeze, "settle above the boards", tilt),
        W(4.0, at_tip(mid, 0.0), squeeze, "tip down to the floor", tilt),
        W(1.5, at_tip(mid, 0.0, -wall), squeeze, "against the right-hand board", tilt),
        W(5.0, at_tip(near, 0.0, -wall), squeeze, "back along it to the near end", tilt),
        W(8.0, at_tip(far, 0.0, -wall), squeeze, "forward along it to the far end (the opening leading)", tilt),
        W(2.0, at_tip(far, 0.0, wall), squeeze, "across to the left-hand board", tilt),
        W(8.0, at_tip(near, 0.0, wall), squeeze, "back along it to the near end", tilt),
        W(8.0, at_tip(far, 0.0, wall), squeeze, "forward along it to the far end", tilt),
        W(2.0, at_tip(far, 0.0, -wall), squeeze, "across to the right-hand board again", tilt),
        W(6.0, at_tip(near, 0.0, -wall), squeeze, "back along it to the near end", tilt),
        W(8.0, at_tip(far, 0.0, -wall), squeeze, "forward along it to the far end", tilt),
        W(5.0, at_tip(mid, 0.0), squeeze, "back to the middle", tilt),
        W(3.0, at_tip(mid, 0.11), squeeze, "out of the gap", tilt),
        W(2.0, at_tip(mid, 0.11), squeeze, "hold", tilt),
    ]
    t = sum(w.duration for w in right)
    left.append(W(max(t - t_left, 0.5), park, C, "wait", 0.0))
    return {"left": left, "right": right}


def self_test():
    """The scoring rules on hand-built states."""
    from partkit import PartState

    def state(pos, R=np.eye(3), sliders=None):
        w = np.sqrt(max(1.0 + np.trace(R), 1e-12)) / 2
        q = np.array([(R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w])
        p = np.asarray(pos, float)
        return PartState(pos=p, quat_xyzw=q, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p, high=p,
                         sliders=sliders or {"catch": 0.0, "button": 0.0})

    vac = state((0.3, -0.2, TOP + 0.3), _rot_y(np.radians(40)))
    home = vac.to_world((-BORE_D, 0.0, AXIS_Z))
    on = state(home, vac.rotation())
    loose = state(vac.to_world((-BORE_D + 8 * MM, 0.0, AXIS_Z)), vac.rotation())
    skew = state(home, vac.rotation() @ _rot_y(np.radians(6)))
    assert clicked(vac, on) and not clicked(vac, loose) and not clicked(vac, skew)
    assert not clicked(state(vac.pos, vac.rotation(), {"catch": 1.0 * MM, "button": 0.0}), on)

    p0, r0 = soilkit.lattice(GRIT)
    n = len(p0)

    def grit(frac, order=None):
        """Grains moved to the bin in ``order`` (default: as listed) up to ``frac`` of the grit's mass."""
        p = p0.copy()
        idx = np.arange(n) if order is None else np.asarray(order)
        share = np.cumsum(r0[idx] ** 3) / (r0 ** 3).sum()
        p[idx[:int(np.searchsorted(share, frac + 1e-9))]] = BIN_AT
        return p, r0

    def by_count(frac, order):
        """The first ``frac`` of the grains in ``order`` moved to the bin, whatever they weigh."""
        p = p0.copy()
        p[np.asarray(order)[:int(round(frac * n))]] = BIN_AT
        return p, r0

    smallest = np.argsort(r0)
    small_mass = (r0[smallest[:int(round(0.86 * n))]] ** 3).sum() / (r0 ** 3).sum()
    assert small_mass < 0.85, small_mass  # the smallest 86 % of the grains weigh under 85 % of the grit

    cases = {
        "nothing removed": (on, grit(0.0), 0.0),
        "20 %": (on, grit(0.20), 0.0),
        "30 %": (on, grit(0.30), 0.25),
        "60 %": (on, grit(0.60), 0.5),
        "80 %": (on, grit(0.80), 0.75),
        "84 %": (on, grit(0.84), 0.75),
        "86 %": (on, grit(0.86), 1.0),
        "95 %": (on, grit(0.95), 1.0),
        "95 % but the tool not home": (loose, grit(0.95), 0.0),
        "86 % of the grains by count, the smallest ones: scored by mass": (on, by_count(0.86, smallest),
                                                                           0.75 if small_mass >= 0.75 else 0.5),
        "all pushed out of the gap, none vacuumed": (on, (p0 + np.array([-0.30, 0.0, 0.0]), r0), 0.0),
    }
    for name, (tool, g, want) in cases.items():
        got = score({"vacuum": vac, "crevice": tool}, g)
        assert abs(got - want) < 1e-9, (name, got, want)

    # suction: a grain just ahead of the tool's opening is taken, one 10 mm away or beside it is not
    tip = on.to_world((TOOL_L, 0.0, 0.0))
    ahead = on.rotation()[:, 0]
    pts = np.array([tip + ahead * 2 * MM, tip + ahead * 10 * MM, tip + on.rotation()[:, 1] * 15 * MM])
    assert list(captured(pts, np.full(3, 1.2 * MM), vac, on, True)) == [True, False, False]
    # the tool fits: thin side through the gap, collar through the bore, rib under the tooth's rest
    assert BLADE_T < GAP_W and COLLAR_R < BORE_R and RIB_TOP > TOOTH_REST_R > COLLAR_R
    assert REACH * np.sin(PITCH) > GAP_D, "the blade must reach the floor tilted"
    return len(cases) + 6


if __name__ == "__main__":
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    p, r = soilkit.start_layout(GRIT)
    print(f"grit: {len(p)} grains, {np.sum(GRIT.density * 4 / 3 * np.pi * r ** 3) * 1000:.1f} g")
    task = load(__file__)
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / 20)[0] for k in range(round(plan.duration * 20) + 5)])
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
