"""On the station table stands a vented 50 ml vial of red-dyed water (a training liquid) in a weighted holder, its
rubber stopper up, and beside it a 20 ml syringe with a 38 mm blunt fill needle stands needle down in a syringe rack.
The barrel is clear: liquid drawn into it shows red below the plunger's dark rubber tip, against black marks every
2 ml and a red mark at 10 ml.

Task: draw 10 ml of the liquid into the syringe without drawing air, then stand the syringe back in its rack, let go
of it and take both arms back to where they started. The needle holds in the stopper only lightly and the plunger is
stiffer to pull than that, so the syringe has to be held in the stopper while the plunger is drawn. The vial's
liquid level is tracked: it falls 0.84 mm per ml drawn, and only plunger travel with the needle's tip below the
current level draws liquid.

Run it:  inspect-robots run --embodiment yam_arena -E task=draw_10_ml_from_a_vial_into_a_syringe.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, rotation,  # noqa: E402
                     set_slide_target, set_slider_drive, set_thread_on, slider, thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the vial: in a weighted holder, stopper up -------------------------------------------------------------------------
VIAL_XY = (0.36, 0.0)
ZS = TOP + 0.066  # the stopper's top
OPENING = 0.008  # the stopper shows through the crimp cap's opening, this far from the axis
D_MAX = 0.035  # the needle can go no deeper (its hub on the cap)
VIAL_R, GLASS_T = 0.021, 0.0015  # the 50 ml vial: outer radius, wall and floor thickness
FLOOR_Z = TOP + 0.008 + GLASS_T  # the inside of the vial's floor
AREA = math.pi * (VIAL_R - GLASS_T) ** 2  # inside cross-section, m^2 (11.9 cm^2)
V0 = 50.0  # ml of liquid in the vial at the start

# -- the syringe: 20 ml, standing needle down in its rack -------------------------------------------------------------
RACK_XY = (0.27, -0.11)
RACK_FLOOR = TOP + 0.075  # the rack's cup floor, where the barrel's end stands
CUP, CUP_H = 0.025, 0.014  # the rack's cup: inside width, height
BAR_R, BAR_L, BOT_WALL = 0.011, 0.085, 0.003  # barrel: outer radius, length, end wall
LIQ_R, PIST_R = 0.0097, 0.0099  # the medicine inside, the plunger's rubber tip and stem
ML = 0.003183  # plunger travel per ml (10 mm inner radius)
HUB_R, HUB_L, NEEDLE_L = 0.005, 0.010, 0.038  # 18 G x 1.5 in blunt fill needle
TIP_Z = -(HUB_L + NEEDLE_L)  # the needle's tip, in the syringe's frame (origin: the barrel's lower end)
RUB_L, STEM_TOP, ROD, ROD_L = 0.006, 0.082, 0.012, 0.045  # plunger: rubber tip, stem inside, square rod outside
THUMB_R = 0.016
SYR_MASS, PLUNGER_MASS = 0.02, 0.01

# -- how things move ----------------------------------------------------------------------------------------------------
K_P, SLIP_P, D_P = 800.0, 0.001, 2.0  # plunger: N/m, m, N s/m (about 0.8 N to start it moving)
K_N, SLIP_N, D_N = 250.0, 0.001, 2.0  # needle in the stopper (about 0.25 N)
CAPTURE_TILT = 15.0  # degrees: the needle catches in the stopper if the syringe is no more tilted than this

# -- scoring ------------------------------------------------------------------------------------------------------------
DOSE, DOSE_TOL, AIR_MAX = 10.0, 1.0, 1.0  # ml
IN_RACK_XY, IN_RACK_Z, IN_RACK_TILT = 0.008, 0.012, 15.0
LET_GO = 0.04
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0

WHITE, CLEAR, MARK, RED, DARK, STEEL, AMBER, ALU, GREY, RUBBER = (0.95, 0.95, 0.95), (0.86, 0.90, 0.93), \
    (0.10, 0.10, 0.10), (0.80, 0.08, 0.10), (0.15, 0.15, 0.16), (0.75, 0.76, 0.78), (0.62, 0.36, 0.10), \
    (0.80, 0.81, 0.83), (0.35, 0.36, 0.38), (0.30, 0.30, 0.32)
MED = (0.92, 0.04, 0.04)


def _vial() -> Body:
    """The vial and its holder; origin on the table under the vial's axis."""
    return Body.build("vial", pos=(VIAL_XY[0], VIAL_XY[1], TOP), mass=0.3, kinematic=True, color=AMBER, shapes=[
        BoxShape((0.0, 0.0, 0.004), (0.07, 0.07, 0.008), GREY),  # the weighted holder
        CylShape((0.0, 0.0, 0.013), 0.024, 0.010, GREY),  # its collar
        CylShape((0.0, 0.0, 0.034), VIAL_R, 0.052, AMBER),  # the glass
        CylShape((0.0, 0.0, 0.032), VIAL_R + 0.0003, 0.024, WHITE, collide=False),  # the label
        CylShape((0.0, 0.0, 0.0315), VIAL_R + 0.0005, 0.004, RED, collide=False),  # its red band
        CylShape((0.0, 0.0, 0.0625), 0.009, 0.005, AMBER)])  # the neck


def _vial_top() -> Body:
    """The crimp cap, with the rubber stopper showing through its opening; origin at the stopper's top."""
    shapes = [CylShape((0.0, 0.0, -0.0028), 0.0125, 0.005, ALU, collide=False),
              CylShape((0.0, 0.0, -0.0003), OPENING + 0.0004, 0.0006, RUBBER, collide=False),
              CylShape((0.0, 0.0105, 0.0012), 0.0022, 0.003, WHITE, collide=False)]  # the cap's filtered air vent
    w = (0.028 - 2 * OPENING) / 2  # the cap's rim, round the opening (collision only)
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((s * (OPENING + w / 2), 0.0, -0.0028), (w, 0.028, 0.005), None))
        shapes.append(BoxShape((0.0, s * (OPENING + w / 2), -0.0028), (2 * OPENING, w, 0.005), None))
    return Body.build("vial_top", pos=(VIAL_XY[0], VIAL_XY[1], ZS), mass=0.01, kinematic=True, color=ALU,
                      shapes=shapes)


def _rack() -> Body:
    """The syringe rack: a block with a bore for the needle and a cup, with a lead-in round its mouth, for the barrel's
    end."""
    h = RACK_FLOOR - TOP
    shapes = [BoxShape((0.0, 0.0, 0.003), (0.07, 0.07, 0.006), GREY)]
    bore, wall = 0.014, 0.012
    zc, hz = (0.006 + h) / 2, h - 0.006
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((0.0, s * (bore / 2 + wall / 2), zc), (bore + 2 * wall, wall, hz), WHITE))
        shapes.append(BoxShape((s * (bore / 2 + wall / 2), 0.0, zc), (wall, bore, hz), WHITE))
    cw, lead = 0.004, 0.004
    c45 = math.cos(math.pi / 8)
    s45 = math.sin(math.pi / 8)
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((0.0, s * (CUP / 2 + cw / 2), h + CUP_H / 2), (CUP + 2 * cw, cw, CUP_H), WHITE))
        shapes.append(BoxShape((s * (CUP / 2 + cw / 2), 0.0, h + CUP_H / 2), (cw, CUP, CUP_H), WHITE))
        # the lead-in: plates leaning out 45 degrees from the cup's rim
        off = CUP / 2 + lead / 2 * math.sqrt(0.5)
        zt = h + CUP_H + lead / 2 * math.sqrt(0.5)
        shapes.append(BoxShape((0.0, s * off, zt), (CUP + 2 * lead, lead, 0.002), WHITE,
                               quat_xyzw=(s * s45, 0.0, 0.0, c45)))
        shapes.append(BoxShape((s * off, 0.0, zt), (lead, CUP + 2 * lead, 0.002), WHITE,
                               quat_xyzw=(0.0, -s * s45, 0.0, c45)))
    return Body.build("rack", pos=(RACK_XY[0], RACK_XY[1], TOP), mass=0.5, kinematic=True, color=WHITE,
                      shapes=shapes)


def _syringe() -> Body:
    """The barrel with its hub and needle; origin at the barrel's lower end, on its axis. The barrel is clear: its
    collision cylinder is not drawn, so what is inside shows (the medicine drawn in, the plunger)."""
    shapes = [CylShape((0.0, 0.0, BAR_L / 2), BAR_R, BAR_L, None),  # the barrel (clear)
              CylShape((0.0, 0.0, BOT_WALL / 2), BAR_R + 0.0002, BOT_WALL, CLEAR, collide=False),
              CylShape((0.0, 0.0, BAR_L - 0.001), BAR_R + 0.0002, 0.002, CLEAR, collide=False),
              CylShape((0.0, 0.0, BOT_WALL + 10 * ML), LIQ_R, 20 * ML, MED, collide=False),  # medicine (shows below
              BoxShape((0.0, 0.0, BAR_L + 0.0005), (0.018, 0.044, 0.003), CLEAR),  # the plunger) / finger flange
              CylShape((0.0, 0.0, -HUB_L / 2), HUB_R, HUB_L, CLEAR),
              CylShape((0.0, 0.0, -HUB_L - NEEDLE_L / 2), 0.0006, NEEDLE_L, STEEL, collide=False),
              CylShape((0.0, 0.0, -HUB_L - NEEDLE_L / 2), 0.001, NEEDLE_L, None)]
    for side in (-1.0, 1.0):  # two faint seams down the clear barrel
        shapes.append(BoxShape((0.0, side * (BAR_R - 0.0002), BAR_L / 2), (0.0012, 0.0006, BAR_L - 0.004), CLEAR,
                               collide=False))
    for k in range(2, 21, 2):  # the scale: a black mark every 2 ml, the 10 ml mark red and wider
        shapes.append(CylShape((0.0, 0.0, BOT_WALL + k * ML), BAR_R + 0.0003, 0.0014 if k == 10 else 0.0006,
                               RED if k == 10 else MARK, collide=False))
    return Body.build("syringe", pos=(RACK_XY[0], RACK_XY[1], RACK_FLOOR), mass=SYR_MASS, shapes=shapes,
                      friction=(1.0, 0.9), color=CLEAR, com=(0.0, 0.0, 0.035), ignore=("vial",), damping=(0.5, 0.5))


def _plunger() -> Body:
    """The plunger; origin at its rubber tip's lower face (on the barrel's end wall when empty)."""
    return Body.build("plunger", pos=(RACK_XY[0], RACK_XY[1], RACK_FLOOR + BOT_WALL), mass=PLUNGER_MASS,
                      friction=(1.0, 0.9), color=WHITE, com=(0.0, 0.0, 0.075), damping=(0.5, 0.5), shapes=[
                          CylShape((0.0, 0.0, RUB_L / 2), PIST_R, RUB_L, DARK, collide=False),
                          CylShape((0.0, 0.0, (RUB_L + STEM_TOP) / 2), PIST_R, STEM_TOP - RUB_L, WHITE,
                                   collide=False),
                          BoxShape((0.0, 0.0, STEM_TOP + ROD_L / 2), (ROD, ROD, ROD_L), WHITE),
                          CylShape((0.0, 0.0, STEM_TOP + ROD_L + 0.0015), THUMB_R, 0.003, WHITE)])


OBJECTS = [_vial(), _vial_top(), _rack(), _syringe(), _plunger()]
slider("plunger_slide", "syringe", "plunger", anchor=(RACK_XY[0], RACK_XY[1], RACK_FLOOR + BOT_WALL), axis="Z",
       limits=(0.0, 20.2 * ML), stiffness=K_P, damping=D_P, target=0.0)
thread("needle", "vial", "syringe", anchor0=(0.0, 0.0, ZS - TOP), axis0=(0.0, 0.0, -1.0), anchor1=(0.0, 0.0, TIP_Z),
       axis1=(0.0, 0.0, -1.0), slide_limits=(-0.05, D_MAX), turn_damping=0.002, slide_stiffness=K_N,
       slide_damping=D_N, slide_max_force=50.0, play=0.003, tilt_deg=5.0)


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"on": False, "hold_n": 0.0, "hold_p": 0.0, "rec": 0.0, "liquid": 0.0, "air": 0.0, "wet": False,
            "vial_ml": V0, "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("SYRINGE_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    s = as_numpy(base.scene["syringe"].data.root_link_pose_w)[0]
    p = as_numpy(base.scene["plunger"].data.root_link_pose_w)[0]
    s_pos, s_rot, p_pos = s[:3] - origin, rotation(s[3:7]), p[:3] - origin
    pulled = float((s_rot.T @ (p_pos - s_pos))[2]) - BOT_WALL  # the plunger's travel from empty
    if abs(pulled - _EP["hold_p"]) > SLIP_P:  # pulled or pushed past its hold: it slides, and stays where left
        _EP["hold_p"] = pulled
        set_slider_drive("plunger_slide", target=pulled)
    tip = s_pos + s_rot @ np.array([0.0, 0.0, TIP_Z])
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(s_rot[2, 2])))))
    off = math.hypot(tip[0] - VIAL_XY[0], tip[1] - VIAL_XY[1])
    depth = ZS - float(tip[2])
    if not _EP["on"]:
        if off <= OPENING and 0.0 <= depth <= 0.012 and tilt <= CAPTURE_TILT:  # through the stopper: caught
            set_slide_target("needle", depth)
            set_thread_on("needle", True)
            _EP["on"], _EP["hold_n"] = True, depth
    else:
        if depth < -0.002:  # pulled out of the stopper
            set_thread_on("needle", False)
            _EP["on"] = False
        elif abs(depth - _EP["hold_n"]) > SLIP_N:
            _EP["hold_n"] = depth
            set_slide_target("needle", depth)
    level = FLOOR_Z + _EP["vial_ml"] * 1e-6 / AREA  # the liquid's surface in the vial
    in_med = _EP["on"] and float(tip[2]) < level  # the needle's tip below the liquid's surface
    _EP["wet"] = _EP["wet"] or in_med
    total = max(pulled, 0.0) / ML  # ml drawn in: liquid if the tip was in it, else air
    if total - _EP["rec"] > 0.05:
        if in_med:
            _EP["liquid"] += total - _EP["rec"]
            _EP["vial_ml"] -= total - _EP["rec"]
        else:
            _EP["air"] += total - _EP["rec"]
        _EP["rec"] = total
    elif _EP["rec"] - total > 0.05:  # pushed back: the needle end is down, so the liquid goes out first
        out = _EP["rec"] - total
        take = min(_EP["liquid"], out)
        _EP["liquid"] -= take
        if _EP["on"]:
            _EP["vial_ml"] += take  # back into the vial
        _EP["air"] = max(0.0, _EP["air"] - (out - take))
        _EP["rec"] = total
    _EP["tcps"] = _tcps(base, origin)
    barrel_mid = s_pos + s_rot @ np.array([0.0, 0.0, BAR_L / 2])
    thumb = p_pos + s_rot @ np.array([0.0, 0.0, STEM_TOP + ROD_L / 2])
    clear = all(min(float(np.linalg.norm(t - barrel_mid)), float(np.linalg.norm(t - thumb))) >= LET_GO
                for t in _EP["tcps"])
    in_rack = (math.hypot(s_pos[0] - RACK_XY[0], s_pos[1] - RACK_XY[1]) <= IN_RACK_XY
               and -0.003 <= s_pos[2] - RACK_FLOOR <= IN_RACK_Z and tilt <= IN_RACK_TILT)
    _EP["s"] = {"in_rack": in_rack and not _EP["on"], "clear": clear}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = _EP["tcps"]
            print(f"[syringe] t={_DEBUG['steps'] / 20:5.1f}s depth={depth * 1000:6.1f}mm "
                  f"under-surface={(level - float(tip[2])) * 1000:5.1f}mm vial={_EP['vial_ml']:4.1f}ml off={off * 1000:4.1f}mm "
                  f"tip-axis={np.round((tip[:2] - np.array(VIAL_XY)) * 1000, 1).tolist()} "
                  f"tip-rack={np.round((tip - np.array([RACK_XY[0], RACK_XY[1], RACK_FLOOR])) * 1000, 1).tolist()} "
                  f"end-rack={np.round((s_pos - np.array([RACK_XY[0], RACK_XY[1], RACK_FLOOR])) * 1000, 1).tolist()} "
                  f"axis={np.round(s_rot[:, 2], 3).tolist()} "
                  f"tilt={tilt:4.1f} on={_EP['on']} pulled={pulled * 1000:5.1f}mm ({total:4.2f} ml) "
                  f"liquid={_EP['liquid']:4.2f} air={_EP['air']:4.2f} in_rack={in_rack} clear={clear} "
                  f"barrel={np.round(s_pos * 1000).astype(int).tolist()} "
                  f"leftTCP-barrel={np.round((tl - s_pos) * 1000).astype(int).tolist()} "
                  f"rightTCP-thumb={np.round((tr - thumb) * 1000).astype(int).tolist()} score={_points(_status())} "
                  f"v={np.linalg.norm(as_numpy(base.scene['syringe'].data.root_lin_vel_w)[0]):.3f}/"
                  f"{np.linalg.norm(as_numpy(base.scene['syringe'].data.root_ang_vel_w)[0]):.2f} "
                  f"vp={np.linalg.norm(as_numpy(base.scene['plunger'].data.root_lin_vel_w)[0]):.3f}/"
                  f"{np.linalg.norm(as_numpy(base.scene['plunger'].data.root_ang_vel_w)[0]):.2f}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    st = _EP["s"]
    return {"dose": abs(_EP["liquid"] - DOSE) <= DOSE_TOL and _EP["air"] <= AIR_MAX, "wet": _EP["wet"],
            "in_rack": bool(st.get("in_rack")), "clear": bool(st.get("clear")), "home": _arms_home()}


def _points(s) -> float:
    if s["dose"] and s["in_rack"] and s["clear"] and s["home"]:
        return 1.0
    if s["dose"]:
        return 0.5
    return 0.25 if s["wet"] else 0.0


def score(objects) -> float:
    """1.0 with 10 ml of liquid in the syringe (9 to 11 ml, and no more than 1 ml of air), the needle out of the vial,
    the syringe standing in its rack and let go of (both TCPs at least 4 cm from its barrel and plunger) and both
    arms back where they started (each TCP within 5 cm of its start); 0.5 with that dose in the syringe wherever it
    is; 0.25 if the needle's tip was below the vial's liquid surface at some point (the surface starts 14.6 mm below
    the stopper's top and falls 0.84 mm per ml drawn); else 0.0. All but the last are judged at the end."""
    return _points(_status())


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
    """Full score with the syringe and its plunger still."""
    return (score(objects) >= 1.0 and objects["syringe"].resting() and objects["plunger"].resting())


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes in level from the left, fingers open, round the syringe's barrel near its lower end, takes it
# and lifts it out of the rack, carries it over the vial and pushes the needle straight down through the stopper, 15
# mm deep, and holds it there. The right hand comes in level from the right round the plunger's rod, takes it and pulls
# it up to the 10 ml mark, lets go and draws back. The left pulls the needle out, carries the syringe back to its rack,
# stands it in, lets go and draws back, and both arms go back to where they started.

def _cols(x, z) -> np.ndarray:
    x, z = np.asarray(x, float), np.asarray(z, float)
    return np.column_stack([x, np.cross(z, x), z])


L_SIDE = _cols((-1.0, 0.0, 0.0), (0.0, -1.0, 0.0))  # level, pointing to the right (-y), fingers closing along x
R_SIDE = _cols((1.0, 0.0, 0.0), (0.0, 1.0, 0.0))  # level, pointing to the left (+y), fingers closing along x
GRIP_UP = 0.032  # the left TCP this far above the barrel's lower end
DEPTH = 0.033  # how deep the needle goes into the vial (its tip 10 mm under the surface once 10 ml is drawn)
PULL = DOSE * ML + 0.0095  # how far the right pulls the plunger (a little extra: the hand lags)
LEAD_S, LEAD_STEPS = 3.0, 15
# Carried back from the vial, the syringe hangs tilted in the left hand (measured: its axis at (0.075, -0.122, 0.99),
# 8.3 degrees, the left TCP 3, -3, 37 mm from the barrel's end): on the way to the rack the grip is turned back by 60%
# of that (all of it is out of the arm's reach there), and aimed so the long needle's tip comes down over the rack's
# bore; the cup's lead-in takes the barrel's end the last few millimetres.
BACK_TILT = np.array([0.075, -0.122, 0.990])
BACK_D = np.array([0.003, -0.003, 0.037])
BACK_FRAC = 0.6
LEFT_OFF = np.array([-0.003, -0.001, -0.005])  # where the lifted barrel sits off its planned place in the left hand
# (measured: it slips a little down the fingers as it leaves the rack); aimed against


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


def _joint_path(arm: str, q0, q1, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, rot), label, finger)


def _go_home(arm: str, pos, rot, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, _good_ik(arm, pos, rot), np.array(READY, float), "back to where it started", finger)


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _upright(a, frac: float) -> np.ndarray:
    """The rotation turning the unit vector along a toward +z by frac of the angle between them."""
    a = np.asarray(a, float) / np.linalg.norm(a)
    axis = np.cross(a, [0.0, 0.0, 1.0])
    axis /= np.linalg.norm(axis)
    t = frac * math.acos(float(a[2]))
    k = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]])
    return np.eye(3) + math.sin(t) * k + (1.0 - math.cos(t)) * k @ k


def _left(release_at: float, total: float) -> tuple[list[Waypoint], float]:
    take = np.array([RACK_XY[0], RACK_XY[1], RACK_FLOOR + GRIP_UP])
    rack = take - LEFT_OFF
    vial = np.array([VIAL_XY[0], VIAL_XY[1], ZS - DEPTH - TIP_Z + GRIP_UP]) - LEFT_OFF
    high = 0.075  # carried this high above the rack's floor: the needle clears everything
    turn = _upright(BACK_TILT, BACK_FRAC)
    l_back = turn @ L_SIDE
    after = turn @ (BACK_TILT / np.linalg.norm(BACK_TILT))  # the syringe's axis, turned back part way
    tip_drop = HUB_L + NEEDLE_L  # the needle's tip, this far down the axis from the barrel's end
    end = np.array([RACK_XY[0] + tip_drop * after[0], RACK_XY[1] + tip_drop * after[1], RACK_FLOOR + 0.004])
    back = end + turn @ BACK_D  # the TCP: the needle's tip over the bore, the barrel's end in the cup
    side = np.array([0.0, 0.07, 0.0])
    out = _lead_in("left", take + side, L_SIDE, "beside the syringe in its rack", FINGER_OPEN)
    out += [_w(1.0, take, FINGER_OPEN, "in round the barrel", L_SIDE),
            _w(0.5, take, FINGER_CLOSED, "take it", L_SIDE),
            _w(1.2, take + [0.0, 0.0, high], FINGER_CLOSED, "lift it out of the rack", L_SIDE),
            _w(2.0, [vial[0], vial[1], rack[2] + high], FINGER_CLOSED, "over the vial", L_SIDE),
            _w(1.0, vial + [0.0, 0.0, DEPTH + 0.02], FINGER_CLOSED, "the needle just over the stopper", L_SIDE),
            _w(1.5, vial, FINGER_CLOSED, "push the needle through the stopper, its tip well under the liquid", L_SIDE)]
    in_at = sum(w.duration for w in out)
    out += [_w(max(release_at - in_at, 0.3), vial, FINGER_CLOSED, "hold the syringe still in the vial", L_SIDE),
            _w(1.2, [vial[0], vial[1], rack[2] + high], FINGER_CLOSED, "pull the needle out", L_SIDE),
            _w(2.0, back + [0.0, 0.0, high - 0.004], FINGER_CLOSED, "over the rack, the syringe turned upright", l_back),
            _w(1.0, back + [0.0, 0.0, 0.030], FINGER_CLOSED, "the needle down into the rack's bore", l_back),
            _w(1.5, back, FINGER_CLOSED, "stand it back in the rack", l_back),
            _w(0.5, back, FINGER_OPEN, "let go", l_back),
            _w(0.6, back + [0.0, 0.03, 0.02], FINGER_OPEN, "draw back", l_back),
            _w(0.8, back + side + [0.0, 0.0, 0.02], FINGER_OPEN, "clear of it", l_back)]
    out += _go_home("left", back + side + [0.0, 0.0, 0.02], l_back, FINGER_OPEN)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out[-1].pos, FINGER_OPEN,
                  "hold still: the trial ends once the full score has held 1 s with both arms home", out[-1].yaw))
    return out, in_at


def _right(start: float) -> tuple[list[Waypoint], float]:
    rod = np.array([VIAL_XY[0], VIAL_XY[1], ZS - DEPTH - TIP_Z + BOT_WALL + STEM_TOP + ROD_L / 2])
    side = np.array([0.0, -0.07, 0.0])
    out = _lead_in("right", rod + side, R_SIDE, "beside the plunger", FINGER_OPEN)
    out += [_w(max(start - LEAD_S, 0.3) + 0.5, rod + side, FINGER_OPEN, "wait for the needle to be in", R_SIDE),
            _w(1.0, rod, FINGER_OPEN, "in round the plunger's rod", R_SIDE),
            _w(0.5, rod, FINGER_CLOSED, "take it", R_SIDE),
            _w(3.0, rod + [0.0, 0.0, PULL], FINGER_CLOSED, "pull the plunger up to the 10 ml mark", R_SIDE),
            _w(0.5, rod + [0.0, 0.0, PULL], FINGER_CLOSED, "hold", R_SIDE),
            _w(0.5, rod + [0.0, 0.0, PULL], FINGER_OPEN, "let go", R_SIDE),
            _w(1.0, rod + side + [0.0, 0.0, PULL], FINGER_OPEN, "draw back", R_SIDE)]
    done = sum(w.duration for w in out)
    out += _go_home("right", rod + side + [0.0, 0.0, PULL], R_SIDE, FINGER_OPEN)
    return out, done


def demo():
    _, in_at = _left(0.0, 0.0)
    right, done = _right(in_at)
    left, _ = _left(done, 0.0)
    total = max(sum(w.duration for w in left), sum(w.duration for w in right)) + 3.0
    left, _ = _left(done, total)
    t = sum(w.duration for w in right)
    right.append(_w(total - t, right[-1].pos, FINGER_OPEN, "wait at the start", right[-1].yaw))
    return {"left": left, "right": right}
