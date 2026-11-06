"""On the station table: a 24 cm vacuum desiccator holding a dish of a moisture-sensitive powder, under vacuum; its
vacuum line running past a dial gauge beside the stopcock to a pump controller box with a pump switch, in front of
the right arm; and a
sample vial standing on the bench to the left.

The desiccator: a white polypropylene bowl base, 24 cm across its flange and 7 cm tall, with a perforated white plate
inside and a dark ring marked on the plate; and a clear dome, 6.5 cm tall, sitting on the flange (an O-ring seals it),
with a solid clear knob on top to lift it by. On the base's right side, on a 2.5 cm port, a red stopcock: its 4 cm
lever pointing at the robots is shut; turned a quarter turn anticlockwise (seen from above, pointing right) it is open
to the vacuum line. The line runs past a dial gauge on a tee just beyond the stopcock (it reads the line: the box's
pressure while the stopcock is open) to the controller box, which carries a rocker switch for the pump: red end down is
on, black end down is off. The pump
starts off, and its line is then open to the room, so opening the stopcock lets air in, faster the further it is
turned. With the pump on and the stopcock open, the box is pumped down.

Under vacuum the air holds the dome down with far more force than a robot can pull. Venting faster than 12 kPa a second
blows the powder in the dish about: at full vacuum that happens once the stopcock is past about 40 degrees.

The task: vent the desiccator gently through the stopcock; lift the dome off by its knob and hold it up clear of the
base (it is not set down) while the other hand stands the vial on the ring; lower the dome back onto the flange; switch
the pump on and, once the gauge reads -90 kPa or lower, swing the stopcock shut and switch the pump off; finally pull
up gently on the knob to check the dome is held down.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "<see task.json>" --scorer task_score ...
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import labkit  # noqa: E402
from labkit import Joint, lathe_mesh, on_reset, on_step, part  # noqa: E402
import yam_arena.demo as _demo  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

PP = (0.95, 0.95, 0.93)  # white polypropylene
CLEAR = (0.82, 0.90, 0.96)  # clear polycarbonate, drawn as glass
labkit.GLASS_COLORS.update({CLEAR})
RED = (0.82, 0.12, 0.10)
BLACK = (0.06, 0.06, 0.07)
DARK = (0.18, 0.19, 0.21)
STEEL = (0.75, 0.76, 0.78)
HOSE = (0.85, 0.75, 0.55)  # amber rubber vacuum hose
DISH_C = (0.85, 0.92, 0.95)
POWDER = (0.93, 0.80, 0.30)
VIAL_C = (0.88, 0.93, 0.95)
CAP_C = (0.12, 0.35, 0.70)
WHITE = (0.97, 0.97, 0.96)

# -- the base (loose). Frame: origin on the table under its axis. -----------------------------------------------------
# A 24 cm desiccator, kept low (7 cm base, 6.5 cm dome) so the arms reach its knob.
BX, BY = 0.25, -0.06
BASE_KG = 2.0  # with its rubber feet, it does not turn on the bench when the stopcock is turned
FLANGE_R, FLANGE_TOP = 0.120, 0.070
WALL_R = 0.104  # inside of the wall at the top
PLATE_Z = 0.030  # the perforated plate's top
PLATE_R = 0.094
_sr, _sz = FLANGE_R / 0.085, FLANGE_TOP / 0.075
BASE_PROFILE = ([(r * _sr, z * _sz) for r, z in [(0.0, 0.0), (0.056, 0.0), (0.060, 0.004), (0.064, 0.020),
                                                  (0.070, 0.045), (0.074, 0.062), (0.076, 0.069)]]
                + [(FLANGE_R, 0.069 * _sz), (FLANGE_R, FLANGE_TOP), (WALL_R, FLANGE_TOP)]
                + [(r * _sr, z * _sz) for r, z in [(0.070, 0.050), (0.062, 0.020), (0.056, 0.006), (0.0, 0.006)]])
N_WALL = 40
MARK_XY = (0.0, 0.045)  # the vial's spot on the plate, a ring towards the robots' left
MARK_R = 0.016
# The stopcock stands out from the base's right side on a 2.5 cm port, so a hand on its lever stays clear of the dome.
STOP_ANGLE = -math.pi / 2
STOP_R = FLANGE_R + 0.025
STOP_Z = 0.045
STOP_XY = (STOP_R * math.cos(STOP_ANGLE), STOP_R * math.sin(STOP_ANGLE))
LEVER_L = 0.040
STOP_OPEN = math.pi / 2  # the lever's quarter turn anticlockwise (positive about z): open
_wall_boxes = []
for _i in range(N_WALL):
    _a = 2 * math.pi * (_i + 0.5) / N_WALL
    _r = WALL_R + 0.002
    _wall_boxes.append(((_r * math.cos(_a), _r * math.sin(_a), (0.006 + FLANGE_TOP) / 2),
                        (0.004, 2 * math.pi * _r / N_WALL + 0.002, FLANGE_TOP - 0.006), PP, _a))
    _rf = (WALL_R + FLANGE_R) / 2
    _wall_boxes.append(((_rf * math.cos(_a), _rf * math.sin(_a), FLANGE_TOP - 0.003),
                        (FLANGE_R - WALL_R, 2 * math.pi * _rf / N_WALL + 0.002, 0.006), PP, _a))
STOPCOCK = Joint(
    "stopcock", kind="hinge", axis="Z", anchor=(*STOP_XY, STOP_Z + 0.010), lower=-0.05, upper=STOP_OPEN + 0.05,
    initial=0.0, target=0.0, stiffness=0.0, damping=0.05, friction=0.0, mass=0.01,  # PhysX joint friction scales with
    #   the load on the hinge, so a push on the post would lock it: plain damping instead
    boxes=[((STOP_XY[0] - LEVER_L / 2, STOP_XY[1], STOP_Z + 0.014), (LEVER_L, 0.008, 0.004), RED)],
    cylinders=[((STOP_XY[0], STOP_XY[1], STOP_Z + 0.013), 0.007, 0.006, RED),
               ((STOP_XY[0] - LEVER_L + 0.004, STOP_XY[1], STOP_Z + 0.024), 0.0045, 0.020, RED)],  # lever post
)
_port_len = STOP_R - (WALL_R + 0.004) + 0.004
_mark = [((MARK_XY[0] + MARK_R * math.cos(a), MARK_XY[1] + MARK_R * math.sin(a), PLATE_Z + 0.0002),
          (0.0025, 0.0025, 0.0004), DARK, a) for a in np.linspace(0, 2 * math.pi, 16, endpoint=False)]
BASE = part(
    "base", pos=(BX, BY, TOP), mass=BASE_KG, friction=0.8,
    meshes=[lathe_mesh(BASE_PROFILE, PP, segments=96),
            lathe_mesh([(0.0, PLATE_Z), (PLATE_R, PLATE_Z), (PLATE_R, PLATE_Z - 0.003), (0.0, PLATE_Z - 0.003)],
                       WHITE, segments=96)],
    boxes=[*_wall_boxes, *_mark,
           ((STOP_XY[0], STOP_XY[1] + _port_len / 2, STOP_Z), (0.010, _port_len, 0.010), PP),  # the port
           ((STOP_XY[0], STOP_XY[1] - 0.015, STOP_Z), (0.006, 0.020, 0.006), PP)],  # hose barb
    cylinders=[((0.0, 0.0, 0.003), 0.079, 0.006, PP),  # floor
               ((0.0, 0.0, PLATE_Z - 0.0015), PLATE_R, 0.003, WHITE),  # the plate
               ((0.0, 0.0, (0.006 + PLATE_Z - 0.003) / 2), 0.010, PLATE_Z - 0.009, WHITE),  # its centre leg
               *[((0.072 * math.cos(a), 0.072 * math.sin(a), PLATE_Z + 0.0002), 0.004, 0.0004, (0.80, 0.80, 0.80))
                 for a in np.linspace(0, 2 * math.pi, 14, endpoint=False)],  # the plate's holes, drawn
               ((STOP_XY[0], STOP_XY[1], STOP_Z), 0.008, 0.016, WHITE)],  # stopcock body
    joints=[STOPCOCK],
)

# -- the lid (loose): a clear dome sitting on the flange, a solid knob on top to lift it by -------------------------
LID_KG = 0.45  # polycarbonate
DOME_H = 0.065
KNOB_R, KNOB_H = 0.016, 0.022
DOME_TOP = FLANGE_TOP + DOME_H + KNOB_H  # the knob's top above the table
_dr, _dz = FLANGE_R / 0.085, DOME_H / 0.076
LID_PROFILE = ([(FLANGE_R, 0.0), (FLANGE_R, 0.008), (FLANGE_R - 0.006, 0.010)]
               + [(r * _dr, z * _dz) for r, z in [(0.078, 0.025), (0.072, 0.045), (0.060, 0.060), (0.042, 0.069),
                                                  (0.024, 0.074)]]
               + [(0.020, DOME_H), (0.0, DOME_H)])
_rim_boxes = []
for _i in range(N_WALL):
    _a = 2 * math.pi * (_i + 0.5) / N_WALL
    _r = (FLANGE_R - 0.007 + FLANGE_R) / 2
    _rim_boxes.append(((_r * math.cos(_a), _r * math.sin(_a), 0.004), (0.007, 2 * math.pi * _r / N_WALL + 0.002, 0.008),
                       CLEAR, _a))
LID = part(
    "lid", pos=(BX, BY, TOP + FLANGE_TOP), mass=LID_KG, friction=1.0,
    meshes=[lathe_mesh(LID_PROFILE, CLEAR, segments=96)],
    boxes=_rim_boxes + [((KNOB_R * math.cos(a), KNOB_R * math.sin(a), DOME_H + KNOB_H / 2), (0.003, 0.003, KNOB_H),
                         CLEAR, a) for a in np.linspace(0, 2 * math.pi, 12, endpoint=False)],  # the knob's grip ribs
    cylinders=[((0.0, 0.0, DOME_H - 0.0015), 0.022, 0.003, CLEAR),  # the knob's boss
               ((0.0, 0.0, DOME_H + KNOB_H / 2), KNOB_R, KNOB_H, CLEAR)],  # the knob
)

# -- the vacuum line (fixed): a hose from the stopcock's barb past a dial gauge on a short stand beside the stopcock, on to
# a controller box with the pump's switch --------------------------------------------------------------------------------
LINE_XY = (0.17, -0.36)  # the pump's controller box, in front of the right arm and clear of the held dome
BOX = (0.080, 0.050, 0.080)  # the controller box: the line's manifold and the pump switch
GAUGE_XY = (0.245, -0.275)  # the gauge, world: on a tee just past the stopcock's hose barb, where the right wrist camera
#   sees it while that hand works the lever (low, so the fingers on the lever pass over it)
GX, GY = GAUGE_XY[0] - LINE_XY[0], GAUGE_XY[1] - LINE_XY[1]  # in the line's frame
GAUGE_R = 0.025  # a 50 mm dial
GAUGE_TOP = 0.030
NEEDLE_PER_KPA = math.radians(270.0) / 100.0
NEEDLE = Joint(
    "needle", kind="hinge", axis="Z", anchor=(GX, GY, GAUGE_TOP + 0.001), lower=-4.8, upper=0.1, initial=0.0,
    target=0.0, stiffness=0.02, damping=0.0008, mass=0.001,
    boxes=[((GX, GY + 0.009, GAUGE_TOP + 0.0012), (0.0025, 0.020, 0.0008), RED)],
)
_ticks = []
for _k in range(11):  # every 10 kPa, from 0 (pointing +y, towards the desiccator) clockwise seen from above
    _a = -_k * 10 * NEEDLE_PER_KPA
    _ticks.append(((GX - 0.020 * math.sin(_a), GY + 0.020 * math.cos(_a), GAUGE_TOP + 0.0003),
                   (0.0025, 0.0025, 0.0004), BLACK if _k < 9 else RED))
# The pump switch: a rocker on the manifold's far end, pivot across it (about y). Red PUMP ON half away from the robots,
# black OFF half towards them. Off: the red end tilted up. Pressing an end down tips it there; it stays where pushed.
SWITCH_X = 0.022
ROCKER_PIVOT = (SWITCH_X, 0.0, BOX[2] + 0.004)
ROCKER = Joint(
    "pump", kind="hinge", axis="Y", anchor=ROCKER_PIVOT, lower=-0.25, upper=0.25, initial=-0.25,
    stiffness=0.0, damping=0.02, target=0.0,
    boxes=[((SWITCH_X + 0.008, 0.0, ROCKER_PIVOT[2]), (0.016, 0.018, 0.004), RED),
           ((SWITCH_X - 0.008, 0.0, ROCKER_PIVOT[2]), (0.016, 0.018, 0.004), BLACK)],
)
ON_ANGLE = 0.10  # the rocker counts as on past this (rad)
_barb = np.array((BX + STOP_XY[0], BY + STOP_XY[1] - 0.025)) - np.array(LINE_XY)  # the barb's end, box frame
_stand = np.array((GX, GY))
_inlet = np.array((0.0, BOX[1] / 2))


def _hose(p, q, z):
    d = q - p
    return (((p + q) / 2)[0], ((p + q) / 2)[1], z), (float(np.linalg.norm(d)), 0.010, 0.010), HOSE, math.atan2(d[1], d[0])


LINE = part(
    "vacuum_line", pos=(*LINE_XY, TOP), fixed=True, friction=0.6,
    boxes=[((0.0, 0.0, BOX[2] / 2), BOX, STEEL),  # manifold
           ((GX, GY, (GAUGE_TOP - 0.008) / 2), (0.012, 0.012, GAUGE_TOP - 0.008), STEEL),  # the gauge's stand
           *_ticks,
           _hose(_barb, _stand, 0.008),  # hose from the barb to the gauge's tee, under the dial
           _hose(_stand, _inlet, 0.008),  # on to the controller box
           ((0.0, -BOX[1] / 2 - 0.025, 0.008), (0.010, 0.050, 0.010), HOSE)],  # on to the pump
    cylinders=[((GX, GY, GAUGE_TOP - 0.004), GAUGE_R, 0.008, DARK),
               ((GX, GY, GAUGE_TOP + 0.0001), GAUGE_R - 0.002, 0.0002, WHITE)],
    joints=[NEEDLE, ROCKER],
)

# -- the dish of powder inside, and the vial ------------------------------------------------------------------------
DISH_R, DISH_H = 0.022, 0.008
DISH_XY = (BX + 0.025, BY - 0.045)
DISH = part("dish", pos=(*DISH_XY, TOP + PLATE_Z), mass=0.020, friction=0.5,
            cylinders=[((0.0, 0.0, 0.001), DISH_R, 0.002, DISH_C), ((0.0, 0.0, 0.0035), DISH_R - 0.003, 0.003, POWDER)],
            boxes=[((DISH_R * math.cos(a), DISH_R * math.sin(a), DISH_H / 2), (0.002, 0.011, DISH_H), DISH_C, a)
                   for a in np.linspace(0, 2 * math.pi, 12, endpoint=False)])
VIAL_R, VIAL_H, CAP_H = 0.011, 0.035, 0.010
VIAL_XY = (0.20, 0.20)
VIAL = part("vial", pos=(*VIAL_XY, TOP), mass=0.030, friction=0.9,
            cylinders=[((0.0, 0.0, VIAL_H / 2), VIAL_R, VIAL_H, VIAL_C),
                       ((0.0, 0.0, VIAL_H + CAP_H / 2), VIAL_R + 0.0005, CAP_H, CAP_C)])

OBJECTS = [BASE, LID, LINE, DISH, VIAL]


# -- the air -----------------------------------------------------------------------------------------------------------
# The pump starts off, and its line is then open to the room (its air-admittance valve). With the stopcock open, air
# flows in through the line while the pump is off (faster the further the stopcock is turned, a quarter turn fully
# open), and the box is pumped down while it is on. The gauge reads the line: the box's pressure while the stopcock is
# open, else the pump's (on) or the room's (off).
ATM = 101.3  # kPa
P0 = 8.0
VENT_K = 60.0  # kPa/s, stopcock fully open at full vacuum
PUMP_K = 0.6  # 1/s towards P_PUMP, stopcock fully open
P_PUMP = 3.0
LEAK = 0.002
GENTLE = 12.0
HOLD_MAX = 400.0  # N on a seated dome under vacuum (capped; the real load is about 3 kN)
SEAL_TILT, SEAL_Z, SEAL_XY = 1.5, 0.0015, 0.008  # the flange is 16 mm wide: 8 mm off still covers the O-ring
_trial = {"p": P0, "gauge": ATM, "max_rate": 0.0, "blown": False, "ids": None}


def lid_sealed(lid, base) -> bool:
    """Seated on the flange: flat within 1.5 degrees, its rim within 1.5 mm of the flange's top, and its centre within
    8 mm of the base's axis."""
    rim = base.to_world((0.0, 0.0, FLANGE_TOP))
    rel = base.to_local(lid.pos)
    return (lid.tilt_deg() <= SEAL_TILT and abs(lid.pos[2] - rim[2]) <= SEAL_Z and math.hypot(rel[0], rel[1]) <= SEAL_XY)


def open_fraction(q: float) -> float:
    return float(np.clip(q / STOP_OPEN, 0.0, 1.0)) ** 2


def pump_on(line) -> bool:
    return line.joints["pump"] > ON_ANGLE


@on_reset
def _reset(env):
    _trial.update(p=P0, gauge=ATM, max_rate=0.0, blown=False)


def _wrench(asset, force):
    import torch

    f = torch.tensor([[[*force]]], device=asset.device, dtype=torch.float32)
    t = torch.zeros_like(f)
    comp = getattr(asset, "permanent_wrench_composer", None)
    if comp is not None:
        comp.set_forces_and_torques_index(forces=f, torques=t, body_ids=[0], is_global=True)
    else:
        asset.set_external_force_and_torque(f, t, body_ids=[0], is_global=True)


@on_step
def _air(env):
    import torch

    base_env = env.unwrapped
    dt = float(getattr(base_env, "step_dt", 0.05))
    s = {n: labkit._part_state(env, labkit._SPECS[n]) for n in ("base", "lid", "vacuum_line")}
    sealed = lid_sealed(s["lid"], s["base"])
    f = open_fraction(s["base"].joints["stopcock"])
    on = pump_on(s["vacuum_line"])
    p = _trial["p"]
    rate = 0.0
    if not sealed:
        p = ATM
    else:
        if on:
            p -= PUMP_K * f * (p - P_PUMP) * dt
        else:
            rate = VENT_K * f * math.sqrt(max(0.0, ATM - p) / ATM)
            p += rate * dt
        p = min(ATM, p + LEAK * dt)
    _trial["p"] = p
    if s["base"].joints["stopcock"] >= math.radians(5.0):
        _trial["gauge"] = p
    else:
        _trial["gauge"] = P_PUMP if on else ATM
    _trial["max_rate"] = max(_trial["max_rate"], rate)
    blow = rate > GENTLE
    _trial["blown"] = _trial["blown"] or blow
    line = base_env.scene["vacuum_line"]
    if _trial["ids"] is None:
        _trial["ids"] = [list(line.joint_names).index(NEEDLE.joint_name)]
    line.set_joint_position_target(torch.tensor([[-(ATM - _trial["gauge"]) * NEEDLE_PER_KPA]], device=line.device),
                                   joint_ids=_trial["ids"])
    push = min(HOLD_MAX, (ATM - p) * math.pi * WALL_R ** 2 * 1000.0) if sealed and p < ATM - 1.0 else 0.0
    _wrench(base_env.scene["lid"], (0.0, 0.0, -push))
    _wrench(base_env.scene["base"], (0.0, 0.0, push))  # the same push holds the base up against the lid
    _wrench(base_env.scene["dish"], (0.0, 0.5, 0.4) if blow else (0.0, 0.0, 0.0))


def gauge_kpa() -> float:
    return -(ATM - _trial["gauge"])


# -- scoring ----------------------------------------------------------------------------------------------------------
UPRIGHT_DEG = 10.0
MARK_TOL = 0.015
TARGET_KPA = -90.0
CLOSED_TOL = math.radians(10.0)


def vial_on_plate(v, base) -> bool:
    rel = base.to_local(v.pos)
    return (v.tilt_deg() <= UPRIGHT_DEG and abs(rel[2] - PLATE_Z) <= 0.003 and math.hypot(rel[0], rel[1]) <= PLATE_R
            - VIAL_R)


def score(objects):
    """0 unless the vial stands upright on the plate inside the base and the dome is seated on the flange. 0.25 then;
    0.5 if the desiccator is also at -90 kPa or lower; 0.75 if, besides, the stopcock is shut (within 10 degrees) and the
    pump switched off; 1.0 if, besides, the vial's centre is within 1.5 cm of the ring's. Capped at 0.25 if the
    desiccator was ever vented faster than 12 kPa a second."""
    v, lid, base, line = objects["vial"], objects["lid"], objects["base"], objects["vacuum_line"]
    if not (vial_on_plate(v, base) and lid_sealed(lid, base)):
        return 0.0
    s = 0.25
    if -(ATM - _trial["p"]) <= TARGET_KPA:
        s = 0.5
        if base.joints["stopcock"] <= CLOSED_TOL and not pump_on(line):
            s = 0.75
            rel = base.to_local(v.pos)
            if math.hypot(rel[0] - MARK_XY[0], rel[1] - MARK_XY[1]) <= MARK_TOL:
                s = 1.0
    return min(s, 0.25) if _trial["blown"] else s


SUCCESS_HOLD_S = 20.0  # long enough after full score for the pull on the knob that checks the seal


def success(objects):
    return score(objects) == 1.0


def distance(objects):
    v, base = objects["vial"], objects["base"]
    rel = base.to_local(v.pos)
    return math.hypot(rel[0] - MARK_XY[0], rel[1] - MARK_XY[1]) + max(0.0, -(ATM - _trial["p"]) - TARGET_KPA) * 0.001


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The demo reads only what a policy can see: the gauge's dial, in view of the right wrist camera while that hand is on
# the stopcock. It never reads the box's pressure, a joint angle or an object pose. Right arm: straddles the stopcock's
# post with its fingers and swings the lever open a little at a time, only while the gauge's reading rises slowly (a
# gentle vent), then fully open; grips the dome's knob, lifts the dome and holds it up beside the
# base. Left arm (meanwhile): picks up the vial and, once the dome is up, stands it on the ring. Right arm: sets the dome
# back on the flange, lets go; presses the pump switch on; after a wait swings the stopcock shut; presses the switch off; pulls up gently on the knob, lets go; both park.
TIP_ABOVE_TCP = 0.013
LEFT_PARK = (0.15, 0.25, TOP + 0.16)
RIGHT_PARK = (0.17, -0.22, TOP + 0.14)
KNOB_Z = TOP + FLANGE_TOP + DOME_H + KNOB_H - 0.008 + TIP_ABOVE_TCP  # pad tips 8 mm down the knob
KNOB_WIDE, KNOB_HOLD = -0.032, -0.0125  # round the 35 mm knob (ribs included), squeezing it
LIFT = 0.020
KNOB_YAW = math.pi / 2  # fingers close along x on the knob
LID_HELD = (0.33, -0.22)  # where the right hand holds the dome while the vial goes in: beside the base, away from the
#   ring the vial goes on
HELD_UP = 0.050  # the dome's rim this far above the flange's top while held
BACK_OFF = (0.0, 0.0)  # the hand's offset from the base's axis when it sets the dome back
VIAL_GRIP_Z = TOP + 0.022 + TIP_ABOVE_TCP
VIAL_WIDE, VIAL_HOLD = -0.020, -0.008
VIAL_CARRY_Z = TOP + FLANGE_TOP + 0.015 + 0.022 + TIP_ABOVE_TCP
POST_Z = TOP + STOP_Z + 0.019 + TIP_ABOVE_TCP  # tips 3 mm above the lever bar, 15 mm down the 20 mm post
POST_WIDE = -0.010
SWITCH_Z = TOP + BOX[2] + 0.004 - 0.005 + TIP_ABOVE_TCP  # closed tips 5 mm below the pivot, near the paddle's end
SWITCH_YAW = math.pi / 2  # closed fingers side by side along x: a narrow press on one end of the paddle
VENT_S = 26.0  # the slow part of the vent
VENT_DEG_S = 8.0  # the lever's opening speed while the gauge rises gently
VENT_AIM = 7.0  # kPa a second, read off the gauge over the last half second
_vent = {"phi": 0.0, "tl": None, "seen": []}

_prev_action7 = _demo.ArmPlan.action7
if not getattr(_prev_action7, "_clock", False):
    def _action7(self, t, _prev=_prev_action7):
        _demo._plan_clock = float(t)
        return _prev(self, t)

    _action7._clock = True
    _demo.ArmPlan.action7 = _action7


def _clock():
    return float(getattr(_demo, "_plan_clock", 0.0))


@on_reset
def _vent_reset(env):
    _vent.update(phi=0.0, tl=None, seen=[])


def _post(phi):
    """World xy of the stopcock lever's post with the lever turned phi (0 shut, pointing at the robots)."""
    ax, ay = BX + STOP_XY[0], BY + STOP_XY[1]
    r = LEVER_L - 0.004
    return ax - r * math.cos(phi), ay - r * math.sin(phi)


def _vent_step():
    """Read the gauge's dial and open the lever further only while its reading rises slower than VENT_AIM."""
    t = _clock()
    dt = 0.0 if _vent["tl"] is None else max(0.0, t - _vent["tl"])
    _vent["tl"] = t
    g = gauge_kpa()  # what the dial shows
    seen = _vent["seen"]
    seen.append((t, g))
    while len(seen) > 2 and seen[1][0] <= t - 0.5:
        seen.pop(0)
    t0, g0 = seen[0]
    rate = (g - g0) / (t - t0) if t - t0 > 0.2 else 0.0
    if rate < VENT_AIM:  # the dial reads 0 until the stopcock is cracked open, then the box, rising as it fills
        _vent["phi"] = min(STOP_OPEN, _vent["phi"] + math.radians(VENT_DEG_S) * dt)


class _PostAt:
    """The right hand on the lever's post at the vent's current opening (advanced each step)."""

    def __init__(self, z):
        self.z = z

    def __array__(self, dtype=None, copy=None):
        _vent_step()
        x, y = _post(_vent["phi"])
        return np.asarray((x, y, self.z), dtype=dtype or float)


class _PostYaw(float):
    """The hand's yaw on the post: turned with the lever (60 percent of its angle; the post turns in the open fingers)."""

    def __new__(cls):
        return float.__new__(cls, 0.0)

    @property
    def rot(self):
        return _prev_grip(0.6 * _vent["phi"])


_prev_grip = _demo.grip_rotation
if not getattr(_prev_grip, "_rot", False):
    def _grip(yaw=0.0):
        return yaw.rot if hasattr(yaw, "rot") else _prev_grip(yaw)

    _grip._rot = True
    _demo.grip_rotation = _grip


def _dur(wps):
    return sum(w.duration for w in wps)


def demo():
    # 1. vent: swing the stopcock open slowly with the pump off
    px, py0 = _post(0.0)
    right = [
        Waypoint(2.5, (px, py0, POST_Z + 0.03), POST_WIDE, "above the stopcock lever's post", 0.0),
        Waypoint(1.0, (px, py0, POST_Z + 0.03), POST_WIDE, "settle over the post", 0.0),
        Waypoint(1.5, (px, py0, POST_Z), POST_WIDE, "fingers either side of the post", 0.0),
    ]
    right += [
        Waypoint(0.1, _PostAt(POST_Z), POST_WIDE, "swing the stopcock open slowly, watching the gauge", _PostYaw()),
        Waypoint(VENT_S, _PostAt(POST_Z), POST_WIDE, "swing the stopcock open slowly, watching the gauge", _PostYaw()),
    ]
    right += [
        Waypoint(1.5, (*_post(STOP_OPEN + 0.08), POST_Z), POST_WIDE, "swing it fully open", 0.6 * STOP_OPEN),
        Waypoint(0.8, (*_post(STOP_OPEN), POST_Z + 0.03), POST_WIDE, "up off the post", 0.6 * STOP_OPEN),
    ]
    # 2. lift the dome off by its knob and hold it up beside the base
    kx, ky = BX, BY
    hx, hy = LID_HELD
    right += [
        Waypoint(2.5, (kx, ky, KNOB_Z + 0.03), KNOB_WIDE, "above the dome's knob", KNOB_YAW),
        Waypoint(1.0, (kx, ky, KNOB_Z + 0.03), KNOB_WIDE, "settle over the knob", KNOB_YAW),
        Waypoint(2.0, (kx, ky, KNOB_Z), KNOB_WIDE, "fingers round the knob", KNOB_YAW),
        Waypoint(0.6, (kx, ky, KNOB_Z), KNOB_HOLD, "grip the knob", KNOB_YAW),
        Waypoint(1.5, (kx, ky, KNOB_Z + LIFT), KNOB_HOLD, "lift the dome off, level", KNOB_YAW),
        Waypoint(1.5, (kx, ky, KNOB_Z + HELD_UP), KNOB_HOLD, "lift it clear of the base", KNOB_YAW),
        Waypoint(3.0, (hx, hy, KNOB_Z + HELD_UP), KNOB_HOLD, "hold it up beside the base, level", KNOB_YAW),
    ]
    t_lid_off = _dur(right)
    # the left hand picks the vial up meanwhile and waits with it
    vx, vy = VIAL_XY
    mx, my = BX + MARK_XY[0], BY + MARK_XY[1]
    left = [
        Waypoint(max(0.3, t_lid_off - 11.0), LEFT_PARK, FINGER_OPEN, "wait"),
        Waypoint(2.5, (vx, vy, VIAL_GRIP_Z + 0.05), VIAL_WIDE, "above the vial"),
        Waypoint(1.2, (vx, vy, VIAL_GRIP_Z), VIAL_WIDE, "fingers round the vial"),
        Waypoint(0.6, (vx, vy, VIAL_GRIP_Z), VIAL_HOLD, "grip it"),
        Waypoint(1.2, (vx, vy, VIAL_CARRY_Z), VIAL_HOLD, "lift it"),
    ]
    left.append(Waypoint(max(0.3, t_lid_off - _dur(left)), (vx, vy, VIAL_CARRY_Z), VIAL_HOLD,
                         "wait for the dome to be off"))
    left += [
        Waypoint(2.5, (mx, my, VIAL_CARRY_Z), VIAL_HOLD, "over the ring on the plate"),
        Waypoint(2.0, (mx, my, VIAL_GRIP_Z + PLATE_Z + 0.001), VIAL_HOLD, "stand it on the ring"),
        Waypoint(0.6, (mx, my, VIAL_GRIP_Z + PLATE_Z + 0.001), VIAL_WIDE, "let go"),
        Waypoint(1.5, (mx, my, VIAL_CARRY_Z + 0.03), VIAL_WIDE, "up out of the base"),
        Waypoint(2.0, LEFT_PARK, FINGER_OPEN, "out of the way"),
    ]
    t_vial = _dur(left) + 0.3  # the dome goes back once the left hand is out of the way
    right.append(Waypoint(max(0.3, t_vial - _dur(right)), (hx, hy, KNOB_Z + HELD_UP), KNOB_HOLD,
                          "hold the dome while the vial goes in", KNOB_YAW))
    # 3. the dome back on, centred
    bx, by = BX + BACK_OFF[0], BY + BACK_OFF[1]
    right += [
        Waypoint(3.0, (bx, by, KNOB_Z + HELD_UP), KNOB_HOLD, "back over the base", KNOB_YAW),
        Waypoint(1.5, (bx, by, KNOB_Z + LIFT), KNOB_HOLD, "down, level", KNOB_YAW),
        Waypoint(2.0, (bx, by, KNOB_Z - 0.0015), KNOB_HOLD, "lower it onto the flange, centred", KNOB_YAW),
        Waypoint(0.8, (bx, by, KNOB_Z - 0.0015), KNOB_HOLD, "let it settle on the seal", KNOB_YAW),
        Waypoint(0.6, (bx, by, KNOB_Z - 0.0015), KNOB_WIDE, "let go", KNOB_YAW),
        Waypoint(1.0, (bx, by, KNOB_Z + 0.03), KNOB_WIDE, "up", KNOB_YAW),
    ]
    # 4. pump down: switch the pump on (the stopcock is still open), wait, swing the stopcock shut, switch it off
    sx, sy = LINE_XY[0] + SWITCH_X, LINE_XY[1]
    on_x, off_x = sx + 0.012, sx - 0.012  # near each end of the 32 mm paddle
    right += [
        Waypoint(2.5, (on_x, sy, SWITCH_Z + 0.04), FINGER_CLOSED, "above the pump switch's red end", SWITCH_YAW),
        Waypoint(1.0, (on_x, sy, SWITCH_Z), FINGER_CLOSED, "press it down: pump on", SWITCH_YAW),
        Waypoint(0.6, (on_x, sy, SWITCH_Z + 0.04), FINGER_CLOSED, "up", SWITCH_YAW),
    ]
    pxo, pyo = _post(STOP_OPEN)
    yo = 0.6 * STOP_OPEN
    right += [
        Waypoint(2.0, (pxo, pyo, POST_Z + 0.03), POST_WIDE, "above the open lever's post, watching the gauge", yo),
        Waypoint(1.0, (pxo, pyo, POST_Z + 0.03), POST_WIDE, "settle over the post", yo),
        Waypoint(1.5, (pxo, pyo, POST_Z), POST_WIDE, "fingers either side of the post", yo),
        Waypoint(3.0, (pxo, pyo, POST_Z), POST_WIDE, "wait for -90 kPa or lower on the gauge", yo),
    ]
    for phi in np.linspace(STOP_OPEN - 0.25, -0.12, 6):  # a little past shut, against the stop
        x, y = _post(phi)
        right.append(Waypoint(0.5, (x, y, POST_Z), POST_WIDE, "swing the stopcock shut", 0.6 * phi))
    right += [
        Waypoint(0.8, (x, y, POST_Z + 0.03), POST_WIDE, "up off the post", 0.0),
        Waypoint(2.0, (off_x, sy, SWITCH_Z + 0.04), FINGER_CLOSED, "above the switch's black end", SWITCH_YAW),
        Waypoint(1.0, (off_x, sy, SWITCH_Z), FINGER_CLOSED, "press it down: pump off", SWITCH_YAW),
        Waypoint(0.6, (off_x, sy, SWITCH_Z + 0.04), FINGER_CLOSED, "up", SWITCH_YAW),
    ]
    # 5. check the dome is held: pull up gently on the knob
    right += [
        Waypoint(2.5, (kx, ky, KNOB_Z + 0.03), KNOB_WIDE, "above the knob", KNOB_YAW),
        Waypoint(1.0, (kx, ky, KNOB_Z + 0.03), KNOB_WIDE, "settle over the knob", KNOB_YAW),
        Waypoint(2.0, (kx, ky, KNOB_Z), KNOB_WIDE, "fingers round the knob", KNOB_YAW),
        Waypoint(0.6, (kx, ky, KNOB_Z), KNOB_HOLD, "grip it", KNOB_YAW),
        Waypoint(1.5, (kx, ky, KNOB_Z + 0.012), KNOB_HOLD, "pull up on the knob: the dome stays down", KNOB_YAW),
        Waypoint(1.5, (kx, ky, KNOB_Z + 0.012), KNOB_HOLD, "keep pulling: still sealed", KNOB_YAW),
        Waypoint(0.6, (kx, ky, KNOB_Z + 0.012), KNOB_WIDE, "let go", KNOB_YAW),
        Waypoint(1.0, (kx, ky, KNOB_Z + 0.04), KNOB_WIDE, "up", KNOB_YAW),
        Waypoint(2.0, RIGHT_PARK, FINGER_OPEN, "park"),
    ]
    t_end = max(_dur(left), _dur(right)) + 3.0
    left.append(Waypoint(t_end - _dur(left), LEFT_PARK, FINGER_OPEN, "rest"))
    right.append(Waypoint(t_end - _dur(right), RIGHT_PARK, FINGER_OPEN, "rest"))
    return {"left": left, "right": right}
