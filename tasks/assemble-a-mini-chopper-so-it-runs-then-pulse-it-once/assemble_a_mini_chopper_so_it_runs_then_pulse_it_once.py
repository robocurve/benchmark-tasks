"""On the kitchen table stand the parts of a cordless electric mini chopper (a mini food processor of about 400 ml):
its clear bowl (a rubber foot ring underneath, a stainless pin up the middle, a handle on its left side), the S-blade on
its storage post to the right, and the motor top (white, 0.28 kg, a big sprung PULSE button on top) standing on the
table in front of them.

Task: assemble the chopper so it can run, then pulse it once. Put the blade on the bowl's pin, handling it only by its
hub. Set the motor top down on the bowl, its arrow at the bowl's open-lock mark, and turn it 30 degrees anticlockwise
(seen from above) until it clicks: its safety switch closes and its light comes on. The bayonet takes a firm twist
(0.3 N m), and the light bowl turns with the motor top unless the other hand holds it by its handle. Push PULSE down past
its switch point (2 mm of its 3.5 mm travel, 6 N) and let it up: the motor spins the blade while it is held. Then take
both arms back to where they started.

Run it:  inspect-robots run --embodiment yam_arena -E task=assemble_a_mini_chopper_so_it_runs_then_pulse_it_once.py \\
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
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step,  # noqa: E402
                     rotation, set_slide_target, set_thread_on, slider, thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the bowl: origin at the middle of its underside, on the table ----------------------------------------------------
BOWL_AT = (0.255, -0.050)
R_IN, WALL_T, RIM_Z = 0.045, 0.0035, 0.074  # inside radius, wall, rim height
FOOT_H, FLOOR_Z = 0.004, 0.009  # the rubber foot ring's height; the floor's top
BOWL_MASS = 0.14
PIN_R, PIN_TOP = 0.004, 0.049  # the stainless pin (and a 4 mm narrower tip above PIN_TOP)
HANDLE_DEG = 90.0  # the handle points left (+y)
H_R, H_RB, H_Z0, H_Z1 = 0.092, 0.010, 0.016, 0.068  # the round upright bar: its axis from the bowl's, radius, bottom, top
H_ARM_W, H_ARM_T, H_ARM_TOP = 0.014, 0.008, 0.060  # the arms joining it to the wall: width, thickness, the top one's
# top (below the motor top's collar)
OPEN_DEG = 0.0  # the bowl's open-lock mark (the motor top's arrow sits here when set on)
# -- the S-blade: origin at its hub's bottom, on the axis --------------------------------------------------------------
HUB_RI, HUB_RO, HUB_H = 0.0065, 0.0115, 0.042
BLADE_R1, BLADE_W, BLADE_T = 0.040, 0.014, 0.002
BLADE_Z = (0.006, 0.015)  # the two blades' heights on the hub (lower one along +x, upper along -x)
BLADE_MASS = 0.035
STAND = (0.335, -0.175)  # the blade's storage post
STAND_H, STAND_POST = 0.008, 0.040
# -- the motor top: origin at the middle of its plate's underside ------------------------------------------------------
MOTOR_AT = (0.195, -0.170)
SKIRT_R, SKIRT_T, SKIRT_H = 0.051, 0.003, 0.010  # the collar round the bowl's rim (inside 0.0495)
PLATE_R, PLATE_T = 0.0525, 0.006
HOUSING_R, HOUSING_TOP = 0.034, 0.056
MOTOR_MASS = 0.27  # with its 10 g PULSE button: 0.28 kg
FIN_DEG, FIN_R0, FIN_R1, FIN_T, FIN_TOP = -90.0, 0.032, 0.076, 0.012, 0.031  # the twist grip, in the motor's frame
FIN_GRIP_R = 0.066  # the hand takes the fin this far from the axis
# -- PULSE: a sprung push button on a slide in the housing's top (its own body) ------------------------------------------
BUTTON_R, BUTTON_TOP, BUTTON_H = 0.017, 0.060, 0.010  # radius; its top at rest, above the plate's underside; its length
BUTTON_MASS = 0.010
BTN_Z0 = BUTTON_TOP - BUTTON_H / 2  # its middle at rest, in the motor top's frame
BTN_TRAVEL = 0.0035  # it goes 3.5 mm down to its stop
SWITCH_AT, SWITCH_OFF = 0.0020, 0.0010  # its switch closes 2 mm down and opens again once it is back above 1 mm
BTN_K, BTN_PRELOAD, BTN_DAMP = 1000.0, 0.004, 3.0  # N/m, its spring 4 mm compressed at rest: 4 N up, 6 N at 2 mm
# -- the bayonet ------------------------------------------------------------------------------------------------------
LOCK_DEG = 30.0  # it locks 30 degrees anticlockwise from "set on"
LIMIT_DEG = 31.5
LOCKED_AT = 28.5  # turned this far: it clicks (the detent takes it to the stop)
K_TWIST, SLIP = 0.12, 2.5  # N m per degree; the lugs hold it until pushed 2.5 degrees past (0.30 N m)
CATCH_R, CATCH_DZ, CATCH_TILT, CATCH_YAW = 0.008, (-0.004, 0.0006), 8.0, 14.0  # set down on the rim
RELEASE_DEG, RELEASE_DZ, RELEASE_STEPS = 8.0, 0.0008, 3  # back at the open mark and pulled up: the top comes off
ON_R, ON_DZ, ON_TILT = 0.003, 0.0025, 3.0  # the motor top seated on the rim (checked live)
# -- rules -------------------------------------------------------------------------------------------------------------
SEAT_R, SEAT_DZ, SEAT_TILT = 0.004, 0.004, 10.0  # the blade on its pin
EDGE_NEAR, EDGE_R0 = 0.008, 0.020  # a fingertip within 8 mm of a blade's edges (its cutting part: beyond 2 cm)
SPIN_W = 25.0  # while the motor runs, the blade on its pin spins at 25 rad/s
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0

WHITE, BAND, DARK, GREY = (0.94, 0.94, 0.92), (0.55, 0.57, 0.60), (0.12, 0.12, 0.13), (0.42, 0.43, 0.46)
CLEAR, CLEAR_D, STEEL, HUB_C = (0.82, 0.88, 0.93), (0.68, 0.76, 0.84), (0.80, 0.81, 0.84), (0.30, 0.31, 0.33)
WOOD, GREEN, RUBBER = (0.62, 0.45, 0.28), (0.20, 0.90, 0.30), (0.10, 0.10, 0.11)
HIDDEN = np.array([0.40, 0.0, 0.30])  # under the table

AXIS = np.array([BOWL_AT[0], BOWL_AT[1], TOP])  # the bowl's origin at the start


def _yawq(deg: float) -> tuple[float, float, float, float]:
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def _polar(r: float, deg: float) -> tuple[float, float]:
    return r * math.cos(math.radians(deg)), r * math.sin(math.radians(deg))


def _ring(r: float, z0: float, z1: float, t: float, color, n: int = 24, collide: bool = True) -> list:
    """A round wall of n flat panels, mid-radius r, from z0 to z1, t thick (color None: clear, collision only)."""
    out = []
    w = 2 * (r + t / 2) * math.tan(math.pi / n) + 0.001
    for k in range(n):
        a = 2 * math.pi * (k + 0.5) / n
        out.append(BoxShape((r * math.cos(a), r * math.sin(a), (z0 + z1) / 2), (t, w, z1 - z0), color, yaw=a,
                            collide=collide))
    return out


def _bowl() -> Body:
    """The clear bowl: drawn as its rim, foot and floor (the clear wall itself is a collision-only shell), with a
    stainless pin up the middle and a handle on its left."""
    r_mid = R_IN + WALL_T / 2
    s = [CylShape((0.0, 0.0, FOOT_H / 2), R_IN, FOOT_H, RUBBER)]  # the rubber foot ring
    s.append(CylShape((0.0, 0.0, (FOOT_H + FLOOR_Z) / 2), R_IN + WALL_T, FLOOR_Z - FOOT_H, CLEAR))
    s += _ring(r_mid, FOOT_H, RIM_Z, WALL_T, None)  # the clear wall: collides, not drawn
    s += _ring(r_mid, RIM_Z - 0.004, RIM_Z, WALL_T + 0.0004, CLEAR_D, collide=False)  # its rim, drawn
    s += _ring(r_mid, FOOT_H, FLOOR_Z + 0.002, WALL_T + 0.0004, CLEAR_D, collide=False)  # its foot edge, drawn
    for k in range(4):  # four faint seams up the wall, so it reads as a clear bowl
        x, y = _polar(R_IN + WALL_T + 0.0002, 45.0 + 90.0 * k)
        s.append(BoxShape((x, y, (FLOOR_Z + RIM_Z) / 2), (0.0006, 0.0015, RIM_Z - FLOOR_Z - 0.006), CLEAR_D,
                          yaw=math.radians(45.0 + 90.0 * k), collide=False))
    s.append(CylShape((0.0, 0.0, (FLOOR_Z + PIN_TOP) / 2), PIN_R, PIN_TOP - FLOOR_Z, STEEL))  # the pin
    s.append(CylShape((0.0, 0.0, PIN_TOP + 0.002), PIN_R - 0.0015, 0.004, STEEL))  # its narrower tip: a lead-in
    a = HANDLE_DEG
    hx, hy = _polar(H_R, a)
    s.append(CylShape((hx, hy, (H_Z0 + H_Z1) / 2), H_RB, H_Z1 - H_Z0, CLEAR_D))  # round: it turns in a grip
    for z0, z1 in ((H_ARM_TOP - H_ARM_T, H_ARM_TOP), (H_Z0, H_Z0 + H_ARM_T)):  # the top and bottom arms
        r0, r1 = R_IN + WALL_T - 0.001, H_R
        cx, cy = _polar((r0 + r1) / 2, a)
        s.append(BoxShape((cx, cy, (z0 + z1) / 2), (r1 - r0, H_ARM_W, z1 - z0), CLEAR_D, yaw=math.radians(a)))
    # the lock marks just under the rim: an open dot where the motor top's arrow goes, a bar 30 degrees on
    for deg, size in ((OPEN_DEG, (0.0008, 0.005, 0.005)), (OPEN_DEG + LOCK_DEG, (0.0008, 0.003, 0.009))):
        x, y = _polar(R_IN + WALL_T + 0.0004, deg)
        s.append(BoxShape((x, y, RIM_Z - 0.010), size, DARK, yaw=math.radians(deg), collide=False))
    return Body.build("bowl", pos=tuple(AXIS.tolist()), mass=BOWL_MASS, shapes=s, color=CLEAR, friction=(0.9, 0.8),
                      com=(0.0, 0.0, 0.025))


def _blade() -> Body:
    s = _ring((HUB_RI + HUB_RO) / 2, 0.0, HUB_H, HUB_RO - HUB_RI, HUB_C, n=10)  # the hub (open at the top: the drive)
    s.append(CylShape((0.0, 0.0, HUB_H - 0.0005), HUB_RI + 0.0005, 0.001, DARK, collide=False))  # the drive socket
    for z, sign in ((BLADE_Z[0], 1.0), (BLADE_Z[1], -1.0)):
        r = (HUB_RO + BLADE_R1) / 2
        s.append(BoxShape((sign * r, sign * 0.002, z + BLADE_T / 2), (BLADE_R1 - HUB_RO, BLADE_W, BLADE_T), STEEL,
                          yaw=sign * math.radians(6.0)))
        s.append(BoxShape((sign * (BLADE_R1 - 0.004), sign * 0.002, z + BLADE_T + 0.0003),
                          (0.006, BLADE_W - 0.002, 0.0006), (0.93, 0.94, 0.96), collide=False))  # the bright edge
    return Body.build("blade", pos=(STAND[0], STAND[1], TOP + STAND_H), mass=BLADE_MASS, shapes=s, color=STEEL,
                      friction=(0.5, 0.4), damping=(0.05, 0.6), com=(0.0, 0.0, 0.012))


def _stand() -> Body:
    s = [CylShape((0.0, 0.0, STAND_H / 2), 0.030, STAND_H, WOOD),
         CylShape((0.0, 0.0, STAND_H + STAND_POST / 2), PIN_R, STAND_POST, (0.55, 0.40, 0.25)),
         CylShape((0.0, 0.0, STAND_H + STAND_POST + 0.002), PIN_R - 0.0015, 0.004, (0.55, 0.40, 0.25))]
    return Body.build("stand", pos=(STAND[0], STAND[1], TOP), mass=0.2, kinematic=True, shapes=s, color=WOOD,
                      friction=(0.6, 0.5))


def _motor() -> Body:
    s = _ring(SKIRT_R, -SKIRT_H, 0.0, SKIRT_T, WHITE)  # the collar that fits round the bowl's rim
    s.append(CylShape((0.0, 0.0, PLATE_T / 2), PLATE_R, PLATE_T, WHITE))
    s.append(CylShape((0.0, 0.0, PLATE_T + 0.002), PLATE_R + 0.0003, 0.004, BAND, collide=False))  # a grey band
    s.append(CylShape((0.0, 0.0, (PLATE_T + HOUSING_TOP) / 2), HOUSING_R, HOUSING_TOP - PLATE_T, WHITE))
    s.append(CylShape((0.0, 0.0, HOUSING_TOP + 0.0002), BUTTON_R + 0.0012, 0.0004, DARK, collide=False))  # PULSE's gap
    lx, ly = _polar(0.026, 180.0)
    s.append(CylShape((lx, ly, HOUSING_TOP + 0.0004), 0.0035, 0.0008, DARK, collide=False))  # the light, dark
    ax_, ay_ = _polar(PLATE_R - 0.004, 0.0)  # the arrow on the plate's edge (lines up with the bowl's marks)
    s.append(BoxShape((ax_, ay_, PLATE_T + 0.0042), (0.008, 0.003, 0.0006), DARK, collide=False))
    fx, fy = _polar((FIN_R0 + FIN_R1) / 2, FIN_DEG)  # the twist grip: a fin standing out on one side
    s.append(BoxShape((fx, fy, (PLATE_T + FIN_TOP) / 2), (FIN_R1 - FIN_R0, FIN_T, FIN_TOP - PLATE_T), WHITE,
                      yaw=math.radians(FIN_DEG)))
    for k in range(3):  # its ribs
        rx, ry = _polar(FIN_R1 - 0.006 - 0.008 * k, FIN_DEG)
        s.append(BoxShape((rx, ry, FIN_TOP + 0.0004), (0.002, FIN_T - 0.002, 0.0008), BAND,
                          yaw=math.radians(FIN_DEG), collide=False))
    s.append(CylShape((0.0, 0.0, -0.009), 0.004, 0.018, STEEL, collide=False))  # the drive shaft underneath
    return Body.build("motor", pos=(MOTOR_AT[0], MOTOR_AT[1], TOP + SKIRT_H), mass=MOTOR_MASS, shapes=s, color=WHITE,
                      friction=(0.7, 0.6), com=(0.0, 0.0, 0.024))


def _button() -> Body:
    """PULSE: a grey push button 3.4 cm across, its own body on a sprung slide in the housing's top (origin: its
    middle). Its lower part sits inside the housing."""
    top = BUTTON_H / 2
    s = [CylShape((0.0, 0.0, 0.0), BUTTON_R, BUTTON_H, GREY),
         BoxShape((0.0, 0.004, top + 0.0003), (0.002, 0.009, 0.0006), WHITE, collide=False)]  # its symbol
    s += _ring(0.009, top, top + 0.0006, 0.0015, WHITE, n=12, collide=False)
    return Body.build("pulse_button", pos=(MOTOR_AT[0], MOTOR_AT[1], TOP + SKIRT_H + BTN_Z0), mass=BUTTON_MASS,
                      shapes=s, color=GREY, friction=(0.8, 0.7), ignore=("motor",))


def _lamp() -> Body:
    """The motor top's light, lit: hidden under the table until its safety switch is closed."""
    return Body.build("power_light", pos=tuple(HIDDEN.tolist()), mass=0.01, kinematic=True, color=GREEN,
                      shapes=[CylShape((0.0, 0.0, 0.0), 0.0038, 0.0012, GREEN, collide=False)])


thread("bayonet", "bowl", "motor", anchor0=(0.0, 0.0, RIM_Z), axis0=(0.0, 0.0, 1.0), anchor1=(0.0, 0.0, 0.0),
       axis1=(0.0, 0.0, 1.0), slide_limits=(-0.001, 0.001), turn_damping=0.002, slide_stiffness=3000.0,
       slide_damping=30.0, slide_max_force=40.0)

# PULSE's slide: 3.5 mm of travel, its spring pushing it up against its top stop (preloaded), so it takes 6 N to push it
# to its switch point 2 mm down
slider("pulse_spring", "motor", "pulse_button", anchor=(MOTOR_AT[0], MOTOR_AT[1], TOP + SKIRT_H + BTN_Z0), axis="Z",
       limits=(-BTN_TRAVEL, 0.0), stiffness=BTN_K, damping=BTN_DAMP, target=BTN_PRELOAD, max_force=30.0)

OBJECTS = [_bowl(), _stand(), _blade(), _motor(), _button(), _lamp()]

# the bayonet joint: a twist limit and a stick-slip spring on its turn (the lugs' hold)
_author_threads_plain = bodykit._author_threads
_TWIST: dict = {}


def _author_threads(stage, prim_path: str, spec) -> None:
    from pxr import UsdPhysics

    _author_threads_plain(stage, prim_path, spec)
    if spec.name != "motor":
        return
    prim = stage.GetPrimAtPath(f"{prim_path}/bayonet")
    if not (prim and prim.IsValid()):
        return
    limit = UsdPhysics.LimitAPI.Apply(prim, "rotX")
    limit.CreateLowAttr(-CATCH_YAW - 4.0)
    limit.CreateHighAttr(LIMIT_DEG)
    drive = UsdPhysics.DriveAPI(prim, "rotX")
    drive.GetStiffnessAttr().Set(float(K_TWIST))
    drive.CreateTargetPositionAttr(0.0)
    drive.GetMaxForceAttr().Set(float(1.6 * K_TWIST * SLIP))
    _TWIST["target"] = drive.GetTargetPositionAttr()


bodykit._author_threads = _author_threads


def _set_twist(deg: float) -> None:
    attr = _TWIST.get("target")
    if attr is not None:
        attr.Set(float(deg))


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"steps": 0, "on": False, "turn": 0.0, "held": 0.0, "detent": False, "pull_steps": 0, "top_seated": False,
            "locked": False, "seated": False, "depth": 0.0, "switch": False, "running": False, "pulses": 0,
            "pulsed_at": None, "cut": False, "tcps": None, "lamp": False, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("CHOP_DEBUG"))}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    try:
        set_thread_on("bayonet", False)
        _set_twist(0.0)
    except Exception:  # before the scene exists
        pass


def _grippers(base, origin) -> list[tuple[np.ndarray, np.ndarray]]:
    """Each arm's TCP and the direction its fingers point (world)."""
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        r = rotation(g[3:7])
        out.append((g[:3] - origin + r @ np.array(TCP_OFFSET), r[:, 2]))
    return out


def _pose(base, origin, name):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _place(base, origin, name, at, yaw_deg=0.0) -> None:
    import torch

    asset = base.scene[name]
    p = np.asarray(at, float) + origin
    asset.write_root_pose_to_sim(torch.tensor([[*p.tolist(), *_yawq(yaw_deg)]], dtype=torch.float32,
                                              device=asset.device))


def _spin(base, w: float) -> None:
    import torch

    asset = base.scene["blade"]
    asset.write_root_velocity_to_sim(torch.tensor([[0.0, 0.0, 0.0, 0.0, 0.0, float(w)]], dtype=torch.float32,
                                                  device=asset.device))


def _yaw(r) -> float:
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def _wrap(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def _tilt(r) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))


def _seg_dist(p, a, b) -> float:
    ab = b - a
    t = max(0.0, min(1.0, float((p - a) @ ab / (ab @ ab))))
    return float(np.linalg.norm(p - (a + t * ab)))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    _EP["steps"] += 1
    t = _EP["steps"] * 0.05
    pw, rw = _pose(base, origin, "bowl")
    pm, rm = _pose(base, origin, "motor")
    pd, rd = _pose(base, origin, "blade")
    # the bayonet: caught when the motor top is set down on the rim with its arrow at the open mark; then its lugs hold
    # it where it was left until it is pushed 2.5 degrees past, and at 28.5 degrees it clicks into its lock (the detent
    # holds it against its stop until it is forced back below 28.5 degrees). Turned back to the open mark and pulled
    # up, its lugs are in their slots and it comes off.
    v = rw.T @ (pm - pw)
    dz, off, tilt = float(v[2] - RIM_Z), math.hypot(v[0], v[1]), _tilt(rw.T @ rm)
    rel = _wrap(_yaw(rm) - _yaw(rw) - OPEN_DEG)
    _EP["turn"] = rel
    if not _EP["on"]:
        if off <= CATCH_R and CATCH_DZ[0] <= dz <= CATCH_DZ[1] and tilt <= CATCH_TILT and abs(rel) <= CATCH_YAW:
            set_slide_target("bayonet", 0.0)
            _set_twist(rel)
            set_thread_on("bayonet", True)
            _EP.update(on=True, held=rel, detent=False, pull_steps=0)
    else:
        pulled = abs(rel) <= RELEASE_DEG and dz > RELEASE_DZ
        _EP["pull_steps"] = _EP["pull_steps"] + 1 if pulled else 0
        if _EP["pull_steps"] >= RELEASE_STEPS:  # lifted off at the open mark
            set_thread_on("bayonet", False)
            _set_twist(0.0)
            _EP.update(on=False, detent=False, pull_steps=0)
        elif _EP["detent"]:
            if rel < LOCKED_AT:  # forced back out of its lock: the lugs' friction holds it again
                _EP.update(detent=False, held=rel)
                _set_twist(rel)
        elif rel >= LOCKED_AT:
            _EP["detent"] = True
            _set_twist(LIMIT_DEG + 1.0)
        elif abs(rel - _EP["held"]) > SLIP:
            _EP["held"] = rel
            _set_twist(rel)
    # locked, live: the motor top caught by the bayonet, seated on the rim and turned into its lock
    _EP["top_seated"] = bool(_EP["on"] and off <= ON_R and abs(dz) <= ON_DZ and tilt <= ON_TILT)
    _EP["locked"] = bool(_EP["top_seated"] and rel >= LOCKED_AT)
    # the light: on while the safety switch is closed (locked)
    if _EP["locked"]:
        lx, ly = _polar(0.026, 180.0)
        _place(base, origin, "power_light", pm + rm @ np.array([lx, ly, HOUSING_TOP + 0.0010]), _yaw(rm))
        _EP["lamp"] = True
    elif _EP["lamp"]:
        _place(base, origin, "power_light", HIDDEN)
        _EP["lamp"] = False
    # the blade on its pin
    u = rw.T @ (pd - pw)
    _EP["seated"] = bool(math.hypot(u[0], u[1]) <= SEAT_R and abs(u[2] - FLOOR_Z) <= SEAT_DZ
                         and _tilt(rw.T @ rd) <= SEAT_TILT)
    # fingertips near the blades' cutting parts
    grips = _grippers(base, origin)
    _EP["tcps"] = [g[0] for g in grips]
    for z, sign in ((BLADE_Z[0], 1.0), (BLADE_Z[1], -1.0)):
        a = pd + rd @ np.array([sign * EDGE_R0, sign * 0.002, z + BLADE_T / 2])
        b = pd + rd @ np.array([sign * BLADE_R1, sign * 0.006, z + BLADE_T / 2])
        for tcp, axis in grips:
            for p in (tcp, tcp + 0.013 * axis):
                if _seg_dist(p, a, b) < EDGE_NEAR + BLADE_W / 2:
                    _EP["cut"] = True
    # PULSE: how far the button is pushed down its slide; its switch closes 2 mm down (and opens again above 1 mm).
    # The motor runs while the switch is closed and the motor top is locked; a pulse is the motor starting with the
    # blade on its pin.
    pb, _ = _pose(base, origin, "pulse_button")
    _EP["depth"] = depth = BTN_Z0 - float((rm.T @ (pb - pm))[2])
    _EP["switch"] = depth > SWITCH_OFF if _EP["switch"] else depth >= SWITCH_AT
    running = _EP["switch"] and _EP["locked"]
    if running and not _EP["running"] and _EP["seated"] and not _EP["cut"]:
        _EP["pulses"] += 1
        if _EP["pulsed_at"] is None:
            _EP["pulsed_at"] = t
    _EP["running"] = running
    if running and _EP["seated"]:
        _spin(base, SPIN_W)
    _EP["s"] = {"bowl_down": _tilt(rw) < 5.0 and pw[2] - TOP < 0.004}
    if _DEBUG["on"] and (_EP["steps"] % 5 == 0 or depth > 0.0005):
        tl, trc = _EP["tcps"]
        print(f"[chop] t={t:5.2f} bowl={np.round((pw - AXIS) * 1000).astype(int).tolist()} byaw={_yaw(rw):5.1f}"
              f" on={int(_EP['on'])} turn={rel:6.1f} held={_EP['held']:5.1f} det={int(_EP['detent'])}"
              f" locked={int(_EP['locked'])} motor=[{v[0] * 1000:.1f}, {v[1] * 1000:.1f}, {dz * 1000:.2f}]"
              f" blade={np.round((u - [0, 0, FLOOR_Z]) * 1000).astype(int).tolist()} seat={int(_EP['seated'])}"
              f" btn={depth * 1000:.2f} sw={int(_EP['switch'])} run={int(running)} pulses={_EP['pulses']}"
              f" cut={int(_EP['cut'])} pulsed={_EP['pulsed_at']}"
              f" L={np.round((tl - [0, 0, TOP]) * 1000).astype(int).tolist()}"
              f" R={np.round((trc - [0, 0, TOP]) * 1000).astype(int).tolist()} score={score(None)}",
              file=sys.stderr, flush=True)
        hp = pw + rw @ np.array([*_polar(H_R, HANDLE_DEG), H_Z1 - 0.010])
        fn = []
        for arm in ("left_robot", "right_robot"):
            rob = base.scene[arm]
            fj = [i for i, n in enumerate(rob.joint_names) if n in ("joint7", "joint8")]
            fn.append(np.round(as_numpy(rob.data.joint_pos)[0][fj] * 1000, 1).tolist())
        print(f"[chop]   handle={np.round((hp - [0, 0, TOP]) * 1000).astype(int).tolist()} fingers L={fn[0]} R={fn[1]}"
              f" Lax={np.round(grips[0][1], 2).tolist()}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """0.0 once a fingertip has come within 1.5 cm of the middle line of either blade's cutting part (the 1.4 cm wide
    blade beyond 2 cm from the hub's axis: about 8 mm from its edges).
    A pulse: PULSE pushed down past its switch point (2 mm of its 3.5 mm travel) while the motor top is locked on the
    bowl, with the blade on its pin (the motor starts). Locked is checked live, at that moment and at the end: the
    motor top caught by the bayonet, seated on the rim and turned at least 28.5 degrees into its lock.
    Otherwise 1.0 after exactly one pulse, with, at the end, PULSE let go (its switch open), the motor top still locked,
    the blade on its pin, the bowl standing on the table and both arms back where they started; 0.75 after at least one
    pulse; 0.5 with the motor top locked and the blade on its pin at the end; 0.25 with the blade on its pin at the end;
    else 0.0."""
    if _EP["cut"]:
        return 0.0
    if _EP["pulses"] >= 1:
        if (_EP["pulses"] == 1 and not _EP["switch"] and _EP["locked"] and _EP["seated"] and _EP["s"].get("bowl_down")
                and _arms_home()):
            return 1.0
        return 0.75
    if _EP["locked"] and _EP["seated"]:
        return 0.5
    return 0.25 if _EP["seated"] else 0.0


_HOME: dict = {}


def _arms_home() -> bool:
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
    """Full score with the bowl, the motor top, its button and the blade still: the trial ends there."""
    return (score(objects) >= 1.0 and objects["bowl"].resting() and objects["motor"].resting()
            and objects["pulse_button"].resting() and objects["blade"].resting())


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right takes the blade by the top of its hub, lifts it off its post, carries it over the bowl and lowers the hub
# over the pin, and lets go. It takes the motor top round its housing, carries it over the bowl and sets it down on the
# rim, its arrow at the open mark, and lets go. The left comes in and holds the bowl by its handle; the right takes the
# motor top's twist grip and carries it 31.5 degrees anticlockwise round the axis (its grip turning with it): the motor
# top clicks and its light comes on. Both let go; the left goes home, the right comes down on PULSE with its closed
# fingers, pushes it to its stop (the switch closes 2 mm down: the blade spins while it is held), lets it up and goes
# home. The left stays home while the right works over the bowl (two top-down grips closer than about 15 cm put the
# wrists into each other).

OVER = 0.035
LEAD_S, LEAD_STEPS = 3.0, 15
HUB_FINGER = -0.020
MOTOR_FINGER = -0.0455
HANDLE_FINGER = -0.0215
HUB_DZ = 0.008  # the TCP this far below the hub's top
HOUSING_DZ = 0.015  # ... below the housing's top
HANDLE_DZ = 0.010  # ... below the handle's top
HOLD_PRESS = 0.003  # the holding hand aims this far lower (the V-jaw pushes it up the bar)
FIN_DZ = 0.010  # ... below the twist grip's top
FIN_FINGER = -0.0205
RELEASE_FINGER = -0.032  # open wide before rising off a handle, so no finger drags it
TWIST = 34.0  # degrees the grip is carried round (the top stops at 31.5 against the bowl; the held bowl gives a little)
PRESS_DEPTH = 0.018  # aim the closed pads' tips this far below PULSE's top (it stops 3.5 mm down; the arm gives)
BACK_OFF = 33.0  # degrees: where the grip eases back to before letting go (only a little: the lock is live)
BLADE_AIM = np.array([0.0, 0.0, 0.0])  # where the blade hangs off the TCP (measured)
MOTOR_AIM = np.array([0.0, 0.014, 0.0])  # the motor top hangs 14 mm off the TCP along the closing axis (measured)


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
                if room > 0.4:
                    break
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


def _g(yaw_deg: float):
    return grip_rotation(math.radians(yaw_deg))


def _plan() -> dict:
    """Both arms' waypoints."""
    from yam_arena.constants import READY

    L, R = [], []
    ax = AXIS.copy()
    # the right: the blade, off its post by the top of its hub, onto the bowl's pin
    hub_top = np.array([STAND[0], STAND[1], TOP + STAND_H + HUB_H])
    g_hub = hub_top + [0.0, 0.0, -HUB_DZ]
    rb = _g(0.0)
    R += _joint_path("right", np.array(READY, float), _good_ik("right", g_hub + [0, 0, OVER], rb), "over the blade's hub",
                     HUB_FINGER)
    R.append(_w(1.2, g_hub, HUB_FINGER, "down round the hub's top", rb))
    R.append(_w(0.6, g_hub, FINGER_CLOSED, "take the blade by its hub", rb))
    lift = STAND_POST + 0.012
    R.append(_w(1.2, g_hub + [0, 0, lift], FINGER_CLOSED, "lift it off its post", rb))
    carry_z = RIM_Z + 0.014 + HUB_H - HUB_DZ
    R.append(_w(2.4, ax + BLADE_AIM + [0.0, 0.0, carry_z], FINGER_CLOSED, "carry it over the bowl", rb))
    R.append(_w(0.6, ax + BLADE_AIM + [0.0, 0.0, carry_z], FINGER_CLOSED, "steady over the pin", rb))
    down = ax + BLADE_AIM + [0.0, 0.0, FLOOR_Z + 0.005 + HUB_H - HUB_DZ]
    R.append(_w(1.8, down, FINGER_CLOSED, "lower the hub over the pin", rb))
    R.append(_w(0.6, down, HUB_FINGER, "let go of the blade", rb))
    R.append(_w(1.0, ax + [0.0, 0.0, carry_z + 0.010], HUB_FINGER, "rise clear of the bowl", rb))
    # the motor top: round its housing, onto the bowl with its arrow at the open mark
    rm0 = _g(0.0)  # fingers close along y (the fin points -y)
    g_mot = np.array([MOTOR_AT[0], MOTOR_AT[1], TOP + SKIRT_H + HOUSING_TOP - HOUSING_DZ])
    R.append(_w(2.2, g_mot + [0, 0, OVER], MOTOR_FINGER, "over the motor top", rm0))
    R.append(_w(1.2, g_mot, MOTOR_FINGER, "down round its housing", rm0))
    R.append(_w(0.6, g_mot, FINGER_CLOSED, "take the motor top", rm0))
    seat = ax + MOTOR_AIM + [0.0, 0.0, RIM_Z + HOUSING_TOP - HOUSING_DZ]
    carry = seat[2] + 0.026 - g_mot[2]  # its collar clear above the bowl's rim
    R.append(_w(1.6, g_mot + [0, 0, carry], FINGER_CLOSED, "lift it clear of the bowl's rim", rm0))
    R.append(_w(2.4, seat + [0, 0, 0.026], FINGER_CLOSED, "carry it over the bowl, arrow at the open mark", rm0))
    R.append(_w(0.6, seat + [0, 0, 0.026], FINGER_CLOSED, "steady over the bowl", rm0))
    R.append(_w(1.4, seat + [0, 0, 0.001], FINGER_CLOSED, "set it on the bowl", rm0))
    R.append(_w(0.6, seat + [0, 0, 0.001], MOTOR_FINGER, "let go of it", rm0))
    R.append(_w(0.8, seat + [0, 0, OVER], MOTOR_FINGER, "rise clear", rm0))
    # the right: to the twist grip
    fin_z = ax[2] + RIM_Z + FIN_TOP - FIN_DZ

    def at_fin(deg):
        fx, fy = _polar(FIN_GRIP_R, deg)
        return np.array([ax[0] + fx, ax[1] + fy, fin_z])

    f0 = at_fin(FIN_DEG)
    R.append(_w(1.6, f0 + [0, 0, OVER], FIN_FINGER, "over the motor top's twist grip", _g(FIN_DEG + 180.0)))
    t_ready = sum(w.duration for w in R)
    # the left: comes in and holds the bowl by its handle
    hx, hy = _polar(H_R, HANDLE_DEG)
    handle = ax + [hx, hy, H_Z1 - HANDLE_DZ]
    rl = _g(HANDLE_DEG - 180.0)  # fingers close across the bar (tangentially)
    from yam_arena import ik, station

    home_l, home_lr = ik.tcp_pose(np.array(READY, float), station.BASES["left"])
    L.append(_w(t_ready - LEAD_S - 1.0, home_l, HANDLE_FINGER, "wait at home", home_lr))
    L += _joint_path("left", np.array(READY, float), _good_ik("left", handle + [0, 0, OVER], rl), "over the handle",
                     HANDLE_FINGER)
    L.append(_w(1.2, handle, HANDLE_FINGER, "down round the handle", rl))
    L.append(_w(0.6, handle - [0, 0, HOLD_PRESS], FINGER_CLOSED, "hold the bowl by its handle", rl))
    t_held = sum(w.duration for w in L)
    R.append(_w(max(0.6, t_held - t_ready - 1.4), f0 + [0, 0, OVER], FIN_FINGER, "wait for the bowl to be held",
                _g(FIN_DEG + 180.0)))
    R.append(_w(1.2, f0, FIN_FINGER, "down round the twist grip", _g(FIN_DEG + 180.0)))
    R.append(_w(0.6, f0, FINGER_CLOSED, "take the twist grip", _g(FIN_DEG + 180.0)))
    n = 9
    for k in range(1, n + 1):
        d = FIN_DEG + TWIST * k / n
        R.append(_w(0.45, at_fin(d), FINGER_CLOSED, "turn it anticlockwise (it clicks, the light comes on)",
                    _g(d + 180.0)))
    d1 = FIN_DEG + TWIST
    f1 = at_fin(d1)
    R.append(_w(0.4, f1, FINGER_CLOSED, "hold it at the stop", _g(d1 + 180.0)))
    d2 = FIN_DEG + BACK_OFF  # ease off the stop (still past the click) so nothing springs back on letting go
    f2 = at_fin(d2)
    R.append(_w(0.5, f2, FINGER_CLOSED, "ease off the stop", _g(d2 + 180.0)))
    t_eased = sum(w.duration for w in R)
    # the left lets go first (the right still holds the locked motor top, and with it the bowl) and goes home
    L.append(_w(t_eased - sum(w.duration for w in L), handle - [0, 0, HOLD_PRESS], FINGER_CLOSED,
                "keep holding the bowl", rl))
    L.append(_w(0.8, handle, RELEASE_FINGER, "let go of the handle (wide open)", rl))
    L.append(_w(1.2, handle + [0, 0, OVER], RELEASE_FINGER, "rise clear", rl))
    L += _joint_path("left", _good_ik("left", handle + [0, 0, OVER], rl), np.array(READY, float), "back home",
                     RELEASE_FINGER)
    R.append(_w(2.2, f2, FINGER_CLOSED, "hold while the left lets go", _g(d2 + 180.0)))
    R.append(_w(0.8, f2, RELEASE_FINGER, "let go of the twist grip (wide open)", _g(d2 + 180.0)))
    above = ax[2] + RIM_Z + BUTTON_TOP + 0.040  # clear above the motor top's housing before moving in over it
    R.append(_w(1.2, np.array([f2[0], f2[1], above]), RELEASE_FINGER, "rise clear of the motor top", _g(d2 + 180.0)))
    t_left_home = sum(w.duration for w in L)
    # the right presses PULSE (once the left is clear) and goes home
    button = ax + [0.0, 0.0, RIM_Z + BUTTON_TOP]
    touch = button + [0.0, 0.0, PAD_TIP_PAST_TCP]  # the TCP with the closed pads' tips on PULSE
    rp = _g(0.0)
    R.append(_w(max(1.2, t_left_home - LEAD_S - sum(w.duration for w in R)), button + [0, 0, 0.040], FINGER_CLOSED,
                "over PULSE", rp))
    R.append(_w(0.8, touch + [0, 0, 0.004], FINGER_CLOSED, "down onto PULSE", rp))
    R.append(_w(0.6, touch - [0, 0, PRESS_DEPTH], FINGER_CLOSED, "push PULSE down: the motor runs, the blade spins", rp))
    R.append(_w(1.0, touch - [0, 0, PRESS_DEPTH], FINGER_CLOSED, "hold it down a moment", rp))
    R.append(_w(0.5, touch + [0, 0, 0.004], FINGER_CLOSED, "let it up: the motor stops", rp))
    R.append(_w(0.6, button + [0, 0, 0.040], FINGER_CLOSED, "rise clear", rp))
    R += _joint_path("right", _good_ik("right", button + [0, 0, 0.040], rp), np.array(READY, float), "back home",
                     FINGER_CLOSED)
    tr = sum(w.duration for w in R)
    tl = sum(w.duration for w in L)
    end = max(tr, tl) + 2.0
    R.append(_w(end - tr, R[-1].pos, FINGER_CLOSED, "hold still", R[-1].yaw))
    L.append(_w(end - tl, L[-1].pos, RELEASE_FINGER, "hold still", L[-1].yaw))
    return {"left": L, "right": R}


def demo():
    return _plan()
