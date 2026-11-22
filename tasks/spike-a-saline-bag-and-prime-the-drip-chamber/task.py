"""Aseptic IV-set preparation at a fixed YAM station.

A hanging bag and upright presentation rack expose two separately capped sterile
interfaces. The robot uncaps, inserts and primes the chamber while avoiding contact
of the exposed interfaces with gripper meshes, the work tray, the presentation rack,
the IV support and the bag's outside. The rack holds the set in a spring clip that
lets go only when the set is pulled out with more than RACK_CLIP_BREAK. The liquid-height
proxy integrates measured squeeze/release travel only when seated and upright.
The scripted demo uses physical joint actions; the giving set is never attached
to the robot by a grasp joint or driven with auxiliary forces.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, hinge, on_reset, on_step,  # noqa: E402
                     rotation, set_slide_target, set_thread_on, thread)
from asepsis import AsepsisMonitor, obstacles_from_body  # noqa: E402
from priming_meter import PrimingMeter  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the tray: smooth steel ------------------------------------------------------------------------------------------
TRAY_X, TRAY_Y, TRAY_T = (0.16, 0.50), (-0.30, 0.24), 0.004
TT = TOP + TRAY_T

# -- the bag: hanging vertically, with its ports downward ------------------------------------------------------------
BAG_AT = np.array([0.39, -0.122, 0.99])
RX90 = (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))  # local +y points up; local -y/ports point down
BAG_R0 = rotation(RX90)
BAG_W, BAG_L, BAG_H = 0.13, 0.20, 0.026  # 13 x 20 x 2.6 cm external pillow; bounding volume 676 mL
BAG_MASS = 0.53  # 500 mL saline plus the PVC bag and ports
PORT_Z = 0.013  # the ports' axis, above the tray
FIN, FIN_Y = (0.028, 0.008, 0.020), BAG_L / 2 - 0.012  # the stiff hanger tab, folded up on the bag's far end
PORT_R, PORT_Y = 0.005, (-0.100, -0.130)  # the spike port: radius, from the bag's end to its mouth (bag frame)
MOUTH = np.array([0.0, PORT_Y[1], PORT_Z])  # the spike port's mouth, in the bag's frame
CAP_L, CAP_R = 0.015, 0.0065
CAP_LOCAL = np.array([0.0, PORT_Y[1] - CAP_L / 2, PORT_Z])
CAP_AT = BAG_AT + BAG_R0 @ CAP_LOCAL
CAP_BREAK = 3.5  # N: the cap's seal tears at this pull

# -- the giving set: upright in a molded sterile presentation rack --------------------------------------------------
SET_AT = np.array([0.36, -0.30, TT + 0.080])  # upright chamber in a reachable presentation rack
SET_R0_XYZW = RX90
SET_R0 = BAG_R0  # local +y/spike axis -> world +z
CH_R, CH_L = 0.010, 0.045  # the drip chamber
SPIKE_TIP = 0.0655  # the spike's tip, ahead of the chamber's middle (set frame, +y)
SEAT = 0.025  # the spike in this deep: its collar on the port's mouth, locked
K_SPIKE, SLIP_S, D_SPIKE = 1500.0, 0.001, 3.0  # the port's grip on the spike: about 1.5 N to push it further
CAPTURE_R, CAPTURE_DEG = 0.007, 12.0
SPIKE_CAP_L, SPIKE_CAP_R = 0.018, 0.005
SPIKE_CAP_AT = SET_AT + SET_R0 @ np.array([0.0, SPIKE_TIP - SPIKE_CAP_L / 2, 0.0])
SPIKE_CAP_BREAK = 0.08  # N: low-force pull-off protective cap
SPIKE_CAP_OFF = 0.006
CHAMBER_VERTICAL_DEG = 12.0
RACK_CLIP_BREAK = 8.0  # N: the rack's spring clip lets the set go above this pull (the set weighs 0.2 N)

# -- priming: measured finger compression produces a continuous liquid-column height ---------------------------------
SEGMENTS = 6
CH_USABLE_H = CH_L - 0.006
SQUEEZE_AT = 0.04  # the TCP this close to the chamber's middle

# -- scoring ------------------------------------------------------------------------------------------------------------
CAP_OFF = 0.005
LET_GO = 0.04
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0

STEEL, BAGC, SEAM, WHITE, BLUE, NAVY, CLEAR, FLUID, DARK, TUBE = (0.62, 0.64, 0.66), (0.86, 0.90, 0.94), \
    (0.74, 0.79, 0.85), (0.96, 0.96, 0.96), (0.10, 0.35, 0.85), (0.10, 0.15, 0.40), (0.85, 0.88, 0.92), \
    (0.35, 0.65, 0.95), (0.12, 0.12, 0.13), (0.78, 0.86, 0.92)
HIDE = (0.32, 0.40, 0.30)  # under the table


def _tray() -> Body:
    return Body.build("tray", pos=((TRAY_X[0] + TRAY_X[1]) / 2, (TRAY_Y[0] + TRAY_Y[1]) / 2, TOP), mass=2.0,
                      shapes=[BoxShape((0.0, 0.0, TRAY_T / 2), (TRAY_X[1] - TRAY_X[0], TRAY_Y[1] - TRAY_Y[0], TRAY_T),
                                       STEEL)],
                      kinematic=True, friction=(0.05, 0.04), slick=True, color=STEEL)


def _iv_support() -> Body:
    """A fixed IV-pole hook and backplate supporting the bag through its hanger opening."""
    return Body.build("iv_support", pos=(BAG_AT[0], BAG_AT[1], TOP), mass=4.0, kinematic=True, color=STEEL,
                      shapes=[CylShape((0.075, 0.0, 0.30), 0.009, 0.60, STEEL, axis="Z"),
                              BoxShape((0.0, 0.004, 0.33), (0.16, 0.014, 0.030), STEEL),
                              CylShape((0.0, 0.004, 0.33), 0.004, 0.030, DARK, axis="Y", collide=False)])


def _bag() -> Body:
    """A filled bag hanging from the support; local -y is world-down."""
    shapes = [BoxShape((0.0, 0.0, BAG_H / 2), (BAG_W - 0.012, BAG_L - 0.015, BAG_H), BAGC),  # the filled bag
              BoxShape((0.0, 0.0, BAG_H / 2), (BAG_W, BAG_L, 0.003), SEAM),  # its welded rim
              BoxShape((0.0, FIN_Y, BAG_H + FIN[2] / 2), FIN, SEAM),  # the stiff hanger tab, folded up
              CylShape((0.0, FIN_Y, BAG_H + FIN[2] * 0.6), 0.004, FIN[1] + 0.0006, DARK, axis="Y", collide=False),
              BoxShape((0.0, 0.012, BAG_H + 0.0002), (0.062, 0.08, 0.0004), WHITE, collide=False),  # the label
              BoxShape((0.0, 0.036, BAG_H + 0.0004), (0.050, 0.010, 0.0004), BLUE, collide=False),
              BoxShape((0.0, 0.016, BAG_H + 0.0004), (0.040, 0.004, 0.0004), NAVY, collide=False),
              BoxShape((0.0, 0.004, BAG_H + 0.0004), (0.046, 0.003, 0.0004), NAVY, collide=False),
              BoxShape((0.0, -0.008, BAG_H + 0.0004), (0.030, 0.003, 0.0004), NAVY, collide=False),
              BoxShape((0.0, PORT_Y[0] + 0.004, PORT_Z), (0.05, 0.010, 0.006), SEAM),  # the port block
              CylShape((0.0, (PORT_Y[0] + PORT_Y[1]) / 2, PORT_Z), PORT_R, PORT_Y[0] - PORT_Y[1], WHITE, axis="Y"),
              CylShape((0.0, PORT_Y[1] + 0.0003, PORT_Z), 0.0033, 0.0008, DARK, axis="Y", collide=False),  # its bore
              CylShape((0.024, PORT_Y[0] - 0.009, PORT_Z), 0.0045, 0.018, WHITE, axis="Y"),  # the medication port
              CylShape((0.024, PORT_Y[0] - 0.020, PORT_Z), 0.0055, 0.006, (0.85, 0.85, 0.80), axis="Y")]
    return Body.build("bag", pos=tuple(BAG_AT), mass=BAG_MASS, shapes=shapes, friction=(1.0, 0.9),
                      color=BAGC, com=(0.0, 0.0, BAG_H / 2), start_xyzw=RX90, kinematic=True)


def _cap() -> Body:
    """The pull-off cap on the spike port, with a tab to pull it by; origin on its axis, in its middle."""
    return Body.build("cap", pos=tuple(CAP_AT.tolist()), mass=0.006, friction=(1.0, 0.9), color=BLUE,
                      start_xyzw=RX90, shapes=[
        CylShape((0.0, 0.0, 0.0), CAP_R, CAP_L, BLUE, axis="Y"),
        # Flexible pull strip projects 5-115 mm away from the bag face after the body is rotated vertical.
        BoxShape((0.0, -0.002, 0.060), (0.012, 0.012, 0.110), BLUE)])


def _spike_cap() -> Body:
    """Protective cap over the sterile giving-set spike, with an off-axis pull tab."""
    return Body.build("spike_cap", pos=tuple(SPIKE_CAP_AT.tolist()), mass=0.003, friction=(1.0, 0.9), color=NAVY,
                      start_xyzw=SET_R0_XYZW,
                      shapes=[CylShape((0.0, 0.0, 0.0), SPIKE_CAP_R, SPIKE_CAP_L, NAVY, axis="Y"),
                              # A long molded tab keeps parallel-gripper pads clear of the sterile spike axis.
                              BoxShape((0.0, 0.0, 0.028), (0.012, 0.012, 0.040), NAVY)])


def _sterile_cradle() -> Body:
    """Fixed molded rack that presents the capped chamber upright without touching its spike."""
    return Body.build("sterile_cradle", pos=(SET_AT[0], SET_AT[1], TOP), mass=0.15, kinematic=True,
                      friction=(0.8, 0.7), color=WHITE,
                      shapes=[BoxShape((0.0, 0.0, 0.004), (0.080, 0.080, 0.008), WHITE),
                              BoxShape((0.0, 0.025, 0.032), (0.025, 0.012, 0.056), WHITE),
                              BoxShape((0.0, 0.014, 0.060), (0.025, 0.026, 0.006), WHITE)])


def _giving_set() -> Body:
    """The giving set: spike, drip chamber, a short length of tubing with its roller clamp. Origin in the drip
    chamber's middle; the spike points +y. The chamber is clear (its wall is not drawn): the saline shows in it."""
    shapes = [CylShape((0.0, 0.0, 0.0), CH_R, CH_L, None, axis="Y"),  # the chamber (clear)
              # Molded wings give the parallel pads a symmetric purchase while staying
              # inside the cap-removal approach envelope.  Their depth and height resist
              # pitch during the required horizontal-to-vertical transfer.
              BoxShape((0.0, 0.0, 0.0), (0.040, 0.020, 0.024), CLEAR),
              # Upper/lower handling flanges form a shallow waist around the pads.  They
              # provide geometric pitch and slip resistance during the vertical carry.
              BoxShape((0.0, -0.014, 0.0), (0.050, 0.004, 0.035), CLEAR),
              BoxShape((0.0, 0.014, 0.0), (0.050, 0.004, 0.035), CLEAR),
              CylShape((0.0, CH_L / 2 - 0.0015, 0.0), CH_R + 0.0003, 0.003, CLEAR, axis="Y", collide=False),
              CylShape((0.0, -CH_L / 2 + 0.0015, 0.0), CH_R + 0.0003, 0.003, CLEAR, axis="Y", collide=False),
              CylShape((0.0, 0.030, 0.0), 0.007, 0.015, WHITE, axis="Y"),  # the spike's base
              CylShape((0.0, 0.039, 0.0), 0.0085, 0.003, WHITE, axis="Y"),  # its collar
              CylShape((0.0, 0.0505, 0.0), 0.0032, 0.020, WHITE, axis="Y"),  # the spike
              CylShape((0.0, SPIKE_TIP - 0.0025, 0.0), 0.0018, 0.005, WHITE, axis="Y"),
              CylShape((0.0, -0.030, 0.0), 0.004, 0.015, WHITE, axis="Y"),  # the outlet
              # Flexible tubing is routed sideways in the presentation rack instead of acting as a rigid rod
              # beneath the vertical chamber.  This segment remains part of the carried rigid proxy.
              CylShape((0.0, -0.040, 0.040), 0.0022, 0.080, TUBE, axis="Z", collide=False),
              BoxShape((0.0, -0.040, 0.082), (0.014, 0.025, 0.012), BLUE, collide=False)]  # roller clamp
    for x, z in ((CH_R, 0.0), (-CH_R, 0.0), (0.0, CH_R), (0.0, -CH_R)):  # faint seams down the clear chamber
        shapes.append(BoxShape((x * 0.98, 0.0, z * 0.98), (0.0008 if x else 0.0012, CH_L - 0.004, 0.0012 if x else 0.0008),
                               CLEAR, collide=False))
    for k in range(1, 4):  # 25%, 50% and 75% liquid-height marks
        shapes.append(CylShape((0.0, -CH_L / 2 + 0.003 + k * CH_USABLE_H / 4, 0.0), CH_R + 0.0003, 0.0006,
                               DARK, axis="Y", collide=False))
    # The carried rigid proxy is the 20 g chamber/spike assembly; flexible tubing is visual only.
    return Body.build("giving_set", pos=tuple(SET_AT.tolist()), mass=0.020, shapes=shapes, friction=(8.0, 6.0),
                      color=WHITE, com=(0.0, 0.0, 0.0), start_xyzw=SET_R0_XYZW, ignore=("bag",))


def _fluid(k: int) -> Body:
    """One visual segment of the measured liquid column."""
    return Body.build(f"saline_{k}", pos=(HIDE[0], HIDE[1] + 0.03 * k, HIDE[2]), mass=0.001, kinematic=True,
                      color=FLUID, shapes=[CylShape((0.0, 0.0, 0.0), CH_R - 0.0006, CH_USABLE_H / SEGMENTS, FLUID,
                                                    axis="Y", collide=False)])


OBJECTS = [_tray(), _iv_support(), _sterile_cradle(), _bag(), _cap(), _giving_set(), _spike_cap(),
           *[_fluid(k) for k in range(1, SEGMENTS + 1)]]
_SPEC = {o.name: o for o in OBJECTS}
# The rack's spring clip: holds the set in place against its weight and the spike cap's pull, and lets go for good
# once the set is pulled out of it with more than RACK_CLIP_BREAK (re-armed at every reset).
hinge("rack_clip", "sterile_cradle", "giving_set", anchor=tuple(SET_AT.tolist()), axis="Z", limits_deg=(-0.5, 0.5),
      break_force=RACK_CLIP_BREAK)

# Nonsterile surfaces an exposed sterile interface must never touch, besides the grippers and the tray: every
# collidable shape of the rack and the IV support, and of the bag's outside except its spike port itself.
_PORT_SHAPE_Y = (PORT_Y[0] + PORT_Y[1]) / 2
FIXED_OBSTACLES = (obstacles_from_body(_SPEC["sterile_cradle"], (SET_AT[0], SET_AT[1], TOP), np.eye(3))
                   + obstacles_from_body(_SPEC["iv_support"], (BAG_AT[0], BAG_AT[1], TOP), np.eye(3)))


def _bag_obstacles(b_pos, b_rot) -> list:
    return obstacles_from_body(_SPEC["bag"], b_pos, b_rot,
                               skip=lambda s: isinstance(s, CylShape) and abs(s.centre[1] - _PORT_SHAPE_Y) < 1e-9
                               and abs(s.centre[0]) < 1e-9)
hinge("cap_seal", "bag", "cap", anchor=tuple(CAP_AT.tolist()), axis="Y", limits_deg=(-0.5, 0.5), break_force=CAP_BREAK)
hinge("spike_cap_seal", "giving_set", "spike_cap", anchor=tuple(SPIKE_CAP_AT.tolist()), axis="Y",
      limits_deg=(-0.5, 0.5), break_force=SPIKE_CAP_BREAK)
thread("spike_in", "bag", "giving_set", anchor0=tuple(MOUTH.tolist()), axis0=(0.0, 1.0, 0.0),
       anchor1=(0.0, SPIKE_TIP, 0.0), axis1=(0.0, 1.0, 0.0), slide_limits=(-0.05, SEAT), turn_damping=0.002,
       slide_stiffness=K_SPIKE, slide_damping=D_SPIKE, slide_max_force=50.0, play=0.001, tilt_deg=3.0)


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"on": False, "hold": 0.0, "locked": False, "cap_off": False, "spike_cap_off": False,
            "meter": PrimingMeter(usable_height=CH_USABLE_H, closed_finger=-0.020),
            "asepsis": AsepsisMonitor(), "tcps": None, "s": {}}


_EP = _fresh()
_ARM_IDS: dict = {}
_DEBUG = {"on": bool(os.environ.get("IVBAG_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0


def _hands(base, origin) -> tuple[list[np.ndarray], list[float], list[tuple[np.ndarray, float]]]:
    """Measure TCPs and finger joints; cache link indices for mesh contact checks."""
    tcps, fingers, surfaces = [], [], []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        if arm not in _ARM_IDS:
            names, joints = list(robot.body_names), list(robot.joint_names)
            palm = next(i for i, n in enumerate(names) if n == "gripper" or n.endswith("/gripper"))
            tips = [i for i, n in enumerate(names)
                    if n in ("tip_left", "tip_right") or n.endswith("/tip_left") or n.endswith("/tip_right")]
            if len(tips) != 2:
                raise RuntimeError(f"expected two YAM fingertip bodies, got {names}")
            _ARM_IDS[arm] = (palm, tips, joints.index("joint7"))
        body_i, tip_ids, finger_j = _ARM_IDS[arm]
        poses = as_numpy(robot.data.body_link_pose_w)[0]
        g = poses[body_i]
        tcps.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
        fingers.append(float(as_numpy(robot.data.joint_pos)[0][finger_j]))
    return tcps, fingers, surfaces


def _quat_xyzw(m: np.ndarray) -> list[float]:
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    if w > 1e-6:
        return [(m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w), (m[1, 0] - m[0, 1]) / (4 * w), w]
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(max(0.0, 1.0 + m[i, i] - m[j, j] - m[k, k])) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[3] = (m[k, j] - m[j, k]) / s
    q[i] = s / 4
    q[j] = (m[j, i] + m[i, j]) / s
    q[k] = (m[k, i] + m[i, k]) / s
    return q


def _place(base, origin, name, pos, rot) -> None:
    import torch

    asset = base.scene[name]
    p = np.asarray(pos, float) + origin
    asset.write_root_pose_to_sim(torch.tensor([[*p.tolist(), *_quat_xyzw(np.asarray(rot, float))]],
                                              dtype=torch.float32, device=asset.device))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    b = as_numpy(base.scene["bag"].data.root_link_pose_w)[0]
    g = as_numpy(base.scene["giving_set"].data.root_link_pose_w)[0]
    c = as_numpy(base.scene["cap"].data.root_link_pose_w)[0]
    sc = as_numpy(base.scene["spike_cap"].data.root_link_pose_w)[0]
    b_pos, b_rot, g_pos, g_rot, c_pos, sc_pos = (b[:3] - origin, rotation(b[3:7]), g[:3] - origin,
                                                 rotation(g[3:7]), c[:3] - origin, sc[:3] - origin)
    seat_c = b_pos + b_rot @ CAP_LOCAL
    if not _EP["cap_off"] and float(np.linalg.norm(c_pos - seat_c)) > CAP_OFF:
        _EP["cap_off"] = True
    spike_cap_seat = g_pos + g_rot @ np.array([0.0, SPIKE_TIP - SPIKE_CAP_L / 2, 0.0])
    if not _EP["spike_cap_off"] and float(np.linalg.norm(sc_pos - spike_cap_seat)) > SPIKE_CAP_OFF:
        _EP["spike_cap_off"] = True
    q = b_rot.T @ (g_pos + g_rot @ np.array([0.0, SPIKE_TIP, 0.0]) - b_pos) - MOUTH  # spike tip from mouth
    off, depth = math.hypot(q[0], q[2]), float(q[1])
    angle = math.degrees(math.acos(max(-1.0, min(1.0, float((b_rot[:, 1]) @ (g_rot[:, 1]))))))
    if not _EP["on"]:
        if (_EP["cap_off"] and _EP["spike_cap_off"] and off <= CAPTURE_R and 0.0 <= depth <= 0.012
                and angle <= CAPTURE_DEG):  # in the port
            set_slide_target("spike_in", depth)
            set_thread_on("spike_in", True)
            _EP["on"], _EP["hold"] = True, depth
    elif not _EP["locked"]:
        if depth < -0.003:  # pulled back out
            set_thread_on("spike_in", False)
            _EP["on"] = False
        elif depth >= SEAT - 0.002:  # all the way in: it locks there
            _EP["locked"] = True
            set_slide_target("spike_in", SEAT)
        elif abs(depth - _EP["hold"]) > SLIP_S:  # pushed (or pulled) past the port's grip: it slides, and stays
            _EP["hold"] = depth
            set_slide_target("spike_in", depth)
    tcps, fingers, gripper_surfaces = _hands(base, origin)
    _EP["tcps"] = tcps
    near = float(np.linalg.norm(tcps[1] - g_pos)) <= SQUEEZE_AT
    mouth_w = b_pos + b_rot @ MOUTH
    chamber_low = float((g_pos + g_rot @ np.array([0.0, -CH_L / 2 + 0.003, 0.0]))[2])
    vertical_deg = math.degrees(math.acos(float(np.clip(g_rot[2, 1], -1.0, 1.0))))
    orientation_ok = vertical_deg <= CHAMBER_VERTICAL_DEG
    head_ok = mouth_w[2] - chamber_low >= 0.010  # measured liquid region is >=10 mm below the wetted port
    meter = _EP["meter"]
    meter.observe(fingers[1], near=near and head_ok and orientation_ok, locked=_EP["locked"])
    spike_base = g_pos + g_rot @ np.array([0.0, 0.039, 0.0])
    spike_tip = g_pos + g_rot @ np.array([0.0, SPIKE_TIP, 0.0])
    from gripper_geometry import link_contact
    contact = False
    if not _EP['locked']:
        for arm in ('left_robot', 'right_robot'):
            robot = base.scene[arm]
            poses = as_numpy(robot.data.body_link_pose_w)[0]
            palm, tips, _ = _ARM_IDS[arm]
            for i in [palm, *tips]:
                pose = poses[i].copy()
                pose[:3] -= origin
                name = robot.body_names[i].split('/')[-1]
                contact |= (_EP['cap_off'] and link_contact(name, pose, mouth_w, mouth_w, PORT_R))
                contact |= (_EP['spike_cap_off'] and link_contact(name, pose, spike_base, spike_tip, .0032))
    _EP["asepsis"].observe(port_open=_EP["cap_off"], spike_exposed=_EP["spike_cap_off"],
                           spike_locked=_EP["locked"], port_center=mouth_w, port_radius=PORT_R,
                           spike_base=spike_base, spike_tip=spike_tip, spike_radius=0.0032,
                           gripper_surfaces=(), gripper_contact=contact, tray_top=TT,
                           obstacles=FIXED_OBSTACLES + _bag_obstacles(b_pos, b_rot))
    for k in range(1, SEGMENTS + 1):  # measured saline column, from the outlet end upward
        name = f"saline_{k}"
        seg = CH_USABLE_H / SEGMENTS
        if (k - 0.5) * seg <= meter.fill_height and g_rot[2, 1] > 0.2:
            outlet = g_pos + g_rot @ np.array([0.0, -CH_L / 2 + 0.003, 0.0])
            # The displayed free surface is horizontal. Layer elevation is measured
            # along world gravity, while its centre follows the chamber centreline.
            center = outlet + g_rot[:, 1] * ((k - 0.5) * seg / g_rot[2, 1])
            _place(base, origin, name, center, SET_R0)
        else:
            _place(base, origin, name, (HIDE[0], HIDE[1] + 0.03 * k, HIDE[2]), np.eye(3))
    clear = all(min(float(np.linalg.norm(t - g_pos)), float(np.linalg.norm(t - mouth_w))) >= LET_GO for t in tcps)
    bag_hanging = (float(np.linalg.norm(b_pos - BAG_AT)) < 0.005
                   and math.degrees(math.acos(min(1.0, float(b_rot[:, 1] @ np.array([0.0, 0.0, 1.0]))))) < 3.0)
    _EP["s"] = {"clear": clear, "bag_hanging": bag_hanging, "orientation_ok": orientation_ok,
                "vertical_deg": vertical_deg}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            robot = base.scene['right_robot']
            print('[ivtracking] q=' + str(np.round(as_numpy(robot.data.joint_pos)[0], 3).tolist())
                  + ' target=' + str(np.round(as_numpy(robot.data.joint_pos_target)[0], 3).tolist()), flush=True)
            print(f"[ivbag] t={_DEBUG['steps'] / 20:5.1f}s cap_off={_EP['cap_off']} depth={depth * 1000:6.1f}mm "
                  f"q={np.round(q * 1000, 1).tolist()} "
                  f"off={off * 1000:4.1f}mm angle={angle:4.1f} on={_EP['on']} locked={_EP['locked']} "
                  f"fill_height={meter.fill_height * 1000:.1f}mm spill={meter.spilled} fingerR={fingers[1]:.4f} "
                  f"near={near} head_ok={head_ok} vertical={vertical_deg:.1f}deg clear={clear} "
                  f"spike_cap_off={_EP['spike_cap_off']} contaminated={_EP['asepsis'].contaminated} "
                  f"cause={_EP['asepsis'].cause} "
                  f"bag={np.round(b_pos * 1000).astype(int).tolist()} set={np.round(g_pos * 1000).astype(int).tolist()} "
                  f"leftTCP-bag={np.round((tcps[0] - b_pos) * 1000).astype(int).tolist()} "
                  f"rightTCP-set={np.round((tcps[1] - g_pos) * 1000).astype(int).tolist()} "
                  f"rightTCP-cap={np.round((tcps[1] - c_pos) * 1000).astype(int).tolist()} score={_points(_status())}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    st = _EP["s"]
    meter = _EP["meter"]
    return {"cap_off": _EP["cap_off"], "spike_cap_off": _EP["spike_cap_off"], "locked": _EP["locked"],
            "primed": meter.primed, "contaminated": _EP["asepsis"].contaminated,
            "fill_height": meter.fill_height, "spilled": meter.spilled,
            "orientation_ok": bool(st.get("orientation_ok")), "clear": bool(st.get("clear")),
            "bag_hanging": bool(st.get("bag_hanging")), "home": _arms_home()}


def _points(s) -> float:
    if s["spilled"] or s["contaminated"]:
        return 0.0
    if (s["cap_off"] and s["spike_cap_off"] and s["locked"] and s["primed"] and s["orientation_ok"]
            and s["clear"] and s["bag_hanging"] and s["home"]):
        return 1.0
    if s["cap_off"] and s["spike_cap_off"] and s["locked"]:
        return 0.5
    return 0.25 if s["cap_off"] and s["spike_cap_off"] else 0.0


def score(objects) -> float:
    """1.0 with the spike locked all the way into the port, the measured liquid column 15-27 mm high,
    the bag hanging vertically and both hands let go (each TCP at least 4 cm from
    the drip chamber and the port) and both arms back where they started (each TCP within 5 cm of its start); 0.5
    with the spike locked in the port; 0.25 with the cap pulled off; else 0.0. The spike can only go in once the cap
    is off, and once locked it stays locked."""
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
    """Full score with the bag and the giving set still."""
    return (score(objects) >= 1.0 and objects["bag"].resting() and objects["giving_set"].resting())


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right arm uncaps, carries, seats and primes; the left stays clear.

X_GRIP = grip_rotation(math.pi / 2)  # straight down, fingers closing along x (right arm)
_tilt = math.radians(35)
SIDE_GRIP = np.array([[1,0,0],[0,math.cos(_tilt),-math.sin(_tilt)],[0,math.sin(_tilt),math.cos(_tilt)]]) @ X_GRIP
CHAMBER_UP = 0.006  # the right TCP this far above the drip chamber's axis
PUSH_PAST = 0.025  # aim past the seat (the hand lags)
VIA_RISE, VIA_OVER = 0.12, np.array([0.0, 0.04, 0.035])  # m: the high crossing from the dropped cap to the spike cap
DROP_AT = np.array([0.24, 0.18, TT + 0.05])
SPIKE_CAP_DROP = np.array([0.47, 0.05, TT + 0.05])  # forward: never crosses the exposed spike or chamber
LEAD_S, LEAD_STEPS = 3.0, 15
SET_OFF = np.array([-0.009, 0.0, -0.003])  # where the spike's tip ends up off the port's mouth if aimed straight at it
GRASP_DELTA = np.array([-0.022, 0.002, 0.004])  # measured TCP minus chamber centre during the tilted pickup
# (measured: the bag turns a little about the held tab as the cap is pulled, and the set hangs low in the grip)
_carry_trim = math.radians(4)
VERT_GRIP = np.array([[1,0,0],[0,math.cos(_carry_trim),-math.sin(_carry_trim)],
                     [0,math.sin(_carry_trim),math.cos(_carry_trim)]]) @ SIDE_GRIP


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    from scipy.optimize import least_squares
    def residual(q):
        p, r = ik.tcp_pose(q, station.BASES[arm])
        return np.r_[p - np.asarray(pos), 0.3 * ik._rot_error(rot, r)]
    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4), (0.0,)):
        result = least_squares(residual, np.clip(seed, ik.LOWER, ik.UPPER),
                               bounds=(ik.LOWER, ik.UPPER), max_nfev=80)
        q = result.x
        err = residual(q)
        pe, re = np.linalg.norm(err[:3]), np.linalg.norm(err[3:]) / 0.3
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


def _cart_path(p0, p1, seconds: float, label: str, finger, rot, steps: int = 20) -> list[Waypoint]:
    """Dense straight Cartesian path with a fixed wrist attitude."""
    a, b = np.asarray(p0, float), np.asarray(p1, float)
    return [_w(seconds / steps, a + (b - a) * k / steps, finger, label, rot)
            for k in range(1, steps + 1)]


def _left(release_at: float, total: float) -> tuple[list[Waypoint], float]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES["left"])
    return [_w(max(total, 1.0), pos, FINGER_OPEN, "left arm remains clear of the sterile field", rot)], 0.0


def _right(start: float) -> tuple[list[Waypoint], float]:
    tab = CAP_AT + [0.0, -0.075, 0.004]
    out = _lead_in("right", tab + [0.0, 0.0, 0.045], X_GRIP,
                   "approach the blue cap's pull tab without crossing the sterile tray", FINGER_OPEN)
    out += [_w(0.8, tab, FINGER_OPEN, "lower around the blue pull tab", X_GRIP),
            _w(0.6, tab, FINGER_CLOSED, "grip the blue pull tab", X_GRIP),
            _w(1.5, tab + [0.0, 0.0, -0.050], FINGER_CLOSED, "pull the cap straight down off the port", X_GRIP),
            _w(0.4, tab + [0.0, 0.0, -0.050], FINGER_OPEN, "release the cap below the sterile port", X_GRIP)]
    spike_cap_tab = SPIKE_CAP_AT + SET_R0 @ np.array([0.0, 0.0, 0.038])
    cap_drop_q = _good_ik("right", tab + [0.0, 0.0, -0.050], X_GRIP)
    spike_above = spike_cap_tab + [0.0, 0.0, 0.015]
    spike_above_q = _good_ik("right", spike_above, SIDE_GRIP)
    spike_tab_q = _good_ik("right", spike_cap_tab, SIDE_GRIP)
    # Rise, cross high and come down onto the spike cap: a direct joint-space swing from the dropped cap would carry
    # the open fingers through the capped spike beside the rack.
    rise_q = _good_ik("right", tab + [0.0, 0.0, -0.050 + VIA_RISE], X_GRIP)
    over_q = _good_ik("right", spike_above + VIA_OVER, SIDE_GRIP)
    out += _joint_path("right", cap_drop_q, rise_q, "rise clear of the sterile field", FINGER_OPEN)
    out += _joint_path("right", rise_q, over_q, "cross high over the rack to the spike cap", FINGER_OPEN)
    out += _joint_path("right", over_q, spike_above_q, "come straight down above the spike cap", FINGER_OPEN)
    out += _joint_path("right", spike_above_q, spike_tab_q, "down round the spike-cap tab", FINGER_OPEN)
    out += [
            _w(2.0, spike_cap_tab, FINGER_CLOSED, "close fully and settle on the spike-cap tab", SIDE_GRIP),
            _w(1.5, spike_cap_tab + [0.0, 0.0, 0.012], FINGER_CLOSED,
               "pull the protective cap upward along the spike axis", SIDE_GRIP),
            _w(0.4, spike_cap_tab + [0.0, 0.0, 0.012], FINGER_OPEN,
               "release the spike cap above its presentation rack", SIDE_GRIP)]
    grip = SET_AT + [0.0, 0.0, CHAMBER_UP]
    mouth = BAG_AT + BAG_R0 @ MOUTH
    align_set = np.array([mouth[0], mouth[1] - 0.006, mouth[2] - SPIKE_TIP + 0.004])
    seated_set = np.array([mouth[0], mouth[1] - 0.006, mouth[2] - SPIKE_TIP + SEAT + 0.008])
    below_tcp = align_set + GRASP_DELTA + [0.0, 0.0, -0.008]
    align_tcp, seated_tcp = align_set + GRASP_DELTA, seated_set + GRASP_DELTA
    low = np.array([0.40, -0.25, below_tcp[2]])
    grip_above = grip + [0.0, -0.055, 0.04]
    spike_drop_q = spike_tab_q  # the 30 mm cap pull is axial; seed the next path from its reachable tab solution
    grip_above_q = _good_ik("right", grip_above, SIDE_GRIP)
    grip_q = _good_ik("right", grip, SIDE_GRIP)
    out += _joint_path("right", spike_drop_q, grip_above_q, "move around to the drip chamber", FINGER_OPEN)
    out += _joint_path("right", grip_above_q, grip_q, "down round the drip chamber", FINGER_OPEN)
    out += [
            _w(2.5, grip, FINGER_CLOSED, "close fully and settle on the drip chamber", SIDE_GRIP),
            _w(2.0, grip + [0.0, 0.0, 0.035], FINGER_CLOSED, "lift it from the sterile rack", SIDE_GRIP),
            _w(2.0, np.array([0.40, -0.25, grip[2] + 0.035]), FINGER_CLOSED,
               "move the capped end outward before descending", VERT_GRIP),
            _w(3.0, low, FINGER_CLOSED, "lower the upright set below the hanging bag", VERT_GRIP)]
    q_low = _good_ik("right", low, VERT_GRIP)
    side_tcp = np.array([0.42, below_tcp[1], below_tcp[2]])
    q_side = _good_ik("right", side_tcp, VERT_GRIP)
    q_below = _good_ik("right", below_tcp, VERT_GRIP)
    q_align = _good_ik("right", align_tcp, VERT_GRIP)
    q_seated = _good_ik("right", seated_tcp, VERT_GRIP)
    out += _joint_path("right", q_low, q_side, "match the port line outside the bag footprint", FINGER_CLOSED)
    out += _joint_path("right", q_side, q_below, "translate inward beneath the port", FINGER_CLOSED)
    out += _joint_path("right", q_below, q_align, "align the upward spike with the port mouth", FINGER_CLOSED)
    out += _joint_path("right", q_align, q_seated, "push the spike upward to its collar: it locks", FINGER_CLOSED)
    out += [_w(3.0, seated_tcp, FINGER_CLOSED, "hold the seated spike while the port grip settles", VERT_GRIP),
            _w(0.5, seated_tcp, FINGER_OPEN, "release the insertion grasp", VERT_GRIP)]
    chamber = np.array([mouth[0], mouth[1], mouth[2] - SPIKE_TIP + SEAT])
    out.append(_w(0.6, chamber, FINGER_OPEN, "round the upright drip chamber", VERT_GRIP))
    for k in (1, 2):
        out += [_w(0.6, chamber, FINGER_CLOSED, f"squeeze the vertical drip chamber ({k})", VERT_GRIP),
                _w(0.3, chamber, FINGER_CLOSED, "hold the squeeze", VERT_GRIP),
                _w(0.7, chamber, FINGER_OPEN, "release: saline rises from the outlet end", VERT_GRIP)]
    up = chamber + [0.0, -0.080, 0.025]
    out.append(_w(0.8, up, FINGER_OPEN, "rise clear", VERT_GRIP))
    done = sum(w.duration for w in out)
    out += _go_home("right", up, VERT_GRIP, FINGER_OPEN)
    return out, done


def demo():
    _, held_at = _left(0.0, 0.0)
    right, done = _right(held_at)
    total = sum(w.duration for w in right) + 3.0
    right.append(_w(3.0, right[-1].pos, FINGER_OPEN, "wait at the start", right[-1].yaw))
    left, _ = _left(done + 0.3, total)
    return {"left": left, "right": right}
