"""On the station table, a grey anti-static mat (28 x 36 cm) with, in the middle, a green circuit board (8 x 7 cm,
1.6 mm thick) lying solder side up on two black rails, so a blue capacitor hangs under it. One of the capacitor's
leads (silver, 2 mm square) stands 14 mm up through the board at its centre. Right of the board, on the mat, lies a
pair of flush-cutting nippers (red and black handles, 19.2 cm long), pointing forward and to the robot's left. On the board's
left, on the mat, stands a small brown paper scrap cup (5.7 cm across, 6 cm tall, white inside).

The task: clip the lead flush to the board with the nippers, and put the offcut in the scrap cup.

``bonds.py`` puts the lead's two pieces and the nippers' moving jaw on joints, ``compound.py`` builds the nippers from
several box colliders, ``simready.py`` loads the scrap cup, ``contacts.py`` reads which robot links touch the offcut, and
``looks.py`` draws the board and the nippers.

Run it:  inspect-robots run --embodiment yam_arena -E task=clip_lead_catch_offcut.py --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import contacts  # noqa: E402
from bonds import Bonded, Sliding, on_pre_step, on_reset, on_spawn, on_step, release  # noqa: E402
from compound import compound  # noqa: E402
from looks import dress  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402
from yam_arena.urdf import quat_xyzw_to_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # success() itself needs the full-success state held for HOLD_S: the run ends when it first does
MM = 0.001

# -- the mat, the board and its rails -------------------------------------------------------------------------------
MAT_XY, MAT_SIZE = (0.34, -0.06), (0.28, 0.36, 0.002)
MAT_TOP = TOP + MAT_SIZE[2]
MAT = Box("mat", pos=(*MAT_XY, TOP + MAT_SIZE[2] / 2), size=MAT_SIZE, color=(0.16, 0.19, 0.2))

LEAD_XY = (0.36, 0.0)  # the lead's axis, and the board's centre
BOARD_SIZE = (0.08, 0.07, 0.0016)
RAIL = (0.08, 0.008, 0.012)  # on the mat, under the board's long edges
BOARD_BOTTOM = MAT_TOP + RAIL[2]
Z_BT = BOARD_BOTTOM + BOARD_SIZE[2]  # the board's top, where the lead leaves it
BOARD = Box("board", pos=(*LEAD_XY, BOARD_BOTTOM + BOARD_SIZE[2] / 2), size=BOARD_SIZE, color=(0.08, 0.36, 0.17))
RAILS = [Box(f"rail_{k}", pos=(LEAD_XY[0], LEAD_XY[1] + s * (BOARD_SIZE[1] / 2 - RAIL[1] / 2), MAT_TOP + RAIL[2] / 2),
             size=RAIL, color=(0.08, 0.08, 0.09)) for k, s in (("a", 1), ("b", -1))]
CAP_SIZE = (0.012, 0.012, 0.010)  # the capacitor hanging under the board
CAP = Box("capacitor", pos=(*LEAD_XY, BOARD_BOTTOM - CAP_SIZE[2] / 2 - 0.0005), size=CAP_SIZE, color=(0.1, 0.2, 0.7))

# -- the lead: a 2 mm square stub, and two pieces on break points 0.8 and 4 mm above the board ------------------------
# The lead is clipped at one of its two break points: the lower piece's joint to the stub lets go (flush: a 0.8 mm stub
# stays), or the upper piece's joint to the lower one (a 4 mm stub stays). Neither joint breaks under load: the clip
# is the nippers closing on the lead, read from their poses (see ``cut_class``).
LEAD_W = 0.002
STUB_H, MID_H, LEAD_H = 0.0008, 0.004, 0.014  # heights above the board of the stub's top, the break points, the tip
LEAD_STUB = Box("lead_stub", pos=(*LEAD_XY, Z_BT + STUB_H / 2), size=(LEAD_W, LEAD_W, STUB_H), color=(0.8, 0.8, 0.84))
LEAD_LOW = Bonded("lead_low", pos=(*LEAD_XY, Z_BT + (STUB_H + MID_H) / 2), size=(LEAD_W, LEAD_W, MID_H - STUB_H),
                  mass=0.0006, color=(0.8, 0.8, 0.84))
LEAD_TOP = Bonded("lead_top", pos=(*LEAD_XY, Z_BT + (MID_H + LEAD_H) / 2), size=(LEAD_W, LEAD_W, LEAD_H - MID_H),
                  mass=0.0012, color=(0.9, 0.9, 0.93), to="lead_low")
BASE_POINT = np.array([*LEAD_XY, Z_BT])  # where the lead leaves the board

# -- the nippers: two halves on a sliding joint, pointing forward and to the robot's left ----------------------------
# Cutter frame: x across (the squeeze), y along the jaws (the jaws at +y), z up, origin on the bottom plane of the
# fixed half. The fixed half has a handle plate at -x and the anvil, a flat 0.8 mm foot that rests on the board. The
# moving half has a handle plate at +x and the blade, a foot 0.3 mm above the anvil. The blade slides toward -x, up to
# 10 mm, held open against its stop by a 25 N detent: a grip of under 25 N holds the nippers without closing them.
AIM = math.radians(35.0)  # the jaws point this far to the left of forward
PHI = AIM - math.pi / 2  # the cutter's yaw
CX, SX = math.cos(PHI), math.sin(PHI)


def rot(v, yaw=PHI):
    """The cutter-frame (u, v) as a world (x, y) offset."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


PLATE_H = 16 * MM
PLATE_Y = (-170 * MM, -8 * MM)  # the handle plates' ends (the grip is 16.5 cm behind the lead)
FOOT_Y = (-12 * MM, 22 * MM)
ANVIL_TOP, BLADE_BOTTOM, BLADE_TOP = 0.8 * MM, 1.1 * MM, 3.4 * MM
ANVIL_X = -1 * MM  # the anvil's working face (cutter frame)
BLADE_X = 9 * MM  # the blade's working face, open: a 10 mm throat
PLATE_F_X = (-22 * MM, -12 * MM)
PLATE_M_X = (12 * MM, 22 * MM)
TRAVEL = 10 * MM
GRIP_Y = -150 * MM  # where the grip centres, along the plates
GRIP_Z = 16 * MM  # the TCP, above the plates' bottom: the pad tips 3 mm up, on 13 mm of the plates
HOLD_SQUEEZE = 38 * MM  # the pads' gap for a firm hold: 3 mm in on each plate (15 N)
LEAD_X_AIM, LEAD_Y_AIM = 4 * MM, 15 * MM  # where the lead sits between the jaws, cutter frame


def _box(x, y, z):
    """(centre, size) of a box from its (lo, hi) in x, y, z, in the cutter frame."""
    return tuple((a + b) / 2 for (a, b) in (x, y, z)), tuple(b - a for (a, b) in (x, y, z))


F_X, F_Y, F_Z = (PLATE_F_X[0], ANVIL_X), (PLATE_Y[0], FOOT_Y[1]), (0.0, PLATE_H)
M_X, M_Y, M_Z = (BLADE_X, PLATE_M_X[1]), (PLATE_Y[0], FOOT_Y[1]), (0.0, PLATE_H)
C_F, SIZE_F = _box(F_X, F_Y, F_Z)  # the fixed half's box: centre in the cutter frame, size
C_M, SIZE_M = _box(M_X, M_Y, M_Z)


def _parts(c, boxes):
    """Collider parts as (name, centre relative to the half's box, size)."""
    out = []
    for name, (x, y, z) in boxes.items():
        centre, size = _box(x, y, z)
        out.append((name, tuple(centre[i] - c[i] for i in range(3)), size))
    return out


PARTS_F = _parts(C_F, {"plate": (PLATE_F_X, PLATE_Y, (0.0, PLATE_H)), "anvil": ((PLATE_F_X[0], ANVIL_X), FOOT_Y, (0.0, ANVIL_TOP))})
PARTS_M = _parts(C_M, {"plate": (PLATE_M_X, PLATE_Y, (0.0, PLATE_H)), "blade": ((BLADE_X, PLATE_M_X[1]), FOOT_Y, (BLADE_BOTTOM, BLADE_TOP))})

# Final pose: the lead between the jaws. Start: lying on the mat to the right, the same turn.
CUT_O = np.array(LEAD_XY) - rot((LEAD_X_AIM, LEAD_Y_AIM))  # the cutter frame's origin when the lead is at its aim
START_SHIFT = (0.10, -0.02)  # the start's origin, from CUT_O, in the cutter frame
START_O = CUT_O + rot(START_SHIFT)


def _place(origin, z0, c):
    p = np.asarray(origin) + rot(c[:2])
    return (float(p[0]), float(p[1]), float(z0 + c[2]))


CUTTER = compound(
    Box("cutter", pos=_place(START_O, MAT_TOP, C_F), size=SIZE_F, yaw=PHI, mass=0.03, color=(0.12, 0.12, 0.13)), PARTS_F)
JAW = compound(
    Sliding("jaw", pos=_place(START_O, MAT_TOP, C_M), size=SIZE_M, yaw=PHI, mass=0.02, color=(0.75, 0.08, 0.06),
            to="cutter", axis="X", lower=-TRAVEL, upper=0.0, rate=1500.0, damping=40.0, target=25.0 / 1500.0), PARTS_M)

# -- the scrap cup: a SimReady paper cup, fixed, on the mat beside the board ------------------------------------------
CUP_XY = (0.36, 0.07)
CUP_SCALE = 0.55  # the 10.4 x 10.4 x 11.1 cm model to 5.7 x 5.7 x 6.1 cm
CUP_H = 0.1107253 * CUP_SCALE
CUP = SimReady("cup", pos=(*CUP_XY, MAT_TOP), size=(0.057, 0.057, CUP_H), package="consumable_cup_paper_a01",
               usd="sm_consumable_cup_paper_a01_01.usd", scale=CUP_SCALE)
CUP_RIM = MAT_TOP + CUP_H
CUP_IN_R = 0.02  # m: the offcut's centre this close to the cup's axis, below its rim, is in the cup

OBJECTS = [MAT, BOARD, *RAILS, CAP, LEAD_STUB, LEAD_LOW, LEAD_TOP, CUTTER, JAW, CUP]

# What the boxes look like: meshes modelled in Blender (looks/make_looks.py), the colliders unchanged. The mat, the
# rails, the lead's pieces and the cup stay as they are.
dress({"board": "board.usdc", "capacitor": "capacitor.usdc", "cutter": "cutter_f.usdc", "jaw": "cutter_m.usdc"})

# -- scoring ---------------------------------------------------------------------------------------------------------
FLUSH_H = 1.4 * MM  # the cutting plane this close to the board makes a flush cut
HIGH_MAX = 8 * MM  # up to this high the nippers can still clip the lead (at its 4 mm break point)
LEVEL_DEG = 10.0  # the nippers' jaws within this many degrees of level
SQUEEZED = 3 * MM  # the blade has moved this far from open
TOUCH = 0.6 * MM  # the blade's face this close to the lead's face is on it
FLY_DIST = 0.10  # m: the offcut this far from where the lead left the board, and not held, has flown
MAT_REST = 0.007  # m: the offcut's centre this low over the mat lies on it
KICK = 3.0  # m/s: the clipped lead's snap
HOLD_S = 1.0  # s: full success needs the offcut at rest in the cup, let go, without a break this long

SPIN_DAMPING = 30.0  # 1/s: the lead's pieces are 2 mm sticks that PhysX would let spin on for ever where they rest


@on_spawn
def _damp(name: str, prim) -> None:
    """Damp the spin of the lead's pieces (a stick resting in the cup would otherwise turn on at about 1 rad/s)."""
    if name not in ("lead_low", "lead_top"):
        return
    from pxr import Sdf

    if "PhysxRigidBodyAPI" not in prim.GetAppliedSchemas():
        prim.AddAppliedSchema("PhysxRigidBodyAPI")
    prim.CreateAttribute("physxRigidBody:angularDamping", Sdf.ValueTypeNames.Float).Set(SPIN_DAMPING)


# What the episode remembers: how the lead was clipped ("flush", "high" or ""), whether the offcut ever went 10 cm from
# the board without being held (free flight), the snap waiting to be given to the offcut at the next control step, and
# for the control step that just ended whether a gripper held the offcut (both finger pads on it, in some physics
# substep) and whether any robot link touched it, and since when (s, ``None`` while not) the offcut has lain at rest in the
# cup, let go, after a flush clip and no free flight, without a break: any lapse restarts it.
STATE = {"cut": "", "flew": False, "kick": None, "held": False, "touch": False, "met": None, "t": 0.0}


@on_reset
def reset_memory(env=None) -> None:
    STATE.update(cut="", flew=False, kick=None, held=False, touch=False, met=None, t=0.0)
    contacts.take()


contacts.watch("lead_top")


def pinched(links) -> bool:
    """Whether a gripper holds the offcut: two robot links of one arm, the two finger pads, touch it (a separation of
    1 mm or less, the contact the simulator reports) in the same control step."""
    arms = [contacts.arm_of(link) for link in links]
    return any(arms.count(arm) >= 2 for arm in ("left", "right"))


def _finite(objects) -> bool:
    return all(np.all(np.isfinite(objects[n].pos)) and np.all(np.isfinite(objects[n].quat_xyzw)) for n in NAMES)


NAMES = ("cutter", "jaw", "lead_low", "lead_top")


def _rot(state) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float))


def cutter_frame(objects):
    """The cutter frame's origin and rotation, from the fixed half's pose."""
    f = objects["cutter"]
    r = _rot(f)
    return np.asarray(f.pos) - r @ np.array(C_F), r


def blade_travel(objects) -> float:
    """How far the moving half has closed (m, 0 open, negative closing), along the fixed half's x axis."""
    d = _rot(objects["cutter"]).T @ (np.asarray(objects["jaw"].pos) - np.asarray(objects["cutter"].pos))
    return float(d[0] - (C_M[0] - C_F[0]))


def cut_class(objects) -> str:
    """How the nippers are closed on the lead right now: "flush", "high" or "" (not on the lead).
    On the lead: jaws within 10 degrees of level, the blade moved 3 mm or more, its face within 0.6 mm of the lead's
    face, and the lead between the anvil's face (its own face not past it by more than 0.5 mm) and the blade, inside
    the jaws' length (4 mm or more from the root, 2 mm from the tips). The cutting plane,
    the anvil's top, is within 1.4 mm of the board (flush) or up to 8 mm."""
    origin, r = cutter_frame(objects)
    if math.degrees(math.acos(np.clip(r[2, 2], -1.0, 1.0))) > LEVEL_DEG:
        return ""
    travel = blade_travel(objects)
    x, y, _ = r.T @ (BASE_POINT - origin)
    gap = (BLADE_X + travel) - (x + LEAD_W / 2)
    on_lead = (travel <= -SQUEEZED and -2 * MM <= gap <= TOUCH and x - LEAD_W / 2 >= ANVIL_X - 0.5 * MM
               and FOOT_Y[0] + 4 * MM <= y <= FOOT_Y[1] - 2 * MM)
    if not on_lead:
        return ""
    plane = float(origin[2] + ANVIL_TOP - Z_BT)
    return "flush" if plane <= FLUSH_H else "high" if plane <= HIGH_MAX else ""


def kick_velocity(objects) -> np.ndarray:
    """The snap: forward along the jaws, and up."""
    r = _rot(objects["cutter"])
    d = 0.6 * r[:, 1] + np.array([0.0, 0.0, 0.8])
    return KICK * d / np.linalg.norm(d)


def offcut_distance(objects) -> float:
    """How far the offcut's top piece is from where the lead left the board (m)."""
    return float(np.linalg.norm(np.asarray(objects["lead_top"].pos) - BASE_POINT))


def observe(objects, dt: float = 0.05, links=()) -> None:
    """One control step of the episode's memory: the clip, the snap it gives the offcut, and the offcut's flight.
    ``links`` are the robot links that touched the offcut during the step (see ``contacts.py``). The offcut flies when
    it is more than 10 cm from the board and no gripper holds it. A held offcut carried higher or further is not
    flying."""
    if not _finite(objects):
        return
    STATE["held"], STATE["touch"] = pinched(links), bool(links)
    if not STATE["cut"]:
        STATE["cut"] = cut_class(objects)
        if STATE["cut"]:
            names = ["lead_low", "lead_top"] if STATE["cut"] == "flush" else ["lead_top"]
            STATE["kick"] = (names, kick_velocity(objects))
    elif offcut_distance(objects) > FLY_DIST and not STATE["held"]:
        STATE["flew"] = True
    STATE["t"] += dt
    if not conditions_met(objects):
        STATE["met"] = None
    elif STATE["met"] is None:
        STATE["met"] = STATE["t"]


@on_step
def watch(env) -> None:
    """Every control step: the poses and the offcut's contacts into the episode's memory, and the joint that was
    clipped let go."""
    from yam_arena.scene import object_states

    observe(object_states(env, NAMES), float(env.unwrapped.step_dt), contacts.take())
    if STATE["kick"] is not None:
        release(STATE["kick"][0][0])


@on_pre_step
def snap(env) -> None:
    """Before the next control step: the offcut's snap. It is given as a velocity, and the grippers' contacts, or
    nothing, then act on it."""
    if STATE["kick"] is None:
        return
    import torch
    from yam_arena.scene import object_states

    names, velocity = STATE["kick"]
    STATE["kick"] = None
    states = object_states(env, names)
    base = env.unwrapped
    for name in names:
        s = states[name]
        v = torch.tensor([[*(np.asarray(s.lin_vel) + velocity), *s.ang_vel]], dtype=torch.float32, device=base.device)
        base.scene[name].write_root_velocity_to_sim_index(root_velocity=v)


def in_cup(state) -> bool:
    p = np.asarray(state.pos)
    return bool(np.linalg.norm(p[:2] - CUP_XY) <= CUP_IN_R and p[2] < CUP_RIM - 0.005)


def on_mat(state) -> bool:
    p = np.asarray(state.pos)
    inside = all(abs(p[i] - MAT_XY[i]) <= MAT_SIZE[i] / 2 for i in range(2))
    return bool(inside and p[2] < MAT_TOP + MAT_REST)


def free(objects) -> bool:
    """The offcut is in free flight now: more than 10 cm from the board and no gripper holds it."""
    return offcut_distance(objects) > FLY_DIST and not STATE["held"]


def conditions_met(objects) -> bool:
    """Every full-success condition, as things stand: clipped flush, the offcut never in free flight beyond 10 cm, and
    it lies still in the scrap cup, let go (no robot link touches it)."""
    if not _finite(objects) or STATE["cut"] != "flush" or STATE["flew"] or free(objects):
        return False
    top = objects["lead_top"]
    return bool(in_cup(top) and top.resting() and not STATE["touch"])


def held_s() -> float:
    """How long every full-success condition has held without a break (s), 0 while one fails."""
    return 0.0 if STATE["met"] is None else STATE["t"] - STATE["met"]


def success(objects) -> bool:
    """Every full-success condition (``conditions_met``) holds now and has held without a break for ``HOLD_S`` (1 s),
    counted in control steps from the first step it held."""
    return bool(conditions_met(objects) and STATE["met"] is not None and held_s() >= HOLD_S - 1e-6)


def score(objects) -> float:
    """1.0: success (the offcut at rest in the cup, let go, for 1 s). 0.75: clipped flush, the offcut lies still on the
    mat, let go, or still in the cup, let go, for less than 1 s. 0.5: clipped flush, the offcut anywhere else (in a
    gripper, on the board, moving). 0.25: clipped high (a 4 mm stub). 0: not clipped, or
    the offcut was ever in free flight (more than 10 cm from the board, no gripper holding it)."""
    if not _finite(objects) or not STATE["cut"] or STATE["flew"] or free(objects):
        return 0.0
    if STATE["cut"] != "flush":
        return 0.25
    top = objects["lead_top"]
    if top.resting() and not STATE["touch"]:
        if in_cup(top):
            return 1.0 if success(objects) else 0.75
        if on_mat(top):
            return 0.75
    return 0.5


# -- scripted demo ---------------------------------------------------------------------------------------------------
PSI_R = AIM  # the right grip closes along the cutter's x axis
PSI_L = AIM  # the left grip closes across the jaws, like the right one: its wide fingers then don't point at the right gripper
HOLD_OFFCUT = True  # False: the left arm stays back (the one-handed shortcut: the offcut flies, scores 0)
CUT_HIGH = 0.0  # m: the nippers are held this much above the board (a high cut, with 6 mm: scores 0.25)
PAD_GAP0 = 1.6 * MM  # the pads' gap with the fingers closed: the gap is this plus twice the finger value
HOLD_F = -(HOLD_SQUEEZE - PAD_GAP0) / 2
L_OPEN = -0.012  # the left grip's opening for the descent: 2.4 cm
SLIDE = 0.04  # m: the nippers slide on 4 cm along the jaws to the lead
BACK = 0.06
GRASP_Z = Z_BT + 0.008 + 0.013  # the left pads' tips 8 mm above the board, over the lead's upper piece
CARRY_UP = 0.07  # m: the left hand lifts the held offcut this far over where it grasped it (past 10 cm from the board)
LEAN = 0.0015  # m: the right TCP is sent this far below the nippers resting on the board
CLEAR = 0.0015  # m: the nippers' feet slide this far above the board
TRIM = (0.0035, 0.0)  # m: the held nippers are aimed this far on (cutter frame x, y): in the hand they sit that far off (tuned)


def _tcp(origin, z0, along=0.0):
    """The right TCP gripping the nippers whose frame origin is ``origin`` (on the plane ``z0``), ``along`` further along
    the jaws."""
    p = np.asarray(origin) + rot((0.0, GRIP_Y + along))
    return (float(p[0]), float(p[1]), float(z0 + GRIP_Z))


def demo():
    w = Waypoint
    lift = 0.04
    o_cut = CUT_O + rot(TRIM)
    o_a = o_cut - rot((0.0, SLIDE))  # the approach: the lead 4 cm ahead of the jaws' aim
    z_slide = Z_BT + CLEAR + CUT_HIGH
    right = [
        w(2.0, _tcp(START_O, MAT_TOP + lift + 0.03), FINGER_OPEN, "over the nippers", PSI_R),
        w(1.2, _tcp(START_O, MAT_TOP), FINGER_OPEN, "down beside the plates", PSI_R),
        w(0.8, _tcp(START_O, MAT_TOP), HOLD_F, "grip the plates", PSI_R),
        w(0.8, _tcp(START_O, MAT_TOP + lift), HOLD_F, "lift", PSI_R),
        w(2.2, _tcp(o_a, z_slide + lift), HOLD_F, "over the board", PSI_R),
        w(1.0, _tcp(o_a, z_slide), HOLD_F, "down to the board", PSI_R),
        w(2.0, _tcp(o_cut, z_slide), HOLD_F, "slide the jaws round the lead", PSI_R),
        w(0.8, _tcp(o_cut, z_slide), HOLD_F, "let the arm settle", PSI_R),
        w(0.6, _tcp(o_cut, z_slide - CLEAR - LEAN), HOLD_F, "rest on the board", PSI_R),
        w(1.2, _tcp(o_cut, z_slide - CLEAR - LEAN), FINGER_CLOSED, "squeeze: clip", PSI_R),
        w(0.8, _tcp(o_cut, z_slide - CLEAR - LEAN), HOLD_F, "let the jaws open", PSI_R),
        w(1.5, _tcp(o_cut, z_slide, -BACK), HOLD_F, "back off along the jaws", PSI_R),
        w(1.2, _tcp(o_cut, z_slide + lift, -BACK), HOLD_F, "lift clear", PSI_R),
    ]
    t_clip = sum(s.duration for s in right[:10])  # the squeeze is over
    t_clear = sum(s.duration for s in right[:11])  # the jaws are open again
    lx, ly = LEAD_XY
    cx, cy = CUP_XY
    carry_z = Z_BT + 0.008 + 0.013 + CARRY_UP  # the pads' tips well above the cup's rim: the offcut goes past 10 cm, held
    drop_z = CUP_RIM + 0.012 + 0.013 + 0.0
    t_ready = 2.5 + 1.5 + 0.8
    left = [
        w(2.5, (lx, ly, GRASP_Z + 0.05), L_OPEN, "over the lead's top", PSI_L),
        w(1.5, (lx, ly, GRASP_Z), L_OPEN, "down round it", PSI_L),
        w(0.8, (lx, ly, GRASP_Z), FINGER_CLOSED, "hold the lead's top", PSI_L),
        w(t_clear + 0.5 - t_ready, (lx, ly, GRASP_Z), FINGER_CLOSED, "hold through the clip", PSI_L),
        w(1.2, (lx, ly, carry_z), FINGER_CLOSED, "lift the offcut", PSI_L),
        w(1.5, (cx, cy, carry_z), FINGER_CLOSED, "over the cup", PSI_L),
        w(1.0, (cx, cy, drop_z), FINGER_CLOSED, "down", PSI_L),
        w(0.6, (cx, cy, drop_z), FINGER_OPEN, "let go", PSI_L),
        w(1.0, (cx, cy, carry_z), FINGER_OPEN, "retreat", PSI_L),
    ]
    assert t_clear + 0.5 > t_ready and t_clip < t_clear
    if not HOLD_OFFCUT:
        left = [w(left[0].duration, left[0].pos, FINGER_OPEN, "stay back", PSI_L)]
    return {"right": right, "left": left}
