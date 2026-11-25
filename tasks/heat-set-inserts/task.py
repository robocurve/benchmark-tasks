"""Install two M3 heat-set inserts in a 3D-printed robot link plate: melt each in with a soldering iron, then press it
flush and square with a flat steel block while it is still soft, and lay the hot iron back on its stand.

On the station table, in front of the arms, lies a 3D-printed PLA robot link plate (30 x 60 x 10 mm) with two 4.0 mm
pilot holes, 30 mm apart; in each hole's mouth sits a brass M3 heat-set insert (4.4 mm across, 5.7 mm long), set in by
0.8 mm as one does before heating it. To the right, a pen-style soldering iron with an M3 heat-set insert tip (set to
225 C for PLA, as CNC Kitchen advises) lies on its stand; to the left stands a steel 1-2-3 block. Everything stands
on a silicone soldering mat covering the work area. The robot
must do what CNC Kitchen's guide says: melt each insert in with the iron to about 90 % of its depth, keeping the iron
square to the plate and pressing lightly, then press the last part with a flat tool (here the 1-2-3 block) so the
insert ends flush and square; and finally lay the iron back on its stand.

What makes it hard: the iron must stay vertical while it sinks the insert (a tilted iron sets the insert crooked)
with only light pressure, and stop short of flush, since the shoulder of the tip melts the plastic round the hole if
it goes past; the iron has to stay on each insert for a few seconds to heat it before it gives; the block must reach
the insert while it is still soft (a few seconds after the iron leaves) and come down flat on the plate, which is what
makes the insert flush; and the iron's hot metal must never touch the robot or the work surface.

Simulation: the plate is a fixed-base articulation whose inserts each slide on a prismatic joint held stiffly at a
target depth. Each insert has a temperature: it heats only while the iron touches it and cools afterwards (faster under
the cold steel block); above T_FLOW (PLA melts) a push of at least F_MIN (iron) or F_MIN_BLOCK (block) moves the joint's
target on from where the insert actually is, at most V_MAX: the melting PLA is a rule, not simulated. Scoring reads each
insert's measured joint position. The insert's tilt is the iron's tilt while the iron sank it.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from scenekit import Shape, UsdBody, on_reset, on_step  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = ("This 3D-printed robot part needs metal threads for the M3 screws that will hold it. Install the two "
               "brass heat-set inserts sitting loose in its holes: melt each one most of the way in with the hot "
               "soldering iron, keeping the iron straight, then press it flush with the steel block while the "
               "plastic is still soft. Lay the iron back on its stand when you are done.")

MM = 0.001
TOP = TABLE_TOP[SCENE]

# -- the plate and inserts (as work/heatset/make_parts.py builds them)
PLATE_SIZE = (30 * MM, 60 * MM, 10 * MM)
PLATE_XY = (0.33, -0.03)
HOLES = {"a": (0.0, -15 * MM), "b": (0.0, 15 * MM)}
INSERT_D, INSERT_L, SET_IN = 4.4 * MM, 5.7 * MM, 0.8 * MM
FLUSH = INSERT_L - SET_IN  # how far each insert must go down from where it sits to end flush (4.9 mm)
INSERT_K, INSERT_DAMP = 60000.0, 30.0  # N/m, N s/m: how stiffly an insert holds its depth (5 N moves it 0.08 mm)
SURFACE = TOP + PLATE_SIZE[2]
PLATE = UsdBody("plate", usd="assets/plate.usdc", pos=(PLATE_XY[0], PLATE_XY[1], TOP), size=PLATE_SIZE, mass=None,
                articulation=True, material=False, solver_iters=32, joint_stiffness=INSERT_K,
                joint_damping=INSERT_DAMP, joint_pos=(("insert_a_joint", 0.0), ("insert_b_joint", 0.0)))


def hole_xy(name: str) -> np.ndarray:
    return np.array(PLATE_XY) + np.array(HOLES[name])


# -- the iron: origin at the pilot pin's end, the iron along its +z; the shoulder's face 3 mm up from the origin
PIN_L, SHOULDER_D = 3.0 * MM, 5.0 * MM
GRIP_FROM, GRIP_L, GRIP_D = (3 + 8 + 60) * MM, 105 * MM, 15 * MM
IRON_MASS = 0.045
# the iron rests on its stand lying along x, tip towards the robot, on two supports: one under the shaft and one under
# the grip, each a bar with a post either side that keeps it from rolling off
REST_Y, REST_TIP_X, REST_AXIS = -0.17, 0.20, TOP + 40 * MM  # the iron's tip and its axis' height when resting
SUPPORTS = {"front": (35 * MM, 3.0 * MM, 3.0 * MM), "rear": (160 * MM, 7.5 * MM, 7.5 * MM)}  # along from the tip, half-widths
IRON = UsdBody("iron", usd="assets/iron.usdc", pos=(REST_TIP_X, REST_Y, REST_AXIS + 0.3 * MM),
               rot_xyzw=(0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4)),  # its +z (tip to grip) along +x
               size=(GRIP_D, GRIP_D, 0.175), mass=IRON_MASS, friction=(0.7, 0.6), solver_iters=32,
               torsional_patch=0.004, report=True)
_hc = (0.25, 0.25, 0.27)
HOLDER = []
for _n, (_along, _half, _hgt) in SUPPORTS.items():
    _x = REST_TIP_X + _along
    _top = REST_AXIS - _hgt  # the support's top, under the shaft or the grip
    HOLDER.append(Shape(f"rest_{_n}_bar", pos=(_x, REST_Y, (TOP + _top) / 2), size=(0.010, 0.030, _top - TOP), color=_hc))
    for _sgn in (-1, 1):
        HOLDER.append(Shape(f"rest_{_n}_post_{'l' if _sgn > 0 else 'r'}",
                            pos=(_x, REST_Y + _sgn * (_half + 1.5 * MM + 1.0 * MM), (TOP + REST_AXIS + 2 * MM) / 2),
                            size=(0.010, 0.003, REST_AXIS + 2 * MM - TOP), color=_hc))

# -- the 1-2-3 block, standing on its 25.4 x 50.8 end, 50.8 along x
BLOCK_SIZE = (50.8 * MM, 25.4 * MM, 76.2 * MM)  # as placed (x, y, z)
BLOCK_XY = (0.25, 0.10)
BLOCK = UsdBody("block", usd="assets/block.usdc", pos=(BLOCK_XY[0], BLOCK_XY[1], TOP + BLOCK_SIZE[2] / 2 + 0.2 * MM),
                rot_xyzw=(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)), size=BLOCK_SIZE, mass=0.77,
                friction=(0.4, 0.35), solver_iters=32, report=True)

# a 1 mm silicone soldering mat covers the work area (heights are measured from its top); drawn, not simulated
MAT = Shape("silicone_mat", pos=(0.30, -0.04, TOP + 0.1 * MM - 0.5 * MM), size=(0.32, 0.40, 1.0 * MM),
            color=(0.42, 0.47, 0.52), collide=False)

OBJECTS = [MAT, PLATE, IRON, BLOCK, *HOLDER]

# -- the heating rule and scoring
# Each insert has a temperature. It heats only while the iron's tip touches it (towards the tip's 225 C) and cools
# afterwards, faster while the cold steel block presses on it (a heat sink). The PLA round it flows, so the insert can
# move, only above T_FLOW, and then only while it is pushed. Estimates (no public figures for these): from room
# temperature a pressed tip brings an M3 insert to T_FLOW in about 2.3 s; once the tip lifts it stays above T_FLOW for
# about 5 s on its own, about 1.1 s under the block.
T_AMB, T_IRON = 22.0, 225.0  # C: the room; the iron's set temperature (CNC Kitchen: 10-20 C above PLA's print temperature)
T_FLOW = 160.0  # C: PLA melts (NatureWorks Ingeo: 145-160 C): above this the insert moves under a push
TAU_HEAT = 2.0  # s: towards T_IRON while the tip touches it
TAU_COOL = 15.0  # s: towards T_AMB once the tip has lifted (brass in a pocket of PLA)
TAU_BLOCK = 4.0  # s: the extra cooling while the steel block's face presses on it
F_MIN = 1.0  # N: the iron sinks the insert only while pressed on it at least this hard
F_MIN_BLOCK = 0.3  # N: a soft insert gives under the block's face from this
V_MAX = 2.0 * MM  # m/s: and sinks at most this fast
LEAD = 0.2 * MM  # the joint's target never runs further than this ahead of where the insert actually is
TOUCH_N = 0.05  # N: a contact
FLUSH_TOL = 0.2 * MM  # flush: its top within this of the plate's surface (a 0.2 mm feeler gauge under a straightedge)
SQUARE_DEG = 3.0  # square: a screw in it within 3 degrees of perpendicular (1.6 mm over 30 mm)
HALF = FLUSH / 2
HOT_FRONT = GRIP_FROM  # the iron's bare metal (pin, shoulder, shaft): its first 71 mm from the tip
HOT_R = 3.0 * MM  # their largest radius (the shaft)


def new_insert() -> dict:
    return dict(depth=0.0, target=0.0, temp=T_AMB, tilt=0.0, w=0.0, marked=False, peak=T_AMB)


def new_state() -> dict:
    return dict(t=0.0, last=-1.0, ins={n: new_insert() for n in HOLES}, holder=True, burn=None, fmax=0.0, f={})


_STATE = new_state()


def melt(ins: dict, dt: float, depth: float, iron_n: float, block_n: float, iron_tilt: float) -> float:
    """One control step for one insert. ``depth``: how far it has gone down from where it sat, as its joint reports
    (m); iron_n / block_n: the iron's / block's force on it (N); iron_tilt: the iron's tilt (degrees). Updates its
    temperature and returns its joint's new target depth: it moves on only while it is soft and pushed, from where it
    actually is."""
    T = ins["temp"]
    if iron_n > TOUCH_N:
        T += (T_IRON - T) * (1.0 - math.exp(-dt / TAU_HEAT))
    else:
        rate = 1.0 / TAU_COOL + (1.0 / TAU_BLOCK if block_n > TOUCH_N else 0.0)
        T += (T_AMB - T) * (1.0 - math.exp(-dt * rate))
    ins["temp"], ins["peak"] = T, max(ins["peak"], T)
    moved = depth - ins["depth"]
    if moved > 0 and iron_n > TOUCH_N:  # sunk by the iron: it takes the iron's tilt (the block only pushes it along its hole)
        ins["tilt"] = (ins["tilt"] * ins["w"] + iron_tilt * moved) / (ins["w"] + moved)
        ins["w"] += moved
    ins["depth"] = depth
    pushed = iron_n >= F_MIN or (iron_n <= TOUCH_N and block_n >= F_MIN_BLOCK)
    if T >= T_FLOW and pushed:
        ins["target"] = min(max(ins["target"], depth) + V_MAX * dt, depth + LEAD, FLUSH + 1.5 * MM)
    return ins["target"]


def insert_score(ins: dict) -> float:
    """From its measured depth: 0.5 if flush (top within 0.2 mm of the surface) and square (within 3 degrees) with no
    melt mark round it; 0 if it is sunk more than 0.2 mm below the surface or not yet half way in; otherwise 0.25."""
    proud = FLUSH - ins["depth"]
    if proud < -FLUSH_TOL or ins["depth"] < HALF:
        return 0.0
    if abs(proud) <= FLUSH_TOL and abs(ins["tilt"]) <= SQUARE_DEG and not ins["marked"]:
        return 0.5
    return 0.25


def score(objects=None, st=None) -> float:
    """0 if the iron's hot metal ever touched a robot link or the work surface. Otherwise the two inserts' scores added
    (0.5 each at best); at most 0.75 until the iron lies back on its stand, let go."""
    st = _STATE if st is None else st
    if st["burn"]:
        return 0.0
    s = sum(insert_score(i) for i in st["ins"].values())
    return s if st["holder"] else min(s, 0.75)


def success(objects) -> bool:
    return score() == 1.0


@on_reset
def _reset(environment) -> None:
    _STATE.clear()
    _STATE.update(new_state())
    _INST.clear()


def _quat_R(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def tilt_deg(R, axis=2) -> float:
    """The angle of the body's ``axis`` from vertical (degrees)."""
    return float(math.degrees(math.acos(min(1.0, abs(float(np.asarray(R)[2, axis]))))))


def iron_in_holder(pos, R) -> bool:
    """The iron lying back on its stand: tip within 15 mm of its resting place, the iron within 10 degrees of level,
    its axis no more than 5 mm above its resting height."""
    tip = np.asarray(pos)
    level = 90.0 - tilt_deg(R)
    return bool(np.linalg.norm(tip[:2] - np.array([REST_TIP_X, REST_Y])) < 15 * MM and level < 10.0
                and tip[2] < REST_AXIS + 5 * MM)


_INST: dict = {}


def hot_points(tip, R, n: int = 15) -> np.ndarray:
    """Points along the iron's bare metal (its first 71 mm), station frame."""
    axis = np.asarray(R)[:, 2]
    return np.asarray(tip)[None] + np.linspace(0.0, HOT_FRONT, n)[:, None] * axis[None]


def on_work_surface(tip, R) -> bool:
    """The iron's bare metal within 1 mm of the work surface (the mat on the table)."""
    return bool((hot_points(tip, R)[:, 2] - HOT_R).min() <= TOP + 1.0 * MM)


def robot_on_hot_metal(environment, tip, R) -> bool:
    """A robot link (finger pads included) touching the iron's bare metal in the last physics step: a contact point
    less than 71 mm from the tip along the iron. (The iron is held by its grip, which starts there.)"""
    import contactkit
    import touch

    touch.touching(environment, ["iron"])  # builds the robot-links x iron contact view
    _, pts, _, sep, counts, starts = touch._VIEW["view"].get_contact_data(dt=environment.unwrapped.physics_dt)
    pts, sep = contactkit._np(pts).reshape(-1, 3), contactkit._np(sep).reshape(-1)
    counts, starts = contactkit._np(counts).reshape(-1), contactkit._np(starts).reshape(-1)
    base = environment.unwrapped
    origin = getattr(base.scene.env_origins, "torch", base.scene.env_origins)[0].detach().cpu().numpy()
    axis = np.asarray(R)[:, 2]
    for c, s0 in zip(counts, starts):
        for k in range(int(s0), int(s0 + c)):
            if sep[k] <= touch.TOUCH_SEP and float((pts[k] - origin - np.asarray(tip)) @ axis) < HOT_FRONT:
                return True
    return False


@on_step
def _watch(environment) -> None:
    import torch
    from yam_arena.scene import object_states

    import contactkit
    import touch

    dt = environment.unwrapped.step_dt
    plate = environment.unwrapped.scene["plate"]
    if not _INST:
        _INST["ids"] = plate.find_joints(["insert_a_joint", "insert_b_joint"], preserve_order=True)[0]
        _INST["dev"] = plate.device
    q = getattr(plate.data.joint_pos, "torch", plate.data.joint_pos)[0]  # the inserts' measured positions
    st = object_states(environment, ["iron", "block"])
    iron, block = st["iron"], st["block"]
    f = contactkit.forces(environment)
    _STATE["f"] = f
    iron_R = iron.rotation()
    iron_tilt, block_tilt = tilt_deg(iron_R), tilt_deg(block.rotation())
    targets = []
    for k, n in enumerate(("a", "b")):
        ins = _STATE["ins"][n]
        depth = -float(q[_INST["ids"][k]])
        if f[("iron", "plate")] > TOUCH_N and np.linalg.norm(np.asarray(iron.pos[:2]) - hole_xy(n)) < 6 * MM:
            ins["marked"] = True  # the tip's shoulder on the PLA round this hole: a melt mark
        targets.append(-melt(ins, dt, depth, f[("iron", f"insert_{n}")], f[("block", f"insert_{n}")], iron_tilt))
    plate.set_joint_position_target_index(target=torch.tensor([targets], device=_INST["dev"]), joint_ids=_INST["ids"])
    if not _STATE["burn"]:
        if robot_on_hot_metal(environment, iron.pos, iron_R):
            _STATE["burn"] = f"robot on hot metal at t={_STATE['t']:.2f}"
        elif on_work_surface(iron.pos, iron_R):
            _STATE["burn"] = f"hot metal on the work surface at t={_STATE['t']:.2f}"
    _STATE["holder"] = iron_in_holder(iron.pos, iron_R) and not touch.touching(environment, ["iron"])["iron"]
    _STATE["fmax"] = max([_STATE["fmax"]] + [v for k, v in f.items() if k[0] == "iron" and k[1] != "plate"])
    _STATE["t"] += dt
    if os.environ.get("HEATSET_LOG") and _STATE["t"] - _STATE["last"] >= 0.5:
        _STATE["last"] = _STATE["t"]
        a, b = _STATE["ins"]["a"], _STATE["ins"]["b"]
        print(f"LOG t={_STATE['t']:.2f} a: depth={a['depth'] * 1e3:.2f} tgt={a['target'] * 1e3:.2f}mm T={a['temp']:.0f}C "
              f"tilt={a['tilt']:.1f} mark={a['marked']} | b: depth={b['depth'] * 1e3:.2f} tgt={b['target'] * 1e3:.2f}mm "
              f"T={b['temp']:.0f}C tilt={b['tilt']:.1f} mark={b['marked']} | iron tilt={iron_tilt:.1f} "
              f"F_ia={f[('iron', 'insert_a')]:.2f} F_ib={f[('iron', 'insert_b')]:.2f} F_ip={f[('iron', 'plate')]:.2f} "
              f"F_ba={f[('block', 'insert_a')]:.2f} F_bb={f[('block', 'insert_b')]:.2f} F_bp={f[('block', 'plate')]:.2f} "
              f"block_tilt={block_tilt:.1f} block=({block.pos[0]:.3f},{block.pos[1]:.3f},{(block.pos[2] - TOP) * 1e3:.0f}mm) "
              f"holder={_STATE['holder']} burn={_STATE['burn']} iron_z={(iron.pos[2] - TOP) * 1e3:.1f}mm score={score():.2f}",
              flush=True)


# -- the scripted demo -----------------------------------------------------------------------------------------------
HOVER = TOP + 0.15
GRASP_H = 0.095  # the right hand holds the iron's grip this far above its tip (the grip runs from 71 to 176 mm)
PRESS_IN = float(os.environ.get("HEATSET_IRON_DEPTH", str(0.8 * FLUSH)))  # the iron takes each insert this far
# pushing a soft insert down (1 N) the arm holding the iron gives about 2.6 mm: the plan aims that far past where the
# insert should end (measured in the simulator: a plan 7.4 mm past the insert's top left it 4.8 mm in)
ARM_GIVE = float(os.environ.get("HEATSET_ARM_GIVE", "0.0026"))
PUSH = float(os.environ.get("HEATSET_PUSH", "0.0035"))  # how far past the insert the plan aims (the arm gives)
HEAT_S = float(os.environ.get("HEATSET_HEAT_S", "3.0"))  # the iron pressed on each insert this long before it can give
BLOCK_PUSH = 0.010
# where the iron's tip ends up relative to where the plan puts it, once held tip-down: it hangs about one tip's
# diameter towards -y of the planned point (seen in the close-ups), so the plan aims that far towards +y
AIM_OFFSET = np.array([float(os.environ.get("HEATSET_AIM_DX", "0.0")), float(os.environ.get("HEATSET_AIM_DY", "0.005")), 0.0])
BLOCK_GRASP = 15 * MM  # the left hand holds the block this far below its top
LOOSE = -(BLOCK_SIZE[1] / 2 + 1.0 * MM)  # finger setting 1 mm clear of the block's 25.4 mm faces each side


def iron_tcp(tip) -> tuple:
    """The right TCP holding the iron upright with its pilot pin's end at ``tip``."""
    p = np.asarray(tip, float) + np.array([0.0, 0.0, GRASP_H])
    return tuple(float(c) for c in p)


def block_tcp(xy, bottom_z) -> tuple:
    return (float(xy[0]), float(xy[1]), float(bottom_z + BLOCK_SIZE[2] - BLOCK_GRASP))


def demo():
    """Right arm picks the iron off its stand by the grip, lifts it clear and turns it tip-down; for each insert: down
    onto it, presses and holds 3 s while it heats, sinks it to 80 % of its depth at about 1 mm/s, lifts and moves off
    to the right; the left arm, holding the block by its top, follows it in about 2 s after the tip has left the insert:
    at each it sets the block down on the insert and opens its fingers 1 mm clear, so the block's own weight presses the
    insert in until the block sits flat on the plate; at the first it then takes the block again and lifts it away, at
    the second it leaves it there while the insert sets. Finally the iron is turned
    level and laid back on its stand."""
    from tiltkit import Tilt, grip, install
    from yam_arena.constants import FINGER_OPEN as O
    from yam_arena.demo import Waypoint as W

    install()
    side = grip(approach=(1, 0, 0), closing=(0, -1, 0))
    side_j = grip(approach=(1, 0, 0), closing=(0, -1, 0), joint=True)
    top = SURFACE + FLUSH  # each insert's top, as set (the shoulder's face meets it with the pin's end PIN_L lower)
    HOLD = 0.0
    gx = REST_TIP_X + GRASP_H  # the grip, GRASP_H along from the tip, lying on the stand
    flat = 0.0  # straight-down grip, fingers closing along y, across the lying grip
    right = [
        W(2.0, (0.20, -0.30, HOVER), O, "start", flat),
        W(2.0, (gx, REST_Y, REST_AXIS + 0.05), -0.015, "over the iron's grip", flat),
        W(1.5, (gx, REST_Y, REST_AXIS), -0.015, "down round the grip", flat),
        W(1.0, (gx, REST_Y, REST_AXIS), HOLD, "grip", flat),
        W(1.5, (gx, REST_Y, REST_AXIS + 0.08), HOLD, "lift off the stand", flat),
        W(1.5, (gx, REST_Y, REST_AXIS + 0.16), HOLD, "up clear", flat),
        W(3.0, (gx, REST_Y, REST_AXIS + 0.16), HOLD, "turn it tip-down", side_j),
    ]
    left = [W(2.0, (0.20, 0.30, HOVER), O, "start", 0.0)]
    bx, by = BLOCK_XY
    # held, the 0.77 kg block (its centre 23 mm below the TCP) sags the left wrist: the plan aims past the sag
    held = Tilt(None, load=(0.77, (0.0, 0.0, BLOCK_SIZE[2] / 2 - BLOCK_GRASP)))
    b0 = TOP + 0.2 * MM
    t_right = sum(w.duration for w in right)
    t_left = sum(w.duration for w in left)

    def wait_left(until):
        nonlocal t_left
        if until > t_left:
            left.append(W(until - t_left, left[-1].pos, left[-1].finger, "wait", left[-1].yaw))
            t_left = until

    def add(arm, w):
        nonlocal t_right, t_left
        (right if arm == "r" else left).append(w)
        if arm == "r":
            t_right += w.duration
        else:
            t_left += w.duration

    # the left hand takes the block early and holds it up, out of the way
    for w in [W(2.0, block_tcp((bx, by), b0 + 0.06), -0.025, "over the block", 0.0),
              W(1.5, block_tcp((bx, by), b0), -0.025, "down round the block", 0.0),
              W(1.0, block_tcp((bx, by), b0), HOLD, "grip the block", 0.0),
              W(1.5, block_tcp((bx, by), b0 + 0.06), HOLD, "lift the block", held)]:
        add("l", w)
    for n in ("a", "b"):
        h = hole_xy(n)
        tip_on = np.array([h[0], h[1], top - PIN_L]) + AIM_OFFSET
        add("r", W(2.5, iron_tcp(tip_on + [0, 0, 0.03]), HOLD, f"over insert {n}", side))
        add("r", W(1.5, iron_tcp(tip_on + [0, 0, 0.001]), HOLD, f"onto insert {n}", side))
        add("r", W(HEAT_S, iron_tcp(tip_on - [0, 0, PUSH]), HOLD, f"heat insert {n}: press and hold", side))
        add("r", W(PRESS_IN / (1.0 * MM), iron_tcp(tip_on - [0, 0, PRESS_IN + ARM_GIVE]), HOLD, f"melt insert {n} in", side))
        t_lift = t_right
        add("r", W(0.5, iron_tcp(tip_on - [0, 0, PRESS_IN] + [0, 0, 0.015]), HOLD, f"lift off insert {n}", side))
        aside = np.array([h[0] - 0.02, -0.12, top + 0.03])  # off to the right (-y), away from the block coming from +y
        add("r", W(0.8, iron_tcp(aside), HOLD, "aside", side))
        # the block waits beside the insert (+y), then follows the iron in as soon as it has lifted: the insert cools
        # from the moment the tip leaves it, and must still be soft when the block's face reaches it
        wait_left(t_lift - 2.0)
        add("l", W(2.0, block_tcp((h[0], h[1] + 0.045), SURFACE + 0.04), HOLD, f"block beside insert {n}", held))
        wait_left(t_lift + 0.7)
        add("l", W(0.8, block_tcp(h, SURFACE + 0.012), HOLD, f"block over insert {n}", held))
        add("l", W(0.4, block_tcp(h, SURFACE + (1 - PRESS_IN / FLUSH) * FLUSH + 0.0015), HOLD, f"block down to insert {n}", held))
        if n == "a":
            # set down on the soft insert, the fingers opened 1 mm clear of the block: its weight presses the insert in
            # until it sits flat on the plate; then the hand takes it again. Planned as an unloaded hand at the block's
            # own grip height (with the block let go, the arm no longer sags under it)
            add("l", W(0.4, block_tcp(h, SURFACE), HOLD, f"block onto insert {n}", held))
            add("l", W(0.3, block_tcp(h, SURFACE), LOOSE, f"fingers open: the block's weight presses insert {n} flush", 0.0))
            add("l", W(1.2, block_tcp(h, SURFACE), LOOSE, "it settles flat on the plate", 0.0))
            add("l", W(0.6, block_tcp(h, SURFACE), HOLD, "take the block again", 0.0))
            add("l", W(0.8, block_tcp(h, SURFACE + 0.03), HOLD, "lift the block", held))
            add("l", W(1.5, block_tcp((h[0] - 0.02, h[1] + 0.09), SURFACE + 0.05), HOLD, "block aside", held))
        else:
            # the last insert: the block is set down on it and let go, so its own weight (7.5 N) presses the insert in
            # until the block sits flat on the plate, however the hand held it, and then holds it while it sets
            add("l", W(0.4, block_tcp(h, SURFACE), HOLD, f"block onto insert {n}", held))
            add("l", W(0.5, block_tcp(h, SURFACE - 0.002), O, f"let go: the block's weight presses insert {n} flush", 0.0))
            add("l", W(1.0, block_tcp(h, SURFACE + 0.06), O, "up, clear of the block", 0.0))
        # the iron waits till the block is clear
        if t_left > t_right:
            right.append(W(t_left - t_right, right[-1].pos, HOLD, "wait", side))
            t_right = t_left
    add("r", W(2.5, (gx, REST_Y, REST_AXIS + 0.16), HOLD, "over the stand", side))
    add("r", W(3.0, (gx, REST_Y, REST_AXIS + 0.16), HOLD, "level again", Tilt(None, joint=True)))
    add("r", W(1.5, (gx, REST_Y, REST_AXIS + 0.05), HOLD, "down over the stand", flat))
    add("r", W(2.0, (gx, REST_Y, REST_AXIS + 0.001), HOLD, "lay it on the stand", flat))
    add("r", W(1.0, (gx, REST_Y, REST_AXIS + 0.001), -0.015, "let go", flat))
    add("r", W(1.0, (gx, REST_Y, REST_AXIS + 0.05), -0.015, "up", flat))
    add("r", W(2.0, (0.20, -0.30, HOVER), O, "back", flat))
    add("l", W(1.5, (0.20, 0.30, HOVER), O, "back", 0.0))
    return {"right": right, "left": left}


def self_test(ObjectState=None):
    """The heating rule and the scoring on hand-made pushes. The joint is taken to follow its target (as it does when
    nothing holds the insert back), and the block's face, resting on the plate, stops it at flush."""
    dt = 0.05

    def run(steps, ins=None, stuck=False, iron_to=0.8 * FLUSH):
        """steps: list of (iron force, block force, iron tilt); returns the insert. ``stuck``: the insert can't move
        (its measured depth stays put whatever the target). Like the demo's, the iron stops going down once the insert
        is ``iron_to`` in: from there it only rests on it (still touching, still heating it)."""
        ins = new_insert() if ins is None else ins
        for iron_n, block_n, it in steps:
            depth = ins["depth"] if stuck else min(ins["target"], FLUSH + 1.5 * MM)
            if iron_n > 0 and depth >= iron_to:
                iron_n = 0.2
            if block_n > 0 and depth >= FLUSH:
                block_n = 0.0  # the block's face is on the plate: it no longer bears on the flush insert
            melt(ins, dt, depth, iron_n, block_n, it)
        if not stuck:
            melt(ins, dt, min(ins["target"], FLUSH + 1.5 * MM), 0.0, 0.0, 0.0)  # settle: the measured depth catches up
        return ins

    def secs(x):
        return int(round(x / dt))

    press = (2.0, 0.0, 0.5)  # the iron pushing 2 N, upright
    idle = (0.0, 0.0, 0.0)
    block = (0.0, 3.0, 0.0)
    cases = 0
    # from cold the pressed iron needs about 2.3 s of contact before the insert gives
    ins = run([press] * secs(2.0))
    assert ins["depth"] == 0.0 and T_FLOW - 20 < ins["temp"] < T_FLOW, ins["temp"]
    ins = run([press] * secs(2.6))
    assert ins["depth"] > 0.0 and ins["temp"] >= T_FLOW
    # pressed on with nothing to stop it, the soft insert would sink past flush (the scorer's 0)
    ins = run([press] * secs(7.0), iron_to=1.0)
    assert ins["depth"] > FLUSH + FLUSH_TOL and insert_score(ins) == 0.0
    cases += 3
    # a brief touch (0.2 s) then the block: the insert never gets soft, the block can't push it (the review's case)
    ins = run([press] * secs(0.2) + [block] * secs(5.0))
    assert ins["depth"] == 0.0 and ins["peak"] < T_AMB + 30, (ins["depth"], ins["peak"])
    # nor several short touches with pauses (heat lost in between)
    ins = run(([press] * secs(0.4) + [idle] * secs(2.0)) * 5 + [block] * secs(3.0))
    assert ins["depth"] == 0.0 and ins["peak"] < T_FLOW, ins["peak"]
    cases += 2
    # the demo's way: 3 s pressed, then sunk to 80 % at 1 mm/s; the block arriving 2 s after the tip leaves pushes it
    # flush (measured), and no further
    heat = [press] * secs(3.0 + 0.8 * FLUSH / (1.0 * MM))
    ins = run(heat)
    assert ins["depth"] >= 0.8 * FLUSH and insert_score(ins) == 0.25, (ins["depth"], insert_score(ins))
    ins = run(heat + [idle] * secs(2.0) + [block] * secs(1.5))
    assert abs(ins["depth"] - FLUSH) <= FLUSH_TOL and insert_score(ins) == 0.5, (ins["depth"] * 1e3, ins["temp"])
    cases += 2
    # the block arriving too late (8 s after the tip left): it has set; the block can't move it
    ins = run(heat + [idle] * secs(8.0) + [block] * secs(2.0))
    assert ins["depth"] < 0.9 * FLUSH and insert_score(ins) == 0.25, ins["depth"]
    # the block cools it: pressing the soft insert only briefly leaves it short
    ins = run(heat + [idle] * secs(2.0) + [block] * secs(0.15))
    assert ins["depth"] < FLUSH - FLUSH_TOL
    cases += 2
    # scored from where the insert is, not from the joint's target: held back (stuck), the target runs at most LEAD ahead
    ins = run(heat, stuck=True)
    assert ins["depth"] == 0.0 and ins["target"] <= LEAD + 1e-12 and insert_score(ins) == 0.0
    ins = new_insert()
    ins.update(target=FLUSH, depth=0.8 * FLUSH)
    assert insert_score(ins) == 0.25  # a flush target does not score; the measured 80 % does
    cases += 2
    # light pushes do nothing even soft; a tilted iron sets it crooked and the block doesn't straighten it
    ins = new_insert()
    ins["temp"] = 200.0
    melt(ins, dt, 0.0, 0.5, 0.0, 0.0)
    assert ins["target"] == 0.0
    tilted = (2.0, 0.0, 8.0)
    ins = run([tilted] * secs(3.0 + 0.8 * FLUSH / (1.0 * MM)) + [idle] * secs(2.0) + [block] * secs(1.5))
    assert abs(ins["depth"] - FLUSH) <= FLUSH_TOL and ins["tilt"] > SQUARE_DEG and insert_score(ins) == 0.25
    cases += 2
    # a load resting on a cold insert (the 0.77 kg block, 7.5 N) moves its joint less than the flush tolerance
    assert 7.5 / INSERT_K < FLUSH_TOL
    cases += 1
    # sunk past flush: 0; marked round it: 0.25; not half way: 0
    ins = new_insert()
    ins.update(depth=FLUSH + 0.5 * MM)
    assert insert_score(ins) == 0.0
    ins.update(depth=FLUSH, tilt=0.0, marked=True)
    assert insert_score(ins) == 0.25
    ins.update(depth=0.3 * FLUSH, marked=False)
    assert insert_score(ins) == 0.0
    cases += 3
    # whole scores; the hot metal on a robot link or the work surface: 0
    st = new_state()
    for n in HOLES:
        st["ins"][n].update(depth=FLUSH, tilt=1.0)
    assert score(st=st) == 1.0
    st["holder"] = False
    assert score(st=st) == 0.75
    st["holder"] = True
    st["burn"] = "robot on hot metal"
    assert score(st=st) == 0.0
    st["burn"] = None
    st["ins"]["b"].update(depth=0.95 * FLUSH)
    assert score(st=st) == 0.75
    cases += 4
    # the hot metal's keep-out: lying on its stand, or upright on an insert, it is clear of the work surface; lying
    # on the mat it is not
    lying = _quat_R((0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4)))
    assert not on_work_surface((REST_TIP_X, REST_Y, REST_AXIS), lying)
    assert not on_work_surface((PLATE_XY[0], PLATE_XY[1], SURFACE - FLUSH), np.eye(3))
    assert on_work_surface((0.30, 0.0, TOP + HOT_R + 0.5 * MM), lying)
    cases += 3
    # geometry: the block's footprint over one insert stays clear of the other; the iron lies on its stand; the mat
    # under the plate, the block and the stand
    assert BLOCK_SIZE[1] / 2 < abs(HOLES["a"][1] - HOLES["b"][1]) - INSERT_D / 2
    assert GRIP_D < 0.094 and BLOCK_SIZE[1] < 0.094
    assert iron_in_holder((REST_TIP_X, REST_Y, REST_AXIS), lying)
    assert not iron_in_holder((REST_TIP_X, REST_Y, REST_AXIS + 0.05), lying)
    assert not iron_in_holder((REST_TIP_X, REST_Y, REST_AXIS), np.eye(3))  # upright, not lying on the stand
    lo, hi = np.array(MAT.pos[:2]) - np.array(MAT.size[:2]) / 2, np.array(MAT.pos[:2]) + np.array(MAT.size[:2]) / 2
    for xy in (PLATE_XY, BLOCK_XY, (REST_TIP_X, REST_Y), (REST_TIP_X + 0.175, REST_Y)):
        assert np.all(np.asarray(xy) > lo + 0.02) and np.all(np.asarray(xy) < hi - 0.02), xy
    cases += 4
    return cases


if __name__ == "__main__":
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / 20)[0] for k in range(round(plan.duration * 20) + 5)])
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
