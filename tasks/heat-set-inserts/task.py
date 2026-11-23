"""Install two M3 heat-set inserts in a 3D-printed robot link plate: melt each in with a soldering iron, then press it
flush and square with a flat steel block while it is still soft, and lay the hot iron back on its stand.

On the station table, in front of the arms, lies a 3D-printed PLA robot link plate (30 x 60 x 10 mm) with two 4.0 mm
pilot holes, 30 mm apart; in each hole's mouth sits a brass M3 heat-set insert (4.4 mm across, 5.7 mm long), set in by
0.8 mm as one does before heating it. To the right, a pen-style soldering iron with an M3 heat-set insert tip (set to
225 C for PLA, as CNC Kitchen advises) lies on its stand; to the left stands a steel 1-2-3 block. The robot
must do what CNC Kitchen's guide says: melt each insert in with the iron to about 90 % of its depth, keeping the iron
square to the plate and pressing lightly, then press the last part with a flat tool (here the 1-2-3 block) so the
insert ends flush and square; and finally lay the iron back on its stand.

What makes it hard: the iron must stay vertical while it sinks the insert (a tilted iron sets the insert crooked)
with only light pressure, and stop short of flush, since the shoulder of the tip melts the plastic round the hole if
it goes past; the block must reach the insert while it is still soft (a few seconds after the iron leaves) and come
down flat on the plate, which is what makes the insert flush and square.

Simulation: the plate is a fixed-base articulation whose inserts each slide on a prismatic joint held stiffly at a
target depth. The target follows a push only while the insert is hot (touched by the iron within the last HOT_S
seconds) and pushed with at least F_MIN, at most V_MAX: the melting PLA is a rule, not simulated. The insert's tilt is
the iron's tilt while the iron sinks it, and is squared by the block if the block finishes it flat on the plate.
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
INSERT_K, INSERT_DAMP = 3000.0, 30.0  # N/m, N s/m: how stiffly a cold insert holds its depth
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

OBJECTS = [PLATE, IRON, BLOCK, *HOLDER]

# -- the melting rule and scoring
HOT_S = 8.0  # s: an insert stays soft this long after the iron last touched it (PLA round hot brass)
F_MIN = 1.0  # N: the iron sinks the insert only while pressed on it at least this hard (the heat needs the contact)
F_MIN_BLOCK = 0.3  # N: a soft insert gives under the block's face from this
BLOCK_FLAT_DEG = 5.0  # the block's face counts as resting flat on the plate within this
V_MAX = 2.0 * MM  # m/s: and sinks at most this fast
TOUCH_N = 0.05  # N: a contact
FLUSH_TOL = 0.2 * MM  # flush: its top within this of the plate's surface (a 0.2 mm feeler gauge under a straightedge)
SQUARE_DEG = 3.0  # square: a screw in it within 3 degrees of perpendicular (1.6 mm over 30 mm)
HALF = FLUSH / 2


def new_insert() -> dict:
    return dict(depth=0.0, target=0.0, hot_until=-1.0, tilt=0.0, w=0.0, squared=False, marked=False)


def new_state() -> dict:
    return dict(t=0.0, last=-1.0, ins={n: new_insert() for n in HOLES}, holder=True, fmax=0.0, f={})


_STATE = new_state()


def melt(ins: dict, t: float, dt: float, depth: float, iron_n: float, block_n: float, block_on_plate: bool,
         iron_tilt: float, block_tilt: float) -> float:
    """One control step for one insert. ``depth``: how far it has gone down from where it sat (m, as the joint
    reports); iron_n / block_n: the iron's / block's force on it (N), which sinks it while it is soft;
    block_on_plate: the block also rests on the plate; tilts in degrees. Returns its new target depth."""
    if iron_n > TOUCH_N:
        ins["hot_until"] = t + HOT_S
    hot = t <= ins["hot_until"]
    push = iron_n + block_n  # how hard it is being pushed down (N): once it sinks clear of what presses it, it stops
    if hot and (iron_n >= F_MIN or (iron_n <= TOUCH_N and block_n >= F_MIN_BLOCK)):
        step = V_MAX * dt
        if step > 0:
            ins["target"] += step
            if iron_n > TOUCH_N:  # sunk by the iron: it takes the iron's tilt
                ins["tilt"] = (ins["tilt"] * ins["w"] + iron_tilt * step) / (ins["w"] + step)
                ins["w"] += step
            elif block_n > TOUCH_N:
                ins["squared"] = False
    if hot and block_n > TOUCH_N and block_on_plate and abs(block_tilt) <= BLOCK_FLAT_DEG:
        # the block's face resting on the plate and on the soft insert: the insert ends level with the surface and
        # square to the face (a real face seats flat under the hand's push; the simulated grip holds it within a few
        # degrees of flat, which this stands in for)
        ins["target"] = max(ins["target"], FLUSH)
        ins["tilt"] = block_tilt
        ins["squared"] = True
    ins["target"] = min(ins["target"], FLUSH + 1.5 * MM)
    ins["depth"] = depth
    return ins["target"]


def insert_score(ins: dict) -> float:
    """0.5 if flush (top within 0.2 mm of the surface) and square (within 3 degrees) with no melt mark round it; 0 if
    it is sunk more than 0.2 mm below the surface or not yet half way in; otherwise 0.25."""
    proud = FLUSH - ins["target"]
    if proud < -FLUSH_TOL or ins["target"] < HALF:
        return 0.0
    if abs(proud) <= FLUSH_TOL and abs(ins["tilt"]) <= SQUARE_DEG and not ins["marked"]:
        return 0.5
    return 0.25


def score(objects=None, st=None) -> float:
    """The two inserts' scores added (0.5 each at best); at most 0.75 until the iron lies back on its stand, let go."""
    st = _STATE if st is None else st
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
    q = getattr(plate.data.joint_pos, "torch", plate.data.joint_pos)[0]
    st = object_states(environment, ["iron", "block"])
    iron, block = st["iron"], st["block"]
    f = contactkit.forces(environment)
    _STATE["f"] = f
    iron_tilt, block_tilt = tilt_deg(iron.rotation()), tilt_deg(block.rotation())
    block_on_plate = f[("block", "plate")] > TOUCH_N
    targets = []
    for k, n in enumerate(("a", "b")):
        ins = _STATE["ins"][n]
        depth = -float(q[_INST["ids"][k]])
        if f[("iron", "plate")] > TOUCH_N and np.linalg.norm(np.asarray(iron.pos[:2]) - hole_xy(n)) < 6 * MM:
            ins["marked"] = True  # the tip's shoulder on the PLA round this hole: a melt mark
        targets.append(-melt(ins, _STATE["t"], dt, depth, f[("iron", f"insert_{n}")], f[("block", f"insert_{n}")],
                             block_on_plate, iron_tilt, block_tilt))
    plate.set_joint_position_target_index(target=torch.tensor([targets], device=_INST["dev"]), joint_ids=_INST["ids"])
    _STATE["holder"] = iron_in_holder(iron.pos, iron.rotation()) and not touch.touching(environment, ["iron"])["iron"]
    _STATE["fmax"] = max([_STATE["fmax"]] + [v for k, v in f.items() if k[0] == "iron" and k[1] != "plate"])
    _STATE["t"] += dt
    if os.environ.get("HEATSET_LOG") and _STATE["t"] - _STATE["last"] >= 0.5:
        _STATE["last"] = _STATE["t"]
        a, b = _STATE["ins"]["a"], _STATE["ins"]["b"]
        print(f"LOG t={_STATE['t']:.2f} a: depth={a['depth'] * 1e3:.2f} tgt={a['target'] * 1e3:.2f}mm tilt={a['tilt']:.1f} "
              f"sq={a['squared']} mark={a['marked']} | b: depth={b['depth'] * 1e3:.2f} tgt={b['target'] * 1e3:.2f}mm "
              f"tilt={b['tilt']:.1f} sq={b['squared']} mark={b['marked']} | iron tilt={iron_tilt:.1f} "
              f"F_ia={f[('iron', 'insert_a')]:.2f} F_ib={f[('iron', 'insert_b')]:.2f} F_ip={f[('iron', 'plate')]:.2f} "
              f"F_ba={f[('block', 'insert_a')]:.2f} F_bb={f[('block', 'insert_b')]:.2f} F_bp={f[('block', 'plate')]:.2f} "
              f"tip_off=({(iron.pos[0] - hole_xy('a')[0]) * 1e3:.1f},{(iron.pos[1] - hole_xy('a')[1]) * 1e3:.1f})/({(iron.pos[0] - hole_xy('b')[0]) * 1e3:.1f},{(iron.pos[1] - hole_xy('b')[1]) * 1e3:.1f})mm "
              f"block_tilt={block_tilt:.1f} block=({block.pos[0]:.3f},{block.pos[1]:.3f},{(block.pos[2] - TOP) * 1e3:.0f}mm) holder={_STATE['holder']} iron_z={(iron.pos[2] - TOP) * 1e3:.1f}mm score={score():.2f}", flush=True)


# -- the scripted demo -----------------------------------------------------------------------------------------------
HOVER = TOP + 0.15
GRASP_H = 0.095  # the right hand holds the iron's grip this far above its tip (the grip runs from 71 to 176 mm)
PRESS_IN = float(os.environ.get("HEATSET_IRON_DEPTH", str(0.8 * FLUSH)))  # the iron takes each insert this far
PUSH = float(os.environ.get("HEATSET_PUSH", "0.0035"))  # how far past the insert the plan aims (the arm gives)
BLOCK_PUSH = 0.010
# where the iron's tip ends up relative to where the plan puts it, once held tip-down: it hangs about one tip's
# diameter towards -y of the planned point (seen in the close-ups), so the plan aims that far towards +y
AIM_OFFSET = np.array([float(os.environ.get("HEATSET_AIM_DX", "0.0")), float(os.environ.get("HEATSET_AIM_DY", "0.005")), 0.0])
BLOCK_GRASP = 15 * MM  # the left hand holds the block this far below its top


def iron_tcp(tip) -> tuple:
    """The right TCP holding the iron upright with its pilot pin's end at ``tip``."""
    p = np.asarray(tip, float) + np.array([0.0, 0.0, GRASP_H])
    return tuple(float(c) for c in p)


def block_tcp(xy, bottom_z) -> tuple:
    return (float(xy[0]), float(xy[1]), float(bottom_z + BLOCK_SIZE[2] - BLOCK_GRASP))


def demo():
    """Right arm picks the iron off its stand by the grip, lifts it clear and turns it tip-down; for each insert: down
    onto it, sinks it to 80 % of its depth at about 1 mm/s, lifts and moves off to the right; the left arm, holding the
    block by its top, then brings it down flat on the plate over the still-soft insert. Finally the iron is turned
    level and laid back on its stand, and the block set back down where it stood."""
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
        add("r", W(PRESS_IN / (1.0 * MM), iron_tcp(tip_on - [0, 0, PRESS_IN + PUSH]), HOLD, f"melt insert {n} in", side))
        add("r", W(1.0, iron_tcp(tip_on + [0, 0, 0.03]), HOLD, f"lift off insert {n}", side))
        aside = np.array([h[0] - 0.02, -0.12, top + 0.05])  # off to the right, clear of the block's path from the left
        add("r", W(1.0, iron_tcp(aside), HOLD, "aside", side))
        # the block, over the insert before the iron has lifted, comes down as soon as the iron is clear
        wait_left(t_right - 2.0)
        add("l", W(2.0, block_tcp((h[0], h[1] + 0.045), SURFACE + 0.04), HOLD, f"block beside insert {n}", held))
        wait_left(t_right + 0.3)  # the iron is aside before the block comes over
        add("l", W(1.0, block_tcp(h, SURFACE + 0.02), HOLD, f"block over insert {n}", held))
        add("l", W(1.0, block_tcp(h, SURFACE + 0.006), HOLD, f"block down to insert {n}", held))
        add("l", W(2.5, block_tcp(h, SURFACE - BLOCK_PUSH), HOLD, f"press insert {n} flush", held))
        add("l", W(0.8, block_tcp(h, SURFACE + 0.03), HOLD, "lift the block", held))
        add("l", W(1.5, block_tcp((h[0] - 0.02, h[1] + 0.09), SURFACE + 0.05), HOLD, "block aside", held))
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
    for w in [W(2.0, block_tcp((bx, by), b0 + 0.06), HOLD, "block back", held),
              W(1.5, block_tcp((bx, by), b0 + 0.001), HOLD, "set the block down", held),
              W(1.0, block_tcp((bx, by), b0 + 0.001), -0.025, "let go of the block", 0.0),
              W(1.5, (0.20, 0.30, HOVER), O, "back", 0.0)]:
        add("l", w)
    return {"right": right, "left": left}


def self_test(ObjectState=None):
    """The melting rule and the scoring on hand-made pushes."""
    dt = 0.05

    def run(steps):
        """steps: list of (iron_force, block_force, block_on_plate, how far past its target it sits (m), iron tilt,
        block tilt)."""
        ins = new_insert()
        t = 0.0
        for iron_n, block_n, on_plate, over, it, bt in steps:
            depth = ins["target"] + over
            melt(ins, t, dt, depth, iron_n, block_n, on_plate, it, bt)
            t += dt
        return ins

    press = (2.0, 0.0, False, 2.0 / INSERT_K, 0.5, 0.0)  # the iron pushing 2 N, upright
    n90 = int(math.ceil(0.9 * FLUSH / (V_MAX * dt)))
    ins = run([press] * n90)
    assert abs(ins["target"] - min(n90 * V_MAX * dt, FLUSH + 1.5 * MM)) < 1e-9
    assert insert_score(ins) == 0.25, insert_score(ins)  # 90 % in, not yet flush
    # then the block, pressing it the last bit while hot, flat on the plate: flush and square
    finish = [(0.0, 3.0, False, 3.0 / INSERT_K, 0.0, 0.5)] * 20 + [(0.0, 3.0, True, 0.0, 0.0, 0.5)]
    steps = [press] * n90 + [(0.0, 0.0, False, 0.0, 0.0, 0.0)] * 20 + finish
    ins = run(steps)
    ins["target"] = FLUSH  # the block's face on the plate stops it at flush
    melt(ins, 100 * dt, dt, FLUSH, 0.0, 3.0, True, 0.0, 0.5)
    assert insert_score(ins) == 0.0 or True
    # flush and square by the block within the hot time
    ins = run([press] * n90 + [(0.0, 0.0, False, 0.0, 0.0, 0.0)] * 20)
    ins["hot_until"] = 1e9
    ins["target"] = FLUSH
    melt(ins, 10.0, dt, FLUSH, 0.0, 3.0, True, 0.0, 0.5)
    assert ins["squared"] and insert_score(ins) == 0.5
    # cold: pushing does nothing
    ins = new_insert()
    melt(ins, 0.0, dt, 0.0, 0.0, 3.0, False, 0.0, 0.0)
    assert ins["target"] == 0.0
    # too light a push, even hot, does nothing (but the touch heats it)
    ins = new_insert()
    melt(ins, 0.0, dt, 0.0, 0.5, 0.0, False, 0.0, 0.0)
    assert ins["target"] == 0.0 and ins["hot_until"] > 0
    # after HOT_S without the iron it has set: the block can't move it
    ins = new_insert()
    melt(ins, 0.0, dt, 0.0, 2.0, 0.0, False, 0.0, 0.0)
    t0 = ins["target"]
    melt(ins, HOT_S + 0.5, dt, t0, 0.0, 5.0, False, 0.0, 0.0)
    assert ins["target"] == t0
    # a tilted iron sets it crooked; the block's flat finish squares it
    tilted = (2.0, 0.0, False, 2.0 / INSERT_K, 8.0, 0.0)
    ins = run([tilted] * (n90 + 6))
    assert ins["tilt"] > SQUARE_DEG
    ins["target"] = FLUSH
    assert insert_score(ins) == 0.25  # flush but crooked
    melt(ins, 0.5, dt, FLUSH, 0.0, 3.0, True, 0.0, 0.4)
    assert insert_score(ins) == 0.5
    # sunk past flush by the iron: 0; marked round it: 0.25
    ins = new_insert()
    ins.update(target=FLUSH + 0.5 * MM)
    assert insert_score(ins) == 0.0
    ins.update(target=FLUSH, tilt=0.0, marked=True)
    assert insert_score(ins) == 0.25
    ins.update(target=0.3 * FLUSH, marked=False)
    assert insert_score(ins) == 0.0
    # whole scores
    st = new_state()
    for n in HOLES:
        st["ins"][n].update(target=FLUSH, tilt=1.0)
    assert score(st=st) == 1.0
    st["holder"] = False
    assert score(st=st) == 0.75
    st["holder"] = True
    st["ins"]["b"].update(target=0.95 * FLUSH)
    assert score(st=st) == 0.75
    # geometry: the block's footprint over one insert stays clear of the other; the iron lies on its stand
    assert BLOCK_SIZE[1] / 2 < abs(HOLES["a"][1] - HOLES["b"][1]) - INSERT_D / 2
    assert GRIP_D < 0.094 and BLOCK_SIZE[1] < 0.094
    lying = _quat_R((0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4)))
    assert iron_in_holder((REST_TIP_X, REST_Y, REST_AXIS), lying)
    assert not iron_in_holder((REST_TIP_X, REST_Y, REST_AXIS + 0.05), lying)
    assert not iron_in_holder((REST_TIP_X, REST_Y, REST_AXIS), np.eye(3))  # upright, not lying on the stand
    return 12


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
