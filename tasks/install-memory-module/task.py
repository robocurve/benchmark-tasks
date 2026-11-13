"""On the station table, between the arms: an ATX motherboard (305 x 244 mm) lying flat, its four memory slots
running away from the robot, the second slot from the left (A2) straight ahead of the gap between the arms. Each
slot has a black latch at each end, with a press tab on top; A2's latches are open. On the right, a DDR5 desktop
memory module (133 x 31 mm, grey heat spreader, 40 g) stands upright on its contact edge in a grey holder, its length
running left-right: a quarter turn from how it fits the slot, so it has to be turned the right way for its key notch
to meet the key in the slot.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 ...
Check the scoring offline:  python -c "import task; task.self_test()"
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from scenekit import Shape, UsdBody, on_reset, on_step  # noqa: E402

import latches  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = "Install the memory module in slot A2 and press it down until both latches click shut."

MM = 0.001
TABLE = 0.7475
GREY = (0.45, 0.47, 0.50)

# -- the board (assets/board.usda: origin at the board's centre, bottom face, slots along its y) --------------------
BOARD_POS = np.array([0.262, 0.073, TABLE])
BOARD_YAW = -math.pi / 2  # board +y along world +x
SLOT_PITCH = 10 * MM  # A1, A2, B1, B2 are 10 mm apart
SLOT_XY = np.array([73 * MM, 58 * MM])  # A2's centre, board frame
FLOOR = 4.8 * MM  # a seated module's contact edge, above the board's bottom face
SLOT_TOP = 7.8 * MM


def yaw_xyzw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def board_to_world(p) -> np.ndarray:
    c, s = math.cos(BOARD_YAW), math.sin(BOARD_YAW)
    x, y, z = p
    return BOARD_POS + np.array([c * x - s * y, s * x + c * y, z])


SLOT = board_to_world((*SLOT_XY, FLOOR))  # A2's centre at floor height, world
SEATED_YAW = math.pi  # the module's yaw when it fits A2 (key notch over the key)

# -- the memory module (assets/dimm_ddr5.usda: origin at the contact edge's centre, x along its length) ---------------
DIMM_MASS = 0.04
DIMM_COMPLIANCE = (2.0e8, 2 * (2.0e8 * DIMM_MASS) ** 0.5)  # contact stiffness, critical damping
DIMM_HALF = 133.35 / 2 * MM
DIMM_MID = 15.6 * MM  # the module's middle above its contact edge

# -- the holder: a grey block with a 2 mm groove along y, on the right ------------------------------------------------
HOLDER_XY = np.array([0.32, -0.19])
HOLDER_LEN, HOLDER_W, HOLDER_BASE, WALL_H, GROOVE = 0.150, 0.030, 0.006, 0.005, 0.002
HOLDER = [
    Shape("holder_base", pos=(*HOLDER_XY, TABLE + HOLDER_BASE / 2), size=(HOLDER_W, HOLDER_LEN, HOLDER_BASE),
          color=GREY),
    *(Shape(f"holder_wall{i}", pos=(HOLDER_XY[0] + s * (GROOVE + (HOLDER_W - GROOVE) / 2) / 2, HOLDER_XY[1],
                                    TABLE + HOLDER_BASE + WALL_H / 2),
            size=((HOLDER_W - GROOVE) / 2, HOLDER_LEN, WALL_H), color=GREY) for i, s in enumerate((-1, 1))),
]
HOLDER_YAW = -math.pi / 2  # the module in the holder: a quarter turn from how it sits in the slot
DIMM_START = np.array([*HOLDER_XY, TABLE + HOLDER_BASE + 0.0003])

BOARD = UsdBody("board", usd="assets/board.usda", pos=tuple(BOARD_POS), rot_xyzw=yaw_xyzw(BOARD_YAW),
                size=(0.244, 0.305, 0.04), articulation=True, material=False,
                joint_pos=tuple((j, s * latches.OPEN) for j, s in zip(latches.JOINTS, latches.OPEN_SIGN)),
                joint_damping=latches.DAMPING)
DIMM = UsdBody("memory_module", usd="assets/dimm_ddr5.usda", pos=tuple(DIMM_START), rot_xyzw=yaw_xyzw(HOLDER_YAW),
               size=(0.134, 0.007, 0.035), mass=DIMM_MASS, friction=(0.6, 0.5), compliance=DIMM_COMPLIANCE)
OBJECTS = [BOARD, DIMM, *HOLDER]

# -- the latch spring, and whether a gripper still holds the module -------------------------------------------------
RELEASED = 0.045  # m: a gripper centre farther than this from the module's middle has let go of it
STILL = 0.001  # m: at rest means the module moved less than this over the last STILL_S seconds
STILL_S = 1.0
_STATE: dict = {"spring": None, "held": False, "still": True, "track": []}


@on_reset
def _reset(environment) -> None:
    board = environment.unwrapped.scene["board"]
    if _STATE["spring"] is None or _STATE["spring"].board is not board:
        _STATE["spring"] = latches.LatchSpring(board)
    else:
        _STATE["spring"].reset()
    _STATE.update(held=False, still=False, track=[], t=0.0)


@on_step
def _grippers(environment) -> None:
    """After each control step: whether either gripper centre (TCP) is within RELEASED of the module's middle, and
    whether the module has stayed within STILL of where it is now for the last STILL_S seconds. (Its velocity is no
    use for this: held in the slot's compliant contacts, it jitters by microns at speeds of a few cm/s.)"""
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS
    from yam_arena.scene import object_states

    d = object_states(environment, ["memory_module"])["memory_module"]
    middle = d.pos + d.rotation()[:, 2] * DIMM_MID
    _STATE["t"] = _STATE.get("t", 0.0) + environment.unwrapped.step_dt
    track = [(t, p) for t, p in _STATE["track"] if _STATE["t"] - t <= STILL_S + 1e-6] + [(_STATE["t"], d.pos.copy())]
    _STATE["track"] = track
    _STATE["still"] = bool(_STATE["t"] - track[0][0] >= STILL_S - 1e-6
                           and max(np.linalg.norm(p - d.pos) for _, p in track) < STILL)
    near = False
    for side in ("left", "right"):
        robot = environment.unwrapped.scene[f"{side}_robot"]
        q = getattr(robot.data.joint_pos, "torch", robot.data.joint_pos)[0]
        q = q[robot.find_joints(ARM_JOINTS, preserve_order=True)[0]].cpu().numpy().astype(float)
        tcp = ik.tcp_pose(q, station.BASES[side])[0]  # the env frame, as object_states
        near = near or bool(np.linalg.norm(tcp - middle) < RELEASED)
    _STATE["held"] = near


# -- scoring ------------------------------------------------------------------------------------------------------
SEATED = 0.3 * MM  # each end of the contact edge within this of the slot floor
LATCH_SHUT = math.radians(5.0)
ALIGN_XY = 1.5 * MM  # module centre from A2's centre line (across) and centre (along)
ALIGN_YAW = math.radians(5.0)  # from the way round that fits the key
UPRIGHT = math.radians(5.0)


def measure(objects) -> dict:
    """The module relative to A2 (offsets, yaw error, tilt, each end's contact edge height above the slot floor),
    whether it is at rest (from the control steps' track, see _grippers), and the latches' angles from shut (rad)."""
    d, b = objects["memory_module"], objects["board"]
    rot = d.rotation()
    off = d.pos - SLOT
    yaw = math.atan2(rot[1, 0], rot[0, 0])
    ends = [float(off[2] + s * DIMM_HALF * rot[2, 0]) for s in (1, -1)]
    return {
        "along": float(off[0]),
        "across": float(off[1]),
        "yaw_err": abs((yaw - SEATED_YAW + math.pi) % (2 * math.pi) - math.pi),
        "tilt": math.acos(float(np.clip(rot[2, 2], -1.0, 1.0))),
        "ends": ends,
        "at_rest": _STATE["still"],
        "latches": [abs(b.joints.get(j, latches.OPEN)) for j in latches.JOINTS],
    }


def _in_a2(m) -> bool:
    """Standing in A2 the right way round, its contact edge below the slot's top."""
    return (abs(m["across"]) < ALIGN_XY and abs(m["along"]) < ALIGN_XY and m["yaw_err"] < ALIGN_YAW
            and m["tilt"] < UPRIGHT and max(m["ends"]) < SLOT_TOP - FLOOR)


def success(objects) -> bool:
    """Released, at rest (moved under 1 mm in the last second), in A2 the right way round with both ends of its
    contact edge within 0.3 mm of the slot floor and both latches within 5 degrees of shut; held 1 s."""
    m = measure(objects)
    return bool(not _STATE["held"] and m["at_rest"] and _in_a2(m) and max(m["ends"]) < SEATED
                and all(a < LATCH_SHUT for a in m["latches"]))


def score(objects) -> float:
    """0 if a gripper still holds the module (a gripper centre within 4.5 cm of its middle) or it is not standing
    in A2 the right way round. Otherwise 1.0 for success, 0.5 with one or both latches shut, 0.25 with neither."""
    m = measure(objects)
    if _STATE["held"] or not _in_a2(m):
        return 0.0
    if success(objects):
        return 1.0
    return 0.5 if any(a < LATCH_SHUT for a in m["latches"]) else 0.25


def self_test(ObjectState=None) -> None:
    """The scoring rules on hand-built states; runs offline, with the real ObjectState or any stand-in."""
    from scenekit import BodyState

    if ObjectState is None:
        from yam_arena.task_file import ObjectState  # noqa: F401  (BodyState extends it)

    def module(dx=0.0, dy=0.0, up=0.0, yaw=SEATED_YAW, pitch=0.0):
        """The module's contact edge centre ``up`` above A2's floor, offset dx (along) / dy (across), turned to
        ``yaw``, pitched along its length by ``pitch``."""
        c, s, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
        q = np.array([-s * sp, c * sp, s * cp, c * cp])  # yaw about z, then pitch about the module's y
        pos = SLOT + (dx, dy, up)
        return BodyState(pos=pos, quat_xyzw=q, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=pos, high=pos)

    def board(*angles_deg):
        pos = BOARD_POS.copy()
        return BodyState(pos=pos, quat_xyzw=np.array(yaw_xyzw(BOARD_YAW)), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                         low=pos, high=pos, joints={j: math.radians(a) for j, a in zip(latches.JOINTS, angles_deg)})

    in_holder = BodyState(pos=DIMM_START, quat_xyzw=np.array(yaw_xyzw(HOLDER_YAW)), lin_vel=np.zeros(3),
                          ang_vel=np.zeros(3), low=DIMM_START, high=DIMM_START)
    shut, open_ = (0.0, 0.0), (30.0, -30.0)
    tilt_up = math.atan2(0.5 * MM, DIMM_HALF)  # one end 0.5 mm higher than the other
    cases = {
        # name: (module, latch angles, held, still, score, success)
        "start: in the holder": (in_holder, open_, False, True, 0.0, False),
        "done": (module(), shut, False, True, 1.0, True),
        "done, 0.25 mm up, 1 mm off along and across, 3 degrees of yaw": (
            module(dx=0.001, dy=-0.001, up=0.25 * MM, yaw=SEATED_YAW + math.radians(3)), shut, False, True, 1.0, True),
        "done but a gripper still holds it": (module(), shut, True, True, 0.0, False),
        "seated and latched but moved 2 mm in the last second": (module(), shut, False, False, 0.5, False),
        "both latches shut, 0.5 mm up": (module(up=0.5 * MM), shut, False, True, 0.5, False),
        "both latches shut, one end 0.5 mm up": (module(pitch=tilt_up), shut, False, True, 0.5, False),
        "one latch shut, the other 20 degrees open": (module(), (0.0, -20.0), False, True, 0.5, False),
        "one latch 4 degrees from shut, the other open": (module(up=1.0 * MM), (4.0, -30.0), False, True, 0.5, False),
        "in A2 the right way round, 2.3 mm up, latches open": (module(up=2.3 * MM), open_, False, True, 0.25, False),
        "reversed, stopped on the key 2.1 mm up": (module(up=2.1 * MM, yaw=0.0), open_, False, True, 0.0, False),
        "a quarter turn out, standing on A2": (module(up=3.0 * MM, yaw=SEATED_YAW + math.pi / 2), open_, False, True,
                                               0.0, False),
        "seated in A1 instead": (module(dy=SLOT_PITCH), shut, False, True, 0.0, False),
        "set on top of A2's housing, 3.5 mm up": (module(up=3.5 * MM), open_, False, True, 0.0, False),
        "leaning 8 degrees in A2": (module(up=2.0 * MM, pitch=math.radians(8)), open_, False, True, 0.0, False),
    }
    failures = []
    for name, (d, angles, held, still, want_score, want_success) in cases.items():
        _STATE.update(held=held, still=still)
        objects = {"memory_module": d, "board": board(*angles)}
        got = (score(objects), success(objects))
        if abs(got[0] - want_score) > 1e-9 or got[1] != want_success:
            failures.append(f"{name}: got score {got[0]}, success {got[1]}; want {want_score}, {want_success}")
    _STATE.update(held=False, still=True)
    if failures:
        raise AssertionError("\n".join(failures))
    print(f"self_test: {len(cases)} cases pass")


# -- the scripted demo ----------------------------------------------------------------------------------------
SPREADER_TOP = 0.03465  # the heat spreader's top above the contact edge
GRIP_BELOW_TOP = 0.012  # the pads' tips this far below the spreader's top while carrying
SET_ABOVE = 0.001  # set it with the contact edge this far above the slot top, in the lead-in
SET_CORRECTION = (0.0020, 0.0005)  # the right arm sets the module this far short in x and y (its sag); aim past it
PUSH_DEPTH = 0.045  # push it home while gripping it: the contact edge commanded this far below the slot floor
PUSH_HOLD = 0.5  # s, held at the bottom of the push
PRESS_ENDS = 0.045  # then press each end this far from its middle, fingers closed (nearer the ends tips it like a see-saw)
PRESS_DEPTH = 0.03  # TCP commanded this far into the seated module's top


def demo():
    """The right arm picks the module by its spreader, turns it a quarter turn to fit the key, sets it in A2's
    lead-in and pushes it home while still gripping it (so it stays upright), lets go, then presses each end with
    closed fingers to make sure both latches shut. The left arm holds its ready pose."""
    from yam_arena.constants import FINGER_CLOSED as C
    from yam_arena.constants import FINGER_OPEN as O
    from yam_arena.constants import PAD_TIP_PAST_TCP as TIP
    from yam_arena.demo import Waypoint as W

    hover = TABLE + 0.16
    over = TABLE + 0.12  # travel height over the board: the arms can't reach 0.16 m up that far out
    hx, hy = HOLDER_XY
    grip_z = TABLE + HOLDER_BASE + SPREADER_TOP - GRIP_BELOW_TOP + TIP
    edge_below_tcp = grip_z - (TABLE + HOLDER_BASE)
    sx, sy = SLOT[:2]
    cx, cy = sx + SET_CORRECTION[0], sy + SET_CORRECTION[1]
    set_z = TABLE + SLOT_TOP + SET_ABOVE + edge_below_tcp
    top = SLOT[2] + SPREADER_TOP + TIP  # TCP with closed fingers resting on a seated module's top
    press_z = top - PRESS_DEPTH
    seated_z = SLOT[2] + edge_below_tcp  # the gripping TCP when the module is seated
    push_z = seated_z - PUSH_DEPTH
    grip_yaw = HOLDER_YAW + math.pi  # the fingers close across the module; turning to 0 seats it
    right = [
        W(2.0, (hx, hy, hover), O, "to the holder", grip_yaw),
        W(1.5, (hx, hy, grip_z), O, "descend", grip_yaw),
        W(2.0, (hx, hy, grip_z), C, "grip", grip_yaw),  # the fingers take about 2 s to close
        W(1.0, (hx, hy, grip_z), C, "settle", grip_yaw),
        W(1.5, (hx, hy, hover), C, "lift", grip_yaw),
        W(2.5, (sx, sy, over), C, "carry and turn", 0.0),
        W(2.0, (cx, cy, set_z + 0.01), C, "over the slot", 0.0),
        # one slow, smooth descent through the lead-in and home, still gripped so it stays upright
        W(5.0, (cx, cy, push_z), C, "set it in and push it home", 0.0),
        W(PUSH_HOLD, (cx, cy, push_z), C, "hold", 0.0),
        W(0.8, (cx, cy, seated_z), C, "ease off", 0.0),  # so the hand doesn't drop down the module when it opens
        W(2.0, (cx, cy, seated_z), O, "let go", 0.0),
        W(1.0, (cx, cy, over), O, "up", 0.0),
        *(w for x in (sx - PRESS_ENDS, sx + PRESS_ENDS) for w in (
            W(1.5, (x, sy, top + 0.015), C, "over an end", 0.0),
            W(1.0, (x, sy, press_z), C, "press the end", 0.0),
            W(1.0, (x, sy, press_z), C, "hold", 0.0),
            W(1.0, (x, sy, top + 0.015), C, "lift", 0.0))),
        W(1.5, (sx, sy, over), C, "up", 0.0),
    ]
    return {"right": right}
