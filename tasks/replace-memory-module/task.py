"""On the station table, between the arms: an ATX motherboard (305 x 244 mm) lying flat, its four memory slots
running away from the robot, the second slot from the left (A2) straight ahead of the gap between the arms. An old
DDR5 memory module is fully seated in A2, both of A2's latches shut on it. On the right, a new module of the same kind
stands upright on its contact edge in a grey holder, its length running left-right: a quarter turn from how it fits
the slot. On the left, beyond the board, an empty grey holder of the same kind has its groove running away from the
robot. The robot has to take the old module out of A2 (opening both latches first), stand it in the empty holder, and
install the new one in A2 until both latches click shut.

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
from scenekit import Shape, UsdBody, arm_joints, on_reset, on_step  # noqa: E402

import latches  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = ("Replace the memory module in slot A2 with the new one from the holder on the right, and stand the old "
               "one in the empty holder on the left.")

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
SEATED_YAW = math.pi  # a module's yaw when it fits A2 (key notch over the key)

# -- the modules (assets/dimm_ddr5.usda: origin at the contact edge's centre, x along its length) ---------------------
DIMM_MASS = 0.04
DIMM_COMPLIANCE = (2.0e8, 2 * (2.0e8 * DIMM_MASS) ** 0.5)  # contact stiffness, critical damping
DIMM_HALF = 133.35 / 2 * MM
DIMM_MID = 15.6 * MM  # a module's middle above its contact edge

# -- the holders: grey blocks with a 2 mm groove; the new module's on the right, the empty one on the left -----------
HOLDER_LEN, HOLDER_W, HOLDER_BASE, WALL_H, GROOVE = 0.150, 0.030, 0.006, 0.005, 0.002
OLD_GROOVE = 0.0035  # the empty holder's groove is wider, so a module set down in it goes in (it leans, at most ~20 deg)
NEW_HOLDER_XY = np.array([0.32, -0.19])  # groove along y
OLD_HOLDER_XY = np.array([0.32, 0.235])  # groove along x, clear of the board's edge (y 0.195)


def holder(name, xy, along_x, groove):
    """A holder's base and its two walls, the groove along x or y."""
    side = (HOLDER_W - groove) / 2
    wall = (groove + side) / 2  # a wall's centre from the groove's centre line
    if along_x:
        return [Shape(f"{name}_base", pos=(*xy, TABLE + HOLDER_BASE / 2), size=(HOLDER_LEN, HOLDER_W, HOLDER_BASE),
                      color=GREY),
                *(Shape(f"{name}_wall{i}", pos=(xy[0], xy[1] + s * wall, TABLE + HOLDER_BASE + WALL_H / 2),
                        size=(HOLDER_LEN, side, WALL_H), color=GREY) for i, s in enumerate((-1, 1)))]
    return [Shape(f"{name}_base", pos=(*xy, TABLE + HOLDER_BASE / 2), size=(HOLDER_W, HOLDER_LEN, HOLDER_BASE),
                  color=GREY),
            *(Shape(f"{name}_wall{i}", pos=(xy[0] + s * wall, xy[1], TABLE + HOLDER_BASE + WALL_H / 2),
                    size=(side, HOLDER_LEN, WALL_H), color=GREY) for i, s in enumerate((-1, 1)))]


NEW_HOLDER_YAW = -math.pi / 2  # the new module in its holder: a quarter turn from how it sits in the slot
NEW_START = np.array([*NEW_HOLDER_XY, TABLE + HOLDER_BASE + 0.0003])

BOARD = UsdBody("board", usd="assets/board.usda", pos=tuple(BOARD_POS), rot_xyzw=yaw_xyzw(BOARD_YAW),
                size=(0.244, 0.305, 0.04), articulation=True, material=False,
                joint_pos=tuple((j, 0.0) for j in latches.JOINTS), joint_damping=latches.DAMPING)
OLD = UsdBody("old_module", usd="assets/dimm_ddr5.usda", pos=tuple(SLOT + (0.0, 0.0, 0.05 * MM)),
              rot_xyzw=yaw_xyzw(SEATED_YAW), size=(0.134, 0.007, 0.035), mass=DIMM_MASS, friction=(0.6, 0.5),
              compliance=DIMM_COMPLIANCE)
NEW = UsdBody("new_module", usd="assets/dimm_ddr5.usda", pos=tuple(NEW_START), rot_xyzw=yaw_xyzw(NEW_HOLDER_YAW),
              size=(0.134, 0.007, 0.035), mass=DIMM_MASS, friction=(0.6, 0.5), compliance=DIMM_COMPLIANCE)
OBJECTS = [BOARD, OLD, NEW, *holder("new_holder", NEW_HOLDER_XY, False, GROOVE),
           *holder("old_holder", OLD_HOLDER_XY, True, OLD_GROOVE)]
MODULES = ("old_module", "new_module")

# -- what is tracked over the episode -------------------------------------------------------------------------------
RELEASED = 0.045  # m: a gripper centre farther than this from a module's middle has let go of it
STILL = 0.001  # m: at rest means a module moved less than this over the last STILL_S seconds
STILL_S = 1.0
OPENED = math.radians(20.0)  # a latch counts as opened at this angle...
OPENED_END = 2.5 * MM  # ...if the old module's end over it was still this low (opening lifts it to about 1.8 mm)

_STATE: dict = {"spring": None}


def _clear() -> None:
    _STATE.update(t=0.0, held={m: False for m in MODULES}, still={m: False for m in MODULES},
                  track={m: [] for m in MODULES}, opened=[False, False], torn=False)


_clear()


@on_reset
def _reset(environment) -> None:
    board = environment.unwrapped.scene["board"]
    if _STATE["spring"] is None or _STATE["spring"].board is not board:
        _STATE["spring"] = latches.LatchSpring(board)
    else:
        _STATE["spring"].reset()
    _clear()


@on_step
def _track(environment) -> None:
    """After each control step: for each module, whether a gripper centre (TCP) is within RELEASED of its middle and
    whether it has stayed within STILL of where it is now for the last STILL_S seconds; for each latch, whether it
    was opened while the old module's end over it was still down; and whether the old module has left A2 without
    both latches having been opened that way (torn out)."""
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS
    from yam_arena.scene import object_states

    st = object_states(environment, [*MODULES, "board"])
    _STATE["t"] += environment.unwrapped.step_dt
    tcps = [ik.tcp_pose(arm_joints(environment, side, ARM_JOINTS).astype(float), station.BASES[side])[0]
            for side in ("left", "right")]
    for name in MODULES:
        d = st[name]
        middle = d.pos + d.rotation()[:, 2] * DIMM_MID
        _STATE["held"][name] = any(bool(np.linalg.norm(tcp - middle) < RELEASED) for tcp in tcps)
        track = [(t, p) for t, p in _STATE["track"][name] if _STATE["t"] - t <= STILL_S + 1e-6]
        track.append((_STATE["t"], d.pos.copy()))
        _STATE["track"][name] = track
        _STATE["still"][name] = bool(_STATE["t"] - track[0][0] >= STILL_S - 1e-6
                                     and max(np.linalg.norm(p - d.pos) for _, p in track) < STILL)
    old = measure(st, "old_module")
    for i in range(2):
        if old["latches"][i] >= OPENED and old["ends_by_latch"][i] < OPENED_END:
            _STATE["opened"][i] = True
    if not _in_slot(old) and not all(_STATE["opened"]):
        _STATE["torn"] = True


# -- scoring ------------------------------------------------------------------------------------------------------
SEATED = 0.3 * MM  # each end of the contact edge within this of the slot floor
LATCH_SHUT = math.radians(5.0)
ALIGN_XY = 1.5 * MM  # a module's centre from A2's centre line (across) and centre (along)
ALIGN_YAW = math.radians(5.0)  # from the way round that fits the key
UPRIGHT = math.radians(5.0)
IN_SLOT_XY = (10 * MM, 5 * MM)  # the old module counts as still in A2 within this of its centre (along, across)...
IN_SLOT_UP = SLOT_TOP - FLOOR + 1 * MM  # ...with its lower end below this (1 mm above the slot top)
HOLDER_ALONG = 15 * MM  # in the old holder: centre within this of the holder's centre along the groove,
HOLDER_ACROSS = OLD_GROOVE / 2  # its edge's centre within the groove,
HOLDER_UP = (-1 * MM, 2 * MM)  # its contact edge this near the groove's floor (the holder base's top),
HOLDER_TILT = math.radians(20.0)  # upright (leaning on a wall at most) and lined up with the groove within this
OFF_TABLE = TABLE - 0.05  # a module whose origin is below this has fallen off the table


def measure(objects, name) -> dict:
    """A module relative to A2 (offsets, yaw error, tilt, each end's contact edge height above the slot floor, and
    those heights ordered as the latches: near, far), and the latches' angles from shut (rad, near and far)."""
    d, b = objects[name], objects["board"]
    rot = d.rotation()
    off = d.pos - SLOT
    yaw = math.atan2(rot[1, 0], rot[0, 0])
    ends = [d.pos + s * DIMM_HALF * rot[:, 0] for s in (1, -1)]
    near, far = sorted(ends, key=lambda p: p[0])  # the near latch is the one at the smaller x
    return {
        "pos": d.pos,
        "rot": rot,
        "along": float(off[0]),
        "across": float(off[1]),
        "yaw_err": abs((yaw - SEATED_YAW + math.pi) % (2 * math.pi) - math.pi),
        "tilt": math.acos(float(np.clip(rot[2, 2], -1.0, 1.0))),
        "ends": [float(p[2] - SLOT[2]) for p in ends],
        "ends_by_latch": [float(near[2] - SLOT[2]), float(far[2] - SLOT[2])],
        "latches": [abs(b.joints.get(j, latches.OPEN)) for j in latches.JOINTS],
    }


def _in_slot(m) -> bool:
    """Still in A2 in any way (the old module before it comes out)."""
    return abs(m["along"]) < IN_SLOT_XY[0] and abs(m["across"]) < IN_SLOT_XY[1] and min(m["ends"]) < IN_SLOT_UP


def _in_a2(m) -> bool:
    """Standing in A2 the right way round, its contact edge below the slot's top (as the install task)."""
    return (abs(m["across"]) < ALIGN_XY and abs(m["along"]) < ALIGN_XY and m["yaw_err"] < ALIGN_YAW
            and m["tilt"] < UPRIGHT and max(m["ends"]) < SLOT_TOP - FLOOR)


def _in_old_holder(m) -> bool:
    """Standing in the left holder's groove (either way round), released."""
    off = m["pos"][:2] - OLD_HOLDER_XY
    yaw = math.atan2(m["rot"][1, 0], m["rot"][0, 0])
    lined_up = abs(math.sin(yaw)) < math.sin(HOLDER_TILT)  # its length along x, either way round
    edge = m["pos"][2] - (TABLE + HOLDER_BASE)
    return (abs(off[0]) < HOLDER_ALONG and abs(off[1]) < HOLDER_ACROSS and HOLDER_UP[0] < edge < HOLDER_UP[1]
            and m["tilt"] < HOLDER_TILT and lined_up and not _STATE["held"]["old_module"])


def success(objects) -> bool:
    """The new module seated in A2 (both ends within 0.3 mm of the floor, both latches within 5 degrees of shut), the
    old one standing in the left holder, both released and at rest; held 1 s."""
    old, new = measure(objects, "old_module"), measure(objects, "new_module")
    return bool(not _STATE["torn"] and _in_old_holder(old) and _in_a2(new) and max(new["ends"]) < SEATED
                and all(a < LATCH_SHUT for a in new["latches"]) and not any(_STATE["held"].values())
                and all(_STATE["still"].values()))


def score(objects) -> float:
    """0 if the old module was torn out (it left A2 before both latches had been opened), a module fell off the
    table, or the old module is still in A2. Otherwise 0.25 for the old module out, 0.5 with it standing in the left
    holder, 0.75 with the new module also standing in A2 the right way round, 1.0 for success."""
    old, new = measure(objects, "old_module"), measure(objects, "new_module")
    if _STATE["torn"] or min(old["pos"][2], new["pos"][2]) < OFF_TABLE or _in_slot(old):
        return 0.0
    if not _in_old_holder(old):
        return 0.25
    if not _in_a2(new):
        return 0.5
    return 1.0 if success(objects) else 0.75


def self_test(ObjectState=None) -> None:
    """The scoring rules on hand-built states; runs offline, with the real ObjectState or any stand-in."""
    from scenekit import BodyState

    if ObjectState is None:
        from yam_arena.task_file import ObjectState  # noqa: F401  (BodyState extends it)

    def body(pos, yaw, pitch=0.0):
        c, s, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
        q = np.array([-s * sp, c * sp, s * cp, c * cp])  # yaw about z, then pitch about the module's y
        pos = np.asarray(pos, float)
        return BodyState(pos=pos, quat_xyzw=q, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=pos, high=pos)

    def in_a2(up=0.0, yaw=SEATED_YAW):
        return body(SLOT + (0.0, 0.0, up), yaw)

    def in_old_holder(dx=0.0, yaw=0.0):
        return body((OLD_HOLDER_XY[0] + dx, OLD_HOLDER_XY[1], TABLE + HOLDER_BASE + 0.3 * MM), yaw)

    def board(*angles_deg):
        pos = BOARD_POS.copy()
        return BodyState(pos=pos, quat_xyzw=np.array(yaw_xyzw(BOARD_YAW)), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                         low=pos, high=pos, joints={j: math.radians(a) for j, a in zip(latches.JOINTS, angles_deg)})

    new_in_holder = body(NEW_START, NEW_HOLDER_YAW)
    on_table = body((0.25, 0.12, TABLE + 0.004), 0.3, pitch=math.pi / 2)  # lying on the board
    off_table = body((0.6, 0.0, TABLE - 0.3), 0.0)
    held_up = body(SLOT + (0.0, 0.0, 0.08), SEATED_YAW)
    shut, open_ = (0.0, 0.0), (30.0, -30.0)
    both, none = (True, True), (False, False)
    cases = {
        # name: (old, new, latch angles, latches opened first, torn, (old held, new held), still, score, success)
        "start": (in_a2(0.05 * MM), new_in_holder, shut, none, False, (False, False), True, 0.0, False),
        "latches opened, old lifted onto their feet": (in_a2(1.8 * MM), new_in_holder, open_, both, False,
                                                       (False, False), True, 0.0, False),
        "old pulled out, held up": (held_up, new_in_holder, open_, both, False, (True, False), True, 0.25, False),
        "old torn out with a latch shut": (held_up, new_in_holder, open_, (True, False), True, (True, False), True,
                                           0.0, False),
        "old put down on the board": (on_table, new_in_holder, open_, both, False, (False, False), True, 0.25,
                                      False),
        "old in the left holder": (in_old_holder(), new_in_holder, open_, both, False, (False, False), True, 0.5,
                                   False),
        "old in the left holder, turned round, 10 mm along": (in_old_holder(dx=0.01, yaw=math.pi), new_in_holder,
                                                              open_, both, False, (False, False), True, 0.5, False),
        "old in the left holder but still gripped": (in_old_holder(), new_in_holder, open_, both, False,
                                                     (True, False), True, 0.25, False),
        "new standing in A2, latches open": (in_old_holder(), in_a2(2.3 * MM), open_, both, False, (False, False),
                                             True, 0.75, False),
        "new reversed on the key": (in_old_holder(), in_a2(2.1 * MM, yaw=0.0), open_, both, False, (False, False),
                                    True, 0.5, False),
        "done": (in_old_holder(), in_a2(), shut, both, False, (False, False), True, 1.0, True),
        "done but the new one still gripped": (in_old_holder(), in_a2(), shut, both, False, (False, True), True,
                                               0.75, False),
        "done, one latch shut": (in_old_holder(), in_a2(), (0.0, -20.0), both, False, (False, False), True, 0.75,
                                 False),
        "new seated, old left on the board": (on_table, in_a2(), shut, both, False, (False, False), True, 0.25,
                                              False),
        "old fell off the table": (off_table, new_in_holder, open_, both, False, (False, False), True, 0.0, False),
    }
    failures = []
    for name, (old, new, angles, opened, torn, (old_held, new_held), still, want_score, want_success) in cases.items():
        _clear()
        _STATE.update(opened=list(opened), torn=torn, held={"old_module": old_held, "new_module": new_held},
                      still={m: still for m in MODULES})
        objects = {"old_module": old, "new_module": new, "board": board(*angles)}
        got = (score(objects), success(objects))
        if abs(got[0] - want_score) > 1e-9 or got[1] != want_success:
            failures.append(f"{name}: got score {got[0]}, success {got[1]}; want {want_score}, {want_success}")
    _clear()
    if failures:
        raise AssertionError("\n".join(failures))
    print(f"self_test: {len(cases)} cases pass")


# -- the scripted demo ----------------------------------------------------------------------------------------
SPREADER_TOP = 0.03465  # a module's heat spreader top above its contact edge
GRIP_BELOW_TOP = 0.012  # the pads' tips this far below the spreader's top while carrying
UNLATCHED_UP = 1.8 * MM  # the old module's contact edge above the floor once both latches are open
# Pressing a latch open (the fingers turned to close along the slot, so the closed tips are 8 mm along it): onto the
# tab's outer part, then straight down and outwards, commanded well past the tab top, since the arm only pushes as hard
# as its joint stiffness times the shortfall and tipping a shut latch past its snap takes about 4 N at the tab.
TAB_FROM_CENTRE = DIMM_HALF + 0.3 * MM + 2.5 * MM + 0.75 * 8.0 * MM  # A2's centre to the pressing point, along x
TAB_TOP = TABLE + 19.8 * MM
PRESS_DOWN, PRESS_OUT = 12 * MM, 8 * MM
PRESS_AIM = {"right": 0.0, "left": -4 * MM}  # x: the left arm lands 2-3 mm further out than commanded there
# Installing the new module (as the install task's demo)
SET_ABOVE = 0.001  # set it with the contact edge this far above the slot top, in the lead-in
SET_CORRECTION = (0.00075, 0.0005)  # aim this far past the slot in x and y (the arm's sag); 0.5-1 mm seat it
PUSH_DEPTH = 0.045  # push it home while gripping it: the contact edge commanded this far below the slot floor
PUSH_HOLD = 0.5
PRESS_ENDS = 0.045  # then press each end this far from its middle, fingers closed
PRESS_DEPTH = 0.03  # TCP commanded this far into the seated module's top


def demo():
    """The right arm presses A2's near latch tab, which opens the latch, and goes for the new module; then the left arm
    presses the far one (the open latches lift the old module onto their feet), pulls the old module straight up and
    stands it in the left holder. Meanwhile the right arm picks the new module from its holder; once the left arm is clear it turns
    the new module to fit the key, sets it in A2, pushes it home while gripping it, lets go and presses each end."""
    from yam_arena.constants import FINGER_CLOSED as C
    from yam_arena.constants import FINGER_OPEN as O
    from yam_arena.constants import PAD_TIP_PAST_TCP as TIP
    from yam_arena.demo import Waypoint as W

    sx, sy, sz = SLOT
    over = TABLE + 0.12
    hover = TABLE + 0.16
    pad_to_edge = SPREADER_TOP - GRIP_BELOW_TOP + TIP  # the gripping TCP above a module's contact edge
    tab_tcp = TAB_TOP + TIP  # TCP with the closed pad tips on a tab's top
    yaw90 = math.pi / 2

    def press(sign, side):
        x = sx + sign * TAB_FROM_CENTRE + PRESS_AIM[side]
        return [
            W(2.5, (x, sy, tab_tcp + 0.02), C, "over the latch tab", yaw90),
            W(1.0, (x, sy, tab_tcp + 0.002), C, "onto the tab", yaw90),
            W(1.0, (x, sy, tab_tcp - PRESS_DOWN), C, "press it down", yaw90),
            W(1.0, (x + sign * PRESS_OUT, sy, tab_tcp - PRESS_DOWN), C, "and outwards", yaw90),
            W(1.0, (x + sign * PRESS_OUT, sy, tab_tcp + 0.02), C, "lift", yaw90),
            W(1.5, (x, sy, over), C, "up", yaw90),
        ]  # 8 s; the arms are slow to turn the grip and cross over, so they rise first and move in separate steps

    # left: open the far latch, pull the old module out, stand it in the left holder
    pull_z = sz + UNLATCHED_UP + pad_to_edge
    ox, oy = OLD_HOLDER_XY
    set_old = TABLE + HOLDER_BASE + 0.001 + pad_to_edge
    # The arms take turns at the slot: reaching the far latch, the left arm's forearm crosses over the near one.
    left = [
        W(9.5, (0.25, 0.2, over), C, "wait for the right arm to open its latch and leave", yaw90),
        *press(1, "left"),
        W(2.0, (sx, sy, over), O, "over the old module, turning the grip", 0.0),
        W(1.5, (sx, sy, pull_z + 0.03), O, "lower", 0.0),
        W(1.5, (sx, sy, pull_z), O, "descend", 0.0),
        W(2.0, (sx, sy, pull_z), C, "grip", 0.0),
        W(1.0, (sx, sy, pull_z), C, "settle", 0.0),
        W(3.0, (sx, sy, pull_z + 0.08), C, "pull it straight up", 0.0),
        W(2.5, (ox, oy, set_old + 0.04), C, "over the left holder", 0.0),
    ]
    clear_of_slot = sum(w.duration for w in left)  # the left arm is over its holder by now
    left += [
        W(2.0, (ox, oy, set_old), C, "set it in the groove", 0.0),
        W(2.0, (ox, oy, set_old), O, "let go", 0.0),
        W(1.5, (ox, oy, hover), O, "up and away", 0.0),
    ]

    # right: open the near latch, pick the new module, wait for the slot, install it
    hx, hy = NEW_HOLDER_XY
    grip_z = TABLE + HOLDER_BASE + pad_to_edge
    cx, cy = sx + SET_CORRECTION[0], sy + SET_CORRECTION[1]
    set_z = TABLE + SLOT_TOP + SET_ABOVE + pad_to_edge
    seated_z = sz + pad_to_edge
    push_z = seated_z - PUSH_DEPTH
    top = sz + SPREADER_TOP + TIP  # TCP with closed fingers resting on a seated module's top
    grip_yaw = NEW_HOLDER_YAW + math.pi  # the fingers close across the module; turning to 0 seats it
    right = [
        *press(-1, "right"),
        W(2.5, (hx, hy, hover), O, "to the new module", grip_yaw),
        W(2.0, (hx, hy, grip_z), O, "descend", grip_yaw),
        W(2.0, (hx, hy, grip_z), C, "grip", grip_yaw),
        W(1.0, (hx, hy, grip_z), C, "settle", grip_yaw),
        W(1.5, (hx, hy, hover), C, "lift", grip_yaw),
    ]
    right += [
        W(max(0.5, clear_of_slot - sum(w.duration for w in right)), (hx, hy, hover), C, "wait for the left arm",
          grip_yaw),
        W(2.5, (sx, sy, over), C, "carry and turn", 0.0),
        W(2.0, (cx, cy, set_z + 0.01), C, "over the slot", 0.0),
        W(5.0, (cx, cy, push_z), C, "set it in and push it home", 0.0),
        W(PUSH_HOLD, (cx, cy, push_z), C, "hold", 0.0),
        W(0.8, (cx, cy, seated_z), C, "ease off", 0.0),
        W(2.0, (cx, cy, seated_z), O, "let go", 0.0),
        W(1.0, (cx, cy, over), O, "up", 0.0),
        *(w for x in (sx - PRESS_ENDS, sx + PRESS_ENDS) for w in (
            W(1.5, (x, sy, top + 0.015), C, "over an end", 0.0),
            W(1.0, (x, sy, top - PRESS_DEPTH), C, "press the end", 0.0),
            W(1.0, (x, sy, top - PRESS_DEPTH), C, "hold", 0.0),
            W(1.0, (x, sy, top + 0.015), C, "lift", 0.0))),
        W(1.5, (sx, sy, over), C, "up", 0.0),
    ]
    return {"left": left, "right": right}
