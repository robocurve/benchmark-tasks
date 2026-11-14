"""On the station table stands a robot arm (fixed: switched off for a tool change), its wrist reaching in low over the
table with its tool flange facing the right arm. It has just parked its gripper tool in the tool rack beside it; in the
rack's other slot waits a suction-cup tool. A locking ring sits round the flange, with a lever reaching out toward the
arms; a light on the wrist shows red, unlocked.

Task: change the robot's tool to the suction-cup tool. One hand takes it from the rack by its handle and holds its plate
flat against the flange: the tool falls if let go before it is locked on. The other presses the ring's lever down, at
least 25 degrees: the ring turns and locks the tool on, and the light turns green. Then both hands let go, the tool
hanging on the robot, and both arms go back to where they started. The ring only locks a tool that is already held on
the flange as it turns (a ring turned with no tool on stays shut to it until it is turned back open).

Run it:  inspect-robots run --embodiment yam_arena -E task=lock_a_tool_onto_a_robots_quick_change_flange.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, glue, hinge,  # noqa: E402
                     on_reset, on_step, rotation, segment, set_glue_on, set_hinge_drive)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the robot's wrist: the flange's axis runs along y; its face looks toward -y (the right arm) ---------------------
FX, FZ = 0.30, TOP + 0.085  # the flange's axis
FACE_Y = 0.02  # the flange's face
FLANGE_R = 0.032
RING_R, RING_Y = 0.044, (0.004, 0.022)  # the locking ring, just behind the face (from it, along +y)
LEVER_L = 0.11  # the lever: from the axis toward the arms (-x) ...
PAD_Y = 0.063  # ... then bent along +y to its pad, this far beyond the face
PAD = (0.045, 0.030, 0.008)
PAD_TOP = 0.006 + PAD[2]  # the pad's top above the lever's line
LAMP = np.array([FX, FACE_Y + 0.07, FZ + 0.034 + 0.004])  # the status light, on top of the wrist

# -- the ring turns on a hinge about the axis; pressing the lever down turns it. It stays where it is left: a spring
# holds it where it last stopped, and pushed SLIP past that it gives (the task file moves the hold to it) ----------
RING_HOLD = 0.05  # N m per degree
SLIP = 2.0  # degrees
RING_DRAG = 0.002  # N m per degree/s
RING_MAX = 35.0
OPEN = 8.0  # a tool can go onto the flange only while the ring is within this of open
LOCK = 25.0  # the ring turned this far with a tool on the flange: the tool is locked on
UNLOCK = 15.0  # turned back below this: unlocked

# -- the tools: each lies along y, its plate's face (the body's origin) toward +y, a square handle on its top -----------
PLATE_R, PLATE_T = 0.030, 0.008
HANDLE = (0.016, 0.016, 0.035)
HANDLE_Y, HANDLE_Z = -0.05, 0.020  # the handle's middle along the tool, and its bottom above the axis
TOOL_R = 0.022  # the tools' bodies reach this far from the axis
TOOL_MASS = 0.12
COM_DOWN = -0.005  # each tool's mass sits a little below its axis, under the handle: it lies handle up
SUCTION_AT = np.array([0.30, -0.12, FZ])  # the suction tool's plate face, in the rack
GRIPPER_AT = np.array([0.18, -0.12, FZ])  # the parked gripper tool's
LOCKS = {"suction_tool": "suction_lock", "gripper_tool": "gripper_lock"}
SEAT = np.array([FX, FACE_Y - 0.0005, FZ])  # where a locked tool's plate face is held
ANCHOR_AT = np.array([0.45, -0.30, TOP - 0.05])  # a hidden fixed body the lock holds a tool to

ORANGE, GREY, STEEL, DARK, BLACK, BLUE, RED, GREEN, WHITE = (0.93, 0.50, 0.10), (0.45, 0.46, 0.48), \
    (0.72, 0.73, 0.75), (0.20, 0.21, 0.23), (0.06, 0.06, 0.07), (0.15, 0.35, 0.78), (0.85, 0.12, 0.10), \
    (0.10, 0.85, 0.20), (0.95, 0.95, 0.93)

# -- scoring ----------------------------------------------------------------------------------------------------------
SEAT_GAP = 0.006  # seated: the plate's face within this of the flange's face ...
SEAT_OFF = 0.010  # ... its middle within this of the flange's axis ...
SEAT_TILT = 12.0  # ... its axis within this many degrees of the flange's ...
SEAT_ROLL = 20.0  # ... and turned about it within this many degrees (handle up)
LET_GO = 0.04  # each TCP at least this far from the tool's handle and the lever's pad
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0
HIDDEN = np.array([0.45, -0.25, TOP - 0.05])


def _robot() -> Body:
    """The robot arm, fixed; origin on the flange's axis at its face."""
    o = np.array([FX, FACE_Y, FZ])

    def at(x, y, z):
        return tuple((np.array([x, y, z]) - o).tolist())

    yb, zs = 0.11, TOP + 0.25  # the arm's plane (y) and its shoulder's height
    shapes = [CylShape(at(0.56, yb, TOP + 0.015), 0.07, 0.03, DARK),  # the base
              segment(at(0.56, yb, TOP + 0.03), at(0.56, yb, zs), 0.07, ORANGE),  # the column
              CylShape(at(0.56, yb, zs), 0.045, 0.08, GREY, axis="Y"),
              segment(at(0.56, yb, zs), at(0.40, yb, zs), 0.06, ORANGE),  # the upper arm
              CylShape(at(0.40, yb, zs), 0.04, 0.075, GREY, axis="Y"),
              segment(at(0.40, yb, zs), at(0.40, yb, FZ), 0.055, ORANGE),  # the forearm, down
              CylShape(at(0.40, yb, FZ), 0.036, 0.07, GREY),
              segment(at(0.40, yb, FZ), at(0.30, yb, FZ), 0.05, ORANGE),  # the wrist, toward the arms
              CylShape(at(FX, yb, FZ), 0.034, 0.068, GREY),
              CylShape(at(FX, (yb + FACE_Y + 0.024) / 2, FZ), 0.034, yb - FACE_Y - 0.024, GREY, axis="Y"),
              CylShape((0.0, 0.01, 0.0), FLANGE_R, 0.02, STEEL, axis="Y"),  # the flange, its face at y = 0 here
              CylShape((0.0, -0.0003, 0.0), 0.006, 0.0006, DARK, collide=False, axis="Y"),  # its locating bore
              CylShape(at(*(LAMP + [0.0, 0.0, -0.003])), 0.009, 0.004, DARK, collide=False),
              SphereShape(at(*LAMP), 0.006, RED, collide=False)]  # the status light: red
    return Body.build("robot", pos=tuple(o.tolist()), mass=5.0, shapes=shapes, kinematic=True, color=ORANGE,
                      friction=(0.6, 0.5))


def _anchor() -> Body:
    return Body.build("lock_anchor", pos=tuple(ANCHOR_AT.tolist()), mass=0.01, kinematic=True, color=DARK,
                      shapes=[SphereShape((0.0, 0.0, 0.0), 0.003, DARK, collide=False)])


def _ring() -> Body:
    """The locking ring and its lever; origin on the flange's axis at its face (its mass on the axis)."""
    yc = (RING_Y[0] + RING_Y[1]) / 2
    shapes = [CylShape((0.0, yc, 0.0), RING_R, RING_Y[1] - RING_Y[0], DARK, axis="Y"),
              BoxShape((0.0, RING_Y[0] - 0.0002, RING_R - 0.006), (0.003, 0.0006, 0.010), WHITE, collide=False),
              BoxShape((-(RING_R - 0.004 + LEVER_L) / 2, yc, 0.0), (LEVER_L - RING_R + 0.004, 0.012, 0.012), ORANGE),
              BoxShape((-LEVER_L, (yc + PAD_Y) / 2, 0.0), (0.012, PAD_Y - yc + 0.012, 0.012), ORANGE),
              BoxShape((-LEVER_L, PAD_Y, 0.006 + PAD[2] / 2), PAD, BLACK),
              BoxShape((-LEVER_L - PAD[0] / 2 + 0.003, PAD_Y, PAD_TOP + 0.003), (0.006, PAD[1], 0.006), BLACK)]  # a lip
    return Body.build("ring", pos=(FX, FACE_Y, FZ), mass=0.05, shapes=shapes, friction=(1.0, 0.9), color=DARK,
                      com=(0.0, yc, 0.0))


def _rack() -> Body:
    """Two slots, each a block under a tool's body with a lip either side; origin on the table."""
    shapes = []
    h = FZ - TOOL_R - TOP
    for at in (SUCTION_AT, GRIPPER_AT):
        x, yc = float(at[0]), float(at[1]) - 0.05
        shapes += [BoxShape((x, yc, h / 2), (0.062, 0.04, h), DARK),
                   *[BoxShape((x + s * 0.028, yc, h + 0.005), (0.006, 0.04, 0.010), DARK) for s in (-1.0, 1.0)]]
    shapes.append(BoxShape(((SUCTION_AT[0] + GRIPPER_AT[0]) / 2, float(SUCTION_AT[1]) - 0.05, 0.004),
                           (float(SUCTION_AT[0] - GRIPPER_AT[0]) + 0.08, 0.06, 0.008), GREY))
    return Body.build("rack", pos=(0.0, 0.0, TOP), mass=1.0, shapes=shapes, kinematic=True, color=DARK,
                      friction=(0.6, 0.5))


def _plate_and_handle() -> list:
    return [CylShape((0.0, -PLATE_T / 2, 0.0), PLATE_R, PLATE_T, STEEL, axis="Y"),
            BoxShape((0.0, HANDLE_Y, HANDLE_Z + HANDLE[2] / 2), HANDLE, DARK)]


def _suction_tool() -> Body:
    shapes = [*_plate_and_handle(),
              CylShape((0.0, -0.0465, 0.0), TOOL_R, 0.077, BLUE, axis="Y"),  # the body
              CylShape((0.0, -0.090, 0.0), 0.008, 0.010, DARK, axis="Y"),  # the stem
              CylShape((0.0, -0.100, 0.0), 0.020, 0.010, BLACK, axis="Y")]  # the suction cup
    return Body.build("suction_tool", pos=tuple(SUCTION_AT.tolist()), mass=TOOL_MASS, shapes=shapes,
                      friction=(1.0, 0.9), color=BLUE, com=(0.0, HANDLE_Y, COM_DOWN), ignore=("ring",))


def _gripper_tool() -> Body:
    shapes = [*_plate_and_handle(),
              BoxShape((0.0, -0.040, 0.0), (0.044, 0.064, 0.044), GREY),  # the body
              *[BoxShape((s * 0.014, -0.088, 0.0), (0.008, 0.034, 0.020), DARK) for s in (-1.0, 1.0)]]  # its fingers
    return Body.build("gripper_tool", pos=tuple(GRIPPER_AT.tolist()), mass=TOOL_MASS + 0.02, shapes=shapes,
                      friction=(1.0, 0.9), color=GREY, com=(0.0, HANDLE_Y, COM_DOWN), ignore=("ring",))


def _light() -> Body:
    return Body.build("lock_light", pos=tuple(HIDDEN.tolist()), mass=0.001, kinematic=True, color=GREEN,
                      shapes=[SphereShape((0.0, 0.0, 0.0), 0.0068, GREEN, collide=False)])


OBJECTS = [_robot(), _anchor(), _ring(), _rack(), _suction_tool(), _gripper_tool(), _light()]
# the ring turns about the flange's axis; pressing the lever down turns it the negative way about +y
hinge("lock_ring", "robot", "ring", anchor=(FX, FACE_Y + (RING_Y[0] + RING_Y[1]) / 2, FZ), axis="Y",
      limits_deg=(-RING_MAX, 1.0), spring=RING_HOLD, damping=RING_DRAG, target_deg=0.0)
# the locks: off until the ring locks a tool on (then the tool is held where it sits on the flange)
for _tool, _lock in LOCKS.items():
    glue(_lock, "lock_anchor", _tool, anchor=tuple(ANCHOR_AT.tolist()))


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"held": 0.0, "tcps": None, "s": {}, "light": False,
            "tools": {name: {"inserted": False, "locked": False, "ever": False, "ever_locked": False}
                      for name in LOCKS}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("FLANGE_DEBUG")), "steps": 0}


def _quat_wxyz(m: np.ndarray) -> tuple[float, float, float, float]:
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return (w / n, x / n, y / n, z / n)


def _lock_frames() -> None:
    """Each lock holds its tool with its plate's face on the flange, centred and square (the ring draws it home)."""
    import bodykit
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdPhysics

    for lock in LOCKS.values():
        joint = UsdPhysics.FixedJoint(get_current_stage().GetPrimAtPath(bodykit._GLUE_PATHS[lock]))
        joint.GetLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in SEAT - ANCHOR_AT]))
        joint.GetLocalRot0Attr().Set(Gf.Quatf(*_quat_wxyz(np.eye(3))))
        joint.GetLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.GetLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    _lock_frames()
    for lock in LOCKS.values():
        set_glue_on(lock, False)


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _place(base, origin, name, at) -> None:
    import torch

    asset = base.scene[name]
    p = np.asarray(at, float) + origin
    asset.write_root_pose_to_sim(torch.tensor([[*p.tolist(), 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32,
                                              device=asset.device))


def _seat_error(pos, rot) -> tuple[float, float, float, float]:
    """How far a tool is from sitting on the flange: the gap from its plate's face to the flange's face (m), its
    middle off the axis (m), its axis tilted off the flange's (deg), and its turn about its axis (deg)."""
    gap = FACE_Y - float(pos[1])
    off = math.hypot(float(pos[0]) - FX, float(pos[2]) - FZ)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(rot[1, 1])))))
    roll = math.degrees(math.atan2(float(rot[0, 2]), float(rot[2, 2])))
    return gap, off, tilt, roll


def _seated(pos, rot) -> bool:
    gap, off, tilt, roll = _seat_error(pos, rot)
    return -0.003 <= gap <= SEAT_GAP and off <= SEAT_OFF and tilt <= SEAT_TILT and abs(roll) <= SEAT_ROLL


def _pad_top(turned: float) -> np.ndarray:
    """The middle of the lever pad's top with the ring turned ``turned`` degrees (lever pressed down)."""
    a = math.radians(turned)
    x, z = -LEVER_L, PAD_TOP
    return np.array([FX + x * math.cos(a) - z * math.sin(a), FACE_Y + PAD_Y, FZ + x * math.sin(a) + z * math.cos(a)])


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    rr = rotation(as_numpy(base.scene["ring"].data.root_link_pose_w)[0][3:7])
    turned = -math.degrees(math.atan2(float(rr[0, 2]), float(rr[0, 0])))  # the lever pressed down: positive
    if abs(turned - _EP["held"]) > SLIP:  # pushed past its hold: the ring gives, and stays where it is left
        _EP["held"] = turned
        set_hinge_drive("lock_ring", target_deg=-turned)
    poses = {}
    for name, lock in LOCKS.items():
        t = as_numpy(base.scene[name].data.root_link_pose_w)[0]
        pos, rot = t[:3] - origin, rotation(t[3:7])
        poses[name] = (pos, rot)
        st = _EP["tools"][name]
        seated = _seated(pos, rot)
        if st["locked"]:
            if turned < UNLOCK:  # the ring turned back open: the tool is free
                set_glue_on(lock, False)
                st["locked"] = st["inserted"] = False
        else:
            if seated and turned < OPEN:  # on the flange with the ring open: the ring can take it
                st["inserted"] = True
            elif not seated:
                st["inserted"] = False
            if st["inserted"] and turned >= LOCK:
                set_glue_on(lock, True)
                st["locked"] = st["ever_locked"] = True
        st["ever"] = st["ever"] or st["inserted"]
    locked = any(st["locked"] for st in _EP["tools"].values())
    if locked != _EP["light"]:
        _place(base, origin, "lock_light", LAMP if locked else HIDDEN)
        _EP["light"] = locked
    _EP["tcps"] = _tcps(base, origin)
    pos, rot = poses["suction_tool"]
    handle = pos + rot @ np.array([0.0, HANDLE_Y, HANDLE_Z + HANDLE[2] / 2])
    pad = _pad_top(turned)
    clear = all(float(np.linalg.norm(t - q)) >= LET_GO for t in _EP["tcps"] for q in (handle, pad))
    _EP["s"] = {"turned": turned, "clear": clear}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = _EP["tcps"]
            gap, off, tilt, roll = _seat_error(pos, rot)
            st = _EP["tools"]["suction_tool"]
            print(f"[flange] t={_DEBUG['steps'] / 20:5.1f}s ring={turned:5.1f}deg suction: gap={gap * 1000:6.1f}mm "
                  f"off={off * 1000:5.1f}mm tilt={tilt:5.1f} roll={roll:6.1f} inserted={st['inserted']} "
                  f"locked={st['locked']} gripper_tool_locked={_EP['tools']['gripper_tool']['locked']} clear={clear} "
                  f"rightTCP-handle={np.round((tr - handle) * 1000).astype(int).tolist()} "
                  f"leftTCP-pad={np.round((tl - pad) * 1000).astype(int).tolist()} score={_points(_status())}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    tools = _EP["tools"]
    return {"locked": tools["suction_tool"]["locked"], "wrong": tools["gripper_tool"]["ever_locked"],
            "ever": tools["suction_tool"]["ever"], "clear": bool(_EP["s"].get("clear"))}


def _points(s) -> float:
    if s["wrong"]:
        return 0.0
    if s["locked"] and s["clear"]:
        return 1.0
    if s["locked"]:
        return 0.5
    return 0.25 if s["ever"] else 0.0


def score(objects) -> float:
    """0.0 if the gripper tool was ever locked on. Else 1.0 with the suction tool locked on the flange, both hands let go
    (each TCP at least 4 cm from its handle and the lever's pad) and both arms back where they started; 0.5 with it
    locked on otherwise; 0.25 if it was ever held on the flange (seated, with the ring open); else 0.0."""
    pts = _points(_status())
    if pts >= 1.0 and not _arms_home():
        return 0.5  # locked on and let go, but the arms not back
    return pts


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
    """Full score with the suction tool still (hanging on the robot): the trial ends there."""
    return score(objects) >= 1.0 and objects["suction_tool"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right hand comes straight down on the suction tool's handle, fingers closing across it, lifts it out of the rack,
# carries it to the flange and holds its plate flat against the flange. The left hand comes straight down on the
# lever's pad, fingers closed, and presses it down along its swing, 32 degrees: the ring turns and locks the tool on
# (the light turns green). The right lets go and draws back, the left rises, the tool hanging on the robot, and both
# arms go back to where they started.

TOOL_GRIP = grip_rotation(math.pi / 2)  # straight down, fingers closing along x, across the handle
PAD_GRIP = grip_rotation(0.0)  # straight down, fingers closed (closing along y)
IN_HANDLE = 0.013  # the right TCP this far below the handle's top
LIFT = 0.018  # out of the rack (its lips are 1 cm high)
SHORT = 0.02  # the plate brought level with the flange this far short of it, then on
PUSH_IN = 0.001  # aimed this far into the flange
TIP = 0.013  # closed fingertips below the TCP
PRESS = 0.005  # aimed this far into the pad
TURN_TO = 32.0
LEAD_S, LEAD_STEPS = 3.0, 15


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


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _go_home(arm: str, pos, rot, finger) -> list[Waypoint]:
    """From a pose back to the arm's start, straight through joint space (the lead-in, reversed)."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = _good_ik(arm, pos, rot), np.array(READY, float)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, "back to where it started", r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _in_hand(plate_face) -> np.ndarray:
    """The right TCP holding a tool by its handle, the tool's plate face at ``plate_face``."""
    return np.asarray(plate_face, float) + [0.0, HANDLE_Y, HANDLE_Z + HANDLE[2] - IN_HANDLE]


def _right(press_done: float) -> tuple[list[Waypoint], float]:
    pick = _in_hand(SUCTION_AT)
    out = _lead_in("right", pick + [0.0, 0.0, 0.04], TOOL_GRIP, "over the suction tool's handle", FINGER_OPEN)
    out += [_w(1.0, pick, FINGER_OPEN, "down round it", TOOL_GRIP),
            _w(0.6, pick, FINGER_CLOSED, "take it", TOOL_GRIP),
            _w(1.0, pick + [0.0, 0.0, LIFT], FINGER_CLOSED, "lift it out of the rack", TOOL_GRIP)]
    on = _in_hand(SEAT + [0.0, PUSH_IN, 0.0])
    out += [_w(1.6, on + [0.0, -SHORT, LIFT], FINGER_CLOSED, "carry it to the flange, its plate facing it", TOOL_GRIP),
            _w(0.8, on + [0.0, -SHORT, 0.0], FINGER_CLOSED, "level with the flange", TOOL_GRIP),
            _w(0.8, on, FINGER_CLOSED, "its plate flat against the flange", TOOL_GRIP)]
    seated_at = sum(w.duration for w in out)
    back = on + [0.0, -0.03, 0.04]
    out += [_w(max(press_done + 0.8 - seated_at, 0.3), on, FINGER_CLOSED, "hold it on while the lever is pressed",
               TOOL_GRIP),
            _w(0.5, on, FINGER_OPEN, "let go: it hangs on the robot, locked", TOOL_GRIP),
            _w(1.0, back, FINGER_OPEN, "draw back", TOOL_GRIP)]
    out += _go_home("right", back, TOOL_GRIP, FINGER_OPEN)
    return out, seated_at


def _on_pad(turned: float, press: float) -> np.ndarray:
    return _pad_top(turned) + [0.0, 0.0, TIP - press]


def _left(seated_at: float, total: float) -> tuple[list[Waypoint], float]:
    over = _on_pad(0.0, -0.04)
    out = _lead_in("left", over, PAD_GRIP, "over the lever's pad", FINGER_CLOSED)
    t = sum(w.duration for w in out)
    out += [_w(max(seated_at + 0.4 - t, 0.3), over, FINGER_CLOSED, "wait for the tool to be held on the flange",
               PAD_GRIP),
            _w(0.8, _on_pad(0.0, 0.0), FINGER_CLOSED, "down onto the pad", PAD_GRIP)]
    for k in range(1, 9):
        out.append(_w(0.5, _on_pad(TURN_TO * k / 8, PRESS), FINGER_CLOSED,
                      "press the lever down: the ring turns and locks the tool on", PAD_GRIP))
    press_done = sum(w.duration for w in out)
    up = _on_pad(TURN_TO, -0.05)
    out += [_w(0.8, _on_pad(TURN_TO, PRESS), FINGER_CLOSED, "hold it there", PAD_GRIP),
            _w(1.0, up, FINGER_CLOSED, "rise", PAD_GRIP)]
    out += _go_home("left", up, PAD_GRIP, FINGER_CLOSED)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out[-1].pos, FINGER_CLOSED,
                  "hold still: the trial ends once the full score has held 1 s with both arms home", out[-1].yaw))
    return out, press_done


def demo():
    _, seated_at = _right(0.0)
    _, press_done = _left(seated_at, 0.0)
    right, _ = _right(press_done)
    total = sum(w.duration for w in right) + 3.0
    right.append(_w(3.0, right[-1].pos, FINGER_OPEN, "wait at the start", right[-1].yaw))
    left, _ = _left(seated_at, total)
    return {"left": left, "right": right}
