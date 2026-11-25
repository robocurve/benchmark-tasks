"""On the station table: a short section of an installed hollow wall stands upright, its face toward the robot: grey
12.7 mm wallboard on both sides of a wooden 2x3 frame, with a closed cavity inside and a plumb pencil line drawn down
its face. At a prepared 12.7 mm hole on the line, low on the wall, a steel coat hook hangs loose on a 3/16 in (10-24)
spring-wing toggle bolt with a hex washer head; the toggle's wings have opened in the cavity behind the board. In
front of the wall a stubby ratcheting nut driver stands socket-down in a bench holder.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402
import tilt  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

BOARD_GREY, WOOD, PENCIL = (0.80, 0.80, 0.78), (0.78, 0.62, 0.40), (0.20, 0.20, 0.22)
STEEL, ZINC, DARK = (0.55, 0.57, 0.60), (0.74, 0.75, 0.77), (0.15, 0.15, 0.16)

# --- the wall section (fixed), standing on the table with its face toward the arms ----------------------------------
# Wall frame (used for the panel and the hook): x along the wall to the robot's left (world +y), y straight down the
# wall (world -z), z out of the face toward the robot (world -x). Its origin is the hole's centre on the face.
WALL_ROT = np.array([[0.0, 0.0, -1.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])  # wall frame -> world
WALL_QUAT = tuple(float(v) for v in parts._matrix_to_quat(WALL_ROT))
OUT = WALL_ROT[:, 2].copy()  # world direction straight out of the face: (-1, 0, 0)
FACE_X = 0.28  # the board's face, in front of the line through the arm bases
HOLE_Y = -0.06  # the hole: 6 cm right of the midline
HOLE_H = 0.150  # and 150 mm above the table
WALL_W, WALL_H = 0.205, 0.300
BOARD_T = 0.0127
STUD_D = 0.0635  # 2x3 stud depth: the cavity's depth
STUD_T = 0.038
HOLE_R = 0.00635
CUT = 0.015  # half side of the square opening the octagonal hole ring fills
WALL_X = (-0.135, 0.070)  # wall frame: across; the hole 7 cm from the left edge, 13.5 cm from the right
WALL_Y = (HOLE_H - WALL_H, HOLE_H)  # wall frame: down (the table is at y = HOLE_H)


def _slab(x0, x1, y0, y1, z0, z1, color, **kw):
    return parts.box((x1 - x0, y1 - y0, z1 - z0), at=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), color=color, **kw)


def _panel_shapes():
    (x0, x1), (y0, y1) = WALL_X, WALL_Y
    front = (-BOARD_T, 0.0)
    cav = (-BOARD_T - STUD_D, -BOARD_T)
    back = (-2 * BOARD_T - STUD_D, -BOARD_T - STUD_D)
    out = [  # frame: bottom plate on the table, top plate, two studs
        _slab(x0, x1, y1 - STUD_T, y1, *cav, WOOD), _slab(x0, x1, y0, y0 + STUD_T, *cav, WOOD),
        _slab(x0, x0 + STUD_T, y0 + STUD_T, y1 - STUD_T, *cav, WOOD),
        _slab(x1 - STUD_T, x1, y0 + STUD_T, y1 - STUD_T, *cav, WOOD),
        _slab(x0, x1, y0, y1, *back, BOARD_GREY),  # the back board closes the cavity
        # the front board round a square opening at the hole
        _slab(x0, x1, y0, -CUT, *front, BOARD_GREY), _slab(x0, x1, CUT, y1, *front, BOARD_GREY),
        _slab(x0, -CUT, -CUT, CUT, *front, BOARD_GREY), _slab(CUT, x1, -CUT, CUT, *front, BOARD_GREY),
    ]
    depth = 0.016  # eight tangent slabs fill the opening and leave an octagonal hole, 12.7 mm across flats
    width = 2 * math.tan(math.pi / 8) * (HOLE_R + depth)
    for i in range(8):
        a = i * math.pi / 4
        c = HOLE_R + depth / 2
        out.append(parts.box((depth, width, BOARD_T), at=(c * math.cos(a), c * math.sin(a), -BOARD_T / 2),
                             color=BOARD_GREY, yaw=a))
    # the plumb pencil line down the face through the hole (drawn, no collider)
    out.append(_slab(-0.0015, 0.0015, y0 + 0.02, y1 - 0.02, 0.0, 0.0003, PENCIL, collide=False))
    return out


PANEL = parts.Part("panel", pos=(FACE_X, HOLE_Y, TOP + HOLE_H), quat=WALL_QUAT, shapes=_panel_shapes(),
                   contact_offset=0.002, compliance=(5.0e4, 60.0))  # wallboard paper gives a little under load

# --- the hook on its toggle bolt (loose): hook frame = wall frame, origin on the face at the bolt's axis --------------
PLATE = (0.040, 0.125, 0.003)  # across, down, thickness; the bolt is 40 mm from the plate's top end
PLATE_Y = (-0.040, 0.085)
ROD_R, ROD_L = 0.004, 0.075  # the hook's arm: an 8 mm steel rod straight out from the plate
ROD_Y = 0.080  # 80 mm below the bolt
LIP = 0.018  # the rod's end turns up 14 mm
LEAD = 0.0254 / 24  # 3/16 in (10-24) thread: 1.06 mm per turn
GAP0 = 0.003  # the wings start 3 mm behind the board's back face (about 2.8 turns to go)
# Hex washer head, as on 10-24 hex washer head machine screws: a 5/16 in (7.9 mm) hex 2.4 mm tall on a washer face
# 10.3 mm across and 0.6 mm thick (3.0 mm, 0.118 in, overall).
HEX_F, HEX_H = 0.00794, 0.0024
WASHER_R, WASHER_T = 0.00515, 0.0006
HEAD_H = WASHER_T + HEX_H
SHANK_R, SHANK_L = 0.0024, 0.0635  # 3/16 in (4.8 mm) x 2-1/2 in
WING = (0.048, 0.011, 0.006)  # opened wing span (across the wall), width, thickness
NUT_Z = -BOARD_T - GAP0 - WING[2] / 2  # nut centre, hook frame
BACK = 0.010  # the nut's travel range back from its start (it never gets there: see the thread lock below)
SAG = HOLE_R - SHANK_R - 0.0002  # the loose bolt hangs on the bottom of the hole


def _head_shapes():
    z0 = PLATE[2]
    side = HEX_F / math.sqrt(3)  # the hexagon's side: three crossed bars make it
    out = [parts.cylinder(WASHER_R, WASHER_T, at=(0, 0, z0 + WASHER_T / 2), color=ZINC)]
    out += [parts.box((side, HEX_F, HEX_H), at=(0, 0, z0 + WASHER_T + HEX_H / 2), color=ZINC, yaw=k * math.pi / 3)
            for k in range(3)]
    out.append(parts.cylinder(SHANK_R, SHANK_L, at=(0, 0, z0 - SHANK_L / 2), color=ZINC))
    return out


BOLT = parts.Hinge("bolt", shapes=_head_shapes(), pivot=(0.0, 0.0, PLATE[2]), axis=(0.0, 0.0, 1.0), lower=-250.0,
                   upper=250.0, mass=0.012, damping=0.002)  # no drive; light drag: the head turning on the plate
THREAD = parts.Slider("thread", shapes=[parts.box((0.004, 0.004, 0.004), at=(0, 0, NUT_Z), color=ZINC,
                                                  collide=False)],
                      axis=(0.0, 0.0, 1.0), travel=GAP0 + 0.0005, back=BACK, mass=0.01, damping=0.0,
                      parent="bolt", thread=("nut", LEAD))
NUT = parts.Hinge("nut", shapes=[parts.box(WING, at=(0, 0, NUT_Z), color=ZINC),
                                 parts.box((0.016, 0.008, 0.010), at=(0, 0, NUT_Z - 0.002), color=DARK)],
                  pivot=(0.0, 0.0, NUT_Z), axis=(0.0, 0.0, 1.0), lower=-250.0, upper=250.0, mass=0.010,
                  damping=0.002, parent="thread")  # damping: thread friction
HOOK_POS = (FACE_X, HOLE_Y, TOP + HOLE_H - SAG)  # hanging flat on the face from the bottom of the hole
LOCK_SLACK = 0.00002  # m the nut may back off from the furthest it has been drawn up the thread
HOOK = parts.Part(
    "hook", pos=HOOK_POS, quat=WALL_QUAT, mass=0.15, contact_offset=0.0005, self_collision=False,
    shapes=[
        _slab(-PLATE[0] / 2, PLATE[0] / 2, *PLATE_Y, 0.0, PLATE[2], STEEL),
        parts.cylinder(ROD_R, ROD_L, at=(0.0, ROD_Y, PLATE[2] + ROD_L / 2), color=STEEL),
        # the rod's end turns up (wall frame -y)
        parts.cylinder(ROD_R, LIP, at=(0.0, ROD_Y - LIP / 2 + ROD_R, PLATE[2] + ROD_L - ROD_R), color=STEEL,
                       axis="y"),
    ],
    joints=[BOLT, THREAD, NUT],
)

# --- the screwdriver (loose) and its bench holder (fixed) -------------------------------------------------------------
# A stubby ratcheting 5/16 in nut driver: a steel socket 13 mm across, its round-cornered bore 10 mm across and 4 mm
# deep (it slides over the hex's 9.2 mm corners with 0.4 mm to spare, and stops on the washer face) with a 45 degree
# lead-in at the mouth opening it to 12 mm, then a 6 mm shaft to the handle.
SOCKET_R, BORE_R_IN, MOUTH_R = 0.0065, 0.0050, 0.0060
BORE_D, LEAD_IN = 0.004, 0.001
SOCKET_L = 0.010
SHAFT_R = 0.003
HANDLE_Z0 = 0.038  # handle's lower end above the socket's mouth
SHAFT_L = HANDLE_Z0 - SOCKET_L


def _socket_shapes():
    out = [parts.cylinder(SOCKET_R, SOCKET_L - BORE_D, at=(0, 0, (BORE_D + SOCKET_L) / 2), color=ZINC)]
    n = 12
    for i in range(n):
        a = i * 2 * math.pi / n
        u = np.array([math.cos(a), math.sin(a), 0.0])
        t = np.array([-math.sin(a), math.cos(a), 0.0])
        w = 2 * SOCKET_R * math.tan(math.pi / n) + 0.0003
        # the wall: full thickness above the lead-in, a thin rim beside it
        rm = (BORE_R_IN + SOCKET_R) / 2
        out.append(parts.box((SOCKET_R - BORE_R_IN, w, BORE_D - LEAD_IN),
                             at=tuple(rm * u + [0, 0, (LEAD_IN + BORE_D) / 2]), color=ZINC, yaw=a))
        rr = (MOUTH_R + SOCKET_R) / 2
        out.append(parts.box((SOCKET_R - MOUTH_R, w, LEAD_IN), at=tuple(rr * u + [0, 0, LEAD_IN / 2]), color=ZINC,
                             yaw=a))
        # the lead-in: a wedge whose inner face runs from the mouth (r 6 mm) to the bore (r 5 mm) at 45 degrees
        down = (-u + np.array([0.0, 0.0, 1.0])) / math.sqrt(2)  # along the face, into the socket
        into = (u + np.array([0.0, 0.0, 1.0])) / math.sqrt(2)  # into the metal
        h = 0.0005
        mid = (MOUTH_R + BORE_R_IN) / 2 * u + np.array([0.0, 0.0, LEAD_IN / 2])
        rot = np.column_stack([t, down, np.cross(t, down)])
        if np.dot(rot[:, 2], into) < 0:
            rot = np.column_stack([-t, down, np.cross(-t, down)])
        out.append(parts.box((w, LEAD_IN * math.sqrt(2), h), at=tuple(mid + into * h / 2), color=ZINC, rot=rot))
    out.append(parts.cylinder(SHAFT_R, SHAFT_L, at=(0, 0, SOCKET_L + SHAFT_L / 2), color=ZINC))
    return out


HANDLE_R, HANDLE_L = 0.0175, 0.050
DRIVER_H = HANDLE_Z0 + HANDLE_L  # 88 mm
HOLDER = (0.050, 0.050, 0.045)
HOLDER_XY = (0.17, -0.24)
BORE_R = 0.007  # holder bore, 14 mm across flats
# The ratchet: the shaft drives clockwise and freewheels in the handle the other way (the pawl is the shaft joint's
# upper limit, moved down after it every control step).
RATCHET_SLACK = 0.02  # rad of play at the pawl
SHAFT = parts.Hinge(
    "shaft", pivot=(0.0, 0.0, 0.0), axis=(0.0, 0.0, 1.0), lower=-80.0, upper=RATCHET_SLACK, mass=0.015,
    damping=1e-4, shapes=_socket_shapes())
DRIVER = parts.Part(
    "screwdriver", pos=(*HOLDER_XY, TOP + HOLDER[2] - HANDLE_Z0), mass=0.075, contact_offset=0.0003,
    self_collision=False,
    shapes=[parts.cylinder(HANDLE_R, HANDLE_L, at=(0, 0, HANDLE_Z0 + HANDLE_L / 2), color=(0.85, 0.15, 0.12)),
            parts.cylinder(0.006, 0.004, at=(0, 0, DRIVER_H + 0.002), color=(0.12, 0.12, 0.13))],  # ratchet cap
    joints=[SHAFT],
)
_pawl = {"upper": None}


@parts.on_step
def _ratchet(sim) -> None:
    """The pawl: the shaft may turn anticlockwise in the handle (q falling) but not clockwise past where it has been."""
    import torch

    q = sim.states()["screwdriver"].joints["shaft"]
    up = min(RATCHET_SLACK if _pawl["upper"] is None else _pawl["upper"], q + RATCHET_SLACK)
    if _pawl["upper"] is not None and up >= _pawl["upper"] - 1e-4:
        return
    _pawl["upper"] = up
    base = sim.env.unwrapped
    tool = base.scene["screwdriver"]
    ids = tool.find_joints(["shaft"], preserve_order=True)[0]
    lim = torch.tensor([[[SHAFT.lower, up]]], dtype=torch.float32, device=base.device)
    tool.write_joint_position_limit_to_sim_index(limits=lim, joint_ids=ids, warn_limit_violation=False)


@parts.on_pre_reset
@parts.on_reset
def _ratchet_reset(sim) -> None:
    import torch

    _pawl["upper"] = RATCHET_SLACK
    base = sim.env.unwrapped
    tool = base.scene["screwdriver"]
    ids = tool.find_joints(["shaft"], preserve_order=True)[0]
    lim = torch.tensor([[[SHAFT.lower, RATCHET_SLACK]]], dtype=torch.float32, device=base.device)
    tool.write_joint_position_limit_to_sim_index(limits=lim, joint_ids=ids, warn_limit_violation=False)
    # the shaft back to where it starts in the handle (the scene reset puts the driver back but not this joint)
    zero = torch.zeros((1, 1), dtype=torch.float32, device=base.device)
    tool.write_joint_state_to_sim_index(position=zero, velocity=zero, joint_ids=ids)


_lock = {"lower": None}


def _thread_limits(sim, lower: float) -> None:
    import torch

    base = sim.env.unwrapped
    hook = base.scene["hook"]
    ids = hook.find_joints(["thread"], preserve_order=True)[0]
    lim = torch.tensor([[[float(lower), THREAD.upper]]], dtype=torch.float32, device=base.device)
    hook.write_joint_position_limit_to_sim_index(limits=lim, joint_ids=ids, warn_limit_violation=False)
    _lock["lower"] = lower


@parts.on_step
def _thread_lock(sim) -> None:
    """Thread friction holds a tightened screw: a real 10-24 thread does not turn back under the clamp load, while
    the simulator's thread joint has no load-dependent friction and would. So the nut may not back off from the
    furthest it has been drawn up the bolt (less 0.02 mm): the joint's lower limit follows it every control step.
    Turning the screw forward is unaffected, and a toggle spinning with the screw does not move along it."""
    travel = sim.states()["hook"].joints["thread"]
    if _lock["lower"] is None or travel - LOCK_SLACK > _lock["lower"] + 1e-5:
        _thread_limits(sim, max(travel - LOCK_SLACK, THREAD.lower))


@parts.on_pre_reset
@parts.on_reset
def _thread_lock_reset(sim) -> None:
    _thread_limits(sim, THREAD.lower)


# The socket on the hex. A socket seated over a hex head turns it; the simulator's round-cornered bore cannot, so a rule
# stands in for the hex's grip: while the socket is seated (its axis within 12 degrees of the bolt's, its mouth within
# 1.5 mm of the bolt's axis and at least 2 mm down over the hex), a torque on the bolt makes it follow the socket's
# clockwise (tightening) turn (a stiff spring, 3 N m/rad, from the bolt to where the socket has turned it), and the
# equal and opposite torque goes onto the driver's handle, which the hand holds and the ratchet's pawl ties to the
# socket when driving, so the hand feels the screw's resistance. At most SOCKET_MAX passes (past that the socket slips
# round the head). The socket's anticlockwise back swing (the ratchet freewheeling) does not drive the screw back.
# Off the head nothing links the screw to anything but the plate's light drag.
SEAT_TILT, SEAT_OFF, SEAT_DEPTH = 12.0, 0.0015, 0.002
SOCKET_K, SOCKET_MAX = 3.0, 0.35  # N m/rad, N m
# The screw's own resistance: a 10-24 screw drawing a toggle tight takes about 0.3 N m at snug, little while loose.
# The simulated thread has no friction, so any torque past snug would become an axial force far beyond a real one.
# Thread and head friction stand in as a torque against the socket's: nothing while the nut is short of the point
# where a flush plate meets the wings, and from 0.1 mm before it, all of the socket's torque but SNUG_NET (which draws
# the clamp up to about 300 N through the frictionless thread, as a snug 0.3 N m does on a real one). Its reaction goes
# onto the plate, as head friction would. So the hand turns the driver against very little while the toggle is loose
# and up to SOCKET_MAX once it is snug.
SNUG_NET, SNUG_RAMP = 0.05, 0.0001
_drive = {"prev": None, "target": None, "q": None, "socket": 0.0, "resist": 0.0}


def socket_seated(hook, driver) -> bool:
    bp, bq = hook.links["bolt"]
    sp, sq = driver.links["shaft"]
    rb, rs = _rot(bq), _rot(sq)
    ab, ad = rb[:, 2], rs[:, 2]
    if float(np.dot(ab, ad)) < math.cos(math.radians(SEAT_TILT)):
        return False
    rel = np.asarray(sp) - np.asarray(bp)
    lateral = rel - np.dot(rel, ab) * ab
    hex_top = HEAD_H  # the hex's top face above the bolt link's origin (the plate's front face), along the axis
    return bool(np.linalg.norm(lateral) <= SEAT_OFF + EPS and hex_top - float(np.dot(rel, ab)) >= SEAT_DEPTH - EPS)


def resist_torque(travel: float, socket: float) -> float:
    """The screw's friction torque (N m, about its axis) against the socket's tightening torque (socket < 0), for the
    nut's travel (m)."""
    near = float(np.clip((travel - (GAP0 - SNUG_RAMP)) / SNUG_RAMP, 0.0, 1.0))
    return near * max(0.0, -socket - SNUG_NET)


@parts.on_step
def _socket_drive(sim) -> None:
    st = sim.states()
    hook, driver = st["hook"], st["screwdriver"]
    q = hook.joints["bolt"]
    rs = _rot(driver.links["shaft"][1])
    prev, _drive["prev"] = _drive["prev"], rs
    _drive["q"] = q
    torque = 0.0
    if prev is not None and socket_seated(hook, driver):
        d = prev.T @ rs  # the socket's turn this step, about its own axis
        turn = min(0.0, math.atan2(d[1, 0] - d[0, 1], d[0, 0] + d[1, 1]))  # clockwise only
        base = q if _drive["target"] is None else _drive["target"]
        lim = SOCKET_MAX / SOCKET_K
        _drive["target"] = float(np.clip(base + turn, q - lim, q))
        torque = SOCKET_K * (_drive["target"] - q)
    else:
        _drive["target"] = None
    _drive["socket"] = float(torque)
    _drive["resist"] = resist_torque(hook.joints["thread"], _drive["socket"])


@parts.on_reset
def _socket_drive_reset(sim) -> None:
    _drive.update(prev=None, target=None, q=None, socket=0.0, resist=0.0)


def _holder_shapes():
    w, d, h = HOLDER
    c, black = BORE_R + 0.006, (0.22, 0.22, 0.24)
    out = [_slab(-w / 2, w / 2, -d / 2, -c, 0, h, black), _slab(-w / 2, w / 2, c, d / 2, 0, h, black),
           _slab(-w / 2, -c, -c, c, 0, h, black), _slab(c, w / 2, -c, c, 0, h, black)]
    depth = 0.007
    width = 2 * math.tan(math.pi / 8) * (BORE_R + depth)
    for i in range(8):
        a = i * math.pi / 4 + math.pi / 8
        r = BORE_R + depth / 2
        out.append(parts.box((depth, width, h), at=(r * math.cos(a), r * math.sin(a), h / 2), color=black, yaw=a))
    return out


HOLDER_PART = parts.Part("driver_holder", pos=(*HOLDER_XY, TOP), shapes=_holder_shapes())

OBJECTS = [PANEL, HOOK, DRIVER, HOLDER_PART]

# --- scoring: checks a fitter can make with simple tools ---------------------------------------------------------------
# - flush: a 1 mm feeler gauge (or a 1 mm card) cannot be pushed under any edge of the plate;
# - tug test: a spring scale hooked on the hook's arm and pulled straight out from the wall to 20 N, held 0.5 s, then let
#   off: no edge of the plate comes 1 mm off the wall during the pull, and afterwards the plate is where it was (both
#   ends within 1 mm, as pencil ticks at its ends would show);
# - square: the plate's long edge within 3 degrees of the plumb line (a protractor), hook end down;
# - let go: nothing within 5 mm of the hook (the video).
EPS = 1e-6
GAUGE = 0.001  # the feeler gauge: 1 mm
SQUARE_DEG = 3.0
CROOKED_DEG = 10.0
CLEAR = 0.005
PULL_N = 20.0  # the spring scale's reading
PULL_AT = (0.0, ROD_Y, PLATE[2] + ROD_L - 2 * ROD_R)  # hook frame: on the arm, just inside its upturned end
PULL_STEPS = 10  # 0.5 s at 20 Hz
SETTLE_STEPS = 10  # 0.5 s
MOVE_MAX = 0.001  # after the tug: each end of the plate within 1 mm of where it was
STILL_STEPS = 20  # the hook at rest for 1 s (20 control steps) in a row before the test

# Released: in the simulator an on_step callback measures every step how close both arms' fingers and grippers (every
# face of their collision hulls, sampled about 3 mm apart) come to the hook (the screwdriver may stay in a hand). Until it
# has measured (or when it cannot), "clear" is None and nothing scores. (Kept in the parts module so near_misses.py,
# which imports this file separately, can set it for offline cases.)
_grip = parts.__dict__.setdefault("_hook_task_grip", {"clear": None})
# The tug test's progress, read by the on_step callback that applies the pull (also kept in the parts module).
_pull = parts.__dict__.setdefault("_hook_task_pull", {"phase": "idle"})
# Control steps in a row with the hook at rest, and whether a gripper has come within 5 mm of the hook yet (the test is
# run on the hook once the robot has handled it and let it go). Both reset with the scene.
# (Kept in the parts module, like _grip, so near_misses.py can set "handled" offline.)
_record = parts.__dict__.setdefault("_hook_task_record", {"still": 0, "handled": False})


def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(q)


def _hook_boxes(hook):
    poses = {"base": (np.asarray(hook.pos), _rot(hook.quat_xyzw))}
    poses.update({k: (np.asarray(p), _rot(q)) for k, (p, q) in hook.links.items()})
    return HOOK.world_boxes(poses)


def _plate_corners(hook) -> np.ndarray:
    """World corners of the plate's back face (the face against the wall)."""
    r, p = _rot(hook.quat_xyzw), np.asarray(hook.pos)
    return np.array([p + r @ np.array([sx * PLATE[0] / 2, y, 0.0]) for sx in (-1, 1) for y in PLATE_Y])


def _plate_ends(hook) -> np.ndarray:
    """World centres of the plate's top and bottom ends, on its front face (where pencil ticks would go)."""
    r, p = _rot(hook.quat_xyzw), np.asarray(hook.pos)
    return np.array([p + r @ np.array([0.0, y, PLATE[2]]) for y in PLATE_Y])


def _gaps(hook) -> np.ndarray:
    """Each corner's distance out from the wall's face (m; negative = pressed into the springy board)."""
    return (_plate_corners(hook) - np.array([FACE_X, 0.0, 0.0])) @ OUT


def _flush(hook) -> bool:
    """A 1 mm feeler gauge cannot go under any edge: every corner of the plate's back within 1 mm of the face."""
    return bool(np.all(np.abs(_gaps(hook)) < GAUGE - EPS))


def _off_plumb_deg(hook) -> float:
    """Angle between the plate's long axis (bolt end to hook end) and the plumb line (straight down), degrees."""
    axis = _rot(hook.quat_xyzw) @ np.array([0.0, 1.0, 0.0])
    return math.degrees(math.atan2(math.hypot(axis[0], axis[1]), -axis[2]))


def _driver_clear(objects) -> bool:
    d = objects["screwdriver"]
    lo, hi = np.asarray(d.low), np.asarray(d.high)
    for _, c, r, s in _hook_boxes(objects["hook"]):
        half = np.abs(r) @ (np.asarray(s) / 2)
        gap = np.maximum(np.maximum(lo - (c + half), (c - half) - hi), 0.0)
        if np.linalg.norm(gap) < CLEAR - EPS:
            return False
    return True


def _count_still(objects) -> None:
    """Once per control step (from score()): consecutive steps with the hook at rest."""
    _record["still"] = _record["still"] + 1 if objects["hook"].resting() else 0


def _ready(objects) -> bool:
    """Let go, the screwdriver clear, the plate flush on the face and within 10 degrees of plumb, hook end down."""
    hook = objects["hook"]
    return bool(_grip["clear"] is True and _flush(hook) and _driver_clear(objects)
                and _off_plumb_deg(hook) <= CROOKED_DEG + EPS)


def _moved(hook, ref) -> float:
    return float(np.max(np.linalg.norm(_plate_ends(hook) - ref, axis=1)))


def _pull_step(objects) -> None:
    """Once per control step (from score()): run the tug test. It starts when the robot has handled the hook (a
    gripper within 5 mm of it at some step) and the hook is ready and has been still for 1 s, and starts over whenever anything touches or moves the hook (a gripper within 5 mm, the screwdriver
    near it, the plate not back where it was after the test)."""
    hook = objects["hook"]
    p = _pull
    if _grip["clear"] is False:
        _record["handled"] = True
    if p["phase"] == "idle":
        if _record["handled"] and _ready(objects) and _record["still"] >= STILL_STEPS:
            p.update(phase="pull", k=0, ref=_plate_ends(hook), gap=-1.0, passed=False, pulled=0)
        return
    if p["phase"] in ("pull", "settle"):
        if _grip["clear"] is not True or not _driver_clear(objects):
            p.update(phase="idle")
            return
        p["gap"] = max(p["gap"], float(np.max(_gaps(hook))))
        if p["phase"] == "pull":  # fail closed: count the steps on which the force was really set to PULL_N
            if "offline_applied" in p:  # near_misses.py stands in for the simulator's wrench offline
                p["applied"] = p["offline_applied"](objects)
            p["pulled"] += int(abs(float(p.get("applied") or 0.0) - PULL_N) < 1e-6)
        p["k"] += 1
        if p["phase"] == "pull" and p["k"] >= PULL_STEPS:
            p.update(phase="settle", k=0)
        elif p["phase"] == "settle" and p["k"] >= SETTLE_STEPS:
            back = _moved(hook, p["ref"]) <= MOVE_MAX + EPS and hook.resting()
            held = p["gap"] < GAUGE - EPS
            p.update(phase="done", passed=bool(back and held and p["pulled"] >= PULL_STEPS - 1))
            print(f"[tug test] max gap {p['gap'] * 1000:.3f} mm, moved {_moved(hook, p['ref']) * 1000:.3f} mm, "
                  f"force on {p['pulled']} of {PULL_STEPS} steps: {'pass' if p['passed'] else 'fail'}", flush=True)
        return
    if not (_ready(objects) and hook.resting() and _moved(hook, p["ref"]) <= MOVE_MAX + EPS):  # done, then disturbed
        p.update(phase="idle")


def _done(objects) -> float:
    """1.0 tight (passed the tug test) and square, 0.5 tight but 3 to 10 degrees off plumb, else 0; the hook
    released, at rest and where the test left it."""
    hook = objects["hook"]
    if _pull["phase"] != "done" or not _pull["passed"] or not _ready(objects) or not hook.resting():
        return 0.0
    off = _off_plumb_deg(hook)
    if off <= SQUARE_DEG + EPS:
        return 1.0
    return 0.5 if off <= CROOKED_DEG + EPS else 0.0


def success(objects):
    """The hook plate flush on the wall, held tight under the tug test, square to the line, let go and still."""
    return bool(_done(objects) == 1.0)


def score(objects):
    _count_still(objects)
    _pull_step(objects)
    return _done(objects)


# --- simulator hooks: the release check and the tug ---------------------------------------------------------------------
FACE_STEP = 0.003
_HAND_LINKS = ("tip_left", "tip_right", "gripper")
_FACES: dict = {}


def _face_points(hull, step=FACE_STEP) -> np.ndarray:
    """Points on every face of a convex hull, about `step` apart (corners included)."""
    pts = [hull.vertices]
    for n, d in zip(hull.planes[:, :3], hull.planes[:, 3]):
        on = hull.vertices[np.abs(hull.vertices @ n - d) < 1e-6]
        if len(on) < 3:
            continue
        c = on.mean(axis=0)
        e1 = on[0] - c
        if np.linalg.norm(e1) < 1e-9:
            continue
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(n, e1)
        ring = on[np.argsort(np.arctan2((on - c) @ e2, (on - c) @ e1))]
        for i in range(len(ring)):
            a, b = ring[i], ring[(i + 1) % len(ring)]
            m = max(1, int(math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c), np.linalg.norm(a - b)) / step)))
            for s_ in range(m + 1):
                for t_ in range(m + 1 - s_):
                    pts.append((c + (a - c) * (s_ / m) + (b - c) * (t_ / m))[None, :])
    pts = np.concatenate(pts)
    _, keep = np.unique(np.round(pts / (step / 2)).astype(np.int64), axis=0, return_index=True)
    return np.concatenate([hull.vertices, pts[np.sort(keep)]])


def _link_faces(name):
    if name not in _FACES:
        from yam_arena import analysis

        _FACES[name] = [_face_points(h) for h in analysis.link_hulls().get(name, [])]
    return _FACES[name]


def hand_points_from_poses(poses: dict[str, tuple[np.ndarray, np.ndarray]]):
    """Face-sampled points (world) of the hand links given as {(arm, link): (pos, rotation)}; None unless all six
    links (both arms' gripper and two finger tips) are there and have hulls."""
    pts = []
    for arm in ("left", "right"):
        for n in _HAND_LINKS:
            faces = _link_faces(n)
            if (arm, n) not in poses or not faces:
                return None
            p, r = poses[(arm, n)]
            pts += [f @ np.asarray(r).T + np.asarray(p) for f in faces]
    return np.concatenate(pts)


def hand_gap(objects, pts) -> float:
    """Closest approach of the hand points to any shape of the hook (m)."""
    from yam_arena import analysis

    boxes = _hook_boxes(objects["hook"])
    ext = np.array([c + sgn * (np.abs(r) @ (np.asarray(s) / 2)) for _, c, r, s in boxes for sgn in (-1, 1)])
    lo, hi = ext.min(axis=0) - 0.02, ext.max(axis=0) + 0.02
    near = pts[np.all((pts > lo) & (pts < hi), axis=1)]  # only points within 20 mm of the hook's extent
    if not len(near):
        return 0.02
    return float(min(analysis.box_signed_distance(near, c, r, s).min() for _, c, r, s in boxes))


@parts.on_step
def _measure_grip(sim) -> None:
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    poses = {}
    for arm, key in (("left_robot", "left"), ("right_robot", "right")):
        asset = base.scene[arm]
        bodies = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in _HAND_LINKS:
                poses[(key, n)] = (bodies[i, :3] - origin, _rot(bodies[i, 3:]))
    pts = hand_points_from_poses(poses)
    _grip["clear"] = None if pts is None else bool(hand_gap(sim.states(), pts) > CLEAR)


def _set_wrenches(sim, force: float) -> None:
    """Every step, all the task's own forces at once (the wrench buffers are reset per asset on each write): the tug
    on the hook's arm, the socket's torque on the bolt and its reaction on the driver's handle, and the screw's friction
    torque on the bolt with its reaction on the plate."""
    import torch

    base = sim.env.unwrapped
    dev = base.device
    origin = parts._np(base.scene.env_origins[0])
    st = sim.states()
    hook, driver = st["hook"], st["screwdriver"]
    env = torch.tensor([0], dtype=torch.int32, device=dev)
    ab = _rot(hook.links["bolt"][1])[:, 2]
    ad = _rot(driver.links["shaft"][1])[:, 2]
    on_bolt = _drive["socket"] + _drive["resist"]
    at = np.asarray(hook.pos) + _rot(hook.quat_xyzw) @ np.asarray(PULL_AT) + origin
    bolt_at = np.asarray(hook.links["bolt"][0]) + origin
    asset = base.scene["hook"]
    names = list(asset.body_names)
    asset.permanent_wrench_composer.set_forces_and_torques_index(
        forces=torch.tensor([[[*(force * OUT)], [0.0, 0.0, 0.0]]], dtype=torch.float32, device=dev),
        torques=torch.tensor([[[*(-_drive["resist"] * ab)], [*(on_bolt * ab)]]], dtype=torch.float32, device=dev),
        positions=torch.tensor([[[*at], [*bolt_at]]], dtype=torch.float32, device=dev),
        body_ids=[names.index("base"), names.index("bolt")], env_ids=env, is_global=True)
    tool = base.scene["screwdriver"]
    tool.permanent_wrench_composer.set_forces_and_torques_index(
        torques=torch.tensor([[[*(-_drive["socket"] * ad)]]], dtype=torch.float32, device=dev),
        body_ids=[list(tool.body_names).index("base")], env_ids=env, is_global=True)
    _pull["applied"] = force


@parts.on_step
def _apply_wrenches(sim) -> None:
    """The tug (on while the test is in its pull phase, straight out from the wall) and the screw's torques."""
    _set_wrenches(sim, PULL_N if _pull.get("phase") == "pull" else 0.0)


@parts.on_step
def _debug(sim) -> None:
    """With HOOK_DEBUG=1 in the environment: one line every 0.5 s (finger tips, the plate's gaps, the thread)."""
    import os

    if not os.environ.get("HOOK_DEBUG") or round(sim.t * 20) % 10:
        return
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    tips = {}
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        bodies = parts._np(asset.data.body_link_pose_w)[0]
        tips[arm[0]] = np.mean([bodies[i, :3] - origin for i, n in enumerate(asset.body_names)
                                if n in ("tip_left", "tip_right")], axis=0).round(4).tolist()
    st = sim.states()
    h, d = st["hook"], st["screwdriver"]
    print(f"[dbg] t={sim.t:.2f} L={tips['l']} R={tips['r']} gaps_mm={(_gaps(h) * 1000).round(2).tolist()} "
          f"still={_record['still']} pull={_pull.get('phase')} plumb={_off_plumb_deg(h):.1f} "
          f"seated={_drive['target'] is not None} sock={_drive['socket']:.3f} res={_drive['resist']:.3f} "
          f"v={np.linalg.norm(h.lin_vel):.4f} w={np.linalg.norm(h.ang_vel):.3f} "
          f"j={ {k: round(v, 4) for k, v in h.joints.items()} } shaft={d.joints['shaft']:.3f} "
          f"drv={np.asarray(d.pos).round(4).tolist()} clear={_grip['clear']}", flush=True)


@parts.on_reset
def _reset_grip(sim) -> None:
    _grip["clear"] = None
    _record.update(still=0, handled=False)
    _pull.clear()
    _pull["phase"] = "idle"
    _drive.update(socket=0.0, resist=0.0)
    _set_wrenches(sim, 0.0)


# --- demo: open loop from the published start layout ------------------------------------------------------------------
TIP = 0.013  # finger tips past the TCP
GRIP_UP = 0.010  # finger tips 10 mm above the handle's lower end
TIP_PAST_TCP = TIP + GRIP_UP + HANDLE_Z0  # driver tip past the TCP while held
HANDLE_GRIP = -0.0135  # pads 27 mm apart on the 35 mm handle (a firm grip: looser, it wobbles when turned)
HANDLE_LOOSE = -0.025  # 50 mm: round the handle without touching it
ROD_GRIP = -0.0008
SEATED_X = FACE_X - PLATE[2] - WASHER_T  # world x of the socket's mouth when it sits on the washer face
HEAD_Z = TOP + HOLE_H - SAG  # the sagging bolt's axis height
# Driving: the screwdriver level, pointing into the wall (tilt 90 degrees toward +x); yaw turns it about its own axis.
# Each stroke turns it 200 degrees clockwise (seen from the handle), from yaw +110 to -90 degrees: the right wrist's
# range here.
DRIVE = dict(tilt=90.0, heading=0.0)
YAW_A, YAW_MID, YAW_B = math.radians(110.0), math.radians(10.0), math.radians(-90.0)
YAW_PICK = YAW_A - math.pi / 2  # picked a quarter turn round from the first stroke's start
N_STROKES = 8  # 8 x 200 = 1600 degrees: 4.7 mm commanded for the 3 mm gap (the socket slips once it is tight)


_LEVELLING = (((0.175, -0.195, 0.170), 58.0, 30.0, -8.0), ((0.180, -0.150, 0.179), 88.0, 57.0, 1.0),
              ((0.185, -0.105, 0.168), 105.0, 79.0, 3.0))


def _driver_plan():
    hx, hy = HOLDER_XY
    # the held driver's tip was measured to settle about 1.3 mm right of and 2.5 mm below where the arm is sent at
    # this pose (the arm's tracking offset, level, open loop): aim that much the other way
    by, bz = HOLE_Y + 0.0013, HEAD_Z + 0.0025
    pick_z = TOP + HOLDER[2] + GRIP_UP + TIP
    carry = TOP + 0.150
    seat_x = SEATED_X - TIP_PAST_TCP + 0.003  # 3 mm past seated: a light push keeps the socket on
    out_x = seat_x - 0.030  # 30 mm back from seated: the tip 20 mm clear of the head

    def dw(d, pos, f, label, yaw):
        return tilt.Waypoint(d, pos, f, label, yaw, **DRIVE)

    w = [
        Waypoint(1.5, (hx, hy, carry), HANDLE_LOOSE, "over the screwdriver", YAW_PICK),
        Waypoint(1.5, (hx, hy, pick_z), HANDLE_LOOSE, "down round the handle", YAW_PICK),
        Waypoint(0.8, (hx, hy, pick_z), HANDLE_GRIP, "grip the handle", YAW_PICK),
        Waypoint(1.5, (hx, hy, carry), HANDLE_GRIP, "lift it out", YAW_PICK),
    ]
    # turn it level on the way over, through three poses on the right arm's way from its straight-down grip to the
    # level one (its wrist flips over on the way): (x, y, height above the table), yaw, tilt (deg), heading
    for i, ((x, y, h), yaw, lean, head) in enumerate(_LEVELLING):
        w.append(tilt.Waypoint(1.0, (x, y, TOP + h), HANDLE_GRIP, f"turning it level ({i + 1}/3)", math.radians(yaw),
                               tilt=lean, heading=math.radians(head)))
    w += [
        dw(1.5, (out_x, by, bz), HANDLE_GRIP, "level, tip toward the wall", YAW_A),
        dw(1.5, (seat_x - 0.012, by, bz), HANDLE_GRIP, "socket just off the head", YAW_A),
        dw(1.5, (seat_x, by, bz), HANDLE_GRIP, "over the head", YAW_A),
    ]
    w.append(dw(1.0, (seat_x, by, bz), HANDLE_GRIP, "pressed in", YAW_A))
    n_seat = len(w)
    for k in range(N_STROKES):  # noqa: B007
        w += [  # two halves: a waypoint turns the short way round, so no step may exceed 180 degrees
            dw(1.0, (seat_x, by, bz), HANDLE_GRIP, f"turn {k + 1}", YAW_MID),
            dw(1.0, (seat_x, by, bz), HANDLE_GRIP, f"turn {k + 1}", YAW_B),
        ]
        if k < N_STROKES - 1:
            w += [
                dw(0.6, (seat_x, by, bz), HANDLE_GRIP, f"swing back {k + 1}", YAW_MID),
                dw(0.6, (seat_x, by, bz), HANDLE_GRIP, f"swing back {k + 1}", YAW_A),
            ]
    w += [
        dw(0.6, (seat_x - 0.004, by, bz), HANDLE_GRIP, "ease the push", YAW_B),
        dw(1.5, (out_x, by, bz), HANDLE_GRIP, "draw the driver back out", YAW_B),
        dw(10.0, (out_x, by, bz), HANDLE_GRIP, "hold it clear (the tug test runs)", YAW_B),
    ]
    return w, n_seat


def _duration(ws):
    return sum(x.duration for x in ws)


# The left hand pinches the hook's arm from above, leaning 60 degrees to the robot's right (finger tips toward -y), so
# its wrist stays to the left of the screwdriver; the fingers close across the round arm, half way along it.
ROD_X = FACE_X - 0.039  # 39 mm out from the wall: the fingers' outer edges just inside the upturned end
ROD_Z = HEAD_Z - ROD_Y
LEAN_DEG = 60.0
LEAN = dict(tilt=LEAN_DEG, heading=-math.pi / 2)
LEAN_DIR = np.array([0.0, -math.sin(math.radians(LEAN_DEG)), -math.cos(math.radians(LEAN_DEG))])  # TCP -> finger tips
ROD_TCP = np.array([ROD_X, HOLE_Y, ROD_Z]) - 0.006 * LEAN_DIR  # the rod 6 mm in from the TCP toward the tips
AWAY = np.array([-0.050, 0.020, 0.020])  # after letting go: out from the wall, left and up
PULL_DX = 0.040  # commanded pull straight out from the wall: the soft arm moves the hook far less


def demo():
    right, n_seat = _driver_plan()
    t_seated = _duration(right[:n_seat]) - 1.0  # the socket is on the head
    t_turned = _duration(right[: n_seat + 4 * N_STROKES - 2]) + 3.1
    up = np.array([0.0, 0.03, 0.05])  # above and to the left, clear of the hook

    def lw(d, pos, f, label):
        return tilt.Waypoint(d, tuple(float(v) for v in pos), f, label, 0.0, **LEAN)

    # pulling, the hand also leans the hook's arm 5 mm to the robot's right: under the pull and the turning the hook
    # was measured to swing its arm about 4 to 5 mm to the left (3 to 3.5 degrees off plumb), and this holds it plumb
    pulled = ROD_TCP + np.array([-PULL_DX, -0.005, 0.0])
    left = [
        # pulled out (the wings bearing on the back of the board) before the tip goes in, and kept so while it drives
        lw(t_seated - 6.5, ROD_TCP + up, -0.012, "over the hook's arm"),
        lw(1.5, ROD_TCP, -0.012, "down round the arm"),
        lw(0.8, ROD_TCP, ROD_GRIP, "pinch the arm"),
        lw(1.2, pulled, ROD_GRIP, "pull it out from the wall"),
    ]
    left.append(lw(t_turned - _duration(left), pulled, ROD_GRIP, "keep pulling"))
    left += [
        lw(1.0, ROD_TCP, ROD_GRIP, "ease off"),
        lw(0.6, ROD_TCP, -0.012, "let go"),
        lw(1.5, ROD_TCP + AWAY, -0.012, "out and away"),
        lw(9.0, ROD_TCP + AWAY, -0.012, "wait clear (the tug test runs)"),
    ]
    import os

    control = os.environ.get("HOOK_CONTROL")  # GPU control runs only (README): never set for the demo
    if control == "nopull":  # the screw driven with nobody pulling the hook: the left arm stays clear
        left = [lw(_duration(right), ROD_TCP + AWAY + np.array([0.0, 0.05, 0.08]), -0.012, "stay clear")]
    elif control == "crank":  # no tool: the left hand pulls the hook out and cranks it to and fro round its bolt
        left = left[:4]
        for k in range(4):
            for deg in (20.0, -20.0):
                a = math.radians(deg)
                swing = np.array([0.0, ROD_Y * math.sin(a), ROD_Y * (1.0 - math.cos(a))])
                left.append(lw(2.0, pulled + swing, ROD_GRIP, f"crank {k + 1}"))
        left += [lw(1.5, pulled, ROD_GRIP, "back plumb"), lw(1.0, ROD_TCP, ROD_GRIP, "ease off"),
                 lw(0.6, ROD_TCP, -0.012, "let go"), lw(6.0, ROD_TCP + AWAY, -0.012, "wait clear")]
        right = [Waypoint(_duration(left), (0.15, -0.30, TOP + 0.20), -0.025, "stay clear")]
    return {"left": left, "right": right}
