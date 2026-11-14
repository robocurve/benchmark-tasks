"""On the station table: a small fixed-light window frame (fixed, aluminium grey) standing on its sill on the midline
about 28 cm in front of the arm bases: a sill, two jambs and a head of 20 x 60 mm sections round a 240 mm wide, 95 mm
tall opening, its room side facing the arms. A fixed outside stop runs round the opening on the far side, 15 mm
into it; the room side is open, its glazing bead taken off (the bead, a 240 x 12 x 15 mm bar, lies on the table to
the left of the frame). Two white quarter-point marks (30 x 30 mm) lie on the rebate floor at y -5 cm and +5 cm.
Two loose black rubber setting blocks (30 x 30 x 8 mm) lie side by side on a dark grey tray 24 cm right of the
midline; a loose glass unit (24 mm thick, 200 mm long, 80 mm tall, 0.20 kg) stands on its long edge on the table
29 cm left of the midline, its faces normal to x. The head is 6 mm above a unit standing on the blocks, so the unit
cannot go in from above: it goes in from the room side. Rule: the unit must be left standing in the frame against
the outside stop on both setting blocks on their marks, on its own (both grippers let go), and must never touch
the sill.

The scorer reads the arms (TCP, pad-tip point, gripper opening) through rigstate.py, which wraps the embodiment's
step and reset; it changes nothing the embodiment does. Unreadable arms never count as let go.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score -E object_jitter=0 ...
Demo:    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 ...
One arm: DEMO_IDLE_ARM=left (or right) before the demo command leaves that arm at its ready pose.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import layout as L  # noqa: E402
import rigstate  # noqa: E402
import tiltgrip  # noqa: E402  (tilted grips for the demo: wraps yam_arena.demo.grip_rotation)
from scoring import Tracker  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import Box  # noqa: E402

ONLINE = rigstate.install()  # reads both arms before every control step (False offline: the checker feeds it)

SCENE = "station"
SCORE_OVER = "end"
# The 1.0 conditions already need the unit and both blocks let go (both grippers' finger zones 30 mm or more from
# each) on every step of the last 1 s and still over it, so a held or hovering unit can never start this hold.
SUCCESS_HOLD_S = 1.0

# The goal only: how to bring the unit in is for the robot to work out.
INSTRUCTION = (
    "Stand the glass unit in the window frame on the two black setting blocks, set on the white marks, and let go. "
    "The glass must never touch the sill."
)

FRAME = [Box(name, pos=c, size=s, color=L.ALUMINIUM) for name, (c, s) in L.FRAME_BOXES.items()]
MARKS = [
    Box(name, pos=(L.MARK_X, y, L.SILL_TOP + L.MARK_SIZE[2] / 2), size=L.MARK_SIZE, color=L.WHITE)
    for name, y in (("mark_right", L.MARK_YS[0]), ("mark_left", L.MARK_YS[1]))
]
BEAD = Box("glazing_bead", pos=L.BEAD_POS, size=L.BEAD_SIZE, color=L.BEAD_GREY)  # a prop: fixed, not scored
TRAY = Box("block_tray", pos=(L.TRAY_X, L.TRAY_Y, L.TOP + L.TRAY_SIZE[2] / 2), size=L.TRAY_SIZE, color=L.TRAY_GREY)
BLOCKS = [
    Box(name, pos=(x, y, L.TRAY_TOP + L.BLOCK_SIZE[2] / 2), size=L.BLOCK_SIZE, mass=L.BLOCK_MASS, color=L.RUBBER)
    for name, (x, y) in zip(("block_1", "block_2"), L.BLOCK_STARTS)
]
UNIT = Box(
    "glass_unit",
    pos=(L.UNIT_X, L.UNIT_Y, L.TOP + L.UNIT_SIZE[2] / 2),
    size=L.UNIT_SIZE,
    mass=L.UNIT_MASS,
    color=L.GLASS,
)
OBJECTS = [*FRAME, *MARKS, BEAD, TRAY, *BLOCKS, UNIT]

_TRACKER = Tracker()

# Offline only (tools/offline/check_task.py: no physics). The checker carries plain boxes upright with a straight-down
# grip; this task tilts the unit and pushes it, so offline the unit's state comes from offline_unit.py, a
# quasi-static model driven by the arms rigstate is fed. In the simulator the physics moves the unit and this is unused.
OFFLINE = None  # set at the end of this file, offline only


@rigstate.on_reset
def _new_episode() -> None:
    """rigstate saw the embodiment reset: a new trial (the scorer also spots a teleport back to the start)."""
    _TRACKER.reset()
    if OFFLINE is not None:
        OFFLINE.reset()


def _trace(row: dict) -> None:
    """With TASK_TRACE=<file>, append one JSON line per control step: what the scorer saw and decided."""
    path = os.environ.get("TASK_TRACE")
    if path:
        try:
            with open(path, "a") as f:
                f.write(json.dumps(row, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
        except OSError:  # a debugging aid only: never let it stop a trial
            pass


def _unit(objects, arms=None, step: bool = False):
    """The unit's state: the simulator's, or offline the proxy's (advanced one control step when ``step``)."""
    if OFFLINE is None:
        return objects["glass_unit"]
    if step:
        OFFLINE.step(arms, objects["block_1"], objects["block_2"])
    return OFFLINE.state()


def success(objects) -> bool:
    """Both blocks set on their marks, the unit standing in the rebate on both of them against the outside stop,
    centred, never having touched the sill; the unit and both blocks let go (finger zones 30 mm or more from each on
    every step of the last 1 s; unreadable arms are not let go) and still over that 1 s; held 1 s. This runs first
    every control step, so it also feeds the tracker (with the arms rigstate read for this step)."""
    arms = rigstate.arms()
    unit, b1, b2 = _unit(objects, arms, step=True), objects["block_1"], objects["block_2"]
    result = _TRACKER.update(unit, b1, b2, arms)
    row = {"episode": rigstate.episode(), "step": _TRACKER.steps, "score": result.score, "success": result.success,
           **result.details, "unit_pos": unit.pos, "unit_quat": unit.quat_xyzw, "unit_vel": unit.lin_vel,
           "block_1_pos": b1.pos, "block_2_pos": b2.pos}
    if rigstate.readable(arms):
        row.update(left_tcp=arms["left"].tcp, right_tcp=arms["right"].tcp,
                   left_tips=arms["left"].tips, right_tips=arms["right"].tips,
                   openings=[arms["left"].opening, arms["right"].opening])
    else:
        row["rigstate_error"] = rigstate.error()
    _trace(row)
    return result.success


def score(objects) -> float:
    """0 if the unit ever touched the sill or a mark (contact: no air gap), or ends lying down or off the table.
    Else 1.0 both blocks set within 8 mm of their marks and the unit standing in the rebate (within 3 mm of the
    outside stop, tilt under 3 degrees) resting on both, centred within 8 mm; 0.75 the same with the blocks within
    20 mm or the unit within 15 mm; 0.5 standing in the rebate resting on at least one block; 0.25 both blocks set;
    0 otherwise. 'Set' and 'resting' need the object let go (finger zones 30 mm or more from it on every step of the
    last 1 s; unreadable arms are not let go), still over that 1 s, and its support within 1 mm."""
    return _TRACKER.result(_unit(objects), objects["block_1"], objects["block_2"]).score


# -- the scripted demo ---------------------------------------------------------------------------------------
# Open loop: every waypoint is computed here from layout.py; nothing reads simulator state while it runs.
# Finger targets: the joint value is minus half the commanded gap; command 8 mm narrower than the part.
OPEN = -0.04695
CLOSED = 0.0
PAD = 0.013  # the pad tips are 13 mm past the TCP along the approach axis
PAD_BACK = 0.025  # the pads reach 25 mm behind the TCP
PAD_THICK, PAD_HALF_W = 0.008, 0.0135  # each pad 8 mm thick, 27 mm wide
# The pad's real face (the URDF meshes, research notes): flat from 9 mm behind the TCP to the tip 13 mm past it,
# 27 mm wide at the TCP, tapering to 15 mm at the tip: (distance along the approach axis, half width). The 27 mm
# rectangle above (PAD_BACK..PAD) is a safe outer bound for clearances; a grip must use this smaller face. Pitched
# 60 degrees, the 27 mm rectangle put its corner 11.7 mm below the tip point, but the real tip corner is only 6.5 mm
# below it: on the GPU (2026-11-14) the pads passed 0.3 mm over the block's top and closed on nothing.
PAD_FACE = ((-0.009, 0.0135), (0.0, 0.0135), (PAD, 0.0075))

# The unit's tilt when its bottom edge is set on the blocks: its top 48 degrees toward the room. Less, and the left
# hand comes within 15 mm of the head as it opens and backs out (offline: 16.2 mm at 45 degrees with no payload sag,
# and the hand springs up toward the head by the sag it carried if the sag is smaller than modelled; 48 keeps both
# above 15 mm).
TILT = math.radians(48.0)
SET_PIVOT_X = 0.2865  # where the unit's room-side bottom edge is set on the blocks (it turns about it)
STAND_PIVOT_X = L.STOP_X - L.UNIT_SIZE[0]  # 0.289: that edge's place with the unit upright against the stop
#   Set 2.5 mm short of it, so that an error of up to 2.5 mm outward in the sag (below) still lets the unit come
#   upright (further out, its outside bottom corner would come down on the stop's top), and 7.5 mm inward still sets
#   it on the blocks (their room-side edge is at x 0.279).
ENTRY_LIFT = 0.008  # the bottom edge comes in over the blocks this far up
ENTRY_BACK = 0.035  # from this far on the room side
SET_PRESS = 0.001  # it is set down commanded this far below the blocks' top: the blocks stop it

# Left arm, yaw -pi/2 (fingers close along x, across the unit's 24 mm). It grips the top edge 75 mm left of the
# unit's centre, so that at the handover the two TCPs are 13 cm or more apart.
YAW_L = -math.pi / 2
GRIP_Y = 0.075
GRIP_DEPTH = 0.010  # the TCP 10 mm below the top edge (pad tips 23 mm below it)
UNIT_GRIP = -(L.UNIT_SIZE[0] - 0.008) / 2  # 16 mm commanded
UNIT_LET_GO = -0.015  # 30 mm: 3 mm clear of each face (wider brings the outer pad toward the head)
HOVER_L = L.TOP + 0.14
LIFT_LOW = 0.030  # the unit's lowest point this far over the table while it is carried
RETREAT_L = (L.UNIT_X, L.UNIT_Y + GRIP_Y, HOVER_L)  # back over the unit's empty starting spot, straight down
# Payload sag: the arm's controller does not carry the unit's weight. The static estimate (J K^-1 J^T of the
# 0.20 kg unit's weight and its moment at the TCP; joints 80/80/80/10/10/10 N m/rad) with the unit tilted 48 degrees
# on the blocks, held 75 mm off its centre: the TCP 5.6 mm toward the arms, 6.2 mm left, 10.4 mm down, turned
# (2.7, 2.1, -0.9) degrees about world x, y, z (tools: tests/test_scoring.py, PayloadSag). Every waypoint that holds the
# unit tilted is commanded that much the other way, so that, loaded, the unit is where the plan puts it. Once it is on
# the blocks and the fingertip is on it, the command moves back to the unsagged pose ("unload": the blocks take the
# weight), so that the hand does not spring toward the head when it lets go. Upright, near the arms (the pick), the
# sag is 2.4 mm and is not compensated.
SAG = (-0.0056, 0.0062, -0.0104)
SAG_ROT_DEG = (2.7, 2.1, -0.9)

# Right arm, blocks: yaw 0 (fingers close along y, across a block's 30 mm), pitched 60 degrees so that the hand
# reaches under the head from the room side (the head is over the whole rebate). The real pad faces' lowest corner
# (a tip corner) 2 mm over the block's bottom: the faces then cover the block's sides from 2 mm up to its 8 mm top
# along their lower edge, 11 mm of it (that edge runs from the tip back to the TCP level only 1.3 mm higher).
BLOCK_PITCH = math.radians(60.0)
BLOCK_GRIP = -(L.BLOCK_SIZE[1] - 0.008) / 2  # 22 mm commanded
BLOCK_LET_GO = -0.024  # 48 mm: 9 mm clear of the block on each side
BLOCK_PAD_UP = 0.002
BLOCK_TIP_BACK = 0.0015  # the pad-tip point this far room-side of the block's centre: on the mark the hand's pads
#                          (as the 27 mm bound) then stay 15 mm from the outside stop; the faces still cover 10 mm
B_CARRY = 0.046  # a carried block's bottom this far over the table on the way to the frame (the hand as high as
#                  with the old 2.5 mm pad corner and 40 mm, which the GPU tracked; lower costs joint 3 margin)
B_ENTRY = 0.010  # it goes in under the head with its bottom this far over its mark
PARK_R = (0.20, -0.22, L.TOP + 0.13)
# Where each block is set, from its mark's centre (x, y): (right mark, left mark). Zero offline. On the GPU, read
# block_mark_mm in TASK_TRACE after each release: if a block lands consistently off its mark, trim here.
BLOCK_TRIM = ((0.0, 0.0), (0.0, 0.0))

# Right arm, the push: yaw +pi/2 (fingers close along x), fingers closed, pitched 55 degrees (pointing forward and
# down, so the hand stays on the room side of the head) and rolled 35 degrees (its wrist leans away from the left
# arm). The front pad's nearest corner touches the unit's room face 45 mm right of its centre: 75 mm up the face
# while it leans 48 degrees (high enough that the leaning unit cannot twist off it), lower as it comes upright.
YAW_R = math.pi / 2
PUSH_PITCH = math.radians(55.0)
PUSH_ROLL = math.radians(35.0)
PUSH_Y = -0.045
PUSH_UP = ((48.0, 0.075), (38.0, 0.075), (28.0, 0.065), (18.0, 0.055), (8.0, 0.050), (0.0, 0.045))  # (deg, up face)
PUSH_LOW = 0.025  # then, upright, 25 mm up the face, it slides the unit out against the stop
PUSH_PAST = 0.0035  # ... to this far past the room face of a unit standing against the stop (the stop takes it):
#   on the GPU (2026-11-14) the unit ended 3.7 mm short of where the finger was sent (0.5 mm past: stop gap 2.3 mm
#   at its far corner, 1 degree askew), 1.4 mm of it the arm giving way to the push
PUSH_START = 0.015  # the finger comes in from this far off the face
PUSH_TOUCH = 0.0005  # and waits this far off it while the left arm lets go
PUSH_BACK = 0.045  # after the push the finger backs off this far from the face
PARK_R2 = (0.20, -0.20, L.TOP + 0.15)
# Where the unit is set down, from its place (x, y). Zero offline. On the GPU, read unit_pos in TASK_TRACE.
PLACE_TRIM = (0.0, 0.0)


def unit_pose(tilt: float, pivot_x: float, pivot_z: float, y: float):
    """(centre, rotation) of the unit tilted ``tilt`` rad (its top toward the room) with its room-side bottom edge at
    (pivot_x, y, pivot_z)."""
    r = tiltgrip.ry(-tilt)
    edge = np.array([pivot_x, y, pivot_z])
    return edge - r @ np.array([-L.UNIT_SIZE[0] / 2, 0.0, -L.UNIT_SIZE[2] / 2]), r


def left_tcp(tilt: float, pivot_x: float, pivot_z: float, y: float):
    """The left TCP gripping the top edge of the unit so placed (GRIP_Y along it, GRIP_DEPTH below its top)."""
    c, r = unit_pose(tilt, pivot_x, pivot_z, y)
    return c + r @ np.array([0.0, GRIP_Y, L.UNIT_SIZE[2] / 2 - GRIP_DEPTH])


def pad_corners(tcp, rot, gap: float = 0.0) -> np.ndarray:
    """The 16 corners of the two finger pads (world) at TCP pose (tcp, rot) with the pads ``gap`` apart."""
    tcp, rot = np.asarray(tcp, dtype=float), np.asarray(rot, dtype=float)
    out = []
    for side in (-1.0, 1.0):
        for t in (gap / 2, gap / 2 + PAD_THICK):
            for w in (-PAD_HALF_W, PAD_HALF_W):
                for a in (-PAD_BACK, PAD):
                    out.append(tcp + rot[:, 0] * side * t + rot[:, 1] * w + rot[:, 2] * a)
    return np.array(out)


def pad_face_corners(tcp, rot, gap: float = 0.0) -> np.ndarray:
    """The corners of the two pads' real gripping faces (PAD_FACE; world) at TCP pose (tcp, rot), ``gap`` apart."""
    tcp, rot = np.asarray(tcp, dtype=float), np.asarray(rot, dtype=float)
    return np.array([tcp + rot[:, 0] * side * gap / 2 + rot[:, 1] * w + rot[:, 2] * a
                     for side in (-1.0, 1.0) for a, hw in PAD_FACE for w in (-hw, hw)])


def push_tcp(tilt: float, up: float, off: float, pivot_x: float = SET_PIVOT_X, y: float = PUSH_Y):
    """The right TCP (closed, pitched, rolled) whose pad corner nearest the unit is ``off`` from the room face of the
    unit (tilted ``tilt``, turning about its room-side bottom edge at ``pivot_x`` on the blocks), ``up`` from that
    edge, at ``y``."""
    rot = tiltgrip.rotation(YAW_R, PUSH_PITCH, PUSH_ROLL)
    edge = np.array([pivot_x + PLACE_TRIM[0], y, L.SET_BLOCK_TOP])
    along = np.array([-math.sin(tilt), 0.0, math.cos(tilt)])  # up the room face
    into = np.array([math.cos(tilt), 0.0, math.sin(tilt)])  # into the unit, normal to its room face
    tcp = edge + along * up - into * 0.02
    for _ in range(3):
        pts = pad_corners(tcp, rot)
        p = pts[int(np.argmax((pts - edge) @ into))]
        tcp = tcp + into * (-off - (p - edge) @ into) + along * (up - (p - edge) @ along)
        tcp[1] += y - p[1]
    return tcp


def block_tcp(x: float, y: float, bottom: float):
    """The right TCP gripping a block whose bottom is at ``bottom`` and centre at (x, y): pitched BLOCK_PITCH, the
    real pad faces' lowest corner (the tip's, PAD_FACE) BLOCK_PAD_UP over the bottom, the pad-tip point
    BLOCK_TIP_BACK room-side of the block's centre (x)."""
    rot = tiltgrip.rotation(0.0, BLOCK_PITCH)
    tcp = np.array([x, y, bottom + 0.02])
    for _ in range(3):
        low = pad_face_corners(tcp, rot, L.BLOCK_SIZE[1])[:, 2].min()
        tcp[2] += bottom + BLOCK_PAD_UP - low
        tcp[0] += x - BLOCK_TIP_BACK - (tcp + rot[:, 2] * PAD)[0]
    return tcp


def _wp(duration, pos, finger, label, yaw):
    return Waypoint(duration, tuple(float(v) for v in pos), finger, label, yaw)


def _span(wps) -> float:
    return sum(w.duration for w in wps)


_YAW_B = tiltgrip.tilted(0.0, BLOCK_PITCH)


def _place_block(start, mark: int, first: bool) -> list[Waypoint]:
    """Pick one block off the tray (pitched grip), carry it to the frame's room side level with its mark, in under
    the head 10 mm over the mark, down onto it, let go, back out and up. Every grip and release takes 1.5 s (the
    CLI's 0.05-per-step gripper limiter); the last 10 mm down takes 2 s."""
    sx, sy = start
    mx, my = L.MARK_X + BLOCK_TRIM[mark][0], L.MARK_YS[mark] + BLOCK_TRIM[mark][1]
    pick = block_tcp(sx, sy, L.TRAY_TOP)
    up = block_tcp(sx, sy, L.TOP + B_CARRY)
    entry_low = L.MARK_TOP + B_ENTRY
    outside = block_tcp(L.FRAME_ROOM_X - 0.035, my, entry_low)  # in front of the frame, level with the rebate
    over = block_tcp(mx, my, entry_low)
    place = block_tcp(mx, my, L.MARK_TOP + 0.0015)
    back = block_tcp(L.FRAME_ROOM_X - 0.035, my, L.MARK_TOP + 0.0015)
    yb = _YAW_B
    lane_x = L.FRAME_ROOM_X - 0.035  # in front of the frame: the hand turns toward the tray or the frame here
    hover = pick + np.array([0, 0, 0.06])
    lead = [] if first else [_wp(1.5, (lane_x, sy, hover[2]), OPEN, "in front of the tray", yb)]
    return lead + [
        _wp(2.0 if first else 1.2, hover, OPEN, "over a setting block", yb),
        _wp(1.0, pick + np.array([0, 0, 0.02]), OPEN, "down to just above it", yb),
        _wp(1.2, pick, OPEN, "slowly down beside its 30 mm sides", yb),
        _wp(0.5, pick, OPEN, "settle", yb),
        _wp(1.5, pick, BLOCK_GRIP, "grip the block", yb),
        _wp(1.0, up, BLOCK_GRIP, "lift", yb),
        _wp(1.2, (lane_x, up[1], up[2]), BLOCK_GRIP, "back toward the arms, clear of the jamb", yb),
        _wp(2.0, (lane_x, my, up[2]), BLOCK_GRIP, "to the frame's room side", yb),
        _wp(1.2, outside, BLOCK_GRIP, "down level with the rebate", yb),
        _wp(1.5, over, BLOCK_GRIP, "in under the head, over the mark", yb),
        _wp(2.0, place, BLOCK_GRIP, "slowly down onto the mark", yb),
        _wp(0.5, place, BLOCK_GRIP, "settle", yb),
        _wp(1.5, place, BLOCK_LET_GO, "let go of the block", yb),
        _wp(0.8, place + np.array([0, 0, 0.003]), BLOCK_LET_GO, "up 3 mm, clear of the mark", yb),
        _wp(1.5, back + np.array([0, 0, 0.003]), BLOCK_LET_GO, "back out from under the head", yb),
        _wp(1.0, back + np.array([0, 0, 0.05]), BLOCK_LET_GO, "up", yb),
    ]


_YAW_P = tiltgrip.tilted(YAW_R, PUSH_PITCH, PUSH_ROLL)


def _right_blocks() -> list[Waypoint]:
    """Right arm: set the far block on the right mark, then the near one on the left mark, then park."""
    return [
        *_place_block(L.BLOCK_STARTS[1], 0, first=True),
        *_place_block(L.BLOCK_STARTS[0], 1, first=False),
        _wp(1.5, PARK_R, OPEN, "out of the way, on its own side", 0.0),
        _wp(1.5, PARK_R, CLOSED, "close the fingers", 0.0),
    ]


_RIGHT_PARKED_AT = _span(_right_blocks())
_ROT_U = tiltgrip.rotation(YAW_L, TILT)


def _rotvec(v) -> np.ndarray:
    """Rotation matrix of a rotation vector (rad)."""
    v = np.asarray(v, dtype=float)
    a = float(np.linalg.norm(v))
    if a < 1e-12:
        return np.eye(3)
    k = v / a
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(a) * kx + (1.0 - math.cos(a)) * kx @ kx


def _held(pivot_x: float, pivot_z: float, label: str, duration: float, sag: bool = True) -> Waypoint:
    """A left waypoint holding the unit tilted TILT with its turning edge at (pivot_x, pivot_z), on the midline:
    commanded against the payload sag (``sag``), or not."""
    pos = left_tcp(TILT, pivot_x + PLACE_TRIM[0], pivot_z, PLACE_TRIM[1])
    rot = _ROT_U
    if sag:
        pos = pos - np.asarray(SAG)
        rot = _rotvec(-np.radians(SAG_ROT_DEG)) @ rot
    return _wp(duration, pos, UNIT_GRIP, label, tiltgrip.TiltedYaw(YAW_L, rot))


def _left_set() -> list[Waypoint]:
    """Left arm up to holding the unit tilted on the blocks: grip the top edge, lift it and hold it, upright, while the
    right arm sets the blocks; then tilt it and bring it to the room side of the frame, in over the blocks and down
    onto them."""
    ux, uy = L.UNIT_X, L.UNIT_Y + GRIP_Y
    pick_z = L.TOP + L.UNIT_SIZE[2] - GRIP_DEPTH
    pick = [
        _wp(2.0, (ux, uy, HOVER_L), OPEN, "over the glass unit", YAW_L),
        _wp(1.0, (ux, uy, pick_z + 0.030), OPEN, "down to just above its top edge", YAW_L),
        _wp(1.5, (ux, uy, pick_z), OPEN, "slowly down beside its faces", YAW_L),
        _wp(0.5, (ux, uy, pick_z), OPEN, "settle", YAW_L),
        _wp(1.5, (ux, uy, pick_z), UNIT_GRIP, "grip its top edge", YAW_L),
        _wp(0.5, (ux, uy, pick_z), UNIT_GRIP, "settle the grip", YAW_L),
        _wp(1.5, (ux, uy, pick_z + LIFT_LOW), UNIT_GRIP, "lift it straight up", YAW_L),
    ]
    wait = max(0.5, _RIGHT_PARKED_AT - _span(pick))
    entry_z, set_z = L.SET_BLOCK_TOP + ENTRY_LIFT, L.SET_BLOCK_TOP - SET_PRESS
    return pick + [
        _wp(wait, pick[-1].pos, UNIT_GRIP, "hold it up while the blocks go in", YAW_L),
        _held(SET_PIVOT_X - ENTRY_BACK - 0.040, entry_z + 0.010, "tilt it, in front of the frame", 3.0),
        _held(SET_PIVOT_X - ENTRY_BACK, entry_z, "up to the frame's room side", 1.5),
        _held(SET_PIVOT_X, entry_z, "bottom edge in over the blocks", 2.0),
        _held(SET_PIVOT_X, entry_z, "settle", 0.5),
        _held(SET_PIVOT_X, set_z, "set the bottom edge on the blocks", 2.0),
        _held(SET_PIVOT_X, set_z, "settle on the blocks", 0.5),
    ]


# The handover, on one clock: the right fingertip arrives once the unit is on the blocks; the left arm unloads and
# lets go after it, and the right arm pushes once the left hand is out.
APPROACH_S, TOUCH_S, UNLOAD_S, LET_GO_S, BACK_OUT_S = 3.0, 1.5, 1.0, 1.5, 1.5
T_SET = _span(_left_set())  # the unit tilted on the blocks, held
T_TOUCH = max(_RIGHT_PARKED_AT, T_SET - APPROACH_S) + APPROACH_S + TOUCH_S  # the fingertip on its room face
T_UNLOAD = T_TOUCH + 0.3  # the left arm starts to give the unit's weight to the blocks
T_LEFT_OUT = T_UNLOAD + UNLOAD_S + LET_GO_S + BACK_OUT_S  # the left hand is out, backed off along the glass
T_PUSH = T_LEFT_OUT + 0.3  # the push starts


def _left() -> list[Waypoint]:
    """Left arm: set the unit tilted on the blocks and hold it until the right fingertip is on it, then give its
    weight to the blocks, let go, back out along the unit and away over its starting spot."""
    plan = _left_set()
    set_z = L.SET_BLOCK_TOP - SET_PRESS
    unloaded = _held(SET_PIVOT_X, set_z, "unload: the blocks take its weight", UNLOAD_S, sag=False)
    last = np.asarray(unloaded.pos)
    back = last - _ROT_U[:, 2] * 0.045
    yaw_u = unloaded.yaw
    return plan + [
        _held(SET_PIVOT_X, set_z, "hold it until the fingertip is on it", T_UNLOAD - T_SET),
        unloaded,
        _wp(LET_GO_S, last, UNIT_LET_GO, "let go", yaw_u),
        _wp(BACK_OUT_S, back, UNIT_LET_GO, "back out along the glass", yaw_u),
        _wp(2.5, RETREAT_L, UNIT_LET_GO, "away, over its starting spot", YAW_L),
    ]


def _right_push() -> list[Waypoint]:
    """Right arm, after the blocks: closed fingertip to the leaning unit's room face (while the left arm still holds
    it), wait while the left arm unloads, lets go and backs out, push the unit upright under the head, slide it out
    against the outside stop, back off and park."""
    t0, up0 = math.radians(PUSH_UP[0][0]), PUSH_UP[0][1]
    plan = _right_blocks()
    wait = T_TOUCH - TOUCH_S - APPROACH_S - _span(plan)
    if wait > 0:
        plan.append(_wp(wait, PARK_R, CLOSED, "wait for the unit", 0.0))
    plan += [
        _wp(APPROACH_S, push_tcp(t0, up0, PUSH_START), CLOSED, "closed fingertip to the leaning unit's room side",
            _YAW_P),
        _wp(TOUCH_S, push_tcp(t0, up0, PUSH_TOUCH), CLOSED, "fingertip on its room face", _YAW_P),
        _wp(T_PUSH - T_TOUCH, push_tcp(t0, up0, PUSH_TOUCH), CLOSED, "hold it while the left hand lets go", _YAW_P),
    ]
    for deg, up in PUSH_UP[1:]:
        plan.append(_wp(1.5, push_tcp(math.radians(deg), up, 0.0), CLOSED, f"push it upright to {deg:.0f} degrees",
                        _YAW_P))
    up_end = PUSH_UP[-1][1]
    return plan + [
        _wp(1.0, push_tcp(0.0, PUSH_LOW, 0.0), CLOSED, "down its face", _YAW_P),
        _wp(1.5, push_tcp(0.0, PUSH_LOW, -PUSH_PAST, STAND_PIVOT_X), CLOSED, "slide it out against the outside stop",
            _YAW_P),
        _wp(0.5, push_tcp(0.0, PUSH_LOW, -PUSH_PAST, STAND_PIVOT_X), CLOSED, "settle", _YAW_P),
        _wp(1.5, push_tcp(0.0, up_end, PUSH_BACK, STAND_PIVOT_X), CLOSED, "back off the glass", _YAW_P),
        _wp(2.0, PARK_R2, CLOSED, "away, on its own side", 0.0),
    ]


def demo():
    """Both arms on one clock. DEMO_HOLD_ONLY=1: no plan (both arms hold the ready pose, a settling check).
    DEMO_IDLE_ARM=left|right: that arm's plan is dropped (the one-arm check)."""
    if os.environ.get("DEMO_HOLD_ONLY") == "1":
        return {}
    plan = {"left": _left(), "right": _right_push()}
    plan.pop(os.environ.get("DEMO_IDLE_ARM", ""), None)
    return plan


if not ONLINE:
    import offline_unit  # noqa: E402

    OFFLINE = offline_unit.UnitProxy(sag=(SAG, SAG_ROT_DEG, math.radians(20.0)))
