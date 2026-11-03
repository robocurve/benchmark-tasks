"""The scripted demo for the keyed-plug task: the left arm grips the loose block, the right arm carries the plug to the
socket, turns it to match the key and pushes it home, then both let go. Waypoints only (numpy), played open loop by
``yam_scripted`` through the task file's ``demo()``. Positions follow the block's own frame (``geometry``), so the
plan works for any block position and rotation.

Every number that depends on contact (where the plug is gripped, how far it is pushed) can be tuned without editing
code, through environment variables read when the plan is built:

    DEMO_PLUG_DX / DEMO_PLUG_DY   shift where the right gripper holds the plug, along the plug's x / y axis (m)
    DEMO_ALIGN_DX / DEMO_ALIGN_DZ shift the plug sideways (along the block's x) / up when it lines up with the socket (m)
    DEMO_LEFT_DWX / DEMO_LEFT_DWY shift the left hand's grip point on the block, in world x / y (m)
    DEMO_OVERSHOOT                how far past the seated pose the push is commanded (m)
    DEMO_PUSH_S                   seconds the push takes
    DEMO_NO_BRACE=1               leave the left arm idle (the one-arm check that shows bracing is needed)
    DEMO_YAW_SHIFT_LEFT / _RIGHT  0 or -3.14159: which of the two equivalent grips each arm uses; ``check_reach.py``
                                  shows which the arm can reach (see SHIFT_DEFAULT)
"""

from __future__ import annotations

import math
import os

import geometry as g
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint

PAD_TO_TCP = 0.007  # the grip point sits this far above the housing's centre line (pad tips 13 mm below the TCP)
SHIFT_DEFAULT = {"left": 0.0, "right": 0.0}  # chosen by check_collisions.py for the layout in geometry.py
LEFT_GRIP_DY = 0.03  # the left hand holds the block this far from its centre, away from the socket (keeps the hands apart)
# Calibrated in the simulator (trace of a run): the gripper's pads are not centred on the commanded point, so the left
# hand closing on the block shoved it 8.3 mm; commanding the grip point this far (world x, y; m) the other way cancels it.
LEFT_GRIP_SHIFT_W = (0.0, 0.0)
DEFAULT_ALIGN_DX, DEFAULT_ALIGN_DZ = 0.0, 0.0005  # plug lined up with the socket: sideways (block x) and up (m)
# Each finger is driven to a position, with at most 50 N. Commanded fully closed (175 N each) both saturate, the pair has
# no centring force, and the clamp slid sideways until a finger hit its limit (a trace showed the block shoved 8-12 mm).
# Commanding a finger only this far (m) past touching the part keeps it unsaturated, so the clamp stays centred.
GRIP_PAST_CONTACT = 0.005
DEFAULT_OVERSHOOT, DEFAULT_PUSH_S = 0.003, 6.0  # push commanded this far past the seated pose, taking this long


def _f(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


def _tcp(origin_xy, yaw: float, origin_z: float, dx: float = 0.0, dy: float = 0.0):
    """Where the right gripper's TCP sits to hold a plug whose origin is at ``origin_xy``, ``origin_z``, turned ``yaw``:
    on the housing's centre (9 mm behind the origin along the plug's -y), plus any tuning shift in the plug's frame."""
    off = g.rotate_xy((dx, -g.HOUSING[1] / 2 + dy), yaw)
    return (origin_xy[0] + off[0], origin_xy[1] + off[1], origin_z + PAD_TO_TCP)


def right_plan() -> list[Waypoint]:
    dx, dy = _f("DEMO_PLUG_DX", 0.0), _f("DEMO_PLUG_DY", 0.0)
    ax, az = _f("DEMO_ALIGN_DX", DEFAULT_ALIGN_DX), _f("DEMO_ALIGN_DZ", DEFAULT_ALIGN_DZ)
    over, push_s = _f("DEMO_OVERSHOOT", DEFAULT_OVERSHOOT), _f("DEMO_PUSH_S", DEFAULT_PUSH_S)
    shift = _f("DEMO_YAW_SHIFT_RIGHT", SHIFT_DEFAULT["right"])
    # Closing the fingers along the plug's x axis (across the 45 mm housing): grip yaw = plug yaw + 90 degrees.
    yaw_pick = g.PLUG_YAW0 + math.pi / 2 + shift
    yaw_seat = g.BLOCK_YAW + math.pi / 2 + shift
    pick = _tcp(g.PLUG_XY0, g.PLUG_YAW0, g.PLUG_Z0, dx, dy)
    stage_origin = g.block_to_world_xy((ax, g.MOUTH_Y - 0.04))  # nose tip 22 mm short of the mouth
    seat_origin = g.block_to_world_xy((ax, g.MOUTH_Y + over))
    back_origin = g.block_to_world_xy((ax, g.MOUTH_Y + over - 0.04))
    stage = _tcp(stage_origin, g.BLOCK_YAW, g.SEAT_Z + az, dx, dy)
    seat = _tcp(seat_origin, g.BLOCK_YAW, g.SEAT_Z + az, dx, dy)
    back = _tcp(back_origin, g.BLOCK_YAW, g.SEAT_Z + az, dx, dy)
    hover_z, carry_z = g.TOP + 0.10, g.TOP + 0.09
    o, c = FINGER_OPEN, -g.HOUSING[0] / 2 + GRIP_PAST_CONTACT
    return [
        Waypoint(1.5, (pick[0], pick[1], hover_z), o, "to the plug", yaw_pick),
        Waypoint(1.2, pick, o, "descend", yaw_pick),
        Waypoint(0.8, pick, c, "grip the plug", yaw_pick),
        Waypoint(1.2, (pick[0], pick[1], carry_z), c, "lift", yaw_pick),
        Waypoint(2.0, (stage[0], stage[1], carry_z), c, "carry and turn to the key", yaw_seat),
        Waypoint(1.0, stage, c, "lower to the socket axis", yaw_seat),
        Waypoint(0.8, stage, c, "settle", yaw_seat),
        Waypoint(push_s, seat, c, "push in", yaw_seat),
        Waypoint(0.6, seat, c, "hold", yaw_seat),
        Waypoint(1.2, seat, o, "let go", yaw_seat),
        Waypoint(1.0, (seat[0], seat[1], hover_z), o, "lift clear", yaw_seat),
        Waypoint(1.0, (back[0], back[1], hover_z), o, "back off", yaw_seat),
    ]


def left_plan(hold_s: float) -> list[Waypoint]:
    """Grip the block across its 70 mm width, hold it for ``hold_s`` seconds, let go and lift away."""
    shift = _f("DEMO_YAW_SHIFT_LEFT", SHIFT_DEFAULT["left"])
    yaw = g.BLOCK_YAW + math.pi / 2 + shift
    # The grip point can sit away from the socket end (block-frame +y), to keep the two hands further apart.
    bx, by = g.block_to_world_xy((0.0, _f("DEMO_LEFT_GRIP_DY", LEFT_GRIP_DY)))
    bx += _f("DEMO_LEFT_DWX", LEFT_GRIP_SHIFT_W[0])
    by += _f("DEMO_LEFT_DWY", LEFT_GRIP_SHIFT_W[1])
    grip = (bx, by, g.TOP + 0.045)  # pad tips 32 mm up the 60 mm block
    hover_z = g.TOP + 0.12
    o, c = FINGER_OPEN, -g.BLOCK_SIZE[0] / 2 + GRIP_PAST_CONTACT
    return [
        Waypoint(1.5, (bx, by, hover_z), o, "to the block", yaw),
        Waypoint(1.2, grip, o, "descend", yaw),
        Waypoint(0.8, grip, c, "grip the block", yaw),
        Waypoint(hold_s, grip, c, "hold while the plug goes in", yaw),
        Waypoint(0.8, grip, o, "let go of the block", yaw),
        Waypoint(1.2, (bx, by, hover_z), o, "clear", yaw),
    ]


def plans() -> dict[str, list[Waypoint]]:
    right = right_plan()
    names = [w.label for w in right]
    # The left arm holds from the end of its grip until the right arm has let go and lifted clear.
    t_backed_off = sum(w.duration for w in right[: names.index("lift clear") + 1])
    t_gripped = 1.5 + 1.2 + 0.8
    plan = {} if os.environ.get("DEMO_NO_RIGHT") == "1" else {"right": right}  # DEMO_NO_RIGHT=1: only the left arm moves
    if os.environ.get("DEMO_NO_BRACE") != "1":
        plan["left"] = left_plan(max(0.5, t_backed_off - t_gripped))
    return plan
