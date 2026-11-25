"""Scripted full-score demo. The right arm picks the hose's plug up off the table by its body and brings it in line
with the coupler's bore, 5 cm out. The left arm takes the collar from the side and pulls it back toward the tap. The
right arm pushes the plug in until its flange meets the mouth; the left lets the collar go, so it springs forward and
locks; then both let go and move clear.

    python demo.py task.py demo_out [--max-steps 1200] [--no-record] [--no-collar]

``--no-collar`` pushes the plug in the same way without pulling the collar back: the pins stop the plug's nose at the
mouth, nothing locks, and the trial scores 0.
Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json to the output folder.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import geometry as g  # noqa: E402
from demokit import run, wait  # noqa: E402

TOP = g.TOP
MOUTH = np.array([g.MOUTH_X, g.MOUTH_Y, TOP + g.AXIS_Z])
AX = np.array([math.cos(g.BORE_YAW), math.sin(g.BORE_YAW), 0.0])        # the bore's direction, into the tap
SIDE = np.array([-AX[1], AX[0], 0.0])                                   # square to it, to the robot's left
COLLAR_GRIP = MOUTH + AX * (g.COLLAR_X0 + 0.75 * g.COLLAR_LEN)   # the collar's tap end, clear of the plug's hand
PULL = 0.013                           # the left hand draws the collar back past its 9 mm stop, so it sits on it
PUSH = 0.0015                          # the right hand aims the flange this far past the mouth (it stops on the face)


def side_rotation():
    """Left: pointing -y (in from the robot's left), fingers closing vertically, round the collar."""
    x, z = np.array([0.0, 0.0, 1.0]), -SIDE
    return np.column_stack([x, np.cross(z, x), z])


def down_rotation(yaw: float):
    from yam_arena.demo import grip_rotation

    return grip_rotation(yaw)                     # pointing down; fingers closing across a heading of yaw


def actual_tcp(ctx, side):
    """The measured tool point and hand rotation (station frame)."""
    from yam_arena.constants import TCP_OFFSET

    base = ctx.emb._env.unwrapped
    robot = base.scene[f"{side}_robot"]
    i = list(robot.body_names).index("gripper")
    pose = robot.data.body_link_pose_w
    pose = getattr(pose, "torch", pose)[0, i].detach().cpu().numpy()
    origin = getattr(base.scene.env_origins, "torch", base.scene.env_origins)[0].detach().cpu().numpy()
    x, y, z, w = pose[3:7]
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    return pose[:3] - origin + R @ np.asarray(TCP_OFFSET), R


def align(a, b):
    """The smallest rotation taking unit vector a onto unit vector b."""
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    v, c = np.cross(a, b), float(a @ b)
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3)
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K / (1 + c)


def plug(ctx):
    p, R, _ = ctx.task.plug_pose(ctx.emb._env)
    return p, R


def take_plug(ctx):
    """Right: pick the plug up by its body, from above, fingers across it."""
    arm = ctx.right
    p, R = plug(ctx)
    heading = math.atan2(R[1, 0], R[0, 0])
    rot = down_rotation(heading)
    at = p + R @ np.array([g.PLUG_GRIP_X, 0.0, 0.0])
    yield from arm.move(at + [0, 0, 0.08], rot, 2.5, grip=1.0)
    yield from arm.move(at + [0, 0, 0.002], rot, 1.2)
    yield from arm.set_grip(0.0, 0.8)
    yield from wait(0.3)
    yield from arm.move(arm.pos + [0, 0, 0.10], rot, 1.5)


def line_up(ctx, standoff: float):
    """Right: carry the plug so its axis lies along the bore with its flange standoff m out of the mouth, measuring
    where the plug actually sits in the hand."""
    arm = ctx.right
    tcp_p, tcp_R = actual_tcp(ctx, "right")
    p, R = plug(ctx)
    rel_p, rel_R = tcp_R.T @ (p - tcp_p), tcp_R.T @ R                   # the plug in the hand
    # turn the commanded hand by the plug's measured axis error (the wrist lags its command under the hose's load, so
    # correcting from the measured hand would never close the gap)
    rot = align(R[:, 0], AX) @ arm.rot
    goal = MOUTH - AX * standoff
    return rot, rel_p, goal


def plug_error(ctx, standoff):
    """How far the plug is from lined up: its flange centre off the bore's axis (m) and its axis off the bore's (rad)."""
    p, R = plug(ctx)
    lat = p - (MOUTH - AX * standoff)
    along = float(lat @ AX)
    return float(np.linalg.norm(lat - along * AX)), float(math.acos(max(-1.0, min(1.0, R[:, 0] @ AX)))), along


def turn_plug(ctx, tries=3):
    """Right: where it was picked up, well away from the tap, turn the hand until the plug's axis lies along the bore's
    direction, measuring the plug each time (the hose's drag turns it in the hand a little)."""
    arm = ctx.right
    for _ in range(tries):
        _, R = plug(ctx)
        if math.acos(max(-1.0, min(1.0, R[:, 0] @ AX))) < math.radians(2):
            break
        yield from arm.move(arm.pos, align(R[:, 0], AX) @ arm.rot, 1.5)
        yield from wait(0.3)


def bring_plug(ctx, standoff=0.05, far=0.08):
    """Right: carry the lined-up plug to the bore's axis far m out (coming from above), then slide it in along the axis
    to standoff m out, measuring where it sits in the hand at each stage and allowing for how far the hand falls short
    of its command (the hose's pull)."""
    arm = ctx.right
    rot, rel_p, goal = line_up(ctx, far)
    above = goal - rot @ rel_p + [0, 0, 0.05]
    yield from arm.move(np.r_[above[:2], max(above[2], arm.pos[2])], rot, 2.0)
    for dist, seconds in ((far, 1.2), (far, 0.6), (standoff, 1.2), (standoff, 0.6)):
        yield from wait(0.3)
        tcp_p, _ = actual_tcp(ctx, "right")
        rot, rel_p, goal = line_up(ctx, dist)
        yield from arm.move(goal - rot @ rel_p + (arm.pos - tcp_p), rot, seconds)
    yield from wait(0.4)


def push_in(ctx, seconds=8.0, rate=0.002):
    """Right: push the plug in along the bore until its flange meets the mouth, correcting the hand every step by the
    measured plug's errors, never letting the command run more than 6 mm ahead of the hand (if the plug stops)."""
    arm = ctx.right
    for _ in range(round(seconds * 20)):
        tcp_p, _ = actual_tcp(ctx, "right")
        p, R = plug(ctx)
        if float((p - MOUTH) @ AX) >= -0.0005:                          # the flange is at the mouth: home
            break
        aim = MOUTH + AX * min(PUSH, float((p - MOUTH) @ AX) + rate)
        rot = arm.rot                                                   # the hand's turn as lined up
        cmd = arm.pos + (aim - p) * 0.5
        lead = cmd - tcp_p
        n = np.linalg.norm(lead)
        cmd = tcp_p + (lead * 0.012 / n if n > 0.012 else lead)
        arm.aim(cmd, rot)
        yield


def take_collar(ctx):
    arm = ctx.left
    rot = side_rotation()
    yield from arm.move(COLLAR_GRIP + SIDE * 0.09 + [0, 0, 0.02], rot, 2.5, grip=1.0)
    yield from arm.move(COLLAR_GRIP + SIDE * 0.09, rot, 0.8)
    yield from arm.move(COLLAR_GRIP, rot, 1.2)
    yield from arm.set_grip(0.0, 0.8)


def script(ctx, collar: bool = True):
    yield from take_plug(ctx)
    yield from turn_plug(ctx)
    yield from bring_plug(ctx)
    if collar:
        yield from take_collar(ctx)
        yield from ctx.left.move(COLLAR_GRIP + AX * PULL, None, 1.0)     # draw the collar back toward the tap
        yield from wait(0.3)
    yield from push_in(ctx)
    yield from wait(0.5)
    if collar:
        yield from ctx.left.set_grip(1.0, 0.6)                          # let the collar spring forward: it locks
        yield from wait(0.5)
        yield from ctx.left.move(ctx.left.pos + SIDE * 0.10 + [0, 0, 0.03], None, 1.2)
    yield from ctx.right.set_grip(1.0, 0.6)
    yield from ctx.right.move(ctx.right.pos + [0, 0, 0.10], None, 1.2)
    yield from wait(2.0)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE
    return (f"collar {S['collar'] * 1000:4.1f} mm  balls {[round(p * 1000, 1) for p in S['pins']]} mm  "
            f"flange {S['flange'] * 1000:6.1f} mm  seated {S['seated']}  locked {S['locked']}  "
            f"pads clear {S['clearance'] * 1000:5.1f} mm  plug drift {S['drift'] * 1000:5.1f} mm")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=1200)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--no-collar", action="store_true", help="push the plug in without pulling the collar back")
    ap.add_argument("--hires", action="store_true", help="record the cameras at 1280 x 704 (for a clearer video)")
    a = ap.parse_args()
    size = dict(cam_width=1280, cam_height=704) if a.hires else {}
    run(a.task, a.out, lambda ctx: script(ctx, collar=not a.no_collar), max_steps=a.max_steps,
        record=not a.no_record, log=log, log_every=10, **size)
