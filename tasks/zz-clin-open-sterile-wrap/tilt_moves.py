"""tilt_moves: demo moves at any gripper orientation (not only straight down), for cabledemo-driven demos.

cabledemo's ``Arm.move`` only takes a yaw (straight-down grip). The arm itself takes joint targets and the kit's IK
accepts any orientation, so this module adds moves to an arbitrary TCP pose without touching cabledemo.py or
cablekit.py. Symlink it next to demo.py (``ln -s ../../cablekit/tilt_moves.py``) and ``import tilt_moves as tm``.

Reach (offline IK, joint limits only, 2026-11-17; TCP height above the table, left arm, right arm mirrors):
  straight down          : tops out at 0.20-0.22 m
  45 deg forward-down    : 0.30-0.42 m at x 0.20-0.40 (down to 0.02 m too)
  horizontal (forward)   : up to 0.44 m, but cannot go low near the robot (min 0.06-0.36 m)

API (all generators; one yield per control step, like ``Arm.move``):
    R = tm.tilted(tilt_deg=45, heading=0.0, roll=0.0)   # approach tilted from straight down towards ``heading``
                                                         # (rad, world yaw; 0 = forward +x); fingers close along the
                                                         # horizontal axis perpendicular to heading (roll turns them)
    yield from tm.move_pose(ctx.left, pos, R, duration=1.5, finger=None)
    yield from tm.grip_pose(ctx.left, finger, duration=0.6)    # open/close holding the current pose
    tm.approach(R) -> unit vector the fingers point along
    tm.reachable(arm_name, pos, R) -> (ok, pos_err_m, rot_err_rad)   # offline check before simulating
    tm.hands_gap(ctx) -> metres between the two grippers' bodies (TCP-to-flange segments)

Notes:
- After a tilted move, do NOT call ``Arm.grip``: it re-commands a straight-down grip at the arm's last yaw. Use
  ``grip_pose``. ``Arm.move`` afterwards is fine (it blends from the current joints back to straight down).
- Hand spacing: grippers whose bodies come within ~12 cm collide at the wrist cameras/housings. With a tilt the
  housing sits back along the approach, not straight above the TCP, so check ``hands_gap(ctx) >= 0.12`` in a debug
  log (or offline with ``gap_between``) for every phase where both arms are near the work.
- The wrist camera looks along the gripper's yaw heading; tilted, it also looks further down. Check the wrist videos.
- Demos must stay camera/feel-driven: this module only uses the arms' own joints.
"""

from __future__ import annotations

import math

import numpy as np

CONTROL_HZ = 20
FLANGE_BACK = 0.134  # TCP is this far from the gripper frame along the approach axis


def _down():
    from yam_arena.demo import grip_rotation

    return grip_rotation(0.0)


def _rot(axis, ang):
    from yam_arena.urdf import axis_angle_matrix

    return axis_angle_matrix(np.asarray(axis, dtype=float), float(ang))


def tilted(tilt_deg: float = 45.0, heading: float = 0.0, roll: float = 0.0) -> np.ndarray:
    """TCP rotation: straight-down grip tilted ``tilt_deg`` so the fingers point towards world yaw ``heading`` (and
    down), then turned ``roll`` (rad) about the approach axis. tilt 0 = straight down, 90 = horizontal."""
    r = _rot([0, 0, 1], heading) @ _rot([0, 1, 0], -math.radians(tilt_deg)) @ _down()
    if roll:
        r = _rot(r[:, 2], roll) @ r
    return r


def approach(r: np.ndarray) -> np.ndarray:
    """Unit vector the fingers point along (TCP z axis)."""
    return np.asarray(r)[:, 2]


def closing_axis(r: np.ndarray) -> np.ndarray:
    """Unit vector the fingers close along (TCP x axis)."""
    return np.asarray(r)[:, 0]


def _grip_norm(finger: float) -> float:
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    return float(np.clip(finger / FINGER_OPEN, 0.0, 1.0)) if FINGER_OPEN != FINGER_CLOSED else 1.0


def move_pose(arm, pos, r, duration: float = 1.0, finger: float | None = None, iters: int = 60, log_err: bool = False):
    """Generator: move ``arm`` (a cabledemo Arm) to TCP position ``pos`` (world m) with rotation ``r`` (3x3),
    blending position (smoothstep) and orientation (slerp) from where its last command left it."""
    from yam_arena import ik

    p0, r0 = ik.tcp_pose(arm.q, arm.base)
    goal = np.asarray(pos, dtype=float)
    r = np.asarray(r, dtype=float)
    f0 = arm.finger
    f1 = f0 if finger is None else float(finger)
    n = max(1, round(duration * CONTROL_HZ))
    worst = (0.0, 0.0)
    for k in range(1, n + 1):
        s = ik.smoothstep(k / n)
        p = p0 + s * (goal - p0)
        rr = ik.slerp_matrix(r0, r, s)
        q, pe, re = ik.solve(p, ik.matrix_to_quat_xyzw(rr), arm.q, base_pos=arm.base, iters=iters)
        worst = (max(worst[0], pe), max(worst[1], re))
        arm.q = q
        arm.cmd = np.r_[q, _grip_norm(f0 + s * (f1 - f0))]
        yield
    arm.finger = f1
    arm._yaw = math.atan2(-r[1, 1], -r[0, 1])  # best straight-down yaw match, in case Arm.move follows
    if log_err and (worst[0] > 0.003 or worst[1] > 0.03):
        print(f"[tilt_moves] {arm.name}: IK error up to {worst[0] * 1000:.1f} mm / {math.degrees(worst[1]):.1f} deg",
              flush=True)


def grip_pose(arm, finger: float, duration: float = 0.6):
    """Generator: open/close in place, keeping the current (possibly tilted) pose."""
    from yam_arena import ik

    p, r = ik.tcp_pose(arm.q, arm.base)
    yield from move_pose(arm, p, r, duration=duration, finger=finger)


def wait(arm, duration: float):
    for _ in range(max(1, round(duration * CONTROL_HZ))):
        yield


def reachable(arm_name: str, pos, r, q0=None, iters: int = 300):
    """Offline check (pure Python): (ok, pos_err_m, rot_err_rad) for one arm reaching ``pos`` at rotation ``r``."""
    from yam_arena import ik
    from yam_arena.constants import READY
    from yam_arena.station import BASES

    q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(np.asarray(r, float)),
                         np.array(READY if q0 is None else q0, float), base_pos=BASES[arm_name], iters=iters)
    return bool(pe < 0.002 and re < 0.02), float(pe), float(re)


def _seg_dist(a0, a1, b0, b1) -> float:
    a0, a1, b0, b1 = (np.asarray(v, float) for v in (a0, a1, b0, b1))
    best = np.inf
    for s in np.linspace(0, 1, 11):
        p = a0 + s * (a1 - a0)
        d = b1 - b0
        t = float(np.clip(np.dot(p - b0, d) / max(np.dot(d, d), 1e-12), 0, 1))
        best = min(best, float(np.linalg.norm(p - (b0 + t * d))))
    return best


def gap_between(q_left, q_right) -> float:
    """Closest distance (m) between the two grippers' TCP-to-flange segments for joint vectors q_left/q_right."""
    from yam_arena import ik
    from yam_arena.station import BASES

    pl, rl = ik.tcp_pose(np.asarray(q_left, float), BASES["left"])
    pr, rr = ik.tcp_pose(np.asarray(q_right, float), BASES["right"])
    return _seg_dist(pl, pl - FLANGE_BACK * rl[:, 2], pr, pr - FLANGE_BACK * rr[:, 2])


def hands_gap(ctx) -> float:
    """``gap_between`` for the arms' current commands (cabledemo Ctx)."""
    return gap_between(ctx.left.q, ctx.right.q)
