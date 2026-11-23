"""Reference action sequence. Outputs 14 YAM joint targets. It has no fixture write access."""

import numpy as np
from scipy.spatial.transform import Rotation
from yam_arena import ik
from yam_arena.constants import READY
from yam_arena.station import BASES

from evd.layout import DATUM, LAYOUTS, MODES

FRONT = Rotation.from_quat([0, 2**-0.5, 0, 2**-0.5])


def make_plan(mode):
    layout, style = MODES[mode]
    ear, zero = LAYOUTS[layout]
    start = DATUM + zero / 1000
    finish = DATUM + ear / 1000 if style not in ("open", "idle") else start
    left = np.array([0.315, 0.045, start])
    brake = np.array([0.294 if finish > start else 0.315, -0.065, start + 0.045])
    # Allow for joint and pad deflection while carrying the sprung brake and carriage.
    # When the pin returns, both arms still support the load above the intended tooth.
    take_up = 0.0095 if finish < start else 0.010
    seating_travel = np.array([0, 0, finish - start + take_up])
    press = np.array([0.017 if finish < start else 0.009, 0, 0])
    arms = {}
    for side in ("left", "right"):
        base = BASES[side]
        origin, ori = ik.tcp_pose(np.array(READY), base)
        q = np.array(READY)
        keys = [(0.0, q.copy(), 1.0)]
        t = 0.0
        current = origin
        cr = Rotation.from_matrix(ori)
        prev_grip = 1.0

        def add(duration, pos, rot=FRONT, grip=1.0, joint_space=False):
            nonlocal current, cr, q, t, prev_grip
            pos = np.array(pos)
            relative = (rot * cr.inv()).as_rotvec()
            # Joint-space entry from READY avoids crossing a wrist singularity in Cartesian space.
            n = (
                1
                if joint_space
                else max(
                    1,
                    int(
                        np.ceil(
                            max(
                                np.linalg.norm(pos - current) / 0.009,
                                np.linalg.norm(relative) / 0.12,
                            )
                        )
                    ),
                )
            )
            for i in range(1, n + 1):
                u = i / n
                v = 3 * u * u - 2 * u * u * u
                target = current + v * (pos - current)
                rr = Rotation.from_rotvec(relative * v) * cr
                if (
                    np.linalg.norm(target - current) > 1e-8
                    or np.linalg.norm(relative) > 1e-8
                ):
                    q, pe, re = ik.solve(
                        target, rr.as_quat(), q, base_pos=base, iters=120
                    )
                    if pe > 0.0015 or re > 0.025:
                        raise ValueError(
                            f"Unreachable {side} point {target}, pos {pe}, rot {re}"
                        )
                keys.append(
                    (t + duration * u, q.copy(), prev_grip + v * (grip - prev_grip))
                )
            t += duration
            current = pos
            cr = rot
            prev_grip = grip

        def wait(duration):
            add(duration, current, cr, prev_grip)

        def back(pt):
            return pt - np.array([0.048, 0, 0])

        def home():
            nonlocal t
            t += 2.5
            keys.append((t, np.array(READY), 1.0))

        def tap_operation(h):
            # Pinch the exposed end of the knob, with both pads forward of the spindle.
            pivot = np.array([0.298, -0.085, h - 0.050])
            pt = pivot + [0, 0, 0.020]
            add(1.5, back(pt), joint_space=True)
            add(1.2, pt)
            add(0.75, pt, grip=0.06)
            for a in range(10, 91, 10):
                rot = Rotation.from_euler("x", -a, degrees=True)
                pt = pivot + rot.apply([0, 0, 0.020])
                add(0.22, pt, rot * FRONT, 0.06)
            wait(0.6)
            opening = 0.45 if style == "held" else 1.0
            add(0.7, pt, rot * FRONT, opening)
            add(0.9, back(pt), rot * FRONT, opening)
            add(1.4, back(pt) + [-0.006, -0.105, 0.012], rot * FRONT, opening)
            home()

        adjust = style in ("guide", "pause", "held", "blocked") and finish != start
        if style != "idle":
            if adjust:
                p = left if side == "left" else brake
                side_press = side == "right" and finish > start
                # On an upward adjustment the pads press the sides of the brake tab.
                # The left gripper bears the carriage load through the lift neck.
                brake_pose = Rotation.from_euler("x", -90, degrees=True) * FRONT if side_press else FRONT
                grasp = 0.075 if side == "left" else (0.12 if side_press else 0.09)
                # Leave room for the adjacent wrist during the upward two-arm move.
                # Both pads clear their handles at this opening before backing away.
                transit_grip = 0.45 if finish > start else 1.0
                add(2.6, back(p), brake_pose, grip=transit_grip, joint_space=True)
                add(1.5, p, brake_pose, grip=transit_grip)
                add(0.8, p, brake_pose, grip=grasp)
                d = press if side == "right" else np.zeros(3)
                add(
                    1.15 if finish < start else 0.9,
                    p + d + [0, 0, 0.002 if finish < start else 0],
                    rot=brake_pose, grip=prev_grip,
                )
                add(3.5, p + d + seating_travel, rot=brake_pose, grip=prev_grip)
                add(
                    0.8,
                    p
                    + seating_travel
                    - np.array([0.010 if side == "right" else 0.0, 0, 0]),
                    rot=brake_pose, grip=prev_grip,
                )
                wait(0.6)
                if not (style == "held" and side == "left"):
                    add(0.8, current, brake_pose, grip=transit_grip)
                    add(1.0, back(p + seating_travel), brake_pose, grip=transit_grip)
                    if side != "right":
                        home()
                    elif style == "pause":
                        # Clear the brake before returning to READY with the tap closed.
                        add(1.5, [0.250, -0.085, finish - 0.030], joint_space=True)
                        add(1.5, [0.230, -0.190, finish - 0.030])
                        home()
                    else:
                        tap_operation(finish)
            elif side == "right" and style != "dream":
                add(2.0, [0.285, -0.210, start - 0.030], joint_space=True)
                tap_operation(start)
        arr = np.array([[t, *v, g] for t, v, g in keys])
        arms[side] = arr
    parked = {"no_lift_support": "left", "no_brake_release": "right"}.get(mode)
    if parked:
        arms[parked] = arms[parked][:1]
    return arms


def action(plan, t):
    return [
        np.array([np.interp(t, path[:, 0], path[:, k]) for k in range(1, 8)])
        for path in plan.values()
    ]
