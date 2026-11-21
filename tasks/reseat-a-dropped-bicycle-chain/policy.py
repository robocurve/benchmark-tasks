"""Robot joint targets; all forces on the chain and pedal come from physical contact."""

from math import tau, cos, sin
import numpy as np
from fixtures import AXLES, RADII


def reseat(s, partial=False, guide=True, test_before=False, turn_budget=None):
    front = AXLES[0]
    center = front + [-0.118, 0, 0.040]

    def pedal(a):
        return center + np.array([0, -0.058 * cos(a), -0.058 * sin(a)])

    def move(side, p, duration, phase, opening=None, yaw=0):
        s.move_arm(side, p, duration, phase, opening, yaw=yaw)

    a = s.joint("front_angle")
    p = pedal(a)
    move("right", p + [-0.047, 0, 0], 1.1, "Approach along the pedal spindle", 0.08)
    a = s.joint("front_angle")
    p = pedal(a)
    move("right", p, 0.65, "Slide socket over the pedal spindle", 0.08)
    s.wait(0.25)
    if test_before:
        # Test the fault before touching the chain. Return to the same crank phase.
        start_angle = s.joint("front_angle")
        for i in range(1, 102):
            target = pedal(start_angle + tau * i / 101)
            commands = s.targets.copy()
            commands[1, :6], _, _ = s.solve(
                "right", target, s.yaw_choices["right"][1], commands[1, :6]
            )
            s.phase = "Test the dropped chain"
            s.action(commands)
        s.wait(0.45)
    # Place the fingers on the inboard side of the loose incoming run.
    q = front + np.array([0.094, 0.000, RADII[0] + 0.0345])
    release_step = 65
    if guide and s.fixture.lift_pick:
        # Enter under the free upper span, lift over the tooth tips, then feed
        # toward the top tangent. Everything after reset is robot contact.
        pos = s.data.xpos[s.fixture.ids]
        target_y = front[1] + 0.064
        upper = pos[pos[:, 2] > front[2]]
        near_axle = upper[abs(upper[:, 1] - front[1]) < 0.016]
        radial_drop = (
            front[2] + RADII[0] - near_axle[:, 2].max()
            if len(near_axle)
            else float("inf")
        )
        shallow = radial_drop < 0.007
        pick = upper[np.argmin(abs(upper[:, 1] - target_y))]
        grip = pick + np.array([0.070 + 0.007, 0, 0.0385 - 0.010])
        move(
            "left",
            grip + [0.025, 0, 0.010],
            0.65,
            "Approach the hanging upper span",
            0.12,
        )
        move("left", grip + [0.025, 0, 0], 0.45, "Lower beside the loose chain", 0.12)
        move("left", grip, 0.60, "Slide the pick under the chain", 0.12)
        move(
            "left",
            front + [0.086, 0.054, 0.100],
            0.8,
            "Lift the loose chain above the teeth",
            0.12,
        )
        if shallow:
            # A high upper span can be laid directly over the top tangent.
            move(
                "left",
                front + [0.080, 0.011, 0.103],
                0.80,
                "Carry rollers across the tooth tips",
                0.12,
            )
            move(
                "left",
                front + [0.079, 0.006, 0.098],
                0.50,
                "Lower rollers onto the front ring",
                0.12,
            )
            q = front + np.array([0.080, 0.041, 0.091])
            move("left", q, 0.75, "Slide the pick clear along the upper span", 0.12)
            release_step = 95
        else:
            # A low, loose span needs support farther upstream while the crank feeds it.
            move(
                "left",
                front + [0.078, 0.030, 0.100],
                0.60,
                "Carry rollers across the tooth tips",
                0.12,
            )
            q = front + np.array([0.077, 0.029, 0.091])
            move("left", q, 0.50, "Lower rollers onto the front ring", 0.12)
            move(
                "left",
                q + [0.006, 0.013, -0.009],
                0.45,
                "Let the teeth support the upper run",
                0.12,
            )
    elif guide:
        move(
            "left",
            q + [0.005, 0.003, 0.006],
            0.6,
            "Approach the dropped upper run",
            0.12,
        )
        move("left", q, 0.45, "Lower fingers beside the upper run", 0.12)
        move(
            "left",
            q + [-0.0195, 0, 0],
            0.65,
            "Guide the rollers to the tooth plane",
            0.12,
        )
    a = s.joint("front_angle")
    count = 155 if partial else (turn_budget or (790 if s.fixture.lift_pick else 430))
    for i in range(1, count + 1):
        target = pedal(a + i * 0.066)
        side = "right"
        targets = s.targets.copy()
        joints, error, _ = s.solve(side, target, s.yaw_choices[side][1], targets[1, :6])
        targets[1, :6] = joints
        s.phase = (
            "Turn the crank with the chain dropped"
            if not guide
            else (
                "Feed the first links onto the teeth"
                if i < 25
                else "Turn the crank through two revolutions"
            )
        )
        s.action(targets)
        if guide and i == release_step:
            if s.fixture.lift_pick and shallow:
                move(
                    "left",
                    q + [0, 0.025, -0.010],
                    0.5,
                    "Lower the pick clear of the chain",
                    0.12,
                )
            move("left", q + [0.028, 0.026, 0.004], 0.65, "Withdraw guiding hand", 0.7)
        # In the fault trial, release the socket in the reachable forward sector.
        ready_to_release = not test_before or 0.4 < s.joint("front_angle") % tau < 0.6
        if (s.fixture.turns >= 2.08 and s.fixture.proven() and ready_to_release) or (
            not guide and s.joint("front_angle") - a >= 2.35 * tau
        ):
            break
    p = s.data.site_xpos[s.sites["right"]].copy()
    move("right", p + [-0.036, 0, 0], 0.65, "Withdraw the pedal socket", 0.08)
    move(
        "right",
        p + [-0.036, -0.01, 0.008],
        0.45,
        "Check both sprockets after release",
        0.95,
    )
    s.wait(0.45)
