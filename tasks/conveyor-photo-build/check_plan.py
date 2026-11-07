"""Offline check of the scripted demo, no GPU: replays both arms' plans through the URDF kinematics at 20 Hz.

    python tasks/conveyor-photo-build/check_plan.py

Asserts every IK solution is within 1 mm and 1 degree of its target and inside the joint limits, that each
catch closes on its part where the hand is (the part's predicted centre within 5 mm of the TCP along the belt),
that the two hands stay at least 15 cm apart, and that the plan fits the 2,400-step run limit.
"""
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from yam_arena import ik  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.station import BASES  # noqa: E402

from demo_plan import CATCH, VARIANTS, plans, part_y  # noqa: E402
from layout import BELT_X, EXTRA, leaves_belt_s  # noqa: E402

DT = 0.05
RUN_LIMIT_S = 120.0


def replay(planner):
    plan = ArmPlan(list(planner.out), BASES[planner.arm])
    rows = []
    for k in range(int(round(plan.duration / DT)) + 1):
        t = (k + 1) * DT
        target, _, _, label = plan.target(t)
        action, _ = plan.action7(t)
        q = action[:6]
        tcp, _ = ik.tcp_pose(q, np.asarray(BASES[planner.arm]))
        margin = float(np.min(np.minimum(q - ik.LOWER, ik.UPPER - q)))
        rows.append((t, tcp, np.asarray(target), label, margin))
    return plan, rows


def check(variant):
    print(f'== {variant}')
    left, right = plans(variant)
    out = {}
    for planner in (left, right):
        plan, rows = replay(planner)
        out[planner.arm] = rows
        worst = min(r[4] for r in rows)
        print(f'{planner.arm}: {plan.duration:.1f} s, worst IK {plan.worst_ik_mm:.2f} mm / {plan.worst_ik_deg:.2f} deg, '
              f'least joint margin {np.degrees(worst):.1f} deg')
        assert plan.worst_ik_mm < 1.0 and plan.worst_ik_deg < 1.0, planner.arm
        assert worst > np.radians(2.0), f'{planner.arm} arm runs within 2 degrees of a joint limit'
        assert plan.duration < RUN_LIMIT_S - 10
    n = min(len(out['left']), len(out['right']))
    gap = min(float(np.linalg.norm(out['left'][k][1] - out['right'][k][1])) for k in range(n))
    print(f'closest the two TCPs come: {gap * 100:.1f} cm')
    assert gap > 0.15
    return out


def main():
    for variant in VARIANTS[1:]:
        check(variant)
    out = check('demo')
    for name, at in CATCH.items():
        if name == 'square_post':
            continue  # caught only in the keep_extra baseline
        arm = 'right' if name == 'post_b' else 'left'
        rows = out[arm]
        # The hand at the moment the fingers start to close, and the part's predicted centre then.
        t, tcp, *_ = min(rows, key=lambda r: abs(r[0] - at))
        y = part_y(name, t)
        print(f'catch {name:7s} by the {arm:5s} arm at {at:5.1f} s: part y {y:+.3f}, TCP ({tcp[0]:.3f}, {tcp[1]:+.3f}, '
              f'{tcp[2]:.3f}); along-belt error {1000 * (tcp[1] - y):+.1f} mm')
        assert abs(tcp[1] - y) < 0.005 and abs(tcp[0] - BELT_X) < 0.005, name
    for name in EXTRA:
        print(f'{name} rides off the belt at {leaves_belt_s(name):.1f} s')
    print('plan OK')


if __name__ == '__main__':
    main()
