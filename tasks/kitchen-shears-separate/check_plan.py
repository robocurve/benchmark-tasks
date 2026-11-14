"""CPU-only warm-started 20 Hz IK check; does not test contacts or physics."""
import argparse
import json
import numpy as np
from yam_arena.constants import LEFT_BASE_POS, RIGHT_BASE_POS
from yam_arena.demo import ArmPlan
import task


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--base-y', type=float, default=None,
                        help='Optional absolute lateral base position in metres')
    args = parser.parse_args()
    result = {'kind': 'kinematics only; no simulation', 'arms': {}}
    for side, original in [('left', LEFT_BASE_POS), ('right', RIGHT_BASE_POS)]:
        base = np.array(original, dtype=float)
        if args.base_y is not None:
            base[1] = args.base_y * (1 if side == 'left' else -1)
        plan = ArmPlan(task.demo()[side], base)
        steps = int(round(plan.duration / task.DT))+1
        for i in range(steps):
            plan.action7(i * task.DT)
        result['arms'][side] = dict(base=base.tolist(), samples=steps,
            duration_s=plan.duration, max_position_mm=plan.worst_ik_mm,
            max_rotation_deg=plan.worst_ik_deg)
    result['pass'] = all(x['max_position_mm'] < 1.0 and x['max_rotation_deg'] < 1.0
                         for x in result['arms'].values())
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['pass'] else 1)


if __name__ == '__main__':
    main()
