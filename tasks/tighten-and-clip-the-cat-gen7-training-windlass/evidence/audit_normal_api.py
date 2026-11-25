"""Compare two native normal-force API outputs; not physical trainer evidence."""
import json, math, sys
from pathlib import Path

raw = Path(sys.argv[1])
worst = worst_opposite = 0.
rows = points = 0
for line in raw.open():
    r = json.loads(line)
    s = r['state']
    pairs = s.get('clip_normal_contacts')
    if pairs is None:
        continue
    matrix = s['lip_forces'][0] + s['lip_forces'][1] + s['clip_base_forces']
    assert len(pairs) == len(matrix) == 6
    for contacts, target in zip(pairs, matrix):
        if not contacts:
            continue
        rows += 1
        points += len(contacts)
        normal_sum = [sum(p['normal_force_n'] * p['normal_world'][j]
                          for p in contacts) for j in range(3)]
        worst = max(worst, math.dist(normal_sum, target))
        worst_opposite = max(worst_opposite,
                             math.dist([-v for v in normal_sum], target))
sign = 1 if worst <= worst_opposite else -1
result = dict(kind='native API consistency, not hardware measurement',
              contact_pairs_observed=rows, contact_points_observed=points,
              maximum_vector_error_n=worst,
              maximum_opposite_sign_error_n=worst_opposite,
              matrix_equals_normal_sum_times_sign=sign,
              normal_matrix_matches_point_sum=bool(rows and min(worst, worst_opposite) < .01))
print(json.dumps(result, indent=2))
if not result['normal_matrix_matches_point_sum']:
    raise SystemExit(1)
