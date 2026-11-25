"""Offline reach scan for the photocopy prop: named points, both arms, three grip directions.

Run from a yam-isaac-lab-arena checkout: PYTHONPATH=. python3 <this file>. Needs only numpy (no Isaac).
Printer body 425 x 304 x 154 mm, front face at x 0.22 m, centred on y 0; scanner lid hinged at the back.
"""
import numpy as np
from yam_arena import ik
from yam_arena.constants import LEFT_BASE_POS, RIGHT_BASE_POS, TABLE_HEIGHT
from yam_arena.demo import grip_rotation
from yam_arena.urdf import axis_angle_matrix

T = TABLE_HEIGHT
FRONT, DEPTH, H = 0.22, 0.304, 0.154
HINGE_X, LID_R = FRONT + DEPTH - 0.015, 0.28  # hinge axis near the back edge; lid front lip 28 cm from it
points = {
    "card pick (book edge, right)": (0.22, -0.30, T + 0.03),
    "glass front-right corner": (0.26, -0.15, T + H + 0.01),
    "card centre placed": (0.28, -0.11, T + H + 0.01),
    "copy button (front left) [position UNVERIFIED]": (0.235, 0.15, T + H + 0.005),
}
for deg in (0, 30, 60, 75, 90):
    a = np.radians(deg)
    for side, y in (("right", -0.10), ("left", 0.10)):
        points[f"lid lip at {deg} deg ({side})"] = (HINGE_X - LID_R * np.cos(a), y, T + H + LID_R * np.sin(a))

down = grip_rotation(0.0)
grips = {
    "down": down,
    "tilt 45 toward +x": axis_angle_matrix(np.array([0.0, 1.0, 0.0]), -np.pi / 4) @ down,
    "level toward +x": axis_angle_matrix(np.array([0.0, 1.0, 0.0]), -np.pi / 2) @ down,
}
seeds = ([0, 1, 1, -0.6, 0, 0], [0, 0.5, 0.5, 0, 0, 0], [0, 1.5, 1.5, -1, 0, 0], [0, 1.0, 0.5, 0.5, 0, 0], [0, 2.0, 1.5, 0, 0, 0])
print(f"{'point':52s} {'arm':5s} " + " ".join(f"{g:>18s}" for g in grips))
for name, p in points.items():
    for arm, base in (("right", RIGHT_BASE_POS), ("left", LEFT_BASE_POS)):
        cells = []
        for r in grips.values():
            quat = ik.matrix_to_quat_xyzw(r)
            best = min((ik.solve(p, quat, q_init=s, base_pos=base, iters=400)[1:] for s in seeds), key=lambda e: e[0] + 0.05 * e[1])
            cells.append("ok" if best[0] < 0.002 and best[1] < np.radians(2) else f"{best[0]*1000:.0f} mm off")
        print(f"{name:52s} {arm:5s} " + " ".join(f"{c:>18s}" for c in cells))
