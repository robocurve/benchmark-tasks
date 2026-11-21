"""Reach check: switch press with the kettle at x 0.50 / 0.54; pinch near x 0.31."""
import math, numpy as np
exec(open("dev/reach2.py").read().split("for kx in")[0])
for kx in (0.50, 0.54):
    print(kx, "high", f"{press(kx, 30, 0.12, -0.06)[0]:.0f}", "on/pressed", [f"{v:.0f}" for v in press(kx, 30, 0.12, -0.06)[1:]])
dn = np.column_stack([[1, 0, 0], [0, -1, 0], [0, 0, -1.0]])
for p in ((0.31, 0.60, T + 0.014), (0.31, 0.60, T + 0.11), (0.28, 0.52, T + 0.15)):
    print(p[:2], f"{err('left', p, dn):.0f}")
