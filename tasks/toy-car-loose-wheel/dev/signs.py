"""Development check: which closing sign the IK reaches for each grasp (tug and body grip)."""
import math, pathlib, sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import layout as L
from reach import err, frame
tilt = math.radians(L.TUG_TILT_DEG)
for arm in ("left", "right"):
    s = 1.0 if arm == "left" else -1.0
    a = np.array([0.0, -s * math.sin(tilt), -math.cos(tilt)])
    for y in L.SLOTS_Y:
        for ax in (L.AXLE_X, -L.AXLE_X):
            c = np.array([L.CAR_X + ax, y + s * (L.WHEEL_Y + 0.002), L.TOP + L.WHEEL_R]) - a * L.TCP_ABOVE_AXLE
            e = [round(err(arm, c, frame(a, (sg, 0, 0)))[0], 1) for sg in (1, -1)]
            print(f"{arm} tug y{y:+.2f} ax{ax:+.3f} close +x/-x: {e}", flush=True)
    p = math.radians(35)
    a = np.array([math.sin(p), 0.0, -math.cos(p)])
    for (x, y, z) in [(L.CAR_X, yy, L.PICK_TCP_Z) for yy in L.SLOTS_Y] + [(L.CAR_X, yy, L.LIFT_Z) for yy in L.SLOTS_Y] + [(sx, sy, L.LIFT_Z) for sx, sy in L.BASKET_SPOTS + (L.BOX_SPOT,)]:
        e = [round(err(arm, np.array([x, y, L.TOP + z]), frame(a, (0, sg, 0)))[0], 1) for sg in (1, -1)]
        print(f"{arm} body ({x:.2f},{y:+.2f},{z:.2f}) close +y/-y: {e}", flush=True)
