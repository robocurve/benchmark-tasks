"""Static check of seated(): a bag slid sideways along the hooks. Old (axis extrapolated) vs new (finite rod)."""
import importlib.util
import sys
import numpy as np

spec = importlib.util.spec_from_file_location("t", sys.argv[1])
T = importlib.util.module_from_spec(spec)
spec.loader.exec_module(T)


class P:
    def __init__(self, pos):
        self.pos = np.asarray(pos, float)

    def rotation(self):
        return np.eye(3)

    def to_world(self, p):
        return self.pos + np.asarray(p, float)

    def to_local(self, p):
        return np.asarray(p, float) - self.pos


mower = P((0, 0, 0))
x0 = sum(T.SLOT) / 2          # rod centre in the slot, mower frame
z0 = T.R_HOOK


def old_seated(objs):
    m, b = objs["mower"], objs["bag"]
    ok = True
    for s in (1, -1):
        a = m.to_local(b.to_world((0.0, -0.1, 0.0)))
        c = m.to_local(b.to_world((0.0, 0.1, 0.0)))
        t = (s * T.HOOK_Y - a[1]) / (c[1] - a[1])
        p = a + t * (c - a)
        ok &= T.SEAT_TOL_X[0] <= p[0] <= T.SEAT_TOL_X[1] and T.SEAT_TOL_Z[0] <= p[2] <= T.SEAT_TOL_Z[1]
    return bool(ok)


print("shift_y_mm  old_seated  new_seated")
for dy in (0.0, 0.03, 0.06, 0.062, 0.066, 0.08, 0.12, 0.30):
    objs = {"mower": mower, "bag": P((x0, dy, z0))}
    print(f"{dy*1000:9.0f}  {old_seated(objs)!s:>10}  {T.seated(objs)!s:>10}")
