"""Static geometry check (no simulator): sample points on a beater placed in a socket and count those inside the
mixer's or the spindle's collision boxes. Run from the task folder with the yam_arena python."""

import math
import sys

import numpy as np

sys.path.insert(0, ".")
import hand_mixer_beaters as T  # noqa: E402
from bodykit import BoxShape, CylShape, SphereShape, rotation  # noqa: E402


def pts_of(shape, n=7):
    c = np.array(shape.centre)
    if isinstance(shape, BoxShape):
        r = rotation(shape.quat())
        h = np.array(shape.size) / 2
        g = np.linspace(-1, 1, n)
        out = [c + r @ (h * np.array([a, b, cc])) for a in g for b in g for cc in g
               if max(abs(a), abs(b), abs(cc)) == 1]
        return np.array(out)
    if isinstance(shape, CylShape):
        ax = "XYZ".index(shape.axis)
        out = []
        for t in np.linspace(-shape.height / 2, shape.height / 2, n):
            for k in range(24):
                a = 2 * math.pi * k / 24
                for rr in (shape.radius, shape.radius * 0.5):
                    v = np.zeros(3)
                    v[ax] = t
                    o = [i for i in range(3) if i != ax]
                    v[o[0]], v[o[1]] = rr * math.cos(a), rr * math.sin(a)
                    out.append(c + v)
        return np.array(out)
    out = []
    for k in range(60):
        phi, th = math.acos(1 - 2 * (k + 0.5) / 60), math.pi * (1 + 5 ** 0.5) * k
        out.append(c + shape.radius * np.array([math.sin(phi) * math.cos(th), math.sin(phi) * math.sin(th),
                                                math.cos(phi)]))
    return np.array(out)


def inside(p, shape, margin):
    if not isinstance(shape, BoxShape) or not shape.collide:
        return False
    r = rotation(shape.quat())
    loc = r.T @ (p - np.array(shape.centre))
    return bool(np.all(np.abs(loc) < np.array(shape.size) / 2 - margin))


def check(kind, sock, s, fin_deg, spin_deg=0.0, margin=0.0002):
    mixer = T._mixer_shapes()
    spin = T._spindle_shapes()
    f = np.array([T.FACE_X, T.SOCK_Y[sock], T.Z_S])
    rb = T._rx(math.radians(fin_deg))
    rs = T._rx(math.radians(spin_deg))
    hits = {"mixer": 0, "spindle": 0}
    for sh in T._beater_shapes(kind):
        for p in pts_of(sh):
            if p[0] < -0.12:
                continue
            w = f + rb @ p + np.array([s, 0, 0])
            if any(inside(w, m, margin) for m in mixer):
                hits["mixer"] += 1
            loc = rs.T @ (w - f)
            if any(inside(loc, m, margin) for m in spin):
                hits["spindle"] += 1
    return hits


for case in [("collared", "large", T.D_FULL, 0.0), ("plain", "small", T.D_FULL, 0.0), ("collared", "large", 0.030, 20.0),
             ("plain", "small", 0.027, 20.0), ("plain", "small", 0.0285, 20.0), ("collared", "small", T.D_FULL, 0.0),
             ("collared", "small", 0.0315, 0.0), ("plain", "large", T.D_FULL, 0.0), ("plain", "small", T.D_FULL, 5.0),
             ("plain", "small", T.D_FULL, 7.0), ("plain", "small", 0.0362, 0.0)]:
    print(case, check(*case))
lo, hi = T.OBJECTS[0].local_bounds()
print("mixer bounds", np.round(lo, 4), np.round(hi, 4), "shapes", len(T.OBJECTS[0].shapes))
for seed in (1, 2, 3):
    lay = T.layout(seed)
    print(seed, {k: (np.round(v[0][:2], 3).tolist() if isinstance(v, tuple) else round(v, 1)) for k, v in lay.items()
                 if k in ("mixer", "beater_collared", "beater_plain", "_spin_large", "_spin_small")})
