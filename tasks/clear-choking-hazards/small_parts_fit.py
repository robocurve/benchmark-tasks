"""Does each object fit entirely inside the 16 CFR 1501.4 small-parts test cylinder?

Fixture: inner diameter 31.7 mm; the floor slopes, so the depth below the rim runs from 25.4 mm on one side to
57.1 mm on the other. A part "fits" if, in SOME orientation and position and without compression, no part of it
projects above the rim. All three constraints (inside the circle, below the rim plane, above the sloped floor)
are convex, so for a convex part it is enough to check points on its surface (box vertices, cylinder rim
circles, sphere surface). For each orientation we lift the part as high as it goes (top touching the rim plane)
and search horizontal positions. Reported slack = smallest distance (mm) by which any constraint is satisfied;
negative = the part cannot fit (best orientation found).
"""

import numpy as np

R = 31.7 / 2
D_SHALLOW, D_DEEP = 25.4, 57.1
rng = np.random.default_rng(0)


def depth(x):  # floor depth below the rim at horizontal position x (deep side at x = +R)
    return D_SHALLOW + (D_DEEP - D_SHALLOW) * (x + R) / (2 * R)


def sphere(d):
    v = rng.normal(size=(300, 3))
    return d / 2 * v / np.linalg.norm(v, axis=1, keepdims=True)


def box(a, b, c):
    return np.array([[sx * a, sy * b, sz * c] for sx in (-.5, .5) for sy in (-.5, .5) for sz in (-.5, .5)])


def cylinder(d, h):
    t = np.linspace(0, 2 * np.pi, 36, endpoint=False)
    ring = np.stack([d / 2 * np.cos(t), d / 2 * np.sin(t)], 1)
    return np.vstack([np.c_[ring, np.full(36, -h / 2)], np.c_[ring, np.full(36, h / 2)]])


def rotations(n):
    q = rng.normal(size=(n, 4))
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    w, x, y, z = q.T
    rand = np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1)], 1)
    # Axis-aligned orientations plus small tilts about them (the tight fits live there).
    base = [np.eye(3)[list(p)] for p in ((0, 1, 2), (1, 2, 0), (2, 0, 1))]
    tilts = []
    for B in base:
        for ax in range(3):
            for a in np.radians(np.linspace(-40, 40, 41)):
                c, s = np.cos(a), np.sin(a)
                Rm = np.eye(3)
                i, j = [k for k in range(3) if k != ax]
                Rm[i, i], Rm[i, j], Rm[j, i], Rm[j, j] = c, -s, s, c
                tilts.append(Rm @ B)
    return np.concatenate([rand, np.array(tilts)])


GRID = np.stack(np.meshgrid(np.linspace(-R, R, 41), np.linspace(-R, R, 41)), -1).reshape(-1, 2)


def _slack_at(p, grid):
    xy = p[None, :, :2] + grid[:, None, :]  # (grid, points, 2)
    radial = R - np.linalg.norm(xy, axis=2)
    floor = p[None, :, 2] + depth(xy[..., 0])  # height above the sloped floor
    s = np.minimum(radial, floor).min(axis=1)
    i = int(s.argmax())
    return float(s[i]), grid[i]


def best_slack(points, n_rot=400, keep=8):
    """Coarse search over orientations and positions, then a fine (0.02 mm) position search for the best few."""
    found = []
    for Rm in rotations(n_rot):
        p = points @ Rm.T
        p = p - [0, 0, p[:, 2].max()]  # top of the part at the rim plane
        s, at = _slack_at(p, GRID)
        found.append((s, at, p))
    found.sort(key=lambda f: -f[0])
    fine = np.stack(np.meshgrid(np.linspace(-1, 1, 101), np.linspace(-1, 1, 101)), -1).reshape(-1, 2)
    return max(_slack_at(p, at + fine)[0] for s, at, p in found[:keep])


OBJECTS = {
    "16 mm sphere":                (sphere(16), True),  # orientation-free
    "20 mm cube":                  (box(20, 20, 20), True),
    "29.6 dia x 22 mm cylinder":   (cylinder(29.6, 22), True),
    "20.9 x 20.9 x 24 mm box":     (box(20.9, 20.9, 24), True),
    "34 mm sphere":                (sphere(34), False),
    "28 dia x 60 mm cylinder":     (cylinder(28, 60), False),
    "60 mm cube":                  (box(60, 60, 60), False),
}

if __name__ == "__main__":
    ok = True
    for name, (pts, hazard) in OBJECTS.items():
        s = best_slack(pts)
        fits = s >= 0
        ok &= fits == hazard and abs(s) >= 1.0
        print(f"{name:26s} fits={fits!s:5s} best slack {s:+6.2f} mm   claimed {'hazard' if hazard else 'safe'}"
              f"   {'OK' if fits == hazard and abs(s) >= 1.0 else 'MISMATCH OR UNDER 1 MM'}")
    print("ALL CLAIMS HOLD WITH >= 1.0 MM SLACK" if ok else "SOME CLAIMS WRONG OR TOO TIGHT")
