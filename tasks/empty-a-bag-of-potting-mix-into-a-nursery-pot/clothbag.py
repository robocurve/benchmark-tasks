"""clothbag: the bag as one triangle mesh of film (NumPy only, no Isaac Sim).

The bag is an opened stand-up quad-seal bag: a tube of film with a flat closed bottom, its top cut off square. Filled,
its cross-section bulges to a rounded oval (a superellipse here) DEPTH across (x, front panel at -x) by WIDTH (y),
with its four vertical corner seals at the oval's diagonals. The mesh is the rest shape, standing in the bag frame:
origin at the middle of the bottom, z up the bag to the cut mouth at HEIGHT.

The two front corner seals are where the hands hold the bag. Each seal is a strip of fused film sticking out of its
corner along the corner's bisector; where a hand pinches it, it is modelled as a small rigid tab that the film is
attached to (see TABS).
"""

from __future__ import annotations

import math

import numpy as np

DEPTH, WIDTH = 0.100, 0.190  # filled cross-section across x and y (m): a bag about 250 mm wide laid flat
HEIGHT = 0.150  # bottom to the cut mouth
SHAPE_N = 2.6  # superellipse exponent: 2 is an ellipse, larger is squarer
EDGE = 0.0065  # target triangle edge (m); smaller than a grain (9 to 12 mm across)
BOTTOM_RINGS = 10

# Rigid tabs at the two front corner seals, where the hands pinch: (name, z low, z high, side: +1 is +y).
SEAL_OUT = 0.035  # how far a tab sticks out in front of the bag's corner
TAB_IN = 0.005  # and how far it reaches into the corner (enclosing the film vertices it holds)
TAB_T = 0.012  # modelled thickness of a pinched gather of seal (see DESIGN.md)
# The mouth hand holds its corner 25 to 55 mm below the cut edge, where the mix behind the film backs the seal (an
# unbacked seal slides out toward the fingertips as they close); the bottom hand holds its corner low on the fill.
TABS = (("mouth_tab", 0.095, 0.125, -1), ("bottom_tab", 0.050, 0.085, +1))
# How far each tab is turned from straight forward (-x) toward its side. A loose corner seal can lie anywhere between
# straight forward and the corner's bisector (58 degrees); the hands find them turned forward, toward the arms.
TAB_YAW_DEG = {"mouth_tab": 0.0, "bottom_tab": 0.0}


def _outline(n: int) -> tuple[np.ndarray, np.ndarray]:
    """``n`` points around the superellipse, evenly spaced by arc length, and the index of each of the four
    corners (at the diagonals) in the order (-x,+y), (-x,-y), (+x,-y), (+x,+y)."""
    a, b, e = DEPTH / 2, WIDTH / 2, 2.0 / SHAPE_N
    t = np.linspace(0, 2 * math.pi, 4000, endpoint=False)
    x = a * np.sign(np.cos(t)) * np.abs(np.cos(t)) ** e
    y = b * np.sign(np.sin(t)) * np.abs(np.sin(t)) ** e
    seg = np.hypot(np.diff(x, append=x[0]), np.diff(y, append=y[0]))
    s = np.r_[0.0, np.cumsum(seg)[:-1]]
    want = np.linspace(0, s[-1] + seg[-1], n, endpoint=False)
    k = np.searchsorted(s, want).clip(0, len(t) - 1)
    pts = np.stack([x[k], y[k]], axis=1)
    corners = []
    for sx, sy in ((-1, 1), (-1, -1), (1, -1), (1, 1)):
        d = np.array([sx * a, sy * b])
        corners.append(int(np.argmax(pts @ (d / np.linalg.norm(d)) / np.linalg.norm(pts, axis=1).clip(1e-9)
                                     - 1e-3 * np.linalg.norm(pts - d, axis=1))))
    return pts, np.array(corners)


def perimeter_count() -> int:
    pts, _ = _outline(400)
    per = np.linalg.norm(np.roll(pts, -1, 0) - pts, axis=1).sum()
    return int(round(per / EDGE / 4)) * 4


def mesh():
    """(points V x 3, triangles F x 3, info) of the bag's rest shape in the bag frame. ``info`` holds the wall's
    (rows, columns) and the vertex index of wall row r, column c is ``r * cols + c``; row 0 is the bottom edge."""
    cols = perimeter_count()
    ring, corners = _outline(cols)
    rows = int(round(HEIGHT / EDGE)) + 1
    zs = np.linspace(0.0, HEIGHT, rows)
    pts = [np.c_[ring, np.full(cols, z)] for z in zs]
    faces = []
    for r in range(rows - 1):
        for c in range(cols):
            a, b = r * cols + c, r * cols + (c + 1) % cols
            a2, b2 = a + cols, b + cols
            faces += [(a, b, b2), (a, b2, a2)] if (r + c) % 2 == 0 else [(a, b, a2), (b, b2, a2)]
    # the bottom: concentric shrinking outlines, each with fewer points, down to a centre point
    n_wall = rows * cols
    prev = np.arange(cols)  # the wall's bottom row
    prev_xy = ring
    base = n_wall
    for k in range(1, BOTTOM_RINGS):
        f = 1.0 - k / BOTTOM_RINGS
        m = max(8, int(round(cols * f / 4)) * 4)
        sub, _ = _outline(m)
        sub = sub * f
        idx = np.arange(base, base + m)
        pts.append(np.c_[sub, np.zeros(m)])
        base += m
        faces += _stitch(prev, prev_xy, idx, sub)
        prev, prev_xy = idx, sub
    centre = base
    pts.append(np.zeros((1, 3)))
    for i in range(len(prev)):
        faces.append((prev[(i + 1) % len(prev)], prev[i], centre))
    pts = np.concatenate(pts)
    faces = np.array(faces, int)
    return pts, faces, {"rows": rows, "cols": cols, "corners": corners, "n_wall": n_wall}


def _stitch(outer, outer_xy, inner, inner_xy):
    """Triangles between two closed loops (outer, then inner, both counter-clockwise), facing down (-z)."""
    ang_o = np.arctan2(outer_xy[:, 1], outer_xy[:, 0])
    ang_i = np.arctan2(inner_xy[:, 1], inner_xy[:, 0])
    i0 = int(np.argmin(np.abs(np.angle(np.exp(1j * (ang_i - ang_o[0]))))))
    inner = np.roll(inner, -i0)
    ang_i = np.roll(ang_i, -i0)
    no, ni = len(outer), len(inner)
    uo = np.unwrap(ang_o - ang_o[0]) % (2 * math.pi)
    ui = np.unwrap(ang_i - ang_o[0])
    ui = (ui - ui[0]) % (2 * math.pi) + (ui[0] - 0)  # start near 0
    faces = []
    a = b = 0
    while a < no or b < ni:
        na, nb = (a + 1) % no, (b + 1) % ni
        go_outer = b >= ni or (a < no and (uo[a + 1] if a + 1 < no else 2 * math.pi)
                                <= (ui[b + 1] if b + 1 < ni else 2 * math.pi + ui[0]))
        if go_outer:
            faces.append((outer[a], inner[b % ni], outer[na]))
            a += 1
        else:
            faces.append((outer[a % no], inner[b], inner[nb]))
            b += 1
    return faces


def corner_column(info, which: int) -> int:
    return int(info["corners"][which])


def tab_spec(name):
    """(z0, z1, side) of a tab; side +1 is the +y (left) front corner."""
    for tab, z0, z1, side in TABS:
        if tab == name:
            return z0, z1, side
    raise KeyError(name)


def tab_out(name) -> np.ndarray:
    """The unit direction (bag frame, horizontal) a tab sticks out in: TAB_YAW_DEG from straight forward toward its
    side."""
    side = tab_spec(name)[2]
    a = math.radians(TAB_YAW_DEG[name])
    return np.array([-math.cos(a), side * math.sin(a), 0.0])


def tab_corner(name, pts=None, info=None) -> np.ndarray:
    """The film's corner (bag frame, at the tab's mid height) that the tab grows out of."""
    if pts is None:
        pts, _, info = mesh()
    z0, z1, side = tab_spec(name)
    c = pts[corner_column(info, 0 if side > 0 else 1)]
    return np.array([c[0], c[1], (z0 + z1) / 2])


def tab_box(name, pts=None, info=None):
    """A tab's box in the bag frame: (centre, size, yaw). The tab is a front corner seal, gathered flat: it runs out
    of the film's corner along the corner's bisector (``tab_out``), from TAB_IN inside the corner to SEAL_OUT outside
    it, TAB_T thick; ``size`` is (length along the bisector, thickness, height) and ``yaw`` the bisector's angle about
    z. The film vertices it encloses are fixed to it."""
    z0, z1, _ = tab_spec(name)
    out = tab_out(name)
    corner = tab_corner(name, pts, info)
    centre = corner + out * (SEAL_OUT - TAB_IN) / 2
    return centre, np.array([SEAL_OUT + TAB_IN, TAB_T, z1 - z0]), math.atan2(out[1], out[0])


def tab_vertices(name, pts=None, info=None) -> np.ndarray:
    """Film vertices inside a tab's box."""
    if pts is None:
        pts, _, info = mesh()
    c, size, yaw = tab_box(name, pts, info)
    rot = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    local = (pts - c) @ rot
    return np.nonzero((np.abs(local) <= size / 2 + 1e-9).all(axis=1))[0]


def mouth_rows(info) -> np.ndarray:
    """Vertex indices of the top (cut) edge."""
    r = info["rows"] - 1
    return np.arange(r * info["cols"], (r + 1) * info["cols"])


def inside(points, film, faces, mouth_idx) -> np.ndarray:
    """Which ``points`` (N x 3) lie inside the bag: inside the closed surface made of the film's triangles plus a lid
    of triangles fanned across the mouth from its centroid. ``film`` is the current vertex positions. Parity of
    crossings of a ray along +z (with a tiny tilt so it misses edges)."""
    mouth = film[mouth_idx]
    lid_c = mouth.mean(axis=0)
    tri = film[faces]
    m = len(mouth_idx)
    lid = np.stack([mouth, np.roll(mouth, -1, 0), np.repeat(lid_c[None], m, 0)], axis=1)
    tri = np.concatenate([tri, lid])
    return _parity(np.asarray(points, float), tri)


def _parity(p, tri):
    d = np.array([1e-4, 2e-4, 1.0])
    d /= np.linalg.norm(d)
    v0, v1, v2 = tri[:, 0], tri[:, 1], tri[:, 2]
    e1, e2 = v1 - v0, v2 - v0
    h = np.cross(d, e2)
    a = (e1 * h).sum(1)
    ok = np.abs(a) > 1e-12
    v0, e1, e2, h, a = v0[ok], e1[ok], e2[ok], h[ok], a[ok]
    f = 1.0 / a
    out = np.zeros(len(p), bool)
    for s in range(0, len(p), 256):
        q = p[s:s + 256]
        sv = q[:, None, :] - v0[None]
        u = f[None] * (sv * h[None]).sum(2)
        qv = np.cross(sv, e1[None])
        v = f[None] * (qv @ d)
        t = f[None] * (qv * e2[None]).sum(2)
        hit = (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 0)
        out[s:s + 256] = hit.sum(1) % 2 == 1
    return out


if __name__ == "__main__":
    pts, faces, info = mesh()
    e = np.linalg.norm(pts[faces[:, [1, 2, 0]]] - pts[faces], axis=2)
    print(f"{len(pts)} vertices, {len(faces)} triangles, wall {info['rows']} x {info['cols']}, "
          f"edges {e.min() * 1000:.1f} to {e.max() * 1000:.1f} mm (mean {e.mean() * 1000:.1f})")
    for name, *_ in TABS:
        c, size, yaw = tab_box(name, pts, info)
        print(name, c.round(4), size.round(4), round(math.degrees(yaw), 1), len(tab_vertices(name, pts, info)),
              "vertices")
    test = np.array([[0, 0, 0.05], [0, 0, 0.2], [0.06, 0, 0.05], [0, 0.07, 0.1], [0, 0.09, 0.1]])
    print("inside:", inside(test, pts, faces, mouth_rows(info)))
