"""Printable parts of the real head, made from the same numbers as the simulated one (writes STL files in mm to
``print/`` next to this file). Run: ``python make_print_parts.py`` (NumPy and VTK; about 5 minutes).

All parts are ours (no third-party files). Head frame as in the task file and ``child_head.py``: origin at the nasal
root, x back from the face, y to the head's left (world +y), z up; mm. The head splits at the mouth's mid-height
(z -50) into two halves that sit on four 4 mm dowel pins; the stand under it holds two wells.

- head_upper.stl  the head above z -50: skin, upper lip, the roof of the mouth, a socket for the upper teeth, the upper
                  halves of the two cheek chambers and spring seats.
- head_lower.stl  the head below z -50, down to its flat base (z -170, on the stand): lower lip, chin, a socket for
                  the lower teeth, the tongue, the floor of the mouth sloping back into the throat, the two
                  cheek-pocket troughs (one each side, between the lower teeth and the inner cheek) with their drain
                  bores joining into one outlet over well A, and the throat's own drain bore to well B.
- teeth_upper.stl, teeth_lower.stl  the 10 upper and 10 lower primary teeth on their gum ridge, the ridge carried on
                  into a tenon (15 mm up, 3 mm down) that is glued into its socket (exactly the teeth and gums of the
                  simulated mouth).
- cheek_plate.stl the inner cheek: a 23.5 x 22 x 2 mm plate with a 1 mm flange behind it (print two; one is used
                  mirrored), sliding 4 mm outward against a soft spring.
- stand.stl       the 10 cm x 3 cm stand with well A (cheek pockets) and well B (throat), each 22 mm across, 25 mm deep.
- syringe_tray.stl the syringe's tray with its two notched rests (its own frame: origin at its middle on the table).
"""

from __future__ import annotations

import importlib.util
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import child_head as ch  # noqa: E402
import manikin_face as mf  # noqa: E402

ch.set_base(np.array([0.0, 0.0, 1.0]), -0.170)  # the head's flat base on its 3 cm stand (as in the task file)
MM = 0.001
SPLIT = -50.0

# numbers of the simulated mouth (copied here so that this script needs no simulator; checked against the task file
# when it can be imported)
TEETH = [(3.0, 2.8, 1.8, 2.8), (5.2, 7.6, 2.0, 2.3), (8.5, 11.5, 2.5, 2.5), (14.5, 14.5, 3.5, 3.0),
         (22.5, 16.0, 4.5, 3.5)]
UPPER_CROWN, LOWER_CROWN = (-43.0, -37.0), (-63.0, -57.0)
UPPER_GUM, LOWER_GUM = (-37.0, -31.0), (-69.0, -63.0)
GUM_PAD = 0.8
TONGUE_BOX = ((24.0, 0.0, -62.0), (14.0, 10.5, 5.0))
CHEEK_FACE, CHEEK_T, CHEEK_X, CHEEK_Z, CHEEK_TRAVEL_MM = 21.5, 3.0, (3.0, 26.5), (-61.0, -39.0), 4.0
TENON_U, TENON_L = 15.0, 3.0  # mm of each gum ridge hidden in its socket (the jaw under the lower one is thin)
FIT = 0.2  # clearance of sockets, slides and dowels
PLATE_T, FLANGE_T, FLANGE_W = 2.0, 1.0, 1.2  # cheek plate: face plate, flange behind it, flange overhang per side
CHAMBER_BACK = CHEEK_FACE + CHEEK_T + CHEEK_TRAVEL_MM  # 28.5: the flange stops here after 4 mm
SPRING_POCKET = (2.5, 6.7)  # radius, depth (mm) of each spring's seat behind the plate's middle
DOWELS = [(60.0, 30.0), (60.0, -30.0), (88.0, 12.0), (88.0, -12.0)]  # (x, y) at the split, 4 mm pins
DOWEL_R, DOWEL_DEPTH = 2.0, 9.0
BORE_R = 2.0  # drain bores 4 mm across
SUMP = (31.0, 23.0, -71.0)  # the low back end of each cheek-pocket trough, behind the last molar, under the floor
POCKET_PATH = [SUMP, (38.0, 22.0, -82.0), (58.0, 16.0, -100.0), (72.0, 0.0, -140.0), (80.0, 0.0, -175.0)]
THROAT_PATH = [(50.0, 0.0, -78.0), (52.0, 0.0, -175.0)]
STAND_C, STAND_R, STAND_H = (66.0, 0.0), 50.0, 30.0
WELLS = {"A": (80.0, 0.0), "B": (52.0, 0.0)}  # head-frame (x, y) of well A (pockets) and well B (throat)
WELL_R, WELL_DEPTH = 11.0, 25.0


def check_against_task() -> None:
    try:
        spec = importlib.util.spec_from_file_location("osyr_task", HERE / "give_a_child_liquid_medicine_with_an_oral_syringe.py")
        t = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(t)
    except Exception as e:  # no simulator here: the numbers above are used as they are
        print(f"(task file not imported: {type(e).__name__}; using the numbers in this script)")
        return
    assert t.TEETH == TEETH and t.UPPER_CROWN == UPPER_CROWN and t.LOWER_CROWN == LOWER_CROWN
    assert t.UPPER_GUM == UPPER_GUM and t.LOWER_GUM == LOWER_GUM and t.GUM_PAD == GUM_PAD
    assert t.TONGUE_BOX == TONGUE_BOX and t.CHEEK_FACE == CHEEK_FACE and t.CHEEK_T == CHEEK_T
    assert t.CHEEK_X == CHEEK_X and t.CHEEK_Z == CHEEK_Z and abs(t.CHEEK_TRAVEL * 1000 - CHEEK_TRAVEL_MM) < 1e-9
    mine = (SYR_TIP0_X, TRAY_X, REST_H, BARREL_FRONT, BARREL_BACK, BARREL_R, FLANGE_T_M, FLANGE_W_M)
    theirs = (t.SYR_TIP0[0], t.TRAY_XY[0], t.REST_H, t.BARREL_FRONT, t.BARREL_BACK, t.BARREL_R, t.FLANGE_T, t.FLANGE_W)
    assert np.allclose(mine, theirs, rtol=0.0, atol=1e-9)
    print("numbers match the task file")


# -- signed distances (mm; negative inside) ---------------------------------------------------------------------------
def box(p, lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    c, h = (lo + hi) / 2, (hi - lo) / 2
    q = np.abs(p - c) - h
    return np.linalg.norm(np.maximum(q, 0.0), axis=-1) + np.minimum(q.max(axis=-1), 0.0)


def seg(p, a, b, r):
    a, b = np.asarray(a, float), np.asarray(b, float)
    ab = b - a
    t = np.clip(((p - a) @ ab) / (ab @ ab), 0.0, 1.0)
    return np.linalg.norm(p - (a + t[..., None] * ab), axis=-1) - r


def poly(p, pts, r):
    return np.min([seg(p, pts[i], pts[i + 1], r) for i in range(len(pts) - 1)], axis=0)


def cyl_y(p, c, y0, y1, r):
    """A cylinder along y from y0 to y1, its axis through (c[0], ., c[1])."""
    d = np.hypot(p[..., 0] - c[0], p[..., 2] - c[1]) - r
    e = np.maximum(y0 - p[..., 1], p[..., 1] - y1)
    return np.minimum(np.maximum(d, e), 0.0) + np.hypot(np.maximum(d, 0.0), np.maximum(e, 0.0))


def cyl_z(p, c, z0, z1, r):
    d = np.hypot(p[..., 0] - c[0], p[..., 1] - c[1]) - r
    e = np.maximum(z0 - p[..., 2], p[..., 2] - z1)
    return np.minimum(np.maximum(d, e), 0.0) + np.hypot(np.maximum(d, 0.0), np.maximum(e, 0.0))


def mirror_union(f, p):
    q = p.copy()
    q[..., 1] = np.abs(q[..., 1])
    return f(q)


def gum_boxes(p, z0, z1, pad=GUM_PAD):
    out = []
    for x, y, hx, hy in TEETH:
        for s in (1.0, -1.0):
            out.append(box(p, (x - hx - pad, s * y - hy - pad, z0), (x + hx + pad, s * y + hy + pad, z1)))
    return np.min(out, axis=0)


def crowns(p, crown):
    out = []
    for x, y, hx, hy in TEETH:
        for s in (1.0, -1.0):
            out.append(box(p, (x - hx * 0.92, s * y - hy * 0.92, crown[0]), (x + hx * 0.92, s * y + hy * 0.92, crown[1])))
    return np.min(out, axis=0)


def teeth_upper(p):
    return np.minimum(crowns(p, UPPER_CROWN), gum_boxes(p, UPPER_GUM[0], UPPER_GUM[1] + TENON_U))


def teeth_lower(p):
    return np.minimum(crowns(p, LOWER_CROWN), gum_boxes(p, LOWER_GUM[0] - TENON_L, LOWER_GUM[1]))


def head_air(p):
    """The mouth's air space for the print: the lips' opening, the mouth and the throat (as in the simulation), the
    opening for each inner cheek, the chamber its flange slides in, and its spring's seat."""
    m = p * MM
    sm = mf._smin
    d = sm(ch._e(m, ch.OPEN_C, ch.OPEN_R), ch._e(m, ch.CAVITY_C, ch.CAVITY_R), 5.0 * MM)
    d = sm(d, ch._e(m, ch.PHARYNX_C, ch.PHARYNX_R), 6.0 * MM) / MM

    def side(q):
        o = box(q, (CHEEK_X[0] - FIT, 17.0, CHEEK_Z[0] - FIT), (CHEEK_X[1] + FIT, CHEEK_FACE + PLATE_T, CHEEK_Z[1] + FIT))
        f = box(q, (CHEEK_X[0] - FLANGE_W - 2 * FIT, CHEEK_FACE + PLATE_T, CHEEK_Z[0] - FLANGE_W - 2 * FIT),
                (CHEEK_X[1] + FLANGE_W + 2 * FIT, CHAMBER_BACK, CHEEK_Z[1] + FLANGE_W + 2 * FIT))
        cx, cz = sum(CHEEK_X) / 2, sum(CHEEK_Z) / 2
        s = cyl_y(q, (cx, cz), CHAMBER_BACK - 1.0, CHAMBER_BACK + SPRING_POCKET[1], SPRING_POCKET[0])
        return np.minimum(np.minimum(o, f), s)

    return np.minimum(d, mirror_union(side, p))


def head_solid(p):
    m = p * MM
    d = mf._smax(ch.outer_sdf(m), -head_air(p) * MM, 1.5 * MM) / MM
    return np.maximum(d, -170.0 - p[..., 2])


def dowels(p):
    return np.min([cyl_z(p, c, SPLIT - DOWEL_DEPTH, SPLIT + DOWEL_DEPTH, DOWEL_R + FIT / 2) for c in DOWELS], axis=0)


def head_upper(p):
    d = np.maximum(head_solid(p), SPLIT - p[..., 2])
    d = np.maximum(d, -gum_boxes(p, UPPER_GUM[0] - 1.0, UPPER_GUM[1] + TENON_U + FIT, GUM_PAD + FIT))
    return np.maximum(d, -dowels(p))


def trough(q):
    """One cheek-pocket trough (|y| side): between the lower gum ridge's outer face and the cheek's opening, from the
    front teeth back past the last molar, its floor falling 2 mm toward the sump at its back."""
    out = []
    for x, y, hx, hy in TEETH:
        y_in = y + hy + GUM_PAD
        out.append(box(q, (x - hx - GUM_PAD, y_in, -72.0), (x + hx + GUM_PAD + 0.6, CHEEK_FACE + 0.8, -60.0)))
    out.append(box(q, (10.0, 21.0, -72.0), (33.0, 26.0, -64.5)))  # wider below the cheek's chamber, on to the sump
    d = np.min(out, axis=0)
    floor = -(q[..., 2] - (-68.0 - 3.0 * (q[..., 0] - 2.0) / 29.0))  # > 0 below the floor (3 mm fall to the sump)
    d = np.maximum(d, floor)
    return np.maximum(d, ch.outer_sdf(q * MM) / MM + 1.5)  # at least 1.5 mm of skin under it


def head_lower(p):
    d = np.maximum(head_solid(p), p[..., 2] - SPLIT)
    c, h = TONGUE_BOX
    tongue = box(p, np.array(c) - h - np.array([0, 0, 15.0]), np.array(c) + h)
    top = -60.0 - 0.14 * (p[..., 0] - 4.0)  # the floor of the mouth inside the teeth, sloping back to the throat
    fill = np.maximum(box(p, (4.0, -15.5, -85.0), (40.0, 15.5, -55.0)), p[..., 2] - top)
    d = np.minimum(d, np.maximum(np.minimum(tongue, fill), ch.outer_sdf(p * MM) / MM))
    d = np.maximum(d, -170.0 - p[..., 2])
    d = np.maximum(d, -gum_boxes(p, LOWER_GUM[0] - TENON_L - FIT, LOWER_GUM[1] + 1.0, GUM_PAD + FIT))
    d = np.maximum(d, -mirror_union(trough, p))
    bores = np.minimum(mirror_union(lambda q: poly(q, POCKET_PATH, BORE_R), p), poly(p, THROAT_PATH, BORE_R))
    d = np.maximum(d, -bores)
    return np.maximum(d, -dowels(p))


def cheek_plate(p):
    """Made for the left (+y) side, its face at y 21.5; for the right side mirror it."""
    body = box(p, (CHEEK_X[0], CHEEK_FACE, CHEEK_Z[0]), (CHEEK_X[1], CHEEK_FACE + PLATE_T, CHEEK_Z[1]))
    fl = box(p, (CHEEK_X[0] - FLANGE_W, CHEEK_FACE + PLATE_T - 0.01, CHEEK_Z[0] - FLANGE_W),
             (CHEEK_X[1] + FLANGE_W, CHEEK_FACE + CHEEK_T, CHEEK_Z[1] + FLANGE_W))
    boss = cyl_y(p, (sum(CHEEK_X) / 2, sum(CHEEK_Z) / 2), CHEEK_FACE + CHEEK_T - 0.01, CHEEK_FACE + CHEEK_T + 2.0, 1.6)
    return np.minimum(np.minimum(body, fl), boss)


def stand(p):
    """Stand frame = head frame; its top at the head's base (z -170), the table at z -200."""
    d = cyl_z(p, STAND_C, -200.0, -170.0, STAND_R)
    for c in WELLS.values():
        d = np.maximum(d, -cyl_z(p, c, -170.0 - WELL_DEPTH, -165.0, WELL_R))
    pins = [(STAND_C[0], 28.0), (STAND_C[0], -28.0)]
    for c in pins:
        d = np.maximum(d, -cyl_z(p, c, -170.0 - DOWEL_DEPTH, -165.0, DOWEL_R + FIT / 2))
    return d


def head_lower_with_pins(p):
    d = head_lower(p)
    for c in [(STAND_C[0], 28.0), (STAND_C[0], -28.0)]:
        d = np.maximum(d, -cyl_z(p, c, -175.0, -170.0 + DOWEL_DEPTH, DOWEL_R + FIT / 2))
    return d


# -- exact meshes of the box-built and round parts (no decimation needed) ---------------------------------------------
def teeth_boxes(upper: bool) -> list:
    """(lo, hi) boxes of one arch: crowns (sunk 0.5 mm into the gum so that they join it) and the gum ridge with its
    tenon, as in the simulated mouth."""
    crown, gum = (UPPER_CROWN, UPPER_GUM) if upper else (LOWER_CROWN, LOWER_GUM)
    cz = (crown[0], crown[1] + 0.5) if upper else (crown[0] - 0.5, crown[1])
    gz = (gum[0], gum[1] + TENON_U) if upper else (gum[0] - TENON_L, gum[1])
    out = []
    for x, y, hx, hy in TEETH:
        for s in (1.0, -1.0):
            out.append(((x - hx * 0.92, s * y - hy * 0.92, cz[0]), (x + hx * 0.92, s * y + hy * 0.92, cz[1])))
            out.append(((x - hx - GUM_PAD, s * y - hy - GUM_PAD, gz[0]), (x + hx + GUM_PAD, s * y + hy + GUM_PAD, gz[1])))
    return out


def plate_boxes() -> list:
    """The left (+y) inner cheek: face plate, flange, and a 2.4 mm square boss that centres the spring."""
    cx, cz = sum(CHEEK_X) / 2, sum(CHEEK_Z) / 2
    return [((CHEEK_X[0], CHEEK_FACE, CHEEK_Z[0]), (CHEEK_X[1], CHEEK_FACE + PLATE_T, CHEEK_Z[1])),
            ((CHEEK_X[0] - FLANGE_W, CHEEK_FACE + PLATE_T, CHEEK_Z[0] - FLANGE_W),
             (CHEEK_X[1] + FLANGE_W, CHEEK_FACE + CHEEK_T, CHEEK_Z[1] + FLANGE_W)),
            ((cx - 1.2, CHEEK_FACE + CHEEK_T, cz - 1.2), (cx + 1.2, CHEEK_FACE + CHEEK_T + 2.0, cz + 1.2))]


# the syringe's tray (m, as in the task file): where the syringe lies and the barrel's sizes
SYR_TIP0_X, TRAY_X, BARREL_FRONT, BARREL_BACK = 0.255, 0.200, -0.013, -0.071
FLANGE_T_M, FLANGE_W_M, BARREL_R, REST_H = 0.0025, 0.030, 0.007, 0.022


def tray_boxes() -> list:
    """The tray (mm, its own frame: origin at its middle on the table, x toward the head): an 110 x 44 x 4 base and
    two notched rests, one under the barrel's front end (22 mm high, notch 14 mm wide) and one under the finger flange
    (23 mm high, notch 30.6 mm wide), exactly as the task file builds it."""
    out = [((-55.0, -22.0, 0.0), (55.0, 22.0, 4.0))]
    for bx, ln, h, gap in ((BARREL_FRONT - 0.004, 0.008, REST_H, BARREL_R),
                           (BARREL_BACK - FLANGE_T_M / 2 - 0.0001, 0.0026, REST_H + 0.001, FLANGE_W_M / 2 + 0.0003)):
        x = (SYR_TIP0_X + bx - TRAY_X) * 1000
        ln, h, gap = ln * 1000, h * 1000, gap * 1000
        out.append(((x - ln / 2, -20.0, 4.0), (x + ln / 2, 20.0, 4.0 + h)))
        for s_ in (1.0, -1.0):
            yc = s_ * (gap + 2.0)
            out.append(((x - ln / 2, yc - 2.0, 4.0 + h - 0.01), (x + ln / 2, yc + 2.0, 4.0 + h + 4.0)))
    return out


def box_union_mesh(boxes: list):
    """The exact surface of a union of axis-aligned boxes: cells of the grid of all their faces, inside or out, and the
    faces between an inside and an outside cell (shared grid nodes, so the surface is closed)."""
    g = [np.unique(np.concatenate([[b[0][k], b[1][k]] for b in boxes])) for k in range(3)]
    c = [(a[1:] + a[:-1]) / 2 for a in g]
    C = np.stack(np.meshgrid(*c, indexing="ij"), -1)
    inside = np.zeros(C.shape[:3], bool)
    for lo, hi in boxes:
        inside |= np.all((C > np.asarray(lo)) & (C < np.asarray(hi)), axis=-1)
    pad = np.pad(inside, 1)
    n = [len(a) for a in g]
    vid = lambda i, j, k: (i * n[1] + j) * n[2] + k  # noqa: E731
    tris = []
    for ax in range(3):
        d = np.diff(pad.astype(np.int8), axis=ax)  # +1: out -> in along +ax, -1: in -> out
        sl = [slice(1, -1)] * 3
        sl[ax] = slice(None)
        d = d[tuple(sl)]
        for sign in (1, -1):
            for idx in np.argwhere(d == sign):
                i = list(idx)  # the face lies on grid plane i[ax] along ax
                u, v = [a for a in range(3) if a != ax]
                corners = []
                for du, dv in ((0, 0), (1, 0), (1, 1), (0, 1)):
                    q = list(i)
                    q[u] += du
                    q[v] += dv
                    corners.append(vid(*q))
                a, b, cc, dd = corners
                # the outward normal is -ax where the cell beyond is inside (sign +1), +ax otherwise
                flip = (sign == 1) ^ ((v - u) % 3 == 1)
                tris += [(a, cc, b), (a, dd, cc)] if flip else [(a, b, cc), (a, cc, dd)]
    P = np.stack(np.meshgrid(*g, indexing="ij"), -1).reshape(-1, 3)
    F = np.asarray(tris, np.int64)
    used, F = np.unique(F, return_inverse=True)
    return P[used], F.reshape(-1, 3)


def stand_mesh(n_out: int = 120, n_well: int = 60, n_pin: int = 20):
    """The stand (head frame, mm): a disc 100 mm across and 30 mm high with two wells 22 mm across and 25 mm deep and
    two 4.2 mm pin holes 9 mm deep, as one closed surface."""
    import trimesh
    from scipy.spatial import Delaunay

    def tri2d(rings):
        """Triangles of the region inside rings[0] and outside the others (circles: Delaunay of their points keeps
        every edge between neighbouring points)."""
        pts = np.concatenate(rings)
        t = Delaunay(pts).simplices
        cen = pts[t].mean(axis=1)
        keep = np.ones(len(t), bool)
        for k, r in enumerate(rings):
            c, rad = r.mean(axis=0), np.linalg.norm(r[0] - r.mean(axis=0))
            inside = np.linalg.norm(cen - c, axis=1) < rad
            keep &= inside if k == 0 else ~inside
        t = t[keep]
        a2 = np.cross(pts[t[:, 1]] - pts[t[:, 0]], pts[t[:, 2]] - pts[t[:, 0]])
        t[a2 < 0] = t[a2 < 0][:, ::-1]  # counter-clockwise: facing +z
        return pts, t

    z0, z1 = -200.0, -170.0
    holes = [(c, WELL_R, z1 - WELL_DEPTH, n_well) for c in WELLS.values()]
    holes += [((STAND_C[0], s * 28.0), DOWEL_R + FIT / 2, z1 - DOWEL_DEPTH, n_pin) for s in (1.0, -1.0)]

    def ring(c, r, n):
        a = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
        return np.stack([c[0] + r * np.cos(a), c[1] + r * np.sin(a)], 1)

    outer = ring(STAND_C, STAND_R, n_out)
    v2, f2 = tri2d([outer] + [ring(c, r, n) for c, r, _, n in holes])
    P, F = [], []

    def add(pts, faces):
        F.append(np.asarray(faces) + sum(len(p) for p in P))
        P.append(np.asarray(pts, float))

    add(np.c_[v2, np.full(len(v2), z1)], f2)  # top face, up
    vb, fb = tri2d([outer])
    add(np.c_[vb, np.full(len(vb), z0)], np.asarray(fb)[:, ::-1])  # bottom face, down

    def wall(r2, za, zb, outward):
        m = len(r2)
        pts = np.r_[np.c_[r2, np.full(m, za)], np.c_[r2, np.full(m, zb)]]
        f = []
        for i in range(m):
            j = (i + 1) % m
            f += [(i, j, m + j), (i, m + j, m + i)] if outward else [(i, m + j, j), (i, m + i, m + j)]
        add(pts, f)

    wall(outer, z0, z1, True)
    for c, r, zf, n in holes:
        rr = ring(c, r, n)
        wall(rr, zf, z1, False)
        vf, ff = tri2d([rr])
        add(np.c_[vf, np.full(len(vf), zf)], ff)  # the hole's floor, facing up
    t = trimesh.Trimesh(np.concatenate(P), np.concatenate(F), process=True)
    t.merge_vertices()
    return np.asarray(t.vertices), np.asarray(t.faces)


def write_exact(P, F, path) -> None:
    import trimesh

    t = trimesh.Trimesh(P, F, process=True)
    t.merge_vertices()
    if t.volume < 0:
        t.invert()
    t.export(str(path))
    b = t.bounds[1] - t.bounds[0]
    print(f"{path.name}: {len(t.faces)} triangles, {b[0]:.1f} x {b[1]:.1f} x {b[2]:.1f} mm, closed {t.is_watertight}, "
          f"{t.volume / 1000:.1f} cm3, {path.stat().st_size / 1e6:.2f} MB")


# -- meshing ----------------------------------------------------------------------------------------------------------
def mesh(f, lo, hi, step, target, path):
    import vtk
    from vtk.util import numpy_support as ns

    lo, hi = np.asarray(lo, float) - 2 * step, np.asarray(hi, float) + 2 * step
    n = np.ceil((hi - lo) / step).astype(int) + 1
    xs, ys, zs = (lo[k] + step * np.arange(n[k]) for k in range(3))
    vals = np.empty((n[2], n[1], n[0]), np.float32)
    Y, X = np.meshgrid(ys, xs, indexing="ij")
    for k, z in enumerate(zs):
        p = np.stack([X, Y, np.full_like(X, z)], -1)
        vals[k] = f(p)
    vals[0], vals[-1] = 1.0, 1.0
    vals[:, 0], vals[:, -1], vals[:, :, 0], vals[:, :, -1] = 1.0, 1.0, 1.0, 1.0
    img = vtk.vtkImageData()
    img.SetDimensions(int(n[0]), int(n[1]), int(n[2]))
    img.SetSpacing(step, step, step)
    img.SetOrigin(*lo)
    img.GetPointData().SetScalars(ns.numpy_to_vtk(vals.ravel(), deep=True))
    fe = vtk.vtkFlyingEdges3D()
    fe.SetInputData(img)
    fe.SetValue(0, 0.0)
    fe.ComputeNormalsOff()
    clean = vtk.vtkCleanPolyData()
    clean.SetInputConnection(fe.GetOutputPort())
    clean.Update()
    o = clean.GetOutput()
    P = ns.vtk_to_numpy(o.GetPoints().GetData()).astype(np.float64)
    F = ns.vtk_to_numpy(o.GetPolys().GetData()).reshape(-1, 4)[:, 1:].astype(np.int64)
    total = len(F)
    write_mesh(P, F, total, target, path)


def write_mesh(P, F, total, target, path) -> None:
    """Simplify a closed triangle surface to about ``target`` triangles (quadric error, fast_simplification), drop
    zero-area triangles, face it outward and write a binary STL."""
    import fast_simplification
    import trimesh

    if total > target:
        P, F = fast_simplification.simplify(P.astype(np.float32), F, target_reduction=1.0 - target / total)
    t = trimesh.Trimesh(np.asarray(P, float), np.asarray(F), process=True)
    t.update_faces(t.nondegenerate_faces())
    t.remove_unreferenced_vertices()
    t.merge_vertices()
    trimesh.repair.fix_normals(t)
    if t.volume < 0:
        t.invert()
    t.export(str(path))
    b = t.bounds[1] - t.bounds[0]
    print(f"{path.name}: {len(t.faces)} triangles (from {total}), {b[0]:.1f} x {b[1]:.1f} x {b[2]:.1f} mm, "
          f"closed {t.is_watertight}, {t.volume / 1000:.1f} cm3, {path.stat().st_size / 1e6:.2f} MB")


def bore_margins() -> None:
    """The drain bores' wall thickness to the outside, to the mouth and throat, and to the lower teeth's socket (mm),
    sampled along each path past its first 4 mm; and how far inside the head's base each well's rim lies."""
    sock = lambda q: gum_boxes(q, LOWER_GUM[0] - TENON_L - FIT, LOWER_GUM[1] + 1.0, GUM_PAD + FIT)  # noqa: E731
    for name, pts in (("cheek-pocket bore", POCKET_PATH), ("throat bore", THROAT_PATH)):
        a, b, c = [], [], []
        for i in range(len(pts) - 1):
            for t in np.linspace(0.0, 1.0, 60):
                q = np.asarray(pts[i], float) * (1 - t) + np.asarray(pts[i + 1], float) * t
                if np.linalg.norm(q - pts[0]) < 4.0 or q[2] < -168.0:
                    continue
                a.append(-float(ch.outer_sdf(q[None] * MM)[0]) / MM - BORE_R)
                if pts is POCKET_PATH:  # (the throat's bore starts in the throat)
                    b.append(float(mirror_union(head_air, q[None])[0]) - BORE_R)
                c.append(float(mirror_union(sock, q[None])[0]) - BORE_R)
        print(f"{name}: thinnest wall to the skin {min(a):.1f} mm, to the mouth {min(b, default=0.0):.1f} mm, "
              f"to the teeth's socket {min(c):.1f} mm")
    for k, (x, y) in WELLS.items():
        rim = [(x + WELL_R * np.cos(a), y + WELL_R * np.sin(a), -169.0) for a in np.linspace(0, 2 * np.pi, 24)]
        print(f"well {k}: its rim at least {min(-float(ch.outer_sdf(np.array(r)[None] * MM)[0]) / MM for r in rim):.1f}"
              " mm inside the head's base")


def main() -> None:
    check_against_task()
    bore_margins()
    out = HERE / "print"
    out.mkdir(exist_ok=True)
    write_exact(*box_union_mesh(teeth_boxes(True)), out / "teeth_upper.stl")
    write_exact(*box_union_mesh(teeth_boxes(False)), out / "teeth_lower.stl")
    write_exact(*box_union_mesh(plate_boxes()), out / "cheek_plate.stl")
    write_exact(*stand_mesh(), out / "stand.stl")
    write_exact(*box_union_mesh(tray_boxes()), out / "syringe_tray.stl")
    if "--small" in sys.argv:
        return
    mesh(head_upper, (-30.0, -75.0, SPLIT), (175.0, 75.0, 110.0), 0.6, 32000, out / "head_upper.stl")
    mesh(head_lower_with_pins, (-30.0, -75.0, -175.0), (175.0, 75.0, SPLIT), 0.6, 32000, out / "head_lower.stl")


if __name__ == "__main__":
    main()
