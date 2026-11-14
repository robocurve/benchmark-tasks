"""Scan a cast part's whole surface with a handheld 3D scanner: start the scanner with its scan button, then cover the part's sides and top
as it turns on a manual turntable, the scanner held inside its working window and moved from a low view of the part's sides up to a steep
view of its top.

The turntable's platter is a hinge on a light bearing; the part, a cast-iron plummer block housing, stands free on it and rides round by
friction. The scanner is a Shining 3D EinScan H2 (its maker's size and mass), a free body standing on its head on the table, handle up. Its button 4
(Preview / Scan / Pause, on the handle) starts scanning; until it is pressed nothing is counted. Coverage is computed by code, as the scanning
software's model grows: about 4,800 points on the part's surface (its underside, bore and bolt slots left out), each counted once, at a
step while the scanner is scanning and tracking, if it lies inside the scanner's view (200 to 700 mm, about 48 x 50 degrees), faces the
scanner within 45 degrees, and is not hidden behind the part itself. A tablet beside the turntable runs a simple scanning app, as the
scanner's own software shows a scan: two live views showing the surface captured so far, the distance, tracking, and the coverage so far.

Debug:  python demokit.py task.py OUT --no-record --stills 40 --mode sweep
Final:  python demokit.py task.py OUT --mode sweep
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demokit as d  # noqa: E402
import gatekit as gk  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
INSTRUCTION = "Start scanning with button 4 on the back of the scanner's handle, scan the part on the turntable until the scan covers 95 % of its outer surface, its top included, then move the scanner and your hands at least 3 cm away from the part."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own 1 s hold (HOLD_STEPS): score() and success() agree
SCORE_OVER = "end"
HOLD_STEPS = 20
TOP = d.TABLE_TOP
DT = d.DT

LOOKS = {
    "base": m.Look(m.rgb(0.16, 0.16, 0.17), roughness=0.5),
    "rubber": m.Look(m.rgb(0.07, 0.07, 0.07), roughness=0.95),
    "tick": m.Look(m.rgb(0.85, 0.85, 0.85), roughness=0.6),
    "cast": m.Look(m.rgb(0.42, 0.44, 0.46), roughness=0.8),  # painted cast iron
    "hole": m.Look(m.rgb(0.03, 0.03, 0.03), roughness=0.9),
    "scanner": m.Look(m.rgb(0.12, 0.12, 0.13), roughness=0.35),
    "lens": m.Look(m.rgb(0.04, 0.06, 0.12), roughness=0.05),
    "cradle": m.Look(m.rgb(0.80, 0.80, 0.82), roughness=0.5),
    "tablet": m.Look(m.rgb(0.10, 0.10, 0.11), roughness=0.3),
    "screen": m.Look(m.rgb(0.02, 0.02, 0.03), roughness=0.1),
    "red_dim": m.Look(m.rgb(0.25, 0.06, 0.05), roughness=0.4),
    "green_dim": m.Look(m.rgb(0.05, 0.22, 0.08), roughness=0.4),
    "grey_dim": m.Look(m.rgb(0.18, 0.18, 0.19), roughness=0.4),
    "button": m.Look(m.rgb(1.0, 0.42, 0.0), roughness=0.35, emissive=m.rgb(0.35, 0.12, 0.0)),  # button 4 in bright orange, so it reads in the top and wrist cameras
    "white_lit": m.Look(m.rgb(1.0, 1.0, 1.0), emissive=m.rgb(0.9, 0.9, 0.9)),
    "green_lit": m.Look(m.rgb(0.2, 1.0, 0.3), emissive=m.rgb(0.15, 0.9, 0.25)),
    "blue_lit": m.Look(m.rgb(0.25, 0.55, 1.0), emissive=m.rgb(0.2, 0.45, 0.95)),
    "red_lit": m.Look(m.rgb(1.0, 0.15, 0.1), emissive=m.rgb(0.9, 0.12, 0.08)),
    "appbar": m.Look(m.rgb(0.10, 0.22, 0.40), roughness=0.3, emissive=m.rgb(0.05, 0.12, 0.24)),  # the app's title bar
    "scanned": m.Look(m.rgb(0.25, 0.85, 0.35), roughness=0.5, emissive=m.rgb(0.15, 0.6, 0.22)),  # scanned surface
}

# -- the turntable: a 300 mm manual turntable on a ball bearing ----------------------------------------------------------------------
TURNTABLE_XY = (0.42, 0.0)
PLATTER_R, PLATTER_TOP, PLATTER_T = 0.150, 0.030, 0.016
STAND_R, STAND_H = 0.12, 0.014
# -- the part: a cast-iron plummer block housing, SKF SY 505 M (the housing of an SY 25 TF unit; 0.55 kg, 52 mm seat, 36.5 mm centre height), in its own
# frame: origin at the middle of its underside, the bore's axis along y
L, W, H = 0.127, 0.038, 0.016  # the base: length (x), width (y), thickness
SHAFT_H, HOUSING_R, BORE_R = 0.0365, 0.034, 0.026  # the bore's axis above the underside; the housing's outer radius; the bore (the 52 mm seat)
PART_KG = 0.55
SLOTS = [(sx * 0.049, 0.020, 0.013) for sx in (-1, 1)]  # the two bolt slots in the base: centre x, length, width
PART_Z = TOP + PLATTER_TOP
# -- the scanner: Shining 3D EinScan H2 (maker's specs: 108 x 110 x 237 mm, 731.1 g, depth of field 200 to 700 mm, field of view up to
# 420 x 440 mm at its best working distance of 470 mm; its quick start guide: button 4 on the handle, Preview / Scan / Pause) -------------
# In its own frame: the origin at the head's middle, the lens face at +x, its width along y, up along z; the handle under the head's rear
HEAD = (0.085, 0.108, 0.090)
HANDLE, HANDLE_AT = (0.045, 0.040, 0.147), (-0.045, 0.0, -0.045 - 0.147 / 2)
SCAN = HEAD  # the head, for its lens
LENS_X = HEAD[0] / 2
SCAN_KG = 0.731
GRIP_N = float(os.environ.get("SCAN_GRIP_N", "30"))  # the hand's squeeze on the handle (the YAM squeezes up to about 50 N)
HOLD = (-0.045, 0.0, -0.097)  # the hand's TCP on the handle, 37 mm under the button (at 25 mm a finger pressed it as the hand closed), the fingers across the handle's 45 mm depth
BUTTON, BUTTON_R = (HANDLE_AT[0] - HANDLE[0] / 2, 0.0, -0.060), 0.006  # button 4, on the handle's rear (the thumb's side, as the guide draws it), 15 mm under the head
NEAR, FAR = 0.200, 0.700
HALF_H, HALF_V = math.atan(0.210 / 0.470), math.atan(0.220 / 0.470)
FACING = math.cos(math.radians(45.0))  # beyond 45 degrees from square on, a structured-light scanner's capture of a surface is poor
TRACK_V, TRACK_W, TRACK_IN_VIEW = 0.25, math.radians(60.0), 0.25
RELOCATE_PTS, RELOCATE_SHARE = 50, 0.3  # once tracking is lost, it comes back only on a view of at least 50 points, 30 % of them already scanned
DROP_V = 0.8  # the scanner falling faster than this (m/s), with no hand holding it, has been dropped: a free fall of 3 cm
PRESS_AT = (-0.004, 0.001)  # a press: a fingertip point from 4 mm past button 4's face to 1 mm off it. Pressed, the fingertip stops at the face (-0.2 to +0.6 mm in a logged run, the scanner pushed back as it held); hovering 2 mm short, it reads 1.6 to 2.0 mm, and does not press
HANDLE_GRIP = gk.Grip(half=HANDLE[0] / 2, across="x", centre=HANDLE_AT, lo=(-1.0, -HANDLE[1] / 2, HANDLE_AT[2] - HANDLE[2] / 2), hi=(1.0, HANDLE[1] / 2, HANDLE_AT[2] + HANDLE[2] / 2))
SCAN_REST = (0.20, -0.12, TOP + HEAD[2] / 2)  # the head's middle, the scanner standing on its head (stable: its weight is over the head's 85 x 108 mm top), its handle up, its lens toward -y, away from the part (pressed where it stands, it scans nothing), button 4 facing +y
SCAN_REST_QUAT = (0.707107, -0.707107, 0.000000, 0.000000)  # xyzw: its x (the lens) along -y, its y along -x, its z (up, in its own frame) down. The hand takes the handle level from -x and lifts it with its weight under the hand: lifted lying on its side, the head's weight would twist the handle about the jaws' axis
COVER_FULL, COVER_HALF, COVER_LOW = 0.95, 0.70, 0.40  # one low turn reaches about 76 %, and a sweep up over the top about 91 %: 95 % needs a turn at a steep view as well
COVER_TOP = 0.50  # the 0.5 band also needs half the top: a scanner held still at a low view, as on a tripod, gets 0.25
CLEAR = 0.03  # the hands' and the scanner's distance from the part at the end
STRIKE = 0.002
KNOCK_XY, KNOCK_TILT = 0.02, 15.0


def _turntable() -> m.Part:
    stand = [("stand", m.cylinder(STAND_R, STAND_H, (0, 0, STAND_H / 2)), "base")]
    zc = PLATTER_TOP - PLATTER_T / 2
    parts = [("platter", m.cylinder(PLATTER_R, PLATTER_T - 0.002, (0, 0, zc - 0.001), segments=64), "base"),
             ("mat", m.cylinder(PLATTER_R - 0.002, 0.002, (0, 0, PLATTER_TOP - 0.001), segments=64), "rubber")]
    for k in range(24):  # ticks round the rim every 15 degrees, longer every 90
        n = 0.020 if k % 6 == 0 else 0.010
        parts.append((f"tick_{k}", m.box((n, 0.002, 0.0006), (PLATTER_R - 0.004 - n / 2, 0, PLATTER_TOP + 0.0003)).turned(15.0 * k, "z"), "tick"))
    cols = [m.col_cylinder("c_platter", PLATTER_R, PLATTER_T, (0, 0, zc))]
    platter = m.hinge("platter", "Z", (0, 0, 0), -math.inf, math.inf, parts, cols, mass=0.6, damping=0.02, friction=0.002,
                      inertia=(0.0034, 0.0034, 0.0068))
    return m.mechanism("turntable", (*TURNTABLE_XY, TOP), [platter], LOOKS, stand, [m.col_cylinder("c_stand", STAND_R, STAND_H, (0, 0, STAND_H / 2))],
                       friction=(0.9, 0.8))


def _housing() -> m.Mesh:
    """The housing: a block up to the shaft's axis and a half round over it, as one solid along y."""
    a = np.linspace(0.0, math.pi, 25)
    xz = np.vstack([[[HOUSING_R, H - 0.001]], np.c_[HOUSING_R * np.cos(a), SHAFT_H + HOUSING_R * np.sin(a)], [[-HOUSING_R, H - 0.001]]])
    outline = np.c_[xz[:, 0], -xz[:, 1]]  # the prism's y becomes -z once turned about x
    area = 0.5 * np.sum(outline[:, 0] * np.roll(outline[:, 1], -1) - np.roll(outline[:, 0], -1) * outline[:, 1])
    if area < 0:
        outline = outline[::-1]
    return m.prism(outline, -W / 2, W / 2).turned(-90.0, "x")


def _part() -> m.Part:
    parts = [("base", m.box((L, W, H), (0, 0, H / 2)), "cast"), ("housing", _housing(), "cast")]
    for s, sy in (("a", 1), ("b", -1)):  # the bore, seen through each end
        parts.append((f"bore_{s}", m.cylinder(BORE_R, 0.0010, (0, sy * (W / 2 + 0.0006), SHAFT_H), axis="y"), "hole"))
    for k, (sx, sl, sw) in enumerate(SLOTS):  # the bolt slots
        parts.append((f"slot_{k}", m.box((sl, sw, 0.0008), (sx, 0, H + 0.0003)), "hole"))
    cols = [m.col_box("c_base", (L, W, H), (0, 0, H / 2)), m.col_box("c_pedestal", (2 * HOUSING_R, W, SHAFT_H - H), (0, 0, (SHAFT_H + H) / 2)),
            m.col_hull("c_housing", m.cylinder(HOUSING_R, W, (0, 0, SHAFT_H), axis="y", segments=24))]
    return m.free("part", (*TURNTABLE_XY, PART_Z + 0.0005), parts, cols, LOOKS, mass=PART_KG, friction=(0.9, 0.8), torsional_patch=0.02)


def _scanner() -> m.Part:
    x0 = LENS_X + 0.0004
    parts = [("head", m.box(HEAD), "scanner"), ("handle", m.box(HANDLE, HANDLE_AT), "scanner"),
             ("window", m.box((0.0008, 0.096, 0.070), (x0 - 0.0002, 0, 0.0)), "screen")]
    for k, (y, z, r) in enumerate([(-0.034, 0.012, 0.009), (0.034, 0.012, 0.009), (0.0, 0.012, 0.011), (0.0, -0.024, 0.006)]):  # two cameras, the projector, the colour camera
        parts.append((f"lens_{k}", m.cylinder(r, 0.001, (x0 + 0.0003, y, z), axis="x"), "lens"))
    parts.append(("button", m.cylinder(BUTTON_R, 0.0015, (BUTTON[0] - 0.0007, 0.0, BUTTON[2]), axis="x"), "button"))
    cols = [m.col_box("c_head", HEAD), m.col_box("c_handle", HANDLE, HANDLE_AT)]
    return m.free("scanner", (SCAN_REST[0], SCAN_REST[1], SCAN_REST[2] + 0.0005), parts, cols, LOOKS,
                  mass=SCAN_KG, friction=(0.9, 0.8), torsional_patch=0.015, start_quat=SCAN_REST_QUAT)  # a finger pad's full 36 x 20 mm contact


# -- the tablet: a simple scanning app, as the scanner's own software (Shining 3D's EXScan H) shows a scan: two live views of the part whose
# scanned surface appears as it is captured (nothing before), a distance indicator (ten 75 mm cells, the working window 200 to 700 mm green, a white mark at the
# scanner's distance), a tracking badge (green tracking, red lost, grey not scanning) and a coverage bar
TABLET_AT, TABLET_LEAN = (0.62, -0.22), 30.0
TABLET = (0.008, 0.20, 0.13)
N_REGIONS = 64  # the part's surface in patches for the live views
VIEWS = ((0.45, -1.0, 0.85), (-0.45, 1.0, 0.85))  # the two live views: from the part's -y side and its +y side, both from above
VIEW_AT = ((-0.026, -0.002), (0.056, -0.002))  # each view's middle on the screen (y, z), 70 mm across
VIEW_SCALE = 0.48  # the part drawn at about half its size


def _tablet_frame(mesh: m.Mesh) -> m.Mesh:
    """From the screen's own frame (the screen in y-z, facing -x, its middle at the origin) to the station: leaned back, on its stand."""
    return mesh.turned(TABLET_LEAN, "y").moved((TABLET_AT[0], TABLET_AT[1], TOP + 0.075))


def _cell(name, y, z, w, h, look, hidden=False):
    return (name, _tablet_frame(m.box((0.0006, w, h), (-TABLET[0] / 2 - 0.0006 - (0.0004 if hidden else 0.0), y, z))), look, hidden)


def _regions(p, n):
    """The surface points grouped into `n` patches (k-means on position and normal, a fixed seed): each point's patch."""
    x = np.c_[p[0], 0.015 * p[1]]
    rng = np.random.default_rng(1)
    c = x[rng.choice(len(x), n, replace=False)]
    for _ in range(25):
        lab = np.argmin(((x[:, None, :] - c[None]) ** 2).sum(-1), axis=1)
        c = np.array([x[lab == k].mean(0) if np.any(lab == k) else c[k] for k in range(n)])
    return lab


def _hull2(q):
    pts = sorted(set(map(tuple, np.round(q, 6))))
    if len(pts) < 3:
        return None

    def half(seq):
        h = []
        for r in seq:
            while len(h) >= 2 and (h[-1][0] - h[-2][0]) * (r[1] - h[-2][1]) - (h[-1][1] - h[-2][1]) * (r[0] - h[-2][0]) <= 0:
                h.pop()
            h.append(r)
        return h
    lo, up = half(pts), half(pts[::-1])
    ring = np.array(lo[:-1] + up[:-1])
    return ring if len(ring) >= 3 else None


def _patch(ring, depth):
    """A flat patch on the screen (screen y, z), `depth` in front of it, as a thin prism."""
    n = len(ring)
    x0 = -TABLET[0] / 2 - 0.0007 - depth
    v = np.vstack([np.c_[np.full(n, x0), ring], np.c_[np.full(n, x0 + 0.0002), ring]])
    f = [[0, i, i + 1] for i in range(1, n - 1)] + [[n, n + i + 1, n + i] for i in range(1, n - 1)]
    f += [[i, n + (i + 1) % n, (i + 1) % n] for i in range(n)] + [[i, n + i, n + (i + 1) % n] for i in range(n)]
    return m.Mesh(v, np.array(f))


REGION_OF = None  # each surface point's patch, set when the tablet is built
VIEW_PATCHES: set = set()  # the (view, patch) pairs drawn


def _tablet() -> m.Part:
    global REGION_OF
    parts = [("tablet", _tablet_frame(m.box(TABLET)), "tablet"),
             ("screen", _tablet_frame(m.box((0.0006, TABLET[1] - 0.012, TABLET[2] - 0.012), (-TABLET[0] / 2 - 0.0003, 0, 0))), "screen"),
             ("titlebar", _tablet_frame(m.box((0.0006, TABLET[1] - 0.012, 0.012), (-TABLET[0] / 2 - 0.0005, 0, 0.053))), "appbar"),
             ("stand", m.box((0.07, 0.10, 0.006), (TABLET_AT[0] + 0.03, TABLET_AT[1], TOP + 0.003)), "tablet"),
             ("prop", m.box((0.006, 0.08, 0.10), (TABLET_AT[0] + 0.06, TABLET_AT[1], TOP + 0.05)), "tablet")]
    for k in range(10):  # the distance indicator down the screen's left edge, near (bottom) to far (top); the coverage bar along the bottom
        z = -0.047 + k * 0.0095
        parts.append(_cell(f"range_{k}", -0.085, z, 0.010, 0.0085, "green_dim" if 3 <= k <= 8 else "red_dim"))  # green from 22.5 to 67.5 cm: cells wholly inside the 20 to 70 cm range
        parts.append(_cell(f"range_at_{k}", -0.085, z, 0.010, 0.0085, "white_lit", hidden=True))
        y = -0.060 + k * 0.0145
        parts.append(_cell(f"cover_{k}", y, -0.053, 0.013, 0.006, "grey_dim"))
        parts.append(_cell(f"cover_on_{k}", y, -0.053, 0.013, 0.006, "green_lit", hidden=True))
    parts.append(_cell("track", 0.070, 0.053, 0.030, 0.008, "grey_dim"))  # the tracking badge in the title bar
    parts.append(_cell("track_on", 0.070, 0.053, 0.030, 0.008, "green_lit", hidden=True))
    parts.append(_cell("track_lost", 0.070, 0.053, 0.030, 0.008, "red_lit", hidden=True))
    # the live views: empty until scanning starts; each patch of the surface appears, seen from that view, once it is scanned (the software
    # shows only what it has captured: it has no model of the part beforehand)
    p, nrm = surface_points()
    REGION_OF = _regions((p, nrm), N_REGIONS)
    centre = np.array([0.0, 0.0, 0.035])
    for a, (vd, at) in enumerate(zip(VIEWS, VIEW_AT)):
        v = np.asarray(vd, float) / np.linalg.norm(vd)
        right = np.cross(v, [0.0, 0.0, 1.0])
        right /= np.linalg.norm(right)
        up = np.cross(right, v)
        drawn = []
        for k in range(N_REGIONS):
            sel = (REGION_OF == k) & (nrm @ v > 0.15)
            if sel.sum() < 6:
                continue
            q = p[sel] - centre
            ring = _hull2(np.c_[-(q @ right), q @ up] * VIEW_SCALE + np.asarray(at))  # screen y runs to the viewer's left of the face it shows: the tablet faces -x
            if ring is not None:
                drawn.append((float((q @ v).mean()), k, ring))
        for rank, (_, k, ring) in enumerate(sorted(drawn)):  # nearer patches a little further out of the screen
            parts.append((f"view{a}_r{k}_on", _tablet_frame(_patch(ring, 0.00003 * rank + 0.00001)), "scanned", True))
            VIEW_PATCHES.add((a, k))
    return m.fixed("tablet", (0.0, 0.0, 0.0), parts, [], LOOKS)




# -- the scan's coverage: pure geometry, also run by check_score.py --------------------------------------------------------------


def surface_points(n_target: int = 5000, seed: int = 0):
    """Points spread evenly over the part's surface, with their outward normals, in its frame; its underside left out."""
    rng = np.random.default_rng(seed)
    faces = []  # (area, a function giving k points and their normals)

    def u(k, a, b):
        return rng.uniform(a, b, k)

    def flat(area, fn, normal):
        faces.append((area, lambda k: (fn(k), np.tile(np.asarray(normal, float), (k, 1)))))

    for sx in (-1, 1):
        flat((L / 2 - HOUSING_R) * W, lambda k, sx=sx: np.c_[sx * u(k, HOUSING_R, L / 2), u(k, -W / 2, W / 2), np.full(k, H)], (0, 0, 1))  # the base's top (its slots dropped below)
        flat(W * H, lambda k, sx=sx: np.c_[np.full(k, sx * L / 2), u(k, -W / 2, W / 2), u(k, 0, H)], (sx, 0, 0))  # the base's ends
        flat(W * (SHAFT_H - H), lambda k, sx=sx: np.c_[np.full(k, sx * HOUSING_R), u(k, -W / 2, W / 2), u(k, H, SHAFT_H)], (sx, 0, 0))  # the pedestal
    for sy in (-1, 1):
        flat(L * H, lambda k, sy=sy: np.c_[u(k, -L / 2, L / 2), np.full(k, sy * W / 2), u(k, 0, H)], (0, sy, 0))  # the base's long sides

        def end(k, sy=sy):  # the housing's end face round the bore
            out = np.empty((0, 3))
            while len(out) < k:
                x, z = u(4 * k, -HOUSING_R, HOUSING_R), u(4 * k, H, SHAFT_H + HOUSING_R)
                r = np.hypot(x, z - SHAFT_H)
                keep = ((z <= SHAFT_H) | (r <= HOUSING_R)) & (r >= BORE_R)
                out = np.vstack([out, np.c_[x[keep], np.full(keep.sum(), sy * W / 2), z[keep]]])
            return out[:k]

        flat(2 * HOUSING_R * (SHAFT_H - H) + math.pi * HOUSING_R ** 2 / 2 - math.pi * BORE_R ** 2, end, (0, sy, 0))

    def arch(k):
        a = u(k, 0.0, math.pi)
        return np.c_[HOUSING_R * np.cos(a), u(k, -W / 2, W / 2), SHAFT_H + HOUSING_R * np.sin(a)], np.c_[np.cos(a), np.zeros(k), np.sin(a)]

    faces.append((math.pi * HOUSING_R * W, arch))
    total = sum(a for a, _ in faces)
    pts, nrm = [], []
    for area, fn in faces:
        p, n = fn(max(1, round(n_target * area / total)))
        pts.append(p)
        nrm.append(n)
    p, n = np.vstack(pts), np.vstack(nrm).astype(float)
    in_slot = np.zeros(len(p), bool)
    for sx, sl, sw in SLOTS:  # the slots are holes: no surface there
        in_slot |= (np.abs(p[:, 0] - sx) < sl / 2) & (np.abs(p[:, 1]) < sw / 2) & (n[:, 2] > 0.99)
    return p[~in_slot], n[~in_slot]


SURFACE_P, SURFACE_N = surface_points()
TOP_PTS = SURFACE_N[:, 2] > math.cos(math.radians(30.0))  # the top: faces within 30 degrees of straight up (the base's top, the crown of the round); a view from 15 degrees up sees none of it
OBJECTS = [_turntable(), _part(), _scanner(), _tablet()]
BOXES = [(np.array([-L / 2, -W / 2, 0.0]), np.array([L / 2, W / 2, H])), (np.array([-HOUSING_R, -W / 2, H]), np.array([HOUSING_R, W / 2, SHAFT_H]))]


def hidden(cam, pts):
    """Whether the segment from the camera to each point (all in the part's frame) passes through the part before reaching it."""
    dv = pts - cam
    t_end = 1.0 - 1e-4
    out = np.zeros(len(pts), bool)
    with np.errstate(divide="ignore", invalid="ignore"):
        for lo, hi in BOXES:
            t1, t2 = (lo - cam) / dv, (hi - cam) / dv
            tmin = np.max(np.where(np.isnan(t1), -np.inf, np.minimum(t1, t2)), axis=1)
            tmax = np.min(np.where(np.isnan(t1), np.inf, np.maximum(t1, t2)), axis=1)
            out |= (tmax > tmin) & (tmin < t_end) & (tmax > 1e-6)
    ox, oz = cam[0], cam[2] - SHAFT_H  # the housing's round top: a cylinder along y, above the shaft's axis
    dx, dz = dv[:, 0], dv[:, 2]
    a = dx * dx + dz * dz
    b = 2 * (ox * dx + oz * dz)
    c = ox * ox + oz * oz - HOUSING_R ** 2
    disc = b * b - 4 * a * c
    ok = (disc > 0) & (a > 1e-12)
    sq = np.sqrt(np.where(ok, disc, 0.0))
    aa = np.where(ok, a, 1.0)
    for t in ((-b - sq) / (2 * aa), (-b + sq) / (2 * aa)):
        out |= ok & (t > 1e-6) & (t < t_end) & (np.abs(cam[1] + t * dv[:, 1]) <= W / 2) & (cam[2] + t * dv[:, 2] >= SHAFT_H)
    if cam[2] < 0.0:  # seen from below the platter's top: the platter, round the part's underside, is in the way
        with np.errstate(divide="ignore", invalid="ignore"):
            t0 = -cam[2] / dv[:, 2]
        out |= (t0 > 0) & (t0 < t_end) & (np.hypot(cam[0] + t0 * dv[:, 0], cam[1] + t0 * dv[:, 1]) <= PLATTER_R)
    return out


def view(cam, look, up):
    """For a scanner at `cam` looking along `look`, its long side across the view and `up` its short side (all in the part's frame):
    which surface points lie inside its view, and which of those it captures (facing it, not hidden)."""
    side = np.cross(up, look)
    rel = SURFACE_P - cam
    dist = rel @ look
    rng_ = np.linalg.norm(rel, axis=1)
    with np.errstate(divide="ignore", invalid="ignore"):
        inside = (dist > NEAR) & (dist < FAR) & (np.abs(np.arctan2(rel @ side, dist)) < HALF_H) & (np.abs(np.arctan2(rel @ up, dist)) < HALF_V)
        facing = np.einsum("ij,ij->i", SURFACE_N, -rel) / np.maximum(rng_, 1e-9) > FACING
    cand = np.flatnonzero(inside & facing)
    seen = np.zeros(len(SURFACE_P), bool)
    seen[cand[~hidden(cam, SURFACE_P[cand])]] = True
    return inside, seen


def part_distance(pts):
    """Distance from points (part frame) to the part's solid, 0 inside."""
    out = np.full(len(pts), np.inf)
    for lo, hi in BOXES:
        c, h = (lo + hi) / 2, (hi - lo) / 2
        out = np.minimum(out, np.linalg.norm(np.maximum(np.abs(pts - c) - h, 0.0), axis=1))
    r = np.maximum(np.hypot(pts[:, 0], pts[:, 2] - SHAFT_H) - HOUSING_R, 0.0)
    return np.minimum(out, np.hypot(r, np.maximum(np.abs(pts[:, 1]) - W / 2, 0.0)))


def turntable_distance(pts):
    """Distance from points (relative to the turntable's centre at table height) to its platter and stand, 0 inside."""
    r = np.hypot(pts[:, 0], pts[:, 1])
    plat = np.hypot(np.maximum(r - PLATTER_R, 0.0), np.maximum(np.maximum(pts[:, 2] - PLATTER_TOP, PLATTER_TOP - PLATTER_T - pts[:, 2]), 0.0))
    stand = np.hypot(np.maximum(r - STAND_R, 0.0), np.maximum(pts[:, 2] - STAND_H, 0.0))
    return np.minimum(plat, stand)


def judge(coverage: float, top: float, knocked: bool, struck: bool, clear: bool, held: int) -> tuple[bool, float]:
    """(whether the full-score state holds now, the score now), from the coverage so far (`top`: of the top alone), the trial's failure latches (`knocked`: the part
    moved off the platter's centre or tipped; `struck`: the scanner touched the part or the turntable, a fingertip touched the part, or the
    scanner was dropped), whether the hands and the scanner are clear of the part, and the steps the full-score state has held, counting this
    one."""
    if knocked or struck:
        return False, 0.0
    full = coverage >= COVER_FULL and clear
    if full and held >= HOLD_STEPS:
        return True, 1.0
    return full, 0.5 if coverage >= COVER_HALF and top >= COVER_TOP else 0.25 if coverage >= COVER_LOW else 0.0


def button_pressed(along, radial) -> bool:
    """Whether a fingertip presses button 4: a point inside its 6 mm radius and at its face (PRESS_AT; `along`: each point's height over the
    face, outward positive)."""
    along, radial = np.asarray(along, float), np.asarray(radial, float)
    return bool(np.any((radial < BUTTON_R) & (along >= PRESS_AT[0]) & (along <= PRESS_AT[1])))


def _box_points(size, n):
    """Points over a box's surface, in its frame: a grid on each face."""
    g = [np.linspace(-s / 2, s / 2, k) for s, k in zip(size, n)]
    p = np.stack(np.meshgrid(*g, indexing="ij"), axis=-1).reshape(-1, 3)
    return p[np.any(np.isclose(np.abs(p), np.asarray(size) / 2), axis=1)]


SCAN_PTS = np.vstack([_box_points(HEAD, (30, 37, 31)), _box_points(HANDLE, (16, 15, 50)) + np.asarray(HANDLE_AT)])  # 3 mm apart: a part's edge cannot touch between them unseen


class Scan:
    def reset(self, env=None) -> None:
        self.steps, self.held = 0, 0
        self.covered = np.zeros(len(SURFACE_P), bool)
        self.full, self.score = False, 0.0
        self.knocked, self.struck, self.dropped, self.tracking = False, False, False, False
        self.lit = None
        self.on, self.pressing, self.presses = False, False, 0  # scanning: off until button 4 is pressed (Preview / Scan / Pause)

    def step(self, env) -> None:
        self.steps += 1
        part, sc = m.state(env, "part"), m.state(env, "scanner")
        Rp, Rs = part.rotation(), sc.rotation()
        cam = sc.pos + Rs @ np.array([LENS_X, 0.0, 0.0])
        cam_p = part.to_local(cam)[0]
        inside, seen = view(cam_p, Rp.T @ Rs[:, 0], Rp.T @ Rs[:, 2])
        v_rel = float(np.linalg.norm(sc.lin_vel - part.lin_vel))
        w_rel = float(np.linalg.norm(sc.ang_vel - part.ang_vel))
        ok = v_rel < TRACK_V and w_rel < TRACK_W and inside.mean() >= TRACK_IN_VIEW
        if ok and not self.tracking and self.covered.any():  # lost before: the scanner finds its place again only on faces already scanned
            ok = int(seen.sum()) >= RELOCATE_PTS and int((seen & self.covered).sum()) >= RELOCATE_SHARE * int(seen.sum())
        # button 4: a fingertip pressed on it (within its 6 mm radius, at its face: PRESS_AT) starts scanning, and pauses it if pressed again
        btn = sc.pos + Rs @ np.asarray(BUTTON)
        rel = m.fingertips(env) - btn
        n_out = -Rs[:, 0]  # the button faces back, away from the lens
        along = rel @ n_out
        radial = np.linalg.norm(rel - np.outer(along, n_out), axis=1)
        press = button_pressed(along, radial)
        near_btn = (radial < 0.012) & (along < 0.015) & (along > -0.015)
        if os.environ.get("SCAN_DEBUG") and near_btn.any():
            k = int(np.argmin(np.where(near_btn, radial, np.inf)))
            print(f"[scan] at button 4: nearest fingertip {1000 * radial[k]:.1f} mm off its middle, {1000 * along[k]:.1f} mm over its face (deepest {1000 * along[near_btn].min():.1f}); scanner speed {np.linalg.norm(sc.lin_vel) * 1000:.0f} mm/s", flush=True)
        if self.pressing:  # a press counts again only after the fingertip has let go of the button: 12 mm off it (a fingertip resting at its face would toggle it every step)
            press = bool(np.any((radial < 0.012) & (along < 0.012) & (along > PRESS_AT[0] - 0.003)))
        if press and not self.pressing:
            self.on, self.presses = not self.on, self.presses + 1
            hit = (radial < BUTTON_R) & (along >= PRESS_AT[0]) & (along <= PRESS_AT[1])
            print(f"[scan] button 4 pressed ({self.presses}): scanning {'on' if self.on else 'paused'} at t {self.steps * float(env.unwrapped.step_dt):.2f}; "
                  f"fingertip {1000 * float(along[hit].min()):.1f} mm off the face, {1000 * float(radial[hit].min()):.1f} mm off the button's middle", flush=True)
        self.pressing = press
        self.tracking = ok and self.on  # the software tracks, and counts, only while scanning
        if self.tracking:
            self.covered |= seen
        cov = float(self.covered.mean())
        centre = np.asarray(TURNTABLE_XY, float)
        if float(np.linalg.norm(part.pos[:2] - centre)) > KNOCK_XY or part.tilt_deg() > KNOCK_TILT:
            self.knocked = True
        scan_w = sc.to_world(SCAN_PTS)
        near = min(float(part_distance(part.to_local(m.fingertips(env))).min()), float(part_distance(part.to_local(scan_w)).min()))
        on_table = float(turntable_distance(scan_w - np.array([TURNTABLE_XY[0], TURNTABLE_XY[1], TOP])).min())
        if near < STRIKE or on_table < STRIKE:
            self.struck = True
        if float(sc.lin_vel[2]) < -DROP_V and not gk.held(m.fingertips(env), sc, HANDLE_GRIP, resting=False):  # resting False: falling, it is off any rest
            self.dropped = True
        top = float(self.covered[TOP_PTS].mean())
        self.full, self.score = judge(cov, top, self.knocked, self.struck or self.dropped, near >= CLEAR, self.held + 1)
        self.held = self.held + 1 if self.full else 0
        self._show(float(np.linalg.norm(cam_p - np.array([0.0, 0.0, SHAFT_H]))), cov)
        if self.held == HOLD_STEPS:
            print(f"[scan] coverage {cov * 100:.1f} %, held 1 s: score 1.0", flush=True)
        if os.environ.get("SCAN_DEBUG") and self.steps % 20 == 0:
            print(f"[scan] t {self.steps * DT:.1f} s: coverage {cov * 100:.1f} %, in view {inside.mean() * 100:.0f} %, seen now {seen.mean() * 100:.1f} %, "
                  f"tracking {self.tracking} (v {v_rel:.2f} m/s, w {math.degrees(w_rel):.0f} deg/s); platter {math.degrees(m.state(env, 'turntable').joint('platter')):.0f} deg; "
                  f"part off centre {np.linalg.norm(part.pos[:2] - centre) * 1000:.1f} mm, tilt {part.tilt_deg():.1f}; nearest {near * 1000:.0f} mm, scanner to turntable {on_table * 1000:.0f} mm; "
                  f"knocked {self.knocked} struck {self.struck} dropped {self.dropped}; score {self.score}; "
                  f"cam {np.round(cam - np.array([0.0, 0.0, TOP]), 3)} look {np.round(Rs[:, 0], 2)} long {np.round(Rs[:, 1], 2)}", flush=True)
            rob = env.unwrapped.scene["right_robot"]
            gp = m._np(rob.data.body_link_pose_w)[0, rob.find_bodies("gripper")[0][0], :3] - m._np(env.unwrapped.scene.env_origins[0])
            print(f"[scan] dbg t {self.steps * DT:.1f}: right gripper {np.round(gp - np.array([0.0, 0.0, TOP]), 3)}, scanner {np.round(sc.pos - np.array([0.0, 0.0, TOP]), 3)}, apart {np.linalg.norm(gp - sc.pos) * 1000:.0f} mm", flush=True)

    def _show(self, dist: float, cov: float) -> None:
        cell = int(np.clip(dist / 0.075, 0, 9))
        state = "tracking" if (self.on and self.tracking) else ("lost" if self.on else "off")
        regions = tuple(bool(self.covered[REGION_OF == k].mean() >= 0.7) for k in range(N_REGIONS))
        lit = (cell, state, min(10, int(cov * 10 + 1e-9)), regions)
        if lit == self.lit:
            return
        old = self.lit
        self.lit = lit
        base = f"{m.ENV_PATH}/tablet/visual"
        if old is None or old[:3] != lit[:3]:
            for k in range(10):
                m.set_visible(f"{base}/range_at_{k}", k == cell)
                m.set_visible(f"{base}/cover_on_{k}", k < lit[2])
            m.set_visible(f"{base}/track_on", state == "tracking")
            m.set_visible(f"{base}/track_lost", state == "lost")
        for k in range(N_REGIONS):
            if old is None or old[3][k] != regions[k]:
                for a in range(len(VIEWS)):
                    if (a, k) in VIEW_PATCHES:
                        m.set_visible(f"{base}/view{a}_r{k}_on", regions[k])


SCAN_ = Scan()
SCAN_.reset()
m.on_step(SCAN_.step)
m.on_reset(SCAN_.reset)


def success(objects) -> bool:
    return SCAN_.full and SCAN_.held >= HOLD_STEPS


def score(objects) -> float:
    return SCAN_.score


# -- the demo: the right hand takes the scanner and holds it in two passes, low and steep; the left turns the table under each ------------------------

PART_MID = np.array([TURNTABLE_XY[0], TURNTABLE_XY[1], PART_Z + 0.035])
PASSES = [(15.0, 0.34), (60.0, 0.30)]  # (the view's elevation above level, degrees; the lens's distance from the part's middle, m): inside 200 to 700 mm
STROKES = int(os.environ.get("SCAN_STROKES", "7"))
SWEEP_STROKES = 16  # the sweep mode: two turns of about 50 degrees a stroke
STROKE_FROM, STROKE_DEG, RIM_R = 70.0, 60.0, 0.13
PRESS_Z = PLATTER_TOP + 0.013 - 0.002  # the left TCP over the platter: the fingertips, 13 mm below it, 2 mm into the rubber


def _scan_frame(look) -> np.ndarray:
    """The scanner's rotation when its lens looks along `look`, its head upright."""
    xs = np.asarray(look, float) / np.linalg.norm(look)
    zs = np.array([0.0, 0.0, 1.0]) - xs[2] * xs
    zs /= np.linalg.norm(zs)
    return np.column_stack([xs, np.cross(zs, xs), zs])


def _look(elev_deg: float) -> np.ndarray:
    e = math.radians(elev_deg)
    return np.array([0.0, math.cos(e), -math.sin(e)])  # from the -y side, `elev_deg` below level


def _handle_grip(Rs) -> d.Grip:
    """The hand on the handle from the scanner's -y side (its own -y), the fingers closing across the handle's depth: every view from 15 to
    60 degrees solves at 0.0 mm and 0 degrees in a seeded dry run (kit README 19); the other way round, the wrist runs to its 120 degree limit."""
    if os.environ.get("SCAN_FLIP", "0") == "1":  # turned over about the approach (wrist camera under the fingers): the wrist runs past its limit, 45 degrees out
        return d._as_grip(np.column_stack([Rs[:, 0], Rs[:, 2], -Rs[:, 1]]))
    return d._as_grip(np.column_stack([-Rs[:, 0], -Rs[:, 2], -Rs[:, 1]]))


def rolled(alpha_deg: float) -> d.Grip:
    """The hand's grip that holds the scanner looking at the part from the -y side, `alpha_deg` below level."""
    return _handle_grip(_scan_frame(_look(alpha_deg)))


def pass_tcp(elev_deg: float, dist: float) -> np.ndarray:
    """The right TCP that puts the scanner's lens `dist` from the part's middle, on the -y side, looking `elev_deg` below level."""
    look = _look(elev_deg)
    return PART_MID - (dist + LENS_X) * look + _scan_frame(look) @ np.asarray(HOLD)


LENS_DOWN_R = np.column_stack([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])  # lifted and rolled a quarter turn: the lens down, on the way to the views
LENS_DOWN_AT = (0.30, -0.24, TOP + 0.24)


def _press_grip(Rs, at, q, base) -> d.Grip:
    """The left hand's grip to press button 4 on a scanner held at rotation `Rs`: the fingers pointing into the button and closing along the
    handle (closed, 9 mm thick there; across, their 36 mm width would reach the right hand's fingers, 37 mm along). Of the grip and the grip
    turned over, the one the arm reaches at `at`."""
    best, err = None, 1e9
    for G in (np.column_stack([Rs[:, 2], -Rs[:, 1], Rs[:, 0]]), np.column_stack([-Rs[:, 2], Rs[:, 1], Rs[:, 0]])):
        qq = np.array(q, float)
        for p in (np.asarray(at) - 0.05 * Rs[:, 0], np.asarray(at)):
            for _ in range(6):
                qq, pe, re_ = d.ik.solve(p, d.ik.matrix_to_quat_xyzw(G), qq, base_pos=base, iters=80)
        e = pe * 1000 + math.degrees(re_)
        print(f"[scan] press grip candidate: {pe * 1000:.1f} mm, {math.degrees(re_):.1f} deg", flush=True)
        if e < err:
            best, err = G, e
    return d._as_grip(best)


def demo_script(ctx):
    """Modes: sweep, the solution, with the scanner never still: it climbs slowly while the table turns, from 15 to 20 degrees down over
    the first turn and on to 60 over the second. full: two passes, the scanner held still at each (1.0 in a run; in another, tracking dropped
    early in the first turn and came back only on faces already scanned, and it fell short). Checks: oneturn (a full turn at the low view, then the sweep up to the steep view, but no turn
    there), still (the scanner held still at the low view through two turns: a scanner set down on a stand), onearm (no turning: the right
    hand alone moves the scanner over the views it reaches on its side of the part), strike (the two passes, but the scanner touches the
    part's top between them), nobutton (the two passes without the press), hover (the sweep, the left fingertip stopping 2 mm short of the
    button: no press). none: the arms wait."""
    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode == "none":
        yield from l.wait(3.0)
        return
    if mode not in ("full", "sweep", "oneturn", "still", "onearm", "strike", "nobutton", "hover"):
        raise ValueError(f"unknown mode {mode!r}")
    cx, cy = TURNTABLE_XY

    def rim(deg: float, z: float) -> np.ndarray:
        a = math.radians(deg)
        return np.array([cx + RIM_R * math.cos(a), cy + RIM_R * math.sin(a), TOP + z])

    def strokes(n: int):
        """The left hand drags the platter round by its rubber top, `STROKE_DEG` a stroke, and comes back over it between strokes."""
        for _ in range(n):
            yield from l.move(rim(STROKE_FROM, PRESS_Z + 0.025), grip=d.down(0.0), finger=d.FINGER_CLOSED, t=0.8, label="over the platter")
            yield from l.move(rim(STROKE_FROM, PRESS_Z), t=0.4, label="fingertips on the rubber")
            yield from l.arc((cx, cy, TOP + PRESS_Z), (0.0, 0.0, 1.0), STROKE_DEG, t=2.0, label="drag the platter round")
            yield from l.move(l.at() + [0.0, 0.0, 0.025], t=0.3, label="lift off")

    held = {}  # the scanner's pose in the hand, as taken: the views are planned from it (a grip 15 mm off left the lens 6 cm off its views)

    def view(elev: float, dist: float):
        if not held:
            return pass_tcp(elev, dist), rolled(elev)
        look = _look(elev)
        Rh = _scan_frame(look) @ held["R"].T
        return PART_MID - (dist + LENS_X) * look - Rh @ held["p"], d._as_grip(Rh)

    lens = lambda: ctx.state("scanner").to_world(np.array([[LENS_X, 0.0, 0.0]]))[0]  # noqa: E731

    def go_view(elev: float, dist: float, t: float, label: str):
        """To a view, then the lens servoed onto its place: the 731 g scanner turns in the hand as the hand's pose changes (up to 6 cm off unservoed)."""
        tcp, g = view(elev, dist)
        yield from r.move(tcp, grip=g, t=t, label=label)
        yield from r.servo(PART_MID - dist * _look(elev), tol=0.004, timeout=1.2, gain=0.3, read=lens, label=f"{label}: the lens on its place")

    def to_pass(k: int):
        elev, dist = PASSES[k]
        yield from go_view(elev, dist, 2.0, f"the scanner to {elev:.0f} degrees")

    def away():
        yield from ctx.together(l.move(l.at() + [-0.08, 0.06, 0.08], t=1.2, label="away"),
                                r.move(r.at() + [-0.06, -0.06, 0.03], t=1.2, label="the scanner back"))
        yield from l.wait(2.0)

    # 1. The right hand takes the scanner by its handle, level from behind, as it stands on its head, and holds it there.
    env = ctx.emb._env
    sc = ctx.state("scanner")
    Rs = sc.rotation()
    g = _handle_grip(Rs)
    hold = sc.pos + Rs @ np.asarray(HOLD)
    yield from r.move(hold - 0.08 * g.rot[:, 2], grip=g, finger=d.FINGER_OPEN, t=2.0, label="behind the handle")
    yield from r.move(hold, t=1.0, label="fingers either side of the handle")
    yield from r.servo(hold, tol=0.003, timeout=1.5, gain=0.3, label="the hand's middle on the handle")
    print(f"[scan] at the handle: TCP off the hold point by {np.round(1000 * (r.tcp() - hold), 1)} mm", flush=True)
    yield from r.close(HANDLE[0], force=GRIP_N, label="take the scanner by its handle")

    def take_hold(what: str) -> None:
        sc_ = ctx.state("scanner")
        tp, Rt = d.ik.tcp_pose(ctx.observed("right")[:6], r.base)  # the hand as it is
        held.update(p=Rt.T @ (sc_.pos - tp), R=Rt.T @ sc_.rotation())
        turned = math.degrees(math.acos(np.clip((np.trace(held["R"].T @ _handle_grip(np.eye(3)).rot.T) - 1) / 2, -1.0, 1.0)))
        print(f"[scan] held, {what}: scanner origin from the TCP {np.round(1000 * held['p'], 1)} mm (nominal {np.round(1000 * _handle_grip(np.eye(3)).rot.T @ -np.asarray(HOLD), 1)}), "
              f"turned {turned:.1f} deg from the grasp's pose; scanner at {np.round(sc_.pos - np.array([0.0, 0.0, TOP]), 3)}", flush=True)

    def hand_for(Rs_to, centre):
        """The right TCP and grip that put the scanner, as held, at rotation `Rs_to` with its middle at `centre`."""
        Rh = np.asarray(Rs_to) @ held["R"].T
        return np.asarray(centre) - Rh @ held["p"], d._as_grip(Rh)

    # 2. The left hand presses button 4 (Preview / Scan / Pause) on the handle with its closed fingertips, as the right holds the scanner still.
    if mode != "nobutton":
        sc = ctx.state("scanner")
        Rs = sc.rotation()
        btn = sc.pos + Rs @ np.asarray(BUTTON)
        into = Rs[:, 0]  # into the button: it faces away from the lens
        pg = _press_grip(Rs, btn - 0.012 * into, ctx.observed("left")[:6], l.base)
        ends = m.fingertips(env)[[2, 8]]  # the left fingers' tip ends at their middles
        k_tip = [2, 8][int(np.argmin(np.linalg.norm(ends - btn, axis=1)))]
        tips = lambda: m.fingertips(env)[k_tip]  # noqa: E731  the one nearer the button, on its axis
        yield from l.move(btn - 0.05 * into, grip=pg, finger=d.FINGER_CLOSED, t=2.5, label="the fingertips before button 4")
        depth = -0.002 if mode == "hover" else 0.010  # hover: the fingertip stops 2 mm short of the face; pressed, it is aimed 10 mm in and stops at the face
        yield from l.servo(btn + depth * into, tol=0.0015, timeout=2.0, gain=0.3, read=tips, label="press button 4: scan" if mode != "hover" else "hover over button 4")
        print(f"[scan] pressed: scanning {SCAN_.on}", flush=True)
        yield from l.move(l.at() - 0.06 * into, t=0.8, label="off the button")
    # 3. The right hand lifts the scanner, its weight under the hand, and rolls it a quarter turn, lens down, on its way to the views.
    yield from r.move(r.at() + [0.0, 0.0, 0.06], t=1.2, label="lift it off the table")
    yield from r.wait(0.3)
    take_hold("lifted")
    tcp, gh = hand_for(LENS_DOWN_R, LENS_DOWN_AT)
    yield from r.move(tcp, grip=gh, t=2.0, label="roll it a quarter turn, lens down")
    take_hold("lens down")
    if mode == "still":
        yield from to_pass(0)
        yield from strokes(2 * STROKES)
        yield from away()
        return
    if mode == "onearm":
        for k, (elev, dist) in enumerate(PASSES):
            yield from to_pass(k)
            for dx in (-0.12, 0.12, 0.0):  # along the part's side, the view turned back to its middle
                turn = math.degrees(math.asin(dx / dist))
                grip = d._as_grip(d.axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(turn)) @ rolled(elev).rot)
                yield from r.move(pass_tcp(elev, dist) + [dx, 0.0, 0.0], grip=grip, t=3.0, label="along the part's side")
        yield from away()
        return
    if mode in ("sweep", "hover"):  # one stroke of the table at a time, the scanner moving on to its next view in step with it
        yield from to_pass(0)
        views = list(np.linspace(15.0, 20.0, SWEEP_STROKES // 2 + 1)[1:]) + list(np.linspace(20.0, 60.0, SWEEP_STROKES // 2 + 1)[1:])
        for elev in views:
            dist = PASSES[0][1] + (PASSES[1][1] - PASSES[0][1]) * (elev - PASSES[0][0]) / (PASSES[1][0] - PASSES[0][0])
            yield from ctx.together(strokes(1), go_view(elev, dist, 2.5, f"the scanner on up, {elev:.0f} degrees"))
        yield from away()
        return
    # 2. Each pass: the scanner into the working window, aimed at the part; then the table turned a full turn and a stroke under it.
    yield from to_pass(0)
    yield from strokes(STROKES)
    if mode == "strike":  # the scanner's lens brought down onto the housing's top, and back
        yield from to_pass(1)
        e = math.radians(PASSES[1][0])
        look = np.array([0.0, math.cos(e), -math.sin(e)])
        yield from r.move(pass_tcp(*PASSES[1]) + (PASSES[1][1] - 0.04) * look, t=2.0, label="the scanner onto the part")
        yield from to_pass(1)
    yield from to_pass(1)
    if mode != "oneturn":
        yield from strokes(STROKES)
    else:
        yield from r.wait(3.0)
    # 3. Both hands away from the part.
    yield from away()
