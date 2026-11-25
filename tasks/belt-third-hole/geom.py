"""Geometry of the belt task (pure NumPy; no simulator). World frame: x forward from the arms, y to the robot's
left, z up, metres. The form lies on its back with its waist toward the arms; the belt goes round the waist in the
y-z plane at x = X_B.

Belt arc length s is measured from the fold round the buckle's hinge bar (s = 0) toward the tip. Holes are counted
from the tip: hole 1 is nearest the tip (loosest), hole 5 nearest the buckle.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475                    # station table top

# -- the form and the trousers ----------------------------------------------------------------------------------
WAIST_A = 0.120                 # half width of the form's waist (y), 26 in (66 cm) waist (ESTIMATE of the section)
WAIST_B = 0.088                 # half depth (z)
SHELL = 0.0015                  # denim over the form
LIFT = 0.020                    # the lying form rests on its seat; the back of the waist is this high (ESTIMATE)
X_W = 0.250                     # waist edge of the form (toward the arms)
FORM_LEN = 0.356                # 14 in tall form, lying along +x
A1, B1 = WAIST_A + SHELL, WAIST_B + SHELL          # outer surface of the trousers at the belt
ZC = TOP + LIFT + B1                                # centre of the waist section
YC = 0.0

# -- the belt -----------------------------------------------------------------------------------------------------
W = 0.030                       # strap width
T = 0.0035                      # strap thickness
X_B = X_W + 0.022               # belt centre plane (the belt's near edge 7 mm in from the waist edge)
PITCH = 0.025                   # link length (one hole every link in the hole run)
FOLD_T = 0.002                  # the leather at the fold is skived to 2 mm per layer
BAR_R = 0.002                   # frame wire radius (4 mm bar)
HOLE_X, HOLE_Y = 0.005, 0.008   # hole across the strap x along it (oval punched holes, ESTIMATE)
MARGIN = 0.060                  # round 2: hole 3 sits ~60 mm past a perfectly tight wrap (rounds to link 29: 737.5 mm
                                # fold to middle hole, a 29 in belt on the 26 in waist; 58 mm slack at hole 3, hole 5 snug)
N_HOLES = 5
TAIL = 0.150                    # strap past hole 1 to the tip
KG_PER_M = 0.095                # 3.5 mm x 30 mm leather (900 kg/m3)

# -- the buckle: a single-prong frame buckle --------------------------------------------------------------------
FRAME_IN_X = 0.033              # inside, across the strap
FRAME_IN_Y = 0.024              # inside, along the strap (hinge bar to front bar, between bar surfaces)
FRAME_L = FRAME_IN_Y + 2 * BAR_R   # hinge bar centre to front bar centre: 28 mm
FRAME_MASS = 0.022
PRONG_R = 0.0016                # 3.2 mm prong
PRONG_L = FRAME_L + 0.003       # pivot to tip (the tip rests on the front bar)
PRONG_RISE = math.radians(50)   # the prong's root is bent up this much over its first 8 mm
PRONG_ROOT = 0.008
PRONG_MASS = 0.003
HINGE_Y = 0.012                 # hinge bar's y when the belt is fastened (frame centred near y 0)

# -- keeper -------------------------------------------------------------------------------------------------------
KEEPER_LINK = 2                 # fixed leather keeper round link 2 (50-75 mm from the fold), stitched into the fold
KEEPER_GAP = 0.007              # room under the keeper for the tail (3.5 mm strap + clearance, ESTIMATE)
KEEPER_T = 0.002
KEEPER_LEN = 0.012

# -- belt loops on the trousers: angle from the top (rad, + toward +y) ---------------------------------------------
LOOPS = {"front_left": math.radians(50), "back_left": math.radians(130), "back_centre": math.radians(180),
         "back_right": math.radians(230), "front_right": math.radians(310)}
LOOP_W = 0.010                  # along the belt
LOOP_GAP = 0.011                # radial room inside a loop (loose denim loops, ESTIMATE)
LOOP_T = 0.002
LOOP_IN_X = 0.036               # inside, across the belt


def ellipse_pt(th, extra=0.0):
    """Point and outward normal on the trousers' surface offset outward by ``extra``; th from the top toward +y."""
    y0, z0 = A1 * math.sin(th), B1 * math.cos(th)
    n = np.array([math.sin(th) / A1, math.cos(th) / B1])
    n /= np.linalg.norm(n)
    return np.array([YC + y0, ZC + z0]) + extra * n, n


def _ellipse_len(extra, n=4000):
    th = np.linspace(0, 2 * math.pi, n + 1)
    pts = np.array([ellipse_pt(t, extra)[0] for t in th])
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


PERIM = _ellipse_len(T / 2)                          # the belt's centre line, wrapped tight
S_HOLE3 = PERIM + MARGIN
# round s of hole 3 to a link centre so every hole is in the middle of a link
_i3 = int(round(S_HOLE3 / PITCH - 0.5))
S_HOLE3 = (_i3 + 0.5) * PITCH
HOLE_S = {k: S_HOLE3 + (3 - k) * PITCH for k in range(1, N_HOLES + 1)}   # hole 1 = loosest = furthest out
HOLE_LINK = {k: int(HOLE_S[k] / PITCH) for k in HOLE_S}
N_LINKS = int(math.ceil((HOLE_S[1] + TAIL) / PITCH))
BELT_LEN = N_LINKS * PITCH
LINK_MASS = KG_PER_M * PITCH


def link_thick(i):
    """Links 0-2 are the fold (two layers round the bar, the keeper stitched in at link 2)."""
    return 2 * FOLD_T + 2 * BAR_R if i <= KEEPER_LINK else T


def polyline_walk(pts, start, step):
    """Points along a polyline ``pts`` (N, 2) from its start, each ``step`` from the last (straight chords)."""
    out = [np.asarray(start, float)]
    seg = 0
    cur = out[0]
    while True:
        found = False
        while seg < len(pts) - 1:
            a, b = pts[seg], pts[seg + 1]
            d = b - a
            f = a - cur
            qa, qb, qc = d @ d, 2 * f @ d, f @ f - step * step
            disc = qb * qb - 4 * qa * qc
            if disc >= 0 and qa > 0:
                r = math.sqrt(disc)
                for t in sorted(((-qb - r) / (2 * qa), (-qb + r) / (2 * qa))):
                    if 0 <= t <= 1:
                        p = a + t * d
                        if np.dot(p - cur, d) > 0:
                            out.append(p)
                            cur = p
                            found = True
                            break
            if found:
                break
            seg += 1
        if not found:
            return np.array(out)


def ring_path(theta0, sweep, extra, n=1500):
    th = theta0 + np.linspace(0, sweep, n)
    return np.array([ellipse_pt(t, extra)[0] for t in th])


def link_poses_from_joints(joints):
    """World (R, p) per link from the joint points (N+1, 2) in the y-z plane; link frame x along the strap (toward
    the tip), z its in-plane normal (outward where the strap wraps the form), y = z x x = world -x."""
    out = []
    for i in range(len(joints) - 1):
        a, b = joints[i], joints[i + 1]
        t = (b - a) / np.linalg.norm(b - a)
        x = np.array([0.0, t[0], t[1]])
        c = (a + b) / 2
        z = np.array([0.0, -t[1], t[0]])        # in-plane normal: outward on the ring (travel is +theta)
        y = np.cross(z, x)
        out.append((np.column_stack([x, y, z]), np.array([X_B, c[0], c[1]])))
    return out


def link_frame_y_sign(poses):
    """+1 if every link's y axis is world +x (consistent layout), else -1 (all must agree)."""
    s = {int(np.sign(R[0, 1])) for R, _ in poses}
    assert len(s) == 1, s
    return s.pop()


# ---------------------------------------------------------------------------------------------- start layouts
FOLD_EXTRA = T / 2 + 0.0012     # link centre line height above the trousers (all links; the fold links are
                                # thicker outward)
FOLD_TOP = 2 * FOLD_T + 2 * BAR_R - T / 2     # top of links 0-1 above their centre line (6.25 mm)
BAR_Z = -T / 2 + FOLD_T + BAR_R              # hinge bar centre above link 0's centre line (2.25 mm)


def theta_of_y_top(y):
    return math.asin(max(-1.0, min(1.0, (y - YC) / A1)))


def _bisect(f, lo, hi, n=60):
    flo = f(lo)
    for _ in range(n):
        mid = (lo + hi) / 2
        if (f(mid) > 0) == (flo > 0):
            lo, flo = mid, f(mid)
        else:
            hi = mid
    return (lo + hi) / 2


def open_layout(hinge_y=HINGE_Y):
    """The open belt's start layout, read from open_layout.json (made by make_layout.py from _open_layout_raw and
    relax_tongue)."""
    import json
    import pathlib
    f = pathlib.Path(__file__).resolve().parent / "open_layout.json"
    if f.exists():
        return np.array(json.loads(f.read_text())["joints"])
    return _open_layout_raw(hinge_y)


def _open_layout_raw(hinge_y=HINGE_Y):
    """The open belt: buckle end over the top with the frame lying flat, the strap round through all five loops,
    the tongue leaving the front-right loop, curling outward and hanging down the form's right side to the table.
    Returns joint points (N_LINKS+1, 2) in (y, z)."""
    th0 = theta_of_y_top(hinge_y)
    th_leave = math.radians(318)
    ring = ring_path(th0, th_leave - th0, FOLD_EXTRA)
    p0, _ = ellipse_pt(th0, FOLD_EXTRA)
    pts = list(ring)
    # leave tangentially and curl outward and down (radius R), then hang straight down, then lie on the table
    a, b = ring[-2], ring[-1]
    t = (b - a) / np.linalg.norm(b - a)
    R = 0.040
    nrm = np.array([-t[1], t[0]])                 # turn left: up, over toward -y, then down outside the form
    c = b + R * nrm
    ang0 = math.atan2(b[1] - c[1], b[0] - c[0])
    k = 0
    while True:
        k += 1
        ang = ang0 + math.radians(k)
        p = c + R * np.array([math.cos(ang), math.sin(ang)])
        tan = np.array([-math.sin(ang), math.cos(ang)])
        pts.append(p)
        if tan[1] < -0.999 or k > 300:
            break
    bottom = TOP + T / 2 + 0.0005
    y_hang = pts[-1][0]
    z = pts[-1][1]
    rr = 0.015
    while z > bottom + rr:
        z -= 0.002
        pts.append(np.array([y_hang, z]))
    for k in range(1, 46):                        # quarter bend onto the table, toward -y
        ang = math.radians(2 * k)
        pts.append(np.array([y_hang - rr * (1 - math.cos(ang)), bottom + rr - rr * math.sin(ang)]))
    yy = pts[-1][0]
    for k in range(1, 200):
        pts.append(np.array([yy - 0.002 * k, bottom]))
    pts = np.array(pts)
    joints = polyline_walk(pts, p0, PITCH)
    assert len(joints) >= N_LINKS + 1, (len(joints), N_LINKS)
    return joints[:N_LINKS + 1]


def fastened_layout(hole=3, hinge_y=HINGE_Y, hole_shift=0.0, keeper=True):
    """The belt fastened at ``hole`` (prong through it, tail under the keeper), for physics probes. The ring floats
    a few mm off the trousers so the loop length matches the hole. Returns joint points."""
    th0 = theta_of_y_top(hinge_y)
    s_target = HOLE_S[hole]
    y_hole = hinge_y - 0.0048 + hole_shift          # where the prong's bent root crosses the tongue
    G_ = [0.0]

    def over_h(y):
        """Tongue centre-line height over the trousers near the top, as a function of y (overlap zone)."""
        hy = hinge_y
        g = G_[0]
        top01 = FOLD_EXTRA + g + FOLD_TOP + T / 2 + 0.0015          # over links 0-1
        top2 = FOLD_EXTRA + g + T + 0.0015                          # over a single link
        if not keeper and hy + (KEEPER_LINK + 0.5) * PITCH - 0.012 < y < hy + (KEEPER_LINK + 1) * PITCH + 0.006:
            return FOLD_EXTRA + g + FOLD_TOP + KEEPER_GAP + KEEPER_T + T / 2 + 0.001     # over the keeper
        if y < hy - 0.024:
            return FOLD_EXTRA + g
        if y < hy - 0.004:
            return FOLD_EXTRA + g + (top01 - FOLD_EXTRA - g) * (y - (hy - 0.024)) / 0.020
        fe = (KEEPER_LINK + 1) * PITCH
        if y < hy + fe:
            return top01
        if y < hy + fe + 0.012:
            return top01 + (top2 - top01) * (y - (hy + fe)) / 0.012
        return top2

    def build(g):
        G_[0] = g
        ring = ring_path(th0, 2 * math.pi - 0.35, FOLD_EXTRA + g)
        # the last part of the way round: from theta 2pi-0.35 up to the top, following over_h(y) from y < hy-0.024
        ths = np.linspace(2 * math.pi - 0.35, 2 * math.pi + th0 + 2.4, 2200)
        tongue = []
        for th in ths:
            y = A1 * math.sin(th)
            if th > 2 * math.pi + th0 - 0.3 or th > 2 * math.pi - 0.15:
                h = over_h(y) if th < 2 * math.pi + 1.2 else over_h(1.0)
                if th < 2 * math.pi + 0.25 and y < hinge_y - 0.024:
                    h = FOLD_EXTRA + g
            else:
                h = FOLD_EXTRA + g
            lp = abs(th - 2 * math.pi - LOOPS["front_left"])
            if lp < 0.14:
                h = max(h, LOOP_GAP + LOOP_T + T / 2 + 0.001)
            tongue.append(ellipse_pt(th, h)[0])
        pts = np.vstack([ring, np.array(tongue)])
        p0, _ = ellipse_pt(th0, FOLD_EXTRA + g)
        return pts, p0

    def hole_err(g):
        pts, p0 = build(g)
        joints = polyline_walk(pts, p0, PITCH)
        i = HOLE_LINK[hole]
        c = (joints[i] + joints[i + 1]) / 2
        return c[0] - y_hole          # >0: hole sits past (left of) where it should; needs a bigger ring

    g = _bisect(lambda g: -hole_err(g), 0.0, 0.010)
    pts, p0 = build(g)
    joints = polyline_walk(pts, p0, PITCH)
    return joints[:N_LINKS + 1], g


FLAT_EI = 0.004                 # N m^2, the strap's flat bending stiffness (ESTIMATE, see README)


def _bump(y, th, hinge_y=HINGE_Y):
    """Height of what lies on the trousers along the top (the buckle end, its fold and keeper, the frame and prong,
    the front-left loop), for the open layout's resting tongue."""
    h = 0.0
    if abs(th - LOOPS["front_left"]) < math.radians(8):
        h = LOOP_GAP + LOOP_T
    if -math.radians(60) < th < math.radians(60):
        if hinge_y - FRAME_L - 0.006 < y < hinge_y + 0.002:
            h = max(h, 0.011)
        elif hinge_y + 0.002 <= y < hinge_y + (KEEPER_LINK + 1) * PITCH:
            h = max(h, 2 * FOLD_T + 2 * BAR_R + 0.0012)
            if abs(y - (hinge_y + (KEEPER_LINK + 0.5) * PITCH)) < KEEPER_LEN / 2 + 0.002:
                h = max(h, 2 * FOLD_T + 2 * BAR_R + KEEPER_GAP + KEEPER_T + 0.0012)
        elif y >= hinge_y + (KEEPER_LINK + 1) * PITCH:
            h = max(h, T + 0.0012)
    return h


def _penetration(p, hinge_y=HINGE_Y):
    """How far a strap centre-line point is inside the trousers (plus half the strap and whatever lies on them) or
    below the table top."""
    y, z = p[0] - YC, p[1] - ZC
    th = math.atan2(y, z)
    r = math.hypot(y, z)
    rs = math.hypot(A1 * math.sin(th), B1 * math.cos(th))
    pen = max(0.0, rs + T / 2 + _bump(p[0], th, hinge_y) - r)
    return max(pen, TOP + T / 2 - p[1])


def relax_tongue(joints, first_free, iters=4000, hinge_y=HINGE_Y):
    """Equilibrium of the free end (links first_free..end) under gravity with straight-rest joint springs
    (k = FLAT_EI / PITCH), resting on the trousers and the table. Planar, numeric gradient descent."""
    joints = np.array(joints, float)
    n = len(joints) - 1
    d0 = joints[first_free] - joints[first_free - 1]
    base_ang = math.atan2(d0[1], d0[0])
    ang = np.array([math.atan2(*(joints[i + 1] - joints[i])[::-1]) for i in range(first_free, n)])
    k = FLAT_EI / PITCH
    mg = LINK_MASS * 9.81
    Kp = 2e4

    def build(a):
        pts = [joints[first_free]]
        for t in a:
            pts.append(pts[-1] + PITCH * np.array([math.cos(t), math.sin(t)]))
        return np.array(pts)

    def energy(a):
        rel = np.diff(np.r_[base_ang, a])
        rel = (rel + math.pi) % (2 * math.pi) - math.pi
        pts = build(a)
        e = 0.5 * k * np.sum(rel ** 2)
        mids = (pts[:-1] + pts[1:]) / 2
        e += mg * np.sum(mids[:, 1])
        for q in np.vstack([pts, mids]):
            pen = _penetration(q, hinge_y)
            e += 0.5 * Kp * pen * pen
        return e

    step = 2e-3
    a = ang.copy()
    e = energy(a)
    for it in range(iters):
        g = np.zeros_like(a)
        for j in range(len(a)):
            da = np.zeros_like(a)
            da[j] = 1e-5
            g[j] = (energy(a + da) - energy(a - da)) / 2e-5
        trial = a - step * g / max(1.0, np.max(np.abs(g)) * step / 0.02)
        et = energy(trial)
        if et < e:
            a, e = trial, et
            step *= 1.2
        else:
            step *= 0.5
        if step < 1e-9:
            break
    out = joints.copy()
    out[first_free:] = build(a)
    return out, e


def tighten_layout():
    """The 'fastened at hole 2' start (from tighten_layout.json, made by make_layout.py)."""
    import json
    import pathlib
    d = json.loads((pathlib.Path(__file__).resolve().parent / "tighten_layout.json").read_text())
    return np.array(d["joints"]), d["ring_float_m"]


# the three start layouts (chosen by the reset seed, see beltkit): how far the tip is drawn through the buckle
# (hole 1 past the hinge bar, m) and where the buckle sits on the waist (hinge bar y, m)
START_LAYOUTS = [(0.070, HINGE_Y), (0.075, HINGE_Y - 0.008), (0.080, HINGE_Y + 0.008)]


def threaded_layout(idx=0):
    """Start layout ``idx``: joints (from threaded_layout_<idx>.json, made by make_layout.py) and the bow height."""
    import json
    import pathlib
    f = pathlib.Path(__file__).resolve().parent / f"threaded_layout_{idx}.json"
    if f.exists():
        d = json.loads(f.read_text())
        return np.array(d["joints"]), d["bow_m"]
    past, hy = START_LAYOUTS[idx]
    return _threaded_layout_raw(hinge_y=hy, hole1_past=past)


def _threaded_layout_raw(hinge_y=HINGE_Y, hole1_past=0.070, g=0.0015, bow=False):
    """The belt's end already through the buckle (unfastened): the ring hugs the trousers, the strap leaving the
    front-right loop rises in a loose bow and passes under the frame's front bar (the frame lies on it), up over
    the fold and over the prong turned back on it, and on over the buckle end and the keeper (outside it) down the
    left side. The bow's height is set so hole 1 lies ``hole1_past`` beyond the hinge bar. Returns (joints, bow)."""
    th0 = theta_of_y_top(hinge_y)
    y_h1 = hinge_y + hole1_past
    fe = (KEEPER_LINK + 1) * PITCH
    th_loop = 2 * math.pi + LOOPS["front_right"] - 2 * math.pi + 0.10       # just past the front-right loop
    th_bar = 2 * math.pi + theta_of_y_top(hinge_y - FRAME_L - 0.012)
    H_ = [0.0]

    def over_h(y, th):
        prong_top = 2 * PRONG_R + 0.0020      # round 2: the turned-back prong clears link 1 (tip ~3.5 mm up)
        top01 = FOLD_EXTRA + g + FOLD_TOP + T / 2 + prong_top + 0.001
        keeper = FOLD_EXTRA + g + FOLD_TOP + KEEPER_GAP + KEEPER_T + T / 2 + 0.001     # over the keeper
        top2 = FOLD_EXTRA + g + T + 0.0015
        hy = hinge_y
        if th < th_bar:
            u = (th - th_loop) / (th_bar - th_loop)
            h = FOLD_EXTRA + g + (H_[0] * math.sin(math.pi * u) if 0 < u < 1 else 0.0)
        elif y < hy - 0.026:
            h = FOLD_EXTRA + g
        elif y < hy - 0.004:
            h = FOLD_EXTRA + g + (top01 - FOLD_EXTRA - g) * (y - (hy - 0.026)) / 0.022
        elif y < hy + fe - PITCH - 0.006:
            h = top01
        elif y < hy + fe + 0.006:
            h = keeper
        elif y < hy + fe + 0.020:
            h = keeper + (top2 - keeper) * (y - (hy + fe + 0.006)) / 0.014
        else:
            h = top2
        if abs(th - 2 * math.pi - LOOPS["front_left"]) < 0.14:
            h = max(h, LOOP_GAP + LOOP_T + T / 2 + 0.001)
        return h

    def build(H):
        H_[0] = H
        ring = ring_path(th0, LOOPS["front_right"] + 0.02, FOLD_EXTRA + g)
        ths = np.linspace(LOOPS["front_right"] + 0.02, 2 * math.pi + th0 + 2.6, 3000)
        tongue = []
        for th in ths:
            y = A1 * math.sin(th)
            h = over_h(y if th < 2 * math.pi + 1.2 else 1.0, th)
            tongue.append(ellipse_pt(th, h)[0])
        pts = np.vstack([ring, np.array(tongue)])
        p0, _ = ellipse_pt(th0, FOLD_EXTRA + g)
        return pts, p0

    def hole_err(H):
        pts, p0 = build(H)
        joints = polyline_walk(pts, p0, PITCH)
        i = HOLE_LINK[1]
        c = (joints[i] + joints[i + 1]) / 2
        return c[0] - y_h1

    if bow:
        H = _bisect(lambda H: -hole_err(H), 0.0, 0.08)
    else:                     # no bow: the slack is spread round the ring (it floats g off the trousers)
        H = 0.0
        g0 = [g]

        def err_g(gg):
            nonlocal g
            g = gg
            return hole_err(0.0)
        g = _bisect(lambda gg: -err_g(gg), 0.0, 0.0065)
    pts, p0 = build(H)
    joints = polyline_walk(pts, p0, PITCH)
    print(f"threaded layout: bow {H * 1000:.1f} mm, ring float {g * 1000:.2f} mm")
    return joints[:N_LINKS + 1], H
