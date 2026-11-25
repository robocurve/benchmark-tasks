"""What the scorer reads from the belt's link poses (the same things a photo shows). Pure NumPy."""

from __future__ import annotations

import math

import numpy as np

import beltkit as B
import geom as G

WAIST_AXIS = np.array([G.X_B, G.YC, G.ZC])      # a point on the waist's centre line (which runs along x)


def outward(p):
    """Unit vector from the waist's centre line out through point p (in the belt's y-z plane)."""
    v = np.array([0.0, p[1] - G.YC, p[2] - G.ZC])
    return v / max(np.linalg.norm(v), 1e-9)


def link_outward_z(st, i):
    """Link i's normal, signed so that it points away from the waist (the strap's outer face)."""
    R, p = st.R(f"link_{i:02d}"), st.p(f"link_{i:02d}")
    z = R[:, 2]
    return z if float(z @ outward(p)) >= 0 else -z


def prong_points(st):
    Rp, pp = st.R("prong"), st.p("prong")
    return [pp + Rp @ v for v in B.prong_segments()]


def prong_in_hole(st, k, tol=0.0):
    """True if the prong's centre line passes through hole ``k`` (crosses the hole link's mid-plane inside the
    opening, less the prong's radius minus ``tol``)."""
    i = G.HOLE_LINK[k]
    R, p = st.R(f"link_{i:02d}"), st.p(f"link_{i:02d}")
    pts = prong_points(st)
    for a, b in zip(pts[:-1], pts[1:]):
        al, bl = R.T @ (a - p), R.T @ (b - p)
        if al[2] * bl[2] < 0:
            t = al[2] / (al[2] - bl[2])
            x = al + t * (bl - al)
            if abs(x[0]) < G.HOLE_Y / 2 + tol and abs(x[1]) < G.HOLE_X / 2 + tol:
                return True
    return False


def hole_engaged(st):
    """The hole the prong passes through (1-5), or 0."""
    for k in range(1, G.N_HOLES + 1):
        if prong_in_hole(st, k, tol=0.0005):
            return k
    return 0


def prong_from_body_side(st, k):
    """The prong passes through hole k the right way round: its pivot on the body side of the strap, its tip on the
    outer side (a photo of the buckle shows the tip out of the hole on the strap's outer face)."""
    i = G.HOLE_LINK[k]
    p = st.p(f"link_{i:02d}")
    n = link_outward_z(st, i)
    pts = prong_points(st)
    return float((pts[0] - p) @ n) < 0 < float((pts[-1] - p) @ n)


def prong_tip_to_front_bar(st):
    """Distance (m) from the prong's tip to the front bar's axis: the prong lies along the frame with its tip
    resting on the bar when this is about the two radii (3.6 mm)."""
    Rf, pf = st.R("frame"), st.p("frame")
    tip = prong_points(st)[-1]
    l = Rf.T @ (tip - pf)
    return float(math.hypot(l[0] + G.FRAME_L, l[2]))


def frame_angle_deg(st):
    """Angle between the frame's plane and the strap it lies on (the link under the front bar), deg."""
    Rf, pf = st.R("frame"), st.p("frame")
    fb = pf + Rf @ np.array([-G.FRAME_L, 0.0, 0.0])
    best = None
    for i in range(3, G.N_LINKS):
        d = np.linalg.norm(st.p(f"link_{i:02d}") - fb)
        if best is None or d < best[0]:
            best = (d, i)
    Rl = st.R(f"link_{best[1]:02d}")
    c = abs(float(np.dot(Rf[:, 2], Rl[:, 2])))
    return math.degrees(math.acos(min(1.0, c))), best[1]


def front_bar_over_strap(st):
    """The front bar lies over the outer face of a tongue link (the strap passes under the front bar): returns the
    gap (m) between the bar and that link's outer face and the link, or None if no tongue link is under the bar."""
    Rf, pf = st.R("frame"), st.p("frame")
    fb = pf + Rf @ np.array([-G.FRAME_L, 0.0, 0.0])
    best = None
    for i in range(3, G.N_LINKS):
        R, p = st.R(f"link_{i:02d}"), st.p(f"link_{i:02d}")
        l = R.T @ (fb - p)
        n = link_outward_z(st, i)
        h = float((fb - p) @ n)
        if abs(l[0]) <= G.PITCH / 2 + 0.002 and abs(l[1]) < G.W / 2 and h > 0:
            gap = h - G.T / 2 - G.BAR_R
            if best is None or abs(gap) < abs(best[0]):
                best = (gap, i)
    return best


def through_window_outward(st):
    """The tongue passes out through the frame's window: a link past the hole run's start crosses the frame's plane
    inside the window from the frame's body side to its outer side (toward the tail), with the frame's outer face
    turned away from the waist (not flipped over). Returns the link or None."""
    Rf, pf = st.R("frame"), st.p("frame")
    if float(Rf[:, 2] @ outward(pf)) <= 0:
        return None
    for i in range(G.HOLE_LINK[5] - 4, G.N_LINKS):
        R, p = st.R(f"link_{i:02d}"), st.p(f"link_{i:02d}")
        pa, pb = p - R[:, 0] * G.PITCH / 2, p + R[:, 0] * G.PITCH / 2       # loop end, tail end
        a, b = Rf.T @ (pa - pf), Rf.T @ (pb - pf)
        if a[2] * b[2] < 0:
            t = a[2] / (a[2] - b[2])
            x = a + t * (b - a)
            if -G.FRAME_L + G.BAR_R < x[0] < -G.BAR_R and abs(x[1]) < G.FRAME_IN_X / 2:
                return i if b[2] > a[2] else None
    return None


def threaded(st):
    """The strap's free end has gone through the buckle frame (either way)."""
    Rf, pf = st.R("frame"), st.p("frame")
    for i in range(G.HOLE_LINK[5] - 4, G.N_LINKS):
        R, p = st.R(f"link_{i:02d}"), st.p(f"link_{i:02d}")
        a = Rf.T @ (p - R[:, 0] * G.PITCH / 2 - pf)
        b = Rf.T @ (p + R[:, 0] * G.PITCH / 2 - pf)
        if a[2] * b[2] < 0:
            t = a[2] / (a[2] - b[2])
            x = a + t * (b - a)
            if -G.FRAME_L + G.BAR_R < x[0] < -G.BAR_R and abs(x[1]) < G.FRAME_IN_X / 2:
                return True
    return False


def hole_in_frame(st, k):
    """Hole k's centre in the frame's coordinates (m): how the strap sits in the buckle."""
    i = G.HOLE_LINK[k]
    return st.R("frame").T @ (st.p(f"link_{i:02d}") - st.p("frame"))


def through_keeper(st):
    """Index of a tongue link (past the hole run's start) that passes through the keeper's opening (some point of
    its centre line within the keeper's length, width and the room under its bar), or None."""
    R, p = st.R(f"link_{G.KEEPER_LINK:02d}"), st.p(f"link_{G.KEEPER_LINK:02d}")
    for i in range(G.HOLE_LINK[5], G.N_LINKS):
        Ri, pi = st.R(f"link_{i:02d}"), st.p(f"link_{i:02d}")
        for s in np.linspace(-G.PITCH / 2, G.PITCH / 2, 11):
            l = R.T @ (pi + Ri[:, 0] * s - p)
            if (abs(l[0]) < G.KEEPER_LEN / 2 and abs(l[1]) < G.W / 2 + 0.001
                    and G.FOLD_TOP - 0.001 < l[2] < G.FOLD_TOP + G.KEEPER_GAP + 0.001):
                return i
    return None


def tip_past_keeper(st):
    """The tip has come out of the keeper on its far side (away from the buckle)."""
    R, p = st.R(f"link_{G.KEEPER_LINK:02d}"), st.p(f"link_{G.KEEPER_LINK:02d}")
    tip = st.p(f"link_{G.N_LINKS - 1:02d}") + st.R(f"link_{G.N_LINKS - 1:02d}") @ np.array([G.PITCH / 2, 0, 0])
    l = R.T @ (tip - p)
    return l[0] > G.KEEPER_LEN / 2


def in_loops(st):
    """For each belt loop, whether some link of the strap passes inside it."""
    out = {}
    for name, th in G.LOOPS.items():
        base, n = G.ellipse_pt(th, 0.0)
        c = np.array([G.X_B, *(base + n * G.LOOP_GAP / 2)])
        t = np.array([0.0, n[1], -n[0]])
        nn = np.array([0.0, n[0], n[1]])
        ok = False
        for i in range(G.N_LINKS):
            R, p = st.R(f"link_{i:02d}"), st.p(f"link_{i:02d}")
            for s in np.linspace(-G.PITCH / 2, G.PITCH / 2, 6):
                q = p + R[:, 0] * s - c
                if (abs(q @ t) < G.LOOP_W / 2 and abs(q @ nn) < G.LOOP_GAP / 2 + 0.001
                        and abs(q[0]) < G.LOOP_IN_X / 2):
                    ok = True
        out[name] = ok
    return out


def gap_to_trousers(p):
    """Signed distance (m) from a point to the trousers' surface at the waist (exact, in the y-z plane)."""
    y, z = p[1] - G.YC, p[2] - G.ZC
    ths = np.linspace(0, 2 * math.pi, 1440, endpoint=False)
    d = np.min(np.hypot(y - G.A1 * np.sin(ths), z - G.B1 * np.cos(ths)))
    inside = (y / G.A1) ** 2 + (z / G.B1) ** 2 < 1
    return float(-d if inside else d)
