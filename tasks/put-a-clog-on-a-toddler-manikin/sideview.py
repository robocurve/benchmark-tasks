"""Side view of the foot going into the clog: a planar model used to plan the demo, not by the task or its scoring.

Clog frame: x back from the inside toe end, z up from the footbed. The lower leg turns about the knee by the knee's bend
(toes down as the knee bends back); the clog stays level and only translates. ``clear`` says whether the leg's outline
is clear of the clog's vamp, toe wall, footbed, heel rim and strap crossbar by a margin; ``schedule`` finds, for each knee
angle, how far forward the toes can be. The demo's ``TOES_AT`` table is this schedule at a 2 mm margin.

    python sideview.py            # prints the schedule
"""

from __future__ import annotations

import math

import numpy as np

import clogkit as K

BAR = np.array([K.PIVOT[0] + K.STRAP_R * math.cos(K.STRAP_REST), K.RIM + K.ROD])     # the crossbar's middle, on the rim


def leg_outline() -> np.ndarray:
    """The lower leg and foot from the side, in the leg's frame turned so that x runs back (as the clog's does)."""
    s = K.SOLE
    top = [(sec[0], sec[3]) for sec in K.FOOT_SECTIONS]
    pts = [(K.HEEL_BACK - 0.004, s), (K.HEEL_BACK, s + 0.008), (K.HEEL_BACK, s + 0.030), (0.020, s + 0.060),
           (0.028, -0.060), (0.031, 0.0), (-0.031, 0.0), (-0.027, -0.060), (-0.0195, s + 0.060)]
    pts += [(K.HEEL_BACK - d, s + h) for d, h in top[3:]]
    pts += [(K.TOE_X + 0.006, s)]
    return np.array(pts)


LEG = leg_outline()


def pose(deg: float, knee) -> np.ndarray:
    """The outline for a knee bent back ``deg``, the knee at ``knee`` (x, z) in the clog's frame."""
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return LEG @ np.array([[c, -s], [s, c]]).T + np.asarray(knee, float)


def _edge(poly: np.ndarray, step: float = 0.0015) -> np.ndarray:
    out = []
    for a, b in zip(poly, np.roll(poly, -1, 0)):
        n = max(1, int(np.linalg.norm(b - a) / step))
        out.append(a + (b - a) * np.linspace(0, 1, n, endpoint=False)[:, None])
    return np.vstack(out)


def _inside(poly: np.ndarray, pts: np.ndarray) -> np.ndarray:
    x, y = pts[:, 0][:, None], pts[:, 1][:, None]
    x0, y0 = poly[:, 0][None], poly[:, 1][None]
    x1, y1 = np.roll(poly[:, 0], -1)[None], np.roll(poly[:, 1], -1)[None]
    cross = ((y0 > y) != (y1 > y)) & (x < (x1 - x0) * (y - y0) / (y1 - y0 + 1e-12) + x0)
    return cross.sum(1) % 2 == 1


def _roof(x):
    return K.ROOF_TOE + (K.ROOF_EDGE - K.ROOF_TOE) * np.clip((x - 0.010) / (K.VAMP - 0.010), 0.0, 1.0)


def _clog_points() -> np.ndarray:
    xs = np.arange(0.0, K.VAMP + 1e-9, 0.002)
    t = np.linspace(0, 2 * math.pi, 16, endpoint=False)
    return np.vstack([np.column_stack([xs, _roof(xs)]),
                      np.column_stack([np.zeros(15), np.linspace(0, K.ROOF_TOE, 15)]),
                      np.column_stack([np.arange(0, K.LIN + K.WALL, 0.002), np.full(77, -0.0005)]),
                      BAR + K.ROD * np.column_stack([np.cos(t), np.sin(t)]),
                      np.column_stack([np.full(8, K.LIN), np.linspace(0, K.RIM, 8)])])


CLOG = _clog_points()


def hits(poly: np.ndarray) -> bool:
    b = _edge(poly)
    x, z = b[:, 0], b[:, 1]
    bad = (z < 0) & (x > -0.01) & (x < K.LIN + K.WALL)
    bad |= (x < 0) & (x > -0.02) & (z < K.ROOF_TOE + K.WALL)
    bad |= (x >= 0) & (x <= K.VAMP) & (z > _roof(x)) & (z < _roof(x) + K.WALL)
    bad |= np.hypot(x - BAR[0], z - BAR[1]) < K.ROD
    bad |= (x > K.LIN) & (x < K.LIN + K.WALL) & (z < K.RIM)
    return bool(bad.any() or _inside(poly, CLOG).any())


def knee_for(deg: float, toes: float, lift: float) -> np.ndarray:
    """Where the knee is (clog frame) with the tip of the toes ``toes`` back from the toe end and the foot's lowest
    point ``lift`` above the footbed."""
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    r = np.array([[c, -s], [s, c]])
    tip = r @ np.array([K.TOE_X, K.SOLE])
    return np.array([toes - tip[0], lift - (LEG @ r.T)[:, 1].min()])


def clear(deg: float, toes: float, lift: float, margin: float = 0.002) -> bool:
    k = knee_for(deg, toes, lift)
    return all(not hits(pose(deg, k + np.array(d))) for d in ((0, 0), (margin, 0), (-margin, 0), (0, margin), (0, -margin)))


def strap_gap(deg: float, toes: float, lift: float) -> float:
    """The least distance from the leg's outline to the strap's crossbar."""
    return float(np.hypot(*(_edge(pose(deg, knee_for(deg, toes, lift))) - BAR).T).min() - K.ROD)


def schedule(margin: float = 0.002, lift: float = 0.003) -> list:
    """For each knee angle from 30 degrees down: the furthest forward the toes can be, and the gap to the strap."""
    out = []
    for deg in range(30, -1, -2):
        for toes in np.arange(0.004, 0.160, 0.002):
            if clear(deg, toes, lift, margin):
                out.append((deg, round(float(toes), 3), round(strap_gap(deg, toes, lift), 4)))
                break
    return out


if __name__ == "__main__":
    for deg, toes, gap in schedule():
        print(f"knee {deg:2d} deg: toes {1000 * toes:5.1f} mm from the toe end, {1000 * gap:5.1f} mm to the strap")
