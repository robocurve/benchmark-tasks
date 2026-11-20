"""Reading a three-strand plait (a braid of three hair sections) from their centre-lines, the way a person reads it
from a top photo.

A crossing is a point where two centre-lines cross seen from above, with the upper one at least MIN_DZ higher there
(10 mm sections: one lying over another sits ~10 mm higher; less is two sections squeezed side by side, not a
crossing). Crossings are read in order of distance from the start of the plait (the gathering elastic) along it,
toward the robot. Positions are counted from the robot's right: 0 = right, 1 = middle, 2 = left, starting from the
order at the start; at each crossing the two strands swap places. A crossing between the right and middle places is
+1 when the strand coming from the right lies over, -1 when it lies under; one between the middle and left places is
-2 when the strand coming from the left lies over, +2 when it lies under. A plait alternates +1 and -2 (outer over
middle, sides alternating, either side first). A strand laid over and straight back cancels (the two crossings are
removed). The plait count is the length of the alternating run from the start: it stops at the first crossing that
breaks the pattern (the same side twice, a strand passing under the middle, a strand laid across both others).
Pure NumPy.
"""
import itertools

import numpy as np

MIN_DZ = 0.004  # m
WINDOW = 0.015  # m: crossings this close along the plait may be read in either order; an adjacent pair goes first
PLAIT = (1, -2)


def _cross2(a, b):
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def crossings(lines, origin, axis):
    """lines: {name: (N, 3) centre-line points}. origin: the plait's start (xy); axis: unit xy direction of the plait."""
    origin, axis = np.asarray(origin, float)[:2], np.asarray(axis, float)[:2]
    out = []
    for a, b in itertools.combinations(sorted(lines), 2):
        P, Q = np.asarray(lines[a], float), np.asarray(lines[b], float)
        if not (np.isfinite(P).all() and np.isfinite(Q).all()):
            continue
        p0, r = P[:-1, :2], P[1:, :2] - P[:-1, :2]
        q0, s = Q[:-1, :2], Q[1:, :2] - Q[:-1, :2]
        den = _cross2(r[:, None, :], s[None, :, :])
        qp = q0[None, :, :] - p0[:, None, :]
        with np.errstate(divide='ignore', invalid='ignore'):
            t = _cross2(qp, s[None, :, :]) / den
            u = _cross2(qp, r[:, None, :]) / den
        mask = (np.abs(den) > 1e-12) & (t >= 0) & (t < 1) & (u >= 0) & (u < 1)
        for i, j in np.argwhere(mask):
            tt, uu = t[i, j], u[i, j]
            pt = p0[i] + tt * r[i]
            za = P[i, 2] + tt * (P[i + 1, 2] - P[i, 2])
            zb = Q[j, 2] + uu * (Q[j + 1, 2] - Q[j, 2])
            over, under = (a, b) if za > zb else (b, a)
            out.append({'over': over, 'under': under, 'xy': pt.tolist(), 'dz': float(abs(za - zb)),
                        'along': float(np.dot(pt - origin, axis)), 'arc': {a: float(i + tt), b: float(j + uu)}})
    out.sort(key=lambda c: c['along'])
    return out


def word(cross_list, order_at_start):
    """The plait's crossings as generators (+-1, +-2), in order from the start, and the crossings used."""
    order = list(order_at_start)
    left = list(cross_list)
    gens, used, skipped = [], [], 0
    while left:
        first = left[0]['along']
        pick = None
        for k, c in enumerate(left):
            if c['along'] > first + WINDOW:
                break
            if abs(order.index(c['over']) - order.index(c['under'])) == 1:
                pick = k
                break
        if pick is None:  # two strands that are not side by side: not readable as a crossing of neighbours
            skipped += 1
            left.pop(0)
            continue
        c = left.pop(pick)
        i = min(order.index(c['over']), order.index(c['under']))
        g = (i + 1) * (1 if order[i] == c['over'] else -1)
        order[i], order[i + 1] = order[i + 1], order[i]
        gens.append(g)
        used.append(c)
    return gens, used, skipped


def reduce(gens, used):
    """A crossing followed straight away by its undoing (a strand laid over and back) cancels."""
    gs, us = [], []
    for g, u in zip(gens, used):
        if gs and gs[-1] == -g:
            gs.pop()
            us.pop()
        else:
            gs.append(g)
            us.append(u)
    return gs, us


def plait_count(gs):
    """Length of the alternating right-over-middle / left-over-middle run from the start."""
    n, prev = 0, None
    for g in gs:
        if g not in PLAIT or g == prev:
            break
        n, prev = n + 1, g
    return n


def read(lines, origin, axis, order_at_start, min_dz=MIN_DZ):
    """{'count': plait count, 'word': reduced generators, 'along': each crossing's distance from the start (m),
    'dz': height of the upper strand over the lower at each (m), 'skipped': unreadable crossings, 'crossings': the
    crossings read, in order}."""
    cl = [c for c in crossings(lines, origin, axis) if c['dz'] >= min_dz]
    gens, used, skipped = word(cl, order_at_start)
    gs, us = reduce(gens, used)
    return {'count': plait_count(gs), 'word': gs, 'along': [u['along'] for u in us], 'dz': [u['dz'] for u in us],
            'skipped': skipped, 'crossings': us}


# ---------------------------------------------------------------- synthetic layouts for the offline tests
def example(schedule, names, origin=(0.55, 0.0), lane=0.012, step=0.03, rest_z=0.0055, over_z=0.0155, tail=0.15):
    """Centre-lines of three strands plaited by `schedule`, a list of (pair, outer_over) with pair 0 = right and
    middle places, 1 = middle and left; outer_over True lays the outer strand of the pair over the inner one.
    The strands run from `origin` along -x (toward the robot), `step` apart per crossing, then `tail` straight."""
    order = list(names)  # right, middle, left at the start
    ys = {0: -lane, 1: 0.0, 2: lane}
    pts = {n: [] for n in names}
    x = origin[0]
    for p, n in enumerate(order):
        pts[n].append((x, origin[1] + ys[p], rest_z))
    for pair, outer_over in schedule:
        x -= step / 2
        a, b = order[pair], order[pair + 1]  # a is on the right of b
        outer = a if pair == 0 else b
        upper = outer if outer_over else (b if outer is a else a)
        for p, n in enumerate(order):
            y = origin[1] + ys[p]
            if n in (a, b):
                y = origin[1] + (ys[pair] + ys[pair + 1]) / 2
            pts[n].append((x, y, over_z if n == upper else rest_z))
        x -= step / 2
        order[pair], order[pair + 1] = order[pair + 1], order[pair]
        for p, n in enumerate(order):
            pts[n].append((x, origin[1] + ys[p], rest_z))
    for p, n in enumerate(order):
        pts[n].append((x - tail, origin[1] + ys[p] * 4, rest_z))
    return {n: np.array(v) for n, v in pts.items()}


def plait_schedule(n, first_side=0):
    """Pairs for an n-crossing plait: outer over middle, sides alternating, starting on the right (0) or left (1)."""
    return [((first_side + k) % 2, True) for k in range(n)]
