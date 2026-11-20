"""Role-explicit full-arm guard and finite contact polygons for CPU planning."""
import numpy as np
from yam_arena import ik
from baseline_safety import convex_box_gap
from fast_safety import ScoopSafety

BASES = {'left': np.array([0., .305, .75]),
         'right': np.array([0., -.305, .75])}
GAP = .0002


def unique_axes(v):
    n = np.linalg.norm(v, axis=1)
    return np.unique(np.round(v[n > 1e-10] / n[n > 1e-10, None], 8), axis=0)


def convex_pair(a, b):
    """Same facet/edge SAT as Astra's existing exact mutual refinement."""
    ea = unique_axes(np.concatenate([a[:, 1]-a[:, 0], a[:, 2]-a[:, 1], a[:, 0]-a[:, 2]]))
    eb = unique_axes(np.concatenate([b[:, 1]-b[:, 0], b[:, 2]-b[:, 1], b[:, 0]-b[:, 2]]))
    axes = unique_axes(np.concatenate([
        np.cross(a[:, 1]-a[:, 0], a[:, 2]-a[:, 0]),
        np.cross(b[:, 1]-b[:, 0], b[:, 2]-b[:, 0]),
        np.cross(ea[:, None], eb[None, :]).reshape(-1, 3)]))
    best = -100.
    for start in range(0, len(axes), 256):
        ax = axes[start:start+256]
        pa, pb = a.reshape(-1, 3) @ ax.T, b.reshape(-1, 3) @ ax.T
        best = max(best, float(np.maximum(pa.min(0)-pb.max(0), pb.min(0)-pa.max(0)).max()))
        if best >= GAP:
            return best
    return best


def clip_polygon(poly, axis, bound, keep_less):
    if not len(poly):
        return np.empty((0, 3))
    output = []
    previous = poly[-1]
    pv = previous[axis]-bound
    inside = lambda value: value <= 1e-14 if keep_less else value >= -1e-14
    for point in poly:
        value = point[axis]-bound
        if inside(value) != inside(pv):
            output.append(previous + (point-previous) * (-pv)/(value-pv))
        if inside(value):
            output.append(point)
        previous, pv = point, value
    return np.asarray(output).reshape(-1, 3)


def union_area_in_rectangle(polygons, rect):
    """Exact piecewise-linear scanline union; vertices/edge crossings split x.

    Handles overlapping convex decomposition facets without adding their areas
    or filling holes by a convex hull. Boundary-only gaps have zero area.
    """
    xmin, xmax, ymin, ymax = rect
    polys, edges, xs = [], [], [xmin, xmax]
    for poly in polygons:
        p = np.column_stack([np.asarray(poly)[:, :2], np.zeros(len(poly))])
        for axis, value, less in [(0, xmin, False), (0, xmax, True),
                                  (1, ymin, False), (1, ymax, True)]:
            p = clip_polygon(p, axis, value, less)
        if len(p) < 3:
            continue
        polys.append(p[:, :2])
        xs.extend(p[:, 0])
        for a, b in zip(p, np.roll(p, -1, axis=0)):
            if abs(b[0]-a[0]) > 1e-14:
                slope = (b[1]-a[1])/(b[0]-a[0])
                edges.append((min(a[0], b[0]), max(a[0], b[0]), slope, a[1]-slope*a[0]))
    for i, (lo, hi, m, c) in enumerate(edges):
        for lo2, hi2, m2, c2 in edges[i+1:]:
            if abs(m-m2) > 1e-14:
                x = (c2-c)/(m-m2)
                if max(lo, lo2) < x < min(hi, hi2):
                    xs.append(x)
    xs = np.unique(xs)
    area = 0.
    for a, b in zip(xs[:-1], xs[1:]):
        x = (a+b)/2
        intervals = []
        for poly in polys:
            y = []
            for p, q in zip(poly, np.roll(poly, -1, axis=0)):
                if min(p[0], q[0]) < x < max(p[0], q[0]):
                    y.append(p[1]+(q[1]-p[1])*(x-p[0])/(q[0]-p[0]))
            if len(y) >= 2:
                intervals.append((min(y), max(y)))
        end, length = -np.inf, 0.
        for lo, hi in sorted(intervals):
            length += max(0., hi-max(lo, end))
            end = max(end, hi)
        area += (b-a)*length
    return float(area)


class Geometry:
    def __init__(self):
        self.model = ScoopSafety()
        self.names = self.model.names
        p, R = ik.tcp_pose(np.zeros(6))
        world = self.model.world(np.zeros(6), [-.003, -.003], np.zeros(3))
        self.hand = [(i, n, (tri-p) @ R) for i, (n, tri) in enumerate(world)
                     if n in ('gripper', 'tip_left', 'tip_right')]
        self.contact_cache = {}

    def parts(self, side, q, fingers):
        return self.model.world(q, fingers, BASES[side])

    def guard(self, side, q, fingers, obstacles, other, *, intended=False,
              intended_targets=('north_bottom',)):
        """All315 pieces, all supplied OBBs/table; opposite arm never deleted."""
        pieces = self.parts(side, q, fingers)
        vertices = np.concatenate([t.reshape(-1, 3) for _, t in pieces])
        obs = [(n, p, R, size) for n, (p, R, size) in obstacles.items()]
        gaps = self.model._pair_gaps(pieces, vertices, obs)
        mask = np.zeros_like(gaps, dtype=bool)
        if intended:
            for column,(target,_,_,_) in enumerate(obs,1):
                if target in intended_targets:
                    mask[:, column] = [n in ('tip_left', 'tip_right') for n in self.names]
        hard = self.model._minimum(gaps, ~mask, pieces, obs)
        target = self.model._minimum(gaps, mask, pieces, obs) if intended else (10., None, None)
        a = np.minimum.reduceat(vertices, self.model.starts, axis=0)
        b = np.maximum.reduceat(vertices, self.model.starts, axis=0)
        ov = np.concatenate([t.reshape(-1, 3) for _, t in other])
        c = np.minimum.reduceat(ov, self.model.starts, axis=0)
        d = np.maximum.reduceat(ov, self.model.starts, axis=0)
        pairs = np.max(np.maximum(a[:, None]-d[None], c[None]-b[:, None]), axis=2)
        i, j = np.unravel_index(pairs.argmin(), pairs.shape)
        mutual = float(pairs[i, j])
        pair_id = [int(i), int(j), pieces[i][0], other[j][0]]
        refinements = []
        for i, j in sorted(np.argwhere(pairs < GAP), key=lambda ij: pairs[tuple(ij)]):
            g = convex_pair(pieces[i][1], other[j][1])
            refinements.append([int(i), int(j), pieces[i][0], other[j][0], g])
            if g < GAP:
                mutual, pair_id = g, [int(i), int(j), pieces[i][0], other[j][0]]
                break
        else:
            if refinements:
                mutual = min(GAP, min(x[-1] for x in refinements))
                pair_id = ['all_unseparated_AABB_pairs_refined', len(refinements)]
        return dict(safe=bool(hard[0] >= GAP and mutual >= GAP),
                    hard_gap_m=float(hard[0]), hard_pair=list(hard[1:]),
                    target_gap_m=float(target[0]), target_pair=list(target[1:]),
                    other_arm_gap_m=mutual, other_arm_pair=pair_id,
                    exact_mutual_refinements=refinements)

    def local_hand(self, openings):
        result = []
        for i, n, tri in self.hand:
            t = tri.copy()
            if n == 'tip_left':
                t[:, :, 0] += .003-openings[0]
            elif n == 'tip_right':
                t[:, :, 0] -= .003-openings[1]
            result.append((i, n, t))
        return result

    def cheap_hand(self, tcp, R, obstacles, openings=(.003, .003),
                   intended_targets=('north_bottom',)):
        from config import TOP
        hard = (10., None, None)
        targets = {'tip_left': 10., 'tip_right': 10.}
        for i, n, tri in self.local_hand(openings):
            t = tri @ R.T+tcp
            g = float(t[:, :, 2].min()-TOP)
            if g < hard[0]:
                hard = (g, n, 'table')
            for name, (p, rot, size) in obstacles.items():
                g = convex_box_gap(t, p, rot, size)
                if name in intended_targets and n in targets:
                    targets[n] = min(targets[n], g)
                elif g < hard[0]:
                    hard = (g, n, name)
        return dict(safe=bool(hard[0] >= GAP and min(targets.values()) >= -1e-5),
                    hard_gap=list(hard), finger_target_gaps=targets)
