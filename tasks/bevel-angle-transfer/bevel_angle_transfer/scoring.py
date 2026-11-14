# SPDX-License-Identifier: MIT
"""Grade the observable bevel transfer and the resulting visible line."""
from __future__ import annotations

import math

import numpy as np

from .contact import (
    DEFAULT_CONFIG, angle_difference, blade_recontacted, blade_regrasped, blade_released, board_placement,
    capture_contacts, contact_gate, gauge_ground_clearance, guide_metrics, lift_gate, pose, relative_angle,
    stock_at_fixture, stock_carried, stock_ground_clearance, target_endpoints,
)

_EPS = 1e-10


def _intervals(intervals):
    """Sorted union, with numerical tolerance only (never fill physical gaps)."""
    out = []
    for lo, hi in sorted(intervals):
        if hi - lo <= _EPS:
            continue
        if out and lo <= out[-1][1] + _EPS:
            out[-1][1] = max(out[-1][1], hi)
        else:
            out.append([float(lo), float(hi)])
    return out


def _length(intervals):
    return sum(hi - lo for lo, hi in _intervals(intervals))


def _unique_segments(segments):
    """Union coincident line segments; quantization is sub-micrometre numerical QA."""
    groups = {}
    for a, b in segments:
        d = b - a
        norm = float(np.linalg.norm(d))
        if norm <= _EPS:
            continue
        d /= norm
        if d[0] < -_EPS or (abs(d[0]) <= _EPS and d[1] < 0):
            d = -d
        normal = np.array([-d[1], d[0]])
        offset = float(a @ normal)
        key = tuple(np.round(d, 9)) + (round(offset, 10),)
        if key not in groups:
            groups[key] = (d, normal * offset, [])
        groups[key][2].append(sorted((float(a @ d), float(b @ d))))
    out = []
    for direction, origin, intervals in groups.values():
        out.extend((origin + lo*direction, origin + hi*direction)
                   for lo, hi in _intervals(intervals))
    return out


def _visible_ink(segments, length, radius):
    """Raster union of filled 2 mm ink capsules, never stroke-to-stroke bridges.

    The target-aligned grading image spans x=[-60 mm, L+60 mm], y=+/-200 mm.
    Ink outside this explicitly reported region fails the bounded-board check.
    The 140 mm task uses 1664000 pixels; at most 2000000 cells are allocated.
    """
    resolution = 0.00025
    lower = np.array([-0.06, -0.20])
    upper = np.array([length+0.06, 0.20])
    resolution = max(resolution, float(np.sqrt(np.prod(upper-lower)/1990000)))
    shape = np.ceil((upper-lower)/resolution).astype(int)
    occupied = np.zeros((int(shape[1]), int(shape[0])), dtype=bool)
    outside = False
    for a, b in segments:
        if np.any(a < lower+radius) or np.any(a > upper-radius) or np.any(b < lower+radius) or np.any(b > upper-radius):
            outside = True
        lo = np.maximum(0, np.floor((np.minimum(a, b)-radius-lower)/resolution).astype(int))
        hi = np.minimum(shape, np.ceil((np.maximum(a, b)+radius-lower)/resolution).astype(int))
        if np.any(hi <= lo):
            continue
        xs = lower[0] + (np.arange(lo[0], hi[0])+.5)*resolution
        ys = lower[1] + (np.arange(lo[1], hi[1])+.5)*resolution
        x, y = np.meshgrid(xs, ys)
        delta = b-a
        norm2 = float(delta @ delta)
        if norm2 > _EPS**2:
            t = np.clip(((x-a[0])*delta[0]+(y-a[1])*delta[1])/norm2, 0, 1)
            distance2 = (x-a[0]-t*delta[0])**2 + (y-a[1]-t*delta[1])**2
        else:
            distance2 = (x-a[0])**2+(y-a[1])**2
        occupied[lo[1]:hi[1], lo[0]:hi[0]] |= distance2 <= radius**2 + 1e-16
    return occupied, outside, resolution


def _separate_marks(occupied, resolution):
    """Distinguish overlapping or offset components of the visible ink."""
    columns = np.flatnonzero(np.any(occupied, axis=0))
    if not len(columns):
        return False
    parent = []
    spans = []
    previous = []
    previous_column = None

    def component(index):
        while index != parent[index]:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    for x in columns:
        rows = np.flatnonzero(occupied[:, x])
        runs = np.split(rows, np.flatnonzero(np.diff(rows) > 1)+1)
        current = []
        for run in runs:
            lo, hi = int(run[0]), int(run[-1])
            index = len(parent)
            parent.append(index)
            spans.append((int(x), lo, hi))
            if previous_column == x-1:
                for prior_lo, prior_hi, prior_index in previous:
                    if prior_lo <= hi+1 and lo <= prior_hi+1:
                        parent[component(prior_index)] = component(index)
            current.append((lo, hi, index))
        previous, previous_column = current, x

    marks = {}
    for index, (column, lo, hi) in enumerate(spans):
        mark = marks.setdefault(component(index), {})
        if column in mark:
            old_lo, old_hi = mark[column]
            lo, hi = min(lo, old_lo), max(hi, old_hi)
        mark[column] = (lo, hi)
    ordered = sorted(marks.values(), key=min)
    separate = False
    for left, right in zip(ordered[:-1], ordered[1:]):
        last, first = max(left), min(right)
        left_lo, left_hi = left[last]
        right_lo, right_hi = right[first]
        join_width = (max(left_hi, right_hi)-min(left_lo, right_lo)+1)*resolution
        if first <= last or join_width > .004+_EPS:
            separate = True
            break
    return separate


def _max_cross_section_width(segments, diameter):
    """Find the largest outer ink span on a target-normal cross-section.

    Two ink points at the same longitudinal station differ by a vector on x=0.
    The difference of their two capsules is the difference of the two straight
    segments, expanded by a disk of the ink diameter. Intersect that expanded
    parallelogram with x=0 using its corner disks and offset edges.
    """
    if not segments:
        return 0.
    pairs = np.asarray(segments)
    first = np.min(pairs[:, :, 0], axis=1)
    last = np.max(pairs[:, :, 0], axis=1)
    order = np.argsort(first)
    pairs, first, last = pairs[order], first[order], last[order]
    maximum = 0.
    for index, pair in enumerate(pairs):
        stop = np.searchsorted(first, last[index]+diameter+_EPS, side="right")
        candidates = pairs[index:stop]
        corners = (pair[None, :, None, :]-candidates[:, None, :, :]).reshape(-1, 4, 2)
        x, y = corners[..., 0], corners[..., 1]
        disks = np.where(np.abs(x) <= diameter+_EPS,
                         np.abs(y)+np.sqrt(np.maximum(0., diameter**2-x**2)), 0.)
        maximum = max(maximum, float(np.max(disks)))
        a, b = corners[:, [0, 0, 1, 2]], corners[:, [1, 2, 3, 3]]
        edge = b-a
        dx, dy = edge[..., 0], edge[..., 1]
        norms = np.maximum(np.linalg.norm(edge, axis=-1), _EPS)
        for sign in (-1, 1):
            shift_x = -sign*diameter*dy/norms
            shift_y = sign*diameter*dx/norms
            t = np.divide(-a[..., 0]-shift_x, dx, where=np.abs(dx) > _EPS,
                          out=np.full_like(dx, np.inf))
            valid = (t >= -_EPS) & (t <= 1.+_EPS)
            y_at_zero = a[..., 1]+np.where(valid, t, 0.)*dy+shift_y
            maximum = max(maximum, float(np.max(np.where(valid, np.abs(y_at_zero), 0.))))
    return maximum


def _point_segment_distance(point, a, b):
    delta = b-a
    norm2 = float(delta @ delta)
    if norm2 <= _EPS**2:
        return float(np.linalg.norm(point-a))
    t = np.clip((point-a) @ delta / norm2, 0., 1.)
    return float(np.linalg.norm(point-(a+t*delta)))


def score_strokes(strokes, target_start=(-.06, 0.), target_end=(.06, 0.), *,
                  tick=None, board_width=None, ink_width=.002):
    """Measure the visible mark against a fixed template, ruler and printed tick.

    With a board width, coverage and gaps project onto the board's x axis.
    Otherwise the target axis is used as the width axis for standalone measurements.
    """
    start, end = np.asarray(target_start, float), np.asarray(target_end, float)
    if start.shape != (2,) or end.shape != (2,) or not np.isfinite([start, end]).all():
        raise ValueError("Target endpoints must be finite 2D points")
    length = float(np.linalg.norm(end-start))
    if length <= _EPS:
        raise ValueError("Target endpoints must differ")
    direction = (end-start)/length
    frame = np.column_stack((direction, [-direction[1], direction[0]]))
    tick = start if tick is None else np.asarray(tick, float)
    if tick.shape != (2,) or not np.isfinite(tick).all():
        raise ValueError("Tick must be a finite 2D point")
    tick = (tick-start) @ frame
    width = length if board_width is None else float(board_width)
    if not math.isfinite(width) or width <= 0 or not math.isfinite(ink_width) or ink_width <= 0:
        raise ValueError("Board width and ink width must be positive and finite")
    axis = np.array([1., 0.]) if board_width is None else frame[0] * np.sign(end[0]-start[0])
    radius = ink_width/2
    runs, invalid = [], 0
    for stroke in strokes:
        points = np.asarray(stroke, dtype=float)
        if points.size == 0:
            continue
        if points.ndim != 2 or points.shape[1] != 2:
            raise ValueError("Every stroke must have shape (N, 2)")
        run = []
        for point in points:
            if not np.isfinite(point).all():
                invalid += 1
                if run:
                    runs.append(np.asarray(run))
                    run = []
                continue
            projected = (point-start) @ frame
            if not run or np.linalg.norm(projected-run[-1]) > _EPS:
                run.append(projected)
        if run:
            runs.append(np.asarray(run))
    segments = _unique_segments([(a, b) for run in runs for a, b in zip(run[:-1], run[1:])])
    segments.extend((run[0], run[0]) for run in runs if len(run) == 1)
    intervals = []
    band_error = 0.
    tick_distance = math.inf
    for a, b in segments:
        lo, hi = sorted((float(a @ axis), float(b @ axis)))
        intervals.append((max(0., lo-radius), min(width, hi+radius)))
        band_error = max(band_error, abs(a[1])+radius, abs(b[1])+radius)
        tick_distance = min(tick_distance, max(0., _point_segment_distance(tick, a, b)-radius))
    intervals = _intervals(intervals)
    covered = _length(intervals)
    gap = max((right[0]-left[1] for left, right in zip(intervals[:-1], intervals[1:])), default=0.)
    raster, outside, resolution = _visible_ink(segments, length, radius)
    line_width = _max_cross_section_width(segments, ink_width)
    separate = _separate_marks(raster, resolution)
    # A line beyond the board ends cannot count toward width or create a second mark.
    outside = bool(outside or any(min(a @ axis, b @ axis) < -.001-_EPS
                                  or max(a @ axis, b @ axis) > width+.001+_EPS
                                  for a, b in segments))
    score = 0.
    if not outside and not separate and line_width <= .004+_EPS:
        for fraction, tolerance, reward in ((.5, .005, .5), (.9, .003, 1.)):
            if (covered+_EPS >= fraction*width and tick_distance <= tolerance+_EPS
                    and band_error <= tolerance+_EPS and gap <= tolerance+_EPS):
                score = reward
    return dict(score=score, coverage=covered/width, marked_coverage=covered,
                max_gap=gap, tick_distance=tick_distance if math.isfinite(tick_distance) else None,
                max_template_offset=float(band_error), max_line_width=line_width, extra_marks=separate,
                target_length=length, board_width=width, invalid_points=invalid,
                ink_outside_grading_region=outside, ink_diameter=ink_width,
                raster_resolution=resolution, raster_cells=int(raster.size), stroke_count=len(runs))


class TransferTracker:
    """Grade the observed reference fit, carry, board placement and marking operation."""

    def __init__(self, spec=DEFAULT_CONFIG):
        from types import SimpleNamespace
        from .contact import config_dict

        self.config = SimpleNamespace(**config_dict(spec))
        self.target_start, self.target_end = target_endpoints(self.config)
        target = np.asarray(self.target_end)-self.target_start
        direction = target/np.linalg.norm(target)
        self._target_frame = np.column_stack((direction, [-direction[1], direction[0]]))
        self._target_tick = (np.asarray(self.config.TICK_CENTER)-np.asarray(self.config.BOARD_CENTER[:2])).tolist()
        self.strokes = []
        self.tick = 0
        self._stroke = None
        self._stroke_guided = False
        self._unguided_bounds = None
        self._capture_angle = self._capture_origin = None
        self._release_origin = None
        self._closed_at_fixture = False
        self._placement_angle = None
        self._release = self._transfer = False
        self._airborne = self._transported = False
        self._angle = self._adjustment = None
        self._initial_angle = None
        self._invalidations = self._captures = 0
        self._nib_count = 0
        self._guidance_failed = False
        self._marker = self._board = None
        self._geometry_cache = None
        self._geometry_dirty = False
        self._cache_tick = 0

    def _invalidate(self):
        if self._capture_angle is not None:
            self._invalidations += 1
        self._capture_angle = self._capture_origin = None
        self._release_origin = None
        self._closed_at_fixture = False
        self._release = self._transfer = self._airborne = self._transported = False

    def _finish_stroke(self):
        if self._stroke is not None and not self._stroke_guided:
            self._guidance_failed = True
        self._stroke = None
        self._stroke_guided = False
        self._unguided_bounds = None

    def _observe_guidance(self, point, guide_used):
        if not self._transfer:
            self._guidance_failed = True
        if guide_used:
            self._stroke_guided = True
            self._unguided_bounds = None
            return
        # Resolve continuous freehand movement at one visible ink width.
        projected = np.asarray(point) @ self._target_frame
        if self._unguided_bounds is None:
            self._unguided_bounds = projected, projected
        low, high = self._unguided_bounds
        low, high = np.minimum(low, projected), np.maximum(high, projected)
        self._unguided_bounds = low, high
        if np.max(high-low) >= self.config.INK_WIDTH-_EPS:
            self._guidance_failed = True

    def _advance_transfer(self, record, frames):
        config = self.config
        stock, blade, board, reference_stock, reference_blade = frames
        self._angle = relative_angle(stock, blade, config)
        if self._initial_angle is None and self._angle is not None:
            self._initial_angle = self._angle
        self._adjustment = (angle_difference(self._angle, self._initial_angle)
                            if self._angle is not None else None)
        if self._angle is None:
            self._invalidate()
            return
        if self._transfer:
            if blade_recontacted(record, blade, config):
                self._invalidate()
            return

        if not self._release:
            if not stock_at_fixture(stock, reference_stock, config):
                self._invalidate()
                return
            fitting = capture_contacts(record, stock, blade, reference_stock, reference_blade, config)
            grounded = gauge_ground_clearance(stock, blade, config) <= .001
            if not grounded and not fitting:
                self._invalidate()
                return
            if blade_regrasped(record, blade, config):
                self._closed_at_fixture = True
            # Record the fit before opening the hand. The stock locates the
            # gauge at the fixture while the hand withdraws from the blade.
            if record["right_gripper_open"] is False and fitting:
                if self._capture_angle is None:
                    self._captures += 1
                self._capture_angle = self._angle
                self._capture_origin = stock[0].copy()
            self._release = (grounded and self._capture_angle is not None and self._closed_at_fixture
                             and blade_released(record, blade, config))
            if self._release:
                self._release_origin = stock[0].copy()
            return

        if not stock_carried(record, stock, config) or blade_recontacted(record, blade, config):
            self._invalidate()
            return
        airborne = gauge_ground_clearance(stock, blade, config) > .001
        if airborne:
            self._airborne = True
            clearance = stock_ground_clearance(stock, config)
            stock_at_table = (stock[0]-[0., 0., clearance], stock[1])
            self._transported |= not stock_at_fixture(stock_at_table, reference_stock, config)
        if self._transported and board_placement(stock, blade, board, config)[0]:
            self._placement_angle = self._angle
            if angle_difference(self._angle, self._capture_angle) <= config.RETENTION_ANGLE_DEG+_EPS:
                self._transfer = True
            else:
                self._invalidate()
            return
        if not airborne and (self._airborne
                             or np.linalg.norm((stock[0]-self._release_origin)[:2]) > config.FIT_GAP+_EPS):
            self._invalidate()

    def sample(self, raw):
        """Add a consecutive physics observation and report observable evidence."""
        config = self.config
        expected = self.tick+1
        dt = float(raw["dt"])
        if (type(raw["tick"]) is not int or raw["tick"] != expected
                or not math.isfinite(dt) or not math.isclose(dt, config.PHYSICS_DT, abs_tol=1e-10, rel_tol=1e-8)):
            raise ValueError(f"Sample {expected}: expected consecutive ticks and {config.PHYSICS_DT} s physics dt")
        self.tick = expected
        self._marker, self._board = raw["marker_pose"], raw["board_pose"]
        try:
            for side in ("left", "right"):
                fingers = np.asarray(raw.get(f"{side}_finger_poses"), dtype=float)
                opening = raw.get(f"{side}_gripper_opening")
                opened = raw.get(f"{side}_gripper_open")
                if (type(opening) not in (int, float) or not 0. <= opening <= 1.
                        or opened is not (opening >= .75) or fingers.shape != (2, 7)):
                    raise ValueError("Invalid observed hand geometry")
                for finger in fingers:
                    pose(finger)
            frames = tuple(pose(raw[name]) for name in
                           ("stock_pose", "blade_pose", "board_pose", "reference_stock_pose", "reference_blade_pose"))
        except (TypeError, ValueError):
            frames = None
        if frames is None:
            self._invalidate()
            self._angle = self._adjustment = None
        else:
            self._advance_transfer(raw, frames)
        accepted, details = contact_gate(
            raw.get("pair_force", [None]*3), raw["pair_point"], self._marker, self._board,
            nib_local=config.MARKER_NIB_LOCAL, nib_radius=config.MARKER_NIB_RADIUS,
            board_size=config.BOARD_SIZE)
        point = details.get("point_board")
        point_score = point[:2] if accepted else None
        segment_start = guide_used = False
        guide = {"guide_geometry": False}
        if accepted:
            self._nib_count += 1
            if frames is not None and self._transfer:
                guide = guide_metrics(raw["pair_point"], *frames[:3], None, None, config)
                guide_used = guide["guide_geometry"]
            segment_start = self._stroke is None or np.linalg.norm(np.asarray(point_score)-self._stroke[-1]) > .01
            if segment_start:
                self._finish_stroke()
                self._stroke = []
                self.strokes.append(self._stroke)
            self._stroke.append(point_score)
            self._observe_guidance(point_score, guide_used)
            self._geometry_dirty = True
        else:
            self._finish_stroke()
        return dict(raw, **details, **guide, ink=accepted, point_score=point_score,
                    ink_segment_start=bool(segment_start), guide_used=bool(guide_used),
                    gauge_pose_angle_deg=self._angle, adjustment_deg=self._adjustment,
                    capture_valid=self._capture_angle is not None, released=self._release,
                    transfer_valid=self._transfer)

    def metrics(self, *, force=True):
        """Report final measurements or limit raster refreshes in live previews."""
        config = self.config
        if self._geometry_cache is None or (self._geometry_dirty and
                (force or self.tick-self._cache_tick >= round(.5/config.PHYSICS_DT))):
            self._geometry_cache = score_strokes(
                self.strokes, self.target_start, self.target_end, tick=self._target_tick,
                board_width=config.BOARD_SIZE[0], ink_width=config.INK_WIDTH)
            self._geometry_dirty = False
            self._cache_tick = self.tick
        geometry = self._geometry_cache
        lifted, lift = False, {"nib_clearance": None}
        if self.tick:
            lifted, details = lift_gate(self._marker, self._board, 0.,
                                        nib_local=config.MARKER_NIB_LOCAL, nib_radius=config.MARKER_NIB_RADIUS,
                                        board_size=config.BOARD_SIZE)
            lift = {"nib_clearance": details["nib_clearance"]}
        marker_guidance = (self._nib_count > 0 and not self._guidance_failed
                           and (self._stroke is None or self._stroke_guided))
        score = 0.
        if self._transfer:
            score = .25
            if marker_guidance and geometry["score"] >= .5:
                score = 1. if geometry["score"] == 1. and lifted else .5
        return dict(geometry, **lift, score=score, success=score == 1.,
                    geometry_score=geometry["score"], marker_guidance_valid=marker_guidance,
                    physics_samples=self.tick,
                    geometry_evaluated_tick=self.tick if force or not self._geometry_dirty else self._cache_tick,
                    nib_contact_samples=self._nib_count,
                    marker_lift_complete=lifted, gauge_pose_angle_deg=self._angle, adjustment_deg=self._adjustment,
                    initial_measured_angle_deg=self._initial_angle,
                    capture_valid=self._capture_angle is not None, captured_angle_deg=self._capture_angle,
                    capture_count=self._captures, retention_invalidations=self._invalidations,
                    released=self._release, placed_angle_deg=self._placement_angle,
                    transfer_valid=self._transfer)
