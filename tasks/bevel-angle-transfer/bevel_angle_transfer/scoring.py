# SPDX-License-Identifier: MIT
"""Geometry-only scoring of a bevel-guided marker line (all distances in metres).

Each input stroke is a continuous polyline; separate strokes and nonfinite points
never create connecting ink. Samples define ink capsules and contact stamps. Exact
collinear retraces are merged for numerical efficiency. Visible-ink metrics use
union occupancy of 2 mm diameter stroke capsules on a 0.25 mm raster, making
microjitter and overlapping retraces behave like photographed ink. No robot state
or timestamps enter grading; the specified reference endpoints define the target.
Coordinates are board-local, not world coordinates.
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np

from .contact import (
    DEFAULT_CONFIG, advance_lift_hold, angle_difference, blade_released, capture_contacts,
    contact_gate, guide_metrics, lift_gate, pose, relative_angle, target_endpoints,
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


def _visible_ink(segments, isolated, length):
    """Raster union of filled 2 mm ink capsules, never stroke-to-stroke bridges.

    The target-aligned grading image spans x=[-60 mm, L+60 mm], y=+/-200 mm.
    Ink outside this explicitly reported region fails the bounded-board check.
    The 140 mm task uses 1664000 pixels; at most 2000000 cells are allocated.
    """
    radius, resolution = 0.001, 0.00025
    lower = np.array([-0.06, -0.20])
    upper = np.array([length+0.06, 0.20])
    resolution = max(resolution, float(np.sqrt(np.prod(upper-lower)/1990000)))
    shape = np.ceil((upper-lower)/resolution).astype(int)
    occupied = np.zeros((int(shape[1]), int(shape[0])), dtype=bool)
    outside = False
    for a, b in [*segments, *((p, p) for p in isolated)]:
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
    # A calibrated photograph supplies this same binary ink mask. Project ink
    # inside each tolerance band onto the reference axis; no unpainted columns
    # are filled between disconnected strokes. Nib end caps are actual ink.
    metrics = {}
    ys = lower[1]+(np.arange(occupied.shape[0])+.5)*resolution
    for label, band in (('full', .003), ('partial', .005)):
        columns = np.flatnonzero(np.any(occupied[np.abs(ys) <= band], axis=0))
        intervals = []
        if len(columns):
            groups = np.split(columns, np.flatnonzero(np.diff(columns) > 1)+1)
            for group in groups:
                lo = max(0.0, lower[0]+float(group[0])*resolution)
                hi = min(length, lower[0]+float(group[-1]+1)*resolution)
                if hi > lo:
                    intervals.append([lo, hi])
        covered = sum(hi-lo for lo, hi in intervals)
        metrics[f'{label}_coverage'] = covered/length
        metrics[f'{label}_continuous_coverage'] = max((hi-lo for lo, hi in intervals), default=0.0)/length
        metrics[f'{label}_max_gap'] = max((b[0]-a[1] for a, b in zip(intervals[:-1], intervals[1:])), default=0.0)
    rows, cols = np.nonzero(occupied)
    points = np.column_stack((lower[0]+(cols+.5)*resolution,
                              lower[1]+(rows+.5)*resolution))
    pixel_area = resolution**2
    # Area/diameter has length units. Remove a single pair of circular end caps;
    # additional dots/branches remain visible extra ink rather than time penalties.
    ink_length = max(0.0, len(points)*pixel_area-np.pi*radius**2)/(2*radius)
    angle, width, stray = 90.0, 0.0, 0.0
    if len(points):
        in_corridor = ((points[:, 0] >= -radius) & (points[:, 0] <= length+radius)
                       & (np.abs(points[:, 1]) <= 0.005+radius))
        stray = float(np.count_nonzero(~in_corridor)*pixel_area/(2*radius))
        # Orientation and spread describe the candidate cut line. Off-target
        # marks are assessed separately by the explicit stray-ink allowance.
        line_points = points[in_corridor]
        if len(line_points) >= 2:
            covariance = np.cov(line_points, rowvar=False, bias=True)
            values, vectors = np.linalg.eigh(covariance)
            principal = vectors[:, -1]
            angle = float(np.degrees(np.arctan2(abs(principal[1]), abs(principal[0]))))
            # Subtract intrinsic variance of a uniform 2 mm-wide straight stroke.
            width = float(np.sqrt(max(0.0, values[0]-radius**2/3)))
    return ink_length, stray, angle, width, outside, resolution, occupied.size, metrics


def score_strokes(strokes, target_start=(-0.06, 0.0), target_end=(0.06, 0.0)):
    """Return a JSON-serializable score and diagnostic measurements.

    Full: >=90% visible-ink projected coverage inside +/-3 mm; internal uncovered gaps <=3 mm;
    PCA orientation <=2 degrees;
    <=5 mm stray ink outside the finite +/-5 mm corridor; unique ink <=150% of
    target length; fitted-line RMS width <=1.5 mm. All ink must lie within the
    target-aligned grading region x=[-60 mm,L+60 mm], y=+/-200 mm. Partial uses >=50% coverage
    inside +/-5 mm, <=5 degrees, <=10 mm stray, <=150% ink,
    <=3 mm RMS width, and <=5 mm internal gaps. Otherwise zero. Endpoint deficits
    are accounted for by coverage, rather than counted a second time as gaps.
    Coverage and gaps use the union of 2 mm ink footprints, including end caps,
    rather than sampled centreline paths. Dense overlapping contact stamps are
    equivalent to a continuous visible line; sparse stamps leave measured gaps.
    Distances and bands are in metres; orientation tolerance is in degrees.
    """
    start, end = np.asarray(target_start, float), np.asarray(target_end, float)
    if start.shape != (2,) or end.shape != (2,) or not np.isfinite([start, end]).all():
        raise ValueError('Target endpoints must be finite 2D points')
    length = float(np.linalg.norm(end-start))
    if length <= _EPS:
        raise ValueError('Target endpoints must differ')
    direction = (end-start)/length
    frame = np.column_stack((direction, [-direction[1], direction[0]]))
    runs, isolated, invalid = [], [], 0
    for stroke in strokes:
        points = np.asarray(stroke, dtype=float)
        if points.size == 0:
            continue
        if points.ndim != 2 or points.shape[1] != 2:
            raise ValueError('Every stroke must have shape (N, 2)')
        run = []
        for point in points:
            if not np.isfinite(point).all():
                invalid += 1
                if run:
                    runs.append(np.asarray(run))
                    run = []
                continue
            p = (point-start) @ frame
            if not run or np.linalg.norm(p-run[-1]) > _EPS:
                run.append(p)
        if run:
            runs.append(np.asarray(run))
    segments = [(a.copy(), b.copy()) for run in runs for a, b in zip(run[:-1], run[1:])]
    isolated = [run[0] for run in runs if len(run) == 1]
    unique = _unique_segments(segments)
    ink_length, stray, angle, width, outside, resolution, raster_cells, metrics = _visible_ink(unique, isolated, length)
    score = 0.0
    for label, threshold, tolerance, max_stray, max_width, max_gap, reward in (
        ('partial', 0.5, 5.0, 0.010, 0.003, 0.005, 0.5),
        ('full', 0.9, 2.0, 0.005, 0.0015, 0.003, 1.0),
    ):
        if (not outside and metrics[f'{label}_coverage'] + _EPS >= threshold
                and metrics[f'{label}_max_gap'] <= max_gap + _EPS
                and angle <= tolerance + _EPS and stray <= max_stray + _EPS
                and ink_length <= 1.5*length + _EPS and width <= max_width + _EPS):
            score = reward
    return dict(score=score, coverage=metrics['full_coverage'],
                stray_length=float(max(0.0, stray)), angle_error_deg=angle,
                unique_ink_length=float(ink_length), line_rms_width=width,
                target_length=length, invalid_points=invalid,
                ink_outside_grading_region=bool(outside),
                ink_diameter=0.002, raster_resolution=resolution, raster_cells=int(raster_cells),
                stroke_count=len(runs), **metrics)



class TransferTracker:
    """Grade a consecutive physics trace using only measured poses and contacts."""

    def __init__(self, spec=DEFAULT_CONFIG):
        # The pure defaults also support replay on a machine without yam-arena.
        from types import SimpleNamespace
        from .contact import config_dict

        self.config = SimpleNamespace(**config_dict(spec))
        self.target_start, self.target_end = target_endpoints(self.config)
        self.strokes = []
        self.guided_strokes = []
        self.tick = 0
        self._stroke = self._guided = None
        capture_ticks = math.ceil(self.config.CAPTURE_HOLD_S/self.config.PHYSICS_DT-_EPS)
        self._capture_window = deque(maxlen=capture_ticks)
        self._capture_angle = None
        self._capture_origin = None
        self._release = self._carried = self._transfer = False
        self._release_hold = 0
        self._airborne_origin = None
        self._max_lift = self._max_translation = 0.
        self._angle = self._adjustment = None
        self._initial_angle = self._initial_reference_error = None
        self._invalidations = self._captures = 0
        self._nib_count = self._guide_count = 0
        self._last_ink_tick = self._lift_hold = 0
        self._marker = self._board = None
        self._geometry_cache = None
        self._geometry_dirty = False
        self._cache_tick = 0

    def _invalidate(self):
        if self._capture_angle is None:
            return
        self._invalidations += 1
        self._capture_window.clear()
        self._capture_angle = self._capture_origin = None
        self._release = self._carried = self._transfer = False
        self._release_hold = 0
        self._airborne_origin = None
        self._max_lift = self._max_translation = 0.
        self._guided = None
        self.guided_strokes.clear()
        self._geometry_dirty = True

    def _advance_transfer(self, record, frames):
        config = self.config
        stock, blade, board, reference_stock, reference_blade = frames
        self._angle = relative_angle(stock, blade, config)
        if self._initial_angle is None and self._angle is not None:
            self._initial_angle = self._angle
            self._initial_reference_error = angle_difference(self._angle, config.REFERENCE_ANGLE_DEG)
        self._adjustment = (angle_difference(self._angle, self._initial_angle)
                            if self._angle is not None else None)
        clear = blade_released(record, blade, config)
        if self._capture_angle is not None and (
                self._angle is None
                or angle_difference(self._angle, self._capture_angle) > config.RETENTION_ANGLE_DEG
                or (self._release and not clear)):
            self._invalidate()
        contacts = capture_contacts(record, stock, blade, reference_stock, reference_blade, config)
        if self._capture_angle is None:
            fitting = (contacts and self._angle is not None
                       and self._initial_reference_error >= config.INITIAL_REFERENCE_ERROR_DEG
                       and self._adjustment >= config.ADJUSTMENT_DEG
                       and angle_difference(self._angle, config.REFERENCE_ANGLE_DEG) <= config.CAPTURE_ANGLE_DEG)
            if not fitting:
                self._capture_window.clear()
                return
            self._capture_window.append((self._angle, *stock[0]))
            if len(self._capture_window) == self._capture_window.maxlen:
                # The XYZ envelope diagonal bounds motion between any two samples.
                angle_span, *stock_span = np.ptp(np.asarray(self._capture_window), axis=0)
                if (angle_span <= config.CAPTURE_ANGLE_SPAN_DEG
                        and np.linalg.norm(stock_span) <= config.CAPTURE_STOCK_SPAN):
                    self._capture_angle = self._angle
                    self._capture_origin = stock[0].copy()
                    self._captures += 1
            return
        displacement = stock[0]-self._capture_origin
        if not self._release:
            # A late opening after departure cannot retroactively authorize a carry.
            still_at_reference = contacts or np.linalg.norm(displacement) <= .005
            if still_at_reference:
                self._release_hold = self._release_hold+1 if clear else 0
                self._release = self._release_hold*config.PHYSICS_DT + _EPS >= config.RELEASE_HOLD_S
            else:
                self._invalidate()
            return
        self._max_lift = max(self._max_lift, float(displacement[2]))
        if not self._carried:
            if displacement[2] + _EPS < config.TRANSFER_MIN_AIRBORNE_HEIGHT:
                self._airborne_origin = None
            elif self._airborne_origin is None and displacement[2] + _EPS >= config.TRANSFER_LIFT:
                self._airborne_origin = stock[0][:2].copy()
            if self._airborne_origin is not None:
                travel = float(np.linalg.norm(stock[0][:2]-self._airborne_origin))
                self._max_translation = max(self._max_translation, travel)
                self._carried = travel + _EPS >= config.TRANSFER_TRANSLATION
        if self._carried and not self._transfer and not contacts:
            start = board[0]+board[1] @ np.array([*self.target_start, config.BOARD_SIZE[2]/2])
            guide = guide_metrics(start, stock, blade, board, [0., 0., 0.], record["stock_board_force"], config)
            self._transfer = guide.get("guide_placement", False)

    def sample(self, raw):
        """Add one observation and return diagnostics suitable for JSONL and ink rendering."""
        config = self.config
        expected = self.tick+1
        dt = float(raw["dt"])
        if (type(raw["tick"]) is not int or raw["tick"] != expected
                or not math.isfinite(dt) or not math.isclose(dt, config.PHYSICS_DT, abs_tol=1e-10, rel_tol=1e-8)):
            raise ValueError(f"Sample {expected}: expected consecutive ticks and {config.PHYSICS_DT} s physics dt")
        self.tick = expected
        self._marker, self._board = raw["marker_pose"], raw["board_pose"]
        try:
            frames = tuple(pose(raw[name]) for name in
                           ("stock_pose", "blade_pose", "board_pose", "reference_stock_pose", "reference_blade_pose"))
        except (TypeError, ValueError):
            frames = None
        if frames is None:
            self._invalidate()
            self._capture_window.clear()
            self._angle = self._adjustment = None
        else:
            self._advance_transfer(raw, frames)
        accepted, details = contact_gate(
            raw["pair_force"], raw["pair_point"], self._marker, self._board,
            nib_local=config.MARKER_NIB_LOCAL, nib_radius=config.MARKER_NIB_RADIUS,
            board_size=config.BOARD_SIZE)
        self._lift_hold = advance_lift_hold(self._lift_hold, accepted, self._marker, self._board, config)
        point = details.get("point_board")
        point_score = point[:2] if accepted else None
        segment_start = guide_used = False
        guide = {"guide_geometry": False}
        if accepted:
            self._last_ink_tick = self.tick
            self._nib_count += 1
            if frames is not None:
                guide = guide_metrics(raw["pair_point"], *frames[:3], raw["guide_pair_force"],
                                      raw["stock_board_force"], config)
                guide_used = self._transfer and guide["guide_geometry"]
            segment_start = self._stroke is None or np.linalg.norm(np.asarray(point_score)-self._stroke[-1]) > .01
            if segment_start:
                self._stroke = []
                self.strokes.append(self._stroke)
                self._guided = None
            self._stroke.append(point_score)
            if guide_used:
                if self._guided is None:
                    self._guided = []
                    self.guided_strokes.append(self._guided)
                self._guided.append(point_score)
                self._guide_count += 1
            else:
                self._guided = None
            self._geometry_dirty = True
        else:
            self._stroke = self._guided = None
        return dict(raw, **details, **guide, ink=accepted, point_score=point_score,
                    ink_segment_start=bool(segment_start), guide_used=bool(guide_used),
                    gauge_pose_angle_deg=self._angle, adjustment_deg=self._adjustment,
                    capture_valid=self._capture_angle is not None, released=self._release,
                    transfer_valid=self._transfer)

    def metrics(self, *, force=True):
        """Report exact final metrics, or limit raster refreshes in live previews."""
        config = self.config
        if self._geometry_cache is None or (self._geometry_dirty and
                (force or self.tick-self._cache_tick >= round(.5/config.PHYSICS_DT))):
            all_ink = score_strokes(self.strokes, self.target_start, self.target_end)
            guided = score_strokes(self.guided_strokes, self.target_start, self.target_end)
            self._geometry_cache = all_ink, guided
            self._geometry_dirty = False
            self._cache_tick = self.tick
        all_ink, guided = self._geometry_cache
        length = all_ink["unique_ink_length"]
        fraction = min(1., guided["unique_ink_length"]/length) if length else 0.
        lifted, lift = (False, {"nib_clearance": None, "ink_quiet_s": 0., "lift_clear_s": 0.})
        if self.tick:
            quiet = (self.tick-self._last_ink_tick)*config.PHYSICS_DT if self._last_ink_tick else 0.
            lifted, lift = lift_gate(self._marker, self._board, quiet,
                                     nib_local=config.MARKER_NIB_LOCAL, nib_radius=config.MARKER_NIB_RADIUS,
                                     board_size=config.BOARD_SIZE, clear_s=self._lift_hold*config.PHYSICS_DT)
        line_score = min(all_ink["score"], guided["score"])
        score = 0.
        if self._transfer:
            score = .25
            if fraction >= .5 and line_score >= .5:
                score = .5
            if fraction >= .9 and line_score >= 1. and lifted:
                score = 1.
        return dict(all_ink, **lift, score=score, success=score == 1.,
                    geometry_score=all_ink["score"], guided_geometry_score=guided["score"],
                    guided_coverage=guided["coverage"], unique_guided_ink_length=guided["unique_ink_length"],
                    guide_used_fraction=fraction, physics_samples=self.tick,
                    geometry_evaluated_tick=self.tick if force or not self._geometry_dirty else self._cache_tick,
                    nib_contact_samples=self._nib_count,
                    guide_used_samples=self._guide_count, marker_lift_complete=lifted,
                    gauge_pose_angle_deg=self._angle, adjustment_deg=self._adjustment,
                    initial_measured_angle_deg=self._initial_angle,
                    initial_reference_error_deg=self._initial_reference_error,
                    capture_valid=self._capture_angle is not None, captured_angle_deg=self._capture_angle,
                    capture_count=self._captures, capture_hold_s=len(self._capture_window)*config.PHYSICS_DT,
                    retention_invalidations=self._invalidations, released=self._release,
                    transfer_valid=self._transfer, transfer_max_lift=self._max_lift,
                    transfer_max_translation=self._max_translation)





