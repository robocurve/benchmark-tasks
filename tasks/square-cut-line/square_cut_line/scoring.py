# SPDX-License-Identifier: MIT
"""Geometry-only scoring of a square-guided marker line (all distances in metres).

Each input stroke is a continuous polyline; separate strokes and nonfinite points
never create connecting ink. Samples define ink capsules and contact stamps. Exact
collinear retraces are merged for numerical efficiency. Visible-ink metrics use
union occupancy of 2 mm diameter stroke capsules on a 0.25 mm raster, making
microjitter and overlapping retraces behave like photographed ink. No robot state
or timestamps enter grading; the specified reference endpoints define the target.
Coordinates are board-local, not world coordinates.
"""
from __future__ import annotations

import numpy as np

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

