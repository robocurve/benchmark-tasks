"""Measure annotated projected-line photographs; never infer hidden surfaces.

Input: four reference_pixels corresponding, in order, to (-10,0), (10,0),
(10,10), (-10,10) mm in the illuminated vertical plane; three independent
calibration_steps_pixels (two points per 0.30 mm step); and profiles, each with
name and pixels in crest/valley/crest order. Coordinates are manually annotated.
pixel_uncertainty is the annotation half-width in pixels, including stripe width.
mapping_uncertainty_mm is an explicit additional height-error allowance for
reference placement/annotation and calibration annotation, supported by the photos.
This optional arithmetic helper is not the task scorer or a surface reconstruction.
"""
import argparse
import json
from pathlib import Path

import numpy as np


REFERENCE_MM = np.array([[-10., 0.], [10., 0.], [10., 10.], [-10., 10.]])


def coordinate_map(pixels):
    pixels = np.asarray(pixels, dtype=float)
    if pixels.shape != (4, 2) or not np.isfinite(pixels).all():
        raise ValueError('Four finite reference pixel pairs are required')
    edges = np.roll(pixels, -1, axis=0) - pixels
    following = np.roll(edges, -1, axis=0)
    turns = edges[:, 0]*following[:, 1] - edges[:, 1]*following[:, 0]
    if not (np.all(turns > 0) or np.all(turns < 0)):
        raise ValueError('Reference points must form an ordered convex quadrilateral')
    rows, values = [], []
    for (u, v), (x, z) in zip(pixels, REFERENCE_MM):
        rows.extend([[u, v, 1, 0, 0, 0, -x*u, -x*v],
                     [0, 0, 0, u, v, 1, -z*u, -z*v]])
        values.extend([x, z])
    try:
        return np.r_[np.linalg.solve(rows, values), 1.].reshape(3, 3)
    except np.linalg.LinAlgError as error:
        raise ValueError('Degenerate reference geometry') from error


def transform(matrix, pixels):
    points = np.asarray(pixels, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
        raise ValueError('Finite pixel pairs are required')
    mapped = np.c_[points, np.ones(len(points))] @ matrix.T
    if (not np.isfinite(mapped).all() or np.any(abs(mapped[:, 2]) < 1e-10)
            or (np.any(mapped[:, 2] > 0) and np.any(mapped[:, 2] < 0))):
        raise ValueError('Points touch or span the projective horizon')
    return mapped[:, :2] / mapped[:, 2, None]


def measure(record):
    matrix = coordinate_map(record['reference_pixels'])
    calibration = record['calibration_steps_pixels']
    if len(calibration) != 3 or any(np.asarray(p).shape != (2, 2) for p in calibration):
        raise ValueError('Three independently photographed reference steps are required')
    calibration_mm = [transform(matrix, p) for p in calibration]
    for points in calibration_mm:
        if (np.any(points[:, 0] < -10) or np.any(points[:, 0] > 10)
                or np.any(points[:, 1] < 0) or np.any(points[:, 1] > 10)):
            raise ValueError('Reference steps must lie inside the calibrated rectangle')
    heights = [float(abs(np.diff(p[:, 1])[0])) for p in calibration_mm]
    errors = [abs(h - .30) for h in heights]
    spread = max(heights) - min(heights)
    quality = max(errors) <= .05 + 1e-9 and spread <= .05 + 1e-9
    halfwidth = float(record['pixel_uncertainty'])
    if not np.isfinite(halfwidth) or halfwidth < .5:
        raise ValueError('Annotation uncertainty must include at least half a pixel')
    mapping_allowance = float(record['mapping_uncertainty_mm'])
    if not np.isfinite(mapping_allowance) or mapping_allowance < .05:
        raise ValueError('Supply a supported mapping-error allowance of at least0.05mm')
    result = {'calibration_step_mm': heights, 'calibration_absolute_error_mm': errors,
              'calibration_spread_mm': spread, 'calibration_passed': quality,
              'meaning': 'Proposed photo procedure; calibration is supplied evidence, not claimed real-world precision.',
              'interval_condition': 'Conditional on the supplied mapping-error allowance covering reference placement/annotation, calibration annotation and optical distortion; the helper cannot verify that allowance.',
              'mapping_uncertainty_mm': mapping_allowance,
              'profiles': []}
    offsets = halfwidth * np.array([[-1, -1], [-1, 1], [1, -1], [1, 1]])
    for profile in record['profiles']:
        pixels = np.asarray(profile['pixels'], dtype=float)
        if pixels.shape != (3, 2):
            raise ValueError('Each profile requires crest, valley, crest pixel pairs')
        points = transform(matrix, pixels)
        if np.any(points[:, 0] < -10) or np.any(points[:, 0] > 10) or np.any(points[:, 1] < 0) or np.any(points[:, 1] > 10):
            raise ValueError('Measure inside the calibrated reference rectangle')
        if not min(points[0, 0], points[2, 0]) < points[1, 0] < max(points[0, 0], points[2, 0]):
            raise ValueError('Valley must lie between the crests')
        boxes = [transform(matrix, p + offsets) for p in pixels]
        # Include annotation bounds and observed step-calibration discrepancy.
        extra = max(errors) + mapping_allowance
        lower = [float(box[:, 1].min() - extra) for box in boxes]
        upper = [float(box[:, 1].max() + extra) for box in boxes]
        interval = [min(lower[0], lower[2]) - upper[1], min(upper[0], upper[2]) - lower[1]]
        decision = 'rephotograph'
        if quality and interval[0] >= .30:
            decision = 'depth_resolved_at_or_above_threshold'
        elif quality and interval[1] < .30:
            decision = 'depth_resolved_below_threshold'
        result['profiles'].append({'name': profile['name'], 'points_mm': points.tolist(),
                                  'ridge_separation_mm': float(abs(points[2, 0] - points[0, 0])),
                                  'dip_mm': float(min(points[0, 1], points[2, 1]) - points[1, 1]),
                                  'dip_annotation_and_calibration_interval_mm': interval,
                                  'depth_evidence': decision})
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('annotations', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    args.out.write_text(json.dumps(measure(json.loads(args.annotations.read_text())), indent=2) + '\n')
