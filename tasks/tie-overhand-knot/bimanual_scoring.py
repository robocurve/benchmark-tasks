"""Geometric scoring for tying an overhand from an uncrossed, unclamped cord.

0.5: an intact overhand remains released for the final five seconds. 1.0 also
requires a preceding one-second interval with that knot present, the entire
cord at least 1 cm above the tabletop, and its outer tips at least L - 14*d
apart. L is nominal centreline length plus both rounded end caps; d is diameter.
This is a declared span target, not a measurement of tension or knot strength.

Only gravity and robot contacts may manipulate the cord. The scorer rejects
declared fixtures or external-force phases; source/recording review must verify
those scene assumptions. It does not infer a particular grasp from proximity.
Both arms are used by the demonstration, but alternative valid strategies are
not penalized. No projection-dependent knot-core or tail metric gates credit.

Topology remains a conservative open-curve certificate, not proof of physics.
Uncertainty is reported as uncertainty and breaks continuous qualification.
Regrasping restarts release time. A lost or uncertified knot also cancels its
earlier span certificate; tying another knot must establish a new certificate.
The last state, rather than the highest historical score, determines credit.
"""
from __future__ import annotations

import gzip
import json
import math
from pathlib import Path

import numpy as np

from scoring import _hand_clearance, _number, _read_cord
from topology import analyze_open_rope


DEFAULTS = dict(radius=.005, segment=.0125, length=1.2, control_hz=20.,
                table_top=.7475, joint_gap_max_m=.002, hand_clearance_m=.03,
                span_allowance_diameters=14., span_seconds=1.,
                suspended_clearance_m=.01, release_seconds=5.,
                topology_interval_s=.25)


def _uncrossed_top_view(points):
    """The fixed initial layout must have no nonadjacent XY crossings/touches."""
    p = np.asarray(points)[:, :2]
    a, u = p[:-1], np.diff(p, axis=0)
    lengths = np.linalg.norm(u, axis=1)
    if np.any(lengths < 1e-9):
        return False  # An edge-on initial segment cannot be certified in this view.
    i, j = np.triu_indices(len(u), k=2)
    v, offset = u[j], a[j] - a[i]
    cross = lambda x, y: x[:, 0] * y[:, 1] - x[:, 1] * y[:, 0]
    denominator = cross(u[i], v)
    regular = np.abs(denominator) > 1e-12
    safe = np.where(regular, denominator, 1.)
    s, t = cross(offset, v)/safe, cross(offset, u[i])/safe
    if np.any(regular & (s >= -1e-9) & (s <= 1+1e-9) & (t >= -1e-9) & (t <= 1+1e-9)):
        return False
    collinear = ~regular & (np.abs(cross(offset, u[i])) <= 1e-12)
    if np.any(collinear):
        norm = lengths[i]**2
        start = np.sum(offset*u[i], axis=1)/norm
        end = np.sum((offset+v)*u[i], axis=1)/norm
        if np.any(collinear & (np.minimum(start, end) <= 1+1e-9) & (np.maximum(start, end) >= -1e-9)):
            return False
    return True


def _span_metrics(cord, config):
    ends, radius = cord['endpoints'], cord['radius']
    first_axis = (ends[0, 1] - ends[0, 0]) / cord['segment']
    last_axis = (ends[-1, 1] - ends[-1, 0]) / cord['segment']
    tips = np.stack((ends[0, 0] - radius*first_axis, ends[-1, 1] + radius*last_axis))
    span = float(np.linalg.norm(tips[1] - tips[0]))
    total_gap = float(np.linalg.norm(ends[:-1, 1] - ends[1:, 0], axis=1).sum())
    physical_length = config['length'] + 2*radius
    return dict(outer_tips_m=tips.tolist(), outer_tip_span_m=span,
                total_joint_gap_m=total_gap,
                span_lower_bound_m=max(0., span-total_gap),
                physical_free_length_m=physical_length,
                required_span_m=physical_length-config['span_allowance_diameters']*2*radius,
                minimum_table_clearance_m=float(ends[:, :, 2].min()-radius-config['table_top']))


class BimanualKnotScorer:
    def __init__(self, config=None):
        self.config = {**DEFAULTS, **(config or {})}
        for key in DEFAULTS:
            self.config[key] = _number(self.config[key], key)
            if key != 'table_top' and self.config[key] <= 0:
                raise ValueError(f'{key} must be positive')
        if self.config['length']+2*self.config['radius'] <= 2*self.config['radius']*self.config['span_allowance_diameters']:
            raise ValueError('cord is too short for a positive span target')
        if not math.isclose(round(self.config['length']/self.config['segment'])*self.config['segment'],
                            self.config['length'], rel_tol=0, abs_tol=1e-8):
            raise ValueError('nominal length must equal the fixed number of segment lengths')
        self.samples = 0
        self.time = self.invalid = self.topology = self.topology_time = None
        self._last_cord = None
        self._diagnostic_only = bool(self.config.get('diagnostic_only'))
        self._finalized = False
        self.initial_validated = False
        self.initial_reason = None
        self.released = self.overhand = self.span_condition = False
        self.clearance = self.span_metrics = None
        self._span_start = self._release_start = None
        self.span_elapsed = self.release_elapsed = 0.
        self.released_after_span = 0.
        self.span_certificate = None
        self.value = self.candidate_value = 0.
        self.status = self.candidate_status = 'initial state not validated'
        if self.config.get('anchor_start') or self.config.get('development_jig') or self.config.get('jig_geometry'):
            self._invalidate('v4 requires an unclamped cord and no jig')

    def _invalidate(self, reason):
        self.invalid = str(reason)
        self.value = self.candidate_value = 0.
        self.span_certificate = None
        self._span_start = self._release_start = None
        self.span_elapsed = self.release_elapsed = 0.
        self.released_after_span = 0.
        self.candidate_status = 'invalid evidence'
        self.status = 'diagnostic_only' if self._diagnostic_only else self.candidate_status

    def _check_topology(self):
        self.topology = analyze_open_rope(self._last_cord['polyline'], 2*self._last_cord['radius'])
        self.topology_time = self.time
        if self.topology.get('status') == 'invalid':
            self._invalidate('topology geometry: '+str(self.topology.get('reason')))
        self.overhand = self.topology.get('is_overhand') is True

    def _cancel_knot_windows(self):
        self._span_start = self._release_start = None
        self.span_elapsed = self.release_elapsed = 0.
        self.released_after_span = 0.
        self.span_certificate = None

    def _set_score(self):
        self.released_after_span = (max(0., self.time-max(self._release_start, self.span_certificate['end_time']))
                                   if self.released and self._release_start is not None and self.span_certificate else 0.)
        if self.invalid:
            self.candidate_value, self.candidate_status = 0., 'invalid evidence'
        elif self.overhand and self.released and self.release_elapsed >= self.config['release_seconds']-1e-9:
            full = self.span_certificate is not None and self.released_after_span >= self.config['release_seconds']-1e-9
            self.candidate_value = 1. if full else .5
            self.candidate_status = ('span target met and released knot retained' if full
                                     else 'released knot retained; span target or subsequent retention not established')
        elif self.topology and self.topology.get('is_overhand') is None:
            self.candidate_value, self.candidate_status = 0., 'topology uncertain; no continuous qualification'
        else:
            self.candidate_value, self.candidate_status = 0., 'released-knot retention interval incomplete'
        self.value = 0. if self._diagnostic_only else self.candidate_value
        self.status = 'diagnostic_only' if self._diagnostic_only else self.candidate_status

    def update(self, sample):
        self.samples += 1
        self._finalized = False
        if isinstance(sample, dict) and sample.get('diagnostic_only'):
            self._diagnostic_only = True
            self.value, self.status = 0., 'diagnostic_only'
        if self.invalid:
            return self.value
        try:
            time = _number(sample['time'], 'sample time')
            period = 1/self.config['control_hz']
            initial = self.time is None
            if initial and abs(time) > 1e-6:
                raise ValueError('recording must include initial time zero')
            if not initial and not math.isclose(time-self.time, period, rel_tol=0, abs_tol=1e-6):
                raise ValueError('non-increasing timestamp or missing 20 Hz capture interval')
            if sample.get('phase', 'policy') != 'policy' or sample.get('proof'):
                raise ValueError('v4 permits no external-force or assessor phase')
            cord = _read_cord(sample['cord'], self.config)
            clearance = _hand_clearance(sample['hands'], cord)
            released = clearance >= self.config['hand_clearance_m']-1e-9
            metrics = _span_metrics(cord, self.config)
            span_condition = (metrics['span_lower_bound_m'] >= metrics['required_span_m']-1e-9
                              and metrics['minimum_table_clearance_m'] >= self.config['suspended_clearance_m']-1e-9)
            changed = released != self.released or span_condition != self.span_condition
            self.time, self._last_cord = time, cord
            self.clearance, self.released = clearance, released
            self.span_metrics, self.span_condition = metrics, span_condition
            # Qualifying windows and their continuity are checked at every
            # capture; exploratory manipulation can use the cheaper interval.
            if (initial or changed or released or span_condition or self.span_certificate is not None
                    or self.topology_time is None or time-self.topology_time >= self.config['topology_interval_s']-1e-9):
                self._check_topology()
            if self.invalid:
                return self.value
            if initial:
                if not _uncrossed_top_view(cord['polyline']):
                    raise ValueError('initial cord has a top-down crossing or ambiguous vertical segment')
                if self.topology.get('status') != 'unknot' or self.topology.get('is_overhand') is not False:
                    raise ValueError('initial cord is not certified untied')
                self.initial_validated = True
                self.initial_reason = 'uncrossed top view and unknot certificate'
            if not self.overhand:
                self._cancel_knot_windows()
            else:
                if span_condition:
                    if self._span_start is None:
                        self._span_start = time
                    self.span_elapsed = time-self._span_start
                    if self.span_elapsed >= self.config['span_seconds']-1e-9 and self.span_certificate is None:
                        self.span_certificate = dict(start_time=self._span_start, end_time=time,
                            seconds=self.span_elapsed, required_span_m=metrics['required_span_m'],
                            minimum_table_clearance_required_m=self.config['suspended_clearance_m'])
                else:
                    self._span_start, self.span_elapsed = None, 0.
                if released:
                    if self._release_start is None:
                        self._release_start = time
                    self.release_elapsed = time-self._release_start
                else:
                    self._release_start, self.release_elapsed = None, 0.
            self._set_score()
        except (KeyError, ValueError, TypeError, ArithmeticError, IndexError) as exc:
            self._invalidate(exc)
        return self.value

    def finalize(self):
        if not self.invalid and self._last_cord is not None:
            try:
                self._check_topology()
                if not self.overhand:
                    self._cancel_knot_windows()
                self._set_score()
            except (KeyError, ValueError, TypeError, ArithmeticError, IndexError) as exc:
                self._invalidate(exc)
        elif not self.samples and self.invalid is None:
            self._invalidate('empty recording')
        self._finalized = True
        return self.value

    def summary(self, final=False):
        if final and not self._finalized:
            self.finalize()
        return dict(score=self.value, candidate_score=self.candidate_value, status=self.status,
            candidate_status=self.candidate_status, invalid=self.invalid, samples=self.samples, time=self.time,
            diagnostic_only=self._diagnostic_only, terminal_validated=self._finalized,
            score_version='overhand-bimanual-v4', initial_validated=self.initial_validated,
            initial_reason=self.initial_reason, released=self.released,
            minimum_robot_clearance_m=self.clearance,
            topology_status=self.topology.get('status') if self.topology else None,
            topology_reason=self.topology.get('reason') if self.topology else None,
            topology_time=self.topology_time, span=self.span_metrics,
            span_condition=self.span_condition, span_elapsed_s=self.span_elapsed,
            span_certificate=self.span_certificate, release_elapsed_s=self.release_elapsed,
            released_after_span_s=self.released_after_span,
            release_required_s=self.config['release_seconds'],
            max_joint_gap_m=self._last_cord['max_joint_gap_m'] if self._last_cord else None,
            minimum_nonlocal_distance_m=self._last_cord['minimum_nonlocal_distance_m'] if self._last_cord else None,
            evidence_scope='Observed geometry and release retention; no force, strength or grasp measurement')


def certify_result(summary, *, finalization_complete):
    """Incomplete recording/finalization cannot claim the full geometric award."""
    result = dict(summary)
    result['geometric_score'] = result['score']
    result['assay_certification'] = dict(required=False, status='not_required', errors=[])
    result['finalization_certification'] = dict(complete=finalization_complete is True)
    if result.get('diagnostic_only'):
        result['score'] = 0.
    elif finalization_complete is not True and result['score'] > .5:
        result['score'] = .5 if result.get('invalid') is None and result.get('release_elapsed_s', 0) >= result.get('release_required_s', math.inf)-1e-9 else 0.
        result['status'] = 'released knot retained; full credit withheld because finalization is incomplete'
    result['score_scope'] = 'Robot-only span target and final released-knot retention, with complete recording required for full credit'
    return result


def replay_samples(samples, config=None):
    scorer = BimanualKnotScorer(config)
    for sample in samples:
        scorer.update(sample)
    return scorer.summary(final=True)


def replay_file(path, config=None):
    path = Path(path)
    scorer = BimanualKnotScorer(config)
    opener = gzip.open if path.suffix == '.gz' else open
    try:
        with opener(path, 'rt', encoding='utf-8') as handle:
            for line_number, line in enumerate(handle, 1):
                try:
                    scorer.update(json.loads(line))
                except (ValueError, TypeError) as exc:
                    scorer._invalidate(f'malformed JSONL sample at line {line_number}: {exc}')
                    break
    except (OSError, EOFError) as exc:
        scorer._invalidate(f'recording could not be read: {exc}')
    return scorer.summary(final=True)
