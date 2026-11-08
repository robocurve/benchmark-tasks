"""Fixed policy-time switch; no actions, force path or stepping.

The existing AssayPhysicsSwitch performs all native dt/configuration work and
audits each interval. This wrapper only schedules it and certifies cleanup.
"""
from copy import deepcopy
import json
import math
from pathlib import Path

from config import POLICY_PHYSICS_SCHEDULE as SCHEDULE


class PolicyPhysicsError(RuntimeError):
    pass


def validate_schedule(schedule, initial_physics_hz):
    """Reject unsupported schedules before constructing a trial or its files."""
    if schedule is None:
        return None
    if not isinstance(schedule, dict) or schedule != SCHEDULE:
        raise ValueError('Unsupported policy physics schedule')
    if initial_physics_hz != SCHEDULE['initial_physics_hz']:
        raise ValueError('The fixed policy physics schedule requires initial --physics-hz 240')
    return deepcopy(SCHEDULE)


class FixedPolicyPhysics:
    """Own one fresh trial; bracket every ordinary 20 Hz policy action.

Caller contract: before_step(t, frames); existing_step(unchanged_action);
after_step(t, frames). On any exception call fail(exc) and re-raise. Always
close(final_capture_time_s) before destroying the simulation, then save().
"""

    def __init__(self, base, *, factory=None):
        self.base, self.factory = base, factory
        self.transition = None
        self.completed_actions = 0
        self.pending = None
        self.failure = None
        self.closed = False
        self.final_capture_time_s = None
        self.expected_intervals = None

    def fail(self, exc):
        if self.failure is None:
            self.failure = f'{type(exc).__name__}: {exc}'
        if self.transition is not None:
            self.transition.fail(exc)

    def _ready(self):
        if self.closed or self.failure is not None:
            raise PolicyPhysicsError(self.failure or 'Policy physics audit is closed')

    def before_step(self, capture_time_s, recorded_frames):
        try:
            self._ready()
            if self.pending is not None:
                raise ValueError('Previous ordinary policy interval is unfinished')
            expected_time = self.completed_actions / SCHEDULE['control_hz']
            if not math.isfinite(capture_time_s) or abs(capture_time_s - expected_time) > 1e-9:
                raise ValueError('Policy time does not match completed control actions')
            if recorded_frames != self.completed_actions + 1:
                raise ValueError('Recorded frames do not match completed control actions')
            if self.completed_actions == SCHEDULE['switch_after_completed_actions']:
                if self.transition is not None:
                    raise ValueError('Policy physics transition already exists')
                factory = self.factory
                if factory is None:
                    from assay_physics import AssayPhysicsSwitch
                    factory = AssayPhysicsSwitch
                self.transition = factory(self.base, SCHEDULE['target_physics_hz'],
                                          capture_time_s, diagnostic_only=False)
                self.transition.start()
            self.pending = (float(capture_time_s), int(recorded_frames))
            if self.transition is not None:
                self.transition.before_step(capture_time_s, recorded_frames)
        except BaseException as exc:
            self.fail(exc)
            raise

    def after_step(self, capture_time_s, recorded_frames):
        try:
            self._ready()
            if self.pending is None:
                raise ValueError('No pending ordinary policy interval')
            previous_time, previous_frames = self.pending
            if (not math.isfinite(capture_time_s)
                    or abs(capture_time_s - previous_time - 1/SCHEDULE['control_hz']) > 1e-9
                    or recorded_frames != previous_frames + 1):
                raise ValueError('Ordinary policy step must add 0.05 seconds and one frame')
            if self.transition is not None:
                self.transition.after_step(capture_time_s, recorded_frames)
            self.completed_actions += 1
            self.pending = None
        except BaseException as exc:
            self.fail(exc)
            raise

    def close(self, final_capture_time_s):
        """Always remove native callback; incomplete evidence stays a failure."""
        if not self.closed:
            try:
                value = float(final_capture_time_s)
                if not math.isfinite(value):
                    raise ValueError('Final capture time is nonfinite')
                self.final_capture_time_s = value
            except (TypeError, ValueError, OverflowError) as exc:
                self.fail(exc)
            if self.pending is not None:
                self.fail(ValueError('Unfinished ordinary policy interval at shutdown'))
            if self.transition is not None:
                try:
                    self.transition.close()
                except BaseException as exc:
                    self.fail(exc)
            try:
                if self.final_capture_time_s is None:
                    raise ValueError('Final capture time is unavailable')
                end_step = round(self.final_capture_time_s * SCHEDULE['control_hz'])
                if (not math.isfinite(self.final_capture_time_s)
                        or abs(end_step / 20 - self.final_capture_time_s) > 1e-9
                        or end_step != self.completed_actions):
                    raise ValueError('Final capture time and completed actions disagree')
                self.expected_intervals = max(0, end_step - SCHEDULE['switch_after_completed_actions'])
                if self.expected_intervals:
                    report = self.transition.report if self.transition is not None else {}
                    closure = report.get('closure') or {}
                    if (closure.get('schema') != 'native-counter-closure-v1'
                            or closure.get('complete') is not True
                            or 'callback_error' not in closure or closure['callback_error'] is not None
                            or closure.get('expected_counters') != closure.get('observed_counters')
                            or not closure.get('observed_counters')):
                        raise ValueError('Policy native final callback/counter closure is incomplete')
                    intervals = report.get('intervals', [])
                    if (report.get('status') != 'validated_intervals' or report.get('error')
                            or report.get('validated_control_intervals') != self.expected_intervals
                            or len(intervals) != self.expected_intervals
                            or any(row.get('validated') is not True for row in intervals)
                            or any(abs(row.get('capture_time_s', -1) -
                                       (SCHEDULE['switch_capture_time_s'] +
                                        (i + 1) / SCHEDULE['control_hz'])) > 1e-9
                                   for i, row in enumerate(intervals))
                            or report.get('callback_removed') is not True
                            or abs(report.get('switch_capture_time_s', -1) -
                                   SCHEDULE['switch_capture_time_s']) > 1e-9):
                        raise ValueError('Policy native interval certificate is incomplete')
                elif self.transition is not None:
                    raise ValueError('Transition began without a completed post-boundary interval')
            except BaseException as exc:
                self.fail(exc)
            self.closed = True
        if self.failure is not None:
            raise PolicyPhysicsError(self.failure)
        return self.report

    @property
    def report(self):
        status = ('failed' if self.failure else
                  'complete' if self.closed and self.transition is not None else
                  'not_reached' if self.closed else 'in_progress')
        return {
            'schema': 'fixed-policy-physics-v2',
            'schedule': deepcopy(SCHEDULE),
            'status': status,
            'error': self.failure,
            'completed_actions': self.completed_actions,
            'pending_interval': self.pending is not None,
            'final_capture_time_s': self.final_capture_time_s,
            'expected_control_intervals': self.expected_intervals,
            'closed': self.closed,
            'native_transition': deepcopy(self.transition.report) if self.transition else None,
            'policy_actions_modified': False,
            'external_force_path_called': False,
        }

    def save(self, path):
        Path(path).write_text(json.dumps(self.report, indent=2, allow_nan=False) + '\n')
