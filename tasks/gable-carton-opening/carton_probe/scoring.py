"""Outcome scorer for the already-unsealed rigid-fold carton surrogate.

Private physics inputs only. Optional --scored launcher branch, pending a
verified full-score run. The same class is used by offline replay and live monitor.
"""
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation

HZ = 240
HOLD_SAMPLES = 3 * HZ + 1  # 721 poses span three full seconds; reject slow overfolding.
MAX_CREASE_GAP = .001


class Outcome:
    def __init__(self, contract, paths):
        self.contract = contract
        self.names = [str(p).rsplit('/', 1)[-1] for p in paths]
        self.index = {n: i for i, n in enumerate(self.names)}
        if len(self.index) != len(self.names) or set(self.names) != set(contract['centres']):
            raise ValueError('Incomplete or duplicated body coverage')
        self.centres = {n: np.asarray(c, float) for n, c in contract['centres'].items()}
        self.body = self.index['carton_body']
        self.history = deque(maxlen=HOLD_SAMPLES)
        self.invalid_reason = None
        self.peak_gap = 0.
        self.samples = 0
        self.value = 0.
        self.detail = {}

    def reject(self, reason):
        self.invalid_reason = self.invalid_reason or reason
        self.value = 0.
        self.history.clear()
        self.detail = dict(score=0., invalid_reason=self.invalid_reason, samples=self.samples)
        return self.value

    def update(self, poses, robot_contact_count, table_contact_counts, *, broken=False):
        self.samples += 1
        if self.invalid_reason:
            return self.reject(self.invalid_reason)
        q = np.asarray(poses, float)
        support = np.asarray(table_contact_counts)
        if (q.shape != (len(self.names), 7) or support.shape != (len(self.names), 1)
                or not np.isfinite(q).all() or not np.isfinite(support).all()
                or not np.isfinite(robot_contact_count) or robot_contact_count < 0
                or np.any(support < 0) or robot_contact_count >= 65536 or support.sum() >= 65536
                or np.any(support != np.floor(support)) or robot_contact_count != int(robot_contact_count)):
            return self.reject('Missing, nonfinite, invalid or saturated physical evidence')
        if np.max(np.abs(np.linalg.norm(q[:, 3:], axis=1) - 1.)) > .001:
            return self.reject('Invalid body orientation')
        if broken:
            return self.reject('Broken carton joint')
        rotations = Rotation.from_quat(q[:, 3:])
        matrices = rotations.as_matrix()
        q=q.copy();q[:,3:]=rotations.as_quat()

        def point(name, p):
            i = self.index[name]
            return q[i, :3] + matrices[i] @ (np.asarray(p) - self.centres[name])

        origin = point('carton_body', [0, 0, 0])
        inv = matrices[self.body].T
        gap = max(np.linalg.norm(point(h['bodies'][0], p) - point(h['bodies'][1], p))
                  for h in self.contract['hinges'] for p in h['edge'])
        self.peak_gap = max(self.peak_gap, float(gap))
        if gap > MAX_CREASE_GAP:
            return self.reject('Carton crease separated by more than1mm')
        a = self.contract['width_m'] / 2
        h = self.contract['inner_pouring_fin_height_m']
        apex = inv @ (point('carton_spout', [0, 0, a]) - origin)
        lip = inv @ (point('carton_inner_fin_minus', [0, 0, a+h]) - origin)
        left = inv @ (point('carton_inner_fin_minus', [-a, 0, a+h]) - origin)
        right = inv @ (point('carton_inner_fin_plus', [-a, 0, a+h]) - origin)
        width = abs(right[1] - left[1])
        projection = min(-apex[0]-a, -lip[0]-a)
        tilt = np.rad2deg(np.arccos(np.clip(matrices[self.body, 2, 2], -1, 1)))
        valid_form = tilt <= 5. and lip[2] >= a
        partial = valid_form and width >= .020 and projection >= 0.
        opened = valid_form and width >= .030 and projection >= .008
        released = robot_contact_count == 0
        supported = support[self.body, 0] > 0
        ready = bool(opened and released and supported)
        if ready:
            self.history.append(q.copy())
        else:
            self.history.clear()
        stable = False
        drift = angle = None
        if len(self.history) == HOLD_SAMPLES:
            window = np.asarray(self.history)
            drift = float(np.linalg.norm(window[:, :, :3] - window[0, :, :3], axis=-1).max())
            dots = np.abs(np.sum(window[:, :, 3:] * window[0, :, 3:], axis=-1))
            angle = float(np.rad2deg(2*np.arccos(np.clip(dots, -1, 1))).max())
            stable = drift <= .001 and angle <= 1.
        self.value = 1. if stable else .5 if opened else .25 if partial else 0.
        self.detail = dict(score=self.value, samples=self.samples, width_mm=float(width*1000),
            apex_projection_mm=float((-apex[0]-a)*1000), upper_lip_projection_mm=float((-lip[0]-a)*1000),
            upper_lip_height_mm=float(lip[2]*1000), body_tilt_deg=float(tilt), crease_gap_mm=float(gap*1000),
            peak_crease_gap_mm=self.peak_gap*1000, robot_released=bool(released), body_supported=bool(supported),
            open_form=bool(opened), qualified_pose_count=len(self.history), stable=bool(stable),
            hold_drift_mm=None if drift is None else drift*1000, hold_angle_deg=angle, invalid_reason=None)
        return self.value

# Installed by the launcher/runtime, never exposed in policy observations.
ACTIVE = None
ENABLED = False
AUTO_TERMINATE = True  # Diagnostic option only; never changes the score.

def score():
    return 0. if not ENABLED or ACTIVE is None else ACTIVE.value
