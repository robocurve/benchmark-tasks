"""Pure scoring policy and episode tracker for the glue-tube foil-seal task.

Nothing here imports YAM Arena, so the whole acceptance-critical state machine (thread unwinding, inversion,
puncture, failure latches, release and stillness) is testable offline by feeding synthetic :class:`Frame` poses.
Positions are metres in the station frame.  Progress milestones and violations are episode history (latched);
release and rest are live end-state checks.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# Tube-local geometry (tube origin = shoulder base; +z along the tube axis, up).
CAP_Z0 = 0.012           # seated cap origin
FOIL_Z = 0.0306          # outer face of the foil
SPIKE_TIP_Z = 0.0253     # spike tip in the cap frame (cap +z points the spike away from the open end)
NECK_R = 0.007
BODY_HALF = (0.021, 0.007)  # compliant body half-width x, half-thickness y
BODY_Z = (-0.080, 0.0)      # compliant body extent along the tube axis
SQUEEZE_MARGIN = 0.005      # a finger pad centre this close to the body surface counts as pressing on it

# Thread and scoring thresholds.
PITCH = 0.0015
TURNS = 1.5
UNWIND_TOL_DEG = 10.0
DETACH_M = 0.025
ALIGN_RADIAL_M = 0.0015
ALIGN_TILT_DEG = 5.0
OVER_NOZZLE_M = 0.010
SHEAR_TILT_DEG = 10.0
PUNCTURE_DEPTH_M = 0.0020
FOIL_BREACH_M = 0.0020
RETRACT_M = 0.030
RELEASE_M = 0.040
HOLD_STEPS = 20          # 1.0 s at 20 Hz


@dataclass(frozen=True)
class Evidence:
    detached: bool = False
    aligned: bool = False
    punctured: bool = False
    retracted: bool = False
    released: bool = False
    still_hold: bool = False
    squeezed: bool = False
    sheared: bool = False
    dropped: bool = False
    foil_tampered: bool = False

    @property
    def violated(self) -> bool:
        return self.squeezed or self.sheared or self.dropped or self.foil_tampered


def score_evidence(e: Evidence) -> float:
    """Highest completed milestone; any latched violation zeroes the episode."""
    if e.violated:
        return 0.0
    score = 0.0
    if e.detached:
        score = 0.25
        if e.aligned:
            score = 0.50
            if e.punctured:
                score = 0.75
                if e.retracted and e.released and e.still_hold:
                    score = 1.0
    return score


@dataclass
class Frame:
    cap_pos: np.ndarray
    cap_rot: np.ndarray
    foil_pos: np.ndarray
    finger_pads: list = field(default_factory=list)  # world positions of every gripper finger pad
    tcps: list = field(default_factory=list)         # world positions of each gripper TCP
    still: bool = True                               # cap and foil at rest


@dataclass
class Command:
    slide_target: float
    release_thread: bool = False


def _angle_deg(cos_value: float) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, cos_value))))


class Tracker:
    """Accumulates one episode.  ``tube_pos`` is the fixed tube origin, ``table_top`` the bench height."""

    def __init__(self, tube_pos, table_top: float):
        self.tube = np.asarray(tube_pos, float)
        self.table_top = float(table_top)
        self.reset()

    def reset(self):
        self.flags = dict(detached=False, aligned=False, punctured=False, retracted=False, squeezed=False,
                          sheared=False, dropped=False, foil_tampered=False, contact_seen=False)
        self.thread_released = False
        self.release_sent = False
        self.unwind = 0.0
        self._last_yaw = None
        self.still_steps = 0
        self.released = False
        self.evidence = Evidence()

    def _squeezing(self, pts) -> bool:
        lo = self.tube + np.array([-BODY_HALF[0], -BODY_HALF[1], BODY_Z[0]]) - SQUEEZE_MARGIN
        hi = self.tube + np.array([BODY_HALF[0], BODY_HALF[1], BODY_Z[1]]) + SQUEEZE_MARGIN
        return any(bool(np.all(np.asarray(p) >= lo) and np.all(np.asarray(p) <= hi)) for p in pts)

    def update(self, f: Frame) -> Command:
        fl = self.flags
        rot = np.asarray(f.cap_rot, float)
        cap = np.asarray(f.cap_pos, float)
        axis = rot[:, 2]

        # Thread: the cap only rises as it is turned counter-clockwise about the tube axis.
        yaw = math.atan2(rot[1][0], rot[0][0])
        if self._last_yaw is not None and not self.thread_released:
            d = (yaw - self._last_yaw + math.pi) % (2 * math.pi) - math.pi
            self.unwind = max(0.0, self.unwind + d)
        self._last_yaw = yaw
        full = TURNS * 2 * math.pi
        if not self.thread_released and self.unwind >= full - math.radians(UNWIND_TOL_DEG):
            self.thread_released = True
        command = Command(PITCH * min(self.unwind, full) / (2 * math.pi),
                          self.thread_released and not self.release_sent)
        self.release_sent = self.release_sent or self.thread_released

        seated = self.tube + np.array([0.0, 0.0, CAP_Z0])
        if self.thread_released and float(np.linalg.norm(cap - seated)) >= DETACH_M:
            fl["detached"] = True

        tip = cap + rot @ np.array([0.0, 0.0, SPIKE_TIP_Z])
        plane = self.tube[2] + FOIL_Z
        nozzle = np.array([self.tube[0], self.tube[1], plane])
        depth = plane - float(tip[2])
        radial = float(np.linalg.norm(tip[:2] - nozzle[:2]))
        tilt = _angle_deg(-float(axis[2]))  # angle from straight down (spike toward the nozzle)
        foil_rest = nozzle - np.array([0, 0, 0.0006])
        foil_disp = float(np.linalg.norm(np.asarray(f.foil_pos, float) - foil_rest))

        aligned_now = (fl["detached"] and tilt <= ALIGN_TILT_DEG and radial <= ALIGN_RADIAL_M
                       and depth >= -OVER_NOZZLE_M)
        if aligned_now:
            fl["aligned"] = True
        if aligned_now and depth >= -0.001:
            fl["contact_seen"] = True
        if aligned_now and depth >= PUNCTURE_DEPTH_M and foil_disp >= FOIL_BREACH_M:
            fl["punctured"] = True
        if fl["punctured"] and float(np.linalg.norm(tip - nozzle)) >= RETRACT_M:
            fl["retracted"] = True

        # Failure latches (whole episode).
        if self._squeezing(list(f.finger_pads) + list(f.tcps)):
            fl["squeezed"] = True
        engaged = fl["detached"] and radial <= NECK_R and depth >= -0.0005
        if engaged and tilt > SHEAR_TILT_DEG:
            fl["sheared"] = True
        if foil_disp >= FOIL_BREACH_M and not fl["contact_seen"]:
            fl["foil_tampered"] = True
        if cap[2] < self.table_top - 0.010:
            fl["dropped"] = True

        self.released = all(float(np.linalg.norm(np.asarray(t) - cap)) > RELEASE_M
                            and float(np.linalg.norm(np.asarray(t) - nozzle)) > RELEASE_M for t in f.tcps)
        self.still_steps = self.still_steps + 1 if f.still else 0
        self.evidence = Evidence(
            detached=fl["detached"], aligned=fl["aligned"], punctured=fl["punctured"], retracted=fl["retracted"],
            released=self.released, still_hold=self.still_steps >= HOLD_STEPS, squeezed=fl["squeezed"],
            sheared=fl["sheared"], dropped=fl["dropped"], foil_tampered=fl["foil_tampered"])
        return command

    def score(self) -> float:
        return score_evidence(self.evidence)
