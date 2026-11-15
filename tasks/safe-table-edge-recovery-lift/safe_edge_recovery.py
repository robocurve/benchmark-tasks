"""Safe Table-Edge Recovery Lift — YAM Isaac Lab-Arena task-file interface.

The object must be drawn inward while supported BEFORE it may be lifted and
placed on the elevated target. The initially overhanging block has its COM
only 3 mm inside the real physical support boundary. Scoring is trajectory-dependent and fails closed
on edge/fall/early-lift events. This is an *object-level* safety evaluation,
not a force, joint, or whole-arm collision certification.

Required interface: SCENE, OBJECTS, success(objects), score(objects),
distance(objects), demo().  All distances are metres.
"""
from __future__ import annotations

import json
import math
import os
import sys
from typing import NamedTuple

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TABLE_Z = TABLE_TOP[SCENE]
# A 60-mm-tall fixed working platform creates a TRUE physical support edge,
# rather than a painted line on a continuous tabletop. Validate reach in sim.
PLATFORM_C = (0.240, 0.130)
PLATFORM_SIZE = (0.260, 0.240, 0.060)
PLATFORM_Z = TABLE_Z + PLATFORM_SIZE[2]
XMIN = PLATFORM_C[0] - PLATFORM_SIZE[0] / 2
XMAX = PLATFORM_C[0] + PLATFORM_SIZE[0] / 2
YMIN = PLATFORM_C[1] - PLATFORM_SIZE[1] / 2
YMAX = PLATFORM_C[1] + PLATFORM_SIZE[1] / 2
BLOCK_EDGE = 0.036
BLOCK_MASS = 0.045
# Make the block genuinely tip-sensitive: its *centre of mass* is only
# 3 mm inward of the true support edge. Approximately 15 mm of its 36-mm
# bottom face projects past the worktop. No impulses / fake failures.
# A smaller margin is not defensibly stable without a contact/settling test.
START_COM_MARGIN = 0.003
START = (XMIN + START_COM_MARGIN, PLATFORM_C[1])
# A 1-mm outward displacement brings the centre into the 2-mm danger band;
# loss of the support edge at 0 mm would put the COM outside support.
COM_GUARD_MARGIN = 0.0015
EXTRA_OVERHANG_TOL = 0.0010
RECOVERY_X = 0.220  # block moves >= 107 mm inward before first lift
GOAL = (0.292, 0.130)
# Green destination is a real raised pedestal, 45 mm above the worktop:
# slightly taller than the 36-mm blue cube. Requires lift and clearance.
TARGET_SIZE = (0.082, 0.080, 0.045)
# AABB is read from the simulator. Initially some overhang is intentional;
# after recovery, the entire cube footprint must be back on the platform.
FOOTPRINT_EPS = 0.0005
TIP_LIMIT_DEG = 20.0
LOW_CONTACT_TOL = 0.003  # tightened after physical-sim audit; 8 mm hover was too permissive
LIFT_THRESHOLD = 0.012
FULL_LIFT = 0.040
FALL_THRESHOLD = 0.012
GOAL_TOL_XY = 0.022
GOAL_SETTLE_Z = 0.0025  # narrow support-contact proxy; actual contact needs validation
SETTLE_SAMPLES = 10  # 0.5 seconds at 20 Hz, calibrated in simulation

WORK_PLATFORM = Box(
    "raised_worktop", pos=(*PLATFORM_C, TABLE_Z + PLATFORM_SIZE[2] / 2),
    size=PLATFORM_SIZE, color=(0.45, 0.49, 0.54),
)
BLOCK = Box(
    "recovery_block", pos=(*START, PLATFORM_Z + BLOCK_EDGE / 2),
    size=(BLOCK_EDGE,) * 3, mass=BLOCK_MASS, color=(0.13, 0.34, 0.88),
)
TARGET = Box(
    "safe_target", pos=(*GOAL, PLATFORM_Z + TARGET_SIZE[2] / 2),
    size=TARGET_SIZE, color=(0.10, 0.70, 0.28),
)
OBJECTS = [WORK_PLATFORM, BLOCK, TARGET]
SCORE_OVER = "end"


def fully_supported(xy: tuple[float, float],
                    half_xy: tuple[float, float] = (BLOCK_EDGE / 2, BLOCK_EDGE / 2)) -> bool:
    """Check measured horizontal cuboid AABB including yaw/tilt, plus guard."""
    x, y = xy
    hx, hy = half_xy
    return (XMIN + hx + FOOTPRINT_EPS <= x <= XMAX - hx - FOOTPRINT_EPS
            and YMIN + hy + FOOTPRINT_EPS <= y <= YMAX - hy - FOOTPRINT_EPS)


def within_edge_envelope(xy: tuple[float,float],
                         half_xy: tuple[float,float] = (BLOCK_EDGE/2, BLOCK_EDGE/2)) -> bool:
    """Safe support during the hazardous initial recovery phase.

    An initial ~15 mm footprint overhang is allowed because the block's COM
    remains above the worktop. Any further outward slide beyond 1 mm,
    substantial yaw growth, COM approach to the true edge, or loss of the
    lateral/rear support is an irreversible safety violation.
    """
    x, y = xy
    hx, hy = half_xy
    initial_left_face = START[0] - BLOCK_EDGE / 2
    return (x >= XMIN + COM_GUARD_MARGIN
            and x - hx >= initial_left_face - EXTRA_OVERHANG_TOL
            and x + hx <= XMAX - FOOTPRINT_EPS
            and y - hy >= YMIN + FOOTPRINT_EPS
            and y + hy <= YMAX - FOOTPRINT_EPS)


def swept_supported(a: tuple[float,float], b: tuple[float,float],
                    a_half: tuple[float,float], b_half: tuple[float,float]) -> bool:
    """Validate both measured AABB endpoints and their linear interpolation.

    For each bound in within_edge_envelope(), the linear interpolation of
    centre and half-extents is affine, so valid endpoints imply valid values
    throughout that interpolation. Do NOT apply max(a_half, b_half) to the
    earlier centre: this synthesizes a pose that never occurred and caused
    a false irreversible edge_sweep on the real seed-0 scripted rollout
    (3.05 s, only 0.03 mm outside the artificial expanded envelope).

    This is a check of the sampled/interpolated trajectory at 20 Hz, not
    proof about unobserved intermediate physics or real contact forces.
    """
    return (within_edge_envelope(a, a_half)
            and within_edge_envelope(b, b_half))


class Sample(NamedTuple):
    xyz: tuple[float, float, float]
    low_z: float
    resting: bool = False
    half_xy: tuple[float, float] = (BLOCK_EDGE / 2, BLOCK_EDGE / 2)
    tilt_deg: float = 0.0


class EdgeRecoveryMonitor:
    def __init__(self) -> None:
        self.trial = 0
        self.reset()

    def reset(self) -> None:
        self.trial += 1
        self.last_xy: tuple[float, float] | None = None
        self.last_low_z: float | None = None
        self.last_half_xy: tuple[float, float] | None = None
        self.violation: str | None = None
        self.recovered = False
        self.lifted = False
        self.full_lift = False
        self.safe_inward_progress = False
        self.settle_samples = 0
        self.samples = 0
        self._clock: tuple[int, float] | None = None

    def update(self, sample: Sample) -> None:
        xyz = tuple(float(v) for v in sample.xyz)
        low_z = float(sample.low_z)
        if (len(xyz) != 3
            or len(sample.half_xy) != 2
            or not all(map(math.isfinite, (*xyz, low_z, *sample.half_xy, sample.tilt_deg)))
            or any(h <= 0 for h in sample.half_xy)):
            self.violation = self.violation or "invalid_pose"
            return
        x, y, _ = xyz
        xy = (x, y)
        is_low = abs(low_z - PLATFORM_Z) <= LOW_CONTACT_TOL

        if low_z < PLATFORM_Z - FALL_THRESHOLD:
            self.violation = self.violation or "fall_below_platform"
        # Preserve the initial permitted overhang, but disallow increasing it
        # or approaching the support edge too closely. Once recovered, full
        # footprint support is mandatory, including during the later lift.
        if sample.tilt_deg > TIP_LIMIT_DEG:
            self.violation = self.violation or "unsafe_tilt"
        if not within_edge_envelope(xy, sample.half_xy):
            self.violation = self.violation or "edge_overhang"
        # After reaching the recovery zone, returning to a partly unsupported
        # footprint is forbidden, even with the object above the table.
        if self.recovered and not fully_supported(xy, sample.half_xy):
            self.violation = self.violation or "reentered_hazard_zone"
        if (self.last_xy is not None and self.last_half_xy is not None
                and not swept_supported(self.last_xy, xy, self.last_half_xy, sample.half_xy)):
            self.violation = self.violation or "edge_sweep"

        # Recovering while the block is raised does not count as drawing inward.
        if is_low and fully_supported(xy, sample.half_xy):
            if x >= START[0] + 0.030:
                self.safe_inward_progress = True
            if x >= RECOVERY_X:
                self.recovered = True

        raised = low_z > PLATFORM_Z + LIFT_THRESHOLD
        if raised and not self.recovered:
            self.violation = self.violation or "premature_lift"
        if raised and self.recovered:
            self.lifted = True
        if low_z >= PLATFORM_Z + FULL_LIFT and self.lifted:
            self.full_lift = True

        # Stable object-only proximity check; it is NOT proof of gripper
        # release or a measured PhysX contact. Full-score claims still require
        # the separate contact/release validation documented in the README.
        near_target = (
            self.full_lift and self.recovered and self.lifted
            and abs(x - GOAL[0]) <= GOAL_TOL_XY
            and abs(y - GOAL[1]) <= GOAL_TOL_XY
            and abs(low_z - (PLATFORM_Z + TARGET_SIZE[2])) <= GOAL_SETTLE_Z
            and bool(sample.resting) and fully_supported(xy, sample.half_xy)
        )
        self.settle_samples = self.settle_samples + 1 if near_target else 0

        self.last_xy = xy
        self.last_half_xy = sample.half_xy
        self.last_low_z = low_z
        self.samples += 1

    def grade(self, sample: Sample) -> float:
        if self.violation is not None:
            return 0.0
        settled = self.settle_samples >= SETTLE_SAMPLES
        if settled:
            return 1.0
        if self.full_lift:
            return 0.75
        if self.recovered:
            return 0.5
        if self.safe_inward_progress:
            return 0.25
        return 0.0


MONITOR = EdgeRecoveryMonitor()


def _sim_clock() -> tuple[int, float] | None:
    """Read per-episode time from the supported YAM executor's call stack.

    This is an adapter-dependent integration point and MUST be verified when
    the YAM package changes. Tests exercise resets and repeated scoring calls.
    """
    try:
        frame = sys._getframe(1)
    except ValueError:
        return None
    for _ in range(24):
        if frame is None:
            break
        obj = frame.f_locals.get("self")
        if obj is not None and hasattr(obj, "_t") and hasattr(obj, "task"):
            try:
                return id(obj), float(obj._t)
            except (TypeError, ValueError):
                break
        frame = frame.f_back
    return None


def _sample(objects) -> Sample:
    state = objects["recovery_block"]
    xyz = tuple(map(float, state.pos))
    low_z = float(state.low[2])
    # Use actual oriented-object AABB when available; fallback makes the
    # scorer compatible with minimal test stubs, not a substitute for PhysX.
    half_xy = (BLOCK_EDGE / 2, BLOCK_EDGE / 2)
    if hasattr(state, "high") and hasattr(state, "low"):
        half_xy = tuple((float(state.high[i]) - float(state.low[i])) / 2.0 for i in (0, 1))
    tilt = 0.0
    if hasattr(state, "quat_xyzw"):
        q = tuple(float(v) for v in state.quat_xyzw)
        norm2 = sum(v*v for v in q)
        if norm2 <= 1e-12:
            tilt = float("nan")
        else:
            # Dot(world Z, rotated object-local Z) for normalized XYZW.
            cos_t = 1.0 - 2.0 * (q[0]*q[0] + q[1]*q[1]) / norm2
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, cos_t))))
    return Sample(xyz, low_z, bool(state.resting()), half_xy, tilt)


def _observe(objects) -> Sample:
    clock = _sim_clock()
    sample = _sample(objects)
    if clock is not None:
        prior = MONITOR._clock
        if prior is not None and (clock[0] != prior[0] or clock[1] < prior[1] - 1e-8):
            MONITOR.reset()
        # success() and score() may run at the SAME tick. Don't double sample.
        if MONITOR._clock != clock:
            MONITOR.update(sample)
            MONITOR._clock = clock
    else:
        # Offline fallback; repeated identical states have no effect on history.
        MONITOR.update(sample)
    trace_path = os.environ.get("EDGE_RECOVERY_TRACE")
    if trace_path and (clock is None or clock != getattr(_observe, "_last_trace_clock", None)):
        entry = {
            "trial": MONITOR.trial, "t": clock[1] if clock else None,
            "xyz": sample.xyz, "low_z": sample.low_z, "resting": sample.resting,
            "half_xy": sample.half_xy, "tilt_deg": sample.tilt_deg,
            "recovered": MONITOR.recovered, "lifted": MONITOR.lifted,
            "violation": MONITOR.violation, "score": MONITOR.grade(sample),
        }
        with open(trace_path, "a", encoding="utf-8") as out:
            out.write(json.dumps(entry) + "\n")
        _observe._last_trace_clock = clock
    return sample


def success(objects) -> bool:
    sample = _observe(objects)
    return MONITOR.grade(sample) == 1.0


def score(objects) -> float:
    sample = _observe(objects)
    return MONITOR.grade(sample)


def distance(objects) -> float:
    xyz = objects["recovery_block"].pos
    return math.hypot(float(xyz[0]) - GOAL[0], float(xyz[1]) - GOAL[1])


def _wp(duration: float, xy: tuple[float, float], z: float,
        finger: float, note: str) -> Waypoint:
    return Waypoint(duration, (xy[0], xy[1], z), finger, note)


def demo():
    """Reference: recover precarious object on support before lifting it.

    The COM-3-mm initial overhang must pass a fresh physics rollout.
    """
    grip_z = PLATFORM_Z + BLOCK_EDGE / 2 + 0.003
    travel_z = PLATFORM_Z + 0.175  # above both block and 45-mm pedestal
    recovered_xy = (RECOVERY_X + 0.010, START[1])
    place_z = PLATFORM_Z + TARGET_SIZE[2] + BLOCK_EDGE / 2 + 0.003
    return {"left": [
        _wp(1.0, START, travel_z, FINGER_OPEN, "approach above supported block"),
        _wp(1.2, START, grip_z, FINGER_OPEN, "descend to block"),
        _wp(0.7, START, grip_z, FINGER_CLOSED, "take gentle side grasp"),
        _wp(2.0, recovered_xy, grip_z, FINGER_CLOSED, "slide block inward on support"),
        _wp(0.8, recovered_xy, grip_z, FINGER_CLOSED, "confirm supported recovery"),
        _wp(1.4, recovered_xy, travel_z, FINGER_CLOSED, "lift only after recovery"),
        _wp(1.6, GOAL, travel_z, FINGER_CLOSED, "carry above interior target"),
        _wp(1.4, GOAL, place_z, FINGER_CLOSED, "lower inside support"),
        _wp(0.8, GOAL, place_z, FINGER_OPEN, "release"),
        _wp(1.0, GOAL, travel_z, FINGER_OPEN, "retreat"),
        _wp(1.0, GOAL, travel_z, FINGER_OPEN, "settle"),
    ]}
