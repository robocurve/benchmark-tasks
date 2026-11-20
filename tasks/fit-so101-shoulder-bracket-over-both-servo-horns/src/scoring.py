"""Scorer for the SO-101 joint-1 closure (engine-independent, numpy only).

Frames (all in metres, right-handed):
* servo frame S: origin on the servo axis at the top horn's outer face, z up along the axis, x = the arm's front
  (the side the bracket's web ends up on). Fixed: the base and the servo case are clamped.
* bracket frame B: rigid on the bracket; origin on its joint axis at the top cheek's inner face, z along the axis
  (up when assembled), x from the cheek ends toward the web. When the bracket is fully home at yaw psi, B = S rotated
  by psi about z (official CAD: B coincides with S at the arm's zero pose).

The scorer reads only the bracket pose, the top horn's angle about the servo axis and the finger pad tips.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# --- thresholds (task.json "scoring" quotes these) ------------------------------------------------------------
ON_SLOT_DEPTH_M = .003       # screw head at least 3 mm inside the top cheek's slot (slot mouth at x = -12.2 mm)
SLOT_MOUTH_X_M = -.0122      # cheek rounded end / slot mouth, bracket frame
ON_LATERAL_M = .0010         # horn axis within 1 mm of the slot centre line (slot 5.84 mm, head 5.0 mm)
ON_AXIAL_M = .0010           # cheeks straddling the horns: bracket height within 1 mm of home
ON_TILT_DEG = 3.0
HOME_RADIAL_M = .0005        # horn axis within 0.5 mm of the cheeks' hole-pattern axis, midway between the cheeks
MID_HEIGHT_M = -.0182        # bracket frame: midway between the two cheek inner faces (0 and -36.4 mm)
HOME_AXIAL_M = .0005
HOME_TILT_DEG = 2.0
# official CAD: the cheeks' hole-pattern axis (also the lip arc's and the slots' centre) is 0.20 mm from frame B's
# origin (the MJCF joint anchor); bracket and horn holes sit on the same 9.9 mm square about their axes
HOLE_AXIS_B_M = np.array([-.000201, .000022])
HOLE_HALF_PITCH_M = .00495
# largest offset of a bracket top hole's bore axis from its horn hole's axis, anywhere along the 3.5 mm bore, at which
# the check screw (thread 2.90 mm across) can still start: the 3.2 mm bore lets it sit up to (3.2 - 2.90) / 2 = 0.15 mm
# off the bore's centre
HOLE_OFFSET_M = .00015
BORE_LEN_M = .0035           # top-hole bore, from the top cheek's inner face (bracket frame z = 0) up
RELEASE_M = .020             # every finger pad tip at least 20 mm from the bracket
STILL_V_M_S = .002
STILL_W_DEG_S = 2.0
HOLD_STEPS = 20              # 1 s at 20 Hz

LEVELS = (0.0, 0.25, 0.5, 0.75, 1.0)


def wrap_deg(a, period):
    """a wrapped into [-period/2, period/2)."""
    return (a + period / 2) % period - period / 2


def yaw_about(r_frame, r_body):
    """Angle (deg) of the body's x axis about the frame's z axis, measured in the frame."""
    v = r_frame.T @ r_body[:, 0]
    return math.degrees(math.atan2(v[1], v[0]))


@dataclass
class Measure:
    px_mm: float           # horn axis in the bracket frame (mm): x along the slot (0 = home, negative = backed off)
    py_mm: float
    pz_mm: float           # bracket height error along the axis (mm)
    tilt_deg: float        # bracket axis vs servo axis
    yaw_deg: float         # bracket yaw about the servo axis (S frame)
    horn_deg: float        # top horn angle about the servo axis (S frame, 0 = CAD hole pattern)
    register_deg: float    # hole-pattern misalignment, wrapped to +-45 deg (logged only)
    hole_offset_mm: float  # largest bore-axis offset of a top hole from its horn hole (both bore ends, best matching)
    on: bool
    home: bool
    registered: bool
    radial_mid_mm: float = 0.  # the home check's radial offset (midway between the cheeks)

    def as_dict(self):
        return {k: (round(v, 4) if isinstance(v, float) else v) for k, v in self.__dict__.items()}


def rz2(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


def _square(c, deg):
    return [c + rz2(deg) @ (HOLE_HALF_PITCH_M * np.array(v)) for v in ((1, 1), (-1, 1), (-1, -1), (1, -1))]


def hole_offset(p_b, r_b, horn_deg, o_s, r_s):
    """Largest distance (m) between a bracket top hole's bore axis and the matching horn hole's axis (servo z), taken
    at both ends of the 3.5 mm bore (the top cheek's inner face and 3.5 mm above it; the larger of the two per hole),
    the largest of the four holes, minimised over the four 90-degree matchings."""
    q = [[(r_s.T @ (p_b + r_b @ np.array([h[0], h[1], z]) - o_s))[:2] for z in (0., BORE_LEN_M)]
         for h in _square(HOLE_AXIS_B_M, 0.)]
    horn = _square(np.zeros(2), horn_deg)
    return min(max(float(np.linalg.norm(e - horn[(i + k) % 4])) for i in range(4) for e in q[i]) for k in range(4))


def measure(p_b, r_b, horn_r, o_s, r_s) -> Measure:
    """p_b, r_b: bracket pose (world); horn_r: top horn body rotation (world); o_s, r_s: servo frame (world)."""
    p_b, r_b, horn_r, o_s, r_s = (np.asarray(x, float) for x in (p_b, r_b, horn_r, o_s, r_s))
    rel = r_b.T @ (o_s - p_b)
    cos_t = float(np.clip(r_b[:, 2] @ r_s[:, 2], -1., 1.))
    tilt = math.degrees(math.acos(cos_t))
    yaw = yaw_about(r_s, r_b)
    horn = yaw_about(r_s, horn_r)
    reg = wrap_deg(yaw - horn, 90.)
    px, py, pz = (float(v) for v in rel)
    # radial offset of the servo axis from the bracket's hole-pattern axis, taken midway between the two cheeks (the
    # mean over both horns): with the modelled face clearance a seated bracket can lean up to about 1 deg on the horns,
    # which moves its top cheek by up to about 0.5 mm while the middle stays within about 0.4 mm
    a = r_b.T @ r_s[:, 2]
    t = (MID_HEIGHT_M - pz) / a[2] if abs(a[2]) > 1e-6 else 0.
    mx, my = px + t * a[0] - HOLE_AXIS_B_M[0], py + t * a[1] - HOLE_AXIS_B_M[1]
    on = (tilt <= ON_TILT_DEG and abs(pz) <= ON_AXIAL_M and abs(py) <= ON_LATERAL_M
          and SLOT_MOUTH_X_M + ON_SLOT_DEPTH_M <= px <= HOME_RADIAL_M)
    home = (math.hypot(mx, my) <= HOME_RADIAL_M and abs(pz) <= HOME_AXIAL_M and tilt <= HOME_TILT_DEG)
    off = hole_offset(p_b, r_b, horn, o_s, r_s)
    registered = home and off <= HOLE_OFFSET_M
    return Measure(px * 1e3, py * 1e3, pz * 1e3, tilt, yaw, horn, reg, off * 1e3, on, home, registered,
                   math.hypot(mx, my) * 1e3)


def boxes_distance(point, boxes_b, p_b, r_b):
    """Distance (m) from a world point to the nearest of the bracket's axis-aligned piece boxes (bracket frame,
    each (lo, hi))."""
    q = np.asarray(r_b, float).T @ (np.asarray(point, float) - np.asarray(p_b, float))
    best = math.inf
    for lo, hi in boxes_b:
        d = np.maximum(np.maximum(np.asarray(lo) - q, q - np.asarray(hi)), 0.)
        best = min(best, float(np.linalg.norm(d)))
    return best


@dataclass
class Scorer:
    """Final-state ladder with a 1 s hold for full score. Call ``step`` once per control step."""
    held: int = 0
    level: float = 0.0
    last: dict = field(default_factory=dict)

    def step(self, m: Measure, tip_clearance_m: float, lin_speed_m_s: float, ang_speed_deg_s: float) -> float:
        released = tip_clearance_m >= RELEASE_M
        still = lin_speed_m_s < STILL_V_M_S and ang_speed_deg_s < STILL_W_DEG_S
        full_now = m.registered and released and still
        self.held = self.held + 1 if full_now else 0
        if m.registered and self.held >= HOLD_STEPS:
            level = 1.0
        elif m.registered:
            level = 0.75
        elif m.home:
            level = 0.5
        elif m.on:
            level = 0.25
        else:
            level = 0.0
        self.level = level
        self.last = dict(measure=m.as_dict(), released=released, still=still, held_steps=self.held,
                         tip_clearance_mm=round(tip_clearance_m * 1e3, 2), lin_speed_mm_s=round(lin_speed_m_s * 1e3, 3),
                         ang_speed_deg_s=round(ang_speed_deg_s, 3), level=level)
        return level
