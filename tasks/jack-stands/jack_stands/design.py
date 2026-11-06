"""Geometry, masses, hydraulics and measured-state helpers of the jack-and-stands trainer. NumPy only (no Isaac).

Units: metres, kilograms, radians unless a name ends in _DEG; quaternions xyzw. Station frame: x forward from the
line between the arm bases, y left, z up; TOP is the table top. Heights named ``*_Z`` are above the table top.

The load is a solid steel beam (an axle stand-in) lying across the table (along y) on two end blocks (its feet,
part of the same rigid body). A fixed floor-jack body sits under the beam's middle: its saddle rides on a vertical
prismatic joint whose position drive follows the jack's hydraulics (:class:`Jack`): every full stroke of the pump
lever adds one step of lift, and the release knob, turned counterclockwise, lets the saddle down slowly. Two loose
jack stands stand in front of the beam; each has a long skid with a grip block at its near end, so a gripper can
slide the stand's column under the beam while staying outside the crush zone. A steel wrench (the "work") lies under
the beam with its grip block just past the beam's near edge.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # station table top (yam_arena.task_file.TABLE_TOP["station"])

# -- load: beam + two end blocks (one rigid body, origin at the beam's centre) ---------------------------------------
BEAM_XY = np.array([0.36, 0.0])
BEAM = (0.10, 0.40, 0.06)  # x (depth), y (length), z (height)
FOOT = (0.10, 0.05, 0.05)  # end blocks: x, y, z (table to the beam's underside at the start)
FOOT_Y = BEAM[1] / 2 - FOOT[1] / 2  # flush with the beam's ends
UNDER_START_Z = FOOT[2]  # beam underside above the table at the start
LOAD_MASS = 23.0  # solid steel: (0.10 x 0.40 x 0.06 + 2 x 0.10 x 0.05 x 0.05) m^3 x 7850 kg/m^3 = 22.8 kg
LIFT_Y = 0.0  # lift point (yellow mark) on the beam
SUPPORT_Y = 0.12  # support points (cyan marks) at +-SUPPORT_Y
MARK_W = {"lift": 0.02, "support": 0.03}

# -- jack (fixed body; the saddle is a loose body on a prismatic joint) ----------------------------------------------
JACK_BODY = ((0.36, 0.0), (0.12, 0.09, 0.03))  # centre xy, size (fixed, task file)
SADDLE = (0.08, 0.08, 0.015)  # saddle plate x, y, thickness
SADDLE_GAP = 0.003  # saddle top below the beam's underside at the start
SADDLE_TOP_START_Z = UNDER_START_Z - SADDLE_GAP
SADDLE_MASS = 0.5
RAM_R = 0.015
JACK_TRAVEL = 0.085  # saddle travel above its start (upper joint limit)
PUMP_STEP = 0.007  # lift per full pump stroke
LIFT_RAMP = 0.03  # m/s: each stroke's step is ramped in, not jumped
LOWER_RATE = 0.012  # m/s with the release valve fully open
SADDLE_K, SADDLE_C, SADDLE_FMAX = 3.0e5, 6.0e3, 3000.0  # position drive: N/m, N s/m, N (sag ~1 mm under the load)

# -- pump lever (revolute about +y at the pivot; angle up is positive) -------------------------------------------------
PIVOT = np.array([0.265, 0.0, 0.038])  # x, y, height above the table: pad 3 cm outside the crush zone
LEVER_LEN = 0.08
LEVER_BAR = (0.08, 0.014, 0.010)
PAD = (0.030, 0.040, 0.012)  # green pad at the tip
PAD_X = -0.075  # pad centre along the lever (local x)
PAD_Z = 0.004  # pad centre above the bar's axis (local z): its top is 10 mm above the axis
LEVER_UP_DEG, LEVER_DOWN_DEG = 25.0, -5.0  # joint limits: spring holds it up; pressed, it stops at -5
LEVER_SPRING_TARGET_DEG = 30.0
LEVER_K = 0.012  # N m per degree (0.06 N m holding it up, 0.42 N m fully pressed: 5.6 N at the pad)
LEVER_C = 0.0005  # N m per (deg/s)
LEVER_MASS = 0.08
STROKE_DOWN_DEG, STROKE_UP_DEG = 0.0, 20.0  # a stroke counts only after the lever passes both
CHEEK = (0.045, 0.006, 0.048)  # fixed cheeks either side of the bar, from 1 cm behind the pivot to the jack body
CHEEK_X = 0.2775  # cheek centre x (0.255 .. 0.300)
CHEEK_Y = 0.012

# -- release knob (revolute about +z; counterclockwise from above opens the valve) ------------------------------------
KNOB_XY = np.array([0.20, -0.07])
KNOB_POST = (0.012, 0.045)  # radius, height (fixed, task file)
KNOB_Z = KNOB_POST[1] + 0.001  # knob body origin height (just clear of the post)
KNOB_HUB = (0.012, 0.008)
KNOB_WING = (0.045, 0.010, 0.025)  # x (along the wing at 0 degrees), y (thickness), z
KNOB_MASS = 0.05
KNOB_LIMIT_DEG = (0.0, 90.0)
KNOB_C = 0.0003  # N m per (deg/s)
VALVE_CRACK_DEG, VALVE_FULL_DEG = 10.0, 50.0  # valve opening ramps from 0 to 1 between these knob angles

# -- jack stands (loose; origin at the column's axis on the table) ----------------------------------------------------
STAND_H = 0.08  # saddle top above the table: fixed height
STAND_SKID = ((-0.06, 0.0), (0.19, 0.04, 0.010))  # centre (local x, y), size
STAND_COL = (0.025, 0.025)  # column x, y
STAND_TOP = (0.06, 0.035, 0.010)  # saddle plate x, y, z (on top of the column)
STAND_GRIP = ((-0.14, 0.0), (0.030, 0.020, 0.040))  # grip block centre (local x, y), size; on the skid
STAND_MASS = 1.2  # steel: 1.58e-4 m^3
STAND_SLIDE = 0.10  # start: this far in front of the support point
STAND_START_X = BEAM_XY[0] - STAND_SLIDE

# -- the work part: a wrench lying across under the beam, grip block just past the beam's near edge ---------------------
WRENCH_XY = np.array([0.265, 0.065])  # grip block centre on the table
WRENCH_GRIP = (0.030, 0.020, 0.040)
WRENCH_BAR = (0.090, 0.016, 0.012)  # from the grip block's far face toward +x, under the beam
WRENCH_MASS = 0.32

# -- scoring tolerances -----------------------------------------------------------------------------------------------
ZONE_MARGIN = 0.06  # crush zone: the beam's footprint grown by this, below its underside
RAISED_ABOVE_STAND = 0.005  # 0.25: underside at least this above the stands' tops
CARRY_TOL = 0.004  # the jack carries the load: saddle top within this of the underside at the lift point
REST_TOL = 0.0015  # a stand carries the load: underside within this of the stand's top (scoring.REST_TOL_MM)
UNDER_X_TOL = 0.02  # stand top within this of the beam's centre line (across the beam)
UNDER_Y_TOL = 0.015  # ... and of its support mark (along the beam): the mark is 3 cm wide
STAND_TILT_DEG = 5.0
STAND_ON_TABLE = 0.003
SADDLE_CLEAR = 0.005  # 0.75: saddle top at least this below the underside
LEVEL_DEG = 2.0
PART_OUT_MARGIN = 0.01  # 1.0: every corner of the wrench at least this outside the beam's footprint
PART_MOVED = 0.02  # the wrench moved this far while the load is not on both stands: violation
TIP_DEG = 10.0  # load tilted past this: tipped
DROP_MPS = 0.25  # load falling faster than this: dropped
SHOVE = 0.03  # load moved this far sideways from its start: shoved


# -- small rotation helpers -------------------------------------------------------------------------------------------
def quat_matrix(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_quat(a: float) -> np.ndarray:
    return np.array([0.0, 0.0, math.sin(a / 2), math.cos(a / 2)])


def pitch_quat(a: float) -> np.ndarray:
    """Rotation by ``a`` about +y."""
    return np.array([0.0, math.sin(a / 2), 0.0, math.cos(a / 2)])


def yaw_of(q) -> float:
    x, y, z, w = (float(v) for v in q)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def pitch_y_of(q) -> float:
    """Angle about +y of a body that only turns about y (the lever)."""
    r = quat_matrix(q)
    return math.atan2(r[0, 2], r[0, 0])


def tilt_of(q) -> float:
    """Angle (rad) between the body's +z and world +z."""
    x, y, z, w = (float(v) for v in q)
    return math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y))))


# -- start layout -----------------------------------------------------------------------------------------------------
def world(xy, z: float) -> np.ndarray:
    return np.array([float(xy[0]), float(xy[1]), TOP + z])


def lever_pad_top(angle_deg: float) -> np.ndarray:
    """World point at the centre of the pad's top face for a lever angle (degrees, up positive)."""
    a = math.radians(angle_deg)
    lx, lz = PAD_X, PAD_Z + PAD[2] / 2
    x = lx * math.cos(a) + lz * math.sin(a)
    z = -lx * math.sin(a) + lz * math.cos(a)
    return np.array([PIVOT[0] + x, PIVOT[1], TOP + PIVOT[2] + z])


def start_poses() -> dict[str, np.ndarray]:
    ident = [0.0, 0.0, 0.0, 1.0]
    load_z = UNDER_START_Z + BEAM[2] / 2
    saddle_z = SADDLE_TOP_START_Z - SADDLE[2] / 2
    return {
        "load": np.array([*world(BEAM_XY, load_z), *ident]),
        "saddle": np.array([*world(BEAM_XY, saddle_z), *ident]),
        "lever": np.array([*world(PIVOT[:2], PIVOT[2]), *pitch_quat(math.radians(LEVER_UP_DEG))]),
        "knob": np.array([*world(KNOB_XY, KNOB_Z), *ident]),
        "stand_left": np.array([*world((STAND_START_X, SUPPORT_Y), 0.0), *ident]),
        "stand_right": np.array([*world((STAND_START_X, -SUPPORT_Y), 0.0), *ident]),
        "wrench": np.array([*world(WRENCH_XY, 0.0), *ident]),
    }


STANDS = ("stand_left", "stand_right")
STAND_SIDE = {"stand_left": +1, "stand_right": -1}


# -- hydraulics (pure) --------------------------------------------------------------------------------------------------
def valve_open(knob_deg: float) -> float:
    return float(np.clip((knob_deg - VALVE_CRACK_DEG) / (VALVE_FULL_DEG - VALVE_CRACK_DEG), 0.0, 1.0))


class Jack:
    """Pump-and-release logic of a floor jack, driving the saddle's travel target (m above its start).

    A stroke is the lever pressed down past STROKE_DOWN_DEG after having been up past STROKE_UP_DEG; each stroke
    adds PUMP_STEP of lift (ramped in at LIFT_RAMP) while the release valve is shut. With the valve open the saddle
    target falls at LOWER_RATE x the valve opening and pumping adds nothing. The target stays within the travel."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.target = 0.0
        self.pending = 0.0
        self.up = True  # the lever was last seen up
        self.strokes = 0

    def update(self, lever_deg: float, knob_deg: float, dt: float) -> float:
        opening = valve_open(knob_deg)
        if self.up and lever_deg <= STROKE_DOWN_DEG:
            self.up = False
            self.strokes += 1
            if opening <= 0.0:
                self.pending += PUMP_STEP
        elif not self.up and lever_deg >= STROKE_UP_DEG:
            self.up = True
        add = min(self.pending, LIFT_RAMP * dt)
        self.target += add
        self.pending -= add
        if opening > 0.0:
            self.pending = 0.0
            self.target -= LOWER_RATE * opening * dt
        self.target = float(np.clip(self.target, 0.0, JACK_TRAVEL))
        if self.target >= JACK_TRAVEL:
            self.pending = 0.0
        return self.target


# -- measured-state geometry ------------------------------------------------------------------------------------------
def to_local(pose, p) -> np.ndarray:
    return quat_matrix(pose[3:]).T @ (np.asarray(p, float) - pose[:3])


def underside_z(load, lx: float, ly: float) -> float:
    """World height of the beam's underside at beam-frame (lx, ly)."""
    return float((load[:3] + quat_matrix(load[3:]) @ np.array([lx, ly, -BEAM[2] / 2]))[2])


def stand_top(stand) -> np.ndarray:
    return stand[:3] + quat_matrix(stand[3:]) @ np.array([0.0, 0.0, STAND_H])


def in_zone(load, p) -> bool:
    """A point in the crush zone: within ZONE_MARGIN of the beam's footprint and below its underside, above the
    table."""
    lx, ly, _ = to_local(load, p)
    if abs(lx) > BEAM[0] / 2 + ZONE_MARGIN or abs(ly) > BEAM[1] / 2 + ZONE_MARGIN:
        return False
    u = underside_z(load, float(np.clip(lx, -BEAM[0] / 2, BEAM[0] / 2)), float(np.clip(ly, -BEAM[1] / 2, BEAM[1] / 2)))
    return TOP - 0.01 <= float(p[2]) < u


def wrench_corners(wrench) -> np.ndarray:
    """The wrench's footprint corners (grip block and bar), world xy."""
    gx, gy, _ = WRENCH_GRIP
    bx, by, _ = WRENCH_BAR
    pts = [(-gx / 2, -gy / 2), (-gx / 2, gy / 2), (gx / 2 + bx, -by / 2), (gx / 2 + bx, by / 2),
           (gx / 2, -gy / 2), (gx / 2, gy / 2)]
    r = quat_matrix(wrench[3:])
    return np.array([(wrench[:3] + r @ np.array([x, y, 0.0]))[:2] for x, y in pts])


def wrench_clear(load, wrench) -> float:
    """Smallest distance (m) from the beam's footprint to any wrench corner, positive outside; footprint-local."""
    best = math.inf
    for xy in wrench_corners(wrench):
        lx, ly, _ = to_local(load, [xy[0], xy[1], load[2]])
        dx, dy = abs(lx) - BEAM[0] / 2, abs(ly) - BEAM[1] / 2
        out = math.hypot(max(dx, 0.0), max(dy, 0.0)) if (dx > 0 or dy > 0) else max(dx, dy)
        best = min(best, out)
    return best
