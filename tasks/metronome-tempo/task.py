"""Set a wind-up metronome to 60 beats per minute and start it. A pyramid-shaped mechanical metronome (22.5 cm tall,
its front cover off) stands on the table in front of the arms, its scale facing them. Its pendulum, a thin steel rod
pivoting low in the case with a fixed weight below the pivot, rests upright at the centre; a brass sliding weight on
the rod sits at the 120 mark of the tempo scale behind the rod (40 at the top to 208 at the bottom). The spring is
wound.

Task: slide the weight up the rod until its top edge is at the 60 mark, then swing the pendulum to one side and let
go, so that it ticks on its own. The rod is notched at every mark and the weight's sprung tooth clicks from notch to
notch when it is pushed along the rod with about 3 N; the pendulum swings freely, so pushing the weight sideways
swings the pendulum instead of sliding the weight. The tempo comes from the physics: where the weight sits sets the pendulum's period.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from metkit import Body, BoxShape, CylShape, as_numpy, joint, on_reset, on_step, rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Set the metronome to 60 beats per minute and start it ticking."

G = 9.81

# -- the metronome ----------------------------------------------------------------------------------------------------
# A pyramid wind-up metronome (for example the Wittner Maelzel 800 series: 22.5 x 13.4 x 12.3 cm, 500 g, 40-208
# beats per minute). Case frame: origin on the table at the centre of its base, x from its front (toward the arms)
# to its back, y to its left, z up. The pendulum swings about the case's x axis, in front of the scale plate.
CASE_XY = (0.33, 0.0)                 # where the case's base centre stands (world, m)
CASE_MASS = 0.410                     # kg: the case and clockwork (the whole metronome is 500 g)
PLINTH = (0.123, 0.134, 0.020)        # x (front to back), y, z of the base plinth
CASE_H = 0.225
X_ROD = -0.030                        # the rod's centre plane (case frame x)
X_PLATE = X_ROD + 0.0045              # the scale plate's face
Z_PIV = 0.075                         # the pivot's height above the table

# Pendulum (estimates, see README): a 4 x 0.8 mm steel strip from 45 mm below the pivot to 150 mm above it, a 70 g
# brass weight 40 mm below the pivot, and a 20 g sliding weight on the upper rod.
ROD_W, ROD_T = 0.004, 0.0008
ROD_L, ROD_LB = 0.150, 0.045
LAM = 7850.0 * ROD_W * ROD_T          # kg per m of rod
BOB_M, BOB_D, BOB_R, BOB_H = 0.070, 0.040, 0.014, 0.014
SL_M = 0.020                          # the sliding weight
SL_W, SL_H, SL_D = 0.022, 0.018, 0.012   # its width (y), height (along the rod) and depth (x, front of the rod)

# The weight's spring clip and the rod's notches. As on a real Maelzel metronome, the rod is notched at every scale mark
# and the weight's clip has a sprung tooth that clicks into the notches. Here the rod's front face carries a row of
# low ridges, one between every two adjacent marks, each a pair of 30 degree flanks meeting at a peak halfway between the
# marks, so that between two ridges, at every mark, is a V-shaped notch. The tooth (a 0.6 mm radius rounded steel
# edge on the weight's back) is pressed into the rod by the clip's spring (a drive pulling the weight back onto the rod,
# 2.8 N). It seats at the bottom of a notch, with the weight's top edge on that mark's line; to move the weight along
# the rod, the tooth has to ride up a flank and over a ridge against the spring and friction (0.3), which takes about
# 3 N (estimate). Contacts only: nothing snaps the weight anywhere.
CLIP_FORCE = 2.8                      # N (estimate)
CLIP_MU = 0.3                         # steel tooth on steel rod
K_CLIP = 500.0                        # N/m: the clip's spring is soft, so its force hardly changes over a ridge
CLIP_PRELOAD = CLIP_FORCE / K_CLIP    # m: the drive's target lies this far into the rod
FLANK_DEG = 30.0                      # the notches' flanks, from the rod's face
TOOTH_R = 0.0006                      # the tooth's rounded edge
TOOTH_W = 0.003                       # its width across the rod
STANDOFF = 0.002                      # the weight's body stands this far in front of the rod's face (clear of the ridges)
CLICK_FORCE = CLIP_FORCE * (math.tan(math.radians(FLANK_DEG)) + CLIP_MU) / (
    1 - CLIP_MU * math.tan(math.radians(FLANK_DEG)))   # N: the push along the rod that takes the tooth over a ridge

MARKS = (40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 63, 66, 69, 72, 76, 80, 84, 88, 92, 96, 100, 104, 108, 112, 116,
         120, 126, 132, 138, 144, 152, 160, 168, 176, 184, 192, 200, 208)   # the Maelzel scale
LABELS = (40, 60, 80, 100, 120, 160, 208)
A_CAL = 25.0                          # degrees: the swing the scale is calibrated at (a typical start; see README)
TARGET = 60
START_MARK = 120


def _ellip_k(m: float) -> float:
    """The complete elliptic integral of the first kind K(m), m = k^2 (arithmetic-geometric mean)."""
    a, b = 1.0, math.sqrt(1.0 - m)
    for _ in range(30):
        a, b = (a + b) / 2, math.sqrt(a * b)
    return math.pi / (2 * a)


def pendulum(s_c: float) -> tuple[float, float]:
    """(moment of inertia about the pivot, kg m^2; gravity's restoring torque per radian, N m) of the pendulum with the
    sliding weight's centre ``s_c`` (m) above the pivot."""
    i_rod = LAM * (ROD_L ** 3 + ROD_LB ** 3) / 3
    i_bob = BOB_M * BOB_D ** 2 + BOB_M * BOB_R ** 2 / 2
    i_sl = SL_M * s_c ** 2 + SL_M * (SL_W ** 2 + SL_H ** 2) / 12
    k = G * (BOB_M * BOB_D + LAM * ROD_LB ** 2 / 2 - LAM * ROD_L ** 2 / 2 - SL_M * s_c)
    return i_rod + i_bob + i_sl, k


def bpm(s_top: float, amp_deg: float = A_CAL) -> float:
    """Ticks per minute (two per period) with the weight's top edge ``s_top`` above the pivot, swinging ``amp_deg``."""
    inertia, k = pendulum(s_top - SL_H / 2)
    if k <= 0:
        return 0.0
    t0 = 2 * math.pi * math.sqrt(inertia / k)
    t = t0 * 2 / math.pi * _ellip_k(math.sin(math.radians(amp_deg) / 2) ** 2)
    return 120.0 / t


def mark_height(b: float) -> float:
    """Where a scale mark is: the weight's top edge (m above the pivot) at which it ticks ``b`` per minute."""
    lo, hi = 0.02, ROD_L
    for _ in range(80):
        mid = (lo + hi) / 2
        if bpm(mid) > b:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


MARK_Z = {b: mark_height(b) for b in MARKS}
SL_TOP_MIN = MARK_Z[208] - 0.006      # the weight's travel (its top edge): stops below 208 and above 40
SL_TOP_MAX = MARK_Z[40] + 0.006
SL_START = MARK_Z[START_MARK]
NOTCH_Z = {b: MARK_Z[b] - SL_H / 2 for b in MARKS}   # where the tooth (at the weight's mid-height) seats, on the rod


def ridges() -> list[tuple[float, float, float]]:
    """The ridges between adjacent notches, and one more beyond each end notch (208 and 40) so that those are notches
    too: (lower edge, upper edge, peak height above the rod's face), m along the rod from the pivot."""
    zs = sorted(NOTCH_Z.values())
    zs = [2 * zs[0] - zs[1]] + zs + [2 * zs[-1] - zs[-2]]
    t = math.tan(math.radians(FLANK_DEG))
    return [(a, b, (b - a) / 2 * t) for a, b in zip(zs, zs[1:])]


def notch_at(top: float) -> int:
    """The mark whose line is nearest the weight's top edge: the notch it is in (or would click into)."""
    return min(MARKS, key=lambda b: abs(MARK_Z[b] - top))

# -- scoring numbers ----------------------------------------------------------------------------------------------------
LINE = 0.0006                         # the scale's lines are 0.6 mm thick
SET_MARK = 60                         # "at 60": the weight in the 60 notch (its top edge nearest the 60 line)
NEAR_MARKS = (52, 69)                 # "near 60": in a notch from 52 to 69
AMP_MIN = 15.0                        # degrees: every swing in the window reaches this far to each side
PASSES = 4                            # centre passes counted in the window (3 intervals)
TEMPO = (56.0, 64.0)                  # beats per minute measured over those passes
WINDOW_S = 5.0
CONTROL_HZ = 20
WINDOW = int(WINDOW_S * CONTROL_HZ)   # control steps
CLEAR = 0.05                          # hands off: no part of either hand or wrist within 5 cm of the metronome
TIP_DEG = 20.0                        # the case tilted this far: it has been tipped over (or lifted), score 0 for good
UPRIGHT_DEG = 5.0                     # standing: tilted no more than this
MOVED = 0.02                          # m: the case's base slid further than this from its place: score 0
TURNED_DEG = 10.0
HOLD_STEPS = 20

# -- the parts ---------------------------------------------------------------------------------------------------------
WOOD = (0.30, 0.16, 0.08)
WOOD2 = (0.24, 0.12, 0.06)
PLATE = (0.92, 0.88, 0.74)
INK = (0.08, 0.08, 0.08)
STEEL = (0.75, 0.76, 0.78)
BRASS = (0.80, 0.64, 0.28)

_SEG = {"0": "abcdef", "1": "bc", "2": "abged", "3": "abgcd", "4": "fgbc", "5": "afgcd", "6": "afgedc", "7": "abc",
        "8": "abcdefg", "9": "abcdfg"}


def _digits(text: str, y_right: float, z: float, x: float, h: float = 0.004, stroke: float = 0.0005) -> list:
    """Seven-segment digits drawn on the plate (visual only), the last digit's right edge at ``y_right`` (y falls to
    the viewer's right), centred on height ``z``."""
    w = h * 0.55
    out = []
    for i, ch in enumerate(reversed(text)):
        yr = y_right + i * (w + 0.0012)          # this digit's right edge
        yl = yr + w
        yc = (yl + yr) / 2
        seg = {"a": (yc, z + h / 2, w, stroke), "g": (yc, z, w, stroke), "d": (yc, z - h / 2, w, stroke),
               "f": (yl, z + h / 4, stroke, h / 2), "b": (yr, z + h / 4, stroke, h / 2),
               "e": (yl, z - h / 4, stroke, h / 2), "c": (yr, z - h / 4, stroke, h / 2)}
        for s in _SEG[ch]:
            cy, cz, sw, sh = seg[s]
            out.append(BoxShape((x, cy, cz), (0.0002, sw, sh), INK, collide=False))
    return out


def _case() -> Body:
    px, py, pz = PLINTH
    shapes = [BoxShape((0.0, 0.0, pz / 2), PLINTH, WOOD)]
    n = 8                                       # the pyramid behind the scale plate, as stacked slabs
    for i in range(n):
        z0, z1 = pz + i * (CASE_H - pz) / n, pz + (i + 1) * (CASE_H - pz) / n
        f = (i + 0.5) / n
        back = 0.050 - 0.035 * f                # its back face, tapering toward the top
        width = 0.112 - 0.080 * f
        shapes.append(BoxShape(((X_PLATE + back) / 2 + 0.0015, 0.0, (z0 + z1) / 2),
                               (back - X_PLATE - 0.003, width, z1 - z0), WOOD))
    plate_z0, plate_z1 = 0.088, CASE_H - 0.008
    shapes.append(BoxShape((X_PLATE + 0.0015, 0.0, (plate_z0 + plate_z1) / 2), (0.003, 0.050, plate_z1 - plate_z0),
                           PLATE))
    # the lower front of the case, in front of the pivot and the fixed weight (the cover's catch sits on it)
    shapes.append(BoxShape(((-px / 2 + 0.006 + X_ROD - 0.004) / 2, 0.0, (pz + 0.088) / 2),
                           (X_ROD - 0.004 - (-px / 2 + 0.006), 0.100, 0.088 - pz), WOOD2))
    xf = X_PLATE - 0.0001
    for b in MARKS:                             # the scale: a 0.6 mm line per mark, to the right of the rod
        zb = Z_PIV + MARK_Z[b]
        long = b in LABELS
        shapes.append(BoxShape((xf, -0.0115 - (0.0045 if long else 0.003), zb), (0.0002, 0.015 if long else 0.012,
                                                                                   LINE), INK, collide=False))
        if long:
            shapes += _digits(str(b), -0.0290, zb, xf, h=0.0030, stroke=0.0004)
    return Body.build("metronome_case", pos=(*CASE_XY, TOP), mass=CASE_MASS, color=WOOD, shapes=shapes,
                      friction=(0.6, 0.5), com=(0.010, 0.0, 0.065), solver_iterations=32)


def _rod() -> Body:
    """The pendulum: the steel rod and the fixed weight below the pivot. Frame: origin at the pivot, z up the rod."""
    mass = BOB_M + LAM * (ROD_L + ROD_LB)
    zc = (-BOB_M * BOB_D + LAM * (ROD_L ** 2 - ROD_LB ** 2) / 2) / mass
    i_piv, _ = pendulum(0.0)
    i_piv -= SL_M * (SL_W ** 2 + SL_H ** 2) / 12           # pendulum() includes the sliding weight's own term
    ixx = i_piv - mass * zc ** 2
    izz = BOB_M * (BOB_R ** 2 / 4 + BOB_H ** 2 / 12) + LAM * (ROD_L + ROD_LB) * ROD_W ** 2 / 12
    shapes = [BoxShape((0.0, 0.0, (ROD_L - ROD_LB) / 2), (ROD_T, ROD_W, ROD_L + ROD_LB), STEEL),
              CylShape((0.0, 0.0, -BOB_D), BOB_R, BOB_H, BRASS, collide=False, axis="X")]
    face, th, phi = -ROD_T / 2, 0.0004, math.radians(FLANK_DEG)
    for lo, hi, h in ridges():                # each ridge: two thin steel plates along its flanks, inside the rod
        m, length = (lo + hi) / 2, (hi - lo) / 2 / math.cos(phi)
        for z0, sgn in ((lo, -1.0), (hi, 1.0)):   # the lower flank (rising toward the peak), then the upper one
            mid = np.array([face - h / 2, (z0 + m) / 2])          # the flank's middle (x, z)
            normal = np.array([math.cos(phi), -sgn * math.sin(phi)])  # into the rod
            cx, cz = mid + th / 2 * normal
            shapes.append(BoxShape((float(cx), 0.0, float(cz)), (th, ROD_W, length), STEEL, pitch=sgn * phi))
    return Body.build("pendulum", pos=(CASE_XY[0] + X_ROD, CASE_XY[1], TOP + Z_PIV), mass=mass, color=STEEL,
                      shapes=shapes, friction=(CLIP_MU, CLIP_MU), com=(0.0, 0.0, zc), inertia=(ixx, ixx, izz),
                      damping=(0.0, 0.0), contact_offset=0.0005, solver_iterations=64, ignore=("metronome_case",))


# The sliding weight. Frame: origin on the rod's centre line at the weight's mid-height, when its tooth is seated in a
# notch. Its 12 mm deep body stands 2 mm in front of the rod; the tooth (a rounded edge across the rod on a short stem)
# reaches from it to the rod's face; cheeks either side of the rod and a 1 mm strap behind it keep it on the rod.
TOOTH_X = -ROD_T / 2 - TOOTH_R / math.sin(math.radians(90.0 - FLANK_DEG))   # the tooth's centre, seated in a notch
SL_FRONT = -ROD_T / 2 - STANDOFF - SL_D                                     # the weight's front face
SL_BACK = ROD_T / 2 + 0.0022                                                # the back of its strap
SL_COM_X = -0.0066


def _slider() -> Body:
    h = ROD_T / 2
    face = -h - STANDOFF
    shapes = [BoxShape((face - SL_D / 2, 0.0, 0.0), (SL_D, SL_W, SL_H), BRASS),
              BoxShape(((face + h + 0.0012) / 2, 0.007, 0.0), (h + 0.0012 - face, 0.008, SL_H), BRASS),
              BoxShape(((face + h + 0.0012) / 2, -0.007, 0.0), (h + 0.0012 - face, 0.008, SL_H), BRASS),
              BoxShape((h + 0.0017, 0.0, 0.0), (0.001, SL_W, SL_H), BRASS),
              BoxShape(((face + TOOTH_X) / 2, 0.0, 0.0), (TOOTH_X - face, TOOTH_W, 0.0008), STEEL),
              CylShape((TOOTH_X, 0.0, 0.0), TOOTH_R, TOOTH_W, STEEL, axis="Y"),
              BoxShape((face - SL_D - 0.0001, 0.0, SL_H / 2 - 0.0008), (0.0002, SL_W, 0.0006), INK, collide=False)]
    inertia = (SL_M * (SL_W ** 2 + SL_H ** 2) / 12, SL_M * (SL_D ** 2 + SL_H ** 2) / 12,
               SL_M * (SL_W ** 2 + SL_D ** 2) / 12)
    zc = SL_START - SL_H / 2
    return Body.build("sliding_weight", pos=(CASE_XY[0] + X_ROD, CASE_XY[1], TOP + Z_PIV + zc), mass=SL_M,
                      color=BRASS, shapes=shapes, friction=(CLIP_MU, CLIP_MU), com=(SL_COM_X, 0.0, 0.0),
                      inertia=inertia, damping=(0.0, 0.0), contact_offset=0.0005, solver_iterations=64,
                      ignore=("metronome_case",))


OBJECTS = [_case(), _rod(), _slider()]

# The pivot: the rod turns freely about the case's x axis (a frictionless pivot; see README).
joint("pivot", "metronome_case", "pendulum", anchor0=(X_ROD, 0.0, Z_PIV), anchor1=(0.0, 0.0, 0.0), free=("rotX",))
# The weight on the rod: it slides along the rod (z) between its stops; its clip pulls its pad onto the rod's front face
# (x) and the pad's contact friction holds it; it cannot turn on the rod or move across it.
joint("clip", "pendulum", "sliding_weight", anchor0=(0.0, 0.0, 0.0), anchor1=(0.0, 0.0, 0.0), collide=True,
      limits={"transZ": (SL_TOP_MIN - SL_H / 2, SL_TOP_MAX - SL_H / 2), "transX": (-0.003, 0.003)},
      drives={"transX": (K_CLIP, 20.0, CLIP_PRELOAD)})


# -- geometry (pure; used both in simulation and by the offline tests) ------------------------------------------------

def swing_deg(case_r, rod_r) -> float:
    """The pendulum's angle from the case's centre line (degrees, positive with its top toward the case's right, -y)."""
    r = np.asarray(case_r, float).T @ np.asarray(rod_r, float)
    return math.degrees(math.atan2(r[2, 1], r[1, 1]))


def weight_top(rod_pos, rod_r, slider_pos) -> float:
    """The sliding weight's top edge, m above the pivot along the rod."""
    return float((np.asarray(slider_pos, float) - np.asarray(rod_pos, float)) @ np.asarray(rod_r, float)[:, 2]
                 + SL_H / 2)


def tilt_deg(case_r) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(np.asarray(case_r, float)[2, 2])))))


def yaw_deg(case_r) -> float:
    r = np.asarray(case_r, float)
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


CASE_HALF = np.array([PLINTH[0] / 2, PLINTH[1] / 2, CASE_H / 2])


def _box_distance(points, centre, r, half) -> np.ndarray:
    local = (np.asarray(points, float) - centre) @ r
    return np.linalg.norm(np.maximum(np.abs(local) - half, 0.0), axis=1)


def clearance(points, case_pos, case_r, rod_pos, rod_r, slider_pos) -> float:
    """The least distance (m) from the robot's hand and wrist points to the metronome: the case's bounding box, the rod
    and the sliding weight."""
    if points is None or len(points) == 0:
        return float("inf")
    pts = np.asarray(points, float)
    case_r, rod_r = np.asarray(case_r, float), np.asarray(rod_r, float)
    d_case = _box_distance(pts, np.asarray(case_pos, float) + case_r @ np.array([0.0, 0.0, CASE_H / 2]), case_r,
                           CASE_HALF)
    a = np.asarray(rod_pos, float) - ROD_LB * rod_r[:, 2]
    d = (ROD_L + ROD_LB) * rod_r[:, 2]
    t = np.clip((pts - a) @ d / (d @ d), 0.0, 1.0)
    d_rod = np.linalg.norm(pts - (a + t[:, None] * d), axis=1) - ROD_W / 2
    centre = np.asarray(slider_pos, float) + rod_r @ np.array([(SL_FRONT + SL_BACK) / 2, 0.0, 0.0])
    d_sl = _box_distance(pts, centre, rod_r, np.array([(SL_BACK - SL_FRONT) / 2, SL_W / 2, SL_H / 2]))
    return float(min(d_case.min(), d_rod.min(), d_sl.min()))


def analyse_swing(angles, clear) -> dict:
    """Free swinging over the window: ``angles`` (degrees) and ``clear`` (hands-off flags) at each control step of the
    last WINDOW steps. Returns the centre passes, the smallest swing between them, the tempo, and whether it counts."""
    th = np.asarray(angles, float)
    out = {"passes": 0, "amp": 0.0, "bpm": 0.0, "free": False}
    if len(th) < WINDOW or not all(clear):
        return out
    idx = [i for i in range(1, len(th)) if (th[i - 1] < 0.0 <= th[i]) or (th[i - 1] >= 0.0 > th[i])]
    times = [(i - 1) + th[i - 1] / (th[i - 1] - th[i]) for i in idx]
    out["passes"] = len(idx)
    if len(idx) < PASSES:
        return out
    # every swing out to a side and back between two passes (and the swings before the first and after the last)
    edges = [0] + idx + [len(th)]
    peaks = [float(np.abs(th[a:b]).max()) for a, b in zip(edges[:-1], edges[1:]) if b > a]
    inner = peaks[1:-1]                       # the swings wholly inside the window
    out["amp"] = min(inner) if inner else 0.0
    out["bpm"] = 60.0 * CONTROL_HZ * (len(times) - 1) / (times[-1] - times[0])
    out["free"] = out["amp"] >= AMP_MIN
    return out


# -- the score, step by step ----------------------------------------------------------------------------------------------

_EP: dict = {}
_DEBUG = os.environ.get("MET_DEBUG", "0") not in ("", "0")
CASE_START = np.array([CASE_XY[0], CASE_XY[1], TOP])


@on_reset
def _reset_state() -> None:
    _EP.clear()
    _EP.update(angles=[], clear=[], tipped=False, score=0.0, steps=0, info={})


_reset_state()


def observe(case_pos, case_r, rod_pos, rod_r, slider_pos, hand_points) -> float:
    """One control step: record the swing and the hands' clearance, and work out the score."""
    case_pos, case_r = np.asarray(case_pos, float), np.asarray(case_r, float)
    th = swing_deg(case_r, rod_r)
    top = weight_top(rod_pos, rod_r, slider_pos)
    c = clearance(hand_points, case_pos, case_r, rod_pos, rod_r, slider_pos)
    _EP["angles"].append(th)
    _EP["clear"].append(c >= CLEAR)
    del _EP["angles"][:-WINDOW], _EP["clear"][:-WINDOW]
    tilt = tilt_deg(case_r)
    _EP["tipped"] |= tilt > TIP_DEG
    slid = float(np.hypot(*(case_pos[:2] - CASE_START[:2])))
    moved = tilt > UPRIGHT_DEG or slid > MOVED or abs(yaw_deg(case_r)) > TURNED_DEG
    sw = analyse_swing(_EP["angles"], _EP["clear"])
    notch = notch_at(top)
    is_set = notch == SET_MARK
    near = NEAR_MARKS[0] <= notch <= NEAR_MARKS[1]
    started = sw["free"]
    if _EP["tipped"] or moved:
        score = 0.0
    elif started and is_set and TEMPO[0] <= sw["bpm"] <= TEMPO[1]:
        score = 1.0
    elif started and near:
        score = 0.75
    elif is_set:
        score = 0.5
    elif started or near:
        score = 0.25
    else:
        score = 0.0
    _EP.update(score=score, steps=_EP["steps"] + 1,
               info={"swing": th, "top": top, "clear": c, "tilt": tilt, "slid": slid, "set": is_set, "near": near,
                     **sw})
    return score


def success(objects) -> bool:
    """True only on steps where score() is 1.0. The runtime then ends the episode after its own 1 s of this."""
    return _EP["score"] >= 1.0


def score(objects) -> float:
    """1.0 while: the weight's top edge is on or between the 58 and 63 lines, and over the last 5 s no part of either
    hand or wrist came within 5 cm of the metronome while the pendulum passed the centre at least 4 times, every swing
    reaching 15 degrees or more to each side, at a measured 56-64 per minute. 0.75: swinging so with the weight on or
    between the 52 and 69 lines; 0.5: the weight set (58-63) but not swinging so; 0.25: swinging so with the weight
    elsewhere, or the weight at 52-69 and not swinging. 0 if the case is tilted over 5 degrees, slid over 2 cm or turned
    over 10 degrees, and for the rest of the trial once it has tilted past 20 degrees."""
    return float(_EP["score"])


# -- in the simulator ---------------------------------------------------------------------------------------------------

_HULLS: dict = {}
HAND_LINKS = ("link4", "link5", "gripper", "tip_left", "tip_right")


def _hand_points(base, origin) -> np.ndarray:
    """Points on both hands and wrists (the vertices of their collision hulls, world frame)."""
    if not _HULLS:
        from yam_arena.analysis import link_hulls

        hulls = link_hulls()
        for link in HAND_LINKS:
            if link in hulls:
                _HULLS[link] = np.vstack([h.vertices for h in hulls[link]])
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        poses = as_numpy(robot.data.body_link_pose_w)[0]
        for j, n in enumerate(robot.body_names):
            link = n.rsplit("/", 1)[-1]
            if link in _HULLS:
                out.append((_HULLS[link] @ rotation(poses[j][3:7]).T) + poses[j][:3] - origin)
    return np.vstack(out) if out else np.zeros((0, 3))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])

    def pose(name):
        p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
        return p[:3] - origin, rotation(p[3:7])

    case_pos, case_r = pose("metronome_case")
    rod_pos, rod_r = pose("pendulum")
    slider_pos, _ = pose("sliding_weight")
    observe(case_pos, case_r, rod_pos, rod_r, slider_pos, _hand_points(base, origin))
    if _DEBUG and _EP["steps"] % 5 == 0:
        i = _EP["info"]
        print(f"[met] step {_EP['steps']} score {_EP['score']} swing {i['swing']:.1f} top {1000 * i['top']:.1f} "
              f"(set {i['set']} near {i['near']}) clear {i['clear']:.3f} tilt {i['tilt']:.1f} slid "
              f"{1000 * i['slid']:.1f} passes {i['passes']} amp {i['amp']:.1f} bpm {i['bpm']:.1f}",
              file=sys.stderr, flush=True)

