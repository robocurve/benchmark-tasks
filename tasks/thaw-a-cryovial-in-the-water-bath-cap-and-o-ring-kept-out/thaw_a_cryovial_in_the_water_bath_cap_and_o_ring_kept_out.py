"""Thaw a frozen cryovial in a 37 C water bath, swirling it with the cap kept out (ATCC handling; Gibco Thawing Cells).

On the bench: a Corning 2 mL cryovial (external thread, silicone washer in an orange cap) stands frozen on a mat's
circle; a Thermo Precision GP 02 water bath stands lid off. The vial is held by its cap in the water with the water line
between the contents mark (15 mm) and the cap mark (33 mm), swirled, lifted out and stood back on the circle.

On the kit: the vial and the bath are free bodies (the bath a heavy one); the water is a drawn surface at a fixed height
with no collider. Scoring is a clock on the marks: the dip runs while the 15 mm mark is under the water; any point of
the 33 mm mark at or under the water is a latched zero; the swirl is the vial's sideways travel in each 5 s of the dip.

Demo: the right hand grips the cap with the hand pitched 45 deg, carries the vial over the rim and turns to hold it
from the bath's right side (fingers in front of and behind the cap), lowers it until the water stands at the 24 mm mark
and moves it in 16 mm circles for 72 s, re-correcting its height every circle, then lifts it out, stands it on the
circle and lets go. Through the dip the idle left arm holds its wrist camera at the bath's left side, 280 mm from the
vial and looking down over the rim at the marks and the water line, with a 5 mm grid on the far wall behind.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import ourkit as k  # noqa: E402
from ourkit import demo as D  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Thaw the frozen vial in the 37 C water bath, swirling it with the contents under the water and the cap above it for about 1 to 2 minutes, then stand it back on the circle."
R = math.radians
HZ = 20.0
DT = 1.0 / HZ

# -- the cryovial (vial frame: origin at the base centre, z up) -------------------------------------------------------
VIAL_H = 0.0487  # Corning 430659, cap on
BODY_D, CAP_D = 0.0127, 0.0137
CAP_H = 0.013
CAP_Z = VIAL_H - CAP_H  # the cap's lower edge (the seal is inside the cap)
VIAL_KG = 0.0035  # vial with 1.0 mL frozen medium
LOW_MARK, HIGH_MARK = 0.015, 0.033  # contents top; 2.7 mm below the cap
GRIP_H = 0.0455  # the hand holds the cap's upper half here

# -- the bath (bath frame: origin at the housing's base centre) -------------------------------------------------------
HOUSING = (0.199, 0.230)  # x deep, y wide (Precision GP 02: 230 x 199 x 233 mm with its lid)
TANK = (0.138, 0.155)  # inside the tank, x and y
FLOOR_Z, RIM_Z = 0.050, 0.170  # tank floor and rim above the bench
WATER_Z = 0.143  # 2.0 L in the tank
BATH_KG = 6.0

# -- the layout -----------------------------------------------------------------------------------------------------
BATH_XY = (0.36, -0.10)
MAT = (0.12, 0.12)
MAT_C = (0.23, 0.09)
MAT_T = 0.002
Z0 = TOP + MAT_T
SPAWN = 0.0005
CIRCLE_D = 0.050

# -- scoring --------------------------------------------------------------------------------------------------------
DIP1_S, DIP2_S, THAW_S, MAX_S = 15.0, 30.0, 45.0, 120.0
SWIRL_MM, SWIRL_S = 10.0, 5.0
UPRIGHT = R(10)
CLEAR_MM, STILL_MM, STILL_S = 20.0, 2.0, 1.0
LET_GO_S = 0.5  # no finger within 5 mm of the vial this long while it is in the water: dropped in
WATER_W = TOP + WATER_Z

PP, CAP_C, MARK, RED = (0.93, 0.93, 0.90), (0.95, 0.50, 0.12), (0.05, 0.05, 0.05), (0.85, 0.08, 0.10)
MARK_W = 0.003  # each mark a 3 mm band (legible to the watching wrist camera at 224 px)
STEEL, WATER_C, MATC, INK = (0.74, 0.75, 0.77), (0.38, 0.58, 0.86), (0.97, 0.97, 0.97), (0.15, 0.15, 0.15)


def _round(prefix: str, r: float, z0: float, z1: float, color, ghost: bool = False) -> list:
    """A round prism as 8 slabs across its diameter at 22.5 deg steps (a 16-sided outline)."""
    w = 2 * r * math.tan(math.pi / 16)
    return [k.box((0, 0, (z0 + z1) / 2), (2 * r, w, z1 - z0), color, turn=(0, 0, i * math.pi / 8),
                  label=f"{prefix}{i}", ghost=ghost) for i in range(8)]


def _ring(prefix: str, r: float, z: float, color, t: float = 0.0008, h: float = 0.0008, at=(0.0, 0.0)) -> list:
    """A thin drawn ring (16 tangent strips), no collider."""
    side = 2 * r * math.tan(math.pi / 16)
    out = []
    for i in range(16):
        a = i * math.pi / 8
        out.append(k.box((at[0] + r * math.cos(a), at[1] + r * math.sin(a), z), (t, side, h), color,
                         turn=(0, 0, a), ghost=True, label=f"{prefix}{i}"))
    return out


VIAL_SOLID = _round("body", BODY_D / 2, 0.0, CAP_Z, PP) + _round("cap", CAP_D / 2, CAP_Z, VIAL_H, CAP_C)
VIAL_BOXES = VIAL_SOLID + _ring("lo", BODY_D / 2 + 0.0002, LOW_MARK, MARK, t=0.0006, h=MARK_W) + \
    _ring("hi", BODY_D / 2 + 0.0002, HIGH_MARK, RED, t=0.0006, h=MARK_W)
VIAL = k.mech("vial", (MAT_C[0], MAT_C[1], Z0 + SPAWN), VIAL_BOXES, VIAL_KG, color=PP, surface=0.8)


def _bath_boxes() -> list:
    hx, hy = HOUSING[0] / 2, HOUSING[1] / 2
    tx, ty = TANK[0] / 2, TANK[1] / 2
    wz, wh = (FLOOR_Z + RIM_Z) / 2, RIM_Z - FLOOR_Z
    out = [k.box((0, 0, FLOOR_Z / 2), (HOUSING[0], HOUSING[1], FLOOR_Z), STEEL, label="base"),
           k.box(((hx + tx) / 2, 0, wz), (hx - tx, HOUSING[1], wh), STEEL, label="wallback"),
           k.box((-(hx + tx) / 2, 0, wz), (hx - tx, HOUSING[1], wh), STEEL, label="wallfront"),
           k.box((0, (hy + ty) / 2, wz), (TANK[0], hy - ty, wh), STEEL, label="wallleft"),
           k.box((0, -(hy + ty) / 2, wz), (TANK[0], hy - ty, wh), STEEL, label="wallright"),
           ]
    # the water: a flat drawn surface across the tank at its level (no collider): the vial shows where it comes through
    # drawn as 2 mm strips every 6 mm across the tank (the kit draws opaque only): the vial's submerged part and its
    # 15 mm mark show between the strips, the water level reads from them
    n_strip = int(TANK[1] / 0.006)
    for i in range(n_strip):
        y = -TANK[1] / 2 + 0.003 + i * 0.006
        out.append(k.box((0, y, WATER_Z - 0.0005), (TANK[0] - 0.002, 0.002, 0.001), WATER_C, ghost=True,
                         label=f"waterstrip{i}"))
    # and a 3 mm band round the tank's inside walls at the same level
    for n, (c, sz) in enumerate((((tx - 0.0005, 0), (0.001, TANK[1], 0.003)), ((-tx + 0.0005, 0), (0.001, TANK[1], 0.003)),
                                 ((0, ty - 0.0005), (TANK[0], 0.001, 0.003)), ((0, -ty + 0.0005), (TANK[0], 0.001, 0.003)))):
        out.append(k.box((c[0], c[1], WATER_Z - 0.0015), sz, WATER_C, ghost=True, label=f"water{n}"))
    # a 5 mm grid strip on the far (right, -y) inner wall at the water line, behind the vial as the watching camera sees it
    for i in range(9):
        out.append(k.box((-0.020 + i * 0.005, -ty + 0.0004, WATER_Z + 0.009), (0.0006, 0.0006, 0.042), INK, ghost=True,
                         label=f"wallv{i}"))
    for i in range(9):
        out.append(k.box((0.0, -ty + 0.0004, WATER_Z - 0.010 + i * 0.005), (0.040, 0.0006, 0.0006), INK, ghost=True,
                         label=f"wallh{i}"))
    # the 10 mm grid sticker on the tank floor, lines across the bath's width every 10 mm (read from above)
    for i in range(-5, 6):
        out.append(k.box((0, i * 0.010, FLOOR_Z + 0.0003), (0.100, 0.0008, 0.0006), INK, ghost=True,
                         label=f"grid{i + 5}"))
    # the 50 mm circle drawn on the mat (carried on the bath, which never moves)
    rel = (MAT_C[0] - BATH_XY[0], MAT_C[1] - BATH_XY[1])
    out += _ring("circle", CIRCLE_D / 2, MAT_T + 0.0003, INK, t=0.0015, h=0.0004, at=rel)
    return out


BATH = k.mech("bath", (BATH_XY[0], BATH_XY[1], TOP + SPAWN), _bath_boxes(), BATH_KG, color=STEEL, surface=0.6)
MATBOX = Box("mat", pos=(MAT_C[0], MAT_C[1], TOP + MAT_T / 2), size=(MAT[0], MAT[1], MAT_T), color=MATC)
OBJECTS = [MATBOX, BATH, VIAL]

# -- the rules and the scorer ---------------------------------------------------------------------------------------
WET, DROPPED = k.Sticky(), k.Sticky()
END = k.Settle(STILL_S, STILL_MM / 1000)
RUN: dict = {}
REC = k.run_record.RunRecord(__file__, "vial")


def _fresh() -> dict:
    return {"dips": [], "on": False, "t": 0.0, "swirled": True, "blk_t": 0.0, "ylo": 0.0, "yhi": 0.0,
            "loose": 0.0}


RUN.update(_fresh())


def _ring_z(st, z: float) -> np.ndarray:
    v = st["vial"]
    r = BODY_D / 2
    return np.array([v.to_world((r * math.cos(a), r * math.sin(a), z))[2] for a in np.arange(8) * math.pi / 4])


def base(st) -> np.ndarray:
    return st["vial"].to_world((0, 0, 0))


def in_tank(st) -> bool:
    """The vial's axis inside the tank's opening (anywhere else the water height means nothing)."""
    c = st["vial"].to_world((0, 0, LOW_MARK))
    return abs(float(c[0]) - BATH_XY[0]) <= TANK[0] / 2 and abs(float(c[1]) - BATH_XY[1]) <= TANK[1] / 2


def contents_under(st) -> bool:
    return in_tank(st) and float(_ring_z(st, LOW_MARK).max()) < WATER_W


def cap_wet(st) -> bool:
    return in_tank(st) and float(_ring_z(st, HIGH_MARK).min()) <= WATER_W


def in_water(st) -> bool:
    return in_tank(st) and float(base(st)[2]) < WATER_W


def dips() -> list:
    """Every dip so far as (seconds, swirled), the one under way included."""
    out = list(RUN["dips"])
    if RUN["on"]:
        out.append((RUN["t"], RUN["swirled"]))
    return out


def on_circle(st) -> bool:
    b = base(st)
    return float(np.hypot(b[0] - MAT_C[0], b[1] - MAT_C[1])) <= CIRCLE_D / 2 and \
        math.radians(st["vial"].tilt_deg()) <= UPRIGHT and abs(float(b[2]) - Z0) <= 0.003


def grade(st) -> float:
    if WET or DROPPED:
        return 0.0
    ds = dips()
    longest = max((t for t, _ in ds), default=0.0)
    thawed = longest <= MAX_S + 1e-9 and any(THAW_S - 1e-9 <= t <= MAX_S + 1e-9 and sw for t, sw in ds) \
        and not in_water(st)
    return k.ladder((0.25, longest >= DIP1_S - 1e-9), (0.5, longest >= DIP2_S - 1e-9), (0.75, thawed),
                    (1.0, on_circle(st) and END.ok))


def score(states) -> float:
    return grade(states)


def success(states) -> bool:
    return grade(states) >= 1.0


def distance(states) -> float:
    b = base(states)
    return float(np.hypot(b[0] - BATH_XY[0], b[1] - BATH_XY[1]))


def _vial_points(st) -> np.ndarray:
    return k.score.surface_points(st["vial"], "", VIAL_SOLID, 0.005)


@k.on_reset
def _reset(states) -> None:
    for s in (WET, DROPPED, END):
        s.reset()
    RUN.clear()
    RUN.update(_fresh())
    REC.reset()


@k.on_step
def _watch(st) -> None:
    v = st["vial"]
    if cap_wet(st):
        WET.trip("the water reached the cap mark")
    y = float(v.to_world((0, 0, LOW_MARK))[1])
    if contents_under(st):
        if not RUN["on"]:
            RUN.update(on=True, t=0.0, swirled=True, blk_t=0.0, ylo=y, yhi=y)
        RUN["t"] += DT
        RUN["ylo"], RUN["yhi"] = min(RUN["ylo"], y), max(RUN["yhi"], y)
        if RUN["t"] - RUN["blk_t"] >= SWIRL_S - 1e-9:  # a full 5 s of the dip: did it move 10 mm side to side?
            if RUN["yhi"] - RUN["ylo"] < SWIRL_MM / 1000:
                RUN["swirled"] = False
            RUN.update(blk_t=RUN["t"], ylo=y, yhi=y)
    elif RUN["on"]:
        RUN["dips"].append((RUN["t"], RUN["swirled"]))
        RUN["on"] = False
    pts = _vial_points(st)
    if in_water(st) and k.hands_clear(pts, 5.0):
        RUN["loose"] += DT
        if RUN["loose"] >= LET_GO_S:
            DROPPED.trip("the vial let go in the bath")
    else:
        RUN["loose"] = 0.0
    END.push(np.asarray(v.pos), k.hands_clear(pts, CLEAR_MM))
    REC.tick(st, lambda: f"dip {RUN['t']:.1f} s swirled {RUN['swirled']} lo "
                         f"{(WATER_W - _ring_z(st, LOW_MARK).max()) * 1000:+.1f} hi "
                         f"{(_ring_z(st, HIGH_MARK).min() - WATER_W) * 1000:+.1f} mm, score {grade(st):g}")


# -- the demo ---------------------------------------------------------------------------------------------------------
G45 = D.Grip.deg(pitch=45)  # approach forward and down, fingers closing along y (the pick and the put-back)
G_SIDE = D.Grip.deg(yaw=90, pitch=45)  # over the bath: from the bath's right side, fingers front and back of the cap,
#                                         across the watching camera's line of sight
# the watching camera: the idle left arm's wrist camera (yam_arena.constants: D405 8 cm off the gripper link's -Y face,
# 134 mm behind the TCP, tilted 8 deg toward the fingers), posed outside the bath's left rim looking down at the waterline
G_WATCH = D.Grip.deg(yaw=-90, pitch=65)  # camera above the fingers' axis, far enough back that the fingers stay outside the rim
WATCH_DIST = 0.28  # m from the camera to the vial at the water line
TIMES: dict = {}
APP = np.asarray(G45.approach)
DIP_MARK = 0.024  # the water line held here: 9 mm inside each mark
SWIRL_R = 0.008  # 16 mm circles
CIRCLE_S, PER_CIRCLE = 2.5, 8
SWIRL_TIME = 72.0
PLUNGE_MARK = 0.036  # the plunge cheat holds the water here, at the cap
BATH_W = np.array([BATH_XY[0], BATH_XY[1], WATER_W])
G0 = np.array([MAT_C[0], MAT_C[1], Z0 + GRIP_H])  # the grip point on the standing vial
CARRY_Z = TOP + 0.245  # the TCP while carrying: the vial's base 30 mm over the rim


def _off(a: float) -> np.ndarray:
    return np.array([SWIRL_R * math.cos(a), SWIRL_R * math.sin(a), 0.0])


def thaw_plan(how: str) -> D.Plan:
    p = D.Plan("right")
    p.move(2.0, (MAT_C[0] - 0.06, MAT_C[1], TOP + 0.15), grip=G45, finger=D.gap_finger(0.03),
           label="in front of the bath, over the mat")
    D.grasp(p, lambda st: st["vial"].to_world((0, 0, GRIP_H)), CAP_D, grip=G45, lift=None, fallback=tuple(G0),
            label="the vial's cap")
    p.move(1.5, D.remember(p, "lift", lambda st: G0 + (0, 0, CARRY_Z - G0[2])), fallback=(G0[0], G0[1], CARRY_Z),
           label="lift the vial")
    over = np.array([BATH_XY[0] + SWIRL_R, BATH_XY[1], CARRY_Z])
    p.move(2.5, D.remember(p, "over", lambda st: over), fallback=tuple(over), grip=G_SIDE,
           label="over the bath, turned to hold it from the side")
    mark = PLUNGE_MARK if how == "plunge" else DIP_MARK
    D.corrected(p, 3.0, lambda st: BATH_W + _off(0.0), lambda st: st["vial"].to_world((0, 0, mark)), key="dip",
                max_fix=0.15, label="lower the vial into the water", fallback=tuple(over))
    TIMES["dip0"] = p.duration
    swirl = how != "still"
    n = int(round(SWIRL_TIME / CIRCLE_S))
    for i in range(n):
        if not swirl:
            p.hold(CIRCLE_S, "hold it in the water")
            continue
        D.corrected(p, CIRCLE_S / PER_CIRCLE, lambda st: BATH_W + _off(0.0),
                    lambda st: st["vial"].to_world((0, 0, mark)), key=f"c{i}", max_fix=0.006,
                    label="swirl", fallback=tuple(over))
        for j in range(1, PER_CIRCLE):
            a = j * 2 * math.pi / PER_CIRCLE
            p.move(CIRCLE_S / PER_CIRCLE,
                   D.remember(p, f"c{i}_{j}", lambda st, i=i, a=a: np.asarray(p.memo[f"c{i}"]) + _off(a) - _off(0.0)),
                   fallback=tuple(over), label="swirl")
    TIMES["dip1"] = p.duration
    p.move(2.0, D.remember(p, "out", lambda st: np.asarray(p.memo["_last"]) + (0, 0, 0.10)), fallback=tuple(over),
           label="lift it out")
    back = np.array([MAT_C[0] - BATH_XY[0] - SWIRL_R, MAT_C[1] - BATH_XY[1], 0.0])
    p.move(2.5, D.remember(p, "back", lambda st: np.asarray(p.memo["out"]) + back), fallback=tuple(G0 + (0, 0, 0.2)),
           grip=G45, label="back over the circle")
    D.corrected(p, 2.5, lambda st: np.array([MAT_C[0], MAT_C[1], Z0 + 0.015]), base_point, key="set1", max_fix=0.25,
                label="lower it over the circle", fallback=tuple(G0 + (0, 0, 0.02)))
    D.corrected(p, 1.5, lambda st: np.array([MAT_C[0], MAT_C[1], Z0 + 0.001]), base_point, key="set2", max_fix=0.03,
                label="stand it down", fallback=tuple(G0))
    p.hold(0.5, "settle")
    if how == "held":
        return p
    p.release(seconds=1.0)
    p.move(1.0, lambda st: np.asarray(p.memo["set2"]) - APP * 0.05, fallback=tuple(G0 - APP * 0.05),
           label="back off the cap")
    p.move(1.0, lambda st: np.asarray(p.memo["set2"]) - APP * 0.05 + (0, 0, 0.08),
           fallback=tuple(G0 - APP * 0.05 + (0, 0, 0.08)), label="up from the vial")
    p.home()
    return p


def base_point(st) -> np.ndarray:
    return base(st)


def watch_tcp() -> np.ndarray:
    """The left TCP that puts its wrist camera WATCH_DIST from the vial at the water line, looking along its axis."""
    r = np.asarray(G_WATCH.rotation)
    fwd = r @ np.array([0.0, math.sin(R(8)), math.cos(R(8))])
    target = np.array([BATH_XY[0] + SWIRL_R, BATH_XY[1], WATER_W + 0.004])
    cam = target - WATCH_DIST * fwd / np.linalg.norm(fwd)
    return cam + r @ np.array([0.0, 0.0, 0.134]) - r @ np.array([0.0, -0.08, 0.0])


def demo_plans(how: str) -> dict:
    r = thaw_plan(how)
    left = D.Plan("left")
    left.hold(max(0.1, TIMES["dip0"] - 4.0), "wait")
    left.move(3.5, tuple(watch_tcp()), grip=G_WATCH, finger=D.FINGER_CLOSED, label="watch the vial at the water line")
    left.hold(max(0.1, TIMES["dip1"] - left.duration), "watch the dip")
    left.home()
    return D.together(left, r)


def demo():
    return D.choose(demo=lambda: demo_plans("demo"), plunge=lambda: demo_plans("plunge"),
                    still=lambda: demo_plans("still"), held=lambda: demo_plans("held"))


# -- what the template tests read ---------------------------------------------------------------------------------
TEXT_NUMBERS = [("48.7 mm", VIAL_H), ("12.7 mm", BODY_D), ("13.7 mm", CAP_D), ("3.5 g", VIAL_KG),
                ("15 mm", LOW_MARK), ("33 mm", HIGH_MARK), ("170 mm", RIM_Z), ("143 mm", WATER_Z),
                ("138 mm", TANK[0]), ("155 mm", TANK[1]), ("10 mm", SWIRL_MM / 1000), ("5 s", SWIRL_S),
                ("15 s", DIP1_S), ("30 s", DIP2_S), ("45 s", THAW_S), ("120 s", MAX_S), ("50 mm", CIRCLE_D),
                ("10 deg", math.degrees(UPRIGHT)), ("20 mm", CLEAR_MM / 1000), ("2 mm", STILL_MM / 1000),
                ("1 s", STILL_S)]
CHEATS = {"none": 0.0, "plunge": 0.0, "still": 0.5, "held": 0.75}
HOLD: dict = {}
