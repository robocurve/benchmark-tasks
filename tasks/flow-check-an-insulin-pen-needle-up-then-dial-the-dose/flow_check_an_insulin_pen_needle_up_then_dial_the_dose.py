"""Prime an insulin pen with the needle up until the counter reads 0, then dial 24 units.

On the station table: a pen tray with a Novo Nordisk Demonstration Pen for FlexTouch (the maker's training pen: a
FlexTouch prefilled with test medium, 2-unit increments) lying in its groove, a NovoFine 32G 6 mm needle on, caps off,
needle away from the robot; a silicone mat to the left. The maker's steps (TRESIBA FlexTouch IFU, steps 7-10): turn the
dose selector to 2 units; hold the pen with the needle pointing up; press and hold the dose button until the counter
shows 0; a drop shows at the needle tip; then turn the selector to the prescribed dose.

On the kit: the pen is one free body. Its dose selector is a hinge about the pen's axis with a click every 15 degrees
(2 units a click), held between clicks by a spring whose target is the nearest click. Its dose button is a 2 mm slider
preloaded outward at 5.1 N (the maker's bench force). With the button at its stop, the pen's torque spring pushes out
10 units/s and turns the selector back toward 0; letting go stops it. Liquid is a ledger: 1 unit of air sits under the
needle end and leaves first only if the needle points up; a ghost drop then shows at the tip.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import ourkit as k  # noqa: E402
from ourkit import demo as D  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 2.0  # the 1.0 state already includes 2 s still; 2 s more so the video shows the end state
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Do the insulin pen's 2-unit flow check, then dial a 24-unit dose and hold the pen ready."
HZ = 20.0

# == the pen's mechanism (fidelity) ===================================================================================
CLICK = math.radians(15.0)  # rad per click of the dose selector
UNITS_PER_CLICK = 2  # a 2-unit-increment pen
CHECK_UNITS = 2  # the flow check dose
DOSE_UNITS = 24  # the prescribed dose
BUTTON_N = 5.1  # N on the dose button (Hemmingsen 2011: 5.1 +- 0.5 N)
BUTTON_TRAVEL = 0.002  # m
BUTTON_RATE = 500.0  # N/m: 5.1 N at rest, 6.1 N at the stop
AT_STOP = 0.0018  # m: the button counts as at its stop past this
RATE_UPS = 10.0  # units/s pushed out while the button is at its stop
AIR_UNITS = 1.0  # units of air under the needle end at the start
DIAL_RATE, DIAL_DAMP = 0.15, 0.002  # N m/rad toward the nearest click (0.02 N m to pass one), N m s/rad
DIAL_MAX = 22  # clicks the sim dial can reach (44 units)
UP_TOL = math.radians(30.0)  # needle within 30 degrees of straight up counts as up
MAX_PRESSES = 6  # the maker's limit on repeated flow checks

# == the scene (x forward from the arm bases' midpoint, y left, z up; metres) =========================================
SPAWN = 0.0005
PEN_X, PEN_Y = 0.31, 0.0
PEN_W, SEL_W = 0.024, 0.024  # barrel and dose selector across their ribs (eight ribs: four plates at 45-degree steps)
RIB_T = 0.010  # each plate's thickness: the section is a ribbed near-round, not a square
BODY = (-0.060, 0.065)  # body along the pen axis (X): button end -0.082, needle tip +0.081
SEL_L = 0.018
TIP_X, NEEDLE_L = 0.081, 0.006
PEN_KG = 0.020
TRAY_L, TRAY_W, TRAY_BASE, TRAY_RAIL_H, GROOVE_W = 0.200, 0.060, 0.008, 0.004, 0.028
TRAY_KG = 0.15
MAT = (0.300, 0.300, 0.003)
MAT_X, MAT_Y, MAT_KG = 0.30, 0.19, 0.30
PEN_Z0 = TOP + TRAY_BASE + PEN_W / 2 + SPAWN  # pen axis height resting in the groove

WHITE, GREY, BLUE, STEEL, TEAL, DROP = (0.93, 0.93, 0.92), (0.55, 0.57, 0.60), (0.20, 0.35, 0.70), (0.75, 0.76, 0.78), \
    (0.20, 0.55, 0.60), (0.70, 0.85, 1.0)

DIAL = k.hinge("dial", (BODY[0], 0, 0), "X", math.radians(-3.0), DIAL_MAX * CLICK + math.radians(3.0),
               [k.box((-SEL_L / 2, 0, 0), (SEL_L, SEL_W, RIB_T), BLUE, turn=(i * math.pi / 4, 0, 0),
                       label=f"pen_selector_{i}") for i in range(4)],
               rate=DIAL_RATE, target=0.0, damping=DIAL_DAMP, mass=0.10, weightless=True)
BUTTON = k.slider("button", (BODY[0] - SEL_L, 0, 0), "X", 0.0, BUTTON_TRAVEL,
                  [k.box((-0.002, 0, 0), (0.004, 0.010, 0.010), GREY, label="pen_button")],
                  rate=BUTTON_RATE, target=-BUTTON_N / BUTTON_RATE, damping=0.5, mass=0.10, weightless=True)
DROPLET = k.slider("drop", (0.070, 0, 0), "X", 0.0, 0.0125,
                   [k.box((0, 0, 0), (0.003, 0.003, 0.003), DROP, ghost=True, label="pen_drop")],
                   rate=2.0, target=0.0, damping=0.5, mass=0.02, weightless=True)
# -- the dose window: two seven-segment digits on a white bezel on the barrel's top, near the selector. The numerals are
#    drawn 14 mm tall (the real window's are about 4 mm) so the wrist camera reads them at 224 px; each segment is its own
#    labelled, collider-free box recoloured every step from the dial's counter.
WIN_X, WIN_Z = -0.040, PEN_W / 2 + 0.0004  # window centre along the pen; just proud of the barrel's top rib
DIG_H, DIG_W, SEG_T = 0.014, 0.008, 0.002  # digit height (along the pen, its top toward the needle), width, stroke
DIG_Y = {"t": 0.0055, "o": -0.0055}  # tens left, ones right, as the robot looks along the pen
INK, PAPER = (0.05, 0.05, 0.06), (0.95, 0.95, 0.93)
SEGS = {"a": (0.0, DIG_H / 2, DIG_W, SEG_T), "b": (DIG_W / 2, DIG_H / 4, SEG_T, DIG_H / 2),
        "c": (DIG_W / 2, -DIG_H / 4, SEG_T, DIG_H / 2), "d": (0.0, -DIG_H / 2, DIG_W, SEG_T),
        "e": (-DIG_W / 2, -DIG_H / 4, SEG_T, DIG_H / 2), "f": (-DIG_W / 2, DIG_H / 4, SEG_T, DIG_H / 2),
        "g": (0.0, 0.0, DIG_W, SEG_T)}  # (h across the digit, v along it, width, height) in the digit's own frame
DIGITS = {0: "abcdef", 1: "bc", 2: "abdeg", 3: "abcdg", 4: "bcfg", 5: "acdfg", 6: "acdefg", 7: "abc", 8: "abcdefg",
          9: "abcdfg"}


def _window_boxes() -> list:
    out = [k.box((WIN_X, 0.0, WIN_Z - 0.0002), (DIG_H + 0.006, 2 * DIG_W + 0.009, 0.0003), PAPER, ghost=True,
                 label="pen_dose_window")]
    for d, yc in DIG_Y.items():
        for name, (h, v, w, ht) in SEGS.items():  # digit up = +x (toward the needle), digit right = -y
            out.append(k.box((WIN_X + v, yc - h, WIN_Z), (ht, w, 0.0003), PAPER, ghost=True,
                             label=f"pen_seg_{d}_{name}"))
    return out


PEN = k.mech("pen", (PEN_X, PEN_Y, PEN_Z0), [
    *[k.box(((BODY[0] + BODY[1]) / 2, 0, 0), (BODY[1] - BODY[0], PEN_W, RIB_T), WHITE, turn=(i * math.pi / 4, 0, 0),
            label=f"pen_barrel_{i}") for i in range(4)],
    k.box((0.070, 0, 0), (0.010, 0.010, 0.010), TEAL, label="needle_hub"),
    k.box((TIP_X - NEEDLE_L / 2, 0, 0), (NEEDLE_L, 0.0008, 0.0008), STEEL, ghost=True, label="needle"),
    *_window_boxes()],
    PEN_KG, joints=[DIAL, BUTTON, DROPLET], color=WHITE, surface=0.8,
    points=(("tip", (TIP_X, 0, 0)), ("needle_mid", (TIP_X - NEEDLE_L / 2, 0, 0)), ("hub", (0.075, 0, 0))))

_RAIL_W = (TRAY_W - GROOVE_W) / 2
TRAY = k.mech("tray", (PEN_X, PEN_Y, TOP + SPAWN), [
    k.box((0, 0, TRAY_BASE / 2), (TRAY_L, TRAY_W, TRAY_BASE), WHITE, label="tray_base"),
    k.box((0, (GROOVE_W + _RAIL_W) / 2, TRAY_BASE + TRAY_RAIL_H / 2), (TRAY_L, _RAIL_W, TRAY_RAIL_H), WHITE,
          label="tray_rail_l"),
    k.box((0, -(GROOVE_W + _RAIL_W) / 2, TRAY_BASE + TRAY_RAIL_H / 2), (TRAY_L, _RAIL_W, TRAY_RAIL_H), WHITE,
          label="tray_rail_r")], TRAY_KG, color=WHITE, surface=0.5)
MATB = k.mech("mat", (MAT_X, MAT_Y, TOP + SPAWN), [k.box((0, 0, MAT[2] / 2), MAT, (0.96, 0.96, 0.96), label="mat")],
              MAT_KG, color=(0.96, 0.96, 0.96), surface=0.9)
OBJECTS = [TRAY, MATB, PEN]

# == the pen's own rules: clicks, delivery, the liquid ledger =========================================================
RUN: dict = {}
DEBUG = bool(__import__("os").environ.get("PEN_DEBUG"))


def clicks_of(angle: float) -> int:
    return int(round(angle / CLICK))


def counter(states) -> int:
    """What the dose window shows: the nearest click, in units (never below 0)."""
    return max(0, clicks_of(float(states["pen"].q["dial"]))) * UNITS_PER_CLICK


def needle_tilt(states) -> float:
    """Angle between the pen's needle direction and straight up (rad)."""
    ax = states["pen"].rotation()[:, 0]
    return math.acos(float(np.clip(ax[2], -1.0, 1.0)))


def show(value: int) -> None:
    """Draw ``value`` (0-99 units) in the dose window: ink on the lit segments, paper on the rest; no tens digit under 10."""
    value = int(max(0, min(99, value)))
    lit = {"t": DIGITS[value // 10] if value >= 10 else "", "o": DIGITS[value % 10]}
    for d in ("t", "o"):
        for name in SEGS:
            k.recolor(f"pen_seg_{d}_{name}", INK if name in lit[d] else PAPER)


def displayed(states) -> int:
    """What the window shows: the dial's click count in units; while the spring pushes a dose out, the units left."""
    if RUN.get("delivering") and RUN.get("units", 0.0) > 0:
        return int(math.ceil(RUN["units"] / UNITS_PER_CLICK - 1e-9)) * UNITS_PER_CLICK
    return counter(states)


def _fresh() -> dict:
    return {"units": 0.0, "delivering": False, "press_units": 0.0, "press_up": True, "presses": 0,
            "air": AIR_UNITS, "liquid": 0.0, "drop": False, "t": 0.0, "lifted": False}


RUN.update(_fresh())

# == the scorer ========================================================================================================
STILL_S, STILL_MM = 2.0, 2.0  # still: the pen moves under 2 mm over 2 s
CLEAR_MM = 30.0  # the right gripper at least 3 cm from the pen
NEEDLE_MM = 1.0  # needle points nearer than this to a gripper or a surface: touched
LIFT = 0.020  # the pen lifted at least 2 cm off the tray
FALL_Z = 0.040  # the pen's axis under 4 cm off the table outside the tray: fallen

BAD, SET2, FLOW = k.Sticky(), k.Ever(), k.Ever()
END = k.Settle(STILL_S, STILL_MM / 1000.0)
REC = k.run_record.RunRecord(__file__, "pen")


def pen_points(s) -> np.ndarray:
    return np.array([s.to_world((x, 0, 0)) for x in (BODY[0] - SEL_L - 0.004, BODY[0], 0.0, BODY[1], TIP_X)])


def grade(states) -> float:
    if BAD:
        return 0.0
    c = counter(states)
    dosed = bool(FLOW) and c == DOSE_UNITS
    return k.ladder((0.25, bool(SET2)), (0.5, bool(FLOW)), (0.75, dosed), (1.0, dosed and END.ok))


def score(states) -> float:
    return grade(states)


def success(states) -> bool:
    return grade(states) >= 1.0


def distance(states) -> float:
    return abs(counter(states) - DOSE_UNITS) + (0 if FLOW else 50)


def _surface_below(p) -> float:
    """Height of the surface under a world point: mat, tray or table."""
    x, y = float(p[0]), float(p[1])
    if abs(x - MAT_X) <= MAT[0] / 2 and abs(y - MAT_Y) <= MAT[1] / 2:
        return TOP + MAT[2]
    if abs(x - PEN_X) <= TRAY_L / 2 and abs(y - PEN_Y) <= TRAY_W / 2:
        return TOP + (TRAY_BASE if abs(y - PEN_Y) <= GROOVE_W / 2 else TRAY_BASE + TRAY_RAIL_H)
    return TOP


@k.on_reset
def _reset(states) -> None:
    BAD.reset(), SET2.reset(), FLOW.reset(), END.reset()
    RUN.clear(), RUN.update(_fresh())
    REC.reset()


@k.on_step
def _watch(states) -> None:
    s = states["pen"]
    dt = 1.0 / HZ
    RUN["t"] += dt
    angle, push = float(s.q["dial"]), float(s.q["button"])
    up = needle_tilt(states) <= UP_TOL
    at_stop = push >= AT_STOP
    # -- the button: a press starts at the stop, delivers while held there, ends when let out
    if at_stop and not RUN["delivering"]:
        RUN["delivering"], RUN["press_units"], RUN["press_up"] = True, float(counter(states)), True
        RUN["units"] = float(counter(states))
        if RUN["press_units"] > 0:
            RUN["presses"] += 1
            if RUN["presses"] > MAX_PRESSES:
                BAD.trip("more than 6 presses", RUN["t"])
            if RUN["press_units"] > CHECK_UNITS:
                BAD.trip("a dose pushed out", RUN["t"])
    if RUN["delivering"]:
        RUN["press_up"] = RUN["press_up"] and up
        if at_stop and RUN["units"] > 0:
            out = min(RUN["units"], RATE_UPS * dt)
            RUN["units"] -= out
            if up:  # air leaves first only with the needle up
                a = min(RUN["air"], out)
                RUN["air"] -= a
                RUN["liquid"] += out - a
            else:
                RUN["liquid"] += out
            k.set_targets("pen", {"dial": RUN["units"] / UNITS_PER_CLICK * CLICK})
            if RUN["liquid"] > 0 and not RUN["drop"]:
                RUN["drop"] = True
                k.set_targets("pen", {"drop": 0.0125})
            if RUN["units"] <= 1e-6 and RUN["press_units"] == CHECK_UNITS and RUN["press_up"] and RUN["drop"]:
                FLOW.update(True, RUN["t"])
        if not at_stop:
            RUN["delivering"] = False
    if not RUN["delivering"]:  # the click detent: the spring pulls to the nearest click
        k.set_targets("pen", {"dial": max(0, clicks_of(angle)) * CLICK})
    # -- milestones and latched zeros
    if counter(states) == CHECK_UNITS and float(s.pos[2]) >= PEN_Z0 + LIFT and not FLOW:
        SET2.update(True, RUN["t"])
    grips = k.score.grippers()
    needle = np.array([s.point("tip"), s.point("needle_mid")])
    if grips and k.score.clearance(needle, grips, beyond=0.01) < NEEDLE_MM / 1000.0:
        BAD.trip("needle touched a gripper", RUN["t"])
    if any(float(p[2]) < _surface_below(p) + NEEDLE_MM / 1000.0 for p in needle):
        BAD.trip("needle touched a surface", RUN["t"])
    on_tray = abs(float(s.pos[0]) - PEN_X) <= TRAY_L / 2 and abs(float(s.pos[1]) - PEN_Y) <= TRAY_W / 2
    if float(s.pos[2]) < TOP + FALL_Z and not on_tray:
        BAD.trip("pen fell", RUN["t"])
    right_clear = not grips or k.score.clearance(pen_points(s), [grips[1]], beyond=CLEAR_MM / 1000.0) >= CLEAR_MM / 1000.0
    END.push(list(np.asarray(s.pos, dtype=float)), right_clear)
    if DEBUG and int(round(RUN["t"] * HZ)) % 10 == 0:
        print(f"[pen] t {RUN['t']:.1f} counter {counter(states)} dial {math.degrees(angle):.1f} button {push*1000:.2f} tilt {math.degrees(needle_tilt(states)):.0f} pos {np.round(np.asarray(s.pos)-np.array([PEN_X,PEN_Y,TOP]),3)} air {RUN['air']:.2f} liq {RUN['liquid']:.2f} pr {RUN['presses']} bad {BAD!r} score {grade(states):g}", file=sys.stderr)
    shown = displayed(states)
    if shown != RUN.get("shown"):  # the window follows the dial every step it changes
        RUN["shown"] = shown
        show(shown)
    REC.tick(states, lambda: (f"window {RUN['shown']} counter {counter(states)} dial {math.degrees(angle):.1f} button {push * 1000:.2f} mm "
                              f"tilt {math.degrees(needle_tilt(states)):.0f} air {RUN['air']:.2f} "
                              f"liquid {RUN['liquid']:.2f} presses {RUN['presses']} score {grade(states):g}"))


# == the demo ==========================================================================================================
G = D.Grip
PICK = G.deg(0, 0, 0)  # straight down, jaws closing across the pen (along y)
DIAL_G = G.deg(0, 90, 0)  # right hand level, pointing forward along the pen, jaws across the selector
HOLD_Z = TOP + 0.14  # the pen's axis while dialling
HOLD_AT = 0.020  # m: the left hand holds the pen this far from its middle toward the needle (61 mm from the tip)
PRESS_XY = (0.32, 0.06)  # over the mat's near edge
OPEN_GAP, SQUEEZE = 0.042, 0.002
PRESS_Q = 0.0021  # m: the button pushed to its stop (2 mm), no further, so the pen never slips in the pads


def _axrot(axis, a):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    kx = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(a) * kx + (1 - math.cos(a)) * kx @ kx


def turned(g, deg: float):
    """The grip ``g`` turned ``deg`` about its own approach axis (a wrist twist)."""
    return G.from_matrix(_axrot(g.approach, math.radians(deg)) @ g.rotation)


PAD_BACK = 0.006  # m: the right hand this far back of the selector's middle, so its pads touch only the selector


def sel_centre(st):
    return st["pen"].to_world((-SEL_L / 2 - PAD_BACK, 0, 0), "dial")


def sel_fallback():
    return (PEN_X + BODY[0] - SEL_L / 2 - PAD_BACK, PEN_Y, HOLD_Z)


def left_hand(how: str) -> D.Plan:
    p = D.Plan("left")
    D.grasp(p, lambda st: st["pen"].to_world((HOLD_AT, 0, 0)), PEN_W, height=PEN_W, grip=PICK,
            fallback=(PEN_X + HOLD_AT, PEN_Y, PEN_Z0), lift=None, label="the pen")
    p.move(2.0, (PEN_X + HOLD_AT, PEN_Y, HOLD_Z), label="lift the pen level")
    return p


def pitch_steps(p: D.Plan, pos, a0: float, a1: float, label: str) -> None:
    n = 6
    for i in range(1, n + 1):
        a = a0 + (a1 - a0) * i / n
        p.move(0.5, pos, grip=G.deg(0, a, 0), label=label)


def press_needle_up(p: D.Plan, upright: bool = True) -> None:
    """Tip the pen needle-up (or needle-down for the cheat), carry it over the mat, push its button end down."""
    above = (PRESS_XY[0], PRESS_XY[1], HOLD_Z)
    p.move(2.0, above, label="carry over the mat")
    pitch_steps(p, above, 0.0, 90.0 if upright else -90.0, "tip the pen needle-up")
    face = HOLD_AT - (BODY[0] - SEL_L - 0.004)  # the button face, this far below the left hand when upright
    contact = TOP + MAT[2] + face  # TCP height with the button face on the mat
    x, y = PRESS_XY
    p.move(1.5, D.remember(p, "cmd", lambda st: (x, y, contact + 0.010)), label="lower the button end to the mat",
           fallback=(x, y, contact + 0.010))

    def have(st):  # the button's travel, as a height: servo the hand down until the button is at its stop
        return (0.0, 0.0, -float(st["pen"].q["button"]))

    def want(st):  # gain 2 on the error: the arm lags each step under the button's load
        return (0.0, 0.0, -PRESS_Q - (PRESS_Q - float(st["pen"].q["button"])))

    for _ in range(8):  # closed loop: step the command down until the button is at its stop, and no further
        D.corrected(p, 0.5, want, have, key="cmd", base="cmd", max_fix=0.005, label="press the button down",
                    fallback=(x, y, contact))
    p.hold(1.5, "hold until the counter shows 0")
    p.move(1.5, above, label="lift off the mat")
    pitch_steps(p, above, 90.0 if upright else -90.0, 0.0, "tip the pen back level")
    p.move(2.0, (PEN_X + HOLD_AT, PEN_Y, HOLD_Z), label="back to the dialling spot")


def turn_dial(p: D.Plan, start_deg: float, end_deg: float, label: str) -> None:
    """Slide the open right hand onto the selector at ``start_deg`` of wrist twist, pinch, twist to ``end_deg``."""
    fb = sel_fallback()
    g0 = turned(DIAL_G, start_deg)
    p.move(1.5, lambda st: np.add(sel_centre(st), (-0.060, 0, 0)), finger=D.gap_finger(OPEN_GAP),
           label="line up behind the selector", grip=g0, fallback=np.add(fb, (-0.060, 0, 0)))
    p.move(1.5, lambda st: sel_centre(st), label="slide onto the selector", grip=g0, track=True, fallback=fb)
    p.fingers(SEL_W - 2 * SQUEEZE, label="pinch the selector")
    steps = max(1, int(round(abs(end_deg - start_deg) / 7.5)))
    for i in range(1, steps + 1):
        a = start_deg + (end_deg - start_deg) * i / steps
        p.move(0.35, lambda st: sel_centre(st), grip=turned(DIAL_G, a), label=label, track=True, fallback=fb)
    p.hold(0.5, "let the click settle")
    p.fingers(OPEN_GAP, label="let go of the selector")
    p.move(1.2, lambda st: np.add(sel_centre(st), (-0.060, 0, 0)), label="slide off the selector",
           fallback=np.add(fb, (-0.060, 0, 0)))


READ_D, READ_TILT = 0.09, 45.0  # m, deg: the right wrist camera this far from the dose window, looking down at it
#                                  45 deg from behind (straight down from above put joint 4 on its limit)
READ_OFF = (-READ_D * math.sin(math.radians(READ_TILT)), 0.0, READ_D * math.cos(math.radians(READ_TILT)))
READ_G = D.Grip.deg(0, READ_TILT, 0)


def read_window(p: D.Plan, label: str) -> None:
    """Hold the open right hand straight above the dose window so its wrist camera reads the number, then move off."""
    def over(st):
        return np.add(st["pen"].to_world((WIN_X, 0.0, WIN_Z)), READ_OFF)
    fb = tuple(np.add((PEN_X + WIN_X, PEN_Y, HOLD_Z), READ_OFF))
    p.move(2.0, over, grip=READ_G, finger=D.gap_finger(OPEN_GAP), label="over the dose window", fallback=fb)
    p.move(2.0, over, grip=READ_G, label=label, track=True, fallback=fb)


def plans(how: str) -> dict:
    left, right = left_hand(how), D.Plan("right")
    right.hold(left.duration, "wait for the pen")
    if how in ("demo", "level", "down", "wasted"):
        turn_dial(right, 0.0, 15.0, "turn one click (2 units)")
        if how == "wasted":
            turn_dial(right, 15.0, 90.0, "turn 5 more clicks")
        hp, hg = D.P.home("right")
        right.move(2.0, hp, grip=hg, label="park clear of the left hand")
        left.hold(right.duration - left.duration, "hold the pen for the dial")
        if how == "level":  # press with the pen still level: the right hand pushes the button
            fb = sel_fallback()
            right.move(1.5, lambda st: np.add(st["pen"].to_world((BODY[0] - SEL_L - 0.004, 0, 0)), (-0.050, 0, 0)),
                       finger=D.gap_finger(0.0), grip=DIAL_G, label="line up on the button",
                       fallback=np.add(fb, (-0.060, 0, 0)))
            right.move(1.5, lambda st: np.add(st["pen"].to_world((BODY[0] - SEL_L - 0.004, 0, 0)), (0.004, 0, 0)),
                       label="push the button", fallback=fb).hold(1.5).back(0.06)
            left.hold(right.duration - left.duration, "hold the pen level")
        else:
            press_needle_up(left, upright=(how != "down"))
            right.hold(left.duration - right.duration, "wait")
        if how in ("demo", "level", "down"):
            turn_dial(right, 0.0, 90.0, "turn 6 clicks")
            turn_dial(right, 0.0, 90.0, "turn 6 more clicks")
        if how == "demo":
            read_window(right, "read 24 in the dose window")
    elif how == "skip":
        turn_dial(right, 0.0, 90.0, "turn 6 clicks")
        turn_dial(right, 0.0, 90.0, "turn 6 more clicks")
    right.home()
    left.hold(max(0.0, right.duration - left.duration) + 3.0, "hold the pen ready")
    return D.together(left, right)


def demo():
    return D.choose(demo=lambda: plans("demo"), level=lambda: plans("level"), down=lambda: plans("down"),
                    skip=lambda: plans("skip"), wasted=lambda: plans("wasted"))


# == what the template tests read ======================================================================================
TEXT_NUMBERS = [("5.1 N", BUTTON_N), ("2 mm", BUTTON_TRAVEL * 1000 / 1000), ("2 s", STILL_S), ("3 cm", CLEAR_MM / 1000),
                ("13 cm", 0.13)]
CHEATS = {"none": 0.0, "level": 0.25, "down": 0.25, "skip": 0.25, "wasted": 0.0}
HOLD = {"pen.dial": 1.0, "pen.button": 0.2}
