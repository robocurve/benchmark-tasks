"""Set both rear brakes of a stroller whose brake pins sit on tooth crests.

On the station table (the floor here), an empty tandem stroller stands with its rear toward the robot, as a parent
leaves it. Each rear wheel turns on its own on the axle and carries a ring of 12 brake teeth on its inner hub. Beside
each wheel a brake lever swings down 25 deg and drops a pin between two teeth; past 20 deg a detent holds it down.
At the start each pin sits on a tooth crest, so a press alone stops about 9 deg down and springs back when let go.
The maker's step: press the lever and move the stroller slightly forward or back until the teeth line up.

On the kit: one free mech (the stroller) with four wheel hinges and two lever hinges. The lever is a hinge with a
return spring; its pin is a box that meets the wheel's tooth boxes (geometry, not a rule). The detent is a ``Latch``
on the lever: once the lever passes 20 deg down (only possible with the pin in a gap) it cannot rise past 21 deg.
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
SUCCESS_HOLD_S = 0.0  # the full score already needs 2 s still with the hands clear
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Set both rear brakes on the stroller so it cannot roll, then let go."
R = math.radians
MM = 0.001

# -- layout (x forward from the arm bases, y left, z up from the table top) ------------------------------------------
XA = 0.400  # rear axle, 400 mm in front of the arm bases
RW = 0.1015  # rear wheel radius (8 in wheel)
YW = 0.240  # rear wheel centre planes, 480 mm apart
TYRE_W = 0.035
WHEELBASE = 0.760  # rear axle to front axle
RF = 0.089  # front wheel radius (7 in)
YF = 0.200

# -- the brake (fidelity: lever force tuned to a hand pinch standing in for a foot) ----------------------------------
N_TEETH = 12  # teeth on each hub ring, one every 30 deg
TOOTH_R = (0.030, 0.046)  # tooth root and tip radius
TOOTH_W = 0.009  # tooth width across (gap at the tips 15 mm)
RING_T = 0.010  # ring thickness along the axle
RING_IN = 0.012  # ring's inner face this far inside the tyre's inner face
PIN_R0 = 0.058  # the pin's centre this far behind the axle with the lever up
ARM_B = 0.050  # pivot to pin centre (pin arm, straight down)
ARM_A = 0.050  # pivot to pad centre (pedal arm, straight back)
PAD_IN = 0.040  # pad centre this far inboard of the pin
ARM_IN = 0.012  # lever arms this far inboard of the ring's middle
PAD = (0.040, 0.030, 0.008)  # pad: along x, across, thick
PIN = (0.008, 0.008, 0.008)
DOWN = R(-25.0)  # full down
DETENT = R(-20.0)  # past this the detent holds it down
CATCH = R(-21.0)  # held no higher than this
F_UP, F_DOWN = 6.4, 20.4  # N at the pad: at rest (up) and full down
RATE = (F_DOWN - F_UP) * ARM_A / (0 - DOWN)  # N m per rad
TARGET = F_UP * ARM_A / RATE  # spring target above the up stop (preload)
HOUSING_GAP = 0.028  # housing top this far below the pad's top face at rest... below the pad centre
HOUSING_T = 0.012

# -- scoring --------------------------------------------------------------------------------------------------------
ROLL_MAX = 0.150  # rolled further than this: score 0
TIP_DEG = 15.0  # tipped further than this: score 0
STILL_S, STILL_MM = 2.0, 0.002
CLEAR_MM = 30.0
STROLLER_KG = 12.5
LEVER_KG = 0.12  # raised from a real lever's ~60 g to keep PhysX's link-mass ratio under 100:1 (declared)
HEIGHT = 1.070  # handlebar top
FLOOR = (1.30, 0.90, 0.012)  # floor panel the stroller stands on: along x, across, thick
FLOOR_X0 = 0.20  # its back edge; it runs out past the table's far edge (x 0.91) to 1.50
BASE_Z = TOP + FLOOR[2] + 0.0005

FRAME, TYRE, HUB, RED, GREY, FABRIC = (0.18, 0.18, 0.2), (0.08, 0.08, 0.08), (0.55, 0.56, 0.58), \
    (0.80, 0.12, 0.10), (0.45, 0.46, 0.48), (0.25, 0.27, 0.32)
ZA = RW  # axle height above the table


def _wheel_boxes(r: float, width: float, ring: bool, side: float) -> list:
    """A wheel in its hinge frame (axis y): 24 tread pieces, a hub, and (rear) the brake-tooth ring on the inner face."""
    n = 24
    seg = 2 * r * math.tan(math.pi / n) + 0.001
    out = []
    for i in range(n):
        a = 2 * math.pi * i / n
        c = ((r - 0.008) * math.sin(a), 0.0, (r - 0.008) * math.cos(a))
        out.append(k.box(c, (seg, width, 0.016), TYRE, turn=(0.0, a, 0.0)))
    out.append(k.box((0, 0, 0), (0.060, width - 0.006, 0.060), HUB))
    out.append(k.box((0, 0, 0), (0.060, width - 0.006, 0.060), HUB, turn=(0.0, math.pi / 4, 0.0)))
    if ring:
        y = -side * (width / 2 + RING_IN - RING_T / 2)  # toward the stroller's centre
        rm = sum(TOOTH_R) / 2
        for i in range(N_TEETH):
            a = -math.pi / 2 + 2 * math.pi * i / N_TEETH  # tooth 0 straight back (-x), under the pin at the start
            c = (rm * math.sin(a), y, rm * math.cos(a))
            out.append(k.box(c, (TOOTH_W, RING_T, TOOTH_R[1] - TOOTH_R[0]), RED, turn=(0.0, a, 0.0)))
    return out


def _wheel(name: str, side: float) -> k.Joint:
    return k.hinge(name, (0.0, side * YW, ZA), "Y", -math.pi, math.pi, _wheel_boxes(RW, TYRE_W, True, side),
                   friction=0.02, damping=0.01, mass=0.45)


def _front(name: str, side: float) -> k.Joint:
    return k.hinge(name, (WHEELBASE, side * YF, RF), "Y", -math.pi, math.pi, _wheel_boxes(RF, 0.030, False, side),
                   friction=0.02, damping=0.01, mass=0.35)


PIN_Y = YW - TYRE_W / 2 - RING_IN + RING_T / 2  # |y| of the tooth ring's middle
PIVOT_X, PIVOT_Z = -PIN_R0, ZA + ARM_B


def _lever(name: str, side: float) -> k.Joint:
    s = side
    py = -s * PAD_IN  # pad offset inboard, in the lever frame
    ay = -s * ARM_IN  # the lever's arms run this far inboard of the tooth ring, clear of the teeth
    boxes = [
        k.box((0, ay, -ARM_B / 2), (0.006, 0.006, ARM_B), RED),  # pin arm
        k.box((0, (ay + s * RING_T / 2) / 2, -ARM_B), (PIN[0], abs(ay) + RING_T / 2 + 0.003, PIN[2]), GREY),  # pin
        k.box((-ARM_A / 2, ay, 0), (ARM_A, 0.008, 0.006), RED),  # pedal arm
        k.box((-ARM_A, (ay + py) / 2, 0), (0.010, abs(py - ay), 0.006), RED),  # cross bar to the pad
        k.box((-ARM_A, py, 0), PAD, RED),  # the pad
    ]
    return k.hinge(name, (PIVOT_X, s * PIN_Y, PIVOT_Z), "Y", DOWN, 0.0, boxes, rate=RATE, target=TARGET,
                   damping=0.02, mass=LEVER_KG, start=0.0)


def _chassis() -> list:
    s = k.span
    out = [s((-0.012, -0.205, ZA - 0.012), (0.012, 0.205, ZA + 0.012), FRAME)]  # rear axle tube (to the hubs)
    for side in (-1.0, 1.0):
        y0 = side * (YW - TYRE_W / 2 - 0.030)
        out += [
            s((-0.015, y0 - 0.012, ZA + 0.012), (0.015, y0 + 0.012, HEIGHT), FRAME),  # rear leg up to the handlebar
            s((0.0, y0 - 0.012, 0.400), (WHEELBASE, y0 + 0.012, 0.424), FRAME),  # side rail
            s((WHEELBASE - 0.015, side * (YF - 0.03) - 0.012, RF), (WHEELBASE + 0.015, side * (YF - 0.03) + 0.012, 0.420),
              FRAME),  # front leg
            # brake mount: a plate from the axle carrying the lever pivot (drawn only)
            s((PIVOT_X - 0.010, side * (PIN_Y - ARM_IN - 0.009) - 0.003, ZA),
              (0.012, side * (PIN_Y - ARM_IN - 0.009) + 0.003, PIVOT_Z + 0.008), GREY, ghost=True),
        ]
        hx, hy = PIVOT_X - ARM_A, side * (PIN_Y - PAD_IN)
        hz_top = PIVOT_Z - PAD[2] / 2 - (ARM_A * math.sin(-DOWN)) - 0.004
        out.append(s((hx - PAD[0] / 2, hy - PAD[1] / 2, hz_top - HOUSING_T), (hx + PAD[0] / 2, hy + PAD[1] / 2, hz_top),
                     GREY))
        out.append(s((hx + PAD[0] / 2, hy - 0.006, hz_top - HOUSING_T), (-0.012, hy + 0.006, hz_top), GREY))
    out += [s((-0.020, -YW + 0.03, HEIGHT - 0.030), (0.020, YW - 0.03, HEIGHT), FRAME),  # handlebar
            s((0.05, -0.20, 0.42), (0.70, 0.20, 0.60), FABRIC),  # seats
            s((0.10, -0.20, 0.17), (0.65, 0.20, 0.30), FABRIC)]  # basket
    return out


WHEEL_R, WHEEL_L = _wheel("wheel_r", -1.0), _wheel("wheel_l", 1.0)
LEVER_R, LEVER_L = _lever("lever_r", -1.0), _lever("lever_l", 1.0)
FRONT_R, FRONT_L = _front("front_r", -1.0), _front("front_l", 1.0)
STROLLER = k.mech("stroller", (XA, 0.0, BASE_Z), _chassis(), STROLLER_KG - 2 * 0.45 - 2 * 0.35,
                  joints=[WHEEL_R, WHEEL_L, LEVER_R, LEVER_L, FRONT_R, FRONT_L], color=FRAME, surface=0.8,
                  self_collide=True)
FLOOR_BOX = Box("floor_panel", pos=(FLOOR_X0 + FLOOR[0] / 2, 0.0, TOP + FLOOR[2] / 2), size=FLOOR, color=(0.62, 0.55, 0.45))
OBJECTS = [STROLLER, FLOOR_BOX]

BRAKES = k.Rules(STROLLER, [k.Latch(j, [CATCH], side="up") for j in ("lever_r", "lever_l")])  # a lever only gets
# below the catch with its pin in a tooth gap; coming back up it is caught there (the detent)

# -- the scorer -------------------------------------------------------------------------------------------------------
ROLLED, END = k.Sticky(), k.Settle(STILL_S, STILL_MM)
RUN = {"pos0": None}
REC = k.run_record.RunRecord(__file__, "stroller")


def locked(st, j: str) -> bool:
    return float(st["stroller"].q[j]) <= DETENT + R(0.5)


def still() -> bool:
    return len(END.rows) >= END.n and END.spread < END.tol


def grade(states) -> float:
    n = locked(states, "lever_r") + locked(states, "lever_l")
    return 0.0 if ROLLED else k.ladder((0.25, n >= 1), (0.5, n == 2), (0.75, n == 2 and still()), (1.0, n == 2 and END.ok))


def score(states) -> float:
    return grade(states)


def success(states) -> bool:
    return grade(states) >= 1.0


def distance(states) -> float:
    return float(states["stroller"].q["lever_r"] - DOWN + states["stroller"].q["lever_l"] - DOWN)


def _rear_points(s) -> np.ndarray:
    pts = []
    for side, j in ((-1.0, "lever_r"), (1.0, "lever_l")):
        pts.append(s.to_world((-ARM_A, -side * PAD_IN, 0.0), j))
        pts.append(s.to_world((0.0, 0.0, -ARM_B), j))
    pts += [s.to_world((0.0, sd * YW, ZA + dz)) for sd in (-1.0, 1.0) for dz in (-RW, 0.0, RW)]
    return np.asarray(pts)


@k.on_reset
def _reset(states) -> None:
    ROLLED.reset(), END.reset()
    RUN["pos0"] = np.asarray(states["stroller"].pos, dtype=float)
    REC.reset()


@k.on_step
def _watch(states) -> None:
    s = states["stroller"]
    if np.linalg.norm(np.asarray(s.pos)[:2] - RUN["pos0"][:2]) > ROLL_MAX:
        ROLLED.trip("stroller rolled away")
    if s.tilt_deg() > TIP_DEG:
        ROLLED.trip("stroller tipped")
    clear = k.hands_clear(_rear_points(s), CLEAR_MM)
    RUN["n"] = RUN.get("n", 0) + 1
    END.push(np.r_[np.asarray(s.pos), math.degrees(s.q["lever_r"]) * MM, math.degrees(s.q["lever_l"]) * MM], clear)
    REC.tick(states, lambda: f"levers {math.degrees(s.q['lever_r']):.1f} {math.degrees(s.q['lever_l']):.1f} deg, "
                             f"wheels {math.degrees(s.q['wheel_r']):.1f} {math.degrees(s.q['wheel_l']):.1f}, "
                             f"score {grade(states):g}")


# -- the demo ---------------------------------------------------------------------------------------------------------
def _pinch_grip(tilt_deg: float, flip: bool = False) -> D.Grip:
    """Approach forward and tilt_deg down, jaws closing nearly vertically; ``flip`` turns the hand 180 deg about the
    approach (the same pinch, the other wrist solution: the left arm's joint 6 needs it)."""
    t = math.radians(tilt_deg)
    close, approach = np.array([math.sin(t), 0.0, math.cos(t)]), np.array([math.cos(t), 0.0, -math.sin(t)])
    if flip:
        close = -close
    return D.Grip.from_matrix(np.column_stack([close, np.cross(approach, close), approach]))


PINCH_TILT = 20.0  # approach forward and 20 deg down, jaws closing nearly vertically
PINCH = _pinch_grip(PINCH_TILT)
PINCH_FLIP = _pinch_grip(PINCH_TILT, flip=True)
OPEN_GAP = 0.070
TOUCH_GAP = 0.046  # fingers just on pad and housing, no press (the nudge-only failure)
LIGHT_GAP = 0.0385  # a light pinch, about at contact with the pin on a tip (GPU run 1: 37 mm skidded one wheel): the pin rests on the crest without jamming the wheel (manual: light pressure)
NUDGE = 0.027  # roll the stroller this far forward: 15 deg of wheel turn, half the tooth pitch


def pinch_at(st, side: float) -> np.ndarray:
    """The TCP for a pinch of pad (top) and housing (bottom): middle of the span, fingertips 25 mm past the pad's back."""
    s = st["stroller"]
    j = "lever_r" if side < 0 else "lever_l"
    pad = s.to_world((-ARM_A, -side * PAD_IN, 0.0), j)
    base = s.to_world((PIVOT_X - ARM_A, side * (PIN_Y - PAD_IN), PIVOT_Z))
    z_top = pad[2] + PAD[2] / 2
    z_bot = base[2] - PAD[2] / 2 - ARM_A * math.sin(-DOWN) - 0.004 - HOUSING_T
    return np.array([base[0] - PAD[0] / 2 + 0.025 - D.P.PAD_TIP, base[1], (z_top + z_bot) / 2])


def _pad_start(side: float) -> np.ndarray:
    return np.array([TOP * 0 + XA + PIVOT_X - ARM_A - PAD[0] / 2 + 0.025 - D.P.PAD_TIP, side * (PIN_Y - PAD_IN),
                     BASE_Z + PIVOT_Z - 0.020])


def hand(side: float, how: str) -> D.Plan:
    p = D.Plan("right" if side < 0 else "left")
    start = _pad_start(side)
    p.move(2.5, start - (0.070, 0, 0), finger=D.gap_finger(OPEN_GAP), grip=PINCH if side < 0 else PINCH_FLIP, label="in line behind the pedal")
    p.move(2.0, lambda st: pinch_at(st, side), label="fingers round pedal and housing", fallback=start)
    p.fingers(TOUCH_GAP if how == "nudge_only" else LIGHT_GAP, 1.5, "pinch the pedal lightly down onto the tooth")
    if how == "press_only":
        p.hold(2.0)
        return p.fingers(OPEN_GAP, 0.8, "let go").back(0.070).home()
    p.hold(0.5)
    if how != "no_nudge":
        p.move(3.0, D.remember(p, "nudge", lambda st: pinch_at(st, side) + (NUDGE, 0, 0)), label="nudge forward",
               fallback=start + (NUDGE, 0, 0))
        p.hold(0.5, "let the pin drop between the teeth")
        p.fingers(TOUCH_GAP if how == "nudge_only" else 0.004, 1.0, "press the pedal fully down past the detent")
        p.hold(0.5)
    if how == "held":
        return p.hold(8.0, "keep holding the pedal")
    p.fingers(OPEN_GAP, 0.8, "let go of the pedal")
    p.move(1.5, lambda st: pinch_at(st, side) - (0.080, 0, 0), label="back out", fallback=start - (0.08, 0, 0))
    return p.home()


def demo():
    def two(how):
        return D.together(hand(-1.0, how), hand(1.0, how))

    def one_side():
        return D.together(hand(-1.0, "demo"), D.idle_plan("left", 20.0))

    return D.choose(demo=lambda: two("demo"), press_only=lambda: two("press_only"), nudge_only=lambda: two("nudge_only"),
                    one_side=one_side, held=lambda: two("held"))


D_ = math.degrees
TEXT_NUMBERS = [("1.07 m", HEIGHT), ("12.5 kg", STROLLER_KG), ("12 mm plywood", FLOOR[2]), ("200 mm in front", FLOOR_X0),
                ("400 mm in front", XA), ("178 mm", 2 * RF), ("203 mm", 2 * RW), ("480 mm apart", 2 * YW),
                ("46 mm from the axle", TOOTH_R[1]), ("50 mm above the axle", ARM_B), ("30 mm pad", PAD[1]),
                ("25 deg", D_(-DOWN)), ("6.4 N", F_UP), ("20.4 N", F_DOWN), ("21 deg down", D_(-CATCH)),
                ("19.5 deg", D_(-(DETENT + R(0.5)))), ("150 mm", ROLL_MAX), ("15 deg", TIP_DEG), ("2 mm", STILL_MM),
                ("2 s", STILL_S), ("30 mm of", CLEAR_MM * MM), ("515 mm wide", 0.515)]
CHEATS = {"none": 0.0, "press_only": 0.0, "nudge_only": 0.0, "one_side": 0.25, "held": 0.75}
HOLD = {"stroller.wheel_r": 0.5, "stroller.wheel_l": 0.5, "stroller.lever_r": 0.5, "stroller.lever_l": 0.5,
        "stroller.front_r": 0.5, "stroller.front_l": 0.5}
