"""Carry a loaded 125 mm wafer carrier into a wet-bench rinse tank by its squeeze handle, then take the handle off.

On the station table: a polypropylene wet bench, a deck 17.5 cm high beside a recessed quick-dump-rinse tank (dry).
On the deck a 125 mm PFA wafer carrier (25 wafers) with its PFA squeeze handle on it: the handle's two feet carry tines
that sit in a notch in each end wall of the carrier. The handle is an inverted U: a top bar, and under it a curved bow.

- Squeezing the bow up toward the bar spreads the legs, so the tines leave the notches: that is how the handle is put
  on and taken off. A hand that closes over bar and bow together, the obvious grip on a squeeze handle, lets the
  carrier go. Carried by the bar alone, the legs' springs keep the tines in.
- Nothing is welded: the carrier hangs on the tines by contact. One code rule stands in for the bent flexure: the legs'
  spring targets follow the bow's travel (Couple). The bench is a heavy free body standing in for a bolted bench.
- Scoring: taking the tines out is allowed once the carrier sits on both rests (the handle must come off for 1.0);
  after the lift, a tine leaving its notch anywhere else latches zero (released_early), as do a fall (the carrier going down 20 mm more than the hands), a tip, a touch or a bench move.
"""

from __future__ import annotations

import collections
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import ourkit as k  # noqa: E402
from ourkit import demo as D  # noqa: E402
from ourkit import scene as S  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the 1.0 state already includes 2 s released and still
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Put the wafer carrier into the rinse tank using its handle, then take the handle off the carrier and "
               "stand it on the bench deck.")

# == the carrier (Entegris 182 series 125 mm, A182-50M; frame: origin at its bottom centre, x across the wafers, y along
# the stack, z up) ======================================================================================================
CARRIER_L, CARRIER_W, CARRIER_H = 0.143, 0.152, 0.146  # catalog 5367: 143 x 152 x 146 mm
PITCH, WAFER_N = 0.00476, 25  # pocket spacing, capacity
WAFER_D, WAFER_T = 0.125, 0.000625
WAFER_KG = 0.0179  # 125 mm x 0.625 mm silicon (2.33 g/cm3)
CARRIER_KG = 0.30  # estimate; weighed before trials
END_T = 0.010  # end walls
SIDE_T = 0.006
RAIL = (0.055, 0.076, 0.015)  # bottom rails: x from, x to, height
HX, HY = CARRIER_W / 2, CARRIER_L / 2  # 0.076, 0.0715
NOTCH_W, NOTCH_H, NOTCH_D, NOTCH_Z = 0.060, 0.008, 0.006, 0.118  # notch in each end wall's outer face
WAFER_Z = WAFER_D / 2 + 0.010  # wafer centres; their lower edges 10 mm above the carrier's bottom

PFA, SILICON, PP, PP_DARK = (0.93, 0.93, 0.90), (0.36, 0.38, 0.47), (0.86, 0.88, 0.90), (0.70, 0.74, 0.78)


def _end_wall(s: int) -> list:
    """The end wall at y = s * HY, with its notch (a pocket in the outer face)."""
    sp = k.span
    tag = "pos" if s > 0 else "neg"
    yo, yi = s * HY, s * (HY - END_T)
    yn = s * (HY - NOTCH_D)
    lo = lambda a, b: (min(a, b), max(a, b))  # noqa: E731
    y0, y1 = lo(yi, yo)
    n0, n1 = lo(yi, yn)
    z0, z1 = NOTCH_Z, NOTCH_Z + NOTCH_H
    return [sp((-HX, y0, 0.0), (HX, y1, z0), PFA, label=f"end_{tag}_low"),
            sp((-HX, y0, z1), (HX, y1, CARRIER_H), PFA, label=f"end_{tag}_top"),
            sp((-NOTCH_W / 2, n0, z0), (NOTCH_W / 2, n1, z1), PFA, label=f"end_{tag}_notch_back"),
            sp((NOTCH_W / 2, y0, z0), (HX, y1, z1), PFA, label=f"end_{tag}_notch_l"),
            sp((-HX, y0, z0), (-NOTCH_W / 2, y1, z1), PFA, label=f"end_{tag}_notch_r")]


def _wafer(i: int) -> list:
    """A wafer drawn as four crossed strips (each its own prim); no collider."""
    y = -(WAFER_N - 1) * PITCH / 2 + i * PITCH
    r = WAFER_D / 2
    w = 0.8 * r
    return [k.box((0.0, y, WAFER_Z), (WAFER_D, WAFER_T, w), SILICON, turn=(0.0, a, 0.0), ghost=True,
                  label=f"wafer_{i}_{j}") for j, a in enumerate((0.0, math.pi / 4, math.pi / 2, 3 * math.pi / 4))]


def _carrier_boxes() -> list:
    sp = k.span
    out = []
    for s, tag in ((1, "l"), (-1, "r")):
        x0, x1 = sorted((s * RAIL[0], s * RAIL[1]))
        out.append(sp((x0, -(HY - END_T), 0.0), (x1, HY - END_T, RAIL[2]), PFA, label=f"rail_{tag}"))
        w0, w1 = sorted((s * (HX - SIDE_T), s * HX))
        out.append(sp((w0, -(HY - END_T), RAIL[2]), (w1, HY - END_T, CARRIER_H - 0.006), PFA, label=f"side_{tag}"))
    out += _end_wall(1) + _end_wall(-1)
    out.append(sp((-0.020, -HY - 0.008, 0.0), (0.020, -HY, 0.080), PFA, label="h_bar"))  # equipment H-bar, -y end
    for i in range(WAFER_N):
        out += _wafer(i)
    return out


# == the squeeze handle (Entegris A72-50-03; frame: origin at its feet's underside, centred; y along the carrier) =====
HANDLE_L, HANDLE_W, HANDLE_H = 0.163, 0.140, 0.183  # catalog 5367
HANDLE_KG = 0.15  # estimate; weighed before trials
BAR = 0.014  # top bar section
BAR_Z = (HANDLE_H - BAR, HANDLE_H)
BOW_GAP = 0.033  # bar underside to the bow's top, at the middle
BOW_T = 0.008
BOW_Z = (BAR_Z[0] - BOW_GAP - BOW_T, BAR_Z[0] - BOW_GAP)  # 0.128-0.136
LEG_T, LEG_D = 0.006, 0.024  # leg plate thickness (y) and depth (x)
LEG_IN = HY + 0.004  # legs' inner faces at |y| = 75.5 mm, 4 mm outside the end walls
TINE_IN, TINE_W, TINE_T = HY - NOTCH_D, 0.050, 0.005  # tines reach in to the notch's back
FOOT_T = 0.008
SPREAD = 0.012  # leg travel outward
LEG_RATE = 2000.0  # N/m toward closed (12 N with the tines just out of the 6 mm notches)
BOW_TRAVEL, BOW_RATE = 0.010, 2000.0  # bow: 10 mm up, 20 N at full travel
RATIO = 1.2  # leg spread per unit of bow travel (the flexure's lever): 12 mm at full travel, free past 5 mm
BAR_C = (BAR_Z[0] + BAR_Z[1]) / 2  # 0.176


def _leg(s: int) -> list:
    tag = "l" if s > 0 else "r"
    y0, y1 = sorted((s * LEG_IN, s * (LEG_IN + LEG_T)))
    t0, t1 = sorted((s * TINE_IN, s * LEG_IN))
    sp = k.span
    return [sp((-LEG_D / 2, y0, FOOT_T), (LEG_D / 2, y1, BAR_Z[0] - 0.004), PFA, label=f"leg_{tag}"),
            sp((-HANDLE_W / 2, y0, 0.0), (HANDLE_W / 2, y1, FOOT_T), PFA, label=f"foot_{tag}"),
            sp((-TINE_W / 2, t0, 0.0), (TINE_W / 2, t1, TINE_T), PFA, label=f"tine_{tag}")]


def _bow() -> list:
    zc = (BOW_Z[0] + BOW_Z[1]) / 2
    out = [k.box((0.0, 0.0, zc), (0.012, 0.060, BOW_T), PFA, label="bow_mid")]
    y_end, z_end = LEG_IN - 0.002, BAR_Z[0] - 0.010
    for s, tag in ((1, "l"), (-1, "r")):
        dy, dz = y_end - 0.030, z_end - zc
        ln, a = math.hypot(dy, dz), math.atan2(dz, dy)
        out.append(k.box((0.0, s * (0.030 + dy / 2), zc + dz / 2), (0.012, ln, BOW_T), PFA, turn=(s * a, 0.0, 0.0),
                         ghost=True, label=f"bow_{tag}"))
    return out


HALF_L = HANDLE_L / 2
FRAME_BOXES = [k.span((-BAR / 2, -HALF_L, BAR_Z[0]), (BAR / 2, HALF_L, BAR_Z[1]), PFA, label="bar")]
LEG_L, LEG_R, BOW = _leg(1), _leg(-1), _bow()
HANDLE = k.mech("handle", (0.0, 0.0, 0.0), FRAME_BOXES, HANDLE_KG - 0.07, color=PFA, surface=0.4, joints=[
    k.slider("legL", (0, 0, 0), "Y", 0.0, SPREAD, LEG_L, rate=LEG_RATE, target=0.0, damping=20.0, mass=0.03),
    k.slider("legR", (0, 0, 0), "Y", -SPREAD, 0.0, LEG_R, rate=LEG_RATE, target=0.0, damping=20.0, mass=0.03),
    k.slider("bow", (0, 0, 0), "Z", 0.0, BOW_TRAVEL, BOW, rate=BOW_RATE, target=0.0, damping=5.0, mass=0.01)])

# == the wet bench (frame: origin at the tank's centre on the table top) ================================================
DECK = 0.175
LIP = 0.010
TANK_IN, TANK_T, FLOOR = 0.200, 0.008, 0.008
REST_H, REST_X, REST_L = 0.025, (0.055, 0.076), 0.140
BENCH_KG = 40.0
CX0, CY = 0.240, -0.100  # carrier start: centre x, and the y of the carrier, deck and tank centre line
TX, TY = 0.430, CY  # the tank's centre
DECK_X = (0.100 - TX, -(TANK_IN / 2 + TANK_T))  # deck slab in front of the tank (bench frame x), toward the robot
DECK_HW = 0.120  # deck half-width in y


def _bench_boxes() -> list:
    sp = k.span
    a, t, rim = TANK_IN / 2, TANK_T, DECK + LIP
    out = [sp((DECK_X[0], -DECK_HW, 0.0), (DECK_X[1], DECK_HW, DECK), PP, label="deck"),
           sp((-a - t, -a - t, 0.0), (a + t, -a, rim), PP_DARK, label="tank_wall_neg"),
           sp((-a - t, a, 0.0), (a + t, a + t, rim), PP_DARK, label="tank_wall_pos"),
           sp((-a - t, -a, 0.0), (-a, a, rim), PP_DARK, label="tank_wall_front"),
           sp((a, -a, 0.0), (a + t, a, rim), PP_DARK, label="tank_wall_back"),
           sp((-a, -a, 0.0), (a, a, FLOOR), PP_DARK, label="tank_floor")]
    for s, tag in ((1, "back"), (-1, "front")):
        x0, x1 = sorted((s * REST_X[0], s * REST_X[1]))
        out.append(sp((x0, -REST_L / 2, FLOOR), (x1, REST_L / 2, FLOOR + REST_H), PFA, label=f"rest_{tag}"))
    return out


SPAWN = 0.0005
BENCH = k.mech("bench", (TX, TY, TOP + SPAWN), _bench_boxes(), BENCH_KG, color=PP, surface=0.4)
CARRIER_BOXES = _carrier_boxes()
CARRIER = k.mech("carrier", (CX0, CY, TOP + DECK + 2 * SPAWN), CARRIER_BOXES, CARRIER_KG + WAFER_N * WAFER_KG,
                 color=PFA, surface=0.4)
HANDLE = k.mech("handle", (CX0, CY, TOP + DECK + 2 * SPAWN + NOTCH_Z + 0.0005), HANDLE.boxes, HANDLE.mass,
                joints=HANDLE.joints, color=PFA, surface=0.4)
OBJECTS = [BENCH, CARRIER, HANDLE]
RULES = k.Rules(HANDLE, [k.Couple("bow", "legL", RATIO), k.Couple("bow", "legR", -RATIO)])

# == scoring ===========================================================================================================
LIFTED = 0.020  # every bottom corner at least 20 mm above the deck
OVER = 0.020  # carrier centre within 20 mm of the tank's centre in x and y
SEAT_TOL = 0.003  # bottom corners within 3 mm above the rests' tops
UPRIGHT = 5.0  # deg
TIP_ZERO = 15.0  # deg: tipped further at any time is a latched zero
DROP, DROP_T = 0.020, 0.25  # a fall: the carrier's centre 20 mm further down than either hand went, within 0.25 s
TOUCH_MM = 3.0  # a gripper within 3 mm of the carrier
OFF_MM = 10.0  # handle at least 10 mm from the carrier everywhere
STAND_DEG, FOOT_TOL = 10.0, 0.003  # handle standing within 10 deg, feet within 3 mm of the deck
CLEAR_MM = 30.0
STILL_S, STILL_TOL = 2.0, 0.002
BENCH_MOVE = 0.005
HZ = 20

CORNERS = [(sx * (RAIL[0] + RAIL[1]) / 2, sy * (HY - END_T - 0.005), 0.0) for sx in (1, -1) for sy in (1, -1)]
REST_TOP = TOP + FLOOR + REST_H
TINE_PT = {"legL": (0.0, (TINE_IN + LEG_IN) / 2 - 0.004, TINE_T / 2), "legR": (0.0, -((TINE_IN + LEG_IN) / 2 - 0.004), TINE_T / 2)}
FEET = [(sx * HANDLE_W / 2 * 0.9, sy * (LEG_IN + LEG_T / 2), 0.0) for sx in (1, -1) for sy in (1, -1)]
HANDLE_PARTS = (("", FRAME_BOXES), ("legL", LEG_L), ("legR", LEG_R), ("bow", [BOW[0]]))
CARRIER_SOLID = [b for b in CARRIER_BOXES if not S.look_of(b).ghost]


def engaged(cs, hs, leg: str) -> bool:
    """That leg's tine sits in its notch: its tip inside the end wall's outer face, within the notch's height."""
    p = cs.rotation().T @ (hs.to_world(TINE_PT[leg], leg) - np.asarray(cs.pos))
    inside = abs(p[1]) < HY - 0.001 and abs(p[0]) < NOTCH_W / 2
    return bool(inside and NOTCH_Z - 0.002 <= p[2] - TINE_T / 2 <= NOTCH_Z + NOTCH_H)


def corner_z(cs) -> np.ndarray:
    return np.array([cs.to_world(c)[2] for c in CORNERS])


def seated(cs) -> bool:
    z = corner_z(cs) - REST_TOP
    centre = np.asarray(cs.pos)
    inside = abs(centre[0] - TX) <= OVER + 0.01 and abs(centre[1] - TY) <= OVER + 0.01
    return bool(inside and np.all(z >= -0.001) and np.all(z <= SEAT_TOL + 1e-9) and cs.tilt_deg() <= UPRIGHT + 1e-9)


def handle_points(hs) -> np.ndarray:
    return np.concatenate([k.score.surface_points(hs, part, boxes, 0.02) for part, boxes in HANDLE_PARTS])


def handle_off(cs, hs) -> bool:
    p = (handle_points(hs) - np.asarray(cs.pos)) @ cs.rotation()
    m = OFF_MM / 1000
    inside = (np.abs(p[:, 0]) < HX + m) & (np.abs(p[:, 1]) < HY + 0.008 + m) & (p[:, 2] > -m) & (p[:, 2] < CARRIER_H + m)
    return not bool(inside.any())


def standing(hs) -> bool:
    feet = np.array([hs.to_world(f, "legL" if f[1] > 0 else "legR")[2] for f in FEET]) - (TOP + DECK)
    x = hs.pos[0] - TX
    on_deck = DECK_X[0] + 0.01 < x < DECK_X[1] - 0.01 and abs(hs.pos[1] - CY) < DECK_HW - 0.01
    return bool(on_deck and np.all(np.abs(feet) <= FOOT_TOL + 1e-9) and hs.tilt_deg() <= STAND_DEG + 1e-9)


def hand_heights(grips) -> tuple:
    """Each gripper's height (the mean of its hulls' points), () outside a simulator."""
    return tuple(float(np.mean(np.concatenate([np.atleast_2d(np.asarray(getattr(h, "vertices", h), dtype=float)) for _, h in hulls])[:, 2]))
                 for hulls in grips)


def fell(c_old: float, hands_old: tuple, c_new: float, hands_new: tuple) -> bool:
    """A fall, not a lowering: over the window the carrier's centre went down more than DROP further than every hand
    did. A carrier lowered by the hand at any speed goes down with it; one let go of, or sliding off the fingers, does
    not. With no hands known (offline), the carrier's own drop is judged."""
    drop = c_old - c_new
    if not hands_old or len(hands_old) != len(hands_new):
        return drop > DROP
    return all(drop - (ho - hn) > DROP for ho, hn in zip(hands_old, hands_new))


def released_early(lifted: bool, was_in: bool, both_in: bool, sit: bool) -> bool:
    """The tines' rule, judged at the moment a tine leaves its notch: before the lift they may come out (nothing is
    carried yet); once the carrier has been lifted on them, they may come out only while it sits on both rests (that is
    how the handle comes off for 1.0). Out anywhere else, the carrier was let go off the rests: a latched zero, whatever
    the height. With the handle already off, nothing more is judged by this rule."""
    return lifted and was_in and not both_in and not sit


ZERO = k.Sticky()
LIFT, OVER_TANK = k.Ever(), k.Ever()
END = k.Settle(STILL_S, STILL_TOL)
RUN = {"z": collections.deque(maxlen=int(round(DROP_T * HZ)) + 1), "bench0": None, "tilt_max": 0.0, "both": True}
REC = k.run_record.RunRecord(__file__, "carrier", "handle", "bench")


def grade(states) -> float:
    if ZERO:
        return 0.0
    cs, hs = states["carrier"], states["handle"]
    sit = seated(cs)
    done = sit and handle_off(cs, hs) and standing(hs) and END.ok
    return k.ladder((0.25, bool(LIFT)), (0.5, bool(OVER_TANK)), (0.75, sit), (1.0, done))


def score(states) -> float:
    return grade(states)


def success(states) -> bool:
    return grade(states) >= 1.0


def distance(states) -> float:
    cs = states["carrier"]
    return float(np.linalg.norm(np.asarray(cs.pos) - np.array([TX, TY, REST_TOP])))


@k.on_reset
def _reset(states) -> None:
    ZERO.reset(), LIFT.reset(), OVER_TANK.reset(), END.reset()
    RUN["z"].clear()
    RUN.update(bench0=np.asarray(states["bench"].pos, dtype=float), tilt_max=0.0, both=True)
    REC.reset()


@k.on_step
def _watch(states) -> None:
    cs, hs, bs = states["carrier"], states["handle"], states["bench"]
    z = float(corner_z(cs).min())
    grips = k.score.grippers()
    RUN["z"].append((float(cs.pos[2]), hand_heights(grips)))
    if len(RUN["z"]) > 1 and fell(*RUN["z"][0], *RUN["z"][-1]):
        ZERO.trip("the carrier fell (it went down 20 mm more than the hands within 0.25 s)")
    tilt = cs.tilt_deg()
    RUN["tilt_max"] = max(RUN["tilt_max"], tilt)
    if tilt > TIP_ZERO:
        ZERO.trip("the carrier tipped past 15 deg")
    surf = k.score.surface_points(cs, "", CARRIER_SOLID, 0.02)
    if grips and not k.hands_clear(surf, TOUCH_MM, grips):
        ZERO.trip("a gripper touched the carrier")
    if RUN["bench0"] is not None and np.linalg.norm(np.asarray(bs.pos) - RUN["bench0"]) > BENCH_MOVE:
        ZERO.trip("the bench moved")
    both = engaged(cs, hs, "legL") and engaged(cs, hs, "legR")
    t = S.LATEST.get("steps", 0) / HZ
    LIFT.update(both and z >= TOP + DECK + LIFTED, t)
    c = np.asarray(cs.pos)
    OVER_TANK.update(bool(LIFT) and abs(c[0] - TX) <= OVER and abs(c[1] - TY) <= OVER, t)
    if released_early(bool(LIFT), RUN["both"], both, seated(cs)):
        ZERO.trip("the tines came out before the carrier sat on the rests")
    RUN["both"] = both
    clear = k.hands_clear(np.concatenate([surf, handle_points(hs)]), CLEAR_MM, grips)
    END.push(np.concatenate([np.asarray(cs.pos), np.asarray(hs.pos), [math.radians(hs.tilt_deg())]]), clear)
    REC.tick(states, lambda: f"carrier z {(z - TOP) * 1000:.1f} mm tilt {tilt:.2f} max {RUN['tilt_max']:.2f} legs "
                             f"{hs.q['legL'] * 1000:.1f}/{hs.q['legR'] * 1000:.1f} bow {hs.q['bow'] * 1000:.1f} mm "
                             f"engaged {engaged(cs, hs, 'legL')}/{engaged(cs, hs, 'legR')} off {np.round(CTL.off * 1000, 1).tolist()} "
                             f"centre {RUN['z'][-1][0] * 1000:.1f} hands {[round(v * 1000, 1) for v in RUN['z'][-1][1]]} mm "
                             f"score {grade(states):g}")


# == the demo: the right hand, horizontal, jaws closing up and down ======================================================
# The hand pinches the top bar alone (lower finger in the 33 mm gap over the bow) and carries the hanging carrier on a
# servo: each control step a capped fraction of the carrier's position error is added to the hand's offset, so the
# arm's sag under the 9 N load is led out as it builds. Seated, the offset eases out; the hand opens, backs out, comes
# in again across bar and bow, squeezes the bow to spread the tines out of the notches and lifts the handle off.
ARM = "right"
OPEN_BAR = 0.030  # open round the bar: pads 15 mm either side of it
PINCH = BAR - 2 * 0.003  # 3 mm into each face: about 15 N a finger
OPEN_BOW = 0.075  # open round bar and bow (55 mm)
SQUEEZE = 0.034  # across bar and bow: the bow at its full 10 mm, about 28 N a finger
SQ_Z = (BOW_Z[0] + BAR_Z[1]) / 2  # TCP height in the handle frame for the squeeze
BACK = 0.040  # stand-off in front of the bar (fingertips 20 mm clear)
LIFT_D = 0.030
CARRY_X = TX - CX0
DOWN = (DECK + LIFT_D) - (FLOOR + REST_H) + 0.003  # to the rests, 3 mm past
SERVO_DT, SERVO_GAIN, SERVO_MAX = 0.15, 0.15, 0.090
RELAX = 0.85
LEAN_GAIN, LEAN_MAX = 0.15, math.radians(20.0)  # wrist lead on the carrier's lean
VIA0 = (0.250, -0.290, TOP + 0.400)  # straight up in front of the right arm's own base, clear of the handle
VIA = (CX0 - 0.040, CY - 0.140, TOP + 0.480)  # beside and above the handle: the way in from and out to home
VIA_TANK = (TX - 0.150, CY - 0.150, TOP + 0.450)


def _r0() -> np.ndarray:  # fingers along +x, jaws closing across z
    c = np.array([0.0, 0.0, 1.0])
    a = np.array([1.0, 0.0, 0.0])
    return np.column_stack([c, np.cross(a, c), a])


GRIP = D.Grip.from_matrix(_r0())
TCP_H = np.array([0.0, 0.0, BAR_C])  # TCP in the handle frame on the bar
TCP_SQ = np.array([0.0, 0.0, SQ_Z])
H0 = np.array(HANDLE.pos)  # nominal handle origin at the start


def _bar(st, back=0.0, local=TCP_H) -> np.ndarray:
    return states_handle(st).to_world(local) - np.array([back, 0.0, 0.0])


def states_handle(st):
    return st["handle"]


def _rot(ax: str, a: float) -> np.ndarray:
    c, s_ = math.cos(a), math.sin(a)
    if ax == "x":
        return np.array([[1, 0, 0], [0, c, -s_], [0, s_, c]])
    return np.array([[c, 0, s_], [0, 1, 0], [-s_, 0, c]])


def lean(cs) -> tuple:
    """The carrier's lean (rad): its up axis toward +x (about y) and toward -y (about x)."""
    r = cs.rotation()
    return math.atan2(r[0, 2], r[2, 2]), math.atan2(-r[1, 2], r[2, 2])


class LiveGrip(D.Grip):
    """A grip whose rotation is re-read from the servo every control step while its move runs (like a tracking
    ``D.Live``), then kept. Before the run steps it is ``fallback``."""

    def __new__(cls, fn, fallback, t1):
        g = cls._make(float(fallback), fallback.rotation, None)
        g.fn, g.fb, g.value, g.trial, g.t1, g.seen = fn, fallback, None, None, t1, -1
        return g

    def __tiltgrip_rotation__(self) -> np.ndarray:
        if not D.started():
            return np.array(self.fb.rotation)
        if self.trial != S.LATEST.get("resets"):
            self.value, self.trial, self.seen = None, S.LATEST.get("resets"), -1
        k_ = S.LATEST["steps"]
        track = D.now() <= self.t1 + 1e-9
        if self.value is None or (track and k_ != self.seen):
            self.value, self.seen = np.asarray(self.fn(), dtype=float), k_
        return np.array(self.value)


class Carry:
    """The hand's servo on the carrier's centre. ``servo(d)`` returns the TCP for the carrier wanted at its start pose
    plus ``d``; ``ease()`` the TCP with the offset decaying to zero."""

    def __init__(self):
        self.trial, self.off, self.c0, self.rel, self.step, self.last = None, np.zeros(3), None, None, None, None
        self.lead = np.zeros(2)  # wrist lead about y and about x, rad

    def _fresh(self, st) -> None:
        if self.trial != S.LATEST.get("resets"):
            self.trial, self.off, self.step, self.lead = S.LATEST.get("resets"), np.zeros(3), None, np.zeros(2)
            cs = st["carrier"]
            self.c0 = np.asarray(cs.pos, dtype=float)
            self.rel = _bar(st) - self.c0  # TCP relative to the carrier, as gripped

    def servo(self, d) -> np.ndarray:
        st = S.LATEST["states"]
        self._fresh(st)
        want = self.c0 + np.asarray(d, dtype=float)
        k_ = S.LATEST["steps"]
        if self.step != k_:
            self.step = k_
            err = want - np.asarray(st["carrier"].pos, dtype=float)
            self.off = np.clip(self.off + SERVO_GAIN * err, -SERVO_MAX, SERVO_MAX)
            self.lead = np.clip(self.lead - LEAN_GAIN * np.array(lean(st["carrier"])), -LEAN_MAX, LEAN_MAX)
        self.last = want + self.rel + self.off
        return self.last

    def ease(self, d) -> np.ndarray:
        st = S.LATEST["states"]
        self._fresh(st)
        k_ = S.LATEST["steps"]
        if self.step != k_:
            self.step = k_
            self.off = RELAX * self.off
            self.lead = RELAX * self.lead
        self.last = self.c0 + np.asarray(d, dtype=float) + self.rel + self.off
        return self.last


    def rotation(self) -> np.ndarray:
        return _rot("x", self.lead[1]) @ _rot("y", self.lead[0]) @ _r0()


CTL = Carry()
C0 = np.array(CARRIER.pos)
REL0 = H0 + TCP_H - C0


def _path(p: D.Plan, seconds: float, d0, d1, label: str, mode: str = "servo", finger=None) -> D.Plan:
    d0, d1 = np.asarray(d0, dtype=float), np.asarray(d1, dtype=float)
    n = max(1, int(round(seconds / SERVO_DT)))
    for i in range(n):
        u = (i + 1) / n
        f = u * u * (3.0 - 2.0 * u)
        d = d0 + f * (d1 - d0)
        fb = tuple(C0 + d + REL0)
        fn = (lambda st, d=d: CTL.servo(d)) if mode == "servo" else (lambda st, d=d: CTL.ease(d))
        grip = LiveGrip(CTL.rotation, GRIP, p.duration + seconds / n)
        p.move(seconds / n, fn, fallback=fb, grip=grip, label=label, finger=finger if i == 0 else None, track=True)
    return p


def _rel(base_fn, fb, off) -> tuple:
    """A live target: ``base_fn(states)`` plus ``off``, read when the move starts."""
    off = np.asarray(off, dtype=float)
    return (lambda st: np.asarray(base_fn(st), dtype=float) + off), tuple(np.asarray(fb, dtype=float) + off)


def _hand(how: str) -> D.Plan:
    """``how``: "demo"; "squeeze" (the natural grab over bar and bow, then lift and carry); "drop" (carried over the
    tank and let go there, the hand snatched back); "sag" (opened over the tank and pulled back slowly); "held" (carried over the tank and held); "seat" (seated, handle left on, let go)."""
    p = D.Plan(ARM)
    fb_bar = H0 + TCP_H
    fb_sq = H0 + TCP_SQ
    bar = lambda st: _bar(st)  # noqa: E731
    sq = lambda st: _bar(st, local=TCP_SQ)  # noqa: E731
    p.move(2.0, VIA0, grip=GRIP, finger=D.gap_finger(OPEN_BAR), label="up in front of its own base")
    p.move(1.5, VIA, grip=GRIP, label="up and out beside the handle")
    if how == "squeeze":
        f, fb = _rel(sq, fb_sq, (-BACK, 0, 0))
        p.move(3.0, f, fallback=fb, grip=GRIP, finger=D.gap_finger(OPEN_BOW), label="in front of bar and bow")
        f, fb = _rel(sq, fb_sq, (0, 0, 0))
        p.move(1.5, f, fallback=fb, grip=GRIP, label="jaws over bar and bow")
        p.fingers(SQUEEZE, 1.0, "close on the handle").hold(0.5)
        f, fb = _rel(sq, fb_sq, (0, 0, 0.08))
        p.move(2.0, f, fallback=fb, grip=GRIP, label="lift")
        f, fb = _rel(sq, fb_sq, (CARRY_X, 0, 0.08))
        p.move(4.0, f, fallback=fb, grip=GRIP, label="carry to the tank")
        p.hold(2.0)
        return p
    f, fb = _rel(bar, fb_bar, (-BACK, 0, 0))
    p.move(2.0, f, fallback=fb, grip=GRIP, label="in front of the top bar")
    f, fb = _rel(bar, fb_bar, (0, 0, 0))
    p.move(1.5, f, fallback=fb, grip=GRIP, label="fingers over and under the bar")
    p.fingers(PINCH, 1.0, "pinch the bar").hold(0.5)
    _path(p, 2.0, (0, 0, 0), (0, 0, LIFT_D), "lift 30 mm", finger=D.gap_finger(PINCH))
    _path(p, 0.6, (0, 0, LIFT_D), (0, 0, LIFT_D), "hold")
    _path(p, 4.0, (0, 0, LIFT_D), (CARRY_X, 0, LIFT_D), "carry over the tank")
    _path(p, 1.0, (CARRY_X, 0, LIFT_D), (CARRY_X, 0, LIFT_D), "steady over the tank")
    if how == "held":
        _path(p, 6.0, (CARRY_X, 0, LIFT_D), (CARRY_X, 0, LIFT_D), "hold over the tank")
        return p
    if how == "drop":  # a true release: open and snatch the hand straight back, so the lower finger leaves the bar
        # before the arm can sag with the load; the carrier falls into the tank
        p.fingers(OPEN_BAR, 0.3, "open")
        # the carry servo commands the hand CTL.off above the bar's nominal pose to lead out the arm's sag under the load;
        # once the load is let go the arm stops sagging, so half that lead keeps the hand at its real height as it goes
        # straight back (the full lead flicks the bar up as the arm unloads; none drags the hand down with the load)
        f, fb = _rel(lambda st: _bar(st) + CTL.off * np.array([1.0, 1.0, 0.5]), H0 + TCP_H + np.array([CARRY_X, 0.0, LIFT_D]),
                     (-BACK - 0.020, 0, 0))
        p.move(0.2, f, fallback=fb, grip=GRIP, label="snatch the hand back: it falls")
        p.hold(3.0)
        return p
    if how == "sag":  # open and pull back slowly: the bar rides the lower finger down as the arm sags with the load
        p.fingers(OPEN_BAR, 0.5, "open")
        f, fb = _rel(bar, H0 + TCP_H + np.array([CARRY_X, 0.0, LIFT_D]), (-BACK, 0, 0))
        p.move(0.8, f, fallback=fb, grip=GRIP, label="pull the hand back slowly: it slides down with the hand")
        p.hold(3.0)
        return p
    end = (CARRY_X, 0, LIFT_D - DOWN)
    _path(p, 5.0, (CARRY_X, 0, LIFT_D), end, "lower onto the rests")
    _path(p, 1.0, end, end, "let it sit")
    _path(p, 1.5, end, end, "ease the hand off", mode="ease")
    p.fingers(OPEN_BAR, 0.8, "open")
    if how == "seat":
        p.back(BACK, 1.2, "back out")
        p.move(2.0, VIA_TANK, grip=GRIP, label="up and out")
        p.move(2.0, VIA, grip=GRIP, label="beside the deck")
        p.move(1.5, VIA0, grip=GRIP, label="in front of its own base")
        p.home()
        return p
    in_tank = H0 + np.array([CARRY_X, 0.0, FLOOR + REST_H - DECK])  # handle origin with the carrier on the rests
    f, fb = _rel(bar, in_tank + TCP_H, (-BACK, 0, 0))
    p.move(1.2, f, fallback=fb, grip=GRIP, label="back out of the bar")
    f, fb = _rel(sq, in_tank + TCP_SQ, (-BACK, 0, 0))
    p.move(1.2, f, fallback=fb, grip=GRIP, finger=D.gap_finger(OPEN_BOW), label="down to bar and bow")
    f, fb = _rel(sq, in_tank + TCP_SQ, (0, 0, 0))
    p.move(1.5, f, fallback=fb, grip=GRIP, label="jaws over bar and bow")
    p.fingers(SQUEEZE, 1.0, "squeeze the bow").hold(0.6)
    f, fb = _rel(sq, in_tank + TCP_SQ, (0, 0, 0.100))
    p.move(2.5, f, fallback=fb, grip=GRIP, label="lift the handle off")
    deck = H0 - np.array([0.0, 0.0, NOTCH_Z]) + TCP_SQ  # the handle's feet on the deck at the carrier's start spot
    p.move(3.0, tuple(deck + np.array([0, 0, 0.060])), grip=GRIP, label="over the deck")
    p.move(2.0, tuple(deck + np.array([0, 0, -0.004])), grip=GRIP, label="stand it on its feet")
    p.fingers(OPEN_BOW, 1.0, "let go").hold(0.3)
    p.move(1.5, tuple(deck + np.array([-BACK - 0.02, 0, 0.0])), grip=GRIP, label="back out")
    p.move(2.0, VIA, grip=GRIP, label="up and out beside the handle")
    p.move(1.5, VIA0, grip=GRIP, label="in front of its own base")
    p.home()
    return p


def _plan(how: str) -> dict:
    p = _hand(how)
    return D.together(p, D.idle_plan("left", p.duration))


def demo():
    return D.choose(demo=lambda: _plan("demo"), squeeze=lambda: _plan("squeeze"), drop=lambda: _plan("drop"), sag=lambda: _plan("sag"),
                    held=lambda: _plan("held"), seat=lambda: _plan("seat"))


# == what the template tests read ======================================================================================
TEXT_NUMBERS = [("143 x 152 x 146 mm", CARRIER_H), ("4.76 mm pitch", PITCH), ("17.9 g each", WAFER_KG), ("0.625 mm thick", WAFER_T), ("about 0.30 kg", CARRIER_KG),
                ("60 mm wide, 8 mm high", NOTCH_W), ("8 mm high and 6 mm deep", NOTCH_H), ("6 mm deep in", NOTCH_D),
                ("118 mm up", NOTCH_Z), ("163 x 140 x 183 mm", HANDLE_H), ("about 0.15 kg", HANDLE_KG),
                ("bar 14 x 14 mm", BAR), ("33 mm below it", BOW_GAP), ("feet 140 mm wide", HANDLE_W),
                ("a 50 mm tine", TINE_W), ("closed at 2000 N/m", LEG_RATE), ("20 N at its stop", BOW_RATE * BOW_TRAVEL),
                ("up 10 mm", BOW_TRAVEL), ("up to 12 mm", SPREAD), ("up 5 mm", NOTCH_D / RATIO),
                ("deck 17.5 cm high", DECK), ("to 32.2 cm", TX + DECK_X[1]), ("8 mm walls", TANK_T),
                ("inside 200 x 200 mm", TANK_IN), ("x 43.0 cm", TX), ("y -10.0 cm", CY), ("10 mm proud", LIP),
                ("25 mm high, 21 mm wide and 140 mm long", REST_H), ("140 mm long", REST_L),
                ("65.5 mm either side", (REST_X[0] + REST_X[1]) / 2), ("x 24.0 cm", CX0), ("40 kg", BENCH_KG),
                ("20 mm further than either hand", DROP), ("within 0.25 s", DROP_T), ("more than 15 deg", TIP_ZERO),
                ("within 3 mm of the carrier", TOUCH_MM / 1000), ("more than 5 mm", BENCH_MOVE),
                ("at least 20 mm above the deck", LIFTED), ("within 20 mm of the tank", OVER),
                ("within 3 mm above the rests", SEAT_TOL), ("upright within 5 deg", UPRIGHT),
                ("at least 10 mm from it", OFF_MM / 1000), ("within 3 mm of the deck", FOOT_TOL),
                ("within 10 deg of upright", STAND_DEG), ("no gripper within 30 mm", CLEAR_MM / 1000),
                ("more than 2 mm over the last 2 s", STILL_TOL), ("over the last 2 s", STILL_S)]
# "drop": let go over the tank (the hand snatched back): a fall, zero. "sag": the hand opens and pulls back slowly; the
# bar rides the lower finger and the arm sags down with the carrier (MuJoCo: hand 1252 -> 1180 mm as the carrier goes
# down 70 mm), which wedges unseated on the tank wall: lowered with the hand, not a fall, so it keeps the 0.5 for carried
# over the tank, as "held" does.
CHEATS = {"none": 0.0, "squeeze": 0.0, "drop": 0.0, "sag": 0.5, "held": 0.5, "seat": 0.75}
HOLD = {"handle.legL": 0.5, "handle.legR": 0.5, "handle.bow": 0.5}
