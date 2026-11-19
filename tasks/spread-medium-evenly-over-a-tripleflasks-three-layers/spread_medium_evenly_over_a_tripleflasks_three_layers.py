"""Seed a TripleFlask evenly: lean it back on its side to 75 degrees, then roll it quickly flat onto its bottom face.

On the station table, in front of the robot: a Thermo Scientific Nunc TripleFlask (cat. 132867) holding 200 mL of
dyed water, capped, already stood on end to level and rested on its long side, top (imprint) face toward the robot.

- Its three culture compartments are joined at the closed end by a slot, so on its side they level together. Rolled
  flat, the upper compartments drain into the lower ones through that slot for as long as the roll lasts (rate set by
  the level difference). The maker's method leans the flask back toward its top face first (layers at 75 degrees),
  which gives the top layer a head start, then tips it quickly but gently into the growth position.
- Rolled from upright, or slowly, the bottom layer ends too full; rolled the wrong way it lies imprint down; landed
  hard, medium splashes into the neck.

The flask is one free rigid body. The medium is a declared analytic model (``Medium``): it only computes volumes from
the flask's pose, every 10 ms; nothing is moved, latched or held by code.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import ourkit as k  # noqa: E402
from ourkit import demo as D  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the 1.0 state already includes 3 s released and still
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Spread the medium in the TripleFlask evenly over its three layers, then leave the flask lying flat in "
               "its growth position, imprint up, with both hands off it.")
HZ, DT = 20.0, 0.05
# One-factor robustness runs only (README): TASK_SWEEP='{"friction": 1.2}' scales a property; unset in every trial.
SWEEP = json.loads(os.environ.get("TASK_SWEEP", "") or "{}")

# == the flask (x forward from the arm bases' midpoint, y left, z up from the table top; metres) ======================
# Body frame (origin at the body's centre; at the start it is the world frame): x across the thickness, the top
# (imprint) face at -x, the bottom face at +x; y along the length, the closed end at +y, neck and cap at -y; z across
# the width (up while the flask lies on its side).
BODY_L, WIDE, THICK = 0.205, 0.147, 0.056  # T175-size body
NECK_D, NECK_L, CAP_D, CAP_L = 0.030, 0.020, 0.040, 0.025  # straight neck; solid screw cap
FLASK_KG, EMPTY_KG = 0.45, 0.25  # with the 200 mL of medium; empty (estimate)
FRICTION = 0.35  # polystyrene on the stainless work surface and on the pads (estimate)
X_BOTTOM = 0.190  # the bottom face (the roll's pivot side) on its side
CAP_Y = -0.100  # the cap's centre
CAP_YL = -(BODY_L / 2 + NECK_L + CAP_L / 2)  # -0.135: the cap's centre in the body frame
START = (X_BOTTOM - THICK / 2, CAP_Y - CAP_YL, TOP + WIDE / 2 + 0.0005)
CLEAR, SHELF, CAP_BLUE, WHITE = (0.82, 0.88, 0.93), (0.70, 0.80, 0.90), (0.18, 0.33, 0.72), (0.96, 0.96, 0.96)


def _ring(y: float, length: float, across: float, color, n: int = 16) -> list:
    """A round part about the body's y axis as ``n`` flat plates (across flats ``across``)."""
    r, t = across / 2, 0.002
    w = 2 * r * math.tan(math.pi / n) + 0.0006
    return [k.box(((r - t / 2) * math.cos(a), y, (r - t / 2) * math.sin(a)), (t, length, w), color, turn=(0.0, -a, 0.0))
            for a in (2 * math.pi * i / n for i in range(n))]


def _flask_boxes() -> list:
    neck_y, cap_y = -(BODY_L / 2 + NECK_L / 2), CAP_YL
    out = [k.box((0.0, 0.0, 0.0), (THICK, BODY_L, WIDE), CLEAR, label="body")]
    out += [k.box((x, 0.010, 0.0), (0.0015, 0.170, 0.098), SHELF, ghost=True) for x in (-0.0075, 0.0075)]  # layers
    out.append(k.box((-THICK / 2 - 0.0003, 0.030, 0.0), (0.0006, 0.080, 0.040), WHITE, ghost=True))  # the imprint
    out += _ring(neck_y, NECK_L, NECK_D, CLEAR)
    out += _ring(cap_y, CAP_L, CAP_D, CAP_BLUE)
    out.append(k.box((0.0, cap_y - CAP_L / 2 + 0.001, 0.0), (0.026, 0.002, 0.026), CAP_BLUE))  # the cap's end
    return out


FLASK_BOXES = _flask_boxes()
FLASK = k.mech("flask", START, FLASK_BOXES, FLASK_KG * SWEEP.get("mass", 1.0), color=CLEAR,
               surface=FRICTION * SWEEP.get("friction", 1.0))
OBJECTS = [FLASK]
CORNERS = [(sx * THICK / 2, sy * BODY_L / 2, sz * WIDE / 2) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
CAP_PTS = [(sx * CAP_D / 2, CAP_YL - CAP_L / 2, sz * CAP_D / 2) for sx in (-1, 1) for sz in (-1, 1)]

# == the medium: an analytic three-compartment model (mm, mL; the card's numbers, estimates) ==========================
# Compartment frame (mm, origin at the cap's centre): X_b toward the top face (bottom, middle, top compartment centres
# at -15, 0, +15), Y_b across the width, Z_b along the length toward the neck. X_b = -x, Y_b = z, Z_b = -(y - CAP_YL).
MEDIUM_ML, EVEN_ML = 200.0, 200.0 / 3
GAP, CW, CL, PITCH = 13.0, 98.0, 170.0, 15.0  # compartment gap, width, length; layer pitch
LIP, K_SLOT, BORE_R = 7.0, 2.5, 14.0  # lip height; slot conductance mL/s per mm of level difference; neck bore radius
Z_LO = 1000 * (CAP_YL - BODY_L / 2) + 15.0  # the compartments' closed end (a 15 mm end chamber and wall)
SIDE_COLUMN_MM = EVEN_ML * 1000 / (GAP * CL)  # 30.2 mm: an even layer's depth on its side
CENTRES = (-PITCH, 0.0, PITCH)
K_RUN = K_SLOT * SWEEP.get("slot", 1.0)  # the slot rate used (TASK_SWEEP only in robustness runs)
SUBSTEPS = 5  # the model runs every 10 ms, the flask's pose interpolated between control steps
_UNIT = np.array([[i, j, kk] for i in (0, 1) for j in (0, 1) for kk in (0, 1)], float)
_SIGN = (-1.0) ** _UNIT.sum(1)


def _frac_below(g, t):
    """Fraction of a unit box with a*g0 + b*g1 + c*g2 <= t (any signs of g): a box under a level plane."""
    g = np.asarray(g, float)
    t = np.asarray(t, float) - np.where(g < 0, g, 0.0).sum()
    g = np.maximum(np.abs(g), 1e-3 * np.abs(g).max())
    v = (_SIGN * np.maximum(0.0, t[..., None] - _UNIT @ g) ** 3).sum(-1) / (6 * g.prod())
    return np.clip(v, 0.0, 1.0)


def _port(c):
    """Points of a compartment's neck-end opening (above its lip) that face the neck's bore."""
    pts = [(x, y, Z_LO + CL) for x in np.linspace(c - GAP / 2 + LIP, c + GAP / 2, 7) for y in np.linspace(-CW / 2, CW / 2, 99)
           if x * x + y * y <= BORE_R ** 2]
    return np.array(pts, float).reshape(-1, 3)


PORTS = [_port(c) for c in CENTRES]


def up_in_compartments(rot) -> np.ndarray:
    """The world's up direction in the compartment frame, from the flask's rotation."""
    u = np.asarray(rot, float)[2, :]
    return np.array([-u[0], u[2], -u[1]])


class Medium:
    """Volumes of the bottom, middle and top compartments and the neck chamber. Each compartment's free level comes
    from its volume and the flask's pose (a box under a level plane); neighbours exchange medium through the
    closed-end slot at K_SLOT x level difference while the source's medium stands above its 7 mm slot lip; medium
    above a neck-end lip, where the compartment faces the neck's bore, runs into the neck."""

    def __init__(self):
        self.V = np.full(3, EVEN_ML)
        self.neck = 0.0

    def levels(self, up):
        g = up * np.array([GAP, CW, CL])
        z0 = np.array([up @ np.array([c - GAP / 2, -CW / 2, Z_LO]) for c in CENTRES])
        lo, hi = z0 + np.minimum(g, 0).sum(), z0 + np.maximum(g, 0).sum()
        f = self.V / (GAP * CW * CL / 1000.0)
        for _ in range(36):
            mid = 0.5 * (lo + hi)
            below = np.array([_frac_below(g, mid[i] - z0[i]) for i in range(3)])
            lo, hi = np.where(below < f, mid, lo), np.where(below < f, hi, mid)
        return 0.5 * (lo + hi), z0, g

    def step(self, up, dt: float) -> None:
        h, z0, g = self.levels(up)
        a = LIP / GAP
        slot = z0 + min(x * g[0] + y * g[1] for x in (a, 1.0) for y in (0.0, 1.0))
        for i, j in ((2, 1), (1, 0)):
            d = h[i] - h[j]
            s, r = (i, j) if d > 0 else (j, i)
            if h[s] > slot[s]:
                q = min(K_RUN * abs(d) * dt, self.V[s])
                self.V[s] -= q
                self.V[r] += q
        for i in range(3):
            if len(PORTS[i]):
                pz = float((PORTS[i] @ up).min())
                if h[i] > pz:
                    q = min(K_RUN * (h[i] - pz) * dt, self.V[i])
                    self.V[i] -= q
                    self.neck += q


# == scoring ===========================================================================================================
BAND_FULL, BAND_HALF = 0.10, 0.25  # every layer within 10% (1.0) or 25% (0.5) of an even split
FLAT_DEG = 5.0  # bottom face down within 5 degrees of flat
REST_MM = 5.0  # its lowest point within 5 mm of the table
NECK_ZERO_ML = 0.5  # more than this in the neck chamber at any time: 0
SPLASH_V = 0.5  # m/s: a corner reaching the table faster than this splashes medium into the neck: 0
NEAR_TABLE = 0.006  # m: a corner this close to the table is landing
OFF_TABLE = 0.010  # any point this far below the table top: the flask has left the table: 0
CLEAR_MM = 20.0  # released: no gripper within 2 cm of the flask
STILL_S, STILL_MM = 3.0, 0.001  # still: no point moving more than 1 mm over the last 3 s
LEAN_DEG = 15.0  # the maker's lean: layers at 75 degrees to the bench

NECK, SPLASH, OFF, END = k.Sticky(), k.Sticky(), k.Sticky(), k.Settle(STILL_S, STILL_MM)
RUN = {"med": Medium(), "up": None, "z": None, "land_v": 0.0, "roll": 0.0}
REC = k.run_record.RunRecord(__file__, "flask")


def roll_deg(st) -> float:
    """The flask's roll about its long axis: 0 on its side as it starts, -15 leaned back, +90 flat imprint up."""
    r = st.rotation()
    return math.degrees(math.atan2(r[0, 2], r[2, 2]))


def flat_imprint_up(st) -> bool:
    top_face_normal = -st.rotation()[:, 0]
    return top_face_normal[2] >= math.cos(math.radians(FLAT_DEG))


def resting(st) -> bool:
    z = min(float(st.to_world(c)[2]) for c in CORNERS) - TOP
    return z <= REST_MM / 1000


def layers_within(band: float) -> bool:
    return all(abs(v - EVEN_ML) <= EVEN_ML * band + 1e-6 for v in RUN["med"].V)


def grade(states) -> float:
    st = states["flask"]
    if NECK or SPLASH or OFF:
        return 0.0
    lying = flat_imprint_up(st) and resting(st) and END.ok
    return k.ladder((0.25, lying), (0.5, layers_within(BAND_HALF)), (1.0, layers_within(BAND_FULL)))


def score(states) -> float:
    return grade(states)


def success(states) -> bool:
    return grade(states) >= 1.0


def distance(states) -> float:
    return float(np.max(np.abs(RUN["med"].V - EVEN_ML)))


@k.on_reset
def _reset(states) -> None:
    NECK.reset(), SPLASH.reset(), OFF.reset(), END.reset()
    st = states["flask"]
    RUN.update(med=Medium(), up=up_in_compartments(st.rotation()), z=None, land_v=0.0, roll=roll_deg(st))
    REC.reset()


@k.on_step
def _watch(states) -> None:
    st = states["flask"]
    med, up = RUN["med"], up_in_compartments(st.rotation())
    prev = RUN["up"] if RUN["up"] is not None else up
    for i in range(1, SUBSTEPS + 1):
        u = prev + (up - prev) * i / SUBSTEPS
        med.step(u / np.linalg.norm(u), DT / SUBSTEPS)
    RUN["up"], RUN["roll"] = up, roll_deg(st)
    if med.neck > NECK_ZERO_ML:
        NECK.trip("medium in the neck")
    z = np.array([st.to_world(c)[2] for c in CORNERS + CAP_PTS]) - TOP
    if RUN["z"] is not None:
        v = (RUN["z"] - z) / DT
        near = z < NEAR_TABLE
        if near.any():
            RUN["land_v"] = max(RUN["land_v"], float(v[near].max()))
            if float(v[near].max()) > SPLASH_V:
                SPLASH.trip("landed hard: medium splashed into the neck")
    RUN["z"] = z
    if float(z.min()) < -OFF_TABLE:
        OFF.trip("the flask left the table")
    surf = k.score.surface_points(st, "", [b for b in FLASK_BOXES if not b[3].ghost], 0.03)
    END.push(np.array([st.to_world(c) for c in CORNERS]).ravel(), k.hands_clear(surf, CLEAR_MM))
    REC.tick(states, lambda: f"roll {RUN['roll']:+.1f} V {'/'.join('%.1f' % v for v in med.V)} neck {med.neck:.2f} "
                             f"land {RUN['land_v']:.2f} m/s low {1000 * float(z.min()):.1f} mm score {grade(states):g}")


# == the demo: one hand holds the cap across its diameter and turns with the flask about its lower edges =============
# The wrist turns with the flask (a rigid grasp of the cap: about 4.5 mm into each side, about 40 N), so the hand's
# tilt follows the roll: 55 deg away from the robot on its side, 70 at the lean, 35 toward the robot flat (IK sweep:
# 0-1 mm along the whole path). The edges and cap are read from the flask's live pose when the grasp closes.
TILT0 = -55.0  # deg: the hand's tilt about y on its side (negative: fingertips away from the robot)
DEPTH = 0.004  # m: the TCP this far back from the cap's axis along the grip
OUT_Y = 0.008  # m: the TCP this far out from the cap's middle toward its free end (the fingers' 34 mm wide roots
#                then clear the body's neck-end face by 7 mm; the 26 mm pads still cover 17 mm of the cap)
SQUEEZE = 0.0045  # m commanded into each side of the cap
OPEN_GAP = CAP_D + 2 * D.P.OPEN_CLEARANCE
T_LEAN, T_LEAN_HOLD, T_SET = 1.5, 3.0, 0.5
T_ROLL, ROLL_TO, T_TAIL = 0.5, 75.0, 0.3
EDGE_T = (-THICK / 2, CAP_YL, -WIDE / 2)  # the top face's lower edge, at the cap's end
EDGE_B = (THICK / 2, CAP_YL, -WIDE / 2)  # the bottom face's lower edge
CAP_C = (0.0, CAP_YL, 0.0)
_RD = np.asarray(D.Grip.deg(yaw=90).rotation)


def roty(deg: float) -> np.ndarray:
    t = math.radians(deg)
    return np.array([[math.cos(t), 0, math.sin(t)], [0, 1, 0], [-math.sin(t), 0, math.cos(t)]])


def grip(theta: float) -> D.Grip:
    """The hand's grip with the flask rolled ``theta`` deg from its start (turning with it)."""
    return D.Grip.from_matrix(roty(theta + TILT0) @ _RD)


def _geometry(st=None) -> tuple:
    """(cap centre, top-face lower edge, bottom-face lower edge) in the world: live, or as planned at the start."""
    st = k.mech_state(FLASK, START) if st is None else st.get("flask", st) if isinstance(st, dict) else st
    return tuple(np.asarray(st.to_world(p), float) for p in (CAP_C, EDGE_T, EDGE_B))


def tcp_at(theta: float, geo) -> np.ndarray:
    """Where the TCP goes with the flask rolled ``theta`` deg about its lower edges (top edge leaning back, bottom
    edge rolling over)."""
    cap, et, eb = geo
    piv = et if theta < 0 else eb
    c = piv + roty(theta) @ (cap - piv)
    return c - DEPTH * np.asarray(grip(theta).approach) - np.array([0.0, OUT_Y, 0.0])


def hand(how: str = "demo", lean: float = LEAN_DEG, t_roll: float = T_ROLL) -> D.Plan:
    """The right hand: take the cap, lean, hold, roll flat, set down, let go, back off. ``how``: "demo", "slam" (let go
    45 deg into the roll) or "held" (never lets go)."""
    p = D.Plan("right")
    geo0 = _geometry()
    a0 = np.asarray(grip(0.0).approach)

    def geo(st=None):
        return p.memo.get("geo", geo0)

    def keep_geo(st):
        p.memo["geo"] = _geometry(st)
        return tcp_at(0.0, p.memo["geo"])

    p.move(2.0, lambda st: tcp_at(0.0, _geometry(st)) - 0.050 * a0, finger=D.gap_finger(OPEN_GAP),
           label="back along the grip from the cap", grip=grip(0.0), fallback=tcp_at(0.0, geo0) - 0.050 * a0)
    p.move(1.5, keep_geo, label="fingers round the cap", fallback=tcp_at(0.0, geo0))
    p.fingers(CAP_D - 2 * SQUEEZE, D.P.T_CLOSE, "pinch the cap").hold(0.5, "settle the grip")
    n = int(round(T_LEAN * HZ))
    for i in range(1, n + 1):
        th = -lean * i / n
        p.move(DT, lambda st, th=th: tcp_at(th, geo()), label="lean it back", grip=grip(th),
               fallback=tcp_at(th, geo0))
    want = lambda st: tcp_at(-lean, geo())  # noqa: E731
    have = lambda st: _cap_now(st) - DEPTH * np.asarray(grip(-lean).approach) - (0, OUT_Y, 0)  # noqa: E731
    p.memo["lean_base"] = tcp_at(-lean, geo0)  # the lean's last command (the planned edges: the flask has not moved)
    D.corrected(p, 0.5, want, have, key="lean", base="lean_base", max_fix=0.015, label="correct the lean", fallback=tcp_at(-lean, geo0))
    p.hold(T_LEAN_HOLD - 0.5, "hold the lean")
    lean_fix = lambda: np.asarray(p.memo.get("lean", tcp_at(-lean, geo0))) - tcp_at(-lean, geo())  # noqa: E731
    path = _roll_path(lean, t_roll)
    for i, th in enumerate(path):
        if how == "slam" and th > 45.0:
            return p.fingers(OPEN_GAP, 0.2, "let go mid-roll").hold(1.0).home()
        fade = max(0.0, 1.0 - (i + 1) / len(path))
        p.move(DT, lambda st, th=th, fade=fade: tcp_at(th, geo()) + fade * lean_fix(), label="roll it flat",
               grip=grip(th), fallback=tcp_at(th, geo0))
    def flat_here(press):
        """The hand where the flask lies flat at its own place: the cap's live x and y, its height when flat."""
        def fn(st):
            c = _cap_now(st)
            want = np.array([c[0], c[1], TOP + THICK / 2 - press])
            return want - DEPTH * np.asarray(grip(90.0).approach) - (0, OUT_Y, 0)
        return fn

    p.move(T_SET, D.remember(p, "flat", flat_here(0.002)), label="set it down flat", fallback=tcp_at(90.0, geo0))
    p.hold(0.3, "let it settle")
    if how == "held":
        return p.hold(8.0, "stay on the cap")
    p.release(seconds=D.P.T_RELEASE)
    off = lambda d, up: lambda st: np.asarray(p.memo["flat"]) - d * np.asarray(grip(90.0).approach) + (0, 0, up)  # noqa: E731
    fb = tcp_at(90.0, geo0)
    p.move(1.0, off(0.025, 0.010), label="back off the cap", grip=grip(70.0), fallback=fb + (0.014, 0, 0.020))
    p.move(1.0, off(0.025, 0.060), label="up clear of the flask", grip=D.Grip.deg(yaw=90),
           fallback=fb + (0.014, 0, 0.080))
    return p.home(hold=4.0)


def _roll_path(lean: float, t_roll: float) -> list:
    """The roll's angle at each 50 ms step: a smooth turn from the lean to ROLL_TO deg in ``t_roll``, then a gentler
    T_TAIL s onto the bottom face (the arm lags under the falling flask; the tail keeps its edge's landing soft)."""
    def smooth(a, b, n):
        return [a + (b - a) * (3 * u * u - 2 * u ** 3) for u in (i / n for i in range(1, n + 1))]
    return smooth(-lean, ROLL_TO, max(1, round(t_roll * HZ))) + smooth(ROLL_TO, 90.0, max(1, round(T_TAIL * HZ)))


def _cap_now(states) -> np.ndarray:
    return np.asarray(states["flask"].to_world(CAP_C), float)


def demo():
    def two(**kw):
        right = hand(**kw)
        return D.together(D.idle_plan("left", 0.1), right)

    return D.choose(demo=lambda: two(), no_lean=lambda: two(lean=0.0), slow_roll=lambda: two(t_roll=4.0),
                    over_lean=lambda: two(lean=40.0),
                    slam=lambda: two(how="slam"), held=lambda: two(how="held"))


# == what the template tests read ======================================================================================
TEXT_NUMBERS = [("205 mm", BODY_L), ("147 mm", WIDE), ("56 mm", THICK), ("30 mm neck", NECK_D), ("20 mm long", NECK_L),
                ("40 mm cap", CAP_D), ("25 mm long", CAP_L), ("0.45 kg", FLASK_KG), ("0.25 kg", EMPTY_KG),
                ("13 mm gap", GAP / 1000),
                ("15 mm pitch", PITCH / 1000), ("7 mm lips", LIP / 1000), ("28 mm bore", 2 * BORE_R / 1000),
                ("30.2 mm deep", SIDE_COLUMN_MM / 1000), ("x 19 cm", X_BOTTOM), ("x 13.4 cm", X_BOTTOM - THICK),
                ("y -10 cm", CAP_Y), ("7.35 cm up", WIDE / 2),
                ("faster than 0.5 m/s", SPLASH_V), ("within 2 cm", CLEAR_MM / 1000), ("more than 1 mm", STILL_MM),
                ("last 3 s", STILL_S), ("1 cm below", OFF_TABLE)]
# mL, degrees and friction (units testkit does not parse) are checked number for number by spec_check.
CHEATS = {"none": 0.0, "no_lean": 0.5, "slow_roll": 0.25, "over_lean": 0.5, "slam": 0.0, "held": 0.0}  # mode: the most it may score
HOLD = {}  # no joints; the flask is watched as a body
