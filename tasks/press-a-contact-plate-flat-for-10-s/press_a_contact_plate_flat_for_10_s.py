"""Take a surface sample with an agar contact plate: press it flat on the surface, steadily, for 10 s, then close it.

On the station table: a 120 mm sample tray holding a 65 mm agar contact plate stored the usual way, agar down, its base
resting on its lid's inner ledge; to its left front, a stainless sampling plate on a bench scale with an 85 mm circle
taped on it.

- The maker's instructions (Biokar contact plates, NF EN ISO 18593): open the plate, apply the agar directly to the
  surface and keep a uniform pressure, for example 500 g for 10 s, then close the plate.
- The agar stands 1.5 mm proud of the plate's rim as a convex meniscus. Pressed too hard, the rim comes down on the
  surface and the agar is crushed; tapped, rocked or dragged, the imprint is incomplete or smeared; agar touched to
  anything else is a contaminated sample.

The agar is a disc on a sprung slider in the base (declared 5 N/mm, its force the press force); the scorer only reads
poses and that compression every control step. Nothing is moved, latched or held by code.
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
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the 1.0 state already includes 1 s released and still
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Take a surface sample inside the marked circle with the contact plate, then put the plate back in its "
               "lid on the tray and let go.")
HZ, DT = 20.0, 0.05
SWEEP = json.loads(os.environ.get("TASK_SWEEP", "") or "{}")
R = math.radians
GAP = 0.0005
PS, AGAR_C, TRAY_C, STEEL, SCALE_C, TAPE = (0.90, 0.92, 0.94), (0.86, 0.72, 0.42), (0.80, 0.82, 0.86), \
    (0.74, 0.75, 0.77), (0.20, 0.21, 0.23), (0.95, 0.80, 0.10)

# -- fixed: the tray and the sampling plate on its scale ------------------------------------------------------------------
TRAY_XY, TRAY_SIZE, TRAY_T = (0.20, -0.19), 0.120, 0.003
TRAY_TOP = TOP + TRAY_T
SURF_XY, SURF_SIZE, SCALE_H, STEEL_T = (0.26, 0.02), 0.200, 0.030, 0.002
S_TOP = TOP + SCALE_H + STEEL_T  # the sampled surface, 32 mm above the table
MARK_D = 0.085  # the taped circle
MARK_XY = SURF_XY


def _fixed() -> list:
    out = [Box("tray", pos=(*TRAY_XY, TOP + TRAY_T / 2), size=(TRAY_SIZE, TRAY_SIZE, TRAY_T), color=TRAY_C),
           Box("scale", pos=(*SURF_XY, TOP + SCALE_H / 2), size=(SURF_SIZE, SURF_SIZE, SCALE_H), color=SCALE_C),
           Box("steel", pos=(*SURF_XY, TOP + SCALE_H + STEEL_T / 2), size=(SURF_SIZE, SURF_SIZE, STEEL_T), color=STEEL)]
    n, r = 24, MARK_D / 2
    for i in range(n):
        a = 2 * math.pi * i / n
        out.append(Box(f"tape{i}", pos=(MARK_XY[0] + r * math.cos(a), MARK_XY[1] + r * math.sin(a), S_TOP + 0.00015),
                       size=(0.004, 2 * r * math.tan(math.pi / n) + 0.0005, 0.0003), color=TAPE, yaw=a))
    return out


# -- the plate: lid and base (free), agar on a slider in the base --------------------------------------------------------
PLATE_D, BASE_H, WALL = 0.065, 0.016, 0.0012
LID_D, LID_H, LID_T = 0.070, 0.009, 0.001
LEDGE_H = 0.005  # the lid's inner ledge the base rim rests on: the agar hangs 2.5 mm off the lid floor
AGAR_D, AGAR_PROUD, AGAR_SLAB = 0.056, 0.0015, 0.006  # the crossed boxes' corners reach 30.3 mm
K_AGAR_N_PER_M = 5000.0  # declared; real agar is stiffer
K_AGAR = K_AGAR_N_PER_M * SWEEP.get("spring", 1.0)
BASE_KG, LID_KG, AGAR_KG = 0.010 * SWEEP.get("mass", 1.0), 0.010, 0.015


def ring(z0, z1, across, color, n=16, t=0.0012, tag="") -> list:
    r = across / 2
    w = 2 * r * math.tan(math.pi / n) + 0.0006
    return [k.box(((r - t / 2) * math.cos(a), (r - t / 2) * math.sin(a), (z0 + z1) / 2), (t, w, z1 - z0), color,
                  turn=(0.0, 0.0, a), label=f"{tag}{i}") for i, a in enumerate(2 * math.pi * j / n for j in range(n))]


def disc(z0, z1, across, color, tag, **look) -> list:
    """A round slab as four crossed boxes (a 16-gon)."""
    w = across * 0.414
    return [k.box((0, 0, (z0 + z1) / 2), (across, w, z1 - z0), color, turn=(0.0, 0.0, a), label=f"{tag}{i}", **look)
            for i, a in enumerate((0.0, math.pi / 4, math.pi / 2, 3 * math.pi / 4))]


def _lid_boxes() -> list:  # frame origin: the middle of the lid's underside
    out = disc(0.0, LID_T, LID_D - 0.001, PS, "lfloor")
    out += ring(0.0, LID_H, LID_D, PS, t=LID_T, tag="lwall")
    out += ring(LID_T, LEDGE_H, LID_D - 2 * LID_T, PS, t=0.003, tag="ledge")  # radius 31-34 mm, under the base rim
    return out


def _base_boxes() -> list:  # frame origin: the middle of the base's rim plane; stored agar down, so it rises from there
    out = ring(0.0, BASE_H, PLATE_D, PS, t=WALL, tag="bwall")
    out += disc(BASE_H - WALL, BASE_H, PLATE_D - 0.002, PS, "bfloor")
    return out


AGAR_BOXES = disc(-AGAR_PROUD, -AGAR_PROUD + AGAR_SLAB, AGAR_D, AGAR_C, "agar")
AGAR = k.slider("agar", (0.0, 0.0, 0.0), "Z", 0.0, AGAR_PROUD, AGAR_BOXES, rate=K_AGAR, target=0.0, damping=20.0,
                mass=AGAR_KG, weightless=True)  # q > 0: the agar pushed into the base
LID_BOXES, BASE_BOXES = _lid_boxes(), _base_boxes()
LID_START = (*TRAY_XY, TRAY_TOP + GAP)
BASE_START = (*TRAY_XY, TRAY_TOP + LEDGE_H + 2 * GAP)
LID = k.mech("lid", LID_START, LID_BOXES, LID_KG, color=PS, surface=0.5)
BASE = k.mech("base", BASE_START, BASE_BOXES, BASE_KG, joints=[AGAR], color=PS, surface=0.6)
OBJECTS = [LID, BASE, *_fixed()]

# -- the scorer -------------------------------------------------------------------------------------------------------
F_LOW, F_HIGH = 3.0, 7.0  # N: 300-700 g on the scale (ISO 18593's 500 g)
PRESS_S = 9.0
FLAT = R(3.0)
SLIDE = 0.002
ON_MARK = 0.010
SEAT = 0.003
Q_TOUCH = 0.0001  # agar in by 0.1 mm (0.5 N): touching
Q_OFF = 0.00002  # and off again only below 0.02 mm (hysteresis)
Q_DIRTY = 0.0003  # agar in by 0.3 mm (1.5 N) anywhere but on the sampling plate: contaminated
Q_RIM = AGAR_PROUD - 0.00005  # the rim is down
CLEAR_MM = 30.0
STILL_S, STILL_TOL = 1.0, 0.002
AGAR_FACE = (0.0, 0.0, -AGAR_PROUD)  # the agar face's middle at q = 0, in the agar part's frame


def press_force(q: float) -> float:
    return K_AGAR * min(max(q, 0.0), AGAR_PROUD)


def tilt(st) -> float:
    return math.acos(float(np.clip(st.rotation()[2, 2], -1.0, 1.0)))


def on_surface(base) -> bool:
    """The agar face is down on the sampling plate (its middle over the steel, within 3 mm of its top)."""
    f = base.to_world(AGAR_FACE, "agar")
    return (abs(f[0] - SURF_XY[0]) < SURF_SIZE / 2 and abs(f[1] - SURF_XY[1]) < SURF_SIZE / 2
            and abs(f[2] - S_TOP) < 0.003 and base.rotation()[2, 2] > 0.9)


def mark_offset(base) -> float:
    return math.hypot(base.pos[0] - MARK_XY[0], base.pos[1] - MARK_XY[1])


class Log:
    """Touches, the press and its quality, from the agar's compression and the base's pose each step."""

    def __init__(self):
        self.touching, self.touches, self.on_mark, self.slid, self.rim = False, 0, False, False, False
        self.run, self.best, self.xy0, self.max_f = 0.0, 0.0, None, 0.0

    def step(self, base) -> str:
        q = float(base.q["agar"])
        f = press_force(q)
        xy = np.asarray(base.pos[:2], float)
        if q > Q_TOUCH and not self.touching:
            self.touching, self.xy0 = True, xy
            self.touches += 1
        elif q <= Q_OFF and self.touching:
            self.touching, self.run = False, 0.0
        if not self.touching:
            return ""
        if not on_surface(base):
            return "the agar touched something other than the sampling surface" if q > Q_DIRTY else ""
        self.max_f = max(self.max_f, f)
        self.rim |= q >= Q_RIM
        self.slid |= float(np.linalg.norm(xy - self.xy0)) > SLIDE
        here = mark_offset(base) <= ON_MARK
        self.on_mark |= here and f >= F_LOW
        good = here and F_LOW <= f <= F_HIGH and tilt(base) <= FLAT
        self.run = self.run + DT if good else 0.0
        self.best = max(self.best, self.run)
        return ""


def seated(base, lid) -> bool:
    d = np.asarray(base.pos, float) - np.asarray(lid.pos, float)
    return (math.hypot(d[0], d[1]) <= SEAT and abs(d[2] - LEDGE_H) <= 0.002 and base.rotation()[2, 2] > 0.996
            and float(base.q["agar"]) <= Q_TOUCH)


def on_tray(lid) -> bool:
    return (abs(lid.pos[0] - TRAY_XY[0]) < TRAY_SIZE / 2 and abs(lid.pos[1] - TRAY_XY[1]) < TRAY_SIZE / 2
            and abs(lid.pos[2] - TRAY_TOP) < 0.003 and tilt(lid) < R(5.0))


DIRTY, DROP, END = k.Sticky(), k.Sticky(), k.Settle(STILL_S, STILL_TOL)
RUN = {"log": Log()}
REC = k.run_record.RunRecord(__file__, "base", "lid")


def grade(states) -> float:
    if DIRTY or DROP:
        return 0.0
    b, lid, g = states["base"], states["lid"], RUN["log"]
    held = g.best >= PRESS_S - 1e-9
    three = held and g.touches == 1 and not g.slid and not g.rim and seated(b, lid)
    return k.ladder((0.25, g.on_mark), (0.5, held), (0.75, three), (1.0, three and on_tray(lid) and END.ok))


def score(states) -> float:
    return grade(states)


def success(states) -> bool:
    return grade(states) >= 1.0


def distance(states) -> float:
    return max(0.0, PRESS_S - RUN["log"].best)


@k.on_reset
def _reset(states) -> None:
    DIRTY.reset(), DROP.reset(), END.reset()
    RUN["log"] = Log()
    REC.reset()


@k.on_step
def _watch(states) -> None:
    b, lid, g = states["base"], states["lid"], RUN["log"]
    why = g.step(b)
    if why:
        DIRTY.trip(why)
    if b.pos[2] < TOP - 0.01 or lid.pos[2] < TOP - 0.01 or tilt(b) > R(100) or tilt(lid) > R(60):
        DROP.trip("the plate fell")
    surf = np.concatenate([k.score.surface_points(b, "", BASE_BOXES, 0.015), k.score.surface_points(lid, "", LID_BOXES, 0.015)])
    END.push(np.concatenate([np.asarray(b.pos, float), np.asarray(lid.pos, float)]), k.hands_clear(surf, CLEAR_MM))
    REC.tick(states, lambda: f"q {float(b.q['agar']) * 1000:.2f} mm F {press_force(float(b.q['agar'])):.1f} N run "
                             f"{g.run:.1f} best {g.best:.1f} s touches {g.touches} slid {g.slid} rim {g.rim} "
                             f"seated {seated(b, lid)} score {grade(states):g}")
    if os.environ.get("TASK_DEBUG"):
        RUN["n"] = RUN.get("n", 0) + 1
        if RUN["n"] % 10 == 0:
            print(f"DBG t {RUN['n'] * DT:.1f} q {float(b.q['agar']) * 1000:.2f} base {np.round(np.asarray(b.pos) * 1000, 1)} "
                  f"tilt {math.degrees(tilt(b)):.1f} run {g.run:.1f} best {g.best:.1f} touch {g.touches} slid {g.slid} "
                  f"rim {g.rim} mark {g.on_mark} seated {seated(b, lid)} g {grade(states)}", flush=True)


# -- the demo ---------------------------------------------------------------------------------------------------------
G = D.Grip.deg(yaw=0)  # straight down, fingers closing along y
SQUEEZE = 0.002
TIP_BELOW_TCP = 0.013  # the pad tips sit this far below the TCP
RIM_CLEAR = 0.003  # the pad tips never go lower than this above the lid's rim
ABOVE_TOP = 0.004  # the TCP this far above the base's top: the pad tips stay above the lid rim
LIFT = 0.030
F_T = 4.5  # N: the press target (ISO 18593's 500 g, a little under to leave room for the arm's lag)
Q_BACK = 6.0 / K_AGAR  # 6.0 N (1.2 mm at 5 N/mm): back off at full speed
LAG_S = 2.0  # s: the servo acts on the compression expected this far ahead (Isaac's arm kept pressing ~5 s)
Q_IN, Q_OUT = 3.3 / K_AGAR, 6.7 / K_AGAR  # 3.3-6.7 N (0.66-1.34 mm at 5 N/mm): the demo's own band for its clock
KI = 2.0  # 1/s: command speed per metre of compression error
KD = 8.0  # command speed per unit compression speed (brakes the lagging approach)
V_DOWN = 0.005  # m/s down to first touch
HOVER = 0.006  # the agar this far above the surface before the descent


def base_tcp(st) -> np.ndarray:
    return st["base"].to_world((0.0, 0.0, BASE_H + ABOVE_TOP))


def hand(how: str) -> D.Plan:
    """Lift the base off its lid, press it on the circle (``how``: "demo", "tap", "hard", "slide", "held"), close it."""
    p = D.Plan("right")
    b0 = np.array([*TRAY_XY, BASE_START[2] + BASE_H + ABOVE_TOP])
    p.move(2.0, lambda st: base_tcp(st) + (0.0, 0.0, 0.05), finger=D.gap_finger(PLATE_D + 2 * D.P.OPEN_CLEARANCE),
           grip=G, label="above the plate", fallback=b0 + (0, 0, 0.05))
    p.move(1.2, D.remember(p, "grip", base_tcp), label="fingers round the base", fallback=b0)
    p.fingers(PLATE_D - 2 * SQUEEZE, D.P.T_CLOSE, "pinch the base").hold(0.3)
    p.move(1.0, D.remember(p, "up", lambda st: np.asarray(p.memo["grip"]) + (0.0, 0.0, LIFT)), label="lift it off the lid",
           fallback=b0 + (0, 0, LIFT))

    def held_off(st):  # the command minus the base's live pose
        return np.asarray(p.memo["_last"]) - np.asarray(st["base"].pos, float)

    def hover(st):
        p.memo["off"] = held_off(st)
        return np.array([*MARK_XY, S_TOP + AGAR_PROUD + HOVER]) + p.memo["off"]

    p.move(2.0, D.remember(p, "hover", hover), label="over the circle", fallback=(*MARK_XY, S_TOP + 0.05))
    p.hold(0.4)
    D.corrected(p, 0.8, lambda st: np.array([*MARK_XY, S_TOP + AGAR_PROUD + HOVER]),
                lambda st: np.asarray(st["base"].pos, float), key="hover2", base="hover", max_fix=0.010,
                label="centre over the circle", fallback=(*MARK_XY, S_TOP + 0.05))
    q_t = 0.0016 if how == "hard" else F_T / K_AGAR  # the agar compression that gives 4.5 N
    hold_s = 1.0 if how == "tap" else 11.0  # continuous seconds at 3.3-6.7 N (the tap: from the first touch)
    ctl = {"z": None, "t": None, "xy": None, "q": 0.0, "v": 0.0, "run": 0.0, "done": False}

    def press(st):
        q = float(st["base"].q["agar"])
        ctl["v"] = 0.7 * ctl["v"] + 0.3 * (q - ctl["q"]) / DT
        ctl["q"] = q
        base = np.asarray(st["base"].pos, float)
        if ctl["z"] is None:
            ctl["z"], ctl["xy"] = np.asarray(p.memo["hover2"], float), np.asarray(p.memo["hover2"][:2], float)
            ctl["off"] = ctl["z"] - base  # the command minus the base, the arm settled at the hover
        z = ctl["z"]
        if ctl["t"] is None:
            z = z - np.array([0.0, 0.0, V_DOWN * DT])
            if q > Q_TOUCH:  # first touch (a slow descent; the servo below brakes the lagging arm)
                ctl["t"] = 0.0
        else:
            ctl["t"] += DT
            inside = how == "tap" or (Q_IN <= q <= Q_OUT) or ctl["done"]
            ctl["run"] = ctl["run"] + DT if inside else 0.0  # the press's own clock: continuous time in the band
            ctl["done"] = ctl["done"] or ctl["run"] > hold_s or (how == "hard" and ctl["t"] > hold_s + 2.0)
            if ctl["done"]:  # done: ease off until the agar is free (at most 60 mm up: the arm lags), then wait
                ctl.setdefault("z_done", float(z[2]))
                if q < Q_OFF and "z_free" not in ctl:  # free: rise 15 mm more so the lagging arm does not settle back
                    ctl["z_free"] = float(z[2])
                if z[2] < min(ctl.get("z_free", 9.0) + 0.015, ctl["z_done"] + 0.060):
                    z = z + np.array([0.0, 0.0, 0.001])
            else:  # a damped integral servo: down with the force error, up with the compression's rise (the arm lags)
                q_ahead = q + LAG_S * ctl["v"]  # where the compression is heading, given the arm's lag
                dz = (-KI * (q_t - q_ahead) + KD * ctl["v"]) * DT
                if how != "hard" and max(q, q_ahead) > Q_BACK:  # heading over 6 N: back off at full speed
                    dz = 0.0003
                z = z + np.array([0.0, 0.0, float(np.clip(dz, -0.0003, 0.0003))])
            if how == "slide" and 3.0 < ctl["run"] < 5.0:
                z = z + np.array([0.0, 0.012 * DT, 0.0])  # 24 mm of command sideways over 2 s, 3 s into the in-band press
        ctl["z"] = z
        p.memo["_last"] = p.memo["press"] = z
        return z

    D.follow(p, press, 40.0 + hold_s, fallback=(*MARK_XY, S_TOP), label="press the agar on the surface")
    p.move(1.0, D.remember(p, "off_up", lambda st: np.asarray(p.memo["press"]) + (0.0, 0.0, 0.040)),
           label="lift straight off", fallback=(*MARK_XY, S_TOP + 0.04))

    def over_lid(st):
        lid = st["lid"]
        if "loff" not in p.memo:
            p.memo["loff"] = held_off(st)
        return np.array([lid.pos[0], lid.pos[1], lid.pos[2] + LEDGE_H + 0.012]) + p.memo["loff"]

    p.move(2.0, D.remember(p, "olid", over_lid), label="over the lid", fallback=b0 + (0, 0, 0.012))
    p.hold(0.4)
    D.corrected(p, 0.8, lambda st: np.array([st["lid"].pos[0], st["lid"].pos[1], st["lid"].pos[2] + LEDGE_H + 0.012]),
                lambda st: np.asarray(st["base"].pos, float), key="olid2", base="olid", max_fix=0.010,
                label="centre over the lid", fallback=b0 + (0, 0, 0.012))
    def onto_ledge(st):  # the base slides up in the grip under the press: stop where the pad tips clear the lid rim and
        want = np.asarray(p.memo["olid2"]) - (0.0, 0.0, 0.012)  # let it drop the last few mm onto its ledge
        floor = float(st["lid"].pos[2]) + LID_H + TIP_BELOW_TCP + RIM_CLEAR
        return np.array([want[0], want[1], max(float(want[2]), floor)])

    p.move(1.2, D.remember(p, "ledge", onto_ledge), label="down onto the ledge (or just above it)", fallback=b0)
    p.hold(0.3)
    D.corrected(p, 0.6, lambda st: np.array([st["lid"].pos[0], st["lid"].pos[1], st["base"].pos[2]]),
                lambda st: np.asarray(st["base"].pos, float), key="ledge2", base="ledge", max_fix=0.006,
                label="centre on the lid before letting go", fallback=b0)
    p.hold(0.3)
    if how == "held":
        return p.hold(5.0, "keep holding the plate")
    p.fingers(PLATE_D + 0.012, 1.0, "let go of the plate").hold(0.8)
    p.move(2.0, lambda st: np.asarray(p.memo["olid2"]) + (0.0, 0.0, 0.040), label="rise straight clear",
           fallback=b0 + (0, 0, 0.05))
    return p.home()


def demo():
    def one(how):
        return D.together(hand(how), D.idle_plan("left", 1.0))

    return D.choose(demo=lambda: one("demo"), tap=lambda: one("tap"), hard=lambda: one("hard"),
                    slide=lambda: one("slide"), held=lambda: one("held"))


# -- what the template tests read ---------------------------------------------------------------------------------------
TEXT_NUMBERS = [("65 mm", PLATE_D), ("1.5 mm", AGAR_PROUD), ("5 N/mm", K_AGAR_N_PER_M / 1000), ("3 N", F_LOW), ("7 N", F_HIGH),
                ("9 s", PRESS_S), ("3 deg", math.degrees(FLAT)), ("2 mm", SLIDE), ("10 mm", ON_MARK),
                ("85 mm", MARK_D), ("3 mm", SEAT), ("30 mm", CLEAR_MM / 1000), ("1 s", STILL_S)]
CHEATS = {"none": 0.0, "tap": 0.25, "hard": 0.25, "slide": 0.5, "held": 0.75}
HOLD = {"base.agar": 0.5}
