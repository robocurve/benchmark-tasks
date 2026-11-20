"""Dose 10 mL from a bottle-top dispenser into a hand-held 50 mL tube, piston stop to stop.

On the station table, in a catch tray: a 500 mL media bottle of water on a rubber mat, with a fixed-volume 10 mL
bottle-top dispenser (BRAND Dispensette S type) screwed on its neck. Its discharge tube points to the robot's right and
turns down at its end. To the right, a rack for 50 mL tubes holds one empty uncapped tube.

- The dispenser's upper housing (the sleeve) is the piston. It slides 40 mm between its lower stop (parked) and the
  fixed 10 mL upper stop, with 4 N of dry friction each way, and stays wherever it is left.
- The maker's method: hold the orifice inside the receiving vessel, lift the piston gently to the upper stop, press it
  slowly and steadily to the lower stop, and leave it parked down.
- A tube cannot reach the orifice while it stands in the rack, so one hand must hold the tube up around the tip while
  the other strokes the piston.

The liquid is a declared analytic model (``Dose``): it only books volumes from the piston's travel and the orifice's
position, every control step; nothing is moved, latched or held by code.
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
INSTRUCTION = ("Dispense one 10 mL dose of the solution from the bottle-top dispenser into the empty 50 mL tube, "
               "then stand the tube back in the rack and let go.")
HZ, DT = 20.0, 0.05
SWEEP = json.loads(os.environ.get("TASK_SWEEP", "") or "{}")
R = math.radians
SPAWN_GAP = 0.0005

# -- the tray and the mat (fixed) ----------------------------------------------------------------------------------
TRAY_C, TRAY_XY, TRAY_T, LIP_H = (0.25, -0.05), (0.40, 0.60), 0.002, 0.015
MAT_XY, MAT_T = (0.20, 0.20), 0.003
FLOOR = TOP + TRAY_T  # the tray's floor
BOTTLE_XY = (0.20, 0.0)  # the marked cross
MAT_TOP = FLOOR + MAT_T

# -- the bottle and the dispenser (one free body; the sleeve is its piston) ---------------------------------------------
BOTTLE_D, BOTTLE_H = 0.086, 0.176  # DURAN 500 mL GL45
SHOULDER_D, NECK_D = 0.064, 0.050
BODY_H = 0.150
BLOCK, BLOCK_H = 0.050, 0.064  # the valve block on the neck
RUN_Z = BOTTLE_H + 0.052  # the discharge tube's axis, 228 mm above the mat
TUBE_OD = 0.007  # the PTFE discharge tube
RUN_END = 0.085  # the bend: this far from the bottle axis, toward -y
TIP_LEN = 0.020  # the tip turned down at the end
ORIFICE = (0.0, -RUN_END, RUN_Z - TIP_LEN)  # in the bottle frame
SLEEVE_D, SLEEVE_H = 0.040, 0.060
SLEEVE_Z0 = BOTTLE_H + BLOCK_H  # 240 mm: the sleeve's bottom with the piston parked
STROKE = 0.040  # piston travel, lower stop to the fixed 10 mL upper stop
BORE_AREA = 2.50e-4  # m2: 2.50 cm2 x 40 mm = 10.0 mL
PISTON_FRICTION_N = 4.0  # N each way (a spring-scale pull on the sleeve)
PISTON_FRICTION = PISTON_FRICTION_N * SWEEP.get("friction", 1.0)
SLEEVE_KG = 0.10
BOTTLE_KG = 1.20 * SWEEP.get("mass", 1.0)  # glass 0.35 + water 0.40 + dispenser 0.45
GLASS, WATER, PP, PTFE, RED, GREY = (0.85, 0.90, 0.92), (0.70, 0.82, 0.95), (0.93, 0.93, 0.90), (0.97, 0.97, 0.97), \
    (0.80, 0.18, 0.15), (0.55, 0.56, 0.58)
MAT_FRICTION = 0.8  # glass on the rubber mat (estimate)


def ring(cx, cy, z0, z1, across, color, n=16, t=0.002, tag="") -> list:
    """A round wall about a vertical axis as ``n`` flat plates (across flats ``across``)."""
    r = across / 2
    w = 2 * r * math.tan(math.pi / n) + 0.0006
    return [k.box((cx + (r - t / 2) * math.cos(a), cy + (r - t / 2) * math.sin(a), (z0 + z1) / 2), (t, w, z1 - z0),
                  color, turn=(0.0, 0.0, a), label=f"{tag}{i}")
            for i, a in enumerate(2 * math.pi * j / n for j in range(n))]


def _bottle_boxes() -> list:
    b = k.box
    out = ring(0, 0, 0.0, BODY_H, BOTTLE_D, GLASS, tag="wall")
    out += [b((0, 0, 0.002), (BOTTLE_D * 0.92, BOTTLE_D * 0.92, 0.004), GLASS, label="base"),
            b((0, 0, 0.065), (BOTTLE_D * 0.9, BOTTLE_D * 0.9, 0.122), WATER, ghost=True, label="water")]
    out += ring(0, 0, BODY_H, BODY_H + 0.014, SHOULDER_D, GLASS, n=12, tag="shoulder")
    out += ring(0, 0, BODY_H + 0.014, BOTTLE_H, NECK_D, GLASS, n=12, tag="neck")
    z = BOTTLE_H
    out += [b((0, 0, z + BLOCK_H / 2), (BLOCK, BLOCK, BLOCK_H), PP, label="block"),
            b((0, 0, z + 0.008), (0.058, 0.058, 0.016), RED, label="collar"),
            b((0, 0, SLEEVE_Z0 + 0.030), (0.022, 0.022, 0.060), GLASS, ghost=True, label="cylinder"),
            b((0, -(BLOCK / 2 + RUN_END + TUBE_OD / 2) / 2, RUN_Z), (TUBE_OD, RUN_END - BLOCK / 2 + TUBE_OD / 2, TUBE_OD),
              PTFE, label="run"),
            b((0, -RUN_END, RUN_Z - TIP_LEN / 2), (TUBE_OD, TUBE_OD, TIP_LEN), PTFE, label="tip")]
    return out


def _sleeve_boxes() -> list:
    out = ring(0, 0, 0.0, SLEEVE_H, SLEEVE_D, PP, n=8, t=0.003, tag="sleeve")
    out.append(k.box((0, 0, SLEEVE_H - 0.0015), (SLEEVE_D * 0.9, SLEEVE_D * 0.9, 0.003), PP, label="cap"))
    out.append(k.box((0, 0, SLEEVE_H + 0.001), (0.024, 0.024, 0.002), RED, label="label"))
    return out


SLEEVE_BOXES = _sleeve_boxes()
PISTON = k.slider("piston", (0.0, 0.0, SLEEVE_Z0), "Z", 0.0, STROKE, SLEEVE_BOXES, friction=PISTON_FRICTION,
                  damping=2.0, mass=SLEEVE_KG, weightless=True)  # its weight is carried by the piston seals (declared)
BOTTLE_BOXES = _bottle_boxes()
BOTTLE = k.mech("bottle", (BOTTLE_XY[0], BOTTLE_XY[1], MAT_TOP + SPAWN_GAP), BOTTLE_BOXES, BOTTLE_KG, joints=[PISTON],
                color=GLASS, surface=MAT_FRICTION)

# -- the rack and the tube (free) -----------------------------------------------------------------------------------
TUBE_OD50, TUBE_ID, TUBE_H, CONE_H = 0.030, 0.027, 0.115, 0.020
TUBE_KG = 0.013
GRIP_BELOW = 0.035  # the hand holds the tube this far below its rim
RACK_C = (0.20, -0.15)  # the rack's centre; long side along x
RACK_L, RACK_W, RACK_H, RACK_T = 0.170, 0.045, 0.060, 0.003
HOLE, PITCH = 0.036, 0.040
HOLES_X = [-1.5 * PITCH, -0.5 * PITCH, 0.5 * PITCH, 1.5 * PITCH]  # in the rack frame
TUBE_HOLE = 2  # the tube starts in this hole (x +20 mm: the one nearest the bottle's spout)
RACK_KG = 0.12
TUBE_FRICTION = 0.5


def _rack_boxes() -> list:
    b, s = k.box, k.span
    out = [b((0, 0, RACK_T / 2), (RACK_L, RACK_W, RACK_T), PP, label="floor")]
    for i, x in enumerate((-RACK_L / 2 + RACK_T / 2, RACK_L / 2 - RACK_T / 2)):
        out.append(b((x, 0, RACK_H / 2), (RACK_T, RACK_W, RACK_H), PP, label=f"end{i}"))
    zt = (RACK_H - RACK_T, RACK_H)
    side = (RACK_W - HOLE) / 2
    for i, y in enumerate((-(HOLE / 2 + side / 2), HOLE / 2 + side / 2)):
        out.append(b((0, y, RACK_H - RACK_T / 2), (RACK_L, side, RACK_T), PP, label=f"rail{i}"))
    edges = [-RACK_L / 2] + [e for x in HOLES_X for e in (x - HOLE / 2, x + HOLE / 2)] + [RACK_L / 2]
    for i in range(0, len(edges), 2):
        out.append(s((edges[i], -HOLE / 2, zt[0]), (edges[i + 1], HOLE / 2, zt[1]), PP, label=f"bar{i}"))
    return out


def _tube_boxes() -> list:
    """The tube's frame origin is the middle of its rim; it hangs down from there."""
    out = ring(0, 0, -(TUBE_H - CONE_H), 0.0, TUBE_OD50, PP, n=8, t=0.002, tag="tw")
    out += ring(0, 0, -(TUBE_H - CONE_H / 2), -(TUBE_H - CONE_H), 0.022, PP, n=8, t=0.002, tag="c1")
    out += ring(0, 0, -TUBE_H + 0.002, -(TUBE_H - CONE_H / 2), 0.012, PP, n=8, t=0.002, tag="c2")
    out.append(k.box((0, 0, -TUBE_H + 0.001), (0.008, 0.008, 0.002), PP, label="tip"))
    out.append(k.box((0, 0, -(TUBE_H - CONE_H) - 0.001), (0.020, 0.020, 0.002), PP, label="floor"))
    return out


RACK_BOXES, TUBE_BOXES = _rack_boxes(), _tube_boxes()
RACK = k.mech("rack", (RACK_C[0], RACK_C[1], FLOOR + SPAWN_GAP), RACK_BOXES, RACK_KG, color=PP, surface=0.5)
TUBE_START = (RACK_C[0] + HOLES_X[TUBE_HOLE], RACK_C[1], FLOOR + RACK_T + TUBE_H + 2 * SPAWN_GAP)
TUBE = k.mech("tube", TUBE_START, TUBE_BOXES, TUBE_KG, color=PP, surface=TUBE_FRICTION)


def _tray() -> list:
    (cx, cy), (lx, ly) = TRAY_C, TRAY_XY
    t = 0.003
    out = [Box("tray_floor", pos=(cx, cy, TOP + TRAY_T / 2), size=(lx, ly, TRAY_T), color=(0.80, 0.82, 0.84)),
           Box("mat", pos=(BOTTLE_XY[0], BOTTLE_XY[1], FLOOR + MAT_T / 2), size=(*MAT_XY, MAT_T), color=(0.20, 0.20, 0.22))]
    for n, (x, y, sx, sy) in {"tray_f": (cx + lx / 2, cy, t, ly), "tray_b": (cx - lx / 2, cy, t, ly),
                              "tray_l": (cx, cy + ly / 2, lx, t), "tray_r": (cx, cy - ly / 2, lx, t)}.items():
        out.append(Box(n, pos=(x, y, FLOOR + LIP_H / 2), size=(sx, sy, LIP_H), color=(0.80, 0.82, 0.84)))
    return out


OBJECTS = [BOTTLE, RACK, TUBE, *_tray()]

# -- the liquid ledger (declared, analytic) ---------------------------------------------------------------------------
DOSE_ML, BAND_ML = 10.0, 0.10
SOME_ML, NEAR_ML, SPILL_OK_ML, SPILL_MAX_ML = 2.0, 9.0, 0.3, 3.0
OUTSIDE_ML = 0.05  # pressed out with the orifice outside the tube beyond this: capped at 0.5 (0.2 mm of travel; the
#                    solver's micron jitter at a stop never reaches it)
MIN_STROKE = 0.005  # a lift and then a press of at least 5 mm each count as a stroke
TOP_TOL = 0.001  # the upper stop counts as reached within 1 mm
RIM_DEPTH = 0.003  # the orifice inside the tube: at least 3 mm below its rim, within its mouth
POUR_DEG = 90.0  # a tube tipped past this pours its water out


def orifice_world(st) -> np.ndarray:
    return st.to_world(ORIFICE)


def orifice_inside(bottle, tube) -> bool:
    o = tube.rotation().T @ (orifice_world(bottle) - np.asarray(tube.pos, float))
    return math.hypot(o[0], o[1]) < TUBE_ID / 2 and -(TUBE_H - CONE_H) < o[2] <= -RIM_DEPTH


class Dose:
    """The cylinder holds BORE_AREA x the piston's travel: drawn from the bottle on the upstroke, delivered at the
    orifice on the downstroke. Delivered liquid goes into the tube only while the orifice is inside it, else it is
    spilled. Strokes: a lift of at least MIN_STROKE from the running low, then a press of at least MIN_STROKE from
    the running high. A stroke is clean if its lift reached the upper stop, its whole press delivered into the tube and
    it ended at the lower stop."""

    def __init__(self, q0: float = 0.0):
        q0 = min(max(q0, 0.0), STROKE)
        self.q, self.tube, self.spilled = q0, 0.0, 0.0
        self.low, self.high, self.phase = q0, q0, "down"
        self.strokes, self.clean, self.out_ml = 0, 0, 0.0
        self.reached_top = False
        self.cur = None  # the stroke being pressed: {"top": bool, "out": bool}

    def step(self, q: float, inside: bool, poured: bool) -> None:
        q = min(max(q, 0.0), STROKE)  # the cylinder ends at its stops (the solver's stops give a fraction of a mm)
        dq = self.q - q  # > 0: pressed down
        if dq > 0:
            ml = dq * BORE_AREA * 1e6
            if inside:
                self.tube += ml
            else:
                self.spilled += ml
                self.out_ml += ml
                if self.cur is not None:
                    self.cur["out"] = True
        if poured and self.tube > 0:
            self.spilled += self.tube
            self.tube = 0.0
        if self.phase == "down":
            self.low = min(self.low, q)
            if q - self.low >= MIN_STROKE:
                self.phase, self.high = "up", q
        else:
            self.high = max(self.high, q)
            if self.high - q >= MIN_STROKE:
                self.phase, self.low = "down", q
                self.strokes += 1
                self.cur = {"top": self.high >= STROKE - TOP_TOL, "out": dq > 0 and not inside}
        if self.cur is not None and self.phase == "down" and q <= TOP_TOL:
            if self.cur["top"] and not self.cur["out"]:
                self.clean += 1
            self.cur = None
        self.q = q


# -- the scorer -------------------------------------------------------------------------------------------------------
TUBE_TILT, BOTTLE_TILT, BOTTLE_TIP = R(10.0), R(5.0), R(15.0)
BOTTLE_MOVE, MAT_LEAVE = 0.020, 0.100
PARK_TOL = 0.002
CLEAR_MM = 30.0
STILL_S, STILL_V = 1.0, 0.005  # still: nothing moves more than 5 mm/s x 1 s over the last 1 s
TIP_PT = (0.0, 0.0, -TUBE_H)  # the tube's lowest point, in its frame

TIPPED, LOST, SPILL, END = k.Sticky(), k.Sticky(), k.Sticky(), k.Settle(STILL_S, STILL_V * STILL_S)
RUN = {"dose": Dose()}
REC = k.run_record.RunRecord(__file__, "bottle", "tube", "rack")


def tilt(st) -> float:
    return math.acos(float(np.clip(st.rotation()[2, 2], -1.0, 1.0)))


def in_hole(tube, rack) -> bool:
    if tilt(tube) > TUBE_TILT:
        return False
    p = rack.rotation().T @ (tube.to_world(TIP_PT) - np.asarray(rack.pos, float))
    return p[2] < RACK_H and abs(p[1]) < HOLE / 2 and any(abs(p[0] - x) < HOLE / 2 for x in HOLES_X)


def bottle_offset(bottle) -> float:
    return math.hypot(bottle.pos[0] - BOTTLE_XY[0], bottle.pos[1] - BOTTLE_XY[1])


def grade(states) -> float:
    if TIPPED or LOST or SPILL:
        return 0.0
    b, t, r, d = states["bottle"], states["tube"], states["rack"], RUN["dose"]
    stood = in_hole(t, r)
    cap = d.strokes >= 2 or d.out_ml > OUTSIDE_ML
    three = (d.strokes == 1 and d.clean == 1 and DOSE_ML - BAND_ML - 1e-9 <= d.tube <= DOSE_ML + BAND_ML + 1e-9
             and d.spilled <= SPILL_OK_ML + 1e-9 and stood and not cap)
    full = (three and float(b.q["piston"]) <= PARK_TOL and tilt(b) <= BOTTLE_TILT and bottle_offset(b) <= BOTTLE_MOVE
            and END.ok)
    return k.ladder((0.25, d.tube >= SOME_ML - 1e-9), (0.5, d.tube >= NEAR_ML - 1e-9 and stood), (0.75, three),
                    (1.0, full))


def score(states) -> float:
    return grade(states)


def success(states) -> bool:
    return grade(states) >= 1.0


def distance(states) -> float:
    return abs(RUN["dose"].tube - DOSE_ML)


@k.on_reset
def _reset(states) -> None:
    TIPPED.reset(), LOST.reset(), SPILL.reset(), END.reset()
    RUN["dose"] = Dose(float(states["bottle"].q["piston"]))
    REC.reset()


def _surfaces(states) -> np.ndarray:
    b, t, r = states["bottle"], states["tube"], states["rack"]
    solid = [x for x in BOTTLE_BOXES if not x[3].ghost]
    return np.concatenate([k.score.surface_points(b, "", solid, 0.03), k.score.surface_points(b, "piston", SLEEVE_BOXES, 0.02),
                           k.score.surface_points(t, "", TUBE_BOXES, 0.02), k.score.surface_points(r, "", RACK_BOXES, 0.03)])


@k.on_step
def _watch(states) -> None:
    b, t, r, d = states["bottle"], states["tube"], states["rack"], RUN["dose"]
    q = float(b.q["piston"])
    d.step(q, orifice_inside(b, t), tilt(t) > R(POUR_DEG))
    if tilt(b) > BOTTLE_TIP or bottle_offset(b) > MAT_LEAVE:
        TIPPED.trip("the bottle tipped or left the mat")
    (cx, cy), (lx, ly) = TRAY_C, TRAY_XY
    if abs(t.pos[0] - cx) > lx / 2 or abs(t.pos[1] - cy) > ly / 2 or t.to_world(TIP_PT)[2] < FLOOR - 0.010:
        LOST.trip("the tube fell outside the tray")
    if d.spilled > SPILL_MAX_ML:
        SPILL.trip("more than 3 mL spilled")
    vals = np.concatenate([np.asarray(b.pos, float), np.asarray(t.pos, float), np.asarray(r.pos, float), [q]])
    END.push(vals, k.hands_clear(_surfaces(states), CLEAR_MM))
    if os.environ.get("TASK_DEBUG"):
        RUN["n"] = RUN.get("n", 0) + 1
        if RUN["n"] % 10 == 0:
            o = t.rotation().T @ (orifice_world(b) - np.asarray(t.pos, float))
            print(f"DBG t {RUN['n'] * DT:.1f} q {q * 1000:.1f} tube {np.round(np.asarray(t.pos) * 1000, 1)} tilt "
                  f"{math.degrees(tilt(t)):.1f} o_local {np.round(o * 1000, 1)} ml {d.tube:.2f} sp {d.spilled:.2f} "
                  f"st {d.strokes} cl {d.clean} hole {in_hole(t, r)} bot {bottle_offset(b) * 1000:.1f} g {grade(states)}",
                  flush=True)
    REC.tick(states, lambda: f"q {q * 1000:.1f} mm tube {d.tube:.2f} mL spilled {d.spilled:.2f} strokes {d.strokes} "
                             f"clean {d.clean} inside {orifice_inside(b, t)} score {grade(states):g}")


# -- the demo ---------------------------------------------------------------------------------------------------------
G_R = D.Grip.deg(yaw=90, pitch=45)  # right hand: tilted 45 deg toward the other arm, fingers closing along x
G_L = D.Grip.deg(yaw=-90, pitch=45)  # left hand: the mirror, from the bottle's left
A_R, A_L = np.asarray(G_R.approach), np.asarray(G_L.approach)
BACK = 0.050  # approach this far back along the grip
SQUEEZE_T, SQUEEZE_S = 0.002, 0.003  # m commanded into each side: tube, sleeve
CLEAR = D.P.OPEN_CLEARANCE
LIFT_BOTTOM = 0.075  # carry the tube with its tip this high above the tray floor (rack top at 60 mm)
UNDER_RIM = 0.018  # move in under the tip with the rim this far below the orifice
INSIDE = 0.007  # then rise until the orifice is this far inside the rim
OVER = 0.010  # sleeve moves are sent this far past each stop (the arm's lag under 4 N)
STOP_PUSH = 0.008  # then corrected from the sleeve's live height to this far past the stop
STROKE_V = 0.020  # m/s, slowly and steadily


def tube_tcp(st) -> np.ndarray:
    return st["tube"].to_world((0.0, 0.0, -GRIP_BELOW))


def rim_target(st, rise: float) -> np.ndarray:
    o = orifice_world(st["bottle"])
    return np.array([o[0], o[1], o[2] + rise])


def right_hand(how: str, wait: float) -> D.Plan:
    """Take the tube out of the rack, bring its mouth up round the orifice, hold it there for ``wait`` s, then stand
    it back in its hole and let go. ``how``: "demo", "in_rack" (never touch it) or "held" (stands it in its hole but never lets go)."""
    p = D.Plan("right")
    t0 = np.asarray(TUBE_START, float) + (0.0, 0.0, -GRIP_BELOW)
    open_gap = D.gap_finger(TUBE_OD50 + 2 * CLEAR)
    if how == "in_rack":
        return p.hold(wait + 14.0, "leave the tube in the rack")
    p.move(2.0, lambda st: tube_tcp(st) - BACK * A_R, finger=open_gap, grip=G_R, label="behind the tube",
           fallback=t0 - BACK * A_R)
    p.move(1.0, D.remember(p, "grip", tube_tcp), label="fingers round the tube", fallback=t0)
    p.fingers(TUBE_OD50 - 2 * SQUEEZE_T, D.P.T_CLOSE, "pinch the tube").hold(0.3, "settle the grip")
    lift = LIFT_BOTTOM - (TUBE_START[2] - TUBE_H - FLOOR)
    p.move(1.5, D.remember(p, "lift", lambda st: np.asarray(p.memo["grip"]) + (0.0, 0.0, lift)),
           label="lift it out of the rack", fallback=t0 + (0.0, 0.0, lift))

    def held_offset(st):  # tcp command minus the live rim: how the tube sits in the hand
        return np.asarray(p.memo["_last"]) - np.asarray(st["tube"].pos, float)

    def under(st):
        p.memo["off"] = held_offset(st)
        return rim_target(st, -UNDER_RIM) + p.memo["off"]

    p.move(2.0, D.remember(p, "under", under), label="under the tip", fallback=t0 + (0.0, 0.07, lift))
    p.hold(0.4, "line up under the tip")
    D.corrected(p, 0.8, lambda st: rim_target(st, -UNDER_RIM), lambda st: np.asarray(st["tube"].pos, float),
                key="under2", base="under", max_fix=0.010, label="centre under the tip", fallback=t0 + (0, 0.07, lift))
    D.follow(p, lambda st: np.asarray(p.memo["under2"]) + (0.0, 0.0, UNDER_RIM + INSIDE), 1.5,
             fallback=t0 + (0, 0.07, lift + 0.025), label="raise the mouth round the orifice")
    p.hold(max(0.5, wait - p.duration), "hold the tube on the orifice")
    p.move(1.0, D.remember(p, "low", lambda st: np.asarray(p.memo["under2"]) - (0.0, 0.0, 0.010)),
           label="lower off the tip", fallback=t0 + (0, 0.07, lift - 0.01))

    def over_hole(st, z):
        r = st["rack"]
        h = r.to_world((HOLES_X[TUBE_HOLE], 0.0, 0.0))
        if "hole_off" not in p.memo:
            p.memo["hole_off"] = held_offset(st)
        return np.array([h[0], h[1], z + TUBE_H]) + p.memo["hole_off"]

    p.move(2.0, D.remember(p, "carry", lambda st: over_hole(st, FLOOR + LIFT_BOTTOM)), label="back over its hole",
           fallback=t0 + (0, 0, lift))
    p.hold(0.4, "settle over the hole")

    def hole_xy(st):
        h = st["rack"].to_world((HOLES_X[TUBE_HOLE], 0.0, 0.0))
        return np.array([h[0], h[1], st["tube"].to_world(TIP_PT)[2]])

    D.corrected(p, 0.8, hole_xy, lambda st: st["tube"].to_world(TIP_PT), key="over", base="carry", max_fix=0.015,
                label="centre over the hole", fallback=t0 + (0, 0, lift))
    D.corrected(p, 0.8, hole_xy, lambda st: st["tube"].to_world(TIP_PT), key="over2", base="over", max_fix=0.015,
                label="centre over the hole again", fallback=t0 + (0, 0, lift))
    p.hold(0.3, "settle over the hole")
    D.corrected(p, 0.8, hole_xy, lambda st: st["tube"].to_world(TIP_PT), key="over3", base="over2", max_fix=0.010,
                label="centre over the hole a third time", fallback=t0 + (0, 0, lift))
    dip = LIFT_BOTTOM - RACK_H + 0.010  # the tip 10 mm into the hole's lead-in
    p.move(1.0, D.remember(p, "dip", lambda st: np.asarray(p.memo["over3"]) - (0.0, 0.0, dip)),
           label="tip into the hole", fallback=t0)
    D.corrected(p, 0.6, lambda st: hole_xy(st), lambda st: st["tube"].to_world(TIP_PT), key="in", base="dip",
                max_fix=0.008, label="centre in the hole", fallback=t0)
    p.move(1.5, lambda st: np.asarray(p.memo["in"]) - (0.0, 0.0, LIFT_BOTTOM - dip - RACK_T - 0.004),
           label="down into the hole", fallback=t0)
    p.hold(0.3)
    if how == "held":
        return p.hold(6.0, "keep holding the tube")
    p.fingers(TUBE_OD50 + 2 * CLEAR, D.P.T_RELEASE, "let go of the tube")
    p.back(BACK, 1.0)
    return p.home()


def sleeve_tcp(st) -> np.ndarray:
    return st["bottle"].to_world((0.0, 0.0, SLEEVE_H / 2), "piston")


def sleeve_tcp_at(st, q: float) -> np.ndarray:
    """Where the sleeve's middle would be with the piston at ``q`` (past a stop: where the hand is sent to press it)."""
    b = st["bottle"]
    return b.to_world((0.0, 0.0, SLEEVE_Z0 + SLEEVE_H / 2 + q))


def left_hand(how: str, wait: float) -> D.Plan:
    """Wait ``wait`` s, grip the sleeve, lift it to the upper stop, press it to the lower stop, let go. ``how``:
    "demo" or "half_lift" (lift it only 20 mm)."""
    p = D.Plan("left").hold(wait, "wait for the tube")
    s0 = np.array([BOTTLE_XY[0], BOTTLE_XY[1], MAT_TOP + SLEEVE_Z0 + SLEEVE_H / 2])
    p.move(2.0, lambda st: sleeve_tcp(st) - BACK * A_L, finger=D.gap_finger(SLEEVE_D + 2 * CLEAR), grip=G_L,
           label="beside the sleeve", fallback=s0 - BACK * A_L)
    p.move(1.0, D.remember(p, "grip", sleeve_tcp), label="fingers round the sleeve", fallback=s0)
    p.fingers(SLEEVE_D - 2 * SQUEEZE_S, D.P.T_CLOSE, "grip the sleeve").hold(0.3, "settle the grip")
    up = STROKE / 2 if how == "half_lift" else STROKE + OVER
    D.ease(p, D.remember(p, "up", lambda st: np.asarray(p.memo["grip"]) + (0.0, 0.0, up)), STROKE_V,
           label="lift to the upper stop", fallback=s0 + (0.0, 0.0, up))
    p.hold(0.5, "at the upper stop")
    if how != "half_lift":  # the arm lags under the 4 N friction: send it on until the sleeve itself is at the stop
        D.corrected(p, 1.0, lambda st: sleeve_tcp_at(st, STROKE + STOP_PUSH), sleeve_tcp, key="up2", base="up",
                    max_fix=0.030, label="on up to the upper stop", fallback=s0 + (0.0, 0.0, up))
        p.hold(0.5, "at the upper stop")
    D.ease(p, D.remember(p, "down", lambda st: np.asarray(p.memo["grip"]) - (0.0, 0.0, OVER)), STROKE_V,
           label="press to the lower stop", fallback=s0 - (0.0, 0.0, OVER))
    p.hold(0.5, "at the lower stop")
    D.corrected(p, 1.0, lambda st: sleeve_tcp_at(st, -STOP_PUSH), sleeve_tcp, key="down2", base="down", max_fix=0.030,
                label="on down to the lower stop", fallback=s0 - (0.0, 0.0, OVER))
    p.hold(0.5, "parked at the lower stop")
    p.fingers(SLEEVE_D + 2 * CLEAR, D.P.T_RELEASE, "let go of the sleeve")
    p.back(BACK, 1.0)
    return p.home()


T_TUBE_READY = 11.3  # s: the right hand has the tube on the orifice
T_STROKE = 16.5  # s: the left hand's whole stroke, from leaving home to letting go


def demo():
    def two(r_how, l_how):
        return D.together(left_hand(l_how, T_TUBE_READY), right_hand(r_how, T_TUBE_READY + T_STROKE))

    return D.choose(demo=lambda: two("demo", "demo"), in_rack=lambda: two("in_rack", "demo"),
                    half_lift=lambda: two("demo", "half_lift"), held=lambda: two("held", "demo"))


# -- what the template tests read ---------------------------------------------------------------------------------------
TEXT_NUMBERS = [("40 mm", STROKE), ("4 N", PISTON_FRICTION), ("5 mm", MIN_STROKE), ("2 mm", PARK_TOL),
                ("3 mm", RIM_DEPTH), ("20 mm", TIP_LEN), ("10 deg", math.degrees(TUBE_TILT)),
                ("5 deg", math.degrees(BOTTLE_TILT)), ("15 deg", math.degrees(BOTTLE_TIP)), ("20 mm", BOTTLE_MOVE),
                ("30 mm", CLEAR_MM / 1000), ("5 mm/s", STILL_V), ("1 s", STILL_S), ("36 mm", HOLE)]
CHEATS = {"none": 0.0, "in_rack": 0.0, "half_lift": 0.25, "held": 0.75}  # mode: the most it may score
HOLD = {"bottle.piston": 0.5}  # joint: the most it may move with no command (mm or deg)
