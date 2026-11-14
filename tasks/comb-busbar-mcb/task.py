"""Open the row, insert into all at once, close the row: back out each breaker's line-terminal screw to open its clamp, lower a pin comb busbar into all
three open terminals at once, then drive each screw back down onto its pin.

A panel's mounting plate stands upright on the table as in a cabinet, a DIN rail across it, with three single-pole breakers of one range (18 mm
modules, Acti9 iC60N type) on the rail, fronts towards the arms. Each breaker's line terminal is on its top: a clamp whose jaw is moved by a captive screw in the front face. Driven
home, the screw holds the jaw shut; backed out, it opens the clamp. A pin comb busbar of the same range, cut to four pins, rests on two posts to the left:
three pins for the row, the fourth standing free beyond it as the hand's grip (a real panel would cut it off or cap it). A screwdriver rests on a stand to
the right. The terminals must be opened before the comb can go in, and closed after: "Loosen the MCB terminal screws. Fit the busbar and tighten the
terminal screws" (Contactum consumer unit instructions).

Model (declared in code): the breakers are one fixed articulation; each terminal's screw turns on a hinge and its jaw slides on a slider. The thread is code:
the screwdriver's turn about its own axis turns a screw whose head its bit is pressed onto (the tip on the head's face, within 2.5 mm of its centre, the shaft within 10 degrees of square),
either way, as a plain screwdriver does; the jaw follows the screw at 0.8 mm a turn (an M5 thread) and stops on whatever it closes on. The jaw's clamping
force is capped at 60 N; the maker's tightening torque (about 2 N m) is left out.

    python demokit.py task.py OUT --mode full
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import mechkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = ("Fit the comb busbar to the three breakers: open their top terminals, lower the comb's pins into all three, tighten the terminal screws, "
               "then put the screwdriver down and move both hands away.")
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 40  # 2 s at 20 Hz, the pull test inside it
TUG_N, TUG_MOVE, TUG_STEPS = 10.0, 0.001, (10, 20)  # the pull test: 10 N up along the pins, out of the terminals, for 0.5 s; the comb may move 1 mm
TOP = TABLE_TOP[SCENE]

# -- the breakers (station frame, heights above the table): their fronts at x FACE, towards the arms
FACE = 0.30
MOD = 0.0178  # one module's width (18 mm pitch)
PITCH_Y = 0.018
YS = (-0.027, -0.009, 0.009)  # the breakers' centres; the comb's fourth pin stands free at +0.027
MCB_D, MCB_H = 0.073, 0.085  # depth from the front to the DIN rail, and height
ZC = 0.1525  # the breakers' centre height
Z_TOP = ZC + MCB_H / 2  # their top faces
T_L, T_W, T_DEPTH = 0.010, 0.008, 0.014  # each line terminal's aperture in the top: along x, across (y), and its depth
XA0 = FACE + 0.011  # the aperture's front wall
Z_SCREW = Z_TOP - 0.013  # the terminal screws' axis height, in the front face
GAP0, JAW_T, JAW_TRAVEL = 0.0005, 0.002, 0.0075  # the clamp's gap with the screw home, the jaw's thickness, its travel
PITCH = 0.0008  # M5: the jaw moves 0.8 mm a turn
CLAMP = 0.002  # a bearing jaw is driven this far past its contact: the clamp
TURNS_OPEN = 3.75  # turns out to open the clamp for a pin: 3 mm and 0.5 mm clearance
BOX = dict(x0=FACE - 0.012, x1=FACE + MCB_D + 0.008, y=0.24, z0=0.055, z1=0.25, t=0.0012)  # the mounting plate (only its back panel is built)
# -- the comb busbar: an insulated bar along y with four copper pins down, at 18 mm pitch
PIN_YS = (-0.027, -0.009, 0.009, 0.027)
COMB_L, COMB_BX, COMB_BZ = 0.076, 0.010, 0.012  # bar 12 mm tall (approximated)
PIN_W, PIN_T, PIN_L = 0.006, 0.003, 0.013
PIN_X = XA0 + 0.0002 + PIN_T / 2  # a seated pin's centre: against the aperture's front wall
COMB_REST = np.array([0.215, 0.16, 0.10])  # the comb's bar centre on its posts, at the start
# -- the screwdriver: a #2 Pozidriv on a stand to the right, along x, handle towards the arms
SD_HANDLE_R, SD_HANDLE_L, SD_SHAFT_R, SD_SHAFT_L = 0.016, 0.060, 0.003, 0.050
SD_START = np.array([0.225, -0.155, 0.17])  # the handle's back end on the stand (right of the plate)
SD_REST_X = 0.247  # the rest under the handle from just past the pads, under its centre of mass
BIT_X = SD_HANDLE_L + SD_SHAFT_L  # the bit's tip, from the handle's back end

LOOKS = {
    "enclosure": m.Look(color=m.rgb(0.93, 0.93, 0.92), roughness=0.5, metallic=0.2),
    "board": m.Look(color=m.rgb(0.80, 0.70, 0.55), roughness=0.85),
    "rail": m.Look(color=m.rgb(0.75, 0.76, 0.78), roughness=0.35, metallic=0.9),
    "mcb": m.Look(color=m.rgb(0.95, 0.95, 0.94), roughness=0.55),
    "toggle": m.Look(color=m.rgb(0.10, 0.10, 0.11), roughness=0.6),
    "label": m.Look(color=m.rgb(0.75, 0.10, 0.10), roughness=0.6),
    "dark": m.Look(color=m.rgb(0.05, 0.05, 0.06), roughness=0.6),
    "screw": m.Look(color=m.rgb(0.80, 0.80, 0.82), roughness=0.3, metallic=0.8),
    "brass": m.Look(color=m.rgb(0.80, 0.65, 0.30), roughness=0.35, metallic=0.9),
    "copper": m.Look(color=m.rgb(0.85, 0.50, 0.30), roughness=0.3, metallic=0.9),
    "insul": m.Look(color=m.rgb(0.55, 0.57, 0.60), roughness=0.6),
    "handle": m.Look(color=m.rgb(0.95, 0.75, 0.10), roughness=0.5),
    "stand": m.Look(color=m.rgb(0.30, 0.30, 0.32), roughness=0.6),
    "wood": m.Look(color=m.rgb(0.62, 0.45, 0.26), roughness=0.8),
}


def _b(name, size, c, look, cols=None):
    if cols is not None:
        cols.append(m.col_box(name + "_c", size, c))
    return (name, m.box(size, c), look)


def _unit():
    """Fixed: the wall board, the mounting plate, the DIN rail, the comb's posts and the screwdriver's stand."""
    parts, cols = [], []
    bx = BOX
    xb = bx["x1"]
    parts.append(_b("board", (0.018, 0.40, 0.27), (xb + 0.010, 0.0, 0.135), "board", cols))  # low: the left arm's elbow passes over it
    parts.append(_b("back", (bx["t"], bx["y"], bx["z1"] - bx["z0"]), (xb - bx["t"] / 2, 0.0, (bx["z0"] + bx["z1"]) / 2), "enclosure", cols))
    # an open mounting plate, as a panel is built: walls round the row would stop the left forearm reaching over it with the comb
    parts.append(_b("rail", (0.0075, 0.20, 0.035), (FACE + MCB_D + 0.0038, 0.0, ZC), "rail", cols))
    hp = COMB_REST[2] - COMB_BZ / 2 - 0.0005  # two posts under the bar's ends, the pins hanging between them
    for k, s_ in enumerate((-1, 1)):
        parts.append(_b(f"comb_post{k}", (0.006, 0.008, hp), (COMB_REST[0], COMB_REST[1] + s_ * (COMB_L / 2 - 0.002), hp / 2), "wood", cols))  # under the bar's last 6 mm, thinner than the bar so jaws across the bar meet the bar
    za, zt = SD_START[2] - SD_HANDLE_R * math.sqrt(2) + 0.002, SD_START[2] - 0.006
    for sg, nm in ((-1, "a"), (1, "b")):
        outline = [(sg * 0.025, 0.0), (0.0, 0.0), (0.0, za), (sg * (zt - za), zt), (sg * 0.025, zt)]
        if sg > 0:
            outline = outline[::-1]
        pr = m.prism([(SD_START[1] + yy, zz) for yy, zz in outline], SD_REST_X, SD_REST_X + 0.040)
        mesh = m.Mesh(pr.v[:, [2, 0, 1]], pr.f)
        parts.append((f"rest{nm}", mesh, "stand"))
        cols.append(m.col_hull(f"rest{nm}_c", mesh))
    return parts, cols


def _breakers():
    """The breakers' bodies, each with its line terminal's aperture open in the top (frame of the row's articulation)."""
    parts, cols = [], []
    zb = Z_TOP - T_DEPTH  # the aperture's floor
    xa1 = XA0 + T_L
    for k, y in enumerate(YS):
        parts.append(_b(f"mcb{k}_body", (MCB_D, MOD, zb - (ZC - MCB_H / 2)), (FACE + MCB_D / 2, y, (ZC - MCB_H / 2 + zb) / 2), "mcb", cols))
        parts.append(_b(f"mcb{k}_front", (XA0 - FACE, MOD, T_DEPTH), (FACE + (XA0 - FACE) / 2, y, zb + T_DEPTH / 2), "mcb", cols))
        parts.append(_b(f"mcb{k}_rear", (FACE + MCB_D - xa1, MOD, T_DEPTH), (xa1 + (FACE + MCB_D - xa1) / 2, y, zb + T_DEPTH / 2), "mcb", cols))
        for s in (1, -1):
            parts.append(_b(f"mcb{k}_cheek{'ab'[s > 0]}", (T_L, (MOD - T_W) / 2, T_DEPTH), (XA0 + T_L / 2, y + s * (T_W / 2 + (MOD - T_W) / 4), zb + T_DEPTH / 2), "mcb", cols))
        parts.append(_b(f"mcb{k}_floor", (T_L, T_W, 0.001), (XA0 + T_L / 2, y, zb + 0.0005), "brass"))  # the terminal's contact, at the aperture's floor
        parts.append(_b(f"mcb{k}_toggle", (0.010, 0.007, 0.018), (FACE - 0.004, y, ZC + 0.004), "toggle"))
        parts.append(_b(f"mcb{k}_label", (0.0008, 0.012, 0.010), (FACE - 0.0004, y, ZC - 0.024), "label"))
        parts.append((f"mcb{k}_recess", m.cylinder(0.0042, 0.0008, (FACE - 0.0004, y, Z_SCREW), axis="x", segments=20), "dark"))
    return parts, cols


def _terminal_links():
    links = []
    for k, y in enumerate(YS):
        head = m.cylinder(0.0035, 0.0025, (FACE - 0.0010, y, Z_SCREW), axis="x", segments=20)
        cross = [m.box((0.0006, 0.0045, 0.0010), (FACE - 0.0024, y, Z_SCREW)), m.box((0.0006, 0.0010, 0.0045), (FACE - 0.0024, y, Z_SCREW))]
        links.append(m.hinge(f"screw{k}", "X", (FACE, y, Z_SCREW), -1e5, 1e5, [(f"head{k}", head, "screw"), (f"cross{k}a", cross[0], "dark"),
                                                                              (f"cross{k}b", cross[1], "dark")], [],
                             mass=0.003, spring=0.05, damping=0.002, friction=0.0, effort=5.0, inertia=(1e-7, 1e-7, 1e-7)))  # the head follows the coded turn
        jaw = m.box((JAW_T, T_W - 0.001, T_DEPTH - 0.002), (XA0 + GAP0 + JAW_T / 2, y, Z_TOP - T_DEPTH / 2 - 0.001))
        links.append(m.slider(f"jaw{k}", "X", 0.0, JAW_TRAVEL, [(f"jawp{k}", jaw, "brass")], [m.col_hull(f"jawp{k}_c", jaw)],
                              at=(XA0 + GAP0 + JAW_T / 2, y, Z_TOP - T_DEPTH / 2), mass=0.004, spring=20000.0, damping=40.0, effort=60.0, friction=0.0,
                              inertia=(1e-7, 1e-7, 1e-7)))
    return links


_up, _uc = _unit()
UNIT = m.fixed("unit", (0.0, 0.0, TOP), _up, _uc, LOOKS, friction=(0.5, 0.45))
_bp, _bc = _breakers()
ROW = m.mechanism("row", (0.0, 0.0, TOP), _terminal_links(), LOOKS, frame_parts=_bp, frame_colliders=_bc, friction=(0.5, 0.45), solver_iters=48)


def _comb():
    """The comb busbar, in its own frame: the bar's centre at the origin, along y; the pins down (-z)."""
    parts = [("bar", m.box((COMB_BX, COMB_L, COMB_BZ), (0.0, 0.0, 0.0)), "insul"),
             ("strip", m.box((COMB_BX + 0.0006, COMB_L - 0.006, 0.003), (0.0, 0.0, COMB_BZ / 2 - 0.0035)), "copper")]
    cols = [m.col_box("bar_c", (COMB_BX, COMB_L, COMB_BZ), (0.0, 0.0, 0.0))]
    for k, y in enumerate(PIN_YS):
        c = (0.0, y, -COMB_BZ / 2 - PIN_L / 2)
        parts.append((f"pin{k}", m.box((PIN_T, PIN_W, PIN_L), c), "copper"))
        cols.append(m.col_box(f"pin{k}_c", (PIN_T, PIN_W, PIN_L), c))
    return parts, cols


_cp, _cc = _comb()
COMB = m.free("comb", tuple(COMB_REST + np.array([0.0, 0.0, TOP + 0.0005])), _cp, _cc, LOOKS, mass=0.05, friction=(0.6, 0.5))


def _driver():
    """Origin at the handle's back end on its axis; the shaft and bit along +x."""
    L, sl = SD_HANDLE_L, SD_SHAFT_L
    parts = [("handle", m.cylinder(SD_HANDLE_R, L, (L / 2, 0.0, 0.0), axis="x", segments=24), "handle"),
             ("ferrule", m.cylinder(0.006, 0.006, (L + 0.003, 0.0, 0.0), axis="x", segments=16), "screw"),
             ("shaft", m.cylinder(SD_SHAFT_R, sl, (L + sl / 2, 0.0, 0.0), axis="x", segments=12), "screw"),
             ("grip_band", m.cylinder(SD_HANDLE_R + 0.0005, 0.008, (L - 0.012, 0.0, 0.0), axis="x", segments=24), "dark")]
    cols = [m.col_hull("handle_c", m.cylinder(SD_HANDLE_R, L, (L / 2, 0.0, 0.0), axis="x", segments=16)),
            m.col_hull("shaft_c", m.cylinder(SD_SHAFT_R, sl, (L + sl / 2, 0.0, 0.0), axis="x", segments=8))]
    return parts, cols


_dp, _dc = _driver()
DRIVER = m.free("driver", tuple(SD_START + np.array([0.0, 0.0, TOP + 0.0025])), _dp, _dc, LOOKS, mass=0.06, friction=(0.8, 0.7),
                com=(SD_HANDLE_L / 2, 0.0, 0.0))
OBJECTS = [UNIT, ROW, COMB, DRIVER]


# -- the terminals' screws and jaws, as code ---------------------------------------------------------------------------------------------------
def _roll(R) -> float:
    """The screwdriver's turn about its own axis (its x), from its rotation."""
    y, x = R[:, 1], R[:, 0]
    ref = np.array([0.0, 0.0, 1.0]) - x * x[2]
    ref = ref / (np.linalg.norm(ref) + 1e-9)
    return math.atan2(float(np.dot(np.cross(ref, y), x)), float(np.dot(ref, y)))


HEAD_FACE = FACE - 0.00225  # the screw heads' outer faces (x)


def head_point(k):
    return np.array([FACE - 0.0015, YS[k], TOP + Z_SCREW])


def may_clamp(out: float, peak: float) -> bool:
    """A jaw held open by a pin clamps it only as its screw is driven back in: the screw at least a quarter turn in from the furthest it was backed out.
    A pin pushed against an open jaw, with the screw still out, does not clamp."""
    return out < peak - 0.25


class Terminals:
    """Each screw's turns out (0: home), from the screwdriver's turn while its bit is in the head, either way (a plain screwdriver); the jaw follows at
    PITCH a turn and stops on what it closes on; the screw then turns no further in."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.out = [0.0, 0.0, 0.0]
        self.peak = [0.0, 0.0, 0.0]
        self.bears = [False, False, False]
        self.last_roll = None
        self.engaged = None

    def step(self, env):
        drv = m.state(env, "driver")
        R = drv.rotation()
        tip = drv.to_world(np.array([[BIT_X, 0.0, 0.0]]))[0]
        roll = _roll(R)
        self.engaged = None
        for k in range(3):
            off = tip - head_point(k)
            ax = float(off[0]) - (HEAD_FACE - head_point(k)[0])  # the tip's place along the screw's axis, from the head's face (+ is into the head)
            on_face = -0.0006 < ax < 0.002  # the tip on the head's face (within 0.6 mm), not merely near it
            if math.hypot(off[1], off[2]) < 0.0025 and on_face and float(R[0, 0]) > math.cos(math.radians(10.0)):
                self.engaged = k
        row = m.state(env, "row")
        for k in range(3):
            meas = row.joint(f"jaw{k}")
            if self.bears[k]:
                if meas < PITCH * self.out[k] - CLAMP + 0.0003:
                    self.bears[k] = False  # the clamping jaw reached its target: nothing in its way any more
            elif meas > PITCH * self.out[k] + 0.0002 and may_clamp(self.out[k], self.peak[k]):
                self.bears[k] = True  # held open by what it closes on, as its screw is driven back in; a screw driven home stays home until turned back out
        if self.last_roll is not None and self.engaged is not None:
            k = self.engaged
            d = (roll - self.last_roll + math.pi) % (2 * math.pi) - math.pi  # + is clockwise seen from the arms: tightening
            if d < -0.01:
                self.bears[k] = False
            if not (d > 0 and self.bears[k]):
                self.out[k] = float(np.clip(self.out[k] - d / (2 * math.pi), 0.0, JAW_TRAVEL / PITCH))
                self.peak[k] = max(self.peak[k], self.out[k])
        self.last_roll = roll
        for k in range(3):
            asset, js = m._joint(env, "row", f"screw{k}")
            asset.set_joint_position_target(m._tensor(asset, -self.out[k] * 2 * math.pi), joint_ids=[js])
            _, jj = m._joint(env, "row", f"jaw{k}")
            # a screw driven home clamps: its jaw is driven 2 mm past the pin it bears on (20 N/mm, capped at 60 N; the jaw sinks into the pin, about 10 N)
            asset.set_joint_position_target(m._tensor(asset, PITCH * self.out[k] - (CLAMP if self.bears[k] else 0.0)), joint_ids=[jj])


TERM = Terminals()
m.on_step(TERM.step)
m.on_reset(TERM.reset)


def now(env) -> dict:
    c = m.state(env, "comb")
    R = c.rotation()
    pins = c.to_world(np.array([[0.0, y, -COMB_BZ / 2 - PIN_L / 2] for y in PIN_YS]))
    seated = [abs(pins[k, 0] - PIN_X) < 0.001 and abs(pins[k, 1] - YS[k]) < 0.003 for k in range(3)]  # in the clamp's gap, in front of the jaw: not in the space behind a shut jaw
    bar_bottom = c.pos[2] - TOP - COMB_BZ / 2
    home = all(seated) and abs(bar_bottom - Z_TOP) < 0.0015 and abs(math.degrees(math.atan2(R[0, 1], R[1, 1]))) < 3.0
    row = m.state(env, "row")
    gaps = [GAP0 + row.joint(f"jaw{k}") for k in range(3)]
    down = [TERM.bears[k] and gaps[k] < PIN_T + 0.0008 and seated[k] for k in range(3)]
    tips = m.fingertips(env)
    comb_pts = c.to_world(np.array([[0.0, yy, 0.0] for yy in np.linspace(-COMB_L / 2, COMB_L / 2, 7)] + [[0.0, yy, -COMB_BZ / 2 - PIN_L] for yy in PIN_YS]))
    hands = float(np.min(np.linalg.norm(tips[:, None, :] - comb_pts[None], axis=2)))  # every fingertip of both hands, from the whole comb
    drv = m.state(env, "driver")
    tip = drv.to_world(np.array([[BIT_X, 0.0, 0.0]]))[0]
    handle = drv.to_world(np.array([[SD_HANDLE_L / 2, 0.0, 0.0]]))[0]
    drv_pts = drv.to_world(np.array([[xx, 0.0, 0.0] for xx in np.linspace(0.0, BIT_X, 6)]))
    driver_free = float(np.min(np.linalg.norm(tips[:, None, :] - drv_pts[None], axis=2))) > 0.04 and min(np.linalg.norm(tip - head_point(k)) for k in range(3)) > 0.02
    fell = bool(c.pos[2] < TOP - 0.01 or drv.pos[2] < TOP - 0.01)
    return {"home": home, "seated": seated, "bar_bottom": bar_bottom, "gaps": gaps, "down": down, "out": list(TERM.out), "hands": hands,
            "driver_free": driver_free, "fell": fell, "comb_z": float(c.pos[2])}


OPEN_EACH, HOME, DOWN_EACH = 0.03, 0.25, 0.10  # the ladder: each terminal opened for a pin (latched), the comb home, each screw down


def _ladder(c, opened: int) -> float:
    """The partial score: 0.03 for each terminal whose screw has been backed out far enough for a pin; 0.25 more while the comb is home, and 0.1 more for each
    screw down on its pin (0.64 at most)."""
    s = OPEN_EACH * opened
    if c["home"]:
        s += HOME + DOWN_EACH * sum(c["down"])
    return round(s, 3)


def _complete(c) -> bool:
    """The full result, before its hold: the comb home, all three screws down, every fingertip 5 cm from the comb and 4 cm from the screwdriver, and
    the screwdriver's tip 2 cm from the heads."""
    return c["home"] and all(c["down"]) and c["hands"] > 0.05 and c["driver_free"]


class Trial:
    """Each step: the state's tier. 1.0 only once the full result has held HOLD_STEPS steps, with the pull test (10 N up along the pins) passed inside
    the hold; a comb pulled out more than 1 mm fails it for the rest of the trial."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.hold = 0
        self.full = False
        self.loose = False
        self.fell = False
        self.opened = [False, False, False]
        self.ref = None
        self.moved = 0.0
        self.score = 0.0
        self.pulling = False

    def step(self, env):
        c = now(env)
        self.fell = self.fell or c["fell"]
        self.opened = [o or n >= TURNS_OPEN - 0.05 for o, n in zip(self.opened, c["out"])]
        # a screw is down only once its terminal has been opened (its screw backed out far enough for a pin, latched): a pin pressed on a shut jaw does not count
        c = dict(c, down=[dn and o for dn, o in zip(c["down"], self.opened)])
        self.hold = self.hold + 1 if (_complete(c) and not self.fell) else 0
        a, b = TUG_STEPS
        if self.hold == a:
            self.ref = c["comb_z"]
        if self.ref is not None and a <= self.hold <= b + 5:
            self.moved = max(self.moved, abs(c["comb_z"] - self.ref))
            if self.moved > TUG_MOVE:
                self.loose = True
        want = a <= self.hold < b
        if want != self.pulling:
            m.push(env, "comb", (0.0, 0.0, TUG_N if want else 0.0))
            self.pulling = want
        if self.hold == 0:
            self.ref = None
        self.full = self.full or (self.hold >= HOLD_STEPS and not self.loose)
        self.score = 0.0 if self.fell else (1.0 if self.full else _ladder(c, sum(self.opened)))


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


def _report(ctx, env, label=""):
    c = now(env)
    if os.environ.get("COMB_DEBUG"):
        dd = m.state(env, "driver")
        print(f"[comb] t {ctx.t:5.1f} driver at {np.round(dd.pos - [0, 0, TOP], 4)} axis {np.round(dd.rotation()[:, 0], 3)}", flush=True)
    print(f"[comb] t {ctx.t:5.1f} {label}home {c['home']} seated {c['seated']} bar {1000 * (c['bar_bottom'] - Z_TOP):+5.1f} mm; turns out "
          f"{np.round(c['out'], 2)}, gaps {np.round(np.array(c['gaps']) * 1000, 1)} mm, down {c['down']}; hands {1000 * c['hands']:.0f} mm, "
          f"driver free {c['driver_free']}; engaged {TERM.engaged}; pull moved {1000 * TRIAL.moved:.2f} mm loose {TRIAL.loose}; score {score(None):.2f}", flush=True)


def demo_script(ctx):
    """Right hand: takes the screwdriver, squares it, and backs each terminal screw out in plain half-turn strokes (bit in, turn, bit out, turn back) until
    its clamp is open. Left hand: takes the comb by its bar over the free fourth pin, lowers its pins into the three open terminals and lets go (it rests
    on the breakers' tops). Right hand: drives each screw back down until it stops on its pin, lays the screwdriver back on its stand; both back away.
    Failure checks (--mode): none does nothing; closed lowers the comb without opening the terminals; loose leaves the screws open after the comb;
    hover turns the screwdriver with its tip held just in front of each head, never on it."""
    import demokit as d
    from yam_arena.constants import FINGER_OPEN

    env = ctx.emb._env
    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode not in ("full", "none", "closed", "loose", "hover", "leave", "behind"):
        raise ValueError(mode)
    dbg = (lambda w: _report(ctx, env, w)) if os.environ.get("COMB_DEBUG") else (lambda w: None)
    for _ in range(10):
        yield
        if os.environ.get("COMB_DEBUG"):
            _report(ctx, env, "settle: ")
    _report(ctx, env, f"{mode} start: ")
    if mode == "none":
        for _ in range(40):
            yield
        return

    def Rx(deg):
        c_, s_ = math.cos(math.radians(deg)), math.sin(math.radians(deg))
        return np.array([[1.0, 0.0, 0.0], [0.0, c_, -s_], [0.0, s_, c_]])

    def turn_onto(a, b):
        a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
        v, c_ = np.cross(a, b), float(np.dot(a, b))
        kx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        return np.eye(3) + kx + kx @ kx / (1.0 + c_)

    tip_off = lambda: m.state(env, "driver").to_world(np.array([[BIT_X, 0.0, 0.0]]))[0]  # noqa: E731

    # 1. the screwdriver: gripped 1 cm into the handle, jaws top and bottom (the wrist rolled -90 about x), lifted, squared to +x
    drv = m.state(env, "driver")
    handle = drv.to_world(np.array([[0.008, 0.0, 0.0]]))[0]  # the fingers' front edge stops short of the stand's back face
    g_lo = d._as_grip(Rx(-90.0) @ d.level(0).rot)  # roll -90: jaws top and bottom
    def ddbg(w):
        if os.environ.get("COMB_DEBUG"):
            dd = m.state(env, "driver")
            tp = m.fingertips(env)[12:]
            print(f"[comb] dbg {w}: driver at {np.round(dd.pos - [0, 0, TOP], 4)} axis {np.round(dd.rotation()[:, 0], 3)}; right tips {np.round(tp.min(0) - [0, 0, TOP], 3)}.."
                  f"{np.round(tp.max(0) - [0, 0, TOP], 3)}; cmd {np.round(r.at() - [0, 0, TOP], 4)}", flush=True)

    def traced(gen, w):
        for n_, _ in enumerate(gen):
            yield
            if n_ % 5 == 4:
                ddbg(f"{w} +{n_ + 1}")

    # from above and behind, so the open fingers never sweep through the handle on the way down
    yield from traced(r.move(handle + np.array([-0.065, 0.0, 0.08]), grip=g_lo, finger=FINGER_OPEN * 0.6, t=2.0, label="above and behind the screwdriver"), "to above")
    yield from traced(r.move(handle + np.array([-0.065, 0.0, 0.0]), t=1.0, label="down behind the screwdriver's handle"), "to behind")
    ddbg("behind")
    yield from traced(r.move(handle, t=1.2, label="fingers either side of the handle"), "to handle")
    ddbg("either side")
    yield from r.close(2 * SD_HANDLE_R, 50.0, t=0.6, label="grip the handle")
    ddbg("gripped")
    yield from r.move(r.at() + np.array([0.0, 0.0, 0.03]), t=0.8, label="lift it off its stand")
    if os.environ.get("COMB_DEBUG"):
        dd = m.state(env, "driver")
        tp = m.fingertips(env)[12:]
        print(f"[comb] dbg driver at {np.round(dd.pos - [0, 0, TOP], 4)} axis {np.round(dd.rotation()[:, 0], 3)}; handle target {np.round(handle - [0, 0, TOP], 4)}; "
              f"right tips {np.round(tp.min(0) - [0, 0, TOP], 3)}..{np.round(tp.max(0) - [0, 0, TOP], 3)}; cmd {np.round(r.at() - [0, 0, TOP], 4)}", flush=True)
    for _ in range(2):
        Rd = m.state(env, "driver").rotation()
        yield from r.move(r.at(), grip=d._as_grip(turn_onto(Rd[:, 0], np.array([1.0, 0.0, 0.0])) @ r.grip.rot), t=0.6, label="square the screwdriver up")
        ddbg("squared")
    # its tip, 11 cm out, starts level with the breakers' depth: draw it back in front of them before going sideways
    yield from r.move(r.at() + np.array([-0.075, 0.0, 0.0]), t=1.0, label="the screwdriver drawn back")

    wrist = {"roll": -90.0}  # the wrist's roll about the screwdriver's axis, from level(0): the pick grip is -90
    from yam_arena import ik
    ik_cmd = lambda: ik.tcp_pose(r.q, r.base)[0]  # noqa: E731

    def roll_to(target, t, label):
        """Turn the screwdriver about its own axis to `target`, through 0: never past +/-90, where joint 6 runs out (it stops near -120). A single move
        through 180 degrees can take the other way round and miss by 10 cm, so the turn is an arc about the shaft's line."""
        d_ = target - wrist["roll"]
        if abs(d_) > 0.5:
            for n_, _ in enumerate(r.arc(tip_off(), np.array([1.0, 0.0, 0.0]), d_, t=t, label=label)):
                yield
                if os.environ.get("COMB_DEBUG") and n_ % 4 == 3:
                    dd = m.state(env, "driver")
                    Rd_ = dd.rotation()
                    tp_ = tip_off()
                    near = min(range(3), key=lambda k_: np.linalg.norm(tp_ - head_point(k_)))
                    print(f"[comb] turn {label} +{n_ + 1}: wrist {wrist['roll'] + d_ * min(1.0, (n_ + 1) / (t * 20)):+.0f} driver roll {math.degrees(_roll(Rd_)):+.0f}; "
                          f"tip off head {near} {np.round((tp_ - head_point(near)) * 1000, 1)} mm, shaft {math.degrees(math.acos(min(1.0, Rd_[0, 0]))):.1f} deg; "
                          f"engaged {TERM.engaged}; out {np.round(TERM.out, 2)}; tcp err {1000 * np.linalg.norm(r.tcp() - ik_cmd()):.1f} mm", flush=True)
        wrist["roll"] = target

    def turn_in_head(hd, d_deg, t, label, stop=None):
        """The engaged half-turn: the wrist rolled through d_deg about the shaft's line while the bit's tip is servoed onto the screw head every step.
        An open-loop arc carries the arm's droop compensation round with the roll and walks the tip up to 2 cm off the head at mid-turn."""
        from yam_arena.urdf import axis_angle_matrix
        c0, R0 = ik.tcp_pose(r.q, r.base)
        ctr = tip_off()
        corr = np.zeros(3)
        n = max(1, round(t * 20))
        steps = n + 6
        rot = R0
        worst = 0.0
        a = 0.0
        for k_ in range(1, steps + 1):
            err = hd - tip_off()
            if np.linalg.norm(err) > 0.012 or (stop is not None and stop()):
                break  # the bit has left the head (or the screw is home): stop here, and let the next stroke find the head again
            a = math.radians(d_deg) * ik.smoothstep(min(1.0, k_ / n))
            Rk = axis_angle_matrix(np.array([1.0, 0.0, 0.0]), a)
            rot = Rk @ R0
            corr = corr + np.clip(0.5 * err, -d.SERVO_STEP, d.SERVO_STEP)
            if np.linalg.norm(corr) > 0.015:
                corr = corr * (0.015 / np.linalg.norm(corr))
            mm, _ = r._solve(ctr + Rk @ (c0 - ctr) + corr, rot)
            worst = max(worst, mm)
            yield
            if os.environ.get("COMB_DEBUG") and k_ % 4 == 0:
                Rd_ = m.state(env, "driver").rotation()
                print(f"[comb] turn {label} +{k_}: wrist {wrist['roll'] + math.degrees(a):+.0f} driver roll {math.degrees(_roll(Rd_)):+.0f}; tip off head "
                      f"{np.round((tip_off() - hd) * 1000, 1)} mm, shaft {math.degrees(math.acos(min(1.0, Rd_[0, 0]))):.1f} deg; engaged {TERM.engaged}; "
                      f"out {np.round(TERM.out, 2)}", flush=True)
        r.grip = d._as_grip(rot)
        r._note(worst, 0.0, label)
        wrist["roll"] += math.degrees(a)

    ROLL_A, ROLL_B, ROLL_T = 90.0, -115.0, -60.0  # a stroke's ends: loosening +90 to -115; tightening from -60, clear of joint 6's end (near -120)

    def strokes(k, sign, until, label):
        """Plain strokes on screw k: the bit in, the wrist turned, the bit out, the wrist back; until(k) is true. sign -1 loosens (from roll +90 to -115),
        +1 tightens (from -60 to +90)."""
        hd = head_point(k)
        start, end = (ROLL_A, ROLL_B) if sign < 0 else (ROLL_T, ROLL_A)
        yield from r.move(r.at() + (hd - np.array([0.03, 0.0, 0.0]) - tip_off()), t=1.5, label=f"the bit in line with the {label} screw")
        yield from roll_to(start, 0.8, "the wrist to the stroke's start")
        ddbg(f"in line with the {label} screw, roll {wrist['roll']:+.0f}")
        Rd = m.state(env, "driver").rotation()
        yield from r.move(r.at(), grip=d._as_grip(turn_onto(Rd[:, 0], np.array([1.0, 0.0, 0.0])) @ r.grip.rot), t=0.3, label="square the screwdriver to the screw")
        yield from r.servo(hd - np.array([0.005, 0.0, 0.0]), tol=0.0005, timeout=1.0, read=tip_off, label=f"the bit in front of the {label} screw")
        short = 0.0045 if mode == "hover" else -0.0005  # hover: the tip held 4 mm in front of the head's face, turned but never touching
        for n in range(3 if mode == "hover" else 14):
            if until(k):
                break
            Rd = m.state(env, "driver").rotation()
            yield from r.move(r.at(), grip=d._as_grip(turn_onto(Rd[:, 0], np.array([1.0, 0.0, 0.0])) @ r.grip.rot), t=0.3, label="square the screwdriver")
            yield from r.servo(lambda tt: hd - np.array([max(0.005 - 0.015 * tt, short), 0.0, 0.0]), tol=0.0005, timeout=1.0, read=tip_off,
                               label=f"the bit into the {label} screw")
            yield from turn_in_head(hd - np.array([short, 0.0, 0.0]), end - start, 2.6, f"turn the {label} screw",
                                    stop=(lambda: TERM.bears[k]) if sign > 0 else None)
            dbg(f"{label} stroke {n}: ")
            yield from r.move(r.at() - np.array([0.005, 0.0, 0.0]), t=0.3, label="the bit out")
            yield from roll_to(start, 1.0, "the wrist back")
        yield from r.move(r.at() - np.array([0.03, 0.0, 0.0]), t=0.5, label="the bit clear")

    names = ("right", "middle", "left")
    yield from roll_to(ROLL_A, 1.2, "the wrist to the loosening stroke's start, in the clear")
    if mode not in ("closed", "behind"):
        for k in range(3):
            yield from strokes(k, -1, lambda k: TERM.out[k] >= TURNS_OPEN, names[k])
        _report(ctx, env, "opened: ")
    yield from r.move(np.array([0.165, -0.10, TOP + Z_SCREW + 0.04]), t=1.5, label="the screwdriver parked clear of the row")

    # 2. the comb: by its bar over the free fourth pin, jaws across the bar, lowered into the open terminals, let go
    c0 = m.state(env, "comb")
    grip_pt = lambda: m.state(env, "comb").to_world(np.array([[0.0, PIN_YS[3], 0.0]]))[0]  # noqa: E731
    gc = d.tilted(30.0, -90.0)  # tilted towards the right arm, jaws closing along x, across the bar
    def cdbg(w):
        if os.environ.get("COMB_DEBUG"):
            cc = m.state(env, "comb")
            tp = m.fingertips(env)[:12]
            print(f"[comb] cdbg {w}: comb at {np.round(cc.pos - [0, 0, TOP], 4)} y-axis {np.round(cc.rotation()[:, 1], 3)} z-axis {np.round(cc.rotation()[:, 2], 3)}; grip pt {np.round(grip_pt() - [0, 0, TOP], 4)}; "
                  f"left tips {np.round(tp.min(0) - [0, 0, TOP], 3)}..{np.round(tp.max(0) - [0, 0, TOP], 3)}; cmd {np.round(l.at() - [0, 0, TOP], 4)}", flush=True)

    cdbg("start")
    yield from l.move(grip_pt() + np.array([0.0, 0.0, 0.06]) - 0.0 * gc.rot[:, 2], grip=gc, finger=FINGER_OPEN * 0.4, t=2.5, label="over the comb's free end")
    cdbg("over")
    yield from l.move(grip_pt(), t=1.0, label="jaws either side of the bar")
    cdbg("either side")
    yield from l.close(COMB_BX, 30.0, t=0.6, label="grip the bar")
    cdbg("gripped")
    yield from l.move(l.at() + np.array([0.0, 0.0, 0.05]), t=1.0, label="lift the comb off its posts")
    cdbg("lifted")
    pins_now = lambda: m.state(env, "comb").to_world(np.array([[0.0, PIN_YS[1], -COMB_BZ / 2 - PIN_L]]))[0]  # noqa: E731  the second pin's tip
    back = 0.0024 if mode == "behind" else 0.0  # behind: the pins aimed into the space behind the shut jaws, then pushed forward onto them
    over = np.array([PIN_X + back, YS[1], TOP + Z_TOP + 0.010])
    yield from l.move(l.at() + (over - pins_now()) + np.array([0.0, 0.0, 0.02]), t=2.5, label="the comb over the row")
    cdbg(f"carried, pin 1 tip {np.round(pins_now() - [0, 0, TOP], 4)} for {np.round(over - [0, 0, TOP], 4)}")
    yield from l.servo(over, tol=0.0006, timeout=2.0, read=pins_now, label="the pins over the terminals")
    cdbg(f"servoed, pin 1 tip {np.round(pins_now() - [0, 0, TOP], 4)}")
    dbg("comb over: ")
    yield from l.servo(lambda tt: np.array([PIN_X + back, YS[1], TOP + Z_TOP + max(0.010 - 0.01 * tt, -PIN_L - 0.002)]), tol=0.0008, timeout=2.5, read=pins_now,
                       label="lower the pins into the terminals")
    if mode == "behind":
        yield from l.move(l.at() - np.array([0.0012, 0.0, 0.0]), t=0.6, label="push the comb forward onto the shut jaws")
    _report(ctx, env, "comb lowered: ")
    yield from l.open(t=0.4, label="let go: it rests on the breakers' tops")
    yield from l.move(l.at() + np.array([-0.02, 0.03, 0.06]), t=1.0, label="the left hand clear")
    for _ in range(10):
        yield
    _report(ctx, env, "comb in: ")

    # 3. the screws driven down onto the pins
    if mode != "loose":
        for k in range(3):
            yield from strokes(k, 1, lambda k: TERM.bears[k], names[k])
        _report(ctx, env, "closed: ")
    if mode == "leave":  # the screws down, but the bit set back in the last head and the hand left on the screwdriver
        yield from r.servo(head_point(2) - np.array([0.002, 0.0, 0.0]), tol=0.001, timeout=1.5, read=tip_off, label="the bit back on the left screw")
        for k in range(80):
            yield
            if k % 40 == 39:
                _report(ctx, env, "left on the screw: ")
        return
    # 4. the screwdriver back on its stand
    st = SD_START + np.array([0.008, 0.0, TOP + 0.0025])
    yield from roll_to(-90.0, 0.8, "the wrist back to the pick grip")
    yield from r.move(st + np.array([-0.075, 0.0, 0.04]), grip=g_lo, t=2.0, label="right of the plate, the tip in front of it")
    yield from r.move(st + np.array([0.0, 0.0, 0.04]), t=1.0, label="over the stand")
    yield from r.move(st + np.array([0.0, 0.0, 0.004]), t=1.0, label="the screwdriver on its stand")
    yield from r.open(t=0.4, label="let go of the screwdriver")
    yield from r.move(r.at() - np.array([0.06, 0.0, -0.03]), t=1.0, label="back away")
    for k in range(80):
        yield
        if k % 40 == 39:
            _report(ctx, env, "let go: ")
