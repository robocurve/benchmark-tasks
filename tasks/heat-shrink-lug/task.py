"""Heat-shrink a sleeve over a crimped cable lug: one hand holds the cable by its jacket and turns it, the other works a heat gun along the sleeve from
end to end, never touching it and never lingering, until every part of the sleeve has recovered.

A power-cable termination on a bench: a compression lug crimped on a 150 mm stub of 2 AWG battery cable rests on two V-blocks, an adhesive-lined
dual-wall heat-shrink sleeve (3:1, 60 mm) already slid over the lug's barrel and the jacket behind it. A compact heat gun lies switched off in its stand to the right; a push switch on its body turns it on.
The maker's steps (TE Connectivity 114-120022): "Start to shrink the tubing starting from one end of the assembly. Work progressively to the end of the
tubing ensuring uniform heat application. This can be achieved by either rotating the assembly or the heat gun. Ensure that the end of the heat gun does
not touch the exterior of the tubing otherwise splitting may occur."

Model (declared in code): the shrink is bookkeeping, not physics. The sleeve is 8 sections of 6 sides; each side takes a heat dose each step from the
gun's jet, by the nozzle's distance along the jet (full to 50 mm, none past 120 mm), its offset from the jet's axis (Gaussian, 12 mm) and how squarely
the side faces it. A side has recovered at dose 1 (about 1 s square in the jet) and chars past dose 4.5; a section recovers when all six of its
sides have, and is then drawn shrunk onto the barrel and jacket. The nozzle within 3 mm of the sleeve splits it. The cable and lug are one rigid body;
the sleeve does not slide.

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
INSTRUCTION = ("Switch the heat gun on, shrink the sleeve over the lug and cable with it, then put the gun back in its stand, switch it off and move both "
               "hands away.")
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 40  # 2 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- the cable assembly (its frame: origin at the jacket's cut end, +x along the cable to the lug)
CY, Z_REST = 0.03, 0.13  # the cable's axis: y, and its height on its V-blocks (3 cm above the jet of the gun lying in its stand)
CY_REST = CY
X0 = 0.22  # the jacket's end at the start
R_JACKET, L_JACKET = 0.0055, 0.10  # 2 AWG battery cable, about 11 mm over the jacket
R_BARREL, L_BARREL = 0.006, 0.030
TONGUE = (0.035, 0.017, 0.0025)  # length, width, thickness
S_X0, S_X1, N_SEC, N_SIDE = 0.065, 0.125, 8, 6  # the sleeve over 35 mm of jacket and 25 mm of barrel
R_EXP = 0.0100  # the sleeve's outside as supplied (19 mm bore, about 0.5 mm wall)
WALL_AREA = (0.00525 ** 2 - 0.0030 ** 2)  # the recovered wall's section over pi (ATUM 19/6 recovered free: 6.0 mm bore, 2.25 mm wall); kept as it shrinks onto the core
# -- the heat gun (its frame: origin at the nozzle's tip, the jet along +y, the pistol grip below the barrel)
GUN_MASS = 0.745  # Weller WLHG1212A, 745 g
SWEEP = 0.018  # m/s: the jet's speed along the sleeve
GOFF, GD = 0.10, 0.095  # the hand's place on the grip: behind the nozzle, and out along the grip from the barrel's axis (its last 3 cm)
G_ROLL = 90.0  # the gun lies, and is held, on its side: the grip points back at the robot (the nozzle's aim does not change)
NOZZLE_REST = np.array([0.31, -0.03, 0.10])  # low: the sweep passes over the stand
GAP = 0.05  # the nozzle's working distance from the cable's axis
SWITCH = np.array([-0.029, -0.085, 0.0])  # the push switch on the barrel's side (gun frame): uppermost as the gun lies in its stand, between its cradles
BODY_N = 9  # GUN_PTS: the body's axis every 22.5 mm, then the grip's every 17.5 mm
GUN_PTS = np.array([[0.0, y, 0.0] for y in np.linspace(0.0, -0.18, BODY_N)] + [[0.0, -GOFF, -z] for z in np.linspace(0.03, 0.10, 5)])  # along the barrel and grip
CABLE_PTS = np.array([[x, 0.0, 0.0] for x in np.linspace(0.0, 0.165, 8)])  # along the cable and lug
# -- the heat
D_FULL, D_NONE, SIGMA = 0.05, 0.12, 0.012
RATE = 1.0  # dose per second, square in the jet at full strength
REC, CHAR = 1.0, 4.5
HAND_LIMIT = 3.0  # the holding fingers' dose that fails the trial: 3 s square in the jet

LOOKS = {
    "jacket": m.Look(color=m.rgb(0.08, 0.08, 0.09), roughness=0.6),
    "copper": m.Look(color=m.rgb(0.82, 0.55, 0.38), roughness=0.3, metallic=0.9),
    "tinned": m.Look(color=m.rgb(0.78, 0.78, 0.80), roughness=0.35, metallic=0.9),
    "sleeve": m.Look(color=m.rgb(0.15, 0.15, 0.17), roughness=0.75),
    "shrunk": m.Look(color=m.rgb(0.04, 0.04, 0.05), roughness=0.35),
    "char": m.Look(color=m.rgb(0.30, 0.18, 0.08), roughness=0.95),
    "glue": m.Look(color=m.rgb(0.85, 0.75, 0.40), roughness=0.4),
    "gun": m.Look(color=m.rgb(0.85, 0.35, 0.10), roughness=0.55),
    "gun_dark": m.Look(color=m.rgb(0.12, 0.12, 0.13), roughness=0.6),
    "nozzle": m.Look(color=m.rgb(0.70, 0.70, 0.72), roughness=0.3, metallic=0.9),
    "lit": m.Look(color=m.rgb(0.95, 0.10, 0.08), roughness=0.3),
    "block": m.Look(color=m.rgb(0.55, 0.55, 0.58), roughness=0.6),
    "stand": m.Look(color=m.rgb(0.20, 0.20, 0.22), roughness=0.6),
}


def _b(name, size, c, look, cols=None):
    if cols is not None:
        cols.append(m.col_box(name + "_c", size, c))
    return (name, m.box(size, c), look)


def _gun_R():
    a = math.radians(G_ROLL)
    return np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]])


STAND_TOPS = []  # the stand's blocks: centre x, y, half sizes in x and y, top height, and the radius of the gun part that lies on it (barrel or grip)


def _fixed():
    """Two V-blocks under the cable; the gun's stand (barrel cradles and a rest under the grip's end)."""
    parts, cols = [], []
    for k, (xx, half, r) in enumerate(((X0 + 0.05, R_JACKET, R_JACKET), (X0 + L_JACKET + L_BARREL + 0.02, TONGUE[1] / 2, TONGUE[2] / 2))):
        top = Z_REST - r
        parts.append(_b(f"vb{k}", (0.02, 0.04, top), (xx, CY_REST, top / 2), "block", cols))
        for s in (1, -1):
            parts.append(_b(f"vb{k}_rail{'pn'[s < 0]}", (0.02, 0.004, 0.012), (xx, CY_REST + s * (half + 0.004), top + 0.006), "block", cols))
    Rg = _gun_R()
    for k in range(2):  # under the barrel, along it
        c = NOZZLE_REST + Rg @ np.array([0.0, -0.04 - 0.09 * k, 0.0])
        top = c[2] - 0.026
        parts.append(_b(f"cradle{k}", (0.03, 0.02, top), (c[0], c[1], top / 2), "stand", cols))
        STAND_TOPS.append((c[0], c[1], 0.015, 0.010, top, 0.026))
    # a block under the grip's inner half (the hand takes its outer end): the gun lies on its side, grip out level
    c = NOZZLE_REST + Rg @ np.array([0.0, -GOFF, -0.045])
    top = c[2] - 0.014
    parts.append(_b("grip_rest", (0.03, 0.03, top), (c[0], c[1], top / 2), "stand", cols))
    STAND_TOPS.append((c[0], c[1], 0.015, 0.015, top, 0.014))
    return parts, cols


_fp, _fc = _fixed()
FIXED = m.fixed("bench", (0.0, 0.0, TOP), _fp, _fc, LOOKS, friction=(0.8, 0.7))


def sec_x(k):
    return S_X0 + (k + 0.5) * (S_X1 - S_X0) / N_SEC


def surface_r(k, recovered: bool) -> float:
    """Station k's outside radius: as supplied, or recovered onto its core (the jacket's or the barrel's) with the wall's section kept."""
    if not recovered:
        return R_EXP
    under = R_BARREL if sec_x(k) > L_JACKET else R_JACKET
    return math.sqrt(under ** 2 + WALL_AREA)


def _cable():
    parts, cols = [], []
    xb = L_JACKET
    parts.append(("jacket", m.cylinder(R_JACKET, L_JACKET, (L_JACKET / 2, 0.0, 0.0), axis="x", segments=20), "jacket"))
    parts.append(("barrel", m.cylinder(R_BARREL, L_BARREL, (xb + L_BARREL / 2, 0.0, 0.0), axis="x", segments=20), "tinned"))
    parts.append(("strands", m.cylinder(R_JACKET - 0.0012, 0.004, (-0.0015, 0.0, 0.0), axis="x", segments=16), "copper"))
    tl, tw, tt = TONGUE
    parts.append(("tongue", m.box((tl, tw, tt), (xb + L_BARREL + tl / 2, 0.0, 0.0)), "tinned"))
    parts.append(("hole", m.cylinder(0.0050, tt + 0.0004, (xb + L_BARREL + tl - 0.010, 0.0, 0.0), segments=16), "gun_dark"))
    cols.append(m.col_hull("jacket_c", m.cylinder(R_JACKET, L_JACKET, (L_JACKET / 2, 0.0, 0.0), axis="x", segments=12)))
    cols.append(m.col_hull("barrel_c", m.cylinder(R_BARREL, L_BARREL, (xb + L_BARREL / 2, 0.0, 0.0), axis="x", segments=12)))
    cols.append(m.col_box("tongue_c", (tl, tw, tt), (xb + L_BARREL + tl / 2, 0.0, 0.0)))
    w = (S_X1 - S_X0) / N_SEC
    for k in range(N_SEC):
        r_on = surface_r(k, True)  # drawn recovered onto its core: 14.0 mm across over the jacket, 14.8 mm over the barrel
        parts.append((f"sec{k}_exp", m.cylinder(R_EXP, w - 0.0004, (sec_x(k), 0.0, 0.0), axis="x", segments=24), "sleeve"))
        parts.append((f"sec{k}_rec", m.cylinder(r_on, w + 0.0002, (sec_x(k), 0.0, 0.0), axis="x", segments=24), "shrunk", True))
        parts.append((f"sec{k}_char", m.cylinder(r_on + 0.0001, w + 0.0002, (sec_x(k), 0.0, 0.0), axis="x", segments=24), "char", True))
    for k, (x, under) in enumerate(((S_X0 - 0.001, R_JACKET), (S_X1 + 0.001, R_BARREL))):
        parts.append((f"glue{k}", m.cylinder(under + 0.0010, 0.002, (x, 0.0, 0.0), axis="x", segments=20), "glue", True))
    return parts, cols


_cp, _cc = _cable()
CABLE = m.free("cable", (X0, CY_REST, TOP + Z_REST), _cp, _cc, LOOKS, mass=0.08, friction=(0.9, 0.8))


def _gun():
    parts = [("barrel", m.cylinder(0.026, 0.14, (0.0, -0.10, 0.0), axis="y", segments=24), "gun"),
             ("cap", m.cylinder(0.024, 0.012, (0.0, -0.176, 0.0), axis="y", segments=24), "gun_dark"),
             ("nozzle", m.cylinder(0.011, 0.030, (0.0, -0.015, 0.0), axis="y", segments=20), "nozzle"),
             ("grip", m.box((0.028, 0.035, 0.085), (0.0, -GOFF, -0.0685)), "gun_dark"),
             ("trigger", m.box((0.010, 0.008, 0.022), (0.0, -GOFF + 0.021, -0.040)), "gun"),
             ("switch_off", m.box((0.005, 0.014, 0.010), tuple(SWITCH + np.array([0.0025, 0.0, 0.0]))), "gun_dark"),
             ("switch_on", m.box((0.005, 0.014, 0.010), tuple(SWITCH + np.array([0.0025, 0.0, 0.0]))), "lit", True)]
    cols = [m.col_hull("barrel_c", m.cylinder(0.026, 0.17, (0.0, -0.115, 0.0), axis="y", segments=16)),
            m.col_hull("nozzle_c", m.cylinder(0.011, 0.030, (0.0, -0.015, 0.0), axis="y", segments=12)),
            m.col_box("grip_c", (0.028, 0.035, 0.085), (0.0, -GOFF, -0.0685))]
    return parts, cols


def _quat(R):
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    return ((R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w)


_gp, _gc = _gun()
GUN = m.free("gun", tuple(NOZZLE_REST + np.array([0.0, 0.0, TOP + 0.001])), _gp, _gc, LOOKS, mass=GUN_MASS, friction=(0.9, 0.8), start_quat=_quat(_gun_R()),
              com=(0.0, -GOFF, -0.05))  # a pistol-grip gun balances over its grip (motor and fan above the hand)
OBJECTS = [FIXED, CABLE, GUN]


# -- the heat and the scorer --------------------------------------------------------------------------------------------------------------------
def _falloff(t):
    if t <= 0.0:
        return 0.0
    return 1.0 if t <= D_FULL else max(0.0, (D_NONE - t) / (D_NONE - D_FULL))


def _rate(p, j, s, n):
    """Dose a second at surface point s (outward normal n) from a jet leaving p along j."""
    v = s - p
    t = float(np.dot(v, j))
    lat = float(np.linalg.norm(v - t * j))
    face = max(0.0, float(np.dot(n, -j)))
    return RATE * math.sqrt(face) * math.exp(-(lat / SIGMA) ** 2) * _falloff(t)


NOZZLE_PTS = np.array([[0.0, 0.0, 0.0]] + [[r * math.cos(a), y, r * math.sin(a)] for y, r in ((0.0, 0.011), (-0.01, 0.011), (-0.02, 0.011), (-0.03, 0.011), (-0.03, 0.026))
                                             for a in np.linspace(0.0, 2 * math.pi, 12, endpoint=False)])  # the nozzle's tip, its rim and side, the barrel's front rim (gun frame)


def nozzle_on_sleeve(c, g, recovered) -> bool:
    """Whether any point of the nozzle (its tip and rim, its side, the barrel's front rim) is within 3 mm of the sleeve's surface, or inside it, along
    the sleeve's length: the nozzle touching the sleeve, which splits it."""
    pts = g.to_world(NOZZLE_PTS) - np.array([0.0, 0.0, TOP])
    cp = c.pos - np.array([0.0, 0.0, TOP])
    ax = c.rotation()[:, 0]
    w = (S_X1 - S_X0) / N_SEC
    for k in range(N_SEC):
        v = pts - (cp + ax * sec_x(k))
        along = v @ ax
        radial = np.linalg.norm(v - np.outer(along, ax), axis=1)
        if np.any((np.abs(along) < w / 2 + 0.003) & (radial < surface_r(k, recovered[k]) + 0.003)):
            return True
    return False


def _grasped(g, tips_world) -> bool:
    """One hand's two fingers on opposite sides of the gun's grip, across its 28 mm (the gun's x), within the grip's length and width."""
    loc = g.to_local(tips_world)
    for arm in (0, 12):
        fa, fb = loc[arm:arm + 6], loc[arm + 6:arm + 12]

        def span(P):
            return (np.abs(P[:, 1] + GOFF) < 0.03) & (P[:, 2] < -0.025) & (P[:, 2] > -0.115)

        def side(P, sgn):
            return bool(np.any(span(P) & (sgn * P[:, 0] > 0.008) & (sgn * P[:, 0] < 0.0215)))  # the jaws closed on the 28 mm grip, within about 3 mm of it

        if (side(fa, 1) and side(fb, -1)) or (side(fa, -1) and side(fb, 1)):
            return True
    return False


BARREL_LINE = np.array([[0.0, y, 0.0] for y in np.linspace(0.0, -0.19, 20)])
GRIP_LINE = np.array([[0.0, -GOFF, -z] for z in np.linspace(0.02, 0.10, 9)])


def on_its_stand(g) -> bool:
    """Whether any of the gun lies on its stand: a point of the barrel's (or the grip's) underside over one of the stand's blocks and within 1 cm of
    its top. A gun slid along its cradles, or lifted a few millimetres, is still on them."""
    for line, which in ((BARREL_LINE, 0.026), (GRIP_LINE, 0.014)):
        pts = g.to_world(line) - np.array([0.0, 0.0, TOP])
        for cx, cy, hx, hy, top, r in STAND_TOPS:
            if r != which:
                continue
            over = (np.abs(pts[:, 0] - cx) < hx + 0.02) & (np.abs(pts[:, 1] - cy) < hy + 0.01)
            if np.any(over & (pts[:, 2] - r - top < 0.01)):
                return True
    return False


class Heat:
    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.dose = np.zeros((N_SEC, N_SIDE))
        self.hand = 0.0
        self.split = False
        self.shown = None
        self.on = False
        self.running = False
        self.pressing = False
        self.presses = 0

    def step(self, env):
        c = m.state(env, "cable")
        g = m.state(env, "gun")
        Rc, Rg = c.rotation(), g.rotation()
        p = g.pos - np.array([0.0, 0.0, TOP])
        j = Rg[:, 1]
        # the push switch: a finger pressed onto it toggles it (within 10 mm across; the fingertip point 4.5 to 15 mm past its face, the finger's real
        # underside sitting about 5 mm above that point); it must be let go before it toggles again
        sw = g.to_world(SWITCH[None, :])[0] - np.array([0.0, 0.0, TOP])
        out = -Rg[:, 0]  # the switch's outward normal
        tips = m.fingertips(env) - np.array([0.0, 0.0, TOP])
        v = tips - sw
        depth = -(v @ out)
        lateral = np.linalg.norm(v - np.outer(v @ out, out), axis=1)
        pressed = bool(np.any((lateral < 0.010) & (depth > 0.0045) & (depth < 0.015)))
        if pressed and not self.pressing:
            self.on = not self.on
            self.presses += 1
        self.pressing = pressed
        cp = c.pos - np.array([0.0, 0.0, TOP])
        # the nozzle on the sleeve splits it, whether the gun is running or not
        if nozzle_on_sleeve(c, g, [self.recovered(k) for k in range(N_SEC)]):
            self.split = True
        # the gun blows hot only when switched on, held by its grip, and off its stand
        held = _grasped(g, m.fingertips(env))
        in_stand = on_its_stand(g)
        low = g.to_world(GUN_PTS)[:, 2] - TOP - np.where(np.arange(len(GUN_PTS)) < BODY_N, 0.026, 0.014)
        clear_of_table = float(np.min(low)) > 0.01
        self.running = self.on and held and not in_stand and clear_of_table
        if not self.running:
            self.show(env)
            return
        for k in range(N_SEC):
            ctr = cp + Rc[:, 0] * sec_x(k)
            r = surface_r(k, self.recovered(k))
            for i in range(N_SIDE):
                a = 2 * math.pi * i / N_SIDE
                nrm = Rc[:, 1] * math.cos(a) + Rc[:, 2] * math.sin(a)
                self.dose[k, i] += _rate(p, j, ctr + r * nrm, nrm) / 20.0
        self.hand += max(_rate(p, j, tp, -j) for tp in tips) / 20.0  # every fingertip of both hands
        self.show(env)

    def recovered(self, k) -> bool:
        return bool(np.all(self.dose[k] >= REC))

    def charred(self, k) -> bool:
        return bool(np.any(self.dose[k] >= CHAR))

    def show(self, env):
        state = tuple((self.recovered(k), self.charred(k)) for k in range(N_SEC))
        if (state, self.on) == self.shown:
            return
        self.shown = (state, self.on)
        for k, (rec, ch) in enumerate(state):
            m.set_visible(f"{m.ENV_PATH}/cable/visual/sec{k}_exp", not rec)
            m.set_visible(f"{m.ENV_PATH}/cable/visual/sec{k}_rec", rec and not ch)
            m.set_visible(f"{m.ENV_PATH}/cable/visual/sec{k}_char", rec and ch)
        m.set_visible(f"{m.ENV_PATH}/cable/visual/glue0", state[0][0])
        m.set_visible(f"{m.ENV_PATH}/cable/visual/glue1", state[-1][0])
        m.set_visible(f"{m.ENV_PATH}/gun/visual/switch_on", self.on)
        m.set_visible(f"{m.ENV_PATH}/gun/visual/switch_off", not self.on)


HEAT = Heat()
m.on_step(HEAT.step)
m.on_reset(HEAT.reset)


def now(env) -> dict:
    n_rec = sum(HEAT.recovered(k) for k in range(N_SEC))
    charred = any(HEAT.charred(k) for k in range(N_SEC))
    g = m.state(env, "gun")
    Rg0 = _gun_R()
    on_stand = bool(np.linalg.norm(g.pos - np.array([0.0, 0.0, TOP]) - NOZZLE_REST) < 0.03 and float(g.rotation()[:, 1] @ Rg0[:, 1]) > 0.966
                    and float(g.rotation()[:, 0] @ Rg0[:, 0]) > 0.966 and on_its_stand(g))  # back in its place, aimed and rolled as it lay, lying on the stand
    tips = m.fingertips(env) - np.array([0.0, 0.0, TOP])
    c = m.state(env, "cable")
    gun_pts = g.to_world(GUN_PTS) - np.array([0.0, 0.0, TOP])
    cable_pts = c.to_world(CABLE_PTS) - np.array([0.0, 0.0, TOP])
    d_gun = float(np.min(np.linalg.norm(tips[:, None, :] - gun_pts[None], axis=2)))
    d_cable = float(np.min(np.linalg.norm(tips[:, None, :] - cable_pts[None], axis=2)))
    clear = d_gun > 0.05 and d_cable > 0.05  # every fingertip of both hands, from the whole gun and the whole cable
    fell = bool(c.pos[2] - TOP < 0.0 or g.pos[2] - TOP < 0.0)
    return {"n_rec": n_rec, "charred": charred, "ends": HEAT.recovered(0) and HEAT.recovered(N_SEC - 1), "on_stand": on_stand, "off": not HEAT.on, "clear": clear,
            "fell": fell, "split": HEAT.split, "hand": HEAT.hand, "d_hands": (d_cable, d_gun)}


def _moment(c) -> float:
    if c["split"] or c["fell"] or c["hand"] > HAND_LIMIT:
        return 0.0
    s = 0.5 * c["n_rec"] / N_SEC  # 0.0625 a recovered station
    if 2 * c["n_rec"] < N_SEC:  # the other tiers only once at least half have recovered: an unheated sleeve has charred nothing
        return round(s, 4)
    s += 0.2 if c["ends"] else 0.0
    s += 0.2 if not c["charred"] else 0.0
    s += 0.1 if (c["on_stand"] and c["off"] and c["clear"]) else 0.0
    return round(s, 4)


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.hold = 0
        self.full = False
        self.score = 0.0

    def step(self, env):
        c = now(env)
        mo = _moment(c)
        self.hold = self.hold + 1 if mo >= 0.999 else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        bad = c["split"] or c["fell"] or c["hand"] > HAND_LIMIT
        self.score = 0.0 if bad else (1.0 if self.full else min(mo, 0.95))


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


def _report(ctx, env, label=""):
    c = now(env)
    print(f"[heat] t {ctx.t:5.1f} {label}recovered {c['n_rec']}/{N_SEC} {''.join('R' if HEAT.recovered(k) else '.' for k in range(N_SEC))}, min side dose "
          f"{np.round(HEAT.dose.min(axis=1), 2)}, max {HEAT.dose.max():.2f}, charred {c['charred']}, split {c['split']}, hand dose {c['hand']:.2f}; gun on stand "
          f"{c['on_stand']} (lying on it {on_its_stand(m.state(env, 'gun'))}, held {_grasped(m.state(env, 'gun'), m.fingertips(env))}, running {HEAT.running}), clear {c['clear']} {np.round(np.array(c['d_hands']) * 1000)} mm; score {score(None):.2f}", flush=True)
    g, cb = m.state(env, "gun"), m.state(env, "cable")
    print(f"[heat]   nozzle {np.round(g.pos - [0, 0, TOP], 4)} jet {np.round(g.rotation()[:, 1], 3)}; cable end {np.round(cb.pos - [0, 0, TOP], 4)} axis "
          f"{np.round(cb.rotation()[:, 0], 3)}, roll {math.degrees(math.atan2(cb.rotation()[2, 1], cb.rotation()[1, 1])):+.1f} deg; right cmd {np.round(ctx.right.at() - [0, 0, TOP], 3)}", flush=True)
    if os.environ.get("HEAT_DEBUG"):
        print(f"[heat]   dose by station (rows) and side (columns):\n{np.round(HEAT.dose, 2)}", flush=True)


def demo_script(ctx):
    """Left: takes the cable by its jacket end, lifts it level and turns it in 48-degree steps (-120 to +120). Right: takes the gun by its grip and, at each
    step, brings the jet's aim onto the cable's axis 50 mm from the nozzle and holds the gun still while the left hand slides the cable through the jet,
    end to end. Then the gun goes back in its stand and the cable back on its V-blocks.
    Failure checks (--mode): none does nothing; still holds the gun still at mid-sleeve without turning the cable; touch drives the nozzle onto the sleeve;
    stand leaves the gun switched on in its stand and slides and turns the cable through its jet; hover does the same with an open hand round the
    gun's grip; nudge first pushes the gun 6 cm out of its stand by its rear cap."""
    import demokit as d
    from yam_arena.constants import FINGER_OPEN
    from yam_arena.urdf import axis_angle_matrix

    env = ctx.emb._env
    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode not in ("full", "none", "still", "touch", "stand", "nudge", "hover"):
        raise ValueError(mode)
    for _ in range(10):
        yield
    _report(ctx, env, f"{mode} start: ")
    if mode == "none":
        for _ in range(40):
            yield
        return
    ZW, XG = 0.16, X0 + 0.01  # the work height; the left hand's place along the cable: 1 cm from its end, clear of the nozzle at the slide's end
    Rg = _gun_R()
    gl = d._as_grip(axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.pi / 2) @ d.level(0).rot)  # fingers along the grip, jaws top and bottom
    a_g = gl.rot[:, 2]

    def grip_for(nozzle):
        return np.asarray(nozzle, float) + Rg @ np.array([0.0, -GOFF, -GD]) + np.array([0.0, 0.0, TOP])

    def joint_move(arm, pos, grip, t, finger):
        """Solve the IK for the goal afresh (from the ready pose and from here, the better), then move the joints straight to it."""
        from yam_arena import ik
        from yam_arena.constants import READY
        best = None
        for seed in (np.array(READY, float), arm.q.copy()):
            q, pe, re_ = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(grip.rot), seed, base_pos=arm.base, iters=200)
            if best is None or pe + 0.01 * re_ < best[1] + 0.01 * best[2]:
                best = (q, pe, re_)
        q1, pe, re_ = best
        arm._note(pe * 1000, math.degrees(re_), "joint move")
        q0 = arm.q.copy()
        n = max(1, round(t * 20))
        fr = float(np.clip(finger / FINGER_OPEN, 0.0, 1.0))
        for k in range(1, n + 1):
            u = k / n
            arm.q = q0 + (q1 - q0) * (u * u * (3 - 2 * u))
            arm.cmd = np.r_[arm.q, fr]
            yield
        arm.grip, arm.finger = grip, finger

    def press_switch(label):
        """Closed fingertips straight down onto the gun's push switch, 4 mm in, and up again."""
        sw = m.state(env, "gun").to_world(SWITCH[None, :])[0]  # the switch's face, uppermost as the gun lies on its side
        low = lambda: min(m.fingertips(env)[12:], key=lambda tp: tp[2])  # noqa: E731
        yield from r.move(r.at(), finger=0.0, t=0.4, label="close the fingers")
        yield from r.move(np.array([r.at()[0], r.at()[1], max(r.at()[2], sw[2] + 0.10)]), t=0.8, label="the hand up, clear of the gun")
        if label == "off":  # from the side grip on the gun the wrist must flip to point down, which no Cartesian path follows
            yield from joint_move(r, sw + np.array([0.0, 0.0, 0.07]), d.down(0), 2.0, 0.0)
        else:
            yield from r.move(sw + np.array([0.0, 0.0, 0.07]), grip=d.down(0), t=1.5, label=f"over the gun's switch, fingers closed: {label}")
        yield from r.servo(lambda tt: sw + np.array([0.0, 0.0, max(0.035 - 0.03 * tt, -0.007)]), tol=0.0015, timeout=2.0, gain=0.3, read=low,
                           label=f"press the switch: {label}")
        for _ in range(4):
            yield
        yield from r.move(r.at() + np.array([0.0, 0.0, 0.05]), t=0.6, label="the fingers off the switch")

    # 1. the right switches the gun on; both hands to their parts: the right to the gun's grip, the left to the cable's jacket end
    yield from press_switch("on")
    _report(ctx, env, "switched: ")
    if mode in ("stand", "nudge", "hover"):  # the gun left switched on in its stand (nudge: pushed out of it; hover, nudge: an open hand round its grip)
        lv0 = d.level(0)
        if mode in ("nudge", "hover"):
            if mode == "nudge":  # closed fingers push the gun's rear cap 6 cm along its barrel, off its cradles
                cap = NOZZLE_REST + _gun_R() @ np.array([0.0, -0.19, 0.0]) + np.array([0.0, 0.0, TOP])
                yield from r.move(r.at(), finger=0.0, t=0.4, label="close the fingers")
                yield from r.move(cap + np.array([0.0, -0.06, 0.04]), grip=d.level(90), t=2.0, label="behind the gun's rear cap")
                yield from r.move(cap + np.array([0.0, -0.03, 0.0]), t=0.8, label="level with the rear cap")
                yield from r.move(cap + np.array([0.0, 0.06, 0.0]), t=1.5, label="nudge the gun out of its stand")
                yield from r.move(r.at() + np.array([0.0, -0.04, 0.06]), t=0.8, label="the hand off the gun")
            gnz = m.state(env, "gun").pos - np.array([0.0, 0.0, TOP])
            rest_g = grip_for(gnz)
            yield from r.move(rest_g - 0.06 * a_g + np.array([0.0, 0.0, 0.03]), grip=gl, finger=FINGER_OPEN, t=2.5, label="behind the gun's grip, hand open")
            yield from r.move(rest_g - 0.05 * a_g, t=0.8, label="level with the grip")
            yield from r.move(rest_g, t=1.0, label="open fingers either side of the grip, not closed")
            _report(ctx, env, f"{mode}: ")
        yield from l.move(np.array([XG - 0.05, CY_REST, TOP + Z_REST + 0.025]), grip=lv0, finger=FINGER_OPEN * 0.5, t=2.5, label="behind the cable's end")
        yield from l.move(np.array([XG - 0.05, CY_REST, TOP + Z_REST]), t=0.8, label="level with the cable")
        yield from l.move(np.array([XG, CY_REST, TOP + Z_REST]), t=1.0, label="fingers either side of the jacket")
        yield from l.close(2 * R_JACKET, 40.0, t=0.5, label="grip the jacket")
        nz = m.state(env, "gun").pos - np.array([0.0, 0.0, TOP])  # the nozzle where the gun now lies
        y_jet, z_jet = nz[1] + GAP, max(nz[2], 0.07)
        x_a, x_b = nz[0] - (S_X1 - 0.02) - 0.012, nz[0] - (S_X0 - 0.02) + 0.012  # the hand's x that puts the jet at each sleeve end
        yield from l.move(np.array([x_a, y_jet, TOP + z_jet + 0.03]), t=1.5, label="the cable over the parked jet")
        for k, a in enumerate((-120.0, -60.0, 0.0, 60.0, 120.0)):
            yield from l.move(l.at(), grip=d._as_grip(axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(a)) @ lv0.rot), t=0.8, label="turn the cable")
            xf, xt = (x_a, x_b) if k % 2 == 0 else (x_b, x_a)
            yield from l.move(np.array([xf, y_jet, TOP + z_jet]), t=0.6, label="the sleeve's end in the jet")
            yield from l.move(np.array([xt, y_jet, TOP + z_jet]), t=abs(xt - xf) / 0.015, label="slide the sleeve through the jet")
        _report(ctx, env, "slid through the parked jet: ")
        yield from l.move(np.array([XG, CY_REST, TOP + Z_REST + 0.03]), grip=lv0, t=1.5, label="the cable back over its V-blocks")
        yield from l.move(np.array([XG, CY_REST, TOP + Z_REST + 0.002]), t=0.8, label="the cable onto its V-blocks")
        yield from l.open(t=0.4, label="let go of the cable")
        yield from l.move(np.array([XG - 0.08, CY_REST, TOP + Z_REST + 0.04]), t=1.0, label="the left hand back")
        yield from press_switch("off")
        yield from r.move(r.at() + np.array([-0.05, 0.0, 0.05]), t=1.0, label="the right hand clear")
        for k in range(60):
            yield
        _report(ctx, env, "end: ")
        return
    rest = grip_for(NOZZLE_REST)
    lv = d.level(0)
    yield from ctx.together(r.move(rest - 0.06 * a_g + np.array([0.0, 0.0, 0.03]), grip=gl, finger=FINGER_OPEN * 0.7, t=2.5, label="behind the gun's grip"),
                            l.move(np.array([XG - 0.05, CY_REST, TOP + Z_REST + 0.025]), grip=lv, finger=FINGER_OPEN * 0.5, t=2.5, label="behind the cable's end"))
    yield from ctx.together(r.move(rest - 0.05 * a_g, t=0.8, label="level with the grip"), l.move(np.array([XG - 0.05, CY_REST, TOP + Z_REST]), t=0.8, label="level with the cable"))
    yield from ctx.together(r.move(rest, t=1.0, label="fingers either side of the grip"), l.move(np.array([XG, CY_REST, TOP + Z_REST]), t=1.0, label="fingers either side of the jacket"))
    yield from ctx.together(r.close(0.024, 50.0, t=0.5, label="grip the gun"), l.close(2 * R_JACKET, 40.0, t=0.5, label="grip the jacket"))
    yield from ctx.together(r.move(rest + np.array([0.0, 0.0, 0.03]), t=0.8, label="lift the gun"), l.move(np.array([XG, CY_REST, TOP + ZW]), t=1.0, label="lift the cable level"))
    yield from l.move(np.array([XG, CY, TOP + ZW]), t=1.2, label="the cable to the work line")
    _report(ctx, env, "picked: ")

    def roll_to(deg, t=0.8):
        yield from l.move(l.at(), grip=d._as_grip(axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(deg)) @ lv.rot), t=t, label=f"turn the cable to {deg:+.0f}")

    xs0, xs1 = X0 + S_X0 + 0.002, X0 + S_X1 + 0.012  # the sweep: from the sleeve's end by the hand (no further towards it), a little past the lug end
    noz = lambda x: grip_for((x, CY - GAP, ZW))  # noqa: E731

    def aim():
        g = m.state(env, "gun")
        return g.pos + GAP * g.rotation()[:, 1]

    BACK = 0.07

    def aim_back():
        g = m.state(env, "gun")
        return g.pos + (GAP + BACK) * g.rotation()[:, 1]

    def cable_at(x):
        c = m.state(env, "cable")
        ax = c.rotation()[:, 0]
        return c.pos + ax * (x - c.pos[0]) / max(ax[0], 0.5)
    JX = 0.31  # the jet's place along x: the right hand keeps the gun here, mid-reach, and the left hand slides the cable through its jet
    LX_FROM, LX_TO = S_X1 + 0.032, S_X0 - 0.008  # each pass along the cable from its jacket end: from 3 cm past the lug end to 1 cm past the jacket end
    # (further, the left hand's fingers meet the nozzle at the -120 turn)

    def cable_x(lx):
        """The world x of the point `lx` along the cable from its jacket end."""
        c = m.state(env, "cable")
        return float(c.pos[0] + c.rotation()[0, 0] * lx)

    def hold_aim(dur):
        """Hold the gun still for `dur` seconds as the cable slides through its jet: no corrections, which set the 745 g gun swinging on the soft wrist."""
        for _ in range(max(1, round(dur * 20))):
            yield

    yield from r.move(noz(JX) + np.array([0.0, -0.03, 0.06]), t=1.5, label="the gun towards the sleeve, 6 cm above its line (no dose there)")

    def level_jet(times=2):
        """Turn the hand by the jet's seen error so that it points along +y, yawed 20 degrees so that the barrel behind the nozzle angles away from the left
        hand (at the -120 turn the left gripper's body met the barrel): the 745 g gun tips the soft wrist several degrees."""
        for _ in range(times):
            jet = m.state(env, "gun").rotation()[:, 1]
            want = np.array([-math.sin(math.radians(20.0)), math.cos(math.radians(20.0)), 0.0])
            v = np.cross(jet, want)
            sn, cs = float(np.linalg.norm(v)), float(jet @ want)
            if sn < 1e-4:
                return
            Rc = axis_angle_matrix(v / sn, math.atan2(sn, cs))
            yield from r.move(r.at(), grip=d._as_grip(Rc @ r.grip.rot), t=0.5, label="level the jet")
            for _ in range(4):
                yield

    yield from level_jet()
    if mode == "touch":  # the nozzle's tip driven onto the sleeve's surface at mid-length
        tip = lambda: m.state(env, "gun").pos  # noqa: E731
        yield from r.servo(lambda tt: cable_at(0.5 * (xs0 + xs1)) - np.array([0.0, R_EXP, 0.0]), tol=0.001, timeout=3.0, gain=0.3, read=tip,
                           label="the nozzle onto the sleeve")
        _report(ctx, env, "touched: ")
    elif mode == "still":  # the jet held on mid-sleeve for 10 s, the cable not turned
        yield from r.servo(lambda tt: cable_at(0.5 * (xs0 + xs1)), tol=0.0015, timeout=10.0, gain=0.3, read=aim, label="the gun held at mid-sleeve")
        _report(ctx, env, "held still: ")
    else:
        yield from roll_to(-120.0, 1.2)
        for k, (a, pace) in enumerate(((-120.0, 0.6), (-72.0, 1.0), (-24.0, 1.15), (24.0, 1.15), (72.0, 1.0), (120.0, 0.6))):  # six passes, 48 degrees
            # apart; slower at the two outermost turns, whose jet alone reaches the side that faces away from the gun at the middle turns
            if k:
                yield from r.move(r.at() + np.array([0.0, -0.02, 0.05]), t=0.5, label="the jet up off the sleeve while the cable turns")  # 5 cm above its line: no dose
                yield from roll_to(a)
            yield from l.move(l.at() + np.array([JX - cable_x(LX_FROM), 0.0, 0.0]), t=1.0, label="the cable back, 3 cm past the lug end at the jet")
            yield from level_jet(1)
            # closed loop on the aim: the point the jet reaches at the working distance, onto the cable's axis where the cable really is (the gun
            # droops 2 to 4 cm in the hand and tips a few degrees); settled 7 cm further back first, where the jet gives no dose (none past 120 mm)
            yield from r.servo(lambda tt: cable_at(JX), tol=0.001, timeout=2.0, gain=0.3, read=aim_back, label="the jet in line with the cable, back")
            yield from r.servo(lambda tt: cable_at(JX), tol=0.0015, timeout=2.5, gain=0.25, read=aim, label="in to the working distance")  # closed loop: never closer
            for _ in range(6):  # the gun settles before the cable slides
                yield
            dist = LX_FROM - LX_TO
            dur = dist / (SWEEP * pace)
            p0 = l.at().copy()
            n = max(1, round(dur * 20))
            slide = (l.move(p0 + np.array([dist * (i + 1) / n, 0.0, 0.0]), t=0.05, label="slide the sleeve through the jet") for i in range(n))

            def slide_all(gens=slide):
                for g in gens:
                    yield from g

            yield from ctx.together(slide_all(), hold_aim(dur + 1.0))  # the hold outlasts the slide: the left arm trails its command
            _report(ctx, env, f"pass at {a:+.0f}: ")
        yield from r.move(r.at() + np.array([0.0, -0.02, 0.05]), t=0.5, label="the jet up off the sleeve")
        yield from roll_to(0.0, 1.2)
    # 2. the gun back in its stand; the cable back on its blocks
    def gtrace(w):
        if os.environ.get("HEAT_DEBUG"):
            gg = m.state(env, "gun")
            print(f"[heat] gun after {w}: nozzle {np.round(gg.pos - [0, 0, TOP], 4)} jet {np.round(gg.rotation()[:, 1], 3)}; on {HEAT.on}", flush=True)

    yield from r.move(rest + np.array([0.0, 0.0, 0.03]), grip=gl, t=2.0, label="the gun over its stand, the hand turned back as it took the gun")
    gtrace("the gun over its stand")
    gun_tip = lambda: m.state(env, "gun").pos  # noqa: E731
    yield from r.servo(NOZZLE_REST + np.array([0.0, 0.0, TOP + 0.02]), tol=0.002, timeout=1.5, gain=0.3, read=gun_tip, label="the gun square over its stand")
    gtrace("the gun square over its stand")
    yield from r.servo(lambda tt: NOZZLE_REST + np.array([0.0, 0.0, TOP + max(0.02 - 0.02 * tt, 0.001)]), tol=0.002, timeout=2.0, gain=0.3, read=gun_tip,
                       label="the gun down into its stand")
    gtrace("the gun down into its stand")
    yield from r.open(t=0.4, label="let go of the gun")
    gtrace("let go of the gun")
    yield from r.move(rest - 0.06 * a_g, t=0.8, label="the open hand slid back off the grip's end")
    yield from r.move(rest - 0.06 * a_g + np.array([0.0, 0.0, 0.06]), t=0.8, label="the right hand up")
    gtrace("the right hand up (its lower j")
    yield from r.move(rest - 0.06 * a_g + np.array([0.0, 0.0, 0.10]), t=0.8, label="the right hand back")
    gtrace("the right hand back")
    yield from press_switch("off")
    gtrace("off")
    yield from r.move(rest - 0.06 * a_g + np.array([0.0, -0.02, 0.10]), t=1.0, label="the right hand clear")
    gtrace("the right hand clear")
    yield from l.move(np.array([XG, CY_REST, TOP + ZW]), t=1.2, label="the cable back over its V-blocks")
    gtrace("the cable back over its V-bloc")
    yield from l.move(np.array([XG, CY_REST, TOP + Z_REST + 0.002]), t=1.2, label="the cable onto its V-blocks")
    gtrace("the cable onto its V-blocks")
    yield from l.open(t=0.4, label="let go of the cable")
    gtrace("let go of the cable")
    yield from l.move(np.array([XG - 0.08, CY_REST, TOP + Z_REST + 0.04]), t=1.0, label="the left hand back")
    gtrace("the left hand back")
    for k in range(60):
        yield
        if k % 30 == 29:
            _report(ctx, env, "end: ")
