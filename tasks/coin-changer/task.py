"""Give exact change from a belt-style four-barrel coin changer into a coin tray.

A four-barrel steel coin changer (quarters, dimes, nickels, pennies, left to right) stands on a stand at belt height, chute toward the +x wall. Each barrel holds a stack of
coins over a floor slot just a little taller than one coin; its thumb tab pushes a spring-return pusher plate along the floor that slides the bottom coin out of the
slot, one coin per full push, and the coin falls off the changer's lip. A shallow coin tray lies on the table at the left. "Give $1.35 change into the tray."

Run it:  python demokit.py task.py OUT --mode full
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import mechkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Give the customer $1.35 change from the coin changer into the tray."
SUCCESS_HOLD_S = 0.0
HOLD_STEPS = 20
TOP = TABLE_TOP[SCENE]

LOOKS = {
    "steel": m.Look(color=m.rgb(0.70, 0.71, 0.73), metallic=0.85, roughness=0.3),
    "dark": m.Look(color=m.rgb(0.16, 0.17, 0.19), metallic=0.6, roughness=0.4),
    "tab": m.Look(color=m.rgb(0.55, 0.57, 0.60), metallic=0.8, roughness=0.35),
    "silver": m.Look(color=m.rgb(0.82, 0.82, 0.84), metallic=0.9, roughness=0.25),
    "copper": m.Look(color=m.rgb(0.72, 0.42, 0.28), metallic=0.8, roughness=0.3),
    "nickel": m.Look(color=m.rgb(0.62, 0.64, 0.66), metallic=0.9, roughness=0.3),
    "tray": m.Look(color=m.rgb(0.20, 0.45, 0.80), roughness=0.5),
    "wood": m.Look(color=m.rgb(0.72, 0.58, 0.40), roughness=0.7),
}

# -- the changer (metres, station frame: x forward, y to the robot's left) --------------------------
BX, BY = 0.32, 0.0  # barrel centre line x / the changer's centre line y
PITCH = 0.040  # barrel spacing along y: to measure on a real changer
STAND = float(os.environ.get("STAND", "0.10"))  # the base's underside over the table: the changer hangs at belt height
BASE_T = 0.006
TUBE_H = 0.050  # tube height over the floor
WALL = 0.003
LIP = 0.030  # the base plate's front edge, from the barrels' centre line: a coin pushed past it falls off
TRAVEL = 0.036
COIN_MU = float(os.environ.get("COIN_MU", "0.25"))
SM = float(os.environ.get("SM", "0.0003"))  # slot height over the coin's thickness
PUSH_K = float(os.environ.get("PUSH_K", "60"))
PUSH_F = float(os.environ.get("PUSH_F", "0.05"))
PUSH_D = float(os.environ.get("PUSH_D", "0.5"))
TAB_W = float(os.environ.get("TAB_W", "0.014"))  # return spring N/m, sliding friction N, damping N s/m

# barrel: name, coin radius, thickness, mass (US Mint specs), look, slot height over the floor, start count, target in the tray
COINS = (
    ("quarter", 0.01213, 0.00175, 0.00567, "silver", None, 8, 5),
    ("dime", 0.00895, 0.00135, 0.00227, "silver", None, 8, 1),
    ("nickel", 0.01060, 0.00195, 0.00500, "nickel", None, 8, 0),
    ("penny", 0.00950, 0.00152, 0.00250, "copper", None, 8, 0),
)
N_BARREL = len(COINS)
BARREL_Y = [(k - (N_BARREL - 1) / 2) * PITCH for k in range(N_BARREL)]  # barrel 0 (quarters) at the robot right, -y: the right hand presses

# -- the tray -----------------------------------------------------------------------------------
TRAY_IN = (0.100, 0.170)  # inside: x depth, y length
TRAY_RIM_H, TRAY_FLOOR, TRAY_WALL = 0.022, 0.003, 0.003
TRAY_MASS = 0.07
TRAY_START = (0.19, 0.20)  # on the table at the left
TRAY_UNDER = (0.395, 0.0)  # where it is held under the chute
TRAY_HOLD_Z = 0.060  # the tray's floor over the table while it is held


def _changer():
    zf = STAND + BASE_T
    fp, fc = [], []

    def add(name, size, center, look="steel"):
        fp.append((name, m.box(size, center), look))
        fc.append(m.col_box(f"{name}_c", size, center))

    add("base", (0.150, 0.190, BASE_T), (-0.045, 0.0, STAND + BASE_T / 2), "dark")  # x from -0.12 to +0.03
    for i, sx in enumerate((-0.10, 0.003)):
        for j, sy in enumerate((-0.08, 0.08)):
            add(f"leg{i}{j}", (0.014, 0.014, STAND), (sx, sy, STAND / 2), "dark")
    links = []
    for k, (cname, cr, ct, cmass, look, slot, n0, tgt) in enumerate(COINS):
        slot = ct + SM
        y = BARREL_Y[k]
        inner = 2 * cr + 0.0016
        hx = inner / 2 + WALL / 2
        add(f"wl{k}", (inner + 2 * WALL, WALL, TUBE_H), (0.0, y + hx, zf + TUBE_H / 2))
        add(f"wr{k}", (inner + 2 * WALL, WALL, TUBE_H), (0.0, y - hx, zf + TUBE_H / 2))
        add(f"wb{k}", (WALL, inner, TUBE_H - slot), (-hx, y, zf + slot + (TUBE_H - slot) / 2))
        add(f"wf{k}", (WALL, inner, TUBE_H - slot), (hx, y, zf + slot + (TUBE_H - slot) / 2))
        PT = 0.0010  # the pusher plate: thinner than the thinnest coin, so it never touches the second coin
        zp = zf + PT / 2
        face = -(cr + 0.0007)  # the pusher plate's front edge, 0.7 mm behind the bottom coin
        pl = 0.030
        pusher = m.box((pl, 0.014, PT), (face - pl / 2, y, zp))
        tail = m.box((0.050, 0.006, PT), (face - pl - 0.025, y, zp))
        tab = m.box((0.010, TAB_W, 0.016), (face - pl - 0.050 - 0.005, y, zf + 0.008))
        links.append(m.slider(f"push{k}", "X", 0.0, TRAVEL,
                              [(f"pusher{k}", pusher, "dark"), (f"tail{k}", tail, "dark"), (f"tab{k}", tab, "tab")],
                              [m.col_box(f"pusher{k}_c", (pl, 0.014, PT), (face - pl / 2, y, zp)),
                               m.col_box(f"tail{k}_c", (0.050, 0.006, PT), (face - pl - 0.025, y, zp)),
                               m.col_box(f"tab{k}_c", (0.010, TAB_W, 0.016), (face - pl - 0.050 - 0.005, y, zf + 0.008))],
                              at=(0.0, y, 0.0), mass=0.03, spring=PUSH_K, damping=PUSH_D, friction=PUSH_F))
    return m.mechanism("changer", (BX, BY, TOP), links, LOOKS, frame_parts=fp, frame_colliders=fc, friction=(0.4, 0.3))


def tab_x(k: int) -> float:
    """World x of barrel k's thumb tab, centre, at rest."""
    cr = COINS[k][1]
    return BX - (cr + 0.0007) - 0.030 - 0.050 - 0.005


def _coin(k: int, i: int) -> m.Part:
    cname, cr, ct, cmass, look, slot, n0, tgt = COINS[k]
    z = STAND + BASE_T + 0.0001 + ct / 2 + i * (ct + 0.0001)
    return m.free(f"{cname}{i}", (BX, BY + BARREL_Y[k], TOP + z), [("c", m.cylinder(cr, ct, (0, 0, 0), segments=32), look)], [m.col_cylinder("c_c", cr, ct)], LOOKS,
                  mass=cmass, friction=(COIN_MU, COIN_MU * 0.8))


def _tray() -> m.Part:
    ix, iy = TRAY_IN
    parts, cols = [], []

    def add(name, size, center, look="tray"):
        parts.append((name, m.box(size, center), look))
        cols.append(m.col_box(f"{name}_c", size, center))

    add("floor", (ix + 2 * TRAY_WALL, iy + 2 * TRAY_WALL, TRAY_FLOOR), (0.0, 0.0, TRAY_FLOOR / 2))
    zc = TRAY_FLOOR + TRAY_RIM_H / 2
    add("rim_n", (ix + 2 * TRAY_WALL, TRAY_WALL, TRAY_RIM_H), (0.0, iy / 2 + TRAY_WALL / 2, zc))
    add("rim_s", (ix + 2 * TRAY_WALL, TRAY_WALL, TRAY_RIM_H), (0.0, -iy / 2 - TRAY_WALL / 2, zc))
    add("rim_e", (TRAY_WALL, iy, TRAY_RIM_H), (ix / 2 + TRAY_WALL / 2, 0.0, zc))
    add("rim_w", (TRAY_WALL, iy, TRAY_RIM_H), (-ix / 2 - TRAY_WALL / 2, 0.0, zc))
    return m.free("tray", (TRAY_START[0], TRAY_START[1], TOP + 0.001), parts, cols, LOOKS, mass=TRAY_MASS, friction=(0.7, 0.6), solver_iters=48)


OBJECTS = [_changer(), _tray()] + [_coin(k, i) for k, c in enumerate(COINS) for i in range(c[6])]
COIN_NAMES = [(k, f"{c[0]}{i}") for k, c in enumerate(COINS) for i in range(c[6])]


# -- the scorer -----------------------------------------------------------------------------------

def in_tray(tray, p) -> bool:
    q = tray.to_local(np.asarray(p, float))[0]
    return abs(q[0]) <= TRAY_IN[0] / 2 and abs(q[1]) <= TRAY_IN[1] / 2 and TRAY_FLOOR - 0.002 <= q[2] <= TRAY_FLOOR + TRAY_RIM_H + 0.004


def in_changer(p) -> bool:
    return p[0] <= BX + LIP + 0.002 and p[2] >= TOP + STAND - 0.001 and abs(p[1] - BY) <= 0.12


def tally(tray, coin_pos) -> dict:
    inside = [0] * N_BARREL
    loose = 0
    for (k, _), p in zip(COIN_NAMES, coin_pos):
        if in_tray(tray, p):
            inside[k] += 1
        elif not in_changer(p):
            loose += 1
    return {"tray": inside, "loose": loose}


def grade(t: dict, tray_tilt: float = 0.0) -> float:
    want = [c[7] for c in COINS]
    have = t["tray"]
    if tray_tilt > 20.0:
        return 0.0
    if have == want and t["loose"] == 0:
        return 1.0
    if all(h >= w for h, w in zip(have, want)):
        return 0.5
    return 0.25 if sum(have) > 0 else 0.0


class Trial:
    def __init__(self) -> None:
        self.reset()

    def reset(self, env=None) -> None:
        self.steps, self.hold, self.score, self.said = 0, 0, 0.0, False
        self.report: dict = {}

    def step(self, env) -> None:
        self.steps += 1
        if getattr(self, "trace", False) and self.steps % 6 == 0:
            print("[trace]", self.steps, [round(float(m.state(env, 'changer').joint('push0')), 4)] + [[round(float(m.state(env, f'quarter{i}').pos[0] - BX), 4), round(float(m.state(env, f'quarter{i}').pos[2] - TOP - STAND - BASE_T), 4)] for i in range(3)], flush=True)
        tray = m.state(env, "tray")
        pos = [m.state(env, n).pos for _, n in COIN_NAMES]
        t = tally(tray, pos)
        g = grade(t, tray.tilt_deg())
        self.hold = self.hold + 1 if g == 1.0 else 0
        self.score = 1.0 if self.hold >= HOLD_STEPS else min(g, 0.5)
        pk = [float(m.state(env, "changer").joint(f"push{k}")) for k in range(N_BARREL)]
        self.peak = [max(a, b) for a, b in zip(getattr(self, "peak", [0.0] * N_BARREL), pk)]
        self.report = {"peak": [round(v, 4) for v in self.peak], "step": self.steps, "score": self.score, "grade": g, **t, "tilt": round(tray.tilt_deg(), 1),
                       "push": [round(float(m.state(env, "changer").joint(f"push{k}")), 4) for k in range(N_BARREL)],
                       "q": [[round(float(v), 4) for v in (m.state(env, f"quarter{i}").pos[0] - BX, m.state(env, f"quarter{i}").pos[2] - TOP - STAND - BASE_T)] for i in range(4)]}


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.score == 1.0


def score(objects) -> float:
    return TRIAL.score


# -- the demo -------------------------------------------------------------------------------------

def _press(ctx, r, d, k, g):
    z_tab = TOP + STAND + BASE_T + 0.008 + 0.012
    tab = np.array([tab_x(k) - 0.014, BARREL_Y[k] + BY, z_tab])
    yield from r.move(tab + np.array([0.0, 0.0, 0.05]), grip=g, t=1.0, label=f"above lever {k}")
    yield from r.close(None, t=0.3, label="close")
    yield from r.move(tab, grip=g, t=1.0, label="to the tab")
    yield from r.servo(tab, timeout=1.0, label="at the tab")
    yield from r.move(tab + np.array([TRAVEL + 0.014 + 0.003, 0.0, 0.0]), t=1.2, label=f"press lever {k}")
    yield from r.move(tab + np.array([0.0, 0.0, 0.06]), t=1.0, label="lift off")  # off the tab, so the pusher springs back
    yield from ctx.until(lambda: float(ctx.joint("changer", f"push{k}")) < 0.001, timeout=4.0, label=f"lever {k} returns")
    print("[coin] press peak", [round(v, 4) for v in TRIAL.peak], flush=True)
    TRIAL.peak = [0.0] * N_BARREL
    yield from r.wait(0.4)


def demo_script(ctx):
    import demokit as d

    l, r = ctx.left, ctx.right
    yaw_l = float(os.environ.get("YAW_L", "0"))
    gl, gr = d.down(yaw_l), d.down(0.0)
    for _ in range(10):
        yield
    if ctx.mode == "none":
        for _ in range(100):
            yield
        print(f"[coin] final: {json.dumps(TRIAL.report)}", flush=True)
        return
    if ctx.mode == "notray":  # a failure check: the levers pressed with no tray under the chute, so the coins fall on the table
        seq0 = [0, 0, 0, 0, 0, 1]
        for k in seq0:
            yield from _press(ctx, r, d, k, gr)
        yield from r.move(np.array([0.20, -0.20, TOP + 0.20]), grip=d.down(0.0), t=2.0, label="park")
        yield from r.wait(1.5)
        print(f"[coin] final: {json.dumps(TRIAL.report)}", flush=True)
        return
    # the left hand takes the tray by its long rim on the left and holds it under the chute
    tp = ctx.pos("tray")
    RIM_OFF = np.array([0.0, TRAY_IN[1] / 2 + TRAY_WALL / 2, 0.0])  # the long rim on the left (+y): within the left arm's reach
    rim = tp + RIM_OFF + np.array([0.0, 0.0, TRAY_FLOOR + TRAY_RIM_H - 0.001 + 0.013 - float(os.environ.get("BITE", "0.008"))])
    yield from l.move(rim + np.array([0.0, 0.0, 0.08]), grip=gl, t=2.5, label="over the tray's far rim")
    yield from l.open(t=0.4, label="open")
    yield from l.move(rim, grip=gl, t=1.0, label="down on the rim")
    yield from l.servo(rim, timeout=1.0, label="down on the rim")
    yield from l.close(None, label="grip the rim")
    yield from l.move(rim + np.array([0.0, 0.0, 0.07]), t=1.0, label="lift the tray")
    print(f"[coin] tray lifted: z {ctx.pos('tray')[2] - TOP:.3f} tilt {ctx.state('tray').tilt_deg():.1f}", flush=True)
    hold = np.array([TRAY_UNDER[0], TRAY_UNDER[1] + RIM_OFF[1], TOP + TRAY_HOLD_Z + TRAY_FLOOR + TRAY_RIM_H - 0.001 + 0.013 - float(os.environ.get("BITE", "0.008"))])
    wp = np.array([hold[0], rim[1], rim[2] + 0.07])  # round the changer, on its left, then in under the chute
    yield from l.move(wp, t=3.0, label="tray round the changer")
    yield from l.move(np.array([hold[0], hold[1], wp[2]]), t=2.0, label="tray in front of the chute")
    yield from l.move(hold, t=1.5, label="tray under the chute")
    yield from l.servo(hold, timeout=1.0, label="tray under the chute")
    print(f"[coin] tray held: {np.round(ctx.pos('tray'), 3)} tilt {ctx.state('tray').tilt_deg():.1f}", flush=True)
    def in_tray_count(k):
        states = ctx.states()
        return t_count(states, k)

    def t_count(states, k):
        tr = states["tray"]
        return sum(1 for kk, n in COIN_NAMES if kk == k and in_tray(tr, states[n].pos))

    finished = {"done": False}

    def hold():
        while not finished["done"]:
            yield from l.wait(0.1)

    def presses():
        # press a barrel's tab until its target count is in the tray (a press that stalls leaves its coin half out; the next one finishes it)
        for k, c in enumerate(COINS):
            tries = 0
            while t_count(ctx.states(), k) < c[7] and tries < 9:
                yield from _press(ctx, r, d, k, gr)
                tries += 1
                for _ in range(10):  # let the coin settle in the tray
                    yield
            print(f"[coin] barrel {k}: {t_count(ctx.states(), k)} in the tray after {tries} presses: {json.dumps({k_: TRIAL.report[k_] for k_ in ('step', 'tray', 'loose', 'tilt')})}", flush=True)
        yield from r.move(np.array([0.20, -0.20, TOP + 0.20]), grip=d.down(0.0), t=2.0, label="park")
        finished["done"] = True

    yield from ctx.together(presses(), hold())
    # the tray down on the table, clear of the changer, and let go
    tray_set = np.array([0.30, 0.20 + RIM_OFF[1], TOP + TRAY_FLOOR + TRAY_RIM_H - 0.001 + 0.013 - float(os.environ.get("BITE", "0.008")) + 0.004])
    yield from l.move(tray_set + np.array([0.0, 0.0, 0.10]), t=3.0, label="carry the tray away")
    yield from l.move(tray_set + np.array([TRAY_IN[0] / 2 + TRAY_WALL / 2 - 0.0, 0.0, 0.0]) * 0, t=1.0, label="down")
    yield from l.open(t=0.5, label="let go")
    yield from l.move(tray_set + np.array([0.0, 0.0, 0.12]), t=1.0, label="clear")
    yield from l.wait(2.0)
    print(f"[coin] final: {json.dumps(TRIAL.report)}", flush=True)
