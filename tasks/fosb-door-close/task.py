"""Close a 300 mm wafer shipping box (Entegris SB300 FOSB) in its manual's order.

The door stands on a rest in front of the box. Carry it by its two D-ring handles, set its left edge (the robot's left, +y) against the opening and swing the
right side in until the wafer cushion meets the wafers, latch the left D-ring (cam, quarter turn), press the right side flush, latch the right ring, and fold
both rings down.

Rigid bodies and sprung joints. The box (7 kg) is a free body with 25 fixed 0.775 mm wafers in its colliders; the door is a free articulation (1.0 kg) with two
D-rings of two joints each (turn, fold) and two sprung cushion pads. A latch is a rule: it engages when its D-ring is turned a quarter while its side of the
door is within 1 mm of flush, and then holds that side flush with a force (the cam's hook); it is never counted if the ring is turned while the door is not flush.

Debug:  python demokit.py task.py OUT --no-record --stills 20 --mode full
Final:  python demokit.py task.py OUT
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
INSTRUCTION = "Close the wafer shipping box. Carry its door by the two D-ring handles, set its left edge into the opening first and latch that side, then press the right side flush and latch it. Fold both D-rings down and let go."
SUCCESS_HOLD_S = 0  # the scorer holds the end state itself (20 steps)
SCORE_OVER = "end"
TOP = d.TABLE_TOP
DT = d.DT

# -- the box (manual: 385 W x 333 D x 337 H, wafers 10 mm pitch, first 44 mm above the datum) ---------------------------------------
POD_X = 0.34  # the opening's front face
PLATE = 0.020
POD_Z = TOP + PLATE
POD_W, POD_D, POD_H = 0.385, 0.333, 0.337
WIN_W, WIN_H, WIN_Z = 0.340, 0.250, 0.184  # the opening: its centre height above the datum
RIM = 0.012  # the rim's depth
WAFER_R, WAFER_T, WAFER_PITCH, WAFER_Z0, NWAF = 0.150, 0.000775, 0.010, 0.044, 25
WAFER_FRONT = 0.018  # the wafers' front edge behind the rim face, on the centre line
WAFER_CX = WAFER_FRONT + WAFER_R
PAD_Y = 0.045
PAD_FRONT = WAFER_CX - math.sqrt(WAFER_R**2 - PAD_Y**2)  # the wafer edge in front of each pad
PAD_STROKE = 0.008  # the cushion's compression at flush
PAD_LEN = PAD_FRONT + PAD_STROKE
PAD_K = 875.0  # N/m per pad: 2 x 875 x 0.008 = 14 N at flush (10 to 18 N, estimate)

# -- the door (manual: 363.7 x 258.0) ------------------------------------------------------------------------------------------
DOOR_W, DOOR_H, DOOR_T, DOOR_KG = 0.3637, 0.258, 0.025, 1.0
DOOR_ZC = TOP + PLATE + WIN_Z  # door centre height at the box
RING_Y = 0.15
RING_PROUD = 0.030
HX = -(DOOR_T + 0.006)  # the rings' hinge line, 6 mm proud of the face so that a folded ring lies on it
BAR_X = HX - RING_PROUD  # the bar's centre in the door's frame (x outward is negative)
CAM_NM = 1.0  # the cam's torque, assumed (manual: 4.4 N m maximum)
DR = 0.28  # the door's inner face on its rest
REST_H = DOOR_ZC - DOOR_H / 2 - TOP

LOOKS = {"pc": m.Look(m.rgb(0.75, 0.82, 0.85), roughness=0.15, opacity=0.35), "door": m.Look(m.rgb(0.70, 0.78, 0.82), roughness=0.15, opacity=0.5), "wafer": m.Look(m.rgb(0.35, 0.36, 0.40), metallic=0.6, roughness=0.25),
         "ring": m.Look(m.rgb(0.9, 0.35, 0.05), roughness=0.4), "pad": m.Look(m.rgb(0.1, 0.45, 0.2), roughness=0.7), "plate": m.Look(m.rgb(0.2, 0.2, 0.22), roughness=0.6), "rest": m.Look(m.rgb(0.55, 0.38, 0.22), roughness=0.7),
         "rim": m.Look(m.rgb(0.95, 0.95, 0.95), roughness=0.4)}


def _pod():
    parts, cols = [], []

    def add(name, size, c, look="pc"):
        parts.append((name, m.box(size, c), look))
        cols.append(m.col_box(name + "_c", size, c))

    hz = POD_H / 2
    add("floor", (POD_D, POD_W, 0.005), (POD_D / 2, 0, 0.0025))
    add("roof", (POD_D, POD_W, 0.005), (POD_D / 2, 0, POD_H - 0.0025))
    add("back", (0.005, POD_W, POD_H), (POD_D - 0.0025, 0, hz))
    for s in (-1, 1):
        n = "l" if s > 0 else "r"
        add(f"side_{n}", (POD_D, 0.005, POD_H), (POD_D / 2, s * (POD_W / 2 - 0.0025), hz))
        add(f"rim_{n}", (RIM, POD_W / 2 - WIN_W / 2, POD_H), (RIM / 2, s * (WIN_W / 2 + (POD_W / 2 - WIN_W / 2) / 2), hz), "rim")
    zb, zt = WIN_Z - WIN_H / 2, WIN_Z + WIN_H / 2
    add("rim_bot", (RIM, WIN_W, zb), (RIM / 2, 0, zb / 2), "rim")
    add("rim_top", (RIM, WIN_W, POD_H - zt), (RIM / 2, 0, (zt + POD_H) / 2), "rim")
    for k in range(NWAF):
        c = (WAFER_CX, 0.0, WAFER_Z0 + WAFER_PITCH * k)
        parts.append((f"w{k:02d}", m.cylinder(WAFER_R, WAFER_T, c, segments=40), "wafer"))
        cols.append(m.col_cylinder(f"w{k:02d}_c", WAFER_R, WAFER_T, c))
    return m.free("pod", (POD_X, 0.0, POD_Z), parts, cols, LOOKS, mass=7.0, friction=(1.2, 1.0), torsional_patch=None)


def _plate():
    size = (POD_D + 0.04, POD_W + 0.04, PLATE)
    c = (POD_X + POD_D / 2, 0.0, TOP + PLATE / 2)
    return m.fixed("plate", (0.0, 0.0, 0.0), [("plate", m.box(size, c), "plate")], [m.col_box("plate_c", size, c)], LOOKS, friction=(1.2, 1.0))


def _rest():
    parts, cols = [], []
    xc = DR - DOOR_T / 2
    base = ((0.09, 0.32, REST_H - 0.001), (xc, 0.0, TOP + (REST_H - 0.001) / 2))
    parts.append(("base", m.box(*base), "rest"))
    cols.append(m.col_box("base_c", *base))
    for s in (-1, 1):
        rail = ((0.008, 0.32, 0.035), (xc + s * (DOOR_T / 2 + 0.0015 + 0.004), 0.0, TOP + REST_H + 0.0175))
        n = "f" if s < 0 else "b"
        parts.append((f"rail_{n}", m.box(*rail), "rest"))
        cols.append(m.col_box(f"rail_{n}_c", *rail))
    return m.fixed("rest", (0.0, 0.0, 0.0), parts, cols, LOOKS, friction=(0.6, 0.6))


def _door():
    slab = ((DOOR_T, DOOR_W, DOOR_H), (-DOOR_T / 2, 0.0, 0.0))
    fparts = [("slab", m.box(*slab), "door")]
    fcols = [m.col_box("slab_c", *slab)]
    links = []
    for s, n in ((1, "L"), (-1, "R")):
        y = s * RING_Y
        hub = m.box((0.010, 0.018, 0.018), (HX + 0.003, y, 0.0))
        links.append(m.hinge(f"turn{n}", "X", (HX, y, 0.0), -95.0, 5.0, [(f"hub{n}", hub, "ring")], [], mass=0.04, friction=CAM_NM, damping=0.5, effort=60.0))
        legs = [m.box((0.024, 0.010, 0.010), (HX - 0.012, y, sz * 0.020)) for sz in (-1, 1)]
        bar = m.box((0.012, 0.012, 0.052), (BAR_X, y, 0.0))
        cols = [m.col_box(f"leg{n}{k}", (0.024, 0.010, 0.010), (HX - 0.012, y, sz * 0.020)) for k, sz in enumerate((-1, 1))]
        cols.append(m.col_box(f"bar{n}", (0.012, 0.012, 0.052), (BAR_X, y, 0.0)))
        links.append(m.hinge(f"fold{n}", "Z", (HX, y, 0.0), -92.0, 92.0, [(f"legs{n}", m.join(*legs), "ring"), (f"bar{n}", bar, "ring")], cols, mass=0.04, friction=0.04, damping=0.3, parent=f"turn{n}", effort=20.0))
    for s, n in ((1, "A"), (-1, "B")):
        y = s * PAD_Y
        c = (PAD_LEN / 2, y, 0.0)
        links.append(m.slider(f"pad{n}", "X", -0.020, 0.0, [(f"pad{n}", m.box((PAD_LEN, 0.020, 0.025), c), "pad")], [m.col_box(f"pad{n}_c", (PAD_LEN, 0.020, 0.025), c)], mass=0.03, spring=PAD_K, damping=6.0, friction=0.05,
                             effort=200.0))
    return m.mechanism("door", (DR, 0.0, DOOR_ZC + 0.0005), links, LOOKS, frame_parts=fparts, frame_colliders=fcols, free_mass=DOOR_KG, friction=(0.9, 0.8),
                       frame_inertia=(DOOR_KG / 12 * (DOOR_W**2 + DOOR_H**2), DOOR_KG / 12 * (DOOR_T**2 + DOOR_H**2), DOOR_KG / 12 * (DOOR_T**2 + DOOR_W**2)))


OBJECTS = [_plate(), _pod(), _rest(), _door()]

# -- the scorer ------------------------------------------------------------------------------------------------------------------
FLUSH = 0.001  # a latch engages within 1 mm
FEELER = 0.0005
HALF_W, HALF_H = DOOR_W / 2, DOOR_H / 2
TURNED = -80.0  # degrees (counter-clockwise seen from the robot is negative about +x)
FOLDED = 80.0  # degrees
HOLD_N = 12.0  # the cam's hook holds its side of the door to the rim
CLEAR = 0.03
STRIKE_YAW = 5.0  # degrees: the cushion touching the wafers with the door this far out of square


def yaw_deg(R) -> float:
    return math.degrees(math.atan2(R[1, 0], R[0, 0]))


class Trial:
    def reset(self, env=None) -> None:
        self.n = 0
        self.eng = {"L": False, "R": False}
        self.order_bad = self.struck = self.dropped = self.moved = False
        self.pod0 = None
        self.lift_n = 0
        self.still = 0
        self.score, self.full = 0.0, False
        self.flushL = self.flushR = self.flat = False
        self.gapL = self.gapR = 9.0
        self.cush = 0.0
        self.tier_peak = 0.0

    def step(self, env) -> None:
        self.n += 1
        pod, door = m.state(env, "pod"), m.state(env, "door")
        if self.pod0 is None:
            self.pod0 = pod.pos.copy()
        R = door.rotation()
        Rp = pod.rotation()
        inner = np.array([door.pos + R @ np.array([0.0, sy * HALF_W, sz * HALF_H]) for sz in (-1, 1) for sy in (1, -1)])  # L-low, R-low, L-up, R-up
        loc = (inner - pod.pos) @ Rp  # in the box's frame: x into the box from the rim face
        gx = loc[:, 0]
        self.gapL, self.gapR = float(gx[[0, 2]].mean()), float(gx[[1, 3]].mean())
        cen = (door.pos - pod.pos) @ Rp
        lateral = abs(float(cen[1])) < 0.006 and abs(float(cen[2] - WIN_Z)) < 0.012
        self.flushL = lateral and abs(self.gapL) <= FLUSH and float(np.abs(gx[[0, 2]]).max()) <= 2 * FLUSH
        self.flushR = lateral and abs(self.gapR) <= FLUSH and float(np.abs(gx[[1, 3]]).max()) <= 2 * FLUSH
        self.flat = lateral and float(gx.max()) <= FEELER and float(gx.min()) >= -FEELER
        turn = {"L": door.deg("turnL"), "R": door.deg("turnR")}
        fold = {"L": door.deg("foldL"), "R": door.deg("foldR")}
        comp = (-door.joint("padA"), -door.joint("padB"))
        self.cush = min(comp)
        yaw = yaw_deg(R) - yaw_deg(Rp)
        if max(comp) > 0.001 and abs(yaw) > STRIKE_YAW and not self.struck:
            self.struck = True
            print(f"[fosb] step {self.n}: the door met the wafers {yaw:.1f} deg out of square: 0", flush=True)
        for n, flush in (("L", self.flushL), ("R", self.flushR)):
            if not self.eng[n] and turn[n] <= TURNED and flush:
                self.eng[n] = True
                print(f"[fosb] step {self.n}: latch {n} engaged (gap {self.gapL * 1000 if n == 'L' else self.gapR * 1000:.2f} mm)", flush=True)
                if n == "R" and not self.eng["L"]:
                    self.order_bad = True
                    print("[fosb] right latch before the left: 0", flush=True)
            elif self.eng[n] and turn[n] > -10.0:
                self.eng[n] = False
        fx = [HOLD_N if self.eng[n] else 0.0 for n in "LR"]
        if sum(fx) > 0:
            y = (fx[0] * HALF_W + fx[1] * -HALF_W) / sum(fx)
            m.push(env, "door", (sum(fx), 0.0, 0.0), at=door.pos + R @ np.array([0.0, y, 0.0]))
        else:
            m.push(env, "door", (0.0, 0.0, 0.0))
        if float(door.pos[2]) < DOOR_ZC - 0.06 and self.n > 5 and abs(float(door.pos[0]) - float(pod.pos[0])) > 0.02:
            self.dropped = True
        dp = pod.pos - self.pod0
        self.lift_n = self.lift_n + 1 if float(dp[2]) > 0.005 else 0  # lifted off the plate for 0.5 s; a one-step contact pop is not a move
        if float(np.linalg.norm(dp[:2])) > 0.005 or self.lift_n >= 10:
            if not self.moved:
                print(f"[fosb] step {self.n}: the box moved {np.linalg.norm(dp) * 1000:.1f} mm: 0", flush=True)
            self.moved = True
        tips = m.fingertips(env)
        rel = (tips - door.pos) @ R
        lo, hi = np.array([BAR_X - 0.01, -HALF_W, -HALF_H]), np.array([0.0, HALF_W, HALF_H])
        out = np.maximum(np.maximum(lo - rel, rel - hi), 0.0)
        clear = float(np.linalg.norm(out, axis=1).min()) >= CLEAR
        still = float(np.linalg.norm(door.lin_vel)) < 0.01 and float(np.linalg.norm(door.ang_vel)) < 0.05
        folded = abs(fold["L"]) >= FOLDED and abs(fold["R"]) >= FOLDED
        bad = self.order_bad or self.struck or self.dropped or self.moved
        s = 0.0
        if self.eng["L"]:
            s = 0.25
        if self.eng["L"] and self.eng["R"]:
            s = 0.5
            if self.flat:
                s = 0.75
        cushion = self.cush >= 0.75 * PAD_STROKE
        self.full = bool(not bad and s >= 0.75 and folded and clear and cushion and self.eng["L"] and self.eng["R"])
        self.score = 0.0 if bad else (1.0 if (self.full and self.still >= 20) else min(s, 0.75))
        self.still = self.still + 1 if (self.full and still) else 0
        if os.environ.get("FOSB_TRACE") and self.n % int(os.environ.get('FOSB_TRACE')) == 0:
            print(f"[fosb] {self.n}: gapL {self.gapL * 1000:.1f} gapR {self.gapR * 1000:.1f} yaw {yaw:.1f} pitch {math.degrees(math.asin(float(np.clip(R[2, 0], -1, 1)))):.1f} pads {np.round(np.array(comp) * 1000, 1)} turn {np.round([turn['L'], turn['R']], 0)} fold {np.round([fold['L'], fold['R']], 0)} "
                  f"podd {np.round((pod.pos - self.pod0) * 1000, 1)} eng {self.eng} flush {self.flushL, self.flushR} lat {lateral} cen {np.round(cen[1:] * 1000, 1)} gx {np.round(gx * 1000, 1)} z {door.pos[2] - TOP:.3f} score {self.score}", flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    """Both latches engaged in order, the door flush all round on both pads, both rings folded, the hands clear, still for 1 s, and nothing struck, dropped or moved."""
    return TRIAL.full and TRIAL.still >= 20


def score(objects) -> float:
    return TRIAL.score


# -- the demo ---------------------------------------------------------------------------------------------------------------------


def Rz(a_deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(a_deg)), math.sin(math.radians(a_deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def smooth(u: float) -> float:
    u = min(max(u, 0.0), 1.0)
    return u * u * (3 - 2 * u)


class Carry:
    """Both hands hold the door by its rings; the door's own pose steers them (read each step, corrected)."""

    def __init__(self, ctx):
        self.ctx = ctx
        st = ctx.state("door")
        R = st.rotation()
        self.rel = {}
        for arm in (ctx.left, ctx.right):
            p, rot = d.ik.tcp_pose(arm.q, arm.base)
            self.rel[arm.side] = (R.T @ (p - arm.trim - st.pos), R.T @ rot)
        self.err = np.zeros(3)

    def pose(self):
        st = self.ctx.state("door")
        return np.asarray(st.pos, float), yaw_deg(st.rotation())

    def go(self, fn, t, label="carry", gain=0.15, lim=0.03):
        """fn(u) -> (door position, yaw in degrees) for u in 0..1; the hands follow it, corrected by the door's observed pose."""
        ctx = self.ctx
        n = max(1, round(t * d.CONTROL_HZ))
        for k in range(1, n + 1):
            tp, ty = fn(smooth(k / n))
            tp = np.asarray(tp, float)
            now = ctx.state("door")
            self.err = np.clip(self.err + gain * (tp - np.asarray(now.pos, float)), -np.array([lim, max(lim, 0.012), max(lim, 0.012)]), np.array([lim, max(lim, 0.012), max(lim, 0.012)]))
            for arm in (ctx.left, ctx.right):
                rp, rr = self.rel[arm.side]
                Rt = Rz(ty)
                arm._solve(tp + self.err + Rt @ rp, Rt @ rr)
            yield
        for arm in (ctx.left, ctx.right):
            arm.grip = d._as_grip(Rz(ty) @ self.rel[arm.side][1])

    def line(self, pos, yaw, t, **kw):
        p0, y0 = self.pose()
        pos = np.asarray(pos, float)
        return self.go(lambda u: (p0 + (pos - p0) * u, y0 + (yaw - y0) * u), t, **kw)

    def swing(self, yaw, px, t, **kw):
        """Turn the door about its left edge, held at x = px on the rim face."""
        y0 = self.pose()[1]
        return self.go(lambda u: (origin_for(y0 + (yaw - y0) * u, px), y0 + (yaw - y0) * u), t, **kw)


POD0 = np.array([POD_X, 0.0, POD_Z])  # the box's origin as the demo sees it, set from its observed pose at the start


def left_gap(ctx) -> float:
    """How far the door's two left inner corners stand in front of the rim face (metres, negative in front), from the observed poses."""
    door, pod = ctx.state("door"), ctx.state("pod")
    R, Rp = door.rotation(), pod.rotation()
    pts = np.array([door.pos + R @ np.array([0.0, HALF_W, sz * HALF_H]) for sz in (-1, 1)])
    return float(((pts - pod.pos) @ Rp)[:, 0].mean())


def origin_for(yaw: float, px: float) -> np.ndarray:
    """The door's origin when its left edge's inner corner line stands at x = px, on the box's centre height."""
    return np.array([px, POD0[1] + HALF_W, POD0[2] + WIN_Z + 0.0005]) - Rz(yaw) @ np.array([0.0, HALF_W, 0.0])


GRIP_N = float(os.environ.get('FOSB_GRIP', '40'))


def demo_script(ctx):
    """Modes: full; none (the arms wait); the lazy routes: rightfirst, oneside, crooked, ringsup."""
    mode = ctx.mode
    stop = os.environ.get("FOSB_STOP", "")
    l, r = ctx.left, ctx.right
    if mode == "none":
        yield from l.wait(3.0)
        return
    st = ctx.state("door")
    POD0[:] = np.asarray(ctx.state("pod").pos, float)
    R0 = st.rotation()
    bar = {s: np.asarray(st.pos) + R0 @ np.array([HX - 0.012, y, 0.0]) for s, y in (("L", RING_Y), ("R", -RING_Y))}
    g0 = d.level(0)
    back = np.array([-0.04, 0.0, 0.0])
    yield from ctx.together(l.move(bar["L"] + back, grip=g0, finger=d.FINGER_OPEN), r.move(bar["R"] + back, grip=g0, finger=d.FINGER_OPEN))
    yield from ctx.together(l.servo(bar["L"] + back, timeout=1.0), r.servo(bar["R"] + back, timeout=1.0))
    yield from ctx.together(l.move(bar["L"], t=1.0), r.move(bar["R"], t=1.0))
    yield from ctx.together(l.servo(bar["L"], timeout=1.0), r.servo(bar["R"], timeout=1.0))
    yield from ctx.together(l.close(0.012, force=GRIP_N), r.close(0.012, force=GRIP_N))
    c = Carry(ctx)
    p = c.pose()[0]
    yield from c.line(p + [0.0, 0.0, 0.045], 0.0, 1.0, label="lift")
    yield from c.line(origin_for(-10.0, POD0[0] - 0.012) + [0.0, 0.0, 0.045], -10.0, 1.8, label="carry out")
    yield from c.line(origin_for(-10.0, POD0[0] - 0.012), -10.0, 1.0, label="lower")
    print(f"[fosb] at the box: door {np.round(c.pose()[0], 3)}, gaps L {TRIAL.gapL * 1000:.1f} R {TRIAL.gapR * 1000:.1f} mm", flush=True)
    if stop == "carry":
        yield from l.wait(1.0)
        return
    yield from c.line(origin_for(-10.0, POD0[0] + 0.001), -10.0, 1.2, label="left edge in", lim=0.004)
    yield from l.wait(0.5)
    print(f"[fosb] left edge in: gaps L {TRIAL.gapL * 1000:.1f} R {TRIAL.gapR * 1000:.1f} mm, flushL {TRIAL.flushL}", flush=True)
    if stop == "seat":
        yield from l.wait(1.0)
        return
    yield from c.swing(-4.0 if mode == "crooked" else 1.0, POD0[0] + 0.001, 2.5, label="swing in", lim=0.004)
    yield from l.wait(0.5)
    if mode != "crooked":  # seat the left edge by feedback: push it in until its inner corners read at the rim (the box's pose and the door's, observed)
        px = POD0[0] + 0.001
        for _ in range(12):
            if left_gap(ctx) >= -0.0003:
                break
            px += 0.0007
            yield from c.go(lambda u, px=px: (origin_for(1.0, px), 1.0), 0.3, label="seat", lim=0.008)
        yield from l.wait(0.3)
    print(f"[fosb] swung: gaps L {TRIAL.gapL * 1000:.1f} R {TRIAL.gapR * 1000:.1f} mm, pads {TRIAL.cush * 1000:.1f}, flushL {TRIAL.flushL}", flush=True)
    if stop == "swing":
        return

    def latch_left():
        # the left hand turns its ring a quarter, the right hand holds the far side
        yield from l.turn(-106.0, axis=(1.0, 0.0, 0.0), t=2.5, label="turn left ring")
        yield from l.wait(0.5)
        print(f"[fosb] left ring {ctx.deg('door', 'turnL'):.0f} deg, latch L {TRIAL.eng['L']}", flush=True)

    def latch_right():
        # press the far side flush, then turn the right ring
        yield from r.move(r.at() + [0.005, 0.0, 0.0], t=1.0, label="press right side")
        yield from l.wait(0.5)
        print(f"[fosb] pressed: gaps L {TRIAL.gapL * 1000:.2f} R {TRIAL.gapR * 1000:.2f} mm, flushR {TRIAL.flushR}", flush=True)
        yield from r.turn(-106.0, axis=(1.0, 0.0, 0.0), t=2.5, label="turn right ring")
        yield from l.wait(0.5)
        print(f"[fosb] right ring {ctx.deg('door', 'turnR'):.0f} deg, latch R {TRIAL.eng['R']}, score {TRIAL.score}", flush=True)

    if mode == "rightfirst":  # lazy: the far side first
        yield from latch_right()
        yield from latch_left()
    elif mode == "oneside":  # lazy: one latch only
        yield from latch_left()
    elif mode == "crooked":  # lazy: the door left cocked, the far side not pressed in
        yield from latch_left()
        yield from r.turn(-106.0, axis=(1.0, 0.0, 0.0), t=2.5, label="turn right ring")
    else:
        yield from latch_left()
        if stop == "latchL":
            return
        yield from latch_right()
    if stop == "latchR":
        return
    yield from ctx.together(l.open(), r.open())
    yield from ctx.together(l.move(l.at() + [-0.07, 0.0, 0.0]), r.move(r.at() + [-0.07, 0.0, 0.0]))
    if stop == "release":
        yield from l.wait(1.0)
        return
    # fold both rings down: closed fingers on top of each bar, swept down along its arc
    dst = ctx.state("door")
    Rd = dst.rotation()

    def bar_at(side_y, phi_deg):
        phi = math.radians(phi_deg)
        hinge = np.array([HX, side_y, 0.0])
        return np.asarray(dst.pos) + Rd @ (hinge + RING_PROUD * np.array([-math.cos(phi), 0.0, -math.sin(phi)]))

    def fold_arm(arm, y):
        top = np.array([-0.018, 0.0, 0.010])
        yield from arm.move(bar_at(y, 0.0) + top + [-0.04, 0.0, 0.03], grip=g0, finger=d.FINGER_CLOSED, t=1.5)
        yield from arm.move(bar_at(y, 0.0) + top, t=1.0)
        n = 40
        for k in range(1, n + 1):
            phi = 95.0 * smooth(k / n)
            arm._solve(bar_at(y, phi) + top, arm.grip.rot)
            yield
        yield from arm.wait(0.5)
        yield from arm.move(arm.at() + [-0.08, 0.0, 0.05], t=1.0)

    if mode == "ringsup":  # lazy: leave the rings up
        yield from l.wait(1.5)
        return
    yield from ctx.together(fold_arm(l, RING_Y), fold_arm(r, -RING_Y))
    print(f"[fosb] folded: L {ctx.deg('door', 'foldL'):.0f} R {ctx.deg('door', 'foldR'):.0f}, score {TRIAL.score}", flush=True)
    yield from l.wait(1.5)
