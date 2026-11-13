"""Open a cocktail shaker after shaking: hold the tins, strike the seam where the small tin leans away, lift it off.

A tin-on-tin shaker stands on a bar mat, shaken and sealed: a 28 oz tin with the drink in it and an 18 oz tin pushed into it mouth-down at a slight
lean. A straight pull does not part them. One hand holds the big tin; the other strikes the big tin's rim at the seam, on the side where the small tin
leans away and the gap is wider; the seal breaks, and the small tin is lifted off and stood on the mat beside the big one.

Sim (stated, all in code): the vacuum is two latches on the big tin, at opposite sides, over the small tin's rolled rim, held locked; they release when
a fingertip of a hand not holding a tin strikes the big tin's rim within 45 degrees of the open side, moving at 0.15 m/s or more toward the tin
relative to it and reaching its wall that step or the next. A strike elsewhere or slower does nothing; 1.0 m/s or more, at any time, is a dent. The drink is rigid mass in the big tin; no liquid.

    python demokit.py task.py OUT --mode scene --stills 1
    python demokit.py task.py OUT --mode full
    python check_score.py
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = ("The drink is shaken. Open the shaker: hold the big tin, strike the big tin's rim at the seam on the side where the small tin leans "
               "away, lift the small tin off, set both tins upright on the mat, let go of them and move your hands clear.")
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0
HOLD_STEPS = 20

MAT_C, MAT_SIZE = np.array([0.28, -0.04]), (0.30, 0.42, 0.01)
MAT_TOP = TOP + MAT_SIZE[2]
SH = np.array([0.28, 0.02])  # the shaker's axis
H1, R1_BOT, R1_TOP, WALL = 0.175, 0.037, 0.046, 0.0012  # the 28 oz tin: height, outer radius at base and mouth, wall
H2, R2_MOUTH, R2_BEAD, R2_TOP = 0.150, 0.0405, 0.0425, 0.034  # the 18 oz tin, mouth down: outer radius at its mouth, its rolled rim, and at its base (on top)
Z_M = 0.146  # the small tin's mouth above the big tin's base: where it wedges
LEAN = -2.5  # degrees about x: the small tin's top leans toward +y, so the seam's gap is wider on the -y side
V_OPEN, V_DENT = 0.15, 1.0  # m/s toward the tin, a fingertip at the rim on the open side
CONTACT = 0.005  # a finger point this close to the big tin's wall: the finger's surface touching it
SMALL_REST = np.array([0.30, -0.14])  # where the demo stands the small tin
BIG_MASS, SMALL_MASS = 0.322 + 0.15, 0.15  # kg: the weighted 28 oz tin (a retailer's listing) with the drink as rigid mass; the 18 oz tin
LATCH_T, LATCH_BACK = 0.002, 0.0015  # each latch 2 mm thick, reaching 1 mm over the rolled rim; opened, it draws back 1.5 mm into the wall, its outer
# face then 0.3 mm proud of the wall's outer surface and its inner face 0.5 mm clear of the rim
LATCHES = (("latch", 1.0, 0.0, LATCH_BACK), ("latch_b", -1.0, -LATCH_BACK, 0.0))  # name, side (+x or -x), travel

LOOKS = {"tin": m.Look(color=m.rgb(0.82, 0.83, 0.85), metallic=1.0, roughness=0.2), "mat": m.Look(color=m.rgb(0.06, 0.06, 0.07), roughness=0.95),
         "latch": m.Look(color=m.rgb(0.70, 0.71, 0.73), metallic=0.8, roughness=0.3)}


def r1(z):
    return R1_BOT + (R1_TOP - R1_BOT) * z / H1


def r2(z):
    return R2_MOUTH + (R2_TOP - R2_MOUTH) * (z - 0.002) / (H2 - 0.002)


def _hex(p):
    """A convex block from 8 corner points (4 at the bottom, then 4 at the top, in order round)."""
    p = np.asarray(p, float)
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7)]
    for i in range(4):
        j = (i + 1) % 4
        f += [(i, j, 4 + j), (i, 4 + j, 4 + i)]
    return m.Mesh(p, np.array(f))


def staves(name, rz_out, z0, z1, wall, n=16):
    """Collider staves round the z axis: the shell between rz_out(z) - wall and rz_out(z), from z0 to z1."""
    cols = []
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        pts = []
        for z in (z0, z1):
            ro, ri = rz_out(z), rz_out(z) - wall
            pts += [(ri * math.cos(a0), ri * math.sin(a0), z), (ro * math.cos(a0), ro * math.sin(a0), z),
                    (ro * math.cos(a1), ro * math.sin(a1), z), (ri * math.cos(a1), ri * math.sin(a1), z)]
        cols.append(m.col_hull(f"{name}_{k}", _hex(pts)))
    return cols


def _big() -> m.Part:
    """The 28 oz tin with the drink, a free articulation: its frame, and two latches (the seal) on slides across its wall at +x and -x."""
    shell = m.lathe([(0.0, 0.0), (R1_BOT, 0.0), (R1_TOP, H1), (R1_TOP + 0.0015, H1 + 0.0015), (R1_TOP - WALL, H1 + 0.002),
                     (r1(0.004) - WALL, 0.004), (0.0, 0.004)], segments=48)
    frame = [("shell", shell, "tin")]
    cols = staves("big", r1, 0.003, H1, 0.0015) + [m.col_cylinder("big_base", R1_BOT, 0.004, (0.0, 0.0, 0.002))]  # 1.5 mm: the small tin's rim sits 0.5 mm inside it at the wedge
    zl = Z_M + 0.0045  # just over the small tin's rolled rim
    ri = r1(zl) - WALL
    latch_box = (LATCH_T, 0.010, 0.004)  # from the wall's inner face 2 mm in: 1 mm over the rolled rim, clear of the small tin's wall above it
    lx = ri - latch_box[0] / 2
    latches = [m.slider(nm, "X", lo, hi, [(nm, m.box(latch_box, (sg * lx, 0.0, zl)), "latch")], [m.col_box(f"{nm}_c", latch_box, (sg * lx, 0.0, zl))],
                        mass=0.005, spring=300.0, damping=2.0, friction=0.0, effort=20.0) for nm, sg, lo, hi in LATCHES]
    return m.mechanism("big", (SH[0], SH[1], MAT_TOP + 0.0005), latches, LOOKS, frame_parts=frame, frame_colliders=cols, free_mass=BIG_MASS,
                       friction=(0.8, 0.7), frame_inertia=(1.4e-3, 1.4e-3, 4.2e-4))


def _small() -> m.Part:
    """The 18 oz tin, mouth down, its origin at its mouth's middle; leaning LEAN about x."""
    shell = m.lathe([(R2_BEAD, 0.0), (R2_BEAD, 0.002), (R2_MOUTH, 0.002), (R2_TOP, H2), (0.0, H2)], segments=48)
    cols = staves("small", r2, 0.002, H2 - 0.003, WALL * 3) + staves("bead", lambda z: R2_BEAD, 0.0, 0.002, 0.003)
    cols.append(m.col_cylinder("small_top", R2_TOP, 0.003, (0.0, 0.0, H2 - 0.0015)))
    a = math.radians(LEAN) / 2
    quat = (math.sin(a), 0.0, 0.0, math.cos(a))
    return m.free("small", (SH[0], SH[1], MAT_TOP + 0.0005 + Z_M), [("shell", shell, "tin")], cols, LOOKS, mass=SMALL_MASS, friction=(0.6, 0.5), start_quat=quat)


def _mat() -> m.Part:
    return m.fixed("mat", (0.0, 0.0, 0.0), [("mat", m.box(MAT_SIZE, (MAT_C[0], MAT_C[1], TOP + MAT_SIZE[2] / 2)), "mat")],
                   [m.col_box("mat_c", MAT_SIZE, (MAT_C[0], MAT_C[1], TOP + MAT_SIZE[2] / 2))], LOOKS, friction=(1.0, 0.9))


OBJECTS = [_mat(), _big(), _small()]


# -- the seal and the scorer ----------------------------------------------------------------------------------------------------------------------

def tilt_deg(R) -> float:
    return math.degrees(math.acos(float(np.clip(R[2, 2], -1.0, 1.0))))


def rim_blow(q, v, gap_dir):
    """A fingertip's blow at the big tin's rim, all in the big tin's frame: q the fingertip, v its velocity relative to the tin, gap_dir the open
    side (unit, level). None unless the fingertip is at the big tin's rim (from 1 mm inside its outer radius to 12 mm out, from 15 mm below its top
    to 2 mm above it: the big tin only, not the small tin above it); else its speed toward the tin's axis (m/s) and its angle round from the open
    side (degrees)."""
    rad = math.hypot(q[0], q[1])
    if not (R1_TOP - 0.001 <= rad <= R1_TOP + 0.012 and H1 - 0.015 <= q[2] <= H1 + 0.002):
        return None
    rhat = np.array([q[0], q[1], 0.0]) / max(rad, 1e-6)
    return -float(np.asarray(v) @ rhat), math.degrees(math.acos(float(np.clip(rhat @ gap_dir, -1.0, 1.0))))


def wall_gap(q) -> float:
    """How far a point (in the big tin's frame) is from the big tin's outer wall, in metres (negative: inside it). Above the rim (more than 2 mm over
    it) the distance runs to the rim's top edge, so a point over the gap between the tins is as far from the wall as it is above the rim."""
    dr = math.hypot(q[0], q[1]) - r1(float(np.clip(q[2], 0.0, H1)))
    dz = float(q[2]) - (H1 + 0.002)
    return math.hypot(max(dr, 0.0), dz) if dz > 0.0 else dr


def contact_blow(pending, blow, gap):
    """A blow needs contact: the fingertip moving in fast enough at the rim (blow: speed, angle, from rim_blow) must reach the wall at that step or the
    next: the finger point within 5 mm of the wall (the points run along the finger's middle, 3.5 to 4.5 mm inside its surface, so the finger itself
    is within about 1 mm). pending: the fast approach carried from the last step, or None. Returns (the blow that landed or None, what to carry)."""
    fast = blow if blow is not None and blow[0] >= V_OPEN else None
    if gap <= CONTACT:
        landed = max([b for b in (pending, fast) if b is not None], key=lambda b: b[0], default=None)
        return landed, None
    return None, fast


def gripping(tips_local, radius, z0, z1):
    """Whether one hand holds a tin between its fingers: tips_local, the hand's 12 fingertip points in the tin's frame (6 on each finger); radius(z),
    the tin's outer radius; z0 to z1, its height. Both fingers within 12 mm of its wall and more than 3 cm apart, so the tin is between them (a closed
    hand striking the rim has its fingers together, and does not count)."""
    def wall(pts):
        z = np.clip(pts[:, 2], z0, z1)
        d = np.abs(np.hypot(pts[:, 0], pts[:, 1]) - np.array([radius(v) for v in z]))
        d[(pts[:, 2] < z0 - 0.005) | (pts[:, 2] > z1 + 0.005)] = np.inf
        return float(d.min())
    a, b = tips_local[:6], tips_local[6:]
    return bool(wall(a) < 0.012 and wall(b) < 0.012 and np.linalg.norm(a.mean(0) - b.mean(0)) > 0.03)


class Seal:
    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.steps, self.locked, self.broken, self.by, self.dent, self.spill, self.fell = 0, False, False, None, False, False, False
        self.prev, self.vmax, self.strikes, self.hold, self.full, self.last, self.held_by = None, (0.0, 0.0), [], 0, False, {}, [False, False]
        self.pending, self.closest = {}, 1.0
        if env is not None:
            for nm, *_ in LATCHES:
                try:
                    m.set_visible(f"{m.ENV_PATH}/big/{nm}/visual/{nm}", True)
                except Exception:
                    pass

    def step(self, env):
        self.steps += 1
        dt = float(env.unwrapped.step_dt)
        if not self.locked and not self.broken:
            for nm, *_ in LATCHES:
                m.lock(env, "big", nm)  # the seal: both latches held where they are, against any pull or tilt a gripper can make
            self.locked = True
        b, s = m.state(env, "big"), m.state(env, "small")
        Rb, Rs = b.rotation(), s.rotation()
        rim = b.to_world(np.array([[0.0, 0.0, H1]]))[0]
        # the open side: where the small tin's wall is farther from the big tin's rim
        up_s = Rs[:, 2]
        lean_b = Rb.T @ up_s
        gap_dir = -np.array([lean_b[0], lean_b[1], 0.0])
        gap_dir = gap_dir / (np.linalg.norm(gap_dir) + 1e-9)
        tips = m.fingertips(env)  # either hand's
        loc = b.to_local(tips)  # in the big tin's frame: their change between steps is their velocity relative to the tin
        loc_s = s.to_local(tips)
        held = [gripping(loc[k:k + 12], r1, 0.0, H1) or gripping(loc_s[k:k + 12], r2, 0.0, H2) for k in (0, 12)]  # a hand holding either tin
        self.held_by = held
        if self.prev is not None:
            vel = (loc - self.prev) / dt
            carry = {}
            for i, (q, v) in enumerate(zip(loc, vel)):
                if held[i // 12]:
                    continue  # a hand holding a tin moves it with it: no blow
                blow = rim_blow(q, v, gap_dir)
                if blow is not None and blow[0] > self.vmax[0]:
                    self.vmax = (round(blow[0], 2), round(blow[1], 0))
                landed, carry[i] = contact_blow(self.pending.get(i), blow, wall_gap(q) if blow is not None or i in self.pending else 1.0)
                if landed is None:
                    continue
                v_in, ang = landed
                self.strikes.append((round(self.steps * dt, 2), round(v_in, 2), round(ang, 0)))
                if v_in >= V_DENT:
                    self.dent = True  # at any time, before or after the seal breaks
                if not self.broken and not self.dent and ang <= 45.0:
                    self.broken, self.by = True, "strike"
                    for nm, sg, *_ in LATCHES:
                        m.unlock(env, "big", nm)
                        m.set_target(env, "big", nm, sg * LATCH_BACK)  # the seal gone: the latches draw back into the wall
                        try:
                            m.set_visible(f"{m.ENV_PATH}/big/{nm}/visual/{nm}", False)
                        except Exception:
                            pass
                    print(f"[sk] seal broken by a strike at {v_in:.2f} m/s, {ang:.0f} degrees from the open side", flush=True)
            self.pending = {i: b for i, b in carry.items() if b is not None}
        free = [wall_gap(q) for i, q in enumerate(loc) if not held[i // 12] and H1 - 0.015 <= q[2] <= H1 + 0.002]
        self.closest = min(self.closest, min(free)) if free else self.closest
        self.prev = loc.copy()
        if self.broken:
            for nm, sg, *_ in LATCHES:
                m.set_target(env, "big", nm, sg * LATCH_BACK)
        if self.broken and tilt_deg(Rb) > 45.0:
            self.spill = True  # sealed, the shaker cannot spill
        if b.pos[2] < TOP - 0.05 or s.pos[2] < TOP - 0.05:
            self.fell = True
        mouth_b = b.to_local(s.pos)[0]  # the small tin's mouth in the big tin's frame
        out = bool(mouth_b[2] > H1 + 0.005 or math.hypot(mouth_b[0], mouth_b[1]) > R1_TOP + R2_BEAD)
        on_mat = lambda p: abs(p[0] - MAT_C[0]) < MAT_SIZE[0] / 2 and abs(p[1] - MAT_C[1]) < MAT_SIZE[1] / 2  # noqa: E731
        allt = m.fingertips(env)
        def far(c, top):  # every fingertip 5 cm out from the tin's side, or 5 cm above its top
            side = np.linalg.norm(allt[:, :2] - c[:2], axis=1) > 0.05 + R1_TOP
            return bool(np.all(side | (allt[:, 2] > top + 0.05)))
        big_set = bool(tilt_deg(Rb) < 10.0 and abs(b.pos[2] - MAT_TOP) < 0.01 and on_mat(b.pos) and far(b.pos, b.pos[2] + H1))
        ends = s.to_world(np.array([[0.0, 0.0, 0.0], [0.0, 0.0, H2]]))  # its mouth, its base
        foot = ends[0] if tilt_deg(Rs) < 10.0 else ends[1] if tilt_deg(Rs) > 170.0 else None  # stood mouth-down or mouth-up
        small_set = bool(out and foot is not None and abs(foot[2] - MAT_TOP) < 0.012 and on_mat(foot) and far(foot, float(ends[:, 2].max())))
        self.last = {"t": round(self.steps * dt, 2), "broken": self.broken, "dent": self.dent, "spill": self.spill, "fell": self.fell, "out": out,
                     "big_set": big_set, "small_set": small_set, "big_tilt": round(tilt_deg(Rb), 1), "small_tilt": round(tilt_deg(Rs), 1),
                     "latch_mm": [round(float(b.joint(nm)) * 1000, 2) for nm, *_ in LATCHES], "mouth_in_big_mm": [round(float(v) * 1000, 1) for v in mouth_b],
                     "gap_dir": [round(float(v), 2) for v in gap_dir[:2]], "strikes": self.strikes[-3:], "fastest_at_rim": list(self.vmax), "holding": self.held_by,
                     "closest_free_tip_to_rim_mm": round(self.closest * 1000, 1)}
        self.hold = self.hold + 1 if _tier(self.last) == 1.0 else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        self.last["hold"] = self.hold
        if self.steps % 10 == 0:
            print("[sk] " + json.dumps({"mode": os.environ.get("TASK_DEMO", ""), **self.last}), flush=True)


def _tier(r) -> float:
    if not r or r["dent"] or r["spill"] or r["fell"]:
        return 0.0
    if not r["broken"]:
        return 0.0
    if r["out"] and r["big_set"] and r["small_set"]:
        return 1.0
    if r["out"] and r["big_set"]:
        return 0.75
    if r["out"]:
        return 0.5
    return 0.25


SEAL = Seal()
m.on_step(SEAL.step)
m.on_reset(SEAL.reset)


def score(objects) -> float:
    return 1.0 if SEAL.full else min(_tier(SEAL.last), 0.75)


def success(objects) -> bool:
    return SEAL.full


# -- the scripted demo ---------------------------------------------------------------------------------------------------------------------------

HOLD_Z = 0.09  # where the left holds the big tin, above its base
TIP_AHEAD = 0.015  # the closed fingertips' front, ahead of the hand's middle along its pointing


def demo_script(ctx):
    """Modes: full, the solution. Failure checks: pull, no blow, the small tin taken and lifted as in the demo; tight, the blow lands 60 degrees round
    from the open side; pulled, the blow stopped 1 cm short of the rim at full speed; nohold, the blow with nothing holding the big tin; carry and rock, no blow: the right hand
    grips the small tin 2 cm above the seam and carries the shaker 6 cm toward +y and back at 0.2 m/s, or rocks it 12 degrees each way, then lifts."""
    env = ctx.emb._env
    L, R = ctx.left, ctx.right
    mode = ctx.mode
    if mode == "scene":
        yield from L.wait(2.0)
        print("[sk] END " + json.dumps({"mode": mode, **SEAL.last}), flush=True)
        return
    hold_left = mode not in ("nohold", "carry", "rock")
    b = m.state(env, "big")
    axis = b.pos.copy()
    gl = d.level(0.0)  # the left from the front, fingers either side of the big tin
    hold = np.array([axis[0], axis[1], MAT_TOP + HOLD_Z])
    if hold_left:
        yield from L.move(hold - 0.09 * L.approach_axis(gl) + np.array([0.0, 0.0, 0.03]), grip=gl, finger=d.FINGER_OPEN, t=2.0, label="before the big tin")
        yield from L.move(hold - 0.09 * L.approach_axis(gl), t=0.8, label="level with its grip")
        yield from L.move(hold, t=1.0, label="fingers either side of the big tin")
        yield from L.close(2 * r1(HOLD_Z), force=40.0, label="hold the big tin")
    gr = d.level(90.0)  # the right from -y, fingers pointing +y, closed: a short blow with the fingertips
    yb = R.approach_axis(gr)
    if mode == "tight":
        gr = d.level(30.0)  # pointing 60 degrees round from the open side's blow
        yb = R.approach_axis(gr)
    if mode in ("full", "tight", "nohold", "pulled"):
        hit_dir = -yb  # from the tin's axis out to where the blow lands
        hit = axis + R1_TOP * hit_dir + np.array([0.0, 0.0, MAT_TOP + H1 - 0.004 - axis[2]])
        tcp_hit = hit - TIP_AHEAD * yb
        yield from R.move(tcp_hit - 0.05 * yb + np.array([0.0, 0.0, 0.04]), grip=gr, finger=d.FINGER_CLOSED, t=2.0, label="the closed fingers back from the seam")
        yield from R.move(tcp_hit - 0.05 * yb, t=0.8, label="level with the rim, 5 cm out")
        stop = -0.010 if mode == "pulled" else 0.004  # pulled: the punch stops 1 cm short of the rim
        yield from R.move(tcp_hit + stop * yb, t=0.12 * (0.05 + stop) / 0.054, label="strike the rim at the seam")
        yield from R.move(tcp_hit - 0.05 * yb, t=0.6, label="back off")
        print("[sk] struck: " + json.dumps(SEAL.last), flush=True)
    gs = d.level(90.0)
    ys = R.approach_axis(gs)
    if mode in ("carry", "rock"):  # the shaker moved by the small tin, held 2 cm above the seam: no blow
        s0 = m.state(env, "small")
        zl = H1 - Z_M + 0.02
        low = s0.to_world(np.array([[0.0, 0.0, zl]]))[0]
        yield from R.move(low - 0.09 * ys + np.array([0.0, 0.0, 0.02]), grip=gs, finger=d.FINGER_OPEN, t=1.5, label="before the small tin, low")
        yield from R.move(low - 0.09 * ys, t=0.6, label="level with it")
        yield from R.move(low, t=1.0, label="fingers either side of the small tin, 2 cm above the seam")
        yield from R.close(2 * r2(zl), force=30.0, label="hold the small tin")
        if mode == "carry":
            yield from R.move(R.at() + np.array([0.0, 0.0, 0.01]), t=0.5, label="lift the shaker 1 cm by the small tin")
            for k in range(2):
                yield from R.move(R.at() + np.array([0.0, 0.06, 0.0]), t=0.3, label=f"carry it 6 cm toward +y at 0.2 m/s ({k + 1})")
                yield from R.move(R.at() - np.array([0.0, 0.06, 0.0]), t=0.3, label=f"and back ({k + 1})")
            yield from R.move(R.at() - np.array([0.0, 0.0, 0.01]), t=0.5, label="set it down")
        else:
            for k in range(3):
                yield from R.turn(12.0, axis=(1.0, 0.0, 0.0), t=0.2, label=f"rock it toward +y ({k + 1})")
                yield from R.turn(-24.0, axis=(1.0, 0.0, 0.0), t=0.3, label=f"rock it toward -y ({k + 1})")
                yield from R.turn(12.0, axis=(1.0, 0.0, 0.0), t=0.2, label=f"upright ({k + 1})")
        print("[sk] moved: " + json.dumps(SEAL.last), flush=True)
        yield from R.open(label="let go of the small tin")
        yield from R.move(R.at() - 0.09 * ys, t=0.8, label="back off")
    # the small tin taken round its middle and lifted off, turned a little each way as it comes
    s0 = m.state(env, "small")
    zg = 0.074  # along the small tin from its mouth, where the hand takes it
    grip_pt = s0.to_world(np.array([[0.0, 0.0, zg]]))[0]
    yield from R.move(grip_pt - 0.09 * ys + np.array([0.0, 0.0, 0.02]), grip=gs, finger=d.FINGER_OPEN, t=1.5, label="before the small tin")
    yield from R.move(grip_pt - 0.09 * ys, t=0.6, label="level with it")
    yield from R.move(grip_pt, t=1.0, label="fingers either side of the small tin")
    yield from R.close(2 * r2(zg), force=30.0, label="hold the small tin")
    for k, (dz, turn) in enumerate(((0.01, 6.0), (0.01, -12.0), (0.012, 6.0))):
        yield from R.move(R.at() + np.array([0.0, 0.0, dz]), t=0.5, label=f"lift and wiggle ({k + 1})")
        yield from R.turn(turn, t=0.4, label=f"wiggle ({k + 1})")
    yield from R.move(R.at() + np.array([0.0, 0.0, 0.07]), t=0.9, label="the small tin up and clear")
    print("[sk] lifted: " + json.dumps(SEAL.last), flush=True)
    s1 = m.state(env, "small")
    off = R.at() - s1.pos  # the hand from the small tin's mouth
    z_over = max(float(s1.pos[2]), MAT_TOP + H1 + 0.04)  # the mouth kept 4 cm over the big tin's rim until it is beside it
    yield from R.move(np.array([SMALL_REST[0], SMALL_REST[1], z_over]) + off, t=1.8, label="over the mat beside the shaker")
    yield from R.move(np.array([SMALL_REST[0], SMALL_REST[1], MAT_TOP + 0.003]) + off, t=1.4, label="stand it mouth-down on the mat")
    yield from R.open(label="let go of the small tin")
    yield from R.move(R.at() + np.array([0.0, 0.0, 0.17]), t=1.2, label="right up off the small tin")
    if hold_left:
        yield from L.open(label="let go of the big tin")
        yield from L.move(L.at() - 0.11 * L.approach_axis(gl) + np.array([0.0, 0.0, 0.04]), t=1.2, label="left clear")
    yield from L.wait(1.5)
    print("[sk] END " + json.dumps({"mode": mode, **SEAL.last}), flush=True)
