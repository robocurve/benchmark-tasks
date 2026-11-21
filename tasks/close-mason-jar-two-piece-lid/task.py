"""Screw a canning band onto a filled regular-mouth pint Mason jar over its flat lid, square and fingertip-tight.

The jar is a Ball regular-mouth pint (16 oz) canning jar at full size, filled to 1/2 in headspace; its flat lid (the disc with its sealing
compound) has already been set centred on the rim. The robot must lift the screw band off the counter, start it square on the jar's
glass thread and turn it down until the band's flange clamps the disc on the rim, then let go.

Model (declared in code, all collision geometry is real contact, nothing is triggered by proximity):
- Jar ("jar", free, 0.75 kg filled): glass body as a convex hull of its lathe profile, the neck as a cylinder, and the glass thread as a
  continuous right-hand helix: 1.25 turns at 4.23 mm pitch (6 per inch), built from 30 convex segments of its trapezoid section.
- Band ("band", free, 11 g): a flange (24 annular segments), a skirt (24 segments) and its rolled internal thread, a helix of the same
  pitch (21 segments, 0.875 turn). Both threads run out to their roots over 29 degrees at each end. The band's thread rides under the glass thread, so turning the band clockwise (seen from above)
  draws it down; it stops when the flange clamps the disc on the rim.
- Disc ("disc", free, 6 g): a 68.6 mm flat lid resting on the rim. It is loose: a band brought down tilted or off-centre lands on its
  edge and cocks it or pushes it off.
- Not modelled: tightening torque (fingertip-tight versus cranked down cannot be told apart: there is no force rule), the sealing
  compound's give, the band's knurl. Uniform rubber-like friction on the arms' pads.

Scoring (end state, read by the tracker every control step; `score()` returns it):
- 0.0 if the jar has ever tipped more than 25 degrees, or the disc has ever left the rim (centre more than 12 mm off the jar's axis,
  or its underside more than 5 mm below the rim); cleared only at a reset.
- 1.0: band seated (its top within 1.0 mm of its seated height above the rim), square (within 2 degrees of the jar's axis) and centred
  (within 2 mm), disc flat (within 2 degrees) and centred (within 2.5 mm), jar upright (within 5 degrees) and standing on the counter
  (within 3 mm), everything still, and every fingertip point at least 3 cm from jar and band, held for 1 s.
- 0.5: band square and centred as above, top within 2.0 mm of seated (most of the way down), disc not cocked (within 5 degrees).
- 0.25: band threaded on: square within 4 degrees, centred within 3 mm, top within 2.5 mm of seated. (Resting on top of the glass
  thread unthreaded, the band's top stands at least one pitch less its 45 degrees of spare thread, 3.7 mm, above its seated height.)
- 0.0 otherwise (the band still on the counter, or sitting on the jar unthreaded or cross-threaded).
The partial levels need no hands clear (they are progress), full credit does.

    python demokit.py task.py OUT --mode full
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Screw the band on over the lid, fingertip-tight, ready for the canner."
SUCCESS_HOLD_S = 0.0  # the tracker counts its own hold
HOLD_STEPS = 20  # 1 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- the jar: Ball regular-mouth pint, 79 mm at its widest, 70 mm across the thread, rim 127 mm above the counter -------------------------
JAR_XY = (0.30, -0.04)
R_BODY = 0.0397  # 3 1/8 in maximum diameter
H_RIM = 0.127  # jar to its rim (130.6 mm overall with lid and band, less the closure's 3.6 mm)
R_RIM = 0.0333  # the rim's sealing surface, outside
R_ROOT = 0.0334  # the neck at the thread's root
R_CREST = 0.03495  # 2.75 in across the thread
PITCH = 0.0254 / 6  # 6 per inch
TURNS_JAR = 1.25
Z_THREAD_TOP = 0.1225  # the glass thread's centre line at its upper end, above the counter
W_JAR = (0.0022, 0.0012)  # the glass thread's axial width at root and crest
N_PER_TURN = 24

# -- the disc (flat lid) and the band ----------------------------------------------------------------------------------------------------
R_DISC, T_DISC = 0.0336, 0.0015  # 67.2 mm; its channel and compound stand 1.5 mm on the rim
R_FL_IN, T_FL = 0.0290, 0.0012  # the band's flange: opening 58 mm
R_SK_IN, R_SK_OUT, H_SK = 0.0356, 0.0368, 0.0135  # the band's skirt (73.6 mm outside), 13.5 mm from its top
R_BT_CREST, R_BT_ROOT = 0.0342, 0.0360  # the band's rolled thread, inward
W_BAND = (0.0016, 0.0010)
H_SEAT = H_RIM + T_DISC + T_FL  # the band's origin (its top face) above the counter when seated on the disc
GAP_THREAD = 0.00175  # the band's thread centre below the glass thread's when they bear
PHI_BAND = (-2.0 * math.pi, -0.25 * math.pi)  # the band's thread, in its own frame: from 1 turn down to 1/8 turn short of its top end

LOOKS = {
    "glass": m.Look(glass=(0.86, 0.93, 0.92)),
    "glass_edge": m.Look(color=m.rgb(0.62, 0.78, 0.74), roughness=0.15, opacity=0.5),
    "jam": m.Look(color=m.rgb(0.55, 0.06, 0.10), roughness=0.35),
    "tin": m.Look(color=m.rgb(0.80, 0.78, 0.70), metallic=0.9, roughness=0.3),
    "gold": m.Look(color=m.rgb(0.83, 0.68, 0.36), metallic=0.85, roughness=0.3),
    "compound": m.Look(color=m.rgb(0.75, 0.42, 0.22), roughness=0.6),
}


TAPER = 0.5  # rad (29 degrees): each thread runs out to its root over this much of each end, as moulded and rolled threads do


def _depth(a, lo, hi):
    """The fraction of full thread depth at angle a of a thread running from lo to hi."""
    return min(1.0, max(0.15, min(a - lo, hi - a) / TAPER))


def _thread_hull(r_in, r_out, w_in, w_out, z_of, a0, a1, crest=None):
    """One convex piece of a helical thread between angles a0 and a1: its trapezoid section (r_in, width w_in) to (r_out, w_out).
    `crest(a)`, if given, is (r_in, r_out) at angle a, for the run-out at the thread's ends."""
    pts = []
    for a in (a0, a1):
        z = z_of(a)
        c, s = math.cos(a), math.sin(a)
        ri, ro = crest(a) if crest else (r_in, r_out)
        for r, w in ((ri, w_in), (ro, w_out)):
            for dz in (-w / 2, w / 2):
                pts.append((r * c, r * s, z + dz))
    v = np.array(pts)
    # faces are only needed for the visual; the hull collider takes the points
    f = np.array([(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)])
    return m.Mesh(v, f)


def _ring_sector(r0, r1, z0, z1, a0, a1):
    pts = []
    for a in (a0, a1):
        for r in (r0, r1):
            for z in (z0, z1):
                pts.append((r * math.cos(a), r * math.sin(a), z))
    return m.Mesh(np.array(pts), np.array([(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]))


def jar_thread_z(a: float) -> float:
    """The glass thread's centre line height (jar frame) at angle a (rad, a <= 0 down the thread)."""
    return Z_THREAD_TOP + PITCH * a / (2 * math.pi)


def band_thread_z(a: float) -> float:
    """The band's thread centre line height (band frame), so that seated at zero turn it bears under the glass thread."""
    return jar_thread_z(a) - GAP_THREAD - H_SEAT


# -- jar ---------------------------------------------------------------------------------------------------------------------------------
_BODY_PROFILE = [(0.0, 0.0), (0.0355, 0.0), (0.0390, 0.004), (R_BODY, 0.012), (R_BODY, 0.086), (0.0388, 0.094), (0.0362, 0.101),
                 (R_ROOT + 0.0004, 0.1055), (R_ROOT, 0.1075)]


def _jar():
    parts, cols = [], []
    outer = _BODY_PROFILE + [(R_ROOT, H_RIM - 0.0015), (R_RIM, H_RIM - 0.0006), (R_RIM - 0.0012, H_RIM), (0.0305, H_RIM), (0.0300, H_RIM - 0.002), (0.0, H_RIM - 0.002)]
    parts.append(("glass", m.lathe(outer, 48), "glass"))
    fill = [(0.0, 0.004), (0.0375, 0.004), (0.0385, 0.012), (0.0385, 0.086), (0.0372, 0.094), (0.0345, 0.100), (0.0318, 0.1045), (0.0, 0.1045)]
    parts.append(("jam", m.lathe(fill, 40), "jam"))
    a_lo = -TURNS_JAR * 2 * math.pi
    pts = [(R_CREST - 0.0004) * np.array([math.cos(a), math.sin(a), 0.0]) + np.array([0, 0, jar_thread_z(a)]) for a in np.linspace(a_lo, 0.0, 60)]
    parts.append(("thread", m.tube(pts, 0.0008, sides=8), "glass_edge"))
    cols.append(m.col_hull("body_c", m.lathe(_BODY_PROFILE + [(0.0, 0.1075)], 32)))
    cols.append(m.col_cylinder("neck_c", R_ROOT, H_RIM - 0.1065, (0.0, 0.0, (H_RIM + 0.1065) / 2)))
    cols.append(m.col_cylinder("rim_c", R_RIM, 0.004, (0.0, 0.0, H_RIM - 0.002)))
    n = int(round(TURNS_JAR * N_PER_TURN))
    step = (0.0 - a_lo) / n
    for k in range(n):
        a0, a1 = a_lo + k * step, a_lo + (k + 1) * step + 0.01
        crest = lambda a: (R_ROOT - 0.0006, R_ROOT + (R_CREST - R_ROOT) * _depth(a, a_lo, 0.0))
        cols.append(m.col_hull(f"thr_{k:02d}", _thread_hull(0, 0, W_JAR[0], W_JAR[1], jar_thread_z, a0, min(a1, 0.0), crest)))
    return parts, cols


_jp, _jc = _jar()
JAR = m.free("jar", (JAR_XY[0], JAR_XY[1], TOP), _jp, _jc, LOOKS, mass=0.75, friction=(0.5, 0.4), com=(0.0, 0.0, 0.052))


# -- disc (the flat lid) -----------------------------------------------------------------------------------------------------------------
def _disc():
    parts = [("top", m.lathe([(0.0, T_DISC), (R_DISC - 0.0012, T_DISC), (R_DISC, T_DISC - 0.0005), (R_DISC, 0.0004), (R_DISC - 0.0006, 0.0), (0.0, 0.0)], 48), "gold"),
             ("compound", m.lathe([(R_RIM - 0.0025, -0.0002), (R_RIM + 0.0003, -0.0002), (R_RIM + 0.0003, 0.0003), (R_RIM - 0.0025, 0.0003)], 48), "compound")]
    cols = [m.col_cylinder("disc_c", R_DISC, T_DISC, (0.0, 0.0, T_DISC / 2))]
    return parts, cols


DISC_START = (JAR_XY[0] + 0.0003, JAR_XY[1] - 0.0002)  # set by hand, within a millimetre of centred
_dp, _dc = _disc()
DISC = m.free("disc", (DISC_START[0], DISC_START[1], TOP + H_RIM + 0.0003), _dp, _dc, LOOKS, mass=0.006, friction=(1.0, 0.9), torsional_patch=0.02, solver_iters=64)


# -- band ----------------------------------------------------------------------------------------------------------------------------------
def _band():
    prof = [(R_SK_OUT - 0.0004, -H_SK), (R_SK_OUT, -H_SK + 0.0008), (R_SK_OUT, -0.0006), (R_SK_OUT - 0.0006, 0.0), (R_FL_IN + 0.0004, 0.0), (R_FL_IN, -0.0005),
            (R_FL_IN + 0.0004, -T_FL), (R_SK_IN, -T_FL), (R_SK_IN, -H_SK + 0.0004), (R_SK_OUT - 0.0004, -H_SK)]
    parts = [("band", m.lathe(prof, 64), "gold")]
    cols = []
    for k in range(N_PER_TURN):
        a0, a1 = 2 * math.pi * k / N_PER_TURN, 2 * math.pi * (k + 1) / N_PER_TURN + 0.01
        cols.append(m.col_hull(f"fl_{k:02d}", _ring_sector(R_FL_IN, R_SK_OUT, -T_FL, 0.0, a0, a1)))
        cols.append(m.col_hull(f"sk_{k:02d}", _ring_sector(R_SK_IN, R_SK_OUT, -H_SK, -T_FL, a0, a1)))
    lo, hi = PHI_BAND
    n = int(round((hi - lo) / (2 * math.pi) * N_PER_TURN))
    step = (hi - lo) / n
    for k in range(n):
        a0, a1 = lo + k * step, min(lo + (k + 1) * step + 0.01, hi)
        crest = lambda a: (R_SK_IN - (R_SK_IN - R_BT_CREST) * _depth(a, lo, hi), R_BT_ROOT)
        hull = _thread_hull(0, 0, W_BAND[1], W_BAND[0], band_thread_z, a0, a1, crest)
        cols.append(m.col_hull(f"bt_{k:02d}", hull))
    return parts, cols


BAND_START = (0.22, -0.21)
BAND_YAW = 125.0  # as it was set down: where its thread starts is not lined up with the jar's
_bp, _bc = _band()
BAND = m.free("band", (BAND_START[0], BAND_START[1], TOP + H_SK + 0.0003), _bp, _bc, LOOKS, mass=0.011, friction=(1.0, 0.9), torsional_patch=None, solver_iters=64,
              yaw_deg=BAND_YAW)

# -- distractors (SimReady): pickling cucumbers, a mixing bowl, a dish cloth ----------------------------------------------------------------
CUKE_A = SimReady("cucumber_a", pos=(0.52, 0.07, TOP), size=(0.052, 0.170, 0.048), mass=0.15, package="food_produce_cucumber_a01",
                  usd="sm_food_produce_cucumber_a01_01.usda", yaw=1.45, convex=True)
CUKE_B = SimReady("cucumber_b", pos=(0.53, -0.03, TOP), size=(0.052, 0.170, 0.048), mass=0.15, package="food_produce_cucumber_a01",
                  usd="sm_food_produce_cucumber_a01_01.usda", yaw=1.75, convex=True)
BOWL = SimReady("bowl", pos=(0.50, 0.25, TOP), size=(0.163, 0.163, 0.055), mass=0.35, package="dishware_bowl_a01",
                usd="sm_dishware_bowl_a01_01.usda")
CLOTH = SimReady("dish_cloth", pos=(0.47, -0.25, TOP), size=(0.260, 0.233, 0.021), mass=0.05, package="cleaning_cloth_a01",
                 usd="sm_cleaning_cloth_a01_01.usd", yaw=0.3)

OBJECTS = [JAR, DISC, BAND, CUKE_A, CUKE_B, BOWL, CLOTH]


# -- the scorer ----------------------------------------------------------------------------------------------------------------------------
def _tilt_between(r_a, r_b) -> float:
    return float(np.degrees(np.arccos(np.clip(float(r_a[:, 2] @ r_b[:, 2]), -1.0, 1.0))))


def measure(jar, band, disc, tips=None) -> dict:
    """Everything the score reads, in the jar's frame."""
    rj = jar.rotation()
    b = jar.to_local(band.pos)[0]
    dsc = jar.to_local(disc.pos)[0]
    out = {
        "jar_tilt": jar.tilt_deg(),
        "jar_floor": float(jar.pos[2] - TOP),
        "band_rise": float(b[2] - H_SEAT),  # the band's top above its seated height
        "band_off": float(math.hypot(b[0], b[1])),
        "band_tilt": _tilt_between(rj, band.rotation()),
        "band_yaw": float(np.degrees(math.atan2(*(rj.T @ band.rotation())[[1, 0], 0]))),
        "disc_off": float(math.hypot(dsc[0], dsc[1])),
        "disc_drop": float(H_RIM - dsc[2]),  # the disc's underside below the rim
        "disc_tilt": _tilt_between(rj, disc.rotation()),
        "v": (float(np.linalg.norm(jar.lin_vel)), float(np.linalg.norm(band.lin_vel)), float(np.linalg.norm(disc.lin_vel)),
              float(np.linalg.norm(jar.ang_vel)), float(np.linalg.norm(band.ang_vel)), float(np.linalg.norm(disc.ang_vel))),
    }
    if tips is not None:
        q = jar.to_local(tips)
        dr = np.maximum(np.hypot(q[:, 0], q[:, 1]) - R_SK_OUT, 0.0)
        dz = np.maximum(q[:, 2] - (H_SEAT + max(out["band_rise"], 0.0)), 0.0) + np.maximum(-q[:, 2], 0.0)
        out["hand_gap"] = float(np.min(np.hypot(dr, dz)))
    return out


def level(c: dict) -> float:
    """The progress level of a state (no hands rule, no harm)."""
    sq = c["band_tilt"] <= 2.0 and c["band_off"] <= 0.002
    if sq and c["band_rise"] <= 0.001 and c["disc_tilt"] <= 2.0 and c["disc_off"] <= 0.0025 and c["jar_tilt"] <= 5.0 and c["jar_floor"] <= 0.003:
        return 1.0
    if sq and c["band_rise"] <= 0.002 and c["disc_tilt"] <= 5.0:
        return 0.5
    if c["band_tilt"] <= 4.0 and c["band_off"] <= 0.003 and c["band_rise"] <= 0.0025:
        return 0.25
    return 0.0


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place a recorded harm is cleared
        self.tipped = False
        self.disc_lost = False
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.last = {}
        self.prev = None

    def step(self, env):
        jar, band, disc = m.state(env, "jar"), m.state(env, "band"), m.state(env, "disc")
        c = measure(jar, band, disc, m.fingertips(env))
        # still: no origin moved more than 0.5 mm, and the band turned less than 0.5 degree, since the last step (poses, not the
        # solver's velocities, which jitter for a clamped 11 g band)
        now = (np.r_[jar.pos, band.pos, disc.pos], c["band_yaw"])
        prev = self.prev if self.prev is not None else now
        dyaw = abs((now[1] - prev[1] + 180.0) % 360.0 - 180.0)
        c["still"] = bool(np.max(np.abs(now[0] - prev[0])) < 0.0005 and dyaw < 0.5)
        self.prev = now
        self.last = c
        if c["jar_tilt"] > 25.0:
            self.tipped = True
        if c["disc_off"] > 0.012 or c["disc_drop"] > 0.005:
            self.disc_lost = True
        harm = self.tipped or self.disc_lost
        lv = level(c)
        ok = (not harm) and lv == 1.0 and c["still"] and c["hand_gap"] >= 0.03
        self.hold = self.hold + 1 if ok else 0
        if self.hold >= HOLD_STEPS:
            self.full = True
        if self.full and (harm or lv < 1.0):  # credit does not outlast the state that earned it
            self.full = False
        if harm:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        else:
            self.score = min(lv, 0.5)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration ---------------------------------------------------------------------------------------------------------------------
DISTRACTORS = ("cucumber_a", "cucumber_b", "bowl", "dish_cloth")


def _report(ctx, label=""):
    if label.startswith(("end", "right clear", "left clear")):
        print("[distractors] " + ", ".join(f"{n} {np.round(ctx.pos(n)[:2] - np.array(o.pos[:2]), 4).tolist()}" for n, o in
                                          zip(DISTRACTORS, (CUKE_A, CUKE_B, BOWL, CLOTH))), flush=True)
    c = TRIAL.last
    if not c:
        return
    print(f"[jar] t {ctx.t:5.1f} {label} band rise {c['band_rise'] * 1000:+6.2f} mm yaw {c['band_yaw']:+6.1f} off {c['band_off'] * 1000:4.1f} tilt {c['band_tilt']:4.1f} | "
          f"disc off {c['disc_off'] * 1000:4.1f} tilt {c['disc_tilt']:4.1f} drop {c['disc_drop'] * 1000:+5.1f} | jar tilt {c['jar_tilt']:4.1f} "
          f"floor {c['jar_floor'] * 1000:+5.1f} | hands {c.get('hand_gap', 0) * 1000:5.0f} mm still {c['still']} {np.round(c['v'], 3).tolist()} | score {TRIAL.score:.2f} full {TRIAL.full}",
          flush=True)


def demo_script(ctx):
    """Left: takes the jar's body from its left side (fingers level, closing fore and aft) and holds it. Right: picks the band off the
    counter across its diameter, brings it over the jar square, lowers it onto the thread, turns it back a little until it drops onto the
    thread's start, then turns it clockwise in strokes of about 150 degrees (open, turn back, close, turn) until the band stops turning
    down, lets go and moves clear; then the left lets go of the jar and moves clear. If the band stops turning before it has come down
    (its thread butting on the glass thread's start) it backs it off 60 degrees and goes again.
    Red-team modes: none (nothing), onearm (the right alone, the jar not held), careless (the band set on tilted and forced round),
    placed (the band set square on the jar and let go, not turned)."""
    import demokit as d
    from yam_arena.constants import FINGER_OPEN

    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode not in ("full", "none", "onearm", "careless", "placed"):
        raise ValueError(mode)
    for _ in range(3):
        yield
    _report(ctx, f"{mode} start:")
    if mode == "none":
        for _ in range(100):
            yield
        _report(ctx, "end:")
        return
    up = np.array([0.0, 0.0, 1.0])
    jar = ctx.pos("jar")
    hold_jar = mode != "onearm"
    if hold_jar:
        grip_l = d.level(-90.0)
        p_hold = jar + np.array([0.0, 0.0, 0.060])
        yield from l.move(p_hold + np.array([0.0, 0.10, 0.03]), grip=grip_l, finger=FINGER_OPEN, t=2.5, label="left beside the jar")
        yield from l.move(p_hold + np.array([0.0, 0.0, 0.0]), t=1.5, label="left round the jar")
        yield from l.servo(p_hold, timeout=1.0, label="left round the jar")
        yield from l.close(2 * R_BODY, 30.0, t=0.6, label="hold the jar")

    # pick the band across its diameter (fingers along y)
    band = ctx.pos("band")
    z_tcp = 0.004  # TCP above the band's top: the pads cover the skirt
    yield from r.move(band + up * 0.08, grip=d.down(90.0), finger=FINGER_OPEN, t=2.5, label="over the band")
    yield from r.servo(band + up * 0.08, timeout=1.0, label="over the band")
    yield from r.move(band + up * z_tcp, t=1.0, label="round the band")
    yield from r.servo(band + up * z_tcp, timeout=1.0, label="round the band")
    yield from r.close(2 * R_SK_OUT, 20.0, t=0.5, label="grip the band")
    yield from r.move(r.at() + up * 0.03, t=0.6, label="lift the band")
    yield from r.move(np.r_[r.at()[:2], TOP + 0.17], t=1.0, label="band up over the jar's height")
    _report(ctx, "band lifted:")
    jar = ctx.pos("jar")
    rest = H_SEAT + 0.0072  # where the band rests on the thread's top, unthreaded
    over = jar + up * (rest + 0.02 + z_tcp)
    yield from r.move(np.r_[over[:2], TOP + 0.17], t=1.5, label="band above the jar")
    if mode == "careless":
        yield from r.move(over, grip=d.tilted(10.0, 0.0), t=2.0, label="over the jar, tilted")
    else:
        yield from r.move(over, t=2.0, label="over the jar")
        yield from r.servo(over, timeout=1.5, label="over the jar")
    _report(ctx, "over the jar:")
    yield from r.move(jar + up * (rest + z_tcp - 0.0015), t=1.2, label="onto the thread")
    yield from r.wait(0.3)
    _report(ctx, "on the thread:")
    # turn back until the band drops onto the thread's start, then clockwise in strokes
    yield from r.turn(20.0, t=1.0, label="turn back to find the start")
    yield from r.turn(-20.0, t=0.6, label="back")
    _report(ctx, "start found:")
    def band_height():
        return float(ctx.pos("band")[2] - ctx.pos("jar")[2])

    def band_yaw():
        """The band's turn about the jar's axis (degrees), from the two poses."""
        rj, rb = m.quat_matrix(ctx.state("jar").quat_xyzw), m.quat_matrix(ctx.state("band").quat_xyzw)
        x = rj.T @ rb[:, 0]
        return math.degrees(math.atan2(x[1], x[0]))

    h_start = band_height()
    snug = mode == "placed"
    tries = 0
    for k in range(0 if snug else 8):
        # a stroke of up to 150 degrees in 30-degree pieces. Fingertip-tight: once the band has come well down the thread (3 mm) and then
        # stops turning with the hand for two pieces running (resistance: the flange is on the lid), stop. If it stops turning before it
        # has come down (its thread butting on the glass thread's start), back it off 60 degrees, ease the hand up 1 mm and go again, as
        # a person does. Reads only the band's and jar's poses.
        flat = stuck = 0
        wrist = 0.0  # the hand's turn in this stroke, degrees
        for piece in range(5):
            y0 = band_yaw()
            yield from r.turn(-30.0, t=0.6, label=f"stroke {k + 1}")
            wrist -= 30.0
            turned = abs((band_yaw() - y0 + 180.0) % 360.0 - 180.0)
            down = h_start - band_height()
            flat = flat + 1 if (turned < 6.0 and down > 0.003) else 0
            stuck = stuck + 1 if (turned < 6.0 and down <= 0.003) else 0
            print(f"[piece] stroke {k + 1}.{piece + 1}: band turned {turned:5.1f} deg, down {down * 1000:+.2f} mm in all", flush=True)
            if flat >= 2:
                snug = True
                break
            if stuck >= 2 and tries < 3:
                tries += 1
                yield from r.move(r.at() + up * 0.001, t=0.3, label="ease off")
                yield from r.turn(60.0, t=1.0, label="back the band off")
                wrist += 60.0
                break
        _report(ctx, f"stroke {k + 1}:")
        if snug:
            break
        yield from r.open(t=0.4, label="let go of the band")
        yield from r.move(r.at() + up * 0.01, t=0.4, label="lift off")
        if abs(wrist) > 1.0:
            yield from r.turn(-wrist, t=abs(wrist) / 75.0 + 0.4, label="turn back")
        b = ctx.pos("band")
        yield from r.move(b + up * (z_tcp + 0.006), t=0.5, label="down round the band")
        yield from r.move(b + up * (z_tcp - 0.0005), t=0.4, label="round the band")
        yield from r.close(2 * R_SK_OUT, 20.0, t=0.4, label="grip the band")
    yield from r.open(t=0.4, label="let go")
    yield from r.move(r.at() + up * 0.05, t=1.0, label="right up and clear")
    yield from r.move(np.array([0.25, -0.30, TOP + 0.25]), grip=d.down(0.0), t=1.5, label="right away")
    _report(ctx, "right clear:")
    if hold_jar:
        yield from l.open(t=0.5, label="let go of the jar")
        yield from l.move(l.at() + np.array([0.0, 0.12, 0.0]), t=1.2, label="left back off the jar")
        yield from l.move(l.at() + np.array([0.0, 0.0, 0.10]), t=1.0, label="left clear")
        _report(ctx, "left clear:")
    for k in range(80):
        yield
        if k % 10 == 9:
            _report(ctx, "end:")
