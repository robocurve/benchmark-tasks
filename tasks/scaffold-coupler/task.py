"""Fix a scaffold transom to a ledger with a right-angle coupler. Sketch build.

An EN 74 class B right-angle (double) coupler for 48.3 mm tube: its lower half already tightened on the ledger (a tube along y, on two blocks), its upper
half open: the cap lying back on its hinge, the swing T-bolt (M12, 19 mm nut) resting outward on the ledger with its nut near the end. The transom, a 0.7 m stock tube
(3.1 kg), rests with its far end on the far bearer and its near end in a small cradle, 14 degrees off the coupler's line, with a chalk mark 150 mm from its
near end. The right hand lifts the near end (the far end pivots on the bearer) and lays the tube in the coupler's open half with the mark at the ledger's
face; the left hand swings the cap over it and holds it shut; the right hand swings the bolt up into the cap's slot by its nut and spins the nut down until
it bears. The cap hinges on the left, so that it lies back clear of the transom swung in from the right.

Model (declared in code): the coupler is one articulation: the cap on a hinge, the bolt on a hinge, the nut sliding and turning on the bolt. The nut's thread is
code: its slide follows its turn at 2 mm a turn (M14), from 5 turns out down to contact with the cap; at contact it stops turning. The final torque (about
50 N m with a podger) is left out.

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
INSTRUCTION = "Fix the transom to the ledger with the coupler at its chalk mark: lay it in, close the cap, put the bolt in the slot and run the nut down."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own 1 s hold: score() and success() agree
HOLD_STEPS = 20  # 1 s at 20 Hz
TOP = TABLE_TOP[SCENE]

TR = 0.02415  # 48.3 mm tube
LX, LZ = 0.36, 0.035  # the ledger's axis (along y)
TZ = LZ + 0.058  # the transom's axis in the coupler's upper half (along x, at y 0)
CW = 0.05  # the coupler's width along the transom
FACE_X = LX - TR  # the ledger's near face: the mark goes here
MARK = 0.150  # the chalk mark, from the transom's near end
T_LEN, T_MASS = 0.70, 3.1
FB_X, FB_Z = 0.80, TZ - 2 * TR  # the far bearer, under the transom's far end
CAP_HINGE = (LX, -0.034, TZ)
CAP_OPEN = 120.0  # degrees: the cap lies back on its hinge
BOLT_HINGE = (LX, 0.042, TZ - 0.014)
BOLT_L, BOLT_R = 0.062, 0.007
BOLT_REST = -14.5  # degrees from horizontal (+y) where the bolt rests on the ledger
FLANGE_TOP = TZ + 0.005
NUT_AF, NUT_H = 0.019, 0.012  # M12 swing bolt, 19 mm nut
NUT_CONTACT = FLANGE_TOP - BOLT_HINGE[2] + NUT_H / 2  # the nut's centre along the bolt, from its hinge, when it bears on the cap
PITCH = 0.002
NUT_TURNS = 5.0  # the nut starts 10 mm out, far enough along the bolt to clear the flange as the bolt swings up
NUT_START = NUT_CONTACT + PITCH * NUT_TURNS
YAW0 = math.degrees(math.atan2(0.20, 0.60))  # the transom's start, degrees off the coupler's line: its far end on the bearer, its near end 0.2 m to the right
N0 = np.array([0.20, -0.20])  # its near end at the start
N_SEG = 24

LOOKS = {
    "galv": m.Look(color=m.rgb(0.70, 0.71, 0.72), roughness=0.45, metallic=0.9),
    "forged": m.Look(color=m.rgb(0.48, 0.49, 0.50), roughness=0.55, metallic=0.8),
    "dark": m.Look(color=m.rgb(0.10, 0.10, 0.11), roughness=0.6),
    "chalk": m.Look(color=m.rgb(0.95, 0.95, 0.92), roughness=0.9),
    "wood": m.Look(color=m.rgb(0.62, 0.45, 0.26), roughness=0.8),
    "nut": m.Look(color=m.rgb(0.60, 0.58, 0.50), roughness=0.4, metallic=0.9),
}


def _b(name, size, c, look):
    return (name, m.box(size, c), look)


def _wedge(r_in, r_out, a0, a1, x0, x1, at):
    """A convex piece of a ring about an axis along x through `at`, between angles a0 and a1 (from +y towards +z), from x0 to x1."""
    sec = 1.0 / math.cos((a1 - a0) / 2)
    loop = ((x0, r_in), (x1, r_in), (x1, r_out), (x0, r_out))
    v = np.array([(x, at[1] + r * sec * math.cos(a), at[2] + r * sec * math.sin(a)) for a in (a0, a1) for x, r in loop])
    n = 4
    f = [(0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6)] + [t for i in range(n) for t in ((i, (i + 1) % n, n + (i + 1) % n), (i, n + (i + 1) % n, n + i))]
    return m.Mesh(v, np.array(f))


def _arc(prefix, r_in, r_out, a0, a1, x0, x1, at, look, n):
    parts, cols = [], []
    for k in range(n):
        w = _wedge(r_in, r_out, a0 + (a1 - a0) * k / n, a0 + (a1 - a0) * (k + 1) / n, x0, x1, at)
        parts.append((f"{prefix}{k}", w, look))
        cols.append(m.col_hull(f"{prefix}{k}_c", w))
    return parts, cols


def _turn_x(mesh, deg, about):
    """Turn a mesh about an axis along x through `about`."""
    return mesh.moved((-about[0], -about[1], -about[2])).turned(deg, "x").moved(about)


def _frame():
    """Fixed: the ledger on its blocks, the far bearer on its blocks, the coupler's lower half on the ledger and its upper half's saddle, the transom's cradle."""
    parts = [("ledger", m.cylinder(TR, 0.60, (LX, 0.0, LZ), axis="y", segments=32), "galv")]
    cols = [m.col_hull("ledger_c", m.cylinder(TR, 0.60, (LX, 0.0, LZ), axis="y", segments=24))]
    for nm, x, z, ys in (("lb", LX, LZ - TR, (-0.24, 0.24)),):
        for k, y in enumerate(ys):
            parts.append(_b(f"{nm}{k}", (0.07, 0.05, z), (x, y, z / 2), "wood"))
            cols.append(m.col_box(f"{nm}{k}_c", (0.07, 0.05, z), (x, y, z / 2)))
    # the lower half, round the ledger, and the neck up to the upper half's saddle
    parts += [("lower_half", m.cylinder(TR + 0.0063, CW, (LX, 0.0, LZ), axis="y", segments=32), "forged"),  # already tightened on the ledger
              _b("lower_bolt", (0.012, 0.016, 0.03), (LX + TR + 0.012, 0.0, LZ + 0.008), "forged"),
              _b("neck", (0.03, 0.03, TZ - TR - LZ - TR), (LX, 0.0, (TZ - TR + LZ + TR) / 2), "forged")]
    sp, sc = _arc("saddle", TR + 0.0005, TR + 0.0065, math.pi, 2 * math.pi, LX - CW / 2, LX + CW / 2, (0.0, 0.0, TZ), "forged", 12)
    parts += sp
    cols += sc
    lugs = [((CW, 0.014, 0.016), (LX, CAP_HINGE[1] - 0.001, TZ - 0.006)), ((CW, 0.020, 0.020), (LX, BOLT_HINGE[1], BOLT_HINGE[2] - 0.002))]
    for k, (sz, cc) in enumerate(lugs):
        parts.append(_b(f"lug{k}", sz, cc, "forged"))
    return parts, cols


def _stage():
    """Fixed, in the station's axes: the far bearer on its blocks, and the small cradle the transom's near end starts in."""
    parts = [("bearer", m.cylinder(TR, 0.40, (FB_X, 0.0, FB_Z), axis="y", segments=32), "galv")]
    cols = [m.col_hull("bearer_c", m.cylinder(TR, 0.40, (FB_X, 0.0, FB_Z), axis="y", segments=24))]
    for k, y in enumerate((-0.15, 0.15)):
        z = FB_Z - TR
        parts.append(_b(f"bb{k}", (0.07, 0.05, z), (FB_X, y, z / 2), "wood"))
        cols.append(m.col_box(f"bb{k}_c", (0.07, 0.05, z), (FB_X, y, z / 2)))
    d = np.array([math.cos(math.radians(YAW0)), math.sin(math.radians(YAW0))])
    cc = N0 + 0.10 * d
    za, zt = TZ - TR * math.sqrt(2), TZ - 0.004
    for s, nm in ((-1, "a"), (1, "b")):
        outline = [(s * 0.03, 0.0), (0.0, 0.0), (0.0, za), (s * (zt - za), zt), (s * 0.03, zt)]
        if s > 0:
            outline = outline[::-1]
        pr = m.prism(outline, -0.015, 0.015)
        mesh = m.Mesh(pr.v[:, [2, 0, 1]], pr.f).turned(YAW0, "z").moved((cc[0], cc[1], 0.0))
        parts.append((f"cradle_{nm}", mesh, "wood"))
        cols.append(m.col_hull(f"cradle_{nm}_c", mesh))
    return parts, cols


def _cap():
    """The cap, written shut over the transom, then turned open on its hinge: a half ring, the hinge knuckle, and the slotted flange the bolt enters."""
    rp, rc = _arc("cap", TR + 0.0005, TR + 0.0055, 0.02, math.pi - 0.05, LX - CW / 2, LX + CW / 2, (0.0, 0.0, TZ), "forged", 10)
    plates = [((CW, 0.007, 0.008), (LX, 0.0295, FLANGE_TOP - 0.004)),  # starts at the ring's end, clear of the tube
              ((CW / 2 - 0.008, 0.017, 0.008), (LX - 0.008 - (CW / 2 - 0.008) / 2, 0.0415, FLANGE_TOP - 0.004)),  # the slot's cheeks, 17 mm long
              ((CW / 2 - 0.008, 0.017, 0.008), (LX + 0.008 + (CW / 2 - 0.008) / 2, 0.0415, FLANGE_TOP - 0.004)),
              ((CW, 0.010, 0.012), (LX, CAP_HINGE[1] + 0.004, TZ + 0.002))]
    parts = rp + [("knuckle", m.cylinder(0.006, CW, CAP_HINGE, axis="x", segments=16), "forged")]
    cols = list(rc)
    for k, (sz, cc) in enumerate(plates):
        parts.append((f"flange{k}", m.box(sz, cc), "forged"))
        cols.append(m.col_hull(f"flange{k}_c", m.box(sz, cc)))
    parts = [(n, _turn_x(mesh, CAP_OPEN, CAP_HINGE), look) for n, mesh, look in parts]
    cols = []
    for k, (sz, cc) in enumerate(plates):
        cols.append(m.col_hull(f"flange{k}_c", _turn_x(m.box(sz, cc), CAP_OPEN, CAP_HINGE)))
    for k in range(10):
        a0, a1 = 0.02 + (math.pi - 0.07) * k / 10, 0.02 + (math.pi - 0.07) * (k + 1) / 10
        cols.append(m.col_hull(f"cap{k}_c", _turn_x(_wedge(TR + 0.0005, TR + 0.0055, a0, a1, LX - CW / 2, LX + CW / 2, (0.0, 0.0, TZ)), CAP_OPEN, CAP_HINGE)))
    com = _turn_x(m.Mesh(np.array([[LX, 0.008, TZ + 0.022]]), np.zeros((0, 3), int)), CAP_OPEN, CAP_HINGE).v[0]
    return m.hinge("cap", "X", CAP_HINGE, -CAP_OPEN - 0.5, 2.0, parts, cols, mass=0.35, gravity=True, com=tuple(com), inertia=(4e-4, 3e-4, 3e-4),
                   damping=0.02, friction=0.0)


def _bolt_links():
    """The swing T-bolt, written level (along +y from its hinge), so that the nut slides and turns about the link's y; it rests 14.5 degrees lower."""
    hx, hy, hz = BOLT_HINGE
    shank = m.cylinder(BOLT_R, BOLT_L, (hx, hy + BOLT_L / 2, hz), axis="y", segments=16)
    eye = m.cylinder(0.010, 0.022, BOLT_HINGE, axis="x", segments=16)
    neck = m.cylinder(BOLT_R, 0.012, (hx, hy + 0.012, hz), axis="y", segments=16)  # the shank's root, which sits in the cap's slot; the nut never comes down this far
    # 7 degrees past upright, its root bears on the slot's closed end
    bolt = m.hinge("bolt", "X", BOLT_HINGE, BOLT_REST - 0.5, 97.0, [("shank", shank, "galv"), ("eye", eye, "forged")],
                   [m.col_hull("eye_c", eye), m.col_hull("neck_c", neck)],  # no collider on the rest of the shank: the nut round it would collide with it
                   mass=0.12, gravity=True, damping=0.01, friction=0.0, inertia=(5e-5, 1e-5, 5e-5))
    nut_s = m.slider("nut_s", "Y", -PITCH * NUT_TURNS, 0.0005, [], [], at=(hx, hy + NUT_START, hz), mass=0.01, spring=20000.0, damping=50.0, effort=300.0,
                     friction=0.0, parent="bolt", inertia=(1e-6, 1e-6, 1e-6))
    r = NUT_AF / 2 / math.cos(math.pi / 6)
    hexo = [(r * math.cos(math.pi / 6 + k * math.pi / 3), r * math.sin(math.pi / 6 + k * math.pi / 3)) for k in range(6)]
    hexm = m.prism(hexo, -NUT_H / 2, NUT_H / 2).aimed("+Y").moved((hx, hy + NUT_START, hz))
    nut = m.hinge("nut", "Y", (hx, hy + NUT_START, hz), -100000.0, 100000.0, [("hex", hexm, "nut")], [m.col_hull("hex_c", hexm)], mass=0.03,
                  damping=0.002, friction=0.0, parent="nut_s", inertia=(2e-6, 2e-6, 2e-6))
    return [bolt, nut_s, nut]


_fp, _fc = _frame()
# written with the cap on the right and the bolt on the left, then turned half round about the ledger's axis: the cap hinges on the left (+y), clear of the
# transom swung in from the right, and the bolt rests on the right
COUPLER = m.mechanism("coupler", (2 * LX, 0.0, TOP), [_cap()] + _bolt_links(), LOOKS, frame_parts=_fp, frame_colliders=_fc, friction=(0.4, 0.35), solver_iters=48,
                      self_collide=True, yaw_deg=180.0)
_sp, _sc = _stage()
STAGE = m.fixed("stage", (0.0, 0.0, TOP), _sp, _sc, LOOKS, friction=(0.5, 0.45))


def _transom():
    L = T_LEN
    parts = [("tube", m.cylinder(TR, L, (0.0, 0.0, 0.0), axis="x", segments=32), "galv"),
             ("bore_a", m.cylinder(TR - 0.004, 0.0008, (-L / 2 - 0.0003, 0.0, 0.0), axis="x", segments=24), "dark"),
             ("bore_b", m.cylinder(TR - 0.004, 0.0008, (L / 2 + 0.0003, 0.0, 0.0), axis="x", segments=24), "dark"),
             ("mark", m.cylinder(TR + 0.0004, 0.004, (-L / 2 + MARK, 0.0, 0.0), axis="x", segments=32), "chalk")]
    cols = [m.col_hull("tube_c", m.cylinder(TR, L, (0.0, 0.0, 0.0), axis="x", segments=24))]
    return parts, cols


_tp, _tc = _transom()
_d0 = np.array([math.cos(math.radians(YAW0)), math.sin(math.radians(YAW0))])
_c0 = N0 + T_LEN / 2 * _d0
_qz = math.radians(YAW0) / 2
TRANSOM = m.free("transom", (_c0[0], _c0[1], TOP + TZ + 0.0005), _tp, _tc, LOOKS, mass=T_MASS, friction=(0.5, 0.45), start_quat=(0.0, 0.0, math.sin(_qz), math.cos(_qz)))
OBJECTS = [COUPLER, STAGE, TRANSOM]


# -- the nut's thread, as code: its slide follows its turn, and it stops turning once it bears on the cap -----------------------------------------------
class Thread:
    def __init__(self):
        self.reset()

    def reset(self, env=None):
        if env is not None and self.locked is not None:  # free the nut's turn again for the next trial
            an, jn = m._joint(env, "coupler", "nut")
            an.write_joint_stiffness_to_sim(m._tensor(an, 0.0), joint_ids=[jn])
        self.locked = None

    def step(self, env):
        st = m.state(env, "coupler")
        turn = st.joint("nut")  # radians; tightening is a negative turn about the bolt's axis
        target = float(np.clip(PITCH * turn / (2 * math.pi), -PITCH * NUT_TURNS, 0.0))
        asset, j = m._joint(env, "coupler", "nut_s")
        asset.set_joint_position_target(m._tensor(asset, target), joint_ids=[j])
        an, jn = m._joint(env, "coupler", "nut")
        if target <= -PITCH * NUT_TURNS + 1e-5 and self.locked is None:  # bearing on the cap: it turns no further
            self.locked = turn
            asset.write_joint_stiffness_to_sim(m._tensor(asset, 50.0), joint_ids=[jn])
            asset.write_joint_effort_limit_to_sim(m._tensor(asset, 50.0), joint_ids=[jn])
            asset.set_joint_position_target(m._tensor(asset, turn), joint_ids=[jn])


THREAD = Thread()
m.on_step(THREAD.step)
m.on_reset(THREAD.reset)


def now(env) -> dict:
    st, t = m.state(env, "coupler"), m.state(env, "transom")
    R = t.rotation()
    mark = t.to_world(np.array([[-T_LEN / 2 + MARK, 0.0, 0.0]]))[0]
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    at_coupler = t.to_world(np.array([[(LX - t.pos[0]) / max(R[0, 0], 1e-3), 0.0, 0.0]]))[0]  # the transom's axis where it crosses the coupler
    tips = m.fingertips(env)
    hands = float(np.min(np.linalg.norm(tips - np.array([LX, 0.0, TOP + TZ]), axis=1)))
    return {"mark_off": float(mark[0] - FACE_X), "yaw": yaw, "axis_y": float(at_coupler[1]), "axis_z": float(at_coupler[2] - TOP - TZ),
            "cap": CAP_OPEN + st.deg("cap"), "bolt": st.deg("bolt"), "nut": float(st.joint("nut_s")), "hands": hands,
            "dropped": bool(t.pos[2] < TOP + 0.03)}


def _laid(c):
    return abs(c["mark_off"]) < 0.010 and abs(c["yaw"]) < 3.0 and abs(c["axis_y"]) < 0.004 and abs(c["axis_z"]) < 0.004


def _closed(c):
    return c["cap"] < 3.0 and c["bolt"] > 85.0


def _tight(c):
    return c["nut"] < -PITCH * NUT_TURNS + 0.0003


class Trial:
    """Each step: the state's tier; 1.0 only once the full result has held HOLD_STEPS steps in a row (until then 0.5)."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.now = None
        self.hold = 0
        self.full = False
        self.score = 0.0

    def step(self, env):
        self.now = c = now(env)
        moment = _moment(c)
        self.hold = self.hold + 1 if moment == 1.0 else 0
        self.full = self.hold >= HOLD_STEPS
        self.score = 1.0 if self.full else (0.5 if moment == 1.0 else moment)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def _full(c):
    return _laid(c) and _closed(c) and _tight(c) and c["hands"] > 0.06


def _moment(c) -> float:
    """The tier of one step's state, before the hold."""
    if c["dropped"]:
        return 0.0
    if _closed(c) and abs(c["mark_off"]) > 0.030:
        return 0.0
    if _laid(c) and _closed(c):
        return 1.0 if _full(c) else 0.5
    return 0.25 if _laid(c) else 0.0


def success(objects) -> bool:
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


def _report(ctx, env, label=""):
    c = now(env)
    print(f"[coupler] t {ctx.t:5.1f} {label}mark {1000 * c['mark_off']:+6.1f} mm, yaw {c['yaw']:+5.1f}, axis off y {1000 * c['axis_y']:+5.1f} z {1000 * c['axis_z']:+5.1f} mm; "
          f"cap {c['cap']:5.1f} deg open, bolt {c['bolt']:5.1f} deg, nut {1000 * c['nut']:+5.2f} mm (bears at {-1000 * PITCH * NUT_TURNS:.1f}); hands {1000 * c['hands']:.0f} mm; "
          f"score {score(None):.2f}", flush=True)


def demo_script(ctx):
    """Right hand: lifts the transom's near end, swings it round the far bearer onto the coupler's line and lays it in the open half with its mark at the
    ledger. Left hand: swings the cap over it with closed fingertips and holds it down. Right hand: pinches the nut, swings the bolt up into the cap's slot by
    it, then spins the nut down in wrist strokes, each one putting the bolt back into its slot, until it bears. Both hands let go and back away.
    Failure checks (TASK_DEMO or --mode): offmark lays the transom with its mark 35 mm short of the ledger; nocap never shuts the cap; capfree shuts it and lets go of it before
    the bolt; loose leaves the nut where it started."""
    import demokit as d
    from yam_arena.constants import FINGER_OPEN

    env = ctx.emb._env
    l, r = ctx.left, ctx.right
    mode = ctx.mode
    for _ in range(10):
        yield
    _report(ctx, env, f"{mode} start: ")
    t = lambda: m.state(env, "transom")  # noqa: E731
    near = t().to_world(np.array([[-T_LEN / 2 + 0.04, 0.0, 0.0]]))[0]
    gy = d.down(YAW0)
    yield from r.move(near + np.array([0.0, 0.0, 0.06]), grip=gy, finger=FINGER_OPEN, t=2.5, label="over the transom's near end")
    yield from r.move(near, t=1.2, label="fingers either side of the tube")
    yield from r.close(2 * TR, 50.0, t=0.8, label="grip the tube")
    _report(ctx, env, "gripped: ")
    yield from r.move(r.at() + np.array([0.0, 0.0, 0.05]), t=1.2, label="lift the near end; the far end pivots on the bearer")
    _report(ctx, env, "lifted: ")
    goal = np.array([FACE_X - MARK + 0.04 + (-0.035 if mode == "offmark" else 0.0), 0.0, TOP + TZ])
    g0 = r.at()
    piv = np.array([FB_X, 0.0])  # the far end turns about where it lies on the bearer
    rho0, rho1 = np.linalg.norm(g0[:2] - piv), FB_X - goal[0]
    for k in range(1, 9):  # the near end round an arc about the bearer, the wrist turning with the tube
        f = k / 8
        phi = math.radians(YAW0 * (1 - f))
        rho = rho0 + (rho1 - rho0) * f
        p = np.array([piv[0] - rho * math.cos(phi), piv[1] - rho * math.sin(phi), g0[2]])
        yield from r.move(p, grip=d.down(YAW0 * (1 - f)), t=0.4, label="swing it onto the coupler's line")
    _report(ctx, env, "swung: ")
    yield from r.move(goal + np.array([0.0, 0.0, 0.004]), t=1.5, label="lower it into the coupler's open half")
    mark = lambda: t().to_world(np.array([[-T_LEN / 2 + MARK, 0.0, 0.0]]))[0]  # noqa: E731
    if mode != "offmark" and np.linalg.norm(mark()[:2] - np.array([FACE_X, 0.0])) < 0.02:
        yield from r.servo(np.array([FACE_X, 0.0, TOP + TZ]), tol=0.001, timeout=2.0, read=mark, label="its mark to the ledger's face")
    _report(ctx, env, "laid: ")
    yield from r.open(t=0.5)
    yield from r.move(r.at() + np.array([0.0, 0.0, 0.05]), t=1.0, label="rise")

    W = lambda p: np.array([2 * LX - p[0], -p[1], p[2]])  # noqa: E731  the coupler's written coordinates to the station's (it is turned half round)

    # left hand: the cap. Closed fingertips push its flange round its hinge, over the top, until it falls shut; then they hold it down
    def cap_pt(deg, y=-0.045, z=FLANGE_TOP):
        """A point of the cap (written as when shut) with the cap open `deg` degrees, in the station's axes."""
        a = math.radians(deg)
        dy, dz = y - CAP_HINGE[1], z - CAP_HINGE[2]
        return W(np.array([LX + 0.012, CAP_HINGE[1] + dy * math.cos(a) - dz * math.sin(a), TOP + CAP_HINGE[2] + dy * math.sin(a) + dz * math.cos(a)]))

    gp = d.tilted(45, -90)
    point = gp.rot @ np.array([0.0, 0.0, 1.0])
    tcp = lambda p: p - 0.013 * point  # noqa: E731
    start = CAP_OPEN + m.state(env, "coupler").deg("cap")
    if mode != "nocap":
        yield from l.move(tcp(cap_pt(start + 5)) + np.array([0.0, 0.03, 0.0]), grip=gp, finger=0.0, t=2.0, label="behind the open cap's flange")
        for deg in np.linspace(start - 4, 50.0, 12):
            yield from l.move(tcp(cap_pt(deg)), t=0.5, label="swing the cap over")
        _report(ctx, env, "cap over: ")
        hold = W(np.array([LX + 0.012, 0.004, TOP + TZ + TR + 0.0055 - 0.002]))
        if mode == "capfree":
            for _ in range(20):
                yield
            _report(ctx, env, "cap pushed over, 1 s later: ")
            yield from l.move(l.at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="let go of the cap")
        else:
            yield from l.move(tcp(hold) + np.array([0.0, 0.0, 0.02]), t=0.8, label="over the cap's top")
            yield from l.move(tcp(hold), t=0.6, label="hold the cap shut")
        _report(ctx, env, "cap held: ")

    # right hand: the bolt, swung up by its nut (a pinch across the nut is a pin about the bolt's hinge), then the nut spun down in wrist strokes
    def bolt_at(deg, rad=NUT_START):
        a = math.radians(deg)
        return W(np.array([BOLT_HINGE[0], BOLT_HINGE[1] + rad * math.cos(a), TOP + BOLT_HINGE[2] + rad * math.sin(a)]))

    b0 = m.state(env, "coupler").deg("bolt")
    gn = d.down(90)
    n0 = bolt_at(b0, NUT_START + 0.006)  # the fingers 6 mm out along the bolt, clear of the cap's flange
    yield from r.move(n0 + np.array([0.0, 0.0, 0.06]), grip=gn, finger=FINGER_OPEN * 0.7, t=2.5, label="over the bolt's nut")
    yield from r.move(n0 + np.array([0.0, 0.0, 0.004]), t=1.0, label="fingers either side of the nut")
    yield from r.close(NUT_AF, 30.0, t=0.6, label="pinch the nut")
    if os.environ.get("COUPLER_DEBUG"):
        tips = m.fingertips(env)[12:]
        nut = m.state(env, "coupler").to_world(np.array([[BOLT_HINGE[0], BOLT_HINGE[1] + NUT_START, BOLT_HINGE[2]]]))[0]
        print(f"[coupler] dbg nut at {np.round(nut - [0, 0, TOP], 4)} (planned {np.round(n0 - [0, 0, TOP], 4)}), right tips {np.round(tips.min(0) - [0, 0, TOP], 4)} to {np.round(tips.max(0) - [0, 0, TOP], 4)}", flush=True)
    for deg in np.linspace(b0 + 8, 120.0, 14):  # past upright (the hand trails its command by about 10 degrees): the bolt then leans into the slot's closed end and stays
        yield from r.move(bolt_at(deg, NUT_START + 0.006 * math.cos(math.radians(deg))) + np.array([0.0, 0.0, 0.004]), t=0.4, label="swing the bolt up into the cap's slot")
        if os.environ.get("COUPLER_DEBUG"):
            tips = m.fingertips(env)[12:]
            print(f"[coupler] dbg swing to {deg:.0f}: bolt {m.state(env, 'coupler').deg('bolt'):.1f}, right tips {np.round(tips.min(0) - [0, 0, TOP], 4)} to {np.round(tips.max(0) - [0, 0, TOP], 4)}, cmd {np.round(r.at() - [0, 0, TOP], 4)}", flush=True)
    _report(ctx, env, "bolt up: ")
    for k in range(0 if mode == "loose" else 18):
        if _tight(now(env)):
            break
        top = bolt_at(96.0, NUT_START + m.state(env, "coupler").joint("nut_s"))  # where the nut is with the bolt leaning into its slot: each pinch puts it back there
        yield from r.move(top + np.array([0.0, 0.0, 0.008]), grip=d.down(150), finger=FINGER_OPEN * 0.5, t=0.5, label="fingers round the nut")
        yield from r.close(NUT_AF, 40.0, t=0.3, label="pinch the nut")
        yield from r.move(r.at(), grip=d.down(30), t=0.8, label="turn the nut a third of a turn, tightening")
        yield from r.move(r.at(), finger=FINGER_OPEN * 0.5, t=0.3, label="let go of the nut")
        yield from r.move(r.at() + np.array([0.0, 0.0, 0.01]), grip=d.down(150), t=0.5, label="turn the wrist back")
        if os.environ.get("COUPLER_DEBUG"):
            _report(ctx, env, f"stroke {k}: ")
    _report(ctx, env, "nut down: ")
    yield from r.move(r.at() + np.array([0.0, -0.05, 0.05]), t=1.0, label="back away")
    yield from l.move(l.at() + np.array([0.0, 0.05, 0.05]), t=1.0, label="back away")
    for k in range(160):  # 8 s with both hands clear: the film's last part shows the closed coupler
        yield
        if k % 40 == 39:
            _report(ctx, env, "let go: ")
