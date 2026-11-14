"""Offline checks of the mortar scorer and demo, no simulator. The peppercorns and cracked pieces are an array of body
poses as cornkit reads them; the moves the scorer asks for (a broken corn parked, its pieces brought in from the
pool) are made between steps, as the simulator makes them. The pestle's head rests on the tallest bodies under it,
and the corn bed's joint is set as the physics would leave it (pushed down by the force through the corns / 5 N per
mm). Covers every tier and every failure, plus adversarial cases: doing nothing, the pestle resting on the corns by
its own weight, circling just above the corns, pressing without moving, rocking in place, a finger pushing the bed,
a mortar turned along with the pestle, corns the head never reaches, pounding, pieces thrown out, a mortar tipped,
the pestle still held at the end, set down on the worktop, moving at the end, the 1 s hold, a new trial, the cited
numbers and the text, the layout, and a kinematic replay of the demo plan.

    python3 test_task.py
"""
import json
import math
from pathlib import Path

import numpy as np
from yam_arena.demo import RIGHT_BASE_POS, ArmPlan, grip_rotation
from yam_arena.task_file import ObjectState, load

HERE = Path(__file__).parent
M = load(HERE / "task.py").module
import cornkit as ck  # noqa: E402
import grindkit as gr  # noqa: E402
import yamkit as kit  # noqa: E402

GRIP_DOWN = grip_rotation(M.GRIP_YAW)  # the real hand frame: tool z down, fingers closing along x
AWAY = {"left_gripper": (0.15, 0.30, M.TOP + 0.30), "right_gripper": (0.15, -0.25, M.TOP + 0.30)}
MORTAR0 = np.array([M.MORTAR_XY[0], M.MORTAR_XY[1], M.TOP])
FLOOR = M.TOP + M.BED_TOP  # the bowl's bottom under the corns (world z)
WEIGHT_Q = -M.PESTLE_MASS * 9.81 / M.BED_K  # the pestle resting on the corns: 0.65 mm
GRIP_W = M.SHAFT_R / 0.0475  # fingers closed on the 2.8 cm handle
L = M.LAW


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        raise AssertionError(what)


def quat(r):
    from yam_arena.ik import matrix_to_quat_xyzw

    return matrix_to_quat_xyzw(np.asarray(r, float))


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def st(pos, rot=np.eye(3)):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=quat(rot), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p - 0.01,
                       high=p + 0.01)


class World:
    """The bodies' poses (world), starting as cornkit spawns them; the scorer's moves are made before each step."""

    def __init__(self):
        kit._EPISODE["count"] += 1
        M._L = M.Ledger()
        self.p = np.array([b.home for b in M.BODIES], float)
        self.mortar, self.mrot = MORTAR0.copy(), np.eye(3)

    def apply(self):
        for i, pos in ck.pending():
            self.p[i] = M.BODIES[i].park if pos is None else pos
        ck.clear()

    def radius(self):
        return np.array([b.radius for b in M.BODIES])

    def rest_z(self, xy, q=0.0):
        """Where the head's end comes to rest over xy (world): on the tallest live body under it, else on the bottom
        (pushed down by -q)."""
        live = [i for i, s in enumerate(M._L.pepper.state) if s == gr.LIVE]
        top = FLOOR + q
        for i in live:
            if math.hypot(*(self.p[i, :2] - xy)) <= M.HEAD_R:
                top = max(top, self.p[i, 2] + M.BODIES[i].radius)
        return top

    def objs(self, c, rot=np.eye(3), q=0.0, held=True, finger=None):
        """The pestle's head end centered at c (world), turned rot; the bed joint at q; the right hand closed on the
        handle if held, else both hands away. `finger`: the right hand's closed fingers there instead."""
        c = np.asarray(c, float)
        o = {"mortar": st(self.mortar, self.mrot), "pestle": st(c, rot),
             "mortar/bed": st(self.mortar + self.mrot @ np.array([0, 0, M.BED_TOP - M.BED_T / 2 + q]), self.mrot)}
        opening = {}
        for g, p in AWAY.items():
            o[g] = st(p, GRIP_DOWN)
            opening[g] = 1.0
        if finger is not None:
            o["right_gripper"] = st(finger, GRIP_DOWN)
            opening["right_gripper"] = 0.0
        elif held:
            o["right_gripper"] = st(c + M.GRIP_LOW * rot[:, 2], rot @ GRIP_DOWN)
            opening["right_gripper"] = GRIP_W
        return o, opening, q

    def feed(self, frames):
        """frames: callables returning (objects, opening, q) given this world (evaluated after the moves)."""
        s = 0.0
        for f in frames:
            self.apply()
            o, opening, q = f(self)
            o = dict(o)
            o[ck.KEY] = np.hstack([self.p, np.tile([0.0, 0.0, 0.0, 1.0], (len(self.p), 1))])
            kit._OPENING.update(opening)
            kit._JOINTS["mortar/bed"] = kit.JointState(pos=q, vel=0.0, latched=False)
            M.success(o)
            s = M.score(o)
        return s


TEST_R = 0.008  # m: the scorer tests' circle (independent of the demo's)


def circle(n, q=-0.0025, r=TEST_R, pts=24, center=(0.0, 0.0), lift=0.0, **kw):
    """n circles of the head (upright) round `center` (mortar frame), resting on the bodies, the bed pushed down -q."""
    out = []
    for k in range(n * pts):
        t = 2 * math.pi * k / pts
        xy = MORTAR0[:2] + np.asarray(center) + [r * math.cos(t), r * math.sin(t)]
        out.append(lambda w, xy=xy: w.objs([*xy, w.rest_z(xy, q) + lift], q=q, **kw))
    return out


def rest(n=25, held=False):
    """The pestle leaning back in the bowl at its start pose, resting on the corns by its weight."""
    return [lambda w: w.objs(M.C_REST, rx(M.LEAN_REST), q=WEIGHT_Q, held=held) for _ in range(n)]


def standing(n=25, held=False):
    """The pestle standing upright in the middle of the bowl, on whatever is left there."""
    return [lambda w: w.objs([*MORTAR0[:2], w.rest_z(MORTAR0[:2])], q=WEIGHT_Q, held=held) for _ in range(n)]


def grind_to(w, share, limit=400):
    """Circle the pressed head round the middle, then round the bowl, until `share` passes (or the limit)."""
    for k in range(limit):
        if M._L.pepper.passing() >= share:
            return k
        ang = 2.399963 * k
        rr = 0.004 + 0.012 * ((k * 0.37) % 1.0)
        w.feed(circle(1, center=(rr * math.cos(ang), rr * math.sin(ang))))
    return limit


def full_grind(w, fine=0.25, limit=600):
    """Grind to 85% through 1.4 mm, then keep the pressed head circling on the bottom until `fine` passes 0.6 mm."""
    grind_to(w, 0.85)
    for k in range(limit):
        if M._L.pepper.fine_share() >= fine:
            return k
        ang = 2.399963 * k
        w.feed(circle(1, center=(0.008 * math.cos(ang), 0.008 * math.sin(ang))))
    return limit


def test_law():
    check(abs(L.k * L.press_min - 7.5) < 1e-9 and abs(L.k * L.travel - 15.0) < 1e-9, "7.5 N pressed, 15 N at the stop")
    check(M.PESTLE_MASS * 9.81 < 0.5 * L.k * L.press_min, "the pestle's own weight is under half the pressed force")
    check(L.e_corn == 0.043 and L.corn_d == 0.0046 and L.corn_g == 0.043,
          "cited corn: 43 mJ to rupture (74.6 N x 0.575 mm), 4.6 mm, 43 mg")
    check(abs(L.e_piece / L.piece_g - L.e_corn / L.corn_g) < 1e-12, "a cracked piece takes the same energy per gram")
    check(L.piece_d > 0.0014 * 2.5, f"a cracked piece ({L.piece_d * 1000:.2f} mm) is held by the 1.4 mm sieve")
    check(abs(L.e_fine_g - 1.59) < 0.01 and L.medium_d < 0.0014 and 0.0006 < L.medium_d and L.fine_d < 0.0006,
          f"fine grinding takes {L.e_fine_g:.2f} J/g (Rittinger's law from the pieces' 1 J/g, 3.65 -> 0.92 -> 0.42 mm)")
    check(abs(M.PEPPER_G - 1.161) < 1e-9 and M.N_CORNS == 27, "27 corns, 1.16 g")
    check(len(M.BODIES) == M.N_CORNS * (1 + L.split), "the pool holds 2 pieces for every corn")


def test_layout():
    h = M.CORN_HOME
    d = np.linalg.norm(h[:, None, :2] - h[None, :, :2], axis=2) + np.eye(len(h))
    check(d.min() > L.corn_d, f"no two corns overlap ({d.min() * 1000:.2f} mm apart at least)")
    check(np.hypot(h[:, 0], h[:, 1]).max() + L.corn_d / 2 < M.BED_R, "every corn on the bowl's bottom, clear of its wall")
    check(np.allclose(h[:, 2], M.BED_TOP + L.corn_d / 2 + 0.0003), "the corns lie on the bottom")
    for c, lean, what in ((M._C0, M.LEAN0, "spawned"), (M.C_REST, M.LEAN_REST, "settled")):
        c0, a0 = c - MORTAR0, M.axis(lean)
        gap = min(M._capsule_gap(p, c0, c0 + M.HEAD_L * a0, M.HEAD_R + L.corn_d / 2) for p in h)
        check(gap > 0, f"no corn inside the {what} pestle's head ({gap * 1000:.1f} mm clear)")
    check(abs(M.C_REST[1] - MORTAR0[1] - 0.0266) < 0.001 and abs(M.C_REST[2] - MORTAR0[2] - 0.0596) < 0.001,
          "the settled pose matches the one measured in the simulator (26.6 mm, 59.6 mm)")
    park = np.array([b.park for b in M.BODIES])
    pd = np.linalg.norm(park[:, None] - park[None], axis=2) + np.eye(len(park))
    check(pd.min() > 0.006 and park[:, 2].max() < 0.01, "pool spots apart, on the floor under the worktop")
    w = World()
    check(w.feed(rest(3)) == 0.0 and M._L.pepper.counts() == (27, 0), "the scene starts with 27 whole corns in the bowl")


def test_nothing():
    w = World()
    check(w.feed(rest(120)) == 0.0, "doing nothing (pestle leaning in the bowl): 0")
    p = M._L.pepper
    check(p.counts() == (27, 0) and p.energy == 0.0 and p.ground == 0.0, "doing nothing breaks nothing")


def test_weight_only():
    w = World()
    s = w.feed(circle(40, q=WEIGHT_Q))
    check(s == 0.0 and M._L.pepper.energy == 0.0, "pestle circled on the corns by its own weight (3.2 N): nothing")


def test_hover():
    w = World()
    s = w.feed(circle(40, q=-0.0025, lift=0.0015))
    check(s == 0.0 and M._L.pepper.energy == 0.0 and not M._L.pepper.caught,
          "circling 1.5 mm above the corns (the bed pushed by something else): nothing caught, nothing ground")


def test_press_still():
    w = World()
    xy = MORTAR0[:2]
    s = w.feed([lambda w: w.objs([*xy, w.rest_z(xy)], q=-0.003) for _ in range(200)])
    p = M._L.pepper
    check(p.presses <= 1 and p.counts() == (27, 0) and s == 0.0, "pressing hard (15 N) without moving: nothing breaks")


def test_rubbing_breaks_what_is_under_the_head():
    w = World()
    side = np.array([0.010, 0.0])
    w.feed(circle(30, r=0.003, center=side))
    p = M._L.pepper
    whole, cracked = p.counts()
    check(p.breaks[gr.CORN] > 0 and whole + p.breaks[gr.CORN] == 27, f"pressed circling cracks corns "
          f"({p.breaks[gr.CORN]} cracked into {cracked} pieces, {p.breaks[gr.PIECE]} ground)")
    far = [i for i in range(27) if math.hypot(*(M.CORN_HOME[i, :2] - side)) > 0.003 + M.HEAD_R + 0.0005]
    check(far and all(p.state[i] == gr.LIVE for i in far), f"the {len(far)} corns the head never reached are whole")
    moved = [j for j in range(27, len(p.kind)) if p.state[j] != gr.POOL]
    check(len(moved) == 2 * p.breaks[gr.CORN], "each cracked corn brought two pieces in from the pool")
    w.apply()
    on = [j for j in moved if p.state[j] == gr.LIVE]
    check(all(abs(w.p[j, 2] - (FLOOR + L.piece_d / 2 + 0.0003)) < 1e-6 for j in on)
          and all(math.hypot(*(w.p[j, :2] - MORTAR0[:2])) < M.BED_R for j in on),
          "the pieces lie on the bowl's bottom where their corn was")
    gone = [i for i in range(27) if p.state[i] == gr.GONE]
    check(all(np.allclose(w.p[i], M.BODIES[i].park) for i in gone), "cracked corns' bodies parked in the pool")
    total = p.ground + p.whole + p.cracked + p.spilled
    check(abs(total - M.PEPPER_G) < 1e-9, "the mass adds up: ground + whole + cracked + spilled = 1.16 g")


def test_tiers():
    w = World()
    grind_to(w, 0.45)
    p = M._L.pepper
    check(w.feed(circle(1, q=0.0, lift=0.01)) == 0.25 and p.passing() >= 0.4,
          f"{p.passing():.2f} through 1.4 mm, held: 0.25")
    w = World()
    grind_to(w, 0.30)
    check(w.feed(circle(1, q=0.0, lift=0.01)) == 0.0, f"under 40% through 1.4 mm ({M._L.pepper.passing():.2f}): 0")
    w = World()
    grind_to(w, 0.85)
    p = M._L.pepper
    p.medium, p.fine = p.medium + p.fine, 0.0  # (as if all of it had stayed medium)
    check(w.feed(circle(1, q=0.0, lift=0.01)) == 0.5 and p.fine_share() < 0.2,
          f"{p.passing():.2f} through 1.4 mm, {p.fine_share():.2f} through 0.6 mm, held: 0.5")
    w = World()
    full_grind(w)
    p = M._L.pepper
    check(w.feed(circle(1, q=0.0, lift=0.01)) == 0.75 and p.passing() >= 0.8 and p.fine_share() >= 0.2,
          f"{p.passing():.2f} through 1.4 mm and {p.fine_share():.2f} through 0.6 mm, held: 0.75")


def test_fine_only_where_the_head_rubs_the_bottom():
    w = World()
    grind_to(w, 0.85)
    p = M._L.pepper
    f0, e0 = p.fine, p.fine_energy
    w.feed(circle(30, lift=0.0015))  # 1.5 mm off the bottom
    check(p.fine == f0, "the head circling 1.5 mm above the powder (bed pushed by something else): nothing ground fine")
    w.feed(circle(30, q=WEIGHT_Q))
    check(p.fine == f0, "the head circled on the bottom by its own weight: nothing ground fine")
    xy = MORTAR0[:2]
    w.feed([lambda w: w.objs([*xy, w.rest_z(xy, -0.003)], q=-0.003) for _ in range(100)])
    check(p.fine == f0, "pressed on the bottom without moving: nothing ground fine")
    w.feed(circle(30))
    check(p.fine > f0 and p.fine_energy > e0, f"pressed and rubbing on the bottom: fine pepper made ({p.fine * 1000:.0f} mg)")
    total = p.medium + p.fine + p.whole + p.cracked + p.spilled
    check(abs(total - M.PEPPER_G) < 1e-9, "the mass adds up: fine + medium + whole + cracked + spilled = 1.16 g")
    shares = []
    for medium in (0.05, 1.0):  # the same rubbing on a thin scatter and on a full layer of medium pepper
        g = gr.Grinder(M.N_CORNS, L)
        g.state = [gr.GONE if k == gr.CORN else gr.POOL for k in g.kind]
        g.medium = medium
        c0, ax = np.array([0.0, 0.0, M.BED_TOP]), np.array([0.0, 0.0, 1.0])
        for k in range(100):
            c = c0 + [0.008 * math.cos(k / 4), 0.008 * math.sin(k / 4), 0.0]
            g.step(np.zeros((len(g.kind), 3)) - 1, c, ax, compression=0.0025, other_on_bed=False, tilt_deg=0.0,
                   rim_r=M.RIM_R, floor_gap=0.0)
        shares.append(g.fine / medium)
    check(abs(shares[0] - shares[1]) < 0.02 and shares[0] < 0.5 and abs(L.layer_g - 2.0) < 0.01,
          f"the same rubbing grinds the same share of a thin scatter (50 mg) as of a full layer (1 g): {shares[0]:.2f} vs "
          f"{shares[1]:.2f} (the work is scaled by the powder's cover of the bottom; one full layer is {L.layer_g:.1f} g)")
    w = World()
    w.feed(circle(30, r=0.001))  # rubbing the bare middle of the bottom, out of reach of the corns
    check(M._L.pepper.fine == 0.0 and M._L.pepper.medium == 0.0,
          "rubbing the bare bottom before any piece is ground: no fine pepper")


def test_score_is_the_pieces():
    w = World()
    grind_to(w, 0.85)
    p = M._L.pepper
    gone = sum(1 for j in range(27, len(p.kind)) if p.state[j] == gr.GONE)
    check(abs(p.passing() - gone * L.piece_g / M.PEPPER_G) < 1e-12,
          f"passing = crushed pieces x 21.5 mg / 1.16 g ({gone} pieces, {p.passing():.3f})")


def test_full():
    w = World()
    full_grind(w)
    check(w.feed(standing(20)) == 0.75, "set down and released, 19 steps still: still 0.75")
    check(w.feed(standing(1)) == 1.0, "set down, released, still, mortar upright for 1 s: 1.0")
    w = World()
    full_grind(w)
    check(w.feed(standing(40, held=True)) == 0.75, "ground but the pestle still held at the end: 0.75")
    w = World()
    full_grind(w)
    c, rot = np.array([0.30, -0.22, M.TOP + M.HEAD_R]), rx(math.pi / 2)
    check(w.feed([lambda w: w.objs(c, rot, held=False) for _ in range(25)]) == 1.0,
          "pestle laid on the worktop, released: 1.0")
    w = World()
    full_grind(w)
    fr = [lambda w, k=k: w.objs(M.C_REST + [0.002 * (k % 2), 0, 0], rx(M.LEAN_REST), q=WEIGHT_Q, held=False)
          for k in range(40)]
    check(w.feed(fr) == 0.75, "pestle rocking at the end: not still, 0.75")
    w = World()
    full_grind(w)
    w.mrot = rx(math.radians(15))
    check(w.feed(rest(30)) == 0.75, "mortar left tilted 15 degrees: no 1.0")


def test_release_is_contact():
    """Released means no gripper touching the pestle (contact), not a distance zone."""
    import guardkit as gk
    w = World()
    full_grind(w)

    def f(w):
        o, op, q = w.objs(M.C_REST, rx(M.LEAN_REST), q=WEIGHT_Q, held=False)
        top = np.asarray(M.C_REST) + M.PESTLE_L * M.axis(M.LEAN_REST)
        o["right_gripper"] = st(top + [0.0, 0.0, 0.012], GRIP_DOWN)
        op["right_gripper"] = 1.0
        return o, op, q

    o, op, _ = f(w)
    kit._OPENING.update(op)
    check(not gk.touching(o, "pestle", M.PESTLE_PARTS, grippers=M.GRIPPERS), "an open hand over the handle's end")
    check(w.feed([f] * 40) == 1.0, "ground, pestle set down, an open hand over it, not touching: released, 1.0")


def test_wobble_in_place():
    """The head pressed on the corns and rocked 3 degrees round and round without moving: nothing slides."""
    w = World()
    tilt = math.radians(3.0)
    fr = [lambda w, k=k: w.objs([*MORTAR0[:2], w.rest_z(MORTAR0[:2])], rz(2 * math.pi * k / 12) @ rx(tilt), q=-0.0025)
          for k in range(400)]
    check(w.feed(fr) == 0.0 and M._L.pepper.counts() == (27, 0), "rocking the pressed head in place: nothing breaks")


def test_finger_on_bed():
    w = World()
    fr = [lambda w, k=k: w.objs([*(MORTAR0[:2] + 0.008 * np.array([math.cos(k / 4), math.sin(k / 4)])), FLOOR + 0.006],
                                q=-0.002, finger=MORTAR0 + [0.01, 0.0, M.BED_TOP]) for k in range(480)]
    check(w.feed(fr) == 0.0 and M._L.pepper.energy == 0.0, "a finger pushing the bed while the pestle hovers: nothing")


def test_mortar_turns_with_pestle():
    w = World()
    fr = []
    for k in range(300):
        def f(w, k=k):
            w.mrot = rz(0.1 * k)
            c = MORTAR0 + w.mrot @ np.array([0.008, 0.0, 0.0])
            c[2] = w.rest_z(c[:2])
            return w.objs(c, w.mrot, q=-0.0025, held=False)
        fr.append(f)
    check(w.feed(fr) == 0.0 and M._L.pepper.energy < 1e-9, "mortar turning with the pestle (no sliding between them): 0")


def test_pound():
    w = World()
    xy = MORTAR0[:2] + [0.012, 0.0]
    fr = []
    for _ in range(20):  # brought down 3.6 cm in one step: 0.72 m/s, 86 mJ shared by the ~30 corns under the head
        fr.append(lambda w: w.objs([*xy, w.rest_z(xy) + 0.036], q=0.0))
        fr.append(lambda w: w.objs([*xy, w.rest_z(xy)], q=-0.003))
    w.feed(fr)
    p = M._L.pepper
    check(p.strikes >= 10 and p.breaks[gr.CORN] >= 3, f"20 pounds from 3.6 cm crack corns under the head "
          f"({p.breaks[gr.CORN]} cracked, {p.energy:.2f} J)")
    w = World()
    fr = []
    for _ in range(100):  # taps from 3 mm: 0.06 m/s, 0.6 mJ each
        fr.append(lambda w: w.objs([*xy, w.rest_z(xy) + 0.003], q=0.0))
        fr.append(lambda w: w.objs([*xy, w.rest_z(xy)], q=-0.003))
    w.feed(fr)
    check(M._L.pepper.breaks[gr.CORN] == 0, "100 taps from 3 mm crack nothing")


def test_thrown_out():
    w = World()
    grind_to(w, 0.55)
    p = M._L.pepper
    live = [i for i, s in enumerate(p.state) if s == gr.LIVE][:8]
    for i in live:
        w.p[i] = MORTAR0 + [0.0, -0.10, 0.003]  # on the worktop beside the mortar
    w.feed(rest(2))
    check(abs(p.spilled - sum(p.mass(i) for i in live)) < 1e-12 and p.spilled > M.SPILL_CAP_G,
          f"pieces on the worktop count as spilled ({p.spilled:.2f} g)")
    check(w.feed(standing(30)) == 0.25, "more than 0.12 g thrown out caps at 0.25, set down or not")


def test_tipped():
    w = World()
    full_grind(w)
    w.mrot = rx(math.radians(65))
    w.feed(rest(1))
    check(M._L.pepper.tipped and M._L.pepper.ground == 0.0, "mortar tipped 65 degrees: emptied, latched")
    w.mrot = np.eye(3)
    check(w.feed(standing(40)) == 0.0, "tipped and set back up: 0")


def test_new_trial():
    w = World()
    grind_to(w, 0.85)
    w = World()
    check(w.feed(rest(5)) == 0.0 and M._L.pepper.counts() == (27, 0), "a new trial starts with 27 whole corns")


def test_jittered_mortar():
    w = World()
    w.mortar = MORTAR0 + [0.02, -0.01, 0.0]
    w.feed(rest(1))
    moves = dict(ck.pending())
    check(len(moves) == 27 and np.allclose(moves[5], w.mortar + M.CORN_HOME[5]),
          "a mortar placed elsewhere: the corns are moved into its bowl, as laid out")
    w.feed(rest(2))
    check(M._L.pepper.spilled == 0.0, "and none counts as spilled")


def text():
    j = json.loads((HERE / "task.json").read_text())
    return j, " ".join(str(v) for v in j.values()) + (HERE / "README.md").read_text()


def test_text():
    j, t = text()
    check(j["instruction"] == M.INSTRUCTION, "instruction matches task.py")
    for s in ("1.4 mm", "0.6 mm", "No. 14", "No. 30", "40%", "1.59 J", "7.5 N", "5 N per mm", "3 mm", "15 N", "80%", "20%", "0.12 g", "60 degrees",
              "10 degrees", "1 cm", "1 s", "27 whole", "1.2 g", "4.6 mm", "43 mg", "43 mJ", "21.5 mJ", "74.6 N",
              "3.7 mm", "1,280 g", "330 g", "12 cm", "8.5 cm", "14 cm", "2.8 cm", "4 cm", "2,400 steps"):
        check(s in t, f"text states {s}")
    check(f"{M.RUN_LIMIT_STEPS:,} steps" in j["run_limit"], "run limit matches")
    check(len(j["context"]) <= 1000, f"context {len(j['context'])} characters (at most 1,000)")


def test_grip_axis():
    """On the real grip frame the fingers close perpendicular to the pestle's handle at every waypoint."""
    check(np.allclose(GRIP_DOWN[:, 0], [1.0, 0.0, 0.0]), "grip_rotation(GRIP_YAW) closes along world x")
    plan = M.demo()["right"]
    arm = ArmPlan(plan, RIGHT_BASE_POS)
    t, worst = 0.0, 0.0
    for w in plan:
        t += w.duration
        _, rot, _, _ = arm.target(t - 1e-6)
        held = w.finger == M.FINGER_CLOSED
        a = M.axis(M.lean_for_roll(w.roll) if held else M.LEAN_REST)
        if held and w.roll == 0.0 and not w.label.startswith(("lift", "turn")):
            a = np.array([0.0, 0.0, 1.0])  # held upright, low down
        if w.label.startswith("lay it over") or (not held and w.label in ("let go", "slide off the handle", "lift clear", "wait")):
            a = M.axis(w.roll)  # held low and laid over (or just let go there)
        worst = max(worst, abs(90.0 - math.degrees(math.acos(min(1.0, abs(float(rot[:, 0] @ a)))))))
        if w.label == "grip":
            check(abs(float(rot[:, 0] @ [1.0, 0.0, 0.0])) > 0.999, "at the grip the fingers close along world x")
    check(worst < 1.0, f"closing axis perpendicular to the handle at all {len(plan)} waypoints ({worst:.2f} deg off)")


def test_demo():
    """Kinematic replay of the grinding: the upright head follows the hand 7.5 cm above it, held within WALL_R of
    the middle (the bowl's wall bounds a head led round by the hand), resting on the bodies under it; the arm reaches
    only `reach` of each aimed press. The bodies stay where they are put (in the simulator they also get pushed
    round), so this checks the plan's energy and coverage, not the corns' motion."""
    plan = M.demo()["right"]
    dur = sum(w.duration for w in plan)
    check(dur / M.DT < M.RUN_LIMIT_STEPS * 0.8, f"demo {dur:.0f} s = {dur / M.DT:.0f} steps, under 80% of the limit")
    WALL_R = 0.012
    for reach in (1.0, 0.8):
        w = World()
        prev = None
        frames = []
        for wp in plan:
            pos = np.asarray(wp.pos, float)
            n = max(1, round(wp.duration / M.DT))
            grinding = wp.label in ("press", "ease off") or wp.label.startswith("grind")
            for k in range(1, n + 1):
                s = k / n
                s = s * s * (3 - 2 * s)
                p = pos if prev is None else prev + s * (pos - prev)
                if not grinding:
                    continue
                c = p - np.array([0.0, 0.0, M.GRIP_LOW])
                d = c[:2] - MORTAR0[:2]
                nd = float(np.linalg.norm(d))
                if nd > WALL_R:
                    c[:2] = MORTAR0[:2] + d * WALL_R / nd

                def f(w, c=c.copy()):
                    z = w.rest_z(c[:2])
                    q = -min(M.BED_TRAVEL, max(0.0, (z - c[2]) * reach / 2.4e-3 / M.BED_K))  # 2.4 mm per N
                    q = min(q, WEIGHT_Q)
                    return w.objs([*c[:2], w.rest_z(c[:2], q)], q=q)
                frames.append(f)
            prev = pos
        s = w.feed(frames + standing(25))
        p = M._L.pepper
        check(s == 1.0 if reach == 1.0 else s >= 0.5,
              f"demo replay, arm reaching {reach:.0%} of the press: {p.passing():.2f} passing, "
              f"{p.counts()[0]} whole / {p.counts()[1]} cracked left, {p.energy:.1f} J, score {s}")


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
    print("all pass")
