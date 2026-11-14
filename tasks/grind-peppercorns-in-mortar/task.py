"""On the worktop in front of the arms: a 12 cm unpolished granite mortar holding 27 whole black peppercorns (1.2 g,
about half a teaspoon), its granite pestle leaning in the bowl with its handle over the rim toward the right arm.

The job is to grind the peppercorns the way a cook does: press the pestle's head down on the corns and work it round
the bowl (or pound them) until at least 80% of the pepper by mass passes a 1.4 mm sieve and at least 20% passes a
0.6 mm sieve, without throwing pepper out of the mortar, then set the pestle down and let go.

The peppercorns are real rigid bodies (cornkit): 4.6 mm, 43 mg each. They lie on a sprung disk in the bowl's bottom
(a mechkit Spring link: 5 N per mm, 3 mm of travel), so how far it is pushed down is the force through the corns.
A corn caught under the pestle's head breaks once the work done on it reaches its measured rupture energy (43 mJ):
its body is replaced by two cracked pieces (bodies moved in from a hidden pool, held by the 1.4 mm sieve), and a
cracked piece that takes its share (21.5 mJ) turns into medium pepper (through 1.4 mm, held on 0.6 mm; its body
leaves, its mass is counted). Medium pepper lying on the bottom is ground fine (through 0.6 mm) where the pressed
head rubs it against the bottom, at 1.59 J per gram (Rittinger's law from the pieces' energy). Work reaches the
caught pieces as friction while the head is pressed (at least 1.5 mm, 7.5 N) and slides across them, and as the
pestle's kinetic energy when a press starts with the head coming down (grindkit). The score reads the sieve classes
by mass. Every threshold below appears once, with the same value in task.json and the README.
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import cornkit as ck  # noqa: E402
import grindkit as gr  # noqa: E402
import guardkit as gk  # noqa: E402
import mechkit as mk  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20  # control step, s
RUN_LIMIT_STEPS = 2400
INSTRUCTION = "Grind the peppercorns."

# -- the mortar (its frame: origin at the middle of its foot on the worktop, z up; m) ---------------------------------
MORTAR_XY = (0.300, -0.080)
MORTAR_MASS = 1.28  # kg, a 12 cm granite mortar without its pestle
OUT_R, HEIGHT = 0.060, 0.085  # 12 cm across, 8.5 cm tall
RIM_R = 0.040  # the bowl is 8 cm across at the rim ...
FLOOR_Z = 0.035  # ... and 5 cm deep
TIERS = ((0.032, FLOOR_Z, 0.055), (0.036, 0.055, 0.070), (RIM_R, 0.070, HEIGHT))  # the bowl's colliders: (inner r, z0, z1)
POST_R, N_POSTS = 0.010, 28  # each tier is a ring of 2 cm granite posts (the bowl's curved wall)
BED_R = 0.031  # the bowl's bottom under the corns: a disk 6.2 cm across ...
BED_T = 0.004  # ... 4 mm thick (collider)
BED_TOP = 0.043  # its top surface at rest, where the corns lie
BED_K = 5000.0  # N/m: 5 N per mm ...
BED_TRAVEL = 0.003  # ... over 3 mm (15 N at the stop)
BED_DAMP = 100.0  # N s/m: about critical with the pestle standing on it, so nothing rocks on a bouncing disk
MORTAR_FRICTION = (0.5, 0.4)  # static, dynamic: unpolished granite on a worktop (and the pestle on the bowl's wall)
CORN_FRICTION = (0.35, 0.3)  # peppercorns on granite (and the bowl's bottom, dusted with pepper)
GRANITE = kit.Look((0.40, 0.40, 0.41), 0.9)
PEPPER = kit.Look((0.10, 0.08, 0.07), 0.8)
FLOOR_LOOK = kit.Look((0.36, 0.36, 0.37), 0.9)

_PROFILE = [(0.0, 0.0), (0.057, 0.0), (OUT_R, 0.004), (OUT_R, HEIGHT - 0.003), (OUT_R - 0.003, HEIGHT), (RIM_R, HEIGHT),
            (0.036, 0.070), (0.032, 0.055), (0.031, 0.040), (0.0, FLOOR_Z)]
_POSTS = tuple(kit.Collider(((r + POST_R) * math.cos(2 * math.pi * (i + 0.5 * t) / N_POSTS),
                             (r + POST_R) * math.sin(2 * math.pi * (i + 0.5 * t) / N_POSTS), (z0 + z1) / 2),
                            radius=POST_R, height=z1 - z0)
               for t, (r, z0, z1) in enumerate(TIERS) for i in range(N_POSTS))
BED_LINK = kit.Link(
    "bed", "prismatic", "Z", anchor=(0.0, 0.0, BED_TOP - BED_T / 2), lower=-BED_TRAVEL, upper=0.0, start=0.0,
    mass=0.02, meshes=(kit.cylinder((0, 0, 0), BED_R, BED_T, FLOOR_LOOK),),
    colliders=(kit.Collider((0, 0, 0), radius=BED_R, height=BED_T),),
    law=mk.Spring(rest=0.0, k=BED_K, damp=BED_DAMP), damping=0.0, surface=CORN_FRICTION)
MORTAR = kit.Mechanism(
    "mortar", pos=(MORTAR_XY[0], MORTAR_XY[1], TOP), mass=MORTAR_MASS, size=(2 * OUT_R, 2 * OUT_R, HEIGHT),
    meshes=(kit.lathe(_PROFILE, GRANITE, n=48),),
    colliders=(kit.Collider((0, 0, FLOOR_Z / 2), radius=OUT_R, height=FLOOR_Z), *_POSTS),
    links=(BED_LINK,), friction=MORTAR_FRICTION)
BED_PARTS = (gk.Ring(0.0, BED_R, -BED_T / 2, BED_T / 2),)  # in the bed link's frame

# -- the pestle (its frame: origin at the middle of the head's end, z up the pestle) ------------------------------------
PESTLE_MASS = 0.33  # kg
PESTLE_L = 0.140  # 14 cm long
HEAD_R, HEAD_L = 0.020, 0.035  # a 4 cm head ...
SHAFT_R = 0.014  # ... and a handle 2.8 cm across
PESTLE_PARTS = (gk.Ring(0.0, HEAD_R, 0.0, HEAD_L), gk.Ring(0.0, SHAFT_R, HEAD_L, PESTLE_L))
LEAN0 = math.radians(52.0)  # spawned leaning in the bowl, its handle toward the rim on -y (the right arm) ...
LEAN_REST = math.radians(58.0)  # ... it settles at once, its handle on the rim, at this lean (_lean_rest_y)


def _lean_rest_y(lean: float) -> float:
    """Where the head's end middle sits (y, mortar frame) when the pestle leans `lean` toward -y with its head's rim on
    the bowl's bottom and its handle resting on the rim's inner edge (the handle's axis 1.4 cm from that edge): the
    pose it settles into by itself."""
    a = np.array([0.0, -math.sin(lean), math.cos(lean)])
    z = BED_TOP + HEAD_R * math.sin(lean)
    edge = np.array([0.0, -RIM_R, HEIGHT])
    lo, hi = -0.02, 0.04
    for _ in range(60):
        y = (lo + hi) / 2
        d = edge - np.array([0.0, y, z])
        gap = float(np.linalg.norm(d - float(d @ a) * a)) - SHAFT_R
        lo, hi = (y, hi) if gap < 0 else (lo, y)  # too close to the edge: the head sits further toward +y
    return (lo + hi) / 2


# Torsional friction at the head: pressed into a layer of corns and grit, the head's end meets them over a patch, not
# at points, so turning the pestle about its axis on the corns is resisted (mu x load x patch radius).
HEAD_PATCH = (0.010, 0.004)  # m: PhysX torsional patch radius, and its minimum
# Spawned 1 to 2 mm clear of the bottom and the rim (spawning a body in resting contact can throw it), it settles
# into C_REST (measured in the simulator: 57.8 degrees, the head's end middle 26.6 mm toward +y, 59.6 mm up).
_C0 = np.array([MORTAR_XY[0], MORTAR_XY[1] + 0.018, TOP + BED_TOP + 0.001 + HEAD_R * math.sin(LEAN0)])
C_REST = np.array([MORTAR_XY[0], MORTAR_XY[1] + _lean_rest_y(LEAN_REST), TOP + BED_TOP + HEAD_R * math.sin(LEAN_REST)])
PESTLE = kit.Prop(
    "pestle", pos=tuple(_C0), mass=PESTLE_MASS, size=(2 * HEAD_R, 2 * HEAD_R, PESTLE_L),
    tilt_xyzw=(math.sin(LEAN0 / 2), 0.0, 0.0, math.cos(LEAN0 / 2)),
    meshes=(kit.lathe([(0.0, 0.0), (0.012, 0.001), (0.018, 0.005), (HEAD_R, 0.012), (HEAD_R, 0.028), (0.016, HEAD_L),
                       (SHAFT_R, 0.050), (SHAFT_R, PESTLE_L - 0.003), (0.011, PESTLE_L), (0.0, PESTLE_L)], GRANITE),),
    colliders=(kit.Collider((0, 0, HEAD_L / 2), radius=HEAD_R, height=HEAD_L),
               kit.Collider((0, 0, (HEAD_L + PESTLE_L) / 2), radius=SHAFT_R, height=PESTLE_L - HEAD_L)),
    torsion=HEAD_PATCH)

OBJECTS = [MORTAR, PESTLE]

# -- the peppercorns (cornkit bodies; not in OBJECTS, so object jitter never piles them up) ----------------------------
# Black pepper, dried (3.3% moisture, dry basis): rupture force 74.6 N at 0.575 mm, rupture energy 42.9 mJ (force x
# deformation); geometric mean diameter 4.63 mm; 1000-seed mass 42.8 g (Balasubramanian et al. 2013, J. Spices and
# Aromatic Crops 22(2): 131-137). Damper corns are softer (50.1 N, 32.8 mJ at 18% moisture), so dry corns are the
# hard case.
N_CORNS = 27
RING_IN = 0.0233  # m: the corns start round the edge of the bowl's flat bottom; its middle, 4.2 cm across, is clear
LAW = gr.Law(corn_d=0.0046, corn_g=0.043, e_corn=0.043, split=2, k=BED_K, travel=BED_TRAVEL, press_min=0.0015,
             mu=CORN_FRICTION[1], touch=0.0007, max_slide=0.01, pestle_kg=PESTLE_MASS, head_r=HEAD_R, tip_deg=60.0,
             medium_d=0.00092, fine_d=0.00042, floor_touch=0.001, bed_r=BED_R)
PEPPER_G = N_CORNS * LAW.corn_g  # g of whole corns at the start (1.16 g)
CORN_LOOK = (0.10, 0.08, 0.07)  # black, wrinkled
PIECE_LOOK = (0.42, 0.34, 0.26)  # a cracked corn shows its pale inside
# Rough, wrinkled corns (95% sphericity) do not roll on like balls: their rolling is damped. Their contacts get a
# small torsional patch (a corn's flattened contact, under a millimeter).
CORN_PHYSICS = ck.Physics(static_friction=CORN_FRICTION[0], dynamic_friction=CORN_FRICTION[1], angular_damping=2.0,
                          torsion=(0.001, 0.0005))
POOL_XY0, POOL_PITCH, POOL_COLS = (0.42, 0.10), 0.012, 15  # the hidden pool: on the floor under the worktop


def _capsule_gap(p, a0, a1, r) -> float:
    """Distance from point p to a capsule of radius r round the segment a0-a1 (negative inside)."""
    d = a1 - a0
    t = min(1.0, max(0.0, float((p - a0) @ d) / float(d @ d)))
    return float(np.linalg.norm(p - (a0 + t * d))) - r


def corn_layout(n: int = N_CORNS, seed: int = 3) -> np.ndarray:
    """Where the corns start, in the mortar's frame: one layer round the edge of the bowl's bottom (centers from
    RING_IN out), on a 5 mm hex grid (jittered 0.2 mm), clear of the bowl's wall and of the pestle where it is spawned
    and where it settles. Deterministic. (The middle is clear: a flat pestle head stood on a layer of round corns rolls
    off them, as on ball bearings.)"""
    rng = np.random.default_rng(seed)
    r = LAW.corn_d / 2
    z = BED_TOP + r + 0.0003
    m0 = np.array([MORTAR_XY[0], MORTAR_XY[1], TOP])
    poses = [(c - m0, np.array([0.0, -math.sin(lean), math.cos(lean)]))
             for c, lean in ((_C0, LEAN0), (C_REST, LEAN_REST))]
    spots = []
    pitch = 0.005
    for j in range(-8, 9):
        for i in range(-8, 9):
            x, y = (i + 0.5 * (j % 2)) * pitch, j * pitch * math.sqrt(3) / 2
            p = np.array([x, y, z])
            if not RING_IN <= math.hypot(x, y) <= BED_R - r - 0.0005:
                continue
            if any(_capsule_gap(p, c0, c0 + HEAD_L * a0, HEAD_R + r + 0.0008) < 0
                   or _capsule_gap(p, c0 + HEAD_L * a0, c0 + PESTLE_L * a0, SHAFT_R + r + 0.0008) < 0
                   for c0, a0 in poses):
                continue
            spots.append(p)
    spots = np.array(spots)
    if len(spots) < n:
        raise ValueError(f"only {len(spots)} spots for {n} corns")
    pick = np.sort(rng.choice(len(spots), size=n, replace=False))
    out = spots[pick].copy()
    out[:, :2] += rng.uniform(-0.0002, 0.0002, size=(n, 2))
    return out


CORN_HOME = corn_layout()  # mortar frame


def _bodies() -> list:
    kinds = gr.bodies(N_CORNS, LAW)
    m0 = np.array([MORTAR_XY[0], MORTAR_XY[1], TOP])
    out = []
    for i, kind in enumerate(kinds):
        r = LAW.size(kind) / 2
        park = (POOL_XY0[0] + POOL_PITCH * (i % POOL_COLS), POOL_XY0[1] + POOL_PITCH * (i // POOL_COLS), r + 0.0005)
        home = tuple(m0 + CORN_HOME[i]) if kind == gr.CORN else park
        corn = kind == gr.CORN
        out.append(ck.Body(name=f"{kind}_{i:03d}", radius=r, mass=(LAW.corn_g if corn else LAW.piece_g) * 1e-3,
                           color=CORN_LOOK if corn else PIECE_LOOK, home=tuple(float(v) for v in home),
                           park=tuple(float(v) for v in park)))
    return out


BODIES = _bodies()
ck.setup(BODIES, CORN_PHYSICS)

# -- thresholds (each with its reason in the README and task.json) -----------------------------------------------------
PASS_14 = 0.80  # share by mass through the 1.4 mm sieve (US No. 14) for 0.5 and up ...
PASS_30 = 0.20  # ... and through the 0.6 mm sieve (US No. 30) as well, for 0.75 and 1.0
PASS_14_LOW = 0.40  # through 1.4 mm for 0.25
SPILL_CAP_G = 0.12  # g thrown out of the mortar (about 10% of the batch, three corns): more than this caps it at 0.25
UPRIGHT_DEG = 10.0  # the mortar standing on its foot at the end
STILL_STEP = 0.0005  # m per control step (1 cm/s) ...
STILL_TURN = math.radians(0.5)  # ... and under 0.5 degree a step
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)  # 20 steps = 1 s
GRIPPERS = ("left_gripper", "right_gripper")
EZ = np.array([0.0, 0.0, 1.0])


def rotm(q) -> np.ndarray:
    """Rotation matrix from a quaternion (x, y, z, w)."""
    x, y, z, w = (float(c) for c in q)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def tilt_deg(q) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(rotm(q)[2, 2])))))


def head_low(c, a) -> np.ndarray:
    """The lowest point of the head's end disk (center c, pestle axis a), in the frame c and a are given in."""
    c, a = np.asarray(c, float), np.asarray(a, float)
    u = EZ - float(EZ @ a) * a
    n = float(np.linalg.norm(u))
    return c - HEAD_R * u / n if n > 1e-9 else c.copy()


def pestle_low_z(pp, a) -> float:
    """World height of the pestle's lowest point (its two ends' rims; the profile is convex between them)."""
    side = math.sqrt(max(0.0, 1.0 - float(a[2]) ** 2))
    top = np.asarray(pp, float) + PESTLE_L * np.asarray(a, float)
    return min(float(pp[2]) - HEAD_R * side, float(top[2]) - SHAFT_R * side)


def bed_q(objects) -> float:
    """The corn bed's joint position (m, 0 at rest, negative pushed down)."""
    return float(kit.joint(objects, "mortar/bed").pos)


class Ledger:
    """One trial's pepper and history, advanced once per control step (success() and score() share it)."""

    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.objs = None
        self.pepper = gr.Grinder(N_CORNS, LAW)
        self.last = None  # (mortar pos, mortar rot, pestle pos, pestle rot)
        self.head_z = []  # the head's end height (world) at the last steps
        self.good = 0  # consecutive steps with the pestle set down, released and still, the mortar upright
        self.placed = False  # the corns checked against where the mortar really stands
        ck.clear()

    def tier(self) -> float:
        """The score from the two sieves alone, with the spill cap: 0.75 for 80% through 1.4 mm and 20% through 0.6 mm,
        0.5 for 80% through 1.4 mm, 0.25 for 40%. A mortar tipped past 60 degrees has poured its pepper out (grindkit
        counts it as spilled), so nothing passes then without a separate rule."""
        p = self.pepper
        f14, f30 = p.passing(), p.fine_share()
        if f14 >= PASS_14 - 1e-12:
            s = 0.75 if f30 >= PASS_30 - 1e-12 else 0.5
        else:
            s = 0.25 if f14 >= PASS_14_LOW - 1e-12 else 0.0
        if p.spilled > SPILL_CAP_G + 1e-12:
            s = min(s, 0.25)
        return s


_L = Ledger()


def _turn(r0, r1) -> float:
    return math.acos(max(-1.0, min(1.0, (float(np.trace(r0.T @ r1)) - 1) / 2)))


def _update(objects):
    global _L
    if _L.episode != kit.episode():
        _L = Ledger()
    n = kit._EPISODE.get("last")
    key = (kit.episode(), n) if n is not None else None
    if (key is not None and key == _L.key) or (key is None and objects is _L.objs):
        return
    _L.key, _L.objs = key, objects
    pm, rm = np.asarray(objects["mortar"].pos, float), rotm(objects["mortar"].quat_xyzw)
    pp, rp = np.asarray(objects["pestle"].pos, float), rotm(objects["pestle"].quat_xyzw)
    p = _L.pepper
    if not _L.placed:  # a mortar placed elsewhere (object jitter): put the corns in its bowl, as laid out
        _L.placed = True
        moved = float(np.linalg.norm(pm - np.array([MORTAR_XY[0], MORTAR_XY[1], TOP]))) > 0.002
        if moved or not np.allclose(rm, np.eye(3), atol=0.01):
            for i in range(N_CORNS):
                ck.move(i, pm + rm @ CORN_HOME[i])
    body = objects.get(ck.KEY)
    pos = (rm.T @ (np.asarray(body, float)[:, :3] - pm).T).T if body is not None and len(body) else \
        np.zeros((len(p.kind), 3)) - 1.0
    if body is not None and len(body) and not _L.head_z:  # the first step: the corns may still be moving in
        pos[:N_CORNS] = CORN_HOME
    q = bed_q(objects)
    end = rm.T @ (pp - pm)  # the middle of the head's end, in the mortar's frame
    a = rm.T @ rp[:, 2]
    _L.head_z = (_L.head_z + [float(pp[2])])[-3:]
    v_down = max([(z0 - z1) / DT for z0, z1 in zip(_L.head_z, _L.head_z[1:])] + [0.0])
    tilt = tilt_deg(objects["mortar"].quat_xyzw)
    other = bool(gk.touching(objects, "mortar/bed", BED_PARTS, grippers=GRIPPERS))
    gap = float(head_low(end, a)[2]) - (BED_TOP + q)  # the head's lowest point above the bowl's bottom
    for i, where in p.step(pos, end, a, compression=-q, other_on_bed=other, tilt_deg=tilt, rim_r=RIM_R,
                           v_down=v_down, floor_gap=gap):
        ck.move(i, None if where is None else pm + rm @ np.asarray(where))
    still = (_L.last is not None and float(np.linalg.norm(pm - _L.last[0])) <= STILL_STEP
             and float(np.linalg.norm(pp - _L.last[2])) <= STILL_STEP
             and _turn(_L.last[1], rm) <= STILL_TURN and _turn(_L.last[3], rp) <= STILL_TURN)
    _L.last = (pm, rm, pp, rp)
    ok = _L.tier() == 0.75 and tilt < UPRIGHT_DEG and set_down(objects) and still
    _L.good = _L.good + 1 if ok else 0
    _DRAW.draw(objects, _L.pepper)


def set_down(objects) -> bool:
    """The pestle rests in the bowl (its head's end inside the bowl, below the rim) or lies on the worktop, and neither
    hand touches it (guardkit's contact check: the gripper's own shape within 2 mm of the pestle)."""
    pm, rm = np.asarray(objects["mortar"].pos, float), rotm(objects["mortar"].quat_xyzw)
    pp, rp = np.asarray(objects["pestle"].pos, float), rotm(objects["pestle"].quat_xyzw)
    c = rm.T @ (pp - pm)
    in_bowl = float(np.hypot(c[0], c[1])) < RIM_R and FLOOR_Z - 0.005 < float(c[2]) < HEIGHT
    lz = pestle_low_z(pp, rp[:, 2])
    on_top = TOP - 0.003 <= lz <= TOP + 0.005 and float(pp[2]) < TOP + 0.04
    return bool((in_bowl or on_top) and not gk.touching(objects, "pestle", PESTLE_PARTS, grippers=GRIPPERS))


def success(objects):
    """At least 80% by mass through 1.4 mm and 20% through 0.6 mm, no more than 0.12 g thrown out, the pestle set
    down and let go, the mortar upright, everything still: this step counts toward the 1 s hold."""
    _update(objects)
    return _L.good >= 1


def score(objects):
    """By the shares of the starting 27 corns' mass broken down through a 1.4 mm and a 0.6 mm sieve: 0.25 for 40%
    through 1.4 mm, 0.5 for 80% through 1.4 mm, 0.75 for that and 20% through 0.6 mm as well; 1.0 for that with the
    pestle set down in the bowl or on the worktop, released (no hand
    touching it), the mortar upright within 10 degrees, and everything still for 1 s. More than 0.12 g thrown out of
    the mortar caps the score at 0.25; a mortar knocked past 60 degrees pours its pepper out (spilled, so nothing
    passes). Doing nothing scores 0."""
    _update(objects)
    s = _L.tier()
    if s == 0.75 and _L.good >= HOLD_STEPS:
        s = 1.0
    return s


# -- what the cameras see -----------------------------------------------------------------------------------------------
class _Ground:
    """The counted pepper (under 1.4 mm, too fine for bodies) drawn as thin layers on the bowl's bottom, each spreading
    as its mass grows: medium in brown, fine in a paler grey-brown on top. They show the counted masses, nothing else.
    The corns and cracked pieces are the real bodies. Off outside the simulator."""

    def __init__(self):
        self.markers, self.origin, self.off, self.shown = None, None, False, None

    def draw(self, objects, pepper):
        if self.off:
            return
        ground, fine = pepper.ground / pepper.start, pepper.fine / pepper.start
        b = objects["mortar/bed"]
        sig = (round(ground, 2), round(fine, 2), tuple(np.round(np.asarray(b.pos, float), 4)))
        if sig == self.shown:
            return
        try:
            import isaaclab.sim as sim_utils
            from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

            if self.markers is None:
                lin = lambda c: tuple(x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4 for x in c)  # noqa
                protos = {k: sim_utils.CylinderCfg(radius=1.0, height=1.0, visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=lin(c), roughness=1.0)) for k, c in (("medium", (0.36, 0.28, 0.21)),
                                                                       ("fine", (0.55, 0.50, 0.44)))}
                self.markers = VisualizationMarkers(VisualizationMarkersCfg(prim_path="/Visuals/grindkit", markers=protos))
            if self.origin is None:
                import omni.usd
                from pxr import UsdGeom

                stage, cache = omni.usd.get_context().get_stage(), UsdGeom.XformCache()
                self.origin = np.zeros(3)
                for prim in stage.Traverse():
                    if prim.GetName() == "mortar":
                        w = np.array(cache.GetLocalToWorldTransform(prim).ExtractTranslation(), float)
                        self.origin = w - np.asarray(objects["mortar"].pos, float)
                        break
            self.shown = sig
            if ground <= 0.02:
                self.markers.set_visibility(False)
                return
            rb = rotm(b.quat_xyzw)
            top = np.asarray(b.pos, float) + self.origin + rb @ np.array([0.0, 0.0, BED_T / 2 + 0.0003])
            tr, sc, ids = [top], [(BED_R * min(1.0, 0.4 + 0.6 * ground),) * 2 + (0.0006,)], [0]
            if fine > 0.02:
                tr.append(top + rb @ np.array([0.0, 0.0, 0.0004]))
                sc.append((BED_R * min(1.0, 0.3 + 1.5 * fine),) * 2 + (0.0006,))
                ids.append(1)
            self.markers.set_visibility(True)
            self.markers.visualize(translations=np.array(tr, np.float32),
                                   orientations=np.tile(np.array([0.0, 0.0, 0.0, 1.0], np.float32), (len(tr), 1)),
                                   scales=np.array(sc, np.float32), marker_indices=np.array(ids, int))
        except ImportError:
            self.off = True  # no simulator: offline tests
        except Exception as e:  # noqa: BLE001 (cosmetic only)
            self.off = True
            print(f"grind: ground-pepper layer off ({e!r}); scoring is unaffected", file=sys.stderr, flush=True)


_DRAW = _Ground()

# -- scripted demo: open loop from the fixed layout ---------------------------------------------------------------------
# The right hand takes the leaning pestle by its handle 10 cm up from the head's end, fingers closing along x across
# the handle, the wrist rolled 43 degrees about x so the hand comes in from the right arm's side, nearly in line with
# the handle. It lifts the pestle 10 cm, turns the wrist back to straight down with the head above the rim, and stands
# the pestle on the corns in the middle of the bowl; let go, it settles upright on its flat 4 cm head. The hand takes
# it again lower down, 7.5 cm up from the head's end (the lowest grip the 14 cm pestle allows under the gripper's palm),
# straight down, so the push goes down the pestle. It aims the head 2 cm below the corns' resting surface (about
# 11 N). Then it works the hand round 33 circles, 2 s each, with its orientation held: a sideways pull on the head bends
# the arm 8 to 12 mm per newton (mostly the wrist servos), so the hand leads the head by 2.5 to 4 cm and circles 5 cm
# out, and the bowl's wall keeps the head within about 1.2 cm of the middle. Then it eases off and, still holding it,
# lays the pestle over toward the rim on -y: the wrist rolls 28 degrees while the head pivots on its edge, past
# the 19 degrees where the pestle would tip by itself. Let go there, it can only fall the rest of the way onto the
# rim, where it rests at about 34 degrees (never left standing on its flat head, where it would rock). The open hand
# slides 8 cm square to the fallen handle, out of the fingers' gap, and lifts clear. The left hand stays where it starts.
GRIP_L = 0.100  # m up the pestle from the head's end: where the hand holds it
GRASP_ROLL = math.radians(43.0)  # wrist roll about x at the grasp (the approach leans toward +y), 15 degrees off the handle
TURN_ROLL = 0.0  # the wrist straight down while the pestle is above the bowl: it leans 15 degrees
GRIND_ROLL = math.radians(-8.0)  # the wrist turned a little further down low in the bowl: the pestle leans 7 degrees
GRIP_YAW = math.pi / 2  # real grip_rotation: yaw 90 closes the fingers along x, out of the pestle's lean plane
GRIP_LOW = 0.075  # m up the standing pestle where the hand takes it again to grind
LAY = (math.radians(14.0), math.radians(28.0))  # the held pestle laid over toward the rim on -y before letting go
SLIDE_OFF = (0.080, 0.0)  # m: the open hand slides this far square to the fallen handle, out of the fingers' gap ...
LIFT_OFF = 0.08  # m ... then lifts straight up
PRESS = 0.020  # m the head's end is aimed below the corns' resting surface (the arm gives about 2.4 mm per N downward)
CIRCLE_R = 0.050  # m: the hand's circle; it leads the head by 2.5 to 4 cm, so the head is worked round the wall
CIRCLE_S = 2.0  # s per circle
CIRCLE_PTS = 8
ROUNDS = 33
# Held, the pestle sits about 3 degrees further over in the grip than planned and its handle about 4 mm toward -y of
# the tool point, so its head's middle is head_off_y(lean) toward +y of the tool point. Nearly upright while pressed,
# the push goes down the pestle instead of skating its head sideways over the corns.
SLIP = math.radians(3.0)
LIFT = 0.10  # m straight up before turning the pestle, so it turns clear of the bowl
CLEAR = 0.055  # m: the head this far above the corns is clear of the rim; the wrist turns only up here
BACK = 0.05  # m back along the approach before the grasp and after letting go


def axis(lean: float) -> np.ndarray:
    """The pestle's axis when it leans `lean` from upright toward -y."""
    return np.array([0.0, -math.sin(lean), math.cos(lean)])


def lean_for_roll(roll: float) -> float:
    """The pestle's lean while held, for a wrist roll: held as taken, it turns with the wrist."""
    return LEAN_REST + roll - GRASP_ROLL


GRIND_LEAN = lean_for_roll(GRIND_ROLL)  # 7 degrees


def head_off_y(lean: float) -> float:
    """How far toward +y of the tool point the held head's middle sits, at a planned lean."""
    return GRIP_L * math.sin(lean + SLIP) - 0.004


def approach(roll: float) -> np.ndarray:
    """The hand's approach direction (tool z) at a wrist roll about x."""
    return np.array([0.0, math.sin(roll), -math.cos(roll)])


def tcp_for_low(low_world, lean: float) -> tuple:
    """The tool point that puts the head's lowest point at `low_world` with the pestle leaning `lean`."""
    a = axis(lean)
    u = np.array([0.0, -math.cos(lean), -math.sin(lean)])  # from the head's center to its lowest point
    c = np.asarray(low_world, float) - HEAD_R * u if lean > 1e-9 else np.asarray(low_world, float)
    return tuple(float(v) for v in c + GRIP_L * a)


def demo():
    """Right hand: take the pestle, stand it on the corns, take it lower down, grind in circles under pressure, let go
    of it standing in the bowl."""
    g0 = C_REST + GRIP_L * axis(LEAN_REST)
    pre = g0 - BACK * approach(GRASP_ROLL)
    up = g0 + np.array([0.0, 0.0, LIFT])
    bed = np.array([MORTAR_XY[0], MORTAR_XY[1], TOP + BED_TOP])

    def T(d, z, r=GRIND_ROLL):
        """The tool point for the held, leaning head's middle at d from the bowl's middle, its lowest point z above the
        corns' resting surface, with the wrist at roll r."""
        lean = lean_for_roll(r)
        return (float(bed[0] + d[0]), float(bed[1] + d[1] - head_off_y(lean)),
                tcp_for_low(bed + np.array([0.0, 0.0, z]), lean)[2])

    def U(d, z, grip=GRIP_LOW):
        """The tool point on the upright pestle `grip` up from its head's end, the head's end z above the corns."""
        return (float(bed[0] + d[0]), float(bed[1] + d[1]), float(bed[2] + z + grip))

    def L(lean, over=0.0, up=0.0):
        """The tool point holding the upright-gripped pestle GRIP_LOW up from its head's end, leaned `lean` toward -y
        about the -y edge of its head (where that edge is when upright, 2 mm above the bowl's bottom); then moved
        `over` square to the handle in its lean plane (up and toward +y: the open fingers' gap runs that way, so the
        handle leaves it sideways), and `up`."""
        edge = bed + np.array([0.0, -HEAD_R, 0.002])
        u = np.array([0.0, -math.cos(lean), -math.sin(lean)])  # from the head's end middle to that edge
        n = np.array([0.0, math.cos(lean), math.sin(lean)])  # square to the handle, away from where it falls
        t = edge - HEAD_R * u + GRIP_LOW * axis(lean) + over * n + np.array([0.0, 0.0, up])
        return tuple(float(v) for v in t)

    roll, hi = GRIND_ROLL, TURN_ROLL
    P = lambda v: tuple(float(x) for x in v)  # noqa: E731
    G, Y = GRASP_ROLL, GRIP_YAW
    plan = [kit.Tilted(2.5, P(pre), FINGER_OPEN, "in line with the handle", yaw=Y, roll=G),
            kit.Tilted(1.5, P(g0), FINGER_OPEN, "around the handle", yaw=Y, roll=G),
            kit.Tilted(0.8, P(g0), FINGER_CLOSED, "grip", yaw=Y, roll=G),
            kit.Tilted(1.5, P(up), FINGER_CLOSED, "lift clear of the rim", yaw=Y, roll=G),
            kit.Tilted(2.0, T((0.0, 0.0), CLEAR, hi), FINGER_CLOSED, "turn it over the bowl", yaw=Y, roll=hi),
            kit.Tilted(1.5, T((0.0, 0.0), 0.012), FINGER_CLOSED, "over the corns", yaw=Y, roll=roll),
            kit.Tilted(1.5, T((0.0, 0.0), 0.0), FINGER_CLOSED, "stand it on the corns", yaw=Y, roll=roll),
            kit.Tilted(0.8, T((0.0, 0.0), 0.0), FINGER_OPEN, "let it settle upright", yaw=Y, roll=roll),
            kit.Tilted(1.0, U((0.0, 0.0), 0.0, GRIP_L + 0.01), FINGER_OPEN, "over the handle", yaw=Y, roll=0.0),
            kit.Tilted(1.5, U((0.0, 0.0), 0.0), FINGER_OPEN, "down the handle", yaw=Y, roll=0.0),
            kit.Tilted(0.8, U((0.0, 0.0), 0.0), FINGER_CLOSED, "grip low", yaw=Y, roll=0.0),
            kit.Tilted(1.0, U((CIRCLE_R, 0.0), -PRESS), FINGER_CLOSED, "press", yaw=Y, roll=0.0)]
    for k in range(ROUNDS):
        for i in range(1, CIRCLE_PTS + 1):
            t = 2 * math.pi * i / CIRCLE_PTS
            plan.append(kit.Tilted(CIRCLE_S / CIRCLE_PTS, U((CIRCLE_R * math.cos(t), CIRCLE_R * math.sin(t)), -PRESS),
                                   FINGER_CLOSED, f"grind, circle {k + 1}", yaw=Y, roll=0.0))
    plan += [kit.Tilted(1.0, U((0.0, 0.0), 0.002), FINGER_CLOSED, "ease off", yaw=Y, roll=0.0),
             *(kit.Tilted(1.0, L(tip), FINGER_CLOSED, "lay it over toward the rim", yaw=Y, roll=tip) for tip in LAY),
             kit.Tilted(1.0, L(LAY[-1]), FINGER_OPEN, "let go", yaw=Y, roll=LAY[-1]),
             kit.Tilted(1.0, L(LAY[-1], *SLIDE_OFF), FINGER_OPEN, "slide off the handle", yaw=Y, roll=LAY[-1]),
             kit.Tilted(1.0, L(LAY[-1], SLIDE_OFF[0], SLIDE_OFF[1] + LIFT_OFF), FINGER_OPEN, "lift clear", yaw=Y,
                        roll=LAY[-1]),
             kit.Tilted(2.0, L(LAY[-1], SLIDE_OFF[0], SLIDE_OFF[1] + LIFT_OFF), FINGER_OPEN, "wait", yaw=Y,
                        roll=LAY[-1])]
    return {"right": plan}
