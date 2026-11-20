"""On the station table: a clear 2.5-quart glass mixing bowl, about half full of cold tap water (1.2 L), in front of
the robot between its arms. To its left, three US Large eggs in brown shells lie side by side on a folded cotton dish
towel, their long axes pointing away from the robot. To its right stands a small clear glass ramekin, empty: the
discard dish. The eggs look alike, but one is fresh, one is a few weeks old and one is old enough to float.

The job is the kitchen float test, and the hard part is the handling: pick each egg off the towel with a light grip
and let it go only at the water's surface (dropped onto glass or squeezed hard, an egg can crack), then pick the
floater, a wet egg standing upright in the water with a few millimeters showing, out of the water with a light grip
and set it down in the ramekin, leaving the others in the bowl. Which egg floats is seen only once it is in: a fresh
egg sinks and lies on its side, an older one sinks and stands on end, a very old one floats (Utah State University
Extension; Egg Safety Center).

There is no fluid in the simulator. floatkit (beside this file) adds the water's buoyancy and drag to each egg before
every physics step; PhysX does every contact. An egg is its real shape and mass; an older egg is lighter for the same
shell, because its air cell at the blunt end has grown, and its center of mass sits further toward the narrow end
(eggmodel.py). Whether it lies, stands or floats comes out of that, not out of a script.
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import yamkit as kit  # noqa: E402
import guardkit as gk  # noqa: E402
import hullkit as hk  # noqa: E402
import floatkit as fk  # noqa: E402
import eggmodel as em  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20  # control step, s
INSTRUCTION = ("Lower each egg into the bowl of water without cracking it, then lift any egg that floats out into "
               "the small dish and leave the others in the bowl.")

# -- looks ----------------------------------------------------------------------------------------------------------
GLASS = kit.Look((0.86, 0.92, 0.94), 0.05, opacity=0.15)
WATER = kit.Look((0.62, 0.78, 0.90), 0.05, opacity=0.12)  # drawn only (no collider): floatkit gives its forces
SHELL = kit.Look((0.80, 0.62, 0.45), 0.55)  # brown eggshell
COTTON = kit.Look((0.80, 0.30, 0.25), 0.9)  # a red-checked cotton tea towel, drawn plain

# -- the bowl: Anchor Hocking 2.5 qt glass mixing bowl (listing: top 8 5/8 in, bottom 4 5/8 in, height 4 1/4 in) ----
BOWL_C = (0.320, 0.000)
BOWL_GLASS = 0.004  # m: wall thickness
BOWL_FLOOR_T = 0.006  # m: the thick pressed-glass base
BOWL_R0 = 0.117 / 2 - BOWL_GLASS - 0.0045  # m: inside radius at the floor (the outside foot is 4 5/8 in; the inside
# floor is smaller by the wall and the rounded corner)
BOWL_TOP_R = 0.219 / 2 - BOWL_GLASS  # m: inside radius at the rim (8 5/8 in outside)
BOWL_DEPTH = 0.108 - BOWL_FLOOR_T  # m: inside depth (4 1/4 in high)
BOWL_FLARE = (BOWL_TOP_R - BOWL_R0) / BOWL_DEPTH  # the inside wall's outward run per unit of height
BOWL_FLOOR_Z = TOP + BOWL_FLOOR_T
BOWL_RIM_Z = BOWL_FLOOR_Z + BOWL_DEPTH
WATER_L = 1.2  # L of cold tap water
BOWL_WALLS = 32  # the sloping side as 32 flat glass panels round the bowl

# -- the discard dish: Duralex Ovenchef 8.5 oz glass ramekin (listing: top 3 7/8 in, height 2 in) --------------------
DISH_C = (0.300, -0.205)
DISH_GLASS = 0.004
DISH_FLOOR_T = 0.006
DISH_TOP_R = 0.098 / 2 - DISH_GLASS
DISH_DEPTH = 0.051 - DISH_FLOOR_T
DISH_R0 = DISH_TOP_R - 0.008  # m: inside floor radius (its side tapers 8 mm)
DISH_FLARE = (DISH_TOP_R - DISH_R0) / DISH_DEPTH
DISH_FLOOR_Z = TOP + DISH_FLOOR_T
DISH_WALLS = 24

# -- the towel and the eggs -----------------------------------------------------------------------------------------
TOWEL_C, TOWEL_SIZE = (0.300, 0.230), (0.130, 0.220, 0.004)  # a cotton tea towel folded to 13 x 22 cm, 4 mm thick
TOWEL_TOP = TOP + TOWEL_SIZE[2]
SLOTS_Y = (0.160, 0.230, 0.300)  # where the three eggs lie on the towel (their middles, 7 cm apart; the hand at the
# first clears the bowl's rim), x = TOWEL_C[0]
EGG_NAMES = ("egg_1", "egg_2", "egg_3")  # by towel slot, nearest the bowl first
# air cell depth (m) for each kind: fresh (USDA grade AA, 1/8 in or less), older (grade B, over 3/16 in), floater
CELL = {"fresh": em.FRESH_CELL, "older": 0.008, "floater": 0.011}
KINDS = ("fresh", "older", "floater")
EGG_FRICTION = (0.4, 0.35)  # eggshell (static, dynamic); PhysX averages it with the other surface
EGG_CD = 0.5  # drag coefficient of the egg in water (a smooth ellipsoid at Reynolds numbers near 10^4)
EGG_CONTACT_OFFSET = 0.002  # m: the scene's 5 mm is large for a 4 cm egg
EGG_SOLVER_ITERS = (16, 4)
EGG_MAX_DEPENETRATION = 0.3  # m/s: PhysX's push-apart speed cap (solver stability, not an assist)


def trial_seed() -> int:
    """EGG_SEED=<n> repeats a run (0 is the canonical run). Unset, each run draws its own seed and prints it, so which
    towel slot holds which egg is not known in advance."""
    v = os.environ.get("EGG_SEED")
    if v is not None:
        try:
            return int(v)
        except ValueError:
            pass
    seed = int.from_bytes(os.urandom(4), "little") % 1_000_000 + 1
    print(f"test-egg-freshness: seed {seed} (EGG_SEED={seed} repeats this run)", file=sys.stderr, flush=True)
    return seed


def trial_kinds(seed: int) -> dict:
    """{egg name: kind} for a seed: the three kinds in a shuffled order along the towel (seed 0: older, floater,
    fresh)."""
    order = (1, 2, 0) if seed == 0 else tuple(np.random.default_rng([int(seed), 7]).permutation(3))
    return {n: KINDS[int(k)] for n, k in zip(EGG_NAMES, order)}


SEED = trial_seed()
EGG_KIND = trial_kinds(SEED)
FLOATER = next(n for n, k in EGG_KIND.items() if k == "floater")

# The egg's own frame: its origin at the center of mass, x along the long axis toward the blunt end. The shell's
# middle (of its length) sits at -com_x along x.


def egg_mid(kind: str) -> float:
    """m along the egg's axis from its center of mass to the middle of its length."""
    return -em.com_x(CELL[kind])


EGG_HULL = em.surface_points()  # about the middle of the length
EGG_CLOUD = em.volume_points(fk.GRID)
_PROFILE = [(float(em.radius(x)), float(x)) for x in
            (-em.EGG_L / 2 * math.cos(math.pi * k / 24) for k in range(25))]


def _egg(name: str, y: float) -> kit.Prop:
    kind = EGG_KIND[name]
    off = egg_mid(kind)
    lie_z = TOWEL_TOP + em.EGG_B / 2 + 0.001  # lying on its side, 1 mm over the towel (it settles)
    return kit.Prop(
        name, pos=(float(TOWEL_C[0] - off), float(y), float(lie_z)), size=(em.EGG_L, em.EGG_B, em.EGG_B),
        mass=float(em.mass(CELL[kind])), friction=EGG_FRICTION,
        meshes=(kit.lathe(_PROFILE, SHELL, center=(off, 0.0, 0.0), axis="x", n=32),),
        colliders=(kit.Collider((off, 0.0, 0.0), hull=tuple(map(tuple, EGG_HULL)), limit=kit.HULL_LIMIT),),
        com=(0.0, 0.0, 0.0), inertia=em.inertia(CELL[kind]), max_depenetration=EGG_MAX_DEPENETRATION, contact_report=True,
        contact_offset=EGG_CONTACT_OFFSET, solver_iterations=EGG_SOLVER_ITERS)


EGGS = [_egg(n, y) for n, y in zip(EGG_NAMES, SLOTS_Y)]


def _flared_vessel(name, c, r0, flare, depth, glass, floor_t, walls, look, water_z=None) -> kit.Prop:
    """A round glass vessel standing on the table: a floor disc and a ring of flat panels leaning out by the flare.
    Its frame: origin on the table under its middle."""
    tilt = math.atan(flare)
    slant = depth / math.cos(tilt)
    cols = [kit.Collider((0.0, 0.0, floor_t / 2), radius=r0 + glass, height=floor_t)]
    rm = r0 + flare * depth / 2  # inside radius halfway up
    w = 2 * math.pi * (rm + glass) / walls * 1.08  # panels overlap a little at their edges
    for k in range(walls):
        a = 2 * math.pi * k / walls
        # panel center: halfway up, its inner face on the inside surface
        cx, cy = (rm + glass / 2 * math.cos(tilt)) * math.cos(a), (rm + glass / 2 * math.cos(tilt)) * math.sin(a)
        cz = floor_t + depth / 2 - glass / 2 * math.sin(tilt)
        # turn: yaw a about z, then lean out by tilt about the panel's own tangent (its local y)
        qy = (0.0, math.sin(tilt / 2), 0.0, math.cos(tilt / 2))
        qz = (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))
        q = kit._quat_mul(qz, qy)
        cols.append(kit.Collider((cx, cy, cz), size=(glass, w, slant), quat=tuple(float(v) for v in q)))
    prof_in = [(r0, floor_t), (r0 + flare * depth, floor_t + depth)]
    prof_out = [(r0 + glass + 0.004, 0.0), (r0 + flare * depth + glass, floor_t + depth)]
    meshes = [kit.lathe([(0.0, floor_t)] + prof_in, look, n=48), kit.lathe([(0.0, 0.0)] + prof_out, look, n=48)]
    if water_z is not None:
        h = water_z - (TOP + floor_t)
        meshes.append(kit.lathe([(0.0, floor_t + h), (r0 + flare * h - 0.0005, floor_t + h)], WATER, n=48))
    R = r0 + flare * depth + glass
    return kit.Prop(name, pos=(float(c[0]), float(c[1]), float(TOP)), size=(2 * R, 2 * R, floor_t + depth), mass=None,
                    meshes=tuple(meshes), colliders=tuple(cols), friction=(0.5, 0.4))


VESSEL = fk.Vessel("bowl", center=BOWL_C, radius=BOWL_R0, floor_z=BOWL_FLOOR_Z, liquid_m3=WATER_L * 1e-3,
                   flare=BOWL_FLARE)


def still_surface() -> float:
    """The water's surface (world z) with no egg in it."""
    lo, hi = BOWL_FLOOR_Z, BOWL_RIM_Z
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if VESSEL.volume_below(mid) < WATER_L * 1e-3 else (lo, mid)
    return 0.5 * (lo + hi)


SURFACE0 = still_surface()
BOWL = _flared_vessel("bowl", BOWL_C, BOWL_R0, BOWL_FLARE, BOWL_DEPTH, BOWL_GLASS, BOWL_FLOOR_T, BOWL_WALLS, GLASS,
                      water_z=SURFACE0)
DISH = _flared_vessel("dish", DISH_C, DISH_R0, DISH_FLARE, DISH_DEPTH, DISH_GLASS, DISH_FLOOR_T, DISH_WALLS, GLASS)
TOWEL = kit.Prop("towel", pos=(float(TOWEL_C[0]), float(TOWEL_C[1]), float(TOP)), size=TOWEL_SIZE, mass=None,
                 meshes=(kit.box((0.0, 0.0, TOWEL_SIZE[2] / 2), TOWEL_SIZE, COTTON),),
                 colliders=(kit.Collider((0.0, 0.0, TOWEL_SIZE[2] / 2), size=TOWEL_SIZE),), friction=(0.8, 0.7))
OBJECTS = [BOWL, DISH, TOWEL] + EGGS

# -- run-time watch: speeds and contact forces, read at 240 Hz from the simulator ------------------------------------
V_DROP = 0.40  # m/s: an egg not held moving faster than this, in any direction: let fall more than 8 mm (a free fall
# from 8 mm reaches 0.396 m/s; more than half of eggs dropped 8 mm on end onto a hard support cracked: Sutanto et al.,
# Communications Physics 2025) or flung; and any egg (held or not) touching the glass, table, towel or an egg whose
# velocity changes by more than this in one physics step (rammed into it: about 10 g, which arm motion and a sinking
# egg's landing never reach)
F_CRUSH = 38.0  # N: an egg pressed harder than this cracks (rupture force across the equator 37.96-45.02 N between
# plates: Krivanek et al. 2016; 45 N in Sutanto et al. 2025). The press is the largest of: any one robot link's force on
# the egg, either arm's links together, both arms' links together (vector sums), and everything else together (glass,
# table, towel, the other eggs: the egg's net contact force less the robot's), the last being what a hand pressing it
# into the glass loads it with
CRUSH_STEPS = 3  # ... in 3 of any 4 physics steps in a row (1/80 s; a press let off for a step still counts)
F_STRIKE = 2 * F_CRUSH  # N: ... or above this on any single step. One step's reading is the impulse averaged over
# 1/240 s, so a sharp knock whose average reaches twice the rupture force peaked higher still
HOLD_N = 0.2  # N: an egg is held when both fingertips of one hand press it at least this hard, from opposite sides
HOLD_COS = -0.5  # ... their forces pointing against each other (cosine of the angle between them below this)
ENV_TOUCH_N = 0.02  # N: the egg touches something other than the robot when that contact force is above this
EGG_DEBUG = bool(os.environ.get("EGG_DEBUG"))


class Watch:
    """Per trial, from the simulator every physics step (240 Hz): each egg's fastest speed while not held, the speed
    at which it first touched anything other than the robot (held or not), and the hardest press on it."""

    def __init__(self):
        self.view = None
        self.reset()

    def reset(self):
        self.n = 0
        self.t_last = -1.0
        self.v_free = {n: 0.0 for n in EGG_NAMES}  # fastest while not held, or at a first touch (an impact)
        self.crush = {n: 0.0 for n in EGG_NAMES}
        self.peak = {n: 0.0 for n in EGG_NAMES}  # hardest press seen at all (telemetry)
        self.over = {n: 0 for n in EGG_NAMES}
        self.held = {n: False for n in EGG_NAMES}
        self.touch = {n: True for n in EGG_NAMES}  # touching something other than the robot (they start on the towel)
        self.v_prev = {n: np.zeros(3) for n in EGG_NAMES}  # last step's velocity (m/s, world)
        self.hist = {n: 0 for n in EGG_NAMES}  # the last 4 steps over F_CRUSH, as bits
        self.read = False

    def judge(self, name: str, vel, parts: dict, net=None) -> None:
        """One physics step for one egg: its velocity (m/s, world), {robot part: contact force vector on the egg (N)}
        with the parts named "<side>/<link>", and the egg's net contact force vector (all contacts)."""
        vec = {k: np.asarray(v, float) for k, v in parts.items()}
        zero = np.zeros(3)
        held = False
        for side in ("left", "right"):
            a, b = vec.get(f"{side}/tip_left", zero), vec.get(f"{side}/tip_right", zero)
            na, nb = float(np.linalg.norm(a)), float(np.linalg.norm(b))
            if na > HOLD_N and nb > HOLD_N and float(a @ b) / (na * nb) < HOLD_COS:
                held = True
        self.held[name] = held
        speed = float(np.linalg.norm(vel))
        if not held:
            self.v_free[name] = max(self.v_free[name], speed)
        robot = sum(vec.values(), zero)
        arms = [sum((v for k, v in vec.items() if k.startswith(side + "/")), zero) for side in ("left", "right")]
        env = (np.asarray(net, float) - robot) if net is not None else zero
        touch = float(np.linalg.norm(env)) > ENV_TOUCH_N
        vel = np.asarray(vel, float)
        if touch and not self.touch[name]:  # first touch of the glass, table, towel or an egg: the speed it hit at
            self.v_free[name] = max(self.v_free[name], float(np.linalg.norm(self.v_prev[name])))
        elif touch:  # already touching (held or not) and rammed on: its velocity changes by over V_DROP in one step
            dv = float(np.linalg.norm(vel - self.v_prev[name]))
            if dv > V_DROP:
                self.v_free[name] = max(self.v_free[name], dv)
        self.touch[name], self.v_prev[name] = touch, vel
        f = max([float(np.linalg.norm(v)) for v in vec.values()] +
                [float(np.linalg.norm(x)) for x in (robot, env, *arms)])
        self.peak[name] = max(self.peak[name], f)
        self.hist[name] = ((self.hist[name] << 1) | int(f > F_CRUSH)) & 0b1111
        self.over[name] = bin(self.hist[name]).count("1")
        if self.over[name] >= CRUSH_STEPS or f > F_STRIKE:
            self.crush[name] = max(self.crush[name], f)


WATCH = Watch()


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    if hasattr(x, "numpy"):
        return np.asarray(x.numpy(), np.float64)
    return np.asarray(x, np.float64)


_arr = _np


class ContactReader:
    """The robot-to-egg contact forces from PhysX: a rigid contact view on the three eggs (their USD carries
    PhysxContactReportAPI), filtered by every rigid link of both arms. Built at the first physics step; anything
    missing is an error, never a silent pass."""

    def __init__(self, base):
        import isaaclab.sim as sim_utils
        from isaaclab_physx.physics import PhysxManager
        from pxr import Usd, UsdPhysics

        eggs = []
        for n in EGG_NAMES:
            prims = sim_utils.find_matching_prims(base.scene[n].cfg.prim_path)
            if not prims:
                raise RuntimeError(f"[egg] no prim for {n}")
            eggs.append(prims[0].GetPath().pathString)
        self.parts = []
        paths = []
        for side, robot in (("left", "left_robot"), ("right", "right_robot")):
            root = sim_utils.find_matching_prims(base.scene[robot].cfg.prim_path)[0]
            for prim in Usd.PrimRange(root):
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    self.parts.append(f"{side}/{prim.GetName()}")
                    paths.append(prim.GetPath().pathString)
        for side in ("left", "right"):
            for tip in ("tip_left", "tip_right"):
                if f"{side}/{tip}" not in self.parts:
                    raise RuntimeError(f"[egg] no {tip} link on the {side} arm (links {self.parts})")
        view = PhysxManager.get_physics_sim_view()
        self.view = view.create_rigid_contact_view(eggs, filter_patterns=[paths] * len(eggs),
                                                   max_contact_data_count=16 * len(eggs))
        self.dt = float(getattr(base, "physics_dt", 1 / 240))
        print(f"[egg] contact view: {len(eggs)} eggs x {len(paths)} robot links", flush=True)

    def forces(self) -> tuple:
        """((egg, robot part, 3) contact force vectors on each egg, (egg, 3) each egg's net contact force), N, over the
        last physics step."""
        m = _np(self.view.get_contact_force_matrix(dt=self.dt)).reshape(len(EGG_NAMES), len(self.parts), 3)
        net = _np(self.view.get_net_contact_forces(dt=self.dt)).reshape(len(EGG_NAMES), 3)
        return m, net


def _watch(base, origin, dt) -> None:
    w = WATCH
    t = fk.state().t
    if t < w.t_last:
        w.reset()
    w.t_last = t
    w.n += 1
    if w.view is None:
        w.view = ContactReader(base)
    F, net = w.view.forces()
    for i, n in enumerate(EGG_NAMES):
        vel = _np(base.scene[n].data.root_lin_vel_w)[0]  # a free body's own velocity (PhysX)
        w.judge(n, vel, dict(zip(w.view.parts, F[i])), net[i])
    if not w.read:
        w.read = True
        print("[egg] watching speeds and contact forces at 240 Hz", flush=True)
    if EGG_DEBUG and w.n % 120 == 0:  # every 0.5 s: each egg's middle, lean and record; the water's surface
        msg = []
        for n in EGG_NAMES:
            st = base.scene[n].data
            p = _np(st.root_pos_w)[0] - origin
            Rq = fk.quat_R(_np(st.root_quat_w)[0])
            mid = p + Rq @ np.array([egg_mid(EGG_KIND[n]), 0.0, 0.0])
            up = math.degrees(math.asin(max(-1.0, min(1.0, float(Rq[2, 0])))))
            msg.append(f"{n}({EGG_KIND[n][:2]}) {mid[0]:.3f},{mid[1]:.3f},{mid[2] - TOP:.3f} up {up:+.0f} "
                       f"v {w.v_free[n]:.2f} f {w.peak[n]:.0f}/{w.crush[n]:.0f}{' H' if w.held[n] else ''}")
        print(f"[egg] t={t:.1f} surf {fk.surface_z() - TOP:.4f} " + " | ".join(msg), flush=True)


def setup_water() -> None:
    fk.setup(VESSEL,
             [fk.Body(n, size=(em.EGG_L, em.EGG_B, em.EGG_B), mass=float(em.mass(CELL[EGG_KIND[n]])), cd=EGG_CD,
                      shape="cloud", cloud=tuple(map(tuple, EGG_CLOUD + [egg_mid(EGG_KIND[n]), 0.0, 0.0])),
                      vol=em.VOLUME)
              for n in EGG_NAMES],
             drift=None, monitors=(_watch,))


setup_water()

# -- what the scorer reads ------------------------------------------------------------------------------------------
TESTED_S = 1.0  # s: an egg has been tested once it has sat in the bowl's water, let go, for 1 s
FLOOR_UP = 0.012  # m: an egg on the bowl's or the dish's floor has its lowest point within 1.2 cm of it
STILL_V = 0.01  # m/s: at rest
CLEAR = 0.02  # m: hands clear = every arm link 2 cm or more from every egg
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)
OFF_TABLE = TOP - 0.05


def _egg_parts(name: str) -> tuple:
    """The egg as balls along its axis (its own frame), for contact and clearance with the arm's hulls."""
    off = egg_mid(EGG_KIND[name])
    xs = np.linspace(-em.EGG_L / 2 + 0.006, em.EGG_L / 2 - 0.006, 9)
    return tuple(gk.Seg((float(x + off), 0.0, 0.0), (float(x + off), 0.0, 0.0), float(em.radius(x))) for x in xs)


EGG_PARTS = {n: _egg_parts(n) for n in EGG_NAMES}


def egg_axis(objects, name: str) -> np.ndarray:
    return fk.quat_R(objects[name].quat_xyzw)[:, 0]


def egg_low_high(objects, name: str) -> tuple:
    """The egg's lowest and highest points (world z), from its shell points."""
    s = objects[name]
    z = (np.asarray(s.pos, float) + (EGG_HULL + [egg_mid(EGG_KIND[name]), 0.0, 0.0]) @ fk.quat_R(s.quat_xyzw).T)[:, 2]
    return float(z.min()), float(z.max())


def egg_mid_world(objects, name: str) -> np.ndarray:
    s = objects[name]
    return np.asarray(s.pos, float) + fk.quat_R(s.quat_xyzw) @ np.array([egg_mid(EGG_KIND[name]), 0.0, 0.0])


def in_bowl(objects, name: str) -> bool:
    """Its middle inside the bowl's inside wall, below the rim."""
    p = egg_mid_world(objects, name)
    return bool(BOWL_FLOOR_Z < p[2] < BOWL_RIM_Z and
                math.hypot(p[0] - BOWL_C[0], p[1] - BOWL_C[1]) < float(VESSEL.radius_at(p[2])))


def in_water(objects, name: str) -> bool:
    return bool(in_bowl(objects, name) and egg_mid_world(objects, name)[2] < fk.surface_z())


def on_bowl_floor(objects, name: str) -> bool:
    return bool(in_bowl(objects, name) and egg_low_high(objects, name)[0] < BOWL_FLOOR_Z + FLOOR_UP)


def in_dish(objects, name: str) -> bool:
    p = egg_mid_world(objects, name)
    r = math.hypot(p[0] - DISH_C[0], p[1] - DISH_C[1])
    return bool(r < DISH_R0 + DISH_FLARE * max(p[2] - DISH_FLOOR_Z, 0.0)
                and egg_low_high(objects, name)[0] < DISH_FLOOR_Z + FLOOR_UP)


def touched(objects, name: str) -> bool:
    return bool(hk.touching(objects, name, EGG_PARTS[name]))


def hands_clear(objects) -> bool:
    return all(hk.gap(objects, g, n, EGG_PARTS[n], upto=CLEAR) >= CLEAR
               for g in hk.GRIPPERS if g in objects for n in EGG_NAMES)


class Ledger:
    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.objs = None
        self.n = 0
        self.wet = {n: 0 for n in EGG_NAMES}  # consecutive steps in the water, let go
        self.tested = {n: False for n in EGG_NAMES}
        self.off_table = False
        self.prev = None


_L = Ledger()


def _update(objects) -> None:
    """Advance the trial's record once per control step (success() and score() share it)."""
    global _L
    if _L.episode != kit.episode():
        _L = Ledger()
    n = kit._EPISODE.get("last")
    key = (kit.episode(), n) if n is not None else None
    if (key is not None and key == _L.key) or (key is None and objects is _L.objs):
        return
    _L.key, _L.objs = key, objects
    _L.n += 1
    for name in EGG_NAMES:
        if in_water(objects, name) and not touched(objects, name):
            _L.wet[name] += 1
            if _L.wet[name] >= round(TESTED_S / DT):
                _L.tested[name] = True
        else:
            _L.wet[name] = 0
        if objects[name].pos[2] < OFF_TABLE:
            _L.off_table = True


def failure() -> str:
    for n in EGG_NAMES:
        if WATCH.v_free[n] > V_DROP:
            return f"{n} let fall, flung or knocked down ({WATCH.v_free[n]:.2f} m/s)"
        if WATCH.crush[n] > F_CRUSH:
            return f"{n} crushed ({WATCH.crush[n]:.0f} N)"
    if _L.off_table:
        return "an egg fell off the table"
    return ""


def stage(objects) -> float:
    if failure():
        return 0.0
    if not all(_L.tested.values()):
        return 0.0
    good = [n for n in EGG_NAMES if n != FLOATER]
    kept = all(on_bowl_floor(objects, n) and not touched(objects, n) for n in good)
    if not kept or in_bowl(objects, FLOATER):
        return 0.25
    if not (in_dish(objects, FLOATER) and not touched(objects, FLOATER)):
        return 0.5
    still = all(np.linalg.norm(objects[n].lin_vel) < STILL_V for n in EGG_NAMES)
    if not (still and hands_clear(objects)):
        return 0.75
    return 1.0


def score(objects) -> float:
    """0 if at any time an egg not held (both fingertips of one hand pressing it from opposite sides) moved faster than
    0.40 m/s in any direction, or an egg, held or not, first touched the glass, table, towel or another egg faster than
    that, or while touching one changed velocity by more than that in one physics step (let fall more than 8 mm, flung,
    knocked or rammed); an egg was pressed harder than 38 N in 3 of any 4 physics steps (1/80 s) or 76 N in one (by one
    robot link, either arm's links, both arms' links, or the glass and table together); or an egg left the table.
    Otherwise 0 until every egg has been tested (sat in the bowl's water, not touched, for 1 s);
    0.25 once all three have; 0.5 and the floater out of the bowl with the two others lying or standing on the bowl's
    floor, not touched; 0.75 and the floater on the ramekin's floor, not touched; 1.0 and every egg at rest with every
    link of both arms 2 cm or more from every egg (held 1 s for success)."""
    _update(objects)
    return stage(objects)


def success(objects) -> bool:
    _update(objects)
    return stage(objects) >= 1.0


# == scripted demo ===================================================================================================
# The left hand puts the eggs in, one at a time from the towel; the right hand then lifts the floater out and sets it
# in the ramekin. Both plans are fixed waypoints written for the canonical run (EGG_SEED=0); neither reads anything
# from the simulator.
O = FINGER_OPEN
GRIP_SQUEEZE = 0.0016  # m per finger: the pads are held this much inside the egg's breadth (a gentle grip)
GRIP_Q = -(em.EGG_B / 2 - GRIP_SQUEEZE)  # finger joint target for a grip across the egg's breadth
FLOAT_UP = 0.005  # m: the floater is gripped this far above its widest section (toward its blunt end, which is up):
# the glass flares outward, so the fingers clear it by 3 mm more there, and the egg is only 0.7 mm narrower
FLOAT_B = 2 * float(em.radius(em.EGG_W + FLOAT_UP))  # m: the floater's breadth where it is gripped
NEAR_Q = -(FLOAT_B / 2 + 0.008)  # fingers opened 8 mm wider than the egg each side: going down round the floater
FLOAT_Q = -(FLOAT_B / 2 - GRIP_SQUEEZE)  # finger joint target for the floater's grip (the same gentle squeeze)
TOWEL_Q = -(em.EGG_B / 2 + 0.008)  # fingers opened 8 mm wider than the egg each side: going down between the eggs
SET_Q = -(em.EGG_B / 2 + 0.006)  # ... and 6 mm: letting go inside the ramekin (fully open, they'd meet its wall)
PAD_HALF = 0.008  # m: a pad's half-thickness beyond its face (for the clearance of the fingers going down)
CARRY_Z = BOWL_RIM_Z + 0.045  # tool point height for carrying an egg over the rim
DROP_SPOTS = ((BOWL_C[0] - 0.025, BOWL_C[1] + 0.040), (BOWL_C[0] + 0.025, BOWL_C[1] + 0.030),
              (BOWL_C[0] - 0.020, BOWL_C[1] - 0.020))  # where each egg goes in (its middle), egg_1..3
DROP_YAW = (0.0, -math.pi / 2, -math.pi / 4)  # the hand's turn as it lets go (0: the egg's axis along x)
DIP = 0.012  # m: an egg is let go with its underside this far under the surface (fingertips just over it)
T_LOOK = 31.0  # s: the right hand sets off for the floater once the left hand is up out of the bowl (by 29.6 s)
DISH_GAP = 0.004  # m: the floater is let go with its lowest point this far over the dish's floor


def _left_plan() -> list:
    plan = []
    for i, name in enumerate(EGG_NAMES):
        x, y = TOWEL_C[0], SLOTS_Y[i]
        zc = TOWEL_TOP + em.EGG_B / 2  # the egg's axis height lying on the towel
        dx, dy = DROP_SPOTS[i]
        yaw = DROP_YAW[i]
        zr = SURFACE0 - DIP + em.EGG_B / 2
        plan += [kit.Tilted(1.6 if i == 0 else 1.4, (x, y, zc + 0.07), TOWEL_Q, f"over {name}"),
                 kit.Tilted(1.2, (x, y, zc), TOWEL_Q, f"around {name}"),
                 kit.Tilted(0.8, (x, y, zc), GRIP_Q, f"grip {name}"),
                 kit.Tilted(1.2, (x, y, CARRY_Z), GRIP_Q, "lift"),
                 kit.Tilted(1.8, (dx, dy, CARRY_Z), GRIP_Q, "over the bowl", yaw=yaw),
                 kit.Tilted(1.8, (dx, dy, zr), GRIP_Q, "lower to the water", yaw=yaw),
                 kit.Tilted(0.6, (dx, dy, zr), O, "let go", yaw=yaw),
                 kit.Tilted(1.0, (dx, dy, CARRY_Z), O, "up", yaw=yaw)]
    plan += [kit.Home(2.0)]
    return plan


# The right hand's plan is fixed: it is written for the canonical run (EGG_SEED=0), where the floater (egg_2, let go
# at the second drop spot) surfaces and settles with its middle at x 34.5, y -4.1 cm, 6.36 cm above the table, standing
# blunt end up (7 of 7 seed-0 probes, to the millimeter, from 34 s on). The plan reads nothing from the simulator; in
# another seed the hand closes where the floater is in the canonical run and lifts what is there (if nothing: 0.25).
FLOATER_XY = (0.345, -0.041)  # m: where the canonical run's floater has settled by the time the hand goes down round it
FLOATER_MID_Z = TOP + 0.0636  # m: its middle's height as it floats (blunt end up, about 5 mm showing)
FLOATER_YAW = math.radians(-50)  # the fingers' turn: closing line clear of the glass and of the eggs on the floor
RIGHT_PARK = (DISH_C[0] - 0.04, DISH_C[1] - 0.02, CARRY_Z + 0.05)  # the right wrist waits well clear of the left arm


def _right_plan() -> list:
    """Wait until the left hand is up out of the bowl, go down round the floater with the fingers 8 mm wider than it
    each side, close on it 5 mm above its widest section, lift it out slowly and set it on the ramekin's floor."""
    x, y = FLOATER_XY
    z = FLOATER_MID_Z + em.EGG_W + FLOAT_UP  # the tool point: on its axis, 5 mm above its widest section
    dz = DISH_FLOOR_Z + DISH_GAP + (em.EGG_W + FLOAT_UP + em.EGG_L / 2)  # its lowest point DISH_GAP over the floor
    yw = FLOATER_YAW
    return [kit.Tilted(T_LOOK, RIGHT_PARK, O, "wait"),
            kit.Tilted(2.0, (x, y, CARRY_Z), NEAR_Q, "over the floater", yaw=yw),
            kit.Tilted(1.6, (x, y, z), NEAR_Q, "fingers round it", yaw=yw),
            kit.Tilted(0.8, (x, y, z), FLOAT_Q, "grip", yaw=yw),
            kit.Tilted(2.5, (x, y, CARRY_Z), FLOAT_Q, "lift it out slowly", yaw=yw),
            kit.Tilted(2.0, (DISH_C[0], DISH_C[1], CARRY_Z), FLOAT_Q, "over the dish"),
            kit.Tilted(1.5, (DISH_C[0], DISH_C[1], dz), FLOAT_Q, "lower into the dish"),
            kit.Tilted(0.6, (DISH_C[0], DISH_C[1], dz), SET_Q, "let go"),
            kit.Tilted(1.0, (DISH_C[0], DISH_C[1], CARRY_Z), SET_Q, "up"),
            kit.Home(2.0)]


def demo():
    """The left hand takes each egg from the towel by its breadth, carries it over the rim and lowers it until its
    underside is 1.2 cm under the surface, and lets go; the right hand then takes the floater out to the ramekin."""
    return {"left": _left_plan(), "right": _right_plan()}
