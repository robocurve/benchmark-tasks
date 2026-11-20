"""Take the pieces off a vegetable kebab onto a plate with a fork: one hand holds the skewer by its bare end, the other pushes the pieces off the point with the fork; then the bare skewer is put down off the plate.

Scene (station frame, metres; x forward from the midpoint between the arm bases, y to the robot's left, z up from the bench top).
A bamboo skewer (300 x 3 mm) lies across a 26 cm dinner plate along y (point toward the right arm's base), its four pierced vegetable pieces resting on the plate's floor: from the
point, a red bell pepper square, a red onion block, a courgette round and a button mushroom. Its bare blunt end reaches over the rim onto the board. A stainless dinner fork
(IKEA FOERNUFT, 190 mm) lies on the board at the front right, resting on its tines and handle end like any table fork.
Each piece is a free rigid body on the skewer by a joint made on the live stage: locked in every direction but the slide along the stick, where a drive caps the force at the
piece's grip on the stick (`GRIP_N`); the joint is taken off when the piece's rear face passes the point, and the piece then falls free onto the plate.

Debug:  python demokit.py task.py OUT --no-record --mode full,none,one_arm
Final:  python demokit.py task.py OUT
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import demokit as d  # noqa: E402
import gatekit as gk  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
INSTRUCTION = ("Take the vegetables off the kebab onto the plate with the fork: hold the skewer by its bare end and push the pieces off its point with the fork, "
               "then put the empty skewer down on the board beside the plate. Do not touch the vegetables with your grippers.")
SUCCESS_HOLD_S = 0.0  # the scorer holds the end state itself
SCORE_OVER = "end"
TOP = d.TABLE_TOP
X, Y, Z = np.eye(3)

# -- sizes -----------------------------------------------------------------------------------------------------------------------------------
SK_L, SK_W = 0.300, 0.003  # bamboo skewer 12 in x 3 mm (Roland 71516)
SK_X0, SK_X1 = 0.22, 0.52  # blunt end, point: distances along the stick, from its blunt end (the stick runs toward -y)
SK_C = (SK_X0 + SK_X1) / 2
SK_YAW = -90.0
STK_X, STK_Y = 0.34, -0.028  # the stick's middle in the station frame


def sw(s: float, v: float = 0.0) -> tuple:
    """A point `s` along the stick (SK_X0 at the blunt end), `v` beside it, in the station frame."""
    return (STK_X + v, STK_Y - (s - SK_C))

PLATE_C = np.array([0.33, -0.16])
PLATE_R, PLATE_RIM_IN, PLATE_T, RIM_H = 0.130, 0.121, 0.008, 0.020
ON_PLATE_R = 0.112
ZS = TOP + PLATE_T + 0.0175 + 0.0005  # the skewer's axis: the 35 mm mushroom rests on the plate's floor
SK_MASS = float(os.environ.get("KB_SKM", "0.010"))
GRIP_N = float(os.environ.get("KB_GRIP", "1.0"))  # N to slide a piece along the stick (estimate)
DAMP = 4000.0  # N s/m: the drive saturates at GRIP_N above 0.25 mm/s
# pieces, from the point: name, thickness along the stick, y size, z size, mass
PIECES = (("pepper", 0.006, 0.030, 0.030, 0.006), ("onion", 0.018, 0.025, 0.030, 0.012), ("courgette", 0.012, 0.032, 0.032, 0.009), ("mushroom", 0.030, 0.035, 0.035, 0.015))
NAMES = [p[0] for p in PIECES]
X_END = SK_X1 - 0.040  # the run ends 40 mm from the point
PX = {}
_x = X_END
for _n, _t, *_ in PIECES:
    PX[_n] = _x - _t / 2
    _x -= _t
RUN_X0 = _x  # the run's rear face
FORK_REST = np.array([0.10, -0.10])
FORK_YAW = float(os.environ.get("KB_YAW", "-35"))  # the fork's heading while lying and while pushing, degrees from +x (tines forward and to the left)

LOOKS = {
    "bamboo": m.Look(m.rgb(0.80, 0.66, 0.40), roughness=0.7),
    "pepper": m.Look(m.rgb(0.80, 0.07, 0.06), roughness=0.35),
    "onion": m.Look(m.rgb(0.55, 0.15, 0.40), roughness=0.5),
    "onion_pale": m.Look(m.rgb(0.93, 0.85, 0.88), roughness=0.5),
    "courgette": m.Look(m.rgb(0.20, 0.45, 0.12), roughness=0.4),
    "flesh": m.Look(m.rgb(0.90, 0.92, 0.70), roughness=0.5),
    "mushroom": m.Look(m.rgb(0.80, 0.68, 0.52), roughness=0.5),
    "stem": m.Look(m.rgb(0.93, 0.90, 0.82), roughness=0.5),
    "plate": m.Look(m.rgb(0.95, 0.95, 0.93), roughness=0.25),
    "steel": m.Look(m.rgb(0.78, 0.79, 0.82), roughness=0.15, metallic=0.95),
}


def _aim(mesh):
    return mesh.aimed("+X")


# -- the plate ------------------------------------------------------------------------------------------------------------------------------
def _plate():
    cols = [m.col_cylinder("floor", PLATE_R, PLATE_T, (0.0, 0.0, PLATE_T / 2))]
    parts = [("floor", m.cylinder(PLATE_R, PLATE_T, (0.0, 0.0, PLATE_T / 2), segments=64), "plate")]
    n = 28
    seg = 2 * math.pi * (PLATE_RIM_IN + 0.004) / n * 1.05
    for k in range(n):
        a = 2 * math.pi * k / n
        c = (math.cos(a) * (PLATE_RIM_IN + 0.004), math.sin(a) * (PLATE_RIM_IN + 0.004), PLATE_T + (RIM_H - PLATE_T) / 2)
        box = m.box((0.008, seg, RIM_H - PLATE_T)).turned(math.degrees(a), "z").moved(c)
        parts.append((f"rim{k}", box, "plate"))
        cols.append(m.col_hull(f"rim{k}", box))
    return m.fixed("plate", (PLATE_C[0], PLATE_C[1], TOP), parts, cols, LOOKS)


# -- the skewer ----------------------------------------------------------------------------------------------------------------------------------
def _skewer():
    prof = [(0.0, -SK_L / 2), (SK_W / 2, -SK_L / 2), (SK_W / 2, SK_L / 2 - 0.015), (0.0004, SK_L / 2)]
    body = _aim(m.lathe(prof, 12))
    return m.free("skewer", (STK_X, STK_Y, ZS), [("stick", body, "bamboo")], [m.col_box("stick", (SK_L, SK_W, SK_W))], LOOKS, mass=SK_MASS, friction=(0.7, 0.6), torsional_patch=None, solver_iters=64, yaw_deg=SK_YAW)


HOLE = 0.002  # each piece has a square hole 4 mm across for the 3 mm stick: its collider is four convex pieces round it, so nothing overlaps the stick at spawn


def _ringed(name: str, mesh):
    """The piece's collider as four convex hulls (above, below, left and right of the hole): the mesh's points clamped to each side of the hole."""
    cols = []
    for k, (ax, sg) in enumerate(((2, 1), (2, -1), (1, 1), (1, -1))):
        v = mesh.v.copy()
        v[:, ax] = np.maximum(v[:, ax], HOLE) if sg > 0 else np.minimum(v[:, ax], -HOLE)
        cols.append(m.col_hull(f"{name}{k}", m.Mesh(v, mesh.f)))
    return cols


def _piece(name, t, wy, wz, mass):
    x = PX[name]
    if name == "pepper":
        body = m.box((t, wy, wz))
        parts = [("body", body, "pepper")]
    elif name == "onion":
        body = m.box((t, wy, wz))
        parts = [(f"l{i}", m.box((t / 3 - 0.0003, wy, wz), (-t / 3 + i * t / 3, 0.0, 0.0)), "onion" if i != 1 else "onion_pale") for i in range(3)]
    elif name == "courgette":
        body = _aim(m.lathe([(0.0, -t / 2), (wy / 2, -t / 2), (wy / 2, t / 2), (0.0, t / 2)], 40))
        flesh = _aim(m.lathe([(0.0, -t / 2 - 0.0003), (wy / 2 - 0.0025, -t / 2 - 0.0003), (wy / 2 - 0.0025, t / 2 + 0.0003), (0.0, t / 2 + 0.0003)], 40))
        parts = [("rim", body, "courgette"), ("flesh", flesh, "flesh")]
    else:
        r = wy / 2
        body = _aim(m.lathe([(0.0, -t / 2), (r, -t / 2), (r, -t / 2 + 0.006), (0.86 * r, -t / 2 + 0.017), (0.5 * r, t / 2 - 0.004), (0.0, t / 2)], 32))
        stem = _aim(m.lathe([(0.0, -t / 2 - 0.0004), (0.0065, -t / 2 - 0.0004), (0.0065, -t / 2 + 0.007), (0.0, -t / 2 + 0.007)], 16))
        parts = [("cap", body, "mushroom"), ("stem", stem, "stem")]
    return m.free(name, (*sw(x), ZS), parts, _ringed("c", body), LOOKS, mass=mass, friction=(0.7, 0.6), torsional_patch=None, solver_iters=32, yaw_deg=SK_YAW)


# -- the fork: a stainless dinner fork (IKEA FOERNUFT, 190 mm), four tines 2.5 mm wide on 7.5 mm centres, a 25 mm head, a neck rising 8 mm and a 3 mm handle that widens
# from 7 mm at the neck to 16 mm at its rounded end and slopes back down to the board, so the fork lies on its tines and its handle end like any table fork.
# Its local x runs toward the tines, origin on the board under the neck.
F_END, F_NECK, F_SHOULDER, F_HEAD1, F_TIP = -0.100, -0.010, 0.020, 0.045, 0.090
NECK_Z = 0.008  # the handle's underside at the neck, over the board
F_TH_H, F_TH_T = 0.003, 0.002  # handle and head/tine thickness
F_TINES_Y = (-0.01125, -0.00375, 0.00375, 0.01125)
FORK_MASS = 0.045
GRAB_X = -0.055  # the right hand takes the handle here


def handle_z(x: float) -> float:
    """The handle's underside over the board at `x`: from 0 at its end up to NECK_Z at the neck."""
    return NECK_Z * (x - F_END) / (F_NECK - F_END)


def _sheared(mesh, x0: float, z0: float, x1: float, z1: float):
    """The mesh lifted by a height that runs linearly from z0 at x0 to z1 at x1 (a straight bent section; a shear keeps a convex part convex)."""
    v = mesh.v.copy()
    v[:, 2] += z0 + (z1 - z0) * (v[:, 0] - x0) / (x1 - x0)
    return m.Mesh(v, mesh.f)


def _fork():
    parts, cols = [], []
    hw = lambda x: 0.0035 + (0.008 - 0.0035) * (F_NECK - x) / (F_NECK - F_END + 0.006)  # half width
    xs = [F_NECK, -0.040, -0.070, -0.088]
    outline = [(x, -hw(x)) for x in xs] + [(-0.096, -0.0075), (F_END, -0.004), (F_END, 0.004), (-0.096, 0.0075)] + [(x, hw(x)) for x in reversed(xs)]
    handle = _sheared(m.prism(outline[::-1], 0.0, F_TH_H), F_END, 0.0, F_NECK, NECK_Z)  # counter-clockwise
    parts.append(("handle", handle, "steel"))
    cols.append(m.col_hull("handle", handle))
    neck = _sheared(m.prism([(F_NECK, -0.0035), (0.008, -0.006), (F_SHOULDER, -0.0125), (F_SHOULDER, 0.0125), (0.008, 0.006), (F_NECK, 0.0035)], 0.0, F_TH_T), F_NECK, NECK_Z, F_SHOULDER, 0.0)
    parts.append(("neck", neck, "steel"))
    cols.append(m.col_hull("neck", neck))
    head = m.prism([(F_SHOULDER, -0.0125), (F_HEAD1, -0.0125), (F_HEAD1, 0.0125), (F_SHOULDER, 0.0125)], 0.0, F_TH_T)
    parts.append(("head", head, "steel"))
    cols.append(m.col_hull("head", head))
    for i, ty in enumerate(F_TINES_Y):
        tine = m.prism([(F_HEAD1 - 0.001, ty - 0.00125), (F_TIP - 0.006, ty - 0.00125), (F_TIP, ty), (F_TIP - 0.006, ty + 0.00125), (F_HEAD1 - 0.001, ty + 0.00125)], 0.0, F_TH_T)
        parts.append((f"tine{i}", tine, "steel"))
        cols.append(m.col_hull(f"tine{i}", tine))
    return m.free("fork", (FORK_REST[0], FORK_REST[1], TOP + 0.0002), parts, cols, LOOKS, mass=FORK_MASS, friction=(0.6, 0.5), torsional_patch=0.004, solver_iters=32, yaw_deg=FORK_YAW)


OBJECTS = [_plate(), _skewer()] + [_piece(*p) for p in PIECES] + [_fork()]


# -- the pieces' joints, made on the live stage --------------------------------------------------------------------------------------------------
JOINT_AT_STEP = 4


def _path(n: str) -> str:
    return f"{m.ENV_PATH}/{n}"


def _jpath(n: str) -> str:
    return f"{_path(n)}/on_skewer"


def drop_joints() -> None:
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    for n in NAMES:
        p = stage.GetPrimAtPath(_jpath(n))
        if p.IsValid():
            stage.RemovePrim(p.GetPath())


def make_joints() -> None:
    """Piece n sits on the skewer: a joint locked in every direction but the slide along the stick (x), whose drive is a damper capped at GRIP_N, so it slides under a push of more than
    GRIP_N and holds below it; the slide's end stops are a little behind the run's rear and a little past the point."""
    import omni.usd
    from pxr import Gf, Sdf, UsdPhysics

    stage = omni.usd.get_context().get_stage()
    for n, t, *_ in PIECES:
        path = Sdf.Path(_jpath(n))
        j = UsdPhysics.Joint.Define(stage, path)
        j.CreateBody0Rel().SetTargets([Sdf.Path(_path("skewer"))])
        j.CreateBody1Rel().SetTargets([Sdf.Path(_path(n))])
        j.CreateLocalPos0Attr(Gf.Vec3f(PX[n] - SK_C, 0.0, 0.0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateCollisionEnabledAttr().Set(False)
        prim = j.GetPrim()
        for ax in ("transY", "transZ", "rotX", "rotY", "rotZ"):
            lim = UsdPhysics.LimitAPI.Apply(prim, ax)
            lim.CreateLowAttr(1.0)
            lim.CreateHighAttr(-1.0)
        lim = UsdPhysics.LimitAPI.Apply(prim, "transX")
        lim.CreateLowAttr(float(-(PX[n] - SK_X0) + t / 2 + 0.002))
        lim.CreateHighAttr(float(SK_X1 - PX[n] + t / 2 + 0.02))
        drv = UsdPhysics.DriveAPI.Apply(prim, "transX")
        drv.CreateTypeAttr("force")
        drv.CreateTargetPositionAttr(0.0)
        drv.CreateTargetVelocityAttr(0.0)
        drv.CreateStiffnessAttr(0.0)
        drv.CreateDampingAttr(float(DAMP))
        drv.CreateMaxForceAttr(float(GRIP_N))


def release_joint(n: str) -> None:
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    p = stage.GetPrimAtPath(_jpath(n))
    if p.IsValid():
        stage.RemovePrim(p.GetPath())


# -- the scorer ----------------------------------------------------------------------------------------------------------------------------------
DRIFT = 0.030  # the skewer is held still while it stays this near where it lay
ASIDE_Z = 0.012  # the skewer put down: every point of its axis this low over the board
PIN_STEPS = 20  # 1 s held
HOLD_GAP = 0.004  # held: each of the left hand's two fingers has its pad this near the skewer's bare part (to its surface)
HOLD_OPEN = 0.006  # and the jaws are shut to this gap or less, so a hand open over the stick does not hold it
FORK_GAP = 0.006  # a fork's point this near a piece on the stick, at the release: the fork is pushing
TOUCH = 0.004  # a gripper point inside a piece's outline grown by this: touching it
CLEAR = 0.030
HOLD_STEPS = 40  # 2 s at 20 Hz
STILL_V = 0.02
HAND_LAT = np.array([(x, y, z) for x in np.arange(-0.04, 0.0401, 0.01) for y in np.arange(-0.03, 0.0301, 0.01) for z in np.arange(-0.12, 0.1471, 0.01)])


def hands_pts(env) -> tuple:
    """Every point of each whole gripper: a lattice 1 cm apart, x +-4 cm, y +-3 cm, from 12 cm behind the gripper frame (the wrist link) to the fingertips 14.7 cm ahead (palm, housing, pads, fingers)."""
    base = env.unwrapped
    origin = m._np(base.scene.env_origins[0])
    out = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        i = robot.find_bodies("gripper")[0][0]
        pose = m._np(robot.data.body_link_pose_w)[0, i]
        R = m.quat_matrix(pose[3:7])
        out.append(pose[:3] - origin + HAND_LAT @ R.T)
    return tuple(out)


def inside_piece(name: str, st, pts, grow: float) -> bool:
    """Any of `pts` inside the piece's real outline grown by `grow`."""
    t, wy, wz = next(p[1:4] for p in PIECES if p[0] == name)
    loc = st.to_local(pts)
    if name in ("pepper", "onion"):
        ok = (np.abs(loc[:, 0]) <= t / 2 + grow) & (np.abs(loc[:, 1]) <= wy / 2 + grow) & (np.abs(loc[:, 2]) <= wz / 2 + grow)
    elif name == "courgette":
        ok = (np.abs(loc[:, 0]) <= t / 2 + grow) & (np.hypot(loc[:, 1], loc[:, 2]) <= wy / 2 + grow)
    else:  # the dome: radius falls from the flat rear face to the crown
        r = np.hypot(loc[:, 1], loc[:, 2])
        u = (loc[:, 0] + t / 2) / t
        rmax = (wy / 2) * np.sqrt(np.clip(1.0 - np.clip(u, 0, 1) ** 2, 0.0, 1.0))
        ok = (loc[:, 0] >= -t / 2 - grow) & (loc[:, 0] <= t / 2 + grow) & (r <= rmax + grow)
    return bool(np.any(ok))


def fork_points(st) -> np.ndarray:
    return st.to_world(np.array([(x, y, F_TH_T / 2) for x in (F_HEAD1, F_TIP) for y in (-0.0125, -0.00375, 0.00375, 0.0125)] + [(F_SHOULDER, 0.0, F_TH_T / 2)]))


STICK_S = np.linspace(-SK_L / 2, SK_L / 2, 13)


def skewer_aside(sk) -> bool:
    """The bare skewer put down off the plate: every point of its axis outside the plate's rim and lying on the board."""
    pts = sk.to_world(np.c_[STICK_S, np.zeros(13), np.zeros(13)])
    return bool(np.all(np.linalg.norm(pts[:, :2] - PLATE_C, axis=1) > PLATE_R + 0.005) and np.all(pts[:, 2] - TOP < ASIDE_Z))


class Trial:
    def reset(self, env=None) -> None:
        self.n = 0
        self.touch = gk.Latch("a gripper on a vegetable")  # a failure: it latches
        self.jointed = False
        self.released = {k: False for k in NAMES}
        self.good = {k: False for k in NAMES}  # released with the skewer held and the fork pushing: this piece counts
        self.on_plate = {k: False for k in NAMES}
        self.held_still = False
        self.pin = 0
        self.hold = 0
        self.held_now = False
        self.drift_max = 0.0
        self.n_ok = 0
        self.aside = False
        self.fault = gk.Latch("a piece off the stick without the fork and the held skewer, or off the plate")
        self.log = {}
        if env is not None:
            drop_joints()

    def step(self, env) -> None:
        self.n += 1
        if not self.jointed and self.n >= JOINT_AT_STEP:
            make_joints()
            self.jointed = True
        sk = m.state(env, "skewer")
        fk = m.state(env, "fork")
        if os.environ.get("KB_DEBUG") and (self.n <= 30 or self.n % 20 == 0):
            print(f"[kb] n {self.n} stick {np.round(sk.pos, 4)} v {np.linalg.norm(sk.lin_vel):.3f} pieces dx {[round(float(sk.to_local(m.state(env, k).pos)[0, 0] - (PX[k] - SK_C)), 4) for k in NAMES]}", flush=True)
        hands = hands_pts(env)
        ps = {k: m.state(env, k) for k in NAMES}
        # the stick's bare part: from the blunt end to the run's rear face (the run slides; use its rear face now)
        rear = min([float(sk.to_local(ps[k].pos)[0, 0]) - next(p[1] for p in PIECES if p[0] == k) / 2 for k in NAMES if not self.released[k]] or [SK_L / 2])
        tips = m.fingertips(env)[:12]  # the left hand's two fingers, 6 points each
        loc = sk.to_local(tips)
        xe = min(rear - 0.002, SK_L / 2)

        def pad_dist(f):  # the finger's pad is the box its 6 points span (tip and 3 cm up, across its width): its distance to the stick's bare part, to the stick's surface
            lo, hi = f.min(axis=0), f.max(axis=0)
            dx = max(-SK_L / 2 - hi[0], lo[0] - xe, 0.0)
            return float(np.hypot(np.hypot(max(lo[1], -hi[1], 0.0), max(lo[2], -hi[2], 0.0)), dx)) - gk.PAD_INSET - SK_W / 2  # the points stand PAD_INSET outside the pad face

        near = (pad_dist(loc[:6]), pad_dist(loc[6:]))
        gap = float(np.mean(np.linalg.norm(tips[:6] - tips[6:], axis=1))) - 2 * gk.PAD_INSET
        drift = float(np.linalg.norm(sk.pos - np.array([STK_X, STK_Y, ZS])))
        if os.environ.get("KB_DEBUG") and self.n % 20 == 0:
            print(f"[kb] hold n {self.n} near {np.round(near, 4)} gap {gap:.4f} drift {drift:.4f} loc {np.round(loc[[0,1,6,7]], 4).tolist()}", flush=True)
        self.drift_max = max(self.drift_max, drift)
        self.held_now = bool(max(near) < HOLD_GAP and gap <= HOLD_OPEN and drift < DRIFT)
        self.pin = self.pin + 1 if self.held_now else 0
        if self.pin >= PIN_STEPS:
            self.held_still = True
        fp = fork_points(fk)
        # the pieces' release: the rear face passes the point
        for k in NAMES:
            if self.released[k] or not self.jointed:
                continue
            t = next(p[1] for p in PIECES if p[0] == k)
            loc = float(sk.to_local(ps[k].pos)[0, 0])
            if loc - t / 2 > SK_L / 2 + 0.0005:
                pushed = False
                for j in NAMES:
                    if self.released[j] or j == k:
                        continue
                    pushed = pushed or bool(np.min(np.linalg.norm(fp[:, None, :] - ps[j].pos[None, :], axis=2)) < FORK_GAP + max(next(p[1] for p in PIECES if p[0] == j) / 2, next(p[2] for p in PIECES if p[0] == j) / 2))
                last = all(self.released[j] or j == k for j in NAMES)
                pushed = pushed or bool(np.min(np.linalg.norm(fp[:, None, :] - ps[k].pos[None, :], axis=2)) < FORK_GAP + max(t / 2, next(p[2] for p in PIECES if p[0] == k) / 2))
                self.released[k] = True
                self.good[k] = bool(self.held_now and pushed)
                if not self.good[k]:
                    self.fault.trip(True)
                self.log[k] = (self.n, bool(self.held_now), bool(pushed))
                release_joint(k)
        # touching a vegetable with a gripper
        for k in NAMES:
            if any(inside_piece(k, ps[k], h, TOUCH) for h in hands):
                self.touch.trip(True)
        for k in NAMES:
            rel = ps[k].pos[:2] - PLATE_C
            self.on_plate[k] = bool(self.released[k] and np.linalg.norm(rel) < ON_PLATE_R and float(ps[k].pos[2]) - TOP < PLATE_T + 0.025 and float(np.linalg.norm(ps[k].lin_vel)) < 0.5)
            if self.released[k] and np.linalg.norm(rel) > PLATE_R + 0.01 and float(ps[k].pos[2]) - TOP < 0.03:
                self.fault.trip(True)  # a piece on the bench
        still = float(np.linalg.norm(sk.lin_vel)) < STILL_V and float(np.linalg.norm(sk.ang_vel)) < 0.5
        veg_clear = not any(inside_piece(k, ps[k], h, CLEAR) for k in NAMES for h in hands)
        n_ok = sum(1 for k in NAMES if self.good[k] and self.on_plate[k])
        self.n_ok = n_ok
        self.aside = skewer_aside(sk)
        self.hold = self.hold + 1 if (n_ok == 4 and self.aside and still and veg_clear and not self.touch and not self.fault) else 0

    def tier(self) -> float:
        if bool(self.touch) or bool(self.fault) or self.n == 0:
            return 0.0
        if self.hold >= HOLD_STEPS:
            return 1.0
        if self.n_ok == 4:
            return 0.5
        if self.pin >= PIN_STEPS:  # held now, not latched
            return 0.25
        return 0.0


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.tier() >= 1.0


def score(objects) -> float:
    return TRIAL.tier()


# -- the demo ------------------------------------------------------------------------------------------------------------------------------------
GL = d.down(-90.0)  # the left hand comes straight down on the stick, its jaws closing across it (along x)
UD = np.array([0.0, -1.0, 0.0])  # along the stick, toward the point
ASIDE = (-0.13, 0.12, 0.0)  # the bare skewer is drawn back along itself and toward the robot, clear of the plate


def report(tag):
    print(f"[kb] {tag}: tier {TRIAL.tier():.2f} n_ok {getattr(TRIAL, 'n_ok', 0)} held_still {TRIAL.held_still} touch {bool(TRIAL.touch)} fault {bool(TRIAL.fault)} drift_max {TRIAL.drift_max:.4f} released {TRIAL.released} good {TRIAL.good} log {TRIAL.log}", flush=True)


def demo_script(ctx):
    """Modes: full; none; one_arm (the fork hand pushes, the skewer left alone); hold_only (the left hand pins the skewer, nothing else); hover (the left hand open 8 mm over the bare
    stick, nothing else); fingers (no fork: the right hand pinches the mushroom and strips the pieces off with its fingers); offplate (the left lifts the kebab and carries its
    point out over the board beyond the plate, then the fork pushes the pieces off there); no_aside (full, but the bare skewer is left on the plate)."""
    modes = ("full", "none", "one_arm", "hold_only", "hover", "fingers", "offplate", "no_aside")
    if ctx.mode not in modes:
        raise ValueError(f"unknown mode {ctx.mode!r}")
    l, r = ctx.left, ctx.right
    SK = lambda: ctx.state("skewer")
    FK = lambda: ctx.state("fork")
    P = lambda k: ctx.state(k)
    sl = lambda xl: SK().to_world(np.array([xl, 0.0, 0.0]))[0]  # a point on the stick, by its distance from the middle along it
    tip = lambda: FK().to_world(np.array([F_TIP, 0.0, F_TH_T / 2]))[0]  # the tines' middle at their ends
    if ctx.mode == "none":
        yield from l.wait(3.0)
        report("none")
        return
    OPEN = d.FINGER_OPEN
    hd = math.radians(FORK_YAW)
    H = np.array([math.cos(hd), math.sin(hd), 0.0])
    PIN = float(os.environ.get("KB_PIN", "0.04"))  # the pinch, this far from the blunt end
    ONE = ctx.mode == "one_arm"
    # 1. the left hand comes down on the skewer near its blunt end, over the board, and pins it
    g = sl(-SK_L / 2 + PIN) + [0.0, 0.0, float(os.environ.get("KB_PZ", "0"))]
    if ctx.mode == "hover":  # the left hand open, its fingertips 8 mm above the bare stick, no jaws shut
        yield from l.move(g + [0.0, 0.0, 0.10], grip=GL, finger=OPEN, t=3.0, label="L over the skewer")
        yield from l.move(g + [0.0, 0.0, 0.008], t=1.5, label="L hovers")
        yield from l.wait(4.0)
        report("hover")
        return
    if not ONE:
        yield from l.move(g + [0.0, 0.0, 0.10], grip=GL, finger=OPEN, t=3.0, label="L over the skewer")
        yield from l.servo(g + [0.0, 0.0, 0.10], timeout=1.0, label="L over")
        yield from l.move(g, t=1.5, label="L down onto it")
        yield from l.servo(g, timeout=1.0, label="L in")
        yield from l.close(SK_W, force=float(os.environ.get("KB_F", "40")), t=float(os.environ.get("KB_CT", "1.5")), label="L pinch")
        yield from l.wait(0.5)
        report("pinned")
    if ctx.mode == "hold_only":
        yield from l.wait(4.0)
        report("hold_only")
        return
    if ctx.mode == "fingers":  # no fork: the right fingers take the mushroom and draw the pieces off the point
        gm = P("mushroom").pos + [0.0, 0.0, 0.004]
        yield from r.move(gm + [0.0, 0.0, 0.10], grip=d.down(0.0), finger=OPEN, t=3.0, label="R over the mushroom")
        yield from r.move(gm, t=1.5, label="R down")
        yield from r.close(0.030, force=20.0, label="R pinch the mushroom")
        yield from r.move(r.at() + UD * 0.14, t=6.0, label="R strip")
        yield from r.open()
        yield from r.move(r.at() + [0.0, 0.0, 0.10], t=1.5, label="R away")
        yield from l.wait(2.0)
        report("fingers")
        return
    if ctx.mode == "offplate":  # the left lifts the kebab and carries its point out beyond the plate's far rim
        yield from l.move(l.at() + [0.0, 0.0, 0.035], t=1.5, label="L lift")
        yield from l.move(l.at() + [0.17, 0.0, 0.0], t=3.0, label="L carry out")
        yield from l.move(l.at() + [0.0, 0.0, -0.012], t=1.0, label="L lower")
    # 2. the right hand takes the fork by its handle, lifts it and brings the tines over the stick behind the run
    grab = lambda: FK().to_world(np.array([GRAB_X, 0.0, handle_z(GRAB_X) + F_TH_H / 2]))[0]
    gr = d.down(FORK_YAW)
    yield from r.move(grab() + [0.0, 0.0, 0.10], grip=gr, finger=OPEN, t=3.0, label="R over the fork")
    yield from r.servo(grab() + [0.0, 0.0, 0.10], timeout=1.0, label="R over")
    yield from r.move(grab() + [0.0, 0.0, 0.001], t=1.5, label="R down")
    yield from r.servo(grab() + [0.0, 0.0, 0.001], timeout=1.0, label="R down")
    yield from r.close(0.008, force=40.0, label="R grip the fork")
    yield from r.move(r.at() + [0.0, 0.0, 0.09], t=1.5, label="R lift")
    rear = lambda: P("mushroom").to_world(np.array([-0.015, 0.0, 0.0]))[0]
    Z_T = lambda: float(SK().pos[2]) + float(os.environ.get("KB_TZ", "0.005"))  # the tines' middle, this far over the stick axis
    aim = lambda back: np.array([float(sl(0.0)[0]), rear()[1] + 0.004, Z_T()]) - back * H
    yield from r.move(r.at() + (aim(0.10) - tip()), t=3.0, label="R tines behind the run")
    yield from r.servo(aim(0.10), read=tip, timeout=2.0, label="R aim")
    yield from r.servo(aim(0.0), read=tip, timeout=3.0, label="R aim in")
    print(f"[kb] fork tip {tip()} rear {rear()}", flush=True)
    report("fork in")
    # 3. push along the stick, straight along it, until the last piece has passed the point
    p0 = tip().copy()
    dist = float(rear()[1] - sl(SK_L / 2)[1]) + float(os.environ.get("KB_OVER", "0.035"))
    t_push = dist / 0.02
    yield from r.servo(lambda t: p0 + UD * min(dist, 0.02 * t), read=tip, tol=0.004, timeout=t_push + 4.0, gain=0.3, label="R push")
    print(f"[kb] push end: tip {tip()} mushroom {P('mushroom').pos} stick end {sl(SK_L / 2)}", flush=True)
    report("pushed")
    yield from r.wait(0.5)
    # 4. the fork goes back to the board; the left lifts the bare skewer, draws it back off the plate and lays it on the board
    yield from r.move(r.at() + [0.0, 0.0, 0.06], t=1.0, label="R fork up")
    if ctx.mode in ("offplate", "one_arm"):
        yield from l.wait(2.5)
        report("end")
        return
    yield from r.move(np.r_[FORK_REST - 0.045 * H[:2], TOP + 0.03], t=2.5, label="R fork over the board")
    yield from r.move(r.at() + [0.0, 0.0, -0.015], t=0.8, label="R fork down")
    yield from r.open()
    yield from r.move(r.at() + [0.0, 0.0, 0.10], t=1.2, label="R away")
    if ctx.mode != "no_aside":
        yield from l.move(l.at() + [0.0, 0.0, 0.04], t=1.5, label="L lift the skewer")
        yield from l.move(l.at() + np.array(ASIDE), t=3.5, label="L skewer off the plate")
        yield from l.move(np.r_[l.at()[:2], TOP + float(os.environ.get("KB_DZ", "0.004"))], t=1.5, label="L lay it on the board")
    yield from l.open()
    yield from l.move(l.at() + [0.0, 0.0, 0.10], t=1.5, label="L away")
    yield from l.wait(2.5)
    report("end")
