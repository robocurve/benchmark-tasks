"""Serve a table-tennis ball across a kitchen-table ping-pong set: toss it up from one hand, strike it with the paddle.

The station table (110 x 130 cm oak top, the robot at its near end) is set up as a ping-pong table with a clamp-on
retractable net set (Franklin Sports Table Tennis To Go type): the net runs across the table at its middle,
x = 0.36 m, 15.25 cm high, so the near half (x -0.19..0.36) is the server's half and the far half (x 0.36..0.91) the
receiver's. A standard paddle (15.7 x 15.0 cm blade, 10 cm handle, 0.17 kg) lies flat on the table near the right
hand; one 40 mm 2.7 g ball rests in an upturned bottle cap near the left hand.

A serve: one hand tosses the ball up out of its open fingers, the other strikes it with the paddle on the way down,
so that it bounces once on the server's half, passes over the net and bounces on the far half. Then the paddle is
put down on the table and both hands back off.

Events are tracked at every 240 Hz physics step from the ball's state: release from the fingers, toss apex, paddle
strike (velocity jump with the ball on the blade), table bounces (vertical velocity reversal at the table surface)
and any other contact (net, robot).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score --instruction "<INSTRUCTION>"
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

import grip_limited as gl  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = "Serve the ping-pong ball so it bounces once on your side and once past the net, then put the paddle down."
RUN_LIMIT_STEPS = 600

# -- colours ----------------------------------------------------------------------------------------------------------
NET_C = (0.08, 0.10, 0.22)
TAPE_C = (0.95, 0.95, 0.95)
POST_C = (0.12, 0.12, 0.13)
RED = (0.78, 0.07, 0.07)
BLACK = (0.06, 0.06, 0.06)
WOOD = (0.80, 0.62, 0.40)
BALL_C = (0.98, 0.98, 0.96)
CAP_C = (0.10, 0.35, 0.80)

# -- table and net (table frame: x forward, y left, z above the table top; metres) -------------------------------------
TX0, TX1 = -0.19, 0.91           # station table top 110 cm along x (centre 0.36)
TY = 0.65                        # +-65 cm across (130 cm)
NET_X = 0.36                     # net across the middle of the table
NET_H = 0.1525                   # regulation net height (ITTF 2.2.1: 15.25 cm)
NET_T = 0.004                    # mesh thickness (collision)
TAPE_H = 0.012                   # white top band

# -- ball (ITTF 40+ ball: 40 mm, 2.7 g) ---------------------------------------------------------------------------------
BALL_R = 0.020
BALL_M = 0.0027
BALL_E = 0.89                    # restitution on the table: ITTF bounce test 30.5 cm drop -> 23-26 cm (e 0.87-0.92)
BALL_MU = 0.25
BALL_DAMP = 0.4                  # 1/s linear damping ~ air drag of a 40 mm 2.7 g ball at 2-3 m/s
CAP_X, CAP_Y = 0.24, 0.22        # upturned bottle cap (38 mm closure, 30 mm bore) the ball rests in
CAP_R_IN, CAP_W, CAP_H = 0.015, 0.003, 0.012
BALL_Z0 = CAP_H + math.sqrt(BALL_R ** 2 - CAP_R_IN ** 2)   # ball centre resting on the cap rim

# -- paddle (standard size: blade 157 x 150 mm, handle 100 mm; 0.17 kg) -------------------------------------------------
P_X, P_Y = 0.28, -0.25           # handle centre; handle axis along +y, blade toward +y
H_L, H_W, H_T = 0.100, 0.032, 0.026   # handle length, width, thickness
B_L, B_W, B_T = 0.157, 0.150, 0.010   # blade length (along the handle axis), width, thickness incl. both rubbers
B_OFF = H_L / 2 + B_L / 2 - 0.012     # handle centre -> blade centre along the handle axis (blade overlaps the neck)
P_M = 0.170
H_M = 0.060

SPAWN_Q: dict = {}


class _Part(Box):
    """A free body built by this file's own spawner (root cuboid plus authored parts)."""


BALL = _Part("ball", pos=(CAP_X, CAP_Y, TOP + BALL_Z0 + 0.001), size=(0.002, 0.002, 0.002), mass=BALL_M, color=BALL_C)
PADDLE = _Part("paddle", pos=(P_X, P_Y, TOP + H_T / 2 + 0.001), size=(0.002, 0.002, 0.002), mass=P_M, color=WOOD)


def _fixed_parts():
    out = [
        Box("net_mesh", pos=(NET_X, 0.0, TOP + (NET_H - TAPE_H) / 2), size=(NET_T, 2 * TY + 0.02, NET_H - TAPE_H),
            color=NET_C),
        Box("net_tape", pos=(NET_X, 0.0, TOP + NET_H - TAPE_H / 2), size=(NET_T + 0.002, 2 * TY + 0.02, TAPE_H),
            color=TAPE_C),
    ]
    for s in (1, -1):    # clamp posts just outside the table's side edges
        out.append(Box(f"net_post_{'l' if s > 0 else 'r'}", pos=(NET_X, s * (TY + 0.02), TOP + NET_H / 2),
                       size=(0.02, 0.02, NET_H + 0.004), color=POST_C))
        out.append(Box(f"net_clamp_{'l' if s > 0 else 'r'}", pos=(NET_X, s * (TY + 0.005), TOP - 0.02),
                       size=(0.05, 0.05, 0.03), color=POST_C))
    # white centre line and side lines of the set's tape (visual only, 1 mm)
    out.append(Box("centre_line", pos=(0.5 * (TX0 + TX1), 0.0, TOP + 0.0005), size=(TX1 - TX0 - 0.02, 0.003, 0.001),
                   color=TAPE_C))
    rm = CAP_R_IN + CAP_W / 2
    seg = 2 * rm * math.tan(math.pi / 10) * 1.1
    for i in range(10):
        a = 2 * math.pi * i / 10
        out.append(Box(f"cap_{i}", pos=(CAP_X + rm * math.cos(a), CAP_Y + rm * math.sin(a), TOP + CAP_H / 2),
                       size=(CAP_W, seg, CAP_H), color=CAP_C, yaw=a))
    return out


FIXED = _fixed_parts()
OBJECTS = [*FIXED, PADDLE, BALL]

TWIN_EXTRA = [
    {"name": "ball", "kind": "material", "real": "ITTF 40+ celluloid-free ball",
     "note": "rigid sphere 40 mm 2.7 g, restitution 0.89 (ITTF bounce test), linear damping 0.4/s for air drag, CCD"},
    {"name": "net", "kind": "fixture", "real": "clamp-on retractable net set",
     "note": "rigid 15.25 cm net across the table middle"},
    {"name": "grasp", "kind": "joint", "real": "finger pads pinching the paddle handle",
     "note": "force-limited compliant grip that engages only on two-pad contact (grip_limited)"},
]

_P: dict = {}

GRIP = gl.LimitedGrip(lambda: _P.get("paddle"), label="paddle handle", name="pad_grip", joint_root=lambda: _P.get("root"))


# -- USD helpers ----------------------------------------------------------------------------------------------------------
def _stage():
    from isaaclab.sim.utils.stage import get_current_stage

    return get_current_stage()


def _material(stage, path, mu, rest=0.0, rest_mode=None):
    from pxr import PhysxSchema, UsdPhysics, UsdShade

    if stage.GetPrimAtPath(path).IsValid():
        return path
    UsdShade.Material.Define(stage, path)
    prim = stage.GetPrimAtPath(path)
    m = UsdPhysics.MaterialAPI.Apply(prim)
    m.CreateStaticFrictionAttr(mu)
    m.CreateDynamicFrictionAttr(mu)
    m.CreateRestitutionAttr(rest)
    px = PhysxSchema.PhysxMaterialAPI.Apply(prim)
    px.CreateFrictionCombineModeAttr("min")
    if rest_mode:
        px.CreateRestitutionCombineModeAttr(rest_mode)
    return path


def _look(stage, path, color, rough=0.5):
    import isaaclab.sim as sim_utils
    from yam_arena.station import linear_rgb

    if stage.GetPrimAtPath(path).IsValid():
        return path
    cfg = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=rough)
    cfg.func(path, cfg)
    return path


def _collider(stage, prim, mat, offset=0.002):
    from isaaclab.sim.utils import bind_physics_material
    from pxr import PhysxSchema, UsdPhysics

    UsdPhysics.CollisionAPI.Apply(prim)
    cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    cp.CreateContactOffsetAttr(offset)
    cp.CreateRestOffsetAttr(0.0)
    if mat:
        bind_physics_material(str(prim.GetPath()), mat, stage=stage)


def _cube(stage, parent, name, ctr, size, look, mat=None, collide=True):
    from isaaclab.sim.utils import bind_visual_material
    from pxr import Gf, UsdGeom

    g = UsdGeom.Cube.Define(stage, f"{parent}/{name}")
    g.CreateSizeAttr(1.0)
    x = UsdGeom.Xformable(g)
    x.AddTranslateOp().Set(Gf.Vec3d(*map(float, ctr)))
    x.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
    p = g.GetPrim()
    if collide:
        _collider(stage, p, mat)
    bind_visual_material(str(p.GetPath()), look, stage=stage)
    return p


def _disc(stage, parent, name, ctr, rx, ry, h, look, mat=None, n=28):
    """Elliptic blade disc (axis z), convex-hull collider."""
    from isaaclab.sim.utils import bind_visual_material
    from pxr import Gf, UsdGeom, UsdPhysics, Vt

    pts, faces = [], []
    for k in range(n):
        a = 2 * math.pi * k / n
        for z in (-h / 2, h / 2):
            pts.append((ctr[0] + rx * math.cos(a), ctr[1] + ry * math.sin(a), ctr[2] + z))
    for k in range(n):
        a0, a1 = 2 * k, 2 * ((k + 1) % n)
        faces.append((a0, a1, a1 + 1, a0 + 1))
    faces.append(tuple(2 * k for k in range(n - 1, -1, -1)))
    faces.append(tuple(2 * k + 1 for k in range(n)))
    m = UsdGeom.Mesh.Define(stage, f"{parent}/{name}")
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in pts]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([len(f) for f in faces]))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray([int(i) for f in faces for i in f]))
    m.CreateSubdivisionSchemeAttr("none")
    p = m.GetPrim()
    if mat:
        _collider(stage, p, mat)
        UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr("convexHull")
    bind_visual_material(str(p.GetPath()), look, stage=stage)
    return p


def _hide_root(stage, prim_path):
    from pxr import Usd, UsdGeom, UsdPhysics

    for p in Usd.PrimRange(stage.GetPrimAtPath(prim_path)):
        if p.IsA(UsdGeom.Gprim) and p.HasAPI(UsdPhysics.CollisionAPI) and p.GetName() == "mesh":
            UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
            UsdGeom.Imageable(p).MakeInvisible()


def _enable_ccd(stage):
    from pxr import PhysxSchema, UsdPhysics

    for p in stage.Traverse():
        if p.IsA(UsdPhysics.Scene):
            PhysxSchema.PhysxSceneAPI.Apply(p).CreateEnableCCDAttr(True)


def _author_part(stage, prim_path: str, name: str) -> None:
    from pxr import PhysxSchema, UsdGeom

    root = str(prim_path).rsplit("/", 1)[0]
    _P["root"] = root
    looks = f"{root}/TTLooks"
    if not stage.GetPrimAtPath(looks).IsValid():
        UsdGeom.Scope.Define(stage, looks)
    prim = stage.GetPrimAtPath(prim_path)
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateSolverPositionIterationCountAttr(16)
    rb.CreateSolverVelocityIterationCountAttr(4)
    rb.CreateMaxDepenetrationVelocityAttr(0.5)
    _hide_root(stage, prim_path)
    if name == PADDLE.name:
        _P["paddle"] = prim_path
        wood = _look(stage, f"{looks}/wood", WOOD, 0.6)
        red = _look(stage, f"{looks}/red", RED, 0.7)
        black = _look(stage, f"{looks}/black", BLACK, 0.7)
        mu_wood = _material(stage, f"{looks}/mu_wood", 0.6)
        mu_rub = _material(stage, f"{looks}/mu_rubber", 0.9)
        # handle: along +y, centred on the body origin
        _cube(stage, prim_path, "handle", (0.0, 0.0, 0.0), (H_W, H_L, H_T), wood, mu_wood)
        # blade: wood core plus red (top) and black (bottom) rubbers
        _disc(stage, prim_path, "blade", (0.0, B_OFF, 0.0), B_W / 2, B_L / 2, B_T - 0.004, wood, mu_rub)
        _disc(stage, prim_path, "rubber_red", (0.0, B_OFF, (B_T - 0.002) / 2), B_W / 2 - 0.001, B_L / 2 - 0.001,
              0.002, red, mu_rub)
        _disc(stage, prim_path, "rubber_black", (0.0, B_OFF, -(B_T - 0.002) / 2), B_W / 2 - 0.001, B_L / 2 - 0.001,
              0.002, black, mu_rub)
        rb.CreateEnableCCDAttr(True)
    elif name == BALL.name:
        _P["ball"] = prim_path
        from isaaclab.sim.utils import bind_visual_material
        from pxr import UsdGeom as G

        look = _look(stage, f"{looks}/ball", BALL_C, 0.35)
        mu = _material(stage, f"{looks}/mu_ball", BALL_MU, BALL_E, "max")
        s = G.Sphere.Define(stage, f"{prim_path}/sphere")
        s.CreateRadiusAttr(BALL_R)
        _collider(stage, s.GetPrim(), mu, offset=0.004)
        bind_visual_material(str(s.GetPath()), look, stage=stage)
        rb.CreateEnableCCDAttr(True)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        rb.CreateLinearDampingAttr(BALL_DAMP)
        rb.CreateAngularDampingAttr(0.05)
        PhysxSchema.PhysxRigidBodyAPI(prim).CreateSleepThresholdAttr(0.0)
        _enable_ccd(stage)


def _spawn_part(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    from isaaclab.sim.spawners.shapes import spawn_cuboid

    prim = spawn_cuboid(prim_path, cfg, translation, orientation, **kwargs)
    _author_part(_stage(), str(prim_path), cfg.part_name)
    return prim


def _register_module():
    if __name__ not in sys.modules:
        import types

        mod = types.ModuleType(__name__)
        mod.__dict__.update(globals())
        sys.modules[__name__] = mod


def _part_object(spec: _Part):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.schemas import PhysxCollisionPropertiesCfg, PhysxRigidBodyPropertiesCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from yam_arena.station import linear_rgb

    _register_module()

    @configclass
    class PartCfg(sim_utils.CuboidCfg):
        func: object = clone(_spawn_part)
        part_name: str = ""

    cfg = PartCfg(size=tuple(spec.size), part_name=spec.name, rigid_props=PhysxRigidBodyPropertiesCfg(),
                  mass_props=sim_utils.MassPropertiesCfg(mass=spec.mass),
                  collision_props=PhysxCollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0),
                  physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.5),
                  visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.5))
    q = SPAWN_Q.get(spec.name, (1.0, 0.0, 0.0, 0.0))
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=cfg,
                  initial_pose=Pose(position_xyz=tuple(map(float, spec.pos)),
                                    rotation_xyzw=(q[1], q[2], q[3], q[0])))


def _patch_scene(scene) -> None:
    if getattr(scene, "_tt_patched", False):
        return
    orig = scene.task_object

    def task_object(spec):
        if isinstance(spec, _Part):
            return _part_object(spec)
        return orig(spec)

    scene.task_object = task_object
    scene._tt_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    _tt = True

    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        orig_exec = spec.loader.exec_module

        def exec_module(module):
            orig_exec(module)
            _patch_scene(module)

        spec.loader.exec_module = exec_module
        return spec


if "yam_arena.scene" in sys.modules:
    _patch_scene(sys.modules["yam_arena.scene"])
elif not any(getattr(f, "_tt", False) for f in sys.meta_path):
    sys.meta_path.insert(0, _Finder())


# -- math ---------------------------------------------------------------------------------------------------------------
def _qrot(q, v):
    w, x, y, z = q
    u = np.array([x, y, z], dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def _qinv_rot(q, v):
    w, x, y, z = q
    return _qrot((w, -x, -y, -z), v)


# -- rules (all checked on the ball's state at every 240 Hz physics step) -----------------------------------------------
TOSS_MIN = 0.16        # ball rises >= 16 cm above the point where it left the fingers (ITTF 2.6.2 toss height)
STRIKE_DV = 0.4        # m/s velocity change not explained by gravity/drag = a contact
PADDLE_DV = 0.1        # a smaller change counts when the ball is at the blade (a push spreads over a few steps)
BLADE_TOL = 0.012      # ball surface within 12 mm of the blade's face/edge when that jump happens = paddle strike
TABLE_TOL = 0.006      # ball bottom within 6 mm of the table top with vz reversing = table bounce
CLEAR = 0.10
STILL = 0.01
STILL_S = 2.0
STILL_N = int(round(STILL_S * 20))
RETOUCH_GRACE = 36     # physics steps (0.15 s) after the ball first leaves the fingers: the toss stroke may still touch it
PUSH_V = 0.2          # m/s: at the strike the blade must be moving into the ball (not a ball dropped on a still paddle)
JAW_OPEN = 0.020       # m: each jaw's finger opening at the end
FLAT_DEG = 10.0       # the laid-down paddle's face within 10 deg of horizontal
REST_Z = 0.03          # paddle at rest on the table: its origin <= 3 cm above the table, not held, still


class _Episode:
    def __init__(self):
        self.step = 0
        self.phys = 0
        self.prev = None            # (pos, vel) of the ball at the previous physics step
        self.in_hand = False
        self.was_held = False
        self.touch_n = 0
        self.fh_max = 0.0
        self.min_gap = (9.0, None, 0)
        self.last_touch_z = None
        self.release_z = None       # ball centre height when it left the fingers
        self.release_phys = None
        self.apex = None            # highest ball centre after the release, before the strike
        self.strikes = []           # (phys step, pos, vz_before, held_in_hand)
        self.events = []            # after the first strike: ("bounce", x, y) | ("other", x, y, z)
        self.foul = ""
        self.last_pad = -999
        self.pad_pos = []
        self.first_release = 0
        self.hist: list = []
        self.paddle_hist: list = []
        self.cb_ok = False


_EP = _Episode()
_TRACE = __import__("os").environ.get("TT_TRACE", "0") == "1"
_ENV: dict = {"env": None}
_SUB: dict = {}
_RB: dict = {}


def reset_episode() -> None:
    global _EP
    _EP = _Episode()
    GRIP.reset()


def _views():
    from isaaclab_physx.physics import PhysxManager

    sv = PhysxManager.get_physics_sim_view()
    if sv is None or "ball" not in _P:
        return None
    key = id(sv)
    if _RB.get("key") != key:
        _RB["ball"] = sv.create_rigid_body_view(_P["ball"])
        _RB["paddle"] = sv.create_rigid_body_view(_P["paddle"])
        links = [GRIP._link("left", n) for n in ("tip_left", "tip_right", "gripper")]
        links = [x for x in links if x]
        gl.enable_contacts()
        _RB["hand"] = sv.create_rigid_contact_view([_P["ball"]], filter_patterns=[links], max_contact_data_count=64)
        _RB["n_links"] = len(links)
        _RB["key"] = key
    return _RB


def _origin():
    env = _ENV.get("env")
    if env is None:
        return np.zeros(3)
    return env.unwrapped.scene.env_origins[0].detach().cpu().numpy().astype(np.float64) + np.array([0, 0, TOP])


def _read(view):
    T = gl._np(view.get_transforms()).reshape(-1)
    V = gl._np(view.get_velocities()).reshape(-1)
    p = T[:3].astype(np.float64) - _origin()
    q = (float(T[6]), float(T[3]), float(T[4]), float(T[5]))   # xyzw -> wxyz
    return p, q, V[:3].astype(np.float64)


def blade_gap(bp, pq, pad_p) -> float:
    """Distance (m) from the ball's surface to the paddle blade (an elliptic disc in the paddle frame)."""
    lp = _qinv_rot(pq, bp - pad_p) - np.array([0.0, B_OFF, 0.0])
    ex, ey = lp[0] / (B_W / 2), lp[1] / (B_L / 2)
    rr = math.hypot(ex, ey)
    if rr > 1.0:   # outside the rim: radial excess (approx.)
        radial = (rr - 1.0) * min(B_W, B_L) / 2
    else:
        radial = 0.0
    dz = max(0.0, abs(lp[2]) - B_T / 2)
    return math.hypot(radial, dz) - BALL_R


def _left_tcp():
    env = _ENV.get("env")
    if env is None:
        return None
    from yam_arena.runtime import tcp_world

    return np.asarray(tcp_world(env.unwrapped.scene["left_robot"]), dtype=np.float64) - _origin()


def _on_physics(dt) -> None:
    try:
        v = _views()
        if v is None:
            return
        bp, bq, bv = _read(v["ball"])
        pp, pq, _ = _read(v["paddle"])
        from yam_arena.constants import PHYSICS_DT

        fh = gl._np(v["hand"].get_contact_force_matrix(dt=PHYSICS_DT)).reshape(-1)
        fsum = float(np.abs(fh).sum())
        touch = fsum > 1e-4
    except Exception as e:  # noqa: BLE001
        if not _SUB.get("warned"):
            print(f"[tt] physics read failed: {e!r}", flush=True)
            _SUB["warned"] = True
        return
    ep = _EP
    ep.cb_ok = True
    ep.fh_max = max(ep.fh_max, fsum)
    ep.phys += 1
    if not np.isfinite(bp).all():
        return
    # in the tossing hand = touching the left gripper's pads/body; the release point is the last touching step
    if touch and not ep.strikes:
        if (ep.release_z is not None and ep.apex is not None and ep.apex - ep.release_z > 0.03
                and not ep.in_hand and ep.phys - ep.first_release > RETOUCH_GRACE):
            ep.foul = ep.foul or "ball touched the tossing hand again after the toss"
        ep.touch_n += 1
        if ep.touch_n >= 24:     # held >= 0.1 s: in the hand (again)
            ep.in_hand = True
            ep.release_z = None
            ep.apex = None
        ep.last_touch_z = float(bp[2])
    elif not touch and not ep.strikes:
        if ep.touch_n > 0 and (ep.in_hand or ep.release_z is not None):
            if ep.in_hand:
                ep.first_release = ep.phys          # the ball first leaves the fingers
            ep.release_z = ep.last_touch_z      # the last step the ball touched the hand
            ep.apex = float(bp[2])
        ep.touch_n = 0
        ep.in_hand = False
    if ep.release_z is not None and not ep.strikes:
        ep.apex = bp[2] if ep.apex is None else max(ep.apex, bp[2])
    ep.pad_pos.append(np.asarray(pp, dtype=float).copy())
    ep.pad_pos = ep.pad_pos[-7:]
    if ep.prev is not None:
        p0, v0 = ep.prev
        h = 1.0 / 240.0
        pred = v0 + h * (np.array([0.0, 0.0, -9.81]) - BALL_DAMP * v0)
        jump = float(np.linalg.norm(bv - pred))
        on_table = (bp[2] - BALL_R) < TABLE_TOL and TX0 <= bp[0] <= TX1 and abs(bp[1]) <= TY
        gap = blade_gap(bp, pq, pp)
        if _TRACE and ep.release_z is not None and ep.phys % 3 == 0 and gap < 0.12 and len(ep.events) < 3:
            lp = _qinv_rot(pq, bp - pp) - np.array([0.0, B_OFF, 0.0])
            print(f"[trace] ph={ep.phys} b={np.round(bp, 3)} v={np.round(bv, 2)} gap={gap:.4f} lp={np.round(lp, 3)} "
                  f"pad={np.round(pp, 3)} jump={jump:.2f}", flush=True)
        if ep.release_z is not None and not ep.strikes and gap < ep.min_gap[0]:
            lp = _qinv_rot(pq, bp - pp) - np.array([0.0, B_OFF, 0.0])
            ep.min_gap = (round(gap, 4), [round(float(x), 3) for x in lp], ep.phys)
        pad_hit = not ep.in_hand and gap < BLADE_TOL and not on_table and jump > PADDLE_DV
        if pad_hit:
            # one contact = consecutive blade-contact steps; a new contact needs a 0.1 s gap
            if not ep.strikes or ep.phys - ep.last_pad > 24:
                # blade speed into the ball: paddle velocity (last 6 physics steps) along the face normal, signed
                # toward the ball's side of the blade
                n = _qrot(pq, (0.0, 0.0, 1.0))
                vpad = (ep.pad_pos[-1] - ep.pad_pos[0]) / (max(1, len(ep.pad_pos) - 1) / 240.0)
                side = 1.0 if float(np.dot(bp - pp, n)) >= 0 else -1.0
                v_in = float(np.dot(vpad, n)) * side
                ep.strikes.append((ep.phys, bp.copy(), float(v0[2]), ep.in_hand, v_in))
                if len(ep.strikes) > 1 and ep.events:
                    ep.foul = ep.foul or "ball struck again after it bounced"
                elif len(ep.strikes) > 1:
                    ep.foul = ep.foul or "ball struck twice"
            ep.last_pad = ep.phys
        elif jump > STRIKE_DV and not ep.in_hand:
            if (not ep.strikes and ep.release_z is not None and not touch
                    and not (gap < BLADE_TOL and not on_table)):
                # after the toss the first thing the ball meets must be the paddle (no bounce before the strike)
                ep.foul = ep.foul or "ball touched something other than the paddle after the toss, before the strike"
            if on_table and v0[2] < 0 < bv[2]:
                if ep.strikes:
                    ep.events.append(("bounce", float(bp[0]), float(bp[1])))
            elif ep.strikes and not on_table:
                last = ep.events[-1] if ep.events else None
                if last is None or last[0] != "other" or ep.phys - last[4] > 24:
                    ep.events.append(("other", float(bp[0]), float(bp[1]), float(bp[2]), ep.phys))
        elif ep.in_hand and gap < BLADE_TOL and jump > STRIKE_DV and not ep.strikes:
            ep.strikes.append((ep.phys, bp.copy(), float(v0[2]), True, 0.0))
    ep.prev = (bp.copy(), bv.copy())


def _subscribe():
    if _SUB.get("sub") is not None:
        return
    try:
        import omni.physx

        _SUB["sub"] = omni.physx.get_physx_interface().subscribe_physics_step_events(_on_physics)
    except Exception as e:  # noqa: BLE001
        print(f"[tt] physics subscription failed: {e!r}", flush=True)
        _SUB["sub"] = False


def _step(env) -> None:
    _EP.step += 1
    GRIP.update(env)
    _subscribe()
    pts = gripper_points()
    if pts:
        _EP.hist.append(np.array(pts[:4]))
        _EP.hist = _EP.hist[-(STILL_N + 2):]
    pd = _pose(PADDLE.name)
    if pd is not None:
        _EP.paddle_hist.append(pd[0].copy())
        _EP.paddle_hist = _EP.paddle_hist[-(STILL_N + 2):]


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_tt_hooked", False):
        orig = cls.reset

        def reset(self, *a, **k):
            reset_episode()
            return orig(self, *a, **k)

        reset._tt_hooked = True
        cls.reset = reset
    if not getattr(cls._task_step, "_tt_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            out = base(self, *a, **k)
            try:
                _step(self._env)
            except Exception as e:  # noqa: BLE001
                print(f"[tt] step failed: {e!r}", flush=True)
            return out

        _task_step._tt_hooked = True
        cls._task_step = _task_step


_install_hooks()


# -- readouts -------------------------------------------------------------------------------------------------------------
def _finite(a) -> bool:
    return a is not None and bool(np.isfinite(np.asarray(a, dtype=float)).all())


def _pose(name, objects=None):
    env = _ENV.get("env")
    if env is not None:
        try:
            base = env.unwrapped
            pose = base.arena_world.get_pose_w(name)[0].detach().cpu().numpy().astype(np.float64)
            origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
            if np.isfinite(pose).all():
                x, y, z, w = pose[3:7]
                return pose[:3] - origin - np.array([0, 0, TOP]), (w, x, y, z)
        except Exception:  # noqa: BLE001
            pass
    if objects is None:
        return None
    o = objects.get(name) if hasattr(objects, "get") else objects[name]
    if o is None or not _finite(o.pos) or not _finite(o.quat_xyzw):
        return None
    x, y, z, w = (float(c) for c in o.quat_xyzw)
    return np.asarray(o.pos, dtype=float) - np.array([0, 0, TOP]), (w, x, y, z)


def gripper_points() -> list:
    env = _ENV["env"]
    if env is None:
        return []
    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    from yam_arena.runtime import link_poses, pad_tips_world, tcp_world

    pts = []
    for k in ("left_robot", "right_robot"):
        rob = base.scene[k]
        pts.append(np.asarray(tcp_world(rob), dtype=np.float64) - origin - np.array([0, 0, TOP]))
        pts.append(np.asarray(pad_tips_world(rob), dtype=np.float64) - origin - np.array([0, 0, TOP]))
    for k in ("left_robot", "right_robot"):
        lp = link_poses(base.scene[k])
        for n in ("tip_left", "tip_right"):
            if n in lp:
                pts.append(np.asarray(lp[n][:3, 3], dtype=np.float64) - origin - np.array([0, 0, TOP]))
    return pts


def _paddle_dist(p) -> float:
    pd = _pose(PADDLE.name)
    if pd is None:
        return 0.0
    lp = _qinv_rot(pd[1], np.asarray(p) - pd[0])
    lo = np.array([-B_W / 2, -H_L / 2, -H_T / 2])
    hi = np.array([B_W / 2, B_OFF + B_L / 2, H_T / 2])
    return float(np.linalg.norm(np.maximum(np.maximum(lo - lp, lp - hi), 0.0)))


def toss_ok() -> bool:
    """The first strike came after a free toss: the ball left the fingers, rose >= 16 cm above that point, and was
    struck while falling (vertical velocity < 0 just before contact), not while still in the hand."""
    if not _EP.strikes:
        return False
    s = _EP.strikes[0]
    if s[3] or _EP.release_z is None or _EP.apex is None:
        return False
    return bool(_EP.apex - _EP.release_z >= TOSS_MIN and s[2] < 0.0 and s[4] >= PUSH_V)


def _first_events():
    """The first two contacts after the strike."""
    return _EP.events[:2]


def own_half_bounce() -> bool:
    ev = _first_events()
    return bool(ev and ev[0][0] == "bounce" and TX0 <= ev[0][1] < NET_X and abs(ev[0][2]) <= TY)


def far_half_bounce() -> bool:
    ev = _first_events()
    return bool(own_half_bounce() and len(ev) > 1 and ev[1][0] == "bounce" and NET_X < ev[1][1] <= TX1
                and abs(ev[1][2]) <= TY)


def paddle_down() -> bool:
    """The paddle lies on the table (origin <= 3 cm above the top, within the table), not held, and still."""
    pd = _pose(PADDLE.name)
    if pd is None or GRIP.holders() != 0:
        return False
    if not (pd[0][2] <= REST_Z and TX0 <= pd[0][0] <= TX1 and abs(pd[0][1]) <= TY):
        return False
    if abs(_qrot(pd[1], (0.0, 0.0, 1.0))[2]) < np.cos(np.radians(FLAT_DEG)):   # lying flat on a face
        return False
    h = _EP.paddle_hist
    return bool(len(h) > STILL_N and np.linalg.norm(h[-1] - h[-1 - STILL_N]) <= STILL)


def jaws_open() -> bool:
    """Both jaws open: finger opening >= 20 mm on each arm."""
    env = _ENV.get("env")
    if env is None:
        return False
    from yam_arena.constants import FINGER_JOINTS

    base = env.unwrapped
    for k in ("left_robot", "right_robot"):
        rob = base.scene[k]
        ids = rob.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        if float(np.abs(rob.data.joint_pos[0][ids].detach().cpu().numpy()).sum()) < JAW_OPEN:
            return False
    return True


def parked() -> bool:
    """Grippers released and open (>= 20 mm), every gripper point >= 10 cm from the paddle, and still (< 1 cm over the
    last 2 s)."""
    if GRIP.holders() != 0 or not jaws_open():
        return False
    pts = gripper_points()
    if not pts or not all(_finite(p) for p in pts):
        return False
    if any(_paddle_dist(p) < CLEAR for p in pts):
        return False
    h = _EP.hist
    if len(h) < STILL_N + 1:
        return False
    return bool(np.max(np.linalg.norm(h[-1] - h[-1 - STILL_N], axis=1)) <= STILL)


def violation() -> str:
    if _EP.foul:
        return _EP.foul
    if _EP.strikes and _EP.strikes[0][3]:
        return "ball struck while still in the hand"
    return ""


def score(objects) -> float:
    """0 if the ball was struck twice or struck while still in the hand. Cumulative: 0.25 a toss (ball out of the
    fingers, risen >= 16 cm above that point) struck by the paddle while falling; 0.5 the first contact after the
    strike is a bounce on the server's half; 0.75 the next contact is a bounce on the far half (net not touched);
    1.0 also the paddle lying on the table, both grippers open, >= 10 cm from it and still for 2 s."""
    if violation() or not toss_ok():
        return 0.0
    if not own_half_bounce():
        return 0.25
    if not far_half_bounce():
        return 0.5
    if not (paddle_down() and parked()):
        return 0.75
    return 1.0


def success(objects) -> bool:
    return bool(score(objects) == 1.0)


def distance(objects) -> float:
    b = _pose(BALL.name, objects)
    if b is None:
        return 1.0
    return float(abs(b[0][0] - 0.6))


def progress(objects) -> dict:
    s0 = _EP.strikes[0] if _EP.strikes else None
    return {"cb": float(_EP.cb_ok), "fh_max": round(_EP.fh_max, 3), "n_links": _RB.get("n_links"),
            "in_hand": _EP.in_hand, "min_gap": _EP.min_gap, "release_z": _EP.release_z, "apex": _EP.apex,
            "strike": None if s0 is None else [s0[0], [round(float(x), 3) for x in s0[1]], round(s0[2], 2), s0[3], round(s0[4], 2)],
            "n_strikes": len(_EP.strikes), "events": _EP.events[:4], "toss_ok": float(toss_ok()),
            "paddle_down": float(paddle_down()), "parked": float(parked()), "violation": violation(),
            "grip": GRIP.summary()}


VARIATION: dict = {}
