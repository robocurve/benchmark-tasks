"""Hang a framed picture on a wall hook by the wire behind it, level, then let go.

A white wall panel (12 mm MDF, 90 cm wide, 60 cm tall) stands on the station table along y, its face 45 cm in front of
the arm bases. One steel nail-in picture hook (OOK #533024, 30 lb rating: published 14 x 32 x 10.4 mm, modelled at
13 x 32 mm plate, 12 mm lip, 9 mm throat) is nailed into it at y 0, its throat 39 cm above the table. A black plastic
poster frame, 30 cm wide and 40 cm tall (outer), 22 mm deep, 0.20 kg, stands on its bottom edge on the table, centred
at y 0, face to the robot, leaning back 25 degrees with its top against the panel just below the hook. On its back two
narrow 1-hole D-ring hangers (OOK #50206, 17.5 mm, 30 lb rating) 10 cm below the top and 2 cm in from each side carry a
27 cm span of OOK DuraSteel #3 7-strand braided stainless steel picture wire (OOK #50112 / Hillman #534616, ~0.8 mm
real diameter, 20 lb working load limit): hung, its peak sits about 6.4 cm below the top.

The robot must hang the picture on the hook by its wire, level (top-rail tilt at most 1.5 degrees), and let go with
both grippers parked at least 10 cm clear. The wire is behind the frame, so once the frame is close to the wall it
cannot be seen: the frame has to be held upright with both hands, brought within about a centimetre of the wall, and
lowered until the slack wire drops into the hook's throat and takes the weight. Released with the wire caught
off-centre, the frame stays crooked (the wire is held on the hook by friction) until a lower corner is pushed up.

Why two arms: the arms can pinch the frame only on its side rails. Pinched at one side, the frame's weight acts 15 cm
from the pinch and turns it round the grip; the middle of the top rail is above the reach of a downward grip.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 900 \\
           --instruction "Hang the picture on the wall hook by its wire so it hangs straight, then let go."
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cable below is simulated)
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 2.0
RUN_LIMIT_STEPS = 900
CONTROL_HZ = 20
RUN_LIMIT_S = 45.0  # = RUN_LIMIT_STEPS / CONTROL_HZ: judged at the 45 s limit if never at rest for SUCCESS_HOLD_S
assert RUN_LIMIT_S == RUN_LIMIT_STEPS / CONTROL_HZ

INSTRUCTION = "Hang the picture on the wall hook by its wire so it hangs straight, then let go."

WHITE = (0.93, 0.93, 0.91)
FRAME_C = (0.06, 0.06, 0.07)
STEEL = (0.62, 0.63, 0.66)
WIRE_C = (0.72, 0.72, 0.70)
BRASS = (0.78, 0.62, 0.28)

# -- wall panel and hook (m, world frame: x forward, y left, z up) ---------------------------------------------------------
WALL_X = 0.45  # panel face
WALL_T = 0.012
WALL_W, WALL_H = 0.90, 0.60
WALL = Box("wall_panel", pos=(WALL_X + WALL_T / 2, 0.0, TOP + WALL_H / 2), size=(WALL_T, WALL_W, WALL_H), color=WHITE)

HOOK_Y = 0.0
THROAT_Z = TOP + 0.39  # top of the hook's seat (where the wire rests)
HOOK_W = 0.013  # across (y)
LIP_OUT = 0.012  # the upturned tip's outer face, from the wall
HOOK_T = 0.003  # strip thickness
PLATE_H = 0.032
TIP_H = 0.009  # the tip rises this far above the seat
HOOK_PLATE = Box("hook_plate", pos=(WALL_X - HOOK_T / 2, HOOK_Y, THROAT_Z - HOOK_T + PLATE_H / 2),
                 size=(HOOK_T, HOOK_W, PLATE_H), color=STEEL)
HOOK_SEAT = Box("hook_seat", pos=(WALL_X - LIP_OUT / 2, HOOK_Y, THROAT_Z - HOOK_T / 2),
                size=(LIP_OUT, HOOK_W, HOOK_T), color=STEEL)
HOOK_TIP = Box("hook_tip", pos=(WALL_X - LIP_OUT + HOOK_T / 2, HOOK_Y, THROAT_Z - HOOK_T + (TIP_H + HOOK_T) / 2),
               size=(HOOK_T, HOOK_W, TIP_H + HOOK_T), color=STEEL)
HOOK_PARTS = (HOOK_PLATE, HOOK_SEAT, HOOK_TIP)

# -- the picture --------------------------------------------------------------------------------------------------------
FD, FW, FH = 0.022, 0.30, 0.40  # depth (x), width (y), height (z)
FRAME_M = 0.20
LEAN = math.radians(25.0)  # start: leaning back on the panel by 25 degrees (top toward the wall)
RING_DOWN = 0.10  # D-rings 10 cm below the top
RING_IN = 0.02  # and 2 cm in from each side
RING_OFF = 0.006  # wire eye 6 mm behind the back face
RING_Y = FW / 2 - RING_IN
WIRE_L = 0.27
WIRE_R = 0.0015  # 3 mm simulated diameter (real OOK #534616 #3 wire is ~0.8 mm) for reliable hook-seat collision
WIRE_M = 0.06
WIRE_K = 0.0005
EYE_M = 0.03
RING_K = 1500.0  # N/m: the hung wire's ~4.5 N pull stretches each ring about 3 mm
RING_C = 3.0
LEAN_YAW = 2.0 * math.pi  # marker yaw (a full turn, so no rotation by itself): the spawn patch below leans the frame


def lean_rot(th: float | None = None) -> np.ndarray:
    th = LEAN if th is None else th
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _start_centre() -> np.ndarray:
    """Frame centre at the start: bottom-back edge on the table, top-back edge against the panel face."""
    R = lean_rot()
    corner_local = np.array([FD / 2, 0.0, -FH / 2])
    xb = WALL_X - FH * math.sin(LEAN) - 0.0005  # bottom-back edge x, so the top-back edge just meets the panel
    return np.array([xb, 0.0, TOP + 0.0005]) - R @ corner_local


FRAME_C0 = _start_centre()
FRAME = Box("picture_frame", pos=tuple(FRAME_C0.tolist()), size=(FD, FW, FH), mass=FRAME_M, color=FRAME_C,
            yaw=LEAN_YAW)


def ring_local(side: float) -> np.ndarray:
    """A wire eye (D-ring) in the frame's body frame; side +1 left, -1 right."""
    return np.array([FD / 2 + RING_OFF, side * RING_Y, FH / 2 - RING_DOWN])


# each wire end is twisted round itself behind its D-ring: a short stiff tail that carries the ring's load
EYE_S = 0.006
EYE = Plug(size=(EYE_S, 0.004, 0.004), mass=EYE_M, color=(0.78, 0.62, 0.28))
RING0_L = FRAME_C0 + lean_rot() @ ring_local(1.0)
WIRE = cable("picture_wire", start=tuple(RING0_L.tolist()), yaw=-math.pi / 2, length=WIRE_L, radius=WIRE_R,
             segment=0.02, start_plug=EYE, end_plug=EYE, cable_mass=WIRE_M,
             bend_stiffness=WIRE_K, bend_damping=0.0002, friction=0.4,
             color=WIRE_C)

OBJECTS = [WALL, *HOOK_PARTS, FRAME, WIRE]


def _wire_v():
    """The wire's start layout: a V in the frame's back plane, from the left D-ring sagging toward the frame's
    bottom to the right one (both tips exactly in the rings)."""
    import cableshape

    R = lean_rot()
    a = FRAME_C0 + R @ ring_local(1.0)
    b = FRAME_C0 + R @ ring_local(-1.0)
    sag = math.sqrt((WIRE_L / 2) ** 2 - RING_Y ** 2)
    apex = (a + b) / 2 + sag * (R @ np.array([0.0, 0.0, -1.0]))
    return cableshape.shape_points(WIRE, np.array([a, apex, b]))


WIRE_V = _wire_v()


def _quat_xyzw(M) -> tuple:
    m = np.asarray(M, float)
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
    return (x, y, z, w)


def _install_wire_start() -> None:
    """Make the wire's spawn and reset pose the V (root = seg_0's pose, default bend angles = the V's)."""
    orig = cablekit._arena_object
    if getattr(orig, "_picture_task", False):
        return

    def _arena_object(spec):
        obj = orig(spec)
        if spec.name != WIRE.name:
            return obj
        from isaaclab_arena.utils.pose import Pose

        obj.initial_pose = Pose(position_xyz=tuple(map(float, WIRE_V.centres[0])),
                                rotation_xyzw=tuple(map(float, _quat_xyzw(WIRE_V.frames[0]))))
        obj.object_cfg = obj._init_object_cfg()
        obj._pose_event_cfg = obj._build_reset_event()
        jp = {}
        for i, (q1, q2) in enumerate(WIRE_V.q, start=1):
            jp[f"bend_{i}:1"] = float(q1)
            jp[f"bend_{i}:2"] = float(q2)
        obj.object_cfg.init_state.joint_pos = jp
        return obj

    _arena_object._picture_task = True
    cablekit._arena_object = _arena_object


_install_wire_start()

TWIN_EXTRA = [
    {"name": "d_rings", "kind": "joint", "real": "two OOK #50206 small 1-hole narrow D-ring hangers (17.5 mm, 30 lb) "
     "screwed to the back of the side rails, 100 mm below the top, 20 mm in from each side",
     "note": "wire ends held 6 mm behind the back face by symmetric 1500 N/m springs"},
    {"name": "picture_inside", "kind": "decal", "size_mm": [240, 340], "colour": "white mat, printed picture",
     "pose": [0.0, 0.0, 0.0]},
    {"name": "wall_brackets", "kind": "body", "size_mm": [150, 150, 20], "colour": "steel",
     "note": "two L shelf brackets behind the panel, clamped to the table"},
]

VARIATION = {"obj:picture_frame": {"dx": 0.0, "dy": 0.01, "yaw": 0.0}, "cable:picture_wire": {"dx": 0.0, "dy": 0.0,
                                                                                               "yaw": 0.0}}


# -- spawn patches: lean the frame; pin the wire's ends in the D-rings; frame look; hook friction ------------------------
def _lean_quat() -> tuple[float, float, float, float]:
    return (0.0, math.sin(LEAN / 2), 0.0, math.cos(LEAN / 2))


_SCENE_MOD = "yam_arena.scene"  # patched after it loads (importing it early would load USD before the app starts)


def _patch_scene(ysc) -> None:
    orig = ysc._yaw_xyzw
    if getattr(orig, "_picture_lean", False):
        return

    def _yaw_xyzw(yaw: float):
        if abs(yaw - LEAN_YAW) < 1e-9:
            return _lean_quat()
        return orig(yaw)

    _yaw_xyzw._picture_lean = True
    ysc._yaw_xyzw = _yaw_xyzw


class _SceneFinder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname != _SCENE_MOD:
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


def _install_lean() -> None:
    if _SCENE_MOD in sys.modules:
        _patch_scene(sys.modules[_SCENE_MOD])
    elif not any(isinstance(f, _SceneFinder) for f in sys.meta_path):
        sys.meta_path.insert(0, _SceneFinder())


_install_lean()


def _install_spawn() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_picture_task", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != WIRE.name:
            return prim
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils import bind_physics_material, bind_visual_material
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

        from yam_arena.station import linear_rgb

        stage = get_current_stage()
        root = str(prim_path).rsplit("/", 1)[0]
        frame = f"{root}/{FRAME.name}"
        n = WIRE.segments
        q0 = Gf.Quatf(1, 0, 0, 0)
        # author the segments in the V (relative to the root = seg_0), so the rings' joints hold from the first step
        # (relative to the root's actual spawn pose, read back from the stage, env frame)
        def _xf(path):
            m = UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0)
            M = np.array([[m[i][j] for j in range(4)] for i in range(4)])  # row-vector convention
            return M[:3, :3].T, M[3, :3]
        Rr, tr = _xf(str(prim_path))
        Re, te = _xf(root)
        F0, c0 = Re.T @ Rr, Re.T @ (tr - te)
        for i in range(n):
            c = F0.T @ (np.asarray(WIRE_V.centres[i]) - c0)
            qx, qy, qz, qw = _quat_xyzw(F0.T @ np.asarray(WIRE_V.frames[i]))
            seg_prim = stage.GetPrimAtPath(f"{prim_path}/seg_{i}")
            for op in UsdGeom.Xformable(seg_prim).GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    op.Set(Gf.Vec3d(*map(float, c)))
                elif op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                    op.Set(Gf.Quatd(qw, qx, qy, qz))
        for end, i, sg in (("start_plug", 0, -1.0), ("end_plug", n - 1, 1.0)):
            Fi = np.asarray(WIRE_V.frames[i])
            c = F0.T @ (np.asarray(WIRE_V.centres[i]) + Fi[:, 0] * sg * (WIRE.seg_len / 2 + EYE_S / 2) - c0)
            qx, qy, qz, qw = _quat_xyzw(F0.T @ Fi)
            pp = stage.GetPrimAtPath(f"{prim_path}/{end}")
            for op in UsdGeom.Xformable(pp).GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    op.Set(Gf.Vec3d(*map(float, c)))
                elif op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                    op.Set(Gf.Quatd(qw, qx, qy, qz))
        # the cable root frame is turned -90 deg about z (it runs along -y); joint frames below are in each body's frame
        for nm, seg, side, tip in (("ring_left", 0, 1.0, -1.0), ("ring_right", n - 1, -1.0, 1.0)):
            j = UsdPhysics.Joint.Define(stage, f"{prim_path}/{nm}")
            j.CreateBody0Rel().SetTargets([Sdf.Path(frame)])
            j.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/seg_{seg}")])
            j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, ring_local(side))))
            j.CreateLocalRot0Attr(q0)
            j.CreateLocalPos1Attr(Gf.Vec3f(float(tip * WIRE.seg_len / 2), 0.0, 0.0))
            j.CreateLocalRot1Attr(q0)
            # each ring closes the loop frame-wire-frame: a stiff spring, the same at both ends, not a rigid lock
            # (a rigid loop through the chain goes unstable)
            for dof in ("transX", "transY", "transZ"):
                drv = UsdPhysics.DriveAPI.Apply(j.GetPrim(), dof)
                drv.CreateTypeAttr("force")
                drv.CreateStiffnessAttr(RING_K)
                drv.CreateDampingAttr(RING_C)
                drv.CreateTargetPositionAttr(0.0)
            j.CreateExcludeFromArticulationAttr(True)
        fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(frame))
        for i in (0, 1, n - 2, n - 1):
            fp.GetFilteredPairsRel().AddTarget(Sdf.Path(f"{prim_path}/seg_{i}"))
        for end in ("start_plug", "end_plug"):
            fp.GetFilteredPairsRel().AddTarget(Sdf.Path(f"{prim_path}/{end}"))
        # frame body: damping like a real frame's air drag and joint friction; looks (visual only)
        fpr = stage.GetPrimAtPath(frame)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(fpr)
        rb.CreateLinearDampingAttr(0.2)
        rb.CreateAngularDampingAttr(1.0)
        rb.CreateSolverPositionIterationCountAttr(32)
        looks = {}
        for key, col in (("mat", (0.95, 0.94, 0.90)), ("sky", (0.45, 0.62, 0.80)), ("hill", (0.30, 0.52, 0.28)),
                         ("ring", BRASS)):
            path = f"{root}/Looks/picture_{key}"
            m = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(col), roughness=0.5)
            m.func(path, m)
            looks[key] = path

        def deco(name, pos, size, look):
            g = UsdGeom.Cube.Define(stage, f"{frame}/{name}")
            g.CreateSizeAttr(1.0)
            g.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
            g.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
            bind_visual_material(str(g.GetPath()), look, stage=stage)

        sx = (1.0, 1.0, 1.0)  # the frame prim is not scaled (its cube mesh child is)
        fx = -FD / 2 - 0.0004

        def dec(name, y, z, w, h, look, dx=0.0):
            deco(name, ((fx - dx) * sx[0], y * sx[1], z * sx[2]), (0.0008 * sx[0], w * sx[1], h * sx[2]), look)

        dec("mat", 0.0, 0.0, 0.25, 0.35, looks["mat"])
        dec("sky", 0.0, 0.045, 0.19, 0.17, looks["sky"], 0.0002)
        dec("hill", 0.0, -0.075, 0.19, 0.11, looks["hill"], 0.0002)
        for s in (1.0, -1.0):
            rl = ring_local(s)
            deco(f"dring_{'l' if s > 0 else 'r'}", ((FD / 2 + RING_OFF / 2) * sx[0], rl[1] * sx[1], rl[2] * sx[2]),
                 (RING_OFF * sx[0], 0.012 * sx[1], 0.010 * sx[2]), looks["ring"])
        # hook: steel on braided steel wire, friction 0.4
        hm = f"{root}/Looks/hook_steel"
        pm = sim_utils.RigidBodyMaterialCfg(static_friction=0.4, dynamic_friction=0.4)
        pm.func(hm, pm)
        for part in HOOK_PARTS:
            p = stage.GetPrimAtPath(f"{root}/{part.name}")
            if p.IsValid():
                for c in [p, *p.GetChildren()]:
                    if c.HasAPI(UsdPhysics.CollisionAPI):
                        bind_physics_material(str(c.GetPath()), hm, stage=stage)
        return prim

    spawn._picture_task = True
    cablekit._spawn_cable = spawn


_install_spawn()

# -- env handle --------------------------------------------------------------------------------------------------------
_ST: dict = {"env": None}


def _write_wire_v(env) -> None:
    """The env reset puts the wire's bend joints back to straight (its root pose is right): write the V's angles back
    at once, before any physics step, so the right D-ring's joint never sees a wire 1 cm too long."""
    import torch

    base = env.unwrapped
    art = base.scene[WIRE.name]
    names = list(art.joint_names)
    jp = getattr(art.data.joint_pos, "torch", art.data.joint_pos)
    q = torch.zeros_like(jp)
    for i, (q1, q2) in enumerate(WIRE_V.q, start=1):
        q[0, names.index(f"bend_{i}:1")] = float(q1)
        q[0, names.index(f"bend_{i}:2")] = float(q2)
    art.write_joint_state_to_sim_index(position=q, velocity=torch.zeros_like(q))
    base.scene.write_data_to_sim()
    base.sim.step(render=False)
    base.scene.update(dt=base.physics_dt)


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.step, "_picture_hooked", False):
        base_step = cls.step

        def step(self, *a, **k):
            _ST["env"] = self._env
            return base_step(self, *a, **k)

        step._picture_hooked = True
        cls.step = step
    if not getattr(cls.reset, "_picture_hooked", False):
        base_reset = cls.reset

        def reset(self, *a, **k):
            _ST["env"] = self._env
            if not getattr(self, "_picture_wrap", False):
                self._picture_wrap = True
                ens = self._ensure_sim

                def ensure(*aa, **kk):
                    r = ens(*aa, **kk)
                    env = self._env
                    if env is not None and not getattr(env, "_picture_wrap", False):
                        env._picture_wrap = True
                        env_reset = env.reset

                        def reset_with_wire(*a2, **k2):
                            out = env_reset(*a2, **k2)
                            _write_wire_v(env)
                            return out

                        env.reset = reset_with_wire
                    return r

                self._ensure_sim = ensure
            out = base_reset(self, *a, **k)
            _ST["env"] = self._env
            return out

        reset._picture_hooked = True
        cls.reset = reset


_install_hooks()


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy()
    return np.asarray(x)


def _rot(q) -> np.ndarray:
    """Rotation matrix of a unit quaternion given in (x, y, z, w) order (yam_arena ObjectState.quat_xyzw)."""
    x, y, z, w = (float(t) for t in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n > 0.0:
        x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def frame_pose(objects):
    """(centre, rotation) of the frame body, env frame.

    Rotation: the embodiment's ``ObjectState.quat_xyzw`` for the frame, in (x, y, z, w) order (Isaac Lab 3.0 /
    yam_arena convention), which is what ``_rot`` takes. Centre: the frame's live root position when the sim is up (the
    state's ``pos`` is a bounding-box centre that also takes in the D-ring decals on the back); offline, ``st.pos``."""
    st = objects[FRAME.name]
    R = _rot(st.quat_xyzw)
    env = _ST["env"]
    if env is not None:
        base = env.unwrapped
        origin = _np(base.scene.env_origins)[0].astype(np.float64)
        return _np(base.scene[FRAME.name].data.root_pos_w)[0].astype(np.float64) - origin, R
    return np.asarray(st.pos, float), R


def corners(pose) -> np.ndarray:
    """The frame's 8 corners (env frame)."""
    c, R = pose
    out = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                out.append(c + R @ np.array([sx * FD / 2, sy * FW / 2, sz * FH / 2]))
    return np.array(out)


def gripper_points(objects=None) -> list[np.ndarray]:
    """For each arm: TCP, pad-tip midpoint, and both individual finger-pad faces/tips (env frame).
    Uses ``_gripper_pts`` only in offline unit tests; returns [] when robot state is unavailable."""
    env = _ST["env"]
    if env is None:
        if objects is not None and "_gripper_pts" in objects:
            return [np.asarray(pts, float) for pts in objects["_gripper_pts"]]
        return []
    from yam_arena.constants import FINGER_JOINTS, PAD_TIP_PAST_TCP, TCP_OFFSET
    from yam_arena.runtime import link_poses

    base = env.unwrapped
    origin = _np(base.scene.env_origins)[0].astype(np.float64)
    z0 = TCP_OFFSET[2]
    z1 = z0 + PAD_TIP_PAST_TCP
    out = []
    for k in ("left_robot", "right_robot"):
        r = base.scene[k]
        g = link_poses(r, 0)["gripper"]
        fids = r.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        fq = _np(r.data.joint_pos)[0, fids]
        dl, dr = float(abs(fq[0])), float(abs(fq[1]))
        local_pts = np.array([
            [0.0, 0.0, z0],
            [0.0, 0.0, z1],
            [dl, 0.0, z0],
            [dl, 0.0, z1],
            [-dr, 0.0, z0],
            [-dr, 0.0, z1],
        ])
        out.append((g[:3, :3] @ local_pts.T).T + g[:3, 3] - origin)
    return out


def arms_resting(max_joint_vel: float = 0.05) -> bool:
    """Both arms stationary (arm joint speeds < max_joint_vel rad/s, < ~2 cm/s at reach)."""
    env = _ST["env"]
    if env is None:
        return True
    base = env.unwrapped
    for k in ("left_robot", "right_robot"):
        jv = _np(base.scene[k].data.joint_vel)[0, :6]
        if not np.isfinite(jv).all() or float(np.abs(jv).max()) >= max_joint_vel:
            return False
    return True


def box_distance(p: np.ndarray, pose) -> float:
    """Distance from point p to the frame's box (0 inside)."""
    c, R = pose
    q = R.T @ (p - c)
    h = np.array([FD / 2, FW / 2, FH / 2])
    d = np.maximum(np.abs(q) - h, 0.0)
    return float(np.linalg.norm(d))


def gripper_gap(pose, objects=None) -> float:
    """Smallest distance from either gripper's TCP or pad tips to the frame (0.0 if robot state unavailable)."""
    g = gripper_points(objects)
    if not g:
        return 0.0
    return min(box_distance(p, pose) for pts in g for p in pts)


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(math.ceil(np.linalg.norm(b - a) / step)))
        out += [a + (b - a) * (i / k) for i in range(1, k + 1)]
    return np.array(out)


def wire_on_hook(objects) -> bool:
    """A stretch of the wire lies in the hook's throat: between the wall and the upturned tip, over the seat and below
    the tip's top, within the hook's width (+ the wire's radius)."""
    pts = _dense(np.asarray(objects[WIRE.name].polyline(), float))
    x0, x1 = WALL_X - LIP_OUT + HOOK_T, WALL_X
    ok = ((pts[:, 0] > x0 - 0.001) & (pts[:, 0] < x1 + 0.001) & (np.abs(pts[:, 1] - HOOK_Y) < HOOK_W / 2 + WIRE_R)
          & (pts[:, 2] > THROAT_Z - 0.001) & (pts[:, 2] < THROAT_Z + TIP_H))
    return bool(ok.any())


def facing_front(pose) -> bool:
    """Picture faces outward toward the robot (body +x back normal points into the wall, R[0, 0] > 0.5)."""
    _, R = pose
    return bool(R[0, 0] > 0.5)


def tilt_deg(pose) -> float:
    """The frame's tilt: roll of its top rail (body +y) about the wall normal (x) from horizontal (+y), in degrees,
    as read by a digital angle gauge or phone inclinometer along the top rail (0 upright, 180 upside-down)."""
    _, R = pose
    w = R[:, 1]
    return float(math.degrees(math.atan2(abs(w[2]), w[1])))


def bottom_gap(pose) -> float:
    """How far the bottom rail's back edge stands off the panel (m, worst of the two bottom-back corners)."""
    c, R = pose
    xs = [(c + R @ np.array([FD / 2, s * FW / 2, -FH / 2]))[0] for s in (1, -1)]
    return float(max(WALL_X - x for x in xs))


def _finite(objects) -> bool:
    f = objects[FRAME.name]
    return bool(np.isfinite(f.pos).all() and np.isfinite(f.quat_xyzw).all()
                and np.isfinite(objects[WIRE.name].points).all())


LEVEL_DEG = 1.5  # full credit: top-rail tilt at most 1.5 degrees
TILT_MAX_DEG = 15.0  # over 15 degrees (sideways, upside-down, or hanging by one corner): 0
_EPS_DEG = 1e-6
WALL_GAP = 0.010  # bottom rail back edge within 1.0 cm of panel
CLEAR = 0.10  # both grippers at least 10 cm clear of frame for 1.0
TOUCH = 0.005  # gripper within 5 mm of frame: 0
TABLE_CLEAR = 0.005  # lowest corner within 5 mm of table: 0
RING_MAX_GAP = 0.025  # wire ends must remain attached at their D-rings (within 2.5 cm; hung give is ~3 mm)


def rings_intact(pose, objects) -> bool:
    """Both wire ends remain attached to the left and right D-rings on the back of the frame."""
    c, R = pose
    poly = np.asarray(objects[WIRE.name].polyline(), float)
    dl = float(np.linalg.norm(poly[0] - (c + R @ ring_local(1.0))))
    dr = float(np.linalg.norm(poly[-1] - (c + R @ ring_local(-1.0))))
    return bool(dl <= RING_MAX_GAP and dr <= RING_MAX_GAP)


def evaluate(objects) -> dict:
    if not _finite(objects):
        return {"finite": False}
    pose = frame_pose(objects)
    cz = corners(pose)[:, 2]
    return {
        "finite": True,
        "off_table": bool(cz.min() > TOP + TABLE_CLEAR),
        "on_hook": wire_on_hook(objects),
        "rings_intact": rings_intact(pose, objects),
        "facing_front": facing_front(pose),
        "resting": bool(objects[FRAME.name].resting() and arms_resting()),
        "gap": gripper_gap(pose, objects),
        "tilt": tilt_deg(pose),
        "bottom_gap": bottom_gap(pose),
        "low_z": float(cz.min() - TOP),
        "judged_at_s": RUN_LIMIT_S,  # latest judging time (45 s limit) unless held at rest SUCCESS_HOLD_S first
    }


def score(objects) -> float:
    """0 if the state is not finite, the frame touches the table (lowest corner within 5 mm of it), the wire is not
    seated in the hook's throat or an end has pulled free of its D-ring (> 2.5 cm), the picture does not face forward,
    any gripper TCP or finger pad is within 5 mm of the frame, or top-rail tilt exceeds 15.0 degrees. Otherwise 1.0
    when top-rail tilt is at most 1.5 degrees, the bottom rail's back edge is within 1.0 cm of the panel, both grippers
    are at least 10 cm clear of the frame, and both the frame and both arms are at rest; else 0.5 (tilt over 1.5 and up
    to 15.0 degrees, bottom rail more than 1.0 cm off the panel, a gripper 5 mm to 10 cm from the frame, or still
    moving)."""
    e = evaluate(objects)
    if (not e["finite"] or not e["off_table"] or not e["on_hook"] or not e["rings_intact"]
            or not e["facing_front"] or e["gap"] < TOUCH or e["tilt"] > TILT_MAX_DEG + _EPS_DEG):
        return 0.0
    if (e["tilt"] <= LEVEL_DEG + _EPS_DEG and e["bottom_gap"] <= WALL_GAP
            and e["gap"] >= CLEAR and e["resting"]):
        return 1.0
    return 0.5


def success(objects) -> bool:
    return bool(score(objects) >= 1.0)


def distance(objects) -> float:
    e = evaluate(objects)
    if not e["finite"]:
        return 3.0
    return float((0.0 if e["on_hook"] else 1.0) + (0.0 if e["off_table"] else 0.5) + min(e["tilt"] / 15.0, 1.0) * 0.3
                 + (0.0 if e["gap"] >= CLEAR else 0.2))


def progress(objects) -> dict:
    e = evaluate(objects)
    if not e["finite"]:
        return {"lifted": 0, "on_hook": 0, "released": 0, "level": 0, "clear": 0}
    return {"lifted": int(e["off_table"]), "on_hook": int(e["on_hook"]),
            "released": int(e["on_hook"] and e["off_table"] and e["gap"] >= TOUCH),
            "level": int(e["on_hook"] and e["facing_front"] and e["tilt"] <= LEVEL_DEG),
            "clear": int(e["gap"] >= CLEAR)}
