"""On the station table: a 1-inch EMT conduit run (galvanised steel, 29.5 mm OD, 26.6 mm bore, 1.45 mm wall) is
strapped on two 41 mm steel channels, so its axis is 56 mm above the table. From its right-hand mouth (facing the
robot) it runs 12 cm straight away from the robot, turns left through a 90 degree sweep of 146 mm centreline
radius, and runs 23 cm straight to its left-hand mouth (facing left): 59 cm of conduit. Both mouths carry a black
plastic bushing. A yellow 4.8 mm fibreglass fish tape is already pushed through the run: its brass tip with an
eyelet sticks out of the right-hand mouth, its handle end out of the left-hand mouth. An orange 6 mm insulated wire
lies on the table in front of the right arm; its last 16 mm is bare copper, bent down and folded back into a J.
The J must be hooked through the eyelet, then the tape pulled back out of the left-hand mouth (hand over hand)
until the wire end comes out.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cables below are simulated)
import cableshape  # noqa: E402
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Pull the orange wire through the conduit with the yellow fish tape: hook the wire's bent copper end "
               "through the eyelet on the fish tape's tip at the conduit's right-hand end, then pull the tape back "
               "out of the left-hand end until the wire's end is at least 5 cm out of the conduit.")

# -- conduit: 1-inch EMT (OD 29.54 mm, ID 26.64 mm), one 90 degree sweep ---------------------------------------------
OD, ID = 0.02954, 0.02664
RI, RO = ID / 2, OD / 2
SUP_H = 0.041  # steel channel height under the conduit
ZA = TOP + SUP_H + RO  # conduit axis height
R_SWEEP = 0.146  # sweep centreline radius
X_NEAR = 0.17  # near (right-hand) mouth, facing -x (towards the robot)
Y_NEAR = -0.20  # near leg runs along +x at this y
X_FAR = 0.44  # far leg runs along +y at this x
Y_FAR = 0.18  # far (left-hand) mouth, facing +y
SC = np.array([X_FAR - R_SWEEP, Y_NEAR + R_SWEEP])  # sweep centre (x, y)
L1 = SC[0] - X_NEAR  # near straight
L2 = R_SWEEP * math.pi / 2  # sweep
L3 = Y_FAR - SC[1]  # far straight
LC = L1 + L2 + L3  # conduit length along its axis (m)
GALV = (0.70, 0.72, 0.74)
BUSH_L, BUSH_R = 0.010, 0.0185  # bushing: 10 mm long, 37 mm OD, throat flares from the bore to 21 mm radius
BUSH_FLARE = 0.021


def axis_at(s: float) -> tuple[np.ndarray, np.ndarray]:
    """Conduit axis point (x, y, z) and unit direction at arc length ``s`` from the near mouth. Outside [0, LC] the
    end legs are extended straight."""
    if s <= L1:
        return np.array([X_NEAR + s, Y_NEAR, ZA]), np.array([1.0, 0.0, 0.0])
    if s <= L1 + L2:
        th = -math.pi / 2 + (s - L1) / R_SWEEP
        return (np.array([SC[0] + R_SWEEP * math.cos(th), SC[1] + R_SWEEP * math.sin(th), ZA]),
                np.array([-math.sin(th), math.cos(th), 0.0]))
    return np.array([X_FAR, SC[1] + (s - L1 - L2), ZA]), np.array([0.0, 1.0, 0.0])


def project(p) -> tuple[float, float]:
    """(s, r): arc length of the nearest conduit-axis point (end legs extended straight) and distance from it."""
    p = np.asarray(p, dtype=float)
    cands = []
    s = float(min(p[0] - X_NEAR, L1))  # near leg (extended to -inf)
    cands.append((s, np.array([X_NEAR + s, Y_NEAR, ZA])))
    th = math.atan2(p[1] - SC[1], p[0] - SC[0])
    th = min(max(th, -math.pi / 2), 0.0)
    cands.append((L1 + (th + math.pi / 2) * R_SWEEP,
                  np.array([SC[0] + R_SWEEP * math.cos(th), SC[1] + R_SWEEP * math.sin(th), ZA])))
    s = float(max(p[1] - SC[1], 0.0))  # far leg (extended to +inf)
    cands.append((L1 + L2 + s, np.array([X_FAR, SC[1] + s, ZA])))
    s, q = min(cands, key=lambda c: float(np.linalg.norm(p - c[1])))
    return float(s), float(np.linalg.norm(p - q))


SUPPORTS = [  # two steel channels (41 x 41 mm) across the run, the conduit strapped on them
    Box("channel_near", pos=(0.255, Y_NEAR, TOP + SUP_H / 2), size=(0.041, 0.10, SUP_H), color=(0.45, 0.47, 0.50)),
    Box("channel_far", pos=(X_FAR, 0.06, TOP + SUP_H / 2), size=(0.10, 0.041, SUP_H), color=(0.45, 0.47, 0.50)),
]

# -- fish tape: fibreglass rod chain; start = handle end (far side), end = brass tip with the eyelet (near side) ------
TAPE_R = 0.0024
TAPE_ZB = ZA - RI + TAPE_R + 0.0005  # tape axis lying on the bore floor
TIP_OUT = 0.02  # bare tape out of the near mouth before the brass tip
HANDLE_OUT = 0.12
TAPE_LEN = HANDLE_OUT + LC + TIP_OUT
HANDLE = Plug(size=(0.04, 0.014, 0.009), mass=0.008, color=(0.08, 0.08, 0.08))
TIP = Plug(size=(0.02, 0.006, 0.006), mass=0.004, color=(0.78, 0.62, 0.25))  # brass
RING_OUT, RING_BAR, RING_T = 0.020, 0.003, 0.003  # eyelet: 20 mm outside, 3 mm bars -> 14 mm hole
RING_IN = RING_OUT - 2 * RING_BAR
RING_X = TIP.size[0] / 2 + RING_OUT / 2 - 0.002  # eyelet centre along the tape from the tip's centre
RING_Z = 0.006  # eyelet plane above the tape's axis (the eyelet head is raised)
TAPE = cable("fish_tape", start=(0.80, 0.40, TOP + TAPE_R + 0.001), yaw=-math.pi / 2, length=TAPE_LEN,
             radius=TAPE_R, start_plug=HANDLE, end_plug=TIP, color=(0.95, 0.80, 0.10), cable_mass=0.03,
             bend_stiffness=0.03, bend_damping=0.003, friction=0.5)  # spawned straight out of the way, laid at reset


def _tape_path() -> np.ndarray:
    pts = [np.array([X_FAR, Y_FAR + HANDLE_OUT, TAPE_ZB]), np.array([X_FAR, SC[1], TAPE_ZB])]
    for k in range(1, 30):
        th = -k / 30 * math.pi / 2
        pts.append(np.array([SC[0] + R_SWEEP * math.cos(th), SC[1] + R_SWEEP * math.sin(th), TAPE_ZB]))
    pts += [np.array([X_NEAR, Y_NEAR, TAPE_ZB]), np.array([X_NEAR - 0.2, Y_NEAR, TAPE_ZB])]
    return np.stack(pts)


TAPE_PATH = _tape_path()

# -- wire: 6 mm orange insulated wire; its end plug = last 30 mm (rigid) + bare copper J ------------------------------
R = 0.003
WIRE_LEN = 0.82
END = Plug(size=(0.03, 0.006, 0.006), mass=0.004, color=(0.95, 0.42, 0.08))
CU = 0.003  # copper conductor section (3 x 3 mm)
J_GAP = 0.006  # between the straight conductor and the folded-back tail
J_X = END.size[0] / 2 + 0.016 - CU / 2  # leg centre along the wire from the end plug's centre (16 mm bare)
J_TAIL = 0.006
LEG_TOP = CU / 2
LEG_BOT = -(CU / 2 + J_GAP + CU)  # bottom of the J (tail underside), relative to the wire axis
E0 = np.array([0.20, -0.30])  # starting end-plug centre (x, y); the wire end points +y
ARC_C, ARC_R = np.array([0.32, -0.38]), 0.12  # the wire is laid round a quarter arc onto the end


def _cable_path() -> np.ndarray:
    """Tail (far right, on the table) -> straight along -x at y -0.50 -> quarter arc -> straight +y to the end. The
    last 4 cm rise so the J starts standing on the table (laid at reset; the wire then settles)."""
    z = TOP + R + 0.0005
    zc = TOP - LEG_BOT + 0.002
    y_back = E0[1] - END.size[0] / 2  # back of the end plug = end of the wire chain
    arc = ARC_R * math.pi / 2
    straight_end = y_back - ARC_C[1]
    x0 = ARC_C[0] + (WIRE_LEN - arc - straight_end)
    pts = [np.array([x0, ARC_C[1] - ARC_R, z]), np.array([ARC_C[0], ARC_C[1] - ARC_R, z])]
    for k in range(1, 20):
        a = -math.pi / 2 - k / 20 * math.pi / 2
        pts.append(np.array([ARC_C[0] + ARC_R * math.cos(a), ARC_C[1] + ARC_R * math.sin(a), z]))
    pts.append(np.array([E0[0], ARC_C[1], z]))
    pts.append(np.array([E0[0], y_back - 0.04, z]))
    pts.append(np.array([E0[0], y_back, zc]))
    pts.append(np.array([E0[0], y_back + 0.2, zc]))
    return np.stack(pts)


CABLE_PATH = _cable_path()
CABLE = cable("cable", start=(0.90, -0.50, TOP + 0.02), yaw=math.pi, length=WIRE_LEN, radius=R, end_plug=END,
              color=(0.95, 0.42, 0.08), cable_mass=0.04, bend_stiffness=0.004, bend_damping=0.0005)

OBJECTS = [*SUPPORTS, TAPE, CABLE]


# -- extra rigid parts: the eyelet on the tape's tip, the copper J on the wire's end ---------------------------------
def _extra_parts(name: str) -> list[tuple[str, tuple, tuple]]:
    """(part name, centre in the plug frame, size) of extra box colliders on ``name``'s end plug."""
    if name == CABLE.name:
        cx = END.size[0] / 2 + 0.016 / 2
        return [
            ("cu_straight", (cx, 0.0, 0.0), (0.016, CU, CU)),
            ("cu_leg", (J_X, 0.0, (LEG_TOP + LEG_BOT) / 2), (CU, CU, LEG_TOP - LEG_BOT)),
            ("cu_tail", (J_X - CU / 2 - J_TAIL / 2 + 0.0005, 0.0, LEG_BOT + CU / 2), (J_TAIL + 0.001, CU, CU)),
        ]
    if name == TAPE.name:
        c, h, b = RING_X, RING_OUT / 2, RING_BAR
        z = RING_Z
        return [
            ("eye_post", (TIP.size[0] / 2 - 0.002, 0.0, z / 2), (0.004, 0.006, z + 0.003)),
            ("ring_out", (c + h - b / 2, 0.0, z), (b, RING_OUT, RING_T)),
            ("ring_in", (c - h + b / 2, 0.0, z), (b, RING_OUT, RING_T)),
            ("ring_l", (c, h - b / 2, z), (RING_OUT, b, RING_T)),
            ("ring_r", (c, -h + b / 2, z), (RING_OUT, b, RING_T)),
        ]
    return []


def _rot_cols(x, z) -> "np.ndarray":
    x = np.asarray(x, float) / np.linalg.norm(x)
    z = np.asarray(z, float)
    z = z - x * (z @ x)
    z /= np.linalg.norm(z)
    return np.stack([x, np.cross(z, x), z], axis=1)


def _quat_wxyz(m) -> tuple[float, float, float, float]:
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return (0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s)
    i = int(np.argmax(np.diag(m)))
    if i == 0:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        return ((m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s)
    if i == 1:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        return ((m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s)
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
    return ((m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s)


N_SIDES = 12
WALL_COLL = 0.003  # collider slab thickness (outside the true bore surface; collision only)


def _ring_frame(s):
    p, t = axis_at(s)
    u = np.cross([0.0, 0.0, 1.0], t)
    u /= np.linalg.norm(u)
    return p, t, u, np.array([0.0, 0.0, 1.0])


def conduit_slabs() -> list[tuple[np.ndarray, np.ndarray, tuple]]:
    """Static collider boxes (centre, rotation, size) lining the bore (12-sided, flats at the bore radius), and the
    bushings' flared throats. Pure numpy."""
    out = []
    half_w = RI * math.tan(math.pi / N_SIDES) + 0.0006

    def tube(s0, s1, r0, r1):
        pa, ta, ua, va = _ring_frame(s0)
        pb, tb, ub, vb = _ring_frame(s1)
        for k in range(N_SIDES):
            ph = 2 * math.pi * k / N_SIDES
            na = math.cos(ph) * ua + math.sin(ph) * va
            nb = math.cos(ph) * ub + math.sin(ph) * vb
            a = pa + (r0 + WALL_COLL / 2) * na
            b = pb + (r1 + WALL_COLL / 2) * nb
            ln = float(np.linalg.norm(b - a))
            n = (na + nb) / 2
            m = _rot_cols(b - a, n)
            out.append(((a + b) / 2, m, (ln + 0.0015, 2 * half_w * (r0 + r1) / (2 * RI), WALL_COLL)))

    tube(0.0, L1, RI, RI)
    na = 15
    for j in range(na):
        tube(L1 + L2 * j / na, L1 + L2 * (j + 1) / na, RI, RI)
    tube(L1 + L2, LC, RI, RI)
    tube(-BUSH_L, 0.0, BUSH_FLARE, RI)  # bushing throats (flared)
    tube(LC, LC + BUSH_L, RI, BUSH_FLARE)
    return out


def _tube_mesh(samples, r_in, r_out, n=32):
    """Points and quads of a thick tube along ``samples`` (list of (point, t, u, v)) with end annuli."""
    pts, quads = [], []
    m = len(samples)
    for p, t, u, v in samples:
        for r in (r_out, r_in):
            for k in range(n):
                ph = 2 * math.pi * k / n
                pts.append(p + r * (math.cos(ph) * u + math.sin(ph) * v))

    def idx(i, layer, k):
        return (i * 2 + layer) * n + k % n

    for i in range(m - 1):
        for k in range(n):
            quads.append([idx(i, 0, k), idx(i + 1, 0, k), idx(i + 1, 0, k + 1), idx(i, 0, k + 1)])
            quads.append([idx(i, 1, k), idx(i, 1, k + 1), idx(i + 1, 1, k + 1), idx(i + 1, 1, k)])
    for i, sgn in ((0, 1), (m - 1, -1)):
        for k in range(n):
            q = [idx(i, 0, k), idx(i, 0, k + 1), idx(i, 1, k + 1), idx(i, 1, k)]
            quads.append(q if sgn > 0 else q[::-1])
    return np.array(pts), quads


def _author_conduit(stage, parent: str) -> None:
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    root = f"{parent}/emt_conduit"
    UsdGeom.Xform.Define(stage, root)
    look = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(GALV), roughness=0.35, metallic=0.4)
    look.func(f"{root}/Looks/galv", look)
    blk = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb((0.05, 0.05, 0.05)), roughness=0.6)
    blk.func(f"{root}/Looks/bushing", blk)
    pm = sim_utils.RigidBodyMaterialCfg(static_friction=0.3, dynamic_friction=0.3)
    pm.func(f"{root}/Looks/phys", pm)
    for i, (c, m, size) in enumerate(conduit_slabs()):
        g = UsdGeom.Cube.Define(stage, f"{root}/wall_{i}")
        g.CreateSizeAttr(1.0)
        g.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, c)))
        w, x, y, z = _quat_wxyz(m)
        g.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))
        g.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        g.CreatePurposeAttr("guide")  # collision only; the visible tube is the mesh below
        UsdPhysics.CollisionAPI.Apply(g.GetPrim())
        cp = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
        cp.CreateContactOffsetAttr(0.002)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(g.GetPath()), f"{root}/Looks/phys", stage=stage)

    def add_mesh(name, samples, r_in, r_out, mat):
        pts, quads = _tube_mesh(samples, r_in, r_out)
        mesh = UsdGeom.Mesh.Define(stage, f"{root}/{name}")
        mesh.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in pts])
        mesh.CreateFaceVertexCountsAttr([4] * len(quads))
        mesh.CreateFaceVertexIndicesAttr([int(i) for q in quads for i in q])
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        bind_visual_material(str(mesh.GetPath()), mat, stage=stage)

    ss = [0.0, L1] + [L1 + L2 * j / 24 for j in range(1, 24)] + [L1 + L2, LC]
    add_mesh("tube", [_ring_frame(s) for s in ss], RI, RO, f"{root}/Looks/galv")
    add_mesh("bushing_near", [_ring_frame(-BUSH_L), _ring_frame(0.002)], RI + 0.0008, BUSH_R, f"{root}/Looks/bushing")
    add_mesh("bushing_far", [_ring_frame(LC - 0.002), _ring_frame(LC + BUSH_L)], RI + 0.0008, BUSH_R,
             f"{root}/Looks/bushing")


def _install_parts() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_fish_parts", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        name = getattr(cfg, "cable_name", "")
        parts = _extra_parts(name)
        if not parts:
            return prim
        from isaaclab.sim.utils import bind_physics_material, bind_visual_material
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

        stage = get_current_stage()
        look = f"{prim_path}/Looks/end_plug"
        if name == CABLE.name:  # bare copper
            import isaaclab.sim as sim_utils

            from yam_arena.station import linear_rgb

            cu = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb((0.80, 0.45, 0.25)), roughness=0.3,
                                             metallic=0.5)
            cu.func(f"{prim_path}/Looks/copper", cu)
            look = f"{prim_path}/Looks/copper"
        for pname, c, s in parts:
            g = UsdGeom.Cube.Define(stage, f"{prim_path}/end_plug/{pname}")
            g.CreateSizeAttr(1.0)
            g.AddTranslateOp().Set(Gf.Vec3d(*map(float, c)))
            g.AddScaleOp().Set(Gf.Vec3f(*map(float, s)))
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            cp = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
            cp.CreateContactOffsetAttr(0.002)
            cp.CreateRestOffsetAttr(0.0)
            bind_physics_material(str(g.GetPath()), f"{prim_path}/Looks/cable_physics", stage=stage)
            bind_visual_material(str(g.GetPath()), look, stage=stage)
        if name == TAPE.name:
            _author_conduit(stage, prim_path.rsplit("/", 1)[0])
        return prim

    spawn._fish_parts = True
    cablekit._spawn_cable = spawn


_install_parts()


# -- starting layout: lay the tape through the conduit and the wire on the table at every reset ----------------------
def _shape(emb):
    cableshape.preshape(emb, TAPE, TAPE_PATH, hold="ramp", settle_s=0)
    cableshape.preshape(emb, CABLE, CABLE_PATH, hold="ramp", settle_s=0)
    cableshape.settle(emb, 2.0)


cableshape.on_reset(_shape)

# -- geometry used by scoring ----------------------------------------------------------------------------------------
HOOK_TOL = 0.009  # J leg crosses the eyelet plane within this of the eyelet centre (x and y, eyelet frame): through
# the 14 mm hole the leg axis is at most 5.5 mm from the centre, outside the eyelet at least 11.5 mm.
INSIDE = 0.02  # the wire end counts as inside once its J is 2 cm past the near mouth (along the conduit axis)
NEAR_HOOK = 0.10  # 0.25 tier: eyelet centre at most this far (straight line) from the near mouth's centre
MOUTH0 = np.array([X_NEAR, Y_NEAR, ZA])
OUT_GOAL = 0.05  # full score: J at least 5 cm past the far mouth
UNHOOKED_STEPS = 6  # J inside the conduit, off the eyelet, for this many consecutive checks (0.3 s) -> score 0
BORE_TOL = 0.004

_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    """Keep a handle on the sim env so scoring can read the end plugs' orientations (CableState has positions only)."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_fish_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._fish_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def _quat_matrix(x, y, z, w) -> np.ndarray:
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def plug_frame(state, name: str) -> np.ndarray:
    """Rotation matrix of ``name``'s end plug (columns: along the cable, across, up). Read from the sim when it is
    available; otherwise built from the end direction (no roll), for offline tests."""
    env = _ENV["env"]
    if env is not None:
        try:
            art = env.unwrapped.scene[name]
            i = list(art.body_names).index("end_plug")
            q = getattr(art.data.body_quat_w, "torch", art.data.body_quat_w)[0, i].detach().cpu().numpy()
            q = q.astype(np.float64)
            cands = [_quat_matrix(*q), _quat_matrix(q[1], q[2], q[3], q[0])]
            return max(cands, key=lambda m: float(m[:, 0] @ state.end_dir))
        except Exception:
            pass
    e = state.end_dir / (np.linalg.norm(state.end_dir) + 1e-12)
    lat = np.cross([0.0, 0.0, 1.0], e)
    lat /= np.linalg.norm(lat) + 1e-12
    return np.stack([e, lat, np.cross(e, lat)], axis=1)


def eyelet_centre(t) -> np.ndarray:
    m = plug_frame(t, "fish_tape")
    return t.end_plug + m[:, 0] * RING_X + m[:, 2] * RING_Z


def leg_axis(c) -> tuple[np.ndarray, np.ndarray]:
    """Top and bottom points of the J's leg axis (wire end)."""
    m = plug_frame(c, "cable")
    base = c.end_plug + m[:, 0] * J_X
    return base + m[:, 2] * LEG_TOP, base + m[:, 2] * LEG_BOT


def wire_end(c) -> np.ndarray:
    """Middle of the J's leg: the wire's leading point."""
    a, b = leg_axis(c)
    return (a + b) / 2


def hooked(objects) -> bool:
    """The J's leg passes through the eyelet's hole: in the eyelet's own frame, the leg axis crosses the eyelet's
    plane (between the leg's top and bottom) within HOOK_TOL of the eyelet centre along and across the eyelet."""
    c, t = objects["cable"], objects["fish_tape"]
    m = plug_frame(t, "fish_tape")
    ec = t.end_plug + m[:, 0] * RING_X + m[:, 2] * RING_Z
    a, b = (m.T @ (p - ec) for p in leg_axis(c))
    if (a[2] > 0) == (b[2] > 0) or abs(b[2] - a[2]) < 1e-6:
        return False
    s = a[2] / (a[2] - b[2])
    x = a + s * (b - a)
    return bool(abs(x[0]) <= HOOK_TOL and abs(x[1]) <= HOOK_TOL)


def in_bore(p, tol: float = BORE_TOL) -> bool:
    """Point inside the conduit's bore (within the bore radius plus ``tol`` of the axis, between the mouths)."""
    s, r = project(p)
    return bool(0.0 <= s <= LC and r <= RI + tol)


def depth(p) -> float:
    """Arc length (m) of a point along the conduit from the near mouth (negative before it, > LC past the far one)."""
    return project(p)[0]


def end_depth(c) -> float:
    """How far the wire's end has gone in, along the conduit axis from the near mouth: its arc length while it is
    within the bore (radius + 1 cm) or past the far mouth; -1 m anywhere else (e.g. on the table beside the run)."""
    s, r = project(wire_end(c))
    return float(s) if (r <= RI + 0.01 or s >= LC) else -1.0


def end_inside(objects) -> bool:
    w = wire_end(objects["cable"])
    s, r = project(w)
    return bool(r <= RI + 0.01 and INSIDE <= s <= LC)


def routed_from_near(objects) -> bool:
    """Following the wire back from its end, it runs inside the bore and leaves through the NEAR mouth: points past
    the far mouth are skipped, then every point must be inside the bore until the wire is back at the near mouth.
    False if it never enters, leaves the bore sideways, or comes in through the far mouth."""
    pts = objects["cable"].points[::-1]
    i, n = 0, len(pts)
    while i < n and depth(pts[i]) > LC - 0.005 and not in_bore(pts[i]):
        i += 1
    entered = False
    while i < n:
        s, r = project(pts[i])
        if s <= 0.005:
            return entered or end_depth(objects["cable"]) > 0.0
        if r > RI + BORE_TOL:
            return False
        entered = True
        i += 1
    return False


def _tape_points(t) -> np.ndarray:
    pts = [np.asarray(t.points, dtype=float)]
    if getattr(t, "start_plug", None) is not None:
        pts.insert(0, np.asarray(t.start_plug, dtype=float)[None])
    if getattr(t, "end_plug", None) is not None:
        pts.append(np.asarray(t.end_plug, dtype=float)[None])
    return np.vstack(pts)


def tape_in_bore(objects) -> bool:
    return any(in_bore(p) for p in _tape_points(objects["fish_tape"]))


def tape_threaded(objects) -> bool:
    """The fish tape still runs through the whole conduit: its handle end is out beyond the far mouth, its tip end
    out beyond the near mouth, and every part of it between the mouths is inside the bore."""
    pts = _tape_points(objects["fish_tape"])
    ss = [project(p) for p in pts]
    if not (ss[0][0] > LC and ss[-1][0] < 0.0):
        return False
    mid = [r for s, r in ss if 0.0 <= s <= LC]
    return len(mid) > 0 and all(r <= RI + BORE_TOL for r in mid)


def eyelet_to_near_mouth(objects) -> float:
    return float(np.linalg.norm(eyelet_centre(objects["fish_tape"]) - MOUTH0))


def end_out(objects) -> float:
    """How far (m, along the conduit's axis) the wire end is past the far mouth (negative = not yet out)."""
    return end_depth(objects["cable"]) - LC


# -- episode memory -----------------------------------------------------------------------------------------------
_EP = {"unhooked_run": 0, "pushed": False, "max_depth": -1.0, "withdrawn": False}


def _reset_episode() -> None:
    _EP.update(unhooked_run=0, pushed=False, max_depth=-1.0, withdrawn=False)


def _install_reset_hook() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls.reset, "_task_reset_hooks", None) is None:
        base_reset = cls.reset

        def reset(self, *a, **k):
            for fn in reset._task_reset_hooks:
                fn()
            return base_reset(self, *a, **k)

        reset._task_reset_hooks = []
        cls.reset = reset
    if _reset_episode not in cls.reset._task_reset_hooks:
        cls.reset._task_reset_hooks.append(_reset_episode)


_install_reset_hook()


def _track(objects) -> None:
    inside = end_inside(objects)
    if inside and not hooked(objects):
        _EP["unhooked_run"] += 1
        if _EP["unhooked_run"] >= UNHOOKED_STEPS:
            _EP["pushed"] = True
    else:
        _EP["unhooked_run"] = 0
    _EP["max_depth"] = max(_EP["max_depth"], end_depth(objects["cable"]))
    if _EP["max_depth"] < INSIDE and not tape_in_bore(objects):
        _EP["withdrawn"] = True  # the tape left the conduit before it had pulled the wire in


def score(objects) -> float:
    """Latched 0 (for the rest of the episode):
    - pushed: the wire's end (its J) was 2 cm or more inside the near mouth while off the eyelet for 0.3 s or longer;
    - withdrawn: no part of the fish tape was inside the bore at some step before the wire end had first been
      pulled 2 cm inside the near mouth.
    At the end, with the J's leg through the eyelet (hooked); otherwise 0:
    - wire end less than 2 cm inside the near mouth: 0.25 if the tape is still threaded through the whole conduit,
      the eyelet centre is within 10 cm of the near mouth's centre and no part of the wire is inside the bore; else 0;
    - wire end 2 cm or more inside: the wire must run from its end back through the bore and out of the near
      mouth, else 0; then 1.0 if the end is at least 5 cm past the far mouth, 0.75 if past it by less, 0.5 if still
      inside the conduit."""
    _track(objects)
    if _EP["pushed"] or _EP["withdrawn"] or not hooked(objects):
        return 0.0
    d = end_depth(objects["cable"])
    if d < INSIDE:
        if not tape_threaded(objects) or eyelet_to_near_mouth(objects) > NEAR_HOOK:
            return 0.0
        return 0.0 if any(in_bore(p) for p in objects["cable"].points) else 0.25
    if not routed_from_near(objects):
        return 0.0
    out = d - LC
    if out >= OUT_GOAL:
        return 1.0
    if out >= 0.0:
        return 0.75
    return 0.5


def success(objects) -> bool:
    s = score(objects)
    return bool(s >= 1.0 and objects["cable"].resting(0.02, 1.0) and objects["fish_tape"].resting(0.02, 1.0))


def distance(objects) -> float:
    """Distance (m, along the conduit) the wire end still has to travel to be 5 cm past the far mouth; 1 m after a
    latched 0."""
    _track(objects)
    if _EP["pushed"] or _EP["withdrawn"]:
        return 1.0
    return float(max(0.0, LC + OUT_GOAL - end_depth(objects["cable"])))


def progress(objects) -> dict:
    """Stage diagnostic for analysis (not the score)."""
    h = hooked(objects)
    d = end_depth(objects["cable"])
    return {
        "end_hooked": int(h),
        "tape_threaded": int(tape_threaded(objects)),
        "hooked_near_mouth": int(h and eyelet_to_near_mouth(objects) <= NEAR_HOOK),
        "end_entered": int(h and d >= INSIDE),
        "end_past_sweep": int(h and d >= L1 + L2),
        "end_out": float(np.clip((d - LC) / OUT_GOAL, 0.0, 1.0)) if h else 0.0,
        "routed_through_bore": int(routed_from_near(objects)),
        "not_pushed_by_hand": int(not _EP["pushed"]),
        "tape_not_withdrawn_early": int(not _EP["withdrawn"]),
    }


# Seeded variation ranges for a later runner (the demo runs on the nominal layout and reads live states).
VARIATION = {
    "cable:cable": {"dx": 0.015, "dy": 0.015, "yaw": 0.0},
    "cable:fish_tape": {"dx": 0.0, "dy": 0.0, "yaw": 0.0},  # laid through the fixed conduit at every reset
}


# -- scripted demo (cabledemo.py) -------------------------------------------------------------------------------------
# Scripted demos may read the simulator's object poses to plan (reviewer prompt v21). This one plans from the fixed
# layout above and the arms' own TCPs, and reads the wire end's and the eyelet's poses from ctx.states() to place
# the J in the eyelet and to check each stage.

HI = TOP + 0.11  # travel height
GRASP_OPEN = -0.014  # pads ~28 mm apart
GRIP_OUT = 0.04  # the left arm grips the bare tape this far out of the far mouth
STROKE = 0.20
DUMP = os.environ.get("FISH_DUMP", "")


def _log(ctx, tag=""):
    st = ctx.states()
    c, t = st["cable"], st["fish_tape"]
    w, ec = wire_end(c), eyelet_centre(t)
    print(f"[t={ctx.t:5.2f}] {tag}: end={np.round(w, 4)} eyelet={np.round(ec, 4)} hooked={hooked(st)} "
          f"threaded={tape_threaded(st)} d_mouth={eyelet_to_near_mouth(st):.3f} depth={end_depth(c):.3f} "
          f"withdrawn={_EP['withdrawn']} inside={end_inside(st)} routed={routed_from_near(st)} "
          f"out={end_out(st):.3f} pushed={_EP['pushed']} score={score(st):.2f} "
          f"tcpR={np.round(ctx.right.tcp(), 4)} tcpL={np.round(ctx.left.tcp(), 4)}", flush=True)


def _tape_z_at_y(ctx, y):
    """Height of the tape's centreline where it crosses ``y`` (outside the far mouth)."""
    pts = ctx.states()["fish_tape"].points
    k = int(np.argmin(np.abs(pts[:, 1] - y) + 10 * np.abs(pts[:, 0] - X_FAR)))
    return float(pts[k, 2]), float(pts[k, 0])


def _grip_tape(ctx, L, y, f_closed):
    """Pinch the bare tape at ``y`` beyond the far mouth and hold it."""
    z, x = _tape_z_at_y(ctx, y)
    yield from L.move((x, y, HI), yaw=-math.pi / 2, finger=GRASP_OPEN, duration=1.0)
    yield from L.move((x, y, z + 0.009), yaw=-math.pi / 2, duration=0.8)
    yield from L.grip(f_closed, 0.6)


def _pull(ctx, L, f_closed, *, holding=False, strokes=4, lift=0.004, last=STROKE):
    """Hand over hand along +y: pull STROKE (the last stroke ``last``), let go, reach back to GRIP_OUT beyond the
    far mouth and regrip."""
    for n in range(strokes):
        length = last if n == strokes - 1 else STROKE
        if not holding:
            yield from _grip_tape(ctx, L, Y_FAR + GRIP_OUT, f_closed)
        holding = False
        p = L.tcp()
        steps = 10
        for i in range(steps):
            yield from L.move((X_FAR, p[1] + length * (i + 1) / steps, p[2] + lift), yaw=-math.pi / 2, duration=0.5)
        _log(ctx, f"stroke {n + 1}")
        yield from L.grip(GRASP_OPEN, 0.4)
        p = L.tcp()
        yield from L.move((p[0], p[1], HI), yaw=-math.pi / 2, duration=0.6)


def _pick_end(ctx, Rt, f_open, f_closed):
    """Pinch the wire's insulated end (the rigid last 30 mm) where it lies and lift it, turned to point +x."""
    c = ctx.states()["cable"]
    e = c.end_plug
    h = math.atan2(c.end_dir[1], c.end_dir[0])
    gy = min(max(h, -math.pi / 2), math.pi / 2)  # grip yaw = heading, so turning to yaw 0 points the end +x
    yield from Rt.move((e[0], e[1], HI), yaw=gy, finger=f_open, duration=1.5)
    yield from Rt.move((e[0], e[1], e[2] + 0.009), yaw=gy, duration=1.0)
    yield from Rt.grip(f_closed, 0.6)
    yield from Rt.move((e[0], e[1], HI), yaw=gy, duration=0.8)
    yield from Rt.move((e[0] - 0.05, e[1] + 0.02, HI), yaw=0.0, duration=1.2)  # wire end now points +x
    yield from Rt.wait(0.3)
    _log(ctx, "end picked")


def _leg_offset(ctx, Rt):
    """J leg top point minus the commanded TCP (measured from the live state)."""
    c = ctx.states()["cable"]
    return leg_axis(c)[0] - Rt.tcp()


def _hook_end(ctx, Rt, f_open, shift=0.0065, entry=0.0025, tail_side=-1.0, eyelet=None):
    """Bring the J over the eyelet (leg ``entry`` m to the mouth side of the hole centre), lower it through, slide it
    ``shift`` m back so the folded tail goes under the eyelet's outer bar, and let go."""
    t = ctx.states()["fish_tape"]
    ec = eyelet_centre(t) if eyelet is None else np.asarray(eyelet, float)
    m = plug_frame(t, "fish_tape")
    out = m[:, 0] * np.array([1.0, 1.0, 0.0])
    out /= np.linalg.norm(out)  # horizontal direction from the eyelet's inner bar to its outer bar
    leg_tgt = ec - out * entry
    for k in range(3):
        off = _leg_offset(ctx, Rt)
        stage = leg_tgt + np.array([0.0, 0.0, LEG_TOP - LEG_BOT + 0.008])
        tcp = stage - off
        yield from Rt.move(tcp, yaw=0.0, duration=1.2 if k == 0 else 0.6)
        yield from Rt.wait(0.4)
        err = leg_axis(ctx.states()["cable"])[0] - stage
        _log(ctx, f"staged {k}: leg err {np.round(err * 1000, 1)} mm")
        if np.linalg.norm(err[:2]) < 0.0008:
            break
    p = Rt.tcp()
    down = (ec[2] + RING_T / 2 + 0.0035) - (leg_axis(ctx.states()["cable"])[0][2] - LEG_TOP)  # wire axis drop
    low = p + np.array([0.0, 0.0, down])
    yield from Rt.move(low, yaw=0.0, duration=1.0)
    yield from Rt.wait(0.3)
    _log(ctx, "lowered through")
    yield from Rt.move(low + out * shift, yaw=0.0, duration=0.8)
    yield from Rt.wait(0.3)
    _log(ctx, "slid back")
    yield from Rt.grip(f_open, 0.5)
    p = Rt.tcp()
    yield from Rt.move((p[0], p[1], p[2] + 0.04), yaw=0.0, duration=0.8)
    yield from Rt.move((0.20, -0.42, HI), yaw=0.0, duration=1.0)


def demo_script(ctx):
    """FISH_DEMO=correct (default) | nothing | tape_only | push | away (see README)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    mode = os.environ.get("FISH_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    _log(ctx, "start")
    if mode == "nothing":
        yield from L.wait(5.0)
        _log(ctx, "end")
        return
    if mode == "push":
        # the tape is pulled out of the conduit first (its tip would block the wire), then the right arm pushes the
        # wire end into the empty near mouth by hand, as far as it reaches
        yield from _pull(ctx, L, FINGER_CLOSED, strokes=4)
        yield from L.move((0.25, 0.30, HI), yaw=-math.pi / 2, duration=1.0)
        yield from _pick_end(ctx, Rt, FINGER_OPEN, FINGER_CLOSED)
        off = _leg_offset(ctx, Rt)
        z = ZA - (LEG_TOP + LEG_BOT) / 2 - 0.001
        yield from Rt.move(np.array([X_NEAR - 0.04, Y_NEAR, z]) - off * np.array([1, 1, 0]) + [0, 0, 0.0], yaw=0.0,
                           duration=1.2)
        for i in range(8):
            yield from Rt.move(np.array([X_NEAR - 0.04 + 0.012 * (i + 1), Y_NEAR, z]) - off * np.array([1, 1, 0]),
                               yaw=0.0, duration=0.4)
        _log(ctx, "pushed in")
        yield from Rt.grip(FINGER_OPEN, 0.5)
        yield from Rt.move((0.20, -0.42, HI), yaw=0.0, duration=1.0)
        yield from Rt.wait(1.0)
        _log(ctx, "end")
        return
    if mode == "away":
        # the right arm slides the tape 10 cm out of the near mouth by its tip (still threaded), then hooks the
        # wire on there, ~14 cm from the mouth
        t = ctx.states()["fish_tape"]
        p0 = t.end_plug
        yield from Rt.move((p0[0], p0[1], HI), yaw=0.0, finger=GRASP_OPEN, duration=1.5)
        yield from Rt.move((p0[0], p0[1], p0[2] + 0.009), yaw=0.0, duration=1.0)
        yield from Rt.grip(FINGER_CLOSED, 0.6)
        for i in range(10):
            yield from Rt.move((p0[0] - 0.01 * (i + 1), p0[1], p0[2] + 0.010), yaw=0.0, duration=0.3)
        yield from Rt.grip(FINGER_OPEN, 0.5)
        yield from Rt.move((p0[0] - 0.10, p0[1], HI), yaw=0.0, duration=0.6)
        _log(ctx, "tape slid out")
        yield from _pick_end(ctx, Rt, FINGER_OPEN, FINGER_CLOSED)
        yield from _hook_end(ctx, Rt, FINGER_OPEN)
        yield from L.wait(1.0)
        _log(ctx, "end")
        return
    # Correct: the left arm pinches the tape beyond the far mouth and holds it while the right arm hooks the wire's
    # J through the eyelet; then the left arm pulls the tape back hand over hand until the wire end is well out.
    holding = False
    if mode != "tape_only":
        def right():
            yield from _pick_end(ctx, Rt, FINGER_OPEN, FINGER_CLOSED)

        yield from ctx.both(_grip_tape(ctx, L, Y_FAR + GRIP_OUT, FINGER_CLOSED), right())
        yield from _hook_end(ctx, Rt, FINGER_OPEN)
        _log(ctx, "hooked")
        holding = True
    # partial-credit checks: FISH_STROKES=0 (hooked only), 2 (inside), 4 with FISH_LAST=0.10 (just out)
    n = int(os.environ.get("FISH_STROKES", "4"))
    if n > 0:
        yield from _pull(ctx, L, FINGER_CLOSED, holding=holding, strokes=n,
                         last=float(os.environ.get("FISH_LAST", STROKE)))
    else:  # hooked, tape still held at the far mouth, nothing pulled
        yield from L.wait(2.0)
        _log(ctx, "end")
        return
    yield from L.move((0.25, 0.30, HI), yaw=-math.pi / 2, duration=1.0)
    for _ in range(3):
        yield from L.wait(0.5)
    _log(ctx, "end")
