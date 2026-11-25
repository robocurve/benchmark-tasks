"""Games table after play: an open 50 x 50 cm quad-fold game board lies face up on a 16 in board-game turntable (lazy
susan), its edges overhanging the turntable all round. The game's box base, with its card insert, stands on the right.

The task: fold the board up and lay it in the box, then park both arms clear. The board is four rigid 25 cm quarters.
The long crease (across the table, x = 30 cm) has two hinges: near-left/far-left and near-right/far-right. The short
crease (along the table, y = 2 cm) has one hinge, near-left/near-right, and a cut between the far quarters. Every
hinge only folds face-in (its face paper is cut at the hinge; the back wrap is the hinge), so the board closes flat in
one order only: both far quarters over first, then the stacked left half over onto the right half.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score --instruction "<INSTRUCTION>"
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import sys

import numpy as np
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
RUN_LIMIT_STEPS = 1500  # 75 s at 20 Hz (scripted demo reaches 1.0 in 837 steps; ceil(1.5 * 837 / 300) * 300 = 1500)

INSTRUCTION = "Fold the game board and put it in its box."

BAMBOO = (0.80, 0.62, 0.38)
BAMBOO_DARK = (0.55, 0.40, 0.22)
BACK_COL = (0.10, 0.12, 0.20)  # the board's dark back wrap
FACE_COLS = {"nl": (0.42, 0.66, 0.38), "fl": (0.35, 0.55, 0.80), "nr": (0.90, 0.78, 0.45), "fr": (0.85, 0.42, 0.38)}
TRACK_COL = (0.96, 0.94, 0.86)
BOX_COL = (0.70, 0.12, 0.12)
INSERT_COL = (0.92, 0.90, 0.84)

# -- turntable: 16 in (40.6 cm) bamboo lazy susan, 3.2 cm tall ---------------------------------------------------------
TTX, TTY = 0.30, 0.02
TT_R = 0.203
TT_H = 0.032  # deck top above the table
TT_BASE_H = 0.012  # bearing base under the deck
TT_DECK_T = TT_H - TT_BASE_H
TT_DECK_MASS = 1.2
TT_DAMP = float(os.environ.get("QF_TT_DAMP", "0.004"))  # N m s/deg: ball-bearing drag

TT_DECK = Cylinder("tt_deck", pos=(TTX, TTY, TOP + TT_BASE_H + TT_DECK_T / 2), radius=TT_R, height=TT_DECK_T,
                   mass=TT_DECK_MASS, color=BAMBOO)

# -- board: four 25 x 25 cm quarters of 2.2 mm greyboard, 90 g each ----------------------------------------------------
Q = 0.25
T = 0.0022
GAP = 0.0006  # seam across the long crease
GAP_Y = 0.0025  # seam along the short crease (the cut between the far quarters is a 2.5 mm slit)
PANEL_MASS = 0.09
Z0 = TOP + TT_H  # board underside
HX = TTX  # long crease (along y)
HY = TTY  # short crease (along x)
_C = {"nl": (-1, 1), "fl": (1, 1), "nr": (-1, -1), "fr": (1, -1)}  # (sx, sy) of each quarter's centre
PANELS = {k: Box(f"q_{k}", pos=(HX + sx * (Q + GAP) / 2, HY + sy * (Q + GAP_Y) / 2, Z0 + T / 2), size=(Q, Q, T),
                 mass=PANEL_MASS, color=BACK_COL) for k, (sx, sy) in _C.items()}
P_NAMES = [p.name for p in PANELS.values()]

# hinges: (key, body0 (stays), body1 (folds), axis, world point on the axis, sign: fold angle = sign * joint angle)
# long crease: the far quarter turns up and over toward the robot about -y (joint angle about +y goes negative);
# axis on the board's top face, so a closed quarter lies face down exactly on its near quarter.
# short crease: the near-left quarter (with the far-left one folded on it) turns up and over to the right about +x;
# axis on the top of the two-layer half, so the closed half lies exactly on the right half.
HINGES = [
    ("long_left", "nl", "fl", "Y", (HX, HY + Q / 2, Z0 + T), -1.0),
    ("long_right", "nr", "fr", "Y", (HX, HY - Q / 2, Z0 + T), -1.0),
    ("short", "nr", "nl", "X", (HX - Q / 2, HY, Z0 + 2 * T), 1.0),
]
LIMIT_BACK = 2.0  # deg it can bend face-out (slop) before the stop
HINGE_FMAX = 0.3  # N m: cap on the hinge drag torque
HINGE_DAMP = float(os.environ.get("QF_HDAMP", "0.003"))  # N m s/deg: drag of the paper-wrap hinge

# -- the game's box base: 27.5 x 27.5 x 5.5 cm outer, card insert top 1.2 cm below the rim -----------------------------
BX, BY = float(os.environ.get("QF_BX", "0.30")), float(os.environ.get("QF_BY", "-0.44"))
B_OUT, B_H, B_WALL = 0.275, 0.055, 0.005
B_IN = B_OUT - 2 * B_WALL  # 26.5 cm
B_FLOOR = 0.003
INSERT_TOP = B_H - 0.012  # 4.3 cm above the table
BOX_MASS = 0.25  # box base and insert

BOX = Box("box_base", pos=(BX, BY, TOP + B_FLOOR / 2), size=(B_OUT, B_OUT, B_FLOOR), mass=BOX_MASS, color=BOX_COL)


class LastPart(Box):
    """The turntable's bearing base (fixed, spawned last); its spawner also authors the board's hinges, the box walls
    and insert, and the turntable bearing."""


TT_BASE = LastPart("tt_base", pos=(TTX, TTY, TOP + TT_BASE_H / 2), size=(0.24, 0.24, TT_BASE_H), color=BAMBOO_DARK)

OBJECTS = [TT_DECK, *PANELS.values(), BOX, TT_BASE]

TWIN_EXTRA = [
    {"name": "board_hinges", "kind": "joint", "real": "the board's paper-wrap hinges",
     "note": "revolute, fold face-in 0-180 deg, stop 2 deg past flat the other way; viscous drag"},
    {"name": "turntable_bearing", "kind": "joint", "real": "the lazy susan's ball bearing",
     "note": "revolute about z with light damping"},
    {"name": "grasp_weld", "kind": "joint", "real": "finger-pad friction on card",
     "note": "engages only after both opposing finger pads physically contact and pinch a board edge inside its outline"},
]

# -- Isaac side ---------------------------------------------------------------------------------------------------------
_P: dict = {}


def _box_parts():
    """(name, centre in box-base body frame, size, colour, collide)."""
    hz = B_H / 2 - B_FLOOR / 2  # walls run from the floor's underside to the rim
    out = []
    for s in (-1, 1):
        out.append((f"wall_x{'mp'[s > 0]}", (s * (B_OUT - B_WALL) / 2, 0.0, hz), (B_WALL, B_OUT, B_H), BOX_COL, True))
        out.append((f"wall_y{'mp'[s > 0]}", (0.0, s * (B_OUT - B_WALL) / 2, hz), (B_IN, B_WALL, B_H), BOX_COL, True))
    ih = INSERT_TOP - B_FLOOR
    out.append(("insert", (0.0, 0.0, B_FLOOR / 2 + ih / 2), (B_IN - 0.001, B_IN - 0.001, ih), INSERT_COL, True))
    return out


def _panel_parts(key):
    """Visual face print of a quarter: a coloured field and a light track along its outer edges."""
    sx, sy = _C[key]
    out = [("face", (0.0, 0.0, 0.0002), (Q - 0.006, Q - 0.006, T), FACE_COLS[key])]
    w = 0.03
    out.append(("track_x", (sx * (Q / 2 - 0.003 - w / 2), 0.0, 0.0003), (w, Q - 0.006, T), TRACK_COL))
    out.append(("track_y", (0.0, sy * (Q / 2 - 0.003 - w / 2), 0.0003), (Q - 0.006, w, T), TRACK_COL))
    return out


def _author(stage, parent: str) -> None:
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    w = UsdGeom.Xformable(stage.GetPrimAtPath(parent)).ComputeLocalToWorldTransform(0).ExtractTranslation()
    W = np.array([w[0], w[1], w[2]], dtype=float)

    def v(p):
        return Gf.Vec3f(*(float(x) for x in p))

    def cube(path, c, size, col, collide, mat=None):
        g = UsdGeom.Cube.Define(stage, path)
        g.CreateSizeAttr(1.0)
        g.CreateDisplayColorAttr([Gf.Vec3f(*col)])
        x = UsdGeom.Xformable(g)
        x.AddTranslateOp().Set(Gf.Vec3d(*map(float, c)))
        x.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        if collide:
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            if mat:
                UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(mat, UsdShade.Tokens.weakerThanDescendants,
                                                                    "physics")

    def body_mat(body):
        for prim in Usd.PrimRange(stage.GetPrimAtPath(f"{parent}/{body}")):
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                m = UsdShade.MaterialBindingAPI(prim).GetDirectBinding(materialPurpose="physics").GetMaterial()
                if m:
                    return m
        return None

    bm = body_mat(BOX.name)
    for name, c, size, col, collide in _box_parts():
        cube(f"{parent}/{BOX.name}/{name}", c, size, col, collide, bm)
    for key, p in PANELS.items():
        for name, c, size, col in _panel_parts(key):
            cube(f"{parent}/{p.name}/{name}", c, size, col, False)
        _P[p.name] = f"{parent}/{p.name}"

    UsdGeom.Xform.Define(stage, f"{parent}/{TT_BASE.name}/qf_joints")

    def joint(name, b0, b1, p_world, axis, lo, hi, rot0=0.0):
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{parent}/{TT_BASE.name}/qf_joints/{name}")
        p_world = np.asarray(p_world, float)
        if b0 is None:
            j.CreateLocalPos0Attr(v(W + p_world))
        else:
            j.CreateBody0Rel().SetTargets([Sdf.Path(f"{parent}/{b0.name}")])
            j.CreateLocalPos0Attr(v(p_world - np.array(b0.pos)))
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{parent}/{b1.name}")])
        j.CreateLocalPos1Attr(v(p_world - np.array(b1.pos)))
        h = math.radians(rot0) / 2  # joint frame 0 turned about the axis: keeps the 0-180 fold clear of +-180
        ax = {"X": (1, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 1)}[axis]
        j.CreateLocalRot0Attr(Gf.Quatf(math.cos(h), *(math.sin(h) * a for a in ax)))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateAxisAttr(axis)
        if lo is not None:
            j.CreateLowerLimitAttr(float(lo))
            j.CreateUpperLimitAttr(float(hi))
        j.CreateExcludeFromArticulationAttr(True)
        _P[name] = str(j.GetPath())
        return j

    def drive(j, k, c, target, fmax):
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(float(k))
        d.CreateDampingAttr(float(c))
        d.CreateTargetPositionAttr(float(target))
        d.CreateTargetVelocityAttr(0.0)
        d.CreateMaxForceAttr(float(fmax))

    for key, b0, b1, axis, p, s in HINGES:
        # PhysX joint angle = s * fold - s * 90 (frame offset), so flat is -s*90 and closed +s*90
        lo, hi = sorted((s * (-LIMIT_BACK) - s * 90.0, s * 180.0 - s * 90.0))
        j = joint(key, PANELS[b0], PANELS[b1], p, axis, lo, hi, rot0=s * 90.0)
        drive(j, 0.0, HINGE_DAMP, -s * 90.0, HINGE_FMAX)
    j = joint("tt_bearing", None, TT_DECK, (TTX, TTY, TOP + TT_BASE_H), "Z", None, None)
    drive(j, 0.0, TT_DAMP, 0.0, 1.0)
    # the two far quarters meet only across the cut: their edges slide past each other there (and the closed board's
    # far-left quarter lies on the far-right one only through the hinges), so they don't collide with each other
    fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(f"{parent}/{PANELS['fl'].name}"))
    fp.CreateFilteredPairsRel().AddTarget(Sdf.Path(f"{parent}/{PANELS['fr'].name}"))
    for b in PANELS.values():
        cr = PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(f"{parent}/{b.name}"))
        cr.CreateThresholdAttr(0.0)
    for b in (*PANELS.values(), BOX, TT_DECK):
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(f"{parent}/{b.name}"))
        rb.CreateSolverPositionIterationCountAttr(16)
        rb.CreateSolverVelocityIterationCountAttr(2)
    _P["parent"] = parent
    _CV.clear()
    if os.environ.get("QF_DEBUG"):
        print(f"[quadfold] authored under {parent}; env origin {W}", flush=True)


def _spawn_last(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from isaaclab.sim.utils.stage import get_current_stage

    prim = spawn_cuboid(prim_path, cfg, translation, orientation, **kwargs)
    _author(get_current_stage(), str(prim_path).rsplit("/", 1)[0])
    return prim


def _last_object(spec: LastPart):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.schemas import CollisionPropertiesCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from yam_arena.constants import GRIPPY_FRICTION
    from yam_arena.station import linear_rgb

    _register_module()

    @configclass
    class LastCfg(sim_utils.CuboidCfg):
        func: object = clone(_spawn_last)

    cfg = LastCfg(size=tuple(spec.size), collision_props=CollisionPropertiesCfg(),
                  physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=GRIPPY_FRICTION[0],
                                                                  dynamic_friction=GRIPPY_FRICTION[1]),
                  visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.6))
    return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=cfg,
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


def _register_module():
    if __name__ not in sys.modules:  # configclass needs this module registered (task files load unregistered)
        import types

        mod = types.ModuleType(__name__)
        mod.__dict__.update(globals())
        sys.modules[__name__] = mod


def _patch_scene(scene) -> None:
    if getattr(scene, "_quadfold_patched", False):
        return
    orig = scene.task_object

    def task_object(spec):
        if isinstance(spec, LastPart):
            return _last_object(spec)
        return orig(spec)

    scene.task_object = task_object
    scene._quadfold_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    _quadfold = True

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


def _install() -> None:
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    elif not any(getattr(f, "_quadfold", False) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


_install()

# -- episode state and hooks ---------------------------------------------------------------------------------------------
_ENV: dict = {"env": None}
CONTROL_HZ = 20.0
PARK_STEPS = 10  # 0.5 s at 20 Hz


class _Episode:
    def __init__(self):
        self.grip_hist = []  # gripper points (world, env frame) over the last PARK_STEPS + 1 control steps
        self.backward = False
        self.off_table = ""
        self.welds = {"left": [], "right": []}  # panel names fixed to each gripper
        self.weld_span = {"left": 0.0, "right": 0.0}
        self.stall_steps = {"left": 0, "right": 0}
        self.was_open = {"left": True, "right": True}  # jaws must open before acquiring a new edge pinch


_EP = _Episode()


def _stage():
    from isaaclab.sim.utils.stage import get_current_stage

    return get_current_stage()


def reset_episode() -> None:
    global _EP
    try:
        st = _stage()
        joints_root = st.GetPrimAtPath(f"{_P.get('parent', '')}/{TT_BASE.name}/qf_joints")
        if joints_root and joints_root.IsValid():
            for ch in joints_root.GetChildren():
                if ch.GetName().startswith("weld_"):
                    at = ch.GetAttribute("physics:jointEnabled")
                    if at and at.IsValid():
                        at.Set(False)
    except Exception:  # noqa: BLE001
        pass
    _EP = _Episode()


def _np(t):
    if hasattr(t, "torch"):
        t = t.torch
    if hasattr(t, "detach"):
        return t.detach().cpu().numpy()
    if hasattr(t, "numpy"):
        return np.asarray(t.numpy())
    return np.asarray(t)


def _robot_parts(side):
    base = _ENV["env"].unwrapped
    o = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    rob = base.scene[f"{side}_robot"]
    return base, o, rob


def _gripper_path(side):
    key = f"gripper_path_{side}"
    if key in _P:
        return _P[key]
    from pxr import UsdPhysics

    tag = "/LeftArm" if side == "left" else "/RightArm"
    for prim in _stage().Traverse():
        ps = prim.GetPath().pathString
        if "/env_0/" in ps and tag in ps and ps.endswith("/gripper") and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            _P[key] = ps
            return ps
    return None


WELD_INSET = 0.001  # pad contact point (tcp..pad_tips) must lie >= 1 mm strictly inside the 25 x 25 cm outline ...
WELD_EDGE = 0.038  # ... and within 3.8 cm of an outer edge
PAD_CONTACT_N = 0.5  # N: minimum PhysX normal force required on BOTH opposing finger links
STALL_PEEL_M = 0.045  # m: TCP tracking lag under 7 N m wrist limit that peels a jammed pinch (3 consecutive steps)
_CV: dict = {}


def _pad_contact_view():
    if "view" in _CV or "err" in _CV:
        return _CV.get("view")
    try:
        from isaaclab_physx.physics.physx_manager import PhysxManager
        from pxr import UsdPhysics

        sv = PhysxManager.get_physics_sim_view()
        st = _stage()
        tips = {}
        panels = {}
        for prim in st.Traverse():
            ps = prim.GetPath().pathString
            if "/env_0/" not in ps or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            for side, tag in (("left", "/LeftArm/"), ("right", "/RightArm/")):
                for fn in ("tip_left", "tip_right"):
                    if tag in ps and ps.endswith(f"/{fn}"):
                        tips[(side, fn)] = ps
            for n in P_NAMES:
                if ps.endswith(f"/{n}"):
                    panels[n] = ps
        sensors = [tips[("left", "tip_left")], tips[("left", "tip_right")],
                   tips[("right", "tip_left")], tips[("right", "tip_right")]]
        flt = [panels[n] for n in P_NAMES]
        _CV["view"] = sv.create_rigid_contact_view(sensors, filter_patterns=[flt] * 4, max_contact_data_count=128)
    except Exception as e:  # noqa: BLE001
        _CV["err"] = repr(e)
        if os.environ.get("QF_DEBUG"):
            print(f"[quadfold] contact view init failed: {e!r}", flush=True)
    return _CV.get("view")


def _pad_board_forces(side: str) -> np.ndarray | None:
    """(2, 4) normal contact force magnitudes [N] between (tip_left, tip_right) of ``side`` and each board quarter."""
    v = _pad_contact_view()
    if v is None:
        return None
    from isaaclab_physx.physics.physx_manager import PhysxManager

    dt = PhysxManager.get_physics_dt()
    fm = _np(v.get_contact_force_matrix(dt=dt)).reshape(4, len(P_NAMES), 3)
    rows = fm[:2] if side == "left" else fm[2:]
    return np.linalg.norm(rows, axis=-1)


def _jaw_state(side: str) -> tuple[float, float]:
    """(measured pad-to-pad separation ap [m], commanded pad-to-pad separation cmd [m]) from joint7 and joint8."""
    _, _, rob = _robot_parts(side)
    names = list(rob.joint_names)
    j7, j8 = names.index("joint7"), names.index("joint8")
    tgt = rob.data.joint_pos_target
    pos = rob.data.joint_pos
    qt = _np(tgt.torch[0] if hasattr(tgt, "torch") else tgt[0]).reshape(-1)
    qp = _np(pos.torch[0] if hasattr(pos, "torch") else pos[0]).reshape(-1)
    ap = abs(float(qp[j7])) + abs(float(qp[j8]))
    cmd = abs(float(qt[j7])) + abs(float(qt[j8]))
    return ap, cmd


def _edge_grip(objects, side: str, ap: float, cmd: float) -> tuple[list[str], float]:
    """Return (contacted_outer_quarters, pinched_thickness_m) ONLY for the 1 or 2 board quarters directly contacted
    (>= 0.5 N) by opposing finger pads (tip_left on one outer face, tip_right on the opposite outer face). Ungrasped
    inner or nearby quarters are never welded."""
    from yam_arena.runtime import link_poses, pad_tips_world, tcp_world

    forces = _pad_board_forces(side)
    if forces is None:
        return [], 0.0
    _, o, rob = _robot_parts(side)
    tcp = np.asarray(tcp_world(rob), float) - o
    tips = np.asarray(pad_tips_world(rob), float) - o
    pad_pts = (tcp, 0.5 * (tcp + tips), tips)
    cx = np.asarray(link_poses(rob)["gripper"][:3, 0], float)
    cand = []
    for idx, n in enumerate(P_NAMES):
        p, R = _pose(objects, n)
        if p is None:
            continue
        if abs(float(cx @ R[:, 2])) < 0.55:  # jaws must close across the plate faces (within ~56 deg of normal)
            continue
        best_z = None
        for pt in pad_pts:
            loc = R.T @ (pt - p)
            dx, dy = Q / 2 - abs(float(loc[0])), Q / 2 - abs(float(loc[1]))
            if abs(float(loc[2])) <= 4 * T + 0.012 and WELD_INSET <= min(dx, dy) <= WELD_EDGE:
                best_z = abs(float(loc[2])) if best_z is None else min(best_z, abs(float(loc[2])))
        if best_z is None:
            continue
        cand.append((best_z, idx, n, p, R))
    if not cand:
        return [], 0.0
    flush = math.cos(math.radians(5.0))
    best_pair = None
    for _, ia, na, pa, Ra in cand:
        if float(forces[0, ia]) < PAD_CONTACT_N:
            continue
        n_ax = Ra[:, 2]
        for _, ib, nb, pb, Rb in cand:
            if float(forces[1, ib]) < PAD_CONTACT_N:
                continue
            if abs(float(Rb[:, 2] @ n_ax)) < flush:
                continue
            sep = abs(float(n_ax @ (pb - pa)))
            if sep > 3 * T + 0.0030:
                continue
            w = sep + T
            if cmd > max(0.0020, w) or not (0.0005 <= ap <= w + 0.0200):
                continue
            o_mid = 0.5 * (pa + pb)
            z_pad = min(abs(float(n_ax @ (pt - o_mid))) for pt in pad_pts)
            if z_pad > w / 2 + 0.0120:
                continue
            score_pair = (sep, float(forces[0, ia]) + float(forces[1, ib]))
            if best_pair is None or score_pair > best_pair[0]:
                names = [na] if na == nb else [na, nb]
                best_pair = (score_pair, names, w)
    if best_pair is None:
        return [], 0.0
    return best_pair[1], best_pair[2]


def _tcp_lag(side: str) -> float:
    """Cartesian lag [m] between commanded arm target and live TCP (rises when an external jam stalls the 7 N m wrist)."""
    from yam_arena import ik
    from yam_arena.constants import ARM_JOINTS
    from yam_arena.runtime import tcp_world

    _, o, rob = _robot_parts(side)
    ids = [list(rob.joint_names).index(j) for j in ARM_JOINTS]
    tgt = rob.data.joint_pos_target
    qt = _np(tgt.torch[0] if hasattr(tgt, "torch") else tgt[0]).reshape(-1)[ids]
    base_pos = _np(rob.data.root_pos_w)[0] - o
    p_cmd, _ = ik.tcp_pose(qt, base_pos)
    p_live = np.asarray(tcp_world(rob), float) - o
    return float(np.linalg.norm(p_live - p_cmd))


def _weld_update(objects) -> None:
    """Bounded contact-pinch aid: fixes ONLY the 1 or 2 outer quarters in direct opposing finger-pad contact (>= 0.5 N
    on tip_left and tip_right) until jaws open or a sustained external jam (> 4.5 cm TCP lag for 3 steps under the
    7 N m wrist torque cap) peels the grip. Inner folded layers are never welded."""
    from pxr import Gf, Sdf, UsdPhysics
    from scipy.spatial.transform import Rotation
    from yam_arena.runtime import link_poses

    st = _stage()
    for side in ("left", "right"):
        ap, cmd = _jaw_state(side)
        cur = _EP.welds[side]
        w_cur = _EP.weld_span[side]
        if cur:
            _EP.stall_steps[side] = _EP.stall_steps[side] + 1 if _tcp_lag(side) > STALL_PEEL_M else 0
            if _EP.stall_steps[side] >= 3 or cmd > max(0.0040, w_cur) or ap > w_cur + 0.0240:
                for n in cur:
                    j = st.GetPrimAtPath(f"{_P['parent']}/{TT_BASE.name}/qf_joints/weld_{side}_{n}")
                    if j and j.IsValid():
                        j.GetAttribute("physics:jointEnabled").Set(False)
                _EP.welds[side] = []
                _EP.weld_span[side] = 0.0
                _EP.stall_steps[side] = 0
                _EP.was_open[side] = ap >= 0.028
            continue
        if ap >= 0.028:
            _EP.was_open[side] = True
        if not _EP.was_open[side]:
            continue
        names, w_span = _edge_grip(objects, side, ap, cmd)
        if not names:
            continue
        gpath = _gripper_path(side)
        if gpath is None:
            continue
        _, o, rob = _robot_parts(side)
        g = link_poses(rob)["gripper"]
        gp, gR = g[:3, 3] - o, g[:3, :3]
        for n in names:
            p, R = _pose(objects, n)
            rel_p = gR.T @ (p - gp)
            x, y, z, qw = Rotation.from_matrix(gR.T @ R).as_quat()
            jp = f"{_P['parent']}/{TT_BASE.name}/qf_joints/weld_{side}_{n}"
            j = st.GetPrimAtPath(jp)
            if not (j and j.IsValid()):
                jj = UsdPhysics.FixedJoint.Define(st, jp)
                jj.CreateJointEnabledAttr(False)
                jj.CreateLocalPos0Attr(Gf.Vec3f(*map(float, rel_p)))
                jj.CreateLocalRot0Attr(Gf.Quatf(float(qw), float(x), float(y), float(z)))
                jj.CreateBody0Rel().SetTargets([Sdf.Path(gpath)])
                jj.CreateBody1Rel().SetTargets([Sdf.Path(_P[n])])
                jj.CreateExcludeFromArticulationAttr(True)
                jj.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
                jj.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
                j = jj.GetPrim()
            j.GetAttribute("physics:localPos0").Set(Gf.Vec3f(*map(float, rel_p)))
            j.GetAttribute("physics:localRot0").Set(Gf.Quatf(float(qw), float(x), float(y), float(z)))
            j.GetAttribute("physics:jointEnabled").Set(True)
        _EP.welds[side] = names
        _EP.weld_span[side] = w_span
        _EP.stall_steps[side] = 0
        _EP.was_open[side] = False
        if os.environ.get("QF_DEBUG"):
            f = _pad_board_forces(side)
            print(f"[quadfold] weld {side}: {names} w={w_span*1e3:.1f}mm ap={ap*1e3:.2f}mm f={np.round(f,2).tolist()}", flush=True)


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_quadfold_hooked", False):
        orig = cls.reset

        def reset(self, *a, **k):
            reset_episode()
            return orig(self, *a, **k)

        reset._quadfold_hooked = True
        cls.reset = reset
    if not getattr(cls._task_step, "_quadfold_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            out = base(self, *a, **k)
            try:
                from yam_arena.scene import object_states

                st = object_states(self._env, [*P_NAMES, BOX.name, TT_DECK.name])
                step(st)
                _weld_update(st)
            except Exception as e:  # noqa: BLE001
                print(f"[quadfold] step hook failed: {e!r}", flush=True)
            return out

        _task_step._quadfold_hooked = True
        cls._task_step = _task_step


_install_hooks()


def step(objects) -> None:
    """Per control step: backward-bend damage latch, off-table latch, gripper history."""
    a = fold_angles(objects)
    for key, *_ in HINGES:
        x = a.get(key, float("nan"))
        if math.isfinite(x) and x < -BACK_BREAK:
            _EP.backward = True
    for n in (*P_NAMES, BOX.name, TT_DECK.name):
        p, _ = _pose(objects, n)
        if p is not None and p[2] < TOP - 0.02 and not _EP.off_table:
            _EP.off_table = n
    gp = gripper_points()
    if gp:
        _EP.grip_hist = (_EP.grip_hist + [np.array(gp, dtype=np.float64)])[-(PARK_STEPS + 1):]


# -- geometry readouts ----------------------------------------------------------------------------------------------
def _finite(a) -> bool:
    return a is not None and bool(np.isfinite(np.asarray(a, dtype=float)).all())


def _mat(q):
    x, y, z, w = (float(c) for c in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _local_bbox_offset(parts):
    c = []
    for _, cc, size, *_ in parts:
        h = np.asarray(size, float) / 2
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    c.append(np.asarray(cc, float) + h * np.array([sx, sy, sz]))
    c = np.array(c)
    return (c.min(axis=0) + c.max(axis=0)) / 2


# the reported position is the bounding-box centre of all of a body's geometry: offsets to the body origin
_OFFSET = {BOX.name: _local_bbox_offset([("floor", (0, 0, 0), BOX.size), *[(n, c, s) for n, c, s, *_ in _box_parts()]])}
for _k, _p in PANELS.items():
    _OFFSET[_p.name] = _local_bbox_offset([("plate", (0, 0, 0), _p.size), *_panel_parts(_k)])


def _pose(objects, name):
    """Body origin and rotation (world, env frame) of a scene body; (None, None) if not finite."""
    o = objects[name]
    if not (_finite(o.pos) and _finite(o.quat_xyzw)):
        return None, None
    R = _mat(o.quat_xyzw)
    return np.asarray(o.pos, float) - R @ _OFFSET.get(name, np.zeros(3)), R


def fold_angles(objects) -> dict:
    """Fold angle (deg) of each hinge: 0 flat, 180 closed face to face, negative = bent backwards (face-out)."""
    out = {}
    for key, b0, b1, axis, _, s in HINGES:
        _, r0 = _pose(objects, PANELS[b0].name)
        _, r1 = _pose(objects, PANELS[b1].name)
        if r0 is None or r1 is None:
            out[key] = float("nan")
            continue
        r = r0.T @ r1
        ang = math.degrees(math.atan2(r[0, 2], r[0, 0]) if axis == "Y" else math.atan2(r[2, 1], r[1, 1]))
        x = s * ang
        if x < -90.0:
            x += 360.0
        out[key] = x
    return out


CLOSED = 170.0  # within 10 deg of closed
BACK_BREAK = 20.0  # bent face-out past this = the hinge's face paper and wrap split


def all_closed(objects) -> bool:
    a = fold_angles(objects)
    return all(math.isfinite(a[k]) and a[k] >= CLOSED for k, *_ in HINGES)


_CORNERS = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)


def panel_corners(objects):
    pts = []
    for n in P_NAMES:
        p, R = _pose(objects, n)
        if p is None:
            return None
        pts += list(p + (_CORNERS * np.array([Q / 2, Q / 2, T / 2])) @ R.T)
    return np.array(pts)


def _in_box_frame(objects, pts):
    """Points in the box base's frame: origin on the insert's top face at the box centre."""
    pb, rb = _pose(objects, BOX.name)
    return (np.asarray(pts, float) - pb) @ rb + np.array([0.0, 0.0, B_FLOOR / 2 - INSERT_TOP])


FIT_TOL = 0.0015
FLAT_COS = math.cos(math.radians(10.0))


def in_box(objects) -> bool:
    """Every corner of every quarter inside the upright, table-resting box base's inner walls (1.5 mm slop), resting on
    the insert (bottom within 5 mm of insert top, no corner below -3 mm or above 2 cm); every quarter flat within
    10 deg."""
    pts = panel_corners(objects)
    pb, rb = _pose(objects, BOX.name)
    if pts is None or pb is None or rb[2, 2] < FLAT_COS or abs(pb[2] - (TOP + B_FLOOR / 2)) > 0.005:
        return False
    loc = _in_box_frame(objects, pts)
    if (np.abs(loc[:, :2]).max() > B_IN / 2 + FIT_TOL
            or loc[:, 2].min() < -0.003 or loc[:, 2].min() > 0.005 or loc[:, 2].max() > 0.020):
        return False
    for n in P_NAMES:
        _, R = _pose(objects, n)
        if abs(R[2, 2]) < FLAT_COS:
            return False
    return True


def gripper_points() -> list[np.ndarray]:
    if _ENV["env"] is None:
        return []
    from yam_arena.runtime import link_poses, pad_tips_world, tcp_world

    base = _ENV["env"].unwrapped
    o = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    pts = []
    for k in ("left_robot", "right_robot"):
        rob = base.scene[k]
        pts += [np.asarray(tcp_world(rob), dtype=np.float64) - o, np.asarray(pad_tips_world(rob), dtype=np.float64) - o]
        lp = link_poses(rob)
        for n in ("tip_left", "tip_right"):
            if n in lp:
                pts.append(np.asarray(lp[n][:3, 3], dtype=np.float64) - o)
    return pts


def _box_dist(p, c, R, half) -> float:
    q = R.T @ (np.asarray(p, float) - c)
    return float(np.linalg.norm(np.maximum(np.abs(q) - half, 0.0)))


def clearance(objects, p) -> float:
    """Distance from a point to the nearest quarter or to the box base's outline."""
    d = []
    for n in P_NAMES:
        c, R = _pose(objects, n)
        if c is None:
            return 0.0
        d.append(_box_dist(p, c, R, np.array([Q / 2, Q / 2, T / 2])))
    pb, rb = _pose(objects, BOX.name)
    if pb is None:
        return 0.0
    cb = pb + rb @ np.array([0.0, 0.0, B_H / 2 - B_FLOOR / 2])
    d.append(_box_dist(p, cb, rb, np.array([B_OUT / 2, B_OUT / 2, B_H / 2])))
    return min(d)


TOUCH = 0.01
PARK = 0.10
PARK_SPEED = 0.02


def held(objects) -> bool:
    """A gripper point within 1 cm of any quarter or the box base, or a quarter fixed to a gripper. Missing state
    counts as held."""
    if _EP.welds["left"] or _EP.welds["right"]:
        return True
    pts = gripper_points()
    if not pts:
        return True
    return any(clearance(objects, p) < TOUCH for p in pts if _finite(p)) or not all(_finite(p) for p in pts)


def parked(objects) -> bool:
    """Over the last 0.5 s every gripper point stayed 10 cm or more from the board and the box and moved under
    2 cm/s. Missing state = not parked."""
    hist = _EP.grip_hist
    if len(hist) < PARK_STEPS + 1 or _EP.welds["left"] or _EP.welds["right"]:
        return False
    for k, frame in enumerate(hist):
        if not _finite(frame):
            return False
        if any(clearance(objects, p) < PARK for p in frame):
            return False
        if k and float(np.linalg.norm(frame - hist[k - 1], axis=1).max()) * CONTROL_HZ >= PARK_SPEED:
            return False
    return True


def _zero(objects) -> str:
    """Reason the run scores 0, or ''."""
    for n in (*P_NAMES, BOX.name, TT_DECK.name):
        p, _ = _pose(objects, n)
        if p is None:
            return "state"
        if p[2] < TOP - 0.02:
            return f"{n} off the table"
    if _EP.off_table:
        return f"{_EP.off_table} fell off the table"
    if _EP.backward:
        return "a hinge bent face-out past 20 deg"
    a = fold_angles(objects)
    if any(math.isfinite(x) and x < -BACK_BREAK for x in a.values()):
        return "a hinge bent face-out past 20 deg"
    return ""


def score(objects) -> float:
    """0 if anything fell off the table or a hinge was bent face-out past 20 deg. Otherwise: 0.15 per far quarter
    closed (within 10 deg); 0.5 all three hinges closed; 0.75 the folded board also lies flat in the box base on the
    insert, not held; 1.0 also both arms parked (10 cm clear of board and box, under 2 cm/s for 0.5 s)."""
    if _zero(objects):
        return 0.0
    a = fold_angles(objects)
    if not all_closed(objects):
        return 0.15 * sum(1 for k in ("long_left", "long_right") if a[k] >= CLOSED)
    if not in_box(objects) or held(objects):
        return 0.5
    return 1.0 if parked(objects) else 0.75


def success(objects) -> bool:
    return bool(score(objects) == 1.0 and all(objects[n].resting(0.01, 0.5) for n in (*P_NAMES, BOX.name)))


def distance(objects) -> float:
    a = fold_angles(objects)
    if not all(map(math.isfinite, a.values())):
        return 1.0
    pts = panel_corners(objects)
    pb, _ = _pose(objects, BOX.name)
    d = float(np.linalg.norm(pts.mean(axis=0)[:2] - pb[:2])) if pts is not None and pb is not None else 1.0
    return float(sum(max(0.0, 180.0 - x) for x in a.values()) / 540.0 + d)


def progress(objects) -> dict:
    a = fold_angles(objects)
    return {**{k: round(v, 1) for k, v in a.items()}, "in_box": float(in_box(objects)),
            "welds": f"{_EP.welds['left']}|{_EP.welds['right']}", "zero": _zero(objects)}


VARIATION = {"obj:box_base": {"dx": 0.01, "dy": 0.01, "yaw": 0.0}}
