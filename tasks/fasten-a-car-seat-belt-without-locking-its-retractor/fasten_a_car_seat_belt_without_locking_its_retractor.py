"""Fasten a three-point seat belt around a fixed adult-sized surrogate in a complete front seat.

The robot lifts the tongue in front of the torso and above the thighs, then inserts its blade into the inboard
buckle. A taut ribbon path wraps against each supporting body surface. Both strands pull on the tongue along
their end tangents, and the total payout sets the retractor's tension and emergency lock. The tongue and buckle
pawl are physical rigid bodies. A released latch scores full credit after a stationary hold with a valid chest and
lap route and no reel lock. Dimensions and the real-world scoring procedure are in README.md.
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, Link, Linkage, allow_matrix_grips, as_numpy, on_reset, on_step, plate  # noqa: E402
from bodykit import rotation  # noqa: E402
from routekit import Ellipsoid, taut_route  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()


def _wide_observer_view() -> None:
    from yam_arena import ik, station
    from yam_arena.rig import Camera, Lens

    eye = np.array([-0.600, -0.300, 1.75])
    forward = np.array([0.27, -0.30, 1.20]) - eye
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    rotation_xyzw = ik.matrix_to_quat_xyzw(np.column_stack((right, up, -forward)))
    camera = Camera(tuple(eye), tuple(rotation_xyzw), "opengl", Lens(330, 330, 320, 180, 640, 360))
    station.top_camera = lambda variant="calibrated": camera


if os.environ.get("BELT_WIDE_CAMERA") == "1":
    _wide_observer_view()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
_E = os.environ.get
G = 9.81

# -- layout (x, y, height above the table) -------------------------------------------------------------------------
WY = -0.070  # the outboard flank is on the robot's left (+y) when facing the seat
OUT = 1.0  # the outboard direction (toward the pillar) along y
WX = 0.320
DRING = (0.425, WY, 0.62)
ANCHOR = (WX, WY, 0.045)
SLOT_Z = 0.150
SLOT_X = WX + (DRING[0] - WX) * (SLOT_Z - ANCHOR[2]) / (DRING[2] - ANCHOR[2])
SLOT0 = (SLOT_X, WY, SLOT_Z)
CUSH_TOP = 0.045
CUSH_Y = (-0.530, -0.080)
CUSH_X = (0.070, 0.560)
BACK_X = (0.560, 0.650)
BUCKLE = (0.311, -0.550)
MOUTH = 0.095  # the buckle mouth's height
ROOT_Z = 0.020  # the stalk's root (in the seat bight)

# webbing and retractor
WEB_W = 0.048
K_RET = float(_E("BELT_K", "2.2"))  # stiffness against total payout, N/m
F_PRE = float(_E("BELT_PRE", "2.0"))  # N at the start
K_LOCK = 1250.0  # locked spool plus webbing stretch, N/m
D_LOCK = 2.0  # N s/m
LOCK_G = float(_E("BELT_LOCK_G", "0.7"))  # webbing acceleration that locks the reel (g)
UNLOCK_BACK = 0.005  # m of webbing let back that frees it
TONGUE_KG = 0.040

# tongue (frame at its slot; blade down, faces toward +-y, width along x)
HEAD = (0.050, 0.013, 0.028)
BLADE_W, BLADE_T, BLADE_L = 0.022, 0.003, 0.024
WIN_W = 0.012
WIN_Z = (-0.044, -0.033)  # the window's lower and upper edges below the slot
TIP_Z = -0.050

# buckle
SLOT_X, SLOT_Y, SLOT_D = 0.026, 0.0045, 0.030
HEAD_LO = 0.025  # head bottom above the root
K_PAWL = float(_E("BELT_KPAWL", "2500"))
PAWL_IN = 0.00275  # how far the pawl sticks into the slot
PAWL_BOT = 0.015  # its flat underside below the mouth
STALK_K = 3.0  # N m/rad
FUN_W, FUN_D = 0.006, 0.006  # the mouth's funnel: 6 mm wider each side, 6 mm deep

WEB, DARK, GREY, STEEL, RED, SEAT, TRIM = ((0.11, 0.11, 0.12), (0.08, 0.08, 0.09), (0.35, 0.35, 0.37),
                                           (0.72, 0.73, 0.75), (0.80, 0.10, 0.10), (0.50, 0.50, 0.53),
                                           (0.55, 0.53, 0.50))
SHIRT, TROUSERS, FOAM = (0.26, 0.44, 0.61), (0.54, 0.46, 0.35), (0.79, 0.63, 0.47)


def _v(p) -> tuple:
    return tuple(float(x) for x in p)


def _w(p) -> tuple:
    return (float(p[0]), float(p[1]), float(TOP + p[2]))


CY = sum(CUSH_Y) / 2
CHEST = Ellipsoid("chest", (0.455, CY, 0.419), (0.108, 0.188, 0.234), SHIRT)
HIPS = Ellipsoid("pelvis", (0.431, CY, 0.123), (0.125, 0.177, 0.078), TROUSERS)
THIGHS = tuple(Ellipsoid(f"thigh_{side}", (0.262, CY + offset, 0.119), (0.190, 0.079, 0.074), TROUSERS)
               for side, offset in (("outboard", 0.084), ("inboard", -0.084)))
UPPER_ARMS = tuple(Ellipsoid(f"arm_{side}", (0.460, CY + offset, 0.497), (0.073, 0.051, 0.112), SHIRT)
                  for side, offset in (("outboard", 0.145), ("inboard", -0.145)))
TORSO = (CHEST, HIPS, *THIGHS, *UPPER_ARMS)
LAP_SURFACES = (HIPS, *THIGHS)
HEAD_SHAPES = (
    Ellipsoid("neck", (0.477, CY, 0.639), (0.060, 0.056, 0.081), FOAM),
    Ellipsoid("head", (0.485, CY, 0.748), (0.094, 0.075, 0.112), FOAM),
)


# -- the seat, pillar and sill (fixed) --------------------------------------------------------------------------------
def _seat() -> Body:
    cy = (CUSH_Y[0] + CUSH_Y[1]) / 2
    wy = CUSH_Y[1] - CUSH_Y[0]
    cx = (CUSH_X[0] + CUSH_X[1]) / 2
    dx = CUSH_X[1] - CUSH_X[0]
    sh = [BoxShape((cx, cy, CUSH_TOP / 2), (dx, wy, CUSH_TOP), SEAT)]
    for y in (CUSH_Y[0] + 0.025, CUSH_Y[1] - 0.025):  # side bolsters
        sh.append(BoxShape((cx, y, CUSH_TOP + 0.01), (dx, 0.05, 0.02), SEAT))
    bx = (BACK_X[0] + BACK_X[1]) / 2
    sh.append(BoxShape((bx, cy, CUSH_TOP + 0.2875), (BACK_X[1] - BACK_X[0], wy, 0.575), SEAT))
    for y in (CUSH_Y[0] + 0.03, CUSH_Y[1] - 0.03):
        sh.append(BoxShape((bx - 0.005, y, CUSH_TOP + 0.30), (0.10, 0.06, 0.52), SEAT))
    for y in (cy - 0.08, cy + 0.08):
        sh.append(CylShape((bx, y, 0.66), 0.006, 0.08, STEEL))
    sh.append(BoxShape((bx, cy, 0.76), (0.08, 0.26, 0.15), SEAT))
    return Body.build("seat", pos=(0.0, 0.0, TOP), mass=20.0, color=SEAT, fixed=True, friction=(0.8, 0.7), shapes=sh)


def _pillar() -> Body:
    o = OUT
    sx = DRING[0]
    sh = [BoxShape((sx + 0.07, WY + o * 0.09, 0.34), (0.14, 0.06, 0.68), TRIM),
          BoxShape((sx + 0.07, WY + o * 0.09, 0.06), (0.12, 0.065, 0.12), DARK),
          BoxShape((0.38, WY + o * 0.035, 0.02), (0.30, 0.05, 0.04), TRIM),
          BoxShape((WX, WY + o * 0.015, 0.0425), (0.03, 0.022, 0.005), STEEL),  # lap anchor plate and bolt
          CylShape((WX, WY + o * 0.008, 0.0425), 0.006, 0.006, STEEL, axis="Y"),
          BoxShape((sx + 0.025, WY + o * 0.03, DRING[2] + 0.015), (0.10, 0.06, 0.02), TRIM),
          BoxShape((sx, WY + o * 0.003, DRING[2] + 0.01), (0.06, 0.008, 0.026), DARK, collide=False),
          BoxShape((sx + 0.07, WY + o * 0.058, 0.23), (0.07, 0.004, 0.33), DARK, collide=False)]
    return Body.build("pillar", pos=(0.0, 0.0, TOP), mass=5.0, color=TRIM, fixed=True, friction=(0.6, 0.5), shapes=sh)


# -- the belt ----------------------------------------------------------------------------------------------------------
def _tongue_shapes() -> list:
    hz = -HEAD[2] / 2 + 0.002
    sh = [BoxShape((0.0, 0.0, hz), HEAD, DARK),
          BoxShape((0.0, 0.0, 0.0), (0.036, 0.0135, 0.004), GREY, collide=False)]  # the slot's lips
    side = (BLADE_W - WIN_W) / 2
    top = -HEAD[2] + 0.002  # the head's bottom
    zc = (top + WIN_Z[1]) / 2
    sh.append(BoxShape((0.0, 0.0, zc), (BLADE_W, BLADE_T, top - WIN_Z[1] + 0.001), STEEL))  # above the window
    for s in (1, -1):
        sh.append(BoxShape((s * (WIN_W + side) / 2, 0.0, (WIN_Z[0] + WIN_Z[1]) / 2),
                           (side, BLADE_T, WIN_Z[1] - WIN_Z[0]), STEEL))
    sh.append(BoxShape((0.0, 0.0, (TIP_Z + WIN_Z[0]) / 2), (BLADE_W, BLADE_T, WIN_Z[0] - TIP_Z), STEEL))  # tip bar
    sh.append(BoxShape((0.010, -0.0016, TIP_Z + 0.010), (0.002, 0.0002, 0.001), DARK, collide=False))
    sh.append(BoxShape((0.010, 0.0016, TIP_Z + 0.010), (0.002, 0.0002, 0.001), DARK, collide=False))
    return sh


def _routes(tongue_position):
    slot = np.asarray(tongue_position) - [0.0, 0.0, TOP]
    return (taut_route(np.asarray(DRING), slot, TORSO, axis=0, sign=-1, clearance=0.009),
            taut_route(np.asarray(ANCHOR), slot, LAP_SURFACES, axis=2, sign=1))


REST_LENGTH = float(np.linalg.norm(np.subtract(DRING, ANCHOR)))


# -- the buckle --------------------------------------------------------------------------------------------------------
def _buckle() -> Linkage:
    r = np.array([BUCKLE[0], BUCKLE[1], TOP + ROOT_Z])
    zm = MOUTH - ROOT_Z  # the mouth above the root
    hx0, hx1 = -0.032, 0.021  # head extent along x (the release button at -x)
    hy = 0.016
    sx, sy = SLOT_X / 2, SLOT_Y / 2
    base = [BoxShape((0.0, 0.0, -0.005), (0.04, 0.03, 0.01), DARK)]
    stalk = [BoxShape((0.0, 0.0, HEAD_LO / 2), (0.030, 0.012, HEAD_LO), DARK)]
    head = []
    z_lo = HEAD_LO
    r2 = math.sqrt(0.5)
    # the long walls either side of the slot: their inner FUN_W stop FUN_D under the mouth, where 45 deg funnel faces
    # lead into the 4.5 mm slot; the end walls likewise with a 3 mm funnel
    for s in (1, -1):
        head.append(BoxShape((0.0, s * (sy + FUN_W / 2), (z_lo + zm - FUN_D) / 2), (SLOT_X, FUN_W, zm - FUN_D - z_lo),
                             DARK))
        head.append(BoxShape(((hx0 + hx1) / 2, s * (sy + FUN_W + hy) / 2, (z_lo + zm) / 2),
                             (hx1 - hx0, hy - sy - FUN_W, zm - z_lo), DARK))
        head.append(plate((0.0, s * (sy + 0.0007), zm - FUN_D - 0.0007), (0.0, s * (sy + FUN_W + 0.0007), zm - 0.0007),
                          SLOT_X, 0.002, (0.0, -s * r2, r2), GREY))
    for s, ex in ((1, hx1), (-1, hx0)):
        span = abs(ex) - sx
        head.append(BoxShape((s * (sx + 0.003 + abs(ex)) / 2, 0.0, (z_lo + zm) / 2),
                             (span - 0.003, 2 * (sy + FUN_W), zm - z_lo), DARK))
        head.append(BoxShape((s * (sx + 0.0015), 0.0, (z_lo + zm - 0.003) / 2), (0.003, 2 * (sy + FUN_W), zm - 0.003 - z_lo),
                             DARK))
        head.append(plate((s * (sx + 0.0007), 0.0, zm - 0.0037), (s * (sx + 0.0037), 0.0, zm - 0.0007),
                          2 * (sy + FUN_W), 0.002, (-s * r2, 0.0, r2), GREY))
    head.append(BoxShape((0.0, 0.0, (z_lo + zm - SLOT_D) / 2), (SLOT_X, SLOT_Y, zm - SLOT_D - z_lo), DARK))
    head.append(BoxShape(((hx0 - sx) / 2 - 0.001, 0.0, zm + 0.002), (-sx - hx0 - 0.006, 0.020, 0.004), RED,
                         collide=False))  # release button
    head.append(BoxShape((0.0, 0.0, zm + 0.0002), (SLOT_X + 0.008, 0.0012, 0.0004), (0.02, 0.02, 0.02),
                         collide=False))  # the dark mouth seen from above
    pz = zm - PAWL_BOT
    pawl = [BoxShape((0.0, sy - PAWL_IN + 0.003, pz + 0.0015), (0.008, 0.006, 0.003), STEEL),
            BoxShape((0.0, sy - PAWL_IN + 0.00275, pz + 0.003), (0.008, 0.0039, 0.0039), STEEL,
                     quat_xyzw=(math.sin(math.pi / 8), 0.0, 0.0, math.cos(math.pi / 8)))]
    o = _v(r)
    links = [Link("st_x", tuple(stalk), mass=0.01, anchor=o, axis="X", stiffness=STALK_K, spring_damping=0.03,
                  limits=(-20.0, 20.0)),
             Link("st_y", tuple(head), mass=0.06, anchor=o, axis="Y", parent="st_x", stiffness=STALK_K,
                  spring_damping=0.03, limits=(-20.0, 20.0)),
             Link("pawl", tuple(pawl), mass=0.003, anchor=_v(r + [0.0, sy, pz]), axis="Y", parent="st_y", slide=True,
                  limits=(0.0, 0.004), stiffness=K_PAWL, spring_damping=2.0)]
    return Linkage.build("buckle", pos=o, mass=0.1, color=DARK, floating=False, friction=(0.3, 0.25),
                         shapes=base, links=links, contact_offset=0.001, solver_iterations=48)


SEAT_BODY = _seat()
PILLAR = _pillar()
OCCUPANT = Body.build("occupant", pos=(0.0, 0.0, TOP), mass=54.0, color=SHIRT, fixed=True,
                      friction=(0.65, 0.55), shapes=[shape.mesh() for shape in (*TORSO, *HEAD_SHAPES)])
TONGUE = Body.build("tongue", pos=_w(SLOT0), mass=TONGUE_KG, color=DARK,
                    friction=(0.55, 0.45), shapes=_tongue_shapes(), damping=(0.2, 0.4), solver_iterations=48)
BUCKLE_LK = _buckle()
OBJECTS = [SEAT_BODY, PILLAR, OCCUPANT, TONGUE, BUCKLE_LK]

# -- tracking ----------------------------------------------------------------------------------------------------------
_DEBUG = _E("BELT_DEBUG") == "1"
_DEBUG_EVERY = int(_E("BELT_DEBUG_EVERY", "5"))
CLEAR = 0.020
DT = 0.05
STILL_LIMIT = 0.001
STILL_SAMPLES = 21
_EP: dict = {}
_IDS: dict = {}
# points on the tongue (its frame): head corners and blade
_TS = np.array([[x, y, z] for x in (-0.025, 0.0, 0.025) for y in (-0.0065, 0.0065) for z in (0.002, -0.012, -0.026)]
               + [[x, 0.0, z] for x in (-0.011, 0.0, 0.011) for z in (-0.03, -0.04, -0.05)])


@on_reset
def _reset() -> None:
    _EP.clear()
    _EP.update(tick=0, step=0, payout=0.0, rate=0.0, rate_prev=0.0, acc=0.0, peak_acc=0.0,
               locked=False, lock_at=0.0, locks=0, entered=False, latched=False, depth=None, pawl=0.0,
               near=1.0, still=False, positions=[], tcp={}, gaps={}, slot=np.array(_w(SLOT0)), routes=None, routed=False)


_reset()


def _finger_points() -> dict:
    import struct
    import xml.etree.ElementTree as ET

    from yam_arena import YAM_URDF
    from yam_arena.urdf import rpy_matrix

    root = pathlib.Path(YAM_URDF).parent
    out = {}
    for link in ET.parse(YAM_URDF).getroot().iter("link"):
        name = link.get("name")
        if name not in ("tip_left", "tip_right"):
            continue
        pts = []
        for col in link.findall("collision"):
            o = col.find("origin")
            xyz = np.array([float(v) for v in (o.get("xyz") or "0 0 0").split()]) if o is not None else np.zeros(3)
            rpy = np.array([float(v) for v in (o.get("rpy") or "0 0 0").split()]) if o is not None else np.zeros(3)
            b = (root / col.find("geometry").find("mesh").get("filename")).read_bytes()
            n = struct.unpack("<I", b[80:84])[0]
            tri = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
            v = tri["v"].reshape(-1, 3).astype(float)
            pts.append(np.unique(np.round(v, 4), axis=0) @ rpy_matrix(rpy).T + xyz)
        out[name] = np.concatenate(pts)
    return out


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("BELT_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _sample_belt(base, origin) -> None:
    import torch

    asset = base.scene["tongue"]
    pose = as_numpy(asset.data.root_link_pose_w)[0]
    velocity = as_numpy(asset.data.root_link_vel_w)[0]
    slot = pose[:3] - origin
    paths = _routes(slot)
    payout = sum(path.length for path in paths) - REST_LENGTH
    dt = base.physics_dt
    raw_rate = (payout - _EP["payout"]) / dt
    rate = _EP["rate"] + dt / 0.025 * (raw_rate - _EP["rate"])
    _EP.update(routes=paths, payout=payout, rate=rate)

    tension = F_PRE + K_RET * max(0.0, payout)
    if _EP["locked"]:
        stretch = max(0.0, payout - _EP["lock_at"])
        tension += min(35.0, K_LOCK * stretch + D_LOCK * max(0.0, rate))
    force = tension * (paths[0].pull_direction + paths[1].pull_direction) - 0.45 * velocity[:3]

    # A sewn stop on the parked webbing supports the free-sliding tongue from below.
    stop_direction = np.asarray(DRING) - ANCHOR
    stop_direction /= np.linalg.norm(stop_direction)
    from_stop = slot - _w(SLOT0)
    along = from_stop @ stop_direction
    if along < 0.0 and np.linalg.norm(from_stop - along * stop_direction) < 0.012:
        force += min(-300.0 * along, 8.0) * stop_direction

    front = rotation(pose[3:7])[:, 0]
    yaw = math.atan2(front[1], front[0])
    torque = (0.0, 0.0, -0.006 * math.sin(yaw))
    wrench = torch.as_tensor(np.stack((force, torque, pose[:3])), dtype=torch.float32, device=asset.device)
    asset.permanent_wrench_composer.set_forces_and_torques_index(
        forces=wrench[0].reshape(1, 1, 3), torques=wrench[1].reshape(1, 1, 3),
        positions=wrench[2].reshape(1, 1, 3), is_global=True)


def _render_belt(paths, slot_normal, origin) -> None:
    from isaaclab.sim import PreviewSurfaceCfg
    from isaaclab.sim.utils import bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdGeom, Vt

    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    root = "/World/RoboCurveSeatBelt"
    for name, route in zip(("shoulder", "lap"), paths):
        path = f"{root}/{name}"
        mesh = UsdGeom.Mesh.Get(stage, path)
        if not mesh:
            mesh = UsdGeom.Mesh.Define(stage, path)
            steps = len(route.points) - 1
            mesh.CreateFaceVertexCountsAttr([4] * steps)
            mesh.CreateFaceVertexIndicesAttr([i + j for i in range(0, steps * 2, 2) for j in (0, 1, 3, 2)])
            mesh.CreateDoubleSidedAttr(True)
            mesh.CreateSubdivisionSchemeAttr("none")
            material_path = f"{root}/Webbing"
            if not stage.GetPrimAtPath(material_path):
                preview = PreviewSurfaceCfg(diffuse_color=linear_rgb(WEB), roughness=0.82)
                preview.func(material_path, preview)
            bind_visual_material(path, material_path, stage=stage)
        points = route.ribbon(WEB_W, slot_normal) + origin + [0, 0, TOP]
        mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)))
        mesh.GetExtentAttr().Set([Gf.Vec3f(*points.min(axis=0)), Gf.Vec3f(*points.max(axis=0))])


def _is_routed(paths, slot_normal) -> bool:
    shoulder, lap = paths
    if shoulder.near_length(CHEST, 0.012) < 0.050:
        return False
    if any(lap.near_length(thigh, 0.012) < 0.025 for thigh in THIGHS):
        return False
    for route in paths:
        edges = route.ribbon(WEB_W, slot_normal).reshape(-1, 2, 3)
        midpoints = (edges[1:] + edges[:-1]) / 2
        vertices = np.concatenate((route.points, edges.reshape(-1, 3), midpoints.reshape(-1, 3)))
        for surface in (*TORSO, *HEAD_SHAPES):
            scaled = (vertices - surface.centre) / np.subtract(surface.radii, 0.001)
            if np.square(scaled).sum(axis=1).min() < 1.0:
                return False
    return True


def _track_inner(env) -> None:
    from scipy.spatial import cKDTree

    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    _EP["tick"] += 1
    base = getattr(env, "unwrapped", env)
    origin = as_numpy(base.scene.env_origins[0])
    if getattr(base.scene, "_robocurve_belt_hook", None) is not _sample_belt:
        update_scene = base.scene.update

        def update(*args, **kwargs):
            result = update_scene(*args, **kwargs)
            _sample_belt(base, origin)
            return result

        base.scene.update = update
        base.scene._robocurve_belt_hook = _sample_belt
    if _EP["routes"] is None:
        _sample_belt(base, origin)
    if "fingers" not in _IDS:
        _IDS["fingers"] = _finger_points()
    fingers = {}
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        if side not in _IDS:
            _IDS[side] = list(robot.find_joints(ARM_JOINTS, preserve_order=True)[0])
            _IDS[side + "_f"] = list(robot.find_joints(FINGER_JOINTS, preserve_order=True)[0])
            _IDS[side + "_tips"] = [robot.body_names.index(n) for n in ("tip_left", "tip_right")]
        q = as_numpy(robot.data.joint_pos)[0]
        _EP["tcp"][side] = np.asarray(ik.tcp_pose(q[_IDS[side]], station.BASES[side])[0], float)
        _EP["gaps"][side] = float(-q[_IDS[side + "_f"]].sum())
        poses = as_numpy(robot.data.body_link_pose_w)[0]
        fingers[side] = np.concatenate([_IDS["fingers"][n] @ rotation(poses[i][3:7]).T + poses[i][:3] - origin
                                        for n, i in zip(("tip_left", "tip_right"), _IDS[side + "_tips"])])
    tongue = base.scene["tongue"]
    buck = base.scene["buckle"]
    if "head" not in _IDS:
        _IDS["head"] = list(buck.body_names).index("st_y")
        _IDS["pawl"] = list(buck.joint_names).index("pawl_turn")
    payout, rate, previous_rate = _EP["payout"], _EP["rate"], _EP["rate_prev"]
    acc = (rate - previous_rate) / DT if _EP["tick"] > 1 else 0.0
    _EP.update(acc=acc, rate_prev=rate)
    if _EP["tick"] > 5:
        _EP["peak_acc"] = max(_EP["peak_acc"], acc)
    # a lock: the webbing accelerating outward faster than LOCK_G (from rest or while paying out; not the rebound of
    # a belt that has just whipped back onto its stop)
    if (not _EP["locked"] and _EP["tick"] > 5 and rate > 0.02 and previous_rate > -0.05
            and acc > LOCK_G * G):
        _EP.update(locked=True, lock_at=payout, locks=_EP["locks"] + 1)
    elif _EP["locked"] and payout <= _EP["lock_at"] - UNLOCK_BACK:
        _EP["locked"] = False
    tp = as_numpy(tongue.data.root_link_pose_w)[0]
    t_pos, t_rot = tp[:3] - origin, rotation(tp[3:7])
    hp = as_numpy(buck.data.body_link_pose_w)[0][_IDS["head"]]
    h_pos, h_rot = hp[:3] - origin, rotation(hp[3:7])
    # the tongue's slot and blade tip in the buckle head's frame (origin: the stalk root, mouth at zm)
    zm = MOUTH - ROOT_Z
    slot_h = h_rot.T @ (t_pos - h_pos)
    tip_h = h_rot.T @ (t_pos + t_rot @ np.array([0.0, 0.0, TIP_Z]) - h_pos)
    pawl = float(as_numpy(buck.data.joint_pos)[0][_IDS["pawl"]])
    in_line = abs(tip_h[0]) < 0.006 and abs(tip_h[1]) < 0.004
    depth = zm - tip_h[2]  # the blade tip's depth inside the mouth
    if in_line and depth >= 0.010:
        _EP["entered"] = True
    # latched: the head's bottom within 3.5 mm of the mouth (the window's lower edge below the pawl) and the pawl back
    head_gap = slot_h[2] - HEAD[2] + 0.002 - zm
    latched = bool(in_line and head_gap <= 0.0035 and pawl <= 0.0008)
    pts = t_pos + _TS @ t_rot.T
    tree = cKDTree(pts)
    near = min(float(tree.query(fp, k=1)[0].min()) for fp in fingers.values())
    tv = float(np.linalg.norm(as_numpy(tongue.data.root_link_vel_w)[0][:3]))
    observed = _EP["positions"]
    observed.append(t_pos.copy())
    del observed[:-STILL_SAMPLES]
    drift = float(np.linalg.norm(np.asarray(observed) - t_pos, axis=1).max())
    _EP["tip_h"], _EP["head"] = tip_h, h_pos
    paths = _EP["routes"]
    _render_belt(paths, t_rot[:, 1], origin)
    _EP.update(latched=latched, depth=depth, head_gap=head_gap, pawl=pawl, near=near, drift=drift,
               still=len(observed) == STILL_SAMPLES and drift <= STILL_LIMIT,
               routed=_is_routed(paths, t_rot[:, 1]) if latched else False,
               slot=t_pos, tv=tv, tilt=math.degrees(math.acos(max(-1.0, min(1.0, float(t_rot[2, 2]))))))


def _tally(o) -> dict:
    done = bool(_EP["latched"] and _EP["routed"] and _EP["near"] >= CLEAR and _EP["still"])
    return dict(done=done)


def _debug(t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    s = _EP["slot"]
    print(f"DBG step={_EP['step']} payout={_EP['payout'] * 1000:.0f}mm rate={_EP['rate']:.3f} "
          f"acc={_EP['acc'] / G:+.2f}g peak={_EP['peak_acc'] / G:.2f}g locked={int(_EP['locked'])} locks={_EP['locks']} "
          f"slot=({s[0]:.3f},{s[1]:.3f},{(s[2] - TOP) * 1000:.1f}mm) tilt={_EP.get('tilt', 0):.0f} "
          f"depth={(_EP['depth'] or 0) * 1000:.1f}mm headgap={_EP.get('head_gap', 0) * 1000:.1f}mm "
          f"pawl={_EP['pawl'] * 1000:.2f}mm entered={int(_EP['entered'])} latched={int(_EP['latched'])} routed={int(_EP['routed'])} "
          f"tip_h={np.round(np.array(_EP.get('tip_h', np.zeros(3))) * 1000, 1).tolist()} head={np.round(_EP.get('head', np.zeros(3)), 3).tolist()} "
          f"near={_EP['near'] * 1000:.0f}mm drift={_EP.get('drift', 0) * 1000:.2f}mm "
          f"tv={_EP.get('tv', 0):.3f} done={int(t['done'])} | "
          + " ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{(v[2] - TOP) * 1000:.1f}mm) gap={_EP['gaps'][k] * 1000:.0f}"
                     for k, v in _EP["tcp"].items()), flush=True)


def score(objects) -> float:
    t = _tally(objects)
    if _DEBUG:
        _debug(t)
    best = 0.25 if _EP["entered"] else 0.0
    if _EP["latched"]:
        best = 0.4
        if t["done"]:
            best = 0.6 if _EP["locks"] else 1.0
    return best


def success(objects) -> bool:
    return bool(_tally(objects)["done"] and not _EP["locks"])


# -- scripted demo -----------------------------------------------------------------------------------------------------
# The prescribed motion lifts the tongue clear of the occupant's thighs, crosses the lap from the front, rotates the
# blade into the narrow inboard buckle and releases it. All gripper targets are fixed waypoints. In jerk mode the
# wide lateral draw begins abruptly from a pose clear of the surrogate.
OPEN_C = -0.047
JERK = _E("BELT_JERK") == "1"
R_Y = grip_rotation(0.0)  # straight down, the fingers closing along y
P = {k: float(_E("BELT_" + k.upper(), v)) for k, v in dict(
    open="-0.018", pinch="-0.0015", grip_dz="-0.010", draw_s="5.0", push="0.042", lift="0.050",
    aim_dx="-0.004", aim_dy="-0.030", over_dz="0.014", roll="-4.0", ready_pitch="18",
    travel_pitch="45", seat_pitch="20").items()}


def _good_ik(arm: str, pos, rot):
    from yam_arena import ik, station

    best = None
    b = station.BASES[arm]
    j1 = math.atan2(float(pos[1]) - b[1], float(pos[0]) - b[0])
    for sd in itertools.product((j1,), (0.8, 1.3), (0.6, 1.2), (-0.8, 0.8), (0.0,), (-1.8, -0.9, 0.0, 0.9, 1.8)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.asarray(sd, float),
                             base_pos=station.BASES[arm], iters=150)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
                if room > 0.3:
                    break
    if best is None:
        raise ValueError(f"no IK solution for the {arm} arm at {pos}")
    return best[0]


def _joint_path(arm: str, q0, q1, finger: float, label: str, seconds: float, steps: int = 15) -> list:
    from yam_arena import ik, station

    out = []
    for k in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, station.BASES[arm])
        out.append(Waypoint(seconds / steps, tuple(float(v) for v in p), finger, label, r))
    return out


def _lead_in(arm: str, pos, rot, finger: float, label: str, seconds: float = 3.0) -> list:
    from yam_arena.constants import READY

    return _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, rot), finger, label, seconds)


def _lead_out(arm: str, pos, rot, finger: float, label: str, seconds: float = 3.0) -> list:
    from yam_arena.constants import READY

    return _joint_path(arm, _good_ik(arm, pos, rot), np.array(READY, float), finger, label, seconds)


def _wp(t, p, finger, label, rot=R_Y) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in p), finger, label, rot)


def _ready(arm: str):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), station.BASES[arm])


def _dur(wps) -> float:
    return float(sum(w.duration for w in wps))


def _grip(roll: float = 0.0, pitch: float = 0.0) -> np.ndarray:
    from yam_arena.urdf import axis_angle_matrix

    rx = axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(roll))
    ry = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), -math.radians(pitch))
    return rx @ ry @ R_Y


def _minjerk(p0, p1, seconds, finger, label, n=20, roll=(0.0, 0.0), pitch=(0.0, 0.0)) -> list:
    out = []
    for k in range(1, n + 1):
        s = k / n
        f = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
        rot = _grip(roll[0] + (roll[1] - roll[0]) * f, pitch[0] + (pitch[1] - pitch[0]) * f)
        out.append(_wp(seconds / n, p0 + (p1 - p0) * f, finger, label, rot))
    return out


def _right() -> list:
    g = np.array(_w(SLOT0)) + [0.0, 0.0, P["grip_dz"]]  # TCP holding the tongue's head
    above = g + [0.0, 0.0, 0.02]
    # The tongue travels forward and up before crossing both thighs. It only descends outside the inboard thigh.
    lift = np.asarray(_w((0.282, -0.092, 0.227)))
    mid = np.asarray(_w((0.247, -0.311, 0.252)))
    side = np.asarray(_w((0.279, -0.532, 0.223)))
    bx, by = BUCKLE[0] + P["aim_dx"], BUCKLE[1] + P["aim_dy"]
    over = np.array([bx, by, TOP + MOUTH - TIP_Z + P["over_dz"] + P["grip_dz"]])  # blade tip just over the mouth
    seat = np.array([bx, by, TOP + MOUTH + HEAD[2] - 0.002 + P["grip_dz"] - P["push"]])
    start_pitch, travel_pitch = P["ready_pitch"], P["travel_pitch"]
    start_rot = _grip(pitch=start_pitch)
    out = _lead_in("right", above, start_rot, P["open"], "over the hanging tongue, fingers open")
    out += [_wp(1.5, g, P["open"], "fingers either side of the tongue's head", start_rot),
            _wp(1.0, g, P["pinch"], "pinch the head", start_rot),
            _wp(0.5, g, P["pinch"], "settle", start_rot)]
    out += _minjerk(g, lift, 2.1, P["pinch"], "raise forward of the occupant", pitch=(start_pitch, travel_pitch))
    start = lift
    if JERK:
        start = lift + 0.61 * (mid - lift)
        out += [_wp(0.1, start, P["pinch"], "snatch the belt out", _grip(pitch=travel_pitch))]
    out += _minjerk(start, mid, P["draw_s"] / 2, P["pinch"], "clear the upper thighs", pitch=(travel_pitch, travel_pitch))
    out += _minjerk(mid, side, P["draw_s"] / 2, P["pinch"], "draw to the inboard side", pitch=(travel_pitch, travel_pitch))
    rl = P["roll"]  # the hand turns so the tongue's blade, pulled askew by the webbing, points straight down
    seated_pitch = P["seat_pitch"]
    seated_rot = _grip(rl, seated_pitch)
    out += _minjerk(side, over, 2.6, P["pinch"], "align above the inboard buckle",
                    roll=(0.0, rl), pitch=(travel_pitch, seated_pitch))
    out += [_wp(0.5, over, P["pinch"], "settle over the mouth", seated_rot)]
    out += _minjerk(over, seat, 2.0, P["pinch"], "push the blade into the buckle until it latches", 10,
                    roll=(rl, rl), pitch=(seated_pitch, seated_pitch))
    out += [_wp(1.0, seat, P["pinch"], "hold", seated_rot),
            _wp(1.0, seat, P["open"], "let go", seated_rot),
            _wp(1.2, seat + [0.0, 0.04, P["lift"]], P["open"], "rise clear")]
    out += _lead_out("right", seat + [0.0, 0.04, P["lift"]], R_Y, OPEN_C, "back to the ready pose")
    return out


def demo():
    right = _right()
    rp, rq = _ready("right")
    right += [_wp(3.0, rp, OPEN_C, "stay clear", rq)]
    lp, lq = _ready("left")
    left = [_wp(_dur(right), lp, OPEN_C, "stay at the ready pose", lq)]
    return {"left": left, "right": right}
