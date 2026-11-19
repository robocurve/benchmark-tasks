"""On the station table: a round contact grill (a George Foreman 360-style grill with removable plates, true size: 30 cm
plates, 34 cm body) stands on the left with its lid open, standing upright at the back. Both cooking plates are latched
in: the lower plate lies in the body, the upper plate stands in the open lid, its cooking face toward the robot. Each
plate has a black release handle on its left and on its right side (39.4 cm apart tip to tip). A drip tray sits in a
slot in the front of the body, under the body's front edge. A blue silicone dish-drying mat lies on the right. Task
(instruction): take both plates off the grill and put them on the drying mat, and take the drip tray out from under
the grill. The instruction does not say how.

How it is done (George Foreman GRP106QPG Use & Care: "Holding a plate with both hands, use your thumbs to pull out the
release handles (C)" and "make sure the latches on each side fit into the openings on each side of the grill";
GRP3CAN Use & Care: "The grill plates may be removed from the appliance by pulling out on the grill plate release
handles"; "Allow the drip tray to cool before removing it from under the grill").

The hand skill: each handle slides outward on a spring (400 N/m, 10 mm travel) and carries a latch tongue that hooks
under a lip of the grill frame below the plate's edge; the tongue clears the lip only once the handle is pulled out
6 mm or more, and it snaps back when the handle is let go. So a plate comes free only while BOTH its handles, 39 cm
apart on opposite sides, are held pulled out at the same time: one gripper on one handle only tilts the plate about
9 degrees about the other latch. The two grippers pinch the two handles, pull them apart, and lift the plate off by
them (the upper plate forward out of the lid, then turned flat), carry it to the mat and set it down flat; the plates
end stacked on the mat. The drip tray is tucked 3.5 cm under the body's front edge, so it cannot be lifted out: it is
slid out toward the robot by its front lip.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- grill body (fixed): stainless, a 34 cm round body 2.4 cm thick on a 25 cm pedestal, centre (0.300, 0.160), its top
  6.4 cm above the table and its underside 4.0 cm up; the lower plate's seat is a ring wall around the plate with a
  notch at each handle and, in each notch, the latch lip (4 x 1.5 x 0.5 cm) 0.2-0.7 cm below the plate's back edge.
- lid (fixed, open upright): a 34 cm round housing at the back, its front face at x 0.475, centre 27.2 cm up; the upper
  plate's seat (ring wall, notches, lips) is the same, turned to face the robot.
- plates (loose, 0.60 kg each, articulated): a 30 cm round cast plate 1.2 cm thick with nine 4 mm ridges on the cooking
  face and a 1.4 cm deep ring on its back (it rests on that ring), and a release handle (4.5 x 3.9 x 2.0 cm, 0.02 kg)
  on each side on a slider along y (tips 39.4 cm apart). Lower plate centre (0.300, 0.160, 8.4 cm up); upper plate
  centre (0.455, 0.160, 27.2 cm up), its face toward -x.
- drip tray (loose, 0.12 kg): 20 x 7 x 2.5 cm with a 1.0 cm thick front lip, x 0.095-0.165 at y 0.160, its back
  3.5 cm under the body's front edge (the body's underside is 4.0 cm up, the tray's lip 3.0 cm tall).
- drying mat (fixed): blue silicone, 38 x 33 x 0.4 cm, centre (0.260, -0.240).

Score (no run-wide zero; a dropped plate only loses its own credit):
- each plate, 0.15: taken off by its handles: its centre 3 cm or more from its seat while each of its two handles is
  touched by a gripper with 1 N or more (PhysX contact force), and never dropped (its centre falling faster than
  0.77 m/s, a free fall of 3 cm). +0.20 (0.35 in all): then lying flat (within 10 deg) on the drying mat, or on the
  other plate lying there, its centre over the mat, its lowest point within 6 mm of what it lies on, and released (no
  arm touching it or its handles, 0.1 N).
- drip tray, 0.20: slid out (its back corners and back middle 17.5 cm or more from the body's centre, i.e. clear of
  the body), resting on the table (lowest point within 4 mm, tilt under 10 deg) and released.
- 1.0 (success, held 1 s): all of that, no arm touching any plate, handle or the tray, and everything still (under
  1 mm in 1 s).
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, Link, Linkage, MeshShape, _quat_from_matrix, allow_matrix_grips  # noqa: E402
from bodykit import as_numpy, on_reset, on_step, rotation  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- the layout ----------------------------------------------------------------------------------------------------
GX, GY = 0.300, 0.160  # the grill body's centre
BODY_R, PED_R = 0.170, 0.125
BODY_Z0, BODY_Z1 = 0.040, 0.064  # the body's underside and top, above the table
PLATE_R, PLATE_T = 0.150, 0.012
RING_R, RING_W = 0.125, 0.010  # the ring on the plate's back
SEAT = 0.020  # the plate's centre is this far above (in front of) its seat's floor
RIM_IN, RIM_OUT = 0.152, 0.170
NOTCH = math.radians(20.0)  # half-width (angle) of the rim's notch at each handle
LIP_Y0, LIP_Y1, LIP_Z0, LIP_Z1 = 0.135, 0.150, -0.013, -0.008  # the latch lip (plate-local, +y side)
H_Y0, H_Y1, H_Z0, H_Z1, H_W = 0.158, 0.197, -0.006, 0.014, 0.045  # the handle
TONGUE_Y0, TONGUE_Z0, TONGUE_Z1 = 0.144, -0.019, -0.0145  # the latch tongue, under the lip
TRAVEL, SPRING = 0.010, 400.0  # handle travel (m) and its spring (N/m)
PLATE_M, HANDLE_M = 0.60, 0.02
LID_X = GX + BODY_R + 0.005  # the lid's front face
UPPER_Z = 0.272  # the lid's centre height: its lower edge clears the lower plate
LOWER_C = np.array([GX, GY, TOP + BODY_Z1 + SEAT])
UPPER_C = np.array([LID_X - SEAT, GY, TOP + UPPER_Z])
TRAY_C = np.array([0.130, GY])
TRAY_L, TRAY_W, TRAY_H, TRAY_LIP = 0.070, 0.200, 0.025, 0.010
TRAY_M = 0.12
MAT_C = np.array([0.260, -0.240])
MAT_L, MAT_W, MAT_T = 0.38, 0.33, 0.004


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


R_LOWER = np.eye(3)  # plate-local to world: the lower plate lies flat, face up
R_UPPER = _ry(-math.pi / 2)  # the upper plate stands in the lid: its local z (face) toward -x, its local x up

STEEL, DARK, PLATE_C, RIDGE_C, BLACK, BLUE, TRAY_COL, RED = (
    (0.74, 0.75, 0.77), (0.20, 0.20, 0.21), (0.13, 0.13, 0.14), (0.24, 0.24, 0.25), (0.04, 0.04, 0.04),
    (0.25, 0.55, 0.85), (0.30, 0.30, 0.32), (0.75, 0.12, 0.10))


def _v(p) -> tuple:
    return tuple(float(x) for x in p)


def _box(rot, centre, size, color, collide=True, yaw=0.0) -> BoxShape:
    """A box given in a plate-local frame (``centre`` relative to the frame's origin, turned ``yaw`` about local z),
    expressed in the world-aligned frame of the body through ``rot``."""
    return BoxShape(_v(rot @ np.asarray(centre, float)), _v(size), color, collide=collide,
                    quat_xyzw=_quat_from_matrix(rot @ _rz(yaw)))


def _disc(rot, centre, radius: float, thick: float, color, approximation: str = "none", n: int = 48) -> MeshShape:
    """A round disc (a 48-sided prism) whose axis is the local z axis turned by ``rot``, centred at ``centre`` (body
    frame): Isaac renders a cylinder lying on its side as a coarse polygon, so the standing disc is a mesh."""
    pts, nrm, faces = [], [], []
    h = thick / 2
    for sgn in (1, -1):  # the two caps: a centre point and a ring each, normals along the axis
        base = len(pts)
        pts.append((0.0, 0.0, sgn * h))
        nrm.append((0.0, 0.0, sgn))
        for k in range(n):
            a = 2 * math.pi * k / n
            pts.append((radius * math.cos(a), radius * math.sin(a), sgn * h))
            nrm.append((0.0, 0.0, sgn))
        for k in range(n):
            i, j = base + 1 + k, base + 1 + (k + 1) % n
            faces.append((base, i, j) if sgn > 0 else (base, j, i))
    base = len(pts)
    for k in range(n):  # the side, normals outward
        a = 2 * math.pi * k / n
        c, si = math.cos(a), math.sin(a)
        pts += [(radius * c, radius * si, h), (radius * c, radius * si, -h)]
        nrm += [(c, si, 0.0), (c, si, 0.0)]
    for k in range(n):
        t0, b0 = base + 2 * k, base + 2 * k + 1
        t1, b1 = base + 2 * ((k + 1) % n), base + 2 * ((k + 1) % n) + 1
        faces += [(b0, b1, t1), (b0, t1, t0)]
    p = np.asarray(pts) @ rot.T + np.asarray(centre, float)
    return MeshShape(p, np.asarray(faces, dtype=np.int32), np.asarray(nrm) @ rot.T, color,
                     approximation=approximation)


def _seat(rot, origin, body_origin) -> list:
    """A plate's seat in the grill's frame: the ring wall around the plate (with a notch at each handle) and the two
    latch lips under the plate's edge, built in the plate-local frame ``rot`` at ``origin``."""
    off = np.asarray(origin, float) - np.asarray(body_origin, float)
    out = []
    n = 28
    rc, w = (RIM_IN + RIM_OUT) / 2, RIM_OUT - RIM_IN
    zc, h = (-SEAT + 0.004) / 2, SEAT + 0.004
    for k in range(n):
        a = 2 * math.pi * (k + 0.5) / n
        if min(abs(a - math.pi / 2), abs(a - 3 * math.pi / 2)) < NOTCH:
            continue
        chord = 2 * rc * math.sin(math.pi / n) + 0.003
        b = _box(rot, (rc * math.cos(a), rc * math.sin(a), zc), (w, chord, h), STEEL, yaw=a)
        out.append(BoxShape(_v(np.asarray(b.centre) + off), b.size, b.color, quat_xyzw=b.quat_xyzw))
    for s in (1, -1):
        b = _box(rot, (0.0, s * (LIP_Y0 + LIP_Y1) / 2, (LIP_Z0 + LIP_Z1) / 2), (0.040, LIP_Y1 - LIP_Y0, LIP_Z1 - LIP_Z0),
                 DARK)
        out.append(BoxShape(_v(np.asarray(b.centre) + off), b.size, b.color, quat_xyzw=b.quat_xyzw))
        # the latch opening's dark recess under the lip, seen through the notch (visual)
        b = _box(rot, (0.0, s * 0.155, -SEAT + 0.0005), (0.050, 0.030, 0.001), BLACK, collide=False)
        out.append(BoxShape(_v(np.asarray(b.centre) + off), b.size, b.color, collide=False, quat_xyzw=b.quat_xyzw))
    return out


def _grill() -> Body:
    o = np.array([GX, GY, TOP])
    shapes = [CylShape((0.0, 0.0, BODY_Z0 / 2), PED_R, BODY_Z0, DARK),
              CylShape((0.0, 0.0, (BODY_Z0 + BODY_Z1) / 2), BODY_R, BODY_Z1 - BODY_Z0, STEEL),
              # the lid, open upright at the back, and its hinge
              _disc(R_UPPER, (LID_X + 0.012 - GX, 0.0, UPPER_Z), BODY_R, 0.024, STEEL),
              BoxShape((LID_X + 0.012 - GX, 0.0, (BODY_Z1 + UPPER_Z - BODY_R) / 2 + 0.003),
                       (0.024, 0.10, UPPER_Z - BODY_R - BODY_Z1 + 0.012), DARK),
              BoxShape((LID_X + 0.016 - GX, 0.0, BODY_Z1 + 0.004), (0.040, 0.12, 0.012), DARK),
              # the temperature dial and lights on the body's front (visual)
              CylShape((-BODY_R + 0.035 - 0.005, -0.120, BODY_Z1 + 0.001), 0.012, 0.004, BLACK, collide=False),
              CylShape((-BODY_R + 0.045, 0.120, BODY_Z1 + 0.001), 0.004, 0.003, RED, collide=False)]
    shapes += _seat(R_LOWER, LOWER_C, o)
    shapes += _seat(R_UPPER, UPPER_C, o)
    return Body.build("grill", pos=_v(o), mass=3.0, fixed=True, color=STEEL, friction=(0.5, 0.4), shapes=shapes)


def _plate(name: str, rot, centre) -> Linkage:
    """A cooking plate (the articulation's floating base) with a release handle on a spring slider on each side."""
    shapes = [_disc(rot, (0.0, 0.0, 0.0), PLATE_R, PLATE_T, PLATE_C, approximation="convexHull")]
    for k in range(9):  # the ridges on the cooking face
        y = -0.120 + 0.030 * k
        length = 2 * math.sqrt(PLATE_R ** 2 - y ** 2) - 0.020
        shapes.append(_box(rot, (0.0, y, PLATE_T / 2 + 0.002), (length, 0.006, 0.004), RIDGE_C))
    n = 16
    for k in range(n):  # the ring on the back that it rests on
        a = 2 * math.pi * (k + 0.5) / n
        chord = 2 * RING_R * math.sin(math.pi / n) + 0.003
        shapes.append(_box(rot, (RING_R * math.cos(a), RING_R * math.sin(a), (-SEAT - PLATE_T / 2) / 2),
                           (RING_W, chord, SEAT - PLATE_T / 2), PLATE_C, yaw=a))
    for s in (1, -1):  # the lugs that carry the handles (visual)
        shapes.append(_box(rot, (0.0, s * (PLATE_R + H_Y0) / 2 - s * 0.002, 0.0), (0.048, H_Y0 - PLATE_R + 0.006,
                                                                                  PLATE_T), PLATE_C, collide=False))
    links = []
    for s, side in ((1, "l"), (-1, "r")):
        hs = [_box(rot, (0.0, s * (H_Y0 + H_Y1) / 2, (H_Z0 + H_Z1) / 2), (H_W, H_Y1 - H_Y0, H_Z1 - H_Z0), BLACK),
              _box(rot, (0.0, s * (0.151 + H_Y0) / 2, (TONGUE_Z0 + H_Z0) / 2), (0.030, H_Y0 - 0.151, H_Z0 - TONGUE_Z0),
                   BLACK),  # the web down past the lip
              _box(rot, (0.0, s * (TONGUE_Y0 + 0.151) / 2, (TONGUE_Z0 + TONGUE_Z1) / 2),
                   (0.030, 0.151 - TONGUE_Y0, TONGUE_Z1 - TONGUE_Z0), BLACK),  # the tongue under the lip
              _box(rot, (0.0, s * (H_Y1 - 0.008), H_Z1 + 0.0005), (0.035, 0.004, 0.001), RIDGE_C, collide=False)]
        anchor = np.asarray(centre) + rot @ np.array([0.0, s * (H_Y0 + H_Y1) / 2, 0.0])
        links.append(Link(f"pad_{side}", tuple(hs), HANDLE_M, _v(anchor), "Y", None, slide=True,
                          limits=((0.0, TRAVEL) if s > 0 else (-TRAVEL, 0.0)), stiffness=SPRING, spring_damping=2.0,
                          report_contacts=True))
    return Linkage.build(name, pos=_v(centre), mass=PLATE_M, color=PLATE_C, floating=True, friction=(0.7, 0.6),
                         shapes=shapes, links=links, contact_offset=0.002, solver_iterations=32, report_contacts=True)


def _tray() -> Body:
    fl = 0.003
    lx = -TRAY_L / 2 + TRAY_LIP / 2
    shapes = [BoxShape((0.0, 0.0, fl / 2), (TRAY_L, TRAY_W, fl), TRAY_COL),
              BoxShape((lx, 0.0, 0.015), (TRAY_LIP, TRAY_W, 0.030), TRAY_COL),  # the front lip
              BoxShape((TRAY_L / 2 - 0.0015, 0.0, TRAY_H / 2), (0.003, TRAY_W, TRAY_H), TRAY_COL),
              BoxShape((0.0, TRAY_W / 2 - 0.0015, TRAY_H / 2), (TRAY_L, 0.003, TRAY_H), TRAY_COL),
              BoxShape((0.0, -TRAY_W / 2 + 0.0015, TRAY_H / 2), (TRAY_L, 0.003, TRAY_H), TRAY_COL)]
    return Body.build("drip_tray", pos=(float(TRAY_C[0]), float(TRAY_C[1]), TOP + 0.001), mass=TRAY_M,
                      color=TRAY_COL, friction=(0.5, 0.4), shapes=shapes, report_contacts=True)


MAT = Body.build("drying_mat", pos=(float(MAT_C[0]), float(MAT_C[1]), TOP + MAT_T / 2), mass=0.3, fixed=True,
                 color=BLUE, friction=(0.9, 0.8), shapes=[BoxShape((0.0, 0.0, 0.0), (MAT_L, MAT_W, MAT_T), BLUE)])
GRILL = _grill()
UPPER = _plate("upper_plate", R_UPPER, UPPER_C + np.array([-0.001, 0.0, 0.0]))
LOWER = _plate("lower_plate", R_LOWER, LOWER_C + np.array([0.0, 0.0, 0.001]))
TRAY = _tray()
OBJECTS = [MAT, GRILL, UPPER, LOWER, TRAY]
PLATES = {"upper_plate": (R_UPPER, UPPER_C), "lower_plate": (R_LOWER, LOWER_C)}
PARTS = [(p, part) for p in PLATES for part in ("base", "pad_l", "pad_r")] + [("drip_tray", "body")]

# -- tracking and scoring -------------------------------------------------------------------------------------------
HOLD_N = 1.0  # a gripper holds a handle when it presses on it with this much force
TOUCH_N = 0.1  # released: no arm link touches the part with this much force
OFF_M = 0.03  # a plate is off its seat once its centre has moved this far
DROP_V = 0.77  # m/s: a plate falling faster than this (a free fall of 3 cm) is dropped
FLAT_DEG = 10.0
REST_M = 0.006
TRAY_OUT_R = 0.175
TRAY_REST_M = 0.004
STILL_STEPS, STILL_MM = 20, 1.0
_DEBUG = os.environ.get("CGRILL_DEBUG") == "1"
_DEBUG_EVERY = int(os.environ.get("CGRILL_DEBUG_EVERY", "5"))
_EP: dict = {}
_IDS: dict = {}
_CV: dict = {}
ARM_PRIMS = ("LeftArm", "RightArm")


def _plate_points(rot) -> np.ndarray:
    """Plate-body-frame points that bound a plate: its back ring's bottom, its rim's back and its ridges' tops."""
    pts = []
    for k in range(12):
        a = 2 * math.pi * k / 12
        c, s = math.cos(a), math.sin(a)
        pts += [((RING_R + RING_W / 2) * c, (RING_R + RING_W / 2) * s, -SEAT),
                (PLATE_R * c, PLATE_R * s, -PLATE_T / 2), (0.13 * c, 0.13 * s, PLATE_T / 2 + 0.004)]
    return np.asarray(pts) @ rot.T


_PTS = {p: _plate_points(r) for p, (r, _) in PLATES.items()}
_NORMAL = {p: r @ np.array([0.0, 0.0, 1.0]) for p, (r, _) in PLATES.items()}
_TRAY_BACK = np.array([[TRAY_L / 2, -TRAY_W / 2, 0.0], [TRAY_L / 2, 0.0, 0.0], [TRAY_L / 2, TRAY_W / 2, 0.0]])
_TRAY_LOW = np.array([[sx * TRAY_L / 2, sy * TRAY_W / 2, 0.0] for sx in (-1, 1) for sy in (-1, 1)])


@on_reset
def _reset() -> None:
    _EP.clear()
    _EP.update(tick=0, step=0, counted=-1, recent=[], forces={}, tcp={},
               off={p: False for p in PLATES}, dropped={p: False for p in PLATES}, max_fall={p: 0.0 for p in PLATES},
               state={})
    _CV.clear()


_reset()


def _contacts(base) -> dict:
    """{(object, part): (left arm's largest contact force, right arm's)} in N, from PhysX's pair-wise contact forces."""
    from isaaclab_physx.physics import PhysxManager

    sim = PhysxManager.get_physics_sim_view()
    if _CV.get("sim") is not sim:
        from yam_arena.robot import asset_link_paths

        env = base.scene.env_prim_paths[0]
        links = [f"{env}/{arm}{path}" for arm in ARM_PRIMS for path in asset_link_paths().values()]
        parts = [f"{env}/{o}/{p}" for o, p in PARTS]
        view = sim.create_rigid_contact_view(parts, filter_patterns=[links] * len(parts),
                                             max_contact_data_count=16 * len(links) * len(parts))
        _CV.update(sim=sim, view=view, dt=float(base.physics_dt), nl=len(links))
    if _DEBUG and "gview" not in _CV:
        env = base.scene.env_prim_paths[0]
        gparts = [f"{env}/lower_plate/{p}" for p in ("base", "pad_l", "pad_r")]
        _CV["gview"] = sim.create_rigid_contact_view(gparts, filter_patterns=[[f"{env}/grill/body"]] * 3,
                                                     max_contact_data_count=64)
    if _DEBUG:
        g = _CV["gview"].get_contact_force_matrix(dt=_CV["dt"])
        g = np.asarray(g.numpy() if hasattr(g, "numpy") else as_numpy(g), float).reshape(3, 1, 3)
        _EP["grill_f"] = [tuple(np.round(g[i, 0], 1)) for i in range(3)]
    m = _CV["view"].get_contact_force_matrix(dt=_CV["dt"])
    a = np.asarray(m.numpy() if hasattr(m, "numpy") else as_numpy(m), float).reshape(len(PARTS), _CV["nl"], 3)
    f = np.linalg.norm(a, axis=-1)
    h = _CV["nl"] // 2
    return {k: (float(f[i, :h].max()), float(f[i, h:].max())) for i, k in enumerate(PARTS)}


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("CGRILL_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _over_mat(p) -> bool:
    return bool(abs(p[0] - MAT_C[0]) <= MAT_L / 2 and abs(p[1] - MAT_C[1]) <= MAT_W / 2)


def _track_inner(env) -> None:
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS

    _EP["tick"] += 1
    base = getattr(env, "unwrapped", env)
    origin = as_numpy(base.scene.env_origins[0])
    dt = float(getattr(base, "step_dt", 0.05))
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        if side not in _IDS:
            _IDS[side] = list(robot.find_joints(ARM_JOINTS, preserve_order=True)[0])
        q = as_numpy(robot.data.joint_pos)[0]
        _EP["tcp"][side] = np.asarray(ik.tcp_pose(q[_IDS[side]], station.BASES[side])[0], float)
    f = _contacts(base)
    st = {}
    for name in PLATES:
        asset = base.scene[name]
        pose = as_numpy(asset.data.root_link_pose_w)[0]
        p, r = pose[:3] - origin, rotation(pose[3:7])
        names = list(asset.joint_names)
        jq = as_numpy(asset.data.joint_pos)[0]
        pts = p + _PTS[name] @ r.T
        prev = _EP["state"].get(name)
        fall = 0.0 if prev is None else (float(prev["p"][2]) - float(p[2])) / dt
        cos_t = abs(float((r @ _NORMAL[name])[2]))
        held = (max(f[(name, "pad_l")]) >= HOLD_N and max(f[(name, "pad_r")]) >= HOLD_N)
        touch = max(max(f[(name, k)]) for k in ("base", "pad_l", "pad_r"))
        moved = float(np.linalg.norm(p - PLATES[name][1]))
        st[name] = dict(p=p, low=float(pts[:, 2].min()) - TOP, high=float(pts[:, 2].max()) - TOP, fall=fall,
                        tilt=math.degrees(math.acos(min(1.0, cos_t))), held=held, touch=touch, moved=moved,
                        pads=(float(jq[names.index("pad_l_turn")]), float(jq[names.index("pad_r_turn")])))
        _EP["max_fall"][name] = max(_EP["max_fall"][name], fall)
        if fall > DROP_V:
            _EP["dropped"][name] = True
        if moved >= OFF_M and held and not _EP["dropped"][name]:
            _EP["off"][name] = True
    tr = base.scene["drip_tray"]
    tp = as_numpy(tr.data.root_pose_w)[0] if hasattr(tr.data, "root_pose_w") else None
    if tp is None:
        tp = as_numpy(tr.data.root_link_pose_w)[0]
    p, r = tp[:3] - origin, rotation(tp[3:7])
    back = p + _TRAY_BACK @ r.T
    low = p + _TRAY_LOW @ r.T
    st["drip_tray"] = dict(p=p, out=float(np.min(np.linalg.norm(back[:, :2] - [GX, GY], axis=1))),
                           low=float(low[:, 2].min()) - TOP,
                           tilt=math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2]))))),
                           touch=max(f[("drip_tray", "body")]))
    _EP.update(forces=f, state=st)


def _tally(o) -> dict:
    st = _EP["state"]
    recent = _EP["recent"]
    if not st:
        return dict(score=0.0, done=False, placed={}, tray=False, still=False, clear=False)
    if _EP["counted"] != _EP["tick"]:
        _EP["counted"] = _EP["tick"]
        recent.append(np.concatenate([st[k]["p"] for k in ("upper_plate", "lower_plate", "drip_tray")]))
        del recent[:-STILL_STEPS]
    still = len(recent) == STILL_STEPS and all(np.abs(q - recent[-1]).max() * 1000 < STILL_MM for q in recent)
    placed = {}
    for name in PLATES:
        s = st[name]
        other = st["lower_plate" if name == "upper_plate" else "upper_plate"]
        on_mat = abs(s["low"] - MAT_T) <= REST_M
        on_other = (_over_mat(other["p"]) and other["tilt"] <= FLAT_DEG and abs(s["low"] - other["high"]) <= REST_M
                    and float(np.linalg.norm(s["p"][:2] - other["p"][:2])) < PLATE_R)
        placed[name] = bool(_EP["off"][name] and not _EP["dropped"][name] and s["tilt"] <= FLAT_DEG
                            and _over_mat(s["p"]) and (on_mat or on_other) and s["touch"] < TOUCH_N)
    t = st["drip_tray"]
    tray = bool(t["out"] >= TRAY_OUT_R and t["low"] <= TRAY_REST_M and t["tilt"] <= FLAT_DEG and t["touch"] < TOUCH_N)
    score = 0.0
    for name in PLATES:
        if _EP["off"][name] and not _EP["dropped"][name]:
            score += 0.15 + (0.20 if placed[name] else 0.0)
    score += 0.20 if tray else 0.0
    clear = max(max(v) for v in _EP["forces"].values()) < TOUCH_N
    done = bool(all(placed.values()) and tray and clear and still)
    return dict(score=round(score, 4), done=done, placed=placed, tray=tray, still=still, clear=clear)


def _debug(t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    st = _EP.get("state", {})
    parts = []
    for name in PLATES:
        s = st.get(name)
        if not s:
            continue
        f = _EP["forces"]
        parts.append(f"{name[:2]}: p=({s['p'][0]:.3f},{s['p'][1]:.3f},{s['p'][2] - TOP:.3f}) mv={s['moved'] * 100:.1f}cm "
                     f"tilt={s['tilt']:.1f} low={s['low'] * 1000:.1f} hi={s['high'] * 1000:.1f} "
                     f"pads=({s['pads'][0] * 1000:+.1f},{s['pads'][1] * 1000:+.1f})mm "
                     f"F[l]={f[(name, 'pad_l')][0]:.1f}/{f[(name, 'pad_l')][1]:.1f} "
                     f"F[r]={f[(name, 'pad_r')][0]:.1f}/{f[(name, 'pad_r')][1]:.1f} "
                     f"F[b]={max(f[(name, 'base')]):.1f} held={int(s['held'])} off={int(_EP['off'][name])} "
                     f"drop={int(_EP['dropped'][name])} vmax={_EP['max_fall'][name]:.2f} "
                     f"placed={int(t['placed'].get(name, False))}")
    tr = st.get("drip_tray")
    if tr:
        parts.append(f"tray: p=({tr['p'][0]:.3f},{tr['p'][1]:.3f}) out={tr['out'] * 100:.1f}cm low={tr['low'] * 1000:.1f} "
                     f"tilt={tr['tilt']:.1f} F={tr['touch']:.1f} ok={int(t['tray'])}")
    print(f"DBG step={_EP['step']} gf={_EP.get('grill_f')} score={t['score']:.2f} still={int(t['still'])} clear={int(t['clear'])} | "
          + " | ".join(parts) + " | "
          + " ".join(f"{k}=({v[0]:.3f},{v[1]:.3f},{v[2] - TOP:.3f})" for k, v in _EP["tcp"].items()), flush=True)


def score(objects) -> float:
    t = _tally(objects)
    if _DEBUG:
        _debug(t)
    if t["done"]:
        return 1.0
    return min(t["score"], 0.9)


def success(objects) -> bool:
    return bool(_tally(objects)["done"])


# -- scripted demo --------------------------------------------------------------------------------------------------
# Open loop, from the known layout. Upper plate: both grippers come in level from the front, pointing forward, and
# pinch the two handles top and bottom; they pull the handles apart to their stops (commanded 4.5 cm past, the arms
# give sideways), draw the plate 5 cm forward out of the lid, carry it over the drying mat while turning it face up
# (both hands turn together about the line through the handles, ending pointing down), lay it flat on the mat and let
# go. Lower plate: both grippers come down on its handles, pinch them front and back, pull them apart, lift the plate
# 8 cm, carry it over the mat and lay it on the upper plate, 4.5 cm further out, and let go. Drip tray: the left gripper
# pinches its front lip and slides it 5.5 cm toward the robot, out from under the body, and lets go.
# CGRILL_MODE=one runs it the wrong way: the left gripper alone pulls the upper plate's left handle and draws the plate
# forward (the right latch holds it), then does the same with the lower plate's left handle and lifts.
OPEN_C = -0.047
MODE = os.environ.get("CGRILL_MODE", "demo")  # demo | one (wrong: the left arm alone) | lower (lower plate only)
PAD_GRIP = float(os.environ.get("CGRILL_PAD_GRIP", "-0.0185"))  # closed on the 4.5 cm handle
LIP_GRIP = float(os.environ.get("CGRILL_LIP_GRIP", "-0.002"))  # closed on the tray's 1.0 cm lip
LIP_OPEN = -0.015  # opened only 3 cm over the lip: the far finger must stay in front of the body's front edge
PULL = float(os.environ.get("CGRILL_PULL", "0.055"))  # handles commanded this far out (they stop at 10 mm)
HOLD_OUT = float(os.environ.get("CGRILL_HOLD_OUT", "0.020"))  # and this far while carrying
TCP_Y = float(os.environ.get("CGRILL_TCP_Y", "0.186"))  # the TCP on the handle, plate-local |y|
TCP_Z = float(os.environ.get("CGRILL_TCP_Z", "0.010"))  # and plate-local z (toward the cooking face)
SAG = float(os.environ.get("CGRILL_SAG", "0.010"))  # the arms sag this much under a plate: commanded higher
FWD = float(os.environ.get("CGRILL_FWD", "0.055"))  # the upper plate is drawn this far forward out of the lid
UP_PLACE = np.array([0.240, -0.245])
LO_PLACE = np.array([0.285, -0.245])
TRAY_SLIDE = float(os.environ.get("CGRILL_TRAY_SLIDE", "0.055"))
DOWN_L = grip_rotation(-math.pi / 2)  # left arm pointing down, fingers closing along x
DOWN_R = grip_rotation(math.pi / 2)
DOWN = {"left": DOWN_L, "right": DOWN_R}
SIDE = {"left": 1, "right": -1}


def _good_ik(arm: str, pos, rot):
    from yam_arena import ik, station

    best = None
    b = station.BASES[arm]
    j1 = math.atan2(float(pos[1]) - b[1], float(pos[0]) - b[0])
    for sd in itertools.product((j1, 0.0), (0.8, 1.3), (0.6, 1.2), (-0.8, 0.8), (-1.2, 0.0, 1.2),
                                (-1.8, -0.9, 0.0, 0.9, 1.8)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.asarray(sd, float),
                             base_pos=b, iters=150)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
                if room > 0.3:
                    break
    if best is None:
        raise ValueError(f"no IK solution for the {arm} arm at {pos}")
    return best[0]


def _lead_in(arm: str, pos, rot, finger: float, label: str, seconds: float = 3.0, steps: int = 15) -> list:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, station.BASES[arm])
        out.append(Waypoint(seconds / steps, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, p, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in p), finger, label, rot)


def _stay(arm: str, t: float = 120.0) -> list:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return [Waypoint(t, tuple(float(v) for v in pos), OPEN_C, "stay put", rot)]


def _tcp(arm: str, rot, centre, out: float = 0.0, z: float = TCP_Z) -> np.ndarray:
    """The TCP on ``arm``'s handle of a plate turned ``rot`` with its centre at ``centre``, the handle ``out`` m out."""
    return np.asarray(centre, float) + rot @ np.array([0.0, SIDE[arm] * (TCP_Y + out), z])


def _grip(arm: str, rot) -> np.ndarray:
    return rot @ DOWN[arm]  # down in the plate's frame: closing across the handle's 4.5 cm width


UP = np.array([0.0, 0.0, 1.0])
XV = np.array([1.0, 0.0, 0.0])


def _upper(arm: str) -> list:
    """The upper plate, for one arm (both run the same timeline)."""
    g = _grip(arm, R_UPPER)
    c = UPPER_C
    pre = _tcp(arm, R_UPPER, c) - XV * 0.06
    out = _lead_in(arm, pre, g, OPEN_C, "in front of the upper plate's handle", seconds=4.0)
    out += [_w(2.0, _tcp(arm, R_UPPER, c), OPEN_C, "fingers over and under the handle", g),
            _w(1.0, _tcp(arm, R_UPPER, c), PAD_GRIP, "pinch the handle", g),
            _w(0.5, _tcp(arm, R_UPPER, c), PAD_GRIP, "settle", g),
            _w(1.5, _tcp(arm, R_UPPER, c, PULL), PAD_GRIP, "pull the handle out", g),
            _w(1.0, _tcp(arm, R_UPPER, c, PULL), PAD_GRIP, "hold it out", g)]
    if MODE == "one" and arm == "left":
        out += [_w(2.0, _tcp(arm, R_UPPER, c - XV * FWD, PULL), PAD_GRIP, "draw the plate forward", g),
                _w(2.0, _tcp(arm, R_UPPER, c - XV * FWD, PULL) + UP * 0.03, PAD_GRIP, "and up", g),
                _w(1.0, _tcp(arm, R_UPPER, c - XV * FWD, PULL) + UP * 0.03, OPEN_C, "let go", g),
                _w(2.0, _tcp(arm, R_UPPER, c - XV * 0.12, 0.0) + UP * 0.03, OPEN_C, "back off", g)]
        return out
    c1 = c - XV * FWD + UP * 0.005
    out += [_w(2.0, _tcp(arm, R_UPPER, c1, PULL) + UP * SAG, PAD_GRIP, "draw it forward out of the lid", g)]
    # carry it over the mat, turning it face up about the line through the handles
    c2 = np.array([UP_PLACE[0], UP_PLACE[1], TOP + MAT_T + SEAT + 0.003])
    hover = 0.10
    n = 14
    for k in range(1, n + 1):
        u = k / n
        a = -math.pi / 2 * (1 - u)  # the plate's turn: R_y(-90) (standing) to identity (flat, face up)
        rot = _ry(a)
        cc = c1 + (c2 + UP * hover - c1) * u
        out.append(_w(7.0 / n, _tcp(arm, rot, cc, HOLD_OUT) + UP * SAG, PAD_GRIP, "carry it over the mat, turning "
                      "it face up", _grip(arm, rot)))
    gd = _grip(arm, np.eye(3))
    out += [_w(2.5, _tcp(arm, np.eye(3), c2, HOLD_OUT) + UP * 0.002, PAD_GRIP, "lay it flat on the mat", gd),
            _w(0.5, _tcp(arm, np.eye(3), c2, HOLD_OUT) + UP * 0.002, PAD_GRIP, "settle", gd),
            _w(1.0, _tcp(arm, np.eye(3), c2, HOLD_OUT) + UP * 0.002, OPEN_C, "let go", gd),
            _w(1.5, _tcp(arm, np.eye(3), c2, HOLD_OUT) + UP * 0.08, OPEN_C, "rise clear", gd)]
    return out


def _lower(arm: str) -> list:
    g = _grip(arm, R_LOWER)
    c = LOWER_C
    t0 = _tcp(arm, R_LOWER, c)
    out = [_w(2.5, t0 + UP * 0.05, OPEN_C, "over the lower plate's handle", g),
           _w(0.8, t0 + UP * 0.05, OPEN_C, "settle over it", g),
           _w(2.0, t0, OPEN_C, "fingers either side of the handle", g),
           _w(1.0, t0, PAD_GRIP, "pinch the handle", g),
           _w(0.5, t0, PAD_GRIP, "settle", g),
           _w(1.5, _tcp(arm, R_LOWER, c, PULL), PAD_GRIP, "pull the handle out", g),
           _w(0.5, _tcp(arm, R_LOWER, c, PULL), PAD_GRIP, "hold it out", g)]
    if MODE == "one" and arm == "left":
        out += [_w(2.0, _tcp(arm, R_LOWER, c, PULL) + UP * 0.08, PAD_GRIP, "lift", g),
                _w(1.0, _tcp(arm, R_LOWER, c, PULL) + UP * 0.08, OPEN_C, "let go", g),
                _w(2.0, _tcp(arm, R_LOWER, c, 0.0) + UP * 0.08, OPEN_C, "rise clear", g)]
        return out
    lift = float(os.environ.get("CGRILL_LIFT", "0.050"))
    c2 = np.array([LO_PLACE[0], LO_PLACE[1], TOP + MAT_T + SEAT + (PLATE_T / 2 + 0.004) + SEAT + 0.003])
    out += [_w(2.0, _tcp(arm, R_LOWER, c, PULL) + UP * (lift + SAG), PAD_GRIP, "lift it off", g),
            _w(0.5, _tcp(arm, R_LOWER, c, HOLD_OUT) + UP * (lift + SAG), PAD_GRIP, "hold it up", g),
            _w(4.0, _tcp(arm, R_LOWER, c2, HOLD_OUT) + UP * (lift + SAG), PAD_GRIP, "carry it over the mat", g),
            _w(2.5, _tcp(arm, R_LOWER, c2, HOLD_OUT) + UP * 0.002, PAD_GRIP, "lay it on the upper plate", g),
            _w(0.5, _tcp(arm, R_LOWER, c2, HOLD_OUT) + UP * 0.002, PAD_GRIP, "settle", g),
            _w(1.0, _tcp(arm, R_LOWER, c2, HOLD_OUT) + UP * 0.002, OPEN_C, "let go", g),
            _w(1.5, _tcp(arm, R_LOWER, c2, HOLD_OUT) + UP * 0.045, OPEN_C, "rise clear", g)]
    return out


def _tray_moves() -> list:
    g = DOWN_L
    lip = np.array([TRAY_C[0] - TRAY_L / 2 + TRAY_LIP / 2, TRAY_C[1], TOP + 0.020])
    out = [_w(2.0, np.array([0.20, 0.10, TOP + 0.12]), OPEN_C, "toward the drip tray", g),
           _w(2.0, lip + UP * 0.08, LIP_OPEN, "over the drip tray's lip", g),
           _w(1.5, lip, LIP_OPEN, "fingers either side of the lip", g),
           _w(1.0, lip, LIP_GRIP, "pinch the lip", g),
           _w(2.5, lip - XV * TRAY_SLIDE + UP * 0.002, LIP_GRIP, "slide the tray out toward you", g),
           _w(0.5, lip - XV * TRAY_SLIDE + UP * 0.002, LIP_GRIP, "settle", g),
           _w(1.0, lip - XV * TRAY_SLIDE + UP * 0.002, OPEN_C, "let go", g),
           _w(1.5, lip - XV * TRAY_SLIDE + UP * 0.10, OPEN_C, "rise clear", g),
           _w(12.0, lip - XV * (TRAY_SLIDE - 0.05) + UP * 0.18, OPEN_C, "stay clear", g)]
    return out


def demo():
    if MODE == "one":
        left = _upper("left") + _lower("left")
        left.append(_w(10.0, np.array([0.20, 0.25, TOP + 0.15]), OPEN_C, "stay clear", left[-1].yaw))
        return {"left": left, "right": _stay("right")}
    left, right = (_upper("left"), _upper("right")) if MODE != "lower" else ([], [])
    if MODE == "lower":
        left = _lead_in("left", _tcp("left", R_LOWER, LOWER_C) + UP * 0.05, DOWN_L, OPEN_C, "start")
        right = _lead_in("right", _tcp("right", R_LOWER, LOWER_C) + UP * 0.05, DOWN_R, OPEN_C, "start")
    left += _lower("left")
    right += _lower("right")
    right.append(_w(16.0, np.array([0.20, -0.30, TOP + 0.16]), OPEN_C, "stay clear", DOWN_R))
    left += _tray_moves()
    return {"left": left, "right": right}
