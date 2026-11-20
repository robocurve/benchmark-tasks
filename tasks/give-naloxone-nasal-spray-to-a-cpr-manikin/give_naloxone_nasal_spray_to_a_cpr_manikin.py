"""On the station table: an adult CPR training manikin lies on its back, head toward the robot, face up. Its head
turns on a sprung neck joint: tilted back (chin up) it springs back to level when let go. To the robot's right, a
naloxone nasal spray training device (the size and shape of the 4 mg NARCAN Nasal Spray) lies in an open holder,
nozzle toward the robot.

Task (instruction): give the manikin its dose of the nasal spray. The procedure is the one printed on the package
(NARCAN Nasal Spray Quick Start Guide, steps 2 and 3): "Hold the NARCAN Nasal Spray with your thumb on the bottom of
the plunger and your first and middle fingers on either side of the nozzle." "Tilt the person's head back and provide
support under the neck with your hand. Gently insert the tip of the nozzle into one nostril, until your fingers on
either side of the nozzle are against the bottom of the person's nose." "Press the plunger firmly to give the dose."

Why two hands: the device is fired by a squeeze between the plunger and the finger flanges, so the hand holding it is
the hand that fires it (a parallel gripper squeezes like thumb and fingers). The head does not stay tilted back on
its own: the neck springs it back to level, so the other hand must hold it tilted while the dose is given.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
Head frame (manikin_head.py): origin at the nasal root, x back from the face, y to the head's left, z up the face.
The manikin lies face up with the top of its head toward the robot: head x = world -z, head y = world -y,
head z = world -x. The nasal root is at (0.210, 0.000) and 0.222 above the table when the head is level.
- neck: a revolute joint about the world y axis, 110 mm behind and 50 mm below the nasal root (head frame), with a
  spring (1.6 N m/rad, preloaded 0.17 N m against the level stop) and a stop at 30 deg of tilt.
- nostrils: two openings in the base of the nose, 12 x 8 mm, their middles 8 mm either side of the midline and 10 mm
  in front of the nasal root's plane, 47 mm below it; each runs 20 mm up into the nose along an axis leaning 25 deg
  back from the face's long axis. With the head level that axis points 25 deg below horizontal toward the top of the
  head; tilted back by t it points (25 + t) deg below horizontal.
- spray device (training device, NARCAN size): nozzle 15 mm long (6 mm across, the last 6 mm 4.5 mm across); finger
  flange 42 x 10 x 3 mm across the nozzle's base; body 16 mm across, 20 mm long; plunger with a 28 mm thumb disc,
  standing 9 mm out of the body, 5 mm stroke. It fires only at the end of the stroke (it clicks): the plunger is held
  out by a spring preloaded to 15 N (rising to 16.5 N at the end). 15 g. It lies in a holder (70 x 50 mm) at
  (0.200, -0.120), its axis along x 20 mm above the table, nozzle toward the robot, the flange across (along y).

Score (read every 50 ms step). A dose is given the first time the plunger reaches the end of its stroke (within
0.5 mm); the device holds one dose, so only that moment counts and the score is fixed from then on:
- 0 when the nozzle's tip is not at least 3 mm inside a nostril opening, or its axis is more than 20 deg off that
  nostril's axis (the spray goes onto the face or the nostril wall);
- 1.0 when, in addition, (a) the device or the hand holding it was pressed against the nose (contact force 0.5 N
  or more in the second before the click: inserted until the fingers are against the bottom of the nose), (b) the
  head was tilted back at least 15 deg, and (c) the other hand was on the head (1 N or more);
- 0.5 when it went into the nostril but (a), (b) or (c) was missing.
No dose by the end of the run scores 0.
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import manikin_head as mh  # noqa: E402
from bodykit import Body, BoxShape, CylShape, Joint, MeshShape, SphereShape  # noqa: E402
from bodykit import allow_matrix_grips, as_numpy, on_reset, on_step, rotation  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
MM = 0.001

SKIN = (0.86, 0.72, 0.62)
LIP = (0.78, 0.52, 0.50)
NOSTRIL_C = (0.18, 0.10, 0.10)
HAIR = (0.30, 0.22, 0.16)
TORSO_C = (0.86, 0.72, 0.62)
SHIRT = (0.25, 0.35, 0.55)
MAT = (0.20, 0.42, 0.30)
DEVICE_W = (0.93, 0.93, 0.92)
PLUNGER_C = (0.85, 0.20, 0.25)
NOZZLE_C = (0.95, 0.95, 0.95)
HOLDER_C = (0.30, 0.32, 0.36)


def _v(p) -> tuple:
    return tuple(float(x) for x in p)


def _quat(m: np.ndarray) -> tuple:
    from bodykit import _quat_from_matrix

    return _quat_from_matrix(np.asarray(m, float))


def unit(v) -> np.ndarray:
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def _rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


# -- the head -----------------------------------------------------------------------------------------------------------
# head frame -> world, the head level: head x = -world z, head y = -world y, head z = -world x
M_HEAD = np.array([[0.0, 0.0, -1.0], [0.0, -1.0, 0.0], [-1.0, 0.0, 0.0]])
S0 = np.array([0.210, 0.000, TOP + 0.222])  # the nasal root, head level
PIVOT_H = np.array([110.0, 0.0, -50.0]) * MM  # the neck joint (head frame)
PIVOT = S0 + M_HEAD @ PIVOT_H
NECK_K = 1.6  # N m/rad
NECK_PRE = 6.0  # deg: the spring's rest lies this far past the level stop (preload 0.17 N m)
NECK_MAX = 30.0  # deg of tilt at the stop
NECK_DAMP = 0.3  # N m s/rad

# the nostrils (head frame): the openings' middles, the axis up into the nose, the opening's half sizes
NOSE_O = np.array([-10.0, 0.0, -47.0]) * MM  # on the midline, between the two openings
NOSTRIL_Y = 8.0 * MM
AXIS_LEAN = math.radians(25.0)
E_W = np.array([math.sin(AXIS_LEAN), 0.0, math.cos(AXIS_LEAN)])  # up the nostril
E_V = np.array([0.0, 1.0, 0.0])
E_U = np.cross(E_V, E_W)  # toward the lip (back and down the face)
HALF_U, HALF_V, DEPTH = 6.0 * MM, 4.0 * MM, 20.0 * MM
R_NOSTRIL = np.column_stack([E_U, E_V, E_W])  # nostril frame -> head frame
UNSEEN = (-1.0, 0.0, 0.0)  # colliders under the drawn skin are not drawn themselves


def _nb(u0, u1, v0, v1, w0, w1, color=UNSEEN, collide=True) -> BoxShape:
    """A box given by ranges (m) in the nostril frame (u toward the lip, v to the head's left, w up the nostril)."""
    c = NOSE_O + R_NOSTRIL @ np.array([(u0 + u1) / 2, (v0 + v1) / 2, (w0 + w1) / 2])
    return BoxShape(_v(c), (u1 - u0, v1 - v0, w1 - w0), color, collide=collide, quat_xyzw=_quat(R_NOSTRIL))


def _hb(c_mm, size_mm, color=UNSEEN, collide=True) -> BoxShape:
    return BoxShape(_v(np.asarray(c_mm, float) * MM), _v(np.asarray(size_mm, float) * MM), color, collide=collide)


def _head_mesh():
    d = np.load(HERE / "manikin_head_mesh.npz")
    return d["points"].astype(float), d["faces"], d["normals"].astype(float)


def _hull(pts: np.ndarray, keep) -> MeshShape:
    """A collider: the convex hull of the skin's points selected by ``keep`` (head frame, mm), not drawn."""
    from scipy.spatial import ConvexHull

    p = pts[keep(pts / MM)]
    h = ConvexHull(p)
    v = p[np.unique(h.simplices)]
    h2 = ConvexHull(v)
    return MeshShape(v, h2.simplices.astype(np.int32), None, UNSEEN, approximation="convexHull")


def _head_shapes() -> list:
    pts, faces, normals = _head_mesh()
    out = [MeshShape(pts, faces, normals, SKIN, collide=False)]  # the moulded skin, seen; it collides as the hulls below
    hu, hv = HALF_U, HALF_V
    yo = NOSTRIL_Y
    # the base of the nose with its two nostrils (each 12 x 8 mm, 20 mm deep); the boxes start 3 mm up so that none
    # of them pokes out under the skin of the base (it lies across the leaning nostrils)
    w0 = 3.0 * MM
    out += [_nb(-12 * MM, -hu, -15 * MM, 15 * MM, w0, 15 * MM),  # in front of the openings, under the tip
            _nb(hu, 9 * MM, -15 * MM, 15 * MM, w0 + 1.5 * MM, 15 * MM),  # behind them (toward the lip)
            _nb(-hu, hu, -(yo - hv), yo - hv, w0, DEPTH + 3 * MM),  # the septum between them
            _nb(-hu, hu, yo + hv, 15 * MM, w0, DEPTH),  # left ala
            _nb(-hu, hu, -15 * MM, -(yo + hv), w0, DEPTH),  # right ala
            _nb(-hu, hu, yo - hv, yo + hv, DEPTH, DEPTH + 3 * MM),  # the nostrils' ends
            _nb(-hu, hu, -(yo + hv), -(yo - hv), DEPTH, DEPTH + 3 * MM)]
    for s in (1.0, -1.0):  # the dark openings, seen, on the skin of the nose's base
        out.append(_hb((-10.0, s * yo / MM, -47.4), (11.0, 7.0, 0.4), NOSTRIL_C, collide=False))
    # the rest of the face and head: hulls of the skin's points
    out += [_hull(pts, lambda q: (q[:, 2] > 18) & (q[:, 2] < 80) & (np.abs(q[:, 1]) < 48) & (q[:, 0] < 30)),  # forehead
            _hull(pts, lambda q: (q[:, 2] > -36) & (q[:, 2] < -14) & (q[:, 0] < -13)),  # tip of the nose
            _hull(pts, lambda q: (q[:, 2] > -68) & (q[:, 2] < -51) & (np.abs(q[:, 1]) < 24) & (q[:, 0] < 12)),  # lips
            _hull(pts, lambda q: (q[:, 2] > -125) & (q[:, 2] < -68) & (q[:, 0] < 40)),  # chin and jaw
            _hull(pts, lambda q: (q[:, 2] > -60) & (q[:, 2] < -12) & (q[:, 1] > 26) & (q[:, 0] < 35)),  # left cheek
            _hull(pts, lambda q: (q[:, 2] > -60) & (q[:, 2] < -12) & (q[:, 1] < -26) & (q[:, 0] < 35)),  # right cheek
            SphereShape(_v(np.array([92.0, 0.0, 30.0]) * MM), 0.072, UNSEEN, collide=True)]
    out.append(_hb((-2.5, 0, -60), (2, 40, 6), LIP, collide=False))
    for s in (1.0, -1.0):  # closed eyes: lid lines
        out.append(_hb((-0.5, s * 31.5, -5), (1.5, 22, 1.2), (0.45, 0.32, 0.28), collide=False))
    return out


HEAD = Body.build("manikin_head", pos=_v(S0), mass=0.9, color=SKIN, shapes=_head_shapes(),
                  init_quat=_quat(M_HEAD), gravity=False, friction=(0.8, 0.7), report_contacts=True,
                  damping=(0.5, 2.0), solver_iterations=32,
                  joint=Joint(anchor=_v(PIVOT), kind="revolute", axis="Y", limits=(-NECK_MAX, 0.0),
                              stiffness=NECK_K, damping=NECK_DAMP, target=NECK_PRE))


def _torso() -> Body:
    """The manikin's chest and shoulders (fixed; only their top half is in view) and its neck (seen only)."""
    c = np.array([0.480, 0.0, TOP + 0.071])  # the chest block's middle: 0.35 to 0.61 m out, 40 cm wide, 14 cm high
    neck = np.array([(PIVOT[0] + 0.35) / 2, 0.0, PIVOT[2] - 0.005])
    sh = [BoxShape((0.0, 0.0, 0.0), (0.26, 0.40, 0.14), SHIRT),
          CylShape(_v(neck - c), 0.058, 0.35 - PIVOT[0] + 0.03, TORSO_C, collide=False, axis="X")]
    return Body.build("manikin_torso", pos=_v(c), mass=5.0, fixed=True, color=SHIRT, shapes=sh)


TORSO = _torso()

# -- the spray device --------------------------------------------------------------------------------------------------
# Device frame: origin on the axis at the flange's nozzle-side face; x toward the plunger (the nozzle points -x),
# y along the flange (across), z the third axis. At the start the device frame is the world frame.
NOZZLE_L = 15.0 * MM
FLANGE_T, FLANGE_W, FLANGE_D = 3.0 * MM, 42.0 * MM, 10.0 * MM
BODY_R, BODY_END = 8.0 * MM, 23.0 * MM
DISC_R, DISC_T = 14.0 * MM, 4.0 * MM
DISC_OUT = 32.0 * MM  # the thumb face (x) at rest
STROKE = 5.0 * MM
PRELOAD_N, PLUNGER_K = 15.0, 300.0  # N, N/m
DEV0 = np.array([0.200, -0.120, TOP + 0.020])  # device origin at the start
TIP_D = np.array([-NOZZLE_L, 0.0, 0.0])


def _device_shapes() -> list:
    return [CylShape(_v([-4.5 * MM, 0, 0]), 3.0 * MM, 9.0 * MM, NOZZLE_C, axis="X"),
            CylShape(_v([-12.0 * MM, 0, 0]), 2.25 * MM, 6.0 * MM, NOZZLE_C, axis="X"),
            BoxShape(_v([FLANGE_T / 2, 0, 0]), (FLANGE_T, FLANGE_W - FLANGE_D, FLANGE_D), DEVICE_W),
            CylShape(_v([FLANGE_T / 2, (FLANGE_W - FLANGE_D) / 2, 0]), FLANGE_D / 2, FLANGE_T, DEVICE_W, axis="X"),
            CylShape(_v([FLANGE_T / 2, -(FLANGE_W - FLANGE_D) / 2, 0]), FLANGE_D / 2, FLANGE_T, DEVICE_W, axis="X"),
            CylShape(_v([(FLANGE_T + BODY_END) / 2, 0, 0]), BODY_R, BODY_END - FLANGE_T, DEVICE_W, axis="X")]


DEVICE = Body.build("naloxone", pos=_v(DEV0), mass=0.012, color=DEVICE_W, shapes=_device_shapes(),
                    friction=(0.8, 0.7), solver_iterations=32, contact_offset=0.001)
PLUNGER0 = DEV0 + np.array([DISC_OUT - DISC_T / 2, 0.0, 0.0])  # the plunger body's origin (its disc's middle)
PLUNGER = Body.build("plunger", pos=_v(PLUNGER0), mass=0.003, color=PLUNGER_C,
                     shapes=[CylShape((0.0, 0.0, 0.0), DISC_R, DISC_T, PLUNGER_C, axis="X"),
                             CylShape(_v([-(DISC_T / 2 + 6.0 * MM), 0, 0]), 6.0 * MM, 12.0 * MM, PLUNGER_C,
                                      axis="X")],
                     friction=(0.8, 0.7), solver_iterations=32, contact_offset=0.001,
                     joint=Joint(anchor=_v(PLUNGER0), kind="slide", axis="X", parent="naloxone",
                                 axial=(PLUNGER_K, 15.0, STROKE), slide_range=(-STROKE, 0.0),
                                 slide_target=PRELOAD_N / PLUNGER_K))


def _holder() -> Body:
    """An open holder: a base plate, a block under the device's body and two lips beside it."""
    z0 = TOP
    ax = DEV0[2] - z0  # the axis above the table
    sh = [BoxShape((0.0, 0.0, 0.002), (0.070, 0.050, 0.004), HOLDER_C),
          BoxShape((0.013, 0.0, (0.004 + ax - BODY_R) / 2), (0.014, 0.010, ax - BODY_R - 0.004), HOLDER_C),
          BoxShape((0.013, 0.0095, (0.004 + ax - 3 * MM) / 2), (0.014, 0.003, ax - 3 * MM - 0.004), HOLDER_C),
          BoxShape((0.013, -0.0095, (0.004 + ax - 3 * MM) / 2), (0.014, 0.003, ax - 3 * MM - 0.004), HOLDER_C)]
    return Body.build("holder", pos=(float(DEV0[0]), float(DEV0[1]), z0), mass=0.1, fixed=True, color=HOLDER_C,
                      shapes=sh)


HOLDER = _holder()
OBJECTS = [TORSO, HEAD, HOLDER, DEVICE, PLUNGER]

# -- tracking and scoring -----------------------------------------------------------------------------------------------
FULL_TOL = 0.5 * MM
IN_DEPTH = 3.0 * MM
ANGLE_MAX = 20.0
TILT_MIN = 15.0
SEAT_N = 0.5
HOLD_N = 1.0
WINDOW = 20  # steps (1 s) before the click in which the nose contact counts
_DEBUG = os.environ.get("NALOX_DEBUG") == "1"
_DEBUG_EVERY = int(os.environ.get("NALOX_DEBUG_EVERY", "5"))
_EP: dict = {}
_IDS: dict = {}
_CV: dict = {}


@on_reset
def _reset() -> None:
    _EP.update(tick=0, step=0, dose=None, why="", tilt=0.0, q=0.0, nose_f=[], other_f=0.0, tip_h=None, ang=None,
               where="-", tcp={}, gaps={}, fmax=0.0)
    _CV.clear()


def _contacts(base) -> np.ndarray:
    """Contact forces (N) on the head from: the device, the plunger, the left arm's links, the right arm's links."""
    from isaaclab_physx.physics import PhysxManager
    from yam_arena.robot import asset_link_paths

    sim = PhysxManager.get_physics_sim_view()
    view = _CV.get("view") if _CV.get("sim") is sim else None
    if view is None:
        env = base.scene.env_prim_paths[0]
        links = list(asset_link_paths().values())
        left = [f"{env}/LeftArm{p}" for p in links]
        right = [f"{env}/RightArm{p}" for p in links]
        filt = [f"{env}/naloxone/body", f"{env}/plunger/body"] + left + right
        view = sim.create_rigid_contact_view([f"{env}/manikin_head/body"], filter_patterns=[filt],
                                             max_contact_data_count=256)
        _CV.update(sim=sim, view=view, n=len(links))
    a = view.get_contact_force_matrix(dt=float(base.physics_dt))
    a = a.numpy() if hasattr(a, "numpy") else as_numpy(a)
    f = np.linalg.norm(np.asarray(a, float).reshape(-1, 3), axis=-1)
    n = _CV["n"]
    return np.array([f[0], f[1], f[2:2 + n].sum(), f[2 + n:2 + 2 * n].sum()])


def _pose(base, name, origin):
    d = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return d[:3] - origin, rotation(d[3:7])


@on_step
def _track(env) -> None:
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    _EP["tick"] += 1
    base = getattr(env, "unwrapped", env)
    origin = as_numpy(base.scene.env_origins[0])
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        if side not in _IDS:
            _IDS[side] = list(robot.find_joints(ARM_JOINTS, preserve_order=True)[0])
            _IDS[side + "_f"] = list(robot.find_joints(FINGER_JOINTS, preserve_order=True)[0])
        q = as_numpy(robot.data.joint_pos)[0]
        _EP["tcp"][side] = np.asarray(ik.tcp_pose(q[_IDS[side]], station.BASES[side])[0], float)
        _EP["gaps"][side] = float(-q[_IDS[side + "_f"]].sum())
    hp, hr = _pose(base, "manikin_head", origin)
    dp, dr = _pose(base, "naloxone", origin)
    pp, _ = _pose(base, "plunger", origin)
    rel = hr @ M_HEAD.T  # the head's turn since the start (about y)
    tilt = -math.degrees(math.atan2(rel[0, 2], rel[0, 0]))
    q = float((dr.T @ (pp - dp))[0] - (DISC_OUT - DISC_T / 2))  # plunger travel (m, negative = pressed in)
    f = _contacts(base)
    # which hand holds the device: the one whose TCP is nearer the device
    holder = min(("left", "right"), key=lambda s: float(np.linalg.norm(_EP["tcp"][s] - dp)))
    nose_f = float(f[0] + f[1] + (f[2] if holder == "left" else f[3]))
    other_f = float(f[3] if holder == "left" else f[2])
    _EP["nose_f"].append(nose_f)
    del _EP["nose_f"][:-WINDOW]
    tip_w = dp + dr @ TIP_D
    tip_h = hr.T @ (tip_w - hp)  # head frame
    nd_h = hr.T @ (dr @ np.array([-1.0, 0.0, 0.0]))  # the nozzle's direction, head frame
    where, ang = "outside", None
    for s, name in ((1.0, "left nostril"), (-1.0, "right nostril")):
        loc = R_NOSTRIL.T @ (tip_h - NOSE_O - np.array([0.0, s * NOSTRIL_Y, 0.0]))
        if abs(loc[0]) <= HALF_U + 0.5 * MM and abs(loc[1]) <= HALF_V + 0.5 * MM and loc[2] >= IN_DEPTH:
            where = name
            ang = math.degrees(math.acos(max(-1.0, min(1.0, float(nd_h @ E_W)))))
    _EP.update(tilt=tilt, q=q, other_f=other_f, tip_h=tip_h, ang=ang, where=where, fnow=f, holder=holder,
               fmax=max(_EP["fmax"], nose_f))
    if _EP["dose"] is None and q <= -(STROKE - FULL_TOL):
        seated = max(_EP["nose_f"]) >= SEAT_N
        tilted = tilt >= TILT_MIN
        held = other_f >= HOLD_N
        if where == "outside" or ang is None or ang > ANGLE_MAX:
            _EP["dose"], _EP["why"] = 0.0, f"sprayed outside a nostril (tip {where}, angle {ang})"
        elif seated and tilted and held:
            _EP["dose"], _EP["why"] = 1.0, f"into the {where}, seated, head tilted {tilt:.1f} deg and held"
        else:
            _EP["dose"] = 0.5
            _EP["why"] = (f"into the {where} but seated={seated} (max {max(_EP['nose_f']):.2f} N) "
                          f"tilted={tilted} ({tilt:.1f} deg) held={held} ({other_f:.2f} N)")
        print(f"DOSE tick={_EP['tick']} score={_EP['dose']} {_EP['why']}", flush=True)


def _debug() -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    th = _EP.get("tip_h")
    th = np.zeros(3) if th is None else th
    f = _EP.get("fnow", np.zeros(4))
    print(f"DBG step={_EP['step']} tilt={_EP['tilt']:.1f} q={_EP['q'] * 1000:.2f}mm where={_EP['where']} "
          f"ang={_EP['ang'] if _EP['ang'] is None else round(_EP['ang'], 1)} tip_h=({th[0] * 1000:.1f},"
          f"{th[1] * 1000:.1f},{th[2] * 1000:.1f})mm f(dev,plg,L,R)=({f[0]:.2f},{f[1]:.2f},{f[2]:.2f},{f[3]:.2f}) "
          f"holder={_EP.get('holder')} dose={_EP['dose']} | "
          + " ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{v[2] - TOP:.3f}) gap={_EP['gaps'][k] * 1000:.1f}"
                     for k, v in _EP["tcp"].items()), flush=True)


def score(objects) -> float:
    """The dose's score, fixed at the first full stroke (0 until then)."""
    if _DEBUG:
        _debug()
    return float(_EP.get("dose") or 0.0)


def success(objects) -> bool:
    return _EP.get("dose") == 1.0


# -- scripted demo --------------------------------------------------------------------------------------------------------
# Open loop, from the known layout. Left arm: fingertips (closed) onto the right side of the forehead from above,
# pressing it down and toward the top of the head so the head tilts back about 23 deg; it holds it there until the
# dose is given, then lets go. Right arm: comes down over the device, fingers closing along its axis, one finger on
# the flange beside the nozzle and one on the plunger's disc (both 21.5 mm to the side of the axis, clear of the
# nozzle), squeezes lightly (8 N, under the 15 N that moves the plunger) and lifts it; turns it nozzle-down along the
# tilted nostril's axis, slides the nozzle into the manikin's left nostril until the flange meets the bottom of the
# nose, squeezes harder (about 35 N: the plunger runs to the end of its stroke and clicks), eases off, draws it
# straight out (it is carried high over the face on the way in).
# NALOX_ONE=1: the left arm stays put (nobody holds the head), the right arm does exactly the same.
OPEN_C = -0.047
ONE = os.environ.get("NALOX_ONE") == "1"
TILT_PLAN = float(os.environ.get("NALOX_TILT", "23"))  # deg the left hand tilts the head
GRIP_AT = np.array([16.0, -21.5, 6.0]) * MM  # the TCP in the device frame
HOLD_C = -(DISC_OUT / 2) + float(os.environ.get("NALOX_SQ", "1.6")) * MM  # the light squeeze (about 8 N per finger)
FIRE_C = -(DISC_OUT / 2) + float(os.environ.get("NALOX_FIRE", "7.0")) * MM  # about 35 N per finger, 22 N at the end
PAST = float(os.environ.get("NALOX_PAST", "3")) * MM  # command the insertion this far past the flange's stop
FORE_H = np.array([0.0, -40.0, 42.0]) * MM  # the forehead point the left fingertips press (head frame, its right side)
LEFT_TILT = math.radians(40.0)  # the left hand leans 40 deg: fingers down and toward the robot's right
LEFT_PUSH = float(os.environ.get("NALOX_LPUSH", "20")) * MM  # pressed this far past the surface


def _rot_axis(axis, a: float) -> np.ndarray:
    axis = unit(axis)
    k = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]])
    return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * (k @ k)


def head_at(tilt_deg: float):
    """(nasal root, head rotation) with the head tilted back ``tilt_deg``."""
    r = _rot_y(-math.radians(tilt_deg))
    return PIVOT + r @ (S0 - PIVOT), r @ M_HEAD


def nostril_at(tilt_deg: float, side: float = 1.0):
    """(opening middle, unit axis up the nostril), world, with the head tilted ``tilt_deg``."""
    s, r = head_at(tilt_deg)
    o = s + r @ (NOSE_O + np.array([0.0, side * NOSTRIL_Y, 0.0]))
    return o, r @ E_W


def dev_rot(axis_in) -> np.ndarray:
    """The device's rotation with its nozzle pointing along ``axis_in`` (in the x-z plane), flange along y."""
    x = -unit(axis_in)
    y = np.array([0.0, 1.0, 0.0])
    z = np.cross(x, y)
    return np.column_stack([x, y, z])


def grip_for(dev_pos, rd):
    """TCP position and rotation holding the device at ``dev_pos`` turned ``rd``: closing along its axis, fingers
    pointing along its -z (straight down at the start)."""
    return np.asarray(dev_pos) + rd @ GRIP_AT, rd @ np.diag([1.0, -1.0, -1.0])


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
    return Waypoint(float(t), tuple(float(v) for v in p), finger, label, np.asarray(rot, float))


def _stay(arm: str, t: float = 120.0) -> list:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return [Waypoint(t, tuple(float(v) for v in pos), OPEN_C, "stay put", rot)]


def _dur(path) -> float:
    return float(sum(w.duration for w in path)) if path else 0.0


def _left_rot() -> np.ndarray:
    p = np.array([0.0, -math.sin(LEFT_TILT), -math.cos(LEFT_TILT)])
    h = np.array([-1.0, 0.0, 0.0])
    c = unit(h - (h @ p) * p)
    return np.column_stack([c, np.cross(p, c), p])


def _fore(tilt_deg: float, push: float = 0.0):
    """The left TCP with its fingertips on the forehead point (head tilted ``tilt_deg``), pressed ``push`` in along
    the fingers."""
    from yam_arena.constants import PAD_TIP_PAST_TCP

    s, r = head_at(tilt_deg)
    pt = s + r @ FORE_H
    rot = _left_rot()
    p = rot[:, 2]
    return pt - p * (PAD_TIP_PAST_TCP - push), rot


def _left_path(t_release: float) -> list:
    r = _left_rot()
    above, _ = _fore(0.0, -0.03)
    out = _lead_in("left", above, r, 0.0, "fingers closed, above the forehead", seconds=3.0)
    out.append(_w(1.5, _fore(0.0, 0.0)[0], 0.0, "fingertips onto the forehead", r))
    n = 8
    for k in range(1, n + 1):
        out.append(_w(3.0 / n, _fore(TILT_PLAN * k / n, LEFT_PUSH)[0], 0.0, "press the forehead: tilt the head back",
                      r))
    out.append(_w(max(0.5, t_release - _dur(out)), _fore(TILT_PLAN, LEFT_PUSH)[0], 0.0, "hold the head tilted", r))
    for k in range(1, n + 1):
        out.append(_w(2.0 / n, _fore(TILT_PLAN * (1 - k / n), LEFT_PUSH * (1 - k / n))[0], 0.0,
                      "ease the head back down", r))
    out.append(_w(2.0, _fore(0.0, -0.04)[0], 0.0, "lift the hand clear", r))
    return out


def _right_path() -> tuple[list, float]:
    r0 = np.eye(3)
    g0, gr0 = grip_for(DEV0, r0)
    up = np.array([0.0, 0.0, 1.0])
    out = _lead_in("right", g0 + up * 0.05, gr0, OPEN_C, "above the spray, open", seconds=3.0)
    out += [_w(1.5, g0, OPEN_C, "fingers down beside the flange and the plunger", gr0),
            _w(0.6, g0, OPEN_C, "settle", gr0),
            _w(1.2, g0, HOLD_C, "squeeze lightly: flange and plunger", gr0),
            _w(0.4, g0, HOLD_C, "wait", gr0),
            _w(1.5, g0 + up * 0.06, HOLD_C, "lift it out of the holder", gr0)]
    o, d = nostril_at(TILT_PLAN, 1.0)
    rd = dev_rot(d)
    pre = o - d * (NOZZLE_L + 0.025)  # the device origin with the nozzle's tip 25 mm short of the opening
    g_pre, gr = grip_for(pre, rd)
    back = o - d * (NOZZLE_L + 0.060)  # 60 mm short, straight back along the nostril's axis
    g_back, _ = grip_for(back, rd)
    lift = g0 + up * (0.20 - (g0[2] - TOP))  # straight up, 20 cm over the table
    out.append(_w(2.0, lift, HOLD_C, "lift it well clear", gr0))
    over = np.array([g_back[0], g_back[1] - 0.04, max(g_back[2], TOP + 0.24) + 0.03])
    n = 6
    for k in range(1, n + 1):  # carry it over, high, turning it nozzle-down
        a = k / n
        rk = dev_rot(unit((1 - a) * np.array([-1.0, 0.0, 0.0]) + a * d))
        p = (1 - a) * lift + a * over
        out.append(_w(3.5 / n, p, HOLD_C, "carry it over, high, turning it along the nostril",
                      rk @ np.diag([1, -1, -1.0])))
    out.append(_w(1.5, g_back, HOLD_C, "over the nose, on the nostril's line", gr))
    out.append(_w(1.0, g_back, HOLD_C, "line the nozzle up with the left nostril", gr))
    out.append(_w(1.5, g_pre, HOLD_C, "bring the tip to the opening", gr))
    t_in0 = _dur(out)
    g_in, _ = grip_for(o - d * (-PAST), rd)  # the flange's face PAST beyond the opening plane: pressed on the nose
    out.append(_w(2.5, g_in, HOLD_C, "slide the nozzle in until the flange is against the nose", gr))
    out.append(_w(0.8, g_in, HOLD_C, "settle", gr))
    out.append(_w(1.5, g_in, FIRE_C, "press the plunger fully (click)", gr))
    out.append(_w(1.0, g_in, FIRE_C, "hold", gr))
    out.append(_w(0.8, g_in, HOLD_C, "ease off the plunger", gr))
    t_out = _dur(out)
    out.append(_w(2.0, g_pre, HOLD_C, "draw the nozzle straight out", gr))
    out.append(_w(2.0, g_pre + up * 0.04, HOLD_C, "hold it clear", gr))
    return out, t_out


def demo():
    right, t_out = _right_path()
    if ONE:
        left = _stay("left", _dur(right) + 6.0)
    else:
        left = _left_path(t_out + 0.5)
    tend = max(_dur(left), _dur(right)) + 3.0
    for path in (left, right):
        last = path[-1]
        if tend > _dur(path):
            path.append(Waypoint(tend - _dur(path), last.pos, last.finger, "stay", last.yaw))
    return {"left": left, "right": right}
