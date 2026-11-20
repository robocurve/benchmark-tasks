"""On the station table: a hinged wire egg slicer (die-cast aluminium, 140 x 80 x 30 mm, ten stainless-steel cutting
wires on a frame hinged at one end, like the Westmark "Justus" egg slicer) with its frame swung open, and a peeled
hard-boiled egg (US large, 57 x 43 mm, 50 g) lying on a saucer. Task (instruction): slice the egg in the egg slicer,
then swing the slicer's frame back open, leaving the slices in the slicer.

How it is done (the slicer's own use: it "cuts the hard-boiled egg quickly and cleanly into round or oval slices";
for round slices the egg lies lengthwise in the egg-shaped hollow of the cradle): the egg is laid in the hollow, the
wire frame is swung over and pressed down through it in one stroke until it closes on the cradle, and the frame is
swung open again to free the slices.

The hand skill: the egg is slick and rounded; it lies across the robot's reach on the saucer and must be turned a
quarter turn and set lengthwise and centred in a hollow only 6 mm deep; the frame is taken by its small tab and swung
over through 95 deg along its arc, then driven through the egg against the wires' cutting resistance (about 9 N at the
tab) until it closes, then swung back past upright onto its stop without knocking the cut slices out.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- slicer cradle: fixed (stands on a non-slip mat). 80 mm (x) by 140 mm (y), 22 mm tall, centred at (0.21, 0.04);
  an egg-shaped hollow (an egg's lower part, 1 mm clearance, 6 mm deep) at its centre, its long axis along y.
- frame: on a hinge across the cradle's +y end (axis along x, 24.5 mm above the table, at y 0.104); bars 4 mm thick
  round an opening 84 mm wide, 126 mm long; a tab 30 mm wide, 32 mm long, 3 mm thick bent up at its free end.
  10 wires across it, 5.5 mm apart, at y 0.04 + (-24.75, -19.25 ... 24.75) mm when it is closed, where they lie in
  the cradle's slots 1 mm below the hollow's floor. It starts swung open 95 deg (just past upright, resting on its
  stop). 40 g; the hinge turns freely (0.03 N m s/rad of drag).
- egg: 57 mm long, 43 mm across, 50 g; modelled as 11 slices (cut at the wires' positions when it lies centred in the
  hollow), welded together until the wires cut them. It starts lying along x on the saucer, its centre at
  (0.13, -0.10).
- saucer: 10 cm across, 6 mm thick, at (0.13, -0.10).

Cutting rule (a stand-in for the wires cutting the egg; stated in the fidelity note): the wires do not collide with
the egg. Once the frame has come down so far that a wire over the egg reaches the egg's top, the hinge resists
closing with 1.2 N m (about 9 N at the tab, 19 N on the wires at the egg's centre: our estimate of the force the wires
need to cut a hard-boiled egg); it holds the frame where it is when the hand lets go. When the frame closes (within
1.5 deg of the cradle), the egg is cut: its ten welds become the light cling of moist cut faces (they part at 0.3 N
or 0.8 mN m). If the egg was not lying in the hollow when a wire first reached it (its long axis more than 20 deg off
the hollow's, its centre more than 6 mm off the hollow's centre, or more than 4 mm above its seat), the wires crush
it instead: the score is 0 for the rest of the trial.

Score (best stage reached; 0 once the egg is crushed):
- 0.25: the egg has lain in the hollow (lengthwise and centred, as above) at some time.
- 0.5: the egg has been cut into its 11 slices (the frame closed over it lying in the hollow).
- 1.0 (success, held 1 s): as 0.5, the frame swung back open (85 deg or more), all 11 slices in the cradle (centre
  over the cradle's 8 x 14 cm top, at most 4.5 cm above the table), every finger point 1 cm or more from every slice,
  all still (under 2 cm/s).
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, Joint, Link, Linkage, MeshShape, _quat_from_matrix  # noqa: E402
from bodykit import allow_matrix_grips, as_numpy, on_reset, on_step, rotation  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
_E = os.environ.get

# -- the layout (world, m) -------------------------------------------------------------------------------------------
CX, CY = 0.21, 0.04  # the cradle's centre = the hollow's centre
BASE_W, BASE_L, BASE_H = 0.080, 0.140, 0.022  # along x, along y, height of its top
FLOOR = 0.016  # the hollow's floor above the table (6 mm deep)
EGG_A, EGG_B = 0.0285, 0.0215  # the egg's half length and half width
EGG_M = 0.050
CLEAR_EGG = 0.001  # the hollow's clearance round the egg
SEAT_Z = TOP + FLOOR + EGG_B  # the egg's centre when it lies in the hollow
N_WIRES = 10
PITCH = 0.0055
WIRES = [(k - (N_WIRES - 1) / 2) * PITCH for k in range(N_WIRES)]  # along the hollow's axis, from its centre
H = np.array([CX, CY + 0.064, TOP + 0.0245])  # the hinge axis (along x)
OPEN_DEG = 95.0  # the frame starts (and rests) swung open this far
WIRE_DROP = (FLOOR - 0.001) - (H[2] - TOP)  # the wires below the frame's bars (closed: 1 mm under the floor)
TAU_CUT = float(_E("EGG_TAU", "1.2"))  # N m: the hinge's resistance while the wires cut
K_CUT, D_CUT = 40.0, 0.3  # the holding drive (N m/rad, N m s/rad), capped at TAU_CUT
CLOSED_DEG = 1.5
HINGE_DAMP = 0.03  # N m s/rad: the hinge's own drag
SAUCER = np.array([0.13, -0.10])
SAUCER_R, SAUCER_T = 0.050, 0.006
IN_CRADLE_Z = 0.045  # a slice "in the cradle": centre over the cradle, at most 4.5 cm above the table
OPEN_DONE = 85.0  # deg: the frame swung back open

ALU, DARK, STEEL, WHITE, YOLK, CHINA, MAT = ((0.72, 0.73, 0.75), (0.25, 0.25, 0.27), (0.82, 0.83, 0.85),
                                             (0.96, 0.95, 0.90), (0.98, 0.78, 0.18), (0.93, 0.94, 0.96),
                                             (0.30, 0.45, 0.35))


def _v(p) -> tuple:
    return tuple(float(x) for x in p)


def _Rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _frame_rot(theta_deg: float) -> np.ndarray:
    """The frame's rotation from its closed pose when it is swung open ``theta_deg``."""
    return _Rx(-math.radians(theta_deg))


# -- the cradle --------------------------------------------------------------------------------------------------
def _surface(x: float, y: float) -> float:
    a, b = EGG_A + CLEAR_EGG, EGG_B + CLEAR_EGG
    u = 1.0 - (x / b) ** 2 - (y / a) ** 2
    if u <= 0.0:
        return BASE_H
    return min(BASE_H, FLOOR + b - b * math.sqrt(u))


def _cradle() -> Body:
    bot = FLOOR - 0.0015
    xs = np.linspace(-BASE_W / 2, BASE_W / 2, 41)
    ys = np.linspace(-BASE_L / 2, BASE_L / 2, 71)
    pts = np.array([[x, y, _surface(x, y)] for y in ys for x in xs])
    nx = len(xs)
    faces = []
    for j in range(len(ys) - 1):
        for i in range(nx - 1):
            a, b, c, d = j * nx + i, j * nx + i + 1, (j + 1) * nx + i + 1, (j + 1) * nx + i
            faces += [[a, b, c], [a, c, d]]
    sh = [BoxShape((0.0, 0.0, bot / 2), (BASE_W, BASE_L, bot), ALU),
          MeshShape(pts, np.array(faces, dtype=np.int32), None, ALU, True, "none")]
    for sx in (1, -1):  # skirts round the top (drawn only)
        sh.append(BoxShape((sx * (BASE_W / 2 - 0.001), 0.0, (bot + BASE_H) / 2), (0.002, BASE_L, BASE_H - bot), ALU,
                           collide=False))
    for sy in (1, -1):
        sh.append(BoxShape((0.0, sy * (BASE_L / 2 - 0.001), (bot + BASE_H) / 2), (BASE_W, 0.002, BASE_H - bot), ALU,
                           collide=False))
    for w in WIRES:  # the wires' slots in the flat of the bed either side of the hollow
        for sx in (1, -1):
            sh.append(BoxShape((sx * 0.029, w, BASE_H + 0.0002), (0.020, 0.0012, 0.0004), DARK, collide=False))
    for sx in (1, -1):  # the hinge lugs
        sh.append(BoxShape((sx * 0.037, H[1] - CY, (H[2] - TOP) / 2 + 0.002), (0.006, 0.010, H[2] - TOP + 0.004),
                           ALU, collide=False))
    sh.append(BoxShape((0.0, 0.0, -0.0006), (BASE_W + 0.03, BASE_L + 0.03, 0.0012), MAT, collide=False))  # the mat
    return Body.build("cradle", pos=(CX, CY, TOP + 0.0001), mass=0.2, color=ALU, fixed=True, friction=(0.5, 0.4),
                      shapes=sh)


# -- the wire frame ----------------------------------------------------------------------------------------------
def _frame() -> Linkage:
    """Shapes authored closed (relative to the hinge), then turned to the open start pose."""
    r_open = _frame_rot(OPEN_DEG)
    q_open = _quat_from_matrix(r_open)

    def box(c, size, color, collide=True):
        return BoxShape(_v(r_open @ np.asarray(c, float)), size, color, collide=collide, quat_xyzw=q_open)

    tip = -0.130
    shapes = [box((sx * 0.044, (tip + -0.004) / 2 + 0.000, 0.0), (0.004, 0.126, 0.004), ALU) for sx in (1, -1)]
    shapes += [box((0.0, tip, 0.0), (0.092, 0.004, 0.004), ALU),
               box((0.0, -0.006, 0.0), (0.092, 0.004, 0.004), ALU),
               box((0.0, tip - 0.0035, 0.014), (0.030, 0.003, 0.032), ALU)]  # the tab, bent up
    for sx in (1, -1):  # the hangers that carry the wires below the bars (drawn only)
        shapes.append(box((sx * 0.044, -0.064, WIRE_DROP / 2), (0.002, 0.056, -WIRE_DROP), ALU, collide=False))
    for w in WIRES:
        shapes.append(box((0.0, w - 0.064, WIRE_DROP), (0.088, 0.0005, 0.0005), STEEL, collide=False))
    links = [Link("frame", tuple(shapes), mass=0.040, anchor=_v(H), axis="X", limits=(0.0, OPEN_DEG),
                  damping=HINGE_DAMP, stiffness=K_CUT, spring_damping=D_CUT, effort_limit=TAU_CUT)]
    knuckle = [BoxShape((0.0, 0.0, 0.0), (0.070, 0.006, 0.006), ALU, collide=False)]
    return Linkage.build("frame", pos=_v(H), mass=0.02, color=ALU, floating=False, friction=(0.5, 0.4),
                         shapes=knuckle, links=links, contact_offset=0.0015, solver_iterations=32)


# -- the egg: 11 slices welded together ----------------------------------------------------------------------------
BOUNDS = [-EGG_A] + WIRES + [EGG_A]
YOLK_R = 0.0150
CLING_F, CLING_T = float(_E("EGG_CLINGF", "0.3")), float(_E("EGG_CLINGT", "0.0008"))  # N, N m: cut faces' cling
GAP = 0.00015  # each cut face set back 0.15 mm, so the slices do not overlap when their welds go


def _ring_r(y: float) -> float:
    return EGG_B * math.sqrt(max(0.0, 1.0 - (y / EGG_A) ** 2))


def _slice_mesh(y1: float, y2: float) -> MeshShape:
    from scipy.spatial import ConvexHull

    n = max(2, int(math.ceil((y2 - y1) / 0.0012)) + 1)
    pts = []
    for y in np.linspace(y1, y2, n):
        r = _ring_r(y)
        if r < 1e-4:
            pts.append([0.0, y, 0.0])
            continue
        for a in np.linspace(0.0, 2 * math.pi, 28, endpoint=False):
            pts.append([r * math.cos(a), y, r * math.sin(a)])
    pts = np.array(pts)
    hull = ConvexHull(pts)
    c = pts.mean(0)
    faces = []
    for s in hull.simplices:
        p0, p1, p2 = pts[s]
        if np.dot(np.cross(p1 - p0, p2 - p0), p0 - c) < 0:
            s = s[[0, 2, 1]]
        faces.append(s)
    used = np.unique(np.array(faces))
    remap = -np.ones(len(pts), int)
    remap[used] = np.arange(len(used))
    p = pts[used]
    nrm = np.array([[q[0] / EGG_B ** 2, q[1] / EGG_A ** 2, q[2] / EGG_B ** 2] for q in p])
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True)
    return MeshShape(p, remap[np.array(faces)].astype(np.int32), nrm, WHITE, True, "convexHull")


def _slice_volume(y1: float, y2: float) -> float:
    f = lambda y: y - y ** 3 / (3 * EGG_A ** 2)  # noqa: E731
    return math.pi * EGG_B ** 2 * (f(y2) - f(y1))


EGG_C0 = np.array([SAUCER[0], SAUCER[1], TOP + SAUCER_T + EGG_B + 0.0015])  # the egg's start centre
EGG_YAW0 = math.pi / 2  # its axis (body y) along world -x at the start
_RY0 = rotation((0.0, 0.0, math.sin(EGG_YAW0 / 2), math.cos(EGG_YAW0 / 2)))


def _slices() -> list:
    m = [EGG_M * _slice_volume(BOUNDS[k], BOUNDS[k + 1]) / _slice_volume(-EGG_A, EGG_A) for k in range(len(BOUNDS) - 1)]
    out = []
    for k in range(len(BOUNDS) - 1):
        y1, y2 = BOUNDS[k], BOUNDS[k + 1]
        g1 = GAP if k > 0 else 0.0
        g2 = GAP if k < len(BOUNDS) - 2 else 0.0
        sh = [_slice_mesh(y1 + g1, y2 - g2)]
        for yf, sgn in ((y1, -1), (y2, 1)):  # the yolk, seen on a cut face
            if abs(yf) < YOLK_R and abs(yf) < EGG_A - 0.001:
                r = math.sqrt(YOLK_R ** 2 - yf ** 2) * 0.95
                sh.append(CylShape((0.0, yf + sgn * 0.00005, 0.0), r, 0.0003, YOLK, collide=False, axis="Y"))
        joint = None
        if k > 0:
            anchor = EGG_C0 + _RY0 @ np.array([0.0, y1, 0.0])
            joint = Joint(anchor=_v(anchor), kind="fixed", parent=f"slice{k - 1}", break_force=400.0,
                          break_torque=20.0)
        out.append(Body.build(f"slice{k}", pos=_v(EGG_C0), mass=m[k], yaw=EGG_YAW0,
                              shapes=sh, color=WHITE, friction=(0.8, 0.7), joint=joint, contact_offset=0.001,
                              solver_iterations=32))
    return out


def _dish(name, xy, r, t, color, mass) -> Body:
    return Body.build(name, pos=(float(xy[0]), float(xy[1]), TOP + 0.0005), mass=mass, color=color,
                      friction=(0.6, 0.5), shapes=[CylShape((0.0, 0.0, t / 2), r, t, color),
                                                   CylShape((0.0, 0.0, t + 0.0001), r - 0.012, 0.0002,
                                                            (0.88, 0.89, 0.92), collide=False)])


CRADLE = _cradle()
FRAME = _frame()
SLICES = _slices()
SAUCER_BODY = _dish("saucer", SAUCER, SAUCER_R, SAUCER_T, CHINA, 0.15)
OBJECTS = [CRADLE, FRAME, SAUCER_BODY] + SLICES
N_SLICES = len(SLICES)
SLICE_MID = [(BOUNDS[k] + BOUNDS[k + 1]) / 2 for k in range(N_SLICES)]

# -- tracking and scoring --------------------------------------------------------------------------------------------
CLEAR = 0.010
STILL = 0.02
_DEBUG = _E("EGG_DEBUG") == "1"
_DEBUG_EVERY = int(_E("EGG_DEBUG_EVERY", "5"))
_EP: dict = {}
_IDS: dict = {}


@on_reset
def _reset() -> None:
    _EP.update(tick=0, step=0, theta=OPEN_DEG, engaged=False, ever_engaged=False, crushed=False, cut=False,
               q_hold=0.0, drive_on=None, seated=False, was_seated=False, n_in=0, near=1.0, still=False, cut_step=None,
               centre=EGG_C0.copy(), axis_dev=0.0, tcp={}, gaps={}, cuts_written=False, mids=None)


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


def _cut_welds() -> None:
    """The wires have gone through: the ten welds become the light cling of moist cut faces (they part at 0.3 N or
    0.8 mN m)."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    stage = get_current_stage()
    names = {f"/slice{k}/joint" for k in range(1, N_SLICES)}
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        if any(p.endswith(n) for n in names):
            j = UsdPhysics.Joint(prim)
            j.GetBreakForceAttr().Set(CLING_F)
            j.GetBreakTorqueAttr().Set(CLING_T)


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("EGG_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _wire_world(theta: float, w: float) -> np.ndarray:
    return H + _frame_rot(theta) @ np.array([0.0, w - 0.064, WIRE_DROP])


def _track_inner(env) -> None:
    import torch

    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    _EP["tick"] += 1
    base = getattr(env, "unwrapped", env)
    origin = as_numpy(base.scene.env_origins[0])
    if "fingers" not in _IDS:
        _IDS["fingers"] = _finger_points()
    fingers = []
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
        fingers.append(np.concatenate([_IDS["fingers"][n] @ rotation(poses[i][3:7]).T + poses[i][:3] - origin
                                       for n, i in zip(("tip_left", "tip_right"), _IDS[side + "_tips"])]))
    fingers = np.concatenate(fingers)
    # the slices
    mids, speeds = [], []
    r_mid = None
    for k in range(N_SLICES):
        obj = base.scene[f"slice{k}"]
        pose = as_numpy(obj.data.root_link_pose_w)[0]
        r = rotation(pose[3:7])
        mids.append(pose[:3] - origin + r @ np.array([0.0, SLICE_MID[k], 0.0]))
        speeds.append(float(np.linalg.norm(as_numpy(obj.data.root_com_vel_w)[0][:3])))
        if k == N_SLICES // 2:
            r_mid, c_mid = r, pose[:3] - origin
    mids = np.array(mids)
    axis = r_mid[:, 1]
    axis_dev = math.degrees(math.acos(min(1.0, abs(float(axis[1])))))
    centre = c_mid
    # the frame
    fr = base.scene["frame"]
    if "frame" not in _IDS:
        _IDS["frame"] = list(fr.joint_names).index("frame_turn")
    qf = float(as_numpy(fr.data.joint_pos)[0][_IDS["frame"]])
    theta = OPEN_DEG - math.degrees(qf)
    in_cradle = bool(abs(centre[0] - CX) <= BASE_W / 2 and abs(centre[1] - CY) <= BASE_L / 2
                     and centre[2] <= TOP + 0.08)
    seated = bool(axis_dev <= 20.0 and math.hypot(centre[0] - CX, centre[1] - CY) <= 0.006
                  and centre[2] <= SEAT_Z + 0.004)
    engaged = False
    if not _EP["cut"] and in_cradle:
        for w in WIRES:  # a wire over the egg's length, down at the egg's top there
            p = _wire_world(theta, w)
            dy = abs(p[1] - centre[1])
            if dy < EGG_A and p[2] <= centre[2] + _ring_r(dy) + 0.001:
                engaged = True
                break
    if engaged and not _EP["ever_engaged"]:
        _EP["ever_engaged"] = True
        if not seated:
            _EP["crushed"] = True
    if not engaged and not _EP["cut"]:
        _EP["ever_engaged"] = False if not in_cradle else _EP["ever_engaged"]
    if engaged:
        _EP["q_hold"] = max(_EP["q_hold"], qf)
        if theta <= CLOSED_DEG:
            _EP["cut"] = True
            _EP["cut_step"] = _EP["tick"]
            engaged = False
            _cut_welds()
    else:
        _EP["q_hold"] = qf
    dev = fr.device
    on = bool(engaged)
    if on != _EP["drive_on"]:
        fr.write_joint_effort_limit_to_sim(torch.tensor([[TAU_CUT if on else 0.0]], dtype=torch.float32, device=dev),
                                           joint_ids=[_IDS["frame"]])
        _EP["drive_on"] = on
    fr.set_joint_position_target(torch.tensor([[_EP["q_hold"]]], dtype=torch.float32, device=dev),
                                 joint_ids=[_IDS["frame"]])
    fr.write_data_to_sim()
    # the slices left in the cradle
    in_cradle = [bool(abs(m[0] - CX) <= BASE_W / 2 and abs(m[1] - CY) <= BASE_L / 2 and m[2] <= TOP + IN_CRADLE_Z)
                 for m in mids]
    d = np.linalg.norm(fingers[:, None, :] - mids[None, :, :], axis=2)
    near = float(d.min()) - EGG_B
    if _DEBUG and _EP["cut"] and (_EP["tick"] - _EP["cut_step"]) % 40 == 1:
        print("DBG slices " + " ".join(f"({(m[0] - CX) * 1000:.0f},{(m[1] - CY) * 1000:.0f},{(m[2] - TOP) * 1000:.0f})"
                                       for m in mids), flush=True)
    _EP.update(theta=theta, engaged=engaged, seated=seated, centre=centre, axis_dev=axis_dev, n_in=sum(in_cradle),
               was_seated=_EP["was_seated"] or seated,
               near=near, still=bool(max(speeds) < STILL), mids=mids, vmax=max(speeds))


def _tally(objects) -> dict:
    done = bool(_EP["cut"] and not _EP["crushed"] and _EP["theta"] >= OPEN_DONE and _EP["n_in"] == N_SLICES
                and _EP["near"] >= CLEAR and _EP["still"])
    return dict(done=done)


def _debug(t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    c = _EP["centre"]
    spread = 0.0 if _EP["mids"] is None else float(np.ptp(_EP["mids"][:, :2], axis=0).max())
    print(f"DBG step={_EP['step']} theta={_EP['theta']:.1f} eng={int(_EP['engaged'])} seated={int(_EP['seated'])} "
          f"crushed={int(_EP['crushed'])} cut={int(_EP['cut'])} egg=({c[0]:.3f},{c[1]:.3f},{(c[2] - TOP) * 1000:.1f}mm)"
          f" axis={_EP['axis_dev']:.0f}deg in_cradle={_EP['n_in']} spread={spread * 1000:.0f}mm "
          f"near={_EP['near'] * 1000:.0f}mm vmax={_EP.get('vmax', 0.0) * 100:.1f}cm/s still={int(_EP['still'])} done={int(t['done'])} | "
          + " ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{(v[2] - TOP) * 1000:.1f}mm) gap={_EP['gaps'][k] * 1000:.0f}"
                     for k, v in _EP["tcp"].items()), flush=True)


def score(objects) -> float:
    t = _tally(objects)
    if _DEBUG:
        _debug(t)
    if _EP["crushed"]:
        return 0.0
    if t["done"]:
        return 1.0
    if _EP["cut"]:
        return 0.5
    return 0.25 if _EP["was_seated"] else 0.0


def success(objects) -> bool:
    return bool(_tally(objects)["done"])


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Open loop, from the known layout. The right gripper (straight down, fingers across the egg) picks the egg up off the
# saucer, turns it a quarter turn on the way so it lies lengthwise and sets it in the cradle's hollow, lets go and goes
# back to its ready pose. The left gripper, straight down with its fingers closing along x, takes the frame's tab by
# its two side edges with the fingertips and swings the frame over along its arc (the tab turns between the
# fingertips) onto the egg, then on to 8 deg past closed so the arm presses the wires through (the hinge resists with
# 1.2 N m from the egg's top down); it holds it closed 1.5 s, swings it back to 98 deg (past upright: the frame stops
# at 95 deg on its stop), lets go and goes back.
# EGG_CROSS=1 (the wrong way): the egg is set in without turning it, lying across the hollow; it cannot sink into
# the hollow, and the wires crush it (score 0).
OPEN_C = -0.047
CROSS = _E("EGG_CROSS") == "1"
P = {k: float(_E("EGG_" + k.upper(), v)) for k, v in dict(
    egg_grip="-0.017", push_past="8.0", tab_grip="-0.008", spread="-0.036").items()}


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


DOWN_Y = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])  # straight down, fingers closing along y
DOWN_X = _rz(math.pi / 2) @ DOWN_Y  # straight down, fingers closing along x


def _good_ik(arm: str, pos, rot):
    from yam_arena import ik, station

    best = None
    b = station.BASES[arm]
    j1 = math.atan2(float(pos[1]) - b[1], float(pos[0]) - b[0])
    for sd in itertools.product((j1,), (0.8, 1.3, 1.8), (0.6, 1.2), (-0.8, 0.8), (0.0,), (-1.8, -0.9, 0.0, 0.9, 1.8)):
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


def _w(t, p, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in p), finger, label, rot)


def _ready(arm: str):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), station.BASES[arm])


def _dur(wps) -> float:
    return float(sum(w.duration for w in wps))


def _place_egg() -> list:
    """Right arm: the egg from the saucer into the hollow, turned lengthwise (or not, the wrong way)."""
    pick = np.array([SAUCER[0], SAUCER[1], EGG_C0[2] - 0.001])
    hi = 0.10 + TOP
    put_rot = DOWN_Y if CROSS else DOWN_X
    seat = np.array([CX, CY, SEAT_Z + 0.002 + (0.004 if CROSS else 0.0)])
    out = _lead_in("right", [pick[0], pick[1], hi], DOWN_Y, OPEN_C, "over the egg, fingers open")
    out += [_w(2.0, pick, OPEN_C, "down round the egg", DOWN_Y),
            _w(1.2, pick, P["egg_grip"], "pinch the egg across its middle", DOWN_Y),
            _w(0.5, pick, P["egg_grip"], "settle", DOWN_Y),
            _w(1.5, [pick[0], pick[1], hi], P["egg_grip"], "lift", DOWN_Y)]
    n = 8
    for k in range(1, n + 1):
        f = k / n
        a = 0.0 if CROSS else (math.pi / 2) * f
        p = np.array([pick[0], pick[1], hi]) * (1 - f) + np.array([CX, CY, hi]) * f
        out.append(_w(2.4 / n, p, P["egg_grip"], "carry it over, turning it lengthwise", _rz(a) @ DOWN_Y))
    out += [_w(2.0, seat, P["egg_grip"], "lower it into the hollow", put_rot),
            _w(0.5, seat, P["egg_grip"], "settle", put_rot),
            _w(1.2, seat, P["spread"], "let go", put_rot),
            _w(1.5, [CX, CY, hi + 0.02], P["spread"], "rise", put_rot)]
    out += _lead_out("right", [CX, CY, hi + 0.02], put_rot, OPEN_C, "back to the ready pose")
    return out


DOWN_X_L = _rz(-math.pi / 2) @ DOWN_Y  # the left arm's straight-down grip, fingers closing along x
TIP_AHEAD = 0.0128  # the closed fingertips' end, ahead of the TCP along the approach axis
PINCH = np.array([0.0, -0.1335, 0.020])  # where the fingertips take the tab (closed frame, from the hinge)


def _tab_pose(theta: float, rise: float = 0.0):
    """The left TCP, straight down, its fingertips' ends 6 mm past the pinch point on the tab's two side edges."""
    p = H + _frame_rot(theta) @ PINCH + np.array([0.0, 0.0, TIP_AHEAD - 0.006 + rise])
    return p, DOWN_X_L


def _swing(a: float, b: float, finger: float, seconds: float, label: str) -> list:
    n = max(3, int(abs(b - a) / 4.0))
    out = []
    for k in range(1, n + 1):
        p, r = _tab_pose(a + (b - a) * k / n)
        out.append(_w(seconds / n, p, finger, label, r))
    return out


def _work_frame() -> list:
    """Left arm: take the tab by its two side edges with the fingertips, swing the frame over (the tab turns between
    the fingertips) and press it down through the egg, then swing it back open onto its stop and let go."""
    p_hi, r = _tab_pose(OPEN_DEG, rise=0.04)
    p_tab, _ = _tab_pose(OPEN_DEG)
    out = _lead_in("left", p_hi, r, OPEN_C, "over the frame's tab, fingers open")
    out += [_w(1.5, p_tab, OPEN_C, "fingertips either side of the tab", r),
            _w(1.0, p_tab, P["tab_grip"], "pinch the tab's edges", r),
            _w(0.5, p_tab, P["tab_grip"], "settle", r)]
    out += _swing(OPEN_DEG, 45.0, P["tab_grip"], 4.0, "swing the frame over onto the egg")
    out += _swing(45.0, -P["push_past"], P["tab_grip"], 3.5, "press the wires down through the egg")
    p_low, _ = _tab_pose(-P["push_past"])
    out += [_w(1.5, p_low, P["tab_grip"], "hold it closed", r)]
    out += _swing(-P["push_past"], OPEN_DEG + 3.0, P["tab_grip"], 5.0, "swing the frame back open onto its stop")
    p_end, _ = _tab_pose(OPEN_DEG + 3.0)
    out += [_w(0.5, p_end, P["tab_grip"], "settle", r),
            _w(1.0, p_end, OPEN_C, "let go", r),
            _w(1.2, p_end + [0.0, 0.0, 0.04], OPEN_C, "rise", r)]
    out += _lead_out("left", p_end + [0.0, 0.0, 0.04], r, OPEN_C, "back to the ready pose")
    return out


def demo():
    rp, rq = _ready("right")
    lp, lq = _ready("left")
    a = _place_egg()
    b = _work_frame()
    right = a + [_w(_dur(b) + 3.0, rp, OPEN_C, "stay at the ready pose", rq)]
    left = [_w(_dur(a), lp, OPEN_C, "wait at the ready pose", lq)] + b + [_w(3.0, lp, OPEN_C, "stay clear", lq)]
    return {"left": left, "right": right}
