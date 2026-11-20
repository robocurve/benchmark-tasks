"""On the station table: a Brannock foot-measuring device (men's, metal, about 35 x 13 cm) lies loose on the table,
its long axis along x, the right heel cup at the far end, toes toward the robot. A rigid true-size foot-and-ankle
model (a hollow plastic display foot, men's US 9: 26.2 cm long, 10 cm wide at the ball, 14 cm tall at the ankle,
0.30 kg) stands on the device's plate, set down 25 mm too far forward: its heel is not in the heel cup. The width bar
is at its widest and the arch pointer slid back, as the maker says to prepare the device. Task (instruction): measure
the foot with the device: heel seated in the heel cup, width bar against the foot, arch pointer at the ball joint,
then let go of everything. The instruction does not say how.

How it is done (Brannock Device Co., "Instructions for use", brannock.com): "Be sure the heel is properly located
against the back of the heel cup, by grasping the customer's ankle and device together"; "Slide the pointer (A on
diagram) forward so the inside curve of the pointer fits the ball joint of the foot"; "Slide the width bar firmly to
the edge of the foot."

The hand skill: light contact on curved anatomy. The foot is drawn back by its ankle until its heel sits against the
back of the heel cup; then the two small sliders are slid along their tracks until each just touches the foot (the
arch pointer's rib against the side of the ball joint, the width bar against the outer edge of the foot at its
widest) without shoving the foot out of place. The 0.30 kg model only rests on the plate: an unheld foot turns about
the heel cup when the arch pointer pushes hard on its ball joint (measured here: the rib commanded 16 mm past first
touch rides 13 mm past the ball joint, about two arch sizes, unless the ankle is held); the heel cup keeps it from
turning much under the width bar (4.4 N: under 2 deg). The demo steadies the ankle with one hand as the maker says;
with gentle pushes one hand alone also succeeds (two arms are not required).

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
Device frame: u along the device from the right heel cup toward the toes (world direction -x), v across it toward the
foot's outer (lateral) side (world +y); origin O at the back of the right heel cup on the plate's top, at world
(0.340, 0.000), plate top 10 mm above the table.
- device (loose, 0.80 kg base): plate u -0.008..0.338, v -0.065..0.065, 10 mm thick; right heel cup: a 25 mm wall
  round the heel, inner radius 35 mm about (u 0.035, v 0); left heel cup at the far end (u 0.330), facing back.
  Width bar (0.05 kg): a bar along u (0.135..0.235) on the lateral side, its inner face at v 0.058 (widest), slides
  across the device (toward -v) on its track with 0.5 N friction; a finger tab on its outer face at u 0.225. Arch
  pointer (0.02 kg): a slider on the medial edge with a rib (8 mm, innermost at v -0.044) and a thumb tab 70 mm ahead
  of the rib, slides along u with 0.4 N friction, starts with the rib at u 0.140.
- foot (loose, 0.30 kg): heel back at u 0.025 (25 mm forward of the cup's back), on the device's axis. The ball joint
  (first metatarsal head, medial bulge) at u 0.190 from the heel back, medial edge v -0.049; the fifth metatarsal
  head (lateral bulge, the widest point) at u 0.170, v +0.051. Ankle: a 66 mm cylinder rising from u 0.050.

Score (taken at the end of the run; nothing is latched):
- heel seated (0.3): the heel's back within 3 mm of the cup's back, the heel centred within 3 mm, the foot turned
  less than 3 deg on the device and flat (tilt under 3 deg).
- width bar touching (+0.3, only with the heel seated): contact force between the width bar and the foot 0.05-5 N
  (mean over the last 0.5 s).
- arch pointer at the ball joint (+0.3, only with the heel seated): contact force between the pointer and the foot
  0.05-5 N (mean over the last 0.5 s) and the pointer's rib within 5 mm of where it meets the ball joint of the seated
  foot (u 0.1768, the arch reading).
- 1.0 (success, held 1 s): all three, every finger point 2 cm or more from the foot and the sliders, and the foot and
  both sliders still (under 1 mm in 1 s).
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, Link, Linkage, SphereShape, as_numpy, capsule  # noqa: E402
from bodykit import allow_matrix_grips, on_reset, on_step, rotation  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
MM = 0.001

# -- the layout ----------------------------------------------------------------------------------------------------
PSI = math.radians(float(os.environ.get("BRANNOCK_PSI", "180")))  # the device's u axis (deg from world +x)
D = np.array([math.cos(PSI), math.sin(PSI), 0.0])  # u: heel -> toes
R = np.array([math.sin(PSI), -math.cos(PSI), 0.0])  # v: toward the foot's outer side (right of u)
PLATE_T = 0.010
O = np.array([float(os.environ.get("BRANNOCK_OX", "0.340")), float(os.environ.get("BRANNOCK_OY", "0.0")),
              TOP + PLATE_T])  # back of the right heel cup, on the plate's top
START_OFF = float(os.environ.get("BRANNOCK_START", "25")) * MM  # the heel this far forward of the cup's back

CUP_C, CUP_R, CUP_T, CUP_H = 0.035, 0.035, 0.005, 0.025  # heel cup: centre u, inner radius, wall, height
FAR_CUP_C = 0.330 - 0.035
BAR_U0, BAR_U1, BAR_V0, BAR_T, BAR_H = 0.135, 0.235, 0.058, 0.010, 0.020  # width bar: along u, inner face, size
BAR_TRAVEL = 0.030
BAR_F = float(os.environ.get("BRANNOCK_BAR_F", "0.5"))  # N: the width bar's track friction
PTR_U0 = float(os.environ.get("BRANNOCK_PTR_U0", "0.140"))  # the rib's u at the start (slid back)
RIB_V, RIB_R = -0.048, 0.004  # rib (vertical cylinder) centre v, radius: innermost v -0.044
PTR_TAB_DU = 0.070  # the pointer's thumb tab this far ahead of its rib (u)
BAR_TAB_U = 0.225  # the width bar's finger tab (u)
PTR_F = float(os.environ.get("BRANNOCK_PTR_F", "0.4"))  # N: the arch pointer's track friction
CONTACT_K = float(os.environ.get("BRANNOCK_K", "3000"))  # N/m: the give of the foot's vinyl skin under a slider
CONTACT_C = float(os.environ.get("BRANNOCK_C", "10"))  # N s/m

# the foot (its own frame: x along u from the heel's back on the sole, y = -v, z up)
HEEL_R, HEEL_C = 0.032, 0.032
BALL_U, BALL_V, BALL_R = 0.190, -0.033, 0.016  # first metatarsal head (medial bulge)
MT5_U, MT5_V, MT5_R = 0.170, 0.037, 0.014  # fifth metatarsal head (lateral bulge: the widest point)
ANKLE_U, ANKLE_R, ANKLE_TOP = 0.050, 0.033, 0.140
FOOT_L = 0.262
FOOT_M = float(os.environ.get("BRANNOCK_FOOT_M", "0.30"))
GRIP_Z = 0.105  # the holding hand's TCP this high above the sole, on the ankle
# where the rib meets the ball joint of the seated foot (rib centre u)
RIB_TARGET = BALL_U - math.sqrt((BALL_R + RIB_R) ** 2 - (RIB_V - BALL_V) ** 2)

METAL, DARK, SCALE, SKIN, CAP, BLUE = ((0.75, 0.76, 0.78), (0.15, 0.15, 0.17), (0.92, 0.92, 0.88),
                                         (0.93, 0.80, 0.68), (0.35, 0.30, 0.28), (0.25, 0.40, 0.75))


def _v(p) -> tuple:
    return tuple(float(x) for x in p)


def dev(u: float, v: float, z: float = 0.0) -> np.ndarray:
    """World point of device coordinates (u, v, z above the plate's top)."""
    return O + u * D + v * R + np.array([0.0, 0.0, z])


def _dbox(u, v, z, su, sv, h, color, collide=True) -> BoxShape:
    """A box in device coordinates (centre, size along u, v, z), as a linkage shape (offsets from O, world-aligned)."""
    return BoxShape(_v(dev(u, v, z) - O), (su, sv, h), color, yaw=PSI, collide=collide)


def _dcyl(u, v, z, r, h, color, collide=True) -> CylShape:
    return CylShape(_v(dev(u, v, z) - O), r, h, color, collide=collide)


def _cup(cu: float, facing: float) -> list:
    """A heel cup: a wall on a half circle round (cu, 0), open toward ``facing`` (+1: toward +u)."""
    out = []
    n = 9
    rm = CUP_R + CUP_T / 2
    seg = 2 * rm * math.tan(math.radians(10.0)) + 0.002
    for k in range(n):
        a = math.radians(90.0 + 20.0 * k + 10.0) if facing > 0 else math.radians(-90.0 + 20.0 * k + 10.0)
        cu_k, cv_k = cu + rm * math.cos(a), rm * math.sin(a)
        c = dev(cu_k, cv_k, CUP_H / 2) - O
        out.append(BoxShape(_v(c), (CUP_T, seg, CUP_H), METAL, yaw=PSI - a))  # v is clockwise of u
    return out


def _device() -> Linkage:
    base = [_dbox(0.165, 0.0, -PLATE_T / 2, 0.346, 0.130, PLATE_T, METAL),
            # the printed scale panel (visual): length lines every men's size (1/3 inch), across the plate
            _dbox(0.165, 0.0, 0.0002, 0.300, 0.090, 0.0004, SCALE, collide=False)]
    base += _cup(CUP_C, +1) + _cup(FAR_CUP_C, -1)
    for k in range(5, 16):  # men's sizes 5..15: foot length 7 2/3 + (size - 1)/3 inch
        L = (7.0 + 2.0 / 3.0 + (k - 1) / 3.0) * 0.0254
        base.append(_dbox(L, -0.010, 0.0005, 0.0012, 0.060 if k % 5 == 0 else 0.040, 0.0004, DARK, collide=False))
    # the two tracks (raised edges along which the sliders run)
    base += [_dbox(0.165, -0.0605, 0.002, 0.180, 0.009, 0.004, DARK, collide=False),
             _dbox(BAR_U0 - 0.006, 0.050, 0.002, 0.004, 0.030, 0.004, DARK, collide=False),
             _dbox(BAR_U1 + 0.006, 0.050, 0.002, 0.004, 0.030, 0.004, DARK, collide=False)]
    # width bar (lateral side): along u, inner face at BAR_V0, a finger tab on its outer face
    bu = (BAR_U0 + BAR_U1) / 2
    bar = [_dbox(bu, BAR_V0 + BAR_T / 2, BAR_H / 2 + 0.0005, BAR_U1 - BAR_U0, BAR_T, BAR_H, BLUE),
           _dbox(bu, BAR_V0 + BAR_T / 2, BAR_H + 0.0008, BAR_U1 - BAR_U0 - 0.004, BAR_T - 0.002, 0.0006, SCALE,
                 collide=False),
           _dbox(BAR_TAB_U, BAR_V0 + BAR_T + 0.006, 0.0175 + 0.0005, 0.020, 0.012, 0.035, BLUE)]
    # arch pointer (medial edge): a block on the track, the rib on its inner side, a thumb tab ahead of the rib
    pu = PTR_U0
    ptr = [_dbox(pu + 0.031, -0.0595, 0.005 + 0.0005, 0.088, 0.013, 0.010, BLUE),
           _dcyl(pu, RIB_V, 0.0085 + 0.0005, RIB_R, 0.017, DARK),
           _dbox(pu + PTR_TAB_DU, -0.064, 0.015 + 0.0005, 0.010, 0.014, 0.030, BLUE)]
    links = [Link("width_bar", tuple(bar), mass=0.05, anchor=_v(dev(bu, BAR_V0, 0.01)), axis="Y", slide=True,
                  frame_yaw=PSI, friction=BAR_F, damping=0.5, limits=(0.0, BAR_TRAVEL), report_contacts=True,
                  compliant=(CONTACT_K, CONTACT_C)),
             Link("arch_pointer", tuple(ptr), mass=0.02, anchor=_v(dev(pu, -0.058, 0.01)), axis="X", slide=True,
                  frame_yaw=PSI, friction=PTR_F, damping=0.5, limits=(-0.04, 0.10), report_contacts=True,
                  compliant=(CONTACT_K, CONTACT_C))]
    return Linkage.build("device", pos=_v(O + np.array([0.0, 0.0, 0.0005])), mass=0.80, color=METAL, floating=True,
                         friction=(0.35, 0.30), shapes=base, links=links, contact_offset=0.001, solver_iterations=48)


def _f(u, v, z) -> tuple:
    """Foot-frame point from (u, v, z): the foot's y axis is -v."""
    return (float(u), float(-v), float(z))


def _pitched(u, v, z, su, sv, h, pitch_deg, color) -> BoxShape:
    a = math.radians(pitch_deg) / 2
    return BoxShape(_f(u, v, z), (su, sv, h), color, quat_xyzw=(0.0, math.sin(a), 0.0, math.cos(a)))


def _foot() -> Body:
    sh = [CylShape(_f(HEEL_C, 0.0, 0.035), HEEL_R, 0.070, SKIN),  # heel
          BoxShape(_f(0.105, 0.002, 0.028), (0.150, 0.062, 0.056), SKIN),  # midfoot (sole to instep)
          CylShape(_f(MT5_U, MT5_V, 0.016), MT5_R, 0.032, SKIN),  # fifth metatarsal head (outer bulge)
          CylShape(_f(BALL_U, BALL_V, 0.018), BALL_R, 0.036, SKIN),  # first metatarsal head: the ball joint
          BoxShape(_f(0.180, 0.002, 0.017), (0.050, 0.070, 0.034), SKIN),  # forefoot
          BoxShape(_f(0.222, 0.004, 0.011), (0.040, 0.060, 0.022), SKIN),  # toes
          capsule(_f(0.206, BALL_V + 0.008, 0.013), _f(FOOT_L - 0.013, BALL_V + 0.009, 0.013), 0.013, SKIN),
          capsule(_f(0.222, 0.028, 0.009), _f(0.236, 0.030, 0.009), 0.009, SKIN),
          _pitched(0.125, 0.002, 0.060, 0.120, 0.052, 0.040, 22.0, SKIN),  # instep
          CylShape(_f(ANKLE_U, 0.0, (0.05 + ANKLE_TOP) / 2), ANKLE_R, ANKLE_TOP - 0.05, SKIN),  # ankle
          SphereShape(_f(ANKLE_U - 0.004, 0.030, 0.070), 0.011, SKIN),  # malleoli
          SphereShape(_f(ANKLE_U, -0.030, 0.075), 0.011, SKIN),
          CylShape(_f(ANKLE_U, 0.0, ANKLE_TOP + 0.0005), ANKLE_R - 0.004, 0.001, CAP, collide=False)]
    p = dev(START_OFF, 0.0, 0.0015)
    return Body.build("foot", pos=_v(p), mass=FOOT_M, yaw=PSI, color=SKIN, friction=(0.40, 0.35), shapes=sh,
                      contact_offset=0.001, damping=(0.0, 0.05))


DEVICE = _device()
FOOT = _foot()
OBJECTS = [DEVICE, FOOT]

# -- tracking and scoring -------------------------------------------------------------------------------------------
SEAT_MM, CENTRE_MM, TURN_DEG, TILT_DEG = 3.0, 3.0, 3.0, 3.0
F_MIN, F_MAX = 0.05, 5.0
RIB_TOL = 5.0 * MM
CLEAR = 0.02
MEAN_STEPS = 10  # 0.5 s at 20 Hz
STILL_STEPS, STILL_MM = 20, 1.0
_DEBUG = os.environ.get("BRANNOCK_DEBUG") == "1"
_DEBUG_EVERY = int(os.environ.get("BRANNOCK_DEBUG_EVERY", "5"))
_EP: dict = {}
_IDS: dict = {}
_CV: dict = {}


def _foot_samples() -> np.ndarray:
    """Points on the foot (foot frame) for the finger-clearance check."""
    pts = []
    for s in FOOT.shapes:
        if not s.collide:
            continue
        c = np.array(s.centre)
        e = s.half_extent()
        for sx, sy, sz in itertools.product((-1, 0, 1), repeat=3):
            pts.append(c + np.array([sx, sy, sz]) * e)
    return np.array(pts)


def _slider_samples(link: Link) -> np.ndarray:
    pts = []
    for s in link.shapes:
        c = np.array(s.centre)
        e = s.half_extent()
        for sx, sy, sz in itertools.product((-1, 0, 1), repeat=3):
            pts.append(c + np.array([sx, sy, sz]) * e)
    return np.array(pts)  # offsets from the linkage origin (world-aligned at the start)


FOOT_PTS = _foot_samples()
BAR_PTS = _slider_samples(DEVICE.links[0])
PTR_PTS = _slider_samples(DEVICE.links[1])


@on_reset
def _reset() -> None:
    _EP.update(tick=0, step=0, counted=-1, f_bar=[], f_ptr=[], recent=[], near=1.0, tcp={}, gaps={},
               peak_bar=0.0, peak_ptr=0.0)
    _CV.clear()


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


def _contact_forces(base) -> tuple:
    """Contact forces (N) between the width bar and the foot, and between the arch pointer and the foot."""
    from isaaclab_physx.physics import PhysxManager

    sim = PhysxManager.get_physics_sim_view()
    if _CV.get("sim") is not sim:
        env = base.scene.env_prim_paths[0]
        _CV.update(sim=sim, dt=float(base.physics_dt), views=[
            sim.create_rigid_contact_view([f"{env}/device/{n}"], filter_patterns=[[f"{env}/foot/body"]],
                                          max_contact_data_count=64) for n in ("width_bar", "arch_pointer")])
    out = []
    for view in _CV["views"]:
        forces = view.get_contact_force_matrix(dt=_CV["dt"])
        a = forces.numpy() if hasattr(forces, "numpy") else as_numpy(forces)
        out.append(float(np.linalg.norm(np.asarray(a, float).reshape(-1, 3), axis=-1).sum()))
    return tuple(out)


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("BRANNOCK_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _track_inner(env) -> None:
    from scipy.spatial import cKDTree

    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    _EP["tick"] += 1
    base = getattr(env, "unwrapped", env)
    origin = as_numpy(base.scene.env_origins[0])
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
    devc = base.scene["device"]
    if "bodies" not in _IDS:
        names, jn = list(devc.body_names), list(devc.joint_names)
        _IDS["bodies"] = {k: names.index(k) for k in ("base", "width_bar", "arch_pointer")}
        _IDS["joints"] = {k: jn.index(f"{k}_turn") for k in ("width_bar", "arch_pointer")}
    bp = as_numpy(devc.data.body_link_pose_w)[0]
    jp = as_numpy(devc.data.joint_pos)[0]
    ib = _IDS["bodies"]
    # the device frame now (it is loose): its base's pose (the base body's frame is the linkage origin, world-aligned
    # at the start)
    dp, dr = bp[ib["base"]][:3] - origin, rotation(bp[ib["base"]][3:7])
    o_now = dp + dr @ (O - np.asarray(DEVICE.pos))
    d_now, r_now = dr @ D, dr @ R
    s_bar = float(jp[_IDS["joints"]["width_bar"]])
    s_ptr = float(jp[_IDS["joints"]["arch_pointer"]])
    # the foot in the device frame
    ft = as_numpy(base.scene["foot"].data.root_link_pose_w)[0]
    fp, fr = ft[:3] - origin, rotation(ft[3:7])
    rel = fp - o_now
    heel_u, heel_v = float(rel @ d_now), float(rel @ r_now)
    fx = fr[:, 0]
    turn = math.degrees(math.atan2(float(fx @ r_now), float(fx @ d_now)))  # + toward the outer side
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(fr[2, 2])))))
    f_bar, f_ptr = _contact_forces(base)
    _EP["f_bar"] = (_EP["f_bar"] + [f_bar])[-MEAN_STEPS:]
    _EP["f_ptr"] = (_EP["f_ptr"] + [f_ptr])[-MEAN_STEPS:]
    # finger clearance from the foot and the two sliders
    foot_w = fp + FOOT_PTS @ fr.T
    bar_w = bp[ib["width_bar"]][:3] - origin + BAR_PTS @ rotation(bp[ib["width_bar"]][3:7]).T
    ptr_w = bp[ib["arch_pointer"]][:3] - origin + PTR_PTS @ rotation(bp[ib["arch_pointer"]][3:7]).T
    tree = cKDTree(np.concatenate([foot_w, bar_w, ptr_w]))
    near = min(float(tree.query(f, k=1)[0].min()) for f in fingers.values())
    _EP.update(heel_u=heel_u, heel_v=heel_v, turn=turn, tilt=tilt, s_bar=s_bar, s_ptr=s_ptr,
               rib_u=PTR_U0 + s_ptr, near=near, f_bar_now=f_bar, f_ptr_now=f_ptr,
               peak_bar=max(_EP["peak_bar"], f_bar), peak_ptr=max(_EP["peak_ptr"], f_ptr),
               dev_xy=o_now[:2].copy(), foot_p=fp.copy(),
               bar_p=bp[ib["width_bar"]][:3].copy(), ptr_p=bp[ib["arch_pointer"]][:3].copy())


def _tally(o) -> dict:
    recent = _EP["recent"]
    if _EP["counted"] != _EP["tick"] and "foot_p" in _EP:
        _EP["counted"] = _EP["tick"]
        recent.append(np.concatenate([_EP["foot_p"], _EP["bar_p"], _EP["ptr_p"]]))
        del recent[:-STILL_STEPS]
    still = len(recent) == STILL_STEPS and all(np.abs(p - recent[-1]).max() * 1000 < STILL_MM for p in recent)
    if "heel_u" not in _EP:
        return dict(still=False, seated=False, bar=False, ptr=False, clear=False, done=False)
    seated = (_EP["heel_u"] <= SEAT_MM * MM and abs(_EP["heel_v"]) <= CENTRE_MM * MM and abs(_EP["turn"]) <= TURN_DEG
              and _EP["tilt"] <= TILT_DEG)
    fb = float(np.mean(_EP["f_bar"])) if _EP["f_bar"] else 0.0
    fpt = float(np.mean(_EP["f_ptr"])) if _EP["f_ptr"] else 0.0
    bar = seated and F_MIN <= fb <= F_MAX
    ptr = seated and F_MIN <= fpt <= F_MAX and abs(_EP["rib_u"] - RIB_TARGET) <= RIB_TOL
    clear = _EP["near"] >= CLEAR
    done = bool(seated and bar and ptr and clear and still)
    return dict(still=still, seated=seated, bar=bar, ptr=ptr, clear=clear, done=done, fb=fb, fp=fpt)


def _debug(t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY or "heel_u" not in _EP:
        return
    print(f"DBG step={_EP['step']} heel_u={_EP['heel_u'] * 1000:+.1f}mm heel_v={_EP['heel_v'] * 1000:+.1f}mm "
          f"turn={_EP['turn']:+.2f}deg tilt={_EP['tilt']:.2f} bar_s={_EP['s_bar'] * 1000:.1f}mm "
          f"rib_u={_EP['rib_u'] * 1000:.1f}mm(tgt {RIB_TARGET * 1000:.1f}) f_bar={_EP['f_bar_now']:.2f}N "
          f"f_ptr={_EP['f_ptr_now']:.2f}N peak=({_EP['peak_bar']:.1f},{_EP['peak_ptr']:.1f}) "
          f"dev=({_EP['dev_xy'][0]:.3f},{_EP['dev_xy'][1]:.3f}) near={_EP['near'] * 1000:.0f}mm "
          f"seated={int(t['seated'])} bar={int(t['bar'])} ptr={int(t['ptr'])} clear={int(t['clear'])} "
          f"still={int(t['still'])} | "
          + " ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{v[2] - TOP:.3f}) gap={_EP['gaps'][k] * 1000:.0f}"
                     for k, v in _EP["tcp"].items()), flush=True)


def score(objects) -> float:
    """0.3 heel seated, +0.3 width bar touching, +0.3 arch pointer at the ball joint (each only with the heel
    seated); full score is success()."""
    t = _tally(objects)
    if _DEBUG:
        _debug(t)
    return 0.3 * t["seated"] + 0.3 * t["bar"] + 0.3 * t["ptr"]


def success(objects) -> bool:
    return bool(_tally(objects)["done"])


# -- scripted demo --------------------------------------------------------------------------------------------------
# Open loop, from the known layout. The right hand comes down round the ankle (fingers closing across the foot),
# closes on it and draws the foot back along the device until the heel sits in the heel cup, then keeps holding it.
# The left hand slides the arch pointer forward by its knob until the rib meets the ball joint, rises, then pushes
# the width bar's tab in until the bar meets the outer edge of the foot, and rises. The right hand lets go and rises.
# BRANNOCK_ONE=1: one hand at a time: the right hand seats the heel and lets go before the left hand works the sliders
# (with the demo's gentle pushes this also scores 1.0; with BRANNOCK_PTR_PAST=16 BRANNOCK_BAR_PAST=35 it scores 0.6).
OPEN_C = -0.047
ONE = os.environ.get("BRANNOCK_ONE") == "1"
ANKLE_PINCH = float(os.environ.get("BRANNOCK_PINCH", "-0.029"))  # closed on the 66 mm ankle
PULL_PAST = float(os.environ.get("BRANNOCK_PULL_PAST", "12")) * MM  # draw the ankle this far past the seated spot
HOLD_PAST = float(os.environ.get("BRANNOCK_HOLD_PAST", "12")) * MM  # then hold the ankle this far past it
PTR_PAST = float(os.environ.get("BRANNOCK_PTR_PAST", "7")) * MM  # push the knob this far past the touch point
BAR_PAST = float(os.environ.get("BRANNOCK_BAR_PAST", "20")) * MM  # push the tab this far past the touch point
HOLD_YAW = math.radians(float(os.environ.get("BRANNOCK_HOLD_YAW", "0")))  # right grip yaw (closes across the foot)
PTR_YAW = math.radians(float(os.environ.get("BRANNOCK_PTR_YAW", "0")))  # left grip yaw at the knob (closes along u)
BAR_YAW = math.radians(float(os.environ.get("BRANNOCK_BAR_YAW", "0")))  # left grip yaw at the tab (along v)
BAR_FH = float(os.environ.get("BRANNOCK_BAR_FH", "10")) * MM  # TCP to the outer face of the closed fingers
FINGER_HALF = float(os.environ.get("BRANNOCK_FH", "13")) * MM  # half the finger's thickness at the tip (the pushing face is this far from the TCP)


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
    return Waypoint(t, tuple(float(v) for v in p), finger, label, rot)


def _right_path() -> list:
    r = grip_rotation(HOLD_YAW)
    at = lambda u, z: dev(u, 0.0, z)  # noqa: E731
    a0 = START_OFF + ANKLE_U
    a1 = ANKLE_U - PULL_PAST
    a2 = ANKLE_U - HOLD_PAST
    out = _lead_in("right", at(a0, GRIP_Z + 0.045), r, OPEN_C, "above the ankle, open", seconds=4.0)
    out += [_w(2.0, at(a0, GRIP_Z), OPEN_C, "lower round the ankle", r),
            _w(0.5, at(a0, GRIP_Z), OPEN_C, "settle", r),
            _w(1.2, at(a0, GRIP_Z), ANKLE_PINCH, "grasp the ankle", r),
            _w(3.0, at(a1, GRIP_Z), ANKLE_PINCH, "draw the foot back into the heel cup", r),
            _w(1.0, at(a2, GRIP_Z), ANKLE_PINCH, "hold the heel against the cup", r)]
    if ONE:
        out += [_w(1.0, at(a2, GRIP_Z), OPEN_C, "let go of the ankle", r),
                _w(1.5, at(a2, GRIP_Z + 0.04), OPEN_C, "rise", r),
                _w(2.0, at(ANKLE_U + 0.09, GRIP_Z + 0.04), OPEN_C, "draw back along the foot", r),
                _w(2.5, np.array([0.25, -0.25, TOP + 0.15]), OPEN_C, "move away to the right", r),
                _w(60.0, np.array([0.25, -0.25, TOP + 0.15]), OPEN_C, "stay clear", r)]
        return out
    out += [_w(T_LEFT_DONE - sum(w.duration for w in out), at(a2, GRIP_Z), ANKLE_PINCH,
               "keep holding the ankle while the other hand measures", r),
            _w(1.0, at(a2, GRIP_Z), OPEN_C, "let go of the ankle", r),
            _w(1.5, at(a2, GRIP_Z + 0.04), OPEN_C, "rise", r),
            _w(2.0, at(ANKLE_U + 0.09, GRIP_Z + 0.04), OPEN_C, "draw back along the foot, fingers either side", r),
            _w(2.0, at(ANKLE_U + 0.09, GRIP_Z + 0.04) + np.array([0.0, -0.12, 0.0]), OPEN_C, "move aside", r),
            _w(20.0, at(ANKLE_U + 0.09, GRIP_Z + 0.04) + np.array([0.0, -0.12, 0.0]), OPEN_C, "stay clear", r)]
    return out


T_RIGHT_SEATED = 4.0 + 2.0 + 0.5 + 1.2 + 3.0 + 1.0
T_LEFT_DONE = 0.0  # set below


def _left_path() -> list:
    rp = grip_rotation(PTR_YAW)
    rb = grip_rotation(BAR_YAW)
    # the arch pointer: the finger's face behind the thumb tab (5 mm half-thickness), pushing it forward (+u)
    back = 0.005 + FINGER_HALF + 0.001
    tab0 = PTR_U0 + PTR_TAB_DU
    tab_t = RIB_TARGET + PTR_TAB_DU  # where the tab is when the rib meets the ball joint
    kz = 0.024  # the TCP this high above the plate at the tab
    pk = lambda u, z=kz: dev(u - back, -0.064, z)  # noqa: E731
    park = np.array([0.120, 0.110, TOP + 0.100])  # beside the toes, clear of the foot
    out = _lead_in("left", park, rp, 0.0, "to the side of the toes", seconds=4.0)
    wait = T_RIGHT_SEATED - 4.0 + (7.0 if ONE else 0.0)  # one arm: the right hand lets go and moves away first
    out += [_w(max(0.5, wait), park, 0.0, "wait for the heel to be seated", rp),
            _w(2.5, pk(tab0 - 0.01, kz + 0.06), 0.0, "over the toes, above and behind the arch pointer's tab", rp),
            _w(2.5, pk(tab0 - 0.008), 0.0, "lower behind the tab", rp),
            _w(3.0, pk(tab_t + PTR_PAST), 0.0, "slide the arch pointer forward to the ball joint", rp),
            _w(1.0, pk(tab_t + PTR_PAST), 0.0, "settle", rp),
            _w(0.6, pk(tab_t - 0.004), 0.0, "ease off", rp),
            _w(1.5, pk(tab_t - 0.008, kz + 0.06), 0.0, "rise off the tab", rp)]
    # the width bar: its tab's outer face at v = BAR_V0 + BAR_T + 0.012 at the start; the bar touches the foot when
    # its inner face reaches the foot's outer edge (v 0.051)
    edge = MT5_V + MT5_R
    tab_face0 = BAR_V0 + BAR_T + 0.012
    tz = 0.022
    pb = lambda vface, z=tz: dev(BAR_TAB_U, vface + BAR_FH + 0.001, z)  # noqa: E731
    touch_face = tab_face0 - (BAR_V0 - edge)
    out += [_w(3.0, pb(tab_face0 + 0.015, tz + 0.06), 0.0, "over, outside the width bar's tab", rb),
            _w(2.0, pb(tab_face0 + 0.008), 0.0, "lower beside the tab", rb),
            _w(3.0, pb(touch_face - BAR_PAST), 0.0, "slide the width bar in to the edge of the foot", rb),
            _w(1.0, pb(touch_face - BAR_PAST), 0.0, "settle", rb),
            _w(0.8, pb(touch_face + 0.004), 0.0, "ease off", rb),
            _w(1.5, pb(touch_face + 0.02, tz + 0.08), 0.0, "rise clear", rb)]
    global T_LEFT_DONE
    T_LEFT_DONE = sum(w.duration for w in out)
    out += [_w(30.0, pb(touch_face + 0.03, tz + 0.10), 0.0, "stay clear", rb)]
    return out


def demo():
    left = _left_path()
    return {"right": _right_path(), "left": left}
