"""Hold a Koch v1.1 arm up as its power is cut, and lay it limp in its cradle.

A Koch v1.1 follower arm (the open-source 3D-printed arm LeRobot drives as koch_follower: Dynamixel XL430-W250 servos on the shoulder pan and lift,
XL330-M288 on the elbow, wrist and gripper) stands on its base on the bench in its working pose, its joints held by their servos' position hold. When both
YAM grippers are closed on the arm, 1 s later its torque is cut: the five arm joints go limp under gravity. The grippers carry the arm, lay it in its
cradle pose on the foam cradle, and let go.

Model: the arm's joints and links from the Koch v1.1 follower URDF and meshes (jess-moss/koch-v1-1, as fixed in ManiSkill's assets; Apache-2.0; meshes
converted and clustered to 1.5 mm by koch/convert.py): joint axes, origins, limits, link masses, centres of mass and inertias (koch_model.py; link_6's
misplaced centre of mass put at its meshes' centroid). The moving jaw is part of the wrist-roll body (the arm's own gripper is not driven). Declared code:
the torque cut (the joints' position hold is released) and the scorer. Unpowered, each joint is back-driven through its Dynamixel gear train: Coulomb
friction, the rotor's reflected inertia and viscous damping are estimates (README). Of the arm's links only the forearm, wrist and gripper collide with
the base, the shoulder and the upper arm (SELF_CONTACT).

    python demokit.py task.py OUT --mode full
    python demokit.py task.py OUT --no-record --mode full,none,onehand,wrong,gate
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import gatekit as gk  # noqa: E402
import mechkit as m  # noqa: E402
import koch_model as km  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = 'One of this robot arm\'s servos has failed and stopped its servo bus, so the arm cannot be driven to a rest pose; its power will be cut for the repair and it will go limp. Hold it up so it does not drop: close both grippers on it, and one second later the power is cut. With both grippers still on it, lay it down in its cradle pose (shoulder lift 0.45, elbow flex 0.8, wrist flex -0.35 rad, the other joints 0) without dropping it, then let go.'
SUCCESS_HOLD_S = 0.0
# HOLD_STEPS: the success state must stay true for 40 scorer steps = 2 s at the 20 Hz scorer rate, so a pose that only passes through the cradle
# window for an instant, or a hand that lets go and lets the arm sag back, does not score; SUCCESS_HOLD_S stays 0.0 because the count is kept here.
HOLD_STEPS = 40  # the final state counted by the scorer: 2 s at 20 Hz
V_DROP = 0.5  # m/s: any point of the arm faster than this after the cut is a drop
F_HOLD = float(os.environ.get("F_HOLD", "1.0"))  # N: each finger of a gripper must push on the arm at least this hard for that gripper to count as holding it
V_HIT = float(os.environ.get("V_HIT", "0.25"))  # m/s: a link striking the bench, pad or cradle faster than this is a hard landing
TOP = TABLE_TOP[SCENE]
HERE = pathlib.Path(__file__).parent
DT = 0.05

# -- the Koch's kinematics (follower_arm_v1.1.urdf; koch_model.py): each link frame aligned with the base's at joint zero; at zero the arm lies straight out along the base's -y
NAMES = km.NAMES  # shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll
ORIGIN, AXIS, SIGN, LIM = km.ORIGIN, km.AXIS, km.SIGN, km.LIM
Q_WORK = np.array([0.0, float(os.environ.get("WORK_SH", "1.2")), float(os.environ.get("WORK_ELB", "1.2")), float(os.environ.get("WORK_WR", "0.0")), 0.0])  # working pose (URDF joint angles, rad): upper arm 69 degrees up, forearm level
Q_SLEEP = np.array([0.0, 0.45, 0.8, -0.35, 0.0])  # the cradle pose: the arm laid out forward along its cradle, gripper level
PAN = (float(os.environ.get("BASE_X", "0.30")), float(os.environ.get("BASE_Y", "0.16")))  # the shoulder pan axis on the bench; the arm points toward -y
BASE = (PAN[0] - km.ORIGIN["shoulder_pan"][0], PAN[1] - km.ORIGIN["shoulder_pan"][1])  # the URDF base origin (the base's back edge)
YAW = 0.0
# An unpowered Dynamixel X-series servo is back-driven through its spur gear train (XL430-W250 258.5:1 on the pan and lift, XL330-M288 288.35:1 on the
# rest; ROBOTIS e-Manual). Robotis publishes no torque-off friction, so these are estimates: Coulomb friction at the joint 0.03 N m (XL430) and 0.012 N m
# (XL330), about 2 % of stall torque; the rotor's reflected inertia 0.005 and 0.002 kg m^2 (gear ratio squared times a coreless rotor's inertia);
# viscous damping 0.02 N m s/rad with the driver's output open (range 0.01 to 0.1).
_XL430 = ("shoulder_pan", "shoulder_lift")
FRICTION_J = {n: float(os.environ.get("K_FRIC430", "0.03")) if n in _XL430 else float(os.environ.get("K_FRIC330", "0.012")) for n in NAMES}
ARMATURE_J = {n: 0.005 if n in _XL430 else 0.002 for n in NAMES}
DAMP_J = {n: float(os.environ.get("K_DAMPING", "0.02")) for n in NAMES}
HOLD_K, HOLD_D = float(os.environ.get("K_HOLD_K", "10.0")), float(os.environ.get("K_HOLD_D", "0.3"))  # a powered joint's position hold (stiff, damped; estimate)
TAU_MAX = {n: 1.4 if n in _XL430 else 0.52 for n in NAMES}  # stall torque (N m): XL430-W250 at 11.1 V, XL330-M288 at 5 V (e-Manual)
PAD_S = (float(os.environ.get("PAD_SX", "0.12")), float(os.environ.get("PAD_SY", "0.44")), 0.02)  # x, y, thickness (m)
# centre: the pad's back edge 2 cm in front of the pan axis, clear of the base (its front edge 1.2 cm in front); its near edge 3.5 cm short of the arm's
# line (x = 26.5 cm), so the YAM hands, which reach in from -x, are not over it when they lay the arm low
PAD_C = (PAN[0] + float(os.environ.get("PAD_DX", "0.025")), PAN[1] - 0.02 - PAD_S[1] / 2)
BASE_Z = 0.0  # the base's underside is at its URDF origin


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


RS = _rz(math.radians(YAW))  # base frame to station frame
BASE_POS = np.array([BASE[0], BASE[1], TOP + BASE_Z])


def to_station(p):
    return np.asarray(p) @ RS.T + BASE_POS


fk = km.fk


def _soup(tri):
    v = tri.reshape(-1, 3)
    key = np.round(v * 1e6).astype(np.int64)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    return m.Mesh(uniq / 1e6, inv.reshape(-1, 3))


LOOKS = {"black": m.Look(color=m.rgb(0.07, 0.07, 0.08), roughness=0.55), "printed": m.Look(color=m.rgb(0.93, 0.93, 0.95), roughness=0.6),
         "foam": m.Look(color=m.rgb(0.20, 0.42, 0.62), roughness=0.9)}
MASS_MOVING = km.MASS_MOVING


def _look(f):
    return "black" if "motor" in f else "printed"


# mechkit authors each link at its joint's zero (mechkit.py, 'model a mechanism at its start pose'), and the Koch's URDF zero lies the arm flat along
# the bench, through the cradle. So the simulator's joint zero is the working pose: each link is built where it is at Q_WORK, and a joint's simulator
# angle is its URDF angle less its working-pose angle (in the joint's sign).
J0 = Q_WORK.copy()
_T0 = fk(J0)


def _at_work(n, pts):
    """Points of link n (base frame, at URDF zero) moved to where they are at the working pose."""
    T = _T0[n]
    return (np.asarray(pts) - ORIGIN[n]) @ T[:3, :3].T + T[:3, 3]


def _koch():
    fparts = [(f"base_{i}", _soup(t), _look(f)) for i, (f, t) in enumerate(km.BASE_TRIS)]
    fcols = [m.col_hull(f"base_{i}_c", _soup(t)) for i, (f, t) in enumerate(km.BASE_TRIS)]
    links = []
    for k, n in enumerate(NAMES):
        tris = [(f, _at_work(n, t.reshape(-1, 3)).reshape(t.shape)) for f, t in km.TRIS[n]]
        parts = [(f"{n}_{i}", _soup(t), _look(f)) for i, (f, t) in enumerate(tris)]
        cols = [m.col_hull(f"{n}_{i}_c", _soup(t)) for i, (f, t) in enumerate(tris)]
        mass, com, inertia = km.INERTIAL[n]
        Rn = _T0[n][:3, :3]
        com, inertia = _at_work(n, com), Rn @ inertia @ Rn.T
        ax = Rn @ np.eye(3)[AXIS[n]]
        assert abs(ax[AXIS[n]]) > 0.999, (n, ax)  # every pitch joint turns about the base's x, so each axis keeps its direction at the working pose
        j0 = math.degrees(SIGN[n] * J0[k])
        lo, hi = sorted((SIGN[n] * LIM[n][0] - j0, SIGN[n] * LIM[n][1] - j0))
        links.append(m.hinge(n, "XYZ"[AXIS[n]], tuple(map(float, _T0[n][:3, 3])), lo, hi, parts, cols, parent=None if k == 0 else NAMES[k - 1], mass=float(mass), gravity=True,
                             com=tuple(map(float, com)), spring=HOLD_K, damping=HOLD_D, friction=FRICTION_J[n], effort=TAU_MAX[n], armature=ARMATURE_J[n],
                             inertia=tuple(float(v) for v in np.diag(inertia))))
    start = [(n, float(SIGN[n] * (a - a0))) for n, a, a0 in zip(NAMES, Q_WORK, J0)]
    return m.mechanism("koch", tuple(BASE_POS), links, LOOKS, frame_parts=fparts, frame_colliders=fcols, yaw_deg=YAW, friction=(0.8, 0.7), solver_iters=64, start=start)


# Self-contact: only the forearm, wrist and gripper collide with the base frame, the shoulder and the upper arm; every other pair of the arm's bodies
# (each joint's two neighbours, whose hulls meet at the joint) is filtered.
SELF_CONTACT = os.environ.get("K_SELF", "1") == "1"
_TOUCH = {(a, b) for a in ("frame", "shoulder_pan", "shoulder_lift") for b in ("elbow_flex", "wrist_flex", "wrist_roll")} - {("shoulder_lift", "elbow_flex")}


def _self_contact(txt):
    import re

    root = re.search(r"rel physics:body1 = <(/[^>]*)/" + NAMES[0] + ">", txt).group(1)
    bodies = ["frame"] + NAMES
    txt = txt.replace("bool physxArticulation:enabledSelfCollisions = 0", "bool physxArticulation:enabledSelfCollisions = 1")
    for i, a in enumerate(bodies):
        others = [b for b in bodies[i + 1:] if (a, b) not in _TOUCH and (b, a) not in _TOUCH]
        if not others:
            continue
        pat = re.compile(r'(def Xform "' + a + r'" \(\n\s*prepend apiSchemas = \[)([^\]]*)(\]\n\s*\)\n(\s*)\{)')
        mt = pat.search(txt)
        rel = ", ".join(f"<{root}/{b}>" for b in others)
        txt = txt[:mt.start()] + mt.group(1) + mt.group(2) + ', "PhysicsFilteredPairsAPI"' + mt.group(3) + f"\n{mt.group(4)}    rel physics:filteredPairs = [{rel}]" + txt[mt.end():]
    return txt


def _with_start_state(part):
    """The USD with each joint's start angle authored as its joint state, so the arm is in its working pose from the first frame."""
    import dataclasses
    build0 = part.build

    def build():
        txt = build0()
        for n, a in zip(NAMES, Q_WORK):
            deg = math.degrees(SIGN[n] * (a - J0[NAMES.index(n)]))
            head = f'def PhysicsRevoluteJoint "{n}_joint" (\n        prepend apiSchemas = ["PhysicsDriveAPI:angular"]'
            k = txt.index(head)
            txt = txt[:k] + head.replace('["PhysicsDriveAPI:angular"]', '["PhysicsDriveAPI:angular", "PhysicsJointStateAPI:angular"]') + txt[k + len(head):]
            anchor = 'uniform token drive:angular:physics:type = "force"'
            j = txt.index(anchor, k)
            txt = txt[:j] + f"float state:angular:physics:position = {deg:.6f}\n        float state:angular:physics:velocity = 0\n        " + txt[j:]
        if SELF_CONTACT:
            txt = _self_contact(txt)
        return txt.replace('"PhysicsMassAPI"', '"PhysicsMassAPI", "PhysxContactReportAPI"')  # contact reports on every link, so the scorer reads the fingers' push on the arm

    return dataclasses.replace(part, build=build)


ARM = _with_start_state(_koch())
PAD = m.fixed("pad", (PAD_C[0], PAD_C[1], TOP), [("pad", m.box((PAD_S[0], PAD_S[1], PAD_S[2]), (0, 0, PAD_S[2] / 2)), "foam")],
              [m.col_box("pad_c", (PAD_S[0], PAD_S[1], PAD_S[2]), (0, 0, PAD_S[2] / 2))], LOOKS, friction=(1.0, 0.9))


def floor_under(pts):
    """The height of what a point would land on: the bench, the pad, or a cradle block (each block stands on the pad)."""
    f = np.full(len(pts), TOP)
    f = np.where((np.abs(pts[:, 0] - PAD_C[0]) < PAD_S[0] / 2) & (np.abs(pts[:, 1] - PAD_C[1]) < PAD_S[1] / 2), TOP + PAD_S[2], f)
    for cx, cy, sx, sy, h in BLOCKS.values():
        f = np.where((np.abs(pts[:, 0] - cx) < sx / 2) & (np.abs(pts[:, 1] - cy) < sy / 2), TOP + h, f)
    return f


# -- reading the arm -------------------------------------------------------------------------

def q_now(st) -> np.ndarray:
    """The arm's joint readings, in the URDF's sense of each joint."""
    return np.array([SIGN[n] * float(st.joint(n)) for n in NAMES]) + J0


SURF = km.SURF  # per link: the surface's points in the link frame (the frame at the joint's origin)


def arm_surface(q, from_link: int = 1):
    """The arm's surface points (station frame) from the shoulder_lift link out (the shoulder link, which only pans on the base, is left out)."""
    T = fk(q)
    out = []
    for n in NAMES[from_link:]:
        out.append(SURF[n] @ T[n][:3, :3].T + T[n][:3, 3])
    return to_station(np.concatenate(out))


BASE_PTS = to_station(km.BASE_SURF)  # the fixed base's surface
_BASE_KD = cKDTree(BASE_PTS)


def _cut(q, cx, cy, sx, sy):
    """A cradle block's height, cut so that the arm at joint angles q just rests on it: the lowest point of the arm over the block."""
    p = arm_surface(q)
    over = (np.abs(p[:, 0] - cx) < sx / 2) & (np.abs(p[:, 1] - cy) < sy / 2)
    return round(float(p[over, 2].min() - TOP), 3)


# the cradle: foam blocks on the pad, each cut to the arm's underside in the cradle pose, clear of the hands' lower fingers (x, y centre; x, y size; top
# height above the bench; m). Along the arm (distance in front of the pan axis): under the elbow, under the forearm before the wrist, and under the gripper.
CRADLE_X = {k: tuple(float(v) for v in os.environ.get("K_" + k.upper(), d).split(",")) for k, d in
            (("rest_elbow", "0.090,0.115"), ("rest_forearm", "0.140,0.175"), ("rest_gripper", "0.250,0.290"))}
BLOCKS = {}
for _k, (_x0, _x1) in CRADLE_X.items():
    _cy = PAN[1] - (_x0 + _x1) / 2
    BLOCKS[_k] = (PAN[0], _cy, 0.06, _x1 - _x0)
    BLOCKS[_k] = (*BLOCKS[_k], _cut(Q_SLEEP, *BLOCKS[_k]))


def _clearance(q):
    """The least gap (m) between the cradle blocks and the arm's surface at joint angles q: below zero is a collider overlap."""
    p = arm_surface(q, from_link=0)
    out = {}
    for k, (cx, cy, sx, sy, h) in BLOCKS.items():
        dx = np.maximum(np.abs(p[:, 0] - cx) - sx / 2, 0)
        dy = np.maximum(np.abs(p[:, 1] - cy) - sy / 2, 0)
        dz = np.maximum(p[:, 2] - (TOP + h), 0)
        out[k] = round(float(np.sqrt(dx * dx + dy * dy + dz * dz).min()), 3)
    return out


_BT = PAD_S[2]  # each block stands on the pad
RESTS = [m.fixed(k, (v[0], v[1], TOP + _BT), [(k, m.box((v[2], v[3], v[4] - _BT), (0, 0, (v[4] - _BT) / 2)), "foam")],
                 [m.col_box(k + "_c", (v[2], v[3], v[4] - _BT), (0, 0, (v[4] - _BT) / 2))], LOOKS, friction=(1.0, 0.9)) for k, v in BLOCKS.items()]
OBJECTS = [ARM, PAD] + RESTS
print(f"[koch] moving mass {MASS_MOVING:.3f} kg; cradle blocks: {BLOCKS}; clearance to the arm at the start {_clearance(Q_WORK)}", flush=True)


def link_points(q, name, local):
    T = fk(q)[name]
    return to_station(T[:3, :3] @ np.asarray(local) + T[:3, 3])


def link_axes(q, name):
    """A link's frame axes in the station frame (columns)."""
    return RS @ fk(q)[name][:3, :3]


def sleep_err_deg(q):
    return np.degrees(np.abs(q - Q_SLEEP))


def _gear(asset, j, name):
    """The gear train's static and dynamic friction (N m) and the rotor's reflected inertia, powered or not."""
    f = FRICTION_J[name]
    try:
        asset.write_joint_friction_coefficient_to_sim_index(joint_friction_coeff=m._tensor(asset, f), joint_dynamic_friction_coeff=m._tensor(asset, 0.7 * f), joint_ids=[j])
        asset.write_joint_armature_to_sim_index(armature=m._tensor(asset, ARMATURE_J[name]), joint_ids=[j])
    except (AttributeError, TypeError) as e:
        print(f"[koch] gear friction not written: {e}", flush=True)


def _powered_gains(asset, j, name):
    _gear(asset, j, name)
    asset.write_joint_effort_limit_to_sim(m._tensor(asset, TAU_MAX[name]), joint_ids=[j])
    asset.write_joint_stiffness_to_sim(m._tensor(asset, HOLD_K), joint_ids=[j])
    asset.write_joint_damping_to_sim(m._tensor(asset, HOLD_D), joint_ids=[j])
    asset.set_joint_position_target(m._tensor(asset, float(SIGN[name] * (Q_WORK - J0)[NAMES.index(name)])), joint_ids=[j])


_unlock0 = m.unlock


def _unlock(env, part, link):
    """At a reset mechkit gives touched joints their own gains back: here that is the powered hold on the working pose."""
    if part != "koch":
        return _unlock0(env, part, link)
    asset, j = m._joint(env, part, link)
    _powered_gains(asset, j, link)


m.unlock = _unlock
m._TOUCHED.update((("koch", n) for n in NAMES))  # the first reset gives the joints the powered hold too, with their target on the working pose


_LAT = np.array([(x, y, z) for x in (-0.04, -0.02, 0.0, 0.02, 0.04) for y in (-0.03, 0.0, 0.03) for z in np.arange(-0.12, 0.1471, 0.02)])
_SURF_LINK = np.concatenate([[n] * len(SURF[n]) for n in NAMES])  # the link of each point of arm_surface(q, from_link=0)


def hand_body(env) -> np.ndarray:
    """Points over each whole YAM gripper (station frame): a lattice 2 cm apart, x +-4 cm, y +-3 cm, from 12 cm behind the gripper frame (the wrist link) to the
    fingertips 14.7 cm ahead of it (palm, housing, pads, fingers), both hands."""
    base = env.unwrapped
    origin = m._np(base.scene.env_origins[0])
    lat = _LAT
    out = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        i = robot.find_bodies("gripper")[0][0]
        pose = m._np(robot.data.body_link_pose_w)[0, i]
        out.append(pose[:3] - origin + lat @ m.quat_matrix(pose[3:7]).T)
    return np.concatenate(out)


# -- the scorer ---------------------------------------------------------------------------------

class Trial:
    def __init__(self) -> None:
        self.reset()

    def reset(self, env=None) -> None:
        m._TOUCHED.update((("koch", n) for n in NAMES))  # the next reset restores the powered hold
        if env is not None:  # the working pose, powered, as the scene's start (mechkit does not write a mechanism's joints at reset)
            import torch
            asset = env.unwrapped.scene["koch"]
            ids = [m._joint(env, "koch", n)[1] for n in NAMES]
            pos = torch.tensor([[float(SIGN[n] * (a - a0)) for n, a, a0 in zip(NAMES, Q_WORK, J0)]], device=asset.device)
            asset.write_joint_state_to_sim(pos, torch.zeros_like(pos), joint_ids=ids)
            for n in NAMES:
                _powered_gains(asset, m._joint(env, "koch", n)[1], n)
        self.t = 0
        self.moved = False
        self.held_steps = 0
        self.cut = False
        self.cut_t = None
        self.powered = True
        self.last_q = None
        self.last_pts = None
        self.latch = False  # a drop: set after the cut
        self.why = ""
        self.peak = 0.0
        self.peak1 = 0.0  # peak joint speed in the first second
        self.clear_steps = 0
        self.dmin, self.near = 9.0, ""
        self.base_gap = 9.0  # least distance (m) from the forearm, wrist and gripper surface to the base surface so far
        self.full = False
        self.score = 0.0
        self.now: dict = {}
        self.held_at_cut = False
        self.view = None
        self.fhist = []
        self.land_t = None  # seconds from the cut until a link first reaches the foam or bench (the unheld fall time)
        self.set_by_hands = False  # the arm within 10 degrees of the cradle pose at the last step both grippers held it

    def _finger_forces(self, env) -> np.ndarray:
        """The contact force (N) of each of the four YAM fingers on the arm's links (all five links summed as vectors), this physics step."""
        base = env.unwrapped
        if self.view is None:
            asset = base.scene["koch"]
            names, paths = list(asset.body_names), list(asset.root_view.link_paths[0])
            links = [paths[names.index(n)] for n in NAMES]
            tips = []
            for side in ("left", "right"):
                robot = base.scene[f"{side}_robot"]
                rp = dict(zip(list(robot.body_names), list(robot.root_view.link_paths[0])))
                tips += [rp["tip_left"], rp["tip_right"]]
            self.view = base.sim.physics_sim_view.create_rigid_contact_view(links, filter_patterns=[tips] * len(links), max_contact_data_count=256 * len(links))
        mat = self.view.get_contact_force_matrix(dt=float(base.sim.get_physics_dt()))
        mat = np.asarray(mat.numpy() if hasattr(mat, "numpy") else mat.detach().cpu().numpy(), float).reshape(len(NAMES), 4, 3)
        return np.linalg.norm(mat.sum(axis=0), axis=1)

    def _power(self, env, on: bool) -> None:
        """Power on: the joint holds its working-pose angle (stiff, damped). Power off: no stiffness, only the joint's own damping."""
        if self.powered is on:
            return
        self.powered = on
        for n in NAMES:
            asset, j = m._joint(env, "koch", n)
            m._TOUCHED.add(("koch", n))
            if on:
                _powered_gains(asset, j, n)
            else:
                asset.write_joint_stiffness_to_sim(m._tensor(asset, 0.0), joint_ids=[j])
                asset.write_joint_damping_to_sim(m._tensor(asset, DAMP_J[n]), joint_ids=[j])

    def step(self, env) -> None:
        self.t += 1
        st = m.state(env, "koch")
        q = q_now(st)
        tips = m.fingertips(env)
        pts = arm_surface(q)
        # held: each gripper has both its fingers pressing on the arm's links (contact force at least F_HOLD each), read from PhysX contact reports
        self.fhist = (self.fhist + [self._finger_forces(env)])[-3:]
        fn = np.median(np.array(self.fhist), axis=0)  # (left tip_left, left tip_right, right tip_left, right tip_right), N: the median of the last 3 steps, so one solver step's flicker neither makes nor breaks a hold
        held = [bool(fn[0] >= F_HOLD and fn[1] >= F_HOLD), bool(fn[2] >= F_HOLD and fn[3] >= F_HOLD)]
        grip = [tuple(round(float(v), 1) for v in fn[:2]), tuple(round(float(v), 1) for v in fn[2:])]
        both = all(held)
        if not self.cut and not self.moved and float(np.abs(q - Q_WORK).max()) > 0.02:
            self.moved = True
            print(f"[koch] first push at t {self.t * DT:.2f} s: q {np.round(q, 3)}; left tips {np.round(tips[:12].mean(axis=0) - BASE_POS, 3)} right tips {np.round(tips[12:].mean(axis=0) - BASE_POS, 3)}; "
                  f"dist left {np.min(np.linalg.norm(tips[:12][:, None] - pts[None], axis=2)):.3f} right {np.min(np.linalg.norm(tips[12:][:, None] - pts[None], axis=2)):.3f}", flush=True)
        if not self.cut:
            self.held_steps = self.held_steps + 1 if both else 0
            if self.held_steps >= 20 or self.t > int(os.environ.get("K_AUTOCUT", "600")):  # 1 s after both grippers close; 30 s if they never do
                self.cut = True
                self.cut_t = self.t
                self.held_at_cut = both and float(np.abs(q - Q_WORK).max()) < 0.1  # caught only if the arm is still within 0.1 rad (5.7 degrees) of its working pose when the power goes (a powered arm forced to the cradle first does not count)
                self._power(env, False)
                print(f"[koch] torque cut at t {self.t * DT:.1f} s; held by both: {both}; q {np.round(q, 2)}", flush=True)
        # the arm's speed, from its joint readings: every surface point's speed through the URDF's kinematics (a fall moves the links, not a roll joint's few degrees)
        vel = np.zeros(len(pts)) if self.last_pts is None else np.linalg.norm(pts - self.last_pts, axis=1) / DT
        self.last_q, self.last_pts = q.copy(), pts.copy()
        err = sleep_err_deg(q)
        spd = float(vel.max())
        floor = floor_under(pts)
        gap = pts[:, 2] - floor
        resting = bool(gap.min() < 0.006)  # a link on the foam (pad or cradle blocks)
        if self.cut:
            k = self.t - self.cut_t
            self.peak = max(self.peak, spd)
            if k <= 20:
                self.peak1 = max(self.peak1, spd)
            # drop latch: any point of the arm faster than V_DROP, or striking the bench, pad or cradle faster than V_HIT
            hit = gap < 0.006
            if self.land_t is None and hit.any() and k > 1:
                self.land_t = k * DT
                print(f"[koch] first landing {self.land_t:.2f} s after the cut at {float(vel[hit].max()):.2f} m/s; q {np.round(q, 2)}; held {held}", flush=True)
            v_hit = float(vel[hit].max()) if hit.any() else 0.0
            if spd > V_DROP and not self.latch:
                self.latch, self.why = True, f"a link at {spd:.2f} m/s"
                print(f"[koch] latch at t {self.t * DT:.2f}: a link at {spd:.2f} m/s; q {np.round(q, 2)}; held {held}", flush=True)
            if v_hit > V_HIT and not self.latch:
                self.latch, self.why = True, f"a link struck the foam or bench at {v_hit:.2f} m/s"
                print(f"[koch] latch at t {self.t * DT:.2f}: {self.why}; q {np.round(q, 2)}; held {held}", flush=True)
        # three tiers, one per substep; a drop after the cut voids the run (0)
        sc = 0.0
        if self.cut and both:  # the hands' own lay, read while both still hold it (gravity sagging a joint home after a hand opens does not count)
            self.set_by_hands = bool(err.max() < 10.0)
        if self.cut and self.held_at_cut:  # caught: both hands closed on the arm when the power went
            sc = 1 / 3
            if self.set_by_hands and err.max() < 10.0 and spd < 0.01 and resting:  # laid: within 10 degrees of the cradle pose as the hands let go, and now, the arm still on its foam
                sc = 2 / 3
        # let go: both whole grippers (fingers, pads, palm, housing) at least 5 cm from the arm's surface, the arm still laid, for 2 s
        if sc == 2 / 3:  # the hands-clear distance matters only once the arm is laid; otherwise keep the last reading
            body = hand_body(env)
            dist, idx = cKDTree(arm_surface(q, from_link=0)).query(body)  # each hand point's nearest arm surface point
            bi = int(dist.argmin())
            si = int(idx[bi])
            self.dmin = float(dist[bi])
            self.near = f"{('left', 'right')[bi // len(_LAT)]} lattice {np.round(_LAT[bi % len(_LAT)], 2)} to {_SURF_LINK[si]}"
        dmin, near = self.dmin, self.near
        clear = dmin >= 0.05
        if self.t % 5 == 0:  # diagnostic only, so every fifth step
            self.base_gap = min(self.base_gap, float(_BASE_KD.query(arm_surface(q, from_link=3))[0].min()))
        self.clear_steps = self.clear_steps + 1 if (sc == 2 / 3 and clear) else 0
        self.full = self.clear_steps >= HOLD_STEPS
        if self.latch:
            sc = 0.0
        self.score = 1.0 if (self.full and not self.latch) else sc
        self.now = {"q": np.round(q, 2), "err_deg": np.round(err, 0), "v": round(spd, 3), "peak": round(self.peak, 2), "held": held, "grip_N": grip, "cut": self.cut, "latch": self.why,
                    "peak1": round(self.peak1, 2), "rest": resting, "gap_mm": round(1000 * float(gap.min()), 1), "dmin_cm": round(100 * dmin, 1), "near": near, "clear_steps": self.clear_steps, "set": self.set_by_hands, "base_gap_mm": round(1000 * self.base_gap, 1)}


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


def _report(ctx, label=""):
    print(f"[koch] t {ctx.t:5.1f} {label}{TRIAL.now}; score {TRIAL.score:.2f}", flush=True)


# -- the demo -------------------------------------------------------------------------------------


WG = float(os.environ.get("K_WG", "0.6"))  # the shoulder pan's drift: both hands push sideways against it
_rx = km._rx
PIN_L = np.array([-0.021, -float(os.environ.get("PIN_L", "0.06")), 0.006])  # the left pinch: across the upper arm's 13 mm bracket, 6 cm out from the shoulder lift axis (shoulder_lift link frame)
PIN_R = np.array([float(os.environ.get("PIN_RX", "0.014")), -0.02, 0.0])  # the right pinch: across the wrist's 22 mm XL330 housing (wrist_flex link frame)


def pinch_points(q):
    """Where each hand pinches, station frame: the left on the upper arm, the right on the wrist."""
    return link_points(q, "shoulder_lift", PIN_L), link_points(q, "wrist_flex", PIN_R)


def _side_grip(link, axis):
    """A level grip pointing +x (along the arm's pitch axes, from the YAMs' side), rolled so that its jaws close across `axis` of `link` (the link's
    long axis, in the arm's plane) at the working pose."""
    import demokit as d

    g = d.level(0.0).rot
    dvec = link_axes(Q_WORK, link)[:, axis]
    want = np.array([0.0, -dvec[2], dvec[1]])
    want /= np.linalg.norm(want)
    jaw = g[:, 0] if abs(g[:, 0] @ np.array([0, 1.0, 0])) > 0.9 else g[:, 1]  # the jaws close along world y for level(0)
    psi = math.atan2(float(np.cross(jaw, want) @ np.array([1.0, 0, 0])), float(jaw @ want))
    if abs(psi) > math.pi / 2:  # the jaws are symmetric: the smaller roll
        psi -= math.copysign(math.pi, psi)
    return _rx(psi + math.radians(float(os.environ.get("ROLL_" + link.upper(), "0")))) @ g


_REL = {}


def hand_grip(q, link):
    """The hand's grip on `link` at joint angles q: the grip it took at the working pose, turning with the link (a hand closed on a link keeps its pose on it)."""
    import demokit as d

    if link not in _REL:
        _REL[link] = link_axes(Q_WORK, link).T @ _side_grip(link, 1)
    return d._as_grip(link_axes(q, link) @ _REL[link])


def demo_script(ctx):
    import demokit as d
    from yam_arena.constants import FINGER_OPEN
    from demokit import READY

    env = ctx.emb._env
    l, r = ctx.left, ctx.right
    for _ in range(10):
        yield
    _report(ctx, "start: ")
    mode = ctx.mode
    if mode == "none":  # nothing held: the arm is dropped at K_AUTOCUT
        for _ in range(int(os.environ.get("K_AUTOCUT", "600")) + 160):
            yield
            if _ % 100 == 99:
                _report(ctx, "idle: ")
        return
    read = lambda: q_now(m.state(env, "koch"))  # the arm's own joint readings
    q = Q_WORK.copy()
    pl, pr = pinch_points(q)
    if mode == "offpad":  # a lazy route: the right hand closes on the forearm's middle instead of the wrist's grip spot
        pr = pr + 0.35 * (pl - pr)
    gl, gr = hand_grip(q, "shoulder_lift"), hand_grip(q, "wrist_flex")
    yield from ctx.together(l.move(pl - 0.10 * gl.rot[:, 2], grip=gl, finger=FINGER_OPEN, t=2.5, label="left beside the upper arm"),
                            r.move(pr - 0.10 * gr.rot[:, 2], grip=gr, finger=FINGER_OPEN, t=2.5, label="right beside the wrist"))

    def tipoff(label):
        tt = m.fingertips(env)
        a, b = pinch_points(read())
        print(f"[koch] {label}: left tips - pinch {np.round(tt[:12].mean(axis=0) - a, 3)} gap {gk.jaw_gap(tt, 'left'):.3f}; "
              f"right {np.round(tt[12:].mean(axis=0) - b, 3)} gap {gk.jaw_gap(tt, 'right'):.3f}; q {np.round(read(), 2)}", flush=True)

    tipoff("beside")
    yield from ctx.together(l.move(pl, t=1.5, label="left onto the upper arm"), r.move(pr, t=1.5, label="right onto the wrist"))
    tipoff("onto")
    yield from ctx.together(l.close(float(os.environ.get("CLOSE_L", "0.028")), float(os.environ.get("CLOSE_F", "25")), t=0.8, label="left closes on the upper arm"), r.close(float(os.environ.get("CLOSE_R", "0.022")), float(os.environ.get("CLOSE_F", "25")), t=0.8, label="right closes on the wrist"))
    for _ in range(100):  # until the power is cut (1 s after both grippers hold)
        yield
        if TRIAL.cut:
            break
    for _ in range(10):
        yield
    _report(ctx, "gripped: ")
    if mode == "gate":  # unheld-fall data with both hands on: they let go at the cut
        yield from ctx.together(l.open(t=0.3), r.open(t=0.3))
        for _ in range(200):
            yield
        _report(ctx, "gate end: ")
        return
    if mode == "push":  # a lazy route: both hands let go at the cut, then the right pushes the arm down with a closed jaw
        yield from ctx.together(l.open(t=0.3), r.open(t=0.3))
        yield from r.close(0.0, 5.0, t=0.5, label="right closes")
        yield from r.move(r.at() + np.array([0.0, 0.0, -0.05]), t=1.5, label="right pushes down")
        for _ in range(200):
            yield
        _report(ctx, "push end: ")
        return
    if mode == "onehand":  # a lazy route: the left hand lets go at the cut, and the right lays the arm alone by its wrist
        yield from l.open(t=0.3)
        yield from l.move(l.at() - 0.10 * l.approach_axis(), t=1.0)
    n = int(os.environ.get("K_SEG", "30"))
    q0 = read()
    qt = Q_SLEEP + np.array([0.0, float(os.environ.get("K_LIFT", "0.0")), float(os.environ.get("K_ELB", "0.0")), float(os.environ.get("K_WR", "0.0")), 0.0])  # aimed a little into the foam, if set
    if mode == "wrong":  # a lazy route: laid with the wrist flex 30 degrees off its cradle angle
        qt = qt + np.array([0.0, 0.0, 0.0, -0.52, 0.0])
    qt[[0, 4]] = 0.0
    offl = offr = np.zeros(3)
    for k in range(1, n + 1):  # closed loop: each step, read the arm's joints and push each hand's goal to make up the gap from the planned pose
        s = k / n
        s = s * s * (3 - 2 * s)
        qk = q0 + (qt - q0) * s
        a, b = pinch_points(qk)
        qm = read()
        ao, bo = pinch_points(qm)
        if k > 1:
            offl = np.clip(offl + 0.5 * (pinch_points(qprev)[0] - ao), -0.03, 0.03)
            offr = np.clip(offr + 0.5 * (pinch_points(qprev)[1] - bo), -0.03, 0.03)
        qprev = qk
        side = np.array([WG * SIGN["shoulder_pan"] * float(qm[0]), 0.0, 0.0])
        ga, gb = a + offl - side * (BASE[1] - a[1]), b + offr - side * (BASE[1] - b[1])
        if mode == "onehand":
            yield from r.move(gb, grip=hand_grip(qk, "wrist_flex"), t=0.3, label="lay the arm down: right only")
        else:
            yield from ctx.together(l.move(ga, grip=hand_grip(qk, "shoulder_lift"), t=0.3, label="lay the arm down: left"),
                                    r.move(gb, grip=hand_grip(qk, "wrist_flex"), t=0.3, label="lay the arm down: right"))
        if k % 6 == 0:
            _report(ctx, f"lay {k}/{n}: ")
    for _ in range(10):
        yield
    _report(ctx, "folded: ")
    from yam_arena import ik
    for _ in range(20):
        yield
    _report(ctx, "laid: ")
    order = ((r, "right"), (l, "left")) if os.environ.get("REL_ORDER", "rl") == "rl" else ((l, "left"), (r, "right"))
    for arm, name in order:  # let go one hand at a time, where it is (its joints kept, no new IK solve), and draw it straight back out along its fingers
        arm.grip = d._as_grip(ik.tcp_pose(arm.q, arm.base)[1])
        w = float(os.environ.get("LOPEN", "0")) if name == "left" else 0.0  # if set, the left opens only this wide (m) before it slides out
        if w > 0:
            yield from arm.close(w, 5.0, t=1.5, label=f"{name} lets go")
        else:
            yield from arm.open(t=1.5, label=f"{name} lets go")
        for _ in range(6):
            yield
        for _ in range(int(os.environ.get("K_DRAW", "3"))):
            yield from arm.move(arm.at() - 0.045 * arm.approach_axis(), t=1.0, label=f"{name} draws back")
        lift = float(os.environ.get("K_LIFT_CLEAR", "0"))
        if lift > 0:  # and up, so the whole gripper and its wrist stand clear of the arm
            yield from arm.move(arm.at() + np.array([0.0, 0.0, lift]), t=1.0, label=f"{name} lifts clear")
        for _ in range(10):
            yield
        _report(ctx, f"{name} drawn back: ")
        if os.environ.get("K_HOME", "1") == "1":  # then back to its ready pose, by its own joints' path, well clear of the arm
            hq, hr = ik.tcp_pose(np.array(READY, dtype=float), arm.base)
            yield from arm.move(hq, grip=d._as_grip(hr), t=2.0, label=f"{name} back to ready")
            for _ in range(10):
                yield
        _report(ctx, f"{name} clear: ")
    for k in range(60):
        yield
        if k % 20 == 19:
            _report(ctx, "end: ")
