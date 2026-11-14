"""Guide a Franka Research 3 arm by hand to its service pose with its Pilot-Grip's enabling button held in its middle band, then teach the pose.

A Franka Research 3 (FR3) stands on the far edge of the bench, stopped mid-task, its hand raised and turned away. It moves by hand only while two
buttons on its Pilot-Grip (on link 7) are held: the Enabling Button at its middle position and the Guiding Button. A gripper closing across the grip
presses both. Pressed too lightly, or through to the end, the arm holds; after a full press it moves again only after a full release and a new
half-press. At the service pose (the hand down over the tool stand) the guiding hand lets the buttons go, the arm stops and holds, and the other hand
presses Teach, on the Pilot-Disc on top of the wrist (link 6), to store the pose.

Model: the FR3's seven links and joints from Franka's franka_description (Apache-2.0: collision meshes, kinematics and joint limits), on a fixed
base; the Franka Hand as its collision mesh, fixed to the flange. The links carry no weight (Free mode is gravity-compensated) and each joint has
damping. Declared code: the enabling logic. While the Enabling Button sits in its middle band (1.5 to 4.0 mm of its 6 mm travel) and the Guiding
Button is in (over 1 mm), the joints are free; otherwise each joint is held where it is (a stiff hold). A press past 4.5 mm stops the arm until the
Enabling Button is let out below 1.0 mm. The buttons are sprung slides (approximated forces, to be measured); the Teach button counts when pressed
in 2 mm.

    python demokit.py task.py OUT --mode full
    python demokit.py task.py OUT --no-record --mode hard     # cheat checks: a full squeeze (the arm stops); no buttons, pushed; the arms wait
    python demokit.py task.py OUT --no-record --mode push
    python demokit.py task.py OUT --no-record --mode none
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import gatekit as gk  # noqa: E402
import mechkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "The Franka arm stopped mid-task. Guide it by hand with its grip's buttons to its service pose: the hand pointing straight down over the tool stand, its fingers about 5 cm above it, the grip turned toward the right-hand arm. Let the buttons go there, press Teach, then let go."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own 1 s hold
HOLD_STEPS = 20
TOP = TABLE_TOP[SCENE]
HERE = pathlib.Path(__file__).parent

# -- the FR3's kinematics (franka_description robots/fr3/kinematics.yaml: x y z roll pitch yaw per joint; joint 8 is the flange)
J = [(0, 0, 0.333, 0, 0, 0), (0, 0, 0, -math.pi / 2, 0, 0), (0, -0.316, 0, math.pi / 2, 0, 0), (0.0825, 0, 0, math.pi / 2, 0, 0),
     (-0.0825, 0.384, 0, -math.pi / 2, 0, 0), (0, 0, 0, math.pi / 2, 0, 0), (0.088, 0, 0, math.pi / 2, 0, 0), (0, 0, 0.107, 0, 0, 0)]
LO = np.array([-2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508])  # joint_limits.yaml
HI = np.array([2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508])
Q_REF = np.array([0.0, 0.0, 0.0, -math.pi / 2, 0.0, math.pi / 2, math.pi / 4])  # the model's zero: every joint axis, and the grip's axes, along the base's
Q_SERVICE = np.array([0.0, 0.18, 0.0, -2.656, 0.0, 2.838, -math.pi / 2])  # the hand straight down 18 cm over the bench, the grip toward the right YAM
Q_START = np.array([0.117, -0.083, 0.119, -2.614, 0.017, 2.532, -1.749])  # stopped mid-task: the hand 10 cm higher, turned away 0.4 rad
BASE = (0.80, 0.0)  # on the bench's far edge, turned to face the YAMs (its +x along the station's -x)
LINKS = [f"link{i}" for i in range(1, 8)]

# -- the Pilot-Grip on link 7 (its frame: z along the flange axis): a handle along link 7's (x+y) diagonal, its buttons on its two flanks
GRIP_R, GRIP_Z = 0.080, 0.078  # its middle: 80 mm out from link 7's axis along the diagonal, 78 mm along it
GRIP_W = 0.060  # across its flanks (Franka's visual mesh: 63 to 73 mm with the buttons)
BTN_R, BTN_TRAVEL = 0.009, 0.006
EN_K = float(os.environ.get("EN_K", "2500"))  # N/m: 7.5 N at the middle position (3 mm), 15 N at the full one (6 mm); approximated
GD_K, TEACH_K = 1500.0, 1500.0
EN_BAND, EN_FULL, EN_OUT = (0.0015, 0.0040), 0.0045, 0.0010
GUIDE_IN, TEACH_IN = 0.001, 0.002
PILOT6 = np.array([0.0736, 0.0794, -0.0022])  # the Pilot-Disc's middle on link 6's top (its frame): the face that points up at the service pose
PILOT_R, PILOT_T = 0.022, 0.003  # the disc: 44 mm across, 3 mm proud of the link
TEACH_R, TEACH_TRAVEL = 0.008, 0.004  # Teach, in the disc's middle: 16 mm across, 4 mm of travel
DAMP = float(os.environ.get("FR3_DAMP", "2.0"))  # N m s/rad per joint in Free mode (to be set from a real FR3's drag)


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def fk(q):
    """Link 0 to link 7 and the flange, in the FR3's base frame, and each joint's origin and axis."""
    T = np.eye(4)
    out, joints = [T.copy()], []
    for i in range(8):
        x, y, z, r, p, yw = J[i]
        A = np.eye(4)
        A[:3, :3], A[:3, 3] = _rpy(r, p, yw), (x, y, z)
        T = T @ A
        if i < 7:
            joints.append((T[:3, 3].copy(), T[:3, 2].copy()))
            R = np.eye(4)
            R[:3, :3] = _rz(q[i])
            T = T @ R
        out.append(T.copy())
    return out, joints


def _read_stl(name):
    raw = (HERE / name).read_bytes()
    n = int(np.frombuffer(raw[80:84], "<u4")[0])
    return np.frombuffer(raw[84:84 + n * 50], np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))["v"].astype(float)


def _mesh(tri, T):
    v = tri.reshape(-1, 3) @ T[:3, :3].T + T[:3, 3]
    key = np.round(v * 1e6).astype(np.int64)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    return m.Mesh(uniq / 1e6, inv.reshape(-1, 3))


def _axis_name(a):
    k = int(np.argmax(np.abs(a)))
    assert abs(abs(a[k]) - 1.0) < 1e-6, a
    return "XYZ"[k], float(np.sign(a[k]))


REF_T, REF_J = fk(Q_REF)
SIGNS = [_axis_name(a)[1] for _, a in REF_J]
T7 = REF_T[7]
DIAG = T7[:3, :3] @ (np.array([1.0, 1.0, 0.0]) / math.sqrt(2))  # the grip's long axis, base frame, at the model's zero
FLANK = T7[:3, :3] @ (np.array([1.0, -1.0, 0.0]) / math.sqrt(2))  # across its two buttons
AX7 = T7[:3, 2]
TEACH_N = REF_T[6][:3, :3] @ np.array([0.0, 1.0, 0.0])  # out of link 6's top, at the model's zero (along the base's z)


def _link7_world(q):
    return fk(q)[0][7]


# which flank faces the left YAM at the service pose (the station's front-left): the Enabling Button goes on the grip's flank on that side, the
# Guiding Button opposite (Teach is on the Pilot-Disc on link 6's top)
_Ts = _link7_world(Q_SERVICE)
_n_serv = _Ts[:3, :3] @ (np.array([1.0, -1.0, 0.0]) / math.sqrt(2))  # base frame
_n_station = np.array([-_n_serv[0], -_n_serv[1]])  # the base is turned 180 degrees
FRONT_LEFT = 1.0 if (_n_station[1] > 0 and _n_station[0] < 0) else -1.0

LOOKS = {
    "white": m.Look(color=m.rgb(0.90, 0.90, 0.88), roughness=0.4),
    "grey": m.Look(color=m.rgb(0.32, 0.33, 0.35), roughness=0.5),
    "black": m.Look(color=m.rgb(0.08, 0.08, 0.09), roughness=0.6),
    "enable": m.Look(color=m.rgb(0.20, 0.22, 0.24), roughness=0.4),
    "guide": m.Look(color=m.rgb(0.55, 0.57, 0.60), roughness=0.4),
    "teach": m.Look(color=m.rgb(0.10, 0.45, 0.80), roughness=0.4),
    "stand": m.Look(color=m.rgb(0.45, 0.35, 0.25), roughness=0.7),
}


def _box(c, size, name, look, parts, cols, visual=True):
    if visual:
        parts.append((name, m.box(size, c), look))
    cols.append(m.col_box(name + "_c", size, c))


def _rotated_box(center, u, v, w, size, name, look, parts, cols):
    """A box with its sides along u, v, w (all along the base's axes at the model's zero): sizes given along u, v, w."""
    full = np.abs(np.outer(u, [1, 0, 0]) * size[0] + np.outer(v, [0, 1, 0]) * size[1] + np.outer(w, [0, 0, 1]) * size[2]).sum(axis=1)
    _box(tuple(center), tuple(full), name, look, parts, cols)


def _fr3():
    """The arm as a mechanism in its base frame, posed at Q_REF."""
    frame_parts, frame_cols = [], []
    tri0 = _read_stl("fr3_link0.stl")
    frame_parts.append(("link0", _mesh(tri0, REF_T[0]), "white"))
    frame_cols.append(m.col_hull("link0_c", _mesh(tri0, REF_T[0])))
    links = []
    for i in range(1, 8):
        T = REF_T[i]
        tri = _read_stl(f"fr3_link{i}.stl")
        parts = [(f"shell{i}", _mesh(tri, T), "white" if i not in (6,) else "grey")]
        if i == 7:  # the body only (the grip is its own box below): a hull round the whole link would stand proud of the grip's buttons
            body = tri[(tri @ (np.array([1.0, 1.0, 0.0]) / math.sqrt(2))).max(axis=1) < 0.046]
            cols = [m.col_hull("link7_c", _mesh(body, T))]
        else:
            cols = [m.col_hull(f"link{i}_c", _mesh(tri, T))]
        if i == 6:  # the Pilot-Disc on link 6's top face
            disc = m.cylinder(PILOT_R, PILOT_T, tuple(PILOT6 + np.array([0.0, PILOT_T / 2, 0.0])), axis="y", segments=28)
            disc = m.Mesh(disc.v @ T[:3, :3].T + T[:3, 3], disc.f)
            parts.append(("pilot", disc, "black"))
            cols.append(m.col_hull("pilot_c", disc))
        origin, axis = REF_J[i - 1]
        name, s = _axis_name(axis)
        lo, hi = sorted((math.degrees(s * (LO[i - 1] - Q_REF[i - 1])), math.degrees(s * (HI[i - 1] - Q_REF[i - 1]))))
        if i == 7:
            # the Franka Hand on the flange (turned -45 degrees, as franka_description mounts it) and its two fingers
            Th = REF_T[8] @ np.block([[_rz(-math.pi / 4), np.zeros((3, 1))], [np.zeros((1, 3)), np.ones((1, 1))]])
            hand = _mesh(_read_stl("fr3_hand.stl"), Th)
            parts.append(("hand", hand, "white"))
            cols.append(m.col_hull("hand_c", hand))
            for k, sy in enumerate((-1, 1)):
                fc = (Th @ np.array([0.0, sy * 0.012, 0.088, 1.0]))[:3]
                _rotated_box(fc, Th[:3, 0], Th[:3, 1], Th[:3, 2], (0.018, 0.008, 0.045), f"finger{k}", "grey", parts, cols)
            # the Pilot-Grip: a handle on link 7 along its diagonal
            gc = T[:3, 3] + DIAG * GRIP_R + AX7 * GRIP_Z
            _rotated_box(gc, DIAG, FLANK, AX7, (0.070, GRIP_W, 0.040), "grip", "grey", parts, cols)
        links.append(m.hinge(f"link{i}", name, tuple(float(v) for v in origin), lo, hi, parts, cols, parent=None if i == 1 else f"link{i - 1}",
                             mass=1.0, spring=0.0, damping=DAMP, friction=0.0, effort=200.0, inertia=(0.01, 0.01, 0.01)))
    # the buttons: sprung slides on link 7, each pressed in toward the grip's middle line or the link's axis
    T = REF_T[7]
    gc = T[:3, 3] + DIAG * GRIP_R + AX7 * GRIP_Z
    for nm, side, k, look in (("enable", FRONT_LEFT, EN_K, "enable"), ("guide", -FRONT_LEFT, GD_K, "guide")):
        n = side * FLANK
        c = gc + n * (GRIP_W / 2 + BTN_TRAVEL / 2)
        a, sg = _axis_name(n)
        cap = m.cylinder(BTN_R, BTN_TRAVEL, (0.0, 0.0, 0.0), axis=a.lower(), segments=20).moved(tuple(c))
        lo, hi = (-BTN_TRAVEL, 0.0) if sg > 0 else (0.0, BTN_TRAVEL)
        links.append(m.slider(nm, a, lo, hi, [("cap", cap, look)], [m.col_hull(nm + "_c", cap)],
                              at=tuple(c), parent="link7", mass=0.01, spring=k, damping=1.0, friction=0.0))
    T6 = REF_T[6]  # the Teach button: in the Pilot-Disc's middle, on link 6's top, pressed down into the link
    c = T6[:3, :3] @ (PILOT6 + np.array([0.0, PILOT_T + TEACH_TRAVEL / 2, 0.0])) + T6[:3, 3]
    a, sg = _axis_name(TEACH_N)
    cap = m.cylinder(TEACH_R, TEACH_TRAVEL, (0.0, 0.0, 0.0), axis=a.lower(), segments=20).moved(tuple(c))
    lo, hi = (-TEACH_TRAVEL, 0.0) if sg > 0 else (0.0, TEACH_TRAVEL)
    links.append(m.slider("teach", a, lo, hi, [("cap", cap, "teach")], [m.col_hull("teach_c", cap)], at=tuple(c), parent="link6", mass=0.01,
                          spring=TEACH_K, damping=1.0, friction=0.0))
    # a plinth under the base
    _box((-0.03, 0.0, -0.006), (0.26, 0.22, 0.012), "plinth", "black", frame_parts, frame_cols)
    start = [(f"link{i}", float(SIGNS[i - 1] * (Q_START[i - 1] - Q_REF[i - 1]))) for i in range(1, 8)]
    return m.mechanism("fr3", (BASE[0], BASE[1], TOP + 0.012), links, LOOKS, frame_parts=frame_parts, frame_colliders=frame_cols, yaw_deg=180.0,
                       friction=(0.8, 0.7), solver_iters=64, start=start)


def _with_start_state(part):
    """The FR3's USD with each joint's start angle authored as its joint state, so the arm is at its start pose from the first frame (the harness's
    reset does not write a mechanism's joints)."""
    import dataclasses
    build0 = part.build

    def build():
        txt = build0()
        for i in range(1, 8):
            deg = math.degrees(SIGNS[i - 1] * (Q_START[i - 1] - Q_REF[i - 1]))
            head = f'def PhysicsRevoluteJoint "link{i}_joint" (\n        prepend apiSchemas = ["PhysicsDriveAPI:angular"]'
            k = txt.index(head)
            txt = txt[:k] + head.replace('["PhysicsDriveAPI:angular"]', '["PhysicsDriveAPI:angular", "PhysicsJointStateAPI:angular"]') + txt[k + len(head):]
            anchor = 'uniform token drive:angular:physics:type = "force"'
            j = txt.index(anchor, k)
            txt = txt[:j] + f"float state:angular:physics:position = {deg:.6f}\n        float state:angular:physics:velocity = 0\n        " + txt[j:]
        return txt

    return dataclasses.replace(part, build=build)


FR3 = _with_start_state(_fr3())
_sp = [("stand", m.box((0.10, 0.10, 0.03), (0.0, 0.0, 0.015)), "stand")]
STAND_XY = (0.38, 0.0)  # the tool stand, under the service pose's hand
STAND = m.fixed("tool_stand", (STAND_XY[0], STAND_XY[1], TOP), _sp, [m.col_box("stand_c", (0.10, 0.10, 0.03), (0.0, 0.0, 0.015))], LOOKS)
OBJECTS = [FR3, STAND]


# -- reading the arm --------------------------------------------------------------------------

def q_now(st) -> np.ndarray:
    return np.array([Q_REF[i] + SIGNS[i] * float(st.joint(f"link{i + 1}")) for i in range(7)])


def to_station(p) -> np.ndarray:
    """A point in the FR3's base frame, in the station frame (the base turned 180 degrees)."""
    return np.array([BASE[0] - p[0], BASE[1] - p[1], TOP + 0.012 + p[2]])


def flange(q) -> tuple[np.ndarray, np.ndarray]:
    T = fk(q)[0][8]
    R180 = np.diag([-1.0, -1.0, 1.0])
    return to_station(T[:3, 3]), R180 @ T[:3, :3]


SERVICE_POS, SERVICE_R = flange(Q_SERVICE)


def presses(st) -> dict:
    def travel(nm):
        a, sg = _axis_name(TEACH_N if nm == "teach" else (FRONT_LEFT if nm == "enable" else -FRONT_LEFT) * FLANK)
        return -sg * float(st.joint(nm))  # pressed in, metres
    return {"enable": travel("enable"), "guide": travel("guide"), "teach": travel("teach")}


def _samples(tri):
    """Points over a mesh's surface: its corners, its edges' middles and its faces' middles. A fingertip's distance to them is never less than its
    distance to the surface, and at most about 3 cm more on the Hand's largest faces (1 to 2 cm on the links')."""
    pts = [tri.reshape(-1, 3), tri.mean(axis=1), (tri[:, 0] + tri[:, 1]) / 2, (tri[:, 1] + tri[:, 2]) / 2, (tri[:, 2] + tri[:, 0]) / 2]
    v = np.concatenate(pts)
    return np.unique(np.round(v, 4), axis=0)


def _box_samples(c, u, v, w, size, n=4):
    """Points over a box's surface: a grid on each face. u, v, w are its axes and size its lengths along them."""
    g = np.linspace(-0.5, 0.5, n)
    grid = np.array([[i, j, k] for i in g for j in g for k in g if max(abs(i), abs(j), abs(k)) == 0.5])
    return np.asarray(c) + grid[:, :1] * size[0] * u + grid[:, 1:2] * size[1] * v + grid[:, 2:] * size[2] * w


SURF = {i: _samples(_read_stl(f"fr3_link{i}.stl")) for i in range(2, 8)}  # each link's surface, in its own frame
_hand_tri = _read_stl("fr3_hand.stl")
HAND_SURF = _samples(_hand_tri)  # in the hand's frame (the flange turned -45 degrees)
_g = np.array([1.0, 1.0, 0.0]) / math.sqrt(2)
_f = np.array([1.0, -1.0, 0.0]) / math.sqrt(2)
L7_EXTRA = _box_samples(_g * GRIP_R + np.array([0, 0, GRIP_Z]), _g, _f, np.array([0.0, 0.0, 1.0]), np.array([0.070, GRIP_W + 2 * BTN_TRAVEL, 0.040]))  # link 7's frame: the grip and its buttons
L6_EXTRA = np.array([PILOT6 + np.array([dx, PILOT_T + TEACH_TRAVEL, dz]) for dx in (-PILOT_R, 0.0, PILOT_R) for dz in (-PILOT_R, 0.0, PILOT_R)])  # the Pilot-Disc and Teach
FINGER_SURF = np.concatenate([_box_samples(np.array([0.0, sy * 0.012, 0.088]), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0]),
                                           np.array([0.018, 0.008, 0.045])) for sy in (-1, 1)])  # in the hand's frame


def arm_surface(q):
    """Points over the arm's surface from link 2 out, its hand and fingers, the grip and its buttons, the Pilot-Disc and Teach, in the station frame."""
    out, _ = fk(q)
    pts = [SURF[i] @ out[i][:3, :3].T + out[i][:3, 3] for i in range(2, 8)]
    pts.append(L7_EXTRA @ out[7][:3, :3].T + out[7][:3, 3])
    pts.append(L6_EXTRA @ out[6][:3, :3].T + out[6][:3, 3])
    Th = out[8] @ np.block([[_rz(-math.pi / 4), np.zeros((3, 1))], [np.zeros((1, 3)), np.ones((1, 1))]])
    pts.append(HAND_SURF @ Th[:3, :3].T + Th[:3, 3])
    pts.append(FINGER_SURF @ Th[:3, :3].T + Th[:3, 3])
    P = np.concatenate(pts)
    return np.stack([BASE[0] - P[:, 0], BASE[1] - P[:, 1], TOP + 0.012 + P[:, 2]], axis=1)


STAND_TOP = 0.03  # the tool stand: 10 x 10 cm, 3 cm tall


class Trial:
    def __init__(self) -> None:
        self.reset()

    def reset(self, env=None) -> None:
        if env is not None and os.environ.get("FR3_DEBUG"):
            art = env.unwrapped.scene["fr3"]
            print(f"[fr3] dbg reset: joints {art.joint_names}; default {art.data.default_joint_pos}; now {art.data.joint_pos}; cfg {art.cfg.init_state.joint_pos}", flush=True)
        self.posed = False
        self.enabled = False
        self.stopped = False  # a full press stopped the arm: it needs the Enabling Button let out before it moves again
        self.locked = None  # the joints' hold, as last written
        self.moved_enabled = 0.0  # how far the flange moved while enabled
        self.last = None
        self.taught = False
        self.forced = 0  # steps a hand pushed the held arm (it moved more than 1 mm while not enabled)
        self.fault = ""
        self.hold, self.full = 0, False
        self.score = 0.0
        self.now: dict = {}

    def _hold(self, env, on: bool) -> None:
        if self.locked is on:
            return
        for nm in LINKS:
            (m.lock if on else m.unlock)(env, "fr3", nm, *((50000.0, 400.0) if on else ()))
        self.locked = on

    def step(self, env) -> None:
        if not self.posed:  # the USD carries the start pose: just hold the joints from the first step
            self.posed = True
            self.locked = None
        st = m.state(env, "fr3")
        q = q_now(st)
        b = presses(st)
        if b["enable"] > EN_FULL:
            if not self.stopped:
                print(f"[fr3] full press ({1000 * b['enable']:.1f} mm): the arm stops", flush=True)
            self.stopped = True
        if self.stopped and b["enable"] < EN_OUT:
            self.stopped = False
        en = (not self.stopped) and EN_BAND[0] <= b["enable"] <= EN_BAND[1] and b["guide"] > GUIDE_IN
        if en != self.enabled:
            print(f"[fr3] {'guiding' if en else 'held'}: enable {1000 * b['enable']:.1f} mm, guide {1000 * b['guide']:.1f} mm", flush=True)
        self.enabled = en
        self._hold(env, not en)
        p, R = flange(q)
        if self.last is not None:
            d = float(np.linalg.norm(p - self.last))
            if en:
                self.moved_enabled += d
            elif d > 0.001:
                self.forced += 1
            else:
                self.forced = 0  # 0.5 s running: a held arm pushed along more than 2 cm/s
        self.last = p
        if self.forced > 10 and not self.fault:  # 0.5 s of a held arm pushed along
            self.fault = "the held arm forced"
        if np.any(q <= LO + 0.005) or np.any(q >= HI - 0.005):
            self.fault = self.fault or "a joint at its limit"
        pts = arm_surface(q)
        if pts[:, 2].min() < TOP + 0.003:
            self.fault = self.fault or "the arm on the bench"
        on_stand = (np.abs(pts[:, 0] - STAND_XY[0]) < 0.05) & (np.abs(pts[:, 1] - STAND_XY[1]) < 0.05) & (pts[:, 2] < TOP + STAND_TOP + 0.003)
        if np.any(on_stand):
            self.fault = self.fault or "the arm on the tool stand"
        err = float(np.linalg.norm(p - SERVICE_POS))
        ang = float(np.degrees(np.arccos(np.clip((np.trace(R.T @ SERVICE_R) - 1) / 2, -1.0, 1.0))))
        at_pose = err < 0.015 and ang < 5.0
        held = not en and not self.stopped or (self.stopped and b["enable"] < EN_OUT)
        if at_pose and not en and b["enable"] < EN_BAND[0] and b["guide"] < GUIDE_IN and b["teach"] > TEACH_IN:
            if not self.taught:
                print(f"[fr3] taught at {1000 * err:.1f} mm, {ang:.1f} deg from the service pose", flush=True)
            self.taught = True
        tips = m.fingertips(env)
        hands = float(np.min(np.linalg.norm(gk.all_tips(tips)[:, None] - pts[None], axis=2)))  # every fingertip to the arm's surface
        mo = self.moment(err, ang, at_pose, en, b, hands)
        self.hold = self.hold + 1 if mo == 1.0 else 0
        self.full = self.hold >= HOLD_STEPS
        self.score = 1.0 if self.full else (0.75 if mo == 1.0 else mo)
        self.now = {"q": np.round(q, 3), "flange_mm": np.round(1000 * (p - SERVICE_POS), 1), "deg": round(ang, 1), "enabled": en, "stopped": self.stopped,
                    "buttons_mm": {k: round(1000 * v, 1) for k, v in b.items()}, "moved_mm": round(1000 * self.moved_enabled), "taught": self.taught,
                    "hands_mm": round(1000 * hands), "fault": self.fault, "held": held}

    def moment(self, err, ang, at_pose, en, b, hands) -> float:
        if self.fault:
            return 0.0
        if self.moved_enabled < 0.10:
            return 0.0
        if not (err < 0.05 and ang < 15.0):
            return 0.25
        if not (at_pose and not en and b["enable"] < EN_BAND[0] and b["guide"] < GUIDE_IN):
            return 0.5
        if not (self.taught and hands >= 0.05):
            return 0.75
        return 1.0


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


def _report(ctx, label=""):
    print(f"[fr3] t {ctx.t:5.1f} {label}{TRIAL.now}; score {TRIAL.score:.2f}", flush=True)


def grip_pose(q):
    """The Pilot-Grip's middle and its long axis and button axis, in the station frame."""
    out, _ = fk(q)
    T = out[7]
    c = T[:3, 3] + T[:3, :3] @ (np.array([1.0, 1.0, 0.0]) / math.sqrt(2) * GRIP_R + np.array([0, 0, GRIP_Z]))
    R180 = np.diag([-1.0, -1.0, 1.0])
    return to_station(c), R180 @ (T[:3, :3] @ (np.array([1.0, 1.0, 0.0]) / math.sqrt(2))), R180 @ (T[:3, :3] @ (np.array([1.0, -1.0, 0.0]) / math.sqrt(2)))


def teach_point(q):
    """Teach's top (released) and its outward normal, in the station frame."""
    out, _ = fk(q)
    T = out[6]
    n = np.array([0.0, 1.0, 0.0])
    return to_station(T[:3, 3] + T[:3, :3] @ (PILOT6 + n * (PILOT_T + TEACH_TRAVEL))), np.diag([-1.0, -1.0, 1.0]) @ (T[:3, :3] @ n)


def demo_script(ctx):
    """The right hand comes along the Pilot-Grip from its outer end, tilted 60 degrees, its fingers either side of the grip's flanks, and closes
    to the middle band: both buttons in, the arm free. It draws the grip in small steps along the path to the service pose, re-reading the grip's pose
    each step, then opens a little (the Enabling Button out, the arm held) and moves clear. The left hand, tilted 60 degrees from straight down, brings
    its closed fingers' ends over Teach on the Pilot-Disc (on link 6's top), presses it down 3 mm and moves clear."""
    import demokit as d
    from yam_arena.constants import FINGER_OPEN

    env = ctx.emb._env
    l, r = ctx.left, ctx.right
    for _ in range(10):
        yield
    _report(ctx, "start: ")
    if ctx.mode == "none":
        for _ in range(200):
            yield
        _report(ctx, "idle: ")
        return
    SQ_W, SQ_F = float(os.environ.get("SQ_W", "0.066")), float(os.environ.get("SQ_F", "7.5"))  # a close that holds the Enabling Button in its band
    q = q_now(m.state(env, "fr3"))
    g, long_ax, flank = grip_pose(q)
    phi = math.degrees(math.atan2(-long_ax[1], -long_ax[0]))
    gr = d.tilted(60.0, phi)
    approach = -long_ax  # along the grip from its outer end
    if ctx.mode == "push":  # cheat check: the arm pushed by its link 7, no buttons
        yield from r.move(g - 0.08 * approach + np.array([0, 0, 0.05]), grip=gr, finger=0.0, t=2.0, label="beside the grip, fingers closed")
        yield from r.move(g - 0.02 * approach, t=1.0, label="against the grip")
        yield from r.move(r.at() + np.array([0.0, 0.05, -0.05]), t=2.0, label="push the arm")
        for _ in range(30):
            yield
        _report(ctx, "push end: ")
        return
    yield from r.move(g - 0.07 * approach + np.array([0, 0, 0.04]), grip=gr, finger=FINGER_OPEN, t=2.5, label="along the grip, open")
    yield from r.move(g - 0.07 * approach, t=1.0, label="level with the grip")
    yield from r.move(g, t=1.2, label="fingers either side of the grip")
    if ctx.mode == "hard":
        yield from r.close(0.040, 30.0, t=0.8, label="squeeze hard")
    else:
        yield from r.close(SQ_W, SQ_F, t=0.8, label="squeeze both buttons to the middle band")
    for _ in range(10):
        yield
    _report(ctx, "squeezed: ")
    if ctx.mode == "hard":
        yield from r.move(r.at() + np.array([0.0, 0.05, -0.05]), t=2.0, label="draw with the arm stopped")
        for _ in range(20):
            yield
        _report(ctx, "hard end: ")
        return
    n = 24
    for k in range(1, n + 1):
        qk = Q_START + (Q_SERVICE - Q_START) * k / n
        gk_, lk, _ = grip_pose(qk)
        phik = math.degrees(math.atan2(-lk[1], -lk[0]))
        yield from r.move(gk_, grip=d.tilted(60.0, phik), t=0.35, label="draw the grip toward the service pose")
    for _ in range(3):  # finish on the flange's own error
        p, _R = flange(q_now(m.state(env, "fr3")))
        yield from r.move(r.at() + (SERVICE_POS - p), t=0.6, label="square the hand over the stand")
    _report(ctx, "at the pose: ")
    yield from r.open(t=0.5, label="let the buttons out: the arm holds")
    yield from r.move(r.at() - 0.10 * (-grip_pose(q_now(m.state(env, "fr3")))[1]) + np.array([0, 0, 0.05]), t=1.2, label="right hand back off the grip")
    tp, tn = teach_point(q_now(m.state(env, "fr3")))  # on the Pilot-Disc, on link 6's top, about 38 cm over the bench
    gt = d.tilted(float(os.environ.get("TEACH_TILT", "60")), float(os.environ.get("TEACH_YAW", "-60")))  # fingers 30 degrees below level: their ends lowest, the palm clear above
    yield from l.move(tp + np.array([0.0, 0.0, 0.06]), grip=gt, finger=0.0, t=2.5, label="over Teach, fingers closed")
    def ends():  # the middle of the closed fingers' ends: mechkit's six tip points (every other one of the left hand's 12)
        return m.fingertips(env)[0:12:2].mean(axis=0)

    for _ in range(2):  # the fingers' ends 2 cm over Teach's middle
        yield from l.move(l.at() + (tp + np.array([0.0, 0.0, 0.02]) - ends()), t=0.8, label="fingers over Teach")
    for k in range(32):  # down in 2.5 mm steps until Teach is 3 mm in, the fingers' ends kept over its middle
        if os.environ.get("FR3_DEBUG"):
            print(f"[fr3] dbg press {k}: teach {1000 * presses(m.state(env, 'fr3'))['teach']:.1f} mm, fingers' ends - Teach {np.round(1000 * (ends() - tp), 1)} mm",
                  flush=True)
        if presses(m.state(env, "fr3"))["teach"] > 0.003:
            break
        e = ends()
        yield from l.move(l.at() + np.array([tp[0] - e[0], tp[1] - e[1], -0.0025]), t=0.25, label="press Teach")
    for _ in range(6):
        yield
    _report(ctx, "teach pressed: ")
    yield from l.move(l.at() + np.array([0.0, 0.0, 0.06]), t=1.0, label="left hand up off Teach")
    yield from ctx.together(l.move(l.at() + np.array([-0.08, 0.06, 0.03]), t=1.0, label="clear"), r.move(r.at() + np.array([-0.08, -0.06, 0.03]), t=1.0, label="clear"))
    for k in range(40):
        yield
        if k % 20 == 19:
            _report(ctx, "end: ")
