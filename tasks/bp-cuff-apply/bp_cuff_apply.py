"""Put a blood-pressure cuff on a patient's left upper arm, ready for a reading, then let go.

Scene (station scene, metres; E = engineering estimate, see README):
- patient arm: a fixed display-mannequin left arm lying along x, its axis 0.10 m over the table on two props (a box under
  the shoulder end, a foam wedge under the forearm). Tapered upper arm (radius 55 -> 49 mm, 325 mm round the middle, E),
  elbow crease at x 0.35 (dark ring), forearm 35 -> 29 mm. A dark artery line runs along the top of the upper arm
  (x 0.17-0.35). The upper arm's underside is 45-51 mm over the table, clear between the props.
- cuff: an adult size-11 cuff, 520 x 140 x 3 mm, 0.10 kg (E), modelled as 26 rigid 20 mm segments joined by 25 hinges
  about x (bending stiffness BEND_K, E). It lies flat on the table across under the patient arm, skin face up, the start
  end on the +y side and the free end on the -y side. Each end segment is sprung curled up (REST_CURL on its hinge,
  stiffer, E): a STAGING AID so a down grip can pinch the cuff (a real flat cuff is picked up by its edge). Hook face =
  outer face of the first N_HOOK segments (start end); loop face = skin face of the last N_LOOP segments (free end). A
  white ARTERY arrow is printed on the outer face of segment
  MARK_SEG (on the outer turn, so it shows once the cuff is on), pointing to the cuff's lower edge.

Closure rule (the only scripted state, disclosed in task.json fidelity; precedent MERGED #2126 contact-gated attachment,
also #1717 #1465 #1862):
- ENGAGE when at least ENGAGE_OVERLAP (60 mm) of the loop face lies on the hook face within ENGAGE_GAP (6 mm) and a
  gripper with its fingers closed (each finger within FIST of closed) PRESSES that lying loop face: pad tip within
  PRESS_CONTACT (2 mm) AND the loop segment under it pushed >= PRESS_DEPTH (0.15 mm) toward the arm since the pad
  arrived -- the cuff's own displacement under the fist is the press (the runner exposes no contact force,
  DEVIATIONS 14-15) -- sustained for PRESS_DWELL_S (0.3 s). A pad hovering or resting without pushing does not close. The fastening is the hinge set that spans the
  overlap: those hinges get a stiff drive (K_LOCK) at the engaged angle and every other hinge keeps its fabric stiffness,
  so the closed cuff keeps its ring through the closure, not through a stiffened frame.
- RELEASE when, with no hand on the closure, the loop face is peeled > PEEL_LIFT (15 mm) off the hook face with
  > PEEL_FORCE (4 N), or the ring is sheared by > SHEAR_FORCE (10 N), both read from the BONDED hinges' deflection, for
  RELEASE_STEPS (2) control steps; the hinges go back to fabric. The joint arrays are written on state edges only.

Run it:
  inspect-robots run --embodiment yam_arena -E task=bp_cuff_apply.py -E object_jitter=0 \\
    -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<see task.json>" --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

_HERE = str(pathlib.Path(__file__).resolve().parent)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import arena_assets_fix  # noqa: E402, F401
import labkit  # noqa: E402
from labkit import Joint, PartState, on_reset, on_step, part  # noqa: E402
from pitch_demo import Waypoint  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402

SCENE = "station"
TOP = station.TABLE_TOP

# -- colours ----------------------------------------------------------------------------------------------------------
SKIN, PROP, DARK = (0.85, 0.72, 0.60), (0.50, 0.50, 0.55), (0.25, 0.12, 0.12)
NAVY, WHITE, HOOK_C = (0.10, 0.16, 0.38), (0.95, 0.95, 0.95), (0.55, 0.57, 0.62)

# -- patient arm (fixed) ------------------------------------------------------------------------------------------------
H_AXIS = 0.10  # arm axis over the table
ARM_X0 = -0.08  # the arm part's origin x: arm shapes below are in its frame
AX = TOP + H_AXIS
CREASE_LX = 0.43  # elbow crease, arm frame
CREASE_X = ARM_X0 + CREASE_LX  # elbow crease, world
# (x centre, radius, length): upper arm 55 -> 49 mm (mannequin-profile estimate, 325 mm circumference mid), elbow, forearm
ARM_SEGS = ((0.140, 0.058, 0.04), (0.190, 0.055, 0.06), (0.260, 0.053, 0.08), (0.340, 0.051, 0.08), (0.405, 0.049, 0.05),
            (0.445, 0.042, 0.03), (0.520, 0.035, 0.12), (0.640, 0.029, 0.12))
ARTERY_X = (0.25, 0.43)
ARM_MU, CUFF_MU = 0.6, 0.7  # skin, cuff fabric (E)
LINE_T, LINE_W = 0.0008, 0.004  # drawn lines: 0.8 mm proud, 4 mm wide


def arm_radius(x: float) -> float:
    """Skin radius of the patient arm at x (0 off the modelled arm)."""
    for cx, r, ln in ARM_SEGS:
        if abs(x - ARM_X0 - cx) <= ln / 2 + 1e-9:
            return r
    return 0.0


def _arm_shapes():
    cyl = [((cx, 0.0, 0.0), r, ln, SKIN, "X") for cx, r, ln in ARM_SEGS]
    boxes = []
    for cx, r, ln in ARM_SEGS:  # artery line on the top of the upper arm, one strip per radius step
        x0, x1 = max(cx - ln / 2, ARTERY_X[0]), min(cx + ln / 2, ARTERY_X[1])
        if x1 > x0:
            boxes.append((((x0 + x1) / 2, 0.0, r + LINE_T / 2 - 0.0002), (x1 - x0, LINE_W, LINE_T), DARK))
    cyl.append(((CREASE_LX, 0.0, 0.0), arm_radius(CREASE_X - 0.001) + LINE_T - 0.0002, 0.003, DARK, "X"))  # elbow crease
    return boxes, cyl


_AB, _AC = _arm_shapes()
ARM = part("patient_arm", (ARM_X0, 0.0, AX), boxes=_AB, cylinders=_AC, fixed=True, color=SKIN, friction=ARM_MU)
SH_TOP, FA_TOP = AX - 0.058, AX - 0.035
PROPS = part("props", (ARM_X0, 0.0, TOP), fixed=True, color=PROP, friction=0.8,
             boxes=[((0.140, 0.0, (SH_TOP - TOP) / 2), (0.08, 0.12, SH_TOP - TOP), PROP),
                    ((0.560, 0.0, (FA_TOP - TOP) / 2), (0.24, 0.14, FA_TOP - TOP), PROP)])

# -- cuff -------------------------------------------------------------------------------------------------------------
N_SEG, SEG, CUFF_W, TH, CUFF_M = 26, 0.020, 0.140, 0.003, 0.10
END_M = 0.015  # kg: each of the two stiffened segments at each end (sewn end + patch, E); the rest share the remainder
TAB_TH = 0.005  # the two end segments at each end: the folded, sewn end with its patch is 5 mm thick (E)
ROOT = 13  # the middle segment is the part's main body; hinges run out to both ends
CUFF_X = 0.25  # cuff centre line (its lower edge 30 mm above the crease when it ends here)
BEND_K, BEND_C = 0.002, 0.01  # N m/rad, N m s/rad per hinge (E)
END_K = 0.012  # the curled end tab hinge (E)
REST_CURL = (1.45, 0.0, 0.0)  # rad, tip hinge first (E): each end tab curls up
N_HOOK, N_LOOP = 11, 5
MARK_SEG = 21
K_LOCK, C_LOCK = 1.0, 0.02  # hold drive of a closed cuff, N m/rad and N m s/rad
ENGAGE_OVERLAP, ENGAGE_GAP = 0.060, 0.006
PRESS_CONTACT = 0.0020  # m: a closed fist's pad tip within 2 mm of the lying loop face = the fingertip resting on it (PhysX rests the modelled
#   tip 1.0-1.4 mm off the face while the cuff is being pushed; the official runner exposes no contact force, DEVIATIONS 14)
PRESS_DEPTH = 0.00015  # m: the PRESS -- the loop segment under the fist pushed >= 0.15 mm toward the arm since the pad arrived. Measured
#   (DEVIATIONS 15): demo press 0.30-0.47 mm MuJoCo / 0.20-0.32 mm Isaac, sustained; a brush peaks at 0.11 mm; a hover reads 0
PRESS_DWELL_S = 0.30  # s: the press must be sustained this long before the closure takes
RELEASE_STEPS = 2  # control steps the release condition must hold (hysteresis)
CONTROL_HZ = 20.0
FIST = 0.0015  # a gripper presses with its fingers closed (each finger within 1.5 mm of closed)
PEEL_LIFT, PEEL_FORCE, SHEAR_FORCE = 0.015, 4.0, 10.0
LIM = 1.6


def seg_mass(i: int) -> float:
    return END_M if (i <= 1 or i >= N_SEG - 2) else (CUFF_M - 4 * END_M) / (N_SEG - 4)


def seg_y(i: int) -> float:
    """Segment centre y in the part frame (start end, segment 0, at +y)."""
    return (ROOT - i) * SEG


def anchor_y(i: int) -> float:
    """Hinge of segment i: on its side toward the root."""
    return seg_y(i) - SEG / 2 if i < ROOT else seg_y(i) + SEG / 2


def sign(i: int) -> float:
    """+1 where a positive hinge angle curls the cuff toward its skin face (start-end side), -1 on the free-end side."""
    return 1.0 if i < ROOT else -1.0


def rest_angle(i: int) -> float:
    k = i if i < ROOT else N_SEG - 1 - i
    return REST_CURL[k] * sign(i) if k <= 2 else 0.0


def _seg_shapes(i: int):
    y = seg_y(i)
    th = TAB_TH if (i <= 1 or i >= N_SEG - 2) else TH
    b = [((0.0, y, 0.0), (CUFF_W, SEG * 0.98, th), NAVY)]
    if i < N_HOOK:  # hook patch on the outer face
        b.append(((0.0, y, -TH / 2 - 0.0002), (CUFF_W - 0.02, SEG * 0.98, 0.0004), HOOK_C))
    if i == MARK_SEG:  # ARTERY arrow on the outer face (bar + head), pointing to the cuff's lower (distal) edge
        b.append(((0.020, y, -TH / 2 - 0.0003), (0.050, 0.004, 0.0006), WHITE))
        b.append(((0.050, y, -TH / 2 - 0.0003), (0.012, 0.012, 0.0006), WHITE))
    return b


def _cuff():
    joints = []
    order = list(range(ROOT - 1, -1, -1)) + list(range(ROOT + 1, N_SEG))
    for i in order:
        parent = "" if i in (ROOT - 1, ROOT + 1) else f"s{i + 1:02d}" if i < ROOT else f"s{i - 1:02d}"
        k = END_K if i in (0, N_SEG - 1) else BEND_K
        a = rest_angle(i)
        joints.append(Joint(f"s{i:02d}", kind="hinge", axis="X", anchor=(0.0, anchor_y(i), 0.0), parent=parent,
                            lower=-LIM, upper=LIM, stiffness=k, damping=BEND_C, target=a, initial=a, mass=seg_mass(i),
                            boxes=_seg_shapes(i)))
    return part("cuff", (CUFF_X, 0.0, TOP + TH / 2 + 0.0005), mass=seg_mass(ROOT), friction=CUFF_MU, color=NAVY,
                boxes=_seg_shapes(ROOT), joints=joints)


CUFF = _cuff()
OBJECTS = (ARM, PROPS, CUFF)
JOINT_ORDER = [j.name for j in CUFF.joints]

# -- cuff kinematics (pure, from the part's pose and hinge angles) -------------------------------------------------------


def _quat_mat(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])


def segments(pos, quat_xyzw, joints: dict) -> list[tuple[np.ndarray, np.ndarray]]:
    """World (centre, rotation) of every cuff segment, segment 0 (start end) first."""
    R0, p0 = _quat_mat(quat_xyzw), np.asarray(pos, float)
    frames = {ROOT: (p0, R0, 0.0)}  # segment -> (hinge origin, rotation, anchor y)
    for i in list(range(ROOT - 1, -1, -1)) + list(range(ROOT + 1, N_SEG)):
        par = i + 1 if i < ROOT else i - 1
        po, Rp, ay_p = frames[par]
        ay = anchor_y(i)
        o = po + Rp @ np.array([0.0, ay - ay_p, 0.0])
        frames[i] = (o, Rp @ _rx(float(joints.get(f"s{i:02d}", 0.0))), ay)
    out = []
    for i in range(N_SEG):
        o, R, ay = frames[i]
        out.append((o + R @ np.array([0.0, seg_y(i) - ay, 0.0]), R))
    return out


def radial(p) -> tuple[float, float]:
    """(distance from the arm axis, angle from the top toward +y) of a world point."""
    dy, dz = float(p[1]), float(p[2]) - AX
    return math.hypot(dy, dz), math.atan2(dy, dz)


def contact_radius() -> float:
    """The largest skin radius under the cuff's width at its scored place (the cuff rests on it)."""
    xs = np.linspace(CUFF_X - CUFF_W / 2, CUFF_X + CUFF_W / 2, 29)
    return max(arm_radius(float(x)) for x in xs)


def wrap_state(segs) -> dict:
    """Unwrapped angle of every segment about the arm axis (from the start end), radial gaps, outer layer."""
    th = []
    for c, _ in segs:
        a = radial(c)[1]
        if th:
            a += 2 * math.pi * round((th[-1] - a) / (2 * math.pi))
        th.append(a)
    th = np.array(th)
    span = abs(th[-1] - th[0])
    d = 1.0 if th[-1] >= th[0] else -1.0
    rc = contact_radius()
    gaps = []
    for i, (c, R) in enumerate(segs):
        rho = radial(c)[0]
        outer = d * (th[i] - th[0]) > 2 * math.pi - SEG / (2 * rc)
        gaps.append(rho - TH / 2 - rc - (TH if outer else 0.0))
    return {"theta": th, "span": span, "dir": d, "gaps": np.array(gaps)}


def outward(c, R) -> float:
    """Cosine between a segment's outer-face normal and the outward radial direction."""
    n = R @ np.array([0.0, 0.0, -1.0])
    rad = np.array([0.0, float(c[1]), float(c[2]) - AX])
    return float(n @ rad / (np.linalg.norm(rad) + 1e-12))


def loop_gaps(segs) -> dict[int, float]:
    """Gap (m) of every loop-face segment that lies over the hook face (same angle about the arm, both faces out)."""
    hook = [(radial(c), outward(c, R)) for c, R in segs[:N_HOOK]]
    out = {}
    for i in range(N_SEG - N_LOOP, N_SEG):
        c, R = segs[i]
        (rho, th), cos_out = radial(c), outward(c, R)
        for (rh, thh), co in hook:
            dth = abs((th - thh + math.pi) % (2 * math.pi) - math.pi)
            if dth * rh <= SEG * 0.75 and cos_out > 0.7 and co > 0.7:
                g = rho - rh - TH
                out[i] = g if i not in out else min(out[i], g, key=abs)
    return out


def overlap(segs) -> tuple[float, float]:
    """(loop-face length lying on the hook face within ENGAGE_GAP, largest lift of a loop segment off the hook face)."""
    gaps = loop_gaps(segs)
    on = SEG * sum(1 for g in gaps.values() if abs(g) <= ENGAGE_GAP)
    return on, max(gaps.values(), default=0.0)


def _obb_dist(p, c, R) -> float:
    q = R.T @ (np.asarray(p, float) - c)
    h = np.array([CUFF_W / 2, SEG / 2, TH / 2])
    return float(np.linalg.norm(np.maximum(np.abs(q) - h, 0.0)))


def mark_point(segs) -> np.ndarray:
    c, R = segs[MARK_SEG]
    return c + R @ np.array([0.0, 0.0, -TH / 2])


def bonded_hinges(segs) -> list[int]:
    """The hinges that bridge the two overlapping surfaces: hinge i is included when segment i and segment i+1 BOTH lie
    on the hook face (loop_gaps within ENGAGE_GAP). This is the fastening: the loop face is pinned to the layer under it
    at the hinges that span the overlap, and every other hinge keeps its own passive stiffness. A single fixed joint
    between the two ends of one 26-segment chain is not expressible (it would close a kinematic loop), so the bond is the
    minimal hinged set spanning the contact."""
    gaps = loop_gaps(segs)
    lying = {i for i, gp in gaps.items() if abs(gp) <= ENGAGE_GAP}
    return sorted(i for i in lying if (i + 1) in lying or (i - 1) in lying)


def pressing_hands(segs, pads, closed) -> list[int]:
    """Hands (0 left, 1 right) that are ON the closure: fingers closed (``closed`` per pad) and the pad tip within
    PRESS_CONTACT of a loop segment lying on the hook face, with >= ENGAGE_OVERLAP of loop face lying."""
    gaps = loop_gaps(segs)
    lying = [i for i, gp in gaps.items() if abs(gp) <= ENGAGE_GAP]
    if SEG * len(lying) < ENGAGE_OVERLAP - 1e-9:
        return []
    return [h for h, (p, c) in enumerate(zip(pads, closed)) if c and any(_obb_dist(p, *segs[i]) <= PRESS_CONTACT for i in lying)]


def engages(segs, pads, closed) -> bool:
    """Some closed fist is on >= ENGAGE_OVERLAP of lying loop face (see ``pressing_hands``)."""
    return bool(pressing_hands(segs, pads, closed))


def release_due(q, lock, bond_idx, lift) -> bool:
    """Peel: the loop face lifted > PEEL_LIFT off the hook face with > PEEL_FORCE; shear: > SHEAR_FORCE round the ring. Both
    are read from the BONDED hinges' deflection (hold-drive torque K_LOCK * deviation over the lever arm): only those hinges
    carry the closure; a passive hinge bent by a press carries 0.002 N m/rad and says nothing about the bond."""
    dev = np.abs(np.asarray(q, float) - np.asarray(lock, float))[list(bond_idx)] if len(bond_idx) else np.zeros(1)
    peel = K_LOCK * float(dev.max()) / SEG
    shear = K_LOCK * float(dev.max()) / contact_radius()
    return (lift > PEEL_LIFT and peel > PEEL_FORCE) or shear > SHEAR_FORCE


def press_depth(segs, pad) -> float:
    """How far (m) the lying loop segment under ``pad`` has moved toward the arm since the pad arrived on it: the
    segment and its radius are anchored in ``_EP`` at the first step the fist is on the closure and dropped when it
    leaves, so a pad resting on the face without pushing reads 0 and a push reads the cuff's displacement."""
    gaps = loop_gaps(segs)
    lying = [i for i, gp in gaps.items() if abs(gp) <= ENGAGE_GAP]
    if not lying:
        _EP["anchor"] = None
        return 0.0
    if _EP["anchor"] is None:
        i = min(lying, key=lambda k: _obb_dist(pad, *segs[k]))
        _EP["anchor"] = (i, radial(segs[i][0])[0])
    i, rho0 = _EP["anchor"]
    return rho0 - radial(segs[i][0])[0]


def press_gate(hands, depth) -> bool:
    """The closure's press decision in one place: a closed fist is on the lying loop face (``hands`` from
    ``pressing_hands``) AND the cuff under it has been pushed toward the arm by >= PRESS_DEPTH since the pad arrived
    (``depth`` from ``press_depth``). Both terms are load-bearing: proximity without displacement is a hover, and
    displacement without a hand cannot be observed by this rule at all."""
    return bool(hands) and depth >= PRESS_DEPTH


def closure_step(q, lift, hinges, col, touching, pressed) -> None:
    """One control step of the closure (state in ``_EP``). ``touching``: a closed fist is on the lying loop face;
    ``pressed``: touching AND the cuff under it is pushed toward the arm by >= PRESS_DEPTH. Engage after
    PRESS_DWELL_S of press. A hand on the closure is a press, not a peel, so the release rule runs only when no hand
    is on it, and it must hold RELEASE_STEPS steps."""
    if not _EP["engaged"]:
        _EP["press_s"] = (_EP["press_s"] + 1.0 / CONTROL_HZ) if pressed else 0.0
        if _EP["press_s"] >= PRESS_DWELL_S - 1e-9 and hinges:
            _EP.update(engaged=True, lock=np.array(q, float).copy(), bonded=list(hinges),
                       bond_idx=sorted(col[i] for i in hinges if i in col), release_n=0)
    elif touching:
        _EP["release_n"] = 0
    else:
        _EP["release_n"] = _EP["release_n"] + 1 if release_due(q, _EP["lock"], _EP["bond_idx"], lift) else 0
        if _EP["release_n"] >= RELEASE_STEPS:
            _EP.update(engaged=False, lock=None, bonded=[], bond_idx=[], press_s=0.0, release_n=0)


# -- per-trial rule state ----------------------------------------------------------------------------------------------
_EP: dict = {}


def _fresh() -> dict:
    return {"engaged": False, "lock": None, "bonded": [], "bond_idx": [], "press_s": 0.0, "release_n": 0, "written": None,
            "anchor": None, "depth": 0.0, "tcps": None, "pads": None}


_PASSIVE = {j.name: (float(j.stiffness), float(j.target)) for j in CUFF.joints}


_EP.update(_fresh())
_IDS: dict = {}


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    return x.detach().cpu().numpy().astype(np.float64) if hasattr(x, "detach") else np.asarray(x, dtype=np.float64)


def _tensor(a, like):
    try:
        import torch
        return torch.tensor(np.asarray(a), dtype=torch.float32, device=like.device)
    except ImportError:
        return np.asarray(a)


def _arm_tcps(env):
    """(tool points, pad-tip points) of both grippers, world frame, from the arm joint angles."""
    from yam_arena import ik
    from yam_arena.constants import ARM_JOINTS
    from yam_arena.constants import FINGER_JOINTS
    tcps, pads = [], []
    for side in ("left", "right"):
        rob = env.unwrapped.scene[f"{side}_robot"]
        if side not in _IDS:
            _IDS[side] = rob.find_joints(ARM_JOINTS, preserve_order=True)[0]
            _IDS[side + "_f"] = rob.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        qa = _np(rob.data.joint_pos)[0]
        q = qa[_IDS[side]]
        _EP["closed_" + side] = float(np.abs(qa[_IDS[side + "_f"]]).max()) <= FIST
        p, rot = ik.tcp_pose(q, station.BASES[side])
        p, rot = np.asarray(p, float), np.asarray(rot, float)
        tcps.append(p)
        pads.append(p + rot[:, 2] * PAD_TIP_PAST_TCP)
    return tcps, pads


def _reset(_env) -> None:
    _EP.update(_fresh())


on_reset(_reset)


@on_step
def _rules(env) -> None:
    """The hook-and-loop closure (engage / release) and the gripper points the score reads."""
    base = env.unwrapped
    try:
        _EP["tcps"], _EP["pads"] = _arm_tcps(env)
    except Exception:  # noqa: BLE001  (bookkeeping only; the score then reads the grippers as not clear)
        _EP["tcps"] = _EP["pads"] = None
    cu = base.scene["cuff"]
    names = list(cu.joint_names)
    q = _np(cu.data.joint_pos)[0]
    pose = _np(cu.data.root_link_pose_w)[0]
    origin = _np(base.scene.env_origins[0])
    qs = {n[:-6]: float(q[k]) for k, n in enumerate(names)}
    segs = segments(pose[:3] - origin, pose[3:7], qs)
    _, lift = overlap(segs)
    closed = (_EP.get("closed_left", False), _EP.get("closed_right", False))
    hands = pressing_hands(segs, _EP["pads"], closed) if _EP["pads"] is not None else []
    if hands:
        _EP["depth"] = press_depth(segs, _EP["pads"][hands[0]])
    else:
        _EP["anchor"], _EP["depth"] = None, 0.0
    pressed = press_gate(hands, _EP["depth"])
    # bonded_hinges() names hinges by SEGMENT number; the joint array is ordered by name and has no entry for the root
    # segment, so map segment -> joint index by name rather than using the number as a position.
    col = {int(nm[1:3]): k for k, nm in enumerate(names)}
    closure_step(q, lift, bonded_hinges(segs) if pressed else [], col, bool(hands), pressed)
    write_drives(cu, names)


def write_drives(cu, names) -> bool:
    """Write the cuff's joint stiffness and target arrays ONLY on a state EDGE (engage / release / first step) -- the
    bond is a latch, not a controller. Engaged: ONLY the hinges that bridge the overlap get the hold drive at the
    engaged angle; every other hinge keeps its own passive stiffness and target, so the released cuff holds its shape
    through the closure and its own material behaviour. Returns True when it wrote."""
    state = (_EP["engaged"], tuple(_EP["bonded"]))
    if state == _EP["written"]:
        return False
    stiff = [_PASSIVE[nm[:-6]][0] for nm in names]
    target = [_PASSIVE[nm[:-6]][1] for nm in names]
    if _EP["engaged"]:
        target = list(_EP["lock"])
        for k in _EP["bond_idx"]:
            stiff[k] = K_LOCK
    cu.write_joint_stiffness_to_sim(_tensor([stiff], cu))
    cu.set_joint_position_target(_tensor([target], cu))
    _EP["written"] = state
    return True


# -- scoring ------------------------------------------------------------------------------------------------------------
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
PASS_SCORE = 1.0 - 1e-9
MARK_ARC = 0.020  # artery mark within 20 mm (arc) of the artery line
EDGE_ABOVE = (0.020, 0.040)  # cuff lower edge 20-40 mm above the elbow crease
SNUG_GAP = 0.010  # largest radial gap cuff-to-skin
LOOSE_GAP = 0.030  # 0.3 tier: one full turn round the arm, every segment within 30 mm of the skin
HAND_CLEAR = 0.030  # both grippers 3 cm clear of the cuff


def components(cu: PartState) -> dict:
    segs = segments(cu.pos, cu.quat_xyzw, cu.joints)
    w = wrap_state(segs)
    m = mark_point(segs)
    rho_m, th_m = radial(m)
    xs = [float(c[0]) for c, _ in segs]
    edge = float(np.mean(xs)) + CUFF_W / 2
    clear = False
    if _EP.get("pads") is not None:
        pts = list(_EP["pads"]) + list(_EP["tcps"])
        clear = min(_obb_dist(p, c, R) for p in pts for c, R in segs) >= HAND_CLEAR
    return {
        "engaged": bool(_EP.get("engaged")),
        "turn": w["span"] >= 2 * math.pi,
        "max_gap": float(w["gaps"].max()),
        "mark_arc": abs(th_m) * rho_m,
        "mark_out": outward(*segs[MARK_SEG]) > 0.7,
        "edge_above": CREASE_X - edge,
        "clear": clear,
    }


def score(objects: dict[str, PartState]) -> float:
    k = components(objects["cuff"])
    snug = k["turn"] and k["max_gap"] <= SNUG_GAP
    if k["engaged"] and snug:
        placed = (k["mark_out"] and k["mark_arc"] <= MARK_ARC and EDGE_ABOVE[0] <= k["edge_above"] <= EDGE_ABOVE[1])
        return 1.0 if placed and k["clear"] else 0.6
    if k["turn"] and k["max_gap"] <= LOOSE_GAP:
        return 0.3
    return 0.0


def success(objects: dict[str, PartState]) -> bool:
    return score(objects) >= PASS_SCORE


def distance(objects: dict[str, PartState]) -> float:
    k = components(objects["cuff"])
    return float(k["max_gap"])


# -- scripted demo (fixed scene geometry only) ---------------------------------------------------------------------

GRIP_OPEN = -0.012  # partial opening: 24 mm between the pads round a 5 mm tab
TAB_Y, FREE_Y = 0.2513, 0.2313  # the curled start / free end tabs stand at y = +TAB_Y / -FREE_Y (settled start)
TAB_PAD_Z = 0.008  # pad tips 8 mm over the table: the tab's lower half


def demo(t_wait=0.0):
    """Both arms, open loop, fixed scene geometry only.
    1. Right pinches the free-end tab and holds it on the table (anchors the cuff); left pinches the start-end tab,
       lifts the start end up the +y side, over the top of the arm and lays it on the -y side, then lets go and parks.
    2. Right draws the free end up the -y side (the cuff's middle comes up under the arm), over the top and lets it go
       over the +y side; then, fingers closed, presses the loop face onto the hook face (the closure engages), lifts
       off and parks."""
    W, g, c, o = Waypoint, GRIP_OPEN, FINGER_CLOSED, FINGER_OPEN
    x, T = CUFF_X, PAD_TIP_PAST_TCP

    def P(y, z):  # TCP for pad tips at (x, y, TOP + z)
        return (x, y, TOP + z + T)

    park_l, park_r = (0.16, 0.32, TOP + 0.20), (0.16, -0.32, TOP + 0.20)
    ly, lz = -0.058, 0.125  # start end laid on the far side, about 30 deg above it
    ry, rz = 0.040, 0.165  # free end let go over the near side
    s1 = (0.024, 0.1260)  # fist press on the closure (pad held on the lying loop face)
    left = [W(2.0, P(TAB_Y, 0.07), g, "over the start tab"), W(1.5, P(TAB_Y, TAB_PAD_Z), g, "round the tab"),
            W(1.0, P(TAB_Y, TAB_PAD_Z), c, "pinch"), W(2.0, P(0.20, 0.15), c, "lift the start end"),
            W(1.5, P(0.08, 0.168), c, "to the arm"), W(1.5, P(-0.01, 0.168), c, "over the top"),
            W(1.5, P(ly, lz), c, "lay it on the far side"), W(0.6, P(ly, lz), g, "let go"),
            W(1.0, P(-0.03, 0.165), g, "up"), W(2.0, park_l, o, "park")]
    t_free = sum(w.duration for w in left) - 2.0
    right = [W(2.0, P(-FREE_Y, 0.07), g, "over the free tab"), W(1.5, P(-FREE_Y, TAB_PAD_Z), g, "round the tab"),
             W(1.0, P(-FREE_Y, TAB_PAD_Z), c, "pinch"), W(t_free - 4.5, P(-FREE_Y, TAB_PAD_Z), c, "hold the free end"),
             W(1.5, P(-0.16, 0.10), c, "draw the free end up"), W(1.5, P(-0.085, 0.17), c, "up the far side"),
             W(1.5, P(0.01, 0.170), c, "over the top"), W(1.0, P(ry, rz), c, "onto the near side"),
             W(0.6, P(ry, rz), g, "let go"), W(0.6, P(ry - 0.01, rz + 0.012), g, "up"),
             W(0.6, P(ry - 0.01, rz + 0.012), c, "close"), W(4.3, P(*s1), c, "press the closure"),
             W(1.0, P(s1[0] + 0.02, s1[1] + 0.03), c, "off"), W(1.0, P(0.09, 0.165), c, "up"), W(1.5, park_r, o, "park")]
    tl, tr = sum(w.duration for w in left), sum(w.duration for w in right)
    end = max(tl, tr) + 2.0 + t_wait
    left.append(W(end - tl, park_l, o, "hold clear"))
    right.append(W(end - tr, park_r, o, "hold clear"))
    return {"left": left, "right": right}
