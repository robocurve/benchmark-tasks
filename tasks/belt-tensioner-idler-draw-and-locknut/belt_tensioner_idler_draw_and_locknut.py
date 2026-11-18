"""Draw a belt-tensioner idler carriage to its calibrated tension and lock the clamp collar on its stud.

Real objects (1:1; see README "Real-world build"): a GT2 timing-belt tensioner sub-frame (anodised 6061 bracket,
360 g, unbolted on the bench), a sliding idler carriage (95 g) held back by a 3.36 N belt return spring, a 6 mm acetal
setup gauge (26 x 28 x 22 mm head) that sets the calibrated draw, and a 40 g knurled clamp collar on an M8 stud that
pins the carriage down once turned home.

Mechanism, exactly as coded (README "Fidelity"):
- simulated: the carriage is a slide joint of the sub-frame (travel 0-22 mm, spring 160 N/m back to 0 mm slack, i.e.
  3.36 N at the 21 mm mark); the collar is a hinge (turn) carrying a slide (thread advance, 4 mm total travel, closing
  the 1.0 mm gap in 180 deg at 2.0 mm/rev). The sub-frame is a free 360 g body on the bench.
- modelled rules (on_step, from the joint values and the setup gauge's pose):
  * gauge stop: with the 6 mm setup gauge seated in the frame's reference pocket and the draw having passed it once,
    return travel is blocked at the 21 mm mark (K_STOP = 2600 N/m holding 3.36 N preload, so both hands are free).
    Remove the gauge before the collar is down and the carriage snaps back to 0 mm slack.
  * clamp seat: the collar's face reaches the carriage plate after 180 deg of turn; once seated >= 180 deg within
    0.2 mm axial gap, the carriage locks where it stands (K_CLAMP = 5200 N/m).

Bench layout (station coordinates, TOP = station.TABLE_TOP, +X forward, +Y left, arm bases at y = +-0.305 m; the frame
origin is at (FX, FY) with z counted from the table top):
- sub-frame 320 x 340 mm foot, deck at 42 mm;
- carriage plate on the deck: draw grip at y = +0.088 (left hand), M8 stud and clamp collar at y = -0.085 (right
  hand), index lug reaching over the frame's gauge pocket at (0.070, 0.125) m;
- setup-gauge tray 190 mm to the left of the frame, its plinth at the same seat height as the pocket.

Run it:
  inspect-robots run --embodiment yam_arena -E task=belt_tensioner_idler_draw_and_locknut.py -E object_jitter=0 \\
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
# Order matters: mechkit first, labkit second (both hook yam_arena's scene import through sys.meta_path).
import mechkit as mk  # noqa: E402, F401
import labkit  # noqa: E402
from labkit import Joint, PartState, on_reset, on_step, part  # noqa: E402
import pitch_demo  # noqa: E402
from pitch_demo import Waypoint  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402

SCENE = "station"
TOP = station.TABLE_TOP

# -- colours -----------------------------------------------------------------------------------------------------------
ANOD = (0.19, 0.21, 0.25)  # black-blue anodised 6061 bracket
ANOD_L = (0.31, 0.34, 0.39)  # machined faces
STEEL = (0.63, 0.65, 0.68)  # brushed stainless
ALU = (0.79, 0.80, 0.82)  # bright aluminium
NUT_C = (0.12, 0.12, 0.14)  # black anodised knurled collar
RUBBER = (0.06, 0.06, 0.07)  # GT2 belt / idler tyre
DELRIN = (0.90, 0.88, 0.80)  # natural acetal setup gauge
TRAY_C = (0.55, 0.57, 0.60)  # grey machined tray
SCREW = (0.42, 0.44, 0.47)

# -- placement (single source of truth for scene, score() and demo()) ---------------------------------------------------
# Frame-local x/y run from the sub-frame origin; z is counted from the table top (the part origin is (FX, FY, TOP)).
FX, FY = 0.300, 0.000  # sub-frame origin on the table
TRAY_X, TRAY_Y = 0.190, 0.250  # setup-gauge tray centre
DECK_Z = 0.042  # deck top: the carriage plate slides on it
PLATE_T = 0.011
PLATE_TOP = DECK_Z + PLATE_T  # 0.053
STUD_Y = -0.085  # M8 stud axis (right hand)
POST_Y = 0.088  # carriage draw post (left hand)
DRAW_MAX = 0.022  # slide joint upper limit
SHIM_STOP = 0.021  # the setup gauge blocks the return at the calibrated mark
SPRING_N = 3.36  # N of belt return at the calibrated mark (softened for arm budget)
K_SPRING, K_STOP, K_CLAMP = 160.0, 2600.0, 5200.0  # N/m
M_FRAME, M_CARR, M_SHIM, M_HUB, M_COLLAR = 0.360, 0.095, 0.040, 0.036, 0.040  # kg
FRAME_MU, GAUGE_MU = 0.90, 0.75

# setup gauge: 6 mm stem on a 26 x 28 x 22 mm head; pocket placed at +0.070, +0.125 (front-left, clear of draw arm).
POCKET_X, POCKET_Y = 0.070, 0.125  # pocket centre line (frame local)
STEM_W, HEAD_H = 0.006, 0.014
STEM_TOP = 0.065  # local z of the stem's top
GAUGE_LIFT = 0.030  # gauge origin over its stem bottom
GX, GY, GZ = FX + POCKET_X, FY + POCKET_Y, TOP + DECK_Z + GAUGE_LIFT  # gauge origin when seated
STEM_FACE = POCKET_X - STEM_W / 2  # local x of the stem face
LUG_FACE0 = POCKET_X - 0.011 - DRAW_MAX - 0.002  # lug +x face clears snug pocket wall at full draw

# clamp collar
COLLAR_Z0 = 0.0540  # bottom face 1.0 mm over the plate at rest
COLLAR_R, COLLAR_H = 0.015, 0.018
NUT_GAP = COLLAR_Z0 - PLATE_TOP  # 1.0 mm of thread left: the last 180 deg take it up
THREAD_TRAVEL = 0.0040  # slide travel of the "clamp" joint along the thread
CLAMP_TURN = 180.0  # deg of turn to clamping contact
SEAT_EPS = 0.0002  # m: face-to-plate tolerance
SPIN_MAX = -math.radians(330.0)
CROWN = np.array([FX, FY + STUD_Y, TOP + COLLAR_Z0 + 0.026])


def _frame_boxes():
    return [
        ((0.0, 0.0, 0.006), (0.320, 0.340, 0.012), ANOD),  # foot plate
        ((0.110, 0.0, 0.027), (0.018, 0.312, 0.030), ANOD),  # front rail
        ((-0.110, 0.0, 0.027), (0.018, 0.312, 0.030), ANOD),  # rear rail
        ((0.0, 0.148, 0.027), (0.240, 0.016, 0.030), ANOD),  # left rail (clear of gauge pocket)
        ((0.0, -0.140, 0.027), (0.240, 0.016, 0.030), ANOD),  # right rail
        ((0.0, 0.0, 0.039), (0.260, 0.284, 0.006), ANOD_L),  # deck the carriage slides on
        ((POCKET_X + 0.0085, POCKET_Y, 0.053), (0.0032, 0.044, 0.022), ANOD),  # gauge pocket wall (+X)
        ((POCKET_X - 0.0085, POCKET_Y, 0.053), (0.0032, 0.044, 0.022), ANOD),  # gauge pocket wall (-X)
        ((POCKET_X, POCKET_Y - 0.0190, 0.053), (0.018, 0.004, 0.022), ANOD),  # pocket locating rib -y
        ((POCKET_X, POCKET_Y + 0.0190, 0.053), (0.018, 0.004, 0.022), ANOD),  # pocket locating rib +y
        ((-0.020, 0.030, 0.0595), (0.024, 0.024, 0.013), STEEL),  # fixed crank boss
        ((0.000, -0.020, 0.0465), (0.150, 0.010, 0.009), RUBBER),  # GT2 belt run to the idler
        ((0.098, -0.120, 0.028), (0.010, 0.040, 0.024), SCREW),  # gusset
        ((-0.055, -0.110, 0.0455), (0.026, 0.026, 0.007), SCREW),  # press pad (right hand steadies here)
    ]


def _frame_cyls():
    return [((0.0, STUD_Y, 0.0570), 0.0040, 0.030, ALU, "Z"),  # M8 stud (flush under wing-nut crown)
            ((0.0, STUD_Y, 0.0425), 0.0078, 0.003, STEEL, "Z")]  # stud collar washer


def _carriage_boxes():
    return [
        ((0.002, 0.055, 0.0475), (0.064, 0.110, PLATE_T), ANOD),  # plate, +y of stud slot (clear of pocket)
        ((-0.0200, -0.1495, 0.0475), (0.100, 0.099, PLATE_T), ANOD),  # plate, -y of the stud slot
        ((LUG_FACE0 - 0.007, POCKET_Y, 0.0570), (0.014, 0.024, 0.010), ANOD_L),  # index lug over the pocket
        ((LUG_FACE0 - 0.007, 0.110, 0.0570), (0.014, 0.036, 0.010), ANOD_L),  # bridge arm from carriage to lug
        ((-0.020, -0.060, 0.0605), (0.026, 0.024, 0.015), STEEL),  # belt clamp block
        ((0.004, POST_Y, 0.0700), (0.020, 0.028, 0.036), RUBBER),  # T-pull grip block (X-pinch form closure)
        ((0.004, POST_Y + 0.017, 0.0700), (0.022, 0.005, 0.038), STEEL),  # +y guard flange
        ((0.004, POST_Y - 0.017, 0.0700), (0.022, 0.005, 0.038), STEEL),  # -y guard flange
    ]


def _carriage_cyls():
    return [((-0.020, 0.040, 0.0605), 0.021, 0.015, RUBBER, "Z"),  # GT2 idler pulley
            ((-0.020, 0.040, 0.0690), 0.024, 0.002, ALU, "Z"),  # upper flange
            ((-0.020, 0.040, 0.0530), 0.024, 0.002, ALU, "Z"),  # lower flange
            ((0.004, POST_Y, 0.0680), 0.006, 0.032, STEEL, "Z")]  # draw post core


def _nut_boxes():
    """Knurled collar + raised T-wing grip bar (16 mm Y thickness for centered finger grip above the stud)."""
    b = [
        ((0.0, STUD_Y, COLLAR_Z0 + 0.0090), (0.0250, 0.0250, 0.0180), NUT_C),
        ((0.0, STUD_Y, COLLAR_Z0 + 0.0260), (0.0400, 0.0180, 0.0220), NUT_C),  # T-wing grip bar (18 mm pinch)
        ((0.0, STUD_Y, COLLAR_Z0 + 0.0380), (0.0360, 0.0180, 0.0030), ALU),  # top index cap
    ]
    for k in range(6):  # knurl ridges on the anodised collar
        a = k * math.pi / 3.0
        b.append(((0.0142 * math.cos(a), STUD_Y + 0.0142 * math.sin(a), COLLAR_Z0 + 0.0090),
                  (0.0045, 0.0090, 0.0165), NUT_C, a))
    return b


def _nut_cyls():
    return [((0.0, STUD_Y, COLLAR_Z0 + 0.0090), COLLAR_R, COLLAR_H, NUT_C, "Z"),  # collar body
            ((0.0, STUD_Y, COLLAR_Z0 + 0.0195), 0.0122, 0.0020, ALU, "Z"),  # chamfered top ring
            ((0.0, STUD_Y, COLLAR_Z0 - 0.0018), 0.0172, 0.0032, ALU, "Z")]  # flange washer face


def _joints():
    """clamp (thread advance along the stud) -> spin (the collar's turn), and draw (the carriage slide)."""
    hub = [((0.0, STUD_Y, COLLAR_Z0 - 0.0095), (0.0158, 0.0158, 0.0160), STEEL)]
    clamp = Joint("clamp", kind="slide", axis="Z", anchor=(0.0, STUD_Y, COLLAR_Z0), boxes=hub,
                  lower=-THREAD_TRAVEL, upper=0.0, stiffness=900.0, damping=0.5, target=0.0, initial=0.0,
                  mass=M_HUB)
    spin = Joint("spin", kind="hinge", axis="Z", anchor=(0.0, STUD_Y, COLLAR_Z0), parent="clamp",
                 boxes=_nut_boxes(), cylinders=_nut_cyls(), lower=SPIN_MAX, upper=0.0, stiffness=0.0, damping=0.02,
                 friction=0.015, target=0.0, initial=0.0, mass=M_COLLAR)
    draw = Joint("draw", kind="slide", axis="X", anchor=(-0.020, 0.0, DECK_Z), boxes=_carriage_boxes(),
                 cylinders=_carriage_cyls(), lower=0.0, upper=DRAW_MAX, stiffness=K_SPRING, damping=0.9,
                 target=0.0, initial=0.0, mass=M_CARR)
    return (clamp, spin, draw)


FRAME = part("actuator_frame", (FX, FY, TOP), boxes=_frame_boxes(), cylinders=_frame_cyls(), mass=M_FRAME,
             friction=FRAME_MU, color=ANOD, joints=_joints())

GAUGE = part("setup_gauge", (TRAY_X, TRAY_Y, TOP + 0.072), mass=M_SHIM, friction=GAUGE_MU, color=DELRIN, boxes=[
    ((0.0000, 0.0, -0.0185), (0.0060, 0.0300, 0.0230), DELRIN),  # 6 mm stem, bottom on the pocket floor
    ((0.0000, 0.0, +0.0040), (0.0260, 0.0280, 0.0220), DELRIN),  # tall grip head (z=0.065..0.087)
    ((0.0000, 0.0, +0.0165), (0.0280, 0.0360, 0.0045), STEEL),   # top overhang cap lip (positive lift)
])

TRAY = part("gauge_tray", (TRAY_X, TRAY_Y, TOP), fixed=True, friction=0.8, color=TRAY_C, boxes=[
    ((0.0, 0.0, 0.0030), (0.110, 0.100, 0.0060), TRAY_C),  # base plate
    ((0.0, 0.0, 0.0210), (0.034, 0.044, 0.0420), TRAY_C),  # plinth: the seat at pocket height
    ((-0.0195, 0.0, 0.0500), (0.0030, 0.034, 0.0160), TRAY_C),  # plinth rib -x
    ((0.0195, 0.0, 0.0500), (0.0030, 0.034, 0.0160), TRAY_C),  # plinth rib +x
    ((0.0, 0.0540, 0.0180), (0.100, 0.0040, 0.0240), TRAY_C),  # wall +y
    ((0.0, -0.0540, 0.0180), (0.100, 0.0040, 0.0240), TRAY_C),  # wall -y
    ((0.0, 0.0, 0.0180), (0.0030, 0.100, 0.0240), TRAY_C, math.pi / 2),  # wall far x
])

BENCH = part("bench_block", (FX + 0.180, FY, TOP), fixed=True, friction=0.85, color=STEEL, boxes=[
    ((0.0, 0.0, 0.0140), (0.040, 0.220, 0.0280), STEEL),  # datum block against the frame's front face
    ((0.0, 0.080, 0.0300), (0.008, 0.008, 0.0060), SCREW),
    ((0.0, -0.080, 0.0300), (0.008, 0.008, 0.0060), SCREW),
])

OBJECTS = (FRAME, GAUGE, TRAY, BENCH)
JOINT_ORDER = ["clamp", "spin", "draw"]

# -- rule state --------------------------------------------------------------------------------------------------------
_EP: dict = {}


def _fresh() -> dict:
    return {"armed": False, "lock": 0.0, "draw": 0.0, "adv": 0.0, "spin": 0.0}


_EP.update(_fresh())


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    return x.detach().cpu().numpy().astype(np.float64) if hasattr(x, "detach") else np.asarray(x, dtype=np.float64)


def _tensor(a, like):
    try:
        import torch
        return torch.tensor(np.asarray(a), dtype=torch.float32, device=like.device)
    except ImportError:
        return np.asarray(a)


def _quat_to_mat(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def seated(gauge_pos, frame_pos=None, frame_quat=None) -> bool:
    """The setup gauge stands in the frame's reference pocket (its origin within 20 mm of the pocket seat)."""
    local = np.array([POCKET_X, POCKET_Y, DECK_Z + GAUGE_LIFT])
    if frame_pos is not None:
        r = _quat_to_mat(frame_quat) if frame_quat is not None else np.eye(3)
        seat = np.asarray(frame_pos, float) + r @ local
    else:
        seat = np.array([GX, GY, GZ])
    return float(np.linalg.norm(np.asarray(gauge_pos, float) - seat)) <= 0.020


def clamp_seated(spin: float, advance: float) -> bool:
    """Collar down on the carriage: at least 180 deg of turn with its face on the plate."""
    return abs(spin) >= math.radians(CLAMP_TURN) - 1e-6 and advance >= NUT_GAP - SEAT_EPS


def _reset(_env) -> None:
    _EP.update(_fresh())


on_reset(_reset)


@on_step
def _rules(env) -> None:
    """Gauge stop, helical thread advance coupled to collar spin, and clamp lock once seated."""
    base = env.unwrapped
    fr = base.scene["actuator_frame"]
    names = list(fr.joint_names)
    q = _np(fr.data.joint_pos)[0]
    v = {n[:-6]: float(q[k]) for k, n in enumerate(names)}
    _EP["draw"], _EP["adv"], _EP["spin"] = v["draw"], -v["clamp"], v["spin"]
    origin = _np(base.scene.env_origins[0])
    fpose = _np(base.scene["actuator_frame"].data.root_link_pose_w)[0]
    fpos, fquat = fpose[:3] - origin, fpose[3:7]
    gp = _np(base.scene["setup_gauge"].data.root_link_pose_w)[0][:3] - origin
    held = seated(gp, fpos, fquat)
    if held and _EP["draw"] >= SHIM_STOP - 0.0015:
        _EP["armed"] = True
    if not held or _EP["draw"] < 0.002:
        _EP["armed"] = False
    clamped = clamp_seated(_EP["spin"], _EP["adv"])
    if clamped and _EP["lock"] == 0.0:
        _EP["lock"] = _EP["draw"]
    elif not clamped:
        _EP["lock"] = 0.0
    if clamped:
        kd, td = K_CLAMP, _EP["lock"]
    elif _EP["armed"]:
        kd, td = K_STOP, SHIM_STOP
    else:
        kd, td = K_SPRING, 0.0
    adv = -NUT_GAP * min(1.0, abs(_EP["spin"]) / math.radians(CLAMP_TURN))
    ks = {"draw": kd, "clamp": 900.0, "spin": 0.0}
    tg = {"draw": td, "clamp": adv, "spin": 0.0}
    acts = fr.__dict__.get("actuators")
    if isinstance(acts, dict):
        for act in acts.values():
            st = getattr(act, "stiffness", None)
            jnames = list(getattr(act, "joint_names", ()) or ())
            if st is not None:
                for idx, jn in enumerate(jnames):
                    if jn[:-6] in ks:
                        st[:, idx] = ks[jn[:-6]]
    fr.write_joint_stiffness_to_sim(_tensor([[ks[n[:-6]] for n in names]], fr))
    fr.set_joint_position_target(_tensor([[tg[n[:-6]] for n in names]], fr))


# -- scoring -----------------------------------------------------------------------------------------------------------
SCORE_OVER = "end"
SUCCESS_HOLD_S = 3.0
PASS_SCORE = 1.0 - 1e-9
DRAW_WINDOW = (0.018, 0.024)  # the calibrated tension window, metres of draw
FRAME_SLIP = 0.025  # sub-frame pushed more than 25 mm: the reading is void
FRAME_TILT = 8.0  # deg of tilt allowed on the sub-frame
HAND_CLEAR = 0.045  # both grippers at least 45 mm clear of every scored part
SHIM_CLEAR = 0.060  # the gauge at least 60 mm from the frame's pocket
TRAY_HOME = 0.075  # ... and back in its tray (centre of the tray within this of the gauge)
S_FULL, S_SPILL, S_HELD = 1.0, 0.7, 0.4


def tilt_deg(st) -> float:
    """Tilt of a part's local +Z off world +Z, deg (works on a bare ObjectState too)."""
    fn = getattr(st, "tilt_deg", None)
    if callable(fn):
        return float(fn())
    x, y, z, w = (float(v) for v in st.quat_xyzw)
    return float(np.degrees(np.arccos(np.clip(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0))))


_IDS: dict = {}


def _arm_tcps(env) -> list:
    from yam_arena import ik
    from yam_arena.constants import ARM_JOINTS
    out = []
    for side in ("left", "right"):
        rob = env.unwrapped.scene[f"{side}_robot"]
        if side not in _IDS:
            _IDS[side] = rob.find_joints(ARM_JOINTS, preserve_order=True)[0]
        qa = _np(rob.data.joint_pos)[0]
        p, _ = ik.tcp_pose(qa[_IDS[side]], station.BASES[side])
        out.append(np.asarray(p, float))
    return out


def _tcps() -> list:
    tcps = mk.tcps()
    if tcps:
        return [np.asarray(p, float) for p in tcps]
    env = labkit.env()
    return _arm_tcps(env) if env is not None else []


def components(objects: dict) -> dict:
    """Pure observable physical end-state readouts (frame pose, draw/spin/clamp joints, gauge pose, gripper TCPs)."""
    fr = objects["actuator_frame"]
    gp = objects["setup_gauge"]
    moved = float(np.linalg.norm(np.asarray(fr.pos, float)[:2] - np.array([FX, FY])))
    frame_ok = moved <= FRAME_SLIP and tilt_deg(fr) <= FRAME_TILT
    joints = getattr(fr, "joints", {}) or {}
    d = float(joints.get("draw", 0.0))
    spin = float(joints.get("spin", 0.0))
    adv = -float(joints.get("clamp", 0.0))
    clamped = clamp_seated(spin, adv)
    gpos = np.asarray(gp.pos, float)
    shim_clear = (not seated(gpos, fr.pos, fr.quat_xyzw)) and \
        float(np.linalg.norm(gpos[:2] - np.array([GX, GY]))) >= SHIM_CLEAR and \
        float(np.linalg.norm(gpos[:2] - np.array([TRAY_X, TRAY_Y]))) <= TRAY_HOME
    tcps = _tcps()
    spots = [np.array([FX + 0.004, FY + POST_Y, TOP + 0.0725]), CROWN, gpos]
    clear = len(tcps) >= 2 and min(float(np.linalg.norm(p - s)) for p in tcps for s in spots) >= HAND_CLEAR
    return {"frame_ok": frame_ok, "draw": d, "spin": spin, "adv": adv,
            "in_window": DRAW_WINDOW[0] <= d <= DRAW_WINDOW[1],
            "clamped": bool(clamped), "shim_clear": bool(shim_clear), "clear": bool(clear),
            "moved": moved}


def score(objects: dict) -> float:
    """Judged at trial end from physical end-state; Possible scores: 0.0, 0.4, 0.7, 1.0."""
    k = components(objects)
    if not k["frame_ok"] or not k["in_window"]:
        return 0.0
    if k["clamped"]:
        return S_FULL if (k["shim_clear"] and k["clear"]) else S_SPILL
    return S_HELD


def success(objects: dict) -> bool:
    return score(objects) >= PASS_SCORE


def distance(objects: dict) -> float:
    return float(components(objects)["moved"])


# -- scripted demo (fixed scene geometry only) -------------------------------------------------------------------------

GAUGE_PAD_Z = TOP + 0.073  # pad tips under the gauge top overhang lip
POST_PAD_Z = TOP + 0.068  # pad tips centered on the T-pull grip block
NUT_PAD_Z = TOP + 0.077  # pad tips centered on the T-wing nut bar (above 0.072 stud)
DRAW_OVERDRIVE = DRAW_MAX + 0.026  # compensates arm PD compliance against return spring


def demo(t_wait=0.0):
    """Both arms, open loop, fixed scene geometry only.
    1. Right presses the sub-frame flat and holds it there.
    2. Left seats the 6 mm setup gauge in the frame pocket, then pinches the carriage T-pull block across X and draws
       the carriage to the 21 mm mark; the gauge blocks recoil so both hands are free.
    3. Right clamps the T-wing nut centered over the stud and rotates it in two strokes (-60 deg then +120 to -30 deg,
       210 deg total) so helical thread advance seats the collar flush onto the carriage plate.
    4. Left returns the gauge to its tray; both grippers park >= 45 mm clear and hold."""
    W, c, o = Waypoint, FINGER_CLOSED, FINGER_OPEN
    T = PAD_TIP_PAST_TCP
    rot0 = pitch_demo.rotation(0.0)
    rot_draw = pitch_demo.rotation(math.radians(80.0))

    def P(px, py, z_pad):
        return (px, py, z_pad + T)

    park_l, park_r = (0.20, 0.28, TOP + 0.175), (0.20, -0.28, TOP + 0.175)
    hx, hy = FX + 0.004, FY + POST_Y
    nx, ny = FX, FY + STUD_Y
    press = (FX - 0.055, FY - 0.110)

    left = [W(0.6, park_l, o, "ready", rot=rot0),
            W(1.4, P(TRAY_X, TRAY_Y, TOP + 0.140), o, "over the tray", rot=rot0),
            W(0.8, P(TRAY_X, TRAY_Y, GAUGE_PAD_Z), o, "down on the gauge head", rot=rot0),
            W(0.6, P(TRAY_X, TRAY_Y, GAUGE_PAD_Z), c, "grip the gauge", rot=rot0),
            W(0.7, P(TRAY_X, TRAY_Y, TOP + 0.130), c, "lift it clear", rot=rot0),
            W(1.5, P(GX + 0.005, GY, TOP + 0.132), c, "over the pocket", rot=rot0),
            W(0.8, P(GX + 0.005, GY, GAUGE_PAD_Z), c, "seat the gauge", rot=rot0),
            W(0.4, P(GX + 0.005, GY, GAUGE_PAD_Z), o, "let go", rot=rot0),
            W(0.7, P(GX + 0.005, GY, TOP + 0.132), o, "up", rot=rot0),
            W(1.2, P(hx, hy, TOP + 0.125), o, "over the draw grip", rot=rot_draw),
            W(0.6, P(hx, hy, POST_PAD_Z), o, "straddle draw grip along X", rot=rot_draw),
            W(0.5, P(hx, hy, POST_PAD_Z), c, "clamp draw grip", rot=rot_draw),
            W(2.2, P(hx + DRAW_OVERDRIVE, hy, POST_PAD_Z), c, "draw the carriage to the mark", rot=rot_draw),
            W(0.5, P(hx + DRAW_OVERDRIVE, hy, POST_PAD_Z), c, "settle at calibrated draw mark", rot=rot_draw),
            W(0.5, P(hx + DRAW_OVERDRIVE, hy, POST_PAD_Z), o, "let go: the gauge holds the preload", rot=rot_draw),
            W(0.9, P(hx + DRAW_OVERDRIVE, hy, TOP + 0.135), o, "up and clear", rot=rot_draw),
            W(2.0, park_l, o, "stand by", rot=rot0)]
    t_left_free = sum(w.duration for w in left)

    right = [W(0.6, park_r, o, "ready", rot=rot0),
             W(1.3, P(*press, TOP + 0.150), o, "over the frame's press pad", rot=rot0),
             W(0.7, P(*press, TOP + 0.056), o, "down", rot=rot0),
             W(0.5, P(*press, TOP + 0.056), c, "close the fingers", rot=rot0),
             W(0.9, P(*press, TOP + 0.042), c, "press the sub-frame flat", rot=rot0),
             W(max(0.8, t_left_free + 0.4 - 4.0), P(*press, TOP + 0.042), c, "steady the frame", rot=rot0),
             W(0.8, P(*press, TOP + 0.150), o, "release", rot=rot0),
             W(1.3, P(nx, ny, TOP + 0.135), o, "over the collar wing-bar", rot=rot0),
             W(0.8, P(nx, ny, NUT_PAD_Z), o, "descend on wing-bar", rot=rot0),
             W(0.6, P(nx, ny, NUT_PAD_Z), c, "grip wing-bar", rot=rot0),
             W(1.5, P(nx, ny, NUT_PAD_Z - 0.0015), c, "stroke 1: turn -60 deg", rot=pitch_demo.rotation(math.radians(-60.0))),
             W(0.4, P(nx, ny, NUT_PAD_Z - 0.0015), o, "release wing-bar", rot=pitch_demo.rotation(math.radians(-60.0))),
             W(0.7, P(nx, ny, TOP + 0.135), o, "lift clear", rot=pitch_demo.rotation(math.radians(-60.0))),
             W(0.9, P(nx, ny, TOP + 0.135), o, "re-index wrist to +120 deg", rot=pitch_demo.rotation(math.radians(120.0))),
             W(0.8, P(nx, ny, NUT_PAD_Z - 0.0015), o, "descend on wing-bar", rot=pitch_demo.rotation(math.radians(120.0))),
             W(0.6, P(nx, ny, NUT_PAD_Z - 0.0015), c, "re-grip wing-bar", rot=pitch_demo.rotation(math.radians(120.0))),
             W(1.2, P(nx, ny, NUT_PAD_Z - 0.0028), c, "stroke 2a: turn +120 to +35 deg", rot=pitch_demo.rotation(math.radians(35.0))),
             W(1.2, P(nx, ny, NUT_PAD_Z - 0.0035), c, "stroke 2b: turn +35 to -50 deg", rot=pitch_demo.rotation(math.radians(-50.0))),
             W(0.4, P(nx, ny, NUT_PAD_Z - 0.0035), o, "let go", rot=pitch_demo.rotation(math.radians(-50.0))),
             W(0.9, P(nx, ny, TOP + 0.150), o, "up", rot=pitch_demo.rotation(math.radians(-50.0))),
             W(1.6, park_r, o, "park", rot=rot0)]
    t_turn1 = sum(w.duration for w in right[:-3])

    hold = max(0.4, t_turn1 + 0.5 - t_left_free)
    left += [W(hold, park_l, o, "stand by", rot=rot0),
             W(1.4, P(GX, GY, TOP + 0.135), o, "back over the gauge", rot=rot0),
             W(0.9, P(GX, GY, GAUGE_PAD_Z), o, "down on the head", rot=rot0),
             W(0.5, P(GX, GY, GAUGE_PAD_Z), c, "grip", rot=rot0),
             W(0.7, P(GX, GY, TOP + 0.135), c, "lift it out", rot=rot0),
             W(1.5, P(TRAY_X, TRAY_Y, TOP + 0.130), c, "back to the tray", rot=rot0),
             W(0.8, P(TRAY_X, TRAY_Y, GAUGE_PAD_Z), c, "set it down", rot=rot0),
             W(0.4, P(TRAY_X, TRAY_Y, GAUGE_PAD_Z), o, "let go", rot=rot0),
             W(0.9, P(TRAY_X, TRAY_Y, TOP + 0.150), o, "up", rot=rot0),
             W(1.4, park_l, o, "park", rot=rot0)]
    tl, tr = sum(w.duration for w in left), sum(w.duration for w in right)
    end = max(tl, tr) + 3.5 + t_wait
    left.append(W(max(0.2, end - tl), park_l, o, "hold clear", rot=rot0))
    right.append(W(max(0.2, end - tr), park_r, o, "hold clear", rot=rot0))
    return {"left": left, "right": right}
