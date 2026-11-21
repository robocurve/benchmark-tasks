"""On the station table: one adjustable walking pole (a three-section aluminium trekking pole with two flip-lock
clamps, Black Diamond Trail Back class: 100-140 cm, about 267 g) lies across the table in front of the robot, its
grip to the robot's right and its tip to the robot's left. The lower clamp is closed at its factory setting and is
not part of the task. The upper flip-lock clamp (a cam lever on a split collar) is closed with the pole at 105 cm.
The owner's length, 120 cm, is marked on the middle shaft's printed length scale with a red line, 15 cm inside the
upper shaft.

Task (instruction): set the pole to the red mark and lock it there. The maker's procedure (Black Diamond trekking
pole instructions, FlickLock): "Hold the pole above the FlickLock to prevent the pole from rotating while pushing the
cam lever open with your thumb."; "Adjust pole to the desired length."; "'Snap' close the lever firmly."

Why two hands: with the lever open the collar still drags on the middle shaft (4 N); the pole lies loose on the table
and its grip and collar slide on the table at about 1.5 N, so pulling the middle shaft with one hand drags the whole
pole along the table and the length does not change; lifting or pressing the lever with no hand on the pole shoves the
pole about too. One hand holds the upper shaft above the clamp (as the maker says) while the other flips the lever
open, draws the middle shaft out to the mark and presses the lever shut, whose cam squeezes the collar (the holding
force builds over the last 25 degrees of its travel).

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
Pole frame: origin on the pole's axis at the upper clamp's lower edge (the edge the scale is read at), at
(0.240, -0.005), 13 mm above the table; the pole's axis along +y (toward the tip).
- grip: dual-density rubber grip 130 x 30 x 26 mm (flattened sides, it does not roll), from y -0.615 (top) to
  -0.485, then a foam grip extension 24 mm across to y -0.410.
- upper shaft: 18 mm aluminium tube to the clamp; upper clamp (black): a collar 25 mm across, 30 mm long, y -0.030 to
  0; its lever (aluminium) 46 x 15 x 9 mm, pivot across the pole 5 mm from the collar's lower edge, on top of the
  collar; closed it lies flat along the clamp toward the grip, open it stands up (opens up to 100 degrees).
- middle shaft: 16 mm tube, slides in the upper shaft on a prismatic joint (0 to 0.37 m of travel from the start); its
  scale: black lines every 5 cm from 100 to 140 cm, read at the clamp's lower edge; the red owner's line at 120 cm;
  the maker's STOP line at 140 cm. Lower clamp (closed, part of the middle section), lower shaft 14 mm, rubber tip.
- masses: upper section with grip and clamp 0.150 kg, middle and lower sections 0.111 kg, lever 0.006 kg.

The clamp (modelled from the cam's working): the middle shaft's sliding friction in the collar is 4 N with the lever
open more than 25 degrees and rises linearly to 250 N as the lever closes from 25 to 2 degrees (fully closed: 2
degrees or less). The lever's pivot friction is 0.06 N m open and rises the same way to 0.20 N m (the cam's squeeze,
about 5 N at the lever's end).

Score (read every 50 ms step; length = grip top to tip, 105 cm at the start, 120 cm at the red line):
- 0.4: the length has been within 1 cm of 120 cm (the red line within 1 cm of the clamp's edge) at some time.
- 0.7: locked at the mark: the lever fully closed (within 5 degrees of flat) with the length within 1 cm of 120 cm.
- 1.0 (success, held 1 s): locked at the mark, released (both grippers' centre points 4 cm or more from the pole's
  axis), still for 1 s, and then the load test passed: once released and still for 1 s, the scorer pushes the pole
  together along its axis (a force across the slide, as a walker leaning on it) for 1 s and the shaft must slip 2 mm
  or less with the lever still within 5 degrees of flat. The simulated lean is 20 N: PhysX's joint friction creeps
  under sustained loads well below its set value (at 40 N the closed lever creeps open past 5 degrees, at 80 N the
  shaft collapses), so the lean is scaled down; a real trial presses 150 N.
- 0 for the rest of the run: any part of the pole off the table (below the table top by 2 cm), or the middle shaft
  drawn past its STOP line (140 cm), which the maker warns weakens the pole.
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

from bodykit import BoxShape, CylShape, Link, Linkage  # noqa: E402
from bodykit import allow_matrix_grips, as_numpy, on_reset, on_step, rotation  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

GRIP_C = (0.12, 0.12, 0.13)
FOAM_C = (0.20, 0.20, 0.22)
UPPER_C = (0.16, 0.30, 0.52)
CLAMP_C = (0.08, 0.08, 0.09)
LEVER_C = (0.78, 0.79, 0.82)
MID_C = (0.70, 0.71, 0.74)
LOWER_C = (0.16, 0.30, 0.52)
LINE_C = (0.05, 0.05, 0.06)
RED_C = (0.85, 0.08, 0.08)
TIP_C = (0.06, 0.06, 0.06)


def _v(p) -> tuple:
    return tuple(float(x) for x in p)


# -- the pole (a floating articulation: the upper section, the middle section on a slide, the lever on its pivot) ----
PX = 0.240
EDGE_Y = -0.005  # the upper clamp's lower edge (world y at the start)
AZ = TOP + 0.013  # the pole's axis height lying on its grip and clamp
GRIP_TOP, GRIP_BOT, FOAM_BOT = -0.615, -0.485, -0.410  # pole frame (y from the clamp edge)
UPPER_R, MID_R, LOWER_R, TIP_R = 0.009, 0.008, 0.007, 0.006
COLLAR_R, COLLAR_L = 0.0125, 0.030
PIVOT = np.array([0.0, -0.005, COLLAR_R + 0.0045])  # the lever's pivot (pole frame)
LEVER_L, LEVER_W, LEVER_T = 0.046, 0.015, 0.009
MID_IN, MID_OUT = -0.400, 0.120  # the middle shaft's ends (pole frame, at the start)
LCOL_L, LOWER_END, TIP_L = 0.025, 0.395, 0.040
L0 = (LOWER_END + TIP_L) - GRIP_TOP  # 1.050 m: the length at the start
MARK_CM = 120.0
STOP_CM = 140.0
Q_MARK = MARK_CM / 100.0 - L0  # 0.150 m
Q_STOP = STOP_CM / 100.0 - L0  # 0.350 m
Q_MAX = 0.370  # the slide's end (the shaft's last 3 cm stay in the collar)

F_OPEN = float(os.environ.get("WS_F_OPEN", "4.0"))  # N: the open collar's drag
F_CLOSED = float(os.environ.get("WS_F_CLOSED", "250.0"))  # N: the closed collar's hold
T_OPEN, T_CLOSED = 0.06, 0.20  # N m: the lever's pivot friction, open and in the cam
ENGAGE_DEG, FULL_DEG = 25.0, 2.0
TEST_N = float(os.environ.get("WS_TEST_N", "20.0"))  # N: the simulated lean (a real trial presses 150 N)
VEL_ITERS = int(os.environ.get("WS_VEL_ITERS", "32"))
LEVER_LOCKED = 5.0  # N m: the lever's hold while the load test runs (the closed cam is over its centre)


def _scale_s(cm: float) -> float:
    """Where the scale line for ``cm`` sits on the middle shaft (pole frame at the start)."""
    return -(cm / 100.0 - L0)


def _pole() -> Linkage:
    o = np.array([PX, EDGE_Y, AZ])

    def at(y, z=0.0, x=0.0):
        return _v((x, y, z))

    upper = [BoxShape(at((GRIP_TOP + GRIP_BOT) / 2), (0.030, GRIP_BOT - GRIP_TOP, 0.026), GRIP_C),
             BoxShape(at(GRIP_TOP + 0.004, 0.0), (0.032, 0.008, 0.028), GRIP_C),
             CylShape(at((GRIP_BOT + FOAM_BOT) / 2), 0.012, FOAM_BOT - GRIP_BOT, FOAM_C, axis="Y"),
             CylShape(at((FOAM_BOT - COLLAR_L) / 2), UPPER_R, -COLLAR_L - FOAM_BOT, UPPER_C, axis="Y"),
             CylShape(at(-COLLAR_L / 2), COLLAR_R, COLLAR_L, CLAMP_C, axis="Y"),
             BoxShape(at(PIVOT[1], PIVOT[2] - 0.002), (0.018, 0.010, 0.006), CLAMP_C)]
    lever_c = PIVOT + np.array([0.0, -LEVER_L / 2 + 0.003, 0.0])
    lever = [BoxShape(at(lever_c[1], lever_c[2]), (LEVER_W, LEVER_L, LEVER_T), LEVER_C),
             BoxShape(at(PIVOT[1] - LEVER_L + 0.006, PIVOT[2] + 0.005), (LEVER_W, 0.006, 0.003), LEVER_C)]
    mid = [CylShape(at((MID_IN + MID_OUT) / 2), MID_R, MID_OUT - MID_IN, MID_C, axis="Y")]
    for cm in range(100, 141, 5):
        s = _scale_s(cm)
        if MID_IN + 0.01 < s < MID_OUT - 0.01:
            mid.append(CylShape(at(s), MID_R + 0.0002, 0.0012, LINE_C, collide=False, axis="Y"))
    mid.append(CylShape(at(_scale_s(MARK_CM)), MID_R + 0.0003, 0.004, RED_C, collide=False, axis="Y"))
    mid.append(CylShape(at(_scale_s(STOP_CM) - 0.004), MID_R + 0.0002, 0.003, LINE_C, collide=False, axis="Y"))
    mid += [CylShape(at(MID_OUT + LCOL_L / 2), 0.011, LCOL_L, CLAMP_C, axis="Y"),
            BoxShape(at(MID_OUT + 0.010, 0.0135), (LEVER_W, 0.030, 0.005), LEVER_C),
            CylShape(at((MID_OUT + LCOL_L + LOWER_END) / 2), LOWER_R, LOWER_END - MID_OUT - LCOL_L, LOWER_C,
                     axis="Y"),
            CylShape(at(LOWER_END + TIP_L / 2), TIP_R, TIP_L, TIP_C, axis="Y")]
    links = [Link("slide", tuple(mid), 0.111, anchor=_v(o), axis="Y", kind="prismatic", limits=(0.0, Q_MAX),
                  friction=F_CLOSED, damping=0.5),
             Link("lever", tuple(lever), 0.006, anchor=_v(o + PIVOT), axis="X", limits=(-100.0, 0.5),
                  friction=T_CLOSED, damping=0.002)]
    return Linkage.build("pole", pos=_v(o + np.array([0.0, 0.0, 0.001])), mass=0.150, shapes=upper, links=links,
                         color=UPPER_C, floating=True, friction=(0.6, 0.5), contact_offset=0.001,
                         solver_iterations=32, velocity_iterations=VEL_ITERS)


POLE = _pole()
OBJECTS = [POLE]

# ======================================================================================================================
# The clamp, tracking and scoring
# ======================================================================================================================
LEN_TOL = 0.010
LOCK_DEG = 5.0
CLEAR = 0.040
STILL_STEPS, STILL_MM = 20, 1.5
TEST_STEPS, TEST_RAMP, SLIP_MM = 20, 4, 2.0
_DEBUG = os.environ.get("WS_DEBUG") == "1"
_DEBUG_EVERY = int(os.environ.get("WS_DEBUG_EVERY", "5"))
_EP: dict = {}
_IDS: dict = {}


def clamp_force(open_deg: float) -> float:
    """The collar's sliding friction (N) with the lever open ``open_deg`` degrees."""
    k = min(1.0, max(0.0, (ENGAGE_DEG - open_deg) / (ENGAGE_DEG - FULL_DEG)))
    return F_OPEN + (F_CLOSED - F_OPEN) * k


def lever_friction(open_deg: float) -> float:
    k = min(1.0, max(0.0, (ENGAGE_DEG - open_deg) / (ENGAGE_DEG - FULL_DEG)))
    return T_OPEN + (T_CLOSED - T_OPEN) * k


@on_reset
def _reset() -> None:
    _EP.update(tick=0, step=0, q=0.0, open_deg=0.0, reached=False, locked=False, released=False, still=False,
               recent=[], tcp={}, gaps={}, near=1.0, off=False, over=False, still_n=0, test=None, test_ok=False,
               test_q=None, slip=None, tests=0, fric=None, lfric=None, low=0.0)


_reset()


def _write_friction(asset, j: int, val: float) -> None:
    import torch

    t = torch.tensor([[float(val)]], dtype=torch.float32, device=asset.device)
    asset.write_joint_friction_coefficient_to_sim_index(joint_friction_coeff=t, joint_dynamic_friction_coeff=0.95 * t,
                                                        joint_ids=[j])


def _apply_test(asset, d: np.ndarray, f: float) -> None:
    """Push the pole together along its axis with ``f`` (N): a force across the slide, on the middle section toward
    the grip and, as its reaction, on the upper section toward the tip (as a person leaning on the grip with the tip
    on the floor)."""
    import torch

    val = torch.tensor([[-float(f)]], dtype=torch.float32, device=asset.device)
    asset.actuators.target_command.set_effort_index(value=val, joint_ids=[_IDS["slide"]])


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("WS_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _seg_dist(p, a, b) -> float:
    ab = b - a
    t = float(np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def _track_inner(env) -> None:
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
        _EP["gaps"][side] = float(np.abs(q[_IDS[side + "_f"]]).sum())
    pole = base.scene["pole"]
    if "slide" not in _IDS:
        jn, bn = list(pole.joint_names), list(pole.body_names)
        _IDS.update(slide=jn.index("slide_turn"), lever=jn.index("lever_turn"), b_base=bn.index("base"),
                    b_slide=bn.index("slide"), b_lever=bn.index("lever"))
    p = as_numpy(pole.data.root_link_pose_w)[0]
    pp, pr = p[:3] - origin, rotation(p[3:7])
    jq = as_numpy(pole.data.joint_pos)[0]
    qs, ql = float(jq[_IDS["slide"]]), float(jq[_IDS["lever"]])
    open_deg = -math.degrees(ql)
    # the clamp: friction follows the lever (written only when it changes)
    f = round(clamp_force(open_deg), 1)
    if _EP["fric"] != f:
        _write_friction(pole, _IDS["slide"], f)
        _EP["fric"] = f
    t = round(lever_friction(open_deg), 3)
    if _EP["test"] is not None:
        t = LEVER_LOCKED  # a closed cam lever is over its centre: a load on the collar does not open it
    if _EP["lfric"] != t:
        _write_friction(pole, _IDS["lever"], t)
        _EP["lfric"] = t
    d = pr[:, 1]  # the pole's axis, toward the tip
    a = pp + d * GRIP_TOP
    b = pp + d * (LOWER_END + TIP_L + qs)
    _EP["near"] = min(_seg_dist(_EP["tcp"][s], a, b) for s in ("left", "right"))
    body_z = as_numpy(pole.data.body_pos_w)[0][:, 2] - origin[2]
    _EP["low"] = float(min(body_z.min(), a[2], b[2]) - TOP)
    if _EP["low"] < -0.02:
        _EP["off"] = True
    if qs > Q_STOP + 0.002:
        _EP["over"] = True
    length = L0 + qs
    at_mark = abs(length - MARK_CM / 100.0) <= LEN_TOL
    _EP["reached"] = _EP["reached"] or at_mark
    locked = at_mark and open_deg <= LOCK_DEG
    released = _EP["near"] >= CLEAR
    recent = _EP["recent"]
    recent.append(np.concatenate([pp, [qs]]))
    del recent[:-STILL_STEPS]
    still = len(recent) == STILL_STEPS and all(np.abs(r - recent[-1]).max() * 1000 < STILL_MM for r in recent)
    # the load test: once locked, released and still for 1 s, push the pole together with 150 N for 1 s
    if _EP["test"] is not None:
        n = _EP["test"] + 1
        _EP["test"] = n
        if n <= TEST_STEPS:
            _apply_test(pole, d, TEST_N * min(1.0, n / TEST_RAMP))
        else:
            _apply_test(pole, d, 0.0)
            _EP["slip"] = abs(qs - _EP["test_q"]) * 1000
            _EP["test_ok"] = bool(_EP["slip"] <= SLIP_MM and locked)
            _EP["tests"] += 1
            _EP["test"] = None
            if _DEBUG:
                print(f"DBG load test {_EP['tests']}: slip {_EP['slip']:.2f} mm ok={int(_EP['test_ok'])}", flush=True)
    elif locked and released and still and not _EP["test_ok"]:
        _EP["test"], _EP["test_q"] = 0, qs
    if _EP["test_ok"] and (not locked or abs(qs - _EP["test_q"]) * 1000 > SLIP_MM + 1.0):
        _EP["test_ok"] = False  # unlocked or moved since: it has to be tested again
    if _DEBUG:
        bp = as_numpy(pole.data.body_pos_w)[0][_IDS["b_lever"]] - origin
        bq = as_numpy(pole.data.body_quat_w)[0][_IDS["b_lever"]]
        _EP["lever_tip"] = bp + rotation(bq) @ np.array([0.0, PIVOT[1] - LEVER_L + 0.003, PIVOT[2]])
    _EP.update(q=qs, open_deg=open_deg, locked=locked, released=released, still=still, length=length, pp=pp)


def _tally() -> dict:
    bad = _EP["off"] or _EP["over"]
    testing = _EP["test"] is not None
    done = bool(_EP["locked"] and _EP["released"] and _EP["test_ok"] and not testing and not bad)
    return dict(bad=bad, done=done)


def _debug(t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    print(f"DBG step={_EP['step']} len={_EP.get('length', L0) * 100:.2f}cm q={_EP['q'] * 1000:.1f}mm "
          f"lever_open={_EP['open_deg']:.1f}deg fric={_EP['fric']}N lfric={_EP['lfric']} reached={int(_EP['reached'])} "
          f"locked={int(_EP['locked'])} released={int(_EP['released'])} near={_EP['near'] * 1000:.0f}mm "
          f"still={int(_EP['still'])} test={_EP['test']} test_ok={int(_EP['test_ok'])} slip={_EP['slip']} "
          f"tip=({_EP.get('lever_tip', np.zeros(3))[1]:.3f},{_EP.get('lever_tip', np.zeros(3))[2] - TOP:.3f}) "
          f"low={_EP['low'] * 1000:.1f}mm off={int(_EP['off'])} over={int(_EP['over'])} "
          f"pole=({_EP.get('pp', np.zeros(3))[0]:.3f},{_EP.get('pp', np.zeros(3))[1]:.3f}) | "
          + " ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{v[2] - TOP:.3f}) gap={_EP['gaps'][k] * 1000:.0f}"
                     for k, v in _EP["tcp"].items()), flush=True)


def score(objects) -> float:
    """0.4 reached the mark at some time; 0.7 locked at the mark; 1.0 success (locked at the mark, released, still and
    the 150 N load test passed). 0 for the rest of the run once the pole is off the table or drawn past its STOP."""
    t = _tally()
    if _DEBUG:
        _debug(t)
    if t["bad"]:
        return 0.0
    if t["done"]:
        return 1.0
    if _EP["locked"]:
        return 0.7
    if _EP["reached"]:
        return 0.4
    return 0.0


def success(objects) -> bool:
    return bool(_tally()["done"])


# ======================================================================================================================
# Scripted demo (open loop, from the known layout)
# ======================================================================================================================
OPEN_C = -0.047
R_RIGHT = grip_rotation(math.pi / 2)  # fingers close along x (across the pole)
R_LEFT = grip_rotation(-math.pi / 2)
TUBE_PINCH = float(os.environ.get("WS_TUBE_PINCH", "-0.0045"))  # on the 18 and 16 mm tubes
LEVER_PINCH = float(os.environ.get("WS_LEVER_PINCH", "-0.0035"))  # on the lever's 14 mm width
HOLD_Y = float(os.environ.get("WS_HOLD_Y", "-0.270"))  # the right hand on the upper shaft (pole frame y)
PULL_Y = float(os.environ.get("WS_PULL_Y", "0.080"))  # the left hand on the middle shaft (pole frame y)
PULL_OVER = float(os.environ.get("WS_PULL_OVER", "0.025"))  # commanded past the mark (the arm lags under load)
LEVER_R = float(os.environ.get("WS_LEVER_R", "0.037"))  # pinch on the lever this far from its pivot
OPEN_TO = float(os.environ.get("WS_OPEN_TO", "108"))  # deg
PRESS_DZ = float(os.environ.get("WS_PRESS_DZ", "0.016"))  # the press is commanded this far below the closed lever
SWEEP_H = float(os.environ.get("WS_SWEEP_H", "0.025"))  # the closing fingertip sweeps this high above the pivot
PRESS_S = float(os.environ.get("WS_PRESS_S", "0.037"))  # and presses the lever this far from its pivot
TUBE_DZ = float(os.environ.get("WS_TUBE_DZ", "0.003"))  # TCP above the axis on a tube
LEVER_DZ = float(os.environ.get("WS_LEVER_DZ", "0.0070"))  # TCP above the lever's mid-plane
ONE = os.environ.get("WS_ONE") == "1"  # wrong way: the left arm alone (nobody holds the upper shaft)
ONE_PULL = os.environ.get("WS_ONE_PULL") == "1"  # wrong way: the right hand lets go once the lever is open
UP = np.array([0.0, 0.0, 1.0])


def pole_w(y: float, z: float = 0.0) -> np.ndarray:
    return np.array([PX, EDGE_Y + y, AZ + z])


def lever_point(deg_open: float, r: float = None) -> np.ndarray:
    """The lever's mid-plane point ``r`` from the pivot with the lever open ``deg_open`` (world)."""
    r = LEVER_R if r is None else r
    a = math.radians(deg_open)
    return pole_w(PIVOT[1] - r * math.cos(a), PIVOT[2] + r * math.sin(a))


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


def _stay(arm: str, t: float) -> list:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return [Waypoint(t, tuple(float(v) for v in pos), OPEN_C, "stay put", rot)]


def _dur(path) -> float:
    return float(sum(w.duration for w in path)) if path else 0.0


def _hold_until(path: list, t: float, label: str) -> None:
    last = path[-1]
    gap = t - _dur(path)
    if gap > 0:
        path.append(Waypoint(gap, last.pos, last.finger, label, last.yaw))


def _arc(path: list, deg0: float, deg1: float, seconds: float, finger: float, rot, label: str, n: int = 12) -> None:
    for k in range(1, n + 1):
        a = deg0 + (deg1 - deg0) * k / n
        path.append(_w(seconds / n, lever_point(a) + UP * LEVER_DZ, finger, label, rot))


def demo():
    """The right hand holds the upper shaft above the clamp the whole time (as the maker says); the left hand flips the
    lever open, draws the middle shaft out to the red line, lets go of it (the open collar's drag keeps it there) and
    sweeps the lever back down and presses it shut. Open loop, from the known layout."""
    h = pole_w(HOLD_Y, TUBE_DZ)
    right = _lead_in("right", h + UP * 0.05, R_RIGHT, OPEN_C, "over the upper shaft")
    right += [_w(1.5, h, OPEN_C, "down round the upper shaft above the clamp", R_RIGHT),
              _w(1.0, h, TUBE_PINCH, "hold the upper shaft", R_RIGHT)]
    t_held = _dur(right)
    left = _lead_in("left", lever_point(0.0) + UP * (LEVER_DZ + 0.05), R_LEFT, OPEN_C, "above the lever's end")
    left += [_w(1.5, lever_point(0.0) + UP * LEVER_DZ, OPEN_C, "fingers either side of the lever's end", R_LEFT)]
    _hold_until(left, t_held + 0.5, "wait for the other hand to hold the pole")
    left.append(_w(1.0, lever_point(0.0) + UP * LEVER_DZ, LEVER_PINCH, "pinch the lever", R_LEFT))
    _arc(left, 0.0, OPEN_TO, 3.0, LEVER_PINCH, R_LEFT, "flip the lever open")
    top = lever_point(OPEN_TO) + UP * LEVER_DZ
    left += [_w(0.8, top, OPEN_C, "let go of the lever", R_LEFT),
             _w(1.0, top + UP * 0.04, OPEN_C, "rise clear", R_LEFT)]
    t_open = _dur(left)
    g0 = pole_w(PULL_Y, TUBE_DZ)
    g1 = pole_w(PULL_Y + Q_MARK + PULL_OVER, TUBE_DZ)
    left += [_w(2.0, g0 + UP * 0.05, OPEN_C, "over the middle shaft", R_LEFT),
             _w(1.5, g0, OPEN_C, "down round the middle shaft", R_LEFT),
             _w(1.0, g0, TUBE_PINCH, "grip the middle shaft", R_LEFT),
             _w(5.0, g1, TUBE_PINCH, "draw it out to the red line", R_LEFT),
             _w(1.0, g1, TUBE_PINCH, "hold it at the mark", R_LEFT),
             _w(0.8, g1, OPEN_C, "let go of the middle shaft", R_LEFT),
             _w(1.0, g1 + UP * 0.05, OPEN_C, "rise clear", R_LEFT)]
    # the closing finger (fingers shut): across behind the open lever 25 mm above its pivot, sweeping it down toward
    # the grip, then pressing its end down flat onto the collar
    hz = PIVOT[2] + SWEEP_H + 0.013
    s0, s1 = pole_w(PIVOT[1] + 0.040, hz), pole_w(PIVOT[1] - 0.017, hz)
    s2 = pole_w(PIVOT[1] - PRESS_S, hz)
    s3 = pole_w(PIVOT[1] - PRESS_S, PIVOT[2] + LEVER_T / 2 + 0.013 - PRESS_DZ)
    left += [_w(2.5, s0 + UP * 0.05, 0.0, "fingers shut, over the far side of the open lever", R_LEFT),
             _w(1.2, s0, 0.0, "down behind the lever", R_LEFT),
             _w(2.0, s1, 0.0, "sweep the lever over toward the grip", R_LEFT),
             _w(1.0, s2, 0.0, "over the lever's end", R_LEFT),
             _w(1.5, s3, 0.0, "press the lever down flat onto the collar", R_LEFT),
             _w(0.8, s3, 0.0, "keep it pressed shut", R_LEFT),
             _w(1.2, s3 + UP * 0.08, 0.0, "rise clear", R_LEFT)]
    if ONE:
        return {"left": left, "right": _stay("right", _dur(left) + 2.0)}
    if ONE_PULL:  # the wrong way: the shaft is pulled and the lever closed with nobody holding the pole
        _hold_until(right, t_open, "hold the upper shaft")
        right += [_w(0.8, h, OPEN_C, "let go of the upper shaft", R_RIGHT),
                  _w(1.2, h + UP * 0.08, OPEN_C, "rise clear", R_RIGHT)]
        tend = _dur(left) + 5.0
        _hold_until(left, tend, "stay clear")
        _hold_until(right, tend, "stay clear")
        return {"left": left, "right": right}
    _hold_until(right, _dur(left), "hold the upper shaft")
    right += [_w(0.8, h, OPEN_C, "let go of the upper shaft", R_RIGHT),
              _w(1.2, h + UP * 0.08, OPEN_C, "rise clear", R_RIGHT)]
    tend = max(_dur(left), _dur(right)) + 5.0
    _hold_until(left, tend, "stay clear")
    _hold_until(right, tend, "stay clear")
    return {"left": left, "right": right}
