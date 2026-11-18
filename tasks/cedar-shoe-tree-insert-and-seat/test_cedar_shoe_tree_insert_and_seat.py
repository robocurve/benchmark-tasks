"""Offline CPU unit tests for cedar-shoe-tree-insert-and-seat."""

import json
import math
import pathlib
import re
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import cedar_shoe_tree_insert_and_seat as t  # noqa: E402
import pitch_demo  # noqa: E402
from labkit import PartState  # noqa: E402
from yam_arena import constants as _C  # noqa: E402
from yam_arena import station  # noqa: E402

PARK_TCPS = (np.array([0.16, 0.33, t.TOP + 0.16]), np.array([0.16, -0.33, t.TOP + 0.16]))
PARK_PADS = (np.array([0.16, 0.33, t.TOP + 0.147]), np.array([0.16, -0.33, t.TOP + 0.147]))


def _quat_axis(axis, angle_rad: float) -> np.ndarray:
    ax = np.asarray(axis, float)
    ax = ax / (np.linalg.norm(ax) or 1.0)
    s = math.sin(angle_rad / 2.0)
    return np.array([ax[0] * s, ax[1] * s, ax[2] * s, math.cos(angle_rad / 2.0)], float)


def make_shoe(dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, tilt_deg: float = 0.0) -> PartState:
    pos = np.array([t.SHOE_X + dx, t.SHOE_Y + dy, t.MAT_TOP + dz], float)
    q = _quat_axis([1.0, 0.0, 0.0], math.radians(tilt_deg))
    return PartState(
        pos=pos, quat_xyzw=q, lin_vel=np.zeros(3), ang_vel=np.zeros(3),
        low=pos + np.array([-0.054, -0.152, 0.0]), high=pos + np.array([0.054, 0.153, 0.090]), joints={},
    )


def make_tree(pos=None, q_slide: float = 0.015, yaw_deg: float = 0.0, roll_deg: float = 0.0) -> PartState:
    p = np.array([t.SHOE_X, t.SHOE_Y + 0.066, t.MAT_TOP + t.INSOLE_Z], float) if pos is None else np.asarray(pos, float)
    qz = _quat_axis([0.0, 0.0, 1.0], math.radians(yaw_deg))
    qy = _quat_axis([0.0, 1.0, 0.0], math.radians(roll_deg))
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qy
    q = np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ], float)
    return PartState(
        pos=p, quat_xyzw=q, lin_vel=np.zeros(3), ang_vel=np.zeros(3),
        low=p + np.array([-0.034, -0.222 + q_slide, 0.0]), high=p + np.array([0.034, 0.070, 0.072]),
        joints={"spring_tube": float(q_slide)},
    )


def eval_score(monkeypatch, sh, tr, *, tcps=PARK_TCPS, pads=PARK_PADS) -> float:
    monkeypatch.setattr(t, "_live_arm_points", lambda: (tcps, pads))
    return t.score({"shoe": sh, "shoe_tree": tr})


def test_noop_nothing_initial_state_zero(monkeypatch):
    """Initial spawn layout (uncompressed tree beside empty shoe on mat) and missing objects score 0.0."""
    sh0 = make_shoe()
    tr0 = make_tree(pos=(t.TREE_X, t.TREE_Y, t.MAT_TOP + 0.0005), q_slide=0.0)
    assert eval_score(monkeypatch, sh0, tr0) == 0.0
    assert not t.success({"shoe": sh0, "shoe_tree": tr0})
    assert t.score({}) == 0.0
    assert t.distance({}) == 1.0


def test_ideal_seated_and_released_scores_one(monkeypatch):
    """Properly seated forepart and heel block with 15 mm active spring preload and clear grippers scores 1.0."""
    sh = make_shoe(dy=0.015)
    tr = make_tree(pos=(t.SHOE_X, t.SHOE_Y + 0.015 + 0.066, t.MAT_TOP + t.INSOLE_Z), q_slide=0.015)
    assert eval_score(monkeypatch, sh, tr) == 1.0
    assert t.success({"shoe": sh, "shoe_tree": tr})
    assert t.distance({"shoe": sh, "shoe_tree": tr}) < 0.01


def test_possible_scores_tiers_and_undo(monkeypatch):
    """Every score in 'Possible scores: 0.0, 0.35, 0.70, 1.0' is reachable statelessly, and unseating drops 1.0."""
    sh = make_shoe()
    tr_ok = make_tree(q_slide=0.015)
    tr_cocked_heel = make_tree(pos=(t.SHOE_X, t.SHOE_Y + 0.066, t.MAT_TOP + t.INSOLE_Z), q_slide=0.002)
    near_tcps = (PARK_TCPS[0], np.array([t.SHOE_X, t.SHOE_Y - 0.110, t.MAT_TOP + 0.095]))
    near_pads = (PARK_PADS[0], np.array([t.SHOE_X, t.SHOE_Y - 0.110, t.MAT_TOP + 0.082]))
    got = {
        eval_score(monkeypatch, sh, make_tree(pos=(t.TREE_X, t.TREE_Y, t.MAT_TOP), q_slide=0.0)),
        eval_score(monkeypatch, sh, tr_cocked_heel),
        eval_score(monkeypatch, sh, tr_ok, tcps=near_tcps, pads=near_pads),
        eval_score(monkeypatch, sh, tr_ok),
    }
    assert got == {0.0, 0.35, 0.70, 1.0}

    # Stateless undo check: popping heel up onto the 70 mm collar immediately drops 1.0 to 0.0
    assert eval_score(monkeypatch, sh, tr_ok) == 1.0
    tr_popped = make_tree(pos=(t.SHOE_X, t.SHOE_Y + 0.066, t.MAT_TOP + 0.072), q_slide=0.001)
    assert eval_score(monkeypatch, sh, tr_popped) == 0.0


def test_still_grasped_hover_negative_cap(monkeypatch):
    """Keeping either gripper within 80 mm or missing arm telemetry caps score at 0.70 (fail-closed)."""
    sh = make_shoe()
    tr = make_tree(q_slide=0.015)
    near_tcps = (np.array([t.SHOE_X, t.SHOE_Y + 0.130, t.MAT_TOP + 0.120]), PARK_TCPS[1])
    near_pads = (np.array([t.SHOE_X, t.SHOE_Y + 0.130, t.MAT_TOP + 0.107]), PARK_PADS[1])
    assert eval_score(monkeypatch, sh, tr, tcps=near_tcps, pads=near_pads) == 0.70
    assert eval_score(monkeypatch, sh, tr, tcps=None, pads=None) == 0.70
    assert not t.success({"shoe": sh, "shoe_tree": tr})


def test_one_hand_displaced_shoe_ablation_scores_zero(monkeypatch):
    """One-handed push sliding the unbraced shoe > 65 mm across the mat or tipping > 10 deg scores 0.0."""
    sh_slid = make_shoe(dy=0.072)
    tr_slid = make_tree(pos=(t.SHOE_X, t.SHOE_Y + 0.072 + 0.066, t.MAT_TOP + t.INSOLE_Z), q_slide=0.015)
    assert eval_score(monkeypatch, sh_slid, tr_slid) == 0.0

    sh_tipped = make_shoe(tilt_deg=14.0)
    assert eval_score(monkeypatch, sh_tipped, make_tree(q_slide=0.015)) == 0.0


def _cocked_heel_state(sh: PartState) -> PartState:
    pitch = -math.radians(11.0)
    q = _quat_axis([1.0, 0.0, 0.0], -pitch)
    Rt = t._rot(q)
    toe_target = np.asarray(sh.pos, float) + np.array([0.0, 0.133, 0.018])
    pt = toe_target - Rt @ np.array([0.0, t.TOE_TIP_Y, 0.002])
    return PartState(
        pos=pt, quat_xyzw=q, lin_vel=np.zeros(3), ang_vel=np.zeros(3),
        low=pt - 0.12, high=pt + 0.12, joints={"spring_tube": 0.015},
    )


def test_unlatched_subgate_negative_ablations(monkeypatch):
    """Flipping any single physical seating/preload/alignment sub-gate drops the score below 1.0."""
    sh = make_shoe()
    assert eval_score(monkeypatch, sh, make_tree(pos=(t.SHOE_X, t.SHOE_Y + 0.035, t.MAT_TOP + t.INSOLE_Z), q_slide=0.015)) == 0.0
    assert eval_score(monkeypatch, sh, make_tree(q_slide=0.015, yaw_deg=15.0)) == 0.0
    assert eval_score(monkeypatch, sh, make_tree(q_slide=0.015, roll_deg=18.0)) == 0.0
    assert eval_score(monkeypatch, sh, _cocked_heel_state(sh)) == 0.35
    assert eval_score(monkeypatch, sh, make_tree(pos=(t.SHOE_X, t.SHOE_Y + 0.082, t.MAT_TOP + t.INSOLE_Z), q_slide=0.004)) == 0.35
    assert eval_score(monkeypatch, sh, make_tree(pos=(t.SHOE_X, t.SHOE_Y + 0.058, t.MAT_TOP + t.INSOLE_Z), q_slide=0.027)) == 0.35


def test_shortcut_sweep_hover_and_drop_zero(monkeypatch):
    """Shortcuts (sweeping off mat, hovering over throat, dropping uncompressed across shoe) score 0.0."""
    sh = make_shoe()
    assert eval_score(monkeypatch, sh, make_tree(pos=(t.SHOE_X, t.SHOE_Y + 0.060, t.MAT_TOP + 0.072), q_slide=0.0)) == 0.0
    sh_off = make_shoe(dx=0.200)
    assert eval_score(monkeypatch, sh_off, make_tree(pos=(t.SHOE_X + 0.200, t.SHOE_Y + 0.066, t.MAT_TOP + t.INSOLE_Z))) == 0.0


def test_plan_kinematic_reach_purity_and_scoring_parity():
    """Demo uses constants only, passes IK (< 1 mm / < 1 deg), and matches task.json run_limit & thresholds."""
    plan = t.demo()
    err = pitch_demo.check_plan(plan, station.BASES)
    for arm, (pos_mm, rot_deg) in err.items():
        assert pos_mm < 1.0 and rot_deg < 1.0, (arm, pos_mm, rot_deg)
    dur = max(sum(w.duration for w in v) for v in plan.values())
    tj = json.loads((HERE / "task.json").read_text())
    steps = int(re.match(r"([\d,]+)\s*steps", tj["run_limit"]).group(1).replace(",", ""))
    assert steps >= math.ceil(dur * 20 * 1.3)
    assert len(tj["instruction"].split()) <= 28
    assert t.SUCCESS_HOLD_S == 3.0
    for fn in (t.demo, t._toe_tcp, t._heel_tcp):
        bad = set(fn.__code__.co_names) & {"env", "scene", "sim", "world", "state", "_ENV", "score", "labkit", "ik"}
        assert not bad, (fn.__name__, bad)


# -- top-camera sight-line / occlusion proxy check --------------------------------------------------------------------
CAM = np.array(station.top_camera("calibrated").pos, float)
GRIP_R = abs(float(_C.FINGER_OPEN))
GRIP_LEN = float(_C.TCP_OFFSET[2])


def _seg_dist(p0: np.ndarray, p1: np.ndarray, q0: np.ndarray, q1: np.ndarray) -> float:
    d1, d2, r = p1 - p0, q1 - q0, p0 - q0
    a, e, f = float(d1 @ d1), float(d2 @ d2), float(d2 @ r)
    s_ = min(max(-float(d1 @ r) / a, 0.0), 1.0) if a > 1e-12 else 0.0
    t_ = (float(d1 @ d2) * s_ + f) / e if e > 1e-12 else 0.0
    if t_ < 0.0:
        t_, s_ = 0.0, min(max(-float(d1 @ r) / a, 0.0), 1.0) if a > 1e-12 else 0.0
    elif t_ > 1.0:
        t_, s_ = 1.0, min(max((float(d1 @ d2) - float(d1 @ r)) / a, 0.0), 1.0) if a > 1e-12 else 0.0
    return float(np.linalg.norm((p0 + d1 * s_) - (q0 + d2 * t_)))


def _occluded(tcps, target_pts) -> bool:
    for tcp in tcps:
        q0 = np.asarray(tcp, float)
        q1 = q0 + np.array([0.0, 0.0, GRIP_LEN])
        for pt in target_pts:
            if _seg_dist(CAM, pt, q0, q1) <= GRIP_R:
                return True
    return False


def test_occlusion_top_camera_sight_lines_clear_at_demo_end():
    """Top-camera rays to oxford vamp, twin brass tubes, and seated cedar heel crest are clear of parked arms."""
    d = t.demo()
    end_tcps = (np.array(d["left"][-1].pos), np.array(d["right"][-1].pos))
    targets = [
        np.array([t.SHOE_X, t.SHOE_Y + 0.130, t.MAT_TOP + 0.050]),
        np.array([t.SHOE_X, t.SHOE_Y, t.MAT_TOP + 0.032]),
        np.array([t.SHOE_X, t.SHOE_Y - 0.115, t.MAT_TOP + 0.070]),
    ]
    assert not _occluded(end_tcps, targets)
    blocker = CAM + 0.72 * (targets[2] - CAM)
    assert _occluded((end_tcps[0], blocker), targets)
