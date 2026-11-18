"""Offline CPU unit tests for apple-wedger-press-through-to-board."""

import json
import math
import pathlib
import re
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import apple_wedger_press_through_to_board as t  # noqa: E402
import pitch_demo  # noqa: E402
from labkit import PartState  # noqa: E402
from yam_arena import constants as _C  # noqa: E402
from yam_arena import station  # noqa: E402

PARK = [np.array([0.18, 0.30, t.TOP + 0.16]), np.array([0.18, -0.30, t.TOP + 0.16])]


def quat_x(angle_rad: float) -> np.ndarray:
    return np.array([math.sin(angle_rad / 2.0), 0.0, 0.0, math.cos(angle_rad / 2.0)])


def make_fruit(dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, cut_m: float = 0.0, tilt_rad: float = 0.0) -> PartState:
    pos = np.array([t.BOARD_X + dx, t.BOARD_Y + dy, t.WELL_Z + dz], float)
    return PartState(pos=pos, quat_xyzw=quat_x(tilt_rad), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                     low=pos - 0.05, high=pos + 0.05, joints={"cut": -cut_m})


def make_wedger(pos=None, tilt_rad: float = 0.0) -> PartState:
    p = np.array([t.BOARD_X, t.BOARD_Y, t.DECK_Z] if pos is None else pos, float)
    return PartState(pos=p, quat_xyzw=quat_x(tilt_rad), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                     low=p - 0.08, high=p + 0.08, joints={})


def eval_score(wedger, fruit, *, tcps=PARK, hold_steps=60) -> float:
    t._reset(None)
    for _ in range(hold_steps):
        t._update_trial(wedger, fruit, list(tcps))
    if hold_steps == 0:
        t._update_trial(wedger, fruit, list(tcps))
        t._trial["hold_steps"] = 0
    return t.score({"wedger": wedger, "fruit": fruit})


def test_noop_initial_state_zero():
    """Initial spawn layout (wedger on mat, uncut apple in cradle) must score 0.0."""
    w0 = make_wedger(pos=(t.START_X, t.START_Y, t.TOP + t.MAT_H + 0.0005))
    f0 = make_fruit(cut_m=0.0)
    assert eval_score(w0, f0, hold_steps=0) == 0.0
    assert not t.success({"wedger": w0, "fruit": f0})


def test_ideal_completed_and_released_one():
    """Full 48 mm cut, level on board deck, grippers retracted > 120 mm for 3 s scores 1.0."""
    w = make_wedger(pos=(t.BOARD_X, t.BOARD_Y, t.DECK_Z + 0.0002), tilt_rad=math.radians(0.5))
    f = make_fruit(cut_m=t.CUT_STROKE)
    assert eval_score(w, f, tcps=PARK, hold_steps=t.HOLD_STEPS) == 1.0
    assert t.success({"wedger": w, "fruit": f})


def test_proximity_hover_still_grasped_neg_cap():
    """Even after full cut, a gripper still within 120 mm cannot reach 1.0 (capped at 0.50) and resets hold."""
    w = make_wedger(pos=(t.BOARD_X, t.BOARD_Y, t.DECK_Z))
    f = make_fruit(cut_m=t.CUT_STROKE)
    near = [np.array([t.BOARD_X, t.HANDLE_Y, t.DECK_Z + t.HANDLE_Z]), PARK[1]]
    assert eval_score(w, f, tcps=near, hold_steps=t.HOLD_STEPS) == 0.50
    t._reset(None)
    for _ in range(59):
        t._update_trial(w, f, PARK)
    assert t._trial["hold_steps"] == 59
    t._update_trial(w, f, near)
    assert t._trial["hold_steps"] == 0


def test_one_hand_cocked_fail_ablation():
    """1-hand press cocking > 5 deg without full cut or knocking apple out of 12 mm cradle fails < 1.0."""
    w_ok = make_wedger()
    f_out = make_fruit(dx=0.020, cut_m=t.CUT_STROKE)
    assert eval_score(w_ok, f_out) == 0.0

    # Cocked wedger (> 5 deg) stuck at shallow cut (< 10 mm) in physics fails at 0.0
    w_jam = make_wedger(pos=(t.BOARD_X, t.BOARD_Y, t.DECK_Z + 0.045), tilt_rad=math.radians(15.0))
    f_jam = make_fruit(cut_m=0.001)
    assert eval_score(w_jam, f_jam) == 0.0


def test_unlatched_or_incomplete_end_state_neg_subgates():
    """Flipping any single scorer sub-gate independently drops the score below 1.0."""
    f_full = make_fruit(cut_m=t.CUT_STROKE)
    w_full = make_wedger()
    assert eval_score(make_wedger(pos=(t.BOARD_X + 0.015, t.BOARD_Y, t.DECK_Z)), f_full) == 0.0
    assert eval_score(w_full, make_fruit(cut_m=0.005)) == 0.0
    assert eval_score(w_full, make_fruit(cut_m=0.030)) == 0.25
    assert eval_score(make_wedger(pos=(t.BOARD_X, t.BOARD_Y, t.DECK_Z + 0.008)), f_full) == 0.25
    assert eval_score(make_wedger(tilt_rad=math.radians(5.8)), f_full) == 0.25
    assert eval_score(w_full, f_full, hold_steps=t.HOLD_STEPS - 1) == 0.75


def test_possible_scores_and_undo():
    """All possible scores listed in task.json (0.0, 0.25, 0.50, 0.75, 1.0) are reachable and lifting wedger undoes cut pass."""
    w_full = make_wedger()
    f_full = make_fruit(cut_m=t.CUT_STROKE)
    near = [np.array([t.BOARD_X, t.HANDLE_Y, t.DECK_Z + t.HANDLE_Z]), PARK[1]]
    got = {
        eval_score(w_full, make_fruit(cut_m=0.0)),
        eval_score(w_full, make_fruit(cut_m=0.025)),
        eval_score(w_full, f_full, tcps=near, hold_steps=t.HOLD_STEPS),
        eval_score(w_full, f_full, tcps=PARK, hold_steps=1),
        eval_score(w_full, f_full, tcps=PARK, hold_steps=t.HOLD_STEPS),
    }
    assert got == {0.0, 0.25, 0.50, 0.75, 1.0}
    # Undo: lifting wedger off deck after 60 clear hold steps resets hold_steps and drops score to 0.25
    t._reset(None)
    for _ in range(t.HOLD_STEPS):
        t._update_trial(w_full, f_full, PARK)
    assert t.score({"wedger": w_full, "fruit": f_full}) == 1.0
    w_lifted = make_wedger(pos=(t.BOARD_X, t.BOARD_Y, t.DECK_Z + 0.020))
    t._update_trial(w_lifted, f_full, PARK)
    assert t.score({"wedger": w_lifted, "fruit": f_full}) == 0.25


def test_shortcuts_score_zero():
    """Listed shortcuts (noop, sweep away, lift/drop off-center, surface nick < 10 mm) score 0.0."""
    f0 = make_fruit(cut_m=0.0)
    assert eval_score(make_wedger(pos=(t.START_X, 0.14, t.TOP + 0.003)), f0) == 0.0
    assert eval_score(make_wedger(pos=(t.BOARD_X + 0.04, t.BOARD_Y - 0.05, t.DECK_Z)), f0) == 0.0
    assert eval_score(make_wedger(pos=(t.BOARD_X, t.BOARD_Y, t.DECK_Z + 0.035)), make_fruit(cut_m=0.004)) == 0.0


def test_plan_kinematic_reach_and_purity():
    """Demo uses constants only, passes IK reach within envelope, and run_limit has >= 30% headroom."""
    plan = t.demo()
    err = pitch_demo.check_plan(plan, station.BASES)
    for arm, (pos_mm, rot_deg) in err.items():
        assert pos_mm < 2.0 and rot_deg < 2.0, (arm, pos_mm, rot_deg)
    dur = max(sum(w.duration for w in v) for v in plan.values())
    tj = json.loads((HERE / "task.json").read_text())
    steps = int(re.match(r"([\d,]+)\s*steps", tj["run_limit"]).group(1).replace(",", ""))
    assert steps >= math.ceil(dur * 20 * 1.3)
    names = set(t.demo.__code__.co_names)
    assert not names & {"env", "scene", "_trial", "score", "labkit", "ik"}
    assert (t.CRADLE_TOL, t.CENTER_TOL, t.ENGAGE_CUT, t.CUT_MIN, t.DECK_TOL, t.LEVEL_DEG,
            t.CLEAR_DIST, t.HOLD_S, t.CUT_YIELD_M) == (0.012, 0.010, 0.010, 0.042, 0.004, 5.0,
                                                        0.120, 3.0, 0.0045)


# -- occlusion / line-of-sight proxy to scored board, apple, and wedger ------------------------------------------------
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
    """Top camera rays to apple core, blade ring, and board cradle are clear of both parked grippers."""
    d = t.demo()
    end_tcps = (np.array(d["left"][-1].pos), np.array(d["right"][-1].pos))
    targets = [
        np.array([t.BOARD_X, t.BOARD_Y, t.DECK_Z + 0.01]),
        np.array([t.BOARD_X + 0.04, t.BOARD_Y, t.DECK_Z + 0.03]),
        np.array([t.BOARD_X, t.HANDLE_Y, t.DECK_Z + t.HANDLE_Z]),
        np.array([t.BOARD_X, -t.HANDLE_Y, t.DECK_Z + t.HANDLE_Z]),
    ]
    assert not _occluded(end_tcps, targets)
    blocker = CAM + 0.75 * (targets[0] - CAM)
    assert _occluded((blocker, end_tcps[1]), targets)
