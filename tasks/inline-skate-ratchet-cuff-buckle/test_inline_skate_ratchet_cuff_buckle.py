"""Offline CPU unit tests for inline-skate-ratchet-cuff-buckle."""

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

import inline_skate_ratchet_cuff_buckle as t  # noqa: E402
import pitch_demo  # noqa: E402
from labkit import PartState  # noqa: E402
from yam_arena import constants as _C  # noqa: E402
from yam_arena import station  # noqa: E402

PARK = [np.array([0.16, 0.32, t.TOP + 0.18]), np.array([0.16, -0.32, t.TOP + 0.18])]


def quat_x(angle_rad: float) -> np.ndarray:
    return np.array([math.sin(angle_rad / 2.0), 0.0, 0.0, math.cos(angle_rad / 2.0)])


def make_cuff(wing=0.0, strap=0.0, lever=0.0, dx=0.0, dy=0.0, tilt_rad=0.0) -> PartState:
    pos = np.array([t.XC + dx, t.YC + dy, t.PAD_Z + 0.0005], float)
    return PartState(
        pos=pos, quat_xyzw=quat_x(tilt_rad), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
        low=pos - 0.09, high=pos + 0.09,
        joints={"wing": float(wing), "strap": float(strap), "lever": float(lever)},
    )


def eval_score(cuff_state, *, clear_steps=60, open_while_clear=0.0, tcps=PARK) -> float:
    t._reset(None)
    t._EP.update({
        "clear_steps": clear_steps,
        "open_while_clear": open_while_clear, "tcps": list(tcps),
    })
    return t.score({"cuff": cuff_state})


def test_noop_initial_state_zero():
    """Initial open cuff (wing +32 deg, strap -70 deg) must score 0.0."""
    c0 = make_cuff(wing=t.OPEN_TARGET, strap=t.STRAP_START, lever=0.0)
    assert eval_score(c0, clear_steps=0) == 0.0
    assert not t.success({"cuff": c0})


def test_ideal_completed_and_released_one():
    """Seated cuff, strap caught, lever flat <= 8 deg, wing <= 1.5 deg open, 3 s clear -> 1.0."""
    c_ok = make_cuff(wing=math.radians(0.8), strap=0.0, lever=math.radians(2.0))
    assert eval_score(c_ok, clear_steps=t.HOLD_STEPS, open_while_clear=math.radians(0.8)) == 1.0
    assert t.success({"cuff": c_ok})


def test_proximity_hover_still_grasped_neg_cap():
    """Grippers still within 5 cm of cuff envelope cannot score > 0.0 even when fully ratcheted."""
    c_ok = make_cuff(wing=0.0, strap=0.0, lever=0.0)
    near = [np.array([t.XC + 0.04, t.YC + 0.04, t.PAD_Z + 0.14]), PARK[1]]
    assert eval_score(c_ok, clear_steps=t.HOLD_STEPS, tcps=near) == 0.0
    t.watch({"wing": 0.0, "strap": 0.0, "lever": 0.0}, near, c_ok)
    assert t._EP["clear_steps"] == 0


def test_one_hand_displaced_base_and_unaligned_fail_ablation():
    """Pushing cuff off pad (> 25 mm / > 8 deg tilt), missing cuff, or feeding strap while wing > 3.5 deg fails."""
    assert eval_score(None, clear_steps=t.HOLD_STEPS) == 0.0
    c_slid = make_cuff(wing=0.0, strap=0.0, lever=0.0, dx=0.035)
    assert eval_score(c_slid, clear_steps=t.HOLD_STEPS) == 0.0
    c_tipped = make_cuff(wing=0.0, strap=0.0, lever=0.0, tilt_rad=math.radians(10.0))
    assert eval_score(c_tipped, clear_steps=t.HOLD_STEPS) == 0.0

    # Strap fed to buckle mouth without squeezing wing within 3.5 deg does not engage pawl
    t._reset(None)
    t.watch({"wing": math.radians(12.0), "strap": 0.0, "lever": 0.0}, PARK, make_cuff(wing=math.radians(12.0)))
    assert t._EP["engaged"] is False


def test_unlatched_or_incomplete_end_state_neg_subgates():
    """Every sub-gate (seated, strap >= -7 deg, wing <= 1.5/8 deg, lever_flat, clear, 3 s hold) drops score < 1.0."""
    c_ok = make_cuff(wing=0.0, strap=0.0, lever=0.0)
    assert eval_score(make_cuff(wing=0.0, strap=-math.radians(10.0), lever=0.0), clear_steps=t.HOLD_STEPS) == 0.0
    assert eval_score(make_cuff(wing=math.radians(9.0), strap=0.0, lever=0.0), clear_steps=t.HOLD_STEPS,
                      open_while_clear=math.radians(9.0)) == 0.0
    assert eval_score(make_cuff(wing=t.hold_angle(1), strap=0.0, lever=0.0), clear_steps=t.HOLD_STEPS,
                      open_while_clear=t.hold_angle(1)) == 0.5
    assert eval_score(make_cuff(wing=0.0, strap=0.0, lever=math.radians(15.0)), clear_steps=t.HOLD_STEPS) == 0.5
    assert eval_score(make_cuff(wing=math.radians(2.5), strap=0.0, lever=0.0), clear_steps=t.HOLD_STEPS,
                      open_while_clear=math.radians(2.5)) == 0.5
    assert eval_score(c_ok, clear_steps=t.HOLD_STEPS - 1) == 0.0


def test_possible_scores_and_strap_undo():
    """All possible scores (0.0, 0.5, 1.0) are reachable and pulling strap out past -7 deg unlatches ratchet."""
    c0 = make_cuff(wing=t.OPEN_TARGET, strap=t.STRAP_START)
    c_half = make_cuff(wing=t.hold_angle(1), strap=0.0, lever=0.0)
    c_one = make_cuff(wing=t.hold_angle(2), strap=0.0, lever=0.0)
    assert {
        eval_score(c0, clear_steps=t.HOLD_STEPS),
        eval_score(c_half, clear_steps=t.HOLD_STEPS, open_while_clear=t.hold_angle(1)),
        eval_score(c_one, clear_steps=t.HOLD_STEPS, open_while_clear=t.hold_angle(2)),
    } == {0.0, 0.5, 1.0}

    # Simulate full engagement + 2 lever pumps then withdraw strap past -STRAP_OUT -> unlatches to 0.0
    t._reset(None)
    t.watch({"wing": 0.0, "strap": 0.0, "lever": 0.0}, PARK, make_cuff(wing=0.0))
    for _ in range(2):
        t.watch({"wing": 0.0, "strap": 0.0, "lever": t.LEVER_UP + 0.05}, PARK, make_cuff(wing=0.0))
        t.watch({"wing": 0.0, "strap": 0.0, "lever": 0.0}, PARK, make_cuff(wing=0.0))
    assert t._EP["engaged"] is True and t._EP["notches"] == 2
    for _ in range(t.HOLD_STEPS):
        t.watch({"wing": 0.0, "strap": 0.0, "lever": 0.0}, PARK, make_cuff(wing=0.0))
    assert t.score({"cuff": c_one}) == 1.0
    c_undo = make_cuff(wing=t.OPEN_TARGET, strap=-math.radians(15.0), lever=0.0)
    t.watch(c_undo.joints, PARK, c_undo)
    assert t._EP["engaged"] is False and t._EP["notches"] == 0 and t.score({"cuff": c_undo}) == 0.0


def test_shortcuts_score_zero():
    """Shortcuts (noop, sweeping cuff away, feeding strap without squeezing wing, early release) score 0.0."""
    assert eval_score(make_cuff(wing=t.OPEN_TARGET, strap=t.STRAP_START), clear_steps=60) == 0.0
    assert eval_score(make_cuff(wing=0.0, strap=t.STRAP_START, dy=0.12), clear_steps=60) == 0.0
    assert eval_score(make_cuff(wing=t.OPEN_TARGET, strap=0.0), clear_steps=60, open_while_clear=t.OPEN_TARGET) == 0.0
    assert eval_score(make_cuff(wing=0.0, strap=0.0), clear_steps=10) == 0.0
    assert t.score({"cuff": make_cuff(wing=t.OPEN_TARGET, strap=t.STRAP_START)}) == 0.0


def test_plan_kinematic_reach_and_purity():
    """Demo uses constants only, passes IK reach cleanly, scoring constants match, and run_limit has >= 30% headroom."""
    plan = t.demo()
    err = pitch_demo.check_plan(plan, station.BASES)
    for arm, (pos_mm, rot_deg) in err.items():
        assert pos_mm < 2.0 and rot_deg < 2.0, (arm, pos_mm, rot_deg)
    dur = max(sum(w.duration for w in v) for v in plan.values())
    tj = json.loads((HERE / "task.json").read_text())
    steps = int(re.match(r"([\d,]+)\s*steps", tj["run_limit"]).group(1).replace(",", ""))
    assert steps >= math.ceil(dur * 20 * 1.3)
    names = set(t.demo.__code__.co_names)
    assert not names & {"env", "scene", "_EP", "score", "labkit", "ik"}
    assert (t.CUFF_DISP_TOL, t.CUFF_TILT_DEG, t.SLACK, t.PITCH, t.CLEAR, t.HOLD_S, t.HOLD_STEPS) == (
        0.025, 8.0, 0.014, 0.006, 0.05, 3.0, 60
    )


# -- occlusion / line-of-sight proxy to cuff, buckle, and ladder strap -------------------------------------------------
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


def _occluded(tcps, pts) -> bool:
    for tcp in tcps:
        q0 = np.asarray(tcp, float)
        q1 = q0 + np.array([0.0, 0.0, GRIP_LEN])
        for pt in pts:
            if _seg_dist(CAM, pt, q0, q1) <= GRIP_R:
                return True
    return False


def test_occlusion_top_camera_sight_lines_clear_at_demo_end():
    """Top camera rays to buckle channel, lever knob, and cuff squeeze ears are clear of both parked grippers."""
    d = t.demo()
    end_tcps = (np.array(d["left"][-1].pos), np.array(d["right"][-1].pos))
    targets = [
        np.array([t.XC, t.YC - 0.067, t.PAD_Z + 0.100]),
        np.array([t.XC, t.YC - 0.080, t.PAD_Z + 0.146]),
        np.array([t.XC + 0.038, t.YC + 0.040, t.PAD_Z + 0.136]),
    ]
    assert not _occluded(end_tcps, targets)
    assert _occluded((CAM + 0.75 * (targets[0] - CAM), end_tcps[1]), targets)
