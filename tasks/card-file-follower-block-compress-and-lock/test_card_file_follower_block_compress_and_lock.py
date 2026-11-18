"""Offline CPU unit tests for card-file-follower-block-compress-and-lock."""

import json
import math
import pathlib
import re
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import card_file_follower_block_compress_and_lock as t  # noqa: E402
import pitch_demo  # noqa: E402
from labkit import PartState  # noqa: E402
from yam_arena import constants as _C  # noqa: E402
from yam_arena import station  # noqa: E402

PARK = [np.array([0.18, 0.30, t.TOP + 0.17]), np.array([0.18, -0.30, t.TOP + 0.17])]


def _quat_rx(deg: float) -> np.ndarray:
    r = math.radians(deg)
    return np.array([math.sin(r / 2.0), 0.0, 0.0, math.cos(r / 2.0)])


def make_states(*, tray_dp=(0.0, 0.0, 0.0), tray_tilt_deg: float = 0.0,
                div_rel=(0.0, t.SLOT_Y_OFF, 0.006), div_tilt_deg: float = 0.0,
                q_fol: float = 0.0145, q_pack: float = 0.0135, cl_deg: float = 1.0) -> dict:
    q = {
        "pack_slide": q_pack,
        "follower_slide": q_fol,
        "clutch_hinge": math.radians(cl_deg),
    }
    moved = {}
    if any(tray_dp) or tray_tilt_deg:
        moved["tray"] = ((t.TRAY_X + tray_dp[0], t.TRAY_Y + tray_dp[1], t.DECK_Z + tray_dp[2]),
                         tuple(_quat_rx(tray_tilt_deg)))
    pd = np.array([t.TRAY_X + tray_dp[0] + div_rel[0],
                   t.TRAY_Y + tray_dp[1] + div_rel[1],
                   t.DECK_Z + tray_dp[2] + div_rel[2]], float)
    dv = PartState(pos=pd, quat_xyzw=_quat_rx(div_tilt_deg), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                   low=pd - 0.03, high=pd + 0.03, joints={})
    return t.kit.fake_states(q=q, moved=moved, extra={"divider": dv}, deg=False)


def eval_score(*, tcps=PARK, **kw) -> float:
    return t.score(make_states(**kw), tcps=tcps)


def test_noop_initial_state_zero():
    """Initial spawn layout (follower locked 34 mm forward, divider in front cradle) scores 0.0."""
    s0 = make_states(div_rel=(0.0, t.DIV_RACK_Y - t.TRAY_Y, 0.0),
                     q_fol=t.FOLLOWER_START_M, q_pack=t.FOLLOWER_START_M, cl_deg=0.0)
    assert not t.follower_snug_and_locked(s0)
    assert t.score(s0, tcps=PARK) == 0.0
    assert not t.success(s0, tcps=PARK)
    assert eval_score(q_fol=t.FOLLOWER_START_M, cl_deg=0.0, tcps=PARK) == 0.25 < 1.0
    assert t.distance(s0) > 0.06


def test_ideal_slotted_snug_locked_and_clear_one():
    """Tray registered, divider upright in slot, follower in 12..26 mm & clutch <= 8 deg, grippers clear -> 1.0."""
    assert eval_score(q_fol=0.0145, cl_deg=1.0, tcps=PARK) == 1.0


def test_proximity_hover_on_handle_divider_follower_or_clutch_neg_cap():
    """Either gripper within 120 mm of rear handle, divider, follower paddle, or clutch lever caps score at 0.75."""
    s_ok = make_states()
    on_handle = [t.kit.point(s_ok, "tray", (t.STAB_X_OFF, t.STAB_Y_OFF, 0.022)), PARK[1]]
    on_div = [PARK[0], np.asarray(s_ok["divider"].pos, float)]
    on_fol = [PARK[0], t.kit.point(s_ok, "follower", (0.0, 0.014, 0.018))]
    on_cl = [PARK[0], t.kit.point(s_ok, "clutch_lever", (-0.046, -0.030, 0.008))]
    assert t.score(s_ok, tcps=on_handle) == 0.75
    assert t.score(s_ok, tcps=on_div) == 0.75
    assert t.score(s_ok, tcps=on_fol) == 0.75
    assert t.score(s_ok, tcps=on_cl) == 0.75
    assert not t.hands_clear(s_ok, tcps=on_handle)


def test_one_hand_unbraced_tray_slide_ablation_zero():
    """1-hand attempt without left stabilizer shifts unbolted 260 g tray > 16 mm on smooth blotter -> 0.0."""
    assert eval_score(tray_dp=(0.0, -0.024, 0.0)) == 0.0
    assert eval_score(tray_tilt_deg=8.0) == 0.0


def test_unreleased_clutch_divider_atop_closed_packs_neg_zero():
    """Dropping divider onto closed packs without releasing clutch (divider z = +30 mm high or tipped) scores 0.0."""
    assert eval_score(div_rel=(0.0, t.SLOT_Y_OFF, 0.030), q_fol=t.FOLLOWER_START_M) == 0.0
    assert eval_score(div_tilt_deg=25.0) == 0.0
    assert eval_score(div_rel=(0.030, t.SLOT_Y_OFF, 0.006)) == 0.0


def test_uncompressed_or_unlocked_follower_partial_tiers():
    """Divider slotted with open follower gives 0.25; compressed follower with open clutch (>8 deg) gives 0.50."""
    assert eval_score(q_fol=0.0, cl_deg=0.0) == 0.25
    assert eval_score(q_fol=0.008, cl_deg=0.0) == 0.25
    assert eval_score(q_fol=0.0145, cl_deg=14.0) == 0.50


def test_possible_scores_tiers_and_undo():
    """All documented Possible scores (0.0, 0.25, 0.50, 0.75, 1.0) are reachable from stateless components()."""
    tj = json.loads((HERE / "task.json").read_text())
    m = re.search(r"Possible scores\s+([0-9.,\s]+)", tj["scoring"])
    assert m is not None
    documented = {float(x) for x in re.findall(r"\d+\.\d+", m.group(1))}

    s_ok = make_states()
    near = [PARK[0], np.asarray(s_ok["divider"].pos, float)]
    got = {
        eval_score(div_rel=(0.0, t.DIV_RACK_Y - t.TRAY_Y, 0.0), q_fol=t.FOLLOWER_START_M),
        eval_score(q_fol=0.0, cl_deg=0.0),
        eval_score(q_fol=0.0145, cl_deg=14.0),
        t.score(s_ok, tcps=near),
        t.score(s_ok, tcps=PARK),
    }
    assert got == documented == {0.0, 0.25, 0.50, 0.75, 1.0}
    # Undo: releasing clutch so follower springs back to 0.0 immediately drops stateless score from 1.0 to 0.25
    assert eval_score(q_fol=0.0, cl_deg=0.5, tcps=PARK) == 0.25


def test_shortcuts_score_zero():
    """Shortcuts (noop, sweeping tray off desk mat, laying divider flat beside tray) score 0.0."""
    assert eval_score(div_rel=(0.0, -0.11, 0.0)) == 0.0
    assert eval_score(tray_dp=(0.04, -0.03, 0.0)) == 0.0
    assert eval_score(div_tilt_deg=90.0) == 0.0


def test_plan_kinematic_reach_and_purity():
    """Demo uses scene constants only, passes IK reach < 2 mm, and run_limit has >= 30% headroom."""
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
    assert (t.TRAY_XY_TOL, t.TRAY_Z_TOL, t.TRAY_TILT_DEG, t.DIVIDER_X_TOL,
            t.DIVIDER_Y_TOL, t.DIVIDER_Z_TOL, t.DIVIDER_TILT_DEG,
            t.FOLLOWER_SNUG_MIN, t.FOLLOWER_SNUG_MAX, t.CLUTCH_LOCK_DEG, t.CLEAR_DIST, t.SUCCESS_HOLD_S) == (
        0.016, 0.006, 5.0, 0.022, 0.015, 0.008, 10.0, 0.012, 0.026, 8.0, 0.120, 3.0
    )


# -- occlusion / line-of-sight proxy to scored tray, divider, follower block, clutch lever, and rear handle ------------
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
    """Top camera rays to card-file tray, slotted divider, follower block, clutch paddle, and handle clear grippers."""
    d = t.demo()
    end_tcps = (np.array(d["left"][-1].pos), np.array(d["right"][-1].pos))
    targets = [
        np.array([t.TRAY_X, t.TRAY_Y + t.SLOT_Y_OFF, t.DECK_Z + 0.035]),
        np.array([t.TRAY_X, t.TRAY_Y + 0.030, t.DECK_Z + 0.040]),
        np.array([t.TRAY_X - 0.082, t.TRAY_Y - 0.042, t.DECK_Z + 0.030]),
        np.array([t.TRAY_X + t.STAB_X_OFF, t.TRAY_Y + t.STAB_Y_OFF, t.DECK_Z + 0.025]),
    ]
    assert not _occluded(end_tcps, targets)
    blocker = CAM + 0.75 * (targets[0] - CAM)
    assert _occluded((blocker, end_tcps[1]), targets)
