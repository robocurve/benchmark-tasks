"""Offline CPU unit tests for hemocytometer-stage-clips-and-coverslip."""

import json
import math
import pathlib
import re
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import hemocytometer_stage_clips_and_coverslip as t  # noqa: E402
import pitch_demo  # noqa: E402
from labkit import PartState  # noqa: E402
from yam_arena import constants as _C  # noqa: E402
from yam_arena import station  # noqa: E402

PARK = [np.array([0.18, 0.30, t.TOP + 0.17]), np.array([0.18, -0.30, t.TOP + 0.17])]


def _quat_rx(deg: float) -> np.ndarray:
    r = math.radians(deg)
    return np.array([math.sin(r / 2.0), 0.0, 0.0, math.cos(r / 2.0)])


def make_states(*, ch_dp=(0.0, 0.005, 0.0), ch_tilt_deg: float = 0.0,
                clip_l_deg: float = 1.0, clip_r_deg: float = 1.0,
                cs_rel=(0.0, 0.0, t.MOAT_RAIL_H), cs_tilt_deg: float = 0.0) -> dict:
    pc = np.array([t.STAGE_X + ch_dp[0], t.STAGE_Y + ch_dp[1], t.DECK_Z + ch_dp[2]], float)
    ps = pc + np.array(cs_rel, float)
    ch = PartState(pos=pc, quat_xyzw=_quat_rx(ch_tilt_deg), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                   low=pc - 0.05, high=pc + 0.05, joints={})
    cs = PartState(pos=ps, quat_xyzw=_quat_rx(cs_tilt_deg), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                   low=ps - 0.03, high=ps + 0.03, joints={})
    q = {
        "datum_slide": 0.010,
        "clip_hinge_left": clip_l_deg,
        "clip_hinge_right": clip_r_deg,
        "detent_slide_left": 0.0,
        "detent_slide_right": 0.0,
    }
    return t.kit.fake_states(q=q, extra={"chamber": ch, "coverslip": cs}, deg=True)


def eval_score(*, tcps=PARK, **kw) -> float:
    return t.score(make_states(**kw), tcps=tcps)


def test_noop_initial_state_zero():
    """Initial spawn layout (chamber 34 mm outside pocket, clips 52 deg open, coverslip in cradle) scores 0.0."""
    s0 = make_states(ch_dp=(0.0, t.CHAMBER_START_DY, 0.0), clip_l_deg=t.CLIP_OPEN_DEG,
                     clip_r_deg=t.CLIP_OPEN_DEG, cs_rel=(0.0, t.CS_RACK_Y - (t.STAGE_Y + t.CHAMBER_START_DY), 0.0))
    assert t.score(s0, tcps=PARK) == 0.0
    assert not t.success(s0)
    assert t.distance(s0) > 0.12


def test_ideal_seated_clipped_covered_and_clear_one():
    """Chamber seated <= 14 mm, both clips latched <= 19 deg, coverslip flat on 10 mm rails, grippers clear -> 1.0."""
    assert eval_score(ch_dp=(0.0, 0.005, 0.0), clip_l_deg=1.0, clip_r_deg=1.0, tcps=PARK) == 1.0


def test_proximity_hover_on_stem_clip_or_coverslip_neg_cap():
    """Either gripper within 120 mm of chamber handling grip, stage clip, or coverslip caps score at 0.75."""
    s_ok = make_states()
    on_grip = [np.array([t.STAGE_X + t.STEM_X_OFF, t.STAGE_Y + 0.005 + t.STEM_Y_OFF, t.DECK_Z + 0.011]), PARK[1]]
    on_cs = [PARK[0], np.asarray(s_ok["coverslip"].pos, float)]
    on_clip = [PARK[0], t.kit.point(s_ok, "clip_left", (-0.004, 0.036, 0.004))]
    assert t.score(s_ok, tcps=on_grip) == 0.75
    assert t.score(s_ok, tcps=on_cs) == 0.75
    assert t.score(s_ok, tcps=on_clip) == 0.75
    assert not t.hands_clear(s_ok, tcps=on_grip)


def test_one_hand_unbraced_datum_ejection_ablation_zero():
    """1-hand attempt (left arm parked or early release -> 140 N/m datum plunger ejects chamber > 13 mm) scores 0.0."""
    assert eval_score(ch_dp=(0.0, t.CHAMBER_START_DY, 0.0)) == 0.0
    assert eval_score(ch_dp=(0.0, 0.016, 0.0), clip_l_deg=1.0, clip_r_deg=52.0) == 0.0


def test_single_clip_or_missing_coverslip_partial_tiers():
    """Chamber seated with 0 or 1 clip latched gives 0.25; both clips latched without coverslip gives 0.50."""
    assert eval_score(clip_l_deg=1.0, clip_r_deg=52.0, cs_rel=(0.0, -0.10, 0.0)) == 0.25
    assert eval_score(clip_l_deg=52.0, clip_r_deg=1.0, cs_rel=(0.0, -0.10, 0.0)) == 0.25
    assert eval_score(clip_l_deg=1.0, clip_r_deg=1.0, cs_rel=(0.0, -0.10, 0.0)) == 0.50


def test_wrong_tilted_inverted_or_off_rail_coverslip_neg():
    """Flipping any single geometric sub-gate drops score strictly below 1.0."""
    assert eval_score(ch_tilt_deg=8.0) == 0.0
    assert eval_score(cs_tilt_deg=10.0) == 0.50
    assert eval_score(cs_tilt_deg=180.0) == 0.50
    assert eval_score(cs_rel=(0.025, 0.0, t.MOAT_RAIL_H)) == 0.50
    assert eval_score(cs_rel=(0.0, 0.0, t.MOAT_RAIL_H + 0.009)) == 0.50


def test_possible_scores_tiers_and_undo():
    """All documented Possible scores (0.0, 0.25, 0.50, 0.75, 1.0) are reachable from stateless components()."""
    tj = json.loads((HERE / "task.json").read_text())
    m = re.search(r"Possible scores\s+([0-9.,\s]+)", tj["scoring"])
    assert m is not None
    documented = {float(x) for x in re.findall(r"\d+\.\d+", m.group(1))}

    s_ok = make_states()
    near = [PARK[0], np.asarray(s_ok["coverslip"].pos, float)]
    got = {
        eval_score(ch_dp=(0.0, t.CHAMBER_START_DY, 0.0), clip_l_deg=52.0, clip_r_deg=52.0),
        eval_score(clip_l_deg=1.0, clip_r_deg=52.0, cs_rel=(0.0, -0.10, 0.0)),
        eval_score(clip_l_deg=1.0, clip_r_deg=1.0, cs_rel=(0.0, -0.10, 0.0)),
        t.score(s_ok, tcps=near),
        t.score(s_ok, tcps=PARK),
    }
    assert got == documented == {0.0, 0.25, 0.50, 0.75, 1.0}
    # Undo: lifting coverslip off central rails immediately drops stateless score from 1.0 to 0.50
    assert eval_score(cs_rel=(0.0, -0.10, 0.04), tcps=PARK) == 0.50


def test_shortcuts_score_zero():
    """Shortcuts (noop, sweeping chamber off stage, setting coverslip on unseated chamber) score 0.0."""
    assert eval_score(ch_dp=(0.0, t.CHAMBER_START_DY, 0.0), cs_rel=(0.0, 0.0, t.MOAT_RAIL_H)) == 0.0
    assert eval_score(ch_dp=(0.05, -0.04, 0.0)) == 0.0
    assert eval_score(ch_dp=(0.0, 0.0, -0.02)) == 0.0


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
    assert (t.POCKET_XY_TOL, t.POCKET_Z_TOL, t.CHAMBER_TILT_DEG, t.CLIP_LATCH_DEG,
            t.COVERSLIP_XY_TOL, t.COVERSLIP_Z_TOL, t.COVERSLIP_TILT_DEG,
            t.MOAT_RAIL_H, t.CLEAR_DIST, t.SUCCESS_HOLD_S) == (
        0.014, 0.005, 5.0, 19.0, 0.018, 0.005, 6.0, 0.010, 0.120, 3.0
    )


# -- occlusion / line-of-sight proxy to scored counting chamber, stage clips, coverslip, and handling grip -------------
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
    """Top camera rays to seated counting chamber, both stage clips, coverslip, and grip clear parked grippers."""
    d = t.demo()
    end_tcps = (np.array(d["left"][-1].pos), np.array(d["right"][-1].pos))
    targets = [
        np.array([t.STAGE_X, t.STAGE_Y, t.DECK_Z + t.MOAT_RAIL_H + 0.015]),
        np.array([t.PIV_L[0], t.PIV_L[1] + 0.025, t.CLIP_Z]),
        np.array([t.PIV_R[0], t.PIV_R[1] + 0.025, t.CLIP_Z]),
        np.array([t.STAGE_X + t.STEM_X_OFF, t.STAGE_Y + t.STEM_Y_OFF, t.DECK_Z + 0.015]),
    ]
    assert not _occluded(end_tcps, targets)
    blocker = CAM + 0.75 * (targets[0] - CAM)
    assert _occluded((blocker, end_tcps[1]), targets)
