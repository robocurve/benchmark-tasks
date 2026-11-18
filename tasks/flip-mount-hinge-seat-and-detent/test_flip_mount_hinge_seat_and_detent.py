"""Offline CPU unit tests for flip-mount-hinge-seat-and-detent."""

import json
import math
import pathlib
import re
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import flip_mount_hinge_seat_and_detent as t  # noqa: E402
import pitch_demo  # noqa: E402
from yam_arena import constants as _C  # noqa: E402
from yam_arena import station  # noqa: E402

PARK = [np.array([0.18, 0.30, t.TOP + 0.17]), np.array([0.18, -0.30, t.TOP + 0.17])]


def make_states(*, slide: float = 0.0, clamp_deg: float = 1.0, hinge_deg: float = 1.0,
                detent: float = 0.0, post_dp=(0.0, 0.0, 0.0), post_tilt_deg: float = 0.0) -> dict:
    q = {
        "collar_slide": slide,
        "clamp_screw": clamp_deg,
        "flip_hinge": hinge_deg,
        "detent_slide": detent,
        "keeper_slide": 0.0,
    }
    moved = {}
    if any(post_dp) or post_tilt_deg:
        dx, dy, dz = post_dp
        moved["post_assy"] = ((t.POST_X + dx, t.POST_Y + dy, t.DECK_Z + dz), t._quat_rx(post_tilt_deg))
    return t.kit.fake_states(q=q, moved=moved, deg=True)


def eval_score(*, tcps=PARK, **kw) -> float:
    return t.score(make_states(**kw), tcps=tcps)


def test_noop_initial_state_zero():
    """Initial spawn layout (collar staged at 11.2 mm, clamp open 48 deg, holder upright 88 deg) scores 0.0."""
    s0 = make_states(slide=t.COLLAR_START_M, clamp_deg=t.CLAMP_OPEN_DEG, hinge_deg=t.HINGE_START_DEG)
    assert t.score(s0, tcps=PARK) == 0.0
    assert not t.success(s0)
    assert t.distance(s0) > 0.06


def test_ideal_seated_snugged_latched_and_clear_one():
    """Collar seated <= 3 mm, clamp snugged <= 15 deg, filter flat <= 6 deg in beam, grippers clear scores 1.0."""
    assert eval_score(slide=0.0005, clamp_deg=2.0, hinge_deg=1.0, tcps=PARK) == 1.0


def test_proximity_hover_on_stabilizer_or_side_tab_neg_cap():
    """Either gripper within 120 mm of rear stabilizer boss, clamp lever, or side flip block caps score at 0.75."""
    s_ok = make_states(slide=0.0005, clamp_deg=2.0, hinge_deg=1.0)
    on_boss = [t.kit.point(s_ok, "post_assy", (t.STAB_X_OFF, t.STAB_Y_OFF, 0.046)), PARK[1]]
    on_tab = [PARK[0], t.kit.point(s_ok, "filter_holder", (-0.034, -0.058, 0.006))]
    on_clamp = [PARK[0], t.kit.point(s_ok, "clamp_lever", (0.048, 0.0, 0.0))]
    assert t.score(s_ok, tcps=on_boss) == 0.75
    assert t.score(s_ok, tcps=on_tab) == 0.75
    assert t.score(s_ok, tcps=on_clamp) == 0.75
    assert not t.hands_clear(s_ok, tcps=on_boss)


def test_one_hand_unbraced_post_shift_or_tip_ablation_zero():
    """1-hand actuation that shifts the unbraced 310 g post past 16 mm or tilts it past 5 deg scores 0.0."""
    assert eval_score(slide=0.0, clamp_deg=2.0, hinge_deg=1.0, post_dp=(0.003, 0.025, 0.0)) == 0.0
    assert eval_score(slide=0.0, clamp_deg=2.0, hinge_deg=1.0, post_tilt_deg=8.0) == 0.0


def test_unlatched_springback_and_unseated_collar_tiers():
    """Releasing holder before detent springs it upright (0.50); swinging holder without seating collar gives 0.25."""
    assert eval_score(slide=0.0, clamp_deg=2.0, hinge_deg=88.0) == 0.50
    assert eval_score(slide=t.COLLAR_START_M, clamp_deg=t.CLAMP_OPEN_DEG, hinge_deg=25.0) == 0.25


def test_wrong_open_clamp_high_collar_or_misaligned_beam_neg():
    """Flipping any single geometric sub-gate drops score strictly below 1.0."""
    assert eval_score(slide=0.008, clamp_deg=2.0, hinge_deg=1.0) == 0.25
    assert eval_score(slide=0.0, clamp_deg=28.0, hinge_deg=1.0) == 0.25
    assert eval_score(slide=0.0, clamp_deg=2.0, hinge_deg=14.0) == 0.50
    assert eval_score(slide=0.0, clamp_deg=2.0, hinge_deg=1.0, post_dp=(0.0, 0.0, 0.010)) == 0.0


def test_possible_scores_tiers_and_undo():
    """All documented Possible scores (0.0, 0.25, 0.50, 0.75, 1.0) are reachable from stateless components()."""
    tj = json.loads((HERE / "task.json").read_text())
    m = re.search(r"Possible scores\s+([0-9.,\s]+)", tj["scoring"])
    assert m is not None
    documented = {float(x) for x in re.findall(r"\d+\.\d+", m.group(1))}

    s_ok = make_states(slide=0.0, clamp_deg=2.0, hinge_deg=1.0)
    near = [t.kit.point(s_ok, "post_assy", (t.STAB_X_OFF, t.STAB_Y_OFF, 0.046)), PARK[1]]
    got = {
        eval_score(slide=t.COLLAR_START_M, clamp_deg=t.CLAMP_OPEN_DEG, hinge_deg=t.HINGE_START_DEG),
        eval_score(slide=t.COLLAR_START_M, clamp_deg=t.CLAMP_OPEN_DEG, hinge_deg=25.0),
        eval_score(slide=0.0, clamp_deg=2.0, hinge_deg=88.0),
        t.score(s_ok, tcps=near),
        t.score(s_ok, tcps=PARK),
    }
    assert got == documented == {0.0, 0.25, 0.50, 0.75, 1.0}
    # Undo: releasing detent so holder springs back to 88 deg drops stateless score from 1.0 to 0.50
    assert eval_score(slide=0.0, clamp_deg=2.0, hinge_deg=88.0, tcps=PARK) == 0.50


def test_shortcuts_score_zero():
    """Shortcuts (noop, knocking post off datum, lifting post off breadboard, partial swing > 45 deg) score 0.0."""
    assert eval_score(slide=t.COLLAR_START_M, clamp_deg=48.0, hinge_deg=60.0) == 0.0
    assert eval_score(slide=0.0, clamp_deg=0.0, hinge_deg=0.0, post_dp=(0.04, -0.05, 0.0)) == 0.0
    assert eval_score(slide=0.0, clamp_deg=0.0, hinge_deg=0.0, post_dp=(0.0, 0.0, -0.02)) == 0.0


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
    assert (t.POST_POS_TOL, t.POST_Z_TOL, t.POST_TILT_DEG, t.COLLAR_SEAT_TOL, t.CLAMP_SNUG_DEG,
            t.HOLDER_PARTIAL_DEG, t.HOLDER_FLAT_DEG, t.BEAM_ALIGN_TOL, t.CLEAR_DIST, t.SUCCESS_HOLD_S) == (
        0.016, 0.006, 5.0, 0.003, 15.0, 45.0, 6.0, 0.016, 0.120, 3.0
    )


# -- occlusion / line-of-sight proxy to scored post shoulder, collar, cam lever, and filter beam port ------------------
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
    """Top camera rays to seated collar, cam lever, calibration beam filter, and rear boss clear parked grippers."""
    d = t.demo()
    end_tcps = (np.array(d["left"][-1].pos), np.array(d["right"][-1].pos))
    targets = [
        np.array([t.POST_X, t.POST_Y, t.DECK_Z + t.SHOULDER_H + 0.020]),
        np.array([t.POST_X + 0.052, t.POST_Y + 0.008, t.DECK_Z + t.SHOULDER_H + 0.016]),
        np.array([t.BEAM_X, t.BEAM_Y, t.HINGE_Z]),
        np.array([t.POST_X + t.STAB_X_OFF, t.POST_Y + t.STAB_Y_OFF, t.DECK_Z + 0.050]),
    ]
    assert not _occluded(end_tcps, targets)
    blocker = CAM + 0.75 * (targets[2] - CAM)
    assert _occluded((blocker, end_tcps[1]), targets)
