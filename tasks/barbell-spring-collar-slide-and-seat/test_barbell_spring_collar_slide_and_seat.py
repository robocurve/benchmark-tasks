"""Offline CPU unit tests for barbell-spring-collar-slide-and-seat."""

import json
import math
import pathlib
import re
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import barbell_spring_collar_slide_and_seat as t  # noqa: E402
import pitch_demo  # noqa: E402
from labkit import PartState  # noqa: E402
from yam_arena import constants as _C  # noqa: E402
from yam_arena import station  # noqa: E402

PARK = [np.array([0.18, 0.30, t.TOP + 0.17]), np.array([0.18, -0.30, t.TOP + 0.17])]


def quat_y(angle_rad: float) -> np.ndarray:
    return np.array([0.0, math.sin(angle_rad / 2.0), 0.0, math.cos(angle_rad / 2.0)])


def make_barbell(dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, tilt_rad: float = 0.0) -> PartState:
    pos = np.array([t.BAR_X + dx, t.BAR_Y + dy, t.BAR_Z + dz], float)
    return PartState(pos=pos, quat_xyzw=quat_y(tilt_rad), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                     low=pos - 0.10, high=pos + 0.10, joints={})


def make_collar(x_rel: float = None, dy: float = 0.0, dz: float = 0.0,
                jaw_m: float = 0.0009, tilt_rad: float = 0.0) -> PartState:
    xr = t.SEAT_REL_X if x_rel is None else x_rel
    pos = np.array([t.BAR_X + xr, t.BAR_Y + dy, t.BAR_Z + dz], float)
    return PartState(pos=pos, quat_xyzw=quat_y(tilt_rad), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                     low=pos - 0.05, high=pos + 0.05, joints={"jaw_l": jaw_m, "jaw_r": -jaw_m})


def eval_score(collar, barbell, *, tcps=PARK) -> float:
    return t.score({"collar": collar, "barbell": barbell}, tcps=tcps)


def test_noop_initial_state_zero():
    """Initial spawn layout (collar clamped on outer pilot outside 58 mm shoulder) scores 0.0."""
    b0 = make_barbell()
    c0 = make_collar(x_rel=t.START_REL_X, jaw_m=t.JAW_INIT_M)
    assert eval_score(c0, b0) == 0.0
    assert not t.success({"collar": c0, "barbell": b0})
    assert t.distance({"collar": c0, "barbell": b0}) > 0.06


def test_ideal_seated_clamped_and_clear_one():
    """Collar seated flush past shoulder, jaws clamped <= 4 mm, and both grippers > 120 mm clear scores 1.0."""
    b = make_barbell(dx=0.005)
    c_seat = make_collar(x_rel=t.SEAT_REL_X + 0.005, jaw_m=0.001)
    assert eval_score(c_seat, b, tcps=PARK) == 1.0


def test_proximity_hover_on_collar_or_stabilizer_handle_neg_cap():
    """Either right gripper near collar OR left gripper still on radial stabilizer handle caps score at 0.75."""
    b = make_barbell()
    c_seat = make_collar(x_rel=t.SEAT_REL_X, jaw_m=0.001)
    near_collar = [PARK[0], np.array([t.BAR_X + t.SEAT_REL_X, t.BAR_Y, t.BAR_Z + t.HANDLE_Z_REL])]
    on_stabilizer = [np.array([t.BAR_X + t.GRIP_OFFSET_X, t.BAR_Y + t.GRIP_OFFSET_Y, t.BAR_Z + 0.002]), PARK[1]]
    assert eval_score(c_seat, b, tcps=near_collar) == 0.75
    assert eval_score(c_seat, b, tcps=on_stabilizer) == 0.75
    assert not t.hands_clear(c_seat, b, tcps=on_stabilizer)


def test_one_hand_unbraced_bar_shift_ablation_zero():
    """1-hand push that shifts the unbraced barbell past 12 mm out of its stand cradle scores 0.0."""
    b_shifted = make_barbell(dx=0.0165)
    c_seat = make_collar(x_rel=t.SEAT_REL_X + 0.0165, jaw_m=0.001)
    assert eval_score(c_seat, b_shifted) == 0.0


def test_unsqueezed_blocked_behind_shoulder_scores_zero():
    """Unsqueezed collar blocked behind the 58 mm sleeve shoulder (slid < 30 mm from start) scores 0.0."""
    b = make_barbell()
    c_blocked = make_collar(x_rel=t.SHOULDER_X - 0.014, jaw_m=0.0015)
    assert eval_score(c_blocked, b) == 0.0


def test_wrong_tilted_unclamped_or_off_axis_subgates_neg():
    """Flipping any single geometric or clamping sub-gate drops score strictly below 1.0."""
    b_ok = make_barbell()
    c_ok = make_collar(x_rel=t.SEAT_REL_X, jaw_m=0.001)
    assert eval_score(c_ok, make_barbell(tilt_rad=math.radians(8.0))) == 0.0
    assert eval_score(make_collar(dy=0.014, jaw_m=0.001), b_ok) == 0.0
    assert eval_score(make_collar(x_rel=t.SEAT_REL_X - 0.015, jaw_m=0.001), b_ok) == 0.25
    assert eval_score(make_collar(x_rel=t.SEAT_REL_X, jaw_m=0.007), b_ok) == 0.50


def test_possible_scores_tiers_and_undo():
    """All documented Possible scores (0.0, 0.25, 0.50, 0.75, 1.0) are reachable from stateless components()."""
    tj = json.loads((HERE / "task.json").read_text())
    m = re.search(r"Possible scores\s+([0-9.,\s]+)", tj["scoring"])
    assert m is not None
    documented = {float(x) for x in re.findall(r"\d+\.\d+", m.group(1))}

    b = make_barbell()
    near = [np.array([t.BAR_X + t.GRIP_OFFSET_X, t.BAR_Y + t.GRIP_OFFSET_Y, t.BAR_Z]), PARK[1]]
    got = {
        eval_score(make_collar(x_rel=t.START_REL_X, jaw_m=t.JAW_INIT_M), b),
        eval_score(make_collar(x_rel=t.START_REL_X + 0.036, jaw_m=0.001), b),
        eval_score(make_collar(x_rel=t.SEAT_REL_X, jaw_m=0.007), b),
        eval_score(make_collar(x_rel=t.SEAT_REL_X, jaw_m=0.001), b, tcps=near),
        eval_score(make_collar(x_rel=t.SEAT_REL_X, jaw_m=0.001), b, tcps=PARK),
    }
    assert got == documented == {0.0, 0.25, 0.50, 0.75, 1.0}
    # Undo: pulling collar 15 mm back from plate immediately drops stateless score from 1.0 to 0.25
    assert eval_score(make_collar(x_rel=t.SEAT_REL_X - 0.015, jaw_m=0.001), b, tcps=PARK) == 0.25


def test_shortcuts_score_zero():
    """Shortcuts (noop, sweep collar off sleeve, lift/drop barbell off stand, shallow slide < 30 mm) score 0.0."""
    b0 = make_barbell()
    assert eval_score(make_collar(x_rel=t.START_REL_X, dy=0.08, dz=-0.04), b0) == 0.0
    assert eval_score(make_collar(x_rel=t.SEAT_REL_X), make_barbell(dy=-0.05, dz=-0.03)) == 0.0
    assert eval_score(make_collar(x_rel=t.START_REL_X + 0.015, jaw_m=0.010), b0) == 0.0


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
    assert (t.CRADLE_TOL, t.BAR_Z_TOL, t.BAR_TILT_DEG, t.COAXIAL_TOL, t.COLLAR_TILT_DEG,
            t.CLAMP_SHUT_M, t.PARTIAL_SLIDE_M, t.PLATE_GAP_TOL, t.CLEAR_DIST, t.SUCCESS_HOLD_S) == (
        0.012, 0.008, 6.0, 0.010, 12.0, 0.004, 0.030, 0.008, 0.120, 3.0
    )


# -- occlusion / line-of-sight proxy to scored barbell sleeve, shoulder, plate hub, and spring collar -----------------
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
    """Top camera rays to seated collar, sleeve shoulder, weight plate, and stabilizer handle clear parked grippers."""
    d = t.demo()
    end_tcps = (np.array(d["left"][-1].pos), np.array(d["right"][-1].pos))
    targets = [
        np.array([t.BAR_X + t.SEAT_REL_X, t.BAR_Y, t.BAR_Z + t.HANDLE_Z_REL]),
        np.array([t.BAR_X + t.SHOULDER_X, t.BAR_Y, t.BAR_Z + 0.025]),
        np.array([t.BAR_X + t.PLATE_FACE_X, t.BAR_Y, t.BAR_Z + 0.045]),
        np.array([t.BAR_X + t.GRIP_OFFSET_X, t.BAR_Y + t.GRIP_OFFSET_Y, t.BAR_Z]),
    ]
    assert not _occluded(end_tcps, targets)
    blocker = CAM + 0.75 * (targets[0] - CAM)
    assert _occluded((blocker, end_tcps[1]), targets)
