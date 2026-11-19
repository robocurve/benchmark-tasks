"""Offline CPU unit tests for barbell-spring-collar-slide-and-seat (revision-1).

Covers: the tray start, every rung of the 0.0 / 0.25 / 0.50 / 1.0 ladder, the geometry-only retention
evidence the scorer reads, the single-hand bar-shift ablation, the no-squeeze push failure, plan purity
and IK reach, top-camera sight lines at the demo end, and text == scorer.
"""

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


def _q(axis: int, angle_rad: float) -> np.ndarray:
    v = [0.0, 0.0, 0.0, math.cos(angle_rad / 2.0)]
    v[axis] = math.sin(angle_rad / 2.0)
    return np.array(v)


def _qmul(p: np.ndarray, q: np.ndarray) -> np.ndarray:
    x1, y1, z1, w1 = p
    x2, y2, z2, w2 = q
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
                     w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def quat_bar() -> np.ndarray:
    """The bar, stand and tray are built along their local x and turned onto world +Y (BAR_YAW)."""
    return _q(2, t.BAR_YAW)


def make_barbell(dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, tilt_rad: float = 0.0) -> PartState:
    """``tilt_rad`` pitches the bar about its own y (what the protractor reads); a roll about its own axis
    is invisible to a protractor and therefore not a failure."""
    pos = np.array([t.BAR_X + dx, t.BAR_Y + dy, t.BAR_Z + dz], float)
    q = _qmul(quat_bar(), _q(1, tilt_rad)) if tilt_rad else quat_bar()
    return PartState(pos=pos, quat_xyzw=q, lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                     low=pos - 0.10, high=pos + 0.10, joints={})


def make_collar(x_rel: float = None, dy: float = 0.0, dz: float = 0.0, jaw_m: float = 0.0009,
                tilt_rad: float = 0.0, roll_rad: float = 0.0) -> PartState:
    """``tilt_rad`` cocks the collar about its own y; ``roll_rad`` spins it about its own axis (invisible)."""
    xr = t.SEAT_REL_X if x_rel is None else x_rel
    pos = np.array([t.BAR_X, t.BAR_Y + xr + dy, t.BAR_Z + dz], float)
    q = quat_bar()
    if tilt_rad:
        q = _qmul(q, _q(1, tilt_rad))
    if roll_rad:
        q = _qmul(q, _q(0, roll_rad))
    return PartState(pos=pos, quat_xyzw=q, lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                     low=pos - 0.05, high=pos + 0.05, joints={"jaw_l": jaw_m, "jaw_r": -jaw_m})


def make_tray_collar() -> PartState:
    """The collar as the trial spawns it: upright on its coil in the holster, off the sleeve."""
    pos = np.array([t.TRAY_X, t.TRAY_Y, t.TOP + t.COLLAR_REST_Z + 0.001], float)
    return PartState(pos=pos, quat_xyzw=quat_bar(), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                     low=pos - 0.05, high=pos + 0.05, joints={"jaw_l": 0.0, "jaw_r": 0.0})


def eval_score(collar, barbell, *, tcps=PARK) -> float:
    return t.score({"collar": collar, "barbell": barbell}, tcps=tcps)


# -- the start -------------------------------------------------------------------------------------------------------
def test_noop_initial_state_zero():
    """The collar starts upright in the bench holster, off the sleeve: doing nothing scores 0.0."""
    b0, c0 = make_barbell(), make_tray_collar()
    assert eval_score(c0, b0) == 0.0
    assert not t.success({"collar": c0, "barbell": b0})
    assert t.distance({"collar": c0, "barbell": b0}) > 0.06
    # the holster keeps the collar off the sleeve axis by much more than the coaxial tolerance
    _, rad_yz, _ = t.collar_rel(c0, b0)
    assert rad_yz > 10.0 * t.COAXIAL_TOL


def test_ideal_seated_flush_clamped_and_clear_one():
    """Collar flush against the plate face, coaxial, jaws on the sleeve, both grippers clear: 1.0."""
    b = make_barbell(dx=0.004)
    c_seat = make_collar(x_rel=t.SEAT_REL_X - 0.002, jaw_m=0.001)
    assert eval_score(c_seat, b) == 1.0
    # success() with no live env cannot see the grippers, so the hold predicate is exercised through score()


# -- single-gate failures (each must be able to fail) ----------------------------------------------------------------
def test_one_hand_unbraced_bar_shift_ablation_zero():
    """A one-hand push that shifts the unbraced dumbbell past 12 mm out of its cradle scores 0.0."""
    b_shifted = make_barbell(dx=0.0165)
    c_seat = make_collar(x_rel=t.SEAT_REL_X, jaw_m=0.001)
    assert eval_score(c_seat, b_shifted) == 0.0


def test_unsqueezed_push_never_threads_and_scores_zero():
    """Pushed without squeezing, the closed collar is stopped at the sleeve end: 0.0 (never reaches 30 mm)."""
    b = make_barbell()
    c_blocked = make_collar(x_rel=t.THREAD_REL_X + 0.015, jaw_m=0.0005)
    assert eval_score(c_blocked, b) == 0.0


def test_off_axis_or_tilted_collar_zero():
    """Dropped off-axis or cocked on the sleeve: 0.0 behind the coaxial / tilt gates."""
    b_ok = make_barbell()
    assert eval_score(make_collar(dz=0.012, jaw_m=0.001), b_ok) == 0.0
    assert eval_score(make_collar(tilt_rad=math.radians(20.0), jaw_m=0.001), b_ok) == 0.0
    assert eval_score(make_collar(roll_rad=math.radians(20.0), jaw_m=0.001), b_ok) == 1.0  # roll is invisible


def test_tilted_or_dropped_barbell_zero():
    """Barbell off its cradle (moved, dropped or tilted past 6 deg) scores 0.0."""
    assert eval_score(make_collar(), make_barbell(tilt_rad=math.radians(8.0))) == 0.0
    assert eval_score(make_collar(), make_barbell(dy=-0.05, dz=-0.03)) == 0.0
    assert eval_score(make_collar(), make_barbell(dz=0.012)) == 0.0


# -- the ladder ------------------------------------------------------------------------------------------------------
def test_possible_scores_tiers_match_task_json():
    """Every documented Possible score is reachable from stateless components() and equals task.json's list."""
    tj = json.loads((HERE / "task.json").read_text())
    m = re.search(r"Possible scores\s+([0-9.,\s]+)", tj["scoring"])
    assert m is not None
    documented = {float(x) for x in re.findall(r"\d+\.\d+", m.group(1))}

    b = make_barbell()
    near_collar = [np.array([t.BAR_X + t.SEAT_REL_X, t.BAR_Y, t.BAR_Z + t.HANDLE_Z_REL]), PARK[1]]
    got = {
        eval_score(make_tray_collar(), b),                                          # 0.0: never picked up
        eval_score(make_collar(x_rel=t.SEAT_REL_X - t.TUG_M - 0.005), b),          # 0.25: slid, not at the plate
        eval_score(make_collar(x_rel=t.SEAT_REL_X), b, tcps=near_collar),           # 0.50: flush, gripper not clear
        eval_score(make_collar(x_rel=t.SEAT_REL_X), b),                             # 1.0: flush and clear
    }
    assert got == documented == {0.0, 0.25, 0.50, 1.0}
    # undo: sliding the collar 15 mm back off the plate drops a clear 1.0 straight to 0.25
    assert eval_score(make_collar(x_rel=t.SEAT_REL_X - 0.015), b) == 0.25


def test_gap_rungs_are_monotone_near_plate_then_flush():
    """8 mm near-plate and 3 mm flush are separate rungs: 8.5 mm -> 0.25, 5 mm -> 0.50, 2 mm -> 1.0."""
    b = make_barbell()
    assert eval_score(make_collar(x_rel=t.SEAT_REL_X - 0.0095), b) == 0.25
    assert eval_score(make_collar(x_rel=t.SEAT_REL_X - 0.0050), b) == 0.50
    assert eval_score(make_collar(x_rel=t.SEAT_REL_X - 0.0020), b) == 1.0


def test_shortcuts_score_zero():
    """Shortcuts (noop, collar left on the tray, barbell knocked off the stand) all score 0.0."""
    b0 = make_barbell()
    assert eval_score(make_tray_collar(), b0) == 0.0
    assert eval_score(make_tray_collar(), make_barbell(dy=-0.05, dz=-0.03)) == 0.0
    assert eval_score(make_collar(x_rel=t.THREAD_REL_X + 0.010, dy=0.08, dz=-0.04), b0) == 0.0


# -- retention evidence and the mechanism behind it -------------------------------------------------------------------
def test_retention_evidence_is_geometry_only_and_complete():
    """The scored retention pair (separation and coaxial grip) is read from poses alone, no joint or step state."""
    b = make_barbell()
    c = make_collar(x_rel=t.SEAT_REL_X + 0.002)
    comp = t.components({"collar": c, "barbell": b}, tcps=PARK)
    assert comp["flush"] and comp["collar_coaxial"] and comp["hands_clear"]
    # the same geometry with a 20 mm gap (what the tug would leave on an unclamped collar) fails flush
    c_slid = make_collar(x_rel=t.SEAT_REL_X - t.TUG_M)
    assert not t.components({"collar": c_slid, "barbell": b}, tcps=PARK)["flush"]


def test_power_three_cases_short_of_contact_no_grip_seated_and_gripping():
    """Power of the flush gate, one variable at a time (the bench reading a person makes):
    (a) collar stopped 5 mm short of contact -> not 1.0; (b) collar at the plate but the jaws not gripping, so the
    retention tug walks it back 20 mm -> not 1.0; (c) seated and gripping -> 1.0. Same collar, same bar, same
    grippers: only the measured end pose differs, so the gate is load-bearing rather than a threshold on faith."""
    b = make_barbell()
    # (a) 5 mm of daylight at the foremost surface: the collar is at the plate but not pushed home
    c_short = make_collar(x_rel=t.SEAT_REL_X - 0.005, jaw_m=0.001)
    assert eval_score(c_short, b) == 0.50, "5 mm short of contact must not score 1.0"
    assert not t.components({"collar": c_short, "barbell": b}, tcps=PARK)["flush"]
    # (b) jaws closed on nothing: the same start pose, after the tug has walked it back 20 mm
    c_slipped = make_collar(x_rel=t.SEAT_REL_X - t.TUG_M, jaw_m=0.001)
    assert eval_score(c_slipped, b) == 0.25, "a collar the tug pulls off the plate must not score 1.0"
    # (c) seated and gripping
    c_home = make_collar(x_rel=t.SEAT_REL_X - 0.002, jaw_m=0.001)
    assert eval_score(c_home, b) == 1.0
    # the three answers differ, which is what makes the gate evidence rather than a restatement
    assert len({eval_score(c_short, b), eval_score(c_slipped, b), eval_score(c_home, b)}) == 3


def test_flush_is_measured_from_the_foremost_surface_not_the_coil():
    """The gap is read from the collar's foremost plate-side surface. Measured: the collision hull is 33.0 mm
    along the sleeve, so its plate-side extreme is +16.5 mm; the coil's own face is +13.5 mm, 3.0 mm behind the
    two plastic squeeze handles. Measuring from the coil would report a 3.0 mm offset, i.e. a different number for
    the same pose -- so the constant and the hull must agree, and SEAT_REL_X must place the hull face on the plate."""
    half = t.COLLAR.size[0] / 2.0
    assert abs(half - t.COLLAR_HALF_X) < 1e-6, (half, t.COLLAR_HALF_X)
    assert abs(t.SEAT_REL_X - (t.PLATE_FACE_X - t.COLLAR_HALF_X)) < 1e-9
    # handle front face vs coil front face, straight from the jaw stirrup's declared boxes
    jaw = [j for j in t.COLLAR.joints if j.name == "jaw_l"][0]
    handle = [(at, size) for at, size, _c, _q in jaw.boxes if abs(size[0] - 0.030) < 1e-9][0]
    handle_front = float(handle[0][0]) + float(handle[1][0]) / 2.0
    coil_front = max(float(at[0]) + float(size[0]) / 2.0 for at, size, _c, _q in t.COLLAR.boxes)
    assert abs(handle_front - t.COLLAR_HALF_X) < 1e-6, (handle_front, t.COLLAR_HALF_X)
    assert abs((handle_front - coil_front) - 0.003) < 1e-6, (handle_front, coil_front)


def test_jaw_spring_geometry_is_physical():
    """The wire jaws are preloaded inside the sleeve at rest and clear it at full stroke."""
    assert t.JAW_REST_IN < t.R_SLEEVE, (t.JAW_REST_IN, t.R_SLEEVE)
    assert t.R_SLEEVE - t.JAW_REST_IN >= 0.002, "preload must be a real interference fit"
    assert t.JAW_REST_IN + t.JAW_OPEN_MAX_M > t.R_SLEEVE + 0.003, "full stroke must clear the sleeve"
    assert t.JAW_K > 0.0 and t.JAW_FRICTION >= 0.0


def test_no_code_held_scored_state():
    """No step hook writes scored state: the only on_step body is an observation print, and no joint target,
    drive stiffness or velocity target is ever written from a hook."""
    src = (HERE / "barbell_spring_collar_slide_and_seat.py").read_text()
    bodies = re.findall(r"@on_step\s*\ndef [\s\S]*?(?=\n@|\ndef |\Z)", src)
    assert len(bodies) <= 1, len(bodies)
    for body in bodies:
        # observation-only: it may read and print, but it must not write any of these
        for banned_write in ("set_joint_position_target", "write_joint_stiffness_to_sim",
                             "set_joint_velocity_target", "labkit._ENV[", "qpos[", "ctrl["):
            assert banned_write not in body, (banned_write, body[:200])
        assert "print(" in body or "return" in body
    for banned in ("set_joint_position_target", "write_joint_stiffness_to_sim", "set_joint_velocity_target"):
        assert banned not in src, banned
    # the score path is a pure function of the two object states
    assert set(t.components.__code__.co_varnames[:2]) == {"objects", "tcps"}


# -- plan ------------------------------------------------------------------------------------------------------------
def test_plan_kinematic_reach_and_purity():
    """Demo uses scene constants only, passes IK reach < 2 mm, and run_limit keeps >= 30% headroom."""
    plan = t.demo()
    err = pitch_demo.check_plan(plan, station.BASES)
    for arm, (pos_mm, rot_deg) in err.items():
        assert pos_mm < 2.0 and rot_deg < 2.0, (arm, pos_mm, rot_deg)
    dur = max(sum(w.duration for w in v) for v in plan.values())
    tj = json.loads((HERE / "task.json").read_text())
    steps = int(re.match(r"([\d,]+)\s*steps", tj["run_limit"]).group(1).replace(",", ""))
    assert steps >= math.ceil(dur * 20 * 1.3)
    assert not set(t.demo.__code__.co_names) & {"env", "scene", "_trial", "score", "labkit", "ik"}


def test_threshold_parity_and_text():
    """The scorer's literals, the README table and task.json scoring agree on every threshold and rung."""
    assert (t.CRADLE_TOL, t.BAR_Z_TOL, t.BAR_TILT_DEG, t.COAXIAL_TOL, t.COLLAR_TILT_DEG,
            t.PARTIAL_SLIDE_M, t.NEAR_TOL, t.FLUSH_TOL, t.CLEAR_DIST, t.SUCCESS_HOLD_S) == (
        0.012, 0.008, 6.0, 0.010, 12.0, 0.030, 0.008, 0.003, 0.120, 3.0)
    tj = json.loads((HERE / "task.json").read_text())
    for token in ("12 mm", "8 mm", "6 deg", "10 mm", "12 deg", "30 mm", "3 mm", "120 mm", "3 s"):
        assert token in tj["scoring"], token
    for rung in ("0.25", "0.50", "1.0"):
        assert rung in tj["scoring"], rung
    assert "0.75" not in tj["scoring"], "the retired 0.75 rung must not linger in the text"
    readme = (HERE / "README.md").read_text()
    for token in ("0.25", "0.50", "1.0", "tray", "retention", "dumbbell"):
        assert token in readme, token


# -- occlusion / line-of-sight proxy to the scored hardware ----------------------------------------------------------
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
    """Top camera rays to the seated collar, the plate face and the bar's free end clear the parked grippers."""
    d = t.demo()
    end_tcps = (np.array(d["left"][-1].pos), np.array(d["right"][-1].pos))
    targets = [
        np.array([t.BAR_X + t.SEAT_REL_X, t.BAR_Y, t.BAR_Z]),
        np.array([t.BAR_X + t.PLATE_FACE_X, t.BAR_Y, t.BAR_Z + 0.045]),
        np.array([t.BAR_X, t.BAR_Y + t.BAR_END_REL_X, t.BAR_Z]),
    ]
    assert not _occluded(end_tcps, targets)
    blocker = CAM + 0.75 * (targets[0] - CAM)
    assert _occluded((blocker, end_tcps[1]), targets)
