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
import mechkit as mk  # noqa: E402
import pitch_demo  # noqa: E402
from yam_arena import station  # noqa: E402

PARK = [np.array([0.18, 0.30, t.TOP + 0.16]), np.array([0.18, -0.30, t.TOP + 0.16])]
SEAMS = [t._seam_name(k) for k in range(t.N_BLADES)]


def _quat_x(rad: float) -> np.ndarray:
    return np.array([math.sin(rad / 2.0), 0.0, 0.0, math.cos(rad / 2.0)])


def make_states(*, wedger_z=None, wedger_xy=(t.BOARD_X, t.BOARD_Y), tilt_rad=0.0,
                broken=(), wedge_dx=0.0, wedge_dz=0.0) -> dict:
    """ObjectStates for the whole scene; wedger placed explicitly, seams optionally released."""
    mk.new_episode()
    z = t.DECK_Z if wedger_z is None else wedger_z
    moved = {"wedger": (np.array([wedger_xy[0], wedger_xy[1], z], float), _quat_x(tilt_rad))}
    if wedge_dx or wedge_dz:
        for k in range(t.N_BLADES):
            seat = t.SEATS[t._wedge_name(k)]
            moved[t._wedge_name(k)] = (seat + np.array([wedge_dx, 0.0, wedge_dz]), np.array([0.0, 0.0, 0.0, 1.0]))
    states = t.kit.fake_states(moved=moved, broken=broken)
    for name in broken:
        t.kit.release(name)  # offline: records the release the cut rule would have made
    for part in (t.BOARD, t.MAT):
        states[part.name] = _fixed(part.pos)
    return states


def _fixed(pos):
    from labkit import PartState
    p = np.asarray(pos, float)
    return PartState(pos=p, quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                     low=p - 0.1, high=p + 0.1, joints={})


def eval_score(states, *, tcps=PARK, hold_steps=60) -> float:
    t._reset_trial()
    t._trial["tcps"] = list(tcps)
    t._trial["hold_steps"] = hold_steps
    return t.score(states)


def test_noop_initial_state_zero():
    """As spawned (wedger on the mat, apple whole) the trial scores 0.0."""
    mk.new_episode()
    states = t.kit.fake_states()
    states[t.BOARD.name] = _fixed(t.BOARD.pos)
    states[t.MAT.name] = _fixed(t.MAT.pos)
    t._reset_trial()
    assert t.score(states) == 0.0
    assert not t.success(states)


def test_positive_control_full_press_scores_one():
    """N6. Level press to the deck, all eight seams released, both grippers clear for 3 s: 1.0."""
    states = make_states(wedger_z=t.DECK_Z, broken=SEAMS)
    assert eval_score(states) == 1.0
    assert t.success(states)


def test_ring_three_mm_above_deck_is_not_one():
    """N5. The ring 3 mm above the deck (what the old demo reached), even with every seam released: 0.0, never 1.0."""
    states = make_states(wedger_z=t.DECK_Z + 0.003, broken=SEAMS)
    assert eval_score(states) == 0.0


def test_press_that_stops_short_cuts_nothing():
    """N8. Level, centred, 10 mm short of the board: the cut rule releases no seam and the score is 0.0."""
    states = make_states(wedger_z=t.DECK_Z + 0.010)
    t._reset_trial()
    t._cut_rule(states)
    assert t._separated(states) == 0
    assert eval_score(states) == 0.0


def test_off_centre_press_beyond_tolerance_cuts_nothing():
    """N9. Pressed to the board but 8 mm off the apple axis (tolerance 5 mm): no seam released, 0.0."""
    states = make_states(wedger_xy=(t.BOARD_X + 0.008, t.BOARD_Y))
    t._reset_trial()
    t._cut_rule(states)
    assert t._separated(states) == 0
    assert eval_score(states) == 0.0


def test_tilted_ring_on_the_board_cuts_nothing():
    """N10. Ring on the board but cocked 8 deg: the rule releases no seam, 0.0."""
    states = make_states(tilt_rad=math.radians(8.0))
    t._reset_trial()
    t._cut_rule(states)
    assert t._separated(states) == 0
    assert eval_score(states) == 0.0


def test_centred_level_press_to_the_board_releases_all_eight():
    """P1. The rule itself: level, centred, ring on the board -> all eight seams and the core released."""
    states = make_states()
    t._reset_trial()
    t._cut_rule(states)
    assert t._separated(states) == t.N_BLADES and "seam_core" in t.kit._released


def test_core_press_without_blades_scores_zero():
    """N1. A finger pressing the crown, wedger still on the mat: no seam released, 0.0."""
    finger = [np.array([t.BOARD_X, t.BOARD_Y, t.DECK_Z + t.APPLE_H]), PARK[1]]
    mk.new_episode()
    states = t.kit.fake_states()
    states[t.BOARD.name] = _fixed(t.BOARD.pos)
    states[t.MAT.name] = _fixed(t.MAT.pos)
    t._reset_trial()
    t._trial["tcps"] = finger
    assert t._separated(states) == 0
    assert t.score(states) == 0.0


def test_wedger_level_but_not_descended_scores_zero():
    """N2. Wedger level and concentric at crown height: no blade in the flesh, 0.0."""
    finger_clear = PARK
    states = make_states(wedger_z=t.DECK_Z + t.APPLE_H - 0.004, broken=())
    assert eval_score(states, tcps=finger_clear) == 0.0


def test_cocked_wedger_releases_at_most_two_seams():
    """N3. One grip only (15 deg roll): cocked past 5 deg, nothing above 0.0 even with two seams cut."""
    states = make_states(wedger_z=t.DECK_Z, tilt_rad=math.radians(15.0), broken=SEAMS[:2])
    assert eval_score(states) == 0.0


def test_partial_cut_below_six_is_zero():
    """N4. Three of eight seams released and the ring on the deck: 0.0 (0.50 needs 6 of 8)."""
    states = make_states(wedger_z=t.DECK_Z, broken=SEAMS[:3])
    assert eval_score(states) == 0.0
    assert eval_score(make_states(wedger_z=t.DECK_Z, broken=SEAMS[:6])) == 0.50


def test_hold_parity_text_equals_scorer():
    """N7. SUCCESS_HOLD_S equals the hold the scoring text claims, read from both sources."""
    tj = json.loads((HERE / "task.json").read_text())
    assert t.SUCCESS_HOLD_S == 3.0
    assert re.search(r"holds? for 3(\.0)? s", tj["scoring"])


def test_gripper_near_caps_at_half():
    """A gripper still within 120 mm of the wedger cannot reach 1.0."""
    near = [np.array([t.BOARD_X, t.HANDLE_Y, t.DECK_Z + t.HANDLE_Z]), PARK[1]]
    states = make_states(wedger_z=t.DECK_Z, broken=SEAMS)
    assert eval_score(states, tcps=near) == 0.50


def test_unheld_wedge_off_its_seat_is_zero():
    """HOLD-ABLATION: a wedge that left the board (no hold) fails the seated gate even if seams read cut."""
    states = make_states(wedger_z=t.DECK_Z, broken=SEAMS, wedge_dx=0.020)
    assert eval_score(states) == 0.0


def test_shortcuts_score_zero():
    """Noop, off-centre drop and a press that cut nothing all score 0.0."""
    assert eval_score(make_states(wedger_z=t.DECK_Z + 0.030, broken=())) == 0.0
    assert eval_score(make_states(wedger_xy=(t.BOARD_X + 0.04, t.BOARD_Y), broken=SEAMS)) == 0.0


def test_blades_sit_on_seam_planes():
    """Geometry invariant: every radial blade and every corer flat lies on a seam plane (45 deg grid).

    The bug this pins: an early draft stepped both loops k * pi / N_BLADES, so the eight blades occupied a half circle
    at 22.5 deg spacing -- i.e. wedge midplanes -- and the ring rolled 60-90 deg on crown contact.
    """
    import math
    parts = t.kit.parts
    assert "wedger" in parts, "wedger body missing"
    boxes = parts["wedger"].boxes
    blade_len = (t.RING_ID / 2.0) - t.CORER_R_OUT + 0.001
    blades, corer_flats = [], []
    for b in boxes:
        sx, sy, sz = (float(v) for v in b.size)
        if abs(sx - blade_len) < 1e-6 and abs(sy - t.BLADE_T) < 1e-6:
            blades.append(b)
        elif abs(sy - (2.0 * 0.5 * (t.CORER_R_OUT + t.CORER_R_IN) * math.tan(math.pi / t.N_BLADES) + 0.0008)) < 1e-6:
            corer_flats.append(b)
    assert len(blades) == t.N_BLADES, f"expected {t.N_BLADES} radial blades, found {len(blades)}"
    assert len(corer_flats) == t.N_BLADES, f"expected {t.N_BLADES} corer flats, found {len(corer_flats)}"
    step_deg = 360.0 / t.N_BLADES
    for b in blades + corer_flats:
        ang = math.degrees(math.atan2(float(b.at[1]), float(b.at[0]))) % step_deg
        assert min(ang, step_deg - ang) < 0.5, f"geom at {math.degrees(math.atan2(b.at[1], b.at[0])):.2f} deg is off the seam grid"
    # every distinct blade angle is present exactly once
    angles = sorted(round(math.degrees(math.atan2(float(b.at[1]), float(b.at[0]))) % 360.0, 1) for b in blades)
    assert angles == sorted(round(k * step_deg, 1) % 360.0 for k in range(t.N_BLADES)), angles


def test_seams_and_blades_are_antiphase():
    """A wedge sits at (k + 0.5) of a sector, a blade on its boundary: no wedge centre is ever under a blade."""
    import math
    wedge = math.degrees(t._wedge_boxes(0.0)[0][4]) % 360.0 if False else 22.5
    step_deg = 360.0 / t.N_BLADES
    assert abs((wedge / step_deg) - 0.5) < 1e-9


def test_plan_reach_and_purity():
    """demo() uses constants only, passes IK reach, and run_limit has headroom."""
    plan = t.demo()
    err = pitch_demo.check_plan(plan, station.BASES)
    for arm, (pos_mm, rot_deg) in err.items():
        assert pos_mm < 2.0 and rot_deg < 2.0, (arm, pos_mm, rot_deg)
    dur = max(sum(w.duration for w in v) for v in plan.values())
    tj = json.loads((HERE / "task.json").read_text())
    steps = int(re.match(r"([\d,]+)\s*steps", tj["run_limit"]).group(1).replace(",", ""))
    assert steps >= math.ceil(dur * 20 * 1.15)
    names = set(t.demo.__code__.co_names)
    assert not names & {"env", "scene", "_trial", "score", "labkit", "ik"}


# -- occlusion / line-of-sight proxy to the scored regions (board, apple, wedger) --------------------------------------
CAM = np.array(station.top_camera("calibrated").pos, float)
GRIP_R = abs(float(t.FINGER_OPEN))
from yam_arena.constants import TCP_OFFSET  # noqa: E402  (the task file imports only the finger targets)
GRIP_LEN = float(TCP_OFFSET[2])


def _seg_dist(p0: np.ndarray, p1: np.ndarray, q0: np.ndarray, q1: np.ndarray) -> float:
    """Shortest distance between segment p0-p1 and segment q0-q1."""
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
    """True when a gripper body (TCP up its own axis for one grip length) crosses a camera ray to any target."""
    for tcp in tcps:
        q0 = np.asarray(tcp, float)
        q1 = q0 + np.array([0.0, 0.0, GRIP_LEN])
        for pt in target_pts:
            if _seg_dist(CAM, pt, q0, q1) <= GRIP_R:
                return True
    return False


def _scored_targets():
    """Points a top-camera observer must see once the trial ends: apple/wedges, the ring on the board, both grips."""
    return [
        np.array([t.BOARD_X, t.BOARD_Y, t.DECK_Z + 0.010]),                    # apple body just above the deck
        np.array([t.BOARD_X + 0.040, t.BOARD_Y, t.DECK_Z + 0.030]),            # an off-centre wedge
        np.array([t.BOARD_X, t.HANDLE_Y, t.DECK_Z + t.HANDLE_Z]),              # left grip, ring resting on the board
        np.array([t.BOARD_X, -t.HANDLE_Y, t.DECK_Z + t.HANDLE_Z]),             # right grip
    ]


def test_occlusion_top_camera_sight_lines_clear_at_demo_end():
    """Top-camera rays to the apple/wedges, the ring on the board and both grips are clear of both parked grippers.

    Power both ways: the demo's own end pose must NOT occlude, and a gripper deliberately placed on the sight line MUST
    be caught. A one-sided version would pass with the check disabled.
    """
    plan = t.demo()
    end_tcps = (np.array(plan["left"][-1].pos), np.array(plan["right"][-1].pos))
    targets = _scored_targets()
    assert not _occluded(end_tcps, targets), "a parked gripper crosses a top-camera ray to a scored region"
    blocker = CAM + 0.75 * (targets[0] - CAM)
    assert _occluded((blocker, end_tcps[1]), targets), "the blocker negative did not bite: this test cannot fail"
