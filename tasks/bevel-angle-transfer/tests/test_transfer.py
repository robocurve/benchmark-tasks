# SPDX-License-Identifier: MIT
"""Observable transfer milestones and independent trace replay."""
import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bevel_angle_transfer.contact import DEFAULT_CONFIG as CFG
from bevel_angle_transfer.contact import blade_released, capture_contacts, config_dict, contact_gate
from bevel_angle_transfer.contact import gauge_ground_clearance, pose, rotation_xyzw, source_manifest
from bevel_angle_transfer.contact import stock_ground_clearance
from bevel_angle_transfer.regrade import reconstruct, regrade_trial
from bevel_angle_transfer.scoring import TransferTracker


def yaw_pose(origin, angle=0.):
    angle = math.radians(angle)
    return [*map(float, origin), 0., 0., math.sin(angle/2), math.cos(angle/2)]


CAPTURE = (.208, .2875, .7655)
ANGLE = CFG.REFERENCE_ANGLE_DEG
SLOPE = math.tan(math.radians(ANGLE))
SEAT = (.208, -.03-(.22-.208)*SLOPE+.0125/math.cos(math.radians(ANGLE))+.1075, .7655)
ZERO = [0., 0., 0.]


def observe(*, stock=CAPTURE, angle=ANGLE, stock_angle=0., clear=True, ink=None):
    stock_pose = yaw_pose(stock, stock_angle)
    stock_r = rotation_xyzw(stock_pose[3:])
    pivot = np.asarray(stock)+stock_r @ CFG.BLADE_PIVOT_LOCAL
    blade_pose = yaw_pose(pivot, angle+stock_angle)
    blade_r = rotation_xyzw(blade_pose[3:])
    marker = [.25, 0., .85]
    pair_point = [None]*3
    if ink is not None:
        marker = [*ink, CFG.BOARD_TOP+.052]
        pair_point = [*ink, CFG.BOARD_TOP]
    right = np.array([.2, -.18, .88]) if clear else pivot+blade_r @ [.115, .015, .029]
    left = [np.asarray(stock)+stock_r @ offset for offset in ((-.012, 0., .01), (.012, 0., .01))]
    return dict(
        trace_schema="bevel-visible-v2", dt=CFG.PHYSICS_DT, stock_pose=stock_pose, blade_pose=blade_pose,
        marker_pose=yaw_pose(marker), board_pose=yaw_pose(CFG.BOARD_CENTER),
        reference_stock_pose=yaw_pose(CFG.REFERENCE_STOCK_CENTER),
        reference_blade_pose=yaw_pose(CFG.REFERENCE_BLADE_CENTER, ANGLE),
        pair_point=pair_point, pair_force=ZERO,
        right_finger_poses=[yaw_pose(right)]*2, right_gripper_open=clear,
        right_gripper_opening=1. if clear else .2,
        left_finger_poses=[yaw_pose(finger, stock_angle) for finger in left],
        left_gripper_open=False, left_gripper_opening=.2,
    )


def pitch_gauge(sample, degrees):
    tilt = math.radians(degrees)
    sy, cy = math.sin(tilt/2), math.cos(tilt/2)
    sz, cz = math.sin(math.radians(ANGLE)/2), math.cos(math.radians(ANGLE)/2)
    origin = np.asarray(sample["stock_pose"][:3])
    sample["stock_pose"] = [*origin, 0., sy, 0., cy]
    pivot = origin + rotation_xyzw(sample["stock_pose"][3:]) @ CFG.BLADE_PIVOT_LOCAL
    sample["blade_pose"] = [*pivot, sy*sz, sy*cz, cy*sz, cy*cz]
    return sample


def fit():
    return [observe(angle=10., clear=False), observe(clear=False)]


def transfer():
    return [observe(), observe(stock=(*CAPTURE[:2], .790)),
            observe(stock=(*SEAT[:2], .790)), observe(stock=SEAT)]


def line(first=.222, last=.359, n=180):
    return [observe(stock=SEAT, ink=[float(x), -.03+(float(x)-.22)*SLOPE])
            for x in np.linspace(first, last, n)]


def lift():
    return [observe(stock=SEAT)]


def ticks(samples):
    return [dict(sample, tick=i) for i, sample in enumerate(samples, 1)]


def grade(samples):
    return reconstruct(ticks(samples))["metrics"]


def mating_faces(sample):
    return capture_contacts(sample, *(pose(sample[key]) for key in
                            ("stock_pose", "blade_pose", "reference_stock_pose", "reference_blade_pose")))


def test_both_reference_faces_fit_at_both_endpoints_without_force_or_hold():
    assert mating_faces(observe())
    assert mating_faces(observe(stock=(CAPTURE[0]-.0015, *CAPTURE[1:])))
    assert not mating_faces(observe(stock=(CAPTURE[0]-.004, *CAPTURE[1:])))
    assert not mating_faces(observe(stock=(*CAPTURE[:2], .782)))
    assert not mating_faces(observe(stock_angle=180.))
    assert not mating_faces(observe(angle=ANGLE+2.))
    assert not mating_faces(observe(stock=(CAPTURE[0], CAPTURE[1]-.006, CAPTURE[2])))
    assert grade([observe(clear=False)])["capture_valid"]


def test_release_placement_marking_and_final_clearance_have_distinct_scores():
    assert grade(fit())["score"] == 0.
    assert grade(fit()+transfer())["score"] == .25
    assert grade(fit()+transfer()+line(last=.294)+lift())["score"] == .5
    assert grade(fit()+transfer()+line())["score"] == .5
    result = grade(fit()+transfer()+line()+lift())
    assert result["score"] == 1.
    assert result["capture_count"] == 1
    assert result["marker_guidance_valid"]
    assert result["marked_coverage"] >= .126


def test_open_fingers_must_withdraw_before_leaving_the_fixture():
    near_tab = dict(observe(clear=False), right_gripper_open=True, right_gripper_opening=1.)
    assert not blade_released(near_tab, pose(near_tab["blade_pose"]))
    assert not grade(fit()+[near_tab])["released"]
    assert grade(fit()+[observe()])["released"]
    assert not grade([observe(), observe()])["released"]
    assert grade([observe()]+transfer())["score"] == 0.
    assert grade(fit()+[near_tab]+transfer()[1:]+line()+lift())["score"] == 0.
    late = dict(transfer()[1], right_gripper_open=False, right_gripper_opening=.2)
    assert grade(fit()+[late]+transfer()[2:])["score"] == 0.


def test_withdrawal_preserves_the_previous_fit_while_stock_remains_at_fixture():
    withdrawing = pitch_gauge(observe(stock=(*CAPTURE[:2], .7676)), -12.)
    stock, blade = pose(withdrawing["stock_pose"]), pose(withdrawing["blade_pose"])
    assert gauge_ground_clearance(stock, blade) <= .001
    assert not mating_faces(withdrawing)
    result = grade(fit()+[withdrawing])
    assert result["released"]
    assert result["captured_angle_deg"] == pytest.approx(ANGLE)
    assert grade(fit()+[withdrawing]+transfer()[1:])["score"] == .25
    assert not grade([observe(angle=10., clear=False), withdrawing])["released"]

    away = observe(stock=(*CAPTURE[:2], .800))
    assert not grade(fit()+[away, withdrawing])["released"]


def test_airborne_carry_rejects_dragging_dropping_and_stock_release():
    released, lifted, above_board, seated = transfer()
    between = (.208, .18, .7655)
    slid = observe(stock=(CAPTURE[0], CAPTURE[1]+.020, CAPTURE[2]))
    assert grade(fit()+[released, slid, lifted, above_board, seated])["score"] == 0.
    assert grade(fit()+[released, observe(stock=between), seated])["score"] == 0.
    assert grade(fit()+[released, lifted, seated])["score"] == 0.
    assert grade(fit()+[released, lifted, observe(stock=between), above_board, seated])["score"] == 0.
    assert grade(fit()+[released, dict(lifted, left_gripper_open=True, left_gripper_opening=1.), above_board, seated])["score"] == 0.
    unsupported = dict(lifted, left_finger_poses=[yaw_pose([.3, 0., 1.])]*2)
    assert grade(fit()+[released, unsupported, above_board, seated])["score"] == 0.
    assert grade(fit()+transfer())["score"] == .25
    diagonal = observe(stock=(SEAT[0]-.018, SEAT[1], .790))
    assert grade(fit()+[released, lifted, diagonal, seated])["score"] == .25
    above_table_seated = observe(stock=(*SEAT[:2], SEAT[2]+.002))
    assert grade(fit()+[released, lifted, diagonal, above_table_seated])["score"] == .25


def test_dragging_blade_invalidates_carry_even_while_stock_is_raised():
    released, lifted, above_board, seated = transfer()
    sample = pitch_gauge(observe(stock=(.208, .18, .793)), 20.)
    stock, blade = pose(sample["stock_pose"]), pose(sample["blade_pose"])
    assert stock_ground_clearance(stock) > .001
    assert gauge_ground_clearance(stock, blade) < 0.
    assert grade(fit()+[released, lifted, sample, above_board, seated])["score"] == 0.


def test_comparison_uses_only_fit_and_board_placement_angles():
    released, lifted, above_board, seated = transfer()
    moving = observe(stock=tuple(lifted["stock_pose"][:3]), angle=ANGLE+4.)
    assert grade(fit()+[released, moving, above_board, seated])["score"] == .25
    assert grade(fit()+[released, lifted, above_board, observe(stock=SEAT, angle=ANGLE+2.)])["score"] == .25
    assert grade(fit()+[released, lifted, above_board, observe(stock=SEAT, angle=ANGLE+2.01)])["score"] == 0.
    miss = (SEAT[0]-.004, *SEAT[1:])
    assert grade(fit()+[released, lifted, observe(stock=(*miss[:2], .790)), observe(stock=miss)])["score"] == 0.


def test_reference_fit_is_not_a_continuous_condition_after_release():
    released, lifted, above_board, seated = transfer()
    turning_in_place = observe(stock=CAPTURE, stock_angle=2.)
    # The fitted setting is already captured. Whole-tool orientation may change
    # before lifting; no horizontal dragging or blade readjustment has occurred.
    assert grade(fit()+[released, turning_in_place, lifted, above_board, seated])["score"] == .25


def test_right_arm_can_close_on_marker_away_from_blade_but_not_regrasp_blade():
    released, lifted, above_board, seated = transfer()
    assert grade(fit()+[released, dict(lifted, right_gripper_open=False, right_gripper_opening=.2), above_board, seated])["score"] == .25
    regrasp = observe(stock=tuple(lifted["stock_pose"][:3]), clear=False)
    assert grade(fit()+[released, regrasp, above_board, seated])["score"] == 0.
    assert grade(fit()+transfer()+line()+[observe(stock=SEAT, clear=False)])["score"] == 0.
    open_push = dict(observe(stock=SEAT, clear=False), right_gripper_open=True, right_gripper_opening=1.)
    assert grade(fit()+transfer()+line()+[open_push])["score"] == 0.
    assert grade(fit()+transfer()+line()+[dict(lift()[0], left_gripper_open=True, left_gripper_opening=1.)])["score"] == 1.


def test_missing_raw_hand_observation_cannot_authorize_transfer():
    released, lifted, above_board, seated = transfer()
    del lifted["left_finger_poses"]
    assert grade(fit()+[released, lifted, above_board, seated])["score"] == 0.
    del released["right_gripper_open"]
    assert not grade(fit()+[released])["released"]
    malformed = dict(above_board, left_finger_poses=[[.3, 0., .8, 0., 0., 0., 0.]]*2)
    assert grade(fit()+transfer()[:2]+[malformed, above_board, seated])["score"] == 0.
    for opening in (None, float("nan"), -1., 2., 1.):
        invalid = dict(above_board, left_gripper_opening=opening)
        assert grade(fit()+transfer()[:2]+[invalid, above_board, seated])["score"] == 0.


def test_detached_blade_invalidates_a_previous_valid_placement():
    detached = lift()[0]
    detached["blade_pose"][0] += .01
    assert grade(fit()+transfer()+line()+[detached])["score"] == 0.


def freehand(samples):
    return [observe(stock=(SEAT[0]-.008, *SEAT[1:]), ink=sample["pair_point"][:2])
            for sample in samples]


def test_pretransfer_ink_and_separate_freehand_strokes_lose_line_credit():
    assert grade(line()+fit()+transfer()+lift())["score"] == .25
    run = fit()+transfer()
    result = grade(run+freehand(line())+lift())
    assert result["geometry_score"] == 1.
    assert not result["marker_guidance_valid"]
    assert result["score"] == .25

    stroke = line()
    stamp = freehand(stroke[90:91])
    for samples in (run+stroke+lift()+stamp, run+stroke+stamp, stamp+run+stroke):
        result = grade(samples+lift())
        assert result["geometry_score"] == 1.
        assert not result["marker_guidance_valid"]
        assert result["score"] == .25


def test_guidance_resolution_distinguishes_a_boundary_sample_from_freehand_motion():
    stroke = line()
    run = fit()+transfer()
    one_sample = stroke[:90]+freehand(stroke[90:91])+stroke[91:]
    result = grade(run+one_sample+lift())
    assert result["marker_guidance_valid"]
    assert result["score"] == 1.

    short_excursion = stroke[:90]+freehand(stroke[90:92])*40+stroke[92:]
    result = grade(run+short_excursion+lift())
    assert result["marker_guidance_valid"]
    assert result["score"] == 1.

    visible_excursion = stroke[:90]+freehand(stroke[90:95])+stroke[95:]
    result = grade(run+visible_excursion+lift())
    assert result["geometry_score"] == 1.
    assert not result["marker_guidance_valid"]
    assert result["score"] == .25


def test_raw_contact_marks_the_nib_without_force_threshold_but_rejects_hover_and_barrel():
    sample = line()[0]
    args = [sample["pair_force"], sample["pair_point"], sample["marker_pose"], sample["board_pose"]]
    assert contact_gate(*args)[0]
    args[2] = yaw_pose([*sample["pair_point"][:2], CFG.BOARD_TOP+.059])
    assert not contact_gate(*args)[0]
    args[2] = yaw_pose([*sample["pair_point"][:2], CFG.BOARD_TOP-.02])
    assert not contact_gate(*args)[0]
    args[1] = [None]*3
    assert not contact_gate(*args)[0]


def test_trace_clock_and_measured_observations_cannot_be_replaced_by_saved_flags():
    no_fit = [dict(observe(angle=10.), capture_valid=True, transfer_valid=True,
                   released=True, guide_used=True)]
    result = reconstruct(ticks(no_fit+transfer()[1:]+line()+lift()))
    assert result["metrics"]["score"] == 0.
    assert result["metrics"]["geometry_score"] == 1.
    assert result["differences"]
    with pytest.raises(ValueError, match="expected consecutive ticks"):
        reconstruct([dict(observe(), tick=2)])


def test_regrade_verifies_source_config_and_ignores_saved_flags(tmp_path):
    tracker = TransferTracker()
    records = [tracker.sample(sample) for sample in ticks(fit()+transfer()+line()+lift())]
    manifest = source_manifest()
    ink = dict(schema="bevel-angle-transfer-contact-ink-v2", strokes=tracker.strokes,
               target_start=tracker.target_start, target_end=tracker.target_end,
               contact_config=config_dict(), environment_revision=manifest["revision"])
    for filename, data in (("metrics.json", tracker.metrics()), ("ink.json", ink),
                           ("source_manifest.json", manifest)):
        (tmp_path/filename).write_text(json.dumps(data), encoding="utf-8")
    trace = tmp_path/"contacts.jsonl"
    trace.write_text("\n".join(map(json.dumps, records)), encoding="utf-8")
    assert regrade_trial(tmp_path)["match"]
    records[0]["transfer_valid"] = True
    trace.write_text("\n".join(map(json.dumps, records)), encoding="utf-8")
    replay = regrade_trial(tmp_path)
    assert not replay["match"]
    assert replay["metrics"]["score"] == 1.


def test_metric_query_frequency_does_not_change_replay_report():
    live = TransferTracker()
    raw = list(ticks(fit()+transfer()+line(n=15)+lift()))
    live.metrics()  # The runtime exports a blank scene before taking actions.
    records = []
    for sample in raw:
        records.append(live.sample(sample))
        live.metrics()
    assert reconstruct(records)["metrics"] == live.metrics()
