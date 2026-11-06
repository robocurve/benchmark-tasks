# SPDX-License-Identifier: MIT
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bevel_angle_transfer.contact import DEFAULT_CONFIG as CFG
from bevel_angle_transfer.contact import blade_released, config_dict, pose, rotation_xyzw, source_manifest
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


def observe(*, stock=CAPTURE, angle=ANGLE, stock_angle=0., fit=False, clear=True, ink=None):
    stock_pose = yaw_pose(stock, stock_angle)
    pivot = np.asarray(stock)+rotation_xyzw(stock_pose[3:]) @ CFG.BLADE_PIVOT_LOCAL
    blade_pose = yaw_pose(pivot, angle+stock_angle)
    blade_r = rotation_xyzw(blade_pose[3:])
    marker = [.25, 0., .85]
    pair_point, pair_force = [None]*3, ZERO
    if ink is not None:
        marker = [*ink, CFG.BOARD_TOP+.052]
        pair_point, pair_force = [*ink, CFG.BOARD_TOP], [0., 0., .1]
    return dict(
        dt=CFG.PHYSICS_DT, stock_pose=stock_pose, blade_pose=blade_pose,
        marker_pose=yaw_pose(marker), board_pose=yaw_pose(CFG.BOARD_CENTER),
        reference_stock_pose=yaw_pose(CFG.REFERENCE_STOCK_CENTER),
        reference_blade_pose=yaw_pose(CFG.REFERENCE_BLADE_CENTER, ANGLE),
        reference_stock_force=[.1, 0., 0.] if fit else ZERO,
        reference_blade_force=(blade_r[:, 1]*.1).tolist() if fit else ZERO,
        reference_stock_point=[.22, *CFG.REFERENCE_STOCK_CENTER[1:]] if fit else [None]*3,
        reference_blade_point=(pivot+blade_r @ [.10, -.01, 0.]).tolist() if fit else [None]*3,
        pair_point=pair_point, pair_force=pair_force, guide_pair_force=ZERO,
        stock_board_force=[-.1, 0., 0.] if stock == SEAT else ZERO,
        right_finger_blade_forces=[ZERO, ZERO] if clear else [[0., .1, 0.], ZERO],
        right_tcp_position=[.2, -.18, .88] if clear else (pivot+blade_r @ [.11, 0., 0.]).tolist(),
    )


def fit():
    return [observe(angle=10.), *(observe(fit=True, clear=False) for _ in range(72))]


def release():
    return [observe() for _ in range(24)]


def transfer():
    return release()+[observe(stock=(.208, .2875, .790)),
            observe(stock=(*SEAT[:2], .790)), observe(stock=SEAT)]


def line(first=.2202, last=.3598):
    return [observe(stock=SEAT, ink=[float(x), -.03+(float(x)-.22)*SLOPE])
            for x in np.linspace(first, last, 180)]


def lift():
    return [observe(stock=SEAT) for _ in range(120)]


def ticks(samples):
    return [dict(sample, tick=i) for i, sample in enumerate(samples, 1)]


def grade(samples):
    return reconstruct(ticks(samples))["metrics"]


def test_capture_transfer_and_ink_are_separate_milestones():
    assert grade(fit())["capture_valid"]
    assert grade(fit())["score"] == 0.
    assert grade(fit()+transfer())["score"] == .25
    assert grade(fit()+transfer()+line(last=.31)+lift())["score"] == .5
    assert grade(fit()+transfer()+line())["score"] == .5
    result = grade(fit()+transfer()+line()+lift())
    assert result["score"] == 1.
    assert result["capture_count"] == 1
    assert result["guided_coverage"] >= .9


def test_trace_cannot_replace_measured_capture_or_transfer():
    no_fit = [observe() for _ in range(72)]
    for sample in no_fit:
        sample.update(capture_valid=True, transfer_valid=True, released=True, guide_used=True)
    result = reconstruct(ticks(no_fit+transfer()+line()+lift()))
    assert result["metrics"]["score"] == 0.
    assert result["metrics"]["geometry_score"] == 1.
    assert result["differences"]
    assert grade(fit()[:-1]+transfer()[1:]+line()+lift())["score"] == 0.
    wrong_side = [dict(observe(fit=True), reference_blade_point=[.3, .3, .76]) for _ in range(72)]
    assert grade(wrong_side+transfer()+line()+lift())["score"] == 0.
    wrong_force = [dict(observe(fit=True), reference_blade_force=[0., 0., .1]) for _ in range(72)]
    assert grade(wrong_force+transfer()+line()+lift())["score"] == 0.
    unchanged = [observe(fit=True, angle=10., stock_angle=-30.) for _ in range(72)]
    assert grade(unchanged+transfer()+line()+lift())["score"] == 0.
    assert grade(fit()+release()+[observe(stock=SEAT)]+line()+lift())["score"] == 0.
    late_release = [observe(stock=(.208, .2875, .790), clear=False), *transfer()[-2:]]
    assert grade(fit()+late_release+line()+lift())["score"] == 0.


def test_angle_recovery_after_slip_does_not_restore_capture():
    slip = observe(stock=SEAT, angle=ANGLE+3.)
    result = grade(fit()+transfer()+line()+[slip]+lift())
    assert result["geometry_score"] == 1.
    assert result["guided_coverage"] == 0.
    assert result["retention_invalidations"] == 1
    assert result["score"] == 0.
    assert grade(fit()+transfer()+[slip]+fit()+transfer()+line()+lift())["score"] == 1.


def test_previous_ink_cannot_be_reused_and_extra_visible_ink_is_counted():
    assert grade([observe(angle=10.)]+line()+fit()+transfer()+line(last=.25)+lift())["score"] == .25
    extra = [observe(stock=SEAT, ink=[float(x), .015]) for x in np.linspace(.23, .35, 80)]
    result = grade([observe(angle=10.)]+extra+fit()+transfer()+line()+lift())
    assert result["geometry_score"] == 0.
    assert result["guided_geometry_score"] == 1.
    assert result["score"] == .25


def test_invalid_pose_invalidates_transfer_and_clock_is_checked():
    invalid = dict(observe(stock=SEAT), blade_pose=[.2, .1, .76, 0., 0., 0., 0.])
    result = grade(fit()+transfer()+[invalid]+line()+lift())
    assert result["score"] == 0.
    assert result["retention_invalidations"] == 1
    with pytest.raises(ValueError, match="expected consecutive ticks"):
        reconstruct([dict(observe(), tick=2)])


def test_regrade_verifies_source_config_and_ignores_saved_flags(tmp_path):
    tracker = TransferTracker()
    records = []
    for sample in ticks(fit()+transfer()+line()+lift()):
        records.append(tracker.sample(sample))
        if sample["tick"] % 12 == 0:
            tracker.metrics(force=False)
    manifest = source_manifest()
    ink = dict(strokes=tracker.strokes, guided_strokes=tracker.guided_strokes,
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



def test_carry_requires_one_continuous_airborne_leg():
    released = release()
    lifted = observe(stock=(.208, .2875, .790))
    lifted_at_board = observe(stock=(*SEAT[:2], .790))
    seated = observe(stock=SEAT)
    assert grade(fit()+released+[seated, lifted_at_board, seated])["score"] == 0.
    assert grade(fit()+released+[lifted, lifted_at_board, seated])["score"] == .25

    # Two sub-threshold legs separated by a drop do not count as one carry.
    first = observe(stock=(.208, .23, .790))
    dropped = observe(stock=(.208, .23, .770))
    second = observe(stock=(.208, .17, .790))
    third = observe(stock=(.208, .115, .790))
    assert grade(fit()+released+[lifted, first, dropped, second, third, seated])["score"] == 0.


def test_board_seating_uses_physical_near_face_pair_force():
    no_contact = [dict(sample, stock_board_force=ZERO) for sample in transfer()+line()+lift()]
    assert grade(fit()+no_contact)["score"] == 0.
    wrong_direction = [dict(sample, stock_board_force=[0., 0., .1]) for sample in line()]
    assert grade(fit()+transfer()+wrong_direction+lift())["score"] == .25


def test_capture_requires_adjustment_from_an_observed_mismatched_reset():
    baseline, *already_at_angle = fit()
    assert grade(already_at_angle+transfer()+line()+lift())["score"] == 0.
    assert grade([observe(angle=ANGLE+14.)]+already_at_angle+transfer())["score"] == 0.
    positive = grade([baseline]+already_at_angle+transfer())
    assert positive["score"] == .25
    assert positive["initial_measured_angle_deg"] == pytest.approx(10.)
    assert positive["initial_reference_error_deg"] == pytest.approx(30.)
    assert positive["adjustment_deg"] == pytest.approx(30.)


def test_capture_waits_for_angle_and_stock_to_settle_inside_fit_band():
    initial = observe(angle=10.)
    moving_angle = [observe(fit=True, angle=ANGLE+1.2*(i%2)) for i in range(144)]
    moving_stock = [observe(fit=True, stock=(CAPTURE[0], CAPTURE[1]+.0007*(i%2), CAPTURE[2]))
                    for i in range(144)]
    stable = [observe(fit=True) for _ in range(72)]
    for moving in (moving_angle, moving_stock):
        assert not grade([initial]+moving)["capture_valid"]
        assert not grade([initial]+moving+stable[:-1])["capture_valid"]
        assert grade([initial]+moving+stable)["capture_valid"]


def test_clearance_includes_raised_tab_and_requires_quiet_hold():
    sample = observe(fit=True)
    blade = pose(sample["blade_pose"])
    # This TCP clears the thin blade body by 26 mm but the tab by only 2 mm.
    sample["right_tcp_position"] = (blade[0]+blade[1] @ [.115, .015, .029]).tolist()
    assert not blade_released(sample, blade)
    assert not grade(fit()+[sample]*24)["released"]

    quiet = release()
    recontact = dict(quiet[0], right_finger_blade_forces=[[0., .0569, 0.], ZERO])
    assert not grade(fit()+quiet[:-1]+[recontact]+quiet[:-1])["released"]
    assert grade(fit()+quiet[:-1]+[recontact]+quiet)["released"]
    assert grade(fit()+quiet+[recontact])["retention_invalidations"] == 1
