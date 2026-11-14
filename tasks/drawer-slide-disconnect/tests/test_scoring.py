# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for drawer-slide-disconnect, pure Python (no simulator). Run from the task folder: pytest -q tests"""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import task as T  # noqa: E402

DT = 0.05
ON_RAILS_REAR = T.B - 0.129 + 0.28  # rear x with the drawer 129 mm out
OFF_REAR = T.B - 0.30 + 0.28  # rear x with the drawer 300 mm out (off the rails and clear of the bay)


def sample(**kw):
    m = {"lL": 0.0, "lR": 0.0, "rear_x": ON_RAILS_REAR, "bottom_z": [0.1] * 4, "d_speed": 0.0,
         "clear": {"left": 0.08, "right": 0.08}, "tcp_speed": {"left": 0.0, "right": 0.0}}
    m.update(kw)
    return m


def feed(samples, rec=None, t0=0.0):
    rec = rec or T.Record()
    t = t0
    for m in samples:
        t += DT
        rec.update(m, t)
    return rec


def secs(m, s):
    return [m] * int(round(s / DT))


def down(**kw):
    return sample(**{"rear_x": OFF_REAR, "bottom_z": [0.0, 0.001, -0.001, 0.0], **kw})


def full_run():
    pressed = sample(lL=0.006, lR=0.006, clear={"left": 0.0, "right": 0.0})
    held = sample(lL=0.006, lR=0.006, rear_x=OFF_REAR, bottom_z=[0.05] * 4, d_speed=0.1,
                  clear={"left": 0.0, "right": 0.0})
    held_down = down(lL=0.006, lR=0.006, clear={"left": 0.0, "right": 0.0})
    return secs(sample(), 1.0) + secs(pressed, 1.0) + secs(held, 2.0) + secs(held_down, 2.0) + secs(down(), 3.0)


def test_nothing_scores_zero():
    assert T.ladder(feed(secs(sample(), 5.0)))[0] == 0.0


def test_one_lever_alone_scores_zero():
    for lever in ("lL", "lR"):
        assert T.ladder(feed(secs(sample(**{lever: 0.006}), 5.0)))[0] == 0.0


def test_drawer_hanging_on_the_slides_scores_zero():
    # pulled to 280 mm and let go, but its rear still in the channel (the old "extension and clearance" shortcut)
    m = sample(rear_x=T.B - 0.26 + 0.28, bottom_z=[0.02, 0.02, 0.13, 0.13])
    assert T.ladder(feed(secs(m, 5.0)))[0] == 0.0


def test_drawer_rear_barely_off_the_rail_is_still_zero():
    m = down(rear_x=T.B - 0.005)
    assert T.ladder(feed(secs(m, 5.0)))[0] == 0.0


def test_off_the_slides_but_held_in_the_air_is_quarter():
    m = sample(rear_x=OFF_REAR, bottom_z=[0.06] * 4, clear={"left": 0.0, "right": 0.0})
    assert T.ladder(feed(secs(m, 5.0)))[0] == 0.25


def test_off_the_slides_but_tilted_on_the_table_is_quarter():
    m = down()
    m["bottom_z"] = [0.0, 0.0, 0.03, 0.03]
    assert T.ladder(feed(secs(m, 5.0)))[0] == 0.25


def test_down_but_still_moving_is_quarter():
    assert T.ladder(feed(secs(down(d_speed=0.05), 5.0)))[0] == 0.25


def test_full_run_is_one():
    score, why = T.ladder(feed(full_run()))
    assert score == 1.0, why


def test_gripper_still_on_the_drawer_is_half():
    rec = feed(secs(down(clear={"left": 0.0, "right": 0.0}), 4.0))
    assert T.ladder(rec)[0] == 0.5


def test_release_needs_one_second_clear_and_still():
    rec = feed(secs(down(clear={"left": 0.0, "right": 0.0}), 2.0) +
               secs(down(tcp_speed={"left": 0.0, "right": 0.05}), 3.0))
    assert T.ladder(rec)[0] == 0.5


def test_one_gripper_not_clear_is_half():
    rec = feed(secs(down(clear={"left": 0.08, "right": 0.01}), 4.0))
    assert T.ladder(rec)[0] == 0.5


def test_final_state_only_a_wrong_first_try_is_not_held_against_it():
    rec = feed(secs(sample(lL=0.006, rear_x=T.B - 0.2 + 0.28), 2.0) + full_run())
    assert T.ladder(rec)[0] == 1.0


def test_drawer_pushed_back_onto_the_slides_at_the_end_is_zero():
    rec = feed(full_run() + secs(sample(), 2.0))
    assert T.ladder(rec)[0] == 0.0


def test_reset_clears_history():
    rec = feed(full_run())
    rec.reset()
    assert rec.H.evers == {} and T.ladder(rec)[0] == 0.0
