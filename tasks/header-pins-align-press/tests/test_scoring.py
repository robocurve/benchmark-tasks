# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for header-pins-align-press, pure Python (no simulator). Run from the task folder: pytest -q tests

The scorer is a whole-run Record (a linkkit History fed every control step) plus ladder(); the tests feed it
synthetic samples. They need the simulator's Python environment on the path (yam_arena is imported by task.py).
"""

import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import task as T  # noqa: E402

DT = 0.05  # 20 Hz control


def sample(**kw):
    """The finished state: seated, level, still, both hands clear."""
    m = {"z": 0.0, "roll": 0.0, "pitch": 0.0, "speed": 0.0, "spin": 0.0,
         "clear": {"left": 0.08, "right": 0.06}, "tcp_speed": {"left": 0.0, "right": 0.0}}
    m.update(kw)
    return m


def run(samples):
    rec = T.Record()
    rec.reset()
    for m in samples:
        rec.H._tick(rec.n * DT)
        rec.update(m)
    return rec


def on(**kw):
    return sample(clear={"left": 0.08, "right": 0.001}, **kw)


def demo_trace(secs_end=2.0):
    out = [sample(z=0.006)] * 5 + [on(z=0.006)] * 5
    out += [on(z=z) for z in (0.0055, 0.004, 0.0025, 0.0012, 0.0004, 0.0)]
    out += [sample()] * int(secs_end / DT)
    return out


def test_demo_shaped_trace_scores_one():
    assert T.ladder(run(demo_trace()))[0] == 1.0


def test_nothing_scores_zero():
    assert T.ladder(run([sample(z=0.006)] * 40))[0] == 0.0


def test_release_above_the_pins_scores_zero():
    # the old loophole: lift the board clear of the header, let go. It falls back onto the tips and is never pressed.
    rec = run([on(z=0.006)] * 5 + [on(z=0.03)] * 5 + [sample(z=0.0062)] * 60)
    assert T.ladder(rec)[0] == 0.0


@pytest.mark.parametrize("roll,gate", [(1.5, False), (1.6, True), (-1.6, True)])
def test_lengthwise_tilt_limit_at_engagement(roll, gate):
    rec = run([on(z=0.006)] * 3 + [sample(z=0.0044, roll=roll)] + [sample()] * 60)
    assert ("bent" in rec.H.evers) == gate
    assert (T.ladder(rec)[0] == 0.0) == gate


@pytest.mark.parametrize("pitch,gate", [(4.0, False), (4.1, True)])
def test_across_tilt_limit_at_engagement(pitch, gate):
    rec = run([sample(z=0.0035, pitch=pitch)] + [sample()] * 60)
    assert ("bent" in rec.H.evers) == gate


def test_tilt_while_the_pins_are_only_on_the_tips_is_no_gate():
    rec = run([sample(z=0.0046, roll=7.0, pitch=3.0), sample(z=0.006, roll=0.5)] + demo_trace()[10:])
    assert "bent" not in rec.H.evers


def test_bend_beats_a_clean_seated_end():
    assert T.ladder(run([sample(z=0.003, roll=5.0)] + demo_trace()))[0] == 0.0


def test_seated_height_limit():
    ok = run(demo_trace()[:-40] + [sample(z=0.0003)] * 40)
    no = run(demo_trace()[:-40] + [sample(z=0.00031)] * 40)
    assert T.ladder(ok)[0] == 1.0
    assert T.ladder(no)[0] == 0.5  # pressed within 2 mm, not seated


def test_pressed_but_pulled_back_out_is_half():
    assert T.ladder(run([on(z=0.0015), sample(z=0.006)]))[0] == 0.5


def test_seated_but_hand_still_on_it_is_075():
    rec = run(demo_trace()[:-40] + [on()] * 40)
    assert T.ladder(rec)[0] == 0.75


def test_release_needs_30mm_and_one_second():
    base = demo_trace()[:-40]
    near = run(base + [sample(clear={"left": 0.08, "right": 0.029})] * 40)
    assert T.ladder(near)[0] == 0.75
    short = run(base + [sample()] * 19)  # 0.95 s
    assert T.ladder(short)[0] == 0.75
    assert T.ladder(run(base + [sample()] * 21))[0] == 1.0


def test_a_moving_hand_or_board_blocks_the_release():
    base = demo_trace()[:-40]
    fast = run(base + [sample(tcp_speed={"left": 0.0, "right": 0.006})] * 40)
    assert T.ladder(fast)[0] == 0.75
    jig = run(base + [sample(speed=0.004)] * 40)
    assert T.ladder(jig)[0] == 0.75
    spin = run(base + [sample(spin=3.5)] * 40)
    assert T.ladder(spin)[0] == 0.75


def test_a_late_move_resets_the_timers():
    base = demo_trace()[:-40]
    rec = run(base + [sample()] * 30 + [sample(speed=0.01)] + [sample()] * 10)
    assert T.ladder(rec)[0] == 0.75


def test_record_is_clean_after_reset():
    rec = run([sample(z=0.003, roll=5.0)])
    assert "bent" in rec.H.evers
    rec.reset()
    assert rec.H.evers == {} and T.ladder(rec)[0] == 0.0
