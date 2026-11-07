"""Scoring rules on hand-built states. No simulator: python -m pytest tasks/conveyor-photo-build/tests -q"""
from dataclasses import dataclass, field, replace
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from layout import (BEAM_TOP, BELT_TOP, BELT_X, EXTRA, GATE_X, GATE_Y, MARK_TOP, MARKS, ORDER, PARTS,  # noqa: E402
                    POST_TOP, SPEED, STAGE, STAGE_TOP, TOP, TOTE, TOTE_FLOOR, start_y, z_extent)
from scoring import CONTROL_DT, Trial, gone_downstream  # noqa: E402


@dataclass(frozen=True)
class S:
    pos: np.ndarray
    quat_xyzw: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))
    lin_vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    ang_vel: np.ndarray = field(default_factory=lambda: np.zeros(3))


def quat(axis, deg):
    axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    h = math.radians(deg) / 2
    return np.array([*(axis * math.sin(h)), math.cos(h)])


def h(name):
    return PARTS[name][0][2]


def start():
    return {n: S(np.array([BELT_X, start_y(n), BELT_TOP + h(n) / 2]), lin_vel=np.array([0, -SPEED, 0]))
            for n in ORDER}


def in_tote(dy=0.0):
    return S(np.array([TOTE[0], TOTE[1] + dy, TOTE_FLOOR + 0.02]))


def built(**over):
    o = start()
    o['post_a'] = S(np.array([*MARKS[0], MARK_TOP + h('post_a') / 2]))
    o['post_b'] = S(np.array([*MARKS[1], MARK_TOP + h('post_b') / 2]))
    o['beam'] = S(np.array([GATE_X, GATE_Y, POST_TOP + h('beam') / 2]))
    o['cap'] = S(np.array([GATE_X, GATE_Y, BEAM_TOP + h('cap') / 2]))
    for i, n in enumerate(EXTRA):
        o[n] = in_tote(-0.06 + 0.04 * i)
    o.update(over)
    return o


def run(objects_seq):
    trial = Trial()
    ok = False
    for objects in objects_seq:
        trial.observe(objects)
        ok = trial.success(objects)
    return trial, trial.score(objects_seq[-1]), ok


def score_of(objects, steps=1):
    # A mid-trial state: not every part at its start, so the tracker is not cleared.
    return run([objects] * steps)[1:]


def moved(state, **kw):
    return replace(state, **kw)


def test_start_scores_zero():
    trial, score, ok = run([start()])
    assert score == 0 and not ok


def test_finished_gate_scores_one_and_succeeds():
    assert score_of(built()) == (1.0, True)


def test_parts_staged_on_the_mat_earn_nothing():
    o = built(cap=S(np.array([*STAGE, STAGE_TOP + 0.02])), beam=S(np.array([STAGE[0] + 0.05, STAGE[1], STAGE_TOP + 0.015])))
    assert score_of(o) == (pytest.approx(0.4), False)


@pytest.mark.parametrize('left, expected', [
    ((), 1.0), (('square_post',), 0.75), (('orange_beam',), 0.75), (('blue_cube',), 0.75), (('short_post',), 0.75),
    (('square_post', 'orange_beam'), 0.5), (EXTRA[:3], 0.25), (EXTRA, 0.0)])
def test_extra_parts_kept_cost_a_quarter_each(left, expected):
    o = built(**{n: S(np.array([STAGE[0], STAGE[1], STAGE_TOP + h(n) / 2])) for n in left})
    score, ok = score_of(o)
    assert score == pytest.approx(expected) and ok == (not left)


def test_penalty_never_below_zero():
    o = start()
    o['post_a'] = S(np.array([*MARKS[0], MARK_TOP + h('post_a') / 2]))
    assert score_of(o)[0] == 0.0  # 0.2 - 1.0


def test_posts_only():
    o = built(beam=in_tote(), cap=in_tote(0.02))
    assert score_of(o)[0] == pytest.approx(0.4)


def test_one_post_only():
    o = built(post_b=in_tote(), beam=in_tote(), cap=in_tote(0.02))
    assert score_of(o)[0] == pytest.approx(0.2)


@pytest.mark.parametrize('offset_mm, counts', [(9.0, True), (10.0, True), (11.0, False)])
def test_post_mark_tolerance(offset_mm, counts):
    p = built()['post_a']
    o = built(post_a=moved(p, pos=p.pos + [offset_mm / 1000, 0, 0]), beam=in_tote(), cap=in_tote(0.02))
    assert score_of(o)[0] == pytest.approx(0.4 if counts else 0.2)


@pytest.mark.parametrize('deg, counts', [(4.0, True), (6.0, False), (180.0, True), (90.0, False)])
def test_post_must_stand(deg, counts):
    p = built()['post_a']
    o = built(post_a=moved(p, quat_xyzw=quat((1, 0, 0), deg)), beam=in_tote(), cap=in_tote(0.02))
    assert score_of(o)[0] == pytest.approx(0.4 if counts else 0.2)


def test_post_raised_off_the_mark_does_not_count():
    p = built()['post_a']
    o = built(post_a=moved(p, pos=p.pos + [0, 0, 0.006]), beam=in_tote(), cap=in_tote(0.02))
    assert score_of(o)[0] == pytest.approx(0.2)


@pytest.mark.parametrize('name', ['short_post', 'square_post'])
def test_a_look_alike_post_on_a_mark_is_not_a_post(name):
    sp = S(np.array([*MARKS[1], MARK_TOP + h(name) / 2]))
    o = built(post_b=in_tote(), **{name: sp}, beam=in_tote(), cap=in_tote(0.02))
    assert score_of(o)[0] == 0.0  # one tall cylinder 0.2, less 0.25 for the look-alike kept back
    o = built(**{name: sp})  # the tall cylinder's mark is taken; here it stands on the other mark's spot too
    assert score_of(o) == (pytest.approx(0.75), False)


def test_both_posts_on_one_mark_count_once():
    a = built()['post_a']
    b = S(a.pos + [0.005, 0, 0])
    o = built(post_b=b, beam=in_tote(), cap=in_tote(0.02))
    assert score_of(o)[0] == pytest.approx(0.2)


@pytest.mark.parametrize('deg, counts', [(8.0, True), (10.0, True), (12.0, False), (180.0, True), (90.0, False)])
def test_beam_along_the_marks(deg, counts):
    b = built()['beam']
    o = built(beam=moved(b, quat_xyzw=quat((0, 0, 1), deg)), cap=in_tote(0.02))
    assert score_of(o)[0] == pytest.approx(0.7 if counts else 0.4)


@pytest.mark.parametrize('dy_mm, counts', [(14.0, True), (15.0, True), (20.0, False)])
def test_beam_centred_on_the_posts(dy_mm, counts):
    b = built()['beam']
    o = built(beam=moved(b, pos=b.pos + [0, dy_mm / 1000, 0]), cap=in_tote(0.02))
    assert score_of(o)[0] == pytest.approx(0.7 if counts else 0.4)


def test_beam_on_its_side_does_not_count():
    b = built()['beam']
    o = built(beam=moved(b, quat_xyzw=quat((0, 1, 0), 90), pos=b.pos + [0, 0, 0.005]), cap=in_tote(0.02))
    assert score_of(o)[0] == pytest.approx(0.4)


def test_beam_needs_both_posts():
    o = built(post_b=in_tote(), cap=in_tote(0.02))
    assert score_of(o)[0] == pytest.approx(0.2)


def test_the_orange_beam_across_the_posts_is_not_the_beam():
    ob = S(np.array([GATE_X, GATE_Y, POST_TOP + h('orange_beam') / 2]))
    o = built(beam=in_tote(), orange_beam=ob, cap=in_tote(0.02))
    assert score_of(o)[0] == pytest.approx(0.4 - 0.25)


def test_the_blue_cube_on_the_beam_is_not_the_cube():
    bc = S(np.array([GATE_X, GATE_Y, BEAM_TOP + h('blue_cube') / 2]))
    o = built(cap=in_tote(0.02), blue_cube=bc)
    assert score_of(o) == (pytest.approx(0.7 - 0.25), False)


@pytest.mark.parametrize('deg', [0.0, 3.0, 30.0, 90.0, 180.0])
def test_cylinder_extent_is_exact_at_any_tilt(deg):
    q = quat((1, 0, 0), deg)
    low, high = z_extent('post_a', (0.0, 0.0, 1.0), q)
    t = math.radians(deg)
    reach = 0.04 * abs(math.cos(t)) + 0.02 * abs(math.sin(t))
    assert low == pytest.approx(1.0 - reach) and high == pytest.approx(1.0 + reach)


@pytest.mark.parametrize('dx_mm, counts', [(10.0, True), (12.0, False)])
def test_cap_in_the_middle(dx_mm, counts):
    c = built()['cap']
    o = built(cap=moved(c, pos=c.pos + [0, dx_mm / 1000, 0]))
    assert score_of(o) == (pytest.approx(1.0 if counts else 0.7), counts)


def test_cap_any_face_down_but_flat():
    c = built()['cap']
    assert score_of(built(cap=moved(c, quat_xyzw=quat((1, 0, 0), 90))))[0] == pytest.approx(1.0)
    assert score_of(built(cap=moved(c, quat_xyzw=quat((1, 0, 0), 30), pos=c.pos + [0, 0, 0.008])))[0] == \
        pytest.approx(0.7)


def test_cap_without_the_beam_does_not_count():
    c = S(np.array([GATE_X, GATE_Y, POST_TOP + h('cap') / 2]))  # wedged on the posts with no beam
    o = built(beam=in_tote(), cap=c)
    assert score_of(o)[0] == pytest.approx(0.4)


def test_moving_parts_do_not_count():
    o = built()
    o['cap'] = moved(o['cap'], lin_vel=np.array([0.02, 0, 0]))
    assert score_of(o) == (pytest.approx(0.7), False)
    o = built()
    o['post_a'] = moved(o['post_a'], ang_vel=np.array([0, 0, 0.6]))
    assert score_of(o)[0] == pytest.approx(0.2)


def test_gone_downstream_needs_past_the_right_edge_and_down():
    assert gone_downstream(in_tote())
    assert gone_downstream(S(np.array([0.3, -0.9, 0.02])))  # on the floor past the edge
    assert not gone_downstream(S(np.array([0.3, -0.5, 0.02])))  # on the floor beside the table
    assert not gone_downstream(S(np.array([BELT_X, -0.70, BELT_TOP + 0.02])))  # still on the end of the belt


def belt_at(t, **over):
    """Every part riding untouched on the belt t seconds into the trial (t >= 1: not at their starts)."""
    o = {n: S(np.array([BELT_X, start_y(n) - SPEED * t, BELT_TOP + h(n) / 2]), lin_vel=np.array([0, -SPEED, 0]))
         for n in ORDER}
    o.update(over)
    return o


def held(name, t0, seconds, creep=0.0, jitter=0.0):
    """``name`` held on the belt from t0 for ``seconds`` while the others ride on; it creeps downstream at ``creep``
    m/s and its reported velocity jitters by up to ``jitter`` m/s (as a part pinned against a gripper does)."""
    rng = np.random.default_rng(0)
    y0 = start_y(name) - SPEED * t0
    seq = []
    for k in range(int(round(seconds / CONTROL_DT)) + 1):
        t = t0 + k * CONTROL_DT
        vy = -creep + rng.uniform(-jitter, jitter)
        part = S(np.array([BELT_X, y0 - creep * k * CONTROL_DT, BELT_TOP + h(name) / 2]), lin_vel=np.array([0, vy, 0]))
        seq.append(belt_at(t, **{name: part}))
    return seq


def riding(t0, seconds):
    return [belt_at(t0 + k * CONTROL_DT) for k in range(int(round(seconds / CONTROL_DT)) + 1)]


def test_parts_riding_the_belt_never_jam():
    trial, score, ok = run(riding(1.0, 20.0))
    assert not trial.jammed


def test_holding_a_part_back_on_the_belt_jams_the_line():
    seq = riding(2.0, 2.95) + held('beam', 5.0, 2.5) + [built()]  # stopped 2.5 s: 20 mm in the last 3 s
    trial, score, ok = run(seq)
    assert trial.jammed and score == 0.0 and not ok


def test_a_hold_with_a_jittering_velocity_still_jams():
    seq = held('post_b', 5.0, 3.2, jitter=0.06) + [built()]
    trial, score, ok = run(seq)
    assert trial.jammed and score == 0.0


def test_a_part_dragged_slowly_along_the_belt_jams():
    seq = held('cap', 5.0, 3.5, creep=0.006) + [built()]
    assert run(seq)[0].jammed


def test_a_part_slowed_but_not_held_back_is_fine():
    seq = held('cap', 5.0, 3.0, creep=0.012) + [built()]
    trial, score, ok = run(seq)
    assert not trial.jammed and ok


def test_brief_hold_while_catching_is_fine():
    """A slow catch: the part stopped on the belt for 1.4 s, then lifted off; and two 1.4 s holds 2 s apart."""
    lifted = [belt_at(6.45 + k * CONTROL_DT, beam=S(np.array([BELT_X, 0.0, BELT_TOP + 0.05]))) for k in range(20)]
    trial, score, ok = run(held('beam', 5.0, 1.4) + lifted + [built()])
    assert not trial.jammed and ok
    seq = held('beam', 5.0, 1.4) + riding(6.45, 2.0) + held('beam', 8.5, 1.4) + [built()]
    trial, score, ok = run(seq)
    assert not trial.jammed and ok


def test_part_lifted_off_the_belt_is_not_held_back():
    seq = []
    for k in range(60):
        o = belt_at(5.0 + k * CONTROL_DT)
        o['cap'] = S(np.array([BELT_X, 0.0, BELT_TOP + 0.02 + 0.010]))  # held 10 mm above the belt, still
        seq.append(o)
    trial, score, ok = run(seq + [built()])
    assert not trial.jammed and ok


def test_a_post_knocked_over_on_the_belt_is_not_held_back():
    """Slowed by a passing hand for 1 s, then tipped over upstream (its centre 40 mm back), then riding on."""
    seq = riding(28.0, 2.0)
    y = start_y('post_a') - SPEED * 30.0
    for k in range(20):  # 1 s at 10 mm/s
        seq.append(belt_at(30.0 + k * CONTROL_DT, post_a=S(np.array([BELT_X, y - 0.01 * k * CONTROL_DT,
                                                                          BELT_TOP + 0.04]))))
    y = y - 0.01 + 0.04  # tipped over upstream, lying on the belt
    for k in range(80):
        seq.append(belt_at(31.0 + k * CONTROL_DT, post_a=S(np.array([BELT_X, y - SPEED * k * CONTROL_DT, BELT_TOP + 0.02]),
                                                           quat_xyzw=quat((1, 0, 0), 90), lin_vel=np.array([0, -SPEED, 0]))))
    assert not run(seq)[0].jammed


def test_parts_lying_still_in_the_tote_under_the_belt_end_do_not_jam():
    seq = []
    for k in range(80):
        o = belt_at(50.0 + k * CONTROL_DT)
        # A tall post that fell off the end lies in the tote, its centre still under the belt's last 5 mm.
        o['post_a'] = S(np.array([BELT_X - 0.04, -0.717, TOTE_FLOOR + 0.02]), quat_xyzw=quat((1, 0, 0), 90))
        o['beam'] = S(np.array([BELT_X, -0.70, TOTE_FLOOR + 0.10]), quat_xyzw=quat((1, 0, 0), 60))
        seq.append(o)
    assert not run(seq)[0].jammed


def test_a_reset_clears_the_jam():
    trial = Trial()
    for o in held('beam', 5.0, 3.5):
        trial.observe(o)
    assert trial.jammed
    trial.observe(start())
    assert not trial.jammed


def test_full_score_end_states_are_the_only_ones():
    o = built()
    assert score_of(o)[0] == 1.0
    o2 = built(post_a=built()['post_b'], post_b=built()['post_a'])  # posts swapped
    assert score_of(o2) == (1.0, True)
    flipped = built(beam=moved(built()['beam'], quat_xyzw=quat((1, 0, 0), 180)))
    assert score_of(flipped) == (1.0, True)
