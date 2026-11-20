"""Scoring checks that matter for the hidden-leak task (pure Python: no Isaac, no GPU)."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from inspection_mirror import design as d  # noqa: E402
from inspection_mirror.scoring import LeakMarkScore, mark_credit, value  # noqa: E402

LEAK = d.LEAK_X[2]


def rest_for(sc, steps, marks, smeared=False):
    for _ in range(steps):
        sc.update(marks, smeared, put_down=True)
    return sc.score


def look_at(sc, steps=10, seen=True, marks=()):
    # a pause: the mirror head held still in the gap (10 steps = 0.5 s at 20 Hz), the trace seen in it or not
    for _ in range(steps):
        sc.update(list(marks), False, put_down=False, paused=True, seen=seen)


@pytest.mark.parametrize("err_mm,expected", [(0, 1.0), (6, 1.0), (10, 0.5), (14, 0.0), (20, 0.0), (-10, 0.5)])
def test_credit_band(err_mm, expected):
    assert mark_credit(err_mm) == pytest.approx(expected)


def test_neighbouring_place_earns_nothing():
    # a mark at the next place along (20 mm away) must score 0, or a guess near the middle would collect credit
    assert value([d.LEAK_X[3]], LEAK, False) == 0.0
    assert d.ZERO_MM < min(np.diff(d.LEAK_X)) * 1000


def test_stray_mark_and_carpet_marking():
    assert value([LEAK, LEAK + 0.030], LEAK, False) == 0.0  # one right, one wrong: the repair may go to the wrong spot
    assert value(list(d.LEAK_X), LEAK, False) == 0.0  # marking every place earns nothing
    assert value([LEAK + 0.002, LEAK - 0.003], LEAK, False) == 1.0  # two dabs on the right spot are fine


def test_smear_and_no_mark_score_zero():
    assert value([LEAK], LEAK, True) == 0.0
    assert value([], LEAK, False) == 0.0


def test_blind_guess_scores_zero():
    for guess in d.LEAK_X:  # a mark with no mirror look first earns nothing, at every leak place, even on the leak
        for leak in d.LEAK_X:
            sc = LeakMarkScore(20, leak)
            rest_for(sc, 25, [guess])
            assert sc.score == 0.0 and not sc.success


def test_score_needs_rest_and_follows_latest_rest():
    sc = LeakMarkScore(20, LEAK)
    look_at(sc)
    for _ in range(40):  # the marker still in the hand: nothing counts
        sc.update([LEAK], False, put_down=False)
    assert sc.score == 0.0
    assert rest_for(sc, 19, [LEAK]) == 0.0  # 0.95 s: not yet
    assert rest_for(sc, 1, [LEAK]) == 1.0 and sc.success
    sc = LeakMarkScore(20, LEAK)
    look_at(sc)
    rest_for(sc, 25, [LEAK + 0.010])
    assert sc.score == pytest.approx(0.5)
    sc.update([LEAK + 0.010, LEAK + 0.030], False, put_down=False)
    assert sc.score == 0.0  # a later stray mark lowers the score at once, before any rest
    assert rest_for(sc, 25, [LEAK + 0.010, LEAK + 0.030]) == 0.0


def test_smear_after_scored_rest_forces_zero():
    sc = LeakMarkScore(20, LEAK)
    look_at(sc)
    assert rest_for(sc, 25, [LEAK + 0.010]) == pytest.approx(0.5)  # a scored partial rest first
    sc.update([LEAK + 0.010], True, put_down=False)  # then the mirror smears the trace, moving
    assert sc.score == 0.0 and not sc.success  # zero at once, with no later rest
    for _ in range(40):  # it stays zero, moving or at rest, even if the smear input is not repeated
        sc.update([LEAK + 0.010], False, put_down=False)
    assert rest_for(sc, 25, [LEAK + 0.010]) == 0.0 and not sc.success


def test_head_clearance_geometry():
    centred = np.array([0.3, d.FACE_Y + d.GAP / 2, 0.85, 0, 0, 0, 1.0])
    clear = d.head_face_clearance(centred)
    # the 60.3 x 8 mm head at 45 degrees is 48 mm across in a 60 mm gap: about 6 mm each side
    assert clear == pytest.approx((d.GAP - d.HEAD_D * math.sin(d.HEAD_YAW) - d.HEAD_T * math.cos(d.HEAD_YAW)) / 2,
                                  abs=1e-4)
    shifted = centred.copy()
    shifted[1] -= 0.009
    assert d.head_face_clearance(shifted) < 0  # the touch check run: 9 mm toward the face goes through it
    rolled = centred.copy()
    rolled[3:] = [math.sin(math.radians(-10) / 2), 0, 0, math.cos(math.radians(-10) / 2)]  # -10 deg roll
    assert d.head_face_clearance(rolled) < clear  # rolling the hand swings the hanging head toward the face


def test_leak_place_varies_with_seed():
    places = [d.leak_index(s) for s in range(200)]
    assert set(places) == set(range(len(d.LEAK_X)))
    assert max(np.bincount(places)) / len(places) < 0.3


def test_hover_makes_no_mark():
    # a mark needs the tip to touch the strip; a rigid tip resting on it sits about 0.07 mm above (measured)
    top = d.TOP + d.CAP_TOP
    assert d.on_strip([0.34, 0.0, top + 0.00007])
    assert not d.on_strip([0.34, 0.0, top + 0.0010])  # hovering 1 mm above: no paint
    assert not d.on_strip([0.34, 0.0, top + 0.0015])
    assert not d.on_strip([0.20, 0.0, top])  # outside the strip


# The right wrist camera and the mirror at the demo's stop at place 2 (logged by a simulator run on 2026-11-17).
DEMO_MIRROR = np.array([0.173712, 0.004872, 0.847059, -0.018099, 0.01881, 0.007968, 0.999627])
DEMO_CAM = (np.array([0.102863, 0.002425, 0.814439]),
            np.array([[-0.010794, -0.002998, -0.999937], [0.998953, -0.044495, -0.01065],
                      [-0.04446, -0.999005, 0.003475]]),
            (390.666, 390.162, 317.526, 176.146, 640, 360))


def _seen(mirror, cam=DEMO_CAM, extra=()):
    return d.trace_seen(mirror, LEAK, [cam], d.static_occluders() + d.tool_occluders(mirror) + list(extra))


def test_look_needs_the_trace_in_the_mirror():
    assert _seen(DEMO_MIRROR)  # the demo's stop: the trace's middle shows in the glass
    off = DEMO_MIRROR.copy()
    off[0] += 0.023  # the head 23 mm along the gap from the trace (camera moved with it): only part of it shows
    cam = (DEMO_CAM[0] + [0.023, 0, 0], DEMO_CAM[1], DEMO_CAM[2])
    assert not _seen(off, cam)
    turned = DEMO_MIRROR.copy()  # the head turned to face the wall: the camera sees its steel back
    yaw = math.radians(90.0)
    c = d.head_center(DEMO_MIRROR)
    q = [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]
    turned[3:] = q
    turned[:3] = c - d.rot(q) @ d.HEAD_POINT
    assert not _seen(turned)
    m = d.reflected_point(DEMO_MIRROR, DEMO_CAM[0], d.trace_points(LEAK)[0])
    block = (DEMO_CAM[0] + 0.5 * (m - DEMO_CAM[0]), np.eye(3), np.array([0.01, 0.01, 0.01]))
    assert not _seen(DEMO_MIRROR, extra=[block])  # something between the camera and the glass
    assert not _seen(DEMO_MIRROR, cam=(DEMO_CAM[0], DEMO_CAM[1] @ np.diag([-1.0, 1.0, -1.0]), DEMO_CAM[2]))  # looking away


def test_last_pause_must_show_the_trace():
    def sweep(sc, places, seen_at):
        for k in places:
            for _ in range(10):  # moving between places: not a pause
                sc.update([], False, put_down=False, paused=False, seen=False)
            look_at(sc, steps=24, seen=(k == seen_at))

    # sweep 0..4 (the trace shows only at place 2), withdraw, mark place 2: the last pause (place 4) did not show it
    sc = LeakMarkScore(20, LEAK)
    sweep(sc, range(5), 2)
    sc.update([], False, put_down=False)
    assert rest_for(sc, 25, [LEAK]) == 0.0
    # the same, but back to place 2 for a still look before the mark: full credit
    sc = LeakMarkScore(20, LEAK)
    sweep(sc, [0, 1, 2, 3, 4, 2], 2)
    sc.update([], False, put_down=False)
    assert rest_for(sc, 25, [LEAK]) == 1.0
    # the trace in view but the mirror moving (a sweep past it): no look
    sc = LeakMarkScore(20, LEAK)
    for _ in range(30):
        sc.update([], False, put_down=False, paused=False, seen=True)
    assert rest_for(sc, 25, [LEAK]) == 0.0
    # a still look that ends 0.45 s after the trace came into view: too short
    sc = LeakMarkScore(20, LEAK)
    look_at(sc, steps=10, seen=False)
    look_at(sc, steps=9, seen=True)
    sc.update([], False, put_down=False)
    assert rest_for(sc, 25, [LEAK]) == 0.0


def test_any_contact_with_the_bounded_face_smears():
    def tool_at(c):  # the mirror level, its head centre at c
        return np.array([c[0] - d.HEAD_REACH, c[1], c[2] + d.HEAD_DROP, 0, 0, 0, 1.0])
    # the head's centre outside the gap's mouth, its rim on the face's near edge (the review's case)
    assert d.mirror_face_clearance(tool_at([0.224, -0.005933243, 0.8225])) <= d.CONTACT
    assert d.mirror_face_clearance(tool_at([0.340, 0.0, d.TOP + d.LEAK_Z])) > 0.005  # centred in the gap: ~6 mm
    assert d.mirror_face_clearance(d.start_poses()["mirror"]) > 0.05  # on its blocks, behind the housing side
    # a 50 x 50 mm finger face lying flat on the face: its corners are outside the face, its middle is on it
    pad = (np.array([0.24, d.FACE_Y + 0.002 + 0.0001, d.TOP + 0.06]), np.eye(3), np.array([0.025, 0.002, 0.025]))
    assert d.robot_face_clearance([pad]) <= d.CONTACT
    side = (np.array([0.20, d.FACE_Y - 0.003, d.TOP + 0.06]), np.eye(3), np.array([0.002, 0.002, 0.002]))
    assert d.robot_face_clearance([side]) > d.CONTACT  # by the housing's near end, not on the face
    # the head turned -45 degrees at the mouth, its centre outside the gap: a point of its glass 0.07 mm from the face
    # (a sampled disc read 0.28 mm here and missed the touch; the review's case)
    yaw = math.radians(-45.0)
    q = np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
    pose = np.r_[np.array([0.222930, 0.000220, 0.8225]) - d.rot(q) @ d.HEAD_POINT, q]
    assert d.mirror_face_clearance(pose) == pytest.approx(0.00007, abs=2e-6)
    assert d.mirror_face_clearance(pose) <= d.CONTACT
    marker = np.array([0.30, d.FACE_Y + d.MARKER[1] / 2, d.TOP + 0.08, 0, 0, 0, 1.0])  # the marker laid on the face
    assert d.marker_face_clearance(marker) <= d.CONTACT


def test_an_open_hand_holding_a_tool_is_not_put_down():
    rest = d.start_poses()["mirror"]
    # a 50 x 50 mm finger face 1 mm under the grip, centred on it: every corner is 25 mm or more from the tool,
    # its middle holds the tool up
    pad = (rest[:3] + [0.0, 0.0, -d.GRIP[1] / 2 - 0.001 - 0.002], np.eye(3), np.array([0.025, 0.025, 0.002]))
    assert d.mirror_robot_distance(rest, [pad]) < d.RELEASE
    assert not d.tool_down(0.0, 0.0, d.mirror_robot_distance(rest, [pad]), rest)
    up = (rest[:3] + [0.0, 0.0, 0.05], np.eye(3), np.array([0.025, 0.025, 0.002]))
    assert d.tool_down(0.0, 0.0, d.mirror_robot_distance(rest, [up]), rest)  # the hand 5 cm up
    assert not d.tool_down(0.0, 0.3, 1.0, rest)  # still turning (0.3 rad/s)
    on_strip = rest.copy()
    on_strip[:3] = [0.30, 0.0, d.TOP + d.CAP_TOP + 0.01]
    assert not d.tool_down(0.0, 0.0, 1.0, on_strip)  # left lying on the housing
