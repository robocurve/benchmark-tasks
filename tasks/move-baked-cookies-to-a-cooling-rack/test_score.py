"""Scorer tests: every band and each shortcut, on made-up object states. No simulator.

Created 11_15_26 19:10 EST.
Run:  python -m pytest test_score.py     (needs numpy, yam_arena and this folder's cookie_rack on the path)
"""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import task  # noqa: E402
from cookie_rack import parts as P  # noqa: E402
from yam_arena.task_file import ObjectState  # noqa: E402

ZERO = np.zeros(3)
RACK_Z = task.TOP + P.RACK_TOP


def quat(yaw=0.0, tilt=0.0):
    """Yaw about z, then a tilt about the turned x axis (xyzw)."""
    qz = np.array((0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))
    qx = np.array((math.sin(tilt / 2), 0.0, 0.0, math.cos(tilt / 2)))
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return np.array((w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2))


def state(pos, q=None, vel=ZERO):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=quat() if q is None else q, lin_vel=np.array(vel, float), ang_vel=ZERO, low=p, high=p)


def layout(on_rack=(), **over):
    """The starting layout, with the cookies whose numbers are in on_rack lying on the demo's rack spots."""
    s = {
        "pan": state((*task.PAN_XY, task.TOP)), "rack": state((*task.RACK_XY, task.TOP)),
        "turner": state((*task.REST_XY, task.TURNER_Z)),
        "left_tcp": state((0.20, 0.305, task.TOP + 0.17)), "right_tcp": state((0.20, -0.305, task.TOP + 0.17)),
    }
    for i, (x, y, a) in enumerate(task.COOKIES, 1):
        p = (*task.SPOTS[i - 1], RACK_Z + 0.0005) if i in on_rack else (x, y, task.FLOOR_Z)
        s[f"cookie_{i}"] = state(p, quat(math.radians(a)))
    s.update(over)
    return s


def run(s, seconds=1.5, touch=None, before=()):
    """Feed the scorer the earlier states once each, then the final state for a while; return the score."""
    task.reset_trial()
    task.note_step(layout(), 0.05, {})
    for b in before:
        b_state, b_touch = b if isinstance(b, tuple) else (b, {})
        task.note_step(b_state, 0.05, b_touch)
    for _ in range(round(seconds / 0.05)):
        task.note_step(s, 0.05, touch or {})
    return task.score(s)


ALL = (1, 2, 3, 4)


def test_nothing_done_scores_zero():
    assert run(layout()) == 0.0
    assert not task.success(layout())


@pytest.mark.parametrize("k", [1, 2, 3])
def test_each_good_cookie_is_a_fifth(k):
    assert run(layout(on_rack=ALL[:k])) == pytest.approx(0.2 * k)


def test_all_four_and_finished_is_full():
    s = layout(on_rack=ALL)
    assert run(s) == pytest.approx(1.0)
    assert task.success(s)


def test_all_four_but_turner_not_on_rest():
    s = layout(on_rack=ALL, turner=state((0.30, -0.52, task.TOP + 0.03)))
    assert run(s) == pytest.approx(0.8)
    s = layout(on_rack=ALL, turner=state((*task.REST_XY, task.TURNER_Z), quat(math.radians(40))))
    assert run(s) == pytest.approx(0.8)


def test_all_four_but_hand_not_clear():
    s = layout(on_rack=ALL, right_tcp=state((*task.REST_XY, task.TURNER_Z + 0.03)))
    assert run(s) == pytest.approx(0.8)
    s = layout(on_rack=ALL, left_tcp=state((0.30, 0.10, task.TOP + 0.03)))  # still at the pan's rim
    assert run(s) == pytest.approx(0.8)


def test_all_four_but_not_still_yet():
    assert run(layout(on_rack=ALL), seconds=0.5) == pytest.approx(0.8)


def test_all_four_but_pan_pushed_away():
    s = layout(on_rack=ALL, pan=state((task.PAN_XY[0] + 0.05, task.PAN_XY[1], task.TOP)))
    assert run(s) == pytest.approx(0.8)
    s = layout(on_rack=ALL, pan=state((*task.PAN_XY, task.TOP), quat(math.radians(15))))
    assert run(s) == pytest.approx(0.8)


def test_pan_within_tolerance_still_full():
    s = layout(on_rack=ALL, pan=state((task.PAN_XY[0] + 0.02, task.PAN_XY[1], task.TOP), quat(math.radians(5))))
    assert run(s) == pytest.approx(1.0)


def test_pan_tipped_to_pour_scores_zero():
    tipped = layout(pan=state((*task.PAN_XY, task.TOP + 0.03), quat(0.0, math.radians(25))))
    assert run(layout(on_rack=ALL), before=[tipped]) == 0.0


def test_pan_lifted_scores_zero():
    lifted = layout(pan=state((*task.PAN_XY, task.TOP + 0.05)))
    assert run(layout(on_rack=ALL), before=[lifted]) == 0.0


def test_touched_cookie_is_spoiled_even_on_the_rack():
    s = layout(on_rack=ALL)
    assert run(s, before=[(layout(), {"cookie_2": {"touch": 0.5}})]) == pytest.approx(0.6 - 0.1)


def test_all_picked_by_hand_scores_zero():
    touch = {f"cookie_{i}": {"touch": 2.0} for i in ALL}
    assert run(layout(on_rack=ALL), before=[(layout(), touch)]) == 0.0


def test_touch_below_threshold_is_noise():
    assert run(layout(on_rack=ALL), before=[(layout(), {"cookie_1": {"touch": 0.001}})]) == pytest.approx(1.0)


@pytest.mark.parametrize("load", [{"bend": 1.2 * P.BREAK_BEND}, {"squeeze": 1.2 * P.BREAK_SQUEEZE}, {"press": 1.2 * P.BREAK_PRESS}])
def test_broken_cookie_is_spoiled_wherever_it_ends(load):
    assert run(layout(on_rack=ALL), before=[(layout(), {"cookie_3": load})]) == pytest.approx(0.6 - 0.1)
    assert run(layout(on_rack=(1, 2)), before=[(layout(), {"cookie_4": load})]) == pytest.approx(0.4 - 0.1)


def test_loads_of_a_careful_transfer_do_not_break():
    gentle = {f"cookie_{i}": {"bend": 0.0122, "squeeze": 0.03, "press": 0.0, "fall": 0.3} for i in ALL}  # the peaks measured in the demo
    assert run(layout(on_rack=ALL), before=[(layout(), gentle)]) == pytest.approx(1.0)


def test_load_criterion_from_contact_forces():
    w = P.COOKIE_MASS * 9.81
    rest = P.loads([(0.03, 0, 0), (-0.03, 0, 0), (0, 0.03, 0), (0, -0.03, 0)], [(0, 0, w / 4)] * 4)
    assert rest[0].max() < 0.2 * P.BREAK_BEND and rest[1].max() == 0.0 and rest[2] == 0.0
    bridge = P.loads([(0.035, 0, 0), (-0.035, 0, 0)], [(0, 0, w / 2)] * 2)  # front edge on the blade, back edge on the pan
    assert bridge[0].max() < 0.2 * P.BREAK_BEND
    levered = P.loads([(0.03, 0, 0.01), (0.0, 0, 0)], [(0, 0, -2.0), (0, 0, 2.0 + w)])  # pressed down beyond its support
    assert levered[0].max() > P.BREAK_BEND
    gripped = P.loads([(0.038, 0, 0.005), (-0.038, 0, 0.005)], [(-5.0, 0, 0), (5.0, 0, 0)])
    assert gripped[1].max() > P.BREAK_SQUEEZE
    shoved = P.loads([(0.038, 0, 0.005)], [(-5.0, 0, 0)])  # a push with nothing behind it only slides the cookie
    assert shoved[1].max() == 0.0
    stamped = P.loads([(0, 0, 0.01), (0.03, 0, 0), (-0.03, 0, 0)], [(0, 0, -12.0), (0, 0, 6.0), (0, 0, 6.0)])
    assert stamped[2] > P.BREAK_PRESS
    assert P.loads(np.zeros((0, 3)), np.zeros((0, 3)))[0].max() == 0.0  # in the air: no contact, no load


def test_dropped_cookie_is_spoiled():
    falling = layout()
    falling["cookie_1"] = state((0.33, -0.30, task.TOP + 0.10), vel=(0, 0, -1.0))
    assert run(layout(on_rack=ALL), before=[falling]) == pytest.approx(0.6 - 0.1)


def test_fall_seen_between_control_steps_is_a_drop():
    assert run(layout(on_rack=ALL), before=[(layout(), {"cookie_1": {"fall": 0.8}})]) == pytest.approx(0.6 - 0.1)
    assert run(layout(on_rack=ALL), before=[(layout(), {"cookie_1": {"fall": 0.3}})]) == pytest.approx(1.0)


def test_gentle_slide_off_the_blade_is_not_a_drop():
    sliding = layout()
    sliding["cookie_1"] = state((0.33, -0.30, RACK_Z + 0.01), vel=(0.2, 0, -0.35))
    assert run(layout(on_rack=ALL), before=[sliding]) == pytest.approx(1.0)


def test_cookie_on_the_counter_is_spoiled():
    s = layout(on_rack=(1,))
    s["cookie_2"] = state((0.33, 0.20, task.TOP), s["cookie_2"].quat_xyzw)
    assert run(s) == pytest.approx(0.2 - 0.1)


def test_only_spoiled_cookies_floor_at_zero():
    s = layout()
    s["cookie_2"] = state((0.33, 0.20, task.TOP), s["cookie_2"].quat_xyzw)
    assert run(s) == 0.0


def test_cookie_overhanging_the_rack_frame_does_not_count():
    s = layout(on_rack=ALL)
    edge = (task.RACK_XY[0] + P.RACK[0] / 2 - 0.010, task.RACK_XY[1], RACK_Z + 0.0005)
    s["cookie_1"] = state(edge, s["cookie_1"].quat_xyzw)
    assert run(s) == pytest.approx(0.6)


def test_cookie_leaning_on_the_rack_does_not_count():
    s = layout(on_rack=ALL)
    s["cookie_1"] = state((*task.SPOTS[0], RACK_Z + 0.006), quat(0.0, math.radians(20)))
    assert run(s) == pytest.approx(0.6)


def test_cookie_stacked_on_another_does_not_count():
    s = layout(on_rack=ALL)
    s["cookie_1"] = state((*task.SPOTS[1], RACK_Z + P.COOKIE_T + 0.0005), s["cookie_1"].quat_xyzw)
    assert run(s) == pytest.approx(0.6)


def test_cookie_upside_down_does_not_count():
    s = layout(on_rack=ALL)
    s["cookie_1"] = state((*task.SPOTS[0], RACK_Z + P.COOKIE_T), quat(0.0, math.pi))
    assert run(s) == pytest.approx(0.6)


def test_cookie_still_on_the_blade_over_the_rack_does_not_count():
    s = layout(on_rack=ALL)
    tu = np.array((task.SPOTS[0][0] - P.BLADE_X0 - P.BLADE_L / 2, task.SPOTS[0][1], RACK_Z + 0.001 + P.NECK_DROP + P.BLADE_T))
    s["turner"] = state(tu)
    s["cookie_1"] = state((*task.SPOTS[0], RACK_Z + 0.001), s["cookie_1"].quat_xyzw)
    assert run(s) == pytest.approx(0.6)


def test_cookie_moving_does_not_count_yet():
    s = layout(on_rack=ALL)
    s["cookie_1"] = state(s["cookie_1"].pos, s["cookie_1"].quat_xyzw, vel=(0.1, 0, 0))
    assert run(s) == pytest.approx(0.6)


def test_rack_moved_with_its_cookies_still_counts():
    s = layout(on_rack=ALL)
    d = np.array((0.02, -0.02, 0.0))
    s["rack"] = state(np.array(s["rack"].pos) + d)
    for i in ALL:
        s[f"cookie_{i}"] = state(np.array(s[f"cookie_{i}"].pos) + d, s[f"cookie_{i}"].quat_xyzw)
    assert run(s) == pytest.approx(1.0)


def test_score_never_leaves_zero_to_one():
    s = layout(on_rack=ALL)
    assert 0.0 <= run(s, before=[(layout(), {f"cookie_{i}": {"touch": 9.0, "bend": 1.0} for i in ALL})]) <= 1.0


def test_demo_plan_is_within_reach_and_time():
    plan = task.demo()["right"][0]
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.5
    assert plan.duration < 115.0
