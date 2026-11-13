"""Scorer cases the generated layout tests cannot express: timed sequences (sticky rules, the 1 s stillness window, the
forcing cap) and exact partial-credit geometry (a sprung point on the chamfer).

    pytest test_red_team.py    (in an environment with yam_arena installed)
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest
from yam_arena.task_file import load

sys.path.insert(0, str(Path(__file__).resolve().parent))
from yam_parts import PartState  # noqa: E402

TASK = load(Path(__file__).with_name("mount_shaft_between_bench_centres.py"))
T = TASK.module
DT = 0.05
MOUNTED = np.array([T.BX, T.MOUNT_MID_Y, T.ZC])
AWAY = {"left": (np.array(T.PARK_L), 1.0), "right": (np.array(T.PARK_R), 1.0)}


def shaft(pos, yaw_deg=0.0, roll_deg=0.0, vel=(0.0, 0.0, 0.0)):
    """A shaft state: turned yaw about z, then rolled about its own axis (local y)."""
    y, r = math.radians(yaw_deg) / 2, math.radians(roll_deg) / 2
    qz = np.array([0.0, 0.0, math.sin(y), math.cos(y)])
    qy = np.array([0.0, math.sin(r), 0.0, math.cos(r)])
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qy
    q = np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                  w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
    p = np.asarray(pos, float)
    return PartState(p, q, np.asarray(vel, float), np.zeros(3), p - 0.08, p + 0.08, p)


def bench(sleeve, lever_deg=0.0):
    return T.BENCH.offline_state((0.0, 0.0, 0.0), 0.0, {"sleeve": sleeve, "lever": math.radians(lever_deg)})


def objs(s, b):
    return {"shaft": s, "bench": b, "vblocks": None}


@pytest.fixture
def clock(monkeypatch):
    """Drive the scorer's run context like the harness: one call per 50 ms control step, fresh episode."""
    state = {"t": 0.0, "step": 0}
    T._RUN["episode"] += 1
    monkeypatch.setitem(T._RUN, "env", None)
    monkeypatch.setattr(T, "TEST_GRIPPERS", AWAY)

    def tick(o, hands=None):
        if hands is not None:
            T.TEST_GRIPPERS = hands
        T._RUN["t"], T._RUN["step"] = state["t"], state["step"]
        value = T.score(o)
        state["t"] += DT
        state["step"] += 1
        return value

    yield tick
    T._RUN["t"] = None


def hold(tick, o, seconds, hands=None):
    value = None
    for _ in range(int(round(seconds / DT))):
        value = tick(o, hands)
    return value


def swung_shaft(psi_deg):
    """The shaft with its left hole on the fixed point's seat, turned psi about z through the tip."""
    return shaft(T.shaft_mid_for(psi_deg, 0.0), yaw_deg=psi_deg)


def chamfer_case(offset):
    """Turned about the fixed point so the free end's axis is `offset` from the line; the sprung point on its chamfer."""
    psi = math.degrees(math.atan2(offset, T.SHAFT_L - T.TIP_DEPTH))
    s = swung_shaft(psi)
    # advance the sprung point until its tip, 0.5 mm in radius, meets the 120-degree chamfer
    for q in np.linspace(-T.SLEEVE_TRAVEL, 0.0, 2101):  # eased forward from fully drawn back
        rad, depth = T.tip_in_end(s, T.right_tip(q))[-1]
        r_wall = T.HOLE_D3 / 2 - depth * math.tan(math.radians(60.0))
        if 0 <= depth <= T.CHAMFER_DEPTH and rad + T.POINT_TIP_R >= r_wall:
            return s, q
    raise AssertionError("no chamfer contact")


def test_honest_mount_scores_one_after_a_second_still(clock):
    assert hold(clock, objs(shaft(T.SHAFT_START), bench(0.0)), 1.0) == 0.0
    hold(clock, objs(swung_shaft(-12.0), bench(-0.010, math.degrees(T.LEVER_MAX))), 1.0)
    assert hold(clock, objs(swung_shaft(0.0), bench(-0.010, math.degrees(T.LEVER_MAX))), 1.0) == 0.25
    first = clock(objs(shaft(MOUNTED), bench(-T.SEAT_BACK)))
    assert first == 0.0 or first <= 0.75  # not yet still for a second
    assert hold(clock, objs(shaft(MOUNTED), bench(-T.SEAT_BACK)), 1.2) == 1.0


def test_spin_about_its_own_axis_does_not_break_stillness(clock):
    for k in range(40):
        value = clock(objs(shaft(MOUNTED, roll_deg=9.0 * k), bench(-T.SEAT_BACK)))
    assert value == 1.0


def test_a_drift_of_one_millimetre_is_not_still(clock):
    for k in range(40):
        value = clock(objs(shaft(MOUNTED + np.array([0.0, 0.0, 0.0006 * (k % 2)])), bench(-T.SEAT_BACK)))
    assert value < 1.0


def test_rammed_against_the_point_is_capped_at_half(clock):
    # lever at rest, the shaft's right end pushed into the sprung point 4.5 mm, then mounted perfectly
    for q in np.linspace(0.0, -0.0045, 10):
        clock(objs(shaft(MOUNTED + np.array([0.0, -0.003, 0.0])), bench(q)))
    assert hold(clock, objs(shaft(MOUNTED), bench(-T.SEAT_BACK)), 2.0) == 0.5


def test_pushed_back_under_three_millimetres_is_not_forcing(clock):
    for q in np.linspace(-T.SEAT_BACK, -0.0040, 10):
        clock(objs(shaft(MOUNTED), bench(q)))
    assert hold(clock, objs(shaft(MOUNTED), bench(-T.SEAT_BACK)), 1.5) == 1.0


def test_point_stopped_on_the_chamfer_then_corrected_is_full_credit(clock):
    s, q = chamfer_case(0.004)
    hold(clock, objs(swung_shaft(0.0), bench(-0.010, math.degrees(T.LEVER_MAX))), 0.5)
    for qq in np.linspace(-0.010, q, 8):  # eased up onto the chamfer: the point stops short, nothing pushes it back
        clock(objs(s, bench(qq, 0.0)))
    assert hold(clock, objs(s, bench(q, 0.0)), 1.5) == 0.5
    hold(clock, objs(s, bench(-0.010, math.degrees(T.LEVER_MAX))), 0.5)  # lever down again, realigned, eased up
    assert hold(clock, objs(shaft(MOUNTED), bench(-T.SEAT_BACK)), 1.5) == 1.0


def test_point_on_the_chamfer_released_and_still_is_half(clock):
    s, q = chamfer_case(0.004)
    rad, depth = T.tip_in_end(s, T.right_tip(q))[-1]
    assert 3.3e-3 < rad < 5e-3 and depth < 1e-3
    assert hold(clock, objs(s, bench(q, 0.0)), 1.5) == 0.5


def test_point_on_the_chamfer_while_held_is_a_quarter(clock):
    s, q = chamfer_case(0.004)
    held = {"left": (T.shaft_mid_for(0.0, 0.0) + np.array([0, 0, T.GRIP_DZ]), 0.0), "right": AWAY["right"]}
    assert hold(clock, objs(s, bench(q, 0.0)), 1.5, held) == 0.25


def test_point_on_the_end_face_released_is_half(clock):
    s, _ = chamfer_case(0.004)
    s = swung_shaft(math.degrees(math.atan2(0.0065, T.SHAFT_L - T.TIP_DEPTH)))
    q = -(T.SEAT_BACK + T.TIP_DEPTH)  # tip on the end face
    assert T.point_state(s, T.right_tip(q))[0] == "bearing"
    assert hold(clock, objs(s, bench(q, 0.0)), 1.5) == 0.5


def test_dropped_is_sticky_even_if_put_back(clock):
    hold(clock, objs(shaft((T.BX, T.MOUNT_MID_Y, T.BENCH_TOP + T.SHAFT_R)), bench(0.0)), 0.3)
    assert hold(clock, objs(shaft(MOUNTED), bench(-T.SEAT_BACK)), 2.0) == 0.0


def test_a_hand_on_the_knob_is_three_quarters(clock):
    near = {"left": AWAY["left"], "right": (np.array(T.knob_tcp(0.0)), 0.0)}
    assert hold(clock, objs(shaft(MOUNTED), bench(-T.SEAT_BACK)), 1.5, near) == 0.75


def test_hand_31_mm_from_the_shaft_counts_as_off(clock):
    bounds = T.shaft_bounds(shaft(MOUNTED))
    tcp = np.array([T.BX, T.MOUNT_MID_Y, bounds.high[2] + 0.031])
    hands = {"left": (tcp, 1.0), "right": AWAY["right"]}
    assert hold(clock, objs(shaft(MOUNTED), bench(-T.SEAT_BACK)), 1.5, hands) == 1.0


def test_lever_left_partly_down_is_three_quarters(clock):
    a = math.degrees(math.asin(0.0015 / T.FINGER_H))  # knob about 3 mm down, finger off the lug
    assert T.knob_drop(math.radians(a)) > T.LEVER_REST_DROP
    assert hold(clock, objs(shaft(MOUNTED), bench(-T.SEAT_BACK, a)), 1.5) == 0.75


def test_reversed_shaft_is_full_credit(clock):
    assert hold(clock, objs(shaft(MOUNTED, yaw_deg=180.0), bench(-T.SEAT_BACK)), 1.5) == 1.0


def test_off_the_table_is_zero(clock):
    assert hold(clock, objs(shaft((0.95, 0.0, T.TOP - 0.1)), bench(0.0)), 0.5) == 0.0


def test_lying_on_the_points_flanks_is_zero():
    s = shaft(MOUNTED + np.array([0.0, 0.0, 0.003]))
    assert T.score(objs(s, bench(-T.SEAT_BACK))) == 0.0


def test_new_episode_clears_the_sticky_rules(clock):
    hold(clock, objs(shaft((T.BX, T.MOUNT_MID_Y, T.BENCH_TOP + T.SHAFT_R)), bench(0.0)), 0.3)
    T._RUN["episode"] += 1
    assert hold(clock, objs(shaft(MOUNTED), bench(-T.SEAT_BACK)), 1.5) == 1.0
