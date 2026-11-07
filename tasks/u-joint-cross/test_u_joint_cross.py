"""Checks of u_joint_cross.py's scoring and demo, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_u_joint_cross.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("u_joint_cross.py"))
M = TASK.module
MM = 0.001
SINK = M.BORE_R - M.JOURNAL_R      # let go, the journals rest on the bottoms of their holes: 6 mm below the axis


def quat(yaw_deg=0.0, tilt_deg=0.0):
    """Tilted about x (the pair's far end up), then turned yaw about z."""
    a, b = math.radians(yaw_deg) / 2, math.radians(tilt_deg) / 2
    x1, y1, z1, w1 = 0.0, 0.0, math.sin(a), math.cos(a)
    x2, y2, z2, w2 = math.sin(b), 0.0, 0.0, math.cos(b)
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def cross(dx=0.0, dy=0.0, dz=-SINK, yaw=0.0, tilt=0.0, moving=False, start=False):
    """The cross with its centre (dx, dy, dz) from the middle of the holes' axis (default: let go there, resting in
    the holes), or where it starts."""
    if start:
        pos, q = np.array([*M.HOLD, M.TOP + M.REST + M.HALF_SPAN]), quat(math.degrees(M.START_YAW))
    else:
        pos, q = np.array([*M.BORE, M.BORE_Z]) + np.array([dx, dy, dz]), quat(yaw, tilt)
    return {"cross": ObjectState(pos=pos, quat_xyzw=q, lin_vel=np.array([0.05, 0, 0]) if moving else np.zeros(3),
                                 ang_vel=np.zeros(3), low=pos - 0.0365, high=pos + 0.0365)}


def test_start_scores_zero():
    assert M.score(cross(start=True)) == 0 and not M.success(cross(start=True))


def test_centred_and_still_is_success():
    """1.0 only from success held for 1 s; score() alone tops out at 0.75."""
    assert M.success(cross()) and M.score(cross()) == 0.75
    assert M.success(cross(dz=0.0)) and M.score(cross(dz=0.0)) == 0.75     # held on the axis


def test_centred_but_moving_is_not_success():
    assert not M.success(cross(moving=True)) and M.score(cross(moving=True)) == 0.75


def test_the_cross_is_symmetric():
    """Turned 180 deg about the vertical (the other journal of the pair on the left) it scores the same."""
    assert M.success(cross(yaw=180.0)) and M.score(cross(dy=8 * MM, yaw=180.0)) == 0.25
    assert M.success(cross(tilt=180.0))                  # upside down


def _turned(axis, deg):
    """The cross let go centred in the holes, turned ``deg`` about world ``axis`` (0 x, 1 y)."""
    pos = cross()["cross"].pos
    q = np.zeros(4)
    q[axis], q[3] = math.sin(math.radians(deg) / 2), math.cos(math.radians(deg) / 2)
    return {"cross": ObjectState(pos=pos, quat_xyzw=q, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=pos, high=pos)}


@pytest.mark.parametrize("roll", [37.0, 90.0, -90.0])
def test_rolled_in_its_holes_is_still_installed(roll):
    """The cross turns freely about the holes' axis once its journals are in."""
    assert M.success(_turned(1, roll))


def test_the_other_pair_can_go_in():
    """Both pairs are the same: turned 90 deg about x, the second pair is the one in the holes."""
    assert M.success(_turned(0, -90.0)) and M.score(_turned(0, 90.0)) == 0.75


def test_the_end_state_of_a_simulated_demo_is_success():
    """The cross as the task's hooks received it at the last step of an `inspect-robots run` of demo()."""
    s = ObjectState(pos=np.array(END_POS), quat_xyzw=np.array(END_QUAT), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                    low=np.zeros(3), high=np.zeros(3))
    assert M.success({"cross": s})


# Each journal reaches 6.6 mm past its ear's inner face when centred. Both at least 3.5 mm in: off centre by up to 3.1 mm
@pytest.mark.parametrize("dy,want", [(3.0 * MM, 0.75), (-3.0 * MM, 0.75), (3.2 * MM, 0.5), (-3.2 * MM, 0.5),
                                     (5.5 * MM, 0.5), (-5.5 * MM, 0.5), (5.7 * MM, 0.25), (-5.7 * MM, 0.25)])
def test_centring(dy, want):
    assert M.score(cross(dy=dy)) == want


def test_pushed_deep_into_one_ear_is_not_installed():
    """One journal through its hole and out past the ear (11 mm over), the other short of its hole: 0.25."""
    for dy in (11 * MM, -11 * MM, 15 * MM):
        assert M.score(cross(dy=dy)) == 0.25 and not M.success(cross(dy=dy))


@pytest.mark.parametrize("yaw,want", [(4.9, 0.75), (-4.9, 0.75), (5.1, 0.5), (-5.1, 0.5), (9.0, 0.5)])
def test_turned_in_the_holes(yaw, want):
    assert M.score(cross(dz=0.0, yaw=yaw)) == want


@pytest.mark.parametrize("tilt,want", [(4.9, 0.75), (-4.9, 0.75), (5.1, 0.5), (-5.1, 0.5)])
def test_tilted_in_the_holes(tilt, want):
    assert M.score(cross(dz=0.0, tilt=tilt)) == want


@pytest.mark.parametrize("off,want", [(7.9 * MM, 0.75), (8.1 * MM, 0.0)])
def test_tip_distance_from_the_axis(off, want):
    """A tip's centre within 8 mm of the holes' axis is in its hole (in the hole it can be 6 mm off at most)."""
    assert M.score(cross(dz=-off)) == want
    assert M.score(cross(dx=off, dz=0.0)) == want


def test_on_top_of_the_ears_scores_zero():
    """Set down level across the ears' tops (the hole boss is 22 mm round the axis): journals over the ears, not in."""
    assert M.score(cross(dz=0.022 + M.JOURNAL_R)) == 0.0
    assert M.score(cross(dz=0.022 + M.JOURNAL_R, dy=10 * MM)) == 0.0


def test_one_journal_in_at_an_angle():
    """Partway through the swing: the right journal 10 mm into its hole, the pair at 30 deg. 0.25."""
    a = math.radians(30.0)
    # the pair points along (-sin a, cos a); its right tip, at centre - HALF_SPAN * that, goes on the axis 10 mm deep
    cx, cy = -M.HALF_SPAN * math.sin(a), -(M.GAP / 2 + 0.010) + M.HALF_SPAN * math.cos(a)
    assert M.score(cross(dx=cx, dy=cy, dz=0.0, yaw=30.0)) == 0.25


def test_outside_the_yoke_scores_zero():
    """On the holes' axis but outboard of the left ear, its inner journal reaching into the left hole from outside."""
    assert M.score(cross(dy=0.060, dz=0.0)) == 0.0


def test_fallen_on_the_table_scores_zero():
    assert M.score(cross(dz=-(M.BORE_Z - M.TOP) + M.JOURNAL_R)) == 0.0


def test_distance_is_zero_only_when_installed():
    assert M.distance(cross()) == 0.0 and M.distance(cross(dy=11 * MM)) > 0 and M.distance(cross(start=True)) > 0.1


def test_layout():
    names = [o.name for o in M.OBJECTS]
    assert len(set(names)) == len(names) and {"yoke", "cross"} <= set(names)
    free = [o.name for o in M.OBJECTS if o.mass is not None]
    assert free == ["cross"]
    # the pair is longer than the gap: it cannot come down level, by 6.6 mm at each end
    assert 2 * M.HALF_SPAN - M.GAP == pytest.approx(0.0132, abs=1e-6)
    assert M.BORE_R * 2 == pytest.approx(1.062 * 0.0254, abs=1e-4)


def test_make_meshes_matches_the_task():
    import importlib.util
    spec = importlib.util.spec_from_file_location("mm", Path(__file__).with_name("make_meshes.py"))
    mm = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mm)
    b = mm.bounds()
    assert b["GAP"] == pytest.approx(M.GAP) and b["BORE_R"] == pytest.approx(M.BORE_R)
    assert mm.AXIS_H + M.REST + M.TOP == pytest.approx(M.BORE_Z)
    assert (mm.HALF_SPAN, mm.JOURNAL_R, mm.HUB) == (M.HALF_SPAN, M.JOURNAL_R, M.HUB)
    lo, hi = b["yoke"]["lo"], b["yoke"]["hi"]
    assert max(hi[0] - lo[0], hi[1] - lo[1]) == pytest.approx(M.YOKE_EXT)
    assert ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2) == pytest.approx(M.YOKE_C)
    lo, hi = b["cross"]["lo"], b["cross"]["hi"]
    assert max(hi[0] - lo[0], hi[1] - lo[1]) == pytest.approx(M.CROSS_EXT)
    assert mm.yoke() == Path(__file__).with_name("ujoint_yoke.usda").read_text()
    assert mm.cross() == Path(__file__).with_name("ujoint_cross.usda").read_text()


def test_demo_is_reachable():
    plan = ArmPlan(M.demo()["right"], station.BASES["right"])
    t = 0.0
    while t < plan.duration:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 1.0 and plan.worst_ik_deg < 0.5


@pytest.mark.parametrize("delay", [0.0, 0.05, 0.10])
def test_demo_delay_only_shifts_the_start(delay):
    w0, w = M.demo()["right"], M.demo(delay)["right"]
    assert w[0].duration == pytest.approx(w0[0].duration + delay)
    assert [x.pos for x in w] == [x.pos for x in w0]


END_POS = [0.37884, -0.15935, 0.80237]
END_QUAT = [-0.00247, -0.04851, -0.00979, 0.99877]
