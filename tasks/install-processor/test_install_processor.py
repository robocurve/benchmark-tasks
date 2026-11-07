"""Checks of install_processor.py's scoring, layout and demo, without the simulator. Needs yam_arena (the official
stack), from this folder:

    python -m pytest -q test_install_processor.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("install_processor.py"))
M = TASK.module
from mechanism import MechanismState  # noqa: E402  (the task file puts its folder on the path)

SX, SY = M.SOCKET_XY
# the processor's box in its own frame (m): substrate mid-plane at z = 0, lid on top
LOCAL_LOW = np.array([-M.CPU[0] / 2, -M.CPU[1] / 2, -M.SUBSTRATE_T / 2]) / 1000
LOCAL_HIGH = np.array([M.CPU[0] / 2, M.CPU[1] / 2, M.SUBSTRATE_T / 2 + M.LID_T]) / 1000


def _quat(rot):
    w = math.sqrt(max(1e-12, 1 + np.trace(rot))) / 2
    return np.array([(rot[2, 1] - rot[1, 2]) / (4 * w), (rot[0, 2] - rot[2, 0]) / (4 * w),
                     (rot[1, 0] - rot[0, 1]) / (4 * w), w])


def processor(x, y, bottom_z, yaw_deg=0.0, tilt_deg=0.0, moving=False):
    """The processor's state with its centre over x, y, turned ``yaw_deg``, tipped ``tilt_deg`` about its long axis,
    its lowest point at ``bottom_z``; pos is its bounding box's centre, as Arena reports it."""
    s, t = math.radians(yaw_deg), math.radians(tilt_deg)
    rz = np.array([[math.cos(s), -math.sin(s), 0], [math.sin(s), math.cos(s), 0], [0, 0, 1]])
    rx = np.array([[1, 0, 0], [0, math.cos(t), -math.sin(t)], [0, math.sin(t), math.cos(t)]])
    rot = rz @ rx
    corners = np.array([[a, b, c] for a in (LOCAL_LOW[0], LOCAL_HIGH[0]) for b in (LOCAL_LOW[1], LOCAL_HIGH[1])
                        for c in (LOCAL_LOW[2], LOCAL_HIGH[2])]) @ rot.T
    origin = np.array([x, y, bottom_z - corners[:, 2].min()])
    world = corners + origin
    return ObjectState(pos=(world.min(axis=0) + world.max(axis=0)) / 2, quat_xyzw=_quat(rot),
                       lin_vel=np.array([0.05, 0.0, 0.0]) if moving else np.zeros(3), ang_vel=np.zeros(3),
                       low=world.min(axis=0), high=world.max(axis=0))


def seated(dx=0.0, dy=0.0, lift=0.0, **kw):
    return processor(SX + dx, SY + dy, M.BOARD_TOP + lift, **kw)


def on_stand():
    return processor(*M.STAND_XY, M.TOP + M.STAND_H, yaw_deg=90.0)


def socket(shield_deg=0.0, lever_deg=0.0):
    joints = {"shield_hinge": math.radians(shield_deg), "lever_hinge": math.radians(lever_deg), "catch_slide": 0.0}
    return MechanismState(pos=np.array([SX, SY, M.TOP]), quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                          lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=np.array([SX - 0.06, SY - 0.05, M.TOP]),
                          high=np.array([SX + 0.07, SY + 0.05, M.TOP + 0.02]), joints=joints)


def scene(cpu=None, shield=0.0, lever=0.0):
    return {"processor": cpu or on_stand(), "socket": socket(shield, lever), "stand": None}


def test_the_start_scores_zero():
    assert M.score(scene()) == 0.0 and not M.success(scene())


def test_seated_with_the_shield_and_lever_shut_is_success():
    done = scene(seated())
    assert M.score(done) == 1.0 and M.success(done)


def test_credit_climbs_with_each_step():
    assert M.score(scene(seated(), shield=120, lever=-125)) == 0.4
    assert M.score(scene(seated(), shield=0, lever=-125)) == 0.7
    assert M.score(scene(seated(), shield=0, lever=0)) == 1.0


def test_shut_limits():
    assert M.score(scene(seated(), shield=2.9)) == 1.0
    assert M.score(scene(seated(), shield=3.1)) == 0.4
    assert M.score(scene(seated(), lever=-1.9)) == 1.0
    assert M.score(scene(seated(), lever=-2.1)) == 0.7


def test_a_lever_laid_on_the_catch_is_not_down():
    # on top of the catch the rod's axis sits sqrt((2+1.5)^2 - 2.5^2) mm above the catch's: about 4 degrees open
    lift = M.CATCH_DZ + math.sqrt((M.CATCH_R + M.LEVER_R) ** 2 - M.CATCH_DY ** 2)
    deg = math.degrees(math.atan2(lift, M.CATCH_X - M.LEVER_PIVOT_X))
    assert 3.5 < deg < 4.5
    assert M.score(scene(seated(), lever=-deg)) == 0.7


def test_nothing_without_the_processor_seated():
    for cpu in (on_stand(), seated(lift=0.0048),          # left on the shut shield
                seated(lift=0.0015, yaw_deg=180.0, tilt_deg=2.0),  # turned the wrong way, riding on the keys
                seated(yaw_deg=2.5), seated(yaw_deg=-2.5), seated(dx=0.0009), seated(dy=-0.0009), seated(tilt_deg=1.2),
                seated(lift=0.0006), seated(moving=True)):
        assert M.score(scene(cpu)) == 0.0
        assert M.score(scene(cpu, shield=120, lever=-125)) == 0.0


def test_seat_limits_inside():
    for cpu in (seated(yaw_deg=1.9), seated(yaw_deg=-1.9), seated(dx=0.0007), seated(dy=0.0005, dx=0.0005),
                seated(tilt_deg=0.9), seated(lift=0.0004)):
        assert M.score(scene(cpu)) == 1.0


def test_keys_fit_the_notches_one_way_only():
    py = M.POCKET[1] / 2
    key_reach = py - (py - 2 * M.KEY_R + 0.25)  # how far each key stands into the pocket from its wall
    assert 2 * M.KEY_R < M.NOTCH[0] and key_reach < M.NOTCH[1]
    assert M.CPU[1] / 2 - M.NOTCH[1] < py - key_reach  # notched edge clears the keys
    assert M.CPU[1] / 2 > py - key_reach  # an edge without notches lands on them


def test_pocket_and_cut_outs():
    assert M.POCKET[0] - M.CPU[0] == pytest.approx(0.5) and M.POCKET[1] - M.CPU[1] == pytest.approx(0.5)
    assert M.CUTOUT >= 26.0 + 2.0  # the fingertips are 26 mm wide
    assert M.LID < M.CPU[1]  # the fingers pinch the lid inside the pocket's long walls' gap


def test_the_shut_lever_holds_the_shield_down():
    bar_y = (-20.5 - 3.0, -20.5 + 3.0)  # the shield's -y bar
    bar_top = M.SHIELD_Z + M.SHIELD_T / 2
    assert bar_y[0] < M.LEVER_Y < bar_y[1]
    assert 0.0 < (M.LEVER_Z - M.LEVER_R) - bar_top < 1.0
    assert M.LEVER_PIVOT_X < -27.0  # the open lever stands clear of the bar


def test_the_catch_overlaps_the_shut_rod_and_clears_the_shield_and_fingers():
    overlap = (M.LEVER_R + M.CATCH_R) - M.CATCH_DY
    assert overlap == pytest.approx(1.0)
    # the shield at 120 degrees reaches x = hinge + r cos 60; past the catch only above it
    for r in np.linspace(0.0, 75.0, 76):
        x, z = M.SHIELD_HINGE_X + r * math.cos(math.radians(60)), M.SHIELD_Z + r * math.sin(math.radians(60))
        if M.CATCH_X - M.CATCH_LEN / 2 <= x <= M.CATCH_X + M.CATCH_LEN / 2:
            assert z > M.LEVER_Z + M.CATCH_DZ + M.CATCH_R
    finger = 10.0  # the fingers are 6 to 10 mm thick where they close
    assert (M.LEVER_END_X - M.KNOB_R) - (M.CATCH_X + M.CATCH_LEN / 2) >= finger


def test_demo_reaches_and_fits_the_run_limit():
    demo = M.demo()
    for arm in ("left", "right"):
        plan = ArmPlan(demo[arm], station.BASES[arm])
        for t in np.arange(0.0, plan.duration, 0.05):
            plan.action7(t)
        assert plan.worst_ik_mm < 2.0 and plan.worst_ik_deg < 2.0, arm
        assert plan.duration < 3000 / 20 * 0.6


def test_the_processor_starts_a_quarter_turn_out():
    assert M.PROCESSOR.yaw == pytest.approx(math.pi / 2)
    assert M.score(scene(processor(SX, SY, M.BOARD_TOP, yaw_deg=90.0))) == 0.0
