"""Checks of load_syringe_pump.py's scoring, layout and demo, without the simulator. Needs yam_arena (the official
stack), from this folder:

    python -m pytest -q test_load_syringe_pump.py
"""

import json
import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("load_syringe_pump.py"))
M = TASK.module
from mechanism import MechanismState  # noqa: E402  (the task file puts its folder on the path)

PY = M.PUMP_XY[1]


def _rx(deg):
    t = math.radians(deg)
    return np.array([[1, 0, 0], [0, math.cos(t), -math.sin(t)], [0, math.sin(t), math.cos(t)]])


def _quat(rot):
    w = math.sqrt(max(1e-12, 1 + np.trace(rot))) / 2
    return np.array([(rot[2, 1] - rot[1, 2]) / (4 * w), (rot[0, 2] - rot[2, 0]) / (4 * w),
                     (rot[1, 0] - rot[0, 1]) / (4 * w), w])


def part(centre_local, origin, rot, moving=False):
    """A part's state from its frame: pos is its bounding box's centre, as Arena reports it."""
    pos = np.asarray(origin, float) + rot @ centre_local
    return ObjectState(pos=pos, quat_xyzw=_quat(rot), lin_vel=np.array([0.05, 0, 0]) if moving else np.zeros(3),
                       ang_vel=np.zeros(3), low=pos - 0.02, high=pos + 0.02)


def scene(origin=None, rot=None, pushed=0.0, gap=None, moving=False, sag_deg=0.0):
    """The syringe's barrel centre at ``origin`` (seated by default), its plunger pushed in by ``pushed`` (m) and sagging
    ``sag_deg`` in the bore, and the driver's face ``gap`` (m) from the thumb disc's outermost point (at its start if
    None)."""
    origin = np.array(M.SEAT if origin is None else origin, float)
    rot = np.eye(3) if rot is None else rot
    axis = rot @ np.array([0.0, 1.0, 0.0])
    p_rot = rot @ _rx(sag_deg)
    p_origin = origin + axis * (M.STOPPER_AT / 1000 - pushed)
    centre = p_origin + p_rot @ np.array([0.0, M.THUMB_FROM_STOPPER / 1000, 0.0])
    thumb_y = centre[1] + M.THUMB_HALF * (abs(p_rot[1, 0]) + abs(p_rot[1, 2]))
    face_home = PY + M.DRIVER_FACE_Y / 1000
    slide = 0.0 if gap is None else (thumb_y + gap) - face_home
    pump = MechanismState(pos=np.array([*M.PUMP_XY, M.TOP]), quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                          lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=np.zeros(3), high=np.ones(3),
                          joints={"driver_slide": slide})
    return {"syringe": part(M._SC, origin, rot, moving), "plunger": part(M._PC, p_origin, p_rot, moving), "pump": pump}


def on_stands():
    return scene(origin=(M.STAND_XY[0], M.STAND_XY[1], M.START_Z))


def seated_at(dx=0.0, dy=0.0, dz=0.0):
    return M.SEAT + np.array([dx, dy, dz])


def test_the_start_scores_zero():
    s = on_stands()
    assert M.score(s) == 0.0 and not M.success(s)


def test_seated_with_the_driver_flush_is_success():
    s = scene(gap=0.0005)
    assert M.score(s) == 1.0 and M.success(s)


def test_credit_climbs_with_each_step():
    assert M.score(scene(origin=seated_at(dz=0.006), rot=_rx(8))) == 0.2  # cocked on the body: the PMDA failure
    assert M.score(scene()) == 0.5  # seated, driver still at its start
    assert M.score(scene(gap=0.004)) == 0.75
    assert M.score(scene(gap=0.0009)) == 1.0


def test_driver_limits():
    assert M.score(scene(gap=0.0011)) == 0.75
    assert M.score(scene(gap=0.0049)) == 0.75
    assert M.score(scene(gap=0.0051)) == 0.5
    assert M.score(scene(gap=0.0005, moving=True)) == 0.75


def test_a_pushed_plunger_scores_zero_whatever_else():
    assert M.score(scene(pushed=0.0011, gap=0.0)) == 0.0
    assert M.score(scene(pushed=0.0011)) == 0.0
    assert M.score(scene(pushed=0.0009, gap=0.0005)) == 1.0
    assert M.score(scene(pushed=-0.002, gap=0.0005)) == 1.0  # pulled back a little is no dose


def test_a_sagging_plunger_meets_the_driver_at_its_rim():
    # a tilted thumb disc reaches the driver with its rim first: the gap is measured to the rim, not the centre
    assert M.score(scene(gap=0.0005, sag_deg=3.0)) == 1.0
    assert M.score(scene(gap=0.0015, sag_deg=3.0)) == 0.75


def test_flange_out_of_its_slot_is_not_seated():
    assert M.score(scene(origin=seated_at(dy=-0.0009), gap=0.0005)) == 0.2
    assert M.score(scene(origin=seated_at(dy=0.0007), gap=0.0005)) == 1.0


def test_seat_limits():
    assert M.score(scene(origin=seated_at(dx=0.0009), gap=0.0005)) == 1.0
    assert M.score(scene(origin=seated_at(dx=0.0015), gap=0.0005)) == 0.2
    assert M.score(scene(origin=seated_at(dx=0.004))) == 0.0  # off the cradle
    assert M.score(scene(origin=seated_at(dz=0.0009), gap=0.0005)) == 1.0
    assert M.score(scene(origin=seated_at(dz=0.0012), gap=0.0005)) == 0.2
    assert M.score(scene(rot=_rx(1.4), gap=0.0005)) == 1.0
    assert M.score(scene(rot=_rx(1.6), gap=0.0005)) == 0.2
    assert M.score(scene(origin=seated_at(dz=0.02))) == 0.0  # held above the cradle


def test_layout():
    assert M.SLIT_W - M.FLANGE_T == pytest.approx(1.0)  # 0.5 mm a side
    flange_y = M.BARREL_SEAT_Y + M._fy
    assert flange_y == pytest.approx(M.SLIT_Y)  # seated, the flange is centred in its slot
    assert M.FLANGE_H / 2 - M.BARREL_R < M.SLIT_DEPTH  # the flange's lower half fits in the slot
    assert M.CRADLE_CLEAR == pytest.approx(1.0)
    assert M.DRIVER_FACE_Y - M.THUMB_FACE_Y == pytest.approx(M.DRIVER_HOME_GAP)
    assert M.DRIVER_TRAVEL * 1000 > M.DRIVER_HOME_GAP
    assert M.FUNNEL_R >= 5.0  # catches the 2 to 5 mm the arm wanders on the way down
    # the stopper stays inside the barrel and the rod clears the flange's bore
    assert M.STOPPER_AT + M.STOPPER_L / 2 < M.BARREL_L / 2 and 8.0 < 2 * (M.BARREL_R - 1.5)
    assert 0.0 < (M.BARREL_R - 1.5) - M.STOPPER_SQ / math.sqrt(2) <= 0.15  # the stopper's play in the bore


def test_setup_text_matches_the_layout():
    setup = json.loads(Path(__file__).with_name("task.json").read_text())["setup"]

    def cm(mm_from_pump):  # a y position along the pump, in cm from the bases' midpoint, as the setup writes it
        return f"{round(PY * 100 + mm_from_pump / 10, 3):g}"

    _, (_, by, _), (_, bl, _) = M.PUMP_BASE[0]
    for words in (f"centred at x {M.PUMP_XY[0] * 100:g}", f"from y {cm(by - bl / 2)} to {cm(by + bl / 2)}",
                  f"centred at y {cm(M.SLIT_Y)}", f"face at y {cm(M.DRIVER_FACE_Y)}, {M.DRIVER_HOME_GAP / 10:g} cm",
                  f"at x {M.STAND_XY[0] * 100:g}, its barrel's centre at y {M.STAND_XY[1] * 100:g}"):
        assert words in setup, words


def test_demo_reaches_and_fits_the_run_limit():
    demo = M.demo()
    for arm in ("left", "right"):
        plan = ArmPlan(demo[arm], station.BASES[arm])
        for t in np.arange(0.0, plan.duration, 0.05):
            plan.action7(t)
        assert plan.worst_ik_mm < 2.0 and plan.worst_ik_deg < 2.0, arm
        assert plan.duration < 1200 / 20 * 0.6
    # aims 0.8 mm past touching: even with no lag the plunger moves only 0.8 mm, inside the 1 mm allowance
    assert -M.SLIDE * 1000 - M.DRIVER_HOME_GAP == pytest.approx(0.8)
    assert -M.SLIDE * 1000 - M.DRIVER_HOME_GAP < M.PUSHED * 1000
