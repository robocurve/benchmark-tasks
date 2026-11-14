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


def _rz(deg):
    t = math.radians(deg)
    return np.array([[math.cos(t), -math.sin(t), 0], [math.sin(t), math.cos(t), 0], [0, 0, 1]])


def part(centre_local, origin, rot, speed=0.0, spin=0.0):
    """A part's state from its frame: pos is its bounding box's centre, as Arena reports it."""
    pos = np.asarray(origin, float) + rot @ centre_local
    return ObjectState(pos=pos, quat_xyzw=_quat(rot), lin_vel=np.array([speed, 0, 0]), ang_vel=np.array([0, 0, spin]),
                       low=pos - 0.02, high=pos + 0.02)


def scene(origin=None, rot=None, pushed=0.0, gap=None, moving=False, sag_deg=0.0, speed=None, spin=0.0,
          stopper_off=(0.0, 0.0), plunger_at=None):
    """The syringe's barrel centre at ``origin`` (seated by default), its plunger pushed in by ``pushed`` (m), sagging
    ``sag_deg`` in the bore and its stopper ``stopper_off`` (x, z, m) off the axis, and the driver's face ``gap`` (m)
    from the thumb disc's outermost point (at its start if None). ``plunger_at`` puts the plunger's stopper there
    instead, upright along y, wherever that is."""
    origin = np.array(M.SEAT if origin is None else origin, float)
    rot = np.eye(3) if rot is None else rot
    axis = rot @ np.array([0.0, 1.0, 0.0])
    p_rot = rot @ _rx(sag_deg)
    p_origin = origin + axis * (M.STOPPER_AT / 1000 - pushed) + rot @ np.array([stopper_off[0], 0.0, stopper_off[1]])
    if plunger_at is not None:
        p_origin, p_rot = np.asarray(plunger_at, float), np.eye(3)
    speed = (0.05 if moving else 0.0) if speed is None else speed
    centre = p_origin + p_rot @ np.array([0.0, M.THUMB_FROM_STOPPER / 1000, 0.0])
    thumb_y = centre[1] + M.THUMB_HALF * (abs(p_rot[1, 0]) + abs(p_rot[1, 2]))
    face_home = PY + M.DRIVER_FACE_Y / 1000
    slide = 0.0 if gap is None else (thumb_y + gap) - face_home
    pump = MechanismState(pos=np.array([*M.PUMP_XY, M.TOP]), quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                          lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=np.zeros(3), high=np.ones(3),
                          joints={"driver_slide": slide})
    return {"syringe": part(M._SC, origin, rot, speed, spin), "plunger": part(M._PC, p_origin, p_rot, speed),
            "pump": pump}


def sc(s):
    """The score of one state on its own, as at the end of a fresh episode (the recorded plunger travel and driver
    position cleared)."""
    M._clear_memory()
    return M.score(s)


def on_stands():
    return scene(origin=(M.STAND_XY[0], M.STAND_XY[1], M.START_Z))


def seated_at(dx=0.0, dy=0.0, dz=0.0):
    return M.SEAT + np.array([dx, dy, dz])


def test_the_start_scores_zero():
    s = on_stands()
    assert sc(s) == 0.0 and not M.success(s)


def test_seated_with_the_driver_flush_is_success():
    s = scene(gap=0.0005)
    # the complete state: success() is true, but score() stays below 1.0, so 1.0 needs success() held 1 s
    assert sc(s) == 0.9 < 1.0 and M.success(s)
    assert M.SUCCESS_HOLD_S == 1.0


def test_credit_climbs_with_each_step():
    assert sc(scene(origin=seated_at(dz=0.006), rot=_rx(8))) == 0.2  # cocked on the body: the PMDA failure
    assert sc(scene()) == 0.5  # seated, driver still at its start
    assert sc(scene(gap=0.004)) == 0.75
    assert sc(scene(gap=0.0009)) == 0.9


def test_driver_limits():
    assert sc(scene(gap=0.0011)) == 0.75
    assert sc(scene(gap=0.0049)) == 0.75
    assert sc(scene(gap=0.0051)) == 0.5
    assert sc(scene(gap=-0.0001)) == 0.9  # touching: the contact's overlap
    assert sc(scene(gap=-0.0003)) == 0.5  # the face past the disc: the plunger isn't where the driver is
    assert sc(scene(gap=-0.0009)) == 0.5
    assert sc(scene(gap=-0.010)) == 0.5


def test_driving_past_a_moved_plunger_never_scores_full():
    # the review's case: draw the plunger out (or back) and run the driver beyond where its thumb disc was
    out = (M.SEAT[0], M.SEAT[1] + (M.STOPPER_AT + 29.5) / 1000, M.SEAT[2])
    for g in (0.0, -0.005, -0.030):
        assert sc(scene(plunger_at=out, gap=g)) == 0.0
    M._clear_memory()
    assert M.score(scene(pushed=-0.004, gap=0.0005)) == 0.0  # drawn back 4 mm
    assert M.score(scene(pushed=0.0, gap=0.0005)) == 0.0  # and put back: still 0, the travel was recorded
    assert not M.success(scene(pushed=0.0, gap=0.0005))


def test_push_then_retract_scores_zero_for_the_rest_of_the_trial():
    M._clear_memory()
    assert M.score(scene(pushed=0.0, gap=0.003)) == 0.75
    assert M.score(scene(pushed=0.0012, gap=0.0)) == 0.0  # pushed 1.2 mm
    for _ in range(5):  # pulled back to the full mark and left flush: zero from then on
        s = scene(pushed=0.0, gap=0.0005)
        assert M.score(s) == 0.0 and not M.success(s)


def test_a_moving_driver_is_not_still():
    # a joint reports no velocity: motion is the driver's change between control steps
    M._clear_memory()
    assert M.score(scene(gap=0.003)) == 0.75
    s = scene(gap=0.0005)  # 2.5 mm in one step: 50 mm/s at 20 Hz
    assert not M.success(s) and M.score(s) == 0.75  # success() then score() on one step's states: counted once
    assert M.score(scene(gap=0.0005)) == 0.9  # the next step, stopped
    M._clear_memory()
    M.score(scene(gap=0.0009))
    assert M.score(scene(gap=0.0005)) == 0.9  # 0.4 mm in a step is still


def test_everything_must_be_still():
    assert sc(scene(gap=0.0005, speed=0.0095)) == 0.9
    assert sc(scene(gap=0.0005, speed=0.0105)) == 0.75
    assert sc(scene(gap=0.0005, moving=True)) == 0.75
    assert sc(scene(gap=0.0005, spin=0.45)) == 0.9
    assert sc(scene(gap=0.0005, spin=0.55)) == 0.75


def test_a_pushed_plunger_scores_zero_whatever_else():
    assert sc(scene(pushed=0.0011, gap=0.0)) == 0.0
    assert sc(scene(pushed=0.0011)) == 0.0
    assert sc(scene(pushed=0.0009, gap=0.0005)) == 0.9
    assert sc(scene(pushed=-0.0029, gap=0.0005)) == 0.9  # drawn back a little
    assert sc(scene(pushed=-0.0031, gap=0.0005)) == 0.0  # drawn back more than 3 mm


def test_a_push_or_a_pull_counts_even_if_undone():
    M._clear_memory()
    assert M.score(scene(pushed=0.005, gap=0.0)) == 0.0
    assert M.score(scene(pushed=0.0, gap=0.0005)) == 0.0  # pulled back to where it was: the dose was given
    M._clear_memory()
    assert M.score(scene(pushed=-0.010, gap=0.0005)) == 0.0
    assert M.score(scene(pushed=0.0, gap=0.0005)) == 0.0  # pushed back after drawing it out
    M._clear_memory()  # what the reset does
    assert M.score(scene(pushed=0.0, gap=0.0005)) == 0.9
    import mechanism
    assert M._clear_memory in mechanism._RESET  # and the reset does run it


def test_the_plunger_must_be_in_its_barrel():
    # seat the barrel, pull the plunger out and leave it anywhere: never more than 0.2, here 0 (drawn out)
    out = (M.SEAT[0], M.SEAT[1] + (M.STOPPER_AT + 29.5) / 1000, M.SEAT[2])
    assert sc(scene(plunger_at=out)) == 0.0
    assert sc(scene(plunger_at=(M.SEAT[0] + 0.08, M.SEAT[1], M.TOP + 0.015))) == 0.0  # on the table beside the pump
    assert sc(scene(plunger_at=(M.SEAT[0], M.PUMP_XY[1] + 0.20, M.TOP + 0.015))) == 0.0  # dropped past the driver
    assert sc(scene(gap=0.0005, stopper_off=(0.0009, 0.0))) == 0.9
    assert sc(scene(gap=0.0005, stopper_off=(0.0011, 0.0))) == 0.0
    assert sc(scene(gap=0.0005, sag_deg=4.9)) == 0.9
    assert sc(scene(gap=0.0005, sag_deg=5.1)) == 0.0
    # the plunger turned end for end, its quaternion a hair over unit length (float32): 0, not an error
    s = scene(gap=0.0005)
    p = s["plunger"]
    s["plunger"] = ObjectState(pos=p.pos, quat_xyzw=np.array([1.0 + 1e-7, 0.0, 0.0, 0.0]), lin_vel=p.lin_vel,
                               ang_vel=p.ang_vel, low=p.low, high=p.high)
    assert sc(s) == 0.0


def test_a_sagging_plunger_meets_the_driver_at_its_rim():
    # a tilted thumb disc reaches the driver with its rim first: the gap is measured to the rim, not the centre
    assert sc(scene(gap=0.0005, sag_deg=3.0)) == 0.9
    assert sc(scene(gap=0.0015, sag_deg=3.0)) == 0.75


def test_flange_out_of_its_slot_is_not_seated():
    assert sc(scene(origin=seated_at(dy=-0.0009), gap=0.0005)) == 0.2
    assert sc(scene(origin=seated_at(dy=0.0007), gap=0.0005)) == 0.9


def test_seat_limits():
    assert sc(scene(origin=seated_at(dx=0.0009), gap=0.0005)) == 0.9
    assert sc(scene(origin=seated_at(dx=0.0015), gap=0.0005)) == 0.2
    assert sc(scene(origin=seated_at(dz=0.0009), gap=0.0005)) == 0.9
    assert sc(scene(origin=seated_at(dz=0.0012), gap=0.0005)) == 0.2
    assert sc(scene(origin=seated_at(dz=-0.0012), gap=0.0005)) == 0.2
    assert sc(scene(rot=_rx(1.4), gap=0.0005)) == 0.9
    assert sc(scene(rot=_rx(1.6), gap=0.0005)) == 0.2
    assert sc(scene(rot=_rz(1.4), gap=0.0005)) == 0.9
    assert sc(scene(rot=_rz(1.6), gap=0.0005)) == 0.2


def test_in_cradle_limits():
    assert sc(scene(origin=seated_at(dx=0.0029))) == 0.2
    assert sc(scene(origin=seated_at(dx=0.0031))) == 0.0  # off the cradle
    assert sc(scene(origin=seated_at(dz=0.0024))) == 0.2
    assert sc(scene(origin=seated_at(dz=0.0026))) == 0.0  # held level above the rods
    assert sc(scene(origin=seated_at(dz=-0.0024))) == 0.2
    assert sc(scene(origin=seated_at(dz=-0.0026))) == 0.0  # below the rods: impossible, not credited
    assert sc(scene(origin=seated_at(dz=0.02))) == 0.0  # held above the cradle
    assert sc(scene(origin=seated_at(dy=-0.0049))) == 0.2  # flange short of its slot, nose still on the rods
    assert sc(scene(origin=seated_at(dy=-0.0051))) == 0.0  # slid toward the nose, off the rods' end
    assert sc(scene(origin=seated_at(dy=-0.040))) == 0.0
    assert sc(scene(rot=_rz(179.9))) == 0.0  # end for end at the seat
    # turned about the nose end, which stays on the rods: the off-line limit decides
    nose = M.SEAT - np.array([0.0, M.BARREL_L / 2000, 0.0])
    for deg, want in ((14.0, 0.2), (16.0, 0.0)):
        r = _rz(deg)
        origin = nose + r @ np.array([0.0, M.BARREL_L / 2000, 0.0])  # the barrel swung sideways about its nose
        assert sc(scene(origin=origin, rot=r)) == want, deg


def test_layout():
    assert M.SLIT_W - M.FLANGE_T == pytest.approx(1.0)  # 0.5 mm a side
    flange_y = M.BARREL_SEAT_Y + M._fy
    assert flange_y == pytest.approx(M.SLIT_Y)  # seated, the flange is centred in its slot
    assert M.FLANGE_H / 2 - M.BARREL_R < M.SLIT_DEPTH  # the flange's lower half fits in the slot
    # the rods touch the barrel's 45-degree staves on their faces, so the seated axis sits at AXIS_Z
    assert math.hypot(M.ROD_X, M.AXIS_Z - M.ROD_Z) == pytest.approx(M.BARREL_R + M.ROD_R)
    assert M.ROD_Z + M.ROD_R < M.AXIS_Z - 4.0 - 1.0  # the rods' tops stay under the fingertips (4 mm below the axis)
    assert M.BODY_TOP < M.FLOOR_Z  # the barrel rests on the rods, not on the body
    assert M.ROD_Y[1] <= M.SLIT_Y - M.SLIT_W / 2 - M.FUNNEL_R  # the rods stop before the slot's mouth
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
                  f"at x {M.STAND_XY[0] * 100:g}, its barrel's centre at y {M.STAND_XY[1] * 100:g}",
                  f"{M.BODY_TOP / 10:g} cm tall", f"{2 * M.ROD_R / 10:g} cm across",
                  f"{round(2 * M.ROD_X / 10, 2):g} cm apart", f"axis {round(M.AXIS_Z / 10, 2):g} cm up"):
        assert words in setup, words


def test_demo_reaches_and_fits_the_run_limit():
    demo = M.demo()
    for arm in ("left", "right"):
        plan = ArmPlan(demo[arm], station.BASES[arm])
        for t in np.arange(0.0, plan.duration, 0.05):
            plan.action7(t)
        assert plan.worst_ik_mm < 2.0 and plan.worst_ik_deg < 2.0, arm
        assert plan.duration < 1200 / 20 * 0.6
        late = ArmPlan(M.demo(delay=0.10)[arm], station.BASES[arm])
        assert late.duration == pytest.approx(plan.duration + 0.10)
    # aims 1.2 mm past touching a centred flange; the flange rests 0.3 mm short, so even with no lag the plunger moves
    # only 0.9 mm, inside the 1 mm allowance
    assert -M.SLIDE * 1000 - M.DRIVER_HOME_GAP == pytest.approx(1.2)
    assert -M.SLIDE * 1000 - M.DRIVER_HOME_GAP - M.FLANGE_SETTLE < M.PUSHED * 1000
