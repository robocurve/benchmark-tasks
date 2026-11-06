"""Offline checks (no GPU): the task file loads, the profile's channel and the T-nut fit as the lock needs, the scorer
covers each outcome, and the demo turns the nut in the channel and stays within reach. Run in the yam_arena
environment:  python -m pytest -q test_t_nut_quarter_turn.py"""

from __future__ import annotations

import math

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = __import__("pathlib").Path(__file__).resolve().parent
TASK = load(HERE / "t_nut_quarter_turn.py")
T = TASK.module


def _quat(yaw=0.0, tilt=0.0):
    """x y z w of a turn ``yaw`` about z after a lean ``tilt`` about x."""
    qz = np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
    qx = np.array([math.sin(tilt / 2), 0.0, 0.0, math.cos(tilt / 2)])
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def _nut(y, yaw, nut_z=None, x=None, vel=(0.0, 0.0, 0.0), tilt=0.0):
    """The T-nut's state with its nut's centre at (x, y, nut_z): by default in the channel, on its floor."""
    nut_z = T.FLOOR + T.NUT_SIZE[2] / 2 if nut_z is None else nut_z
    x = T.PROFILE_X if x is None else x
    q = _quat(yaw, tilt)
    centre = np.array([x, y, nut_z]) + T._rot(q) @ np.array([0.0, 0.0, T.NUT_BELOW])
    return {"tnut": ObjectState(pos=centre, quat_xyzw=q, lin_vel=np.asarray(vel, float), ang_vel=np.zeros(3),
                                low=centre - 0.03, high=centre + 0.03)}


START = {"tnut": ObjectState(pos=np.asarray(T.TNUT.pos), quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                             lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3))}
CROSS, ALONG = 0.0, math.pi / 2


@pytest.fixture(autouse=True)
def fresh_episode():
    T.reset_memory()


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"tnut"}


def test_the_nut_goes_in_lengthwise_only():
    nut_l, nut_w, nut_t = T.NUT_SIZE
    assert nut_w + 2 * 0.002 <= T.SLOT_OPEN  # 2 mm of play each side, lengthwise through the opening
    assert nut_l >= T.SLOT_OPEN + 2 * 0.004  # crosswise, 4 mm under each lip
    assert math.hypot(nut_l, nut_w) + 2 * 0.0015 <= T.SLOT_IN  # turns in the channel with room to spare
    assert nut_t + 0.002 <= T.CHANNEL_H  # and slides under the lips
    assert math.hypot(T.STUD_W, T.STUD_W) + 2 * 0.001 <= T.SLOT_OPEN  # the stud turns in the opening


def test_profile_parts():
    lips = [b for b in T.PROFILE_PARTS if b.name.startswith("lip_")]
    walls = [b for b in T.PROFILE_PARTS if b.name.startswith("wall_")]
    caps = [b for b in T.PROFILE_PARTS if b.name.startswith("cap_")]
    assert len(lips) == len(walls) == len(caps) == 2
    for lip in lips:
        assert abs(lip.pos[0] - T.PROFILE_X) - lip.size[0] / 2 == pytest.approx(T.SLOT_OPEN / 2)
        assert lip.pos[2] + lip.size[2] / 2 == pytest.approx(T.PROFILE_TOP)
        assert lip.pos[2] - lip.size[2] / 2 == pytest.approx(T.LIP_BOTTOM)
    for wall in walls:
        assert abs(wall.pos[0] - T.PROFILE_X) - wall.size[0] / 2 == pytest.approx(T.SLOT_IN / 2)
        assert wall.pos[2] - wall.size[2] / 2 == pytest.approx(T.FLOOR)
    for cap in caps:  # the caps close the channel's ends
        assert cap.size[0] == T.PROFILE_W and cap.pos[2] + cap.size[2] / 2 == pytest.approx(T.PROFILE_TOP)
    assert T.PROFILE.pos[2] + T.PROFILE.size[2] / 2 == pytest.approx(T.FLOOR)
    assert T.PROFILE.pos[2] - T.PROFILE.size[2] / 2 == pytest.approx(T.MAT_TOP)
    for box in (T.PROFILE, *T.PROFILE_PARTS, T.MAT):
        assert not box.free


def test_the_tnut_parts():
    parts = dict((name, (c, s)) for name, c, s in T.TNUT.parts)
    (nc, ns), (sc, ss) = parts["nut"], parts["stud"]
    assert ns == T.NUT_SIZE and nc[2] - ns[2] / 2 == pytest.approx(-T.TNUT_SIZE[2] / 2)  # the nut at the bottom
    assert sc[2] + ss[2] / 2 == pytest.approx(T.TNUT_SIZE[2] / 2)  # the stud to the top
    assert sc[2] - ss[2] / 2 == pytest.approx(nc[2] + ns[2] / 2)  # standing on the nut
    assert T.TNUT.pos[2] - T.TNUT_SIZE[2] / 2 == pytest.approx(T.MAT_TOP)


def test_start_scores_zero():
    assert not T.in_slot(START)
    assert T.score(START) == 0.0 and not T.success(START)


def test_in_channel_lengthwise_quarter():
    objects = _nut(T.MARK_Y, ALONG)
    assert T.in_slot(objects) and not T.locked(objects)
    assert T.score(objects) == 0.25


def test_half_turned_is_not_locked():
    assert T.score(_nut(T.MARK_Y, math.radians(45))) == 0.25
    assert T.score(_nut(T.MARK_Y, math.radians(11))) == 0.25


def test_locked_away_from_the_mark_half():
    assert T.score(_nut(-0.04, CROSS)) == 0.5
    assert T.score(_nut(T.MARK_Y + 0.011, CROSS)) == 0.5


def test_locked_at_the_mark_succeeds():
    for yaw in (CROSS, math.pi, math.radians(9), math.radians(-9), math.radians(171)):
        objects = _nut(T.MARK_Y + 0.009, yaw)
        assert T.locked(objects) and T.success(objects) and T.score(objects) == 1.0, yaw
    assert T.score(_nut(T.MARK_Y - 0.009, CROSS, nut_z=T.FLOOR + 0.0045)) == 1.0  # raised under the lips


def test_still_held_at_the_mark_half():
    objects = _nut(T.MARK_Y, CROSS)
    finger = np.asarray(objects["tnut"].pos) + np.array([0.0, T.TNUT_SIZE[1] / 2 + 0.002, 0.01])
    assert T.touching(finger[None, :], objects)
    T.STATE["held"] = True
    assert not T.success(objects) and T.score(objects) == 0.5
    T.reset_memory()
    assert T.score(objects) == 1.0


def test_touching_needs_a_finger_near():
    objects = _nut(T.MARK_Y, CROSS)
    far = np.asarray(objects["tnut"].pos) + np.array([0.0, T.TNUT_SIZE[1] / 2 + 0.004, 0.0])
    assert not T.touching(far[None, :], objects)
    assert not T.touching(None, objects)


def test_moving_at_the_mark_half():
    assert T.score(_nut(T.MARK_Y, CROSS, vel=(0.0, 0.05, 0.0))) == 0.5


def test_on_top_of_the_profile_zero():
    on_top = _nut(T.MARK_Y, CROSS, nut_z=T.PROFILE_TOP + T.NUT_SIZE[2] / 2)
    assert not T.in_slot(on_top) and T.score(on_top) == 0.0


def test_beside_the_slot_zero():
    assert T.score(_nut(T.MARK_Y, CROSS, x=T.PROFILE_X + 0.03, nut_z=T.MAT_TOP + 0.003)) == 0.0


def test_beyond_the_caps_zero():
    assert T.score(_nut(T.PROFILE_Y0 - 0.02, CROSS)) == 0.0


def test_leaning_zero():
    assert T.score(_nut(T.MARK_Y, CROSS, tilt=math.radians(30))) == 0.0


def test_off_table_zero():
    objects = _nut(0.0, CROSS, nut_z=T.TOP - 0.5, x=0.3)
    assert T.score(objects) == 0.0 and not T.success(objects)


def test_non_finite_zero():
    objects = _nut(T.MARK_Y, CROSS)
    objects["tnut"] = ObjectState(pos=np.array([math.nan, 0, 0]), quat_xyzw=np.array([0, 0, 0, 1.0]),
                                  lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3))
    assert T.score(objects) == 0.0


def test_cross_error():
    for deg, err in ((0, 0), (90, 90), (180, 0), (-170, 10), (100, 80)):
        assert T.cross_error_deg(_nut(0.0, math.radians(deg))) == pytest.approx(err, abs=1e-6)


# -- the demo ---------------------------------------------------------------------------------------------------------


def _at(steps, label):
    for w in steps:
        if w.label == label:
            return w
    raise KeyError(label)


def test_demo_turns_in_the_channel():
    plan = T.demo()["right"]
    lower, turn, slide = _at(plan, "lower into the channel"), _at(plan, "quarter turn"), _at(plan, "slide to the mark")
    assert lower.yaw == T.ALONG and turn.yaw == T.CROSS == slide.yaw
    assert lower.pos == turn.pos and turn.pos[2] == slide.pos[2] and slide.pos[1] == T.MARK_Y
    assert _at(plan, "close").yaw == T.CROSS  # picked up crosswise, as it stands


def test_demo_heights():
    tips = T.SLOT_Z - T.PAD_TIP_PAST_TCP  # the pad tips while the nut is in the channel
    stud_top = T.FLOOR + T.HOVER_NUT + T.TNUT_SIZE[2]
    assert tips == pytest.approx(stud_top - T.GRIP_DEPTH)
    assert tips > T.PROFILE_TOP + 0.012  # the fingers stay well above the profile
    nut_top = T.FLOOR + T.HOVER_NUT + T.NUT_SIZE[2]
    assert nut_top < T.LIP_BOTTOM - 0.001  # the nut slides under the lips
    grip_tips = T.GRIP_Z - T.PAD_TIP_PAST_TCP
    assert grip_tips > T.MAT_TOP + T.NUT_SIZE[2] + 0.02  # the fingers close on the stud, clear of the nut
    nut_bottom = T.TRAVEL_Z - T.PAD_TIP_PAST_TCP + T.GRIP_DEPTH - T.TNUT_SIZE[2]
    assert nut_bottom > T.PROFILE_TOP + 0.02  # carried over the profile


def test_demo_keeps_to_the_right_half():
    plan = T.demo()
    assert set(plan) == {"right"}
    assert max(w.pos[1] for w in plan["right"]) < 0.0


@pytest.mark.parametrize("arm", ["right"])
def test_demo_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    for k in range(int(plan.duration * 20) + 1):
        plan.action7(k / 20)
    assert plan.worst_ik_mm < 2.0 and plan.worst_ik_deg < 2.0, (arm, plan.worst_ik_mm, plan.worst_ik_deg)



def test_ablations_change_only_the_turn_and_the_end(monkeypatch):
    """TURN = 0 keeps the nut lengthwise in the channel. PULL = 1 pulls straight up after the turn (the lips' check)."""
    monkeypatch.setattr(T, "TURN", 0.0)
    plan = T.demo()["right"]
    assert _at(plan, "quarter turn").yaw == T.ALONG == _at(plan, "slide to the mark").yaw
    monkeypatch.setattr(T, "TURN", 1.0)
    monkeypatch.setattr(T, "PULL", 1.0)
    plan = T.demo()["right"]
    pull = _at(plan, "pull straight up")
    assert pull.pos[:2] == _at(plan, "quarter turn").pos[:2] and pull.pos[2] == T.TRAVEL_Z
    assert pull.finger == T.FINGER_CLOSED
    arm = ArmPlan(plan, BASES["right"])
    for k in range(int(arm.duration * 20) + 1):
        arm.action7(k / 20)
    assert arm.worst_ik_mm < 2.0 and arm.worst_ik_deg < 2.0


# -- the meshes --------------------------------------------------------------------------------------------------------


def _bounds(file):
    pytest.importorskip("pxr.Usd")
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(HERE / "looks" / file))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    box = cache.ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
    return np.asarray(box.GetMin()), np.asarray(box.GetMax())


def test_every_dressed_box_exists_and_its_mesh_is_there():
    names = {o.name for o in T.OBJECTS}
    for name, file in T.LOOKS.items():
        assert name in names
        assert (HERE / "looks" / file).is_file()


def test_the_tnut_mesh_fits_its_box():
    """The T-nut's mesh stays inside its box and fills it, so what the cameras see is what the scorer measures."""
    lo, hi = _bounds(T.LOOKS["tnut"])
    half = np.asarray(T.TNUT_SIZE) / 2
    assert np.all(hi <= half + 1e-5) and np.all(lo >= -half - 1e-5)
    assert np.all(hi - lo >= 2 * half - 0.001)


def test_the_profile_mesh_matches_its_colliders():
    """The profile's mesh spans the 45 mm section, the profile's length and its end caps, around the core box."""
    lo, hi = _bounds(T.LOOKS["profile"])
    centre = np.asarray(T.PROFILE.pos)
    assert hi[0] - lo[0] == pytest.approx(T.PROFILE_W, abs=0.001)
    assert hi[1] - lo[1] == pytest.approx(T.PROFILE_LEN + 2 * T.CAP_T, abs=0.001)
    assert lo[2] + centre[2] == pytest.approx(T.MAT_TOP, abs=0.001)
    assert hi[2] + centre[2] == pytest.approx(T.PROFILE_TOP, abs=0.001)  # the tape adds 0.3 mm
