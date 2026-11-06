"""Offline checks (no GPU): the task file loads, the spool's colliders leave its bore open round the rod, the scorer
covers every outcome (the let-go check included), the demo's grips fit the spool and stay within reach, and the
spool's mesh fits its box. Run in the yam_arena environment:  python -m pytest -q test_spool_onto_holder.py"""

from __future__ import annotations

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import FINGER_OPEN, ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "spool_onto_holder.py")
T = TASK.module

FLAT = (0.0, 0.0, 0.0, 1.0)
STANDING = (0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4))  # axis along x
SEATED_C = (T.STOP_X - T.SPOOL_W / 2, 0.0, T.ROD_Z - (T.BORE_R - T.ROD_R))  # hanging on the rod against the stop
ON_ROD_C = (T.STOP_X - T.SPOOL_W / 2 - 0.03, 0.0, T.ROD_Z - (T.BORE_R - T.ROD_R))  # 3 cm short of the stop


def _tilt(deg: float, about=(0.0, 0.0, 1.0)):
    """The standing spool turned ``deg`` about ``about``."""
    a = np.asarray(about, float) / np.linalg.norm(about)
    h = math.radians(deg) / 2
    q1 = np.array([*(a * math.sin(h)), math.cos(h)])
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = STANDING
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def _spool(pos, quat=STANDING, vel=(0.0, 0.0, 0.0)):
    pos = np.asarray(pos, float)
    return {"spool": ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float),
                                 ang_vel=np.zeros(3), low=pos - 0.1, high=pos + 0.1)}


@pytest.fixture(autouse=True)
def fresh_episode():
    T.reset_memory()


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"spool"}


# -- the spool and the holder ----------------------------------------------------------------------------------------


def test_bore_is_open_round_the_rod():
    """Every collider's inner face is at least the bore's radius from the axis: the bore is a 24-sided hole whose
    flats are 26 mm from the axis, 3.5 mm clear of the rod."""
    for name, (x, y, _), turn, (radial, tangential, _) in T.SPOOL.parts():
        inner = math.hypot(x, y) - radial / 2
        assert inner >= T.BORE_R - 1e-9, name
        assert abs(math.atan2(y, x) - turn) < 1e-9 or abs(abs(math.atan2(y, x) - turn) - 2 * math.pi) < 1e-9
    assert T.BORE_R - T.ROD_R == pytest.approx(0.0035)
    assert T.BORE_R / math.cos(math.pi / T.SPOOL.sides) < T.COLLAR_R  # the collar stops the bore's corners too


def test_colliders_stay_inside_the_spool():
    for name, (x, y, z), _, (radial, tangential, axial) in T.SPOOL.parts():
        outer = math.hypot(math.hypot(x, y) + radial / 2, tangential / 2)
        assert outer <= T.SPOOL_R / math.cos(math.pi / T.SPOOL.sides) + 1e-9, name
        assert abs(z) + axial / 2 <= T.SPOOL_W / 2 + 1e-9, name
    gap = [p for p in T.SPOOL.parts() if p[0].startswith("flange")]
    assert len(gap) == 2 * T.SPOOL.sides  # between the two flanges, the gap above the filament is open


def test_holder_layout():
    assert T.ROD_Z - T.TOP == pytest.approx(0.25)
    assert T.STOP_X - T.ROD_TIP_X > T.SPOOL_W + 0.03  # the rod is longer than the spool is wide
    assert T.STOP_X - T.ROD_TIP_X == pytest.approx(0.11)
    assert T.ROD_Z - T.SPOOL_R - (T.BORE_R - T.ROD_R) > T.TOP + 0.1  # a seated spool hangs clear of the table
    assert T.FOOT.pos[0] - T.FOOT_SIZE[0] / 2 > T.SPOOL_XY[0] + T.SPOOL_R + 0.05  # the spool lies clear of the foot


# -- scoring ---------------------------------------------------------------------------------------------------------


def test_start_scores_zero():
    objects = _spool(T.SPOOL.pos, FLAT)
    assert T.score(objects) == 0.0 and not T.success(objects)


def test_seated_and_let_go_succeeds():
    objects = _spool(SEATED_C)
    assert T.seated(objects) and T.success(objects) and T.score(objects) == 1.0


def test_seated_but_still_held_half():
    T.STATE["held"] = True
    objects = _spool(SEATED_C)
    assert not T.success(objects) and T.score(objects) == 0.5


def test_seated_but_moving_half():
    objects = _spool(SEATED_C, vel=(0.05, 0.0, 0.0))
    assert not T.success(objects) and T.score(objects) == 0.5


def test_on_rod_short_of_the_stop_half():
    objects = _spool(ON_ROD_C)
    assert T.on_rod(objects) and not T.seated(objects) and T.score(objects) == 0.5
    near = _spool((SEATED_C[0] - 0.009, *SEATED_C[1:]))
    assert T.score(near) == 1.0  # 9 mm from the stop still counts


def test_either_flange_toward_the_post():
    flipped = (0.0, -math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4))
    assert T.score(_spool(SEATED_C, flipped)) == 1.0


def test_tilted_on_the_rod():
    assert T.score(_spool(ON_ROD_C, _tilt(8))) == 0.5
    assert not T.on_rod(_spool(ON_ROD_C, _tilt(12)))
    assert not T.on_rod(_spool(ON_ROD_C, _tilt(12, (0.0, 1.0, 0.0))))


def test_only_partly_on_the_rod():
    """Its near face short of the rod's tip: not on the rod, but at the tip."""
    half_on = (T.ROD_TIP_X + 0.02, 0.0, T.ROD_Z)
    objects = _spool(half_on)
    assert not T.on_rod(objects) and T.score(objects) == 0.25


def test_held_up_at_the_tip_quarter():
    for c in ((T.ROD_TIP_X - 0.045, 0.0, T.ROD_Z), (T.ROD_TIP_X - 0.03, 0.02, T.ROD_Z + 0.02)):
        assert T.score(_spool(c)) == 0.25
    assert T.score(_spool((T.ROD_TIP_X - 0.06, 0.0, T.ROD_Z))) == 0.0
    assert T.score(_spool((T.ROD_TIP_X - 0.03, 0.0, T.ROD_Z), FLAT)) == 0.25  # lying flat, at the tip


def test_beside_the_rod_zero():
    """Leaning on the rod from the side or hanging below it: the rod is not in the bore."""
    assert T.score(_spool((0.38, 0.0, T.ROD_Z - 0.12))) == 0.0
    assert T.score(_spool((0.38, 0.12, T.ROD_Z))) == 0.0


def test_standing_on_its_box_zero():
    assert T.score(_spool((T.SPOOL_XY[0], 0.0, T.BOX_TOP + T.SPOOL_R))) == 0.0


def test_off_the_table_zero():
    assert T.score(_spool((0.3, 0.6, T.TOP - 0.4), FLAT)) == 0.0


def test_non_finite_zero():
    assert T.score(_spool((math.nan, 0.0, T.ROD_Z))) == 0.0


def test_fingers_on_the_spool():
    objects = _spool(SEATED_C)
    c = np.asarray(SEATED_C)
    on_rim = c + (0.0, T.SPOOL_R + 0.003, 0.0)  # a fingertip 3 mm off the rim
    on_face = c + (-T.SPOOL_W / 2 - 0.004, 0.08, 0.0)  # a pad 4 mm off a flange's face
    away = c + (0.0, T.SPOOL_R + 0.02, 0.0)
    assert T.fingers_on(objects, np.array([on_rim]))
    assert T.fingers_on(objects, np.array([on_face]))
    assert not T.fingers_on(objects, np.array([away]))
    assert not T.fingers_on(objects, None)


# -- the demo --------------------------------------------------------------------------------------------------------


def _at(steps, label):
    t = 0.0
    for w in steps:
        t += w.duration
        if w.label == label:
            return t, w
    raise KeyError(label)


def test_demo_grips_fit_the_spool():
    tips = T.TCP_Y - 0.013  # the pad tips' distance from the axis
    assert T.CORE_R + 0.008 < tips < T.SPOOL_R - 0.02  # past the rim, short of the filament
    assert T.CARTON_SIZE[1] / 2 < tips - 0.015  # the rim overhangs the box: the lower finger fits under it
    assert -FINGER_OPEN > T.SPOOL_W / 2 + 0.008  # the open fingers clear both flanges
    assert T.GRIP_Z == pytest.approx(T.SPOOL.pos[2])  # the grips' line runs through the spool's centre


def test_demo_clears_the_box_and_the_rod():
    """Turning, the spool sweeps a circle round its centre: clear of the box below and the rod's tip ahead."""
    sweep = math.hypot(T.SPOOL_R, T.SPOOL_W / 2)
    assert T.CARRY_Z - sweep > T.BOX_TOP + 0.01
    assert T.SPOOL_XY[0] + sweep < T.ROD_TIP_X - 0.01
    assert T.START_X + T.SPOOL_W / 2 < T.ROD_TIP_X - 0.03  # stood up, it rises to the rod's height short of the tip
    assert T.SEAT_X + T.SPOOL_W / 2 > T.STOP_X  # pushed past the stop: the arms lean it on the collar


def test_demo_order():
    for arm in ("left", "right"):
        plan = T.demo()[arm]
        assert (_at(plan, "take it across its width")[0] < _at(plan, "roll it upright")[0]
                < _at(plan, "up to the rod")[0] < _at(plan, "slide it on to the stop")[0] < _at(plan, "let go")[0])
    left, right = T.demo()["left"], T.demo()["right"]
    for label in ("take it across its width", "roll it upright", "let go"):
        assert _at(left, label)[0] == pytest.approx(_at(right, label)[0])  # the hands work together


def test_demo_keeps_the_hands_apart():
    plan = T.demo()
    assert min(w.pos[1] for w in plan["left"][3:]) > 0.085  # the TCPs 17 cm apart or more
    assert max(w.pos[1] for w in plan["right"][3:]) < -0.085


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    for k in range(int(plan.duration * 20) + 1):
        plan.action7(k / 20)
    assert plan.worst_ik_mm < 2.0 and plan.worst_ik_deg < 2.0, (arm, plan.worst_ik_mm, plan.worst_ik_deg)


# -- the meshes ------------------------------------------------------------------------------------------------------


def test_every_dressed_box_exists_and_its_mesh_is_there():
    names = {o.name for o in T.OBJECTS}
    for name, file in T.LOOKS.items():
        assert name in names
        assert (HERE / "looks" / file).is_file()


def test_spool_mesh_fits_its_box():
    """The spool's mesh stays inside its box (the scorer's centre is the box's) and fills it."""
    pytest.importorskip("pxr.Usd")
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(HERE / "looks" / T.LOOKS["spool"]))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    box = cache.ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
    half = np.asarray(T.SPOOL.size) / 2
    assert np.all(np.asarray(box.GetMax()) <= half + 1e-5)
    assert np.all(np.asarray(box.GetMin()) >= -half - 1e-5)
    assert np.all(np.asarray(box.GetMax()) - np.asarray(box.GetMin()) >= 2 * half - 0.002)
