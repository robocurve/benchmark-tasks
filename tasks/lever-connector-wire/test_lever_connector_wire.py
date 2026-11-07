"""Offline checks (no GPU): the task file loads, the port and the lever's clamp are built as the trap needs, the lever
swings clear of the housing, every scoring outcome, and the demo's order, heights and reach. Run in the yam_arena
environment:  python -m pytest -q test_lever_connector_wire.py"""

from __future__ import annotations

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "lever_connector_wire.py")
T = TASK.module
MM = 0.001


def _ry(degrees: float) -> np.ndarray:
    """x, y, z, w of the lever open ``degrees``: a turn about -y."""
    a = math.radians(degrees) / 2
    return np.array([0.0, -math.sin(a), 0.0, math.cos(a)])


def _state(pos, quat=(0.0, 0.0, 0.0, 1.0), vel=(0.0, 0.0, 0.0)):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float),
                       ang_vel=np.zeros(3), low=pos - 0.01, high=pos + 0.01)


def _wire_at(d: float, dy: float = 0.0, lying: bool = True, vel=(0.0, 0.0, 0.0)):
    """The wire level in the port with its tip ``d`` in: lying on the bar (its insulation on the floor), or held on
    the port's axis."""
    w = T.WIRE_AXIS_W if lying else T.BORE_AXIS_W
    centre = T.at(d - T.WIRE_LEN / 2, dy, w)
    return _state(centre, vel=vel)


def _scene(wire=None, lever=0.0):
    return {
        "wire": wire if wire is not None else _state(T.WIRE.pos),
        "lever": _state(T.LEVER.pos, quat=_ry(lever)),
    }


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"lever", "wire"}


# -- the port, the clamp and the lever --------------------------------------------------------------------------------


def test_port_play():
    """2 mm round the held wire's insulation, each side, and the copper clears the bar on the way in."""
    assert T.BORE_HALF - T.INS_D / 2 == pytest.approx(0.002)
    assert (T.BORE_TOP - T.FLOOR_T) / 2 - T.INS_D / 2 == pytest.approx(0.002)
    assert T.BORE_AXIS_W - T.BARE_D / 2 > T.BAR_TOP + 0.0015
    assert T.BORE_AXIS_W - T.INS_D / 2 > T.BAR_TOP  # an over-pushed level wire slides over the bar to the stop


def test_trap_is_reachable_and_the_band_is_short_of_it():
    assert T.BAND[1] < T.TRAP < T.STOP_U  # a wire pushed until it stops is in the trap
    assert T.BARE_LEN - T.BAND[0] == pytest.approx(0.004)  # at most 4 mm of copper shows
    assert T.BAND[1] - T.BARE_LEN == pytest.approx(0.002)  # at most 2 mm of insulation in
    assert T.TRAP - T.BARE_LEN == pytest.approx(0.004)


def test_lever_sits_just_above_the_housing():
    assert T.HANDLE[0][2] - T.HOUSING_TOP == pytest.approx(0.0005)
    assert T.PIVOT == (T.HANDLE[0][0], 0.0, T.HANDLE[0][2])  # the hinge on the handle's front bottom edge
    assert T.PIVOT[0] == 0.0  # at the port's face


def test_tooth_in_its_window():
    (u0, v0, w0), (u1, v1, w1) = T.TOOTH
    assert -v0 == v1 and T.BORE_HALF - v1 >= 0.001
    assert u0 >= 0.005 and T.CEILING_FROM - u1 >= 0.002
    assert w1 == T.HANDLE[0][2]
    assert T.BAR[0] <= u0 and u1 <= T.BAR[1]  # the clamp comes down on the bar


def _tooth_foot_w(degrees):
    """The lowest point of the tooth's foot with the lever open ``degrees``."""
    a = math.radians(degrees)
    out = []
    for u in (T.TOOTH[0][0], T.TOOTH[1][0]):
        du, dw = u - T.PIVOT[0], T.TOOTH[0][2] - T.PIVOT[2]
        out.append(T.PIVOT[2] + du * math.sin(a) + dw * math.cos(a))
    return min(out)


def _angle_when_foot_at(w):
    return next(a for a in np.arange(0.0, 30.0, 0.01) if _tooth_foot_w(a) >= w)


def test_shut_lever_blocks_the_port():
    gap = T.TOOTH[0][2] - T.BAR_TOP
    assert gap < T.BARE_D  # the bare end can't pass under the shut clamp
    assert T.TOOTH[0][2] < T.BORE_AXIS_W  # nor can a held one, on the port's axis


def test_clamp_on_copper_is_shut_and_on_insulation_is_not():
    copper_top = T.BAR_TOP + T.BARE_D
    assert copper_top > T.TOOTH[0][2]  # the clamp presses on the copper
    assert _angle_when_foot_at(copper_top) < T.SHUT_DEG - 2.0
    insulation_top = T.FLOOR_T + T.INS_D
    assert _angle_when_foot_at(insulation_top) > T.SHUT_DEG


def _lever_points(degrees):
    """Points on the lever's handle and tooth (a 1 mm grid on their faces), open ``degrees``, connector frame."""
    pts = []
    for lo, hi in (T.HANDLE, T.TOOTH):
        axes = [np.linspace(a, b, max(2, int((b - a) / 0.001) + 1)) for a, b in zip(lo, hi)]
        g = np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)
        on_face = np.any([np.isclose(g[:, k], lo[k]) | np.isclose(g[:, k], hi[k]) for k in range(3)], axis=0)
        pts.append(g[on_face])
    p = np.vstack(pts) - np.asarray(T.PIVOT)
    a = math.radians(degrees)
    u = p[:, 0] * math.cos(a) - p[:, 2] * math.sin(a)
    w = p[:, 0] * math.sin(a) + p[:, 2] * math.cos(a)
    return np.stack([u, p[:, 1], w], -1) + np.asarray(T.PIVOT)


def _fixed_boxes():
    """Every fixed box near the connector, as (low, high) corners in the connector's frame."""
    out = []
    for b in (*T.HOUSING, T.FAR_WIRE, T.BOARD):
        c = np.asarray(b.pos) - np.asarray(T.at(0.0, 0.0, 0.0))
        out.append((b.name, c - np.asarray(b.size) / 2, c + np.asarray(b.size) / 2))
    return out


@pytest.mark.parametrize("degrees", np.arange(0.0, 61.0, 2.0))
def test_lever_swings_clear_of_the_housing(degrees):
    pts = _lever_points(degrees)
    for name, lo, hi in _fixed_boxes():
        inside = np.all((pts > lo + 0.0002) & (pts < hi - 0.0002), axis=1)
        assert not inside.any(), (name, degrees, pts[inside][:3])


def test_lever_angle_reads_the_turn():
    for a in (0.0, 5.0, 30.0, 60.0):
        assert T.lever_angle(_state(T.LEVER.pos, quat=_ry(a))) == pytest.approx(a)


def test_spring_presses_at_both_stops():
    shut_torque = T.LEVER_STIFFNESS * (T.SHUT_TARGET - T.LEVER.upper)  # joint degrees: shut is 0, open is -60
    open_torque = T.LEVER_STIFFNESS * (T.LEVER.lower - T.OPEN_TARGET)
    assert shut_torque == pytest.approx(0.024, rel=0.05) and open_torque == pytest.approx(0.024, rel=0.05)
    assert T.LEVER.upper == 0.0 and T.LEVER.lower == -T.OPEN_DEG and 0 < T.SNAP_DEG < T.OPEN_DEG


# -- scoring ---------------------------------------------------------------------------------------------------------


def test_tip_and_depth():
    assert T.depth(_scene()) == 0.0
    s = _scene(wire=_wire_at(0.021))
    assert T.tip(s["wire"])[0] == pytest.approx(T.FACE_X + 0.021)
    assert T.depth(s) == pytest.approx(0.021)
    assert T.depth(_scene(wire=_wire_at(0.021, lying=False))) == pytest.approx(0.021)


def test_start_scores_zero():
    assert T.score(_scene()) == 0.0 and not T.success(_scene())


def test_lever_open_no_wire_zero():
    assert T.score(_scene(lever=60.0)) == 0.0


def test_in_band_lever_shut_full():
    for d in (0.0181, 0.021, 0.0239):
        s = _scene(wire=_wire_at(d), lever=1.5)
        assert T.score(s) == 1.0 and T.success(s), d


def test_in_band_still_moving_not_success():
    s = _scene(wire=_wire_at(0.021, vel=(0.05, 0.0, 0.0)), lever=1.5)
    assert not T.success(s) and T.score(s) == 0.0


def test_in_band_lever_open_half():
    for lever in (6.0, 30.0, 60.0):
        assert T.score(_scene(wire=_wire_at(0.021), lever=lever)) == 0.5


def test_started_lever_open_quarter():
    for d in (0.006, 0.012, 0.0175, 0.025):
        assert T.score(_scene(wire=_wire_at(d), lever=60.0)) == 0.25, d


def test_barely_in_zero():
    assert T.score(_scene(wire=_wire_at(0.004), lever=60.0)) == 0.0


def test_too_deep_is_zero_open_or_shut():
    for d in (0.026, 0.030):
        for lever in (0.0, 3.0, 11.0, 60.0):
            assert T.score(_scene(wire=_wire_at(d), lever=lever)) == 0.0, (d, lever)


def test_shut_on_a_short_wire_zero():
    for d in (0.0, 0.006, 0.012, 0.0175):
        assert T.score(_scene(wire=_wire_at(d), lever=0.5)) == 0.0, d


def test_beside_the_port_is_not_in_it():
    assert T.depth(_scene(wire=_wire_at(0.021, dy=0.012))) == 0.0
    assert T.score(_scene(wire=_wire_at(0.021, dy=0.012), lever=1.0)) == 0.0
    over = _wire_at(0.021)
    over = _state(over.pos + np.array([0.0, 0.0, 0.03]))  # above the connector
    assert T.depth({"wire": over}) == 0.0


def test_wire_off_table_zero():
    gone = _state((0.3, -0.5, T.TOP - 0.3))
    assert T.score(_scene(wire=gone, lever=60.0)) == 0.0


def test_non_finite_zero():
    assert T.score(_scene(wire=_state((math.nan, 0.0, 0.0)))) == 0.0
    assert not T.success(_scene(wire=_state((math.nan, 0.0, 0.0))))


# -- the demo ----------------------------------------------------------------------------------------------------------


def _times(steps):
    t, out = 0.0, {}
    for w in steps:
        t += w.duration
        out[w.label] = (t, w)
    return out


def test_demo_order():
    plan = T.demo()
    assert set(plan) == {"right"}  # the left arm stays at its ready pose
    at = _times(plan["right"])
    assert at["pinch"][0] < at["let the lever go"][0] < at["grip the wire"][0] < at["push in"][0]
    assert at["push in"][0] < at["let the wire go"][0] < at["pinch it again"][0] < at["let the lever shut"][0]
    assert at["let the lever go"][1].finger < -0.01 and at["let the lever shut"][1].finger < -0.01
    assert at["pinch"][1].finger == at["pinch it again"][1].finger == T.PINCH


def test_demo_pinch_is_light():
    half = T.LEVER.size[1] / 2
    assert -half < T.PINCH and half + T.PINCH < 0.002  # the pads are sent at most 2 mm inside the lever's sides


def test_demo_lets_go_past_the_snap():
    assert T.SNAP_DEG + 10 < T.SWING[-1] < T.OPEN_DEG
    assert T.SWING_BACK[-1] < T.SHUT_DEG and T.SWING_BACK[0] < T.OPEN_DEG


def test_demo_heights():
    tips = lambda z: z - T.PAD_TIP_PAST_TCP  # noqa: E731
    plan = _times(T.demo()["right"])
    pinch = plan["pinch"][1].pos
    assert tips(pinch[2]) - (T.BASE_TOP + T.HOUSING_TOP) == pytest.approx(0.0025)  # clear of the housing's top
    assert tips(pinch[2]) < T.BASE_TOP + T.HANDLE[1][2] - 0.005  # 6 mm of the lever's side between the pads
    grip = plan["grip the wire"][1].pos
    assert tips(grip[2]) - T.TOP == pytest.approx(T.GRIP_TIPS)
    held_axis = plan["push in"][1].pos[2] - T.WIRE_BELOW_TCP
    assert held_axis == pytest.approx(T.BASE_TOP + T.BORE_AXIS_W)
    assert plan["round the open lever"][1].pos == pytest.approx(T.swing_point(T.OPEN_DEG))
    assert math.degrees(plan["round the open lever"][1].pitch) == pytest.approx(T.OPEN_DEG)


def test_demo_pushes_into_the_band():
    plan = _times(T.demo()["right"])
    tcp_x = plan["push in"][1].pos[0]
    assert tcp_x + T.GRIP_BACK - T.FACE_X == pytest.approx(T.PUSH_TO)
    assert T.BAND[0] + 0.002 < T.PUSH_TO < T.BAND[1] - 0.002
    lined = plan["line up"][1].pos[0] + T.GRIP_BACK - T.FACE_X
    assert lined < 0  # the tip lines up in front of the face, then goes in straight
    assert plan["line up"][1].pos[1:] == plan["push in"][1].pos[1:]


def test_demo_fingers_clear_the_face():
    """The pads reach 1.8 cm ahead of the TCP: pushed in, they stay 2 cm in front of the connector."""
    plan = _times(T.demo()["right"])
    pads_front = plan["push in"][1].pos[0] + 0.018
    assert T.FACE_X - pads_front > 0.015


def test_demo_wrist_turns_with_the_lever():
    for step in T.demo()["right"]:
        if step.label.startswith("swing"):
            assert math.degrees(step.pitch) == pytest.approx(float(step.label.split()[-2]))
            assert step.pos == pytest.approx(T.swing_point(math.degrees(step.pitch)))


def test_demo_swing_follows_the_hinge():
    r0 = np.hypot(*(np.asarray(T.swing_point(0.0))[[0, 2]] - np.asarray(T.at(*T.PIVOT))[[0, 2]]))
    for a in T.SWING:
        r = np.hypot(*(np.asarray(T.swing_point(a))[[0, 2]] - np.asarray(T.at(*T.PIVOT))[[0, 2]]))
        assert r == pytest.approx(r0)
    assert T.swing_point(0.0) == pytest.approx(_times(T.demo()["right"])["pinch"][1].pos)


def test_wire_lies_clear_of_the_board():
    board_right = T.BOARD.pos[1] - T.BOARD.size[1] / 2
    assert T.WIRE_XY[1] + T.INS_D / 2 + 0.03 < board_right  # 3 cm for the open fingers


@pytest.mark.parametrize("arm", ["right"])
def test_demo_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    for k in range(int(plan.duration * 20) + 1):
        plan.action7(k / 20)
    assert plan.worst_ik_mm < 2.0 and plan.worst_ik_deg < 2.0, (arm, plan.worst_ik_mm, plan.worst_ik_deg)


def test_demo_fits_the_demo_run():
    assert sum(w.duration for w in T.demo()["right"]) + T.SUCCESS_HOLD_S < 600 / 20 - 2.0


# -- the meshes --------------------------------------------------------------------------------------------------------


def test_every_dressed_box_exists_and_its_mesh_is_there():
    names = {o.name for o in T.OBJECTS}
    for name, file in T.LOOKS.items():
        assert name in names
        assert (HERE / "looks" / file).is_file()


def test_wire_mesh_fits_its_box():
    """The wire's mesh stays inside its box, so its centre is the box's and the tip is where the scorer reads it."""
    pytest.importorskip("pxr.Usd")
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(HERE / "looks" / T.LOOKS["wire"]))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    box = cache.ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
    half = np.asarray(T.WIRE.size) / 2
    assert np.all(np.asarray(box.GetMax()) <= half + 1e-5)
    assert np.all(np.asarray(box.GetMin()) >= -half - 1e-5)
    assert np.asarray(box.GetMax())[0] - np.asarray(box.GetMin())[0] >= 2 * half[0] - 0.001
