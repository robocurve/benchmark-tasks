"""Offline checks for grease_gun_cartridge_load: contract, final-state scoring tiers, shortcuts, demo reach."""

from __future__ import annotations

import json
import math
import pathlib
import re

import numpy as np
import pytest

import grease_gun_cartridge_load as M
from labkit import PartState
from yam_arena import demo as _demo
from yam_arena import ik, station
from yam_arena.constants import CONTROL_HZ
from yam_arena.urdf import quat_xyzw_to_matrix

FAR_TCPS = ((0.140, 0.320, M.TOP + 0.150), (0.140, -0.320, M.TOP + 0.150))
NEAR_TCPS = ((M.GUN_X, 0.20, M.TOP + 0.10), (0.140, -0.320, M.TOP + 0.150))
CAUGHT = {"follower": -0.2369, "rod": -0.0140}
SEATED_Y = M.MOUTH_Y + 0.002 - M.CART_HALF  # cartridge centre (barrel frame) with its end 2 mm proud of the mouth


def _quat_yaw_pitch(yaw_deg=0.0, pitch_deg=0.0) -> np.ndarray:
    y, p = math.radians(yaw_deg) / 2, math.radians(pitch_deg) / 2
    qz = np.array([0.0, 0.0, math.sin(y), math.cos(y)])
    qx = np.array([math.sin(p), 0.0, 0.0, math.cos(p)])
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def _state(pos, joints=None, quat=(0.0, 0.0, 0.0, 1.0), resting=True, half=0.05) -> PartState:
    p = np.asarray(pos, float)
    v = np.zeros(3) if resting else np.array([0.05, 0.0, 0.0])
    return PartState(pos=p, quat_xyzw=np.asarray(quat, float), lin_vel=v, ang_vel=np.zeros(3), low=p - half,
                     high=p + half, joints=dict(joints or {}))


def _objs(joints=CAUGHT, cart_local=(0.0, SEATED_Y, 0.0), cart_quat=(0.0, 0.0, 0.0, 1.0), cart_resting=True):
    g = M.GUN_SPAWN_POS
    cart = _state((g[0] + cart_local[0], g[1] + cart_local[1], g[2] + cart_local[2]), quat=cart_quat,
                  resting=cart_resting, half=0.03)
    return {"gun": _state(g), "plunger": _state(g, joints=joints), "cartridge": cart,
            "mat": _state(M.MAT_POS), "rest": _state(M.REST.pos)}


def _run(objs, tcps=FAR_TCPS, steps=None):
    M._reset_trial()
    M._set_test_trial(tcps=tcps)
    n = steps if steps is not None else int(round(M.HOLD_S / M.CONTROL_DT)) + 1
    s = 0.0
    for _ in range(n):
        s = M.score(objs)
    return s


START = {"follower": 0.0, "rod": M.ROD_REST_Z}
CART_ON_REST = (M.CART_SPAWN_POS[0] - M.GUN_X, M.CART_SPAWN_POS[1], M.CART_SPAWN_POS[2] - M.GUN_SPAWN_POS[2])


def test_module_contract():
    for name in ("OBJECTS", "SCENE", "score", "success", "demo", "distance"):
        assert hasattr(M, name)
    assert {o.name for o in M.OBJECTS} == {"mat", "gun", "plunger", "cartridge", "rest"}
    assert M.CARTRIDGE.free and not M.GUN.free and not M.PLUNGER.free
    assert [j.name for j in M.PLUNGER.joints] == ["follower", "rod"] and M.ROD.parent == "follower"


def test_spring_and_keyhole_numbers():
    assert M.SPRING_K * 0.240 == pytest.approx(14.0)  # 14 N at the 240 mm catch
    assert M.FOLLOWER.target == 0.0 and M.FOLLOWER.stiffness == pytest.approx(M.SPRING_K)
    assert 2 * M.SLOT_HALF < 2 * M.ROD_R  # the shoulder cannot pass the slot
    assert 2 * M.NECK_R < 2 * M.SLOT_HALF  # the neck can
    assert 2 * M.ROD_R < 2 * M.HOLE_HALF  # the rod passes the hole
    assert M.BORE_R - M.CART_R == pytest.approx(0.001)  # 1 mm radial play


def test_every_total_is_listed_in_task_json():
    tj = json.loads((pathlib.Path(M.__file__).parent / "task.json").read_text())
    nums = {float(x) for x in re.findall(r"\d+(?:\.\d+)?", tj["scoring"].split("Totals:")[1])}
    for lv in M.SCORE_LEVELS:
        assert lv in nums
    assert len(tj["title"]) <= 120 and len(tj["instruction"]) <= 1000 and len(tj["context"]) <= 1000
    assert len(tj["setup"]) <= 4000 and len(tj["fidelity"]) <= 500 and len(", ".join(tj["keywords"])) <= 200


def test_start_state_scores_zero():
    assert _run(_objs(joints=START, cart_local=CART_ON_REST)) == 0.0


def test_intended_end_state_full_and_success():
    o = _objs()
    assert _run(o) == 1.0
    assert M.success(o)


def test_hold_needed_for_full():
    assert _run(_objs(), steps=5) == pytest.approx(0.80)


def test_rod_caught_cartridge_still_on_rest():
    assert _run(_objs(cart_local=CART_ON_REST)) == pytest.approx(0.35)


def test_cartridge_without_caught_rod_gets_nothing():
    drawn_not_dropped = {"follower": -0.2369, "rod": M.ROD_REST_Z}
    assert _run(_objs(joints=drawn_not_dropped)) == 0.0
    short_stroke = {"follower": -0.200, "rod": -0.014}
    assert _run(_objs(joints=short_stroke)) == 0.0


@pytest.mark.parametrize("dy", [0.007, -0.0075])
def test_cartridge_not_flush_is_not_seated(dy):
    assert _run(_objs(cart_local=(0.0, SEATED_Y + dy, 0.0))) == pytest.approx(0.35)


def test_cartridge_inside_tolerance_counts():
    assert _run(_objs(cart_local=(0.0, SEATED_Y - 0.0045, 0.0))) == 1.0


def test_cartridge_tilted_or_off_axis_is_not_seated():
    assert _run(_objs(cart_quat=_quat_yaw_pitch(yaw_deg=6.0))) == pytest.approx(0.35)
    assert _run(_objs(cart_local=(0.0, SEATED_Y, 0.005))) == pytest.approx(0.35)


def test_wrong_way_round_is_not_seated():
    assert _run(_objs(cart_quat=_quat_yaw_pitch(yaw_deg=180.0))) == pytest.approx(0.35)


def test_full_requires_clear_hands_and_rest():
    assert _run(_objs(), tcps=NEAR_TCPS) == pytest.approx(0.80)
    assert _run(_objs(), tcps=None) == 0.0  # unknown hands: the rod tier fails closed
    assert _run(_objs(cart_resting=False)) == pytest.approx(0.80)


def test_rod_held_by_a_hand_earns_nothing():
    g = M.GUN_SPAWN_POS
    t_handle = (g[0], g[1] + M.TBAR_Y + CAUGHT["follower"], g[2] + CAUGHT["rod"])
    holding = ((0.140, 0.320, M.TOP + 0.150), (t_handle[0] + 0.02, t_handle[1], t_handle[2] + 0.02))
    assert _run(_objs(), tcps=holding) == 0.0
    assert _run(_objs(cart_local=CART_ON_REST), tcps=holding) == 0.0


def test_drop_then_recover_is_remembered_as_failed():
    M._reset_trial()
    M._set_test_trial(tcps=FAR_TCPS)
    fallen = _objs()
    c = fallen["cartridge"]
    fallen["cartridge"] = PartState(pos=c.pos, quat_xyzw=c.quat_xyzw, lin_vel=c.lin_vel, ang_vel=c.ang_vel,
                                    low=np.array([0.0, 0.0, 0.01]), high=np.array([0.1, 0.1, 0.06]))
    assert M.score(fallen) == 0.0
    good = _objs()
    s = 0.0
    for _ in range(int(round(M.HOLD_S / M.CONTROL_DT)) + 5):
        s = M.score(good)
    assert s == 0.0 and not M.success(good)
    M._reset_trial()
    M._set_test_trial(tcps=FAR_TCPS)
    assert _run(good) == 1.0  # a fresh episode is not affected


def test_cartridge_on_the_floor_zeroes():
    o = _objs()
    c = o["cartridge"]
    o["cartridge"] = PartState(pos=c.pos, quat_xyzw=c.quat_xyzw, lin_vel=c.lin_vel, ang_vel=c.ang_vel,
                               low=np.array([0.0, 0.0, 0.01]), high=np.array([0.1, 0.1, 0.06]))
    assert _run(o) == 0.0


def test_missing_part_fails_closed():
    o = _objs()
    del o["plunger"]
    assert _run(o) == 0.0


def test_undo_after_success_drops():
    o = _objs()
    assert _run(o) == 1.0
    assert M.score(_objs(joints=START, cart_local=(0.0, SEATED_Y + 0.227, 0.0))) == 0.0


def test_demo_does_not_depend_on_trial_state():
    a = M.demo()
    M._trial["hold_s"] = 99.0
    b = M.demo()
    M._reset_trial()
    assert [(w.pos, w.finger) for w in a["left"]] == [(w.pos, w.finger) for w in b["left"]]


def test_demo_reach_within_ik_and_joint_limits():
    d = M.demo()
    for side in ("left", "right"):
        plan = _demo.ArmPlan(d[side], station.BASES[side])
        steps = int(math.ceil(plan.duration * CONTROL_HZ))
        assert steps < 2400
        for k in range(steps + 1):
            action, _ = plan.action7(k / CONTROL_HZ)
            assert np.all(action[:6] >= ik.LOWER - 1e-6) and np.all(action[:6] <= ik.UPPER + 1e-6)
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1
        for w in d[side]:
            assert w.pos[2] - M.TOP <= 0.220


def test_scored_regions_visible_and_unoccluded_in_top_camera():
    cam = station.top_camera("calibrated")
    lens = cam.lens.resized(320, 176)
    rot = quat_xyzw_to_matrix(np.array(cam.rot_xyzw))

    def project(p_world):
        p = rot.T @ (np.asarray(p_world, float) - cam.pos)
        return np.array([lens.cx + lens.fx * p[0] / (-p[2]), lens.cy - lens.fy * p[1] / (-p[2])])

    d = M.demo()
    final_tcps = {s: _demo.ArmPlan(d[s], station.BASES[s]).target(1e9)[0] for s in ("left", "right")}
    g = M.GUN_SPAWN_POS
    regions = {
        "barrel_mouth": (g[0], g[1] + M.MOUTH_Y, g[2]),
        "barrel_cap": (g[0], g[1] + M.CAP_OUT_Y, g[2]),
        "t_handle_drawn": (g[0], g[1] + M.TBAR_Y - 0.237, g[2] - 0.014),
        "cartridge_rest": M.CART_SPAWN_POS,
    }
    for name, pt in regions.items():
        u, v = project(pt)
        assert 0 <= u < lens.width and 0 <= v < lens.height, f"{name} out of top_cam frame"
        for tcp in final_tcps.values():
            arm_px = 0.08 * lens.fx / float(np.linalg.norm(tcp - cam.pos))
            assert np.linalg.norm(project(tcp) - (u, v)) > arm_px, f"{name} occluded by parked gripper"
    assert M._hands_clear(_objs(), tuple(final_tcps.values()))
    span = np.linalg.norm(project(regions["barrel_mouth"]) - project(regions["barrel_cap"]))
    assert span >= 20.0, f"barrel spans {span:.1f} px at 320x176"
