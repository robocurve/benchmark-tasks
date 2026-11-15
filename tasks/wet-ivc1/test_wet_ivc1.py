"""Offline checks for wet_ivc1: contract, final-state scoring tiers, shortcuts, demo IK reach, top-cam occlusion."""

from __future__ import annotations

import itertools
import json
import math
import re
import pathlib

import numpy as np
import pytest

import wet_ivc1 as M
from labkit import PartState
from yam_arena import demo as _demo
from yam_arena import ik, station
from yam_arena.constants import CONTROL_HZ
from yam_arena.urdf import quat_xyzw_to_matrix

FAR_TCPS = ((0.140, 0.320, M.TOP + 0.150), (0.140, -0.320, M.TOP + 0.150))
DOCKED_CAGE_X = M.STOP_WORLD_X - M.CAGE_HALF - 0.001  # 1 mm short of the stop -> inside 3 mm


def _rot_quat(roll_deg: float = 0.0, pitch_deg: float = 0.0, yaw_deg: float = 0.0) -> np.ndarray:
    r, p, y = map(math.radians, (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(r / 2.0), math.sin(r / 2.0)
    cp, sp = math.cos(p / 2.0), math.sin(p / 2.0)
    cy, sy = math.cos(y / 2.0), math.sin(y / 2.0)
    x = sr * cp * cy - cr * sp * sy
    yv = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return np.array([x, yv, z, w], float)


def _cage(pos_x=DOCKED_CAGE_X, pos_y=0.0, low_z=M.HANG_LOW_Z, pitch_deg=0.0, resting=True, joints=None) -> PartState:
    z0 = M.TOP + low_z
    p = np.array([pos_x, pos_y, z0], float)
    v = np.zeros(3) if resting else np.array([0.08, 0.0, 0.0])
    low = np.array([pos_x - M.CAGE_HALF, pos_y - M.CAGE_WIDTH / 2.0, z0])
    high = np.array([pos_x + M.CAGE_HALF, pos_y + M.CAGE_WIDTH / 2.0, z0 + M.CAGE_H])
    return PartState(pos=p, quat_xyzw=_rot_quat(pitch_deg=pitch_deg), lin_vel=v, ang_vel=np.zeros(3), low=low, high=high,
                     joints=dict(joints or {}))


def _bottle(cage: PartState, local_offset=(0.0, 0.0, 0.0), tilt_deg: float = 0.0, resting: bool = True) -> PartState:
    bp = cage.to_world((M.POCKET_X + local_offset[0], local_offset[1], M.POCKET_FLOOR_Z + M.BOTTLE_HALF_H + local_offset[2]))
    v = np.zeros(3) if resting else np.array([0.05, 0.0, 0.0])
    return PartState(
        pos=bp,
        quat_xyzw=_rot_quat(roll_deg=tilt_deg),
        lin_vel=v,
        ang_vel=np.zeros(3),
        low=bp - 0.03,
        high=bp + 0.03,
    )


def _rack() -> PartState:
    p = np.asarray(M.RACK_POS, float)
    return PartState(pos=p, quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                     low=p - 0.25, high=p + 0.25)


OPEN = {"valve_l": -0.007, "valve_r": -0.007}
CLOSED = {"valve_l": 0.0, "valve_r": 0.0}


def _lid(cage: PartState | None = None, offset=(0.0, 0.0, 0.0), yaw_deg: float = 0.0, resting: bool = True) -> PartState:
    """The filter top seated on ``cage`` (shifted by ``offset`` m and turned ``yaw_deg`` in the cage frame), or lying
    flat on the bench at its spawn place when ``cage`` is None."""
    if cage is None:
        p, q = np.asarray(M.LID_SPAWN_POS, float), _rot_quat(yaw_deg=yaw_deg)
    else:
        p = cage.to_world(offset)
        c = cage.quat_xyzw
        y = _rot_quat(yaw_deg=yaw_deg)  # q = q_cage * q_yaw
        q = np.array([
            c[3] * y[0] + c[0] * y[3] + c[1] * y[2] - c[2] * y[1],
            c[3] * y[1] - c[0] * y[2] + c[1] * y[3] + c[2] * y[0],
            c[3] * y[2] + c[0] * y[1] - c[1] * y[0] + c[2] * y[3],
            c[3] * y[3] - c[0] * y[0] - c[1] * y[1] - c[2] * y[2],
        ])
    v = np.zeros(3) if resting else np.array([0.05, 0.0, 0.0])
    return PartState(pos=p, quat_xyzw=q, lin_vel=v, ang_vel=np.zeros(3), low=p + (-0.19, -0.08, 0.115),
                     high=p + (0.19, 0.08, 0.16))


def _objs(*, cage_x=DOCKED_CAGE_X, low_z=M.HANG_LOW_Z, joints=None, cage_pitch=0.0, cage_resting=True,
          bottle_offset=(0.0, 0.0, 0.0), bottle_tilt=0.0, lid="seated", lid_offset=(0.0, 0.0, 0.0), lid_yaw=0.0
          ) -> dict[str, PartState]:
    c = _cage(cage_x, 0.0, low_z, cage_pitch, cage_resting, OPEN if joints is None else joints)
    top = _lid(None if lid == "bench" else c, lid_offset, lid_yaw)
    return {"cage": c, "lid": top, "bottle": _bottle(c, bottle_offset, bottle_tilt), "rack": _rack()}


def _run(seq, tcps=FAR_TCPS):
    """Feed a sequence of states to score() as the runtime does (once per control step); return the last score."""
    M._reset_trial()
    M._set_test_trial(tcps=tcps)
    s = 0.0
    for objs in seq:
        s = M.score(objs)
    return s


BENCH = dict(cage_x=M.CAGE_SPAWN_POS[0], low_z=M.MAT_H)
HOLD_N = int(round(M.HOLD_S / M.CONTROL_DT)) + 1


def _tier_states():
    """One final state per combination of the scored conditions, built from real geometry (not from the weights)."""
    out = {}
    places = {"bench": BENCH, "runners_short": dict(cage_x=0.42), "home": {}}
    valves = {0: CLOSED, 1: {"valve_l": -0.007, "valve_r": -0.002}, 2: OPEN}
    for (place, kw), lid, (nv, j), hands in itertools.product(places.items(), ("bench", "seated", "off_register"),
                                                              valves.items(), ("far", "near")):
        if place != "home" and nv:
            continue  # a valve only opens on its nozzle, with the cage home
        o = _objs(**kw, joints=j, lid="bench" if lid == "bench" else "seated",
                  lid_offset=(0.006, 0.0, 0.004) if lid == "off_register" else (0.0, 0.0, 0.0))
        tcps = FAR_TCPS if hands == "far" else ((o["cage"].pos[0], 0.05, M.TOP + 0.25), FAR_TCPS[1])
        out[(place, lid, nv, hands)] = _run([o] * HOLD_N, tcps=tcps)
    return out


# -- contract ----------------------------------------------------------------------------------------------


def test_module_contract():
    assert M.SCENE == "station" and M.SCORE_OVER == "end" and M.SUCCESS_HOLD_S == 1.0
    assert [o.name for o in M.OBJECTS] == ["mat", "cage", "lid", "bottle", "rack"]
    assert M.MAT.fixed and M.RACK.fixed and not M.CAGE.fixed and not M.LID.fixed and not M.BOTTLE.fixed
    assert [j.name for j in M.CAGE.joints] == ["valve_l", "valve_r"] and M.RACK.joints == () and M.LID.joints == ()
    assert M.LID.mass == pytest.approx(0.120)
    assert M.CAGE_LEN == pytest.approx(0.391) and M.CAGE_WIDTH == pytest.approx(0.199) and M.CAGE_H == pytest.approx(0.160)
    assert M.BOTTLE_W == pytest.approx(0.055) and M.BOTTLE_H == pytest.approx(0.129)
    assert M.HANG_LOW_Z - M.CELL_FLOOR_H == pytest.approx(0.016)
    # a docked piston (tip 10 mm past the stop face) is pushed in 7 mm by its nozzle: > 5 mm open, < 8 mm travel
    travel = M.STOP_WORLD_X + M.VALVE_PROTRUDE - M.NOZZLE_TIP_X
    assert M.VALVE_OPEN_M < travel < M.VALVE_TRAVEL_MAX
    # docked spring push (0.7 N each) minus seal friction is far below runner friction: the cage stays home
    assert 2 * (M.VALVE_K * travel - M.VALVE_FRICTION_N) < 1.0


def test_top_starts_off_the_cage_flat_on_the_bench():
    lid_low = M.LID_SPAWN_POS[2] + M._LID_LOW_Z
    assert lid_low == pytest.approx(M.TOP + 0.0005)
    # its footprint (+-0.194 x +-0.082 m) lies clear of the cage flanges (y <= 0.0995)
    assert M.LID_SPAWN_XY[1] - 0.082 > M.CAGE_SPAWN_POS[1] + M.CAGE_WIDTH / 2.0


def test_no_scored_state_is_driven_by_a_step_hook():
    import inspect
    src = inspect.getsource(M._watch) + inspect.getsource(M._on_reset)
    assert "set_joint_position_target" not in src and "write_joint" not in src and "write_root" not in src


def test_every_total_is_reachable_and_listed():
    tj = json.loads((pathlib.Path(__file__).parent / "task.json").read_text())
    levels = sorted(set(_tier_states().values()))
    assert levels == list(M.SCORE_LEVELS)
    s = tj["scoring"]
    listed = s[s.index("Possible totals"):s.index("Cage tipped")]
    assert sorted(float(x) for x in re.findall(r"\d+(?:\.\d+)?", listed)) == levels


def test_task_json_limits_and_thresholds():
    tj = json.loads((pathlib.Path(__file__).parent / "task.json").read_text())
    for k, n in (("title", 120), ("instruction", 1000), ("context", 1000), ("setup", 4000), ("fidelity", 500),
                 ("scores", 300), ("run_limit", 200)):
        assert len(tj[k]) <= n, k
    assert len(", ".join(tj["keywords"])) <= 200 and tj["category"] == "Impact on jobs"
    text = " ".join(str(v) for v in tj.values()).lower()
    assert not re.search(r"\blab(oratory)?\b", text) and "@" not in text
    s = tj["scoring"]
    for token in ("0.15", "0.20", "0.25", "8 mm", "5 degrees", "1.5 mm", "5 mm", "3 mm", "12 cm", "3 s", "20 degrees"):
        assert token in s, token
    assert tj["instruction"] == ("Seal the mouse cage and dock it in the empty bay of the ventilated rack so it is "
                                 "connected to the rack's air supply, then move both arms clear of the cage and rack.")


# -- scoring & shortcuts -----------------------------------------------------------------------------------


def test_start_state_scores_zero():
    assert _run([_objs(**BENCH, joints=CLOSED, lid="bench")] * HOLD_N) == 0.0


def test_intended_end_state_full_and_success():
    assert _run([_objs()] * (HOLD_N - 2)) == pytest.approx(0.85)  # not yet held 3 s
    assert _run([_objs()] * HOLD_N) == pytest.approx(1.0)
    assert M.success(_objs())


def test_final_state_only_no_history():
    # order does not matter: the same end state reached after an unsealed docking scores the same
    detour = [_objs(**BENCH, joints=CLOSED, lid="bench"), _objs(cage_x=0.42, joints=CLOSED, lid="bench")]
    assert _run(detour + [_objs()] * HOLD_N) == pytest.approx(1.0)


def test_sealing_on_the_bench_alone():
    assert _run([_objs(**BENCH, joints=CLOSED)] * HOLD_N) == pytest.approx(0.20)


def test_shortcut_top_left_on_the_bench():
    assert _run([_objs(lid="bench")] * HOLD_N) == pytest.approx(0.65)


@pytest.mark.parametrize("offset,yaw", [((0.004, 0.0, 0.0), 0.0), ((0.0, 0.003, 0.0), 0.0), ((0.0, 0.0, 0.006), 0.0),
                                        ((0.0, 0.0, 0.0), 2.0)])
def test_shortcut_top_on_but_out_of_register(offset, yaw):
    assert _run([_objs(lid_offset=offset, lid_yaw=yaw)] * HOLD_N) == pytest.approx(0.65)


def test_top_inside_register_tolerance_counts():
    assert _run([_objs(lid_offset=(0.001, -0.001, 0.0005))] * HOLD_N) == pytest.approx(1.0)


def test_shortcut_one_valve_only():
    one = {"valve_l": -0.007, "valve_r": -0.002}
    assert _run([_objs(joints=one)] * HOLD_N) == pytest.approx(0.60)
    assert _run([_objs(joints=one, lid="bench")] * HOLD_N) == pytest.approx(0.40)


def test_shortcut_cage_on_bay_floor_or_tipped():
    assert _run([_objs(low_z=M.CELL_FLOOR_H)] * HOLD_N) == pytest.approx(0.20)  # sealed top only, no rack credit
    assert _run([_objs(low_z=M.CELL_FLOOR_H, lid="bench")] * HOLD_N) == 0.0
    assert _run([_objs(cage_pitch=30.0)] * HOLD_N) == 0.0


def test_shortcut_short_of_the_stop_opens_no_valve():
    assert _run([_objs(cage_x=DOCKED_CAGE_X - 0.010, joints=CLOSED)] * HOLD_N) == pytest.approx(0.35)


def test_shortcut_undo_pull_back_or_top_pushed_off_after_success():
    pulled = _objs(cage_x=DOCKED_CAGE_X - 0.05, joints=CLOSED)
    assert _run([_objs()] * HOLD_N + [pulled]) == pytest.approx(0.35)
    pushed = _objs(lid_offset=(-0.012, 0.0, 0.010), lid_yaw=1.0)
    assert _run([_objs()] * HOLD_N + [pushed]) == pytest.approx(0.65)


def test_full_requires_clear_hands_rest_and_bottle():
    near = ((DOCKED_CAGE_X, 0.08, M.TOP + 0.2), FAR_TCPS[1])
    assert _run([_objs()] * HOLD_N, tcps=near) == pytest.approx(0.85)
    assert _run([_objs(bottle_tilt=40.0)] * HOLD_N) == pytest.approx(0.85)
    assert _run([_objs(cage_resting=False)] * HOLD_N) == pytest.approx(0.85)


def test_missing_part_fails_closed():
    o = _objs()
    del o["lid"]
    assert _run([o] * HOLD_N) == 0.0


def test_arms_cannot_set_a_top_on_a_docked_cage():
    """Real-geometry reason the top goes on before docking: a straight-down grip over a docked cage is out of reach."""
    W = _demo.Waypoint
    x = DOCKED_CAGE_X + M.LID_GRIP_X
    for z in (0.10, 0.15, M.HANG_LOW_Z + M.LID_GRIP_Z + M.TIP_BELOW_TCP, 0.20):
        for side in ("left", "right"):
            plan = _demo.ArmPlan([W(2.0, (x, 0.0, M.TOP + z), M.FINGER_OPEN, "over docked cage")], station.BASES[side])
            for k in range(int(plan.duration * CONTROL_HZ) + 1):
                plan.action7(k / CONTROL_HZ)  # worst_ik_mm is filled lazily as the plan is sampled
            assert plan.worst_ik_mm > 100.0, (side, z, plan.worst_ik_mm)


def test_demo_does_not_depend_on_trial_state():
    M._reset_trial()
    a = M.demo()
    M._trial["tcps"] = FAR_TCPS
    M._trial["hold_s"] = 2.0
    assert M.demo() == a


def test_demo_reach_within_ik_and_joint_limits():
    d = M.demo()
    for side in ("left", "right"):
        plan = _demo.ArmPlan(d[side], station.BASES[side])
        steps = int(math.ceil(plan.duration * CONTROL_HZ))
        assert steps * 2 < 2400
        for k in range(steps + 1):
            action, _ = plan.action7(k / CONTROL_HZ)
            assert np.all(action[:6] >= ik.LOWER - 1e-6) and np.all(action[:6] <= ik.UPPER + 1e-6)
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1
    # no idle hovering > 1.0 s except the final parked hold the 3 s scoring hold needs
    for side in d:
        for a, w in zip(d[side][:-2], d[side][1:-1]):
            assert not (a.pos == w.pos and a.finger == w.finger and w.duration > 1.0), w.label


def test_scored_regions_visible_and_unoccluded_in_top_camera():
    cam = station.top_camera("calibrated")
    lens = cam.lens.resized(320, 176)
    rot = quat_xyzw_to_matrix(np.array(cam.rot_xyzw))

    def project(p_world):
        p = rot.T @ (np.asarray(p_world, float) - cam.pos)
        return np.array([lens.cx + lens.fx * p[0] / (-p[2]), lens.cy - lens.fy * p[1] / (-p[2])])

    d = M.demo()
    final_tcps = {s: _demo.ArmPlan(d[s], station.BASES[s]).target(1e9)[0] for s in ("left", "right")}

    cage_final = _cage(DOCKED_CAGE_X)
    regions = {
        "cage_start": (M.CAGE_SPAWN_POS[0], 0.0, M.TOP + M.CAGE_H * 0.5),
        "top_on_bench": (M.LID_SPAWN_XY[0], M.LID_SPAWN_XY[1], M.TOP + 0.03),
        "cage_docked_centre": (DOCKED_CAGE_X, 0.0, M.TOP + M.HANG_LOW_Z + M.CAGE_H * 0.5),
        "bottle_docked": cage_final.to_world((M.POCKET_X, 0.0, M.POCKET_FLOOR_Z + M.BOTTLE_HALF_H)),
    }
    for name, pt in regions.items():
        u, v = project(pt)
        assert 0 <= u < lens.width and 0 <= v < lens.height, f"{name} out of top_cam frame"
        for tcp in final_tcps.values():
            arm_px = 0.08 * lens.fx / float(np.linalg.norm(tcp - cam.pos))
            assert np.linalg.norm(project(tcp) - (u, v)) > arm_px, f"{name} occluded by parked gripper"

    assert M._hands_clear({"cage": cage_final, "lid": _lid(cage_final), "bottle": _bottle(cage_final), "rack": _rack()},
                          tuple(final_tcps.values()))
    bottle_span_px = np.linalg.norm(
        project((M.BOTTLE_SPAWN_POS[0] - M.BOTTLE_W / 2.0, 0.0, M.BOTTLE_SPAWN_POS[2]))
        - project((M.BOTTLE_SPAWN_POS[0] + M.BOTTLE_W / 2.0, 0.0, M.BOTTLE_SPAWN_POS[2]))
    )
    assert bottle_span_px >= 5.0, f"bottle spans {bottle_span_px:.1f} px at 320x176 (< 20 px at 1280x704)"
