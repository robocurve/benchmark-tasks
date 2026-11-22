"""Bench-style terminal measurements and contact replay for two nested skillets."""
import json
from pathlib import Path
import math
import re

import numpy as np
import pytest

import nested_stainless_skillets_lift_upper_extract_bottom as task
from yam_arena import station


def _states(upper_xy=None, lower_xy=None, *, height=0.0, tilt=0.0,
            yaw=task.UPPER_YAW, lower_yaw=task.LOWER_YAW):
    upper_xy = task.UPPER_SPOT if upper_xy is None else upper_xy
    lower_xy = task.LOWER_SPOT if lower_xy is None else lower_xy
    upper_rot = task.mk.quat_to_mat(task.mk.yaw_quat(yaw))
    upper_rot = upper_rot @ task.mk.axis_angle((0, 1, 0), np.radians(tilt))
    return task.kit.fake_states(moved={
        task.UPPER: ((*upper_xy, task.MAT_TOP + height), task.mk.mat_to_quat(upper_rot)),
        task.LOWER: ((*lower_xy, task.MAT_TOP), task.mk.yaw_quat(lower_yaw)),
    })


def _parked():
    return [(0.1, 0.4, 1.0), (0.1, -0.4, 1.0)]


def test_documented_scoring_tiers(monkeypatch):
    scoring = json.loads(Path(__file__).with_name("task.json").read_text())["scoring"]
    tiers = re.search(r"Possible scores ([\d., ]+)\.", scoring).group(1)
    expected = {float(value) for value in tiers.split(",")}
    states = _states()
    monkeypatch.setattr(task.mk, "tcps", lambda: [])
    measured = {task.score(task.kit.fake_states()), task.score(states)}
    staged = _states(upper_xy=task.UPPER_SPOT + np.array([-0.018, 0.006]))
    measured.add(task.score(staged))
    monkeypatch.setattr(task.mk, "tcps", _parked)
    measured.add(task.score(states))
    assert measured == expected


def test_zero_for_neither_skillet_moved():
    assert task.score(task.kit.fake_states()) == 0.0


def test_only_upper_placed_shortcut():
    assert task.score(_states(lower_xy=task.NEST_XY)) == 0.0


def test_only_lower_pulled_shortcut():
    states = task.kit.fake_states(moved={
        task.UPPER: ((*task.LOWER_SPOT, task.MAT_TOP + task.LOWER_THICK),
                     task.mk.yaw_quat(task.UPPER_YAW)),
        task.LOWER: ((*task.LOWER_SPOT, task.MAT_TOP), task.mk.yaw_quat(task.LOWER_YAW)),
    })
    assert task.score(states) == 0.0


@pytest.mark.parametrize("changes", [
    {"height": 0.004}, {"height": -0.004}, {"tilt": 5.1},
    {"upper_xy": (0.30, 0.28)}, {"upper_xy": (0.37, 0.13)}, {"yaw": 0.0},
])
def test_wrong_support_or_separation_scores_zero(changes):
    assert task.score(_states(**changes)) == 0.0


def test_handle_touching_opposite_bowl_at_valid_center_gap_scores_zero():
    toward_lower = task.LOWER_SPOT - task.UPPER_SPOT
    assert np.linalg.norm(toward_lower) >= task.CENTER_GAP
    states = _states(yaw=np.arctan2(toward_lower[1], toward_lower[0]))
    upper, lower = task._pose(states, task.UPPER), task._pose(states, task.LOWER)
    assert task._seated(task.UPPER, *upper) and task._seated(task.LOWER, *lower)
    assert task._min_xy_clearance(upper, lower) < 0.0
    assert task.score(states) == 0.0


def test_handles_touching_at_valid_center_gap_scores_zero():
    between = task.UPPER_SPOT - task.LOWER_SPOT
    center_gap = np.linalg.norm(between)
    assert center_gap >= task.CENTER_GAP
    crossing = (task.UPPER_SPOT + task.LOWER_SPOT) / 2 + 0.25 * np.array([-between[1], between[0]]) / center_gap
    upper_ray, lower_ray = crossing - task.UPPER_SPOT, crossing - task.LOWER_SPOT
    states = _states(yaw=np.arctan2(upper_ray[1], upper_ray[0]), lower_yaw=np.arctan2(lower_ray[1], lower_ray[0]))
    upper, lower = task._pose(states, task.UPPER), task._pose(states, task.LOWER)
    assert task._seated(task.UPPER, *upper) and task._seated(task.LOWER, *lower)
    assert task._min_xy_clearance(upper, lower) == pytest.approx(0.0)
    assert task.score(states) == 0.0


def test_mat_penetration_scores_zero():
    assert task.score(_states(height=-0.0008)) == 0.0


def test_tilt_above_3_deg_scores_zero():
    assert task.score(_states(tilt=3.2)) == 0.0


def test_wrong_or_missing_clearance_keeps_placed_tier(monkeypatch):
    states = _states()
    bar = task.kit.parts[task.UPPER].boxes[-1]
    bar_top = task.kit.point(states, task.UPPER, np.array(bar.at) + [0, 0, bar.size[2] / 2])
    for tcps in ([], [_parked()[0]], [_parked()[0], (float("nan"), 0.0, 0.0)],
                 [_parked()[0], bar_top + [0.0, 0.0, 0.044]]):
        monkeypatch.setattr(task.mk, "tcps", lambda: tcps)
        assert task.score(states) == 0.7
        assert not task.success(states)
    monkeypatch.setattr(task.mk, "tcps", lambda: [_parked()[0], bar_top + [0.0, 0.0, 0.046]])
    assert task.score(states) == 1.0
    assert task.success(states)


def test_missing_body_scores_zero():
    states = _states()
    del states[task.UPPER]
    assert task.score(states) == 0.0


def test_clear_handles_can_finish_at_other_yaws(monkeypatch):
    monkeypatch.setattr(task.mk, "tcps", _parked)
    assert task.score(_states(yaw=np.radians(-92), lower_yaw=np.radians(-35))) == 0.0
    assert task.score(_states(yaw=np.radians(-90), lower_yaw=np.radians(-35))) == 1.0


def test_missing_gripper_link_keeps_placed_tier(monkeypatch):
    def missing_link():
        raise KeyError("missing gripper link")

    monkeypatch.setattr(task.mk, "tcps", missing_link)
    assert task.score(_states()) == 0.7


def test_scene_has_two_free_parts():
    assert {obj.name for obj in task.OBJECTS if obj.free} == {task.UPPER, task.LOWER}
    assert not task.kit.joints_
    assert all(Path(__file__).with_name("assets").joinpath(task.SLUG, obj.name + ".usdc").is_file()
               for obj in task.OBJECTS)


@pytest.fixture(scope="module")
def physics():
    # The optional offline runner is distributed separately from Arena.
    runner = pytest.importorskip("mj_" "check")
    loaded = runner.load_task(Path(task.__file__))
    limit = runner.run_limit_steps(Path(task.__file__))
    policy = runner.Precomputed(runner.make_scripted(loaded, 20.0), 0.05, limit)
    policy.collect()
    if policy.arms is None:
        actions = [runner.scripted_action(policy.pol, i * 0.05)[0] for i in range(policy.n)]
    else:
        actions = [policy.action(i, i * 0.05)[0] for i in range(policy.n)]
    return runner, loaded, actions


def _replay(physics, parked=None):
    runner, loaded, actions = physics
    sim = runner.Sim(loaded, 20.0)
    sim.reset()
    ready = sim.ready_action()
    for action in actions:
        command = action.copy()
        if parked is not None:
            command[parked] = ready[parked]
        result = sim.step(command)
        if result["terminated"]:
            break
    return sim, result


def test_replay_scores_full_after_hold_and_clears_camera_occlusion(physics):
    runner, loaded, actions = physics
    sim = runner.Sim(loaded, 20.0)
    sim.reset()
    assert loaded.score(sim.object_states()) == 0.0
    hold_steps = round(2.0 / sim.dt)
    seated_before_withdrawal = False
    held = 0
    for action in actions:
        result = sim.step(action)
        if result["score"] == 0.7:
            seated_before_withdrawal = True
        if result["score"] == 1.0:
            assert seated_before_withdrawal
            held += 1
        else:
            held = 0
        assert result["terminated"] == (held >= hold_steps)
        if result["terminated"]:
            break
    assert result["score"] == 1.0
    assert result["terminated"]
    assert sim.held == held == hold_steps
    renderer = runner.Renderer(sim)
    try:
        for view in renderer.measure(arms=True).values():
            assert view["in_frame"]
            assert view["px"] >= 25
            assert view["occluded"] <= runner.VIS_MAX_END_OCCLUSION
    finally:
        renderer.r.close()


@pytest.mark.parametrize("parked", [slice(0, 7), slice(7, 14)], ids=["left", "right"])
def test_one_hand_physics_without_hold_scores_zero(physics, parked):
    _, result = _replay(physics, parked)
    assert result["score"] == 0.0
    assert not result["terminated"]


# -- occlusion at the end pose: sight lines from the top cameras to the scored region --------------------------------
# End pose: cloud run 8's demo record (runs/20261121T075146Z-verified, demo/actions/20261121_075254_b7e6f9e6/
# scene-0-e0.jsonl; record.json terminates 'success' at t=387). The record carries commanded joint targets only, no
# link states: link origins are the arena's URDF forward kinematics of those targets. Row 387 is the last row and
# 367 twenty steps earlier (the real arm lags its target, so the test checks both ends of that interval); row 200 has
# the arms over the skillets mid-task (4 rim points hidden), the real-data positive control.
# Generated from that record (388 step rows, last t=387).
END_BODIES = ('base', 'link1', 'link2', 'link3', 'link4', 'link5', 'gripper', 'tip_left', 'tip_right')
END_LINKS = {
    387: {
        'left_robot': (  # TCP (0.16002, 0.33995, 0.86), gripper 1.000
            (0.0, 0.305, 0.75),  # base
            (0.0, 0.305, 0.818),  # link1
            (0.01231, 0.34239, 0.8635),  # link2
            (-0.07897, 0.25204, 1.1042),  # link3
            (0.15279, 0.37306, 1.1036),  # link4
            (0.19959, 0.34858, 1.0296),  # link5
            (0.16002, 0.33994, 0.994),  # gripper
            (0.18401, 0.34239, 0.93732),  # tip_left
            (0.13603, 0.33751, 0.93732),  # tip_right
        ),
        'right_robot': (  # TCP (0.16004, -0.37993, 0.86001), gripper 1.000
            (0.0, -0.305, 0.75),  # base
            (0.0, -0.305, 0.818),  # link1
            (0.03248, -0.28277, 0.8635),  # link2
            (-0.08312, -0.30463, 1.10964),  # link3
            (0.17443, -0.34921, 1.10361),  # link4
            (0.19674, -0.39708, 1.02961),  # link5
            (0.16006, -0.37991, 0.99401),  # gripper
            (0.18404, -0.37748, 0.93733),  # tip_left
            (0.13606, -0.38236, 0.93733),  # tip_right
        ),
    },
    367: {
        'left_robot': (  # TCP (0.27728, 0.24295, 0.87947), gripper 1.000
            (0.0, 0.305, 0.75),  # base
            (0.0, 0.305, 0.818),  # link1
            (0.02692, 0.33371, 0.8635),  # link2
            (0.02357, 0.26396, 1.12723),  # link3
            (0.28471, 0.27603, 1.12307),  # link4
            (0.31682, 0.2341, 1.04907),  # link5
            (0.27729, 0.24295, 1.01347),  # gripper
            (0.30136, 0.2414, 0.95679),  # tip_left
            (0.25322, 0.24451, 0.95679),  # tip_right
        ),
        'right_robot': (  # TCP (0.16004, -0.37993, 0.86001), gripper 1.000
            (0.0, -0.305, 0.75),  # base
            (0.0, -0.305, 0.818),  # link1
            (0.03248, -0.28277, 0.8635),  # link2
            (-0.08312, -0.30463, 1.10964),  # link3
            (0.17443, -0.34921, 1.10361),  # link4
            (0.19674, -0.39708, 1.02961),  # link5
            (0.16006, -0.37991, 0.99401),  # gripper
            (0.18404, -0.37748, 0.93733),  # tip_left
            (0.13606, -0.38236, 0.93733),  # tip_right
        ),
    },
    200: {
        'left_robot': (  # TCP (0.2585, 0.12394, 0.93887), gripper -0.000
            (0.0, 0.305, 0.75),  # base
            (0.0, 0.305, 0.818),  # link1
            (0.03618, 0.32049, 0.8635),  # link2
            (0.00881, 0.25524, 1.12698),  # link3
            (0.24829, 0.16479, 1.18015),  # link4
            (0.26853, 0.10789, 1.1122),  # link5
            (0.24086, 0.13002, 1.07156),  # gripper
            (0.2563, 0.07763, 1.01877),  # tip_left
            (0.24035, 0.17727, 1.01209),  # tip_right
        ),
        'right_robot': (  # TCP (0.27811, -0.20275, 0.80574), gripper -0.000
            (0.0, -0.305, 0.75),  # base
            (0.0, -0.305, 0.818),  # link1
            (0.00709, -0.26628, 0.8635),  # link2
            (0.06423, -0.31859, 1.12509),  # link3
            (0.26635, -0.17105, 1.04934),  # link4
            (0.31608, -0.1889, 0.97536),  # link5
            (0.27808, -0.20284, 0.93974),  # gripper
            (0.32857, -0.19986, 0.88307),  # tip_left
            (0.22761, -0.20573, 0.88304),  # tip_right
        ),
    },
}

# Top camera: yam_arena/station.py:99 TOP_CAM_POS (calibration frame) -> from_calibration in top_camera(); the run used
# variant "calibrated". Second camera: top_camera("demo"), Dream's documented demo view (15 cm lower, 5 deg up).
CAMS = {v: station.top_camera(v) for v in station.TOP_CAMS}
FINGER_R = 0.010  # m: proxy half-thickness of a finger / link capsule (small on purpose: favours "clear")
LINK_R = 0.030
RIM_H = {task.UPPER: 0.042, task.LOWER: 0.046}  # bowl heights passed to skillet_boxes() in the task file
RIM_R = {task.UPPER: task.UPPER_RAD, task.LOWER: task.LOWER_RAD}
SPOT = {task.UPPER: task.UPPER_SPOT, task.LOWER: task.LOWER_SPOT}


def _seg_dist(p0, p1, q0, q1):
    """Closest distance between segments p0-p1 and q0-q1 (Ericson, Real-Time Collision Detection, 5.1.9)."""
    d1, d2, r = p1 - p0, q1 - q0, p0 - q0
    a, e, f = float(d1 @ d1), float(d2 @ d2), float(d2 @ r)
    c, b = float(d1 @ r), float(d1 @ d2)
    denom = a * e - b * b
    s_ = min(max((b * f - c * e) / denom, 0.0), 1.0) if denom > 1e-15 else 0.0
    t_ = (b * s_ + f) / e
    if t_ < 0.0:
        t_, s_ = 0.0, min(max(-c / a, 0.0), 1.0)
    elif t_ > 1.0:
        t_, s_ = 1.0, min(max((b - c) / a, 0.0), 1.0)
    return float(np.linalg.norm((p0 + d1 * s_) - (q0 + d2 * t_)))


def _capsules(links):
    L = [np.asarray(p, float) for p in links]
    return [(L[i], L[i + 1], LINK_R) for i in range(6)] + [(L[6], L[7], FINGER_R), (L[6], L[8], FINGER_R)]


def _occluders(cam, caps, targets):
    return [(lab, i) for lab, pt in targets for i, (a, b, r) in enumerate(caps) if _seg_dist(cam, pt, a, b) <= r]


def _pixel(cam, p):
    """(u, v) of world point p in the 640 x 360 image of an OpenGL-convention camera (looks along -z, +y up)."""
    from yam_arena.urdf import quat_xyzw_to_matrix

    rot = np.asarray(quat_xyzw_to_matrix(np.asarray(cam.rot_xyzw, float)))
    x, y, z = rot.T @ (np.asarray(p, float) - np.asarray(cam.pos, float))
    k = cam.lens
    return (k.cx + k.fx * x / -z, k.cy - k.fy * y / -z) if z < 0 else (float("nan"),) * 2


def _targets():
    """The scored region: each skillet at its marked spot -- its centre on the mat and eight points on its rim."""
    out = []
    for name in (task.UPPER, task.LOWER):
        cx, cy = SPOT[name]
        out.append((f"{name} centre", np.array([cx, cy, task.MAT_TOP + 0.003])))
        for k in range(8):
            az = 2 * math.pi * k / 8
            out.append((f"{name} rim {45 * k}", np.array([cx + RIM_R[name] * math.cos(az), cy + RIM_R[name] * math.sin(az),
                                                          task.MAT_TOP + RIM_H[name]])))
    return out


def _arm_caps(row):
    return _capsules(END_LINKS[row]["left_robot"]) + _capsules(END_LINKS[row]["right_robot"])


def test_seg_dist_matches_brute_force():
    rng = np.random.default_rng(0)
    s = np.linspace(0.0, 1.0, 1501)[:, None]
    for _ in range(200):
        p0, p1, q0, q1 = rng.normal(size=(4, 3))
        bf = np.min(np.linalg.norm((p0 + (p1 - p0) * s)[:, None] - (q0 + (q1 - q0) * s)[None], axis=2))
        assert bf - 5e-3 <= _seg_dist(p0, p1, q0, q1) <= bf + 1e-9


def test_occlusion_positive_control_detects_a_blocker():
    tg = _targets()
    cam = np.asarray(CAMS["calibrated"].pos, float)
    mid = cam + 0.75 * (tg[0][1] - cam)
    assert _occluders(cam, [(mid - (0, 0, 0.02), mid + (0, 0, 0.02), FINGER_R)], tg[:1])
    assert not _occluders(cam, [(mid + (0.06, 0, -0.02), mid + (0.06, 0, 0.02), FINGER_R)], tg[:1])  # 60 mm aside
    # real data: mid-task (row 200) the arms are over the skillets and hide part of the scored region
    assert len({lab for lab, _ in _occluders(cam, _arm_caps(200), tg)}) >= 3


def test_scored_region_is_in_frame_of_both_top_cameras():
    for var, cam in CAMS.items():
        for lab, p in _targets():
            u, v = _pixel(cam, p)
            assert 0.0 <= u < cam.lens.width and 0.0 <= v < cam.lens.height, (var, lab, u, v)


def test_occlusion_top_cameras_see_scored_region_at_demo_end():
    for row in (387, 367):
        for var, cam in CAMS.items():
            hits = _occluders(np.asarray(cam.pos, float), _arm_caps(row), _targets())
            assert not hits, (row, var, hits)


# -- per-step latches: each must fail the whole run even when the final pose would score 1.0 ---------------------
def _pose_states(upper=None, lower=None):
    moved = {}
    for name, value in ((task.UPPER, upper), (task.LOWER, lower)):
        if value is not None:
            xyz, tilt = value
            rot = task.mk.quat_to_mat(task.mk.yaw_quat(task.UPPER_YAW if name == task.UPPER else task.LOWER_YAW))
            rot = rot @ task.mk.axis_angle((0, 1, 0), np.radians(tilt))
            # a tilted pan rests on its rim: raise it so its lowest collider point keeps the requested height
            low = (task._GEOMETRY[name][3] @ rot.T)[:, 2].min()
            xyz = (xyz[0], xyz[1], xyz[2] + max(0.0, -low)) if tilt else xyz
            moved[name] = (xyz, task.mk.mat_to_quat(rot))
    return task.kit.fake_states(moved=moved)


def _run_steps(monkeypatch, frames, tcps=_parked):
    task.mk.new_episode()
    monkeypatch.setattr(task.mk, "tcps", tcps)
    for frame in frames:
        task.latch_step(frame)
    return task.mk.MEMORY["latch"]["failed"]


def _final_full(monkeypatch):
    monkeypatch.setattr(task.mk, "tcps", _parked)
    return task.score(_states())


SPOT_U = (*task.UPPER_SPOT, task.MAT_TOP)
SPOT_L = (*task.LOWER_SPOT, task.MAT_TOP)


def test_latch_free_run_keeps_full_score(monkeypatch):
    assert _run_steps(monkeypatch, [_states(), _states()]) is None
    assert _final_full(monkeypatch) == 1.0


def test_latch_tipped_unheld_skillet(monkeypatch):
    tilt = task.FREE_TILT_DEG + 5
    failed = _run_steps(monkeypatch, [_pose_states(upper=(SPOT_U, tilt), lower=(SPOT_L, 0))])
    assert failed and "tipped" in failed["why"]
    assert _final_full(monkeypatch) == 0.0


def test_latch_tilt_while_held_is_allowed(monkeypatch):
    frame = _pose_states(upper=(SPOT_U, task.FREE_TILT_DEG + 5), lower=(SPOT_L, 0))
    pos, rot = task._pose(frame, task.UPPER)
    bar = pos + rot @ task._GEOMETRY[task.UPPER][0][-1]
    assert _run_steps(monkeypatch, [frame], tcps=lambda: [tuple(bar), (0.1, -0.4, 1.0)]) is None


def test_latch_dropped_skillet(monkeypatch):
    high = (*task.UPPER_SPOT, task.MAT_TOP + task.DROP_HEIGHT + 0.010)
    failed = _run_steps(monkeypatch, [_pose_states(upper=(high, 0), lower=(SPOT_L, 0))])
    assert failed and "above its support" in failed["why"]
    assert _final_full(monkeypatch) == 0.0


def test_latch_skillet_left_the_mat(monkeypatch):
    off = (task.MAT_CENTER[0], task.MAT_CENTER[1] - task.MAT_SIZE[1] / 2 - 0.05, task.MAT_TOP)
    failed = _run_steps(monkeypatch, [_pose_states(upper=(SPOT_U, 0), lower=(off, 0))])
    assert failed and "left the mat" in failed["why"]
    assert _final_full(monkeypatch) == 0.0


def test_latch_flung_skillet(monkeypatch):
    step = task.FLING_SPEED * task.CONTROL_DT * 1.5
    moved = (task.LOWER_SPOT[0] + step, task.LOWER_SPOT[1], task.MAT_TOP)
    failed = _run_steps(monkeypatch, [_pose_states(upper=(SPOT_U, 0), lower=(SPOT_L, 0)),
                                      _pose_states(upper=(SPOT_U, 0), lower=(moved, 0))])
    assert failed and "flung" in failed["why"]
    assert _final_full(monkeypatch) == 0.0


def test_latch_pair_dragged_while_nested(monkeypatch):
    shift = np.array([task.DRAG_DIST + 0.010, 0.0])
    lower = (*(task.NEST_XY + shift), task.MAT_TOP)
    upper = (*(task.NEST_XY + shift), task.MAT_TOP + task.LOWER_THICK)
    failed = _run_steps(monkeypatch, [_pose_states(upper=(upper, 0), lower=(lower, 0))])
    assert failed and "dragged" in failed["why"]
    assert _final_full(monkeypatch) == 0.0
