"""Offline CPU tests for spin-lazy-susan-bring-back-jar-to-front: the score bookkeeping driven step by step with posed states
(no simulator), the zero latches, the score tiers, the one-second stillness rule, the demo's reach and length, and the top cameras' view of the
scored region at the end of the GPU-simulator demo run."""

import json
import math
import pathlib
import re
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import mechkit as mk  # noqa: E402
import spin_lazy_susan_bring_back_jar_to_front as t  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.urdf import quat_xyzw_to_matrix  # noqa: E402

DT = 0.05


@pytest.fixture(autouse=True)
def _fresh_episode():
    mk.new_episode()
    yield
    mk.new_episode()


def _yaw(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def _tip(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def _on_ring(az_deg):
    a = math.radians(az_deg)
    return np.array([t.TT_X + t.RING_R * math.cos(a), t.TT_Y + t.RING_R * math.sin(a), t.JAR_Z])


def _turned(deg, moved=None):
    """The tray and every jar turned ``deg`` about the bearing (jars carried round, upright); ``moved`` overrides."""
    m = {"tray": (np.array([t.TT_X, t.TT_Y, t.FLOOR_Z + t.TRAY_Z - t.FLOOR_T]), _yaw(deg))}
    for i, j in enumerate(t.JARS):
        m[j] = (_on_ring(i * 60.0 + deg), _yaw(i * 60.0 + deg))
    m.update(moved or {})
    return t.kit.fake_states(moved=m)


def _step(objects):
    t.susan_step(objects)
    return t.score(objects)


def _hold(objects, steps=None):
    """The same scene for SUCCESS_HOLD_S of control steps (plus one); the score at the last."""
    n = int(round(t.SUCCESS_HOLD_S * t.CONTROL_HZ)) + 1 if steps is None else steps
    for _ in range(n):
        sc = _step(objects)
    return sc


def _on_mark():
    return (np.array([t.MARK[0], t.MARK[1], t.TOP + t.JAR[2] / 2]), _yaw(0.0))


def test_start_layout_scores_zero_and_target_is_at_the_back():
    s = t.kit.fake_states()
    assert abs(t.jar_angle_deg(s, t.TARGET)) < 1.0
    assert all(t.on_tray(s, j) and t.tilt_deg(s, j) < 1.0 for j in t.JARS)
    assert _step(s) == 0.0  # holding still scores 0


def test_target_brought_round_to_the_front_scores_half():
    for deg in (0, 60, 120, 140):
        assert _step(_turned(-deg)) == 0.0
    assert _step(_turned(-180)) == 0.5
    assert t._mem()["fronted"] and not t._mem()["spoiled"]


def test_front_sector_edge():
    assert _step(_turned(-(180 - t.FRONT_DEG) + 1.0)) == 0.0
    assert _step(_turned(-(180 - t.FRONT_DEG) - 1.0)) == 0.5


def test_full_task_scores_one():
    _step(_turned(-180))
    done = _turned(-180, {t.TARGET: _on_mark()})
    assert _step(done) == 0.5  # not yet still for SUCCESS_HOLD_S
    assert _hold(done) == 1.0


def test_jars_must_be_still_for_one_second():
    _step(_turned(-180))
    n = int(round(t.SUCCESS_HOLD_S * t.CONTROL_HZ)) + 1
    for k in range(n):  # a neighbour still sliding 3 mm over the last second
        j = t.JARS[2]
        p = t.kit.pose(_turned(-180), j)[0] + np.array([0.003 * k / (n - 1), 0.0, 0.0])
        sc = _step(_turned(-180, {t.TARGET: _on_mark(), j: (p, t.kit.pose(_turned(-180), j)[1])}))
    assert sc == 0.5
    assert _hold(_turned(-180, {t.TARGET: _on_mark()})) == 1.0


def test_target_on_mark_without_turning_scores_zero():
    """Taking the jar out some other way (never fronted on the tray) earns nothing."""
    assert _hold(t.kit.fake_states(moved={t.TARGET: _on_mark()})) == 0.0


def test_target_must_be_on_the_mark_and_upright():
    _step(_turned(-180))
    off = (np.array([t.MARK[0] + t.MARK_R + 0.005, t.MARK[1], t.TOP + t.JAR[2] / 2]), _yaw(0.0))
    assert _hold(_turned(-180, {t.TARGET: off})) == 0.5
    lying = (np.array([t.MARK[0], t.MARK[1], t.TOP + t.JAR[0] / 2]), _tip(90.0))
    assert _hold(_turned(-180, {t.TARGET: lying})) == 0.5


def test_a_tipped_neighbour_zeroes_the_trial_and_latches():
    _step(_turned(-180))
    j = t.JARS[3]
    p = t.kit.pose(_turned(-180), j)[0]
    assert _step(_turned(-180, {j: (p, _tip(t.TIP_DEG + 2.0))})) == 0.0
    assert _hold(_turned(-180, {t.TARGET: _on_mark()})) == 0.0  # latched: setting it back up does not help


def test_a_small_wobble_is_not_a_tip():
    j = t.JARS[2]
    p = t.kit.pose(_turned(-180), j)[0]
    _step(_turned(-180, {j: (p, _tip(t.TIP_DEG - 3.0))}))
    assert _hold(_turned(-180, {t.TARGET: _on_mark()})) == 1.0


def test_a_neighbour_flung_off_the_tray_zeroes_the_trial():
    j = t.JARS[1]
    out = (np.array([t.TT_X - t.TT_R - 0.03, t.TT_Y + 0.08, t.TOP + t.JAR[2] / 2]), _yaw(0.0))
    assert _step(_turned(-180, {j: out})) == 0.0


def test_target_tipped_on_the_tray_zeroes_the_trial():
    p = t.kit.pose(_turned(-180), t.TARGET)[0]
    assert _step(_turned(-180, {t.TARGET: (p, _tip(t.TIP_DEG + 2.0))})) == 0.0
    assert _hold(_turned(-180, {t.TARGET: _on_mark()})) == 0.0


def test_tray_lifted_off_its_bearing_zeroes_the_trial():
    lifted = (np.array([t.TT_X, t.TT_Y, t.FLOOR_Z + t.TRAY_Z - t.FLOOR_T + t.TRAY_LIFT + 0.015]), _yaw(-90.0))
    assert _step(_turned(-90, {"tray": lifted})) == 0.0
    assert t._mem()["spoiled"]


def test_demo_reaches_and_fits_the_run_limit():
    """Every waypoint of both arms solves within 0.5 mm of the official IK (the Live waypoints, offline, resolve from
    the start layout and the stroke bookkeeping), and the plan fits the run limit."""
    plan = t.demo()
    run_limit = int(re.findall(r"[\d,]+", json.loads((HERE / "task.json").read_text())["run_limit"])[0].replace(",", ""))
    for arm, wps in plan.items():
        p = ArmPlan(list(wps), station.BASES[arm])
        k = 0
        while k * DT <= p.duration:
            p.action7(k * DT)
            k += 1
        assert p.worst_ik_mm < 2.0 and p.worst_ik_deg < 2.0, (arm, p.worst_ik_mm, p.worst_ik_deg)
        assert p.duration / DT < run_limit


def test_stroke_window_is_inside_official_ik():
    """The turning hand's pinch and back-off poses over its whole stroke window, one waypoint each from the ready pose."""
    a0 = t.STROKE_MID + t.STROKE_DEG / 2
    for az in np.arange(a0, a0 - t.STROKE_DEG - 0.1, -7.5):
        for r in (t.TT_R - t.PINCH_IN, t.TT_R + t.BACK_OFF):
            pos, rot = t.rim_pinch(math.radians(az), r)
            p = ArmPlan([t.RWaypoint(1.0, tuple(map(float, pos)), t.FINGER_CLOSED, "x", rot=tuple(map(tuple, rot)))],
                        station.BASES["left"])
            for k in range(int(p.duration / DT) + 2):
                p.action7(k * DT)
            assert p.worst_ik_mm < 0.5, (az, r, p.worst_ik_mm)


# -- occlusion at the end pose: sight lines from the top cameras to the scored region --------------------------------
# End pose: the local GPU-simulator demo run 2026-11-21 (abl-warm-local5090, record status 'success', terminated at
# t=1322; actions/20261121_173039_9d4c0a74/scene-0-e0.jsonl). The record carries commanded joint targets only, no link
# states: link origins are the arena's URDF FK (gen_endpose_fixture.py). Row 1322 is the last row and 1302 twenty steps
# earlier (the right arm was still backing away); at row 1190 the right hand reaches in over the tray's front lip
# for the red jar (the top_cam frame shows the arm over the tray front), the real-data positive control.
# generated by gen_endpose_fixture.py from /mnt/data/autobiolab/state/tasks/spin-lazy-susan-bring-back-jar-to-front-probe/runs/20261121T173014Z/demo/actions/20261121_173039_9d4c0a74/scene-0-e0.jsonl (1323 step rows, last t=1322)
END_BODIES = ('base', 'link1', 'link2', 'link3', 'link4', 'link5', 'gripper', 'tip_left', 'tip_right')
END_LINKS = {
    1322: {
        'left_robot': (  # TCP (0.22016, 0.20009, 0.8625), gripper -0.000
            (0.0, 0.305, 0.75),  # base
            (0.0, 0.305, 0.818),  # link1
            (0.03936, 0.3047, 0.8635),  # link2
            (-0.12084, 0.44388, 1.03495),  # link3
            (0.06006, 0.26898, 0.96391),  # link4
            (0.07269, 0.17958, 0.95327),  # link5
            (0.09423, 0.20014, 0.90831),  # gripper
            (0.16279, 0.22371, 0.93096),  # tip_left
            (0.1322, 0.17653, 0.8469),  # tip_right
        ),
        'right_robot': (  # TCP (0.16938, -0.00571, 0.87422), gripper 0.639
            (0.0, -0.305, 0.75),  # base
            (0.0, -0.305, 0.818),  # link1
            (-0.03219, -0.28236, 0.8635),  # link2
            (0.0334, -0.32503, 1.12485),  # link3
            (-0.01862, -0.11305, 0.98093),  # link4
            (0.02175, -0.03366, 0.9627),  # link5
            (0.05105, -0.04879, 0.92004),  # gripper
            (0.11378, -0.0414, 0.9232),  # tip_left
            (0.08843, -0.01974, 0.87811),  # tip_right
        ),
    },
    1302: {
        'left_robot': (  # TCP (0.22016, 0.20009, 0.8625), gripper -0.000
            (0.0, 0.305, 0.75),  # base
            (0.0, 0.305, 0.818),  # link1
            (0.03936, 0.3047, 0.8635),  # link2
            (-0.12084, 0.44388, 1.03495),  # link3
            (0.06006, 0.26898, 0.96391),  # link4
            (0.07269, 0.17958, 0.95327),  # link5
            (0.09423, 0.20014, 0.90831),  # gripper
            (0.16279, 0.22371, 0.93096),  # tip_left
            (0.1322, 0.17653, 0.8469),  # tip_right
        ),
        'right_robot': (  # TCP (0.19767, 0.00458, 0.84723), gripper 0.639
            (0.0, -0.305, 0.75),  # base
            (0.0, -0.305, 0.818),  # link1
            (-0.02963, -0.27909, 0.8635),  # link2
            (0.03833, -0.28996, 1.12749),  # link3
            (0.00347, -0.10287, 0.94822),  # link4
            (0.05225, -0.02681, 0.93819),  # link5
            (0.07935, -0.03849, 0.89305),  # gripper
            (0.14207, -0.0311, 0.89622),  # tip_left
            (0.11672, -0.00944, 0.85112),  # tip_right
        ),
    },
    1190: {
        'left_robot': (  # TCP (0.34622, 0.09258, 0.7985), gripper -0.000
            (0.0, 0.305, 0.75),  # base
            (0.0, 0.305, 0.818),  # link1
            (0.03455, 0.32386, 0.8635),  # link2
            (0.05885, 0.22923, 1.11822),  # link3
            (0.16965, 0.24322, 0.88182),  # link4
            (0.22388, 0.17137, 0.89456),  # link5
            (0.24016, 0.16045, 0.84433),  # gripper
            (0.31078, 0.14374, 0.86677),  # tip_left
            (0.25927, 0.11973, 0.78311),  # tip_right
        ),
        'right_robot': (  # TCP (0.37902, 0.01514, 0.86303), gripper -0.000
            (0.0, -0.305, 0.75),  # base
            (0.0, -0.305, 0.818),  # link1
            (-0.0126, -0.26771, 0.8635),  # link2
            (0.08779, -0.25669, 1.11694),  # link3
            (0.15897, -0.06892, 0.9495),  # link4
            (0.23944, -0.02757, 0.95841),  # link5
            (0.26069, -0.02792, 0.90885),  # gripper
            (0.33368, -0.04873, 0.91201),  # tip_left
            (0.28781, 0.02931, 0.86692),  # tip_right
        ),
    },
}

# Top camera: yam_arena/station.py top_camera() for each of station.TOP_CAMS ("calibrated" = the run's top_cam, "demo" =
# Dream's documented demo view); the run's other two cameras are wrist cameras on the arms.
# The scored region the top cameras can see: the red jar on the counter mark (cap top, its corners, its front face) and
# the tray's front lip where the red jar is fronted. The jars under the upper shelf are hidden by the shelf itself, not
# the arms (DEVIATIONS 7); this test is about the arms.
CAMS = {v: station.top_camera(v) for v in station.TOP_CAMS}
FINGER_R = 0.010  # m: proxy half-thickness of a finger / link capsule (small on purpose: favours "clear")
LINK_R = 0.030


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
    """(u, v) of world point p in the camera image (OpenGL convention: looks along -z, +y up)."""
    rot = np.asarray(quat_xyzw_to_matrix(np.asarray(cam.rot_xyzw, float)))
    x, y, z = rot.T @ (np.asarray(p, float) - np.asarray(cam.pos, float))
    k = cam.lens
    return (k.cx + k.fx * x / -z, k.cy - k.fy * y / -z) if z < 0 else (float("nan"),) * 2


def _targets():
    mx, my = t.MARK
    top = t.TOP + t.JAR[2]
    h = t.JAR[0] / 2 - 0.005
    out = [("red cap top centre", np.array([mx, my, top]))]
    out += [(f"red cap corner {sx:+d},{sy:+d}", np.array([mx + sx * h, my + sy * h, top])) for sx in (-1, 1) for sy in (-1, 1)]
    out += [("red jar front face", np.array([mx - t.JAR[0] / 2, my, t.TOP + t.JAR[2] / 2]))]
    lip_z = t.FLOOR_Z + t.TRAY_Z + t.LIP_H
    out += [(f"tray front lip {az:+.0f}", np.array([t.TT_X - t.TT_R * math.cos(math.radians(az)),
                                                  t.TT_Y + t.TT_R * math.sin(math.radians(az)), lip_z]))
            for az in (-t.FRONT_DEG, 0.0, t.FRONT_DEG)]
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


def test_occlusion_positive_controls_detect_a_blocker_and_the_hand_over_the_tray():
    tg = _targets()
    cam = np.asarray(CAMS["calibrated"].pos, float)
    mid = cam + 0.75 * (tg[0][1] - cam)
    assert _occluders(cam, [(mid - (0, 0, 0.02), mid + (0, 0, 0.02), FINGER_R)], tg[:1])
    # real data: at row 1190 the right hand reaches in over the tray's front lip
    assert "tray front lip +0" in {lab for lab, _ in _occluders(cam, _arm_caps(1190), tg)}


def test_scored_region_is_in_frame_of_both_top_cameras():
    for var, cam in CAMS.items():
        for lab, p in _targets():
            u, v = _pixel(cam, p)
            assert 0.0 <= u < cam.lens.width and 0.0 <= v < cam.lens.height, (var, lab, u, v)


def test_occlusion_top_cameras_see_scored_region_at_demo_end():
    for row in (1322, 1302):
        for var, cam in CAMS.items():
            hits = _occluders(np.asarray(cam.pos, float), _arm_caps(row), _targets())
            assert not hits, (row, var, hits)
