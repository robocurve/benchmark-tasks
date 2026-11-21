"""Offline CPU tests for plasma-cut-installed-panel-along-straightedge: the cut bookkeeping driven step by step with
posed states (no simulator), the score tiers, the zero latches, the demo's reach, and the top cameras' view of the
scored region at the demo's end pose."""

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
import plasma_cut_installed_panel_along_straightedge as t  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.urdf import quat_xyzw_to_matrix  # noqa: E402

UP = (0.0, 0.0, 0.0, 1.0)
FAR_HAND = np.array([0.20, 0.30, t.TOP + 0.20])  # the straightedge hand, well clear of the nozzle


@pytest.fixture(autouse=True)
def _fresh_episode():
    mk.new_episode()
    yield
    mk.new_episode()


def _torch_origin(rim, quat=UP):
    """Torch frame origin that puts the shield rim's centre at ``rim`` for the torch turned to ``quat``."""
    return np.asarray(rim, float) - mk.quat_to_mat(quat) @ np.array([0.0, t.HEAD_DY, -t.RIM_DZ])


def _tilt_quat(deg):
    """Torch turned ``deg`` about x."""
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def _states(rim=None, torch_quat=UP, se_edge_y=t.EDGE_Y, se_z=None, strip=None, torch_at_rest=False):
    moved = {"straightedge": ((t.XM, se_edge_y, (t.PZ + t.SE_T / 2) if se_z is None else se_z), UP)}
    if rim is not None and not torch_at_rest:
        moved["torch"] = (_torch_origin(rim, torch_quat), torch_quat)
    if strip is not None:
        moved["strip"] = (np.asarray(strip, float), UP)
    return t.kit.fake_states(moved=moved)


def _hands(objects):
    """A torch hand on the grip block and the other hand far away (left, right order)."""
    grip = t.kit.pose(objects, "torch")[0] + np.array([0.0, 0.0, t.GRIP_DZ + t.TIP])
    return [FAR_HAND, grip]


def _drag(speed, y=t.LINE_Y, rim_dz=0.0, tilt=0.0, se_edge_y=t.EDGE_Y, stop_x=None, se_z=None):
    """Pull the rim from beyond the far edge to beyond the near edge at ``speed``, one control step at a time."""
    x = t.X1 + 0.003
    end = (t.X0 - 0.003) if stop_x is None else stop_x
    while x > end:
        objects = _states((x, y, t.PZ + rim_dz), _tilt_quat(tilt), se_edge_y=se_edge_y, se_z=se_z)
        t.cut_step(objects, _hands(objects))
        x -= speed * t.DT
    return t._state()


def _rest_states(strip_in_pan=True):
    """Both tools back where they started; the strip in the pan (or still on its weld)."""
    strip = ((t.PAN_X0 + t.PAN_X1) / 2, (t.PAN_Y0 + t.PAN_Y1) / 2, t.TOP + 0.005) if strip_in_pan else None
    return t.kit.fake_states(moved={} if strip is None else {"strip": (np.asarray(strip), UP)})


def test_start_layout_scores_zero_and_arc_is_off():
    objects = t.kit.fake_states()
    hands = [FAR_HAND, FAR_HAND + np.array([0.0, -0.5, 0.0])]
    assert not t.arc_on(objects, hands)
    t.cut_step(objects, hands)
    assert t.score(objects) == 0.0  # holding still scores 0


def test_rated_pass_along_the_straightedge_is_all_sound_and_releases_the_strip():
    m = _drag(t.V_RATED)
    assert np.all(m["t"] >= t.T_CUT) and np.all(m["t"] <= t.T_HOT)
    assert t.sound(m).all()
    assert t.SEAM in t.kit._released
    assert t.score(_rest_states(strip_in_pan=False)) == 0.8  # strip not in the pan yet


def test_score_tiers_after_a_sound_cut():
    _drag(t.V_RATED)
    in_pan = _rest_states()
    hands_clear = [np.array([0.20, 0.45, t.TOP + 0.20]), np.array([0.20, -0.45, t.TOP + 0.20])]
    assert t.strip_in_pan(in_pan) and t.tools_back(in_pan)
    assert t.hands_clear(in_pan, hands_clear)
    # score() reads the live TCPs (none offline): offline the hands count as clear
    assert t.score(in_pan) == 1.0
    moved_torch = t.kit.fake_states(moved={"strip": (t.kit.pose(in_pan, "strip")[0], UP),
                                           "torch": (np.array([t.REST[0] + 0.03, t.REST[1], t.REST_Z]), UP)})
    assert not t.tools_back(moved_torch)
    assert t.score(moved_torch) == 0.9


def test_straightedge_must_stay_where_it_was_laid_for_10():
    _drag(t.V_RATED)
    strip = (np.asarray(((t.PAN_X0 + t.PAN_X1) / 2, (t.PAN_Y0 + t.PAN_Y1) / 2, t.TOP + 0.005)), UP)
    se0 = np.array([t.SE_START[0], t.SE_START[1], t.SE_Z0])
    pushed = t.kit.fake_states(moved={"strip": strip, "straightedge": (se0 + np.array([0.0, 0.025, 0.0]), UP)})
    assert not t.tools_back(pushed)  # shoved 25 mm across the deck
    assert t.score(pushed) == 0.9
    nudged = t.kit.fake_states(moved={"strip": strip, "straightedge": (se0 + np.array([0.010, 0.010, 0.0]), UP)})
    assert t.tools_back(nudged) and t.score(nudged) == 1.0  # 14 mm in plan: inside the 20 mm tolerance


def test_too_fast_does_not_cut_through_and_the_strip_stays():
    m = _drag(0.030)  # 30 mm/s, over the 20.75 mm/s maximum
    assert not np.any(m["t"] >= t.T_CUT)
    assert t.SEAM not in t.kit._released
    assert t.score(_rest_states(strip_in_pan=False)) == 0.0


def test_too_slow_overheats_every_segment():
    m = _drag(0.005)
    assert np.all(m["t"] > t.T_HOT)
    assert t.SEAM in t.kit._released  # cut through, the strip drops ...
    assert t.score(_rest_states()) == 0.0  # ... but no segment is sound


def test_freehand_cut_off_the_straightedge_is_not_sound():
    m = _drag(t.V_RATED, se_edge_y=t.EDGE_Y + 0.004)  # straightedge 4 mm away from the shield
    assert np.all(m["t"] >= t.T_CUT) and not t.sound(m).any()


def test_straightedge_not_flat_on_the_deck_does_not_guide():
    m = _drag(t.V_RATED, se_z=t.PZ + t.SE_T / 2 + 0.003)  # held 3 mm up off the deck
    assert not t.sound(m).any()


def test_off_the_line_and_out_of_square_are_not_sound():
    m = _drag(t.V_RATED, y=t.LINE_Y - 0.003, se_edge_y=t.EDGE_Y - 0.003)  # guided, but 3 mm off the line
    assert np.all(m["off_line"]) and not t.sound(m).any()
    mk.new_episode()
    m = _drag(t.V_RATED, tilt=12.0)
    assert np.all(m["tilted"]) and not t.sound(m).any()


def test_arc_needs_the_shield_on_the_deck():
    objects = _states((0.27, t.LINE_Y, t.PZ + 0.004))  # 4 mm up: stretched arc, no cut
    assert not t.arc_on(objects, _hands(objects))
    objects = _states((0.27, t.LINE_Y, t.PZ + 0.0015))
    assert t.arc_on(objects, _hands(objects))
    assert not t.arc_on(objects, [FAR_HAND, FAR_HAND])  # torch not in a hand


def test_half_cut_keeps_the_strip_and_scores_the_sound_half():
    m = _drag(t.V_RATED, stop_x=(t.X0 + t.X1) / 2)
    assert t.SEAM not in t.kit._released
    assert t.score(_rest_states(strip_in_pan=False)) == pytest.approx(0.8 * t.sound(m).mean())
    assert 0.3 <= t.score(_rest_states(strip_in_pan=False)) <= 0.45


def test_other_hand_near_the_arc_zeroes_the_trial():
    objects = _states((0.27, t.LINE_Y, t.PZ))
    near = t.nozzle(objects) + np.array([0.0, 0.03, 0.02])  # 36 mm from the nozzle
    t.cut_step(objects, [near, _hands(objects)[1]])
    _drag(t.V_RATED)
    assert t._state()["hand_near"] and t.score(_rest_states()) == 0.0


def test_a_tool_off_the_table_zeroes_the_trial():
    _drag(t.V_RATED)
    off = t.kit.fake_states(moved={"straightedge": ((0.27, 0.6, t.TOP - 0.3), UP)})
    t.cut_step(off, [FAR_HAND, FAR_HAND])
    assert t.score(_rest_states()) == 0.0


def test_strip_must_be_in_the_pan_for_09():
    _drag(t.V_RATED)
    on_table = t.kit.fake_states(moved={"strip": (np.array([0.27, -0.30, t.TOP + 0.002]), UP)})
    assert not t.strip_in_pan(on_table)
    assert t.score(on_table) == 0.8


def test_cut_speeds_match_the_manual():
    assert t.V_MAX == pytest.approx(1245 / 60 / 1000)
    assert t.V_RATED == pytest.approx(762 / 60 / 1000)
    assert t.T_CUT == pytest.approx(0.010 / t.V_MAX)
    assert t.T_HOT == pytest.approx(0.010 / (t.V_RATED / 2))


def test_demo_reaches_and_fits_the_run_limit():
    plan = t.demo()
    run_limit = int(re.findall(r"[\d,]+", json.loads((HERE / "task.json").read_text())["run_limit"])[0].replace(",", ""))
    for arm, wps in plan.items():
        p = ArmPlan(list(wps), station.BASES[arm])
        k = 0
        while k * t.DT <= p.duration:
            p.action7(k * t.DT)
            k += 1
        assert p.worst_ik_mm < 2.0 and p.worst_ik_deg < 2.0, arm
        assert p.duration / t.DT < run_limit


# -- occlusion at the end pose: sight lines from the top cameras to the scored region --------------------------------
# End pose: the cloud run's demo record (runs/20261121T081419Z-verified, demo/actions/20261121_081508_f6f90c91/
# scene-0-e0.jsonl; record.json terminates 'success' at t=857). The record carries commanded joint targets only, no
# link states: link origins are the arena's URDF FK (gen_endpose_fixture.py, line below). Row 857 is the last row
# and 837 twenty steps earlier (the right arm was still rising to its last target until ~row 848, so the lagging
# real arm lies between the two); row 720 carries the torch over its rest (the top_cam frame 721 shows the right arm
# over the rest), the real-data positive control.
# generated by gen_endpose_fixture.py from /mnt/data/autobiolab/state/tasks/plasma-cut-installed-panel-along-straightedge/runs/20261121T081419Z-verified/demo/actions/20261121_081508_f6f90c91/scene-0-e0.jsonl (858 step rows, last t=857)
END_BODIES = ('base', 'link1', 'link2', 'link3', 'link4', 'link5', 'gripper', 'tip_left', 'tip_right')
END_LINKS = {
    857: {
        'left_robot': (  # TCP (0.29497, 0.02503, 0.90662), gripper 0.520
            (0.0, 0.305, 0.75),  # base
            (0.0, 0.305, 0.818),  # link1
            (0.03784, 0.31582, 0.8635),  # link2
            (0.0941, 0.16757, 1.08551),  # link3
            (0.31829, 0.04963, 1.15022),  # link4
            (0.32433, -0.00284, 1.07622),  # link5
            (0.29496, 0.02504, 1.04062),  # gripper
            (0.31896, 0.00492, 0.98394),  # tip_left
            (0.27097, 0.04515, 0.98393),  # tip_right
        ),
        'right_robot': (  # TCP (0.24997, -0.25001, 0.96245), gripper 0.639
            (0.0, -0.305, 0.75),  # base
            (0.0, -0.305, 0.818),  # link1
            (0.01225, -0.26759, 0.8635),  # link2
            (0.02335, -0.3356, 1.12747),  # link3
            (0.24266, -0.2169, 1.20604),  # link4
            (0.28951, -0.24131, 1.13205),  # link5
            (0.24996, -0.25001, 1.09645),  # gripper
            (0.26448, -0.22601, 1.03976),  # tip_left
            (0.23545, -0.274, 1.03976),  # tip_right
        ),
    },
    837: {
        'left_robot': (  # TCP (0.29497, 0.02503, 0.90662), gripper 0.426
            (0.0, 0.305, 0.75),  # base
            (0.0, 0.305, 0.818),  # link1
            (0.03784, 0.31582, 0.8635),  # link2
            (0.0941, 0.16757, 1.08551),  # link3
            (0.31829, 0.04963, 1.15022),  # link4
            (0.32433, -0.00284, 1.07622),  # link5
            (0.29496, 0.02504, 1.04062),  # gripper
            (0.31896, 0.00053, 0.98394),  # tip_left
            (0.27097, 0.04954, 0.98393),  # tip_right
        ),
        'right_robot': (  # TCP (0.24999, -0.25, 0.92558), gripper 0.639
            (0.0, -0.305, 0.75),  # base
            (0.0, -0.305, 0.818),  # link1
            (0.01225, -0.26759, 0.8635),  # link2
            (0.01457, -0.33753, 1.12719),  # link3
            (0.2427, -0.2169, 1.16918),  # link4
            (0.28954, -0.2413, 1.09518),  # link5
            (0.24999, -0.25, 1.05958),  # gripper
            (0.2645, -0.22601, 1.0029),  # tip_left
            (0.23548, -0.274, 1.0029),  # tip_right
        ),
    },
    720: {
        'left_robot': (  # TCP (0.29497, 0.02503, 0.90037), gripper -0.000
            (0.0, 0.305, 0.75),  # base
            (0.0, 0.305, 0.818),  # link1
            (0.03784, 0.31582, 0.8635),  # link2
            (0.09275, 0.16884, 1.08669),  # link3
            (0.31829, 0.04963, 1.14397),  # link4
            (0.32433, -0.00284, 1.06997),  # link5
            (0.29496, 0.02504, 1.03437),  # gripper
            (0.31896, -0.01948, 0.97769),  # tip_left
            (0.27097, 0.06954, 0.97768),  # tip_right
        ),
        'right_robot': (  # TCP (0.18393, -0.18363, 0.9387), gripper -0.000
            (0.0, -0.305, 0.75),  # base
            (0.0, -0.305, 0.818),  # link1
            (-0.00198, -0.26569, 0.8635),  # link2
            (-0.00161, -0.34788, 1.12364),  # link3
            (0.16525, -0.15532, 1.1823),  # link4
            (0.21772, -0.16131, 1.1083),  # link5
            (0.18392, -0.18362, 1.0727),  # gripper
            (0.22844, -0.15963, 1.01602),  # tip_left
            (0.13941, -0.20761, 1.01602),  # tip_right
        ),
    },
}

# Top camera: yam_arena/station.py:99 TOP_CAM_POS (calibration frame) -> from_calibration (station.py:42) in
# top_camera() (station.py:118-126); the run used variant "calibrated" (runs-*-demo.out: top_cam TiledCameraCfg
# pos=(-0.16649, -0.009, 1.69182)). Second camera: top_camera("demo"), Dream's documented demo view (15 cm lower,
# 5 deg up); the run's other two cameras are wrist cameras on the arms, whose view at the end is set by the arm.
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
    """(u, v) of world point p in the 640 x 360 image of an OpenGL-convention camera (looks along -z, +y up)."""
    rot = np.asarray(quat_xyzw_to_matrix(np.asarray(cam.rot_xyzw, float)))
    x, y, z = rot.T @ (np.asarray(p, float) - np.asarray(cam.pos, float))
    k = cam.lens
    return (k.cx + k.fx * x / -z, k.cy - k.fy * y / -z) if z < 0 else (float("nan"),) * 2


def _cut_line():
    """The white line's 14 segment centres on the kept deck edge (the strip has dropped)."""
    return [(f"cut seg {k}", np.array([t.X0 + (k + 0.5) * t.SEG, t.KEEP_Y0 + 0.0005, t.PZ + 0.0003]))
            for k in range(t.N_SEG)]


def _targets():
    s0 = t.kit.fake_states()  # torch and straightedge at their start poses (the 'success' row has them back)
    out = _cut_line()
    out += [("torch shield rim", t.nozzle(s0)),
            ("torch grip block top", t.kit.point(s0, "torch", (0.0, 0.0, t.GRIP_DZ + 0.012)))]
    out += [(f"straightedge edge x{dx:+.2f}", t.kit.point(s0, "straightedge", (dx, 0.0, t.SE_T / 2)))
            for dx in (-t.SE_L / 2 + 0.01, 0.0, t.SE_L / 2 - 0.01)]
    out += [("straightedge handle top",
             t.kit.point(s0, "straightedge", (0.0, t.HANDLE_OFF, t.SE_T / 2 + t.HANDLE)))]
    # strip: dropped straight down from under the line onto the pan floor (its pose is not in the record; the
    # 'success' row only proves it is in the pan): its four corners and centre
    z = t.TOP + 0.0015 + t.T / 2
    out += [(f"strip {x:.3f},{y:.3f}", np.array([x, y, z]))
            for x in (t.X0 + 0.005, t.XM, t.X1 - 0.005) for y in (t.OFF_Y - t.OFF_W / 2 + 0.005, t.OFF_Y,
                                                                    t.OFF_Y + t.OFF_W / 2 - 0.005)]
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


def test_occlusion_positive_controls_detect_a_blocker_and_the_arm_over_the_rest():
    tg = _targets()
    cam = np.asarray(CAMS["calibrated"].pos, float)
    mid = cam + 0.75 * (tg[0][1] - cam)
    assert _occluders(cam, [(mid - (0, 0, 0.02), mid + (0, 0, 0.02), FINGER_R)], tg[:1])
    # real data: at row 720 the right arm carries the torch over its rest and covers the rest's grip-block spot
    assert "torch grip block top" in {lab for lab, _ in _occluders(cam, _arm_caps(720), tg)}


def test_scored_region_is_in_frame_of_both_top_cameras():
    for var, cam in CAMS.items():
        for lab, p in _targets():
            u, v = _pixel(cam, p)
            assert 0.0 <= u < cam.lens.width and 0.0 <= v < cam.lens.height, (var, lab, u, v)


def test_occlusion_top_cameras_see_scored_region_at_demo_end():
    for row in (857, 837):
        for var, cam in CAMS.items():
            hits = _occluders(np.asarray(cam.pos, float), _arm_caps(row), _targets())
            assert not hits, (row, var, hits)
