"""Offline CPU tests for push-tv-back-from-console-edge-without-tipping: the score bookkeeping driven step by step with
posed states (no simulator), the zero latches, the score tiers, the one-second stillness rule, the text == constants
rule, the demo's reach and length, and the top cameras' view of the scored region at the end of the GPU-simulator demo."""

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
import push_tv_back_from_console_edge_without_tipping as t  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.urdf import quat_xyzw_to_matrix  # noqa: E402

DT = 0.05
TASK = json.loads((HERE / "task.json").read_text())


@pytest.fixture(autouse=True)
def _fresh_episode():
    mk.new_episode()
    yield
    mk.new_episode()


def _q(axis, deg):
    a = math.radians(deg) / 2
    s = math.sin(a)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(a))


def _at(dx=0.0, dy=0.0, dz=0.0, quat=(0.0, 0.0, 0.0, 1.0)):
    """The TV moved dx back from its start (and turned by quat about its frame origin)."""
    return t.kit.fake_states(moved={"tv": (np.array([t.TV_X + dx, dy, t.TOP + dz]), quat)})


def _step(objects):
    t.tv_step(objects)
    return t.score(objects)


def _hold(objects, steps=None):
    n = int(round(t.SUCCESS_HOLD_S * t.CONTROL_HZ)) + 1 if steps is None else steps
    for _ in range(n):
        sc = _step(objects)
    return sc


def test_start_layout():
    s = t.kit.fake_states()
    assert abs(t.setback(s) - t.START_SETBACK) < 1e-9
    assert t.tilt_deg(s) < 1e-6 and t.yaw_deg(s) < 1e-6 and abs(t.lift(s)) < 1e-9


def test_holding_still_scores_zero():
    assert _hold(t.kit.fake_states()) == 0.0


def test_pushed_back_and_left_still_scores_one():
    done = _at(t.SETBACK - t.START_SETBACK + 0.02)
    assert _step(done) == 0.5  # not yet still for SUCCESS_HOLD_S
    assert _hold(done) == 1.0


def test_setback_edge():
    assert _hold(_at(t.SETBACK - t.START_SETBACK - 0.002)) == 0.0
    mk.new_episode()
    assert _hold(_at(t.SETBACK - t.START_SETBACK + 0.002)) == 1.0


def test_must_be_still_for_one_second():
    base = t.SETBACK - t.START_SETBACK + 0.02
    n = int(round(t.SUCCESS_HOLD_S * t.CONTROL_HZ)) + 1
    for k in range(n):  # still sliding 3 mm over the last second
        sc = _step(_at(base + 0.003 * k / (n - 1)))
    assert sc == 0.5
    assert _hold(_at(base)) == 1.0


def test_shortcut_screen_push_tipping_back_onto_the_wall_scores_zero():
    """The screen-centre push (dev variant: 0 in local Isaac and MuJoCo) leaves the TV leaning back on the wall, well
    past the setback line: a tilt past TIP_DEG zeroes it however far back the feet went."""
    dx = t.SETBACK - t.START_SETBACK + 0.05
    for k in range(10):
        _step(_at(dx * k / 9, quat=_q((0.0, 1.0, 0.0), 45.0 * k / 9)))
    assert _hold(_at(dx, quat=_q((0.0, 1.0, 0.0), 45.0))) == 0.0
    assert t._mem()["spoiled"]


def test_tipped_tv_zeroes_and_latches():
    dx = t.SETBACK - t.START_SETBACK + 0.02
    assert _step(_at(dx, quat=_q((0.0, 1.0, 0.0), t.TIP_DEG + 1.0))) == 0.0
    assert _hold(_at(dx)) == 0.0  # set back upright: still 0


def test_small_rock_is_not_a_tip_but_must_settle_upright():
    dx = t.SETBACK - t.START_SETBACK + 0.02
    assert _step(_at(dx, quat=_q((0.0, 1.0, 0.0), t.TIP_DEG - 2.0))) == 0.0  # rocking: not upright
    assert _hold(_at(dx)) == 1.0


def test_lifted_tv_zeroes_and_latches():
    dx = t.SETBACK - t.START_SETBACK + 0.02
    assert _step(_at(dx, dz=t.LIFT_M + 0.005)) == 0.0
    assert _hold(_at(dx)) == 0.0


def test_foot_over_the_front_edge_zeroes_and_latches():
    assert _step(_at(-t.START_SETBACK - 0.005)) == 0.0
    assert _hold(_at(t.SETBACK - t.START_SETBACK + 0.02)) == 0.0


def test_turned_tv_is_not_placed():
    dx = t.SETBACK - t.START_SETBACK + 0.06
    assert _hold(_at(dx, quat=_q((0.0, 0.0, 1.0), t.YAW_DEG + 2.0))) == 0.0  # turned past YAW_DEG: not placed
    mk.new_episode()
    assert _hold(_at(dx, quat=_q((0.0, 0.0, 1.0), t.YAW_DEG - 2.0))) == 1.0


def test_one_foot_short_is_not_placed():
    """Turned so that one foot's front is behind the setback line and the other short of it."""
    dx = t.SETBACK - t.START_SETBACK + 0.005
    s = _at(dx, quat=_q((0.0, 0.0, 1.0), 5.0))
    assert t.setback(s) < t.SETBACK and t.yaw_deg(s) < t.YAW_DEG
    assert _hold(s) == 0.0


def test_text_equals_constants():
    sc = TASK["scoring"]
    for txt in (f"{t.TIP_DEG:.0f} degrees", f"{t.LIFT_M * 100:.0f} cm off", f"{t.SETBACK * 100:.0f} cm behind",
                f"within {t.UPRIGHT_DEG:.0f} degrees", f"within {t.YAW_DEG:.0f} degrees", f"under {t.STILL_MM:.0f} mm",
                f"under {t.STILL_DEG:.0f} degree"):
        assert txt in sc, txt
    assert f"at least {t.SETBACK * 100:.0f} cm" in TASK["instruction"]
    for txt in ("73.3 x 46.3 x 16.6", f"{t.TV_M:.1f} kg", "68.4 cm", f"x {t.EDGE_X * 100:.0f}", f"x {t.WALL_X * 100:.0f}",
                f"{t.START_SETBACK * 100:.1f} cm behind"):
        assert txt in TASK["setup"], txt


def test_screen_centre_is_above_the_tipping_height_and_the_push_below_it():
    """Static tipping rule: pushed at height h it tips over the rear foot edge before sliding when h > (D / 2) / mu."""
    centre = t.STAND_H + t.PANEL_H / 2
    for mu in (0.4, 0.6, 0.85):
        h_tip = (t.SET_D / 2) / mu
        assert centre > h_tip
        assert t.PUSH_Z - t.TOP < h_tip / 1.5


def test_demo_reaches_and_fits_the_run_limit():
    plan = t.demo()
    run_limit = int(re.findall(r"[\d,]+", TASK["run_limit"])[0].replace(",", ""))
    for arm, wps in plan.items():
        p = ArmPlan(list(wps), station.BASES[arm])
        k = 0
        while k * DT <= p.duration:
            p.action7(k * DT)
            k += 1
        assert p.worst_ik_mm < 2.0 and p.worst_ik_deg < 2.0, (arm, p.worst_ik_mm, p.worst_ik_deg)
        assert p.duration / DT < run_limit


def test_demo_pushes_low_above_the_feet():
    for arm, wps in t.demo().items():
        tips = [np.asarray(w.pos) + np.asarray(w.rot)[:, 2] * t.PAD_TIP_PAST_TCP for w in wps if "push" in w.label]
        assert tips
        for tip in tips:
            assert abs(tip[2] - t.PUSH_Z) < 1e-9
            assert t.FOOT_Y - t.FOOT_W / 2 <= abs(tip[1]) <= t.FOOT_Y + t.FOOT_W / 2


# -- occlusion at the end pose: sight lines from the top cameras to the scored region ----------------------------------
# End pose: the local GPU-simulator demo run 2026-11-21 (abl-warm-local5090, probe record 'success', last row t=269;
# actions/20261121_191057_a7714335/scene-0-e0.jsonl). The record carries commanded joint targets only, no link states:
# link origins are the arena's URDF FK (dev/gen_endpose_fixture.py). Row 269 is the last row and 249 twenty steps earlier;
# at row 160 both hands push the bottom bezel just above the feet' front ends (the real-data positive control).
# generated by gen_endpose_fixture.py from /mnt/data/autobiolab/state/tasks/push-tv-back-from-console-edge-without-tipping-probe/runs/20261121T191041Z/demo/actions/20261121_191057_a7714335/scene-0-e0.jsonl (270 step rows, last t=269)
END_BODIES = ('base', 'link1', 'link2', 'link3', 'link4', 'link5', 'gripper', 'tip_left', 'tip_right')
END_LINKS = {
    269: {
        'left_robot': (  # TCP (0.41044, 0.41997, 0.86014), gripper -0.000
            (0.0, 0.305, 0.75),  # base
            (0.0, 0.305, 0.818),  # link1
            (0.00407, 0.34415, 0.8635),  # link2
            (0.04394, 0.28673, 1.12721),  # link3
            (0.17449, 0.42217, 0.94563),  # link4
            (0.26475, 0.42622, 0.95574),  # link5
            (0.28452, 0.41996, 0.90596),  # gripper
            (0.353, 0.44396, 0.92841),  # tip_left
            (0.32256, 0.39597, 0.84475),  # tip_right
        ),
        'right_robot': (  # TCP (0.41044, -0.41997, 0.86014), gripper -0.000
            (0.0, -0.305, 0.75),  # base
            (0.0, -0.305, 0.818),  # link1
            (0.0323, -0.2825, 0.8635),  # link2
            (0.01488, -0.3502, 1.12721),  # link3
            (0.20271, -0.36053, 0.94563),  # link4
            (0.26475, -0.42622, 0.95574),  # link5
            (0.28452, -0.41996, 0.90596),  # gripper
            (0.35301, -0.39597, 0.9284),  # tip_left
            (0.32256, -0.44395, 0.84475),  # tip_right
        ),
    },
    249: {
        'left_robot': (  # TCP (0.41944, 0.35408, 0.85114), gripper -0.000
            (0.0, 0.305, 0.75),  # base
            (0.0, 0.305, 0.818),  # link1
            (0.01337, 0.34202, 0.8635),  # link2
            (0.03804, 0.27666, 1.12722),  # link3
            (0.18559, 0.37452, 0.93484),  # link4
            (0.27388, 0.35668, 0.94712),  # link5
            (0.29352, 0.35407, 0.89696),  # gripper
            (0.362, 0.37807, 0.91941),  # tip_left
            (0.33156, 0.33008, 0.83575),  # tip_right
        ),
        'right_robot': (  # TCP (0.41944, -0.35408, 0.85114), gripper -0.000
            (0.0, -0.305, 0.75),  # base
            (0.0, -0.305, 0.818),  # link1
            (0.02594, -0.2754, 0.8635),  # link2
            (0.0251, -0.34525, 1.12722),  # link3
            (0.19816, -0.30789, 0.93484),  # link4
            (0.27388, -0.35668, 0.94712),  # link5
            (0.29352, -0.35407, 0.89696),  # gripper
            (0.362, -0.33008, 0.91941),  # tip_left
            (0.33156, -0.37807, 0.83575),  # tip_right
        ),
    },
    160: {
        'left_robot': (  # TCP (0.45126, 0.27355, 0.80015), gripper -0.000
            (0.0, 0.305, 0.75),  # base
            (0.0, 0.305, 0.818),  # link1
            (0.02352, 0.33656, 0.8635),  # link2
            (0.10924, 0.25813, 1.11034),  # link3
            (0.22642, 0.3147, 0.88356),  # link4
            (0.30572, 0.27206, 0.89617),  # link5
            (0.32534, 0.27355, 0.84596),  # gripper
            (0.39382, 0.29754, 0.86841),  # tip_left
            (0.36338, 0.24956, 0.78475),  # tip_right
        ),
        'right_robot': (  # TCP (0.45126, -0.27355, 0.80015), gripper -0.000
            (0.0, -0.305, 0.75),  # base
            (0.0, -0.305, 0.818),  # link1
            (0.01625, -0.26915, 0.8635),  # link2
            (0.11672, -0.32753, 1.11034),  # link3
            (0.21916, -0.24729, 0.88356),  # link4
            (0.30572, -0.27206, 0.89617),  # link5
            (0.32534, -0.27355, 0.84596),  # gripper
            (0.39382, -0.24956, 0.86841),  # tip_left
            (0.36338, -0.29754, 0.78475),  # tip_right
        ),
    },
}

# Top camera: yam_arena/station.py top_camera() for each of station.TOP_CAMS; the run's other two cameras are wrist cameras.
# The scored region: the front ends of both feet (top front corners, where the TV ends after the demo's push) and the tape
# at the front edge in line with them.
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
    xf = t.EDGE_X + t.START_SETBACK + t.PUSH_D  # feet front after the demo's push
    out = [(f"foot front {s * t.FOOT_Y + d * t.FOOT_W / 2:+.3f}", np.array([xf, s * t.FOOT_Y + d * t.FOOT_W / 2, t.TOP + t.STAND_H]))
           for s in (-1, 1) for d in (-1, 1)]
    out += [(f"edge tape {y:+.2f}", np.array([t.EDGE_X - 0.01, y, t.TOP])) for y in (-t.FOOT_Y, 0.0, t.FOOT_Y)]
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


def test_occlusion_positive_controls_detect_a_blocker_and_the_pushing_hands():
    tg = _targets()
    cam = np.asarray(CAMS["calibrated"].pos, float)
    mid = cam + 0.75 * (tg[0][1] - cam)
    assert _occluders(cam, [(mid - (0, 0, 0.02), mid + (0, 0, 0.02), FINGER_R)], tg[:1])
    assert any(lab.startswith("foot front") for lab, _ in _occluders(cam, _arm_caps(160), tg))  # real data: hands on the bezel


def test_scored_region_is_in_frame_of_both_top_cameras():
    for var, cam in CAMS.items():
        for lab, p in _targets():
            u, v = _pixel(cam, p)
            assert 0.0 <= u < cam.lens.width and 0.0 <= v < cam.lens.height, (var, lab, u, v)


def test_occlusion_top_cameras_see_scored_region_at_demo_end():
    for row in (269, 249):
        for var, cam in CAMS.items():
            hits = _occluders(np.asarray(cam.pos, float), _arm_caps(row), _targets())
            assert not hits, (row, var, hits)
