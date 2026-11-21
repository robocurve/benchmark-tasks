"""Offline CPU tests for keep-a-station-stocked-for-a-timed-arm."""

import json
import pathlib
import re
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import keep_a_station_stocked_for_a_timed_arm as t  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import ObjectState  # noqa: E402

CRATE_PT = (t.CX, t.CY, t.TOP + t.CRATE_FLOOR + t.CUBE / 2)


def _state(pos):
    p = np.asarray(pos, float)
    h = np.full(3, t.CUBE / 2)
    z = np.zeros(3)
    return ObjectState(pos=p, quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=z, ang_vel=z.copy(), low=p - h, high=p + h)


def _start():
    """Every block where the scene spawns it."""
    return {b.name: _state(b.pos) for b in t._CUBES}


@pytest.fixture(autouse=True)
def _clear_watch():
    t._WATCH.update(touched=False, min_gap=float("inf"))
    yield
    t._WATCH.update(touched=False, min_gap=float("inf"))


def test_start_scores_zero():
    assert t.score(_start()) == 0.0 and not t.success(_start())


def test_holding_still_scores_zero():
    # The tested arm does nothing: the coworker still crates the preloaded block_1 on its first visit.
    s = _start()
    s["block_1"] = _state(CRATE_PT)
    assert t.in_crate(s["block_1"])
    assert t.crated(s) == [] and t.score(s) == 0.0


def test_preloaded_block_never_credited_and_score_is_over_five():
    s = _start()
    s["block_1"] = _state(CRATE_PT)
    for k, name in enumerate(t.SUPPLIED, start=1):
        s[name] = _state((t.CX + 0.02 * (k - 3), t.CY, CRATE_PT[2]))
        assert t.score(s) == pytest.approx(k / 5)
    assert t.success(s)
    # Four supplied blocks crated (block_1 crated too) is still 0.8, never 5/6 or 1.0.
    s["block_6"] = _state(t.MAT_SPOTS[4] + (t.TOP + t.CUBE / 2,))
    assert t.score(s) == pytest.approx(0.8) and not t.success(s)


def test_touch_zeroes_the_score():
    s = _start()
    for name in t.SUPPLIED:
        s[name] = _state(CRATE_PT)
    assert t.score(s) == 1.0
    t._WATCH["touched"] = True
    assert t.score(s) == 0.0


def test_crate_region_rejects_blocks_on_the_rim_tray_or_table():
    assert not t.in_crate(_state((t.CX, t.CY, t.TOP + t.CRATE_H + 0.016)))  # resting on top of the stack, above the rim
    assert not t.in_crate(_state((t.CX + t.CRATE_IN_X / 2 + 0.01, t.CY, CRATE_PT[2])))  # outside the walls
    assert not t.in_crate(_state((t.TX, t.TY, t.TOP + t.TRAY_FLOOR + t.CUBE / 2)))  # still on the tray


def test_coworker_visits_the_tray_on_its_fixed_clock():
    plan = ArmPlan(t.coworker_plan(), station.BASES["right"])
    closes = [g for g in (t.GRAB_T0 + k * t.CYCLE_S for k in range(t.N_VISITS))]
    for g in closes:
        pos, _rot, finger, label = plan.target(g)
        tip = np.asarray(pos) - np.array([0.0, 0.0, t.TIP])
        assert np.allclose(tip[:2], (t.TX, t.TY), atol=1e-6) and abs(tip[2] - (t.TOP + t.PINCH_TRAY)) < 1e-6
        assert "close" in label or finger == pytest.approx(t.FINGER_CLOSED)
    assert t.coworker_plan() == t.demo()["right"]  # the demo's right arm IS the coworker routine


def test_plans_reachable_and_left_places_only_between_visits():
    plans = t.demo()
    for arm in ("left", "right"):
        p = ArmPlan(plans[arm], station.BASES[arm])
        for k in range(int(p.duration / 0.05) + 2):
            p.action7(k * 0.05)
        assert p.worst_ik_mm < 2.0 and p.worst_ik_deg < 2.0, arm
    # The left tips are never over the tray while the coworker is in it (1.6 s before to 0.8 s after each close).
    left = ArmPlan(plans["left"], station.BASES["left"])
    for k in range(t.N_VISITS):
        g = t.GRAB_T0 + k * t.CYCLE_S
        for tt in np.arange(g - 1.6, g + 0.8, 0.05):
            pos = np.asarray(left.target(tt)[0])
            assert np.hypot(pos[0] - t.TX, pos[1] - t.TY) > 0.10


def test_demo_reads_no_simulator_state():
    src = (HERE / "keep_a_station_stocked_for_a_timed_arm.py").read_text()
    body = src[src.index("def demo("):]
    assert not re.search(r"object_states|\.data\.|_WATCH|env\(", body)


def test_text_matches_scorer():
    tj = json.loads((HERE / "task.json").read_text())
    assert tj["category"] == "Impact on jobs"
    assert "possible scores: 0.0, 0.2, 0.4, 0.6, 0.8, 1.0" in tj["scoring"].lower()
    for needle in ("five", "never credited", "holds still scores 0.0", "50 mm"):
        assert needle in tj["scoring"], needle
    assert (t.TOUCH, t.SUCCESS_HOLD_S, t.GRAB_T0, t.CYCLE_S, t.N_VISITS) == (0.050, 1.0, 4.0, 11.0, 8)
    assert (t.CUBE, t.CUBE_M, len(t.SUPPLIED)) == (0.0305, 0.016, 5)
    assert (t.CRATE_H, t.TRAY_H, t.TRAY_IN + 2 * t.TRAY_WALL, t.CRATE_IN_X + 2 * t.CRATE_WALL) == pytest.approx((0.051, 0.025, 0.089, 0.152))
    assert "51 mm rim" in tj["scoring"] and "89 x 89 x 25 mm" in tj["setup"] and "152 x 152 x 51 mm" in tj["setup"]


def test_shortcut_end_states_are_not_credited():
    """Shortcut end states, scored by the package's own score(): none of them reaches credit for an uncrated block."""
    rim_z = t.TOP + t.CRATE_H + t.CUBE / 2  # resting on the 51 mm rim / on top of the stack, not inside the bin
    wall_x = t.CX + t.CRATE_IN_X / 2 + t.CRATE_WALL / 2  # centred on the +x wall top
    shortcuts = {
        "all five balanced on the crate rim": [(wall_x, t.CY + 0.04 * (k - 2), rim_z) for k in range(5)],
        "all five piled above the rim over the crate": [(t.CX, t.CY, rim_z + k * t.CUBE) for k in range(5)],
        "all five left on the hand-off tray": [(t.TX, t.TY, t.TOP + t.TRAY_FLOOR + t.CUBE / 2 + k * t.CUBE)
                                               for k in range(5)],
        "all five on the table next to the crate": [(t.CX + t.CRATE_IN_X / 2 + t.CRATE_WALL + 0.03, t.CY + 0.035 * (k - 2),
                                                      t.TOP + t.CUBE / 2) for k in range(5)],
        "all five knocked off the table under the crate": [(t.CX, t.CY, station.FLOOR_Z + t.CUBE / 2)] * 5,
        "all five pushed to the staging mat spots": [m + (t.TOP + t.CUBE / 2,) for m in t.MAT_SPOTS],
    }
    for label, spots in shortcuts.items():
        s = _start()
        s["block_1"] = _state(CRATE_PT)  # the coworker crates the preloaded block whatever the tested arm does
        for name, p in zip(t.SUPPLIED, spots):
            s[name] = _state(p)
        assert t.crated(s) == [], label
        assert t.score(s) == 0.0 and not t.success(s), label
    # holding still: only the preloaded block_1 ends crated -> 0.0
    s = _start()
    s["block_1"] = _state(CRATE_PT)
    assert t.score(s) == 0.0
    # half done: two crated, three left on the rim -> 0.4, never more; one shortcut block caps a full run at 0.8
    s = _start()
    for k, name in enumerate(t.SUPPLIED):
        s[name] = _state((t.CX + 0.03 * (k - 2), t.CY, CRATE_PT[2]) if k < 2 else (wall_x, t.CY, rim_z))
    assert t.score(s) == pytest.approx(0.4)
    for name in t.SUPPLIED[2:4]:
        s[name] = _state(CRATE_PT)
    assert t.score(s) == pytest.approx(0.8) and not t.success(s)
    # positive control: the same states with the block moved inside the bin are credited (the check can say yes)
    s[t.SUPPLIED[4]] = _state(CRATE_PT)
    assert t.score(s) == 1.0 and t.success(s)


# -- occlusion: the top cameras see the scored region at the end of the run -------------------------------------------
# END pose = the last step (k=1268, success) of a local GPU-simulator scripted demo of this scene (Isaac Lab-Arena,
# RTX 5090, 2026-11-21T08:54Z), arm link and block positions in the env frame; the coworker is still on its way back to
# its wait pose then. The parked pose (a Dream-1 run lasts the full run limit) comes from the coworker's own plan below.
END_BODIES = ("base", "link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")
END_LINKS = {
    'left_robot': (
        (-0.0, 0.305, 0.75),  # base
        (0.0, 0.305, 0.818),  # link1
        (0.03699, 0.31847, 0.8635),  # link2
        (-0.01326, 0.27071, 1.12736),  # link3
        (0.22237, 0.16588, 1.17033),  # link4
        (0.23186, 0.11417, 1.09616),  # link5
        (0.20045, 0.13996, 1.06073),  # gripper
        (0.22421, 0.14108, 1.00395),  # tip_left
        (0.17624, 0.13904, 1.00414),  # tip_right
    ),
    'right_robot': (
        (-0.0, -0.305, 0.75),  # base
        (0.0, -0.305, 0.818),  # link1
        (0.03393, -0.28505, 0.8635),  # link2
        (-0.0365, -0.32467, 1.12408),  # link3
        (0.21609, -0.38601, 1.15227),  # link4
        (0.23838, -0.43712, 1.08047),  # link5
        (0.20462, -0.4184, 1.04282),  # gripper
        (0.22955, -0.4367, 0.98765),  # tip_left
        (0.18483, -0.40297, 0.98478),  # tip_right
    ),
}
END_BLOCKS = {
    'block_1': (0.26093, -0.34074, 0.76575),
    'block_2': (0.30245, -0.33963, 0.76575),
    'block_3': (0.34009, -0.33897, 0.76575),
    'block_4': (0.26235, -0.2968, 0.76575),
    'block_5': (0.30428, -0.30529, 0.76575),
    'block_6': (0.3399, -0.29796, 0.76575),
}
CAMS = {v: np.array(station.top_camera(v).pos, float) for v in station.TOP_CAMS}  # station.py top_camera()
FINGER_R, LINK_R = 0.010, 0.030  # m: capsule radii for a finger and an arm link
CLEAR = 0.040  # m: required gap between every sight line and every arm capsule


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
    p = [np.asarray(x, float) for x in links]
    return [(p[i], p[i + 1], LINK_R) for i in range(6)] + [(p[6], p[7], FINGER_R), (p[6], p[8], FINGER_R)]


def _targets(blocks):
    out = [(n, np.array(blocks[n])) for n in t.SUPPLIED]
    out += [(n + " top", np.array(blocks[n]) + (0, 0, t.CUBE / 2)) for n in t.SUPPLIED]
    zf = t.TOP + t.CRATE_FLOOR
    out += [(f"crate floor {dx},{dy}", np.array([t.CX + dx * t.CRATE_IN_X / 2, t.CY + dy * t.CRATE_IN_Y / 2, zf]))
            for dx in (-1, 0, 1) for dy in (-1, 0, 1)]
    return out


def _min_clearance(arms, targets):
    """(smallest sight-line-to-capsule gap in m, which) over both top cameras, all targets and all arm capsules."""
    return min((_seg_dist(cam, p, a, b) - r, f"{var} -> {lab}")
               for var, cam in CAMS.items() for lab, p in targets for links in arms for a, b, r in _capsules(links))


def test_seg_dist_matches_brute_force():
    rng = np.random.default_rng(0)
    s = np.linspace(0.0, 1.0, 1501)[:, None]
    for _ in range(200):
        p0, p1, q0, q1 = rng.normal(size=(4, 3))
        bf = np.min(np.linalg.norm((p0 + (p1 - p0) * s)[:, None] - (q0 + (q1 - q0) * s)[None], axis=2))
        assert bf - 5e-3 <= _seg_dist(p0, p1, q0, q1) <= bf + 1e-9


def test_occlusion_positive_control_detects_a_blocker():
    """A 40 mm finger-sized bar placed across one sight line is reported (the check can say no)."""
    lab, p = _targets(END_BLOCKS)[0]
    cam = CAMS["calibrated"]
    mid = cam + 0.75 * (p - cam)
    assert _seg_dist(cam, p, mid - (0, 0, 0.02), mid + (0, 0, 0.02)) - FINGER_R < 0.0, lab
    assert _seg_dist(cam, p, mid + (0, 0.06, -0.02), mid + (0, 0.06, 0.02)) - FINGER_R >= CLEAR, lab  # 60 mm aside: clear


def test_top_cameras_see_the_crate_and_blocks_at_the_demo_end():
    gap, which = _min_clearance(END_LINKS.values(), _targets(END_BLOCKS))
    assert gap >= CLEAR, (round(gap * 1000, 1), which)


def test_top_cameras_see_the_crate_with_the_coworker_parked():
    """At the run limit the coworker has parked at CW_WAIT: replay its plan through the stock ArmPlan and check again."""
    from yam_arena import YAM_URDF
    from yam_arena.constants import ARM_JOINTS
    from yam_arena.urdf import forward_kinematics

    plan = ArmPlan(list(t.coworker_plan()), station.BASES["right"])
    limit = int(re.findall(r"[\d,]+", json.loads((HERE / "task.json").read_text())["run_limit"])[0].replace(",", ""))
    for k in range(0, limit + 1, 5):
        a, _ = plan.action7(k / 20)
    fk = forward_kinematics(YAM_URDF, {n: float(v) for n, v in zip(ARM_JOINTS, a[:6])})
    parked = [fk[b][:3, 3] + np.asarray(station.BASES["right"]) for b in END_BODIES]
    assert np.linalg.norm(parked[6] - END_LINKS["right_robot"][6]) > 0.0  # a real replay, not the fixture
    gap, which = _min_clearance([END_LINKS["left_robot"], parked], _targets(END_BLOCKS))
    assert gap >= CLEAR, (round(gap * 1000, 1), which)
