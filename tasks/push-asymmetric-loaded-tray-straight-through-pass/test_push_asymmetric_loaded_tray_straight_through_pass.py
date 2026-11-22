"""Offline CPU tests for push-asymmetric-loaded-tray-straight-through-pass."""

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

import push_asymmetric_loaded_tray_straight_through_pass as t  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from labkit import PartState, _extent  # noqa: E402

_I = np.array([0.0, 0.0, 0.0, 1.0])
_Z = np.zeros(3)
_SPEC = {o.name: o for o in t.OBJECTS}

DISH_LOCAL = (0.0, t.DISH_Y, t.PAD_H + t.FLR)   # the load's local offsets inside the tray
TUM_LOCAL = (0.0, t.TUM_Y, t.PAD_H + t.FLR)


def _quat(roll=0.0, pitch=0.0, yaw=0.0):
    """xyzw quaternion for yaw about Z, pitch about Y, roll about X (radians)."""
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def _state(name, pos, quat=None):
    """A resting PartState for `name` at body origin `pos`; world bounds like labkit._part_state."""
    o = _SPEC[name]
    lo, hi = _extent(o.boxes, o.cylinders)
    probe = PartState(pos=np.asarray(pos, float),
                      quat_xyzw=_I.copy() if quat is None else np.asarray(quat, float),
                      lin_vel=_Z.copy(), ang_vel=_Z.copy(), low=np.zeros(3), high=np.zeros(3), joints={})
    corners = np.array([[a, b, c] for a in (lo[0], hi[0]) for b in (lo[1], hi[1]) for c in (lo[2], hi[2])])
    w = np.array([probe.to_world(p) for p in corners])
    return PartState(pos=probe.pos, quat_xyzw=probe.quat_xyzw, lin_vel=_Z.copy(), ang_vel=_Z.copy(),
                     low=w.min(0), high=w.max(0), joints={})


def _scene(tray_pos=None, tray_quat=None, dish_pos=None, tum_pos=None, dish_quat=None, tum_quat=None):
    """Tray plus its load: the items ride at their local offsets (rotated with the tray)."""
    tray = _state("tray", tray_pos or (t.TX0, t.TY0, t.TOP + t.MAT_T), tray_quat)
    dish = _state("heavy_dish", dish_pos if dish_pos is not None else tray.to_world(DISH_LOCAL), dish_quat)
    tum = _state("tumbler", tum_pos if tum_pos is not None else tray.to_world(TUM_LOCAL), tum_quat)
    return {"tray": tray, "heavy_dish": dish, "tumbler": tum}


def _delivered(cx=0.53, cy=t.TY0, yaw_deg=0.0):
    """The tray fully through the window: rear 0.379 past the 0.361 frame face, front 0.681, both items seated."""
    return _scene(tray_pos=(cx, cy, t.TOP + t.MAT_T), tray_quat=_quat(yaw=math.radians(yaw_deg)))


PARKED = ((0.150, 0.340, t.TOP + 0.14), (0.20, -0.35, t.TOP + 0.14))


@pytest.fixture(autouse=True)
def _clear_trial():
    t._reset(None)
    yield
    t._reset(None)


def test_start_scores_zero():
    s = _scene()
    assert t.score(s) == 0.0 and not t.success(s)


def test_holding_still_scores_zero():
    t._set_test_trial(tcps=PARKED)
    s = _scene()
    assert t.score(s) == 0.0 and not t.success(s)


def test_partial_engagement_in_window_scores_half():
    t._set_test_trial(tcps=PARKED)
    s = _scene(tray_pos=(0.360, t.TY0, t.TOP + t.MAT_T))   # front 0.511 in the window, rear 0.209 short of it
    c = t.components(s)
    assert c["front_x"] >= t.PARTIAL_FRONT and c["rear_x"] < t.REAR_MIN
    assert t.score(s) == pytest.approx(t.S_PARTIAL) and not t.success(s)


def test_clean_delivery_scores_one():
    t._set_test_trial(tcps=PARKED)
    s = _delivered()
    assert t.score(s) == 1.0 and t.success(s)


def test_grippers_inside_clearance_scores_zero():
    s = _delivered()
    t._set_test_trial(tcps=((0.53, t.TY0, t.TOP + 0.05), PARKED[1]))   # a hand still over the tray
    assert t.score(s) == 0.0
    t._set_test_trial(tcps=(PARKED[0], (0.53, t.TY0 + t.DISH_Y, t.TOP + 0.05)))
    assert t.score(s) == 0.0                                         # a hand still over the dish


def test_center_push_wedge_scores_zero():
    """A geometric-midline push spins the tray past the 12 deg wedge limit: the wedged state is never credited."""
    t._set_test_trial(tcps=PARKED)
    for yaw in (13.0, 30.0, 41.0):
        s = _scene(tray_pos=(0.460, t.TY0, t.TOP + t.MAT_T), tray_quat=_quat(yaw=math.radians(yaw)))
        assert abs(t.components(s)["yaw_deg"]) > t.YAW_WEDGE and t.score(s) == 0.0, yaw


def test_shortcut_adversarial_states_score_below_one():
    """Adversarial/shortcut end states a gaming policy might leave: none reaches full credit."""
    t._set_test_trial(tcps=PARKED)
    shortcuts = {
        "tray parked at the window entry": _scene(tray_pos=(0.30, t.TY0, t.TOP + t.MAT_T)),
        "tray through but skewed at the wedge limit": _delivered(yaw_deg=13.0),
        "tray through with the tumbler tipped over": {
            **_delivered(),
            "tumbler": _state("tumbler", (0.53, t.TY0 + t.TUM_Y, t.TOP + t.MAT_T + t.PAD_H + t.FLR),
                              _quat(roll=math.radians(80.0))),
        },
        "tray slid beside the window, never through": _delivered(cy=t.TY0 + 0.20),
    }
    for label, s in shortcuts.items():
        assert t.score(s) < 1.0 and not t.success(s), label


def test_moderate_skew_past_yaw_ok_scores_zero():
    """9 deg clears the wedge limit but fails the straight-through bar even fully delivered."""
    t._set_test_trial(tcps=PARKED)
    s = _delivered(cx=0.55, yaw_deg=9.0)     # rear 0.369 through, front 0.731: only `straight` fails
    c = t.components(s)
    assert c["through"] and not c["straight"] and abs(c["yaw_deg"]) == pytest.approx(9.0, abs=0.01)
    assert t.score(s) == 0.0


def test_items_spilled_or_tray_dumped_score_zero():
    t._set_test_trial(tcps=PARKED)
    dumped = _scene(tray_pos=(0.53, t.TY0, t.TOP + t.MAT_T), tray_quat=_quat(pitch=math.radians(12.0)))
    assert t.score(dumped) == 0.0                                    # tray tipped past 10 deg
    ds = _delivered()
    ds["heavy_dish"] = _state("heavy_dish", ds["heavy_dish"].pos, _quat(roll=math.radians(20.0)))
    assert t.score(ds) == 0.0                                        # dish tipped past 15 deg
    ts = _delivered()
    ts["tumbler"] = _state("tumbler", ts["tumbler"].pos, _quat(roll=math.radians(20.0)))
    assert t.score(ts) == 0.0                                        # tumbler tipped past 15 deg
    off = _delivered()
    off["tumbler"] = _state("tumbler", (0.53, t.TY0 - 0.30, t.TOP + t.MAT_T + t.PAD_H + t.FLR))
    assert t.score(off) == 0.0                                       # tumbler off the tray


def test_bypassed_lateral_drift_scores_zero():
    t._set_test_trial(tcps=PARKED)
    s = _delivered(cy=t.TY0 + t.BYPASS_Y + 0.01)                     # slid around instead of through
    assert t.components(s)["bypassed"] and t.score(s) == 0.0


def test_past_front_max_scores_zero():
    t._set_test_trial(tcps=PARKED)
    s = _delivered(cx=t.FRONT_MAX - t.TW / 2 + 0.015)                # front edge off the delivery limit
    c = t.components(s)
    assert c["front_x"] > t.FRONT_MAX and c["bypassed"] and t.score(s) == 0.0


def test_movable_parts_have_no_fixed_or_welded_joints():
    """The tray and its load are plain rigid bodies: no welded/fixed parts, no helper joints."""
    for name in ("tray", "heavy_dish", "tumbler"):
        o = _SPEC[name]
        assert o.fixed is False and o.joints == (), name


def test_plans_reachable():
    plans = t.demo()
    for arm in ("left", "right"):
        p = ArmPlan(plans[arm], station.BASES[arm])
        for k in range(int(p.duration / 0.05) + 2):
            p.action7(k * 0.05)
        assert p.worst_ik_mm < 2.0 and p.worst_ik_deg < 2.0, arm


def test_demo_reads_no_simulator_state():
    src = (HERE / "push_asymmetric_loaded_tray_straight_through_pass.py").read_text()
    body = src[src.index("def demo("):]
    assert not re.search(r"object_states|\.data\.|_trial|env\(", body)


def test_text_matches_scorer():
    tj = json.loads((HERE / "task.json").read_text())
    assert tj["category"] == "Societal relatability"
    assert tj["instruction"] == t.INSTRUCTION
    assert "0.0, 0.5, 1.0" in tj["scoring"]
    for needle in ("8 deg", "12 deg", "10 deg", "15 deg", "6 mm", "12 mm", "40 mm", "120 mm", "800 mm"):
        assert needle in tj["scoring"], needle
    assert (t.YAW_OK, t.YAW_WEDGE, t.TILT_TRAY, t.TILT_ITEM, t.TILT_ITEM_OK) == (8.0, 12.0, 10.0, 15.0, 10.0)
    assert (t.REAR_MIN, t.PARTIAL_FRONT, t.FRONT_MAX, t.BYPASS_Y, t.HAND_CLEAR) == (0.361, 0.375, 0.800, 0.12, 0.040)
    assert tj["real_trial_cost_usd"] == t.REAL_COST_USD == 12.99
    for needle in ("302 x 410 x 20 mm", "203 x 203 x 64 mm", "457.2 mm", "0.359 kg", "1.45 kg", "0.073 kg"):
        assert needle in tj["setup"], needle
    assert re.fullmatch(r"[\d,]+ steps \(\d+ s at 20 Hz\)", tj["run_limit"])


# -- occlusion: the top cameras see the scored objects at the end of the demo --------------------------------
# The end arm poses are the demo's actual final joint angles: both plans are replayed through the stock ArmPlan
# (action7 every 0.05 s, the same calls mj_check's Precomputed makes) and the resulting joint angles go through
# URDF forward kinematics; the scored objects are at their end state (tray delivered through the window).
END_BODIES = ("base", "link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")
CAMS = {v: np.array(station.top_camera(v).pos, float) for v in station.TOP_CAMS}
FINGER_R, LINK_R = 0.010, 0.030   # m: capsule radii for a finger and an arm link
CLEAR = 0.040                     # m: required gap between every sight line and every arm capsule


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


def _targets():
    """Scored geometry at the demo's end: the delivered tray (~0.53) with its load, and the pass-through exit."""
    cx, cy, a = 0.53, t.TY0, t.DISH_E / 2.0
    z_flr = t.TOP + t.MAT_T + t.PAD_H + t.FLR
    pts = [("tray centre", (cx, cy, z_flr)),
           ("dish top", (cx, cy + t.DISH_Y, z_flr + t.DISH_H)),
           ("tumbler top", (cx, cy + t.TUM_Y, z_flr + t.TUM_H))]
    pts += [(f"tray rim {sx},{sy}", (cx + sx * t.TW / 2, cy + sy * t.TD / 2, t.TOP + t.MAT_T + t.PAD_H + t.TH))
            for sx in (-1, 1) for sy in (-1, 1)]
    pts += [(f"dish corner {sx},{sy}", (cx + sx * a, cy + t.DISH_Y + sy * a, z_flr + t.DISH_H))
            for sx in (-1, 1) for sy in (-1, 1)]
    pts += [(f"tumbler rim {sx},{sy}", (cx + sx * t.TUM_R, cy + t.TUM_Y + sy * t.TUM_R, z_flr + t.TUM_H))
            for sx in (-1, 1) for sy in (-1, 1)]
    pts += [(f"exit corner {sy}", (t.CHUTE_X + t.FRAME_T / 2, t.CHUTE_Y + sy * t.CHUTE_W / 2, t.TOP + t.MAT_T))
            for sy in (-1, 1)]
    return [(n, np.asarray(p, float)) for n, p in pts]


def _end_links():
    """Replay both demo plans through ArmPlan to their actual end: action7 every 0.05 s through the plan and its
    3 s park hold (the calls mj_check's Precomputed makes), then URDF FK of the final joint angles."""
    from yam_arena import YAM_URDF
    from yam_arena.constants import ARM_JOINTS
    from yam_arena.urdf import forward_kinematics

    out = {}
    for arm in ("left", "right"):
        plan = ArmPlan(t.demo()[arm], station.BASES[arm])
        for k in range(int((plan.duration + 3.0) / 0.05) + 2):
            a, _ = plan.action7(k * 0.05)
        fk = forward_kinematics(YAM_URDF, {n: float(v) for n, v in zip(ARM_JOINTS, a[:6])})
        out[arm] = [fk[b][:3, 3] + np.asarray(station.BASES[arm]) for b in END_BODIES]
    return out


def _min_clearance(arms, targets):
    """(smallest sight-line-to-capsule gap in m, which) over both top cameras, all targets and all arm capsules."""
    return min((_seg_dist(cam, p, a, b) - r, f"{var} -> {lab}")
               for var, cam in CAMS.items() for lab, p in targets for links in arms for a, b, r in _capsules(links))


def test_seg_dist_matches_brute_force():
    rng = np.random.default_rng(0)
    s = np.linspace(0.0, 1.0, 501)[:, None]
    for _ in range(200):
        p0, p1, q0, q1 = rng.normal(size=(4, 3))
        bf = np.min(np.linalg.norm((p0 + (p1 - p0) * s)[:, None] - (q0 + (q1 - q0) * s)[None], axis=2))
        assert bf - 5e-3 <= _seg_dist(p0, p1, q0, q1) <= bf + 1e-9


def test_occlusion_positive_control_detects_a_blocker():
    """A 40 mm finger-sized bar placed across one sight line is reported (the check can say no)."""
    lab, p = _targets()[0]
    cam = CAMS["calibrated"]
    mid = cam + 0.75 * (p - cam)
    assert _seg_dist(cam, p, mid - (0, 0, 0.02), mid + (0, 0, 0.02)) - FINGER_R < 0.0, lab
    assert _seg_dist(cam, p, mid + (0, 0.06, -0.02), mid + (0, 0.06, 0.02)) - FINGER_R >= CLEAR, lab


def test_top_cameras_see_all_scored_objects_at_demo_end():
    links = _end_links()
    gap, which = _min_clearance(links.values(), _targets())
    assert gap >= CLEAR, (round(gap * 1000, 1), which)
