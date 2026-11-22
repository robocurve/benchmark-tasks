"""Offline CPU tests for slide-flush-cutting-board-to-edge-and-rack."""

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

import slide_flush_cutting_board_to_edge_and_rack as t  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from labkit import PartState, _extent  # noqa: E402

_I = np.array([0.0, 0.0, 0.0, 1.0])
_Z = np.zeros(3)
_SPEC = {o.name: o for o in t.OBJECTS}


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


def _scene(board_pos=None, board_quat=None):
    """All three task objects; only the board moves."""
    return {
        "butcher_block": _state("butcher_block", (t.BLK_CX, t.BLK_CY, t.TOP)),
        "board": _state("board", board_pos or (t.BRD_X0, t.BRD_Y0, t.BLK_TOP), board_quat),
        "dish_rack": _state("dish_rack", (t.RK_CX, t.RK_CY, t.TOP)),
    }


def _overhang(cy=0.2474):
    """The board slid to the safe overhang window: -Y edge at +0.095, >30 mm past the +0.150 cliff."""
    return _scene(board_pos=(t.BRD_X0, cy, t.BLK_TOP))


def _racked(cx=0.647, cy=-0.330, cz=None, tilt_deg=0.0):
    """The board flat on the rack floor inside the inner cavity (the demo's measured end state)."""
    return _scene(board_pos=(cx, cy, t.TOP + t.RK_FLR if cz is None else cz),
                  board_quat=_quat(roll=math.radians(tilt_deg)))


PARKED = ((0.200, +0.340, 0.920), (0.100, -0.520, 0.860))


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


def test_flat_grab_resettled_on_block_scores_zero():
    """A top-down flat grab cannot lift a flush board; re-settled on the block it never entered the window."""
    t._set_test_trial(tcps=PARKED)
    for cy in (t.BRD_Y0, t.BRD_Y0 + 0.004):
        s = _scene(board_pos=(t.BRD_X0, cy, t.BLK_TOP))
        assert t.components(s)["past_window"] is False
        assert t.score(s) == 0.0 and not t.success(s)


def test_overpush_tipoff_past_cliff_scores_zero():
    """Pushing past the 50 % balance point tips the board off the cliff (stage-0: tilt 15.2 deg, z_lo 0.747)."""
    t._set_test_trial(tcps=PARKED)
    s = _scene(board_pos=(0.321, +0.038, 0.787), board_quat=_quat(roll=math.radians(15.2)))
    c = t.components(s)
    assert c["tilt_deg"] == pytest.approx(15.2, abs=0.05)
    assert s["board"].low[2] == pytest.approx(0.747, abs=0.005)
    assert c["dropped_low"] and c["tilt_deg"] > t.TILT_TIP
    assert t.score(s) == 0.0 and not t.success(s)


def test_safe_overhang_window_scores_half():
    t._set_test_trial(tcps=PARKED)
    s = _overhang()
    c = t.components(s)
    assert c["past_window"] and c["tilt_deg"] <= t.TILT_WINDOW and not c["in_rack_xy"]
    assert t.score(s) == pytest.approx(t.S_PARTIAL) and not t.success(s)


def test_lifted_clear_scores_half():
    """The board released mid-air past the window (a 'lifted clear' state) is partial credit."""
    t._set_test_trial(tcps=PARKED)
    s = _scene(board_pos=(0.45, -0.10, 0.90))
    c = t.components(s)
    assert not c["in_rack_xy"] and not c["dropped_low"] and c["past_window"]
    assert t.score(s) == pytest.approx(t.S_PARTIAL)


def test_clean_racked_delivery_scores_one():
    t._set_test_trial(tcps=PARKED)
    s = _racked()
    assert t.score(s) == 1.0 and t.success(s)


def test_tilted_inside_rack_scores_zero():
    """8 deg tilt inside the cavity is not 'seated flat' (the 6 deg limit)."""
    t._set_test_trial(tcps=PARKED)
    s = _racked(tilt_deg=8.0)
    c = t.components(s)
    assert c["in_rack_xy"] and c["tilt_deg"] == pytest.approx(8.0, abs=0.05)
    assert t.score(s) == 0.0


def test_resting_high_on_rack_walls_scores_zero():
    """Caught on the 4-inch walls (underside at 0.85 m), not on the rack floor."""
    t._set_test_trial(tcps=PARKED)
    s = _racked(cz=0.85)
    c = t.components(s)
    assert c["in_rack_xy"] and not c["seated_z"]
    assert t.score(s) == 0.0


def test_dropped_on_table_outside_rack_scores_zero():
    t._set_test_trial(tcps=PARKED)
    s = _scene(board_pos=(0.40, +0.60, t.TOP))
    c = t.components(s)
    assert c["dropped_low"] and not c["in_rack_xy"]
    assert t.score(s) == 0.0


def test_still_moving_inside_rack_scores_zero():
    """Inside, flat and seated but not yet at rest is not full credit."""
    t._set_test_trial(tcps=PARKED)
    s = _racked()
    s["board"] = PartState(pos=s["board"].pos, quat_xyzw=s["board"].quat_xyzw,
                           lin_vel=np.array([0.05, 0.0, 0.0]), ang_vel=_Z.copy(),
                           low=s["board"].low, high=s["board"].high, joints={})
    assert t.score(s) == 0.0


def test_grippers_inside_clearance_scores_zero():
    s = _racked()
    t._set_test_trial(tcps=((0.638, -0.335, 0.80), PARKED[1]))   # a hand still over the board
    assert t.score(s) == 0.0
    t._set_test_trial(tcps=(PARKED[0], (0.60, -0.40, 0.80)))
    assert t.score(s) == 0.0


def test_undo_replay_end_pose_scores_zero():
    """The undo probe replays the demo backward to the last grasp: the right hand ends hovering closed over
    the seated board at the settle-check pose, inside the 40 mm clearance -- never credited."""
    t._set_test_trial(tcps=(PARKED[0], (0.648, -0.470, 0.808)))
    s = _racked()
    assert t.components(s)["grippers_clear"] is False
    assert t.score(s) == 0.0


def test_half_freeze_during_carry_scores_zero():
    """The half probe freezes mid-demo while a hand is on the board: never credited."""
    t._set_test_trial(tcps=((0.180, +0.095, 0.822), (0.35, -0.095, 0.818)))
    s = _overhang()
    assert t.score(s) == 0.0


def test_shortcut_adversarial_states_score_below_one():
    """Adversarial/shortcut end states a gaming policy might leave: none reaches full credit."""
    t._set_test_trial(tcps=PARKED)
    shortcuts = {
        "board untouched on the block": _scene(),
        "board nudged a few mm, still flush": _scene(board_pos=(t.BRD_X0, t.BRD_Y0 - 0.004, t.BLK_TOP)),
        "board at the cliff edge, not 30 mm past": _scene(board_pos=(t.BRD_X0, 0.290, t.BLK_TOP)),
        "board balanced on the -Y wall, outside the cavity": _scene(board_pos=(0.638, -0.150, 0.86)),
        "board flat on the table beside the rack": _scene(board_pos=(0.35, -0.40, t.TOP)),
        "board tipped against the rack's outer wall": _scene(
            board_pos=(0.40, -0.10, 0.82), board_quat=_quat(roll=math.radians(60.0))),
    }
    for label, s in shortcuts.items():
        assert t.score(s) < 1.0 and not t.success(s), label


def test_movable_parts_have_no_fixed_or_welded_joints():
    """The board is a plain rigid body: no welded/fixed parts, no helper joints."""
    o = _SPEC["board"]
    assert o.fixed is False and o.joints == ()
    assert _SPEC["butcher_block"].fixed is True and _SPEC["dish_rack"].fixed is True


def test_plans_reachable():
    plans = t.demo()
    for arm in ("left", "right"):
        p = ArmPlan(plans[arm], station.BASES[arm])
        prev = None
        max_dq = 0.0
        for k in range(int(p.duration / 0.05) + 2):
            a, _ = p.action7(k * 0.05)
            if prev is not None:
                max_dq = max(max_dq, float(np.abs(a[:6] - prev[:6]).max()))
            prev = a
        assert p.worst_ik_mm < 2.0 and p.worst_ik_deg < 2.0, arm
        assert max_dq < 0.18, (arm, round(max_dq, 4))


def test_demo_reads_no_simulator_state():
    src = (HERE / "slide_flush_cutting_board_to_edge_and_rack.py").read_text()
    body = src[src.index("def demo("):]
    assert not re.search(r"object_states|\.data\.|_trial|env\(", body)


def test_text_matches_scorer():
    tj = json.loads((HERE / "task.json").read_text())
    assert tj["category"] == "Societal relatability"
    assert tj["instruction"] == t.INSTRUCTION
    assert "0.0, 0.5, 1.0" in tj["scoring"]
    for needle in ("6 deg", "10 deg", "15 deg", "30 mm", "40 mm", "12 mm", "8 mm"):
        assert needle in tj["scoring"], needle
    assert (t.TILT_FLAT, t.TILT_TIP, t.TILT_WINDOW, t.HAND_CLEAR) == (6.0, 15.0, 10.0, 0.040)
    assert (t.WINDOW_PASS, t.DROP_Z, t.SEAT_LO, t.SEAT_HI) == pytest.approx((0.120, 0.7595, 0.7585, 0.7785))
    assert (t.PASS_SCORE, t.S_PARTIAL) == (1.0, 0.5)
    assert tj["real_trial_cost_usd"] == t.REAL_COST_USD == 198.93
    for needle in ("457.2 x 304.8 x 12.7 mm", "508 x 381 x 57.2 mm", "502 x 502 x 101.6 mm",
                   "1.47 kg", "9.07 kg", "1.43 kg", "19 mm", "6 mm"):
        assert needle in tj["setup"], needle
    assert re.fullmatch(r"[\d,]+ steps \(\d+ s at 20 Hz\)", tj["run_limit"])


# -- occlusion: the top cameras see the scored objects at the end of the demo --------------------------------
# The end arm poses are the demo's actual final joint angles: both plans are replayed through the stock ArmPlan
# (action7 every 0.05 s, the same calls mj_check's Precomputed makes) and the resulting joint angles go through
# URDF forward kinematics; the scored board is at its end state (seated inside the rack).
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
    """Scored geometry at the demo's end: the seated board inside the rack inner cavity and the cavity itself."""
    cx, cy, cz = 0.647, -0.330, t.TOP + t.RK_FLR
    bx, by = t.BW_BRD / 2.0, t.BD_BRD / 2.0
    pts = [("board centre", (cx, cy, cz + t.BH_BRD)),
           ("board top", (cx, cy, cz + t.BH_BRD))]
    pts += [(f"board corner {sx},{sy}", (cx + sx * bx, cy + sy * by, cz + t.BH_BRD))
            for sx in (-1, 1) for sy in (-1, 1)]
    for nx, x in (("x0", t.RK_IN_X0), ("x1", t.RK_IN_X1)):
        for ny, y in (("y0", t.RK_IN_Y0), ("y1", t.RK_IN_Y1)):
            pts.append((f"cavity {nx},{ny}", (x, y, t.TOP + t.RK_FLR + 0.01)))
    pts.append(("cavity centre", (t.RK_CX, t.RK_CY, t.TOP + t.RK_FLR + 0.01)))
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
    s = np.linspace(0.0, 1.0, 1501)[:, None]
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
