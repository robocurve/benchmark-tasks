"""Offline CPU tests for portion-disher-level-and-sweep-release."""

import json
import math
import pathlib
import re
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import labkit  # noqa: E402
import mechkit as mk  # noqa: E402
import pitch_demo  # noqa: E402
import portion_disher_level_and_sweep_release as t  # noqa: E402
from labkit import PartState  # noqa: E402
from yam_arena import constants as _C  # noqa: E402
from yam_arena import station  # noqa: E402

PARK = [np.array([0.14, 0.30, t.TOP + 0.16]), np.array([0.18, -0.30, t.TOP + 0.16])]
BENCH = ("mat", "rest_tray", "coaster_ramekin", "coaster_tub", "ramekin", "tub")


def _rot(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def _quat(yaw_deg=0.0, tilt_deg=0.0):
    a, b = math.radians(yaw_deg), math.radians(tilt_deg)
    r = np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])
    if tilt_deg:
        c, s = math.cos(b), math.sin(b)
        r = r @ np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    w = math.sqrt(max(0.0, 1.0 + tr)) / 2.0
    if w < 1e-9:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return np.array([(r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w), (r[1, 0] - r[0, 1]) / (4 * w), w])


def _bench_state(spec, pos, yaw=0.0, tilt=0.0):
    if hasattr(spec, "cylinders"):
        lo, hi = labkit._extent(spec.boxes, spec.cylinders)
    else:
        lo, hi = np.asarray(spec.lo, float), np.asarray(spec.hi, float)
    p = np.asarray(pos, float)
    q = _quat(yaw, tilt)
    corners = np.array([[a, b, c] for a in (lo[0], hi[0]) for b in (lo[1], hi[1]) for c in (lo[2], hi[2])])
    w = corners @ _rot(q).T + p
    return PartState(pos=p, quat_xyzw=q, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=w.min(0), high=w.max(0))


def states(q=None, moved=None, mash=None, ramekin=None, tub=None):
    mk.new_episode()
    st = t.kit.fake_states(q=q or {}, moved=moved or {})
    for name, spec in (("mat", t.MAT_P), ("rest_tray", t.REST), ("coaster_ramekin", t.COASTER_R),
                       ("coaster_tub", t.COASTER_T), ("ramekin", t.RAMEKIN), ("tub", t.TUB)):
        st[name] = _bench_state(spec, spec.pos)
    if ramekin:
        st["ramekin"] = _bench_state(t.RAMEKIN, *ramekin)
    if tub:
        st["tub"] = _bench_state(t.TUB, *tub)
    if mash:
        st["mash"] = _bench_state(t.kit.parts["mash"], *mash)
    return st


def in_ramekin(x=0.0, y=0.0, z=0.040):
    return (np.array([t.RX + x, t.RY + y, t.TOP + z]), 0.0, 0.0)


def dumped(sweep_deg=170.0, rack_m=0.0):
    return dict(q={"thumb_rack": rack_m, "wiper_sweep": math.radians(sweep_deg)}, mash=in_ramekin())


def _eval(monkeypatch, tcps=PARK, **kw) -> float:
    monkeypatch.setattr(t.mk, "tcps", lambda: list(tcps))
    return t.score(states(**kw))


def test_noop_initial_state_zero(monkeypatch):
    monkeypatch.setattr(t.mk, "tcps", lambda: list(PARK))
    st = states()
    assert t.score(st) == 0.0
    assert not t.success(st)
    assert t.components(st)["sweep_deg"] == 0.0


def test_ideal_completed_and_released_one(monkeypatch):
    assert _eval(monkeypatch, **dumped()) == 1.0
    monkeypatch.setattr(t.mk, "tcps", lambda: list(PARK))
    assert t.success(states(**dumped()))


def test_proximity_hover_still_grasped_cap(monkeypatch):
    on_handle = [PARK[0], np.array([t.DX, t.DY + t.GRIP_Y, t.TOP + t.DISHER_Z0 + t.GRIP_Z])]
    assert _eval(monkeypatch, tcps=on_handle, **dumped()) == 0.7
    on_tab = [np.array([t.DX + t.TAB_X, t.DY + t.TAB_Y, t.TOP + t.DISHER_Z0 + t.TAB_Z]), PARK[1]]
    assert _eval(monkeypatch, tcps=on_tab, **dumped()) == 0.7
    assert _eval(monkeypatch, tcps=[], **dumped()) == 0.7


def test_unlatched_or_incomplete_end_state(monkeypatch):
    assert _eval(monkeypatch, **dumped(sweep_deg=20.0)) == 0.0
    assert _eval(monkeypatch, **dumped(sweep_deg=140.0)) == 0.0
    assert _eval(monkeypatch, q={"thumb_rack": 0.0, "wiper_sweep": math.radians(170.0)},
                 mash=(np.array([t.DX, t.DY, t.TOP + t.DISHER_Z0]), 0.0, 0.0)) == 0.4
    assert _eval(monkeypatch, q={"thumb_rack": 0.0, "wiper_sweep": math.radians(170.0)},
                 mash=(np.array([t.RX, t.RY, t.TOP + 0.005]), 0.0, 0.0)) == 0.4
    assert _eval(monkeypatch, q={"thumb_rack": 0.0, "wiper_sweep": math.radians(170.0)},
                 mash=in_ramekin(x=0.030)) == 0.4
    carried = (np.array([t.DX + 0.055, t.DY, t.TOP + t.DISHER_Z0 + 0.06]), _quat())
    assert _eval(monkeypatch, moved={"disher": carried}, **dumped()) == 0.7
    tipped = (np.array([t.DX, t.DY, t.TOP + t.DISHER_Z0]), _quat(tilt_deg=25.0))
    assert _eval(monkeypatch, moved={"disher": tipped}, **dumped()) == 0.7


def test_one_hand_displaced_base_ablation(monkeypatch):
    assert _eval(monkeypatch, ramekin=(np.array([t.RX + 0.030, t.RY + 0.005, t.TOP]), 0.0, 0.0), **dumped()) == 0.0
    assert _eval(monkeypatch, ramekin=(np.array([t.RX, t.RY, t.TOP + 0.004]), 0.0, 25.0), **dumped()) == 0.0
    assert _eval(monkeypatch, tub=(np.array([t.TX, t.TY + 0.040, t.TOP]), 0.0, 0.0), **dumped()) == 0.0
    assert _eval(monkeypatch, tub=(np.array([t.TX, t.TY, t.TOP]), 0.0, 20.0), **dumped()) == 0.0


def test_shortcuts_score_zero(monkeypatch):
    # Shaking or flipping bowl without clearing rack detent leaves wiper < 150 deg -> 0.0
    assert _eval(monkeypatch, q={"wiper_sweep": math.radians(18.0)}, mash=in_ramekin()) == 0.0
    flipped = (np.array([t.DX, t.DY, t.TOP + t.DISHER_Z0 + 0.05]), _quat(tilt_deg=150.0))
    assert _eval(monkeypatch, moved={"disher": flipped}, q={"wiper_sweep": math.radians(15.0)}, mash=in_ramekin()) == 0.0


def test_possible_scores_and_undo(monkeypatch):
    near = [np.array([t.DX, t.DY, t.TOP + t.DISHER_Z0]), PARK[1]]
    got = {
        _eval(monkeypatch, q={"wiper_sweep": math.radians(15.0)}, mash=in_ramekin()),
        _eval(monkeypatch, q={"wiper_sweep": math.radians(170.0)}, mash=in_ramekin(x=0.05)),
        _eval(monkeypatch, tcps=near, **dumped()),
        _eval(monkeypatch, **dumped()),
    }
    assert got == {0.0, 0.4, 0.7, 1.0}
    monkeypatch.setattr(t.mk, "tcps", lambda: list(PARK))
    assert t.distance(states(**dumped())) < t.distance(states(q={"wiper_sweep": math.radians(20.0)}, mash=in_ramekin(x=0.05)))


def test_scoring_text_numbers_and_contract():
    tj = json.loads((HERE / "task.json").read_text())
    assert len(tj["instruction"].split()) <= 28
    assert not set(re.findall(r"\w+", tj["instruction"].lower())) & {"person", "patient", "human", "people"}
    assert "o*net" not in tj["context"].lower() and "bls" not in tj["context"].lower()
    assert "possible scores: 0.0, 0.4, 0.7, 1.0" in tj["scoring"].lower()
    assert tj["real_trial_cost_usd"] == 28
    assert (t.CLEAR, t.RAMEKIN_TRAVEL, t.TUB_TRAVEL, t.VESSEL_TILT_DEG) == (0.045, 0.020, 0.025, 8.0)
    assert (t.RACK_FULL, t.SWEEP_MIN_DEG) == (0.027, 150.0)
    assert (t.PARK_XY, t.PARK_Z, t.PARK_TILT_DEG) == (0.045, 0.040, 15.0)
    assert t.MASH_XY == 0.020 and t.MASH_Z == (0.012, 0.070) and t.SUCCESS_HOLD_S == 3.0
    assert all(m >= 0.025 for m in (t.DISHER_M, t.RACK_M, t.WIPER_M, t.MASH_M, t.RAMEKIN_M, t.TUB_M))


def test_plan_kinematic_reach_and_purity():
    plan = t.demo()
    reach = pitch_demo.check_plan(plan, station.BASES)
    for arm, (mm, deg) in reach.items():
        assert mm <= 5.0 and deg <= 8.0, (arm, reach[arm])
    dur = max(sum(w.duration for w in v) for v in plan.values())
    steps = int(re.match(r"([\d,]+)\s*steps", json.loads((HERE / "task.json").read_text())["run_limit"]).group(1).replace(",", ""))
    assert steps >= math.ceil(dur * 20 * 1.3) and steps % 20 == 0
    names = set(t.demo.__code__.co_names)
    assert not names & {"env", "scene", "objects", "score", "components"}


# -- top camera line-of-sight occlusion proxy ----------------------------------------------------------------------
CAM = np.array(station.top_camera("calibrated").pos, float)
GRIP_R = abs(float(_C.FINGER_OPEN))
GRIP_LEN = float(_C.TCP_OFFSET[2])


def _seg_dist(p0: np.ndarray, p1: np.ndarray, q0: np.ndarray, q1: np.ndarray) -> float:
    d1, d2, r = p1 - p0, q1 - q0, p0 - q0
    a, e, f = float(d1 @ d1), float(d2 @ d2), float(d2 @ r)
    s_ = min(max(-float(d1 @ r) / a, 0.0), 1.0) if a > 1e-12 else 0.0
    t_ = (float(d1 @ d2) * s_ + f) / e if e > 1e-12 else 0.0
    if t_ < 0.0:
        t_, s_ = 0.0, min(max(-float(d1 @ r) / a, 0.0), 1.0) if a > 1e-12 else 0.0
    elif t_ > 1.0:
        t_, s_ = 1.0, min(max((float(d1 @ d2) - float(d1 @ r)) / a, 0.0), 1.0) if a > 1e-12 else 0.0
    return float(np.linalg.norm((p0 + d1 * s_) - (q0 + d2 * t_)))


def test_top_camera_line_of_sight_clear_at_end():
    targets = [
        np.array([t.DX, t.DY, t.TOP + t.DISHER_Z0]),
        np.array([t.RX, t.RY, t.TOP + 0.040]),
        np.array([t.TX, t.TY, t.TOP + 0.040]),
    ]
    for tcp in PARK:
        q0 = np.asarray(tcp, float)
        q1 = q0 + np.array([0.0, 0.0, GRIP_LEN])
        for pt in targets:
            assert _seg_dist(CAM, pt, q0, q1) >= GRIP_R
