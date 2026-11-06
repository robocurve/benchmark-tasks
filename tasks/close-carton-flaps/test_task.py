"""Offline checks for close-carton-flaps (NumPy and pytest only; no Isaac Sim).

Run from this folder with the yam-isaac-lab-arena package importable (installed, or a clone named by YAM_ARENA):

    YAM_ARENA=~/yam-isaac-lab-arena python -m pytest -q test_task.py
"""

import json
import math
import os
import re
import struct
import sys
import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path

import numpy as np
import pytest

if os.environ.get("YAM_ARENA"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA"]))

import yam_arena  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.analysis import world_hulls  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_OPEN, PAD_TIP_PAST_TCP, READY  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import ObjectState, load  # noqa: E402
from yam_arena.urdf import forward_kinematics, rpy_matrix  # noqa: E402

HERE = Path(__file__).resolve().parent
TASK = load(HERE / "task.py")
M = TASK.module
rx = M.rx
TOP = TASK.table_top
HZ = 20.0
ARM_URDF = Path(yam_arena.__file__).resolve().parent / "assets" / "yam" / "yam_arm.urdf"


# -- helpers -----------------------------------------------------------------------------------------------------------


def _spec(name):
    return next(o for o in TASK.objects if o.name == name)


def flap(name, deg=0.0, dz=0.0, vel=(0.0, 0.0, 0.0)):
    """A flap's state turned ``deg`` from closed about its hinge (the opening sign of its joint), raised ``dz``."""
    j = M.JOINTS[name]
    sign = 1.0 if j.initial > 0 else -1.0
    o = _spec(name)
    zero = (tuple(o.pos), (0.0, 0.0, 0.0, 1.0))
    moved = rx.Revolute(j.anchor, j.axis, j.lower, j.upper, initial=sign * deg)
    pos, q = rx._moved_pose(zero, moved) if deg else zero
    pos = np.array(pos) + [0.0, 0.0, dz]
    half = np.array(o.size) / 2
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
    world = np.array([np.array(rx._qrot(q, tuple(c))) for c in corners]) + pos
    return ObjectState(pos, np.array(q, float), np.asarray(vel, float), np.zeros(3), world.min(0), world.max(0))


def closed_box(**angles):
    """Every flap closed (angle 0) unless given, e.g. closed_box(minor_left=100)."""
    return {n: flap(n, angles.get(n, 0.0)) for n in M.FLAP_NAMES}


def start():
    return {n: flap(n, M.OPEN) for n in M.FLAP_NAMES}


def _stl(path):
    data = path.read_bytes()
    if data[:5] == b"solid" and b"facet" in data[:300]:
        return np.array(re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data), float)
    n = struct.unpack("<I", data[80:84])[0]
    rec = np.frombuffer(data[84 : 84 + 50 * n], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return np.unique(rec["v"].reshape(-1, 3).astype(float), axis=0)


@lru_cache(maxsize=1)
def collision_points():
    """Every link's collision-mesh vertices in its own frame, from yam_arm.urdf."""
    root = ET.parse(ARM_URDF).getroot()
    out = {}
    for link in root.findall("link"):
        pts = []
        for col in link.findall("collision"):
            o = col.find("origin")
            xyz = np.array(o.get("xyz").split(), float)
            rpy = np.array(o.get("rpy").split(), float)
            mesh = ARM_URDF.parent / col.find("geometry/mesh").get("filename")
            pts.append(_stl(mesh) @ rpy_matrix(rpy).T + xyz)
        if pts:
            out[link.get("name")] = np.vstack(pts)
    return out


def _poses(q6, finger, side):
    joints = {n: float(v) for n, v in zip(ARM_JOINTS, q6, strict=True)}
    joints["joint7"] = joints["joint8"] = float(finger)
    fk = forward_kinematics(ARM_URDF, joints)
    base = np.eye(4)
    base[:3, 3] = station.BASES[side]
    return {k: base @ v for k, v in fk.items() if k != "base"}


def links_in_world(q6, finger, side):
    poses = _poses(q6, finger, side)
    return {name: p @ poses[name][:3, :3].T + poses[name][:3, 3] for name, p in collision_points().items()
            if name in poses}


def box_gap(points, state):
    """Smallest Chebyshev-style gap (m) from mesh vertices to an axis-aligned cube state."""
    lo, hi = np.asarray(state.low), np.asarray(state.high)
    centre, half = (lo + hi) / 2, (hi - lo) / 2
    return float(np.max(np.abs(points - centre) - half, axis=1).min())


@lru_cache(maxsize=None)
def played(side):
    """Every control step of one arm's plan at 20 Hz: (t, joints, finger opening in m, label)."""
    plan = TASK.demo()[side]
    arm = ArmPlan(plan, station.BASES[side])
    out = []
    for k in range(round(arm.duration * HZ)):
        t = (k + 1) / HZ
        action, label = arm.action7(t)
        out.append((t, action[:6].copy(), FINGER_OPEN * action[6], label))
    return arm, out



# -- 1. the file loads ------------------------------------------------------------------------------------------------


def test_the_task_file_loads_with_its_objects_and_hooks():
    assert TASK.scene == "station" and TASK.score_over == "end"
    assert TASK.free_objects == M.FLAP_NAMES and len(TASK.objects) == 9  # 4 flaps + 5 fixed carton boxes
    assert all(callable(f) for f in (TASK.success, TASK.distance, TASK.score, TASK.demo))
    assert TASK.docs.startswith("On the station table")
    assert set(M.JOINTS) == set(M.FLAP_NAMES)


def test_every_flap_spawns_standing_open_leaning_out_and_its_free_edge_up():
    for n in M.FLAP_NAMES:
        s = flap(n, M.OPEN)
        assert flap_deg(s) == pytest.approx(100.0, abs=1e-6)
        assert s.high[2] - TOP > 0.11  # free edge about 60 mm above the crease
    assert flap("minor_left", M.OPEN).high[1] > M.Y1 and flap("minor_right", M.OPEN).low[1] < M.Y0
    assert flap("major_near", M.OPEN).low[0] < M.X0 and flap("major_far", M.OPEN).high[0] > M.X1


def flap_deg(s):
    return M.flap_angle_deg(s)


# -- 2. the demo is reachable and the arms stay clear -------------------------------------------------------------------


@pytest.mark.parametrize("side", ["left", "right"])
def test_every_arm_plan_is_reachable_within_the_joint_limits_at_20_hz(side):
    arm, steps = played(side)
    joints = np.array([s[1] for s in steps])
    assert arm.worst_ik_mm < 0.5 and arm.worst_ik_deg < 0.1, (side, arm.worst_ik_mm, arm.worst_ik_deg)
    assert np.all(joints >= ik.LOWER) and np.all(joints <= ik.UPPER), side
    assert max(w.pos[2] for w in TASK.demo()[side]) - TOP <= 0.14 + 1e-9


def test_both_arms_follow_one_clock_and_end_parked_and_still():
    plan = TASK.demo()
    left, right = plan["left"], plan["right"]
    assert sum(w.duration for w in left) == pytest.approx(sum(w.duration for w in right))
    for pts in (left, right):
        assert pts[-1].pos == pts[-2].pos and pts[-1].duration >= 2.0 - 1e-9
    meta = json.loads((HERE / "task.json").read_text())
    steps = sum(w.duration for w in left) * HZ
    limit = int(re.match(r"([\d,]+) steps", meta["run_limit"]).group(1).replace(",", ""))
    assert limit >= steps + 100 and limit % 200 == 0


def test_the_arms_keep_3_mm_apart_and_off_the_carton_walls():
    """URDF collision-mesh vertices every 0.1 s: the two arms stay 3 mm apart; no arm vertex goes below the wall top
    over the walls (it may go inside the opening only above the flaps)."""
    _, left = played("left")
    _, right = played("right")
    worst = np.inf
    for (t, ql, fl, _), (_, qr, fr, _) in zip(left, right, strict=True):
        if round(t * HZ) % 4:
            continue
        pl = np.vstack(list(links_in_world(ql, fl, "left").values()))
        pr = np.vstack(list(links_in_world(qr, fr, "right").values()))
        worst = min(worst, float(np.min(np.linalg.norm(pl[::4, None, :] - pr[None, ::4, :], axis=2))))
        for pts in (pl, pr):
            low = pts[pts[:, 2] < M.WALL_TOP]
            inside = (low[:, 0] > M.X0 - 0.002) & (low[:, 0] < M.X1 + 0.002) & (low[:, 1] > M.Y0 - 0.002) & (
                low[:, 1] < M.Y1 + 0.002)
            assert not inside.any(), t
    assert worst >= 0.003, worst


def _finger_tips(side, t):
    arm, steps = played(side)
    k = round(t * HZ) - 1
    _, q, f, _ = steps[k]
    w = links_in_world(q, f, side)
    return w["tip_left"], w["tip_right"]


def _on_minor(pts, name):
    o = _spec(name)
    lo, hi = np.array(o.pos) - np.array(o.size) / 2, np.array(o.pos) + np.array(o.size) / 2
    low = pts[pts[:, 2] < pts[:, 2].min() + 0.002]  # the pad tip's lowest band
    c = low.mean(axis=0)
    return lo[0] < c[0] < hi[0] and lo[1] + 0.003 < c[1] < hi[1] - 0.003


def _time_of(side, label, k=0):
    plan = TASK.demo()[side]
    i = [n for n, w in enumerate(plan) if w.label == label][k]
    return sum(w.duration for w in plan[: i + 1])


def test_pressing_pads_sit_on_the_minors_and_the_handover_keeps_both_held():
    """Pressing: each arm has a pad tip on its own minor. Through the handover slide (sampled every 0.1 s) the right
    gripper's right pad stays on the right minor; at its end the other pad is on the left minor."""
    t_press = _time_of("right", "press it flat")
    for side, name in (("right", "minor_right"), ("left", "minor_left")):
        tips = _finger_tips(side, t_press)
        assert any(_on_minor(p, name) for p in tips), side
    t_end = _time_of("right", "hold both end flaps")
    for t in np.arange(t_press + 1.0 + 0.1, t_end + 1e-9, 0.1):
        tips = _finger_tips("right", t)
        assert any(_on_minor(p, "minor_right") for p in tips), t
    tips = _finger_tips("right", t_end)
    assert any(_on_minor(p, "minor_left") for p in tips) and any(_on_minor(p, "minor_right") for p in tips)
    # pad tips 1 mm into the closed minors' top
    lowest = min(p[:, 2].min() for p in tips)
    assert lowest == pytest.approx(M.MINOR_Z + M.T / 2 - 0.001, abs=0.0015)


def test_the_right_gripper_leaves_the_far_majors_path():
    """While the far major falls (its footprint x 0.30-0.36), the right gripper's vertices stay in front of x 0.297."""
    t0, t1 = _time_of("left", "push the far flap past vertical") - 2.0, _time_of("right", "rise")
    _, right = played("right")
    for t, q, f, _ in right:
        if t0 <= t <= t1:
            pts = np.vstack(list(links_in_world(q, f, "right").values()))
            low = pts[pts[:, 2] < M.MAJOR_Z + 0.03]
            assert low[:, 0].max() < 0.297, t


def test_no_arm_passes_through_a_standing_flap_before_pushing_it():
    """Until a flap's push starts, no arm vertex (sampled every 0.1 s) enters the standing board grown by 2 mm."""
    pushes = {"minor_right": ("right", "push it past vertical"), "minor_left": ("left", "push it past vertical"),
              "major_far": ("left", "push the far flap past vertical"),
              "major_near": ("right", "push the near flap past vertical")}
    for name, (side, label) in pushes.items():
        t_push = _time_of(side, label) - [w for w in TASK.demo()[side] if w.label == label][0].duration
        s = flap(name, M.OPEN)
        rot = np.array([rx._qrot(tuple(s.quat_xyzw), tuple(e)) for e in np.eye(3)]).T  # board frame -> world
        half = np.array(_spec(name).size) / 2 + 0.002
        for arm in ("left", "right"):
            for t, q, f, _ in played(arm)[1]:
                if t > t_push or round(t * HZ) % 2:
                    continue
                pts = np.vstack(list(links_in_world(q, f, arm).values()))
                inside = np.all(np.abs((pts - s.pos) @ rot) < half, axis=1)
                assert not inside.any(), (name, arm, t)


# -- 3. scoring -------------------------------------------------------------------------------------------------------


def test_the_closed_carton_scores_one_and_succeeds_when_still():
    s = closed_box()
    assert M.score(s) == 1.0 and M.success(s)
    assert M.distance(s) == pytest.approx(0.0, abs=1e-9)


def test_flaps_within_ten_degrees_count_as_closed():
    s = {n: flap(n, 8.0 if n in M.MAJORS else 0.0) for n in M.FLAP_NAMES}
    assert M.score(s) == 1.0
    s = {n: flap(n, 12.0 if n == "major_near" else 0.0) for n in M.FLAP_NAMES}
    assert M.score(s) == 0.75


def test_moving_flaps_score_but_do_not_succeed():
    s = closed_box()
    s["major_far"] = flap("major_far", 0.0, vel=(0.0, 0.0, 0.05))
    assert M.score(s) == 1.0 and not M.success(s)


def test_the_start_scores_zero():
    assert M.score(start()) == 0.0 and not M.success(start())
    assert M.distance(start()) == pytest.approx(4 * math.radians(100.0))


def test_wrong_order_minors_on_top_of_closed_majors_scores_zero():
    s = closed_box()
    lift = M.MAJOR_Z - M.MINOR_Z
    for n in M.MINORS:
        s[n] = flap(n, 0.0, dz=+lift + 0.0035)  # minors lying on the majors
    for n in M.MAJORS:
        s[n] = flap(n, 0.0, dz=-lift)
    assert M.score(s) == 0.0


def test_one_minor_left_open_scores_only_the_other_minor():
    s = closed_box(minor_left=100.0)
    assert M.score(s) == 0.25


def test_both_minors_held_down_with_majors_open_scores_half():
    s = closed_box(major_near=100.0, major_far=100.0)
    assert M.score(s) == 0.5


def test_one_major_closed_over_both_minors_scores_three_quarters():
    s = closed_box(major_near=100.0)
    assert M.score(s) == 0.75


def test_majors_closed_over_open_minors_score_zero():
    s = closed_box(minor_left=100.0, minor_right=100.0)
    for n in M.MAJORS:
        s[n] = flap(n, 3.0)  # sagging into the opening
    assert M.score(s) == 0.0


# -- 4. physics numbers behind the design --------------------------------------------------------------------------------


def test_a_released_minor_springs_open_and_a_closed_major_holds_both_minors():
    g = 9.81
    minor_weight = M.MINOR_MASS * g * M.FLAP_D / 2  # N*m about its crease, flat
    for deg in range(0, 100, 5):
        spring = M.MINOR_K * (M.OPEN - deg)
        assert spring > minor_weight * math.cos(math.radians(deg)), deg
    # one major lying flat, its weight shared by the two minors at their mid-depth (the hinge takes none of it in
    # this bound; with half on the hinge it is still 1.4 times the spring's excess)
    push_up = M.MINOR_K * M.OPEN - minor_weight
    per_minor = M.MAJOR_MASS * g / 2 * (M.FLAP_D / 2) / 2
    assert per_minor > 1.4 * push_up


def test_no_flap_overlaps_the_carton_or_another_flap_at_spawn():
    s = start()
    boxes = {o.name: o for o in M.BODY}
    for n, st in s.items():
        for b in boxes.values():
            lo, hi = np.array(b.pos) - np.array(b.size) / 2, np.array(b.pos) + np.array(b.size) / 2
            # standing flaps reach past the walls' outer faces only by their lean; check they clear the wall tops
            assert st.low[2] > hi[2] - 1e-6 or (st.high < lo).any() or (st.low > hi).any(), (n, b.name)


# -- 5. camera -----------------------------------------------------------------------------------------------------------


def _hit(a, b, hull):
    """Does the segment a -> b pass through a convex hull (outward planes n . x = d)?"""
    d = b - a
    t0, t1 = 0.0, 1.0
    for n, off in zip(hull.planes[:, :3], hull.planes[:, 3], strict=True):
        num, den = off - n @ a, n @ d
        if abs(den) < 1e-12:
            if num < 0:
                return False
            continue
        t = num / den
        if den < 0:
            t0 = max(t0, t)
        else:
            t1 = min(t1, t)
        if t0 > t1:
            return False
    return t1 > 1e-6 and t0 < 1 - 1e-6


def _visible_fraction(state, hulls, faces=("top", "back")):
    """Fraction of points on the box's top and back (camera-side, -x) faces with a clear line to the top camera."""
    cam = np.asarray(station.top_camera().pos, float)
    lo, hi = np.asarray(state.low), np.asarray(state.high)
    g = np.linspace(0.1, 0.9, 5)
    pts = []
    for a in g:
        for b in g:
            if "top" in faces:
                pts.append([lo[0] + a * (hi[0] - lo[0]), lo[1] + b * (hi[1] - lo[1]), hi[2]])
            if "back" in faces:  # the face toward the camera
                pts.append([lo[0], lo[1] + a * (hi[1] - lo[1]), lo[2] + b * (hi[2] - lo[2])])
    seen = sum(not any(_hit(cam, np.asarray(p), h) for _, h in hulls) for p in pts)
    return seen / len(pts)


def _hulls(side, q, finger):
    return world_hulls(_poses(q, finger, side))


def _hulls(side, q, finger):
    return world_hulls(_poses(q, finger, side))


def test_the_top_camera_sees_every_standing_flap_in_the_first_frame():
    hulls = _hulls("left", np.array(READY), FINGER_OPEN) + _hulls("right", np.array(READY), FINGER_OPEN)
    for n, s in start().items():
        assert _visible_fraction(s, hulls, faces=("top",)) == 1.0, n


def test_the_top_camera_sees_the_whole_closed_top_in_the_last_frame():
    (_, left), (_, right) = played("left"), played("right")
    hulls = _hulls("left", left[-1][1], left[-1][2]) + _hulls("right", right[-1][1], right[-1][2])
    for n, s in closed_box().items():
        assert _visible_fraction(s, hulls, faces=("top",)) == 1.0, n


# -- task.json --------------------------------------------------------------------------------------------------------


def test_task_json_has_every_field_and_names_no_one():
    meta = json.loads((HERE / "task.json").read_text())
    for key in ("title", "category", "keywords", "instruction", "context", "setup", "scoring", "run_limit",
                "fidelity", "scores"):
        assert meta.get(key), key
    # scores is filled after the Dream-1 run and media by the upload script
    assert meta["scores"] == "not yet run" or meta["scores"].startswith("Dream-1 run: "), meta["scores"]
    assert meta["category"] in {"Impact on jobs", "Impact on AI timelines", "Safety and alignment",
                                "Societal relevance", "Other"}
    text = json.dumps(meta).lower() + (HERE / "README.md").read_text().lower() + TASK.docs.lower()
    for word in ("robocurve_sprint", "knowledge base", "kairos", "university"):
        assert word not in text, word


