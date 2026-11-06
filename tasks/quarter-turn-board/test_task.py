"""Offline checks for quarter-turn-board (NumPy and pytest only; no Isaac Sim).

Run from this folder with the yam-isaac-lab-arena package importable (installed, as on the Brev VM, or a clone
named by YAM_ARENA):

    YAM_ARENA=~/yam-isaac-lab-arena python -m pytest -q test_task.py
"""

import json
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

if os.environ.get("YAM_ARENA"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA"]))

from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import ArmPlan, grip_rotation  # noqa: E402
from yam_arena.task_file import ObjectState, load  # noqa: E402

HERE = Path(__file__).resolve().parent
TASK = load(HERE / "task.py")
M = TASK.module
TOP = TASK.table_top
HZ = 20.0
RUN_LIMIT_STEPS = 2400
GRIPPER_MAX = 2 * abs(FINGER_OPEN)  # pads 94 mm apart fully open
# Closed fingers, from the URDF collision meshes (tip_left / tip_right) between the pad tips and the TCP:
# 12.8 mm across the closing direction, 27.4 mm along the pads.
FINGER_HALF_ACROSS, FINGER_HALF_ALONG = 0.0064, 0.0137
PLAN = TASK.demo()


# -- helpers -----------------------------------------------------------------------------------------------------------


def quat_from(yaw=0.0, roll=0.0):
    """XYZW for a yaw about z, then a roll about the body x axis (to tilt the board)."""
    qz = np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
    qx = np.array([math.sin(roll / 2), 0.0, 0.0, math.cos(roll / 2)])
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ]
    )


def board(x=M.START_XY[0], y=M.START_XY[1], yaw_deg=90.0, roll_deg=0.0, bottom=None, vel=(0.0, 0.0, 0.0)):
    """An ObjectState for the board with its world-aligned bounding box, lying with its bottom at ``bottom``."""
    q = quat_from(math.radians(yaw_deg), math.radians(roll_deg))
    r = M._rotation(type("S", (), {"quat_xyzw": q})())
    half = np.array([M.BOARD_L, M.BOARD_W, M.BOARD_T]) / 2
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
    ext = np.abs(corners @ r.T).max(axis=0)
    bottom = TOP if bottom is None else bottom
    pos = np.array([x, y, bottom + ext[2]])
    return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - ext, pos + ext)


def objs(**kw):
    return {"board": board(**kw)}


def rect(center, angle_rad, half):
    """Corners of a rectangle in the table plane."""
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    r = np.array([[c, -s], [s, c]])
    hx, hy = half
    return (r @ np.array([[hx, hy], [-hx, hy], [-hx, -hy], [hx, -hy]]).T).T + np.asarray(center)


def separated(a, b, gap):
    """True if convex polygons a and b are at least ``gap`` apart along some edge normal (separating axis)."""
    for poly in (a, b):
        for i in range(len(poly)):
            e = poly[(i + 1) % len(poly)] - poly[i]
            n = np.array([-e[1], e[0]]) / np.linalg.norm(e)
            pa, pb = a @ n, b @ n
            if pa.max() + gap <= pb.min() or pb.max() + gap <= pa.min():
                return True
    return False


def finger_rect(xy, yaw):
    """Closed fingers' footprint: thin across the closing direction (world (-sin yaw, cos yaw))."""
    return rect(xy, yaw, (FINGER_HALF_ALONG, FINGER_HALF_ACROSS))


def board_rect(theta_deg, dxy=(0.0, 0.0)):
    return rect(np.asarray(M.START_XY) + np.asarray(dxy), math.radians(theta_deg), (M.BOARD_L / 2, M.BOARD_W / 2))


def segments(side):
    """(label, start xy, end xy, duration) for every waypoint of one arm, starting from the first waypoint."""
    wps = PLAN[side]
    return [(w.label, np.array(p.pos[:2]), np.array(w.pos[:2]), w.duration, p, w) for p, w in zip(wps, wps[1:])]


CONTACT_WORDS = ("close in", "push")


# -- 1. the file loads ------------------------------------------------------------------------------------------------


def test_the_task_file_loads_with_its_objects_and_hooks():
    assert TASK.scene == "station" and TASK.score_over == "end"
    assert TASK.free_objects == ("board",)
    assert all(callable(f) for f in (TASK.success, TASK.distance, TASK.score, TASK.demo))
    assert TASK.docs.startswith("On the station table")


# -- 2. the demo is reachable -----------------------------------------------------------------------------------------


@pytest.mark.parametrize("side", ["left", "right"])
def test_the_demo_is_reachable_within_the_joint_limits_at_20_hz(side):
    """The whole plan at 20 Hz, arcs included: every interpolated target solves and stays inside the limits."""
    arm = ArmPlan(PLAN[side], station.BASES[side])
    steps = round(arm.duration * HZ) + 5
    actions = np.array([arm.action7((k + 1) / HZ)[0] for k in range(steps)])
    assert arm.worst_ik_mm < 0.5 and arm.worst_ik_deg < 0.1, (side, arm.worst_ik_mm, arm.worst_ik_deg)
    assert np.all(actions[:, :6] >= ik.LOWER) and np.all(actions[:, :6] <= ik.UPPER)


def test_both_arms_run_in_step_and_the_demo_fits_the_run_limit():
    durations = {s: [w.duration for w in PLAN[s]] for s in PLAN}
    assert set(PLAN) == {"left", "right"}
    assert durations["left"] == durations["right"]  # the couple needs both hands pushing in the same intervals
    total = sum(durations["left"])
    assert total * HZ + 100 <= 0.55 * RUN_LIMIT_STEPS  # README --max-steps: plan plus ~100 steps
    assert PLAN["left"][-1].label == "hold still" and PLAN["left"][-1].duration >= 1.5


# -- 3. the fingers reach the edge without striking the table ---------------------------------------------------------


def test_the_push_height_puts_finger_on_the_edge_and_keeps_the_tips_off_the_table():
    tips = M.PUSH_Z - PAD_TIP_PAST_TCP
    assert PAD_TIP_PAST_TCP == pytest.approx(0.013)
    assert tips - TOP >= 0.005  # pad tips at least 5 mm above the table (wrist sag measured at 1-2 mm)
    assert TOP + M.BOARD_T - tips >= 0.010  # at least 1 cm of finger against the 19 mm edge
    assert M.PUSH_Z <= TOP + M.BOARD_T  # the TCP no higher than the board's top: the narrow part of the fingers
    # every waypoint is at push height, at travel height (tips over the board's top) or higher
    for side in PLAN:
        for w in PLAN[side]:
            z = w.pos[2]
            assert z == pytest.approx(M.PUSH_Z) or z - PAD_TIP_PAST_TCP >= TOP + M.BOARD_T + 0.015, (side, w.label)
    assert M.TRAVEL_Z - M.PUSH_Z >= 0.03 and M.TRAVEL_Z - TOP <= 0.14


@pytest.mark.parametrize("side", ["left", "right"])
def test_sideways_moves_happen_only_at_travel_height_or_while_pushing(side):
    for label, a, b, _, p, w in segments(side):
        if np.linalg.norm(b - a) > 1e-6 and w.pos[2] == pytest.approx(M.PUSH_Z):
            assert p.pos[2] == pytest.approx(M.PUSH_Z), label
            assert any(k in label for k in CONTACT_WORDS) or "back off" in label, label


# -- 4. gentle pushes -------------------------------------------------------------------------------------------------


@pytest.mark.parametrize("side", ["left", "right"])
def test_every_contact_move_keeps_speed_and_acceleration_low(side):
    """Smoothstep: peak speed 1.5 d / T, peak acceleration 6 d / T^2. The first GPU round dragged a board safely at
    4.5 cm/s and 0.02 m/s^2."""
    n = 0
    for label, a, b, dur, p, w in segments(side):
        if w.pos[2] == pytest.approx(M.PUSH_Z) and p.pos[2] == pytest.approx(M.PUSH_Z):
            d = float(np.linalg.norm(b - a))
            if "back off" in label:  # moving away from the edge: no load, but still slow
                assert 1.5 * d / dur <= 0.02, (label, d, dur)
                continue
            assert 1.5 * d / dur <= 0.045 + 1e-9, (label, d, dur)
            assert 6 * d / dur**2 <= 0.020 + 1e-9, (label, d, dur)
            n += 1
    assert n >= 10  # several short pushes, not one sweep


def test_each_push_turns_the_board_15_deg_or_less():
    for cfg in (M.PHASE_A, M.PHASE_B):
        t0, t1 = cfg["theta"]
        assert (t1 - t0) / cfg["steps"] <= 15.0
    assert M.PHASE_A["theta"][0] == M.START_YAW == 0.0
    assert M.PHASE_B["theta"][1] >= 90.0  # planned slightly past the goal; the squaring push removes the excess


# -- 5. the contacts form a couple and sit on the edges ---------------------------------------------------------------


@pytest.mark.parametrize("phase", ["A", "B"])
def test_the_two_hands_form_a_couple_turning_the_board_one_way(phase):
    torques = []
    for side in ("left", "right"):
        (px, py), (dx, dy) = M.CONTACTS[phase][side]
        torques.append(px * dy - py * dx)
        # the contact lies on an edge, with the whole finger face on it (clear of the corners)
        on_long = abs(abs(py) - M.BOARD_W / 2) < 1e-9 and abs(px) + FINGER_HALF_ALONG <= M.BOARD_L / 2 - 0.01
        on_end = abs(abs(px) - M.BOARD_L / 2) < 1e-9 and abs(py) + FINGER_HALF_ALONG <= M.BOARD_W / 2 - 0.005
        assert on_long or on_end, (phase, side)
        # the push points into the board, normal to that edge
        assert (dx, dy) in ((0.0, -math.copysign(1, py)), (-math.copysign(1, px), 0.0))
    assert all(t > 0.03 for t in torques)  # both counter-clockwise, each with at least 3 cm of lever
    a, b = (np.asarray(M.CONTACTS[phase][s][1]) for s in ("left", "right"))
    assert np.allclose(a + b, 0.0)  # equal and opposite directions: no net push


@pytest.mark.parametrize("phase", ["A", "B"])
@pytest.mark.parametrize("side", ["left", "right"])
def test_each_pressed_finger_follows_its_contact_point_round_the_turn(phase, side):
    """Waypoint k of the push lies on the board's edge at that step's angle (pressed in by PRESS), so the hands turn
    the board about its own centre."""
    cfg = M.PHASE_A if phase == "A" else M.PHASE_B
    thetas = np.linspace(*cfg["theta"], cfg["steps"] + 1)
    pushes = [w for w in PLAN[side] if w.label.startswith(f"{phase}: push")]
    assert len(pushes) == cfg["steps"]
    p, d = (np.asarray(v) for v in M.CONTACTS[phase][side])
    for w, th in zip(pushes, thetas[1:]):
        face = M.board_point(p, th)
        inward = M._rot2(th) @ d
        tcp = np.asarray(w.pos[:2])
        assert np.dot(face - tcp, inward) == pytest.approx(M.FINGER_HALF - M.PRESS, abs=1e-9)
        # the finger's flat face is parallel to the edge: closing direction along the push
        closing = np.array([-math.sin(w.yaw), math.cos(w.yaw)])
        assert abs(np.dot(closing, inward)) == pytest.approx(1.0, abs=1e-9)
    # the straight chord between waypoints stays within 1.5 mm of the circular arc
    r = float(np.linalg.norm(p - d * (M.FINGER_HALF - M.PRESS)))
    step = math.radians(thetas[1] - thetas[0])
    assert r * (1 - math.cos(step / 2)) < 0.0015


def test_the_hands_stay_on_their_own_side_and_apart():
    for side, sign in (("left", 1), ("right", -1)):
        for w in PLAN[side]:
            assert sign * w.pos[1] >= -0.02, (side, w.label)  # never more than 2 cm past the midline
    for wl, wr in zip(PLAN["left"], PLAN["right"]):
        assert np.linalg.norm(np.subtract(wl.pos[:2], wr.pos[:2])) > 0.10, (wl.label, wr.label)


# -- 6. hands land on the table, not on the board, even when the board lags the plan ------------------------------------

# Where the board may be when each phase's hands come down: (planned angle, lag allowed, lead allowed, drift allowed)
LANDING = {
    "A": (0.0, 1.0, 1.0, 0.003),  # the untouched start
    "B": (M.PHASE_A["theta"][1], 12.0, 4.0, 0.010),  # after phase A: the soft wrists make it lag
    "square": (M.PHASE_B["theta"][1], 14.0, 4.0, 0.010),  # after phase B
}


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("stage", list(LANDING))
def test_hands_come_down_clear_of_the_board(stage, side):
    planned, lag, lead, slack = LANDING[stage]
    downs = [w for w in PLAN[side] if w.label.startswith(f"{stage}: down")]
    assert len(downs) == 1
    w = downs[0]
    f = finger_rect(w.pos[:2], w.yaw)
    for th in np.linspace(planned - lag, planned + lead, 17):
        for ang in np.linspace(0, 2 * math.pi, 8, endpoint=False):
            dxy = slack * np.array([math.cos(ang), math.sin(ang)])
            assert separated(f, board_rect(th, dxy), 0.002), (stage, side, th, dxy)


@pytest.mark.parametrize("side", ["left", "right"])
def test_hands_end_clear_of_the_turned_board(side):
    w = PLAN[side][-1]
    assert separated(finger_rect(w.pos[:2], w.yaw), board_rect(90.0), 0.05)


def test_the_squaring_push_leaves_the_centre_well_inside_the_full_score_band():
    # a perfectly turned board's near edge starts at START_X - W/2 and is pushed SQUARE_OVER past it
    assert M.SQUARE_OVER <= M.FULL["drift"] / 2
    sq = [w for w in PLAN["left"] if w.label == "square: push the near edge to the line"][0]
    face_x = sq.pos[0] + M.FINGER_HALF - M.PRESS
    assert face_x == pytest.approx(M.START_XY[0] - M.BOARD_W / 2 + M.SQUARE_OVER)
    # the two hands push the same edge, far enough apart to set its angle, inside its 24 cm length
    spread = M.SQUARE_Y["left"] - M.SQUARE_Y["right"]
    assert 0.10 <= spread and spread / 2 + FINGER_HALF_ALONG <= M.BOARD_L / 2 - 0.02


# -- 7. scoring ---------------------------------------------------------------------------------------------------------


@pytest.mark.parametrize("yaw", [90.0, -90.0, 270.0, 94.0, 85.5])
def test_a_quarter_turn_in_place_succeeds_either_way_round(yaw):
    o = objs(yaw_deg=yaw, x=M.START_XY[0] + 0.008, y=0.004)
    assert TASK.score(o) == 1.0 and TASK.success(o), yaw


def test_distance_is_zero_at_the_goal_and_grows_with_yaw_and_drift():
    assert TASK.distance(objs()) == pytest.approx(0.0, abs=1e-9)
    assert TASK.distance(objs(yaw_deg=0.0)) == pytest.approx(math.pi / 2 * M.HALF_DIAG)
    assert TASK.distance(objs(x=0.33)) == pytest.approx(0.03)


@pytest.mark.parametrize(
    "what, o, value",
    [
        ("10 deg short", objs(yaw_deg=80.0), 0.5),
        ("in angle, 30 mm off", objs(x=M.START_XY[0] + 0.030), 0.5),
        ("15 deg over, 39 mm off", objs(yaw_deg=105.0, y=0.039), 0.5),
        ("turned, 60 mm away", objs(x=M.START_XY[0] + 0.06), 0.2),
        ("turned 14 deg off, 95 mm away", objs(yaw_deg=76.0, y=-0.095), 0.2),
    ],
)
def test_partial_credit_tiers(what, o, value):
    assert TASK.score(o) == value, what
    assert not TASK.success(o), what


def test_full_score_while_still_moving_is_not_yet_success():
    o = objs(vel=(0.03, 0.0, 0.0))
    assert TASK.score(o) == 1.0 and not TASK.success(o)


# -- 8. construct validity: shortcuts and failures score 0 -------------------------------------------------------------


@pytest.mark.parametrize(
    "what, o",
    [
        ("untouched start", objs(yaw_deg=0.0)),
        ("slid 5 cm sideways, not turned", objs(yaw_deg=0.0, y=0.05)),
        ("slid 15 cm forward, not turned", objs(yaw_deg=0.0, x=0.45)),
        ("slid 20 cm sideways, slightly turned", objs(yaw_deg=10.0, y=-0.20)),
        ("turned half way", objs(yaw_deg=45.0)),
        ("turned 20 deg short of a quarter", objs(yaw_deg=70.0)),
        ("turned, but 12 cm away", objs(x=M.START_XY[0] + 0.12)),
        ("turned 180 deg (long side still forward)", objs(yaw_deg=180.0)),
        ("pushed off the far edge", objs(x=0.95, bottom=TOP - 0.30)),
        ("pushed off the side, falling", objs(y=0.66, bottom=TOP - 0.01)),
        ("on the floor", objs(bottom=0.0)),
        ("propped up on a finger at 8 deg", objs(roll_deg=8.0, bottom=TOP)),
        ("held 2 cm above the table", objs(bottom=TOP + 0.02)),
        ("standing on its long edge", objs(roll_deg=90.0)),
    ],
)
def test_shortcuts_and_failures_score_zero(what, o):
    assert TASK.score(o) == 0.0, what
    assert not TASK.success(o), what


def test_flat_upside_down_still_counts_as_lying_flat():
    o = objs(roll_deg=180.0)
    assert TASK.score(o) == 1.0


# -- 9. layout ----------------------------------------------------------------------------------------------------------


def test_the_board_rests_on_the_table_and_its_whole_turn_stays_on_it():
    b = M.BOARD
    assert b.pos[2] - b.size[2] / 2 - TOP == pytest.approx(M.REST_GAP)
    assert b.size == (M.BOARD_L, M.BOARD_W, M.BOARD_T) and b.yaw == M.START_YAW
    tx = (station.TABLE_CENTER[0] - station.TABLE_SIZE[0] / 2, station.TABLE_CENTER[0] + station.TABLE_SIZE[0] / 2)
    ty = (station.TABLE_CENTER[1] - station.TABLE_SIZE[1] / 2, station.TABLE_CENTER[1] + station.TABLE_SIZE[1] / 2)
    assert M.TABLE_X == pytest.approx(tx) and M.TABLE_Y == pytest.approx(ty)
    x, y = M.START_XY
    r = M.HALF_DIAG + 0.03
    assert M.TABLE_X[0] + 0.10 < x - r and x + r < M.TABLE_X[1] and M.TABLE_Y[0] < y - r and y + r < M.TABLE_Y[1]
    assert x - r > 0.0  # the swept circle stays in front of the arm bases


def test_the_board_cannot_be_grasped_from_above():
    assert GRIPPER_MAX == pytest.approx(0.0939, abs=1e-3)
    assert min(M.BOARD_L, M.BOARD_W) >= GRIPPER_MAX + 0.04  # both sides at least 4 cm wider than the open gripper


def test_the_board_starts_long_side_forward_and_scores_zero():
    start = objs(yaw_deg=math.degrees(M.START_YAW))
    assert M.yaw_error_deg(start["board"]) == pytest.approx(90.0)
    assert TASK.score(start) == 0.0


# -- task.json ----------------------------------------------------------------------------------------------------------


def test_task_json_has_every_field_and_names_no_one():
    meta = json.loads((HERE / "task.json").read_text())
    for key in ("title", "category", "keywords", "instruction", "context", "setup", "scoring", "run_limit",
                "fidelity", "scores"):
        assert meta.get(key), key
    # scores is filled after the Dream-1 run and media by the upload script
    assert meta["scores"] == "not yet run" or meta["scores"].startswith("Dream-1 run: "), meta["scores"]
    assert meta["category"] in {"Impact on jobs", "Impact on AI timelines", "Safety and alignment",
                                "Societal relevance", "Other"}
    assert "2,400 steps" in meta["run_limit"]
    text = json.dumps(meta).lower() + (HERE / "README.md").read_text().lower() + TASK.docs.lower()
    for word in ("robocurve_sprint", "knowledge base", "kairos"):
        assert word not in text, word
