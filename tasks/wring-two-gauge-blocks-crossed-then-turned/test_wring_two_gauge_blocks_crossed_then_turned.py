"""The kit's 7 tests for wring_two_gauge_blocks_crossed_then_turned (written by sync_kit.py --with-tests; edit freely below the line).

    PYTHONPATH=<yam-isaac-lab-arena> PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q

The task file provides ``TEXT_NUMBERS`` [(snippet as task.json says it, the code's value in SI)], ``CHEATS``
{demo mode: the most it may score} and ``HOLD`` {"mech.joint": the most it may move with no command, mm or deg}.
Tests 4-6 read physcheck runs (``.planning/kit/kit_check.sh <task>``) and run holdcheck; they skip where our pipeline
isn't (a public copy of the task).
"""

import pathlib
import sys

import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from ourkit import testkit as T  # noqa: E402
from yam_arena import task_file  # noqa: E402

TASK = task_file.load(T.task_file_of(HERE))
M = TASK.module
SPEC = T.spec(HERE)
README = (HERE / "README.md").read_text() if (HERE / "README.md").exists() else ""
PIPE = T.pipeline_dir(HERE)
needs_pipeline = pytest.mark.skipif(PIPE is None, reason="no .planning/pipeline (public copy)")


def test_1_text_matches_code():
    assert getattr(M, "TEXT_NUMBERS", None), "list TEXT_NUMBERS in the task file"
    assert T.check_text_numbers(SPEC, M.TEXT_NUMBERS) == []


def test_2_blind_text():
    assert T.check_blind(SPEC, README) == []


def test_3_json_fields_and_run_limit():
    assert T.check_json(SPEC, M) == []


@needs_pipeline
@pytest.mark.parametrize("mode", sorted(getattr(M, "CHEATS", {"none": 0.0})))
def test_4_cheats_score_low(mode):
    cap = getattr(M, "CHEATS", {"none": 0.0})[mode]
    r, problem = T.physcheck_result(HERE, mode)
    assert r is not None and not problem, problem
    assert r["final_score"] <= cap + 1e-9, f"{mode} scored {r['final_score']} (cap {cap})"


@needs_pipeline
def test_5_demo_scores_1_in_physcheck():
    r, problem = T.physcheck_result(HERE, "demo")
    assert r is not None and not problem, problem
    assert r["final_score"] == 1.0 and r["success"], r["final_score"]
    assert r["steps"] <= T.run_limit_steps(SPEC)
    assert not r["arm_arm"], f"arms touched: {r['arm_arm']}"


@needs_pipeline
def test_6_state_holds_without_command():
    h = T.run_hold(HERE, seconds=5.0)
    caps = getattr(M, "HOLD", {})
    for joint, row in h["joints"].items():
        assert row["drift"] <= caps.get(joint, 0.5), f"{joint} moved {row['drift']} {row['unit']} with no command"
    assert all(v <= 2.0 for v in h["body_mm"].values()), h["body_mm"]
    assert (h["score"] or 0.0) == 0.0


def test_7_text_limits():
    assert T.check_limits(SPEC) == []


def _stack(yaw_deg: float, tilt_deg: float, lift: float = 0.0, dx: float = 0.0, dy: float = 0.0) -> dict:
    """The 25 mm block on the 50 mm block's face, turned ``yaw_deg`` and tilted ``tilt_deg`` about x (across the face's
    9 mm), its low edge touching the face: the high edge is 9 mm x tan(tilt) clear."""
    import math

    import numpy as np
    a, b = math.radians(yaw_deg), math.radians(tilt_deg)
    rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
    rx = np.array([[1, 0, 0], [0, math.cos(b), -math.sin(b)], [0, math.sin(b), math.cos(b)]])
    r = rx @ rz
    w = math.sqrt(max(1e-12, 1 + np.trace(r))) / 2
    q = ((r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w), (r[1, 0] - r[0, 1]) / (4 * w), w)
    face_z = M.SURF + M.BASE_L
    foot = (M.BASE_X + dx, M.BASE_Y + dy, face_z + M.BLOCK_T / 2 * math.sin(b) + lift)
    return {"base": M.k.mech_state(M.BASE, (M.BASE_X, M.BASE_Y, M.SURF)), "top": M.k.mech_state(M.TOPB, foot, q)}


def _turn(tilt_deg: float, lift: float = 0.0, force: float = 6.0, free: bool = True) -> dict:
    M.RUN.update(turn=0.0, prev=None, wrung=False, base_free=free, force=force)
    faces = []
    for i in range(19):  # crossed (90 deg) into line, pressed at ``force`` N, held, ``free``: no hand on the 50 mm block
        r = M.relation(_stack(90.0 - 5.0 * i, tilt_deg, lift))
        faces.append(r["face"])
        M._watch_wring(r, True)
    return {"wrung": M.RUN["wrung"], "turn": M.RUN["turn"], "face_mm": [1000 * f for f in faces]}


def test_8_tilted_turn_on_one_edge_counts_nothing():
    """The reviewer's case: 1.5 deg of tilt, one edge on the face and the other 0.24 mm clear, through a pressed,
    held 90 deg turn: no step counts, so nothing is wrung. The same turn flat on the face is wrung."""
    tilted = _turn(1.5)
    assert all(0.23 <= f <= 0.24 for f in tilted["face_mm"]), tilted["face_mm"]
    assert not tilted["wrung"] and tilted["turn"] == 0.0, tilted
    flat = _turn(0.0)
    assert all(f <= 1e-6 for f in flat["face_mm"]) and flat["wrung"], flat
    assert _turn(0.3)["wrung"]  # 0.047 mm at the high edge: within 0.05 mm, face to face
    assert not _turn(0.4)["wrung"]  # 0.063 mm: over


def test_9_gap_counts_nothing():
    """Round 1: 'accepts a 0.2 mm face gap'. The same pressed, held 90 deg turn with the faces 0.2 mm apart counts
    nothing; 0.06 mm apart (over 0.05 mm) counts nothing either."""
    assert not _turn(0.0, lift=0.0002)["wrung"]
    assert not _turn(0.0, lift=0.00006)["wrung"]


def test_10_no_press_counts_nothing():
    """Round 1: 'requires no pressing force'. Face to face but pressed with nothing, or 2.9 N (under the 3 N floor):
    nothing counts. At 3 N it counts."""
    assert not _turn(0.0, force=0.0)["wrung"]
    assert not _turn(0.0, force=2.9)["wrung"]
    assert _turn(0.0, force=3.0)["wrung"]


def test_11_hand_on_the_base_counts_nothing():
    """Our mock's point on round 1's fix: a hand on the 50 mm block (or the plate or scale) could supply the scale's
    3 N. With a gripper within 5 mm of any of them, the face-to-face, pressed turn counts nothing."""
    assert not _turn(0.0, free=False)["wrung"]


def test_12_one_edge_0_06_mm_counts_nothing():
    """Fallback tolerance check: one edge down, the other 0.06 mm clear (0.38 deg over the 9 mm face), through a
    pressed, held 90 deg turn: nothing counts."""
    import math
    tilt = math.degrees(math.asin(0.00006 / 0.009))
    r = _turn(tilt)
    assert all(f > 0.05 for f in r["face_mm"]) and not r["wrung"], r


def test_13_peel_and_put_back_loses_the_wringing():
    """Red team: after the latch, tip the block 12 deg on one long edge (far edge 1.9 mm up, its centre under 1 mm up),
    lay it back and let go. A real peel breaks the film; here any corner of the overlap more than 1 mm off the face
    loses the wringing, and laying it back does not restore it. A small rock while squaring up (0.5 deg) keeps it."""
    assert _turn(0.0)["wrung"]  # a flat, pressed turn that ends in line
    peel = M.relation(_stack(0.0, 12.0))
    assert peel["gap"] <= M.SEP < peel["face"]
    M._watch_wring(peel, True)
    assert not M.RUN["wrung"]
    M._watch_wring(M.relation(_stack(0.0, 0.0)), True)
    assert not M.RUN["wrung"]
    assert _turn(0.0)["wrung"]
    M._watch_wring(M.relation(_stack(0.0, 0.5)), True)
    assert M.RUN["wrung"]


def _box_hull(lo, hi):
    import itertools

    import numpy as np
    from yam_arena.analysis import Hull
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    planes = [(*n, d) for i in range(3) for n, d in ((tuple(np.eye(3)[i]), hi[i]), (tuple(-np.eye(3)[i]), -lo[i]))]
    return Hull(vertices=np.array(list(itertools.product(*zip(lo, hi)))), planes=np.array(planes))


def test_14_an_arm_link_on_the_plate_or_scale_is_not_hands_off():
    """Red team: the hands-off fence saw only the grippers, so a forearm resting on the plate could load the scale.
    It now takes every link of both arms; a link, or a narrow fingertip between the fence's grid points, on or within
    5 mm of the plate, the scale's platform or the 50 mm block makes the step not count."""
    import numpy as np
    from yam_arena import station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, READY
    pts_b = M.k.score.surface_points(_stack(0.0, 0.0)["base"], "", M.BASE_BOXES, 0.01)
    arms = [M.arm_hulls(dict(zip(ARM_JOINTS, READY)) | dict.fromkeys(FINGER_JOINTS, 0.0), station.BASES[a])
            for a in ("left", "right")]
    assert {n for n, _ in arms[0]} - {"gripper", "tip_left", "tip_right"}  # arm links, not only the gripper
    assert M.hands_off(arms, pts_b)  # both arms at their ready pose: clear
    x, s = M.PLATE_X, M.SURF
    forearm = _box_hull((x + 0.05, 0.03, s), (x + 0.30, 0.09, s + 0.06))  # lying on the plate's top, reaching past it
    fingertip = _box_hull((x + 0.068, 0.068, s + 0.002), (x + 0.072, 0.072, s + 0.05))  # 4 mm wide, between grid points
    on_edge = _box_hull((x - 0.4, -0.02, s + 0.003), (x - 0.14, 0.02, s + 0.05))  # over the plate's top edge only
    beside = _box_hull((x - 0.4, -0.02, M.TOP), (x - M.PLATE_L / 2 - 0.007, 0.02, M.TOP + 0.01))  # touching the platform
    for hull in (forearm, fingertip, on_edge, beside):
        assert not M.hands_off([[("link", hull)]], pts_b)
    assert M.hands_off([[("link", _box_hull((x, 0.0, s + 0.02), (x + 0.1, 0.05, s + 0.08)))]], pts_b)  # 20 mm above
    M.RUN.update(turn=0.0, prev=None, wrung=False, base_free=M.hands_off([[("link", forearm)]], pts_b), force=6.0)
    for i in range(19):  # the pressed, face-to-face turn with that forearm on the plate: nothing counts
        M._watch_wring(M.relation(_stack(90.0 - 5.0 * i, 0.0)), True)
    assert not M.RUN["wrung"] and M.RUN["turn"] == 0.0


def _moved(dx: float, dy: float, dz: float = 0.0) -> dict:
    """The 25 mm block in line with the 50 mm block but moved (dx, dy, dz) m from its place on the face."""
    s = _stack(0.0, 0.0)
    s["top"] = M.k.mech_state(M.TOPB, (M.BASE_X + dx, M.BASE_Y + dy, M.SURF + M.BASE_L + dz))
    return s


def test_15_leaving_the_face_loses_the_wringing():
    """Our mock on the revision: 'wrung' survived the block leaving the face, because with no overlap there is no
    corner to measure and a block below the face was never tested. Slid 12 mm off across the 9 mm face (nothing over
    it), stood on the plate beside the 50 mm block, or lowered beside the face: each loses the wringing, and putting
    it back does not restore it. A block beside the face, over nothing, is not on the face either (0, not 0.25)."""
    for dx, dy, dz in ((0.0, 0.012, 0.0), (0.0, 0.015, -M.BASE_L), (0.0, 0.012, -0.002), (0.019, 0.010, 0.0)):  # no overlap
        assert _turn(0.0)["wrung"]
        off = _moved(dx, dy, dz)
        M._watch_wring(M.relation(off), True)
        assert not M.RUN["wrung"], (dx, dy, dz)
        assert M.grade(off) == 0.0, (dx, dy, dz)  # over nothing: not on the face
        back = _moved(0.0, 0.0)
        M._watch_wring(M.relation(back), True)
        assert not M.RUN["wrung"] and M.grade(back) == 0.25, (dx, dy, dz)  # put back: on the face, not wrung
    assert _turn(0.0)["wrung"]
    slid = _moved(0.002, 0.004)  # slid 4 mm across, 5 mm of the 9 mm still over the face, flat: still wrung
    M._watch_wring(M.relation(slid), True)
    assert M.RUN["wrung"] and M.grade(slid) == 0.5


def test_16_a_turn_made_before_the_contact_counts_does_not_count():
    """Our mock on the revision: the first counted step took the yaw change since the step before it, which was not
    counted. Only the turn between two counted steps counts: 55 deg turned with no press, then 10 deg pressed, is
    10 deg, not 65."""
    M.RUN.update(turn=0.0, prev=None, wrung=False, base_free=True, force=0.0)
    M._watch_wring(M.relation(_stack(90.0, 0.0)), True)  # crossed, no press: not counted
    M.RUN["force"] = 6.0
    for yaw in (35.0, 30.0, 25.0):  # pressed from here on, already turned 55 deg
        M._watch_wring(M.relation(_stack(yaw, 0.0)), True)
    assert not M.RUN["wrung"] and abs(abs(M.RUN["turn"]) - 0.1745) < 1e-3, M.RUN["turn"]
    sliver = _moved(0.0, 0.0089, 0.0005)  # held in the air, a 0.1 mm sliver over the face: not on the face
    assert M.grade(sliver) == 0.0 and M.grade(_moved(0.0, 0.004)) == 0.25  # its centre over the face: on it


def test_17_peel_on_a_corner_of_overlap_loses_the_wringing():
    """Red team on the revision: the peel was read only at the corners of the overlap, so a wrung block slid until
    only its corner was over the face could be tipped 26 deg (a corner 8 mm up), laid back and still score. The peel
    is now read at the block's four lower corners against the face's plane: any corner more than 1 mm off loses it."""
    assert _turn(0.0)["wrung"]
    tipped = M.relation(_stack(13.0, 26.0, dx=0.0157, dy=0.008))
    assert tipped["off"] <= M.ON_TOL and tipped["face"] < float("inf") and tipped["peel"] > M.SEP, tipped
    M._watch_wring(tipped, True)
    assert not M.RUN["wrung"]
    M._watch_wring(M.relation(_stack(0.0, 0.0)), True)
    assert not M.RUN["wrung"]  # laid back in line: not restored
    assert _turn(0.0)["wrung"]
    M._watch_wring(M.relation(_stack(0.0, 2.4, dy=0.0077)), True)  # the demo's worst: hung 7.7 mm off, tipped 2.4 deg
    assert M.RUN["wrung"] and M.RUN["lift"] < 0.0006


def _step(states: dict) -> bool:
    """One scored step outside a simulator (no arms); True if the trial has been zeroed."""
    states = dict(states, plate=M.k.mech_state(M.PLATE, (M.PLATE_X, 0.0, M.TOP)))
    M._watch(states)
    return bool(M.FELL)


def test_18_dropped_is_below_the_plate_top_as_the_text_says(monkeypatch):
    """Our mock on the revision: the code also zeroed a block carried past the plate's outline in the air, which the
    scoring text never said. Dropped is what the text says: a block more than 5 mm below the plate's top. A block
    carried out past the plate's edge above it is not dropped; one below the top is, and so is a 50 mm block lifted
    more than 5 mm, and a block tipped more than 30 deg."""
    monkeypatch.setattr(M.REC, "tick", lambda *a: None), monkeypatch.setattr(M.REC, "reset", lambda *a: None)  # no trace
    M._reset({})
    carried = _moved(0.0, 0.0)
    carried["top"] = M.k.mech_state(M.TOPB, (M.PLATE_X + M.PLATE_L / 2 + 0.05, 0.0, M.SURF + 0.02))  # 5 cm past the edge
    assert not _step(carried)
    M._reset({})
    low = _moved(0.0, 0.0)
    low["top"] = M.k.mech_state(M.TOPB, (M.PLATE_X + M.PLATE_L / 2 + 0.05, 0.0, M.SURF - 0.006))  # 6 mm below the top
    assert _step(low) and M.grade(low) == 0.0
    assert M.grade(_moved(0.0, 0.0)) == 0.0  # put back: still 0
    M._reset({})
    lifted = _moved(0.0, 0.0)
    lifted["base"] = M.k.mech_state(M.BASE, (M.BASE_X, M.BASE_Y, M.SURF + 0.006))  # the 50 mm block 6 mm up
    assert _step(lifted)
    M._reset({})
    assert not _step(_stack(0.0, 29.0)) and _step(_stack(0.0, 31.0))  # tipped past 30 deg: fallen
    M._reset({})
