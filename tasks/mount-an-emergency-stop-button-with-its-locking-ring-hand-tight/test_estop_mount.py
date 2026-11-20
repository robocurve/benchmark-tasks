"""The emergency-stop task's scene and rules, without the simulator: every state is set directly."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest
from yam_arena import task_file

TASK = Path(__file__).resolve().parent / "task.py"  # the task file in this test's own folder


@pytest.fixture
def m():
    before = sys.modules.get("mechkit")
    module = task_file.load(TASK).module
    module.begin(0)
    yield module
    module.mk.MECH.clear()
    sys.modules.pop("mechkit", None)
    if before is not None:
        sys.modules["mechkit"] = before


def test_the_scene_is_the_one_specified(m):
    assert m.INSTRUCTION == ("Mount the emergency-stop button in the panel hole with its tab in the notch, then screw its locking ring on from "
                             "behind until it is hand-tight, and let go.")
    assert m.RUN_LIMIT_STEPS <= 3000
    b = m.mk.MECH.bodies
    # The hole: 22.3 mm with the 3.2 mm notch at the top, 24.1 mm across both, centred at (0.25, 0), 16 cm up
    assert tuple(m.HOLE) == pytest.approx((0.25, 0.0, m.TOP + 0.16)) and 2 * m.HOLE_R == pytest.approx(0.0223)
    assert m.HOLE_R + m.NOTCH_TO == pytest.approx(0.0241) and m.NOTCH_W == pytest.approx(0.0032)
    # The operator stands on its stem's end at (0.30, -0.28); the ring two turns on, hanging 5 mm clear of the bench
    assert b["operator"].pos[2] + m.STEM_FROM == pytest.approx(m.TOP) and tuple(b["operator"].pos[:2]) == pytest.approx((0.30, -0.28))
    assert b["ring"].pos[2] - m.RING_H == pytest.approx(m.TOP + 0.005) and m.A_START - m.A_END == pytest.approx(0.002)
    assert m.A_TOUCH - m.A_END == pytest.approx(0.0095)  # 9.5 turns from the thread's end to the panel's back
    assert 2 * m.RING_OUT == pytest.approx(0.0284) and m.RING_H == 0.005 and m.LEAD == 0.001
    # No tool: the ring is tightened by hand (designer 2026-11-20 11:56)
    assert set(b) == {"panel", "operator", "ring", "carrier", "turner", "tightener", "seat"} and m.PARTS == ("operator", "ring")
    # The head stands 32 mm in front of the panel; the thread 12 mm behind the bezel; a 20 mm stem 8 mm long
    assert m.HEAD_TO == pytest.approx(0.032) and m.THREAD_TO - m.THREAD_FROM == pytest.approx(0.012) and m.THREAD_FROM - m.STEM_FROM == pytest.approx(0.008)
    assert m.TIGHT_K * math.radians(30.0) == pytest.approx(2.0)
    # The four starts by the trial's seed; the harness turns --seed 0 into the trial seed 1415158438: the third (345)
    from inspect_robots.rollout import derive_seed

    assert [round(math.degrees(a), 6) for a in m.START_ANGLES] == [30.0, 120.0, 345.0, 300.0]
    assert m.start_angle(None) == m.start_angle(0) == m.START_ANGLES[0]
    assert derive_seed(0, None, 0) == 1415158438 and m.start_angle(1415158438) == m.START_ANGLES[2]


def test_every_weld_made_during_a_trial_is_anchored_on_its_child(m):
    """mechkit.make keeps a weld's frame where its anchor sat on the child as built: anchored away from the child it is
    a long lever the solver holds loosely (e2n: a weld anchored 0.56 m from its child let the two come 2.6 mm apart while
    joined)."""
    made_later = {name: j for name, j in m.mk.MECH.joints.items() if j.kind == "weld" and not j.made}
    assert {"set", "on_again", "clamp", "seated"} <= set(made_later)
    for name, j in made_later.items():
        assert np.linalg.norm(np.subtract(j.anchor, m.mk.MECH.bodies[j.child].pos)) < 1e-9, name


def test_the_hole_takes_the_barrel_and_the_tab_only_in_the_notch(m):
    """The plate's shapes leave the barrel's 22 mm free on the hole's axis, and its tab (12 mm out) only at the top."""
    panel = m.mk.MECH.bodies["panel"]
    boxes = [(s.bounds(), s) for s in panel.shapes if s.solid]

    def inside(point) -> bool:
        local = np.asarray(point) - m.HOLE
        for (low, high), shape in boxes:
            rot = m.mk.quat_to_matrix(shape.quat)
            q = rot.T @ (local - np.asarray(shape.pos))
            half = np.asarray(shape.dims) / 2
            if np.all(np.abs(q) <= half - 1e-9):
                return True
        return False

    for a in np.linspace(0.0, 2 * math.pi, 3601):  # a tenth of a degree apart: the notch's sides once reached 0.03 mm into it
        for y in (-0.0009, 0.0, 0.0009):
            assert not inside(m.HOLE + np.array([0.0111 * math.cos(a), y, 0.0111 * math.sin(a)])), a  # the barrel's surface, 0.1 mm to spare
    for x in np.linspace(-0.0015, 0.0015, 31):  # the tab's whole section (3 mm wide, out to 12 mm) passes through the notch
        for z in np.linspace(0.0108, 0.0120, 13):
            for y in (-0.0009, 0.0, 0.0009):
                assert not inside(m.HOLE + np.array([x, y, z])), (x, y, z)
    section = [(x, z) for x in np.linspace(-0.0015, 0.0015, 31) for z in np.linspace(0.0112, 0.0120, 9)]
    for deg in (1.0, -1.0, 6.0, 10.0, 45.0, 90.0, 180.0):  # the tab turned off the notch: some of it meets the plate
        c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
        assert any(inside(m.HOLE + np.array([c * x - s * z, 0.0, s * x + c * z])) for x, z in section), deg
    assert inside(m.HOLE + np.array([0.0, 0.0, 0.0135]))  # over the notch


TURN = 2 * math.pi


def no_simulator(m, monkeypatch) -> dict:
    """``set_limits`` and ``write_pose`` without the simulator: the stops noted (name -> [lower, upper]), a written
    pose put in as the body's pose."""
    mk = m.mk
    stops: dict = {}

    def set_limits(name, lower=None, upper=None):
        old = stops.get(name, [None, None])
        stops[name] = [old[0] if lower is None else lower, old[1] if upper is None else upper]

    def write_pose(name, position, quat=None, **_):
        rotation = mk.MECH.poses[name][1] if quat is None else mk.quat_to_matrix(np.asarray(quat, dtype=float))
        mk.MECH.poses[name] = (np.asarray(position, dtype=float), rotation)

    monkeypatch.setattr(mk, "set_limits", set_limits)
    monkeypatch.setattr(mk, "write_pose", write_pose)
    return stops


def test_the_thread_moves_the_rings_stops_a_millimetre_a_turn_and_lets_it_off_the_end(m, monkeypatch):
    stops = no_simulator(m, monkeypatch)
    m.mk.set_state(time=0.5)
    travel = m.mk.MECH.travel
    travel["turn"] = -1.5 * TURN  # unscrewed one and a half turns: 1.5 mm nearer the thread's end
    m.thread()
    assert stops["along"] == pytest.approx([-0.0015 - m.PLAY, -0.0015 + m.PLAY]) and m.STATE["ring_on"] and m.ring_weld() == "on"
    travel["turn"] = 3.0 * TURN  # screwed on three turns from where it started
    m.thread()
    assert stops["along"] == pytest.approx([0.003 - m.PLAY, 0.003 + m.PLAY])
    travel["turn"] = -2.01 * TURN  # past the thread's end: off and free
    m.thread()
    assert not m.STATE["ring_on"] and m.mk.is_cut("on") and m.STATE["events"][-1] == (0.5, "ring off the thread") and m.turns_on() == 0.0


def test_a_free_ring_is_started_again_only_in_line_on_the_threads_end_while_turned_or_pushed_on(m, monkeypatch):
    no_simulator(m, monkeypatch)
    mk = m.mk
    mk.set_state(time=0.5)
    mk.MECH.travel["turn"] = -2.01 * TURN
    m.thread()
    assert not m.STATE["ring_on"]
    p, r = mk.built_pose("operator")
    axis = r @ m.Z
    held_at = -0.00201  # the carrier's travel along the barrel where the thread let the ring go

    def ring(front: float, off: float = 0.0, tilt_deg: float = 0.0, spin: float = 0.0, closing: float = 0.0) -> bool:
        rotation = mk.axis_angle(r @ m.X, math.radians(tilt_deg)) @ r
        mk.set_state({"ring": (p + r @ np.array([off, 0.0, front]), rotation)}, {"ring": (closing * axis, spin * axis), "operator": (np.zeros(3), np.zeros(3))},
                     joints={"turn": -2.01 * TURN, "along": held_at}, time=1.0)  # fmt: skip
        m.thread()
        return m.STATE["ring_on"]

    assert not ring(m.A_END - 0.0005)  # in line on the thread's end, held still
    assert not ring(m.A_END - 0.0005, off=0.0015, spin=3.0)  # turned on, 1.5 mm off the barrel's line
    assert not ring(m.A_END - 0.0005, tilt_deg=6.0, spin=3.0)  # turned on, tilted 6 degrees
    assert not ring(m.A_END - 0.0015, spin=3.0)  # turned on, 1.5 mm short of the thread's end
    assert not ring(m.A_END - 0.0005, spin=-3.0)  # turned the other way
    assert not ring(m.A_END - 0.0005, closing=0.01)  # pushed on in line without a turn
    assert not ring(m.A_END - 0.0005, spin=3.0, closing=-0.01)  # turned on but drawn away at 10 mm/s
    assert ring(m.A_END - 0.0005, off=0.0008, tilt_deg=4.0, spin=3.0)  # turned on in line: started
    assert m.ring_weld() == "on_again" and not mk.is_cut("on_again") and m.STATE["events"][-1] == (1.0, "ring taken on by the thread")
    assert m.ring_along() == pytest.approx(m.A_START + held_at) and m.ring_off_axis()[0] == pytest.approx(0.0, abs=1e-12)  # on the turner
    mk.MECH.travel["turn"] = -2.02 * TURN  # and off again
    m.thread()
    assert not m.STATE["ring_on"] and mk.is_cut("on_again")
    mk.set_state({"ring": (p + r @ np.array([0.0, 0.0, m.A_END - 0.0003]), r)}, {"ring": (0.01 * axis, np.zeros(3)), "operator": (np.zeros(3), np.zeros(3))},
                 joints={"turn": -2.02 * TURN, "along": held_at}, time=2.0)  # fmt: skip
    m.thread()  # pushed on in line at 10 mm/s without a turn: not started
    assert not m.STATE["ring_on"]
    mk.set_state({"ring": (p + r @ np.array([0.0, 0.0, m.A_END - 0.0003]), r)}, {"ring": (0.01 * axis, 1.0 * axis), "operator": (np.zeros(3), np.zeros(3))},
                 joints={"turn": -2.02 * TURN, "along": held_at}, time=2.05)  # fmt: skip
    m.thread()  # and turned on while pushed: started
    assert m.STATE["ring_on"] and m.ring_weld() == "on_again"
    mk.MECH.travel["turn"] = -2.03 * TURN  # off once more; then found in line 3 mm onto the thread (slid on within a step)
    m.thread()
    assert not m.STATE["ring_on"]
    mk.set_state({"ring": (p + r @ np.array([0.0, 0.0, m.A_END + 0.003]), r)}, {"ring": (np.zeros(3), np.zeros(3)), "operator": (np.zeros(3), np.zeros(3))},
                 joints={"turn": -2.03 * TURN, "along": held_at}, time=3.0)  # fmt: skip
    m.thread()
    assert m.STATE["ring_on"] and m.ring_along() == pytest.approx(m.A_START + held_at)  # caught, put back where the thread's first turn stops it
    mk.MECH.travel["turn"] = -2.04 * TURN
    m.thread()
    mk.set_state({"ring": (p + r @ np.array([0.0, 0.0, m.A_TOUCH + 0.001]), r)}, {"ring": (np.zeros(3), np.zeros(3)), "operator": (np.zeros(3), np.zeros(3))},
                 joints={"turn": -2.04 * TURN, "along": held_at}, time=4.0)  # fmt: skip
    m.thread()
    assert not m.STATE["ring_on"]  # in line but beyond the barrel's threaded part: not on it


def test_out_of_the_panel_the_ring_turns_at_the_tabs_end_without_going_further(m, monkeypatch):
    stops = no_simulator(m, monkeypatch)
    mk = m.mk
    mk.set_state(time=2.0)
    travel = mk.MECH.travel
    travel["turn"] = 7.25 * TURN  # 0.25 mm short of the tab's end: running on
    m.thread()
    assert not m.STATE["tight"] and stops["along"] == pytest.approx([0.00725 - m.PLAY, 0.00725 + m.PLAY])
    travel["turn"] = 7.6 * TURN  # turned on past the tab's end: it stays there and nothing is tightened
    m.thread()
    assert not m.STATE["tight"] and m.ring_weld() == "on" and stops["along"] == pytest.approx([0.0075 - m.PLAY, 0.0075 + m.PLAY])
    assert m.STATE["events"][-1] == (2.0, "ring at the tab's end") and m.STATE["slip"] == pytest.approx(0.1 * TURN)
    travel["turn"] = 7.4 * TURN  # turned back a fifth of a turn: it comes back at once
    m.thread()
    assert stops["along"] == pytest.approx([0.0073 - m.PLAY, 0.0073 + m.PLAY]) and not m.STATE["at_end"]


def test_on_a_seated_operator_the_ring_goes_onto_the_tightener_held_unless_a_hand_is_on_it(m, monkeypatch):
    stops = no_simulator(m, monkeypatch)
    mk = m.mk
    m.STATE["seated"] = True
    mk.set_state({"operator": (m.SEAT_AT.copy(), m.SEATED_ROT)}, joints={"seat": -0.00005}, time=2.0)
    travel = mk.MECH.travel
    travel["turn"] = 7.35 * TURN  # within TOUCH_TOL of the panel's back: off the thread and welded to the tightener there
    m.thread()
    assert m.STATE["tight"] and mk.is_cut("on") and not mk.is_cut("clamp") and m.ring_weld() == "clamp"
    assert m.ring_along() == pytest.approx(m.A_TOUCH - m.CLAMP_CLEAR) and m.ring_gap() <= m.TOUCH_TOL and m.ring_off_axis()[0] == pytest.approx(0.0, abs=1e-12)
    assert stops["seat"] == pytest.approx([-m.PLAY, 0.0])  # the seat's stops closed at the panel: the ring drawn against the panel's back pulls the flange home
    assert m.STATE["events"][-1] == (2.0, "ring against the panel's back") and m.tightened() == 0.0
    m.thread()  # no hand on the ring: the tightener is held where it stands
    assert not mk.is_cut("set")
    ring = mk.pose("ring")[0]
    behind = ring - m.SEATED_ROT @ np.array([0.0, 0.0, 0.0135])  # a grasp point 13.5 mm behind the ring's front face, on its axis
    for hands in ({"left": (behind, 0.42)}, {"left": (behind + np.array([0.0, 0.0, 0.03]), 0.30)}):  # open round it; closed but 3 cm off
        mk.MECH.grippers = hands
        assert not m.hand_on_ring()
        m.thread()
        assert not mk.is_cut("set")
    mk.MECH.grippers = {"left": (behind, 0.30)}  # a hand closed on the ring: the hold is let go and the spring takes the turning
    assert m.hand_on_ring()
    m.thread()
    assert mk.is_cut("set")
    travel["tight"] = math.radians(28.0)  # 1.0 N m at the scene's rate
    m.thread()
    assert m.tightened() == pytest.approx(math.radians(28.0)) and m.TIGHT_READ * m.tightened() == pytest.approx(1.0, abs=0.003)
    assert m.tightened() > m.LEVEL_ANGLE  # 28 degrees: past hand-tight (19.0)
    mk.MECH.grippers = {"left": (behind, 0.42)}  # the hand opened: held again where it stands; the most it was turned kept
    travel["tight"] = math.radians(27.0)
    m.thread()
    assert not mk.is_cut("set") and m.tightened() == pytest.approx(math.radians(28.0))
    travel["tight"] = math.radians(75.0)  # beyond the upper stop (the solver's give): kept as at the stop
    m.thread()
    assert m.tightened() == pytest.approx(m.TIGHT_MAX)


def test_the_operator_is_taken_into_the_hole_only_in_line_with_its_tab_at_the_notch(m, monkeypatch):
    no_simulator(m, monkeypatch)
    mk = m.mk

    def offer(rear_in: float, off: float = 0.0, tilt_deg: float = 0.0, tab_deg: float = 0.0) -> bool:
        rotation = mk.axis_angle(np.array([0.0, -1.0, 0.0]), math.radians(tab_deg)) @ mk.axis_angle(m.X, math.radians(tilt_deg)) @ m.SEATED_ROT
        mk.set_state({"operator": (np.array([m.HOLE[0] + off, m.FRONT_Y + rear_in + m.THREAD_FROM, m.HOLE[2]]), rotation)}, time=3.0)
        m.seat()
        return m.STATE["seated"]

    assert not offer(-0.0090)  # the stem's end 1 mm short of the panel
    assert not offer(-0.0040, off=0.0012)  # 1.2 mm off the hole's line
    assert not offer(-0.0040, tilt_deg=6.0)
    assert not offer(-0.0040, tab_deg=6.0)  # the tab 6 degrees off the notch
    assert not offer(0.0095, tab_deg=20.0)  # the barrel in to the tab's end, the tab 20 degrees off on the panel's face
    assert not offer(0.0125)  # further in than the barrel goes at all: past the gate
    assert offer(0.0095, tab_deg=4.0)  # the search: turned under a light push until the tab finds the notch
    mk.cut("seated")
    m.begin(0)
    m.STATE["carried"] = True
    assert offer(-0.0040, off=0.0003, tilt_deg=2.0, tab_deg=-4.0)
    assert not mk.is_cut("seated") and m.STATE["events"][-1][1].startswith("operator taken into the hole")
    position, rotation = mk.MECH.poses["operator"]
    assert rotation == pytest.approx(m.SEATED_ROT) and position[[0, 2]] == pytest.approx(m.HOLE[[0, 2]])  # put on the hole's line, its tab up
    mk.set_state(joints={"seat": -0.026}, time=4.0)  # pulled back out
    m.seat()
    assert not m.STATE["seated"] and mk.is_cut("seated") and mk.MECH.poses["seat"][0] == pytest.approx(m.SEAT_AT)


def put(m, *, flange: float = 0.0, turns: float | None = None, lock_deg: float | None = None, ring_pose=None, grippers=None, moved=None) -> float:
    """The operator seated with its gasket ``flange`` off the panel; the ring ``turns`` on the barrel (None: aside on the
    bench, or at ``ring_pose``), tightened ``lock_deg`` past its touch (None: not against the panel); the grippers far
    off unless given. Read twice 0.3 s apart, at rest unless ``moved`` (name -> m) moves a part between the two
    readings."""
    mk = m.mk
    operator = (m.SEAT_AT + np.array([0.0, -flange, 0.0]), m.SEATED_ROT)
    poses = {"operator": operator}
    if turns is not None:
        poses["ring"] = (operator[0] + m.SEATED_ROT @ np.array([0.0, 0.0, m.A_END + m.LEAD * turns]), m.SEATED_ROT)
    else:
        poses["ring"] = ring_pose if ring_pose is not None else (np.array([0.40, -0.15, m.TOP + m.RING_H]), np.eye(3))
    far = {"left": (np.array([0.20, 0.30, m.TOP + 0.30]), 1.0), "right": (np.array([0.20, -0.30, m.TOP + 0.30]), 1.0)}
    m.STATE.update(seated=True, ring_on=turns is not None, tight=lock_deg is not None, lock=math.radians(lock_deg or 0.0), track=[])
    for t in (9.7, 10.0):
        if t == 10.0 and moved:
            poses = {name: (pose[0] + np.asarray(moved.get(name, (0.0, 0.0, 0.0))), pose[1]) for name, pose in poses.items()}
        mk.set_state(poses, joints={"seat": -flange}, grippers=grippers or far, time=t)
        m.track()
    return m.score(None)


def test_hand_tight_is_what_fingers_put_on_and_under_what_the_pinch_carries(m):
    """The level: what fingers put on the ring (25 to 50 N at its 14.2 mm radius, an estimate), under the 1.13 N m at
    which the gripper's pinch on the ring slipped in the tighten trial (f1a); read at the scene's 2.05 N m/rad: 0.68 N m,
    19.0 degrees past the touch."""
    fingers = [force * m.RING_OUT for force in (25.0, 50.0)]
    assert fingers == pytest.approx([0.355, 0.71]) and fingers[0] < m.LEVEL_NM <= fingers[1]
    assert m.SLIP_NM == 1.13 and m.LEVEL_NM == 0.68 < m.SLIP_NM
    assert m.TIGHT_READ == 2.05 and m.LEVEL_ANGLE == pytest.approx(m.LEVEL_NM / m.TIGHT_READ)
    assert math.degrees(m.LEVEL_ANGLE) == pytest.approx(19.0, abs=0.01)
    assert math.degrees(m.SLIP_NM / m.TIGHT_READ) == pytest.approx(31.6, abs=0.05)  # where the trial's pinch stopped turning it
    # The clamp on the panel by the torque-tension rule T = K F d (README; an estimate): d = 22 mm, nut factor 0.2 to 0.3
    clamp = [m.LEVEL_NM / (k * 0.022) for k in (0.3, 0.2)]
    assert clamp == pytest.approx([103.0, 154.5], abs=0.5) and min(clamp) > 10 * m.PULL_N


def test_the_score_goes_from_seated_to_hand_tight(m):
    level = math.degrees(m.LEVEL_ANGLE)
    assert put(m) == 0.25  # seated, the ring aside
    assert put(m, flange=0.0012) == 0.0  # the gasket 1.2 mm off the panel: not seated
    assert put(m, turns=0.8) == 0.25 and put(m, turns=1.2) == 0.5
    assert put(m, turns=9.5, lock_deg=level / 3) == 0.75  # against the panel's back, a third of the way to hand-tight
    assert put(m, turns=9.5, lock_deg=level + 3.0) == 1.0  # hand-tight, hands clear
    assert put(m, turns=9.5, lock_deg=level + 3.0, flange=0.0003) == 0.75  # the flange 0.3 mm off
    ring_at = m.SEAT_AT + m.SEATED_ROT @ np.array([0.0, 0.0, m.A_END + 0.0095])
    near = {"left": (ring_at + np.array([0.0, 0.02, 0.0]), 0.6), "right": (np.array([0.20, -0.30, m.TOP + 0.30]), 1.0)}
    assert put(m, turns=9.5, lock_deg=level + 3.0, grippers=near) == 0.75  # a hand 2 cm from the ring
    off_bench = (np.array([0.40, -0.15, m.TOP - 0.10]), np.eye(3))
    assert put(m, ring_pose=off_bench) == 0.0  # the ring off the bench


def test_each_limit_of_the_score_is_met_just_inside_and_missed_just_outside(m):
    """Every limit the score asks, set directly just inside and just outside it (the README's "States set directly")."""
    mk, home = m.mk, 9.5  # the ring's turns on the barrel at the panel's back
    level = math.degrees(m.LEVEL_ANGLE)
    tight = level + 3.0  # tightened past the level
    assert put(m, flange=0.00099) == 0.25 and put(m, flange=0.00101) == 0.0  # the gasket within 1 mm of the panel
    assert put(m, turns=0.99) == 0.25 and put(m, turns=1.01) == 0.5  # the ring a turn on the barrel
    assert put(m, turns=home) == 0.5 and put(m, turns=home, lock_deg=0.0) == 0.75  # against the panel's back
    assert put(m, turns=home, lock_deg=level - 0.05) == 0.75 and put(m, turns=home, lock_deg=level + 0.05) == 1.0
    assert put(m, turns=home, lock_deg=tight, flange=0.00019) == 1.0 and put(m, turns=home, lock_deg=tight, flange=0.00021) == 0.75

    put(m, turns=home, lock_deg=tight)  # both grippers 3 cm or more from button and ring: a hand behind and over the ring
    ring_mid = mk.pose("ring")[0] + np.array([0.0, m.RING_H / 2 + 0.0115, 0.0])  # behind the ring: there the ring is the part nearest (the head's box reaches 26 mm over it)

    def clear_of_ring(d: float) -> np.ndarray:  # over that point, ``d`` from the ring's nearest stave (bisection)
        lo, hi = 0.0, 0.1
        for _ in range(60):
            s = (lo + hi) / 2
            lo, hi = (s, hi) if m.distance_to("ring", ring_mid + np.array([0.0, 0.0, s])) < d else (lo, s)
        point = ring_mid + np.array([0.0, 0.0, (lo + hi) / 2])
        assert min(m.distance_to(name, point) for name in m.PARTS) == pytest.approx(d, abs=1e-6)
        return point

    far_right = (np.array([0.20, -0.30, m.TOP + 0.30]), 1.0)
    for d, score in ((0.0301, 1.0), (0.0299, 0.75)):
        assert put(m, turns=home, lock_deg=tight, grippers={"left": (clear_of_ring(d), 1.0), "right": far_right}) == score, d

    still = put(m, turns=home, lock_deg=tight)  # at rest: every part slower than 20 mm/s over the last 0.25 s
    assert still == 1.0
    assert put(m, turns=home, lock_deg=tight, moved={"ring": (0.019 * 0.3, 0.0, 0.0)}) == 1.0  # 19 mm/s
    assert put(m, turns=home, lock_deg=tight, moved={"ring": (0.021 * 0.3, 0.0, 0.0)}) == 0.75  # 21 mm/s
    # a part 2 cm under the bench top: 0 (the ring aside, so seated is the level)
    assert put(m, ring_pose=(np.array([0.40, -0.15, m.TOP - 0.0199]), np.eye(3))) == 0.25
    assert put(m, ring_pose=(np.array([0.40, -0.15, m.TOP - 0.0201]), np.eye(3))) == 0.0
