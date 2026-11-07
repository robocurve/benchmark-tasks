"""Checks of kit_robot_joint.py's scoring and demo, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_kit_robot_joint.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("kit_robot_joint.py"))
M = TASK.module


def part_state(part, x, y, bottom_z, yaw_deg=0.0, tilt_deg=0.0, moving=False):
    """A part with its centre at x, y, turned about the vertical then tilted about world y, its lowest corner at
    bottom_z; the bounding box is built as Arena 0.3.1 builds it (turned local corners)."""
    (fx, fy), _, _ = M.PARTS[part]
    s, t = math.radians(yaw_deg), math.radians(tilt_deg)
    rz = np.array([[math.cos(s), -math.sin(s), 0], [math.sin(s), math.cos(s), 0], [0, 0, 1]])
    ry = np.array([[math.cos(t), 0, math.sin(t)], [0, 1, 0], [-math.sin(t), 0, math.cos(t)]])
    rot = ry @ rz
    corners = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]) * np.array(
        [fx / 2, fy / 2, M.PART_H / 2]) @ rot.T
    centre = np.array([x, y, bottom_z - corners[:, 2].min()])
    q = np.array([math.sin(t / 2) * math.sin(s / 2), math.sin(t / 2) * math.cos(s / 2),
                  math.cos(t / 2) * math.sin(s / 2), math.cos(t / 2) * math.cos(s / 2)])
    world = corners + centre
    return ObjectState(pos=centre, quat_xyzw=q, lin_vel=np.array([0.05, 0, 0]) if moving else np.zeros(3),
                       ang_vel=np.zeros(3), low=world.min(axis=0), high=world.max(axis=0))


def at_start(part):
    _, (x, y), _ = M.PARTS[part]
    return part_state(part, x, y, M.TOP)


def in_own_pocket(part, dx=0.0, dy=0.0, **kw):
    (px, py), turned = M.POCKETS[part]
    kw.setdefault("yaw_deg", 90.0 if turned else 0.0)
    return part_state(part, px + dx, py + dy, M.FLOOR_Z, **kw)


def scene(**overrides):
    return {part: overrides.get(part, at_start(part)) for part in M.PARTS}


def all_kitted(**overrides):
    return scene(**{part: overrides.get(part, in_own_pocket(part)) for part in M.PARTS})


def test_each_part_fits_only_where_the_task_says():
    """Footprints vs pocket insides: the motor only fits its own (turned) pocket, the bracket only its own, and the
    bearing block fits all three."""
    fits = {}
    for part, ((fx, fy), _, _) in M.PARTS.items():
        fits[part] = [p for p in M.PARTS if
                      (fx <= M.pocket_inner(p)[0] and fy <= M.pocket_inner(p)[1])
                      or (fy <= M.pocket_inner(p)[0] and fx <= M.pocket_inner(p)[1])]
    assert fits == {"servo_motor": ["servo_motor"], "bracket_block": ["bracket_block"],
                    "bearing_block": ["servo_motor", "bracket_block", "bearing_block"]}


def test_the_start_scores_zero():
    assert M.score(scene()) == 0.0 and not M.success(scene())


def test_all_three_kitted_is_success():
    assert M.success(all_kitted()) and M.score(all_kitted()) == 1.0


@pytest.mark.parametrize("part", list(M.PARTS))
def test_each_part_in_its_pocket_scores_a_third(part):
    assert M.score(scene(**{part: in_own_pocket(part)})) == pytest.approx(1 / 3)


def test_the_motor_lying_along_its_pocket_counts_and_across_it_does_not():
    assert M.score(scene(servo_motor=in_own_pocket("servo_motor", yaw_deg=90))) == pytest.approx(1 / 3)
    assert M.score(scene(servo_motor=in_own_pocket("servo_motor", yaw_deg=0))) == 0.0  # sticks out over the walls


def test_the_bearing_block_in_the_motors_pocket_earns_nothing():
    (px, py), _ = M.POCKETS["servo_motor"]
    wrong = part_state("bearing_block", px, py, M.FLOOR_Z)
    assert M.score(scene(bearing_block=wrong)) == 0.0


@pytest.mark.parametrize(("dx_mm", "inside"), [(3.5, True), (4.0, True), (4.3, True), (4.6, False)])
def test_corners_must_stay_inside_the_pocket(dx_mm, inside):
    """4 mm to spare on each side; touching the wall (4.0) or pressed 0.3 mm into it still counts."""
    objects = scene(bracket_block=in_own_pocket("bracket_block", dx=dx_mm / 1000))
    assert M.score(objects) == pytest.approx(1 / 3 if inside else 0.0)


def test_a_tilted_part_resting_on_a_rim_is_not_in():
    objects = scene(bracket_block=in_own_pocket("bracket_block", tilt_deg=15))
    assert M.score(objects) == 0.0


def test_upside_down_parts_count():
    """A plain block looks the same either way up, from above and in the photo check."""
    assert M.success(all_kitted(**{part: in_own_pocket(part, tilt_deg=180) for part in M.PARTS}))


def test_the_bracket_counts_on_any_face():
    """It is a 4 cm cube."""
    assert M.score(scene(bracket_block=in_own_pocket("bracket_block", tilt_deg=90))) == pytest.approx(1 / 3)


def posed(part, quat_xyzw, x, y, bottom_z):
    """A part in any orientation, its centre at x, y and its lowest corner at bottom_z."""
    (fx, fy), _, _ = M.PARTS[part]
    r = np.array(M._rot(quat_xyzw))
    corners = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]) * np.array(
        [fx / 2, fy / 2, M.PART_H / 2]) @ r.T
    centre = np.array([x, y, bottom_z - corners[:, 2].min()])
    world = corners + centre
    return ObjectState(pos=centre, quat_xyzw=np.array(quat_xyzw), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=world.min(axis=0), high=world.max(axis=0))


def test_the_motor_standing_on_its_end_does_not_count():
    """On its end the motor's footprint (2.5 x 4 cm) fits its pocket, but it is the wrong way round for the kit."""
    (px, py), _ = M.POCKETS["servo_motor"]
    on_end = posed("servo_motor", (-0.5, -0.5, -0.5, 0.5), px, py, M.FLOOR_Z)  # 60 mm axis up, 40 mm along y
    bottom = sorted(M._corners("servo_motor", on_end), key=lambda c: c[2])[:4]
    ix, iy = M.pocket_inner("servo_motor")
    assert all(abs(c[0] - px) <= ix / 2 and abs(c[1] - py) <= iy / 2 for c in bottom)  # it does fit
    assert abs(bottom[0][2] - M.FLOOR_Z) < 1e-9
    assert M.score(scene(servo_motor=on_end)) == 0.0


def test_a_part_held_above_its_pocket_is_not_in():
    (px, py), _ = M.POCKETS["bracket_block"]
    held = part_state("bracket_block", px, py, M.FLOOR_Z + 0.01)
    assert M.score(scene(bracket_block=held)) == 0.0


def test_still_moving_is_not_yet_success():
    objects = all_kitted(bearing_block=in_own_pocket("bearing_block", moving=True))
    assert M.score(objects) == 1.0 and not M.success(objects)


def test_the_demo_places_each_part_over_its_own_pocket():
    """At its pocket's centre plus the measured aim offset, which stays well under a pocket's width."""
    places = {w.label.split(":")[0]: w.pos[:2] for w in M.demo()["left"] if w.label.endswith("let go")}
    expected = {part: (M.POCKETS[part][0][0] + M.PLACE_AIM[part][0], M.POCKETS[part][0][1] + M.PLACE_AIM[part][1])
                for part in M.PARTS}
    assert places == {part: pytest.approx(xy) for part, xy in expected.items()}
    assert all(math.hypot(*M.PLACE_AIM[part][:2]) < 0.016 for part in M.PARTS)


def test_parts_are_let_go_inside_their_pocket_walls():
    """Released above the floor but below the wall tops, so each part drops straight in."""
    assert 0 < M.RELEASE_ABOVE < M.WALL_H


def test_the_demo_is_within_reach():
    plan = ArmPlan(M.demo()["left"], station.BASES["left"])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)
