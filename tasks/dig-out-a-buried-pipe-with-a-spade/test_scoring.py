import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import digplan as d
from bodykit import BodyState
from scoring import Progress, blade_clear_of_bed, exposed_length, grade, regions
from soilkit import _icosahedron
from yam_arena.ik import matrix_to_quat_xyzw


@pytest.mark.parametrize(
    "length,mass,trips,expected",
    [
        (0, 0, 0, 0),
        (0.019, 0.5, 3, 0),
        (0.020, 0.030, 1, 0.25),
        (0.039, 0.5, 3, 0.25),
        (0.040, 0.070, 2, 0.5),
        (0.079, 0.5, 3, 0.5),
        (0.080, 0.120, 3, 1),
        (0.120, 0.5, 4, 1),
        (0.12, 0, 4, 0),
        (0.12, 0.5, 0, 0),
        (0.12, 0.5, 1, 0.25),
        (0.12, 0.5, 2, 0.5),
    ],
)
def test_thresholds(length, mass, trips, expected):
    assert grade(length, mass, trips, 0) == expected


@pytest.mark.parametrize(
    "reason", ["invalid soil", "bare hand soil removal", "lost soil", "one hand"]
)
def test_failure_overrides_full_progress(reason):
    assert grade(0.12, 1, 4, 0, reason) == 0


def test_spill_boundary():
    assert grade(0.12, 1, 4, 0.030) == 1
    assert grade(0.12, 1, 4, 0.030001) == 0


def test_full_score_requires_blade_withdrawn_from_fixture():
    tool, _ = held_tool(0.180)
    assert not blade_clear_of_bed(tool)
    assert grade(0.12, 1, 4, 0, tool_clear=False) == 0.5
    tool.pos[:2] += d.TARP_C - d.CENTRE
    assert blade_clear_of_bed(tool)
    assert grade(0.12, 1, 4, 0, tool_clear=True) == 1
    assert grade(0.12, 1, 4, 0, blade_load=0.005) == 1
    assert grade(0.12, 1, 4, 0, blade_load=0.005001) == 0.5


def test_exposure_requires_contiguous_crown_strip():
    empty = np.empty((0, 3))
    assert exposed_length(empty, np.empty(0)) == pytest.approx(0.120)
    # Dense material over the full target blocks all rays.
    p = np.array(
        [
            (*d.from_trench(u, v), d.PIPE_TOP + 0.006)
            for u in np.arange(-0.03, 0.115, 0.005)
            for v in (-0.008, 0, 0.008)
        ]
    )
    r = np.full(len(p), 0.005)
    assert exposed_length(p, r) == 0
    # The same soil BELOW the pipe cannot occlude its crown.
    p[:, 2] = d.PIPE_Z - 0.02
    assert exposed_length(p, r) == pytest.approx(0.120)
    # A central obstruction splits the span; scattered clear bins are not added.
    p = np.array([(*d.from_trench(0.040, 0), d.PIPE_TOP + 0.006)])
    assert exposed_length(p, np.array([0.010])) < 0.060


@pytest.mark.parametrize("seed", [7, 19, 31])
def test_oriented_grain_occlusion_matches_triangle_intersections(seed):
    """Independent triangle-ray oracle for the scorer's convex-face calculation."""
    from bodykit import rotation

    rng = np.random.default_rng(seed)
    vertices, faces = _icosahedron()
    positions = np.array(
        [
            (*d.from_trench(u, v), d.PIPE_TOP + z)
            for u, v, z in zip(
                rng.uniform(-0.02, 0.10, 7),
                rng.uniform(-0.015, 0.015, 7),
                rng.uniform(-0.008, 0.010, 7),
                strict=True,
            )
        ]
    )
    radii = rng.uniform(0.004, 0.007, 7)
    quats = rng.normal(size=(7, 4))
    quats /= np.linalg.norm(quats, axis=1, keepdims=True)
    clear = []
    for u in np.arange(-0.02, 0.10 - 0.0025, 0.005):
        points = np.array(
            [
                d.from_trench(u + du, v)
                for du in (0, 0.0025, 0.005)
                for v in np.linspace(-0.008, 0.008, 5)
            ]
        )
        crown = np.tile(
            d.PIPE_Z + np.sqrt(d.PIPE_R**2 - np.linspace(-0.008, 0.008, 5) ** 2), 3
        )
        blocked = np.zeros(len(points), bool)
        for pos, radius, quat in zip(positions, radii, quats, strict=True):
            mesh = vertices @ rotation(quat).T * radius + pos
            for face in faces:
                a, b, c = mesh[np.array(face)]
                matrix = np.column_stack((b[:2] - a[:2], c[:2] - a[:2]))
                if abs(np.linalg.det(matrix)) < 1e-12:
                    continue
                bary = np.linalg.solve(matrix, (points - a[:2]).T).T
                inside = (bary >= 0).all(axis=1) & (bary.sum(axis=1) <= 1)
                height = a[2] + bary @ np.array([b[2] - a[2], c[2] - a[2]])
                blocked |= inside & (height >= crown)
        clear.append(not blocked.any())
    longest = current = 0
    for bit in clear:
        current = current + 1 if bit else 0
        longest = max(longest, current)
    assert exposed_length(positions, radii, quats) == pytest.approx(longest * 0.005)


def test_grain_straddling_tarp_edge_is_not_delivered():
    c, s = np.cos(d.TARP_YAW), np.sin(d.TARP_YAW)
    p = np.array(
        [
            (*d.TARP_C, d.TOP + 0.008),
            (
                *(d.TARP_C + (d.TARP_SIZE[0] / 2 - 0.002) * np.array([c, s])),
                d.TOP + 0.008,
            ),
        ]
    )
    _, on_tarp, _ = regions(p, np.array([0.005, 0.005]))
    assert on_tarp.tolist() == [True, False]


def test_material_below_table_counts_as_spill_even_under_fixture():
    pos = np.array([(*d.CENTRE, d.TOP - 0.20), (*d.TARP_C, d.TOP - 0.20)])
    bed, tray, spill = regions(pos, np.array([0.005, 0.005]))
    assert not bed.any() and not tray.any()
    assert spill.all()


def test_unattributed_soil_on_tarp_earns_no_credit():
    p = np.array([(*d.TARP_C, d.TOP + 0.008)])
    spade = BodyState(
        pos=np.array([0, 0, 1]),
        quat_xyzw=np.array([0, 0, 0, 1]),
        lin_vel=np.zeros(3),
        ang_vel=np.zeros(3),
        low=np.zeros(3),
        high=np.ones(3),
    )
    progress = Progress()
    assert progress.update(p, np.array([0.005]), spade, {}) == 0
    assert progress.metrics["delivered_kg"] == 0
    progress.fail("direct soil contact")
    progress.fail("later failure")
    assert progress.failure == "direct soil contact"


def held_tool(tip_height):
    origin, rotation = d.pose_from_tip(
        (*d.from_trench(0.04, 0), d.TOP + tip_height), d.YAW, 0
    )
    state = BodyState(
        pos=origin,
        quat_xyzw=matrix_to_quat_xyzw(rotation),
        lin_vel=np.zeros(3),
        ang_vel=np.zeros(3),
        low=np.zeros(3),
        high=np.ones(3),
    )
    hands = {
        arm: (origin + rotation @ p, rotation @ r, -0.014)
        for arm, (p, r) in d.grips_local().items()
    }
    return state, hands


def test_hovering_closed_hands_cannot_fake_a_grasp():
    tool, hands = held_tool(d.WALL_H - 0.010)
    pos = np.array([(*d.from_trench(0.12, 0.06), d.TOP + 0.070)])
    progress = Progress()
    for _ in range(6):
        progress.update(
            pos, np.array([0.005]), tool, hands, grip_forces={"left": 0, "right": 0}
        )
    assert progress.failure == "spade used without both grips for over 0.20 s"
    progress.update(
        pos, np.array([0.005]), tool, hands, grip_forces={"left": 50, "right": 50}
    )
    assert progress.failure is not None  # a later correct grip cannot erase the bypass


def test_pipe_contact_is_allowed_but_direct_soil_handling_still_fails():
    tool, hands = held_tool(0.180)
    pos = np.array([(*d.TARP_C, d.TOP + 0.008)])
    progress = Progress()
    progress.update(pos, np.array([0.005]), tool, hands, pipe_contact=True)
    assert progress.failure is None
    assert progress.metrics["pipe_contact"] is True
    progress.update(pos, np.array([0.005]), tool, hands, hand_soil_contact=True)
    assert progress.failure == "robot touched soil directly"


def test_shallow_start_is_covered_and_has_less_material_than_deep_start():
    folder = Path(__file__).resolve().parent
    with (
        np.load(folder / "soil_shallow_start.npz") as shallow,
        np.load(folder / "soil_angular_start.npz") as deep,
    ):
        assert 0 < len(shallow["pos"]) < len(deep["pos"])
        assert exposed_length(shallow["pos"], shallow["radius"], shallow["quat"]) == 0
        assert (shallow["pos"][:, 2] + shallow["radius"] <= d.SOIL_TOP).all()


def test_delivery_requires_loading_then_settling_off_blade():
    tool, hands = held_tool(0.180)
    p = np.array([tool.pos + tool.rotation() @ np.array([0.070, 0, 0.020])])
    radius = np.array([0.005])
    progress = Progress()
    progress.update(p, radius, tool, hands, grip_forces={"left": 50, "right": 50})
    assert progress.carried[0]
    p = np.array([(*d.TARP_C, d.TOP + 0.008)])
    for _ in range(4):
        progress.update(p, radius, tool, hands, grip_forces={"left": 50, "right": 50})
    assert progress.metrics["delivered_kg"] == 0
    for _ in range(3):
        progress.update(p, radius, tool, hands, grip_forces={"left": 50, "right": 50})
    assert progress.metrics["delivered_kg"] > 0


def test_nonfinite_or_disappearing_soil_fails_closed():
    tool, hands = held_tool(0.18)
    progress = Progress()
    p = np.array([(*d.from_trench(0, 0), d.TOP + 0.06)])
    progress.update(p, np.array([0.005]), tool, hands)
    assert progress.update(np.empty((0, 3)), np.empty(0), tool, hands) == 0
    assert progress.failure == "missing or invalid soil state"
    assert progress.metrics["score"] == 0
