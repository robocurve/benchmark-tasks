"""The garlic press task's scene and rules, without the simulator: every state is set directly."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest
from yam_arena import task_file

TASK = Path(__file__).resolve().parent / "task.py"  # the task file in this test's own folder
AWAY = {"left": (np.array([0.30, 0.30, 1.00]), 1.0), "right": (np.array([0.30, -0.30, 1.00]), 1.0)}  # both grippers at their ready places


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


def extent(m, name: str) -> np.ndarray:
    """The size of a body's solid shapes (drawn garlic left out)."""
    lows, highs = zip(*(s.bounds() for s in m.mk.MECH.bodies[name].shapes if s.solid))
    return np.max(highs, axis=0) - np.min(lows, axis=0)


def turn_z(angle: float) -> np.ndarray:
    return np.array([[math.cos(angle), -math.sin(angle), 0.0], [math.sin(angle), math.cos(angle), 0.0], [0.0, 0.0, 1.0]])


def put(m, *, shift: float = 0.0, phi: float = math.radians(120.0), clove=None, knife=None, t: float = 1.0, grippers=AWAY, saucer_drop: float = 0.0,
        contacts: dict | None = None) -> None:
    """The press ``shift`` m along +y from where it starts (towards the saucer), opened ``phi``; ``clove``: (u, v of
    its middle, yaw in the press's frame (0: its length along the press), lift over the floor m) or None (it lies on
    the board where seed 0 puts it); ``knife``: (position, rotation) or None (where it is built)."""
    mk = m.mk
    lower = (m.START_HINGE + np.array([0.0, shift, 0.0]), np.eye(3))
    poses = {"press": lower, "upper": mk.joint_pose(mk.MECH.joints["hinge"], phi, lower)}
    k, side, yaw, x, y = m.STATE["lie"]
    if clove is None:
        position, quat = m.clove_pose(k, side, yaw, x, y)
        poses[m.STATE["clove"]] = (position, mk.quat_to_matrix(quat))
    else:
        u, v, along, lift = clove
        thick = m.CLOVE_SIZES[k][2]
        at = lower[0] + np.array([v, -(u - m.HINGE_UW[0]), m.FLOOR_W - m.HINGE_UW[1] + thick / 2 + lift])
        poses[m.STATE["clove"]] = (at, turn_z(-math.pi / 2 + along))  # the clove's x (root to tip) along the press (-y)
    if knife is not None:
        poses["knife"] = knife
    if saucer_drop:
        poses["saucer"] = (np.array([*m.SAUCER_XY, m.TOP - saucer_drop]), np.eye(3))
    mk.set_state(poses=poses, joints={"hinge": phi}, grippers=grippers, time=t, contacts=contacts)


def test_the_scene_is_the_one_specified(m):
    assert m.INSTRUCTION == "Press the garlic clove through the garlic press onto the saucer, scrape what clings to the press onto the saucer with the knife, then put the press and the knife down."
    assert m.RUN_LIMIT_STEPS == 3000 and m.SCORE_OVER == "end" and m.SUCCESS_HOLD_S == 1.0
    bodies = m.mk.MECH.bodies
    assert extent(m, "board") == pytest.approx((0.150, 0.250, 0.030)) and bodies["board"].mass == pytest.approx(0.73)
    # The press: 155 mm long and 26 mm wide (the maker's page), about 150 g (our figure) in two parts; the chamber 34 x 24 mm inside, 24 deep
    lower, upper = extent(m, "press"), extent(m, "upper")
    assert lower[1] == pytest.approx(0.155) and upper[1] == pytest.approx(0.155) and lower[0] == pytest.approx(0.026) and upper[0] == pytest.approx(0.026)
    assert bodies["press"].mass + bodies["upper"].mass == pytest.approx(0.150)
    assert m.CHAMBER_U[1] - m.CHAMBER_U[0] == pytest.approx(0.034) and 2 * m.CHAMBER_V == pytest.approx(0.024) and m.RIM_W - m.FLOOR_W == pytest.approx(0.024)
    assert len(m.HOLES) == 30 and all(0.006 <= s <= 0.010 for s in m.STRANDS)
    assert math.degrees(m.OPEN_MOST) == pytest.approx(120.0)
    j = m.mk.MECH.joints["hinge"]
    assert (j.lower, j.upper) == (0.0, pytest.approx(m.OPEN_MOST))
    # The plunger: its middle 26 mm from the pin; 250 N there is 6.5 N m; closed its face stands 1 mm over the floor
    assert m.PLUNGER_ARM == pytest.approx(0.026) and m.YIELD_TORQUE == pytest.approx(6.5)
    assert m.face_height(0.0) == pytest.approx(0.001) and m.face_height(math.radians(25.0)) == pytest.approx(0.0130, abs=0.0002)
    # Saucer 150 x 20 mm, knife 210 mm, the cloves within the bench text's 26 to 31 mm and 4 to 6 g
    assert extent(m, "saucer")[:2] == pytest.approx((0.150, 0.150), abs=0.0015) and extent(m, "saucer")[2] == pytest.approx(0.020, abs=0.0005)
    assert extent(m, "knife")[0] == pytest.approx(0.210) and bodies["knife"].mass == pytest.approx(0.070)
    (lo_l, hi_l), (lo_w, hi_w), (lo_t, hi_t), (lo_m, hi_m) = (m.MATERIAL[k][0] for k in ("length_mm", "width_mm", "thickness_mm", "mass_g"))
    for name, (length, width, thick, mass) in zip(m.CLOVES, m.CLOVE_SIZES):
        size = extent(m, name)
        assert lo_l <= 1000 * length <= hi_l and lo_w <= 1000 * width <= hi_w and lo_t <= 1000 * thick <= hi_t and lo_m <= 1000 * mass <= hi_m
        assert size[0] == pytest.approx(length, abs=0.0005) and size[1] <= width + 1e-9 and size[2] == pytest.approx(thick, abs=0.0005)
        assert len([s for s in bodies[name].shapes if s.solid]) == 4 and bodies[name].mass == mass
        # its sample points fill it: the share inside its bounding box near the volume of four such parts
        assert 0.35 < len(m.CLOVE_POINTS[name]) * 0.0012**3 / (length * width * thick) < 0.75
    # The upper part goes through every clove (the solver never presses one); nothing else does
    assert all(set(s.passes) == set(m.CLOVES) for s in bodies["upper"].shapes if s.solid)
    assert not any(s.passes for name in ("press", "board", "saucer", "knife") for s in bodies[name].shapes)


def test_the_press_starts_wholly_on_the_board_and_tips_only_open_at_the_edge(m):
    assert m.START_TIP_Y == pytest.approx(m.BOARD_Y[1] - 0.010)
    put(m)
    assert m.overhang() == pytest.approx(-0.010) and m.sieve_on_board()
    # Where it is pressed the press stands PRESSING_OVERHANG past the board's edge: it tips there when its centre of
    # mass lies farther from its tip than that. Open on its stop it does; with the handle on a clove (30 deg) and
    # closed it stands with more than 15 mm to spare, and 10 g moved between the parts shifts that by less than 3 mm.
    assert m.centre_u(m.OPEN_MOST) < m.PRESSING_OVERHANG - 0.015
    assert m.centre_u(math.radians(30.0)) > m.PRESSING_OVERHANG + 0.015 and m.centre_u(0.0) > m.PRESSING_OVERHANG + 0.020
    assert abs(m.centre_u(math.radians(30.0), 0.070, 0.080) - m.centre_u(math.radians(30.0))) < 0.003
    put(m, shift=m.PRESSING_OVERHANG + 0.010, phi=0.0)
    assert m.overhang() == pytest.approx(m.PRESSING_OVERHANG) and not m.sieve_on_board()


def test_by_seed_one_clove_lies_on_the_board_on_its_flat_face_or_its_side(m):
    assert m.lie(0) == (0, False, pytest.approx(math.radians(25.0)), *m.CLOVE_XY) and m.lie(None) == m.lie(0)
    seen = {m.lie(seed)[:2] for seed in range(1, 40)}
    assert {k for k, _ in seen} == set(range(len(m.CLOVES))) and {side for _, side in seen} == {False, True}
    for seed in range(1, 40):
        k, side, yaw, x, y = m.lie(seed)
        assert abs(x - m.CLOVE_XY[0]) <= 0.01 and abs(y - m.CLOVE_XY[1]) <= 0.01
        position, _ = m.clove_pose(k, side, yaw, x, y)
        length, width, thick, _ = m.CLOVE_SIZES[k]
        assert position[2] - (m.TOP + m.BOARD_T) == pytest.approx((width if side else thick) / 2 + 0.0005)
        assert m.BOARD_X[0] + length / 2 < x < m.BOARD_X[1] - length / 2 and m.BOARD_Y[0] + length / 2 < y < m.BOARD_Y[1] - length / 2


def test_the_clove_resists_the_plunger_up_to_its_yield_and_goes_through_in_proportion(m):
    # Crushed in proportion to the load, 2.5 mm at the yield; with the spring's own 0.5 mm the plunger has then gone 3 mm
    assert m.crushing(0.0) == 0.0 and m.crushing(-1.0) == 0.0 and m.crushing(3.25) == pytest.approx(0.00125) and m.crushing(6.5) == pytest.approx(0.0025)
    assert m.crushing(13.0) == pytest.approx(0.0025)  # no more than that, whatever the load
    assert m.CRUSHED + m.YIELD_TORQUE / m.CLOVE_K * m.PLUNGER_ARM == pytest.approx(m.COMPRESSION) and m.YIELD_TORQUE == pytest.approx(6.5)
    # Through the sieve: none before the yield (3 mm under the clove's top), 85 % with the face 3 mm over the floor
    assert m.through_share(0.013, 0.0101) == 0.0 and m.through_share(0.013, 0.003) == pytest.approx(0.85) and m.through_share(0.013, 0.001) == pytest.approx(0.85)
    assert m.through_share(0.013, 0.0065) == pytest.approx(0.425)
    assert m.CLOVE_K * m.GIVE_ANGLE == pytest.approx(2.0 * m.YIELD_TORQUE)  # SPRINGBACK behind its rest the spring would give twice the yield: the cap sets it


def in_upper(m, points) -> np.ndarray:
    """World points as (u, v, w) of the press's upper part in its closed coordinates: the plunger's face is the
    rectangle PLUNGER_U x +-PLUNGER_V at w = CLOSED_FACE_W (read from the part's pose, not from the task's geometry)."""
    local = m.to_body("upper", points)
    return np.column_stack([m.HINGE_UW[0] - local[:, 1], local[:, 0], m.HINGE_UW[1] + local[:, 2]])


def test_the_plunger_arrives_on_the_clove_inside_the_chamber_and_consumes_it(m):
    # The clove lengthways in the chamber, flat face down: the handle closed half a degree a step (as at 20 Hz): the
    # plunger arrives at the step its face has come down to the clove, and the touch is put at the opening where the
    # face first met it. The face is tilted then, so it meets the clove near the hinge first: its middle stands above
    # the clove's 13 mm.
    deg = 40.0
    clove = (0.030, 0.0, 0.0, 0.0)
    while not m.STATE["touched"] and deg > 0.0:
        put(m, phi=math.radians(deg), clove=clove)
        m.press_on()
        deg -= 0.5
    reached = deg + 0.5
    assert m.STATE["touched"] and 20.0 < reached < 45.0  # 34.5 deg for this clove: the tilted face meets its rounded back near the root end
    assert reached <= m.STATE["touch_deg"] <= reached + 0.505
    assert m.STATE["inside"] == pytest.approx(m.STATE["mass"]) and m.STATE["waste"] == pytest.approx(0.0, abs=1e-9)
    touch = m.plunger_into_clove(0.0)[0]
    assert m.STATE["top"] == pytest.approx(m.face_height(touch)) and m.STATE["top"] > 0.013 and m.STATE["lowest"] == m.STATE["top"]
    assert m.STATE["contact"] and m.STATE["cap"] == m.YIELD_TORQUE and m.STATE["rest"] == pytest.approx(touch) and m.STATE["crush"] == 0.0  # just touched: nothing crushed
    # The touch angle is exact: the handle there, the face's plane passes through the clove's highest covered point
    assert m.plunger_into_clove(touch + 1e-4) is None and m.plunger_into_clove(touch - 1e-4) is not None
    put(m, phi=touch, clove=clove)
    upper = in_upper(m, m.to_world(m.STATE["clove"], m.CLOVE_POINTS[m.STATE["clove"]]))
    covered = (upper[:, 0] >= m.PLUNGER_U[0]) & (upper[:, 0] <= m.PLUNGER_U[1]) & (np.abs(upper[:, 1]) <= m.PLUNGER_V)
    assert covered.any() and (upper[covered, 2] - m.CLOSED_FACE_W).max() == pytest.approx(0.0, abs=1e-6)
    # Pressed down past the yield on the board: the sieve rests on it and all that comes through goes onto the board
    for deg in (20.0, 15.0, 10.0, 5.0, 2.0, 0.0):
        put(m, phi=math.radians(deg), t=2.0)
        m.press_on()
    assert m.STATE["through"] == pytest.approx(0.85 * m.STATE["mass"]) and sum(m.STATE["board"]) == pytest.approx(m.STATE["through"]) and sum(m.STATE["hanging"]) == 0.0
    assert m.STATE["cap"] == pytest.approx(m.YIELD_TORQUE) and m.level() == 0.5  # 85 % through, nothing on the saucer
    m.draw()
    assert all(not m.mk.shown("press", k) for k in m.STRAND_SHAPES) and m.mk.shown("press", m.MASH_SHAPES[0]) and not m.mk.shown("press", m.MASH_SHAPES[-1])


def test_the_clove_crushes_under_a_load_yields_and_never_holds_a_lifted_handle(m):
    clove = (0.030, 0.0, 0.0, 0.0)
    put(m, phi=math.radians(40.0), clove=clove)
    m.press_on()
    put(m, phi=math.radians(30.0), clove=clove)
    m.press_on()
    touch = m.STATE["touch"]
    assert m.STATE["touched"] and m.STATE["contact"] and m.STATE["lowest"] == pytest.approx(m.face_height(touch))  # a handle that came down past the touch is taken as on it
    # Half the yield on the plunger: the spring gives, the clove is crushed half of CRUSHED, the rest sinks with it
    load = 0.5 * m.YIELD_TORQUE
    put(m, phi=touch - load / m.CLOVE_K, clove=clove)
    m.press_on()
    assert m.STATE["crush"] == pytest.approx(0.5 * m.CRUSHED) and m.STATE["rest"] == pytest.approx(touch - 0.5 * m.CRUSHED / m.PLUNGER_ARM)
    assert m.STATE["cap"] == m.YIELD_TORQUE and m.STATE["through"] == 0.0
    # Under the same load at the new rest: no further (it stands under a load below its yield)
    put(m, phi=m.STATE["rest"] - load / m.CLOVE_K, clove=clove)
    m.press_on()
    assert m.STATE["crush"] == pytest.approx(0.5 * m.CRUSHED)
    # Lifted off: the drive gives nothing. Let down past where it was crushed to: stopped there, crushed no further
    rest = m.STATE["rest"]
    put(m, phi=rest + math.radians(5.0), clove=clove)
    m.press_on()
    assert not m.STATE["contact"] and m.STATE["cap"] == 0.0 and m.STATE["rest"] == pytest.approx(rest)
    put(m, phi=rest - math.radians(3.0), clove=clove)
    m.press_on()
    assert m.STATE["contact"] and m.STATE["cap"] == m.YIELD_TORQUE and m.STATE["rest"] == pytest.approx(rest) and m.STATE["crush"] == pytest.approx(0.5 * m.CRUSHED)
    # Past the yield (the drive at its cap, the handle running down): the rest follows SPRINGBACK behind
    phi = rest - 4 * m.GIVE_ANGLE
    put(m, phi=phi, clove=clove)
    m.press_on()
    assert m.STATE["crush"] == pytest.approx(m.CRUSHED) and m.STATE["rest"] == pytest.approx(phi + m.GIVE_ANGLE) and m.STATE["through"] > 0.0


def test_over_the_saucer_30_percent_drops_onto_it_and_70_percent_hangs(m):
    shift = m.PRESSING_OVERHANG + 0.010
    put(m, shift=shift, phi=math.radians(25.4), clove=(0.030, 0.0, 0.0, 0.0))
    m.press_on()
    for deg in (20.0, 10.0, 4.0, 0.0):
        put(m, shift=shift, phi=math.radians(deg))
        m.press_on()
    through = 0.85 * m.STATE["mass"]
    assert m.STATE["through"] == pytest.approx(through)
    assert sum(m.STATE["saucer"]) == pytest.approx(0.30 * through) and sum(m.STATE["hanging"]) == pytest.approx(0.70 * through)
    assert m.on_saucer() == pytest.approx(0.255) and m.level() == 0.5  # through, but under 45 % on the saucer
    m.draw()
    assert all(m.mk.shown("press", k) for k in m.STRAND_SHAPES) and sum(m.mk.shown("saucer", k) for k in m.SAUCER_SHAPES) >= 3


def test_a_clove_across_the_chamber_is_pinched_off_at_the_rim(m):
    put(m, phi=math.radians(80.0), clove=(0.030, 0.0, math.pi / 2, m.RIM_W - m.FLOOR_W))  # across, lying on the rim
    m.press_on()
    assert not m.STATE["touched"]
    for deg in range(78, 40, -2):
        put(m, phi=math.radians(deg), clove=(0.030, 0.0, math.pi / 2, m.RIM_W - m.FLOOR_W))
        m.press_on()
        if m.STATE["touched"]:
            break
    assert m.STATE["touched"]
    length = m.CLOVE_SIZES[0][0]
    assert 0.0 < m.STATE["waste"] < 0.5 * m.STATE["mass"] and m.STATE["inside"] / m.STATE["mass"] == pytest.approx(0.024 / length, abs=0.12)
    assert m.STATE["rim_spots"] and set(m.STATE["rim_spots"]) <= set(range(6)) and len(m.STATE["rim_spots"]) == 2  # pinched off at both side walls
    assert m.STATE["touch_deg"] > 55.0  # the face meets a clove on the rim high up
    # More than a third pinched off scores 0, whatever else
    m.STATE["waste"] = 0.34 * m.STATE["mass"]
    m.STATE["saucer"][0] = m.STATE["mass"]
    assert m.level() == 0.0


def test_the_knife_takes_a_strand_only_scraping_the_sieve_and_leaves_it_only_wiped_along_the_saucer(m):
    shift = m.PRESSING_OVERHANG + 0.010
    put(m, shift=shift, phi=0.0, clove=(0.030, 0.0, 0.0, 0.0))
    m.STATE.update(touched=True, top=0.013, inside=m.STATE["mass"], lowest=0.001, through=0.85 * m.STATE["mass"])
    sieve = m.press_point(m.PLUNGER_MID[0], 0.0, 0.0)
    knife_rot = turn_z(-math.pi / 2)  # the blade pointing -y, flat, its top face up; its width along +x
    on_edge = knife_rot @ m.mk.axis_angle((1.0, 0.0, 0.0), math.pi / 2)  # the same, turned onto its edge: its +y edge up, its underside facing +x
    middle = (m.BLADE_X[0] + m.BLADE_X[1]) / 2
    half = m.KNIFE_BLADE[1] / 2
    every = 0.0001 * len(m.HOLES)

    def stroke(gap: float, newtons: float, moves=(-0.030, 0.030), turn=knife_rot, on=None) -> float:
        """The knife turned ``turn`` with its point ``on`` (knife frame; default the top face's middle) under the
        sieve's middle, ``gap`` under the sieve, moved across (x) through ``moves`` (each a (dx, dz) or a dx), the
        contact sensor reading ``newtons`` between knife and press; what the knife then carries."""
        on = np.array([middle, 0.0, m.BLADE_Z[1]]) if on is None else on
        m.STATE["hanging"] = [0.0001] * len(m.HOLES)
        m.STATE.update(edges=None, knife=0.0, loaded=[], carried=0.0)
        for move in moves:
            dx, dz = move if isinstance(move, tuple) else (move, 0.0)
            at = sieve - turn @ on + np.array([dx, 0.0, dz - gap])
            put(m, shift=shift, phi=0.0, knife=(at, turn), t=2.0, contacts={("knife", "press"): newtons})
            m.scrape()
        return m.STATE["knife"]

    # Flat: the top face on the sieve, a long edge passing under the holes
    assert stroke(0.0, 0.0) == 0.0  # on the sieve and slid across, but not pressing: nothing
    assert stroke(0.0, 0.45) == 0.0  # pressing with less than SCRAPE_PRESS: nothing
    assert stroke(0.0015, 1.0) == 0.0  # pressing (on the press somewhere), but 1.5 mm under the holes: near, not touching
    assert stroke(0.0, 1.0, moves=(-0.0015,) * 3) == 0.0  # touching and pressing with a row of holes on the edge, not sliding
    assert stroke(0.0, 1.0, moves=[(-half + 0.0085, -0.002 + 0.000525 * k) for k in range(5)]) == 0.0  # pressed up onto a row of holes at its edge (0.1 mm in at the end, as the solver lets it): nothing
    assert stroke(0.0003, 0.6) == pytest.approx(every)  # pressed on it (0.3 mm: touching) and slid across: every strand
    assert {(face, edge) for face, edge, _ in m.STATE["loaded"]} == {(1, 1)} and m.STATE["carried"] == pytest.approx(every)  # top face, leading edge
    xs = sorted(x for _, _, x in m.STATE["loaded"])
    assert xs[0] == pytest.approx(middle - 0.0145) and xs[-1] == pytest.approx(middle + 0.0145)  # where the holes passed along the blade

    # On its edge: that edge's face on the sieve, run across from the chamber's one side to the other, its corners
    # passing the holes; the underside's corner leads (the knife moves +x, its underside facing that way)
    edge_mid = np.array([middle, half, (m.BLADE_Z[0] + m.BLADE_Z[1]) / 2])
    across = [-0.0115 + 0.0004 * k for k in range(58)]  # 0.4 mm a step, as the script's scrape goes
    assert stroke(0.0, 1.0, moves=across, turn=on_edge, on=edge_mid) == pytest.approx(every)
    assert {(face, edge) for face, edge, _ in m.STATE["loaded"]} == {(-1, 1)}  # the underside's corner on the +y edge
    assert sorted(x for _, _, x in m.STATE["loaded"])[0] == pytest.approx(middle - 0.0145)
    m.draw()  # the garlic is drawn where it rides: on the underside along the +y edge, over the band, and nowhere else
    drawn = {corner: [k for k, shape in enumerate(shapes) if m.mk.shown("knife", shape)] for corner, shapes in m.LOAD_SHAPES.items()}
    assert drawn[(-1, 1)] and not any(drawn[c] for c in m.LOAD_SHAPES if c != (-1, 1))
    assert all(middle - 0.0145 - m.BAND - 0.0036 <= m.KNIFE_SPOTS[k] <= middle + 0.0145 + m.BAND + 0.0036 for k in drawn[(-1, 1)])
    assert stroke(0.0, 0.3, moves=across, turn=on_edge, on=edge_mid) == 0.0  # not pressing
    assert stroke(0.0015, 1.0, moves=across, turn=on_edge, on=edge_mid) == 0.0  # 1.5 mm under the holes
    lead = -0.0085 + (m.BLADE_Z[0] - edge_mid[2])  # the knife placed with its leading corner (1 mm ahead of the edge's middle, +x) under the first row of holes
    assert stroke(0.0, 1.0, moves=[(lead, -0.002 + 0.000525 * k) for k in range(5)], turn=on_edge, on=edge_mid) == 0.0  # pressed up onto them
    assert stroke(0.0, 1.0, moves=[lead + 0.00002 * (-1) ** k for k in range(8)], turn=on_edge, on=edge_mid) == 0.0  # trembling across a corner

    # Wiping on the well: the loaded knife along x over the saucer's well, turned 60 degrees about its length so that
    # the loaded edge goes down; its underside's corner on that edge on the well, the garlic 1.0 mm over it
    roll = m.mk.axis_angle((1.0, 0.0, 0.0), math.radians(-60.0))
    corner = roll @ np.array([middle, half, m.BLADE_Z[0]])
    load = 0.003

    def wipe(moves, newtons: float, turn=roll, low=corner, lift: float = 0.0, loaded=None) -> float:
        """The knife turned ``turn`` with the point ``low`` (knife frame, turned) on the well, moved along x through
        ``moves``, the sensor reading ``newtons`` between knife and saucer; what is then on the saucer."""
        loaded = [(1, 1, x) for x in xs] if loaded is None else loaded
        m.STATE.update(knife=load, carried=load, loaded=loaded, wipe_at=None, wiped=0.0, saucer=[0.0] * len(m.SAUCER_SPOTS))
        for dx in moves:
            at = np.array([m.SAUCER_XY[0] - 0.015 + dx, m.SAUCER_XY[1], m.TOP + m.WELL_W + lift]) - low
            put(m, shift=shift, phi=0.0, knife=(at, turn), t=3.0, contacts={("knife", "saucer"): newtons})
            m.wipe()
        return sum(m.STATE["saucer"])

    assert wipe((0.0, 0.0, 0.0), 1.0) == 0.0 and m.STATE["knife"] == load  # touching and pressing, not drawn: nothing
    assert wipe((0.0, 0.030), 0.0) == 0.0  # drawn along the well, touching, not pressing: nothing
    assert wipe((0.0, 0.030), 1.0, lift=0.003) == 0.0  # drawn and pressing, 3 mm over the well: near, not touching
    flat = np.array([middle, 0.0, m.BLADE_Z[0]])  # flat, top face up: its underside on the well, the garlic 2 mm over it
    assert wipe((0.0, 0.030), 1.0, turn=np.eye(3), low=flat) == 0.0
    assert wipe((0.0, 0.015), 1.0) == pytest.approx(load / 2) and m.STATE["knife"] == pytest.approx(load / 2)  # half after 15 mm
    m.STATE["wipe_at"] = None
    at = np.array([m.SAUCER_XY[0], m.SAUCER_XY[1], m.TOP + m.WELL_W]) - corner
    put(m, shift=shift, phi=0.0, knife=(at, roll), t=3.0, contacts={("knife", "saucer"): 1.0})
    m.wipe()
    put(m, shift=shift, phi=0.0, knife=(at + np.array([0.015, 0.0, 0.0]), roll), t=3.0, contacts={("knife", "saucer"): 1.0})
    m.wipe()
    assert m.STATE["knife"] == 0.0 and sum(m.STATE["saucer"]) == pytest.approx(load)  # all of it after 30 mm, on the saucer
    assert "wiped off the knife onto the saucer" in m.STATE["events"][-1][1]

    # The loaded part is a band along the corner that took the garlic: BAND either side of where, joined up where
    # strands were taken closer than twice that, a point every millimetre
    m.STATE["loaded"] = [(-1, 1, 0.100), (-1, 1, 0.1058), (-1, 1, 0.130)]
    points = m.loaded_points()
    assert set(map(tuple, points[:, 1:].round(6))) == {(round(half, 6), round(m.BLADE_Z[0], 6))}
    assert points[:, 0].min() == pytest.approx(0.100 - m.BAND) and points[:, 0].max() == pytest.approx(0.130 + m.BAND)
    assert not any(0.1058 + m.BAND + 0.0005 < x < 0.130 - m.BAND - 0.0005 for x in points[:, 0])  # no garlic between 109 and 127 mm

    # Wiping on the rim: the knife pointing +x, standing on its loaded edge (turned -90 degrees about its length),
    # that edge across the near rim's top edge and drawn back (-x) over it, so that the rim passes under the whole
    # band (the scraped rows, 35 mm with BAND): all of it once the rim has gone 30 mm along the band
    down = m.mk.axis_angle((1.0, 0.0, 0.0), -math.pi / 2)
    band = [(-1, 1, x) for x in xs]
    lo, hi = min(xs) - m.BAND, max(xs) + m.BAND

    def over_rim(newtons: float, lift: float = 0.0, turn=down, edge: int = 1) -> float:
        m.STATE.update(knife=load, carried=load, loaded=band, wipe_at=None, wiped=0.0, saucer=[0.0] * len(m.SAUCER_SPOTS))
        for x_on in np.arange(lo - 0.005, hi + 0.005, 0.001):  # where along the blade the rim's top edge is
            low = turn @ np.array([x_on, edge * half, m.BLADE_Z[0]])
            at = np.array([m.SAUCER_XY[0] - m.RIM_OUTER, m.SAUCER_XY[1], m.TOP + m.SAUCER_H + lift]) - low
            put(m, shift=shift, phi=0.0, knife=(at, turn), t=4.0, contacts={("knife", "saucer"): newtons})
            m.wipe()
        return sum(m.STATE["saucer"])

    assert over_rim(1.0) == pytest.approx(load) and m.STATE["knife"] == 0.0  # drawn over the rim, pressing: all of it
    m.draw()
    assert not any(m.mk.shown("knife", shape) for shapes in m.LOAD_SHAPES.values() for shape in shapes)  # none left drawn on the knife
    assert "wiped off the knife onto the saucer" in m.STATE["events"][-1][1]
    assert over_rim(0.2) == 0.0  # not pressing
    assert over_rim(1.0, lift=0.003) == 0.0  # 3 mm over the rim
    assert over_rim(1.0, turn=m.mk.axis_angle((1.0, 0.0, 0.0), math.pi / 2), edge=-1) == 0.0  # the other edge, not loaded, on the rim


def test_the_score_levels_and_the_release(m):
    mass = m.STATE["mass"]
    put(m, phi=0.0)  # the press closed on the board, the knife where it is built, both lying, hands at their ready places
    m.STATE.update(touched=True, top=0.013, inside=mass, lowest=0.001)

    def at(saucer: float, through: float = 0.85, speed: float = 0.0) -> float:
        m.STATE["saucer"] = [0.0] * len(m.SAUCER_SPOTS)
        m.STATE["saucer"][0] = saucer * mass
        m.STATE["through"] = through * mass
        m.STATE["track"] = []
        for t, dx in ((1.0, 0.0), (1.0 + m.SETTLE_S, speed * m.SETTLE_S)):
            m.STATE["track"].append((t, {name: m.mk.pose(name)[0] + np.array([dx, 0.0, 0.0]) for name in m.PARTS}))
        return m.score(None)

    assert m.lying("press") and m.lying("knife") and m.let_go()
    assert at(0.65) == 1.0 and at(0.649) == 0.75 and at(0.45) == 0.75 and at(0.449) == 0.5 and at(0.44, through=0.699) == 0.25
    assert at(0.65, speed=0.019) == 1.0 and at(0.65, speed=0.021) == 0.75  # every part at rest
    m.STATE["touched"] = False
    assert at(0.0, through=0.0) == 0.0
    m.STATE["touched"] = True
    # A gripper 2.9 cm from the knife keeps it at 0.75; 3.1 cm lets it be 1.0
    knife = m.mk.pose("knife")[0]
    for gap, expected in ((0.029, 0.75), (0.031, 1.0)):
        handle_top = knife + np.array([0.0, 0.0, m.KNIFE_HANDLE[2] / 2 + gap])
        put(m, phi=0.0, grippers={"left": (handle_top, 1.0), "right": AWAY["right"]})
        m.STATE.update(touched=True, top=0.013, inside=mass, lowest=0.001)
        assert at(0.70) == expected
    # The press held 1 cm over the board does not lie on it: 0.75
    lower = (m.START_HINGE + np.array([0.0, 0.0, 0.010]), np.eye(3))
    m.mk.set_state(poses={"press": lower, "upper": m.mk.joint_pose(m.mk.MECH.joints["hinge"], 0.0, lower)}, joints={"hinge": 0.0}, grippers=AWAY, time=1.0)
    assert not m.lying("press") and at(0.70) == 0.75
    # Set down where it is pressed, its chamber past the board's edge over the saucer: it lies on the board. Tipped
    # over the edge (its tip 10 degrees down, below the board's top): it does not
    shift = m.PRESSING_OVERHANG - (m.START_TIP_Y - m.BOARD_Y[1])
    put(m, shift=shift, phi=0.0)
    assert m.overhang() == pytest.approx(m.PRESSING_OVERHANG) and m.lying("press")
    edge = np.array([m.PRESS_X, m.BOARD_Y[1], m.TOP + m.BOARD_T])
    tilt = m.mk.axis_angle((1.0, 0.0, 0.0), math.radians(-10.0))  # about the board's edge: the tip (+y) goes down
    lower = (edge + tilt @ (m.START_HINGE + np.array([0.0, shift, 0.0]) - edge), tilt)
    m.mk.set_state(poses={"press": lower, "upper": m.mk.joint_pose(m.mk.MECH.joints["hinge"], 0.0, lower)}, joints={"hinge": 0.0}, grippers=AWAY, time=1.0)
    assert m.press_point(0.0, 0.0, 0.0)[2] < m.TOP + m.BOARD_T - 0.005 and not m.lying("press")
    # The saucer 2.1 cm under the bench top has left it: 0
    put(m, phi=0.0, saucer_drop=0.021)
    assert at(0.70) == 0.0
    put(m, phi=0.0, saucer_drop=0.019)
    assert at(0.70) == 1.0
