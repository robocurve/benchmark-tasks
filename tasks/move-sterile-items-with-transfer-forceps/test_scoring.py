"""Checks of the transfer-forceps task's shapes, contamination record and scoring rule. No simulator needed:

    python -m pytest -q test_scoring.py
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from yam_arena import task_file  # noqa: E402

T = task_file.load(HERE / "task.py").module
K = T.K
I = (0.0, 0.0, 0.0, 1.0)
FAR = np.array([[0.0, 0.0, T.TOP + 0.40]])                # an arm point nowhere near anything
AT_REST = {"forceps": 0.0, **{n: 0.0 for n in T.ITEMS}}
LEVEL = K.turn_y(K.syringe_rest()[0])                      # the syringe as it lies
REST = {"cup": 0.0, "syringe": K.syringe_rest()[1], "scissors": float(-K.item_local("scissors", (0, 0, 0))[2])}


def blades(tail=None, yaw: float = 0.0, pitch: float = 0.0, gap: float = K.F_OPEN):
    """The two blades' poses for forceps whose tail is at ``tail``, turned ``yaw`` about z and tipped ``pitch`` rad
    nose down, with the tips ``gap`` apart."""
    tail = T.tail_start() if tail is None else np.asarray(tail, float)
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    base = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.0]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    out = []
    for side in (1.0, -1.0):
        a = side * gap / K.F_LEN / 2
        rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1.0]])
        out.append((tail, K._quat(base @ rz)))
    return tuple(out)


def items(**moved) -> dict:
    """Every item where it starts, except those given as name=(x, y, height above the table of what it rests on):
    the item's own origin is put over (x, y), the item lying or standing as it does on a level surface."""
    out = {n: T.item_start(n) for n in T.ITEMS}
    for n, (x, y, z) in moved.items():
        out[n] = (np.array([x, y, T.TOP + z + REST[n]]), out[n][1])
    return out


def step(s=None, arm=FAR, bl=None, it=None, speeds=AT_REST, n: int = 1) -> dict:
    s = T.new_state() if s is None else s
    for _ in range(n):
        T.advance(s, arm, blades() if bl is None else bl, items() if it is None else it, speeds)
    return s


def total(s) -> float:
    return T.score_from(not s["dirty"]["field"], s["items_on"], s["ready"])


ON_FIELD = {"cup": (0.392, -0.070, T.FIELD_TOP), "syringe": (0.410, -0.015, T.FIELD_TOP),
            "scissors": (0.412, 0.045, T.FIELD_TOP)}


# -- shapes ------------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("pieces", [lambda: K.blade_pieces(1.0), lambda: K.blade_pieces(-1.0), K.cup_pieces,
                                    K.syringe_pieces, K.scissors_pieces])
def test_every_piece_is_a_solid_convex_hull(pieces):
    for pts, color in pieces():
        assert 4 <= len(pts) <= 64 and len(color) == 3
        faces = K.hull_faces(pts)
        assert len(faces) >= 4
        for f in faces:
            a, b, c = pts[f[0]], pts[f[1]], pts[f[2]]
            assert ((pts - a) @ np.cross(b - a, c - a)).max() < 1e-9      # every other point is behind the face
        assert np.ptp(pts, axis=0).min() > 0.0005                         # not flat


def test_forceps_are_the_stated_size():
    pts = np.vstack([p for p, _ in K.blade_pieces(1.0)])
    assert np.ptp(pts[:, 0]) == pytest.approx(0.140)
    assert np.ptp(pts[:, 2]) == pytest.approx(0.011)
    f = K.forceps_points(*blades()[0], *blades()[1])
    assert f["gap"] == pytest.approx(K.F_OPEN, abs=1e-4) and f["up"] == pytest.approx(0.0, abs=1e-9)
    assert K.forceps_points(*blades(gap=0.0)[0], *blades(gap=0.0)[1])["gap"] == pytest.approx(0.0, abs=1e-9)
    # with the tips together the bowed blades still stand apart where the pads hold them: 8 mm over both blades
    assert 2 * (K.bow(K.F_BOW_AT) + K.F_TH) == pytest.approx(0.008)
    assert K.WORK + K.HANDLE < K.F_LEN


def test_spring_gives_the_stated_squeeze():
    assert K.F_STIFF * K.F_ANGLE / K.F_GRIP == pytest.approx(K.F_CLOSE_N)


def test_cup_is_the_stated_size_and_hollow():
    pts = np.vstack([p for p, _ in K.cup_pieces()])
    assert np.ptp(pts[:, 0]) == pytest.approx(0.051, abs=0.0005) and pts[:, 2].max() == pytest.approx(0.0445)
    inside = np.array([[0.0, 0.0, 0.02]])
    for p, _ in K.cup_pieces()[1:]:                                       # no wall piece reaches the axis
        assert np.hypot(p[:, 0], p[:, 1]).min() > 0.015
    assert K.in_cup(inside, (0, 0, 0), I, 0.0)[0] and not K.in_cup([[0.04, 0, 0.02]], (0, 0, 0), I, 0.004)[0]


def test_syringe_is_the_stated_size_and_weight():
    barrel = K.syringe_pieces()[0][0]
    assert np.ptp(barrel[:, 0]) == pytest.approx(0.0683) and 2 * K.S_BARREL_R == pytest.approx(0.27 * 0.0254, abs=5e-6)
    assert np.ptp(barrel[:, 1]) == pytest.approx(2 * K.S_BARREL_R / math.cos(math.pi / 12) * math.cos(math.pi / 12), abs=3e-4)
    allp = np.vstack([p for p, _ in K.syringe_pieces()])
    assert 0.085 < np.ptp(allp[:, 0]) < 0.100 and np.ptp(allp[:, 1]) == pytest.approx(0.020)
    assert 2 * K.S_BARREL_R + 0.003 <= 0.010                              # the tips, 10 mm apart, go astride the barrel
    # the body is heavier than the syringe and a force carries the difference
    assert K.S_BODY > K.S_MASS and K.body_mass(K.Syringe("s", pos=(0, 0, 0), size=(0.09, 0.02, 0.01))) == K.S_BODY
    pitch, height = K.syringe_rest()
    low = K.place((0, 0, height), K.turn_y(pitch), allp)[:, 2]
    assert low.min() == pytest.approx(0.0, abs=5e-5) and 0 < math.degrees(pitch) < 3


def test_scissors_are_the_stated_size_and_balance():
    allp = np.vstack([p for p, _ in K.scissors_pieces()])
    assert np.ptp(allp[:, 0]) == pytest.approx(K.X_LEN, abs=2e-4) and K.X_LEN == 0.115
    assert allp[:, 2].min() == pytest.approx(-REST["scissors"], abs=1e-9)
    vol = K._item("scissors")["volume"]
    assert 6000 < K.X_MASS / vol < 8000                                    # kg/m3: about what steel weighs
    back = K.ring_back()
    assert back[0] == pytest.approx(allp[:, 0].min() + K.X_WIRE / 2, abs=2e-4) and back[1] < 0
    # the bar at the back of a ring, with room for a tip inside the ring
    assert K.near_item("scissors", [back], (0, 0, 0), I, 0.0)[0]
    assert not K.near_item("scissors", [back + (0.006, 0.0, 0.0)], (0, 0, 0), I, 0.0015)[0]
    assert not K.near_item("scissors", [K.item_local("scissors", (0.05, 0.0, 0.0015))], (0, 0, 0), I, 0.0015)[0]
    assert K.near_item("scissors", [K.item_local("scissors", (0.10, 0.0, 0.001))], (0, 0, 0), I, 0.0)[0]


def test_arm_shapes_are_read_from_the_robots_urdf():
    links = K.link_points()
    assert set(links) == set(K.ARM_LINKS) and all(len(v) > 30 for v in links.values())
    assert np.ptp(links["gripper"], axis=0).max() < 0.2


# -- the layout --------------------------------------------------------------------------------------------------------

def test_layout_starts_sterile_and_scores_nothing():
    assert T.LOT in ("A", "B")
    s = step(n=3)
    assert not any(s["dirty"].values()) and s["items_on"] == 0 and total(s) == 0.0
    f = K.forceps_points(*blades()[0], *blades()[1])
    assert np.all(T.over(f["work"], "field")) and not np.any(T.over(f["handle"], "field"))
    assert T.touches(f["work"], "field")                                  # the working end rests on the field
    assert T.NEAR - f["tail"][0] == pytest.approx(T.OVERHANG)             # the handle sticks out over the edge
    for n in T.ITEMS:
        p = K.item_points(n, *T.item_start(n))
        assert np.all(T.over(p, "pack"))
        assert 0.0003 < p[:, 2].min() - T.TOP - T.PACK_TOP < 0.0008      # just above the wrapper
    assert s["down"] and s["ready"]                                       # the forceps start where they should end


def test_items_start_apart_from_each_other():
    pts = {n: K.item_points(n, *T.item_start(n)) for n in T.ITEMS}
    for a in T.ITEMS:
        for b in T.ITEMS:
            if a < b:
                assert not np.any(K.near_item(b, pts[a], *T.item_start(b), 0.010))


def test_scissors_lie_with_one_shank_pointing_straight_away():
    pos, quat = T.item_start("scissors")
    screw = K.place(pos, quat, K.item_local("scissors", (K.X_PIVOT_AT, 0, 0)))
    assert screw[:2] == pytest.approx(T.SCISSORS_AT) and math.degrees(T.SCISSORS_YAW) == pytest.approx(15.1, abs=0.1)
    ring = K.place(pos, quat, K.item_local("scissors", (2 * K.X_RING[0] - K.X_WIRE, K.X_RING_Y, 0)))   # the shank's ends
    assert ring[1] == pytest.approx(screw[1], abs=1e-6) and ring[0] < screw[0]
    hold = K.place(pos, quat, K.shank_hold())
    assert hold[0] == pytest.approx(pos[0], abs=1e-6) and hold[1] - pos[1] == pytest.approx(0.008, abs=0.0005)
    assert hold[1] == pytest.approx(screw[1], abs=1e-6)                    # on that shank's centre line
    # nothing else of the scissors within 4 mm of the shank's sides there: room for a blade either side
    for dy in (-1, 1):
        beside = hold + (0.0, dy * (K.X_SHANK[0] / 2 + 0.003), 0.0)
        assert not K.near_item("scissors", [beside], pos, quat, 0.001)[0]
    pos, quat = T.item_start("syringe")
    assert K.item_points("syringe", pos, quat)[:, 0].min() == pytest.approx(T.SYRINGE_AT[0], abs=3e-4)


def test_resting_arms_are_clear_of_both_sterile_areas():
    # the arms' rest pose reaches x = 0.337 m at most (the fingertips); both sterile parts start beyond that
    assert T.FIELD_IN[0] > 0.337 + 0.015 and T.PACK_IN[0] > 0.337 + 0.015


def test_demo_places_are_inside_the_border_and_apart():
    import demo as D

    r = K.C_TOP_R
    assert T.FIELD_IN[0] + r + 0.008 <= D.CUP_SLOT[0] and T.FIELD_IN[2] + r + 0.02 <= D.CUP_SLOT[1]
    assert D.CUP_SLOT[1] - r > T.FORCEPS_Y + 0.015                        # clear of where the forceps go back
    assert D.SYR_SLOT_Y - K.S_FLANGE[0] > D.CUP_SLOT[1] + r + 0.01
    pos, quat = T.item_start("scissors")
    hold = K.place(pos, quat, K.shank_hold())
    placed = K.item_points("scissors", pos + (0.0, D.SCI_SLOT_Y - hold[1], T.FIELD_TOP - T.PACK_TOP), quat)
    assert placed[:, 1].min() > D.SYR_SLOT_Y + K.S_FLANGE[0] + 0.01 and np.all(T._inside(placed, T.FIELD_IN))
    assert placed[:, 0].min() > T.FIELD_IN[0] + 0.01 and T.SYRINGE_AT[0] > T.FIELD_IN[0] + 0.01
    assert math.degrees(D.L_PICK - D.L_WORK) == pytest.approx(44.0)
    assert D.pads_for(K, D.TIPS_HOLD) < D.pads_for(K, D.TIPS_OPEN) < 2 * (K.F_BOW + K.F_TH) + K.F_OPEN * D.PINCH / K.F_LEN
    assert D.pads_for(K, K.X_SHANK[0], D.SCI_FORCE) < D.pads_for(K, K.X_SHANK[0]) < D.pads_for(K, D.SCI_OPEN)
    assert D.SCI_OPEN >= K.X_SHANK[0] + 0.004 and D.TIPS_OPEN >= 2 * K.S_BARREL_R + 0.003


# -- the contamination record ------------------------------------------------------------------------------------------

@pytest.mark.parametrize("area,xy", [("field", (0.40, 0.20)), ("pack", (0.56, -0.40))])
def test_an_arm_above_a_sterile_area_contaminates_it(area, xy):
    s = step(arm=np.array([[xy[0], xy[1], T.TOP + 0.30]]))
    assert s["dirty"][area] and s["arm_over"]
    other = "pack" if area == "field" else "field"
    assert not s["dirty"][other]


def test_an_arm_above_a_border_contaminates_nothing():
    s = step(arm=np.array([[T.NEAR + 0.020, 0.0, T.TOP + 0.05], [T.NEAR + 0.020, -0.40, T.TOP + 0.05]]))
    assert not any(s["dirty"].values())


def test_field_contaminated_scores_zero_whatever_lies_on_it():
    s = step(it=items(**ON_FIELD), n=2)
    assert s["items_on"] == 3 and total(s) == 1.0
    step(s, arm=np.array([[0.50, 0.25, T.TOP + 0.30]]), it=items(**ON_FIELD))
    step(s, it=items(**ON_FIELD))
    assert s["dirty"]["field"] and s["dirty"]["cup"] and s["dirty"]["scissors"] and s["dirty"]["forceps"]
    assert total(s) == 0.0


def test_pack_contaminated_takes_the_items_still_on_it():
    s = step(arm=np.array([[0.56, -0.40, T.TOP + 0.20]]))
    assert s["dirty"]["pack"] and all(s["dirty"][n] for n in T.ITEMS) and not s["dirty"]["field"]
    step(s, it=items(**ON_FIELD))                                         # ... and they then contaminate the field
    assert s["dirty"]["field"] and total(s) == 0.0


def test_tips_above_the_handle_contaminate_held_forceps():
    up = T.tail_start() + (-0.10, 0.0, 0.08)                               # held in the air in front of the field
    hand = np.array([up + (0.03, 0.012, 0.0)])                             # a gripper point at the handle
    assert not step(arm=hand, bl=blades(tail=up, pitch=math.radians(20)))["dirty"]["forceps"]     # nose down
    assert not step(arm=hand, bl=blades(tail=up, pitch=math.radians(-1.5)))["dirty"]["forceps"]   # 3.7 mm up: allowed
    s = step(arm=hand, bl=blades(tail=up, pitch=math.radians(-4.0)))                              # 9.8 mm up
    assert s["dirty"]["forceps"] and "tips raised" in s["why"]["forceps"]
    # forceps that have been let go may rock on the tray's edge: the rule is about how they are held
    rocking = blades(tail=T.tail_start() + (0.0, 0.0, -0.004), pitch=math.radians(-4.0))
    assert not step(bl=rocking)["dirty"]["forceps"]


def test_working_end_on_the_table_or_a_border_contaminates_the_forceps():
    on_table = np.array([0.20, 0.0, T.TOP + 0.006])
    assert step(bl=blades(tail=on_table))["dirty"]["forceps"]
    # lying along the field's near border, 1 cm in from the edge
    border = np.array([T.NEAR + 0.010, 0.10, T.TOP + T.FIELD_TOP + 0.006])
    assert step(bl=blades(tail=border, yaw=math.pi / 2))["dirty"]["forceps"]
    # the same lying wholly inside the border: the handle on the field contaminates the field instead
    inside = np.array([T.NEAR + 0.080, 0.10, T.TOP + T.FIELD_TOP + 0.006])
    s = step(bl=blades(tail=inside, yaw=math.pi / 2))
    assert s["dirty"]["field"] and "handle" in s["why"]["field"]


def test_an_arm_on_the_working_end_contaminates_the_forceps():
    f = K.forceps_points(*blades()[0], *blades()[1])
    near_tip = f["tip"] + (0.0, 0.0, 0.006)                                # over the field, though: the field goes too
    s = step(arm=np.array([near_tip]))
    assert s["dirty"]["forceps"] and s["dirty"]["field"]
    by_handle = f["tail"] + (0.03, 0.012, 0.0)
    assert not any(step(arm=np.array([by_handle]))["dirty"].values())      # holding the handle is what it is for


def _on_cup_wall() -> tuple:
    c = T.item_start("cup")[0]
    tail = c + (-K.F_LEN, K.C_TOP_R - 0.002, 0.08)                         # nose down into the cup's side wall
    return blades(tail=tail, pitch=math.asin(0.045 / K.F_LEN), gap=0.003)


def test_contaminated_forceps_contaminate_what_they_touch():
    s = T.new_state()
    T._soil(s, "forceps", "test")
    step(s, bl=_on_cup_wall())
    assert s["dirty"]["cup"] and "contaminated forceps" in s["why"]["cup"]
    # the contaminated cup stands on the pack: the pack goes, and the other items on it; the field is untouched
    assert s["dirty"]["pack"] and s["dirty"]["syringe"] and s["dirty"]["scissors"] and not s["dirty"]["field"]
    assert not any(step(bl=_on_cup_wall())["dirty"].values())              # clean forceps on a clean cup: nothing


@pytest.mark.parametrize("name", ["syringe", "scissors"])
def test_contaminated_forceps_contaminate_the_lying_items_too(name):
    pos, quat = T.item_start(name)
    at = pos if name == "syringe" else K.place(pos, quat, K.shank_hold())
    reach = math.sqrt(K.F_LEN ** 2 - 0.08 ** 2)
    tail = np.array([at[0] - reach, at[1], at[2] + 0.08])                  # the tips astride it, nose down
    bl = blades(tail=tail, pitch=math.asin(0.08 / K.F_LEN), gap=0.008)
    assert not any(step(bl=bl)["dirty"].values())                          # clean forceps: nothing
    s = T.new_state()
    T._soil(s, "forceps", "test")
    step(s, bl=bl)
    assert s["dirty"][name] and s["dirty"]["pack"]


def test_a_contaminated_item_contaminates_one_it_touches():
    # the cup has been on the table and is set down against the syringe, both in the air above the field's front
    held = items(cup=(0.25, 0.0, 0.10), syringe=(0.25 + K.C_BASE_R + 0.040, 0.0, 0.10 + 0.01))
    s = T.new_state()
    T._soil(s, "cup", "test")
    step(s, it=held)
    assert s["dirty"]["syringe"] and "contaminated cup" in s["why"]["syringe"]
    assert not step(it=held)["dirty"]["syringe"]


@pytest.mark.parametrize("name", list(T.ITEMS))
def test_item_on_the_table_or_a_border_is_contaminated(name):
    assert step(it=items(**{name: (0.25, -0.20, 0.0005)}))["dirty"][name]
    x = T.NEAR + {"cup": 0.030, "syringe": 0.039, "scissors": 0.022}[name]  # part of what it rests on is the border
    s = step(it=items(**{name: (x, 0.05, T.FIELD_TOP)}))
    assert s["dirty"][name] and not s["on"][name]
    s = step(it=items(**{name: (T.NEAR + 0.25, 0.05, T.FIELD_TOP + 0.06)}))   # carried over the field: touches nothing
    assert not s["dirty"][name]


def test_cup_against_the_trays_side_is_contaminated():
    s = step(it=items(cup=(T.NEAR - K.C_BASE_R - 0.0005, 0.0, 0.005)))
    assert s["dirty"]["cup"]


def test_an_arm_at_the_cup_contaminates_it():
    held = (0.25, -0.20, 0.10)                                             # the cup in the air in front of the pack
    c = np.array([held[0], held[1], T.TOP + held[2]])
    s = step(arm=np.array([c + (-K.C_TOP_R - 0.003, 0.0, 0.04)]), it=items(cup=held))
    assert s["dirty"]["cup"] and "arm" in s["why"]["cup"]
    assert not any(v for k, v in s["dirty"].items() if k != "cup")
    assert not step(arm=np.array([c + (-K.C_TOP_R - 0.03, 0.0, 0.04)]), it=items(cup=held))["dirty"]["cup"]
    inside = step(arm=np.array([c + (0.0, 0.0, 0.03)]), it=items(cup=held))       # a fingertip inside the cup
    assert inside["dirty"]["cup"]


@pytest.mark.parametrize("name", ["syringe", "scissors"])
def test_an_arm_at_a_lying_item_contaminates_it(name):
    held = (0.25, -0.20, 0.10)                                             # in the air in front of the pack
    it = items(**{name: held})
    pos, quat = it[name]
    at = pos if name == "syringe" else K.place(pos, quat, K.shank_hold())
    s = step(arm=np.array([at + (0.0, 0.0, 0.005)]), it=it)                # a pad 2 to 3 mm off its surface
    assert s["dirty"][name] and "arm" in s["why"][name]
    assert not step(arm=np.array([at + (0.0, 0.0, 0.012)]), it=it)["dirty"][name]
    if name == "scissors":                                                 # a fingertip through the ring touches nothing
        middle = K.place(pos, quat, K.item_local("scissors", (K.X_RING[0], -K.X_RING_Y, K.X_WIRE / 2)))
        assert not step(arm=np.array([middle]), it=it)["dirty"][name]


# -- what counts as on the field ---------------------------------------------------------------------------------------

def test_cup_must_be_wholly_inside_the_border():
    x0 = T.FIELD_IN[0]
    s = step(it=items(cup=(x0 + K.C_TOP_R + 0.002, 0.0, T.FIELD_TOP)))
    assert s["on"]["cup"] and total(s) == 0.25
    s = step(it=items(cup=(x0 + K.C_BASE_R + 0.002, 0.0, T.FIELD_TOP)))   # base inside, rim over the border
    assert not s["on"]["cup"] and not s["dirty"]["cup"] and total(s) == 0.0


def test_cup_must_be_upright_let_go_and_at_rest():
    on = items(cup=ON_FIELD["cup"])
    assert step(it=on)["on"]["cup"]
    assert not step(it=on, speeds={**AT_REST, "cup": 0.05})["on"]["cup"]
    tipped = dict(on)
    tipped["cup"] = (on["cup"][0], (math.sin(math.radians(6)), 0.0, 0.0, math.cos(math.radians(6))))   # 12 degrees
    assert not step(it=tipped)["on"]["cup"]
    c = on["cup"][0]
    tail = c + (-K.F_LEN, K.C_TOP_R - 0.002, 0.08)
    held = blades(tail=tail, pitch=math.asin(0.045 / K.F_LEN), gap=0.003)  # the tips still on its wall
    assert not step(it=on, bl=held)["on"]["cup"]
    lifted = items(cup=(ON_FIELD["cup"][0], ON_FIELD["cup"][1], T.FIELD_TOP + 0.02))
    assert not step(it=lifted)["on"]["cup"]


@pytest.mark.parametrize("name", ["syringe", "scissors"])
def test_lying_item_must_be_inside_the_border_low_let_go_and_at_rest(name):
    on = items(**{name: ON_FIELD[name]})
    s = step(it=on)
    assert s["on"][name] and total(s) == 0.25
    assert not step(it=on, speeds={**AT_REST, name: 0.05})["on"][name]
    x, y, z = ON_FIELD[name]
    assert not step(it=items(**{name: (x, y, z + 0.02)}))["on"][name]      # held 2 cm above the field
    pos, quat = on[name]
    back = float(pos[0] - K.item_points(name, pos, quat)[:, 0].min())      # from its near end to its own origin
    edge = step(it=items(**{name: (T.FIELD_IN[0] + back - 0.012, y, z)}))  # its near end over the border, in the air
    assert not edge["on"][name]
    at = pos if name == "syringe" else K.place(pos, quat, K.shank_hold())
    assert not step(it=on, arm=np.array([at + (-0.02, 0.0, 0.01)]))["on"][name]            # an arm 2 cm away
    # propped up: one end on the field, the other 4 cm up (leaning on something)
    lean = math.asin(0.04 / float(np.ptp(K.item_points(name, pos, quat)[:, 0])))
    q = K._quat(K._rotation(K.turn_y(-lean)) @ K._rotation(quat))
    low = K.item_points(name, (0, 0, 0), q)[:, 2].min()
    propped = dict(on)
    propped[name] = (np.array([x, y, T.TOP + T.FIELD_TOP - low + 0.0005]), q)
    s = step(it=propped)
    assert not s["on"][name] and not s["dirty"][name]


def test_forceps_must_be_let_go_and_left_on_a_sterile_area():
    done = items(**ON_FIELD)
    assert total(step(it=done)) == 1.0
    f = K.forceps_points(*blades()[0], *blades()[1])
    held = np.array([f["tail"] + (0.03, 0.012, 0.0)])
    s = step(it=done, arm=held)
    assert not s["down"] and total(s) == 0.75
    assert total(step(it=done, speeds={**AT_REST, "forceps": 0.1})) == 0.75
    on_table = blades(tail=np.array([0.20, 0.0, T.TOP + 0.006]))
    s = step(it=done, bl=on_table)
    assert s["down"] and not s["ready"] and total(s) == 0.75
    on_pack = blades(tail=np.array([T.NEAR - 0.05, -0.30, T.TOP + T.PACK_TOP + 0.006]))
    assert total(step(it=done, bl=on_pack)) == 1.0                        # the pack's sterile part will do as well


def test_each_item_counts_a_quarter():
    assert total(step(it=items(cup=ON_FIELD["cup"]))) == 0.25
    assert total(step(it=items(cup=ON_FIELD["cup"], scissors=ON_FIELD["scissors"]))) == 0.5
    assert total(step(it=items(syringe=ON_FIELD["syringe"], scissors=ON_FIELD["scissors"]))) == 0.5


# -- the scoring rule --------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("clean,n,ready,want", [
    (True, 0, True, 0.0), (False, 3, True, 0.0), (False, 1, False, 0.0),
    (True, 1, False, 0.25), (True, 1, True, 0.25), (True, 2, False, 0.5), (True, 2, True, 0.5),
    (True, 3, False, 0.75), (True, 3, True, 1.0)])
def test_score_rule(clean, n, ready, want):
    assert T.score_from(clean, n, ready) == want


def test_score_only_reads_the_trial_state():
    T._clear()
    assert T.score({}) == 0.0 and not T.success({})
    T._STATE.update(items_on=3, down=True, ready=True)
    assert T.score({}) == 1.0 and T.success({})
    T._STATE["dirty"]["field"] = True
    assert T.score({}) == 0.0
    T._clear()


def test_text_matches_code():
    doc = json.loads((HERE / "task.json").read_text())
    assert doc["instruction"] == T.INSTRUCTION
    s = doc["scoring"]
    for phrase in ("1.5 mm", "4 mm", "2.5 cm", "5 cm", "6.5 cm", "5 mm", "10 degrees", "3 cm", "2 cm", "0.25", "0.5", "0.75", "1.0"):
        assert phrase in s, phrase
    assert (T.TOUCH, T.NEAR_ARM, T.BORDER, K.WORK, K.HANDLE, T.TIPS_UP, T.CLEAR, T.TIP_CLEAR, T.STILL, T.LYING) == \
        (0.0015, 0.004, 0.025, 0.050, 0.065, 0.005, 0.030, 0.020, 0.02, 0.025)
    assert math.degrees(T.UPRIGHT) == pytest.approx(10.0)
    assert "2,400 steps" in doc["run_limit"]
    assert len(doc["fidelity"]) <= 500 and len(doc["title"]) <= 120
    assert len(doc.get("context", "")) <= 1000 and len(doc["scores"]) <= 300
