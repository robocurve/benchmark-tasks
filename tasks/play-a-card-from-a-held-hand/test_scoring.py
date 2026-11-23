"""Offline checks for the play-a-card task: the scoring rule, how cards are judged in and out of the hand, the fan's
geometry, the right hand's pinch on the ace and the left hand's hold. No simulator needed (yam_arena's task-file
loader and IK are plain Python).

    python -m pytest -q test_scoring.py
"""
import math
import pathlib
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


@pytest.fixture(scope="module")
def T():
    from yam_arena import task_file

    return task_file.load(str(HERE / "task.py")).module


@pytest.fixture(scope="module")
def D():
    import demo

    return demo


ALL_HELD = [True] * 5


def held_but(T, *gone):
    return [T.CARDS[k] not in gone for k in range(T.N)]


GOOD = dict(ace_out=True, ace_table=True, face_up=True, flat=True, dist=0.01, rest=True, clear=True)


# -- the rule ----------------------------------------------------------------------------------------------------------
def test_full_score(T):
    assert T.judge(held_but(T, T.ACE), **GOOD) == 1.0


def test_nothing_done_scores_zero(T):
    assert T.judge(ALL_HELD, **{**GOOD, "ace_out": False, "ace_table": False}) == 0.0


@pytest.mark.parametrize("other", range(4))
def test_any_other_card_out_of_the_hand_scores_zero(T, other):
    name = T.OTHERS[other]
    assert T.judge(held_but(T, T.ACE, name), **GOOD) == 0.0


def test_ace_out_but_not_on_the_table(T):
    assert T.judge(held_but(T, T.ACE), **{**GOOD, "ace_table": False}) == 0.25


@pytest.mark.parametrize("fault", [dict(face_up=False), dict(flat=False), dict(dist=0.101)])
def test_ace_on_the_table_but_face_down_tilted_or_far(T, fault):
    assert T.judge(held_but(T, T.ACE), **{**GOOD, **fault}) == 0.5


@pytest.mark.parametrize("fault", [dict(dist=0.051), dict(clear=False), dict(rest=False)])
def test_ace_near_the_cross_but_not_finished(T, fault):
    assert T.judge(held_but(T, T.ACE), **{**GOOD, **fault}) == 0.75


def test_thresholds(T):
    assert T.judge(held_but(T, T.ACE), **{**GOOD, "dist": 0.050}) == 1.0
    assert T.judge(held_but(T, T.ACE), **{**GOOD, "dist": 0.100, "clear": False}) == 0.75


# -- judging cards from their poses --------------------------------------------------------------------------------------
def start_cards(T) -> dict:
    return {n: T.card_start(k) for k, n in enumerate(T.CARDS)}


def left_pads(T) -> np.ndarray:
    """Points on the left pads at the start, as ``pad_points`` gives them in the simulator."""
    tcp, rot = T.hold_frame()
    half = T.stack_thickness() / 2
    pts = []
    for s in (-1.0, 1.0):
        for along in np.linspace(-0.005, 0.013, 4):
            for across in np.linspace(-0.0135, 0.0135, 4):
                pts.append(tcp + rot[:, 0] * s * half + rot[:, 2] * along + rot[:, 1] * across)
    return np.array(pts)


FAR = np.array([[0.3, -0.5, 1.2]])        # a right gripper well away from everything


def face_up_on_table(T, xy, yaw=0.3, lift=0.0):
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ np.diag([1.0, -1.0, -1.0])
    return np.array([xy[0], xy[1], T.TOP + T.K.CARD_T / 2 + lift]), T.K._quat(rot)


def test_start_all_in_hand(T):
    out = T.sort_cards(start_cards(T), left_pads(T), FAR)
    assert out["in_hand"] == ALL_HELD and not out["ace_out"] and not out["ace_table"]
    assert max(out["hold"]) < 0.002


def test_played_ace_scores_full(T):
    cards = start_cards(T)
    cards[T.ACE] = face_up_on_table(T, (T.CROSS_XY[0] + 0.02, T.CROSS_XY[1] - 0.01))
    out = T.sort_cards(cards, left_pads(T), FAR)
    assert out["in_hand"] == held_but(T, T.ACE)
    assert out["ace_out"] and out["ace_table"] and out["face_up"] and out["flat"] and out["clear"]
    assert T.judge(out["in_hand"], out["ace_out"], out["ace_table"], out["face_up"], out["flat"], out["dist"], True,
                   out["clear"]) == 1.0


def test_ace_face_down_on_the_cross_scores_half(T):
    cards = start_cards(T)
    pos, _ = face_up_on_table(T, T.CROSS_XY)
    cards[T.ACE] = (pos, T.K._quat(np.eye(3)))
    out = T.sort_cards(cards, left_pads(T), FAR)
    assert out["ace_table"] and not out["face_up"]
    assert T.judge(out["in_hand"], out["ace_out"], out["ace_table"], out["face_up"], out["flat"], out["dist"], True,
                   out["clear"]) == 0.5


def test_a_card_dropped_on_the_table_is_out_of_the_hand(T):
    cards = start_cards(T)
    cards[T.ACE] = face_up_on_table(T, T.CROSS_XY)
    cards[T.OTHERS[0]] = face_up_on_table(T, (0.25, 0.10))
    out = T.sort_cards(cards, left_pads(T), FAR)
    assert not out["in_hand"][0]
    assert T.judge(out["in_hand"], out["ace_out"], out["ace_table"], out["face_up"], out["flat"], out["dist"], True,
                   out["clear"]) == 0.0


def test_a_card_hanging_from_the_pads_near_the_table_is_out_of_the_hand(T):
    """A card that slipped and leans on the table, its corner still at the pads, is not held."""
    cards = start_cards(T)
    name = T.OTHERS[1]
    pos, quat = cards[name]
    cards[name] = (pos - np.array([0.0, 0.0, pos[2] - T.TOP - 0.02]), quat)   # its corners dip below 3 cm
    out = T.sort_cards(cards, left_pads(T), FAR)
    assert not out["in_hand"][T.CARDS.index(name)]


def test_right_gripper_on_the_ace_is_not_clear(T):
    cards = start_cards(T)
    cards[T.ACE] = face_up_on_table(T, T.CROSS_XY)
    hand = np.array([[T.CROSS_XY[0], T.CROSS_XY[1], T.TOP + 0.01]])
    out = T.sort_cards(cards, left_pads(T), hand)
    assert not out["clear"]


# -- the fan -----------------------------------------------------------------------------------------------------------
def test_fan_order_and_layers(T):
    assert T.RANKS == (1, 2, 6, 4, 5) and T.PLAY == 0
    assert [round(math.degrees(T.fan_angle(k))) for k in range(T.N)] == [40, 20, 0, -20, -40]
    # the leftmost card, the ace, is in front (toward the faces' side, so uppermost); the rightmost behind
    offs = [float((T.card_start(k)[0] - T.PIVOT) @ T.FACE) for k in range(T.N)]
    assert all(a > b for a, b in zip(offs, offs[1:]))


def test_cards_face_the_overhead_camera(T):
    for k in range(T.N):
        _, quat = T.card_start(k)
        rot = T.K._rotation(quat)
        assert np.allclose(rot.T @ rot, np.eye(3), atol=1e-9)
        assert np.allclose(rot[:, 2], -T.FACE, atol=1e-9)           # backs away from the camera
        assert T.FACE[2] > 0.8 and T.FACE[0] < 0                       # faces up and toward the robot


def test_every_card_is_pinched_at_the_pivot(T):
    """All five cards cover the middle of the left pads."""
    tcp, rot = T.hold_frame()
    p2 = T.to_plane(tcp)[:2]
    for k in range(T.N):
        assert inside(p2, T.card_polygon(k), 0.003), k


def test_a_middle_card_would_take_the_one_above_it(T):
    """Why the ace is the front card. The fan is reclined, so each card rests on the one under it: at the pinch the
    card on top of a drawn card is pressed onto it harder (by its own weight) than onto the card above it, and with
    one friction coefficient for every pair, drawing a card together with the one on top of it takes less force than
    drawing it alone. For the front card there is nothing on top but the pad."""
    mu, mu_pad = T.K.CARD_FRICTION[0], T.K.PAD_ON_CARD[0]
    w = T.K.CARD_MASS * 9.81 * T.FACE[2]                               # each card's weight across the plane
    for clamp in (0.05, 0.3, 1.0, 3.0):
        # interfaces from the front pad down: pad|c0, c0|c1, ..., c4|pad; the normal force grows by w at each
        normal = [clamp + k * w for k in range(T.N + 1)]

        def pull(top, bottom):                                         # cards top..bottom drawn together
            return ((mu_pad if top == 0 else mu) * normal[top]
                    + (mu_pad if bottom == T.N - 1 else mu) * normal[bottom + 1])

        assert pull(1, 2) < pull(2, 2)                                 # a middle card goes with the one on top of it
        assert pull(0, 0) < min(pull(0, b) for b in range(1, T.N))     # the front card goes alone


def inside(p, poly, margin=0.0) -> bool:
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        e = b - a
        nrm = np.array([e[1], -e[0]]) / np.linalg.norm(e)
        if (np.asarray(p) - a) @ nrm > -margin:
            return False
    return True


def zone(tcp2, ang, start, end, half, step=0.0005):
    a = np.array([math.cos(ang), math.sin(ang)])
    s = np.array([-a[1], a[0]])
    return [tcp2 + u * a + v * s for u in np.arange(start, end + 1e-9, step) for v in np.arange(-half, half + 1e-9, step)]


@pytest.fixture(scope="module")
def P(T, D):
    return D.plan(T)


def _fan2(T, vec) -> np.ndarray:
    """A world direction in the fan's own 2-D frame."""
    return np.array([float(vec @ T.FAN_RIGHT), float(vec @ T.FAN_UP)])


def test_plan_pinches_the_ace_alone(T, D, P):
    """The right pads' faces lie on the ace (about 200 mm2 of them; at least 180 measured on a finer grid) and at least 3 mm clear of every other card,
    and the palm and finger carriages are clear of every card, all the way along the draw."""
    tcp2 = T.to_plane(P["tcp"])[:2]
    a2 = _fan2(T, P["rot"][:, 2])
    ang = math.atan2(a2[1], a2[0])
    d2 = _fan2(T, P["draw_dir"])
    pad = zone(tcp2, ang, *D.PAD_ZONE)
    assert sum(inside(p, T.card_polygon(T.PLAY)) for p in pad) * 0.0005 ** 2 >= 180e-6
    for s in np.arange(0.0, P["s2"] + 1e-9, 0.01):
        pad = zone(tcp2 + s * d2, ang, *D.PAD_ZONE, step=0.001)
        palm = zone(tcp2 + s * d2, ang, *D.PALM_ZONE, step=0.003)
        for k in range(T.N):
            if k != T.PLAY:
                assert not any(inside(p, T.card_polygon(k), -0.002) for p in pad), (s, k)
                assert not any(inside(p, T.card_polygon(k), -0.004) for p in palm), (s, k)
    assert not any(inside(p, T.card_polygon(T.PLAY), -0.004) for p in zone(tcp2, ang, *D.PALM_ZONE, step=0.003))


def test_plan_draws_the_ace_clear(T, D, P):
    """The left hand is eased only until the ace has left its pads; the ace is then drawn clear of every card."""
    d2 = _fan2(T, P["draw_dir"])
    left2 = T.to_plane(T.hold_frame()[0])[:2]
    lpad = D._rect(left2, T.HOLD_APPROACH, D.PAD_ZONE)
    ace = T.card_polygon(T.PLAY)
    assert D._overlap(ace + (P["s1"] - D.DRAW_MARGIN - 0.001) * d2, lpad, 0.002)
    assert not D._overlap(ace + P["s1"] * d2, lpad, 0.002)
    assert P["s1"] < 0.06 and P["s1"] <= P["s2"] < 0.15
    for k in range(1, T.N):
        assert not D._overlap(ace + P["s2"] * d2, T.card_polygon(k), 0.003)


def test_plan_is_reachable_and_lets_go_above_the_table(T, D, P):
    from yam_arena import ik, station

    base = station.BASES["right"]
    face = T.FACE / np.linalg.norm(T.FACE)
    tcp, rot = P["tcp"], P["rot"]
    seeds = [np.array([0.0, 1.0, 1.0, -0.6, 0.0, 0.0])] + [np.random.default_rng(3).uniform(ik.LOWER, ik.UPPER)
                                                          for _ in range(24)]
    poses = [(tcp, rot), (tcp - P["back"] * rot[:, 2], rot), (tcp + P["s2"] * P["draw_dir"], rot),
             (tcp + P["s2"] * P["draw_dir"] + D.LIFT * face, rot), (P["place_tcp"], P["place_rot"])]
    for pos, r in poses:
        assert D._solve(pos, r, seeds, base)[1], pos
    # at the release the ace is face up and tipped, its lowest corner 4 mm above the table, over the cross, and every
    # part of the gripper clear of the table
    pos, quat = T.card_start(T.PLAY)
    card = np.linalg.inv(D.frame(tcp, rot)) @ D.frame(pos, T.K._rotation(quat))
    placed = D.frame(P["place_tcp"], P["place_rot"]) @ card
    assert placed[2, 2] < -0.8                                         # its back down: face up
    low = float(T.K.corners(placed[:3, 3], T.K._quat(placed[:3, :3]))[:, 2].min() - T.TOP)
    assert abs(low - D.EDGE_UP) < 1e-6
    assert float((D.GRIPPER_PTS @ P["place_rot"].T + P["place_tcp"])[:, 2].min()) >= T.TOP + D.GRIP_UP - 1e-9
    assert np.hypot(*(placed[:2, 3] - np.array(T.CROSS_XY))) < 1e-6


def test_left_hold_is_reachable(T):
    from yam_arena import ik, station

    q = T.hold_q()
    tcp, rot = T.hold_frame()
    p, r = ik.tcp_pose(q, station.BASES["left"])
    assert np.linalg.norm(p - tcp) < 1e-3
    assert abs(float(r[:, 2] @ rot[:, 2])) > 0.999                     # pointing into the pinch
    assert abs(float(r[:, 0] @ rot[:, 0])) > 0.999                     # closing across the fan


def test_stack_and_grip(T, D):
    assert abs(T.stack_thickness() - (5 * 0.00029 + 4 * 0.00002)) < 1e-12
    # easing: the left pads are commanded a little inside the stack's faces, so they press on it lightly
    assert 0 < D.gap(T.stack_thickness() - 2 * D.EASE) < 0.05


def test_instruction_and_names(T):
    assert "ace" in T.INSTRUCTION and "face up" in T.INSTRUCTION and "other four" in T.INSTRUCTION
    assert T.ACE == "card_0" and len(T.OTHERS) == 4
    names = [o.name for o in T.OBJECTS]
    assert len(names) == len(set(names)) and T.CROSS_MARK in names
