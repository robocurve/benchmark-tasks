"""Offline checks for open-shelf-ready-tray (no simulator): geometry, the finger's gap, the tear's force balance,
scoring and the scripted demo's reach. Run:  PYTHONPATH=<yam-isaac-lab-arena> python -m pytest -q test_task.py"""

import math
import os

import numpy as np
import pytest

for _k in ("DEMO_VARIANT", "TIE_K", "F_LIP", "F_DIAG", "PINCH_DEPTH", "PEEL_DEG", "ZIP_S", "ZIP_F"):
    os.environ.pop(_k, None)

import task as T  # noqa: E402


def test_objects():
    names = [o.name for o in T.OBJECTS]
    assert len(names) == len(set(names)) == 4 + 12
    assert names.index("tray") < names.index("panel") < names.index("flap_left")
    assert len(T.TIES) == len(T.LIP_TIE_Y) + 6 == 15


def test_cans_fit_the_tray():
    inner_y = T.TW / 2 - T.T if hasattr(T, "T") else T.TW / 2 - T.B
    assert max(abs(y) for y in T.CAN_Y) + T.CAN_R < inner_y - 0.0005
    assert T.CAN_X[-1] + T.CAN_R < T.TD - T.B - 0.0005
    assert T.CAN_X[0] - T.CAN_R - T.B == pytest.approx(T.CAN_FRONT_GAP)
    assert T.CAN_PITCH > 2 * T.CAN_R  # no overlap
    assert T.CAN_H < T.TH  # below the rim


def _gap_width(x_tray: float) -> float:
    """Free width (y) between the two middle front cans at depth x (tray frame)."""
    dx = T.CAN_X[0] - x_tray
    half = T.CAN_Y[1] - math.sqrt(max(T.CAN_R ** 2 - dx ** 2, 0.0))
    return 2 * half


def test_back_finger_fits_between_the_middle_cans():
    """The back finger: its pad BACK_CLEAR behind the panel, its body up to about 10 mm thick behind that, 27 mm wide."""
    pad_x = T.B + T.BACK_CLEAR
    assert T.BACK_CLEAR >= 0.004  # the arm lands a few mm off (2 mm seen)
    for x in np.linspace(pad_x, pad_x + 0.010, 6):
        assert _gap_width(x) > 0.030, (x, _gap_width(x))


def _peel_loads(n_diag_left=3, hook_h=None):
    """Rigid panel+flaps swinging out about the lip line, ties as equal stiff springs: (pull to break the outermost
    diagonal tie, worst lip tie load per newton of pull)."""
    h = hook_h if hook_h is not None else (T.TH - T.PINCH_DEPTH) - T.LIP
    pts = [T.diag_point(f) for f in T.DIAG_TIE_F][:n_diag_left]
    rel = [(x, z - T.LIP) for x, z in pts] * 2  # both sides, about the lip ties on the panel's outer bottom edge
    r2 = sum(dx * dx + dz * dz for dx, dz in rel)
    r_max = max(math.hypot(dx, dz) for dx, dz in rel)
    pull_to_tear = T.F_DIAG * r2 / (h * r_max)
    fx = sum(dz for dx, dz in rel) * h / r2  # the diagonal ties' pull back, per newton
    fz = sum(dx for dx, dz in rel) * h / r2
    lip_per_n = math.hypot(1.0 - fx, fz) / len(T.LIP_TIE_Y)
    return pull_to_tear, lip_per_n


def test_tie_strengths_are_one_factor_on_the_base_values():
    assert (T.F_DIAG, T.F_LIP) == pytest.approx((0.6 * T.TIE_K, 1.2 * T.TIE_K))


def test_peel_tears_and_the_lip_holds_meanwhile():
    pull, lip_per_n = _peel_loads()
    assert pull == pytest.approx(3.03 * T.TIE_K, rel=0.02), pull  # at the pinch, 2 cm below the top edge: 9.1 N
    assert lip_per_n * pull < T.F_LIP, (lip_per_n * pull, T.F_LIP)  # the lip doesn't go first


def _tcp_stiffness(pos, R) -> np.ndarray:
    """The left arm's translational stiffness at a TCP pose (N/m), the wrist free to turn: joint stiffness 80 / 10
    N m/rad through the IK Jacobian (the pinch on 3 mm board carries little moment)."""
    from yam_arena import ik, station

    base = np.array(station.BASES["left"])
    q, pe, _ = ik.solve(np.asarray(pos), ik.matrix_to_quat_xyzw(R), base_pos=base, iters=400)
    assert pe < 0.001
    J = np.zeros((6, 6))
    p0, R0 = ik.tcp_pose(q, base)
    for j in range(6):
        dq = np.zeros(6)
        dq[j] = 1e-6
        p1, R1 = ik.tcp_pose(q + dq, base)
        dR = R1 @ R0.T
        J[:3, j] = (p1 - p0) / 1e-6
        J[3:, j] = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]) / 2e-6
    return np.linalg.inv((J @ np.diag(1.0 / np.array([80.0, 80, 80, 10, 10, 10])) @ J.T)[:3, :3])


def test_peel_command_reaches_the_tear():
    """With the panel held by its ties, the peel command's lag alone builds a peel moment past what the top diagonal
    ties need at TIE_K before the command ends (the pinch adds some wrist moment on top: on Brev at TIE_K 1 they tore
    by 3 degrees). start_only holds after this full peel, so its diagonals tear while the lip stays whole."""
    p0, R0 = T.peel_pose(0.0)
    K = _tcp_stiffness(p0, R0)
    r = p0 - T._LIP_AXIS
    need = T.F_DIAG * 2 * sum(math.hypot(x, z - T.LIP) ** 2 for x, z in map(T.diag_point, T.DIAG_TIE_F)) \
        / math.hypot(*(np.array(T.diag_point(T.DIAG_TIE_F[-1])) - [0.0, T.LIP]))
    moment = lambda deg: -np.cross(r, K @ (T.peel_pose(deg)[0] - p0))[1]  # noqa: E731
    assert moment(T.PEEL_DEG) > 1.2 * need, (moment(T.PEEL_DEG), need)  # about 0.65 against 0.49 N m


def test_tear_runs_on_once_started():
    """With the outermost diagonal ties gone, the next ones need less pull: the tear runs down the diagonals."""
    p3, _ = _peel_loads(3)
    p2, _ = _peel_loads(2)
    p1, _ = _peel_loads(1)
    assert p1 < p2 < p3


def test_straight_lift_spreads_over_all_ties():
    """Lifting the panel straight up, in its plane: every tie shares it, so it takes far more than the peel."""
    lift_to_break = len(T.LIP_TIE_Y) * T.F_LIP + 6 * T.F_DIAG
    pull, _ = _peel_loads()
    assert lift_to_break > 4.0 * pull  # about 43 N against 8.9 N
    assert lift_to_break > 3 * PINCH_HOLD_N  # far past what a pinch can pull in the panel's plane


PEEL_TEST_N = (4.5, 9.0)  # the batch pull test (README): a peel at the panel's top edge starts the tear in this band
LIFT_TEST_N = 20.0  # ... and a straight lift of this much holds (over twice the band's top, half the sim's lift)
PINCH_HOLD_N = 2 * 0.5 * 7.5  # in-plane pull a pinch holds: friction 0.5 on both pads, 7.5 N squeeze on 3 mm board


def test_batch_pull_test_brackets_the_sim():
    """The README's pull test on a spare tray: the sim's own numbers must sit inside its bands."""
    peel_top, lip_per_n = _peel_loads(hook_h=T.TH - T.LIP)  # pulled square to the panel at its top edge
    assert PEEL_TEST_N[0] < peel_top < PEEL_TEST_N[1], peel_top  # about 6.7 N
    assert PEEL_TEST_N[1] / PEEL_TEST_N[0] == pytest.approx(2.0) and 0.8 < peel_top / math.sqrt(4.5 * 9.0) < 1.2
    assert lip_per_n * peel_top < T.F_LIP  # the lip holds while the diagonals tear
    lift = len(T.LIP_TIE_Y) * T.F_LIP + 6 * T.F_DIAG  # about 43 N
    assert lift > 2 * LIFT_TEST_N and LIFT_TEST_N > 2 * PEEL_TEST_N[1]


def test_template_matches_the_sim():
    """tray_template.svg is the sim's tear line: same ties, same places, and the file is up to date."""
    import pathlib

    import template as P

    marks = {name: (X, z) for name, X, z in P.tie_marks()}
    assert set(marks) == {n for n, *_ in T.TIES}
    for name, seg, p_seg, p_tray in T.TIE_LOCAL:
        X, z = marks[name]
        assert z == pytest.approx(p_tray[2] * 1000)
        if seg == "panel":  # front wall: across from the left fold
            assert X - P.TD == pytest.approx(T.TW / 2 * 1000 - p_tray[1] * 1000)
        else:  # side walls: back from the front face
            assert abs(X - (P.TD if p_tray[1] > 0 else P.TD + P.TW)) == pytest.approx(p_tray[0] * 1000)
    segs = P.cut_segments()
    assert len(segs) == 15 + 3  # three straight pieces, broken at the 15 ties
    gaps = sum(math.hypot(a - c, b - d) for a, b, c, d in segs)
    pieces = sum(math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in P._pieces())
    assert pieces - gaps == pytest.approx(9 * P.TIE_LIP_MM + 6 * P.TIE_DIAG_MM, abs=1e-6)
    svg_file = pathlib.Path(T.__file__).parent / "tray_template.svg"
    assert svg_file.read_text() == P.svg(), "re-run template.py"


def test_tie_geometry():
    for name, seg, p_seg, p_tray in T.TIE_LOCAL:
        # at the start each tie's two ends coincide
        sp = T._SEG_POS[seg]
        assert np.allclose(sp + p_seg, T.TRAY_POS + p_tray)
    for f in T.DIAG_TIE_F:
        x, z = T.diag_point(f)
        assert T.B < x < T.DX and T.LIP < z < T.TH


def _R_y(deg):
    t = math.radians(deg)
    return np.array([[math.cos(t), 0, math.sin(t)], [0, 1, 0], [-math.sin(t), 0, math.cos(t)]])


def test_torn_detection():
    poses = {"tray": (T.TRAY_POS, np.eye(3))}
    for n in T.SEGMENTS:
        poses[n] = (T._SEG_POS[n], np.eye(3))
    assert T.torn_ties(poses) == set()
    pp = T._SEG_POS["panel"]
    poses["panel"] = (pp + [-0.01, 0, 0], np.eye(3))
    assert {n for n in T.torn_ties(poses) if n.startswith("lip")} == {n for n, s, *_ in T.TIE_LOCAL if s == "panel"}


def test_points():
    base = {"torn": set(), "off": False, "removed": False, "tray_ok": True, "cans_ok": True, "let_go": True,
            "still": True, "tipped": False}
    assert T.points(base) == 0.0
    assert T.points({**base, "torn": {"diag_l2"}}) == 0.25
    full = {**base, "torn": {n for n, *_ in T.TIES}, "off": True, "removed": True}
    assert T.points(full) == 1.0
    assert T.points({**full, "let_go": False}) == 0.5
    assert T.points({**full, "removed": False}) == 0.5
    assert T.points({**full, "cans_ok": False}) == 0.5
    assert T.points({**full, "tipped": True}) == 0.0


def test_in_or_on_tray():
    assert T.in_or_on_tray(T.TRAY_POS + [0.1, 0, 0.05], T.TRAY_POS, np.eye(3))
    assert not T.in_or_on_tray(T.TRAY_POS + [-0.05, 0, 0.01], T.TRAY_POS, np.eye(3))


@pytest.mark.parametrize("variant", ["full", "nothing", "lift", "start_only", "hand_over_tray", "push_down"])
def test_demo_builds_and_arms_end_together(variant, monkeypatch):
    monkeypatch.setenv("DEMO_VARIANT", variant)
    plans = T.demo()
    assert T._length(plans["left"]) == pytest.approx(T._length(plans["right"]))


@pytest.mark.parametrize("variant", ["full", "lift", "start_only", "hand_over_tray", "push_down"])
def test_demo_reaches_every_waypoint(variant, monkeypatch):
    from yam_arena import ik, station

    monkeypatch.setenv("DEMO_VARIANT", variant)
    bases = {k: np.array(v) for k, v in station.BASES.items()}
    for side, wps in T.demo().items():
        q = None
        for w in wps:
            R = np.asarray(w.yaw, float)
            q, pe, re = ik.solve(np.asarray(w.pos), ik.matrix_to_quat_xyzw(R), q_init=q, base_pos=bases[side], iters=400)
            assert pe < 0.002 and math.degrees(re) < 1.5, (side, w.label, np.round(w.pos, 3), pe, math.degrees(re))


def zip_tear_forces(s_pinch: float, f_tie: float) -> list[tuple[float, float]]:
    """Ties as equal stiff springs on the rigid panel, pulled in its plane, away from the lip line, at lateral position
    s_pinch: (the tie that breaks next, the pull it takes), in order, until the panel is off."""
    left = list(T.LIP_TIE_Y)
    out = []
    while left:
        s = np.array(left)
        per_n = 1.0 / len(s) + (s_pinch - s.mean()) * (s - s.mean()) / max(((s - s.mean()) ** 2).sum(), 1e-12)
        i = int(np.argmax(np.abs(per_n)))
        out.append((left.pop(i), f_tie / abs(per_n[i])))
    return out


def test_corner_pull_tears_the_lip_from_its_left_end():
    """Pulled at the left end the lip tears from that end, each next tie needing less; the first takes about a third of
    the pull a straight pull at the middle needs (all nine at once). On these equal-spring numbers the pinch's grip is
    exceeded, but PhysX shares redundant ties unevenly: on Brev the corner pull tore the lip at estimates up to 1.37
    times the grip (TIE_K 3, 10.3 N) and slipped at 1.83 (TIE_K 4); the straight pull slipped at 2.9 (TIE_K 2)."""
    seq = zip_tear_forces(T.ZIP_S, T.F_LIP)
    assert [s for s, _ in seq[:4]] == sorted(T.LIP_TIE_Y, reverse=True)[:4]  # from the left end, in turn
    pulls = [f for _, f in seq]
    assert all(b < a for a, b in zip(pulls[:7], pulls[1:8]))  # the tear runs on with less and less
    straight = len(T.LIP_TIE_Y) * T.F_LIP
    assert pulls[0] == pytest.approx(3.43 * T.TIE_K, rel=0.02) and pulls[0] < straight / 2.9
    assert pulls[0] <= 1.4 * PINCH_HOLD_N < straight  # within what tore on Brev; a straight pull isn't


def test_corner_pull_command_matches_the_arm_compliance():
    """ZIP_OFFSET_PER_N: the arm's static compliance at the corner pinch, recomputed."""
    p, R = T.peel_pose(T.PEEL_DEG)
    C = np.linalg.inv(_tcp_stiffness(T.zip_end(p), R))
    u = T._ry(-T.PEEL_DEG)[:, 2]  # along the peeled panel, away from the lip line
    assert np.allclose(C @ u, T.ZIP_OFFSET_PER_N, atol=0.03e-3)


def _finger_points(w) -> np.ndarray:
    """Sample points over both fingers at a waypoint: pad tips 13 mm past the TCP back to the throat 44 mm behind it,
    27 mm wide, each finger body about 10 mm thick behind its pad (the pads 1.5 mm out when on the board)."""
    R = np.asarray(w.yaw, float)
    gap = max(-w.finger, T.B / 2)
    pts = []
    for a in np.linspace(-0.044, T.TIP, 6):
        for wy in (-0.0135, 0.0, 0.0135):
            for c in (gap, gap + 0.010):
                for sgn in (1.0, -1.0):
                    pts.append(np.asarray(w.pos) + R @ np.array([sgn * c, wy, a]))
    return np.array(pts)


@pytest.mark.parametrize("variant", ["full", "hand_over_tray"])
def test_corner_tear_clearance(variant, monkeypatch):
    """From easing the pinch to lifting the panel clear: the fingers stay 2 cm or more off the tray (its whole outer
    box, cans inside) and 1 cm or more off the left flap's plane; once lifted clear and carried off, the panel's
    corners stay out of the tray's box."""
    monkeypatch.setenv("DEMO_VARIANT", variant)
    left = T.demo()["left"]
    i0 = next(i for i, w in enumerate(left) if w.label.startswith("ease the pinch"))
    i1 = next((i for i, w in enumerate(left) if w.label.startswith("lift the panel clear")), len(left) - 2)
    lo, hi = T.TRAY_POS + T._TRAY_LO, T.TRAY_POS + T._TRAY_HI
    for w in left[i0:i1 + 1]:
        pts = _finger_points(w)
        d = np.linalg.norm(pts - np.clip(pts, lo, hi), axis=1).min()
        assert d > 0.02, (w.label, d)
        assert pts[:, 1].max() < T.TW / 2 - T.B - 0.01, w.label
    # the panel held at its left end, as pinched (rigidly, at the peel's angle): corners and the right flap's tip
    u, n = T._ry(-T.PEEL_DEG)[:, 2], -T._ry(-T.PEEL_DEG)[:, 0]
    h = T.TH - T.PINCH_DEPTH + T.TIP - T.LIP  # the TCP's distance from the lip line, along the panel
    corners = [(du, dy, 0.0) for du in (-h, T.TH - T.LIP - h) for dy in (T.TW / 2 - T.ZIP_S, -T.TW / 2 - T.ZIP_S)]
    corners.append((T.TH - T.LIP - h, -T.TW / 2 - T.ZIP_S, -(T.DX - T.B)))  # the right flap's tip, inside the panel
    if variant != "full":  # the hand goes back over the tray on purpose (checked below)
        return
    path = left[i1:i1 + 3]  # lifted clear (its bottom edge, torn free, has risen off the lip), carry, lower
    for a, b in zip(path, path[1:]):
        for t in np.linspace(0.0, 1.0, 11):
            tcp = (1 - t) * np.asarray(a.pos) + t * np.asarray(b.pos)
            for du, dy, dn in corners:
                c = tcp + du * u + dy * np.array([0.0, 1.0, 0.0]) + dn * n
                assert np.linalg.norm(c - np.clip(c, lo, hi)) > 0.01, (b.label, t, np.round(c, 3))


def _holds(kind: str, v) -> tuple[float, list[float]]:
    """A tie's response to its segment moving by v: (the part along its opening, the parts along its locked axes)."""
    import bodykit

    t = next(t for t in bodykit._TIES if t.name.startswith(f"tie_{kind}"))
    F = bodykit.tie_frame(t)
    locked = [F[:, 1] @ v] * t.lock_second + [F[:, 2] @ v] * t.lock_third
    return float(F[:, 0] @ v), [float(x) for x in locked]


def test_ties_hold_only_against_opening_and_out_of_plane_shear():
    """A push down on the panel or a flap closes every tie (free) and is square to every locked axis: only the board's
    contact can carry it. The peel turns the panel about the lip ties' own line (no load on them) and opens the
    diagonals. Lifting opens them all."""
    down = np.array([0.0, 0.0, -1.0])
    for kind in ("lip", "diag"):
        opening, locked = _holds(kind, down)
        assert opening < -0.7 and all(abs(x) < 1e-9 for x in locked), (kind, opening, locked)
        assert _holds(kind, -down)[0] > 0.7  # a lift opens them
    for name, seg, p_seg, p_tray in T.TIE_LOCAL:  # the peel: a small turn about the lip line (y), top out
        rel = p_tray - np.array([0.0, 0.0, T.LIP])
        move = np.array([-rel[2], 0.0, rel[0]])  # about the outer bottom edge: x' = -z, z' = x
        if seg == "panel":
            assert np.allclose(move, 0.0)  # the lip ties sit on the turn's axis
        else:
            opening, locked = _holds("diag", move)
            assert opening > 0 and all(abs(x) < 1e-9 for x in locked)


def test_the_board_below_the_line_bears_the_removable_part():
    """The panel's collision box stands GAP over the lip's top, and each flap's collision triangle GAP off the side
    wall's edge below its diagonal: touching nothing at rest, so no tie is loaded, and bearing after GAP."""
    import bodykit

    panel = next(sh for sh in T.PANEL.shapes if sh.collide)
    assert T.PANEL_POS[2] + panel.centre[2] - panel.size[2] / 2 == pytest.approx(T.TOP + T.LIP + T.GAP)
    flap = next(sh for sh in T.FLAPS[1.0].shapes if sh.collide)
    n = T.DIAG_OPENING[[0, 2]]
    for x, _, z in flap.section_a[:1] + flap.section_a[2:]:  # the two ends of its diagonal edge, flap-local
        assert np.dot([x + T.B, z + T.LIP], n) - np.dot([T.B, T.LIP], n) == pytest.approx(T.GAP)
    assert all(t.break_force for t in bodykit._TIES) and len(bodykit._TIES) == len(T.TIES)
    assert not [b for b in bodykit._BALLS if b.body0 == "tray"]  # no force-only ball ties left


def test_push_down_presses_firmly_and_misses_the_cans(monkeypatch):
    """Each press is aimed PRESS_DEPTH under its edge (about 10 N at the arm's stiffness down there), and the closed
    fingertips' footprint misses the cans' tops (just under the edges' height)."""
    monkeypatch.setenv("DEMO_VARIANT", "push_down")
    left = T.demo()["left"]
    presses = [w for w in left if w.label.startswith("press ")]
    assert len(presses) == 3
    for w, (edge, _, _) in zip(presses, T.PRESSES):
        assert w.pos[2] - T.TIP == pytest.approx(edge[2] - T.PRESS_DEPTH)
        K = _tcp_stiffness(np.asarray(w.pos) + [0.0, 0.0, T.PRESS_DEPTH], np.asarray(w.yaw, float))
        assert 6.0 < K[2, 2] * T.PRESS_DEPTH < 16.0, K[2, 2] * T.PRESS_DEPTH
        tips = [p for p in _finger_points(w) if abs((np.asarray(w.yaw) @ [0, 0, 1]) @ (p - w.pos) - T.TIP) < 1e-9]
        for c in T.CANS:
            d = min(np.hypot(*(np.asarray(p[:2]) - np.asarray(c.pos[:2]))) for p in tips) - T.CAN_R
            assert d > 0.0005, (w.label, c.name, d)


def test_hand_over_tray_ends_over_the_wall_without_touching(monkeypatch):
    """hand_over_tray ends with the TCP inside the let-go distance of the tray (so 0.5, not 1.0), its closed fingertips
    5 mm over the left side wall's top edge and over the cans' tops: nothing touched, nothing pushed."""
    monkeypatch.setenv("DEMO_VARIANT", "hand_over_tray")
    w = T.demo()["left"][-1]
    lo, hi = T.TRAY_POS + T._TRAY_LO, T.TRAY_POS + T._TRAY_HI
    tcp = np.asarray(w.pos)
    assert np.linalg.norm(tcp - np.clip(tcp, lo, hi)) < T.LET_GO - 0.005
    assert T.HOVER_WALL[0] > T.XF + T.DX + 0.015  # over the wall behind the torn-off flap
    pts = _finger_points(w)
    assert pts[:, 2].min() > T.TOP + T.TH + 0.004  # the rim; the cans' tops are lower still
    assert T.TOP + T.B + T.CAN_H + 0.0005 < T.TOP + T.TH


def test_prim_names_are_valid_usd_identifiers():
    import re

    names = [f"tie_{n}" for n, *_ in T.TIES] + ["crease_l", "crease_r"] + [o.name for o in T.OBJECTS]
    assert all(re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", n) for n in names), names
