"""Offline checks of the eye-drop task: dropkit's bottle model, the head's geometry, every score tier, every failure
and adversarial histories, step by step with hand-made object states (no simulator).

    PYTHONPATH=<yam_arena stubs> python -m pytest -q test_task.py      (or: python test_task.py)
"""
import math
import os

import numpy as np
from yam_arena.demo import grip_rotation
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit, dk, mn, gk = m.kit, m.dk, m.mn, m.gk
FO = 0.0475  # the stub's FINGER_OPEN (inner face 47.5 mm from the tool point fully open)

PARK = {"left_gripper": np.array([0.15, 0.30, m.TOP + 0.30]), "right_gripper": np.array([0.15, -0.30, m.TOP + 0.30])}
I4 = np.array([0.0, 0.0, 0.0, 1.0])


def st(pos, quat=I4, half=0.01):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - half, high=p + half)


def rot_axis(u, a):
    u = np.asarray(u, float) / np.linalg.norm(u)
    K = np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


def bottle_pose_at(tip_w, d, a):
    """Bottle origin and quaternion with its tip end at tip_w, pointing d, held from approach a."""
    zb = -np.asarray(a, float)
    yb = np.cross(zb, d)
    Rb = np.column_stack([d, yb, zb])
    return np.asarray(tip_w, float) - (m.B_TIP - m.TIP_R) * np.asarray(d, float), m._quat(Rb)


REST_POSE = (np.array(m.BOTTLE_START), I4)


class Trial:
    """One trial: call step(...) once per control step with the state of that step."""

    def __init__(self):
        kit._EPISODE["count"] += 1
        kit._EPISODE["last"] = None
        self.out = None

    def step(self, bottle=REST_POSE, squeeze_mm=0.0, lid_mm=0.0, head_rot=None, grippers=None, openings=None,
             n=1):
        for _ in range(n):
            Rh = m.R_H if head_rot is None else head_rot @ m.R_H
            head_pos = m.PIV_W if head_rot is None else m.PIV_W  # the hinges pass through the pivot
            for k, v in (("manikin/lid", -lid_mm / 1000.0), ("manikin/head", 0.0), ("manikin/tilt", 0.0),
                         ("bottle/pad_l", -squeeze_mm / 1000.0), ("bottle/pad_r", squeeze_mm / 1000.0),
                         ("bottle/pad_t", 0.0), ("bottle/pad_b", 0.0)):
                kit._JOINTS[k] = kit.JointState(v, 0.0, False)
            g = dict(PARK, **(grippers or {}))
            o = {"left_gripper": 1.0, "right_gripper": 1.0}
            o.update(openings or {})
            kit._OPENING.update(o)
            objs = {"manikin": st(m.PIV_W, m.Q_H), "manikin/tilt": st(m.PIV_W, m.Q_H),
                    "manikin/head": st(head_pos, m._quat(Rh)), "manikin/lid": st(m.PIV_W, m._quat(Rh)),
                    "bottle": st(bottle[0], bottle[1]), "tray": st((*m.TRAY_C, m.TOP)), "headrest": st((0.2, 0.04, 0)),
                    "bottle_rest": st((*m.TRAY_C, m.TRAY_TOP)), "cap": st((0.3, -0.1, m.TRAY_TOP))}
            for k, v in g.items():
                objs[k] = v if isinstance(v, ObjectState) else st(v)
            ok = m.success(objs)
            self.out = (m.score(objs), ok)
        return self.out


# -- the demo's aim, used by most histories ------------------------------------------------------------------------------
GRIP_W, R_EYE, D_EYE, A_EYE = m._bottle_hand_at_eye()
TIP_POCKET = m.HEAD_C + m.R_H @ m.POCKET_H + np.array([0, 0, m.TIP_ABOVE])
AT_POCKET = bottle_pose_at(TIP_POCKET, D_EYE, A_EYE)
TIP_EYE = m.EYE_W + np.array([0, 0, m.TIP_ABOVE])
AT_EYE = bottle_pose_at(TIP_EYE, D_EYE, A_EYE)


def squeeze_profile(t, at, mm=2.5, steps=20, lid=7.0, hold_mm=0.5):
    """Hold, ramp to `mm` over 8 steps, hold `steps`, ease off over 8 steps (as the demo does)."""
    t.step(at, hold_mm, lid, n=5)
    for k in range(1, 9):
        t.step(at, hold_mm + (mm - hold_mm) * k / 8, lid)
    t.step(at, mm, lid, n=steps)
    for k in range(7, -1, -1):
        t.step(at, hold_mm + (mm - hold_mm) * k / 8, lid)
    return t


def put_back(t, n=25, lid=0.0, grippers=None, openings=None):
    return t.step(REST_POSE, 0.0, lid, grippers=grippers, openings=openings, n=n)


def full_trial(**kw):
    t = Trial()
    t.step(n=10)
    squeeze_profile(t, AT_POCKET, **kw)
    return t


# -- dropkit ------------------------------------------------------------------------------------------------------------
def test_firm_hold_dispenses_nothing():
    t = Trial()
    t.step(AT_POCKET, 1.9, 7.0, n=200)  # 3.8 N for 10 s, tip down: below the 4 N opening force
    assert dk.drops() == [] and dk.hanging() == 0.0


def test_one_drop_needs_long_enough_squeeze():
    t = Trial()
    t.step(AT_POCKET, 2.5, 7.0, n=15)  # 5 N for 0.75 s: 37.5 uL hanging, not yet a drop
    assert dk.drops() == [] and abs(dk.hanging() - 37.5) < 1e-6
    t.step(AT_POCKET, 0.5, 7.0)  # easing off sucks it back
    assert dk.hanging() == 0.0 and dk.drops() == []
    t.step(AT_POCKET, 2.5, 7.0, n=16)
    assert len(dk.drops()) == 1


def test_harder_or_longer_gives_more_drops():
    t = Trial()
    t.step(AT_POCKET, 4.0, 7.0, n=20)  # 8 N for 1 s: 200 uL = 5 drops
    assert len(dk.drops()) == 5
    t = Trial()
    t.step(AT_POCKET, 2.5, 7.0, n=40)  # 5 N for 2 s: 2 drops
    assert len(dk.drops()) == 2


def test_tip_angle_gate():
    for below_deg, want in ((25.0, 0), (35.0, 1)):
        a = math.radians(below_deg)
        d = np.array([math.cos(a), 0.0, -math.sin(a)])
        av = np.array([0.0, 0.0, -1.0]) - math.sin(a) * d
        pose = bottle_pose_at(TIP_POCKET, d, av / np.linalg.norm(av))
        t = Trial()
        t.step(pose, 2.5, 7.0, n=17)
        assert len(dk.drops()) == want, below_deg


def test_either_pad_pair_squeezes():
    """A grasp across the top and bottom pads (the bottle turned 90 degrees in the hand) squeezes it too; one pad
    pushed alone (the bottle pressed against something) gives half the travel."""
    t = Trial()
    t.step(AT_POCKET, 0.0, 7.0)
    for _ in range(17):
        kit._JOINTS["bottle/pad_t"] = kit.JointState(-0.0025, 0.0, False)
        kit._JOINTS["bottle/pad_b"] = kit.JointState(0.0025, 0.0, False)
        o = _objs(7.0, AT_POCKET)
        assert abs(dk.squeeze_force(o) - 5.0) < 1e-9
        dk.update(o)
    assert len(dk.drops()) == 1
    kit._JOINTS["bottle/pad_t"] = kit.JointState(-0.0025, 0.0, False)
    kit._JOINTS["bottle/pad_b"] = kit.JointState(0.0, 0.0, False)
    assert abs(dk.squeeze_force(_objs(7.0, AT_POCKET)) - 2.5) < 1e-9


def test_tip_up_squeeze_moves_air_only():
    t = Trial()
    d = np.array([0.0, 0.0, 1.0])
    t.step(bottle_pose_at(TIP_POCKET + np.array([0, 0, 0.08]), d, np.array([1.0, 0, 0])), 4.5, 7.0, n=60)
    assert dk.drops() == []


def test_squeeze_on_the_tray_lying_flat_gives_nothing():
    t = Trial()
    assert t.step(REST_POSE, 4.5, 0.0, n=60) == (0.0, False)
    assert dk.drops() == []


def test_capacity_limits_total():
    spec = dk.Bottle("bottle", pads=m.BOTTLE_SPEC.pads, tip=m.BOTTLE_SPEC.tip, capacity_ul=100.0)
    dk.setup(spec, m.land)
    try:
        t = Trial()
        t.step(AT_POCKET, 5.0, 7.0, n=100)
        assert len(dk.drops()) == 2  # 100 uL: two whole drops
    finally:
        dk.setup(m.BOTTLE_SPEC, m.land)


# -- head geometry and the demo's aim -------------------------------------------------------------------------------------
def test_head_real_size():
    p = m._HEAD_PTS
    span = p.max(0) - p.min(0)
    assert 0.20 <= span[0] <= 0.215  # front to back incl. nose
    assert abs(span[1] - 0.150) < 1e-3  # breadth 15 cm
    assert 0.23 <= span[2] <= 0.24  # chin to crown
    assert abs(2 * mn.EYE_Y - 0.064) < 1e-9 and 2 * mn.EYE_HALF_W == 0.030 and 2 * mn.EYE_HALF_H == 0.010


def test_colliders_inside_skin_and_clear_of_the_pad():
    for lo, hi in m.HEAD_BOXES:
        g = np.array([[a, b, c] for a in np.linspace(lo[0], hi[0], 6) for b in np.linspace(lo[1], hi[1], 6)
                      for c in np.linspace(lo[2], hi[2], 6)])
        assert float(np.max(mn.head_sdf(g))) <= 0.0006, (lo, hi)
    world = (m.HEAD_C + np.array([[x, y, z] for lo, hi in m.HEAD_BOXES for x in (lo[0], hi[0]) for y in (lo[1], hi[1])
                                  for z in (lo[2], hi[2])]) @ m.R_H.T)
    assert world[:, 2].min() > m.RING_TOP  # the head never rests on the pad in the sim (the neck holds it)
    assert m.HEAD_LOW_Z - m.RING_TOP == 0.003 or abs(m.HEAD_LOW_Z - m.RING_TOP - 0.003) < 1e-9
    assert abs(m.RING_TOP - m.TOP - 0.010) < 0.0005  # the 1 cm gel pad


def test_demo_aim_lands_in_pocket_and_eye():
    zone, at, info = m.land(_objs(lid_mm=7.0), TIP_POCKET)
    assert zone == "pocket" and abs(info["height"] - m.TIP_ABOVE) < 1e-3
    zone, at, info = m.land(_objs(lid_mm=0.0), TIP_POCKET)
    assert zone == "face"  # the same aim with the lid closed lands on the lid margin
    zone, at, info = m.land(_objs(lid_mm=0.0), TIP_EYE)
    assert zone == "eye"
    zone, _, _ = m.land(_objs(lid_mm=2.0), TIP_POCKET)
    assert zone in ("face", "eye")  # 2 mm: not a pocket
    zone, _, _ = m.land(_objs(), m.HEAD_C + m.R_H @ (mn.eye_center(1) + np.array([0.01, 0, 0])) + [0, 0, 0.02])
    assert zone == "face"  # the left eye is not the right eye


def _objs(lid_mm=0.0, bottle=None):
    kit._JOINTS["manikin/lid"] = kit.JointState(-lid_mm / 1000, 0.0, False)
    o = {"manikin/head": st(m.PIV_W, m.Q_H), "bottle": st(*(bottle or REST_POSE))}
    o.update({k: st(v) for k, v in PARK.items()})
    kit._OPENING.update({"left_gripper": 1.0, "right_gripper": 1.0})
    return o


def test_demo_bottle_pose_is_clear_of_skin():
    o = _objs(7.0, AT_POCKET)
    assert not m.tip_touches_skin(o)
    assert m.lid_pulled(o) == 0.007


def test_decompose_matches_tilted():
    rng = np.random.default_rng(3)
    for _ in range(50):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        R = gk._rot(q)
        yaw, roll, pitch = m._decompose(R)
        G = np.asarray(grip_rotation(yaw), float)
        assert np.allclose(kit._rot_xyz(roll, pitch) @ G, R, atol=1e-9)


def test_demo_plan_shape():
    plan = m.demo()
    for arm in ("left", "right"):
        assert sum(w.duration for w in plan[arm]) <= m.RUN_LIMIT_STEPS * m.DT
        for w in plan[arm]:
            assert 0.0 <= w.finger * m.F_SIGN <= abs(m.FINGER_OPEN)
    r = plan["right"]
    labels = [w.label for w in r]
    i = labels.index("hold the squeeze: one drop")
    pad = m.K_F / (m.K_F + m.WALL_K)  # the finger servo and the wall spring in series
    assert r[i].finger == m.F_SIGN * m.FINGER_SQUEEZE
    assert (m.B_R - m.FINGER_SQUEEZE) * pad > m.PAD_TRAVEL  # the squeeze takes the walls to their stop
    assert abs((m.B_R - m.FINGER_HOLD) * pad * m.WALL_K - 1.0) < 1e-9  # the hold squeezes 1 N, under the 4 N opening
    # the lid is held from before the squeeze until after the ease-off
    tl, t_hold_end = 0.0, None
    for w in plan["left"]:
        tl += w.duration
        if w.label.startswith("hold the lid"):
            t_hold_end = tl
    t_ease = sum(w.duration for w in r[:labels.index("ease off") + 1])
    assert t_hold_end >= t_ease



def test_demo_squeeze_gives_one_drop_with_a_step_to_spare():
    """The demo's press, sampled at the control rate with any phase and one step longer or shorter, releases exactly
    one drop: the walls' force from the finger (servo and wall in series, capped at the walls' stop)."""
    from yam_arena.demo import ArmPlan
    from yam_arena.station import BASES

    plan = ArmPlan(m.demo()["right"], BASES["right"])
    labels = [w.label for w in plan.waypoints]
    t0 = sum(w.duration for w in plan.waypoints[:labels.index("squeeze")])
    t1 = sum(w.duration for w in plan.waypoints[:labels.index("ease off") + 1])
    pad = m.K_F / (m.K_F + m.WALL_K)
    spec = m.BOTTLE_SPEC

    def force(t):
        depth = m.B_R - plan.target(t)[2] * m.F_SIGN
        return m.WALL_K * min(max(0.0, depth * pad), m.PAD_TRAVEL)

    for phase in (0.0, 0.0125, 0.025, 0.0375):
        for extra in (-1, 0, 1):
            fs = [force(t) for t in np.arange(t0 - 0.2 + phase, t1 + 0.2, m.DT)]
            k = max(range(len(fs)), key=lambda j: fs[j])
            fs = fs[:k] + [fs[k]] * max(0, 1 + extra) + fs[k + 1:]
            ul = sum(spec.flow * (f - spec.open_force) * m.DT for f in fs if f >= spec.open_force)
            assert spec.drop_ul <= ul < 2 * spec.drop_ul, (phase, extra, ul)

# -- score tiers ---------------------------------------------------------------------------------------------------------
def test_do_nothing_scores_zero():
    t = Trial()
    assert t.step(n=60) == (0.0, False)


def test_full_score():
    t = full_trial()
    assert [d.zone for d in dk.drops()] == ["pocket"]
    assert put_back(t, n=19)[0] == 0.75  # 19 still steps (the first has no previous position): not yet 1 s
    assert put_back(t, n=2) == (1.0, True)


def test_bottle_still_held_at_end_is_075():
    t = full_trial()
    grip = np.array(m.BOTTLE_START) + np.array([m.GRIP_X, 0, 0.004])
    q = m._quat(m._frame((0, 0, -1), (0, 1, 0)))
    s = put_back(t, n=30, grippers={"right_gripper": st(grip, q)}, openings={"right_gripper": m.B_R / FO})
    assert s == (0.75, False)
    assert put_back(t, n=21) == (1.0, True)  # let go: full credit once still for 1 s


def test_lid_still_pulled_at_end_is_075():
    t = full_trial()
    assert put_back(t, n=30, lid=3.0) == (0.75, False)
    assert put_back(t, n=21, lid=0.8) == (1.0, True)  # within 1 mm of closed counts as let go


def test_bottle_moving_on_tray_not_full():
    t = full_trial()
    for k in range(30):
        p = np.array(m.BOTTLE_START) + np.array([0.002 * (k % 2), 0, 0])
        t.step((p, I4))
    assert t.out == (0.75, False)


def test_bottle_left_off_the_tray_but_not_dropped():
    t = full_trial()
    p = np.array([0.15, -0.25, m.TOP + 0.15])  # still held up in the air, away from the tray
    assert t.step((p, I4), n=30) == (0.75, False)


def test_drop_in_eye_without_lid_is_05():
    t = Trial()
    squeeze_profile(t, AT_EYE, lid=0.0)
    assert [d.zone for d in dk.drops()] == ["eye"]
    assert put_back(t, n=30)[0] == 0.5


def test_lid_drawn_under_3mm_is_not_a_pocket():
    t = Trial()
    squeeze_profile(t, AT_EYE, lid=2.0)
    assert dk.drops()[0].zone == "eye"
    assert put_back(t, n=30)[0] == 0.5


def test_two_drops_cap_05():
    t = Trial()
    squeeze_profile(t, AT_POCKET, steps=40)  # held 2 s: two drops
    assert len(dk.drops()) == 2 and dk.drops()[0].zone == "pocket"
    assert put_back(t, n=30)[0] == 0.5


def test_many_drops_cap_025():
    t = Trial()
    squeeze_profile(t, AT_POCKET, mm=4.0, steps=20)  # a hard squeeze: several drops
    assert len(dk.drops()) >= 3
    assert put_back(t, n=30)[0] == 0.25


def test_too_high_is_025():
    t = Trial()
    high = bottle_pose_at(TIP_POCKET + np.array([0, 0, 0.03]), D_EYE, A_EYE)  # 5 cm above
    squeeze_profile(t, high)
    assert dk.drops()[0].zone == "pocket" and dk.drops()[0].info["height"] > m.MAX_TIP_H
    assert put_back(t, n=30)[0] == 0.25


def test_three_cm_is_allowed():
    t = Trial()
    at = bottle_pose_at(TIP_POCKET + np.array([0, 0, 0.0095]), D_EYE, A_EYE)  # 2.95 cm above
    squeeze_profile(t, at)
    assert put_back(t, n=30)[0] == 1.0


def test_second_drop_lands_on_cheek_after_first_in_eye():
    t = Trial()
    squeeze_profile(t, AT_POCKET)
    cheek = bottle_pose_at(TIP_POCKET - 0.03 * m.U_W, D_EYE, A_EYE)
    squeeze_profile(t, cheek)
    assert [d.zone for d in dk.drops()] == ["pocket", "face"]
    assert put_back(t, n=30)[0] == 0.5


def test_first_drop_misses_then_hits_is_025():
    t = Trial()
    cheek = bottle_pose_at(TIP_POCKET - 0.03 * m.U_W, D_EYE, A_EYE)
    squeeze_profile(t, cheek)
    squeeze_profile(t, AT_POCKET)
    assert [d.zone for d in dk.drops()] == ["face", "pocket"]
    assert put_back(t, n=30)[0] == 0.25


def test_miss_scores_zero():
    t = Trial()
    squeeze_profile(t, bottle_pose_at(TIP_POCKET - 0.03 * m.U_W, D_EYE, A_EYE))
    assert put_back(t, n=30)[0] == 0.0


def test_wrong_eye_scores_zero():
    t = Trial()
    left_eye = m.HEAD_C + m.R_H @ mn.eye_center(1) + np.array([0, 0, 0.02])
    squeeze_profile(t, bottle_pose_at(left_eye, D_EYE, A_EYE))
    assert dk.drops()[0].zone == "face"
    assert put_back(t, n=30)[0] == 0.0


def test_drop_on_tray_or_table_scores_zero():
    t = Trial()
    over_table = bottle_pose_at(np.array([0.30, 0.20, m.TOP + 0.10]), D_EYE, A_EYE)
    squeeze_profile(t, over_table)
    assert dk.drops()[0].zone == "table"
    assert put_back(t, n=30)[0] == 0.0


def test_drop_caught_by_a_hand_scores_zero():
    t = Trial()
    under = m.HEAD_C + m.R_H @ m.POCKET_H + np.array([0, 0, 0.010])  # a fingertip between tip and eye
    q = m._quat(m._frame((0, 0, -1), (0, 1, 0)))
    t.step(AT_POCKET, 0.5, 7.0, n=3)
    for _ in range(25):
        t.step(AT_POCKET, 2.5, 7.0, grippers={"left_gripper": st(under + np.array([0, 0, 0.005]), q)},
               openings={"left_gripper": 0.0})
    assert dk.drops() and dk.drops()[0].zone == "hand"


# -- latched failures ------------------------------------------------------------------------------------------------------
def test_tip_touching_skin_latches_zero():
    t = Trial()
    touch = bottle_pose_at(m.HEAD_C + m.R_H @ m.POCKET_H + np.array([0, 0, 0.0012]), D_EYE, A_EYE)
    t.step(touch, 0.5, 7.0)
    assert m._L.touched
    squeeze_profile(t, AT_POCKET)
    assert put_back(t, n=30) == (0.0, False)


def test_tip_on_the_cheek_or_brow_latches():
    for where in (-0.03 * m.U_W, 0.03 * m.U_W):
        t = Trial()
        p_w = m.HEAD_C + m.R_H @ m.POCKET_H + where
        hit = mn.ray_hit(m.R_H.T @ (p_w + np.array([0, 0, 0.05]) - m.HEAD_C), m.R_H.T @ np.array([0, 0, -1.0]))
        tip = m.HEAD_C + m.R_H @ hit + np.array([0, 0, 0.001])
        t.step(bottle_pose_at(tip, D_EYE, A_EYE), 0.5, 7.0)
        assert m._L.touched, where


def test_tip_2mm_clear_is_not_a_touch():
    t = Trial()
    clear = bottle_pose_at(m.HEAD_C + m.R_H @ m.POCKET_H + np.array([0, 0, 0.0035]), D_EYE, A_EYE)
    t.step(clear, 0.5, 7.0)
    assert not m._L.touched


def test_hand_on_eye_latches_zero():
    t = Trial()
    eye = m.EYE_W
    a = -m.F_W
    q = m._quat(m._frame(a, m.U_W))
    tcp = eye - 0.0128 * a + 0.001 * a  # fingertips 1 mm into the eye opening's surface
    t.step(grippers={"left_gripper": st(tcp, q)}, openings={"left_gripper": 0.0})
    assert m._L.poked
    squeeze_profile(t, AT_POCKET)
    assert put_back(t, n=30)[0] == 0.0


def test_fist_on_cheek_pad_is_allowed():
    t = Trial()
    pad = m.HEAD_C + m.R_H @ m.LID_PAD_H
    a = -m.F_W + np.array([0, -0.8, 0])
    a /= np.linalg.norm(a)
    q = m._quat(m._frame(a, m.U_W))
    fist = {"left_gripper": st(pad - 0.0128 * a + 0.002 * a - 0.007 * m.U_W, q)}
    t.step(n=3, grippers=fist, openings={"left_gripper": 0.0}, lid_mm=7.0)
    assert not m._L.poked
    squeeze_profile(t, AT_POCKET)
    assert put_back(t, n=30)[0] == 1.0


def test_strike_latches_zero():
    t = Trial()
    nod = rot_axis(m.L_W, 0.055)  # nodded 0.055 rad about the neck: the eye moves about 9.8 mm
    piv = m.PIV_W
    shift = 1000 * np.linalg.norm(piv + nod @ (m.EYE_W - piv) - m.EYE_W)
    assert shift > m.JAB_MM
    t.step(head_rot=nod)
    assert m._L.strike
    squeeze_profile(t, AT_POCKET)
    assert put_back(t, n=30)[0] == 0.0


def test_strike_needs_two_steps_at_6mm():
    t = Trial()
    nod = rot_axis(m.L_W, 0.039)  # about 7 mm
    t.step(head_rot=nod)
    assert not m._L.strike
    t.step()
    t.step(head_rot=nod)
    assert not m._L.strike
    t.step(head_rot=nod)
    assert m._L.strike


def test_small_head_motion_is_fine():
    t = Trial()
    nod = rot_axis(m.L_W, 0.015)  # under 3 mm at the eye: the lid hand's light press
    t.step(head_rot=nod, n=10)
    assert not m._L.strike


def test_bottle_dropped_on_table_latches_zero():
    t = full_trial()
    t.step((np.array([0.30, -0.25, m.TOP + 0.0125]), I4), n=3)
    assert m._L.fell
    assert put_back(t, n=30)[0] == 0.0


def test_bottle_laid_on_table_by_hand_can_be_put_back():
    t = full_trial()
    q = m._quat(m._frame((0, 0, -1), (0, 1, 0)))
    held = {"right_gripper": m.B_R / FO}
    start, end = np.array([0.30, -0.25, m.TOP + 0.08]), np.array([0.30, -0.25, m.TOP + 0.0125])
    for k in range(21):  # carried down to the table in 1 s, held the whole way
        p = start + (end - start) * k / 20
        t.step((p, I4), grippers={"right_gripper": st(p + np.array([m.GRIP_X, 0, 0.004]), q)}, openings=held)
    assert t.step((end, I4), n=10) == (0.75, False)  # let go on the table: not on the tray, not a fall
    assert not m._L.fell
    assert put_back(t, n=21) == (1.0, True)  # picked up again and put on the tray


def test_slow_slide_off_the_tray_edge_is_not_a_fall():
    t = full_trial()
    put_back(t, n=2)  # on the rest
    for k in range(10):  # unheld, sinking 2 cm per second: well under a free fall
        t.step((np.array([0.30, -0.25, m.TOP + 0.0325 - 0.001 * 2 * k]), I4))
    assert not m._L.fell


def test_strike_constants():
    assert abs(m.STRIKE_MM / m.NOD_MM_PER_N - 4.0) < 1e-9 and abs(m.JAB_MM / m.NOD_MM_PER_N - 6.0) < 1e-9
    # a 1 N push straight into the face at the eye moves it 1.5 mm through the nod hinge
    r = m.mn.eye_center(-1) - m.PIV_H
    torque = r[2] * 1.0
    assert abs(torque / m.K_NOD * math.hypot(r[0], r[2]) * 1000 - 1.5) < 1e-6


def test_new_trial_resets():
    full_trial()
    put_back(Trial(), n=1)
    assert dk.drops() == [] and not m.failed()


def test_demo_fist_stays_off_the_eyes():
    """The demo's lid hand at its hold pose (pad tips 12.8 mm past the tool point, as guardkit's hand model) touches
    neither eye, and the bottle hand at the squeeze pose clears the skin."""
    keep = m.PAD_TIP_PAST_TCP
    m.PAD_TIP_PAST_TCP = 0.0128
    try:
        plan = m.demo()
    finally:
        m.PAD_TIP_PAST_TCP = keep
    wl = [w for w in plan["left"] if w.label.startswith("hold the lid")][0]
    wr = [w for w in plan["right"] if w.label.startswith("hold the squeeze")][0]
    def q_of(w):
        return m._quat(kit._rot_xyz(w.roll, w.pitch) @ np.asarray(grip_rotation(w.yaw), float))
    o = _objs(7.0, AT_POCKET)
    o["left_gripper"] = st(wl.pos, q_of(wl))
    o["right_gripper"] = st(wr.pos, q_of(wr))
    kit._OPENING.update({"left_gripper": 0.0, "right_gripper": m.FINGER_SQUEEZE / FO})
    assert not m.hand_on_eye(o)
    assert m.gk.hand_gap(o, "left_gripper", "manikin/head", m.EYE_PARTS, upto=0.05) > 0.004
    assert not m.tip_touches_skin(o)
    zone, _, _ = m.land(o, TIP_POCKET)
    assert zone == "pocket"  # neither hand is under the falling drop


if __name__ == "__main__":
    import sys

    import pytest

    sys.exit(pytest.main([__file__, "-q"]))
