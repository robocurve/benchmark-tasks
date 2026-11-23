"""Scorer tests (no simulator): the pure ladder() on hand-made measurements, with negative controls.

    python -m pytest -q test_scoring.py      (or: python test_scoring.py)
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
os.environ.setdefault("TRIAL", "0")
import parts as P  # noqa: E402
import task as T  # noqa: E402


def meas(slot=None, cassette=None, dz=0.0, tilt_deg=0.0, grip=0.0, tips=0.08, in_stand=True, robot=0.1, nb=0.0,
         vz=0.0, speed=0.0, low=None, edge_slots=None, touch=0.0, pos=None, depth=0.0):
    """Measurements for a wafer standing in `slot` of `cassette` (default: the target), dz above its seat."""
    cassette = T.FAR_XY if cassette is None else cassette
    slot = T.DST if slot is None else slot
    x, y = T.slot_xy(cassette, slot)
    pos = np.array([x, y, T.SEAT_Z + dz]) if pos is None else np.asarray(pos, float)
    ys = [y, y] if edge_slots is None else [cassette[1] + P.slot_y(k) for k in edge_slots]
    return dict(wafer_pos=pos, wafer_vz=vz, wafer_speed=speed,
                wafer_low=pos[2] - P.WAFER_R if low is None else low, edge_y=tuple(ys),
                edge_x=(x + P.WAFER_R, x - P.WAFER_R), upright_deg=tilt_deg, nb_moved=nb, grip_N=grip,
                tips_to_wafer=tips, tw_in_stand=in_stand, robot_to_tweezers=robot, robot_touch_N=touch,
                tip_depth=depth)


def fresh():
    s: dict = {}
    s.update(was_held=False, parted_pos=None, checked=False, dropped=False, displaced=False, table=False,
             robot=False, face=False, max_tip_depth=0.0, entered=False, free_steps=0, events=[])
    return s


def run(seq, s=None):
    s = fresh() if s is None else s
    out = None
    for m in seq:
        out = T.ladder(m, s)
    return out, s


def settled(**kw):
    return [meas(**kw)] * 12  # 12 steps free of the tweezers (resting needs 10)


def test_full_score():
    assert run(settled())[0]["score"] == 1.0


def test_tweezers_not_back_in_stand():
    assert run(settled(in_stand=False))[0]["score"] == 0.75


def test_robot_still_touching_tweezers():
    assert run(settled(robot=0.01))[0]["score"] == 0.75


def test_in_slot_not_fully_down():
    assert run(settled(dz=0.005))[0]["score"] == 0.5


def test_held_wafer_scores_zero():  # negative control: still in the tweezers, even when seated
    assert run([meas(grip=0.5)] * 12)[0]["score"] == 0.0


def test_tips_not_clear_scores_zero():
    assert run(settled(tips=0.03))[0]["score"] == 0.0


def test_wrong_slot_scores_zero():  # negative control
    assert run(settled(slot=T.DST + 2))[0]["score"] == 0.0
    assert run(settled(cassette=T.NEAR_XY, slot=T.SRC))[0]["score"] == 0.0  # put back where it came from


def test_cross_slotted_scores_zero():
    assert run(settled(edge_slots=(T.DST, T.DST + 1)))[0]["score"] == 0.0


def test_tilted_scores_zero():
    assert run(settled(tilt_deg=8.0))[0]["score"] == 0.0


def test_neighbour_displaced_latches_zero():  # negative control
    out, s = run([meas(nb=0.003)] + settled())
    assert out["score"] == 0.0 and s["displaced"]


def test_dropped_wafer_latches_zero():  # negative control: let go 20 mm above its slot, then it falls in
    out, s = run([meas(grip=0.5, dz=0.02)] + [meas(dz=0.02)] * 3 + settled())
    assert out["score"] == 0.0 and s["dropped"]


def test_let_go_outside_a_cassette_latches_zero():
    out, s = run([meas(grip=0.5, pos=(0.26, 0.30, T.TOP + 0.10))] + [meas(pos=(0.26, 0.30, T.TOP + 0.10))] * 3)
    assert out["score"] == 0.0 and s["dropped"]


def test_released_resting_in_its_slot_is_not_a_drop():
    out, s = run([meas(grip=0.5)] * 5 + settled())
    assert out["score"] == 1.0 and not s["dropped"]


def test_release_within_3_mm_is_not_a_drop():
    out, s = run([meas(grip=0.5, dz=0.0025)] + [meas(dz=0.0025)] * 3 + settled())
    assert out["score"] == 1.0 and not s["dropped"]


def test_contact_flicker_while_carried_is_not_a_drop():  # free for 2 steps mid-carry, then held again
    high = dict(dz=0.05)
    seq = [meas(grip=0.5, **high), meas(**high), meas(**high), meas(grip=0.5, **high), meas(grip=0.5)] + settled()
    out, s = run(seq)
    assert out["score"] == 1.0 and not s["dropped"]


def test_robot_touching_a_wafer_latches_zero():  # negative control: fingers or arm on any wafer
    out, s = run([meas(touch=0.01)] + settled())
    assert out["score"] == 0.0 and s["robot"]


def test_tip_on_face_inside_edge_zone_latches_zero():  # negative control: tips pressed 8 mm into the face
    out, s = run([meas(grip=0.5, depth=0.008), meas(grip=0.5)] + settled())
    assert out["score"] == 0.0 and s["face"]


def test_tip_within_edge_zone_is_allowed():  # a grip 5 mm below the top edge (the demo grips 4 mm below)
    out, s = run([meas(grip=0.5, depth=0.005)] * 3 + settled())
    assert out["score"] == 1.0 and not s["face"]


def test_table_touch_latches_zero():
    out, s = run([meas(low=T.TOP + 0.0005)] + settled())
    assert out["score"] == 0.0 and s["table"]


def test_start_state_scores_zero():  # the untouched start: wafer in its source slot, tweezers in the stand
    assert run(settled(cassette=T.NEAR_XY, slot=T.SRC))[0]["score"] == 0.0


# -- known poses: the quaternion order (x, y, z, w, measured in the stack; README "Quaternion order") ---------------

S45 = math.sin(math.pi / 4)
Q_X90 = (S45, 0.0, 0.0, S45)  # 90 deg about x
Q_Z90 = (0.0, 0.0, S45, S45)  # 90 deg about z: how the tweezers are spawned
Q_PROBE = (0.683, 0.183, 0.183, 0.683)  # read back in the simulator for a part spawned 90 deg about x, then 30 about z
Q_TW_MEASURED = (-0.0045, 0.0045, 0.707, 0.7071)  # tweezer leaf A read back while standing in the stand


def close(a, b, tol=1e-3):
    return np.allclose(np.asarray(a, float), np.asarray(b, float), atol=tol)


def test_quaternion_order_is_xyzw():
    R = T.quat_xyzw_to_matrix(Q_X90)
    assert close(R @ [0, 1, 0], [0, 0, 1]) and close(R @ [0, 0, 1], [0, -1, 0])
    # read the other way round (w first), the same numbers would be a different rotation
    assert not close(T.quat_xyzw_to_matrix((Q_X90[1], Q_X90[2], Q_X90[3], Q_X90[0])) @ [0, 1, 0], [0, 0, 1])


def test_measured_probe_pose_gives_the_spawned_axes():
    R = T.quat_xyzw_to_matrix(Q_PROBE)
    c30, s30 = math.cos(math.pi / 6), math.sin(math.pi / 6)
    assert close(R[:, 0], [c30, s30, 0], 2e-3) and close(R[:, 1], [0, 0, 1], 2e-3) and close(R[:, 2], [s30, -c30, 0], 2e-3)


def test_wafer_orientation_known_poses():
    assert abs(T.upright_deg(T.wafer_normal((0, 0, 0, 1)))) < 1e-6  # standing in a slot (faces normal to y)
    assert abs(T.upright_deg(T.wafer_normal(Q_X90)) - 90) < 1e-3  # lying flat
    q30 = (0.0, 0.0, math.sin(math.pi / 12), math.cos(math.pi / 12))  # turned 30 deg about z
    assert abs(T.upright_deg(T.wafer_normal(q30)) - 30) < 1e-3
    q4 = (0.0, 0.0, math.sin(math.radians(2)), math.cos(math.radians(2)))  # 4 deg: still counts as upright (< 5)
    assert T.upright_deg(T.wafer_normal(q4)) < T.UPRIGHT_DEG


def test_tweezer_tip_positions_known_pose():
    hinge = np.array([T.STAND_XY[0], T.STAND_XY[1], T.TOP + P.TW_LEN + 0.0005])
    off = P.TW_HINGE_GAP / 2 + P.TIP_T / 2
    # leaf A spawned 90 deg about z: its local -x (away from leaf B) points along world -y, its length along world -z
    assert close(T.tip_point(hinge, Q_Z90, "half_a"), hinge + [0.0, -off, -P.TW_LEN], 1e-6)
    tb = T.tip_point(hinge, Q_Z90, "half_b")  # leaf B is drawn sprung open: its tip sits on the other side
    assert abs(tb[0] - hinge[0]) < 1e-6 and tb[1] > hinge[1] and abs(tb[2] - (hinge[2] - P.TW_LEN)) < 2e-4
    gap = tb[1] - T.tip_point(hinge, Q_Z90, "half_a")[1] - P.TIP_T
    assert abs(gap - P.TW_OPEN_GAP) < 2e-4  # the sprung-open tip gap
    # the measured leaf pose standing in the stand: tip at the table, within 4 mm of the stand's centre line (1.85 mm
    # of it is the leaf's offset from the hinge's mid-plane; the slot is 16 x 18 mm)
    ta = T.tip_point(hinge, Q_TW_MEASURED, "half_a")
    assert abs(ta[2] - (T.TOP + 0.0005)) < 1e-3 and np.hypot(ta[0] - hinge[0], ta[1] - hinge[1]) < 0.004


def test_upright_stand_detection_known_poses():
    mid = np.array([T.STAND_XY[0], T.STAND_XY[1], T.TOP + 0.0005])
    assert T.in_stand(mid, T.tweezer_axis(Q_Z90), True)
    assert T.in_stand(mid, T.tweezer_axis(Q_TW_MEASURED), True)
    tilt20 = (math.sin(math.radians(10)), 0.0, 0.0, math.cos(math.radians(10)))  # leaning 20 deg
    assert not T.in_stand(mid, T.tweezer_axis(tilt20), True)
    assert not T.in_stand(mid, T.tweezer_axis(Q_X90), True)  # lying on its side
    assert not T.in_stand(mid, T.tweezer_axis(Q_Z90), False)  # still moving
    assert not T.in_stand(mid + [0.02, 0, 0], T.tweezer_axis(Q_Z90), True)  # tips 2 cm off the slot
    assert not T.in_stand(mid + [0, 0, 0.02], T.tweezer_axis(Q_Z90), True)  # held 2 cm up, above the stand's top


def test_layouts_rotate_and_differ():
    srcs, dsts = zip(*T.LAYOUTS.values())
    assert len(set(srcs)) == 3 and len(set(dsts)) == 3 and all(s != d for s, d in T.LAYOUTS.values())


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print("ok  ", name)
            except AssertionError:
                fails += 1
                print("FAIL", name)
    print(f"{fails} failed")
    sys.exit(1 if fails else 0)
