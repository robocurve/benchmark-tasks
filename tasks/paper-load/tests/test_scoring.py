"""Scoring checks without the simulator: task.py's monitor (advance()) fed hand-built trials: the paper stack's strip
poses, the guide's travel, the paper support's angle, the keys and the grippers' gap (from their collision hulls,
placed by yam_arena's IK and forward kinematics), step by step at 20 Hz. NumPy and SciPy only.

    python tests/test_scoring.py          # or: pytest tests/
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import geometry as g  # noqa: E402
import task  # noqa: E402
from yam_arena import analysis, ik, station  # noqa: E402
from yam_arena.constants import ARM_JOINTS, READY  # noqa: E402

SNUG = -(g.GUIDE_OPEN_Y - (g.RIGHT_Y + g.SHEET_L))       # guide travel that brings its face to the stack's left edge
LOADED = g.loaded_strip_poses()


def on_cover():
    return [(g.start_strip_pos(i), np.eye(3)) for i in range(g.N_STRIPS)]


def arm_poses(side, tcp=None, rot=None, gap=0.094):
    if tcp is None:
        q = np.array(READY, float)
    else:
        q, err, rerr = ik.solve(tcp, ik.matrix_to_quat_xyzw(rot), base_pos=station.BASES[side], iters=400)
        assert err < 0.002 and rerr < math.radians(1.0), (side, tcp, err)
    qd = dict(zip(ARM_JOINTS, q))
    qd.update(joint7=-gap / 2, joint8=-gap / 2)
    return analysis.fk_link_poses_world(qd, station.BASES[side])


def parts(guide_q, support_deg):
    hx, hz = g.SUPPORT_HINGE
    gx = (g.SLOT_X[0] + g.SLOT_X[1]) / 2
    out = {"cover": (np.array([g.HINGE[0], 0.0, g.TOP + g.HINGE[1]]), np.eye(3)),
           "support": (np.array([hx, 0.0, g.TOP + hz]), g.rot_y(math.radians(support_deg))),
           "guide": (np.array([gx, g.GUIDE_OPEN_Y + guide_q, g.TOP]), np.eye(3))}
    for k, xy in (("btn_black", g.BLACK_XY), ("btn_color", g.COLOR_XY), ("btn_ok", g.OK_XY)):
        out[k] = (np.array([*xy, g.TOP + g.BTN_TOP_H]), np.eye(3))
    return out


def gap_of(strips, guide_q=SNUG, support_deg=110.0, left=None, right=None):
    hulls = task.hand_hulls_from_poses({"left": left or arm_poses("left"), "right": right or arm_poses("right")})
    return task.gap_to_hulls(hulls, task.touch_objects(strips, parts(guide_q, support_deg)))


FAR = None


def trial(segments, gap=None, hinge=None):
    """segments: (steps, strips, guide_q, support_deg, press_ok); OK is pushed in 1.2 mm for 4 steps at the segment's
    start when press_ok. Returns the final state."""
    global FAR
    if FAR is None:
        FAR = gap_of(LOADED)
    S = task.fresh_state()
    k = 0
    for steps, strips, gq, sup, press in segments:
        for i in range(steps):
            depth = 0.0012 if (press and i < 4) else 0.0
            task.advance(S, strips, gq, sup, {"btn_ok": depth}, (hinge or (lambda _: 0.0))(k),
                         FAR if gap is None else gap(k))
            k += 1
    return S


def full(strips=LOADED, gq=SNUG, sup=110.0):
    return [(10, strips, gq, sup, False), (40, strips, gq, sup, True)]


CASES = []


def case(fn):
    CASES.append(fn)
    return fn


def expect(S, score, note=""):
    assert abs(S["tier"] - score) < 1e-9, f"{note}: expected {score}, got {S['tier']} ({S['rep']})"


@case
def loaded_snug_ok():
    expect(trial(full()), 1.0, "loaded, guide snug, OK")


@case
def guide_left_wide():
    expect(trial(full(gq=0.0)), 0.5, "guide at its widest")
    expect(trial(full(gq=SNUG + 0.0010)), 0.5, "guide 1 mm off the edge")
    expect(trial(full(gq=SNUG + 0.0004)), 1.0, "guide 0.4 mm off the edge")


@case
def not_to_the_right():
    expect(trial(full(strips=g.loaded_strip_poses(dy=0.001), gq=SNUG + 0.001)), 0.25, "right edge 1 mm off the side")


@case
def not_seated():
    expect(trial(full(strips=g.loaded_strip_poses(lift=0.005))), 0.25, "5 mm above the tray's floor")


@case
def ok_not_pressed():
    expect(trial([(50, LOADED, SNUG, 110.0, False)]), 0.75, "OK not pressed")
    expect(trial([(10, on_cover(), 0.0, 0.0, True), (40, LOADED, SNUG, 110.0, False)]), 0.75,
           "OK pressed before loading")


@case
def support_shut():
    expect(trial(full(sup=0.0)), 0.75, "paper support shut at the end")


@case
def nothing_and_folded():
    expect(trial([(50, on_cover(), 0.0, 0.0, False)]), 0.0, "doing nothing")
    expect(trial(full(), hinge=lambda k: 160.0 if k == 5 else 0.0), 0.0, "a hinge at 160 degrees")


@case
def hand_touching():
    import demo

    rot = demo.down(demo.GUIDE_YAW, demo.GUIDE_TILT)
    touching = None
    for dy in np.arange(0.030, 0.010, -0.0005):        # the demo's push pose, slid in until it meets the grip
        tip = np.array([demo.SLOT_XC, g.GUIDE_OPEN_Y + SNUG + dy, g.TOP + g.GUIDE_H[1] - 0.007])
        gap = gap_of(LOADED, left=arm_poses("left", tip - demo.TIP * rot[:, 2], rot, gap=0.0))
        if gap <= g.TOUCH:
            touching = gap
            break
    assert touching is not None and touching > -0.002, touching
    expect(trial(full(), gap=lambda k: touching), 0.75, "a fingertip against the guide")


@case
def ready_pose_clear():
    assert gap_of(LOADED) > 0.05


def main() -> int:
    bad = 0
    for fn in CASES:
        try:
            fn()
            print(f"ok   {fn.__name__}")
        except AssertionError as e:
            bad += 1
            print(f"FAIL {fn.__name__}: {e}")
    print(f"{len(CASES) - bad}/{len(CASES)} passed")
    return 1 if bad else 0


def test_all():
    assert main() == 0


if __name__ == "__main__":
    sys.exit(main())
