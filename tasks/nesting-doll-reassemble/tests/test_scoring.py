"""Scorer and fit-geometry tests for nesting-doll-reassemble (no simulator; needs yam_arena importable).

    python -m pytest tests/test_scoring.py -q
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import meshkit as mk  # noqa: E402
import partkit  # noqa: E402
import task as T  # noqa: E402

X, Y = 0.22, 0.04


def quat(yaw: float, tilt_deg: float = 0.0):
    """(x, y, z, w): a turn ``yaw`` about z, then ``tilt_deg`` about x."""
    t = math.radians(tilt_deg)
    qz = np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
    qx = np.array([math.sin(t / 2), 0.0, 0.0, math.cos(t / 2)])
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def top_state(n, pos, yaw=0.0, tilt=0.0, fit=0.0):
    return partkit.PartState(pos=np.asarray(pos, float), quat_xyzw=np.asarray(quat(yaw, tilt)), lin_vel=np.zeros(3),
                             ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3), sliders={f"fit{n}": fit})


def assembled(proud1=0.0, proud2=0.0, face1=0.0, face2=0.0):
    """Every piece where a closed set puts it, each top ``proud`` (m) above home and turned ``face`` (rad)."""
    z0 = T.TOP + 0.002
    d1, d2 = T.dims(1), T.dims(2)
    return {"doll1_bottom": mk.state_at(T.SPECS["doll1_bottom"], (X, Y, z0), quat(0.0)),
            "doll2_bottom": mk.state_at(T.SPECS["doll2_bottom"], (X, Y, z0 + T.FLOOR), quat(0.0)),
            "doll3": mk.state_at(T.SPECS["doll3"], (X, Y, z0 + 2 * T.FLOOR), quat(0.0)),
            "doll2_top": top_state(2, (X, Y, z0 + T.FLOOR + d2["zs"] + proud2), face2, fit=T.FIT_TRAVEL - proud2),
            "doll1_top": top_state(1, (X, Y, z0 + d1["zs"] + proud1), face1, fit=T.FIT_TRAVEL - proud1)}


def test_pressed_home_set_is_closed():
    lv = T.levels(assembled())
    assert lv == {"three_in_two": True, "two_closed": True, "two_in_one": True, "one_closed": True}


def test_tops_only_let_go_on_the_fit_are_not_closed():
    # a top let go on its spigot stands FIT_TRAVEL (4 mm) proud: the rest-only attempt scores the 0.25 level only
    lv = T.levels(assembled(proud1=T.FIT_TRAVEL, proud2=T.FIT_TRAVEL))
    assert lv["three_in_two"] and not lv["two_closed"] and not lv["one_closed"]


def test_half_pressed_top_is_not_closed():
    for proud in (0.0015, 0.002, 0.003):
        assert not T.seated(assembled(proud1=proud), 1)
    assert T.seated(assembled(proud1=0.0008), 1)


def test_second_doll_pressed_first_resting():
    lv = T.levels(assembled(proud1=T.FIT_TRAVEL))
    assert lv["two_in_one"] and not lv["one_closed"]


def test_face_must_line_up():
    assert not T.seated(assembled(face2=math.radians(20)), 2)
    assert T.seated(assembled(face2=math.radians(10)), 2)


def test_fit_ring_lands_on_the_spigot_rim():
    for n in T.FIT_FORCE:
        d = T.dims(n)
        lip_in, lip_out = d["sp_in"] + 0.0005, d["sp_out"] + 0.001
        assert lip_in < d["sp_out"] - 0.001           # at least 1 mm of the ring bears on the rim
        assert lip_in > d["sp_in"]                    # the inner doll passes the bore without touching it
        assert lip_out < d["top_in"] - 0.0003         # inside the top's skirt
        assert T.SPIGOT_H - T.FIT_TRAVEL > 0.002      # a top standing on its rim: the ring is off the table


def test_skirt_lead_in():
    prof = T.top_profile(1)
    d = T.dims(1)
    near = lambda q: any(abs(r - q[0]) < 1e-9 and abs(z - q[1]) < 1e-9 for r, z in prof)   # noqa: E731
    assert near((d["top_in"], T.LEAD_IN)) and near((d["top_in"] + T.LEAD_IN, 0.0))


def test_inner_dolls_clear_the_ring():
    # the head of the doll inside, at the ring's height when the top is home, is well inside the ring's bore
    for n, inner, foot in ((2, 3, T.FLOOR), (1, 2, T.FLOOR)):
        d = T.dims(n)
        z_ring = d["zs"] + T.SPIGOT_H - foot           # ring height in the inner doll's frame
        assert T.outer(inner, z_ring) < d["sp_in"] + 0.0005 - 0.005


def test_fit_force_far_above_the_tops_weight():
    for n in T.FIT_FORCE:
        weight = T.SPECS[f"doll{n}_top"].mass * 9.81
        assert T.FIT_FORCE[n] > 10 * weight
        creep_mm_2min = weight / T.FIT_DAMPING * 120 * 1000
        assert creep_mm_2min < 0.5


def test_engaged_only_over_its_own_spigot():
    st = assembled(proud1=T.FIT_TRAVEL)
    assert T.engaged(st["doll1_top"], st["doll1_bottom"], 1)
    lifted = top_state(1, np.asarray(st["doll1_top"].pos) + [0, 0, 0.02], fit=0.002)
    assert not T.engaged(lifted, st["doll1_bottom"], 1)
    aside = top_state(1, np.asarray(st["doll1_top"].pos) + [0.01, 0, 0], fit=0.002)
    assert not T.engaged(aside, st["doll1_bottom"], 1)
