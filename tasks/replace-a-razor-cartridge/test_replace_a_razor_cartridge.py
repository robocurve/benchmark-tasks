"""Offline checks (no GPU, no Isaac Sim; needs yam_arena importable): python -m pytest -q
The physics (the latches holding against a pull, releasing when the head is pinched, snapping into a new socket) is
checked by check_demo.py (PyBullet)."""

from __future__ import annotations

import json
import math
import pathlib
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
pytest.importorskip("yam_arena.task_file")
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402

import household as H  # noqa: E402
import task as T  # noqa: E402

FAR = [np.array([0.0, 0.5, 1.2])] * 8
PLACE = (0.33, -0.02)
YAW = T.DISP_YAW


def handle_at(xy=PLACE, yaw=YAW, z=0.0, **joints):
    return H.state_of(T.HANDLE, origin=(xy[0], xy[1], H.MAT_TOP + z), yaw=yaw, joints=joints)


def cart_on(spec, handle, dx=0.0, yaw_extra=0.0):
    """A cartridge seated on the handle's post (dx: shifted along the handle, metres)."""
    yaw = math.atan2(handle.rotation()[1, 0], handle.rotation()[0, 0]) + yaw_extra
    return H.state_of(spec, origin=handle.to_world((dx, 0.0, 0.0)), yaw=yaw)


def old_in_tray():
    o = H.state_of(T.OLD, origin=(0, 0, 0))
    c = H.corners(T.OLD, o)
    mid = (c.min(0) + c.max(0)) / 2
    return H.state_of(T.OLD, origin=(T.TRAY_XY[0] - mid[0], T.TRAY_XY[1] - mid[1],
                                     H.MAT_TOP + H.TRAY_FLOOR_T - c[:, 2].min()))


def old_on_mat():
    return H.state_of(T.OLD, origin=(T.RAZOR_XY[0], T.RAZOR_XY[1], H.MAT_TOP))


def objs(handle=None, old=None, new=None):
    return {"handle": handle or H.state_of(T.HANDLE), "old": old or H.state_of(T.OLD),
            "new": new or H.state_of(T.NEW, yaw=YAW), "dispenser": H.state_of(T.DISPENSER, yaw=YAW), "tray": H.state_of(T.TRAY)}


def watch(o, fingers=FAR, steps=1):
    for _ in range(steps):
        T._watch(o, fingers)


def test_start_state_old_latched_new_in_dispenser_scores_zero():
    T._fresh()
    o = objs()
    assert T.latched(o["handle"], o["old"])
    assert T.in_dispenser(o["new"], o["dispenser"]) and not T.latched(o["handle"], o["new"])
    watch(o, steps=100)
    assert T.score(o) == 0.0


def test_latch_geometry_overlaps_the_windows():
    face = T.HOLE[1] / 2
    assert T.LATCH_OUT > 0.5                                       # each tooth reaches into its window
    assert T.WINDOW[0] < T.LATCH_Z - T.LATCH_HALF and T.LATCH_Z + T.LATCH_HALF < T.WINDOW[1]
    assert face + T.LATCH_OUT < T.BLOCK[1] / 2                     # and stays inside the block's wall
    assert T.HEAD_W / 2 + T.BUTTON_OUT - T.LATCH_OUT > T.HEAD_W / 2  # the buttons stand proud while latched


def test_latched_requires_seat_alignment_and_latches_out():
    h = handle_at()
    assert T.latched(h, cart_on(T.NEW, h))
    assert not T.latched(h, cart_on(T.NEW, h, dx=0.002))                # off the post
    assert not T.latched(h, cart_on(T.NEW, h, yaw_extra=math.pi))       # on backwards
    assert not T.latched(h, cart_on(T.NEW, h, yaw_extra=math.radians(20)))
    assert not T.latched(handle_at(latch_l=-0.0012, latch_r=0.0012), cart_on(T.NEW, h))   # buttons held in


def test_old_on_the_mat_detached_scores_quarter():
    T._fresh()
    o = objs(handle=handle_at(PLACE, yaw=0.0, z=0.02), old=old_on_mat())
    watch(o, steps=100)
    assert T.detached(o, "old") and T.score(o) == 0.25


def test_old_in_tray_needs_two_untouched_seconds():
    T._fresh()
    o = objs(handle=handle_at(PLACE, yaw=0.0, z=0.02), old=old_in_tray())
    watch(o, steps=39)
    assert T.score(o) == 0.25
    watch(o, steps=1)
    assert T.score(o) == 0.5


def test_replacement_seated_in_the_dispenser_is_not_retained():
    T._fresh()
    new = H.state_of(T.NEW, yaw=YAW)
    h = H.state_of(T.HANDLE, origin=new.origin, yaw=YAW)
    o = objs(handle=h, old=old_in_tray(), new=new)
    assert T.latched(h, new)
    watch(o, steps=200)
    assert not T.S["passed"] and T.score(o) == 0.5


def test_retained_replacement_and_stowed_old_scores_one_after_the_load():
    T._fresh()
    h = handle_at()
    o = objs(handle=h, old=old_in_tray(), new=cart_on(T.NEW, h))
    watch(o, steps=40)                    # quiet for 2 s
    assert not T.S["passed"] and T.score(o) == 0.5
    watch(o, steps=40)                    # the load for 2 s
    assert T.S["passed"] and T.score(o) == 1.0


def test_retained_replacement_with_old_on_the_mat_scores_three_quarters():
    T._fresh()
    h = handle_at()
    o = objs(handle=h, old=old_on_mat(), new=cart_on(T.NEW, h))
    watch(o, steps=100)
    assert T.score(o) == 0.75


def test_replacement_lost_during_the_load_fails():
    T._fresh()
    h = handle_at()
    o = objs(handle=h, old=old_in_tray(), new=cart_on(T.NEW, h))
    watch(o, steps=50)
    o["new"] = cart_on(T.NEW, h, dx=0.005)                              # pulled off
    watch(o, steps=60)
    assert not T.S["passed"] and T.score(o) == 0.5


def test_old_cartridge_back_on_the_handle_scores_zero():
    T._fresh()
    h = handle_at()
    o = objs(handle=h, old=cart_on(T.OLD, h), new=old_in_tray())
    watch(o, steps=100)
    assert T.score(o) == 0.0


def test_anything_off_the_table_scores_zero_for_good():
    T._fresh()
    h = handle_at()
    o = objs(handle=h, old=H.state_of(T.OLD, origin=(0.3, 0.9, H.TOP - 0.3)), new=cart_on(T.NEW, h))
    watch(o)
    o["old"] = old_in_tray()
    watch(o, steps=100)
    assert T.score(o) == 0.0


def _play(plan):
    plans = {a: ArmPlan(list(w), station.BASES[a]) for a, w in plan.items()}
    end = max(p.duration for p in plans.values())
    t = 0.0
    while t <= end + 3.0:
        t += 0.05
        for p in plans.values():
            p.action7(t)
    return plans, int(round((end + 3.0) / 0.05))


def test_demo_is_reachable_and_inside_the_run_limit():
    plans, steps = _play(T.demo())
    for p in plans.values():
        assert p.worst_ik_mm < 1.0 and p.worst_ik_deg < 1.0
    assert steps <= 3600


def test_counterexamples_are_reachable():
    for name, (fn, _expect, _why) in T.COUNTEREXAMPLES.items():
        plans, _ = _play(fn())
        for p in plans.values():
            assert p.worst_ik_mm < 1.0, name


def test_task_json_matches_the_brief():
    meta = json.loads((HERE / "task.json").read_text())
    assert meta["category"] == "Societal relatability"
    assert meta["instruction"] == ("Replace the cartridge on the razor handle with the replacement in the dispenser. "
                                   "Put the old cartridge in the collection tray.")
    assert meta["context"] == (
        "Replacing a used blade cartridge is a routine part of home grooming with a reusable cartridge razor. This "
        "task completes the consumable-replacement operation: the old cartridge is removed and placed in a collection "
        "tray, and the replacement is fitted securely to the same handle. The result is an assembled razor whose "
        "replacement cartridge remains attached after release.")
    assert len(meta["fidelity"]) <= 500
    assert meta["run_limit"].startswith("3,600")
