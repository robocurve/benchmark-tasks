"""Near misses and unintended strategies for rinsing the molar while the ejector catches the water, in lot A.

The score rests on a history (the tracked water, whether the hands were clear, the still hold), so behaviour cases drive
the scorer's own recorder (`task.record`, which the simulator calls every control step) through a sequence of tool
poses. The water model itself decides where each step's water lands. CASES (run by factory/check.py) then check end
states on top of a full, valid history."""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import lots  # noqa: E402
import task  # noqa: E402

lots.use("A")
HZ = 20


def qz(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def head():
    return task.HEAD.state(pos=tuple(lots.current().pos("head")), quat_xyzw=qz(lots.current().yaw_of("head")))


def ej(tip_head=None, yaw_head=None, rest_dxy=(0.0, 0.0), dz=0.0, dyaw=0.0, vel=None):
    """The ejector at a pose in the head frame (its opening, its yaw), or on its rest moved by rest_dxy."""
    if tip_head is None:
        p, y = task.S_REST + np.array([*rest_dxy, dz]), task.S_REST_YAW + dyaw
    else:
        p, y = task._tool_world(tip_head, task.S_YAW_USE if yaw_head is None else yaw_head)
    return task.EJECTOR.state(pos=tuple(p), quat_xyzw=qz(y), lin_vel=vel)


def sy(noz_head=None, yaw_head=None, rest_dxy=(0.0, 0.0), dz=0.0, dyaw=0.0, vel=None):
    if noz_head is None:
        p, y = task.Y_REST + np.array([*rest_dxy, dz]), task.Y_REST_YAW + dyaw
    else:
        p, y = task._tool_world(noz_head, task.Y_HEADING if yaw_head is None else yaw_head)
    return task.SYRINGE.state(pos=tuple(p), quat_xyzw=qz(y), lin_vel=vel)


def world(e=None, s=None):
    return {"head": head(), "ejector": e if e is not None else ej(), "syringe": s if s is not None else sy(),
            "ejector_rest": task.EJECTOR_REST.state(), "syringe_rest": task.SYRINGE_REST.state()}


def run(t_end, make, clear=lambda t: False):
    """Record from the current time to t_end (s), the tools placed by make(t)."""
    st = None
    while task._L["t"] < t_end - 1e-9:
        t = task._L["t"]
        st = make(t)
        task.record(st, clear(t))
    return st


EJ_IN = task.S_TIP_USE
SY_IN = task.Y_NOZ_USE
EJ_OTHER = EJ_IN * np.array([1.0, -1.0, 1.0])  # the left pocket instead


def rinse(ej_tip=EJ_IN, sy_noz=SY_IN, sy_yaw=None, ej_until=31.5, sy_until=30.5, ej_yaw=None):
    """Both tools in place from 15 s; the syringe comes out at sy_until, the ejector at ej_until."""
    def make(t):
        e = ej(ej_tip, ej_yaw) if 15.0 <= t < ej_until else ej()
        s = sy(sy_noz, sy_yaw) if 15.0 <= t < sy_until else sy()
        return world(e, s)
    return make


def finish(make, home_s=1.5, clear=True):
    run(33.0, make)
    return run(33.0 + home_s, lambda t: world(), lambda t: clear)


RESULTS = []


def case(label, fn, want):
    task.reset_state()
    task._L["known"] = True
    st = fn()
    got = task.score(st if st is not None else world())
    RESULTS.append((label, got, want))
    print(f"{'ok  ' if abs(got - want) < 1e-9 else 'FAIL'} {label}: {got} (want {want}) "
          f"[dose {task._L['dose']:.2f} throat {task._L['throat']:.2f} spill {task._L['spill']:.2f} "
          f"left {task.mouth_water():.2f} mL]")


# the intended method and its partial steps
case("rinse with the ejector in the right pocket, tools back, let go, still 1.5 s", lambda: finish(rinse()), 1.0)
case("same, but still holding the tools", lambda: finish(rinse(), clear=False), 0.5)
case("same, only 0.9 s still", lambda: finish(rinse(), home_s=0.9), 0.5)
case("same, ejector taken out as the water stops (water left in the mouth)",
     lambda: finish(rinse(ej_until=30.0)), 0.25)
case("same, ejector taken out 0.3 s after the water stops (0.18 mL left: dry enough)",
     lambda: finish(rinse(ej_until=30.3)), 1.0)
case("rinse, then both tools left in the mouth, let go", lambda: run(34.5, rinse(ej_until=99, sy_until=99),
                                                                     lambda t: True), 0.5)
# habits and shortcuts
case("no ejector: rinse only", lambda: finish(rinse(ej_until=0.0)), 0.0)
case("ejector in the left pocket instead", lambda: finish(rinse(ej_tip=EJ_OTHER, ej_yaw=-task.S_YAW_USE)), 0.0)
case("ejector tip pushed into the throat", lambda: finish(rinse(ej_tip=EJ_IN + np.array([0.014, 0.0, -0.002]))),
     0.0)
case("ejector held just above the water (opening 4 mm over the pocket's lip)",
     lambda: finish(rinse(ej_tip=EJ_IN + np.array([0.0, 0.0, 0.0065]))), 0.0)
case("syringe left on its rest while the water runs", lambda: finish(rinse(sy_until=0.0)), 0.0)
case("syringe pulled out of the mouth at 25 s, water still running", lambda: finish(rinse(sy_until=25.0)), 0.0)
def aim(target, heading=task.Y_HEADING):
    """The nozzle 25 mm short of target (head frame) and as far above it: the 45 degree jet lands on target."""
    return np.asarray(target, float) - 0.025 * np.array([math.cos(heading), math.sin(heading), -1.0])


case("jet aimed at the second premolar instead", lambda: finish(rinse(sy_noz=aim((-0.0095, 0.021, 0.0)))), 0.0)
case("jet aimed at the left first molar (wrong side)",
     lambda: finish(rinse(sy_noz=aim((0.0, -0.023, 0.0), -task.Y_HEADING), sy_yaw=-task.Y_HEADING)), 0.0)
case("jet into the right pocket, beside the ejector (no rinse)",
     lambda: finish(rinse(sy_noz=aim((0.025, 0.020, -0.020)))), 0.0)
case("on the target only 2.6 s, then on the second molar",
     lambda: (run(22.6, rinse(ej_until=99, sy_until=99)), finish(rinse(sy_noz=aim((0.011, 0.0245, 0.0)))))[1], 0.0)
case("nothing done", lambda: run(35.0, lambda t: world(), lambda t: True), 0.0)
case("tools put in place, taken out before the water starts", lambda: finish(rinse(ej_until=19.0, sy_until=19.0)),
     0.0)


def boundary(throat=None, spill=None):
    def fn():
        st = finish(rinse())
        if throat is not None:
            task._L["throat"] = throat
        if spill is not None:
            task._L["spill"] = spill
        return st
    return fn


case("1.0 mL reached the throat", boundary(throat=1.0), 1.0)
case("1.01 mL reached the throat", boundary(throat=1.01), 0.0)
case("1.0 mL landed outside the mouth", boundary(spill=1.0), 1.0)
case("1.01 mL landed outside the mouth", boundary(spill=1.01), 0.0)
case("hand state unknown", lambda: (finish(rinse()), task._L.update(known=False))[0], 0.0)


def face_on_syringe_handle():
    """A flat hand face 0.5 mm above the syringe handle's top flat, 200 mm across, so its corners are over 5 mm from
    every shape: a check on corners alone would call the hand clear. The scorer samples the face itself."""
    tri = np.array([[0.115, 0.0, 0.0], [-0.0575, 0.0996, 0.0], [-0.0575, -0.0996, 0.0]])
    task._HAND_PTS["_flat_face"] = task.sample_triangles([tri])
    st = world()
    g = task.Y_REST + task._Rz(task.Y_REST_YAW) @ task.Y_GRIP
    pose = np.eye(4)
    pose[:3, 3] = (g[0], g[1], g[2] + task.Y_HANDLE_R + 0.0005)
    from yam_arena import analysis
    corners = tri + pose[:3, 3]
    far = min(analysis.box_signed_distance(corners, np.asarray(c), np.asarray(r), np.asarray(s)).min()
              for spec, name in ((task.EJECTOR, "ejector"), (task.SYRINGE, "syringe"))
              for _, c, r, s in spec.world_boxes({"base": (np.asarray(st[name].pos), task._R(st[name].quat_xyzw))}))
    assert far > task.CLEAR + 0.001, f"face corners only {far * 1000:.1f} mm away"
    return task.hands_clear_of([("_flat_face", pose)], st)


case("full rinse, tools home, a flat hand face 0.5 mm above the syringe handle (corners far off)",
     lambda: finish(rinse(), clear=face_on_syringe_handle()), 0.5)

BAD = [r for r in RESULTS if abs(r[1] - r[2]) > 1e-9]
if BAD:
    raise SystemExit(f"near_misses: {len(BAD)} behaviour case(s) failed: {BAD}")

# End states on top of a full, valid history (rinsed, dry, tools home, 1.5 s still and clear).
case("history for CASES", lambda: finish(rinse()), 1.0)
CASES = [
    ("both tools on their rests", {"ejector": ej(), "syringe": sy()}, 1.0, True),
    ("ejector 9 mm off its rest's spot", {"ejector": ej(rest_dxy=(0.009, 0.0))}, 1.0, True),
    ("ejector 12 mm off its rest's spot", {"ejector": ej(rest_dxy=(0.012, 0.0))}, 0.5, False),
    ("syringe turned 12 degrees on its rest", {"syringe": sy(dyaw=math.radians(12))}, 0.5, False),
    ("syringe held 4 mm up", {"syringe": sy(dz=0.004)}, 0.5, False),
    ("syringe still moving", {"syringe": sy(vel=(0.05, 0.0, 0.0))}, 0.5, False),
    ("ejector left in the mouth", {"ejector": ej(EJ_IN)}, 0.5, False),
]
