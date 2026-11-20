"""Offline checks of score() and success() on synthetic states: python test_scoring.py (or pytest).

yam_arena is stubbed when it is not installed; states are simple stand-ins with the fields the scorer reads.
Offline, release and stillness are assumed (no arm poses or step clock), so these test the placement rules.
"""

import importlib.util
import math
import pathlib
import sys
import types
from types import SimpleNamespace as NS

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent


def _load():
    try:
        import yam_arena.task_file  # noqa: F401
    except ImportError:
        from dataclasses import dataclass

        @dataclass(frozen=True)
        class Box:
            name: str
            pos: tuple
            size: tuple = (0.04, 0.04, 0.04)
            color: tuple = (0.7, 0.7, 0.7)
            mass: float = None
            yaw: float = 0.0

            def _check(self):
                pass

        @dataclass(frozen=True)
        class ObjectState:
            pos: np.ndarray
            quat_xyzw: np.ndarray
            lin_vel: np.ndarray
            ang_vel: np.ndarray
            low: np.ndarray
            high: np.ndarray

        c = types.ModuleType("yam_arena.constants")
        c.FINGER_CLOSED, c.FINGER_OPEN = 0.0, -0.04695
        c.ARM_JOINTS, c.FINGER_JOINTS = (), ()
        t = types.ModuleType("yam_arena.task_file")
        t.TABLE_TOP, t.Box, t.ObjectState, t.GREY = {"station": 0.7475}, Box, ObjectState, (0.7, 0.7, 0.7)
        root = types.ModuleType("yam_arena")
        root.task_file = t
        sys.modules.update({"yam_arena": root, "yam_arena.constants": c, "yam_arena.task_file": t})
    sys.path.insert(0, str(HERE))
    spec = importlib.util.spec_from_file_location("dental_task", HERE / "assemble_dental_xray_holder.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


T = _load()


def quat(yaw=0.0, tilt=0.0, tilt_axis=(1.0, 0.0, 0.0)):
    """Yaw about z, then a tilt about a world axis."""
    def q(axis, deg):
        h = math.radians(deg) / 2
        a = np.asarray(axis, float) / np.linalg.norm(axis)
        return np.array([*(a * math.sin(h)), math.cos(h)])

    def mul(a, b):
        x1, y1, z1, w1 = a
        x2, y2, z2, w2 = b
        return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                         w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
    return mul(q(tilt_axis, tilt), q((0, 0, 1), math.degrees(yaw)))


def st(origin, q):
    o = np.asarray(origin, float)
    return NS(origin=o, pos=o, quat_xyzw=np.asarray(q, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
              low=o - 0.05, high=o + 0.05)


K = T.KIT_YAW
BLOCK = st((*T.BB_XY, T.TOP + 0.0005), quat(K))


def layout(arm_dz=0.0, arm_tilt=0.0, ring="start", ring_dz=0.0, ring_tilt=0.0, ring_yaw=0.0, ring_dxy=(0, 0),
           block=BLOCK, sensor_dx=0.0, arm="seated"):
    objs = {T.BITE_BLOCK.name: block, T.TRAY_SLOT.name: st((*T.BB_XY, T.TOP), quat(K)),
            T.STAND.name: st((*T.STAND_XY, T.TOP), quat(K))}
    sen = T.kit_point(objs, (T.SENSOR_SEATED[0] + sensor_dx, 0), T.PADDLE[2] + T.SENSOR[2] / 2)
    objs[T.SENSOR_PART.name] = st(sen, block.quat_xyzw)
    if arm == "stand":
        objs[T.ARM.name] = st(T.ARM_START, quat(K))
    else:
        objs[T.ARM.name] = st(T.arm_target(objs) + [0, 0, arm_dz], quat(K, arm_tilt, (1, 0, 0)))
    if ring == "start":
        objs[T.RING.name] = st((*T.RING_START, T.TOP + T.RING_T / 2), quat(T.RING_START_YAW))
    else:
        foot = T.upright_foot(objs)
        rest = foot + [0, 0, T.HUB_Z + T.HUB[2] / 2 + T.RING_T / 2 + ring_dz]
        q = quat(K + ring_yaw, ring_tilt, (1, 0, 0))
        origin = rest - T._mat(q) @ np.array([T.RING_LUG, 0.0, 0.0]) + [*ring_dxy, 0]
        objs[T.RING.name] = st(origin, q)
    return objs


CASES = [
    ("start: arm in its stand, ring on the table", dict(arm="stand"), 0.0),
    ("arm seated, ring still on the table", dict(), 0.5),
    ("arm seated and ring fitted on the hub", dict(ring="fitted"), 1.0),
    ("ring on the upright but 10 mm above the hub (not slid down)", dict(ring="fitted", ring_dz=0.010), 0.5),
    ("ring 3 mm above the hub (within the 4 mm rest band)", dict(ring="fitted", ring_dz=0.003), 1.0),
    ("ring fitted but tilted 10 degrees", dict(ring="fitted", ring_tilt=10.0), 0.5),
    ("ring turned 180 degrees on the upright (centre away from the sensor)", dict(ring="fitted", ring_yaw=math.pi), 0.5),
    ("ring beside the upright, 6 mm off its axis", dict(ring="fitted", ring_dxy=(0.006, 0.0)), 0.5),
    ("prongs half in: arm 6 mm high", dict(arm_dz=0.006, ring="fitted"), 0.0),
    ("prongs 1 mm high (within 1.5 mm)", dict(arm_dz=0.001), 0.5),
    ("arm leaning 10 degrees", dict(arm_tilt=10.0), 0.0),
    ("bite block tipped 8 degrees", dict(block=st(BLOCK.origin, quat(K, 8.0, (1, 0, 0))), ring="fitted"), 0.0),
    ("bite block pushed 3 cm", dict(block=st(BLOCK.origin + [0.03, 0, 0], quat(K)), ring="fitted"), 0.0),
    ("sensor slid 5 mm out of its pocket", dict(sensor_dx=-0.005, ring="fitted"), 0.0),
]


def test_cases():
    for name, kw, want in CASES:
        got = T.score(layout(**kw))
        assert abs(got - want) < 1e-9, f"{name}: score {got}, expected {want}"
        assert T.success(layout(**kw)) == (want == 1.0)


def test_beam_line():
    fitted = layout(ring="fitted")
    assert T.beam_miss(fitted) <= 0.002
    # The aiming line 11 mm off the sensor's centre is not fitted even with the ring otherwise on the hub.
    objs = layout(ring="fitted")
    r = objs[T.RING.name]
    objs[T.RING.name] = st(r.origin + [0.011, 0, 0], r.quat_xyzw)
    assert T.beam_miss(objs) > T.RING_CENTRE_TOL and not T.ring_fitted(objs)


if __name__ == "__main__":
    test_cases()
    test_beam_line()
    for name, kw, want in CASES:
        print(f"{T.score(layout(**kw)):.1f}  {name}")
    print("all passed")
