"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: ready state}, expected score, expected success). States come from the parts' own
kinematics (Part.state), so turned and tilted cases have the bounding boxes the simulator would report.
"""

import numpy as np

from task import (BASE_XY, BASE_Z, FOOT, FOOT_H, FOOT_XY, FRAME, MOTOR_BASE, PT, SHIM, SHIM_HOME_X, SHIM_PLATE,
                  SHIM_START, SHIM_Z, SHORT_H)

NL_Y = BASE_XY[1] + FOOT_XY["near_left"][1]
PAD_NEAR_X = BASE_XY[0] + FOOT_XY["near_left"][0] - FOOT / 2  # near edge of the near-left pad (0.200)
MOVING = (0.0, 0.0, -0.05)


def q_axis(axis, angle):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    return (*(a * np.sin(angle / 2)), np.cos(angle / 2))


def q_yaw(a):
    return q_axis((0, 0, 1), a)


def shim(x=SHIM_HOME_X, y=NL_Y, z=SHIM_Z, yaw=0.0, quat=None, vel=None):
    return SHIM.state(pos=(x, y, z), quat_xyzw=quat if quat is not None else q_yaw(yaw), lin_vel=vel)


def shim_cover(c, y=NL_Y):
    """Shim straight in from the robot's side, covering fraction c of the near-left pad's length (full width)."""
    return shim(x=PAD_NEAR_X + c * FOOT - SHIM_PLATE[0] / 2, y=y)


def base(dx=0.0, dy=0.0, dz=0.0, yaw=0.0, quat=None, vel=None):
    return MOTOR_BASE.state(pos=(BASE_XY[0] + dx, BASE_XY[1] + dy, BASE_Z + dz),
                            quat_xyzw=quat if quat is not None else q_yaw(yaw), lin_vel=vel)


def rocked(corner, rise):
    """The base turned about the near-right/far-left diagonal (through those pads' undersides, which stay put) until
    the far-right pad's centre is `rise` higher than at the start."""
    a = np.array([*FOOT_XY["near_right"], -FRAME[2] / 2 - FOOT_H])
    b = np.array([*FOOT_XY["far_left"], -FRAME[2] / 2 - FOOT_H])
    axis = (b - a) / np.linalg.norm(b - a)
    f = np.array([*FOOT_XY[corner], -FRAME[2] / 2 - FOOT_H])
    v = f - a
    d = np.linalg.norm(v - axis * np.dot(v, axis))  # pad centre's distance from the diagonal
    origin = np.array([*BASE_XY, BASE_Z])
    for sign in (1.0, -1.0):
        q = q_axis(axis, sign * np.arcsin(rise / d))
        st = MOTOR_BASE.state(pos=(0, 0, 0), quat_xyzw=q)
        r = np.array(st.quat_xyzw)
        x, y, z, w = r
        m = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        if (m @ f)[2] - f[2] + (a - m @ a)[2] >= -1e-9:
            return MOTOR_BASE.state(pos=tuple(origin + a - m @ a), quat_xyzw=q)
    raise ValueError("no rise")


GAP = FOOT_H - SHORT_H - SHIM_PLATE[2]  # 0.8 mm left under the shimmed foot
FR_X, FR_Y = BASE_XY[0] + FOOT_XY["far_right"][0], BASE_XY[1] + FOOT_XY["far_right"][1]
# Shim turned 90 degrees, under the far-right pad from the right, its tab outside the frame.
SHIM_UNDER_FR = dict(x=FR_X, y=FR_Y + 0.02, yaw=np.pi / 2)

CASES = [
    ("demo: shim fully under the near-left (short) foot", {"shim": shim()}, 1.0, True),
    ("as measured in the trace (centre 1.3 mm right)", {"shim": shim(x=0.1862, y=0.0513)}, 1.0, True),
    ("untouched start: near-left foot 5 mm off the plate", {}, 0.0, False),
    ("shimmed, every foot sunk exactly 0.5 mm (limit)", {"shim": shim(), "motor_base": base(dz=-0.0005)}, 1.0, True),
    ("shimmed, every foot sunk 0.6 mm", {"shim": shim(), "motor_base": base(dz=-0.0006)}, 0.0, False),
    ("shim covering exactly 75 % of the pad", {"shim": shim_cover(0.75)}, 1.0, True),
    ("shim covering 74 % of the pad (part-way)", {"shim": shim_cover(0.74)}, 0.0, False),
    ("shim covering 58 % (the part-way trace)", {"shim": shim(x=0.1673, y=0.0513)}, 0.0, False),
    ("shim edge just touching the pad (1 %): not under it", {"shim": shim_cover(0.01)}, 0.0, False),
    ("shim 10 mm to the robot's right: 83 % cover", {"shim": shim(y=NL_Y - 0.010)}, 1.0, True),
    ("shim 18 mm to the right: 73 % cover", {"shim": shim(y=NL_Y - 0.018)}, 0.0, False),
    ("shim against the near-right (bearing) foot, as in the trace", {"shim": shim(x=0.1485, y=-0.066)}, 0.0, False),
    ("shim turned 20 degrees under the short foot", {"shim": shim(yaw=np.radians(20))}, 1.0, True),
    ("near-left corner pressed down and held (far-right foot up 5 mm)",
     {"motor_base": rocked("far_right", 0.005)}, 0.0, False),
    ("near-left corner pressed, shim under it, still held",
     {"motor_base": rocked("far_right", 0.005), "shim": shim()}, 0.0, False),
    ("shim under the far-right foot (its diagonal partner): no rock but tilted 2.6 degrees",
     {"motor_base": rocked("far_right", SHIM_PLATE[2]), "shim": shim(**SHIM_UNDER_FR)}, 0.0, False),
    ("shim under the short foot, base still rocking", {"shim": shim(), "motor_base": base(vel=MOVING)}, 0.0, False),
    ("shim placed but still moving", {"shim": shim(vel=(0.02, 0.0, 0.0))}, 0.0, False),
    ("shim tilted 3 degrees (propped on something)",
     {"shim": shim(quat=q_axis((0, 1, 0), np.radians(3)))}, 0.0, False),
    ("shim 0.2 mm lower (gap exactly 1.0 mm)", {"shim": shim(z=SHIM_Z - (0.0010 - GAP))}, 1.0, True),
    ("shim 0.3 mm lower (gap 1.1 mm)", {"shim": shim(z=SHIM_Z - (0.0011 - GAP))}, 0.0, False),
    ("shim lying on the frame's top", {"shim": shim(x=0.30, y=0.0, z=BASE_Z + FRAME[2] / 2 + SHIM_PLATE[2] / 2)},
     0.0, False),
    ("base moved 3.0 mm toward the shim side, shim under it", {"shim": shim(), "motor_base": base(dy=0.003)},
     1.0, True),
    ("base moved 3.2 mm off its marks, shim under it", {"shim": shim(), "motor_base": base(dy=0.0032)}, 0.0, False),
    ("base turned 1.9 degrees on its marks, shim under it",
     {"shim": shim(), "motor_base": base(yaw=np.radians(1.9))}, 1.0, True),
    ("base turned 2.1 degrees", {"shim": shim(), "motor_base": base(yaw=np.radians(2.1))}, 0.0, False),
    ("base turned 180 degrees in place, shim at the start", {"motor_base": base(yaw=np.pi)}, 0.0, False),
    ("shim still at its start, base pressed level by a hand", {"motor_base": rocked("far_right", 0.0025)}, 0.0, False),
    ("shim left at its start", {"shim": shim(x=SHIM_START[0], y=SHIM_START[1])}, 0.0, False),
]


def _run() -> int:
    import task

    start = {o.name: (o.state() if hasattr(o, "state") else None) for o in task.OBJECTS}
    from yam_arena.task_file import ObjectState

    for o in task.OBJECTS:
        if start[o.name] is None:
            p = np.asarray(o.pos, float)
            start[o.name] = ObjectState(p, np.array([0, 0, 0, 1.0]), np.zeros(3), np.zeros(3),
                                        p - np.asarray(o.size) / 2, p + np.asarray(o.size) / 2)
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(start) | changes
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        g = {k: (None if v is None else round(v * 1000, 2)) for k, v in task.gaps(st).items()}
        print(f"{flag} {got:4.2f}  {label}  gaps mm {g}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
