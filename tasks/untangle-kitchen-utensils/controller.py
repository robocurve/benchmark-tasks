"""Joint-target demonstration for the fixed reset. No live utensil data enters the planner."""

import numpy as np
from scipy.spatial.transform import Rotation

from planner import frame
from scene import SLOTS, TABLE

SPAT_YAW = 32
SPOON_POINT = np.array([0.311, 0.134, TABLE + 0.013])
SPAT_POINT = np.array([0.205, -0.067, TABLE + 0.018])
WHISK_POINT = np.array([0.164, 0.032, TABLE + 0.017])


def yaw(degrees):
    return Rotation.from_euler("z", degrees, degrees=True).as_matrix()


def motions(planner, joint_observation, trial="demo"):
    def move(label, arm, position, jaw, opening, ticks):
        for row in planner.move(arm, np.asarray(position), jaw, opening, ticks):
            yield label, row

    def grasp(name, arm, point, jaw):
        yield from move("approach " + name, arm, point + [0, 0, 0.095], jaw, 0.64, 24)
        yield from move("straddle " + name, arm, point, jaw, 0.64, 18)
        yield from move("grip " + name, arm, point, jaw, 0, 12)
        opening = np.asarray(joint_observation()).reshape(2, 7)[arm, 6]
        if not 0.055 < opening < 0.45:
            raise RuntimeError(f"Unexpected public gripper opening for {name}: {opening:.3f}")

    def deposit(name, arm, at, jaw):
        at = np.asarray(at)
        yield from move("align " + name, arm, at + [0, 0, 0.095], jaw, 0, 32)
        yield from move("seat " + name, arm, at, jaw, 0, 22)
        yield from move("release " + name, arm, at, jaw, 0.62, 12)
        yield from move("clear " + name, arm, at + [0, 0, 0.103], jaw, 0.62, 18)

    def park(arm):
        side = 1 if arm == 0 else -1
        yield from move(
            "park arm",
            arm,
            [0.22, side * 0.27, TABLE + 0.18],
            frame((0.75, 0, -0.66), (0, -1, 0)),
            0.65,
            24,
        )

    left, right = 0, 1
    # The left pads brace the wooden shaft while the silicone blade slides out.
    spoon_jaw = yaw(-90) @ frame((0.70, 0, -0.714), (0, -1, 0))
    spat_jaw = yaw(SPAT_YAW) @ frame((0.70, 0.08, -0.72), (0, -1, 0))
    whisk_jaw = frame((0.72, 0, -0.69), (0, -1, 0))
    yield from grasp("spoon", left, SPOON_POINT, spoon_jaw)
    if trial == "lift":
        yield from move("lift spoon alone", left, SPOON_POINT + [0, 0, 0.080], spoon_jaw, 0, 30)
        return
    if trial == "pull":
        yield from grasp("whisk", right, WHISK_POINT, whisk_jaw)
        yield from move("pull past blade", left, SPOON_POINT + [0, 0.118, 0.005], spoon_jaw, 0, 32)
        return
    yield from grasp("spatula", right, SPAT_POINT, spat_jaw)
    retract = SPAT_POINT + yaw(SPAT_YAW) @ [-0.108, 0, 0.011]
    yield from move("withdraw spatula", right, retract, spat_jaw, 0, 30)
    yield from move("lift spatula", right, retract + [0.035, -0.030, 0.115], spat_jaw, 0, 24)
    spat_level = yaw(-SPAT_YAW) @ spat_jaw
    yield from deposit("spatula", right, [0.527, SLOTS[0], TABLE + 0.022], spat_level)
    yield from park(right)
    yield from grasp("whisk", right, WHISK_POINT, whisk_jaw)
    slide = SPOON_POINT + [0, 0.118, 0.005]
    yield from move("withdraw spoon", left, slide, spoon_jaw, 0, 32)
    yield from move("lift spoon", left, slide + [0, 0, 0.125], spoon_jaw, 0, 20)
    # Point the bowl toward the near end of the drawer. This puts the grasp
    # within reach while the deeper end receives the narrow handle.
    spoon_level = yaw(90) @ spoon_jaw
    yield from deposit("spoon", left, [0.518, SLOTS[2], TABLE + 0.022], spoon_level)
    yield from park(left)
    yield from move("lift whisk", right, WHISK_POINT + [0, 0, 0.135], whisk_jaw, 0, 25)
    yield from deposit("whisk", right, [0.495, SLOTS[1], TABLE + 0.028], whisk_jaw)
    yield from park(right)
    for _ in range(24):
        yield "settle", planner.q.ravel().copy()
