"""Probe scene for the quaternion order (not a task): a fixed stand rotated 90 deg about x then 30 deg about z, and the
tweezers standing in a stand turned 90 deg about z, as in the wafer task."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import artkit as ak  # noqa: E402
import parts as P  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
A = P.asset_paths(Path(__file__).resolve().parent / "assets")


def qmul(a, b):  # xyzw
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


QX90 = (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4))
QZ30 = (0.0, 0.0, math.sin(math.pi / 12), math.cos(math.pi / 12))
Q_TEST = qmul(QZ30, QX90)  # first 90 deg about x, then 30 deg about z (world), as xyzw
Q_Z90 = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
OBJECTS = [
    ak.UsdPart("probe", pos=(0.30, 0.0, TOP + 0.10), size=P.STAND, usd=str(A["stand"]), kind="kinematic", rot_xyzw=Q_TEST),
    ak.UsdPart("stand", pos=(0.22, 0.33, TOP), size=P.STAND, usd=str(A["stand"]), kind="kinematic", rot_xyzw=Q_Z90),
    ak.UsdPart("tweezers", pos=(0.22, 0.33, TOP + P.TW_LEN + 0.0005), mass=2 * P.TW_MASS_HALF, size=(0.01, 0.01, P.TW_LEN),
               usd=str(A["tweezers"]), kind="articulation", rot_xyzw=Q_Z90, joint_pos0=(("hinge", 0.0),),
               springs=(("hinge", P.TW_SPRING, P.TW_SPRING_DAMPING),)),
]
