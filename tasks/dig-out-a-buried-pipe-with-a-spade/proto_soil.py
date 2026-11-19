"""Prototype: buried pipe, soil grains and a loose spade; the arms stay still."""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402
import soilkit  # noqa: E402
from spade import build_rests, build_spade  # noqa: E402
from bodykit import Body, BoxShape, CylShape  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
X0, X1, Y0, Y1 = 0.33, 0.51, -0.17, 0.01
WALL, FLOOR, H = 0.012, 0.010, 0.080
CEDAR = (0.62, 0.42, 0.27)
PVC = (0.95, 0.95, 0.93)
bodykit.FINISH[PVC] = (0.25, 0.0)
cx, cy = (X0 + X1) / 2, (Y0 + Y1) / 2
PIPE_R = 0.0167
PIPE_Z = TOP + FLOOR + PIPE_R + 0.004
SOIL_TOP = TOP + FLOOR + 0.062

BED = Body.build("planter", pos=(cx, cy, TOP), mass=5.0, kinematic=True, shapes=[
    BoxShape((0, 0, FLOOR / 2), (X1 - X0 + 2 * WALL, Y1 - Y0 + 2 * WALL, FLOOR), CEDAR),
    BoxShape(((X1 - X0 + WALL) / 2, 0, H / 2), (WALL, Y1 - Y0 + 2 * WALL, H), CEDAR),
    BoxShape((-(X1 - X0 + WALL) / 2, 0, H / 2), (WALL, Y1 - Y0 + 2 * WALL, H), CEDAR),
    BoxShape((0, (Y1 - Y0 + WALL) / 2, H / 2), (X1 - X0, WALL, H), CEDAR),
    BoxShape((0, -(Y1 - Y0 + WALL) / 2, H / 2), (X1 - X0, WALL, H), CEDAR),
    CylShape((0, 0, PIPE_Z - TOP), PIPE_R, X1 - X0 + 0.08, PVC, axis="X"),
])
TARP = Body.build("tarp", pos=(0.40, 0.17, TOP), mass=1.0, kinematic=True, shapes=[
    BoxShape((0, 0, 0.001), (0.22, 0.20, 0.002), (0.20, 0.35, 0.22))])

soilkit.soil("soil", host="planter", low=(X0, Y0, TOP + FLOOR), high=(X1, Y1, SOIL_TOP + 0.02),
             keep_out=(((cx, cy, PIPE_Z), (1, 0, 0), PIPE_R),),
             start_file=str(pathlib.Path(__file__).with_name("proto_soil_start.npz")))
SPADE_RESTS = build_rests()
SPADE = build_spade()
OBJECTS = [BED, TARP, SPADE_RESTS, SPADE]


def score(objects):
    import numpy as np
    p, r = soilkit.grains("soil")
    inside = (p[:, 0] > X0) & (p[:, 0] < X1) & (p[:, 1] > Y0) & (p[:, 1] < Y1)
    print(f"grains {len(p)} inside {inside.sum()} top z {np.percentile(p[inside, 2], 98) - TOP:.4f}", flush=True)
    return 0.0


def demo():
    from yam_arena import ik, station
    from yam_arena.constants import READY
    import numpy as np
    out = {}
    for arm in ("left", "right"):
        p, rot = ik.tcp_pose(np.array(READY), station.BASES[arm])
        out[arm] = [Waypoint(6.0, tuple(p), FINGER_OPEN, "hold", rot)]
    bodykit.allow_matrix_grips()
    return out
