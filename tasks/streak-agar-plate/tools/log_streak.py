"""The task with the demo video's extra views: four close-up cameras round the plate and the streak-map panel (not
policy cameras; frames saved to LATCH_FRAMES). From the task folder:
    LATCH_FRAMES=<dir> inspect-robots run ... -E task=tools/log_streak.py
then python tools/frames_to_mp4.py <run frames dir> out.mp4 20 <dir> (LABELS=1 names the panels)."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ.setdefault("STREAK_LOG", "1")

import numpy as np  # noqa: E402
import task  # noqa: E402
from task import *  # noqa: E402,F401,F403

sys.path.insert(1, os.path.dirname(os.path.abspath(__file__)))
import latch_cam  # noqa: E402

latch_cam.BRIGHTEN = (0.8, 1.0)  # the white bench needs little brightening

__doc__ = task.__doc__
_c = (task.PLATE_XY[0], task.PLATE_XY[1], task.TOP + 0.005)
_a = (task.PLATE_XY[0], task.PLATE_XY[1], task.TOP + task.AGAR_TOP_H)  # the agar's surface at the plate's centre
latch_cam.install_many([
    ("plate_close", _c, (0.13, 0.09, 0.17), 18.0),  # from above the far side: the arm comes from the near right
    ("side_left", _a, (0.0, 0.17, 0.05), 32.0),  # low over the rim, from the robot's left
    ("side_far", _a, (0.17, 0.0, 0.05), 32.0),  # low over the rim, from the far side, looking back at the robot
    ("side_far_left", _a, (0.12, 0.12, 0.035), 40.0),  # lower still, diagonal: the ring on the agar
])

from scenekit import on_step  # noqa: E402

# the streak map: the agar from above (x up the image, y to the left), each point coloured by the quadrant whose turn
# drew it; the cross and the drop marked
_P = task.AGAR_PTS
_S = 352 / (2 * task.AGAR_R * 1.08)
_PX = np.round(176 - _S * _P[:, 1]).astype(int)
_PY = np.round(176 - _S * _P[:, 0]).astype(int)
_COL = {0: (235, 205, 135), 1: (20, 40, 200), 2: (200, 40, 40), 3: (30, 150, 50), 4: (150, 40, 170)}
_CROSS = (np.abs(_P[:, 0]) < 0.6e-3) | (np.abs(_P[:, 1]) < 0.6e-3)
_DROP = np.linalg.norm(_P - task.DROP_XY, axis=1) < task.DROP_R
_step = [0]


def _map(streak) -> np.ndarray:
    img = np.full((352, 352, 3), 40, np.uint8)
    col = np.array([_COL[int(s)] for s in range(5)], np.uint8)[streak]
    col[_CROSS & (streak == 0)] = (20, 20, 20)
    col[_DROP & (streak == 0)] = (90, 140, 230)
    for dx in (-1, 0, 1):  # points 0.5 mm apart are about 1.8 px apart: fill the gaps
        for dy in (-1, 0, 1):
            img[np.clip(_PY + dy, 0, 351), np.clip(_PX + dx, 0, 351)] = col
    return img


@on_step
def _closeups(environment) -> None:
    latch_cam.save_frame(environment)
    d = os.environ.get("LATCH_FRAMES")
    if d:
        os.makedirs(f"{d}/9_streak_map", exist_ok=True)
        np.save(f"{d}/9_streak_map/{_step[0]:05d}.npy", _map(task._STATE["streak"]))
        _step[0] += 1
