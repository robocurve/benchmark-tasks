"""open-can-pull-tab with close-up cameras on the tab and a state log every 0.25 s (LOG lines).
  TASK_DIR=<tasks/open-can-pull-tab> LATCH_FRAMES=<dir> inspect-robots run ... -E task=log_can.py | grep ^LOG"""
import os
import sys

sys.path.insert(0, os.environ["TASK_DIR"])
import task  # noqa: E402
from task import *  # noqa: E402,F401,F403

sys.path.insert(1, os.path.dirname(os.path.abspath(__file__)))
import latch_cam  # noqa: E402

__doc__ = task.__doc__
cx, cy = task.CAN_XY
zt = task.Z0 + 0.113
ty = cy + 0.0135
latch_cam.install_many([
    ("a_tab_from_the_robot", (cx, ty, zt), (-0.10, -0.07, 0.07), 30.0),
    ("b_tab_from_the_far_side", (cx, ty, zt), (0.11, -0.07, 0.06), 30.0),
    ("c_tab_from_the_left_arm", (cx, ty, zt), (0.00, 0.14, 0.10), 30.0),
    ("d_tab_from_above", (cx, ty, zt), (-0.05, -0.05, 0.13), 30.0),
])

from scenekit import _state, robot_links, on_step  # noqa: E402

_F = {"n": 0}


@on_step
def _closeups(environment) -> None:
    latch_cam.save_frame(environment)
    _F["n"] += 1
    if _F["n"] % 5:
        return
    c = _state(environment, task.CAN)
    o = _state(environment, task.OPENER)
    links = robot_links(environment)
    f = lambda v: tuple(int(round(float(x) * 1000)) for x in v)  # noqa: E731
    print(f"LOG t={_F['n'] / 20:.2f} tab_deg={task.tab_angle(c) * 57.2958:.1f} can={f(c.pos)} tilt={task.tilt(c) * 57.2958:.1f} "
          f"opener={f(o.pos)} op_deg={__import__('math').degrees(__import__('math').atan2(o.rotation()[2, 1], o.rotation()[1, 1])):.0f} grip_R={f(links['right']['gripper'])} grip_L={f(links['left']['gripper'])} "
          f"free={task._STATE['free']} score={task.score({'can': c})}", flush=True)
