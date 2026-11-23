"""Register the bedside reference only in this Inspect process."""

from dataclasses import replace
import os
import numpy as np
from inspect_robots.registry import policy
from inspect_robots.cli import main
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.packing import pack
from yam_arena.scripted import YamScriptedPolicy
from yam_arena.constants import READY, FINGER_OPEN
from task.policy import Push
from task.station import make_model


class Bedside(YamScriptedPolicy):
    def __init__(self):
        super().__init__()
        self.info = replace(self.info, name="heel_scripted")

    def reset(self, scene):
        self.controller = Push(make_model(), os.environ["HEEL_MODE"])
        self.tick = 0

    def act(self, observation):
        obs = self._task.module.monitor.obs
        q = self.controller.command(self.tick, obs)
        self.tick += 1
        return ActionChunk(
            actions=[
                Action(data=pack(np.r_[READY, 1.0], np.r_[q[:6], q[6] / FINGER_OPEN]))
            ],
            control_hz=20,
        )


@policy("heel_scripted")
def create():
    return Bedside()


if __name__ == "__main__":
    raise SystemExit(main())
