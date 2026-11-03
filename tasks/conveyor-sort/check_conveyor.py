"""Hands-off test: visible staged blocks, loading, movement and reject scoring."""
from pathlib import Path

import numpy as np
from inspect_robots import Scene
from inspect_robots.types import Action
from yam_arena.constants import READY
from conveyor_sort.embodiment import ConveyorEmbodiment

emb = ConveyorEmbodiment(task=str(Path(__file__).with_name("task.py")), device="cpu", object_jitter=0,
                         control_hz=20, cam_width=480, cam_height=270)
emb.reset(Scene(id="belt_check", instruction="Hold both arms still."))
from yam_arena.scene import object_states
names = tuple(name for name, _ in emb.task.module.STARTS)
start = object_states(emb._env, names)
for i, name in enumerate(names):
    assert abs(start[name].pos[0] - (0.46 + i*0.09)) < 0.01
    assert abs(start[name].pos[1] + 0.50) < 0.01
    assert start[name].pos[2] > emb.task.module.BELT_TOP
print("All five blocks start on the visible loading shelf")
action = Action(data=np.array([*READY, 1.0, *READY, 1.0]))
for _ in range(100):
    result = emb.step(action)
end = object_states(emb._env, names)
assert "red_1" in emb._released
displacement = float(end["red_1"].pos[1] - start["red_1"].pos[1])
assert 0.065 <= displacement <= 0.085, displacement
assert abs(end["blue_1"].pos[0] - start["blue_1"].pos[0]) < 0.01
assert result.info["score"] == 0
print(f"First block loaded and moved {displacement:.4f} m; later blocks remain staged; score zero")
emb._place_block("red_1", (0.34, 0.605, emb.task.module.BELT_TOP + 0.02))
result = emb.step(action)
assert "red_1" in emb._discarded
assert result.info["score"] == 0
print("A missed block leaves at the belt end and scores zero")
emb.close()
